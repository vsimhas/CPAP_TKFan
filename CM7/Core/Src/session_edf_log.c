/**
  ******************************************************************************
  * @file    session_edf_log.c
  * @brief   Session EDF logger: 25 Hz therapy waveforms → SD FAT file.
  *
  *  RTOS safety: FatFs/SDMMC I/O runs ONLY in SessionEdf_Service() (call from
  *  SelfTestTask). Therapy_Start / SensorTask never block on the SD card.
  ******************************************************************************
  */

#include "session_edf_log.h"
#include "edf_writer.h"
#include "therapy.h"
#include "flow_sensor.h"
#include "blower_ctrl.h"
#include "leak_estimator.h"
#include "apap_ctrl.h"
#include "main.h"
#include "ff.h"
#include "rtc_clock.h"

#include <stdio.h>
#include <string.h>

volatile uint8_t  g_session_edf_enable = 0U;
volatile uint8_t  g_session_edf_state  = (uint8_t)SESSION_EDF_IDLE;
volatile uint32_t g_session_edf_records = 0U;
volatile uint32_t g_session_edf_error  = 0U;
volatile uint32_t g_session_edf_fsize  = 0U;

/* PLD is written best-effort, which used to mean silently: an Edf_Open()
 * failure just left .open false and nothing recorded why. OSCAR takes its
 * default Pressure / Mask Pressure / Leak graphs from PLD, so a missing PLD
 * shows up as "flow plots only" with no clue on the device side. These two
 * make that state visible via `therapy edf status`. */
volatile uint32_t g_session_edf_pld_error   = 0U;
volatile uint32_t g_session_edf_pld_records = 0U;
/* EVE (checklist #59): see session_edf_log.h. */
volatile uint32_t g_session_edf_eve_records = 0U;   /* incl. "Recording starts" */
volatile uint32_t g_session_edf_eve_error   = 0U;
volatile uint32_t g_session_edf_eve_dropped = 0U;   /* queue full / not logging */
/* CSL (#32): periodic-breathing spans, same layout as EVE. */
volatile uint32_t g_session_edf_csl_records = 0U;   /* incl. "Recording starts" */
volatile uint32_t g_session_edf_csl_error   = 0U;

#define EDF_HZ              25U
#define EDF_REC_S           1U
#define EDF_NSIG            2U
#define EDF_SAMPLES_PER_SIG (EDF_HZ * EDF_REC_S)
#define EDF_SAMPLE_BUF_N    (EDF_NSIG * EDF_SAMPLES_PER_SIG)
/* Queue several 1 s records so FatFs can write larger chunks (fewer SD txns). */
#define EDF_PENDING_MAX     8U

static FATFS s_fs;
static uint8_t s_mounted = 0U;
static EdfWriter_t s_edf;
static int16_t s_sample_buf[EDF_SAMPLE_BUF_N];
static int16_t s_pending_flat[EDF_PENDING_MAX * EDF_SAMPLE_BUF_N];
static int16_t s_write_flat[EDF_PENDING_MAX * EDF_SAMPLE_BUF_N];
static volatile uint8_t s_pend_count = 0U;
static uint8_t s_write_n = 0U;           /* records awaiting successful SD write */
static uint8_t s_write_fail_streak = 0U;
static uint16_t s_fill[EDF_NSIG];
static float s_accum_s = 0.0f;
static char s_path[64];
static uint8_t s_yy = 26U, s_mo = 1U, s_dd = 1U;
static uint8_t s_hh = 0U, s_mi = 0U, s_ss = 0U;
static uint8_t s_file_seq = 0U;

static volatile uint8_t s_req_start = 0U;
static volatile uint8_t s_req_stop = 0U;
static volatile uint8_t s_req_mount = 0U;    /* retry after a failed mount */
/* The card is mounted whenever one is in and has passed its self-test,
 * whether or not EDF logging is enabled: SUMMARY.CSV (#57) and adherence
 * (#65) need it for every session. A failed mount is not retried until the
 * card is reinserted or "therapy edf enable 1" is sent. */
static uint8_t s_mount_gave_up = 0U;
static volatile uint8_t s_collecting = 0U;

/* BRP: the 25 Hz waveform file. Labels, units and file suffix follow the
 * ResMed card layout because that is what OSCAR's loader matches - flow in
 * L/s (NOT L/min) and the ".40ms" suffix are both part of that contract.
 * Only the layout is borrowed; Identification.json below carries our own
 * identity, not a ResMed serial. */
/* Physical ranges MUST be symmetric about zero.
 *
 * EDF defines phys = dig * gain + offset, with
 *     gain   = (pmax - pmin) / (dmax - dmin)
 *     offset = pmin - gain * dmin
 * but OSCAR's ResMed loader applies the gain only and assumes offset 0,
 * because every genuine ResMed signal is written symmetric so its offset
 * really is zero. An asymmetric range therefore reads back shifted by the
 * offset OSCAR discarded.
 *
 * Measured on a real card before this was fixed: Flow at -2..3 carries
 * offset +0.5 L/s, and OSCAR plotted the trace 30 L/min low (axis -47..-5
 * instead of -17..+24). Press.40ms at 0..40 carries offset +20, so every
 * sample decoded to -20..-14.5 cmH2O and the Mask Pressure graph rendered
 * blank - the channel and samples were both present and every value was
 * negative. Keep pmin = -pmax when adding or retuning a signal. */
static const EdfSignalDesc_t s_sigs[EDF_NSIG] = {
    { "Flow.40ms",  "Patient flow", "L/s", -3.0f, 3.0f, -32768, 32767, "LP ~15Hz", EDF_SAMPLES_PER_SIG },
    { "Press.40ms", "Mask pressure", "cmH2O", -40.0f, 40.0f, -32768, 32767, "none", EDF_SAMPLES_PER_SIG },
};

/* PLD: the low-rate summary file, where OSCAR looks for leak and the therapy
 * setpoint. ResMed samples these at 0.5 Hz; we use 1 Hz with 1 s records so
 * the cadence matches BRP exactly and no second record duration is needed
 * (OSCAR derives the rate from the header, not from the ".2s" label).
 *
 * Best-effort by design: if PLD cannot be opened or written, BRP logging
 * carries on. A new file must not be able to break the existing one. */
#define PLD_NSIG            5U
#define PLD_SPR             1U      /* samples per 1 s record */
#define PLD_BUF_N           (PLD_NSIG * PLD_SPR)

/* Symmetric about zero for the same reason as s_sigs above. */
static const EdfSignalDesc_t s_pld_sigs[PLD_NSIG] = {
    { "MaskPress.2s", "Mask pressure", "cmH2O", -40.0f, 40.0f, -32768, 32767, "none", PLD_SPR },
    { "Press.2s",     "Therapy target", "cmH2O", -50.0f, 50.0f, -32768, 32767, "none", PLD_SPR },
    { "Leak.2s",      "Unintentional leak", "L/s", -2.0f, 2.0f, -32768, 32767, "LP", PLD_SPR },
    /* #60 / #61: ranges as the AirSense writes them. Both start at 0 / dig
     * 0, so their EDF offset is zero too, which is what OSCAR assumes. */
    { "Snore.2s",     "Snore",            "",    0.0f,  5.0f, 0, 250, "none", PLD_SPR },
    { "FlowLim.2s",   "Flow limitation",  "",    0.0f,  1.0f, 0, 100, "none", PLD_SPR },
};

static EdfWriter_t s_pld;
static int16_t  s_pld_buf[PLD_BUF_N];
static int16_t  s_pld_pending[EDF_PENDING_MAX * PLD_BUF_N];
static int16_t  s_pld_write[EDF_PENDING_MAX * PLD_BUF_N];
static volatile uint8_t s_pld_pend = 0U;
static uint8_t  s_pld_write_n = 0U;
static float    s_pld_acc[PLD_NSIG];
static uint16_t s_pld_acc_n = 0U;
static char     s_pld_path[64];
static char     s_dir_path[32];

/* ---- EVE annotation file (checklist #59) ---------------------------------
 * Byte-for-byte the layout of the AirSense 11's own EVE.edf (checked on the
 * 20 Oct 2026 reference card): EDF+D, two signals "EDF Annotations" (31
 * samples = 62 bytes) and "Crc16" (1 sample), record duration 0. Each 64-byte
 * record is a time-keeping TAL "+0" followed by one event TAL
 * "+<onset>\x15<duration>\x14<text>\x14", zero-padded, then CRC-16/CCITT-
 * FALSE (poly 0x1021, init 0xFFFF) of the 62 annotation bytes, little-endian.
 * The first record is "Recording starts". Onset is whole seconds from the
 * header start time and marks the event's END (ResMed's convention, which
 * OSCAR follows); OSCAR matches the text case-insensitively by prefix.
 *
 * CSL (#32) is the same file layout (checked against the AirSense 11 CSL.edf
 * on the same card): OSCAR reads a "CSR Start" / "CSR End" pair from it as
 * one Cheyne-Stokes span, from the onsets alone. Both files share the event
 * queue; the type picks the file. */
#define EVE_REC_BYTES       64U
#define EVE_ANN_BYTES       62U
#define EVE_HDR_BYTES       768U
#define EVE_NREC_OFFSET     236U
#define EVE_Q_N             16U
#define EDF_TICKS_PER_S     EDF_HZ           /* 40 ms session ticks */

typedef struct
{
    uint8_t  type;
    uint32_t end_ticks;
    uint16_t dur_ds;                         /* 0.1 s */
} EveItem_t;

typedef struct
{
    FIL      fil;
    uint8_t  open;
    uint8_t  err_tag;                        /* 0x5 EVE, 0x6 CSL */
    char     path[64];
    volatile uint32_t *records;
    volatile uint32_t *error;
} AnnFile_t;

static AnnFile_t s_eve = { .err_tag = 0x5U, .records = &g_session_edf_eve_records,
                           .error = &g_session_edf_eve_error };
static AnnFile_t s_csl = { .err_tag = 0x6U, .records = &g_session_edf_csl_records,
                           .error = &g_session_edf_csl_error };
static EveItem_t s_eve_q[EVE_Q_N];
static volatile uint8_t s_eve_q_head = 0U;   /* producer: therapy tick */
static volatile uint8_t s_eve_q_tail = 0U;   /* consumer: Service */
static volatile uint32_t s_ticks = 0U;       /* BRP samples since open */

static const char * const k_eve_text[SESSION_EVT_COUNT] = {
    "Obstructive Apnea", "Central Apnea", "Hypopnea", "CSR Start", "CSR End"
};

static uint16_t crc16CcittFalse(const uint8_t *p, uint32_t n)
{
    uint16_t crc = 0xFFFFU;
    for (uint32_t i = 0U; i < n; i++)
    {
        crc ^= (uint16_t)((uint16_t)p[i] << 8);
        for (uint32_t b = 0U; b < 8U; b++)
        {
            crc = ((crc & 0x8000U) != 0U) ? (uint16_t)((crc << 1) ^ 0x1021U)
                                          : (uint16_t)(crc << 1);
        }
    }
    return crc;
}

/* One 64-byte record: "+0" 0x14 0x14 0x00, then "+<onset>" 0x15 "<dur>" 0x14
 * "<text>" 0x14 0x00, zero padding, CRC. */
static void eveBuildRecord(uint8_t rec[EVE_REC_BYTES], uint32_t onset_s,
                           uint32_t dur_s, const char *text)
{
    memset(rec, 0, EVE_REC_BYTES);
    uint32_t n = 0U;
    rec[n++] = (uint8_t)'+';
    rec[n++] = (uint8_t)'0';
    rec[n++] = 0x14U;
    rec[n++] = 0x14U;
    rec[n++] = 0x00U;
    const int m = snprintf((char *)&rec[n], EVE_ANN_BYTES - n, "+%lu%c%lu%c%s%c",
                           (unsigned long)onset_s, 0x15, (unsigned long)dur_s,
                           0x14, text, 0x14);
    if ((m > 0) && ((uint32_t)m < (EVE_ANN_BYTES - n)))
    {
        n += (uint32_t)m;
        rec[n] = 0x00U;                      /* TAL terminator */
    }
    const uint16_t crc = crc16CcittFalse(rec, EVE_ANN_BYTES);
    rec[EVE_ANN_BYTES]      = (uint8_t)(crc & 0xFFU);
    rec[EVE_ANN_BYTES + 1U] = (uint8_t)(crc >> 8);
}

static void annSetErr(AnnFile_t *f, uint32_t stage, FRESULT fr)
{
    *f->error = ((uint32_t)f->err_tag << 12) | ((stage & 0xFU) << 8)
              | ((uint32_t)fr & 0xFFU);
}

/* Header record count, rewritten after each batch so a session cut short
 * (power loss) still reads back with the events written so far. */
static bool annWriteCount(AnnFile_t *f)
{
    char cnt[9];
    UINT bw = 0U;
    (void)snprintf(cnt, sizeof(cnt), "%-8lu", (unsigned long)*f->records);
    FRESULT fr = f_lseek(&f->fil, EVE_NREC_OFFSET);
    if (fr == FR_OK) { fr = f_write(&f->fil, cnt, 8U, &bw); }
    if ((fr != FR_OK) || (bw != 8U))
    {
        annSetErr(f, 4U, fr);
        return false;
    }
    return true;
}

static bool annAppend(AnnFile_t *f, const uint8_t rec[EVE_REC_BYTES])
{
    UINT bw = 0U;
    const FSIZE_t at = (FSIZE_t)EVE_HDR_BYTES
                     + ((FSIZE_t)*f->records * EVE_REC_BYTES);
    FRESULT fr = f_lseek(&f->fil, at);
    if (fr == FR_OK) { fr = f_write(&f->fil, rec, EVE_REC_BYTES, &bw); }
    if ((fr != FR_OK) || (bw != EVE_REC_BYTES))
    {
        annSetErr(f, 3U, fr);
        return false;
    }
    (*f->records)++;
    return true;
}

static void eveField(char *hdr, uint32_t off, uint32_t width, const char *text)
{
    const size_t len = strlen(text);
    memcpy(&hdr[off], text, (len < width) ? len : width);
}

static void annOpen(AnnFile_t *f, const char *patient, const char *recording)
{
    static char hdr[EVE_HDR_BYTES];
    char tmp[24];
    UINT bw = 0U;
    uint8_t rec[EVE_REC_BYTES];

    f->open = 0U;
    *f->records = 0U;
    *f->error = 0U;

    FRESULT fr = f_open(&f->fil, f->path, FA_CREATE_ALWAYS | FA_WRITE);
    if (fr != FR_OK)
    {
        annSetErr(f, 1U, fr);
        return;
    }
    memset(hdr, ' ', sizeof(hdr));
    eveField(hdr, 0U, 8U, "0");
    eveField(hdr, 8U, 80U, patient);
    eveField(hdr, 88U, 80U, recording);
    (void)snprintf(tmp, sizeof(tmp), "%02u.%02u.%02u",
                   (unsigned)s_dd, (unsigned)s_mo, (unsigned)s_yy);
    eveField(hdr, 168U, 8U, tmp);
    (void)snprintf(tmp, sizeof(tmp), "%02u.%02u.%02u",
                   (unsigned)s_hh, (unsigned)s_mi, (unsigned)s_ss);
    eveField(hdr, 176U, 8U, tmp);
    eveField(hdr, 184U, 8U, "768");
    eveField(hdr, 192U, 44U, "EDF+D");
    eveField(hdr, 236U, 8U, "0");
    eveField(hdr, 244U, 8U, "0.00");
    eveField(hdr, 252U, 4U, "2");
    /* Signal headers: each field for both signals in turn (EDF layout). */
    eveField(hdr, 256U, 16U, "EDF Annotations");
    eveField(hdr, 272U, 16U, "Crc16");
    /* transducer 2x80 and dimension 2x8 stay blank */
    eveField(hdr, 464U, 8U, "-32768.0");     /* physical min */
    eveField(hdr, 472U, 8U, "-32768.0");
    eveField(hdr, 480U, 8U, "32767.00");     /* physical max */
    eveField(hdr, 488U, 8U, "32767.00");
    eveField(hdr, 496U, 8U, "-32768");       /* digital min */
    eveField(hdr, 504U, 8U, "-32768");
    eveField(hdr, 512U, 8U, "32767");        /* digital max */
    eveField(hdr, 520U, 8U, "32767");
    /* prefiltering 2x80 blank */
    eveField(hdr, 688U, 8U, "31");           /* samples per record */
    eveField(hdr, 696U, 8U, "1");
    /* reserved 2x32 blank */

    fr = f_write(&f->fil, hdr, sizeof(hdr), &bw);
    if ((fr != FR_OK) || (bw != sizeof(hdr)))
    {
        annSetErr(f, 2U, fr);
        (void)f_close(&f->fil);
        return;
    }
    f->open = 1U;
    eveBuildRecord(rec, 0U, 0U, "Recording starts");
    if (!annAppend(f, rec) || !annWriteCount(f) || (f_sync(&f->fil) != FR_OK))
    {
        (void)f_close(&f->fil);
        f->open = 0U;
    }
}

static void annDrop(AnnFile_t *f)
{
    (void)f_close(&f->fil);
    f->open = 0U;
}

/* Drain the RAM queue into records, EVE or CSL by type. Like PLD, errors are
 * swallowed: a bad annotation file must never take down the BRP session. */
static void eveService(void)
{
    uint8_t rec[EVE_REC_BYTES];
    bool wrote_eve = false;
    bool wrote_csl = false;

    while (s_eve_q_tail != s_eve_q_head)
    {
        const EveItem_t it = s_eve_q[s_eve_q_tail];
        s_eve_q_tail = (uint8_t)((s_eve_q_tail + 1U) % EVE_Q_N);
        if (it.type >= (uint8_t)SESSION_EVT_COUNT)
        {
            continue;
        }
        const bool csl = (it.type >= (uint8_t)SESSION_EVT_CSR_START);
        AnnFile_t *f = csl ? &s_csl : &s_eve;
        if (f->open == 0U)
        {
            continue;
        }
        eveBuildRecord(rec, it.end_ticks / EDF_TICKS_PER_S,
                       ((uint32_t)it.dur_ds + 5U) / 10U, k_eve_text[it.type]);
        if (!annAppend(f, rec))
        {
            annDrop(f);
            continue;
        }
        if (csl) { wrote_csl = true; } else { wrote_eve = true; }
    }
    if (wrote_eve && (!annWriteCount(&s_eve) || (f_sync(&s_eve.fil) != FR_OK)))
    {
        annDrop(&s_eve);
    }
    if (wrote_csl && (!annWriteCount(&s_csl) || (f_sync(&s_csl.fil) != FR_OK)))
    {
        annDrop(&s_csl);
    }
}

static void eveClose(void)
{
    if ((s_eve.open != 0U) || (s_csl.open != 0U))
    {
        eveService();
    }
    if (s_eve.open != 0U) { annDrop(&s_eve); }
    if (s_csl.open != 0U) { annDrop(&s_csl); }
}

static void setErr(uint32_t e)
{
    g_session_edf_error = e;
    g_session_edf_state = (uint8_t)SESSION_EDF_ERROR;
    s_collecting = 0U;
}

bool SessionEdf_IsMounted(void)
{
    return (s_mounted != 0U);
}

bool SessionEdf_Mount(void)
{
    FRESULT fr;
    if (s_mounted != 0U)
    {
        return true;
    }
    if (sd_card_present == 0U)
    {
        setErr(0x1000U); /* no card */
        return false;
    }
    /* Self-test still running/failed — keep Service retrying; not a permanent EDF fault. */
    if (sd_test_result == HAL_BUSY)
    {
        g_session_edf_error = 0x1002U;
        return false;
    }
    if (sd_test_result != HAL_OK)
    {
        g_session_edf_error = 0x1000U | ((uint32_t)sd_test_result & 0xFFU);
        return false;
    }
    if (sd_fatfs_ready == 0U)
    {
        g_session_edf_error = 0x1002U; /* settling after self-test */
        return false;
    }
    fr = f_mount(&s_fs, "0:", 1);
    if (fr != FR_OK)
    {
        /* One remount attempt after discarding a stale FatFs object. */
        (void)f_mount(NULL, "0:", 0);
        memset(&s_fs, 0, sizeof(s_fs));
        fr = f_mount(&s_fs, "0:", 1);
    }
    if (fr != FR_OK)
    {
        setErr((uint32_t)fr);
        return false;
    }
    s_mounted = 1U;
    g_session_edf_error = 0U;
    if (g_session_edf_state == (uint8_t)SESSION_EDF_ERROR)
    {
        g_session_edf_state = (uint8_t)SESSION_EDF_IDLE;
    }
    return true;
}

void SessionEdf_Unmount(void)
{
    if (s_edf.open)
    {
        (void)Edf_Close(&s_edf);
    }
    if (s_pld.open)
    {
        (void)Edf_Close(&s_pld);
    }
    eveClose();
    if (s_mounted != 0U)
    {
        (void)f_mount(NULL, "0:", 0);
        s_mounted = 0U;
    }
    s_collecting = 0U;
    s_pend_count = 0U;
    s_write_n = 0U;
    s_write_fail_streak = 0U;
}

void SessionEdf_Init(void)
{
    /* On from boot: a session after a power cycle used to get no waveform
     * files until `therapy edf enable 1` was sent again. All FatFs work is
     * in SelfTestTask, so it cannot stall USB or the sensor loop. */
    g_session_edf_enable = 1U;
    g_session_edf_state = (uint8_t)SESSION_EDF_IDLE;
    g_session_edf_records = 0U;
    g_session_edf_error = 0U;
    s_mounted = 0U;
    s_path[0] = '\0';
    s_accum_s = 0.0f;
    s_req_start = 0U;
    s_req_stop = 0U;
    s_req_mount = 0U;
    s_pend_count = 0U;
    s_write_n = 0U;
    s_write_fail_streak = 0U;
    s_collecting = 0U;
    memset(s_fill, 0, sizeof(s_fill));
    memset(&s_edf, 0, sizeof(s_edf));
}

void SessionEdf_SetEnable(bool en)
{
    /* Waveform logging on therapy start only - the card is mounted either
     * way. Enabling also retries a mount that failed. */
    g_session_edf_enable = en ? 1U : 0U;
    if (en)
    {
        s_req_mount = 1U;
    }
}

/* The card went away: forget the volume and any open files WITHOUT I/O to
 * the missing card, so nothing waits on SDMMC timeouts. */
static void detachCard(void)
{
    if (s_edf.open || s_pld.open || (s_eve.open != 0U) || (s_csl.open != 0U))
    {
        setErr(0x1000U);                     /* removed during a session */
    }
    s_edf.open = false;
    s_pld.open = false;
    s_eve.open = 0U;
    s_csl.open = 0U;
    (void)f_mount(NULL, "0:", 0);
    s_mounted = 0U;
    s_collecting = 0U;
    s_pend_count = 0U;
    s_write_n = 0U;
    s_write_fail_streak = 0U;
}

bool SessionEdf_IsEnabled(void)
{
    return (g_session_edf_enable != 0U);
}

void SessionEdf_SetDateTime(uint8_t yy, uint8_t mo, uint8_t dd,
                            uint8_t hh, uint8_t mi, uint8_t ss)
{
    s_yy = (yy > 99U) ? 99U : yy;
    s_mo = ((mo < 1U) || (mo > 12U)) ? 1U : mo;
    s_dd = ((dd < 1U) || (dd > 31U)) ? 1U : dd;
    s_hh = (hh > 23U) ? 0U : hh;
    s_mi = (mi > 59U) ? 0U : mi;
    s_ss = (ss > 59U) ? 0U : ss;
    /* Checklist #66: the same command sets the RTC, so the clock keeps
     * running across sessions and resets. */
    {
        const RtcDateTime_t dt = { s_yy, s_mo, s_dd, s_hh, s_mi, s_ss };
        (void)RtcClock_Set(&dt);
    }
}

/* Session start time from the RTC when it runs; otherwise the last value
 * set over USB (or the 2026-01-01 default), as before the RTC. */
static void latchSessionStart(void)
{
    RtcDateTime_t dt;
    if (RtcClock_Get(&dt))
    {
        s_yy = dt.yy;
        s_mo = dt.mo;
        s_dd = dt.dd;
        s_hh = dt.hh;
        s_mi = dt.mi;
        s_ss = dt.ss;
    }
}

static const char* monthAbbrev(uint8_t mo)
{
    static const char *k[12] = { "JAN", "FEB", "MAR", "APR", "MAY", "JUN",
                                 "JUL", "AUG", "SEP", "OCT", "NOV", "DEC" };
    return ((mo >= 1U) && (mo <= 12U)) ? k[mo - 1U] : "JAN";
}

/* Card-level identity. OSCAR's ResMed loader will not look at DATALOG at all
 * unless it first finds this file at the card root, so it is written once per
 * mount rather than per session.
 *
 * The ProductCode does NOT have to be a real ResMed code - unknown codes are
 * accepted - so this advertises our own device honestly instead of
 * impersonating an AirSense. Only the card LAYOUT is borrowed. */
static void writeIdentification(void)
{
    FIL f;
    FILINFO fno;
    UINT bw = 0U;
    static const char json[] =
        "{\"FlowGenerator\":{\"IdentificationProfiles\":{\"Product\":{"
        "\"SerialNumber\":\"SL0000000001\","
        "\"ProductCode\":\"90001\","
        "\"ProductName\":\"SomnaLink CPAP\""
        "},\"Software\":{"
        "\"PlatformIdentifier\":46,"
        "\"VariantIdentifier\":3"
        "}}}}";

    if (f_stat("0:/Identification.json", &fno) == FR_OK)
    {
        return;   /* already present */
    }
    if (f_open(&f, "0:/Identification.json", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
    {
        return;   /* best-effort: never fail a session over this */
    }
    (void)f_write(&f, json, (UINT)(sizeof(json) - 1U), &bw);
    (void)f_close(&f);
}

/* OSCAR expects STR.edf to exist at the card root. Per bench testing the
 * filename is what matters, not a full ResMed summary implementation, so this
 * is a valid but empty EDF: correct 256-byte header, zero signals, zero
 * records. Replace with real per-session summary channels only if OSCAR is
 * later found to need them. */
static void writeStrStub(void)
{
    FIL f;
    FILINFO fno;
    UINT bw = 0U;
    char hdr[256];

    if (f_stat("0:/STR.edf", &fno) == FR_OK)
    {
        return;
    }
    memset(hdr, ' ', sizeof(hdr));
    memcpy(&hdr[0], "0", 1);                       /* version              */
    /* Valid dd.mm.yy / hh.mm.ss even though the file carries no records:
     * a strict EDF reader will reject blank date/time fields outright. */
    {
        char dt[9];
        (void)snprintf(dt, sizeof(dt), "%02u.%02u.%02u",
                       (unsigned)s_dd, (unsigned)s_mo, (unsigned)s_yy);
        memcpy(&hdr[168], dt, 8);
        (void)snprintf(dt, sizeof(dt), "%02u.%02u.%02u",
                       (unsigned)s_hh, (unsigned)s_mi, (unsigned)s_ss);
        memcpy(&hdr[176], dt, 8);
    }
    memcpy(&hdr[184], "256     ", 8);              /* header bytes         */
    memcpy(&hdr[192], "EDF", 3);                   /* reserved             */
    memcpy(&hdr[236], "0       ", 8);              /* n data records       */
    memcpy(&hdr[244], "1       ", 8);              /* record duration (s)  */
    memcpy(&hdr[252], "0   ", 4);                  /* n signals            */

    if (f_open(&f, "0:/STR.edf", FA_CREATE_ALWAYS | FA_WRITE) != FR_OK)
    {
        return;
    }
    (void)f_write(&f, hdr, (UINT)sizeof(hdr), &bw);
    (void)f_close(&f);
}

/* ResMed card layout: DATALOG/YYYYMMDD/YYYYMMDD_HHMMSS_{BRP,PLD}.edf.
 * Needs LFN (see ffconf.h); the old 8.3 YYMMDDNN.EDF name could not express
 * this. Seconds are taken from the session start clock, so a second session
 * in the same second is the only collision case - resolved by bumping ss. */
static bool pickSessionPath(void)
{
    FILINFO fno;
    uint8_t tries;
    uint8_t ss = s_ss;

    (void)f_mkdir("0:/DATALOG");
    (void)snprintf(s_dir_path, sizeof(s_dir_path), "0:/DATALOG/20%02u%02u%02u",
                   (unsigned)s_yy, (unsigned)s_mo, (unsigned)s_dd);
    (void)f_mkdir(s_dir_path);

    for (tries = 0U; tries < 60U; tries++)
    {
        (void)snprintf(s_path, sizeof(s_path),
                       "%s/20%02u%02u%02u_%02u%02u%02u_BRP.edf",
                       s_dir_path, (unsigned)s_yy, (unsigned)s_mo, (unsigned)s_dd,
                       (unsigned)s_hh, (unsigned)s_mi, (unsigned)ss);
        if (f_stat(s_path, &fno) == FR_NO_FILE)
        {
            break;
        }
        ss = (uint8_t)((ss + 1U) % 60U);
    }
    (void)snprintf(s_pld_path, sizeof(s_pld_path),
                   "%s/20%02u%02u%02u_%02u%02u%02u_PLD.edf",
                   s_dir_path, (unsigned)s_yy, (unsigned)s_mo, (unsigned)s_dd,
                   (unsigned)s_hh, (unsigned)s_mi, (unsigned)ss);
    (void)snprintf(s_eve.path, sizeof(s_eve.path),
                   "%s/20%02u%02u%02u_%02u%02u%02u_EVE.edf",
                   s_dir_path, (unsigned)s_yy, (unsigned)s_mo, (unsigned)s_dd,
                   (unsigned)s_hh, (unsigned)s_mi, (unsigned)ss);
    (void)snprintf(s_csl.path, sizeof(s_csl.path),
                   "%s/20%02u%02u%02u_%02u%02u%02u_CSL.edf",
                   s_dir_path, (unsigned)s_yy, (unsigned)s_mo, (unsigned)s_dd,
                   (unsigned)s_hh, (unsigned)s_mi, (unsigned)ss);
    s_file_seq++;
    return true;
}

static bool openSessionFile(void)
{
    char patient[80];
    char recording[80];
    FRESULT fr;

    if (!SessionEdf_Mount())
    {
        return false;
    }

    latchSessionStart();
    if (!pickSessionPath())
    {
        setErr(0x2000U | (uint32_t)FR_DENIED);
        return false;
    }

    /* EDF+ style local patient / recording identification, shaped the way the
     * ResMed loader parses it: SRN/MID/VID must be present and space-
     * separated after the "Startdate DD-MMM-YYYY" prefix. */
    (void)snprintf(patient, sizeof(patient), "X X X X SomnaLink");
    (void)snprintf(recording, sizeof(recording),
                   "Startdate %02u-%s-20%02u X X X SRN=SL0000000001 MID=46 VID=3",
                   (unsigned)s_dd, monthAbbrev(s_mo), (unsigned)s_yy);

    writeIdentification();
    writeStrStub();

    memset(s_fill, 0, sizeof(s_fill));
    s_accum_s = 0.0f;
    g_session_edf_records = 0U;
    g_session_edf_fsize = 0U;
    s_pend_count = 0U;
    s_write_n = 0U;
    s_write_fail_streak = 0U;

    fr = Edf_Open(&s_edf, s_path, patient, recording,
                  s_dd, s_mo, s_yy, s_hh, s_mi, s_ss,
                  EDF_REC_S, s_sigs, EDF_NSIG,
                  s_sample_buf, EDF_SAMPLE_BUF_N);
    if (fr != FR_OK)
    {
        /* 0x2sxx: s = Edf_Open stage that failed, xx = FRESULT. */
        setErr(0x2000U
               | (((uint32_t)s_edf.open_step & 0xFU) << 8)
               | ((uint32_t)fr & 0xFFU));
        return false;
    }
    /* PLD is best-effort: a failure here leaves BRP logging fully intact and
     * only costs the leak / target-pressure summary channels in OSCAR. */
    s_pld_pend = 0U;
    s_pld_write_n = 0U;
    s_pld_acc_n = 0U;
    memset(s_pld_acc, 0, sizeof(s_pld_acc));
    g_session_edf_pld_records = 0U;
    g_session_edf_pld_error   = 0U;
    {
        const FRESULT pfr = Edf_Open(&s_pld, s_pld_path, patient, recording,
                                     s_dd, s_mo, s_yy, s_hh, s_mi, s_ss,
                                     EDF_REC_S, s_pld_sigs, PLD_NSIG,
                                     s_pld_buf, PLD_BUF_N);
        if (pfr != FR_OK)
        {
            /* Same encoding as the BRP open error: 0x4Sxx, S = open step. */
            g_session_edf_pld_error = 0x4000U
                                    | (((uint32_t)s_pld.open_step & 0xFU) << 8)
                                    | ((uint32_t)pfr & 0xFFU);
            memset(&s_pld, 0, sizeof(s_pld));   /* .open stays false */
        }
    }

    /* EVE (#59) and CSL (#32), best-effort like PLD. */
    s_eve_q_tail = s_eve_q_head;
    s_ticks = 0U;
    annOpen(&s_eve, patient, recording);
    annOpen(&s_csl, patient, recording);

    g_session_edf_state = (uint8_t)SESSION_EDF_LOGGING;
    g_session_edf_error = 0U;
    g_session_edf_fsize = (uint32_t)f_size(&s_edf.fil);
    s_collecting = 1U;
    return true;
}

void SessionEdf_OnTherapyStart(void)
{
    /* Non-blocking: SelfTestTask opens the file via SessionEdf_Service(). */
    if (g_session_edf_enable == 0U)
    {
        return;
    }
    s_req_stop = 0U;
    s_req_start = 1U;
}

void SessionEdf_OnTherapyStop(void)
{
    s_req_start = 0U;
    s_req_stop = 1U;
    s_collecting = 0U;
}

static void pushSample(float flow, float press, float leak, float target)
{
    uint8_t i;
    /* BRP carries the waveforms; flow converted L/min -> L/s to match the
     * Flow.40ms channel's declared unit. */
    const float vals[EDF_NSIG] = { flow / 60.0f, press };

    s_ticks++;                               /* EVE session clock (#59) */

    /* Full queue checked when committing a 1 s record (see below). */

    for (i = 0U; i < EDF_NSIG; i++)
    {
        const uint16_t idx = s_fill[i];
        if (idx < EDF_SAMPLES_PER_SIG)
        {
            const uint32_t off = ((uint32_t)i * EDF_SAMPLES_PER_SIG) + idx;
            s_sample_buf[off] = Edf_PhysToDig(&s_sigs[i], vals[i]);
            s_fill[i] = (uint16_t)(idx + 1U);
        }
    }

    /* Mean over the second for the PLD summary channels, so leak is not
     * sampled on an arbitrary instant of the breath. */
    s_pld_acc[0] += press;
    s_pld_acc[1] += target;
    s_pld_acc[2] += leak / 60.0f;      /* L/min -> L/s (Leak.2s) */
    s_pld_acc[3] += Apap_GetSnoreLevel();
    s_pld_acc[4] += Apap_GetFlowLimIndex();
    s_pld_acc_n++;

    if (s_fill[0] >= EDF_SAMPLES_PER_SIG)
    {
        uint8_t slot;
        const float n = (s_pld_acc_n > 0U) ? (float)s_pld_acc_n : 1.0f;

        for (i = 0U; i < PLD_NSIG; i++)
        {
            s_pld_buf[i] = Edf_PhysToDig(&s_pld_sigs[i], s_pld_acc[i] / n);
        }

        __disable_irq();
        slot = s_pend_count;
        if (slot < EDF_PENDING_MAX)
        {
            const uint32_t dst =
                (uint32_t)slot * (uint32_t)EDF_SAMPLE_BUF_N;
            memcpy(&s_pending_flat[dst], s_sample_buf, sizeof(s_sample_buf));
            s_pend_count = (uint8_t)(slot + 1U);
        }
        if (s_pld_pend < EDF_PENDING_MAX)
        {
            memcpy(&s_pld_pending[(uint32_t)s_pld_pend * PLD_BUF_N],
                   s_pld_buf, sizeof(s_pld_buf));
            s_pld_pend = (uint8_t)(s_pld_pend + 1U);
        }
        __enable_irq();

        memset(s_fill, 0, sizeof(s_fill));
        memset(s_pld_acc, 0, sizeof(s_pld_acc));
        s_pld_acc_n = 0U;
    }
}

void SessionEdf_Update(float dt_s)
{
    /* SensorTask: RAM sampling only — never FatFs. */
    if ((s_collecting == 0U) || (g_session_edf_enable == 0U))
    {
        return;
    }
    if (!(dt_s > 0.0f) || dt_s > 0.1f)
    {
        return;
    }

    s_accum_s += dt_s;
    const float period = 1.0f / (float)EDF_HZ;
    while (s_accum_s >= period)
    {
        s_accum_s -= period;
        pushSample(Flow_GetPatientSlm(),
                   BlowerCtrl_GetMeasured(),
                   Leak_GetUnintentionalSlm(),
                   Therapy_GetTargetPressure());
    }
}

/* Drain and write the PLD queue. Deliberately swallows every error: PLD is a
 * convenience file for OSCAR, and must never take down the BRP session or
 * trip the fail-streak logic that guards it. */
static void servicePld(void)
{
    uint8_t n;

    if (!s_pld.open)
    {
        s_pld_pend = 0U;
        return;
    }
    if (s_pld_write_n == 0U)
    {
        __disable_irq();
        n = s_pld_pend;
        if (n != 0U)
        {
            memcpy(s_pld_write, s_pld_pending, (size_t)n * sizeof(s_pld_buf));
            s_pld_pend = 0U;
        }
        __enable_irq();
        s_pld_write_n = n;
    }
    if (s_pld_write_n != 0U)
    {
        const FRESULT wfr = Edf_WriteRecords(&s_pld, s_pld_write,
                                             s_pld_write_n);
        if (wfr == FR_OK)
        {
            g_session_edf_pld_records += (uint32_t)s_pld_write_n;
        }
        else
        {
            /* Still swallowed - PLD must never take BRP down - but visible. */
            g_session_edf_pld_error = 0x4200U | ((uint32_t)wfr & 0xFFU);
        }
        s_pld_write_n = 0U;
    }
}

void SessionEdf_Service(void)
{
    /* SelfTestTask / low-priority context only. */
    if (s_req_stop != 0U)
    {
        s_req_stop = 0U;
        s_collecting = 0U;
        if (s_edf.open)
        {
            uint8_t n;
            /* Drain pending queue into write buffer if idle. */
            if (s_write_n == 0U)
            {
                __disable_irq();
                n = s_pend_count;
                if (n != 0U)
                {
                    memcpy(s_write_flat, s_pending_flat,
                           (size_t)n * sizeof(s_sample_buf));
                    s_pend_count = 0U;
                }
                __enable_irq();
                s_write_n = n;
            }
            if (s_write_n != 0U)
            {
                (void)Edf_WriteRecords(&s_edf, s_write_flat, s_write_n);
                s_write_n = 0U;
            }
            servicePld();
            if (s_pld.open)
            {
                (void)Edf_Close(&s_pld);
            }
            eveClose();
            {
                FRESULT cr = Edf_Close(&s_edf);
                g_session_edf_records = s_edf.n_records;
                if (cr != FR_OK)
                {
                    /* Close/flush failed — keep mount; report write error only. */
                    g_session_edf_error = 0x3100U | ((uint32_t)cr & 0xFFU);
                }
                else
                {
                    g_session_edf_fsize =
                        (uint32_t)(s_edf.header_bytes
                                   + (s_edf.n_records * s_edf.bytes_per_record));
                }
            }
        }
        if (g_session_edf_state == (uint8_t)SESSION_EDF_LOGGING)
        {
            g_session_edf_state = (uint8_t)SESSION_EDF_IDLE;
        }
        /* Volume stays mounted while the card is in. */
        s_pend_count = 0U;
        s_write_n = 0U;
        s_write_fail_streak = 0U;
    }

    /* Card presence drives the mount, not the EDF enable (see s_mount_gave_up). */
    if ((sd_card_present == 0U) || (s_req_mount != 0U))
    {
        s_mount_gave_up = 0U;                /* new card, or an explicit retry */
        s_req_mount = 0U;
    }
    {
        const bool card_ok = (sd_card_present != 0U) && (sd_test_result == HAL_OK)
                          && (sd_fatfs_ready != 0U);
        if ((s_mounted != 0U) && !card_ok)
        {
            detachCard();
        }
        else if ((s_mounted == 0U) && card_ok && (s_mount_gave_up == 0U))
        {
            if (SessionEdf_Mount())
            {
                g_session_edf_error = 0U;
                if (g_session_edf_state == (uint8_t)SESSION_EDF_ERROR)
                {
                    g_session_edf_state = (uint8_t)SESSION_EDF_IDLE;
                }
            }
            else
            {
                s_mount_gave_up = 1U;        /* hard FatFs failure: stop hammering SDMMC */
            }
        }
    }

    if (s_req_start != 0U)
    {
        s_req_start = 0U;
        if (g_session_edf_enable == 0U)
        {
            return;
        }
        if ((sd_card_present == 0U) || (sd_test_result != HAL_OK) || (sd_fatfs_ready == 0U))
        {
            /* Soft fail — keep start request until SD is ready, or user stops. */
            g_session_edf_error = 0x1001U;
            s_req_start = 1U;
            return;
        }
        if (s_edf.open)
        {
            (void)Edf_Close(&s_edf);
        }
        if (!openSessionFile())
        {
            /* openSessionFile already set err; leave ERROR so status shows it. */
            return;
        }
    }

    if (s_edf.open)
    {
        /* Pull new samples only when the previous SD write finished. */
        if (s_write_n == 0U)
        {
            uint8_t n;
            __disable_irq();
            n = s_pend_count;
            if (n != 0U)
            {
                memcpy(s_write_flat, s_pending_flat,
                       (size_t)n * sizeof(s_sample_buf));
                s_pend_count = 0U;
            }
            __enable_irq();
            s_write_n = n;
        }

        if (s_write_n != 0U)
        {
            if (Edf_WriteRecords(&s_edf, s_write_flat, s_write_n) != FR_OK)
            {
                s_write_fail_streak++;
                g_session_edf_error = 0x3000U; /* sticky; keep logging while retrying */
                if (s_write_fail_streak >= 8U)
                {
                    setErr(0x3000U);
                    (void)Edf_Close(&s_edf);
                    if (s_pld.open) { (void)Edf_Close(&s_pld); }
                    eveClose();
                    s_collecting = 0U;
                    s_write_n = 0U;
                    s_write_fail_streak = 0U;
                }
                /* else: leave s_write_n set and retry next Service tick */
            }
            else
            {
                g_session_edf_records = s_edf.n_records;
                g_session_edf_error = 0U;
                g_session_edf_fsize = (uint32_t)f_size(&s_edf.fil);
                s_write_n = 0U;
                s_write_fail_streak = 0U;
                servicePld();
                eveService();
            }
        }
    }
}

SessionEdfState_t SessionEdf_GetState(void)
{
    return (SessionEdfState_t)g_session_edf_state;
}

const char* SessionEdf_GetStateName(void)
{
    switch ((SessionEdfState_t)g_session_edf_state)
    {
    case SESSION_EDF_LOGGING: return "Logging";
    case SESSION_EDF_ERROR:   return "Error";
    case SESSION_EDF_IDLE:
    default:                  return "Idle";
    }
}

const char* SessionEdf_GetLastPath(void)
{
    return s_path;
}

uint32_t SessionEdf_GetRecords(void)
{
    return g_session_edf_records;
}

uint32_t SessionEdf_GetLastError(void)
{
    return g_session_edf_error;
}

uint32_t SessionEdf_GetPldRecords(void)
{
    return g_session_edf_pld_records;
}

uint32_t SessionEdf_GetPldError(void)
{
    return g_session_edf_pld_error;
}

const char* SessionEdf_GetPldPath(void)
{
    return s_pld_path;
}

bool SessionEdf_IsLogging(void)
{
    return (s_collecting != 0U) && (g_session_edf_enable != 0U);
}

uint32_t SessionEdf_NowTicks(void)
{
    return s_ticks;
}

void SessionEdf_LogEvent(SessionEdfEvent_t type, uint32_t end_ticks, float dur_s)
{
    if (!SessionEdf_IsLogging() || ((uint32_t)type >= (uint32_t)SESSION_EVT_COUNT))
    {
        g_session_edf_eve_dropped++;
        return;
    }
    float ds = dur_s * 10.0f;
    if (!(ds > 0.0f)) { ds = 0.0f; }
    if (ds > 65000.0f) { ds = 65000.0f; }
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const uint8_t next = (uint8_t)((s_eve_q_head + 1U) % EVE_Q_N);
    if (next == s_eve_q_tail)
    {
        g_session_edf_eve_dropped++;
    }
    else
    {
        s_eve_q[s_eve_q_head].type = (uint8_t)type;
        s_eve_q[s_eve_q_head].end_ticks = end_ticks;
        s_eve_q[s_eve_q_head].dur_ds = (uint16_t)(ds + 0.5f);
        s_eve_q_head = next;
    }
    __set_PRIMASK(primask);
}

uint32_t SessionEdf_GetEveRecords(void)
{
    return g_session_edf_eve_records;
}

uint32_t SessionEdf_GetEveError(void)
{
    return g_session_edf_eve_error;
}

const char* SessionEdf_GetEvePath(void)
{
    return s_eve.path;
}

uint32_t SessionEdf_GetCslRecords(void)
{
    return g_session_edf_csl_records;
}

uint32_t SessionEdf_GetCslError(void)
{
    return g_session_edf_csl_error;
}

uint32_t SessionEdf_GetFileSize(void)
{
    return g_session_edf_fsize;
}
