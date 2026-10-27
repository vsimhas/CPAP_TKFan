/**
  ******************************************************************************
  * @file    session_summary.c
  * @brief   Per-session summary record -> 0:/SUMMARY.CSV (see the header).
  ******************************************************************************
  */

#include "session_summary.h"
#include "session_edf_log.h"
#include "rtc_clock.h"
#include "therapy.h"
#include "leak_estimator.h"
#include "apap_ctrl.h"
#include "periodic_breath.h"
#include "adherence.h"
#include "ff.h"
#include "main.h"

#include <stdio.h>
#include <string.h>

#define SUMMARY_PATH  "0:/SUMMARY.CSV"

static const char k_header[] =
    "schema,start,end,rtc_valid,run_s,mask_on_s,mask_off_s,mask_off_n,"
    "mode,p_cpap,apap_min,apap_max,epr_type,epr_level,ramp_min,"
    "p_avg,p_min,p_max,p50,p90,p95,"
    "leak_avg,leak_p50,leak_p95,leak_max,leak_high_s,leak_low_s,"
    "events_scored,oa,ca,hyp,fl,snore,ahi,rdi,si,edf,"
    "fl_idx_avg,fl_idx_p95,fl_s,snore_s,"
    "csr_s,csr_n,ramp_auto,onset_s,onset_cause";

static RtcDateTime_t s_start;
static bool          s_start_valid = false;
/* Mode and settings as the session ran, latched at start: they can change
 * before the stop (the bench harness's shutdown sets 4 cmH2O, which also
 * switches to CPAP mode - snapshotting them at stop logged an AutoSet
 * session as CPAP with no events). */
static TherapyMode_t s_mode = THERAPY_MODE_CPAP;
static float         s_p_cpap = 0.0f;
static float         s_apap_min = 0.0f;
static float         s_apap_max = 0.0f;
static uint8_t       s_epr_type = 0U;
static uint8_t       s_epr_level = 0U;
static uint8_t       s_ramp_min = 0U;
static uint8_t       s_ramp_auto = 0U;
static char          s_line[448];
static bool          s_has_last = false;
static volatile bool s_pending = false;
static uint32_t      s_written = 0U;
static uint32_t      s_error = 0U;

static void fmtTime(char *buf, size_t len, bool ok, const RtcDateTime_t *dt)
{
    if (ok)
    {
        (void)snprintf(buf, len, "20%02u-%02u-%02u %02u:%02u:%02u",
                       (unsigned)dt->yy, (unsigned)dt->mo, (unsigned)dt->dd,
                       (unsigned)dt->hh, (unsigned)dt->mi, (unsigned)dt->ss);
    }
    else
    {
        buf[0] = '\0';
    }
}

static const char* modeName(TherapyMode_t m)
{
    switch (m)
    {
    case THERAPY_MODE_AUTOSET:         return "AutoSet";
    case THERAPY_MODE_AUTOSET_FOR_HER: return "AutoSetForHer";
    case THERAPY_MODE_CPAP:
    default:                           return "CPAP";
    }
}

/* EDF session name, e.g. "20261026_140818", from the BRP path. */
static void edfBaseName(char *buf, size_t len)
{
    const char *p = SessionEdf_GetLastPath();
    const char *slash = strrchr(p, '/');
    const char *name = (slash != NULL) ? (slash + 1) : p;
    const char *tail = strstr(name, "_BRP.edf");
    size_t n = (tail != NULL) ? (size_t)(tail - name) : strlen(name);
    if (n >= len) { n = len - 1U; }
    memcpy(buf, name, n);
    buf[n] = '\0';
}

void SessionSummary_OnTherapyStart(void)
{
    s_start_valid = RtcClock_Get(&s_start);
    s_mode = Therapy_GetMode();
    s_p_cpap = Therapy_GetTargetPressure();
    s_apap_min = Therapy_GetApapMinPressure();
    s_apap_max = Therapy_GetApapMaxPressure();
    s_epr_type = Therapy_GetEprType();
    s_epr_level = Therapy_GetEprLevel();
    s_ramp_min = Therapy_GetRampMinutes();
    s_ramp_auto = Therapy_GetRampAuto() ? 1U : 0U;
}

void SessionSummary_OnTherapyStop(void)
{
    char t_start[24];
    char t_end[24];
    char edf[24];
    RtcDateTime_t end;
    const bool end_ok = RtcClock_Get(&end);

    fmtTime(t_start, sizeof(t_start), s_start_valid, &s_start);
    fmtTime(t_end, sizeof(t_end), end_ok, &end);
    edfBaseName(edf, sizeof(edf));

    const uint32_t run_s = Therapy_GetRunSeconds();
    const uint32_t off_s = Therapy_GetSessionMaskOffSeconds();
    const uint32_t on_s = (run_s > off_s) ? (run_s - off_s) : 0U;
    /* Events are scored by the APAP detectors: AutoSet, and CPAP unless
     * scoring is switched off there (apap set cpap_scoring 0). */
    const bool scored = Apap_WasSessionScored();
    const uint32_t oa = scored ? Apap_GetEventOaCount() : 0U;
    const uint32_t ca = scored ? Apap_GetEventCaCount() : 0U;
    const uint32_t hyp = scored ? Apap_GetEventHypCount() : 0U;
    const uint32_t fl = scored ? Apap_GetEventFlCount() : 0U;
    const uint32_t sn = scored ? Apap_GetEventSnoreCount() : 0U;
    /* Indices per hour of mask-on time, as ResMed reports them. */
    const float h = (float)on_s / 3600.0f;
    const float ahi = (h > 0.0167f) ? ((float)(oa + ca + hyp) / h) : 0.0f;
    const float rdi = (h > 0.0167f) ? ((float)(oa + ca + hyp + fl) / h) : 0.0f;
    const float si  = (h > 0.0167f) ? ((float)sn / h) : 0.0f;

    const int n = snprintf(s_line, sizeof(s_line),
        "%u,%s,%s,%u,%lu,%lu,%lu,%lu,"
        "%s,%.1f,%.1f,%.1f,%u,%u,%u,"
        "%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,"
        "%.1f,%.1f,%.1f,%.1f,%.0f,%.0f,"
        "%u,%lu,%lu,%lu,%lu,%lu,%.2f,%.2f,%.2f,%s,"
        "%.2f,%.2f,%lu,%lu,"
        "%lu,%lu,%u,%lu,%u",
        (unsigned)SESSION_SUMMARY_SCHEMA, t_start, t_end,
        (RtcClock_IsTimeValid() && s_start_valid) ? 1U : 0U,
        (unsigned long)run_s, (unsigned long)on_s, (unsigned long)off_s,
        (unsigned long)Therapy_GetMaskOffEnterCount(),
        modeName(s_mode),
        (double)s_p_cpap, (double)s_apap_min, (double)s_apap_max,
        (unsigned)s_epr_type, (unsigned)s_epr_level, (unsigned)s_ramp_min,
        (double)Therapy_GetSessionPressureAvg(), (double)Therapy_GetSessionPressureMin(),
        (double)Therapy_GetSessionPressureMax(), (double)Therapy_GetSessionPressureP50(),
        (double)Therapy_GetSessionPressureP90(), (double)Therapy_GetSessionPressureP95(),
        (double)Leak_GetSessionAvgSlm(), (double)Leak_GetSessionPercentileSlm(50.0f),
        (double)Leak_GetSessionPercentileSlm(95.0f), (double)Leak_GetSessionMaxSlm(),
        (double)Leak_GetSessionHighTimeS(), (double)Leak_GetSessionLowTimeS(),
        scored ? 1U : 0U, (unsigned long)oa, (unsigned long)ca, (unsigned long)hyp,
        (unsigned long)fl, (unsigned long)sn,
        (double)ahi, (double)rdi, (double)si, edf,
        (double)(scored ? Apap_GetSessionFlIndexAvg() : 0.0f),
        (double)(scored ? Apap_GetSessionFlIndexP95() : 0.0f),
        (unsigned long)(scored ? Apap_GetSessionFlSeconds() : 0U),
        (unsigned long)(scored ? Apap_GetSessionSnoreSeconds() : 0U),
        (unsigned long)(scored ? PeriodicBreath_GetSessionSeconds() : 0U),
        (unsigned long)(scored ? PeriodicBreath_GetEpisodes() : 0U),
        (unsigned)s_ramp_auto,
        (unsigned long)((s_ramp_auto != 0U) ? Therapy_GetAutoRampOnsetSeconds() : 0U),
        (unsigned)((s_ramp_auto != 0U) ? Therapy_GetAutoRampCause() : 0U));
    if ((n > 0) && ((size_t)n < sizeof(s_line)))
    {
        s_has_last = true;
        s_pending = true;
    }
}

/* True if the last "schema," line in the (open, append-mode) file is our
 * header. Reads the whole file - one short line per session - then leaves
 * the pointer at the end again. */
static bool lastHeaderIsCurrent(FIL *f)
{
    static char buf[256];
    static char line[sizeof(k_header) + 8U];
    const FSIZE_t end = f_size(f);
    size_t ln = 0U;
    bool overflow = false;
    bool current = false;
    bool seen = false;
    if (f_lseek(f, 0U) != FR_OK)
    {
        return true;            /* cannot tell: do not add a header */
    }
    for (;;)
    {
        UINT br = 0U;
        if ((f_read(f, buf, sizeof(buf), &br) != FR_OK) || (br == 0U))
        {
            break;
        }
        for (UINT i = 0U; i < br; i++)
        {
            const char c = buf[i];
            if ((c == '\n') || (c == '\r'))
            {
                if (ln > 0U)
                {
                    line[ln] = '\0';
                    if (strncmp(line, "schema,", 7U) == 0)
                    {
                        seen = true;
                        current = !overflow && (strcmp(line, k_header) == 0);
                    }
                }
                ln = 0U;
                overflow = false;
                continue;
            }
            if (ln < (sizeof(line) - 1U))
            {
                line[ln++] = c;
            }
            else
            {
                overflow = true;
            }
        }
    }
    (void)f_lseek(f, end);
    return seen ? current : true;
}

void SessionSummary_Service(void)
{
    if (!s_pending || !SessionEdf_IsMounted())
    {
        return;
    }
    FIL f;
    UINT bw = 0U;
    FRESULT fr = f_open(&f, SUMMARY_PATH, FA_OPEN_APPEND | FA_WRITE | FA_READ);
    if (fr != FR_OK)
    {
        s_error = 0x6100U | ((uint32_t)fr & 0xFFU);
        s_pending = false;             /* do not hammer the card */
        return;
    }
    /* A file started under an older schema gets the current header line
     * before this record; readers re-map columns at every header line. */
    if ((fr == FR_OK) && (f_size(&f) > 0U) && !lastHeaderIsCurrent(&f))
    {
        char hdr[sizeof(k_header) + 64U];
        const int hn = snprintf(hdr, sizeof(hdr), "# schema %u\r\n%s\r\n",
                                (unsigned)SESSION_SUMMARY_SCHEMA, k_header);
        if (hn > 0)
        {
            fr = f_write(&f, hdr, (UINT)hn, &bw);
        }
    }
    if (f_size(&f) == 0U)
    {
        char hdr[sizeof(k_header) + 64U];
        const int hn = snprintf(hdr, sizeof(hdr),
                                "# SomnaLink session summary, schema %u\r\n%s\r\n",
                                (unsigned)SESSION_SUMMARY_SCHEMA, k_header);
        if (hn > 0)
        {
            fr = f_write(&f, hdr, (UINT)hn, &bw);
        }
    }
    if (fr == FR_OK)
    {
        fr = f_write(&f, s_line, (UINT)strlen(s_line), &bw);
    }
    if (fr == FR_OK)
    {
        fr = f_write(&f, "\r\n", 2U, &bw);
    }
    const FRESULT cr = f_close(&f);
    if ((fr == FR_OK) && (cr == FR_OK))
    {
        s_written++;
        s_error = 0U;
        Adherence_OnRecordWritten(s_line);   /* #65 */
    }
    else
    {
        s_error = 0x6200U | ((uint32_t)((fr != FR_OK) ? fr : cr) & 0xFFU);
    }
    s_pending = false;
}

bool        SessionSummary_HasLast(void)       { return s_has_last; }
const char* SessionSummary_GetLastLine(void)   { return s_has_last ? s_line : ""; }
const char* SessionSummary_GetHeaderLine(void) { return k_header; }
bool        SessionSummary_IsPending(void)     { return s_pending; }
uint32_t    SessionSummary_GetWritten(void)    { return s_written; }
uint32_t    SessionSummary_GetError(void)      { return s_error; }
