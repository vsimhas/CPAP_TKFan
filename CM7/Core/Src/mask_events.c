/**
  ******************************************************************************
  * @file    mask_events.c
  * @brief   Timestamped mask-off / mask-on episodes (see mask_events.h).
  ******************************************************************************
  */

#include "mask_events.h"
#include "session_edf_log.h"
#include "rtc_clock.h"
#include "dbg_log.h"
#include "ff.h"
#include "main.h"

#include <stdio.h>
#include <string.h>

#define MASK_PATH     "0:/MASK.CSV"
#define MASK_SCHEMA   1U
#define MASK_RING_N   16U
/* A mask-on is held this long before it is written: therapy.c re-parks a
 * reconnect that does not pressurise (probation, g_therapy_mask_off_verify_ms)
 * and that is the same disconnection, not a new one. */
#define MASK_HOLD_MS  20000U

typedef struct
{
    RtcDateTime_t start;      /* session start, as SUMMARY.CSV */
    RtcDateTime_t off;
    RtcDateTime_t on;
    uint8_t       rtc_valid;
    uint8_t       path;
    uint8_t       by_stop;
    uint32_t      off_s;      /* seconds into the session */
    uint32_t      on_s;
    float         dur_s;
} MaskRow_t;

static const char * const k_path[] = {
    "probation", "flow", "ratio", "pressure", "rpm_ratio", "sustained",
};

static RtcDateTime_t s_start;
static uint8_t       s_start_valid;
static uint32_t      s_start_ms;
static bool          s_open;
static MaskRow_t     s_cur;
static uint32_t      s_off_ms;
static uint32_t      s_session_n;
static MaskRow_t     s_last;
static bool          s_pending;      /* closed, waiting out a re-park */
static uint32_t      s_pending_ms;
static uint32_t      s_end_ticks;
static bool          s_have_last;

static MaskRow_t         s_ring[MASK_RING_N];
static volatile uint32_t s_rows = 0U;
static volatile uint32_t s_written = 0U;
static uint32_t          s_err = 0U;

static uint8_t nowRtc(RtcDateTime_t *dt)
{
    return (RtcClock_IsTimeValid() && RtcClock_Get(dt)) ? 1U : 0U;
}

void MaskEvents_SessionStart(void)
{
    s_start_valid = nowRtc(&s_start);
    s_start_ms = HAL_GetTick();
    s_open = false;
    s_pending = false;
    s_session_n = 0U;
}

/* Write the closed episode out: EVE annotation (stamped where it ended) and
 * the MASK.CSV queue. */
static void emit(void)
{
    s_pending = false;
    s_session_n++;
    s_last = s_cur;
    s_have_last = true;
    SessionEdf_LogEvent(SESSION_EVT_MASK_OFF, s_end_ticks, s_cur.dur_s);

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_ring[s_rows % MASK_RING_N] = s_cur;
    s_rows++;
    if ((s_rows - s_written) > MASK_RING_N)
    {
        s_written = s_rows - MASK_RING_N;
    }
    __set_PRIMASK(primask);
}

void MaskEvents_Off(uint8_t path)
{
    if (s_open)
    {
        return;
    }
    if (s_pending && (path == 0U))
    {
        /* Probation re-park: that mask-on was wrong, same episode goes on. */
        s_pending = false;
        s_open = true;
        return;
    }
    if (s_pending)
    {
        emit();
    }
    memset(&s_cur, 0, sizeof(s_cur));
    s_cur.start = s_start;
    s_cur.rtc_valid = (uint8_t)(s_start_valid & nowRtc(&s_cur.off));
    s_cur.path = path;
    s_off_ms = HAL_GetTick();
    s_cur.off_s = (s_off_ms - s_start_ms) / 1000U;
    s_open = true;
}

void MaskEvents_On(void)
{
    if (!s_open)
    {
        return;
    }
    s_open = false;
    const uint32_t now = HAL_GetTick();
    if (nowRtc(&s_cur.on) == 0U)
    {
        s_cur.rtc_valid = 0U;
    }
    s_cur.on_s = (now - s_start_ms) / 1000U;
    s_cur.dur_s = (float)(now - s_off_ms) * 0.001f;
    s_cur.by_stop = 0U;
    s_end_ticks = SessionEdf_NowTicks();
    s_pending = true;                    /* until the reconnect holds */
    s_pending_ms = 0U;
}

void MaskEvents_Update(uint32_t dt_ms)
{
    if (s_pending)
    {
        s_pending_ms += dt_ms;
        if (s_pending_ms >= MASK_HOLD_MS)
        {
            emit();
        }
    }
}

void MaskEvents_SessionEnd(void)
{
    if (s_open)
    {
        MaskEvents_On();
        s_cur.by_stop = 1U;
    }
    if (s_pending)
    {
        emit();
    }
}

static void fmtTime(const RtcDateTime_t *dt, uint8_t valid, char *buf, size_t len)
{
    if (valid != 0U)
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

static const char* pathName(uint8_t p)
{
    return (p < (sizeof(k_path) / sizeof(k_path[0]))) ? k_path[p] : "?";
}

static int formatRow(const MaskRow_t *r, char *buf, unsigned len)
{
    char ts[24], toff[24], ton[24];
    fmtTime(&r->start, r->rtc_valid, ts, sizeof(ts));
    fmtTime(&r->off, r->rtc_valid, toff, sizeof(toff));
    fmtTime(&r->on, r->rtc_valid, ton, sizeof(ton));
    const int n = snprintf(buf, len, "%u,%s,%u,%s,%lu,%s,%lu,%.1f,%s,%s\r\n",
                           (unsigned)MASK_SCHEMA, ts, (unsigned)r->rtc_valid,
                           toff, (unsigned long)r->off_s, ton, (unsigned long)r->on_s,
                           (double)r->dur_s, pathName(r->path),
                           (r->by_stop != 0U) ? "stop" : "mask_on");
    return ((n > 0) && ((unsigned)n < len)) ? n : 0;
}

void MaskEvents_Service(void)
{
    if ((s_written == s_rows) || !SessionEdf_IsMounted())
    {
        return;
    }
    FIL f;
    UINT bw = 0U;
    FRESULT fr = f_open(&f, MASK_PATH, FA_OPEN_APPEND | FA_WRITE);
    if (fr != FR_OK)
    {
        s_err = 0xB100U | ((uint32_t)fr & 0xFFU);
        return;
    }
    if (f_size(&f) == 0U)
    {
        static const char hdr[] =
            "# SomnaLink mask-off episodes, schema 1\r\n"
            "schema,session_start,rtc_valid,off_time,off_s,on_time,on_s,dur_s,path,end\r\n";
        fr = f_write(&f, hdr, (UINT)(sizeof(hdr) - 1U), &bw);
    }
    while ((fr == FR_OK) && (s_written != s_rows))
    {
        char line[160];
        MaskRow_t r;
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        r = s_ring[s_written % MASK_RING_N];
        __set_PRIMASK(primask);
        const int n = formatRow(&r, line, sizeof(line));
        if (n > 0)
        {
            fr = f_write(&f, line, (UINT)n, &bw);
        }
        if (fr == FR_OK)
        {
            s_written++;
        }
    }
    const FRESULT cr = f_close(&f);
    s_err = ((fr == FR_OK) && (cr == FR_OK)) ? 0U
          : (0xB200U | ((uint32_t)((fr != FR_OK) ? fr : cr) & 0xFFU));
}

uint32_t MaskEvents_GetSessionCount(void) { return s_session_n; }
uint32_t MaskEvents_GetRows(void)         { return s_written; }
uint32_t MaskEvents_GetError(void)        { return s_err; }

int MaskEvents_FormatStatus(char *buf, unsigned len)
{
    char toff[24], ton[24];
    const MaskRow_t *r = &s_last;
    fmtTime(&r->off, s_have_last ? r->rtc_valid : 0U, toff, sizeof(toff));
    fmtTime(&r->on, s_have_last ? r->rtc_valid : 0U, ton, sizeof(ton));
    for (unsigned i = 0U; (i < sizeof(toff)) && (toff[i] != '\0'); i++)
    {
        if (toff[i] == ' ') { toff[i] = 'T'; }
    }
    for (unsigned i = 0U; (i < sizeof(ton)) && (ton[i] != '\0'); i++)
    {
        if (ton[i] == ' ') { ton[i] = 'T'; }
    }
    const int n = snprintf(buf, len,
        "# MASK n=%lu open=%u pending=%u last_off=%s last_off_s=%lu last_on=%s last_on_s=%lu "
        "dur=%.1f path=%s end=%s rows=%lu err=0x%04lX\r\n",
        (unsigned long)s_session_n, s_open ? 1U : 0U, s_pending ? 1U : 0U,
        (toff[0] != '\0') ? toff : "-", (unsigned long)r->off_s,
        (ton[0] != '\0') ? ton : "-", (unsigned long)r->on_s, (double)r->dur_s,
        s_have_last ? pathName(r->path) : "-",
        s_have_last ? ((r->by_stop != 0U) ? "stop" : "mask_on") : "-",
        (unsigned long)s_written, (unsigned long)s_err);
    return ((n > 0) && ((unsigned)n < len)) ? n : 0;
}
