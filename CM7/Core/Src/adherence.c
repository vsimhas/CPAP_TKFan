/**
  ******************************************************************************
  * @file    adherence.c
  * @brief   Adherence metrics from SUMMARY.CSV (see adherence.h).
  ******************************************************************************
  */

#include "adherence.h"
#include "session_summary.h"
#include "session_edf_log.h"
#include "rtc_clock.h"
#include "ff.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SUMMARY_PATH   "0:/SUMMARY.CSV"
#define ADH_RING_DAYS  128U                 /* > window, power of two */
#define ADH_LINE_MAX   400U
#define ADH_MAX_FIELDS 48
#define ADH_NO_DAY     INT32_MIN

typedef struct
{
    int col_start;
    int col_valid;
    int col_on;
} AdhCols_t;

/* Per-night usage for the recent window (ring keyed by day number) and for
 * the first 90 nights from first use (reimbursement test). */
static int32_t  s_ring_day[ADH_RING_DAYS];
static uint32_t s_ring_s[ADH_RING_DAYS];
static uint32_t s_init_s[ADH_INITIAL_DAYS];
static int32_t  s_first_day = ADH_NO_DAY;
static int32_t  s_last_day = ADH_NO_DAY;
static uint32_t s_sessions = 0U;
static uint32_t s_undated = 0U;
static uint64_t s_total_s = 0U;
static bool     s_built = false;
static uint32_t s_error = 0U;
static AdhCols_t s_own_cols = { -1, -1, -1 };

/* Days since 1970-01-01 for a civil date (H. Hinnant's algorithm). */
static int32_t daysFromCivil(int32_t y, int32_t m, int32_t d)
{
    y -= (m <= 2) ? 1 : 0;
    const int32_t era = ((y >= 0) ? y : (y - 399)) / 400;
    const int32_t yoe = y - (era * 400);
    const int32_t doy = ((153 * (m + ((m > 2) ? -3 : 9))) + 2) / 5 + d - 1;
    const int32_t doe = (yoe * 365) + (yoe / 4) - (yoe / 100) + doy;
    return (era * 146097) + doe - 719468;
}

static void civilFromDays(int32_t z, int32_t *y, int32_t *m, int32_t *d)
{
    z += 719468;
    const int32_t era = ((z >= 0) ? z : (z - 146096)) / 146097;
    const int32_t doe = z - (era * 146097);
    const int32_t yoe = (doe - (doe / 1460) + (doe / 36524) - (doe / 146096)) / 365;
    const int32_t doy = doe - ((365 * yoe) + (yoe / 4) - (yoe / 100));
    const int32_t mp = ((5 * doy) + 2) / 153;
    *d = doy - (((153 * mp) + 2) / 5) + 1;
    *m = mp + ((mp < 10) ? 3 : -9);
    *y = (yoe + (era * 400)) + ((*m <= 2) ? 1 : 0);
}

/* Therapy night: the date, or the date before when the hour is before noon. */
static int32_t nightOf(int32_t y, int32_t mo, int32_t d, int32_t hh)
{
    return daysFromCivil(y, mo, d) - ((hh < 12) ? 1 : 0);
}

static void reset(void)
{
    for (uint32_t i = 0U; i < ADH_RING_DAYS; i++)
    {
        s_ring_day[i] = ADH_NO_DAY;
        s_ring_s[i] = 0U;
    }
    memset(s_init_s, 0, sizeof(s_init_s));
    s_first_day = ADH_NO_DAY;
    s_last_day = ADH_NO_DAY;
    s_sessions = 0U;
    s_undated = 0U;
    s_total_s = 0U;
}

static uint32_t ringGet(int32_t day)
{
    const uint32_t slot = (uint32_t)day & (ADH_RING_DAYS - 1U);
    return (s_ring_day[slot] == day) ? s_ring_s[slot] : 0U;
}

static void addUsage(int32_t day, uint32_t secs)
{
    const uint32_t slot = (uint32_t)day & (ADH_RING_DAYS - 1U);
    uint32_t ring_secs = secs;
    if (s_ring_day[slot] != day)
    {
        if ((s_ring_day[slot] != ADH_NO_DAY) && (day < s_ring_day[slot]))
        {
            ring_secs = 0U;              /* older than what the slot holds now */
        }
        else
        {
            s_ring_day[slot] = day;
            s_ring_s[slot] = 0U;
        }
    }
    s_ring_s[slot] += ring_secs;

    if (s_first_day == ADH_NO_DAY)
    {
        s_first_day = day;               /* the file is in time order */
    }
    if ((day >= s_first_day) && ((day - s_first_day) < (int32_t)ADH_INITIAL_DAYS))
    {
        s_init_s[day - s_first_day] += secs;
    }
    if ((s_last_day == ADH_NO_DAY) || (day > s_last_day))
    {
        s_last_day = day;
    }
}

/* Split a CSV line in place (no quoting in our format; empty fields kept). */
static int splitFields(char *p, char *f[], int max)
{
    int n = 0;
    f[n++] = p;
    while ((*p != '\0') && (n < max))
    {
        if (*p == ',')
        {
            *p = '\0';
            f[n++] = p + 1;
        }
        else if ((*p == '\r') || (*p == '\n'))
        {
            *p = '\0';
            break;
        }
        p++;
    }
    return n;
}

/* Column indices from a CSV header line (by name, not position). */
static AdhCols_t parseHeader(const char *line)
{
    AdhCols_t c = { -1, -1, -1 };
    char tmp[ADH_LINE_MAX];
    char *f[ADH_MAX_FIELDS];
    (void)strncpy(tmp, line, sizeof(tmp) - 1U);
    tmp[sizeof(tmp) - 1U] = '\0';
    const int n = splitFields(tmp, f, ADH_MAX_FIELDS);
    for (int i = 0; i < n; i++)
    {
        if (strcmp(f[i], "start") == 0)     { c.col_start = i; }
        if (strcmp(f[i], "rtc_valid") == 0) { c.col_valid = i; }
        if (strcmp(f[i], "mask_on_s") == 0) { c.col_on = i; }
    }
    return c;
}

static void addRecord(const char *line, const AdhCols_t *c)
{
    if ((c->col_start < 0) || (c->col_valid < 0) || (c->col_on < 0))
    {
        return;
    }
    char tmp[ADH_LINE_MAX];
    char *f[ADH_MAX_FIELDS];
    (void)strncpy(tmp, line, sizeof(tmp) - 1U);
    tmp[sizeof(tmp) - 1U] = '\0';
    const int n = splitFields(tmp, f, ADH_MAX_FIELDS);
    if ((c->col_start >= n) || (c->col_valid >= n) || (c->col_on >= n))
    {
        return;
    }
    s_sessions++;
    unsigned y = 0U, mo = 0U, d = 0U, hh = 0U, mi = 0U, ss = 0U;
    const bool valid = (atoi(f[c->col_valid]) == 1)
                    && (sscanf(f[c->col_start], "%u-%u-%u %u:%u:%u",
                               &y, &mo, &d, &hh, &mi, &ss) == 6)
                    && (y >= 2000U) && (mo >= 1U) && (mo <= 12U)
                    && (d >= 1U) && (d <= 31U) && (hh <= 23U);
    if (!valid)
    {
        s_undated++;
        return;
    }
    const long on = strtol(f[c->col_on], NULL, 10);
    const uint32_t secs = (on > 0) ? (uint32_t)on : 0U;
    s_total_s += secs;
    addUsage(nightOf((int32_t)y, (int32_t)mo, (int32_t)d, (int32_t)hh), secs);
}

/* One header or data line from the file. */
static void takeLine(const char *line, AdhCols_t *c, bool *have_header)
{
    if ((line[0] == '#') || (line[0] == '\0'))
    {
        return;
    }
    if (strncmp(line, "schema,", 7U) == 0)
    {
        *c = parseHeader(line);          /* a newer header re-maps columns */
        *have_header = true;
        return;
    }
    if (*have_header)
    {
        addRecord(line, c);
    }
}

/* FatFs string functions are off in ffconf.h, so lines are assembled from
 * f_read blocks here. Over-long lines are skipped. */
static void rebuild(void)
{
    FIL f;
    static uint8_t blk[512];
    static char line[ADH_LINE_MAX];
    uint32_t ll = 0U;
    bool overlong = false;
    AdhCols_t c = { -1, -1, -1 };
    bool have_header = false;

    reset();
    s_built = true;                      /* a card without the file = no sessions */
    FRESULT fr = f_open(&f, SUMMARY_PATH, FA_READ);
    if (fr == FR_NO_FILE)
    {
        s_error = 0U;
        return;
    }
    if (fr != FR_OK)
    {
        s_error = 0x7100U | ((uint32_t)fr & 0xFFU);
        s_built = false;
        return;
    }
    for (;;)
    {
        UINT br = 0U;
        fr = f_read(&f, blk, sizeof(blk), &br);
        if ((fr != FR_OK) || (br == 0U))
        {
            break;
        }
        for (UINT i = 0U; i < br; i++)
        {
            const char ch = (char)blk[i];
            if (ch == '\n')
            {
                line[ll] = '\0';
                if (!overlong)
                {
                    takeLine(line, &c, &have_header);
                }
                ll = 0U;
                overlong = false;
            }
            else if (ll < (ADH_LINE_MAX - 1U))
            {
                line[ll++] = ch;
            }
            else
            {
                overlong = true;
            }
        }
    }
    if ((ll > 0U) && !overlong)          /* last line without a newline */
    {
        line[ll] = '\0';
        takeLine(line, &c, &have_header);
    }
    (void)f_close(&f);
    s_error = (fr == FR_OK) ? 0U : (0x7200U | ((uint32_t)fr & 0xFFU));
}

void Adherence_Service(void)
{
    if (!SessionEdf_IsMounted())
    {
        if (s_built)
        {
            reset();                     /* nothing left over from the old card */
        }
        s_built = false;                 /* rebuild for whichever card is next */
        return;
    }
    if (!s_built)
    {
        rebuild();
    }
}

void Adherence_OnRecordWritten(const char *csv_line)
{
    if (!s_built || (csv_line == NULL))
    {
        return;                          /* the next rebuild reads it from the card */
    }
    if (s_own_cols.col_start < 0)
    {
        s_own_cols = parseHeader(SessionSummary_GetHeaderLine());
    }
    addRecord(csv_line, &s_own_cols);
}

bool Adherence_IsBuilt(void)
{
    return s_built;
}

int Adherence_Format(char *buf, unsigned len)
{
    if ((buf == NULL) || (len == 0U))
    {
        return 0;
    }
    if (!s_built || (s_first_day == ADH_NO_DAY))
    {
        const int n = snprintf(buf, len,
                               "# ADH built=%u sessions=%lu undated=%lu err=0x%lX\r\n",
                               s_built ? 1U : 0U, (unsigned long)s_sessions,
                               (unsigned long)s_undated, (unsigned long)s_error);
        return ((n > 0) && ((unsigned)n < len)) ? n : 0;
    }

    /* Tonight from the RTC; without a valid clock, the last night with a
     * session stands in. */
    int32_t today = s_last_day;
    RtcDateTime_t now;
    const bool clock_ok = RtcClock_IsTimeValid() && RtcClock_Get(&now);
    if (clock_ok)
    {
        today = nightOf(2000 + (int32_t)now.yy, (int32_t)now.mo, (int32_t)now.dd,
                        (int32_t)now.hh);
    }

    /* The last window, but never before first use. */
    int32_t from = today - (int32_t)ADH_WINDOW_DAYS + 1;
    if (from < s_first_day) { from = s_first_day; }
    uint32_t n_days = 0U, used = 0U, ge4 = 0U;
    uint64_t win_s = 0U;
    for (int32_t day = from; day <= today; day++)
    {
        const uint32_t s = ringGet(day);
        n_days++;
        win_s += s;
        if (s > 0U) { used++; }
        if (s >= ADH_THRESHOLD_S) { ge4++; }
    }

    /* Reimbursement test: the best 30-night window inside the first 90. */
    const int32_t since = today - s_first_day + 1;
    uint32_t span = ADH_INITIAL_DAYS;
    if (since < (int32_t)ADH_INITIAL_DAYS)
    {
        span = (since > 0) ? (uint32_t)since : 0U;
    }
    uint32_t best = 0U;
    for (uint32_t s0 = 0U; (s0 + ADH_WINDOW_DAYS) <= span; s0++)
    {
        uint32_t k = 0U;
        for (uint32_t i = 0U; i < ADH_WINDOW_DAYS; i++)
        {
            if (s_init_s[s0 + i] >= ADH_THRESHOLD_S) { k++; }
        }
        if (k > best) { best = k; }
    }
    if (span < ADH_WINDOW_DAYS)          /* fewer than 30 nights so far */
    {
        for (uint32_t i = 0U; i < span; i++)
        {
            if (s_init_s[i] >= ADH_THRESHOLD_S) { best++; }
        }
    }

    int32_t ty, tm, td, fy, fm, fd;
    civilFromDays(today, &ty, &tm, &td);
    civilFromDays(s_first_day, &fy, &fm, &fd);
    const unsigned pct = (n_days > 0U) ? (unsigned)(((100U * ge4) + (n_days / 2U)) / n_days) : 0U;
    const unsigned avg_all = (n_days > 0U) ? (unsigned)(win_s / 60U / n_days) : 0U;
    const unsigned avg_used = (used > 0U) ? (unsigned)(win_s / 60U / used) : 0U;
    const int n = snprintf(buf, len,
        "# ADH built=1 clock=%u night=%04ld-%02ld-%02ld days=%lu used=%lu ge4h=%lu pct4h=%u "
        "avg_min=%u avg_used_min=%u first=%04ld-%02ld-%02ld since=%ld sessions=%lu "
        "undated=%lu total_h=%lu best30=%lu%s met=%u\r\n",
        clock_ok ? 1U : 0U, (long)ty, (long)tm, (long)td,
        (unsigned long)n_days, (unsigned long)used, (unsigned long)ge4, pct,
        avg_all, avg_used, (long)fy, (long)fm, (long)fd, (long)since,
        (unsigned long)s_sessions, (unsigned long)s_undated,
        (unsigned long)(s_total_s / 3600U), (unsigned long)best,
        (span < ADH_WINDOW_DAYS) ? "/partial" : "",
        (best >= ADH_COMPLIANT_NIGHTS) ? 1U : 0U);
    return ((n > 0) && ((unsigned)n < len)) ? n : 0;
}
