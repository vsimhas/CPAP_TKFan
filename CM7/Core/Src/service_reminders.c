/**
  ******************************************************************************
  * @file    service_reminders.c
  * @brief   Supplies replacement reminders (see service_reminders.h).
  ******************************************************************************
  */

#include "service_reminders.h"
#include "adherence.h"
#include "precheck.h"
#include "therapy.h"
#include "session_edf_log.h"
#include "rtc_clock.h"
#include "dbg_log.h"
#include "ff.h"
#include "main.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#define SVC_PATH          "0:/SERVICE.CSV"
#define SVC_SCHEMA        1U
#define SVC_HOURS_PER_DAY 8.0f      /* the hour limit = the period at 8 h a night */
#define SVC_SOON_FRAC     0.9f
#define SVC_SPIN_N        3U        /* starts in the reference / recent median */
#define SVC_BLOCK_DROP    0.5f      /* cmH2O below the reference: blocked */
#define SVC_RING_N        8U
#define SVC_LINE_MAX      160U

typedef enum { EV_START = 0, EV_REPLACED, EV_SPIN } SvcEvent_t;

typedef struct
{
    bool          have;
    uint8_t       dt_valid;
    RtcDateTime_t dt;
    float         run_h;      /* device usage hours when replaced */
} ItemRec_t;

typedef struct
{
    uint8_t       event;
    uint8_t       item;
    uint8_t       dt_valid;
    RtcDateTime_t dt;
    uint32_t      uptime_ms;
    float         run_h;
    float         value;
} SvcRow_t;

static const char * const k_name[SVC_COUNT]  = { "Filter", "Mask cushion", "Mask", "Tube",
                                                 "Water chamber" };
static const char * const k_token[SVC_COUNT] = { "filter", "cushion", "mask", "tube",
                                                 "chamber" };
static const char * const k_event[]          = { "start", "replaced", "spin" };
static const int32_t      k_days[SVC_COUNT]  = { 30, 30, 90, 90, 180 };

static ItemRec_t s_rec[SVC_COUNT];
static bool      s_loaded = false;
static bool      s_ready = false;
/* Filter blockage: spin residuals since the filter was replaced. */
static float     s_base[SVC_SPIN_N];
static uint8_t   s_base_n = 0U;
static float     s_recent[SVC_SPIN_N];
static uint8_t   s_recent_n = 0U;
static uint8_t   s_recent_i = 0U;
static uint32_t  s_spin_seq_seen = 0U;
static bool      s_active_prev = false;
/* Bench offsets. */
static float     s_test_h = 0.0f;
static int32_t   s_test_d = 0;
static float     s_test_spin = 0.0f;
static char      s_due_text[48];

static SvcRow_t          s_ring[SVC_RING_N];
static volatile uint32_t s_rows = 0U;
static volatile uint32_t s_written = 0U;
static uint32_t          s_err = 0U;

/* Days since 1970-01-01 (H. Hinnant). */
static int32_t dayNumber(const RtcDateTime_t *dt)
{
    int32_t y = 2000 + (int32_t)dt->yy;
    const int32_t m = (int32_t)dt->mo;
    const int32_t d = (int32_t)dt->dd;
    y -= (m <= 2) ? 1 : 0;
    const int32_t era = ((y >= 0) ? y : (y - 399)) / 400;
    const int32_t yoe = y - (era * 400);
    const int32_t doy = ((153 * (m + ((m > 2) ? -3 : 9))) + 2) / 5 + d - 1;
    const int32_t doe = (yoe * 365) + (yoe / 4) - (yoe / 100) + doy;
    return (era * 146097) + doe - 719468;
}

static uint8_t nowRtc(RtcDateTime_t *dt)
{
    return (RtcClock_IsTimeValid() && RtcClock_Get(dt)) ? 1U : 0U;
}

static bool sessionActive(void)
{
    const TherapyState_t st = Therapy_GetState();
    return (st == THERAPY_STATE_RAMP) || (st == THERAPY_STATE_RUNNING);
}

/* Usage hours: every session on the card plus the one running now. */
static float deviceHours(void)
{
    const uint64_t s = Adherence_GetRunSeconds()
                     + (sessionActive() ? (uint64_t)Therapy_GetRunSeconds() : 0U);
    return ((float)s / 3600.0f) + s_test_h;
}

static float median3(const float *v, uint8_t n)
{
    if (n == 0U) { return 0.0f; }
    if (n == 1U) { return v[0]; }
    if (n == 2U) { return 0.5f * (v[0] + v[1]); }
    const float a = v[0], b = v[1], c = v[2];
    return fmaxf(fminf(a, b), fminf(fmaxf(a, b), c));
}

static bool filterBlocked(float *base, float *now)
{
    *base = median3(s_base, s_base_n);
    *now = median3(s_recent, s_recent_n);
    return (s_base_n >= SVC_SPIN_N) && (s_recent_n >= SVC_SPIN_N)
        && ((*base - *now) >= SVC_BLOCK_DROP);
}

static void spinReset(void)
{
    s_base_n = 0U;
    s_recent_n = 0U;
    s_recent_i = 0U;
}

static void spinAdd(float r)
{
    if (s_base_n < SVC_SPIN_N)
    {
        s_base[s_base_n++] = r;
    }
    s_recent[s_recent_i] = r;
    s_recent_i = (uint8_t)((s_recent_i + 1U) % SVC_SPIN_N);
    if (s_recent_n < SVC_SPIN_N) { s_recent_n++; }
}

static void queue(SvcEvent_t ev, uint8_t item, uint8_t dt_valid, const RtcDateTime_t *dt,
                  float run_h, float value)
{
    SvcRow_t r;
    memset(&r, 0, sizeof(r));
    r.event = (uint8_t)ev;
    r.item = item;
    r.dt_valid = dt_valid;
    if (dt != NULL) { r.dt = *dt; }
    r.uptime_ms = HAL_GetTick();
    r.run_h = run_h;
    r.value = value;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_ring[s_rows % SVC_RING_N] = r;
    s_rows++;
    if ((s_rows - s_written) > SVC_RING_N)
    {
        s_written = s_rows - SVC_RING_N;
    }
    __set_PRIMASK(primask);
}

static void setItem(uint8_t item, uint8_t dt_valid, const RtcDateTime_t *dt, float run_h)
{
    s_rec[item].have = true;
    s_rec[item].dt_valid = dt_valid;
    if (dt != NULL) { s_rec[item].dt = *dt; }
    s_rec[item].run_h = run_h;
    if (item == (uint8_t)SVC_FILTER)
    {
        spinReset();
    }
}

/* === Public ================================================================ */

void Service_Update(uint32_t dt_ms)
{
    (void)dt_ms;
    /* One spin row per therapy start: the #83 verdict that let it start. */
    const bool active = sessionActive();
    if (active && !s_active_prev)
    {
        const PrecheckSpin_t *sp = Precheck_GetLastSpin();
        if ((sp->seq != s_spin_seq_seen) && (sp->fail == PRECHECK_OK) && (sp->n > 0U)
            && !sp->injected)
        {
            s_spin_seq_seen = sp->seq;
            const float r = (sp->p_mean - sp->p_model) + s_test_spin;
            spinAdd(r);
            RtcDateTime_t dt;
            const uint8_t v = nowRtc(&dt);
            queue(EV_SPIN, (uint8_t)SVC_FILTER, v, &dt, deviceHours(), r);
        }
    }
    s_active_prev = active;
}

bool Service_IsReady(void) { return s_ready; }

void Service_GetStatus(SvcItem_t item, SvcStatus_t *out)
{
    memset(out, 0, sizeof(*out));
    out->days = -1;
    if ((uint32_t)item >= (uint32_t)SVC_COUNT)
    {
        return;
    }
    out->limit_d = k_days[item];
    out->limit_h = (float)k_days[item] * SVC_HOURS_PER_DAY;
    if (!s_ready || !s_rec[item].have)
    {
        out->state = SVC_UNKNOWN;
        return;
    }
    out->used_h = fmaxf(0.0f, deviceHours() - s_rec[item].run_h);
    RtcDateTime_t now;
    if ((s_rec[item].dt_valid != 0U) && (nowRtc(&now) != 0U))
    {
        out->days = (dayNumber(&now) - dayNumber(&s_rec[item].dt)) + s_test_d;
        if (out->days < 0) { out->days = 0; }
    }
    float frac = out->used_h / out->limit_h;
    if (out->days >= 0)
    {
        frac = fmaxf(frac, (float)out->days / (float)out->limit_d);
    }
    if (item == SVC_FILTER)
    {
        float b, n;
        out->blocked = filterBlocked(&b, &n);
    }
    out->state = ((frac >= 1.0f) || out->blocked) ? SVC_DUE
               : (frac >= SVC_SOON_FRAC) ? SVC_SOON : SVC_OK;
}

bool Service_AnyDue(void)
{
    for (uint32_t i = 0U; i < (uint32_t)SVC_COUNT; i++)
    {
        SvcStatus_t st;
        Service_GetStatus((SvcItem_t)i, &st);
        if (st.state == SVC_DUE) { return true; }
    }
    return false;
}

const char* Service_DueText(void)
{
    size_t n = 0U;
    s_due_text[0] = '\0';
    for (uint32_t i = 0U; i < (uint32_t)SVC_COUNT; i++)
    {
        SvcStatus_t st;
        Service_GetStatus((SvcItem_t)i, &st);
        if (st.state != SVC_DUE) { continue; }
        const int m = snprintf(&s_due_text[n], sizeof(s_due_text) - n, "%s%s",
                               (n > 0U) ? ", " : "", k_token[i]);
        if ((m < 0) || ((size_t)m >= (sizeof(s_due_text) - n))) { break; }
        n += (size_t)m;
    }
    return s_due_text;
}

void Service_Replaced(SvcItem_t item)
{
    if ((uint32_t)item >= (uint32_t)SVC_COUNT)
    {
        return;
    }
    RtcDateTime_t dt;
    const uint8_t v = nowRtc(&dt);
    const float h = deviceHours();
    setItem((uint8_t)item, v, &dt, h);
    queue(EV_REPLACED, (uint8_t)item, v, &dt, h, 0.0f);
    DBG_I("SVC", "%s replaced at %.1f h", k_token[item], (double)h);
}

const char* Service_ItemName(SvcItem_t item)
{
    return ((uint32_t)item < (uint32_t)SVC_COUNT) ? k_name[item] : "?";
}

const char* Service_ItemToken(SvcItem_t item)
{
    return ((uint32_t)item < (uint32_t)SVC_COUNT) ? k_token[item] : "?";
}

SvcItem_t Service_ParseItem(const char *s)
{
    for (uint32_t i = 0U; i < (uint32_t)SVC_COUNT; i++)
    {
        if (strcmp(s, k_token[i]) == 0) { return (SvcItem_t)i; }
    }
    return SVC_COUNT;
}

void Service_SetTest(float hours, int32_t days, float spin_cmh2o)
{
    s_test_h = fmaxf(0.0f, hours);
    s_test_d = (days > 0) ? days : 0;
    s_test_spin = spin_cmh2o;
}

void Service_SetTestHours(float hours)    { s_test_h = fmaxf(0.0f, hours); }
void Service_SetTestDays(int32_t days)     { s_test_d = (days > 0) ? days : 0; }
void Service_SetTestSpin(float spin_cmh2o) { s_test_spin = spin_cmh2o; }

static const char* stateName(SvcState_t s)
{
    switch (s)
    {
    case SVC_OK:   return "ok";
    case SVC_SOON: return "soon";
    case SVC_DUE:  return "due";
    default:       return "unknown";
    }
}

int Service_FormatStatus(uint32_t k, char *buf, unsigned len)
{
    int n = 0;
    if (k < (uint32_t)SVC_COUNT)
    {
        SvcStatus_t st;
        Service_GetStatus((SvcItem_t)k, &st);
        const ItemRec_t *r = &s_rec[k];
        char since[16] = "-";
        if (r->have && (r->dt_valid != 0U))
        {
            (void)snprintf(since, sizeof(since), "20%02u-%02u-%02u", (unsigned)r->dt.yy,
                           (unsigned)r->dt.mo, (unsigned)r->dt.dd);
        }
        n = snprintf(buf, len,
                     "# SVC item=%s state=%s used_h=%.1f limit_h=%.0f days=%ld limit_d=%ld "
                     "since=%s at_h=%.1f blocked=%u\r\n",
                     k_token[k], stateName(st.state), (double)st.used_h, (double)st.limit_h,
                     (long)st.days, (long)st.limit_d, since, (double)r->run_h,
                     st.blocked ? 1U : 0U);
    }
    else if (k == (uint32_t)SVC_COUNT)
    {
        float b, now;
        const bool blk = filterBlocked(&b, &now);
        n = snprintf(buf, len,
                     "# SVC blockage base=%.3f (n %u) recent=%.3f (n %u) drop=%.3f limit=%.2f "
                     "blocked=%u\r\n",
                     (double)b, (unsigned)s_base_n, (double)now, (unsigned)s_recent_n,
                     (double)(b - now), (double)SVC_BLOCK_DROP, blk ? 1U : 0U);
    }
    else if (k == ((uint32_t)SVC_COUNT + 1U))
    {
        n = snprintf(buf, len,
                     "# SVC ready=%u hours=%.2f card_run_h=%.2f test_h=%.1f test_d=%ld "
                     "test_spin=%.2f rows=%lu err=0x%04lX\r\n",
                     s_ready ? 1U : 0U, (double)deviceHours(),
                     (double)((float)Adherence_GetRunSeconds() / 3600.0f), (double)s_test_h,
                     (long)s_test_d, (double)s_test_spin, (unsigned long)s_written,
                     (unsigned long)s_err);
    }
    return ((n > 0) && ((unsigned)n < len)) ? n : 0;
}

/* === SERVICE.CSV ============================================================ */

static int formatRow(const SvcRow_t *r, char *buf, unsigned len)
{
    char t[24];
    if (r->dt_valid != 0U)
    {
        (void)snprintf(t, sizeof(t), "20%02u-%02u-%02u %02u:%02u:%02u",
                       (unsigned)r->dt.yy, (unsigned)r->dt.mo, (unsigned)r->dt.dd,
                       (unsigned)r->dt.hh, (unsigned)r->dt.mi, (unsigned)r->dt.ss);
    }
    else
    {
        t[0] = '\0';
    }
    const int n = snprintf(buf, len, "%u,%s,%u,%lu,%s,%s,%.3f,%.3f\r\n",
                           (unsigned)SVC_SCHEMA, t, (unsigned)r->dt_valid,
                           (unsigned long)(r->uptime_ms / 1000U), k_event[r->event],
                           k_token[r->item], (double)r->run_h, (double)r->value);
    return ((n > 0) && ((unsigned)n < len)) ? n : 0;
}

static void writePending(void)
{
    if ((s_written == s_rows) || !SessionEdf_IsMounted())
    {
        return;
    }
    FIL f;
    UINT bw = 0U;
    FRESULT fr = f_open(&f, SVC_PATH, FA_OPEN_APPEND | FA_WRITE);
    if (fr != FR_OK)
    {
        s_err = 0xC100U | ((uint32_t)fr & 0xFFU);
        return;
    }
    if (f_size(&f) == 0U)
    {
        static const char hdr[] =
            "# SomnaLink supplies / service log, schema 1\r\n"
            "schema,time,rtc_valid,uptime_s,event,item,run_h,value\r\n";
        fr = f_write(&f, hdr, (UINT)(sizeof(hdr) - 1U), &bw);
    }
    while ((fr == FR_OK) && (s_written != s_rows))
    {
        char line[SVC_LINE_MAX];
        SvcRow_t r;
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        r = s_ring[s_written % SVC_RING_N];
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
          : (0xC200U | ((uint32_t)((fr != FR_OK) ? fr : cr) & 0xFFU));
}

/* schema,time,rtc_valid,uptime_s,event,item,run_h,value */
static void takeLine(char *line)
{
    if ((line[0] < '0') || (line[0] > '9'))
    {
        return;
    }
    char *col[8];
    uint32_t nc = 0U;
    char *p = line;
    col[nc++] = p;
    while ((*p != '\0') && (nc < 8U))
    {
        if (*p == ',')
        {
            *p = '\0';
            col[nc++] = p + 1;
        }
        p++;
    }
    if ((nc < 8U) || (strtoul(col[0], NULL, 10) != SVC_SCHEMA))
    {
        return;
    }
    const SvcItem_t item = Service_ParseItem(col[5]);
    if (item == SVC_COUNT)
    {
        return;
    }
    const float run_h = strtof(col[6], NULL);
    if ((strcmp(col[4], "start") == 0) || (strcmp(col[4], "replaced") == 0))
    {
        RtcDateTime_t dt;
        memset(&dt, 0, sizeof(dt));
        unsigned y = 0U, mo = 0U, d = 0U, hh = 0U, mi = 0U, ss = 0U;
        uint8_t v = (strcmp(col[2], "1") == 0) ? 1U : 0U;
        if ((v != 0U) && (sscanf(col[1], "%u-%u-%u %u:%u:%u", &y, &mo, &d, &hh, &mi, &ss) == 6)
            && (y >= 2000U) && (mo >= 1U) && (mo <= 12U) && (d >= 1U) && (d <= 31U))
        {
            dt.yy = (uint8_t)(y - 2000U);
            dt.mo = (uint8_t)mo;
            dt.dd = (uint8_t)d;
            dt.hh = (uint8_t)hh;
            dt.mi = (uint8_t)mi;
            dt.ss = (uint8_t)ss;
        }
        else
        {
            v = 0U;
        }
        setItem((uint8_t)item, v, &dt, run_h);
    }
    else if ((strcmp(col[4], "spin") == 0) && (item == SVC_FILTER))
    {
        spinAdd(strtof(col[7], NULL));
    }
}

static void loadFromCard(void)
{
    FIL f;
    static uint8_t blk[512];
    static char line[SVC_LINE_MAX];
    uint32_t ll = 0U;
    bool overlong = false;

    memset(s_rec, 0, sizeof(s_rec));
    spinReset();
    const FRESULT fo = f_open(&f, SVC_PATH, FA_READ);
    if (fo == FR_OK)
    {
        for (;;)
        {
            UINT br = 0U;
            const FRESULT fr = f_read(&f, blk, sizeof(blk), &br);
            if ((fr != FR_OK) || (br == 0U))
            {
                break;
            }
            for (UINT i = 0U; i < br; i++)
            {
                const char ch = (char)blk[i];
                if ((ch == '\n') || (ch == '\r'))
                {
                    line[ll] = '\0';
                    if (!overlong && (ll > 0U)) { takeLine(line); }
                    ll = 0U;
                    overlong = false;
                }
                else if (ll < (SVC_LINE_MAX - 1U))
                {
                    line[ll++] = ch;
                }
                else
                {
                    overlong = true;
                }
            }
        }
        if ((ll > 0U) && !overlong)
        {
            line[ll] = '\0';
            takeLine(line);
        }
        (void)f_close(&f);
    }
    else if (fo != FR_NO_FILE)
    {
        s_err = 0xC300U | ((uint32_t)fo & 0xFFU);
    }

    /* Items this card has never seen start their clock now. */
    RtcDateTime_t dt;
    const uint8_t v = nowRtc(&dt);
    const float h = deviceHours();
    for (uint32_t i = 0U; i < (uint32_t)SVC_COUNT; i++)
    {
        if (!s_rec[i].have)
        {
            setItem((uint8_t)i, v, &dt, h);
            queue(EV_START, (uint8_t)i, v, &dt, h, 0.0f);
        }
    }
    DBG_I("SVC", "SERVICE.CSV loaded, %.1f h on the card", (double)h);
}

void Service_Service(void)
{
    if (!SessionEdf_IsMounted())
    {
        s_loaded = false;
        s_ready = false;
        return;
    }
    if (!s_loaded && Adherence_IsBuilt())
    {
        s_loaded = true;
        loadFromCard();
        s_ready = true;
    }
    writePending();
}
