/**
  ******************************************************************************
  * @file    sensor_cal.c
  * @brief   Sensor calibration: in-service zero, field span, traceability
  *          (see sensor_cal.h).
  ******************************************************************************
  */

#include "sensor_cal.h"
#include "flow_sensor.h"
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

/* flow_sensor.c: the flow zero's limit around the factory zero. */
extern volatile float g_flow_az_limit_mbar;

volatile uint8_t g_cal_ps_auto = 1U;
volatile uint8_t g_cal_restore = 1U;

#define CAL_PATH               "0:/CAL.CSV"
#define CAL_SCHEMA             1U
#define CAL_MBAR_PER_CMH2O     0.980665f
/* PS zero learning. */
#define CAL_PS_REST_MS         3000U    /* blower stopped this long first */
#define CAL_PS_TILE_MS         1000U
#define CAL_PS_TILES           3U
#define CAL_PS_TILE_MIN_N      50U      /* ~190 Hz: a tile has ~190 */
#define CAL_PS_QUIET_PP        0.2f     /* cmH2O peak-peak in a tile */
#define CAL_PS_Q_MAX           3.0f     /* L/min: nothing flowing */
#define CAL_PS_ZERO_MAX        1.5f     /* cmH2O from the factory zero */
#define CAL_PS_APPLY_MIN       0.01f    /* smaller moves are not applied */
#define CAL_PS_LOG_MIN         0.05f    /* ... nor logged */
#define CAL_PS_RELEARN_MS      600000U  /* 10 min of idle */
#define CAL_PS_FACTORY_OFFSET  0.0f
#define CAL_PS_FACTORY_GAIN    1.0f
#define CAL_PS_GAIN_MIN        0.95f
#define CAL_PS_GAIN_MAX        1.05f
#define CAL_AS_FACTORY_OFFSET  0.0f
#define CAL_AS_OFFSET_MAX      10.0f    /* mbar */
#define CAL_FS_LOG_MIN         0.002f   /* mbar, ~0.2 L/min */
#define CAL_TEST_PS_MAX        5.0f
#define CAL_TEST_FS_MAX        0.2f
#define CAL_RING_N             8U
#define CAL_LINE_MAX           200U

typedef enum { SEN_PS = 0, SEN_FS, SEN_AS, SEN_COUNT } Sensor_t;
typedef enum { PAR_OFFSET = 0, PAR_GAIN, PAR_ZERO, PAR_COUNT } Param_t;
typedef enum { SRC_FACTORY = 0, SRC_AUTO, SRC_FIELD, SRC_RESET, SRC_CARD } Source_t;

static const char * const k_sensor[SEN_COUNT] = { "ps", "fs", "as" };
static const char * const k_param[PAR_COUNT]  = { "offset", "gain", "zero" };
static const char * const k_source[]          = { "factory", "auto", "field", "reset", "card" };

typedef struct
{
    RtcDateTime_t dt;
    uint8_t       dt_valid;
    uint32_t      uptime_ms;
    uint8_t       sensor;
    uint8_t       param;
    uint8_t       source;
    uint8_t       applied;
    float         old_v;
    float         new_v;
} CalRow_t;

/* Applied calibration. PS offset in cmH2O. */
static float    s_ps_offset = CAL_PS_FACTORY_OFFSET;
static float    s_ps_gain   = CAL_PS_FACTORY_GAIN;
static float    s_as_offset = CAL_AS_FACTORY_OFFSET;
static Source_t s_ps_src    = SRC_FACTORY;
static Source_t s_gain_src  = SRC_FACTORY;
static Source_t s_as_src    = SRC_FACTORY;
/* Bench injection. */
static float    s_test_ps = 0.0f;
static float    s_test_fs = 0.0f;
/* PS zero learner. */
static bool     s_idle_prev;
static uint32_t s_rest_ms;
static uint32_t s_tile_ms;
static uint32_t s_tile_n;
static float    s_tile_sum;
static float    s_tile_min;
static float    s_tile_max;
static float    s_run_sum;
static uint8_t  s_run_n;
static bool     s_ps_latched;
static uint32_t s_ps_since_ms;
static bool     s_ps_learned_this_boot;
static bool     s_ps_rejected;
static float    s_ps_cand;
static float    s_ps_logged;
static uint32_t s_ps_learn_n;
static uint32_t s_ps_age_ms;          /* since the PS zero was last confirmed */
/* Flow zero watcher. */
static uint32_t s_fs_az_seen;
static bool     s_fs_rej_prev;
static float    s_fs_logged;
static bool     s_fs_logged_valid;
/* CAL.CSV */
static CalRow_t          s_ring[CAL_RING_N];
static volatile uint32_t s_rows = 0U;
static volatile uint32_t s_written = 0U;
static uint32_t          s_csv_err = 0U;
static bool              s_loaded = false;

static void queue(Sensor_t sen, Param_t par, float old_v, float new_v,
                  Source_t src, bool applied)
{
    CalRow_t r;
    memset(&r, 0, sizeof(r));
    r.uptime_ms = HAL_GetTick();
    r.dt_valid = (RtcClock_IsTimeValid() && RtcClock_Get(&r.dt)) ? 1U : 0U;
    r.sensor = (uint8_t)sen;
    r.param = (uint8_t)par;
    r.source = (uint8_t)src;
    r.applied = applied ? 1U : 0U;
    r.old_v = old_v;
    r.new_v = new_v;
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_ring[s_rows % CAL_RING_N] = r;
    s_rows++;
    if ((s_rows - s_written) > CAL_RING_N)
    {
        s_written = s_rows - CAL_RING_N;
    }
    __set_PRIMASK(primask);
}

/* === Corrections ========================================================== */

float Cal_ApplyPs(float raw_mbar)
{
    const float raw = (raw_mbar / CAL_MBAR_PER_CMH2O) + s_test_ps;   /* cmH2O */
    if (s_tile_n == 0U)
    {
        s_tile_min = raw;
        s_tile_max = raw;
    }
    if (raw < s_tile_min) { s_tile_min = raw; }
    if (raw > s_tile_max) { s_tile_max = raw; }
    s_tile_sum += raw;
    s_tile_n++;
    return (raw - s_ps_offset) * s_ps_gain * CAL_MBAR_PER_CMH2O;
}

float Cal_ApplyAs(float raw_mbar)
{
    return raw_mbar + s_as_offset;
}

float Cal_FsInjectMbar(void)
{
    return s_test_fs;
}

/* === PS zero learner ====================================================== */

static void tileReset(void)
{
    s_tile_ms = 0U;
    s_tile_n = 0U;
    s_tile_sum = 0.0f;
}

static void runReset(void)
{
    s_run_sum = 0.0f;
    s_run_n = 0U;
}

static void psDecide(float cand)
{
    s_ps_cand = cand;
    s_ps_latched = true;
    s_ps_since_ms = 0U;
    s_ps_learn_n++;
    if (fabsf(cand - CAL_PS_FACTORY_OFFSET) > CAL_PS_ZERO_MAX)
    {
        if (!s_ps_rejected || (fabsf(cand - s_ps_logged) >= CAL_PS_LOG_MIN))
        {
            queue(SEN_PS, PAR_OFFSET, s_ps_offset, cand, SRC_AUTO, false);
            s_ps_logged = cand;
            DBG_I("CAL", "ps zero %.2f cmH2O refused (limit %.1f)", (double)cand,
                  (double)CAL_PS_ZERO_MAX);
        }
        s_ps_rejected = true;
        return;
    }
    s_ps_rejected = false;
    s_ps_age_ms = 0U;
    s_ps_learned_this_boot = true;
    const float old = s_ps_offset;
    if (fabsf(cand - old) >= CAL_PS_APPLY_MIN)
    {
        s_ps_offset = cand;
        s_ps_src = SRC_AUTO;
    }
    if (fabsf(cand - s_ps_logged) >= CAL_PS_LOG_MIN)
    {
        queue(SEN_PS, PAR_OFFSET, old, cand, SRC_AUTO, true);
        s_ps_logged = cand;
        DBG_I("CAL", "ps zero %.3f -> %.3f cmH2O", (double)old, (double)cand);
    }
}

void Cal_Update(uint32_t dt_ms, bool idle, float q_slm)
{
    if (s_ps_age_ms < 0xFFFFFFF0U) { s_ps_age_ms += dt_ms; }

    /* Re-arm on every new idle period (therapy stop, end of a blower run). */
    if (idle && !s_idle_prev)
    {
        s_ps_latched = false;
        s_rest_ms = 0U;
        tileReset();
        runReset();
    }
    s_idle_prev = idle;

    const bool quiet_flow = (fabsf(q_slm) < CAL_PS_Q_MAX);
    if (!idle || (g_cal_ps_auto == 0U) || !quiet_flow)
    {
        s_rest_ms = idle ? s_rest_ms : 0U;
        tileReset();
        runReset();
    }
    else if (s_ps_latched)
    {
        s_ps_since_ms += dt_ms;
        if (s_ps_since_ms >= CAL_PS_RELEARN_MS)
        {
            s_ps_latched = false;
        }
        tileReset();
    }
    else if (s_rest_ms < CAL_PS_REST_MS)
    {
        s_rest_ms += dt_ms;
        tileReset();
    }
    else
    {
        s_tile_ms += dt_ms;
        if (s_tile_ms >= CAL_PS_TILE_MS)
        {
            const bool ok = (s_tile_n >= CAL_PS_TILE_MIN_N)
                         && ((s_tile_max - s_tile_min) <= CAL_PS_QUIET_PP);
            if (ok)
            {
                s_run_sum += s_tile_sum / (float)s_tile_n;
                s_run_n++;
                if (s_run_n >= CAL_PS_TILES)
                {
                    psDecide(s_run_sum / (float)s_run_n);
                    runReset();
                }
            }
            else
            {
                runReset();
            }
            tileReset();
        }
    }

    /* Flow zero: log what the idle auto-zero did. */
    const uint32_t az = Flow_GetAutoZeroCount();
    const float zero = Flow_GetZeroOffset();
    if (!s_fs_logged_valid)
    {
        s_fs_logged = zero;
        s_fs_logged_valid = true;
        s_fs_az_seen = az;
    }
    if ((az != s_fs_az_seen) && (fabsf(zero - s_fs_logged) >= CAL_FS_LOG_MIN))
    {
        queue(SEN_FS, PAR_ZERO, s_fs_logged, zero, SRC_AUTO, true);
        DBG_I("CAL", "flow zero %.4f -> %.4f mbar", (double)s_fs_logged, (double)zero);
        s_fs_logged = zero;
    }
    s_fs_az_seen = az;
    const bool rej = Flow_IsZeroRejected();
    if (rej && !s_fs_rej_prev)
    {
        queue(SEN_FS, PAR_ZERO, zero, Flow_GetZeroRejectedMbar(), SRC_AUTO, false);
        DBG_I("CAL", "flow zero %.4f mbar refused", (double)Flow_GetZeroRejectedMbar());
    }
    s_fs_rej_prev = rej;
}

bool Cal_PsZeroRejected(void) { return s_ps_rejected; }
bool Cal_FsZeroRejected(void) { return Flow_IsZeroRejected(); }

/* === Field calibration ===================================================== */

bool Cal_SetPsGain(float gain)
{
    if (!(gain >= CAL_PS_GAIN_MIN) || !(gain <= CAL_PS_GAIN_MAX))
    {
        return false;
    }
    queue(SEN_PS, PAR_GAIN, s_ps_gain, gain, SRC_FIELD, true);
    s_ps_gain = gain;
    s_gain_src = SRC_FIELD;
    return true;
}

bool Cal_SetAsOffset(float mbar)
{
    if (!(fabsf(mbar) <= CAL_AS_OFFSET_MAX))
    {
        return false;
    }
    queue(SEN_AS, PAR_OFFSET, s_as_offset, mbar, SRC_FIELD, true);
    s_as_offset = mbar;
    s_as_src = SRC_FIELD;
    return true;
}

void Cal_ArmPsZero(void)
{
    s_ps_latched = false;
    s_rest_ms = CAL_PS_REST_MS;          /* blower already known to be at rest */
    tileReset();
    runReset();
}

void Cal_ArmFsZero(void)
{
    Flow_AutoZeroArm();
}

void Cal_ResetToFactory(void)
{
    queue(SEN_PS, PAR_OFFSET, s_ps_offset, CAL_PS_FACTORY_OFFSET, SRC_RESET, true);
    queue(SEN_PS, PAR_GAIN, s_ps_gain, CAL_PS_FACTORY_GAIN, SRC_RESET, true);
    queue(SEN_AS, PAR_OFFSET, s_as_offset, CAL_AS_FACTORY_OFFSET, SRC_RESET, true);
    queue(SEN_FS, PAR_ZERO, Flow_GetZeroOffset(), Flow_GetZeroFactory(), SRC_RESET, true);
    s_ps_offset = CAL_PS_FACTORY_OFFSET;
    s_ps_gain = CAL_PS_FACTORY_GAIN;
    s_as_offset = CAL_AS_FACTORY_OFFSET;
    s_ps_src = SRC_RESET;
    s_gain_src = SRC_RESET;
    s_as_src = SRC_RESET;
    s_ps_rejected = false;
    s_ps_logged = CAL_PS_FACTORY_OFFSET;
    Flow_SetZeroOffset(Flow_GetZeroFactory());
    s_fs_logged = Flow_GetZeroFactory();
}

void Cal_SetTest(float ps_cmh2o, float fs_mbar)
{
    s_test_ps = fmaxf(-CAL_TEST_PS_MAX, fminf(CAL_TEST_PS_MAX, ps_cmh2o));
    s_test_fs = fmaxf(-CAL_TEST_FS_MAX, fminf(CAL_TEST_FS_MAX, fs_mbar));
}

int Cal_FormatStatus(char *buf, unsigned len)
{
    const int n = snprintf(buf, len,
        "# CAL ps_off=%.3f ps_gain=%.4f ps_src=%s gain_src=%s ps_rej=%u ps_cand=%.3f "
        "ps_n=%lu ps_age_s=%lu ps_auto=%u fs_zero=%.4f fs_fact=%.4f fs_rej=%u "
        "fs_cand=%.4f fs_az=%lu as_off=%.2f as_src=%s test_ps=%.2f test_fs=%.4f "
        "rows=%lu csverr=0x%04lX\r\n",
        (double)s_ps_offset, (double)s_ps_gain, k_source[s_ps_src], k_source[s_gain_src],
        s_ps_rejected ? 1U : 0U, (double)s_ps_cand, (unsigned long)s_ps_learn_n,
        (unsigned long)(s_ps_age_ms / 1000U), (unsigned)g_cal_ps_auto,
        (double)Flow_GetZeroOffset(), (double)Flow_GetZeroFactory(),
        Flow_IsZeroRejected() ? 1U : 0U, (double)Flow_GetZeroRejectedMbar(),
        (unsigned long)Flow_GetAutoZeroCount(), (double)s_as_offset, k_source[s_as_src],
        (double)s_test_ps, (double)s_test_fs, (unsigned long)s_written,
        (unsigned long)s_csv_err);
    return ((n > 0) && ((unsigned)n < len)) ? n : 0;
}

/* === CAL.CSV ============================================================== */

static const char k_header[] =
    "schema,time,rtc_valid,uptime_s,sensor,param,old,new,unit,source,result";

static const char* unitOf(uint8_t sen, uint8_t par)
{
    if (par == (uint8_t)PAR_GAIN) { return "-"; }
    return (sen == (uint8_t)SEN_PS) ? "cmH2O" : "mbar";
}

static int formatRow(const CalRow_t *r, char *buf, unsigned len)
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
    const int n = snprintf(buf, len, "%u,%s,%u,%lu,%s,%s,%.5f,%.5f,%s,%s,%s\r\n",
                           (unsigned)CAL_SCHEMA, t, (unsigned)r->dt_valid,
                           (unsigned long)(r->uptime_ms / 1000U),
                           k_sensor[r->sensor], k_param[r->param],
                           (double)r->old_v, (double)r->new_v,
                           unitOf(r->sensor, r->param), k_source[r->source],
                           (r->applied != 0U) ? "applied" : "refused");
    return ((n > 0) && ((unsigned)n < len)) ? n : 0;
}

static FRESULT writeLine(FIL *f, const CalRow_t *r)
{
    char line[CAL_LINE_MAX];
    UINT bw = 0U;
    const int n = formatRow(r, line, sizeof(line));
    return (n > 0) ? f_write(f, line, (UINT)n, &bw) : FR_OK;
}

static void writePending(void)
{
    if ((s_written == s_rows) || !SessionEdf_IsMounted())
    {
        return;
    }
    FIL f;
    UINT bw = 0U;
    FRESULT fr = f_open(&f, CAL_PATH, FA_OPEN_APPEND | FA_WRITE);
    if (fr != FR_OK)
    {
        s_csv_err = 0xA100U | ((uint32_t)fr & 0xFFU);
        return;
    }
    if (f_size(&f) == 0U)
    {
        /* A new file starts with the factory values it is trimmed from. */
        char hdr[sizeof(k_header) + 64U];
        const int hn = snprintf(hdr, sizeof(hdr), "# SomnaLink sensor calibration, schema %u\r\n%s\r\n",
                                (unsigned)CAL_SCHEMA, k_header);
        if (hn > 0) { fr = f_write(&f, hdr, (UINT)hn, &bw); }
        CalRow_t r;
        memset(&r, 0, sizeof(r));
        r.uptime_ms = HAL_GetTick();
        r.dt_valid = (RtcClock_IsTimeValid() && RtcClock_Get(&r.dt)) ? 1U : 0U;
        r.source = (uint8_t)SRC_FACTORY;
        r.applied = 1U;
        const struct { uint8_t s, p; float v; } k[] = {
            { SEN_PS, PAR_OFFSET, CAL_PS_FACTORY_OFFSET },
            { SEN_PS, PAR_GAIN,   CAL_PS_FACTORY_GAIN },
            { SEN_FS, PAR_ZERO,   0.0f },
            { SEN_AS, PAR_OFFSET, CAL_AS_FACTORY_OFFSET },
        };
        for (uint32_t i = 0U; (i < (sizeof(k) / sizeof(k[0]))) && (fr == FR_OK); i++)
        {
            r.sensor = k[i].s;
            r.param = k[i].p;
            r.old_v = (k[i].s == (uint8_t)SEN_FS) ? Flow_GetZeroFactory() : k[i].v;
            r.new_v = r.old_v;
            fr = writeLine(&f, &r);
        }
    }
    while ((fr == FR_OK) && (s_written != s_rows))
    {
        CalRow_t r;
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        r = s_ring[s_written % CAL_RING_N];
        __set_PRIMASK(primask);
        fr = writeLine(&f, &r);
        if (fr == FR_OK)
        {
            s_written++;
        }
    }
    const FRESULT cr = f_close(&f);
    s_csv_err = ((fr == FR_OK) && (cr == FR_OK)) ? 0U
              : (0xA200U | ((uint32_t)((fr != FR_OK) ? fr : cr) & 0xFFU));
}

/* Last applied value of each sensor / parameter in the file. */
typedef struct
{
    bool  have[SEN_COUNT][PAR_COUNT];
    float v[SEN_COUNT][PAR_COUNT];
} CalLoad_t;

static void takeLine(char *line, CalLoad_t *ld)
{
    if ((line[0] < '0') || (line[0] > '9'))
    {
        return;
    }
    char *col[11];
    uint32_t nc = 0U;
    char *p = line;
    col[nc++] = p;
    while ((*p != '\0') && (nc < 11U))
    {
        if (*p == ',')
        {
            *p = '\0';
            col[nc++] = p + 1;
        }
        p++;
    }
    if ((nc < 11U) || (strtoul(col[0], NULL, 10) != CAL_SCHEMA)
        || (strncmp(col[10], "applied", 7U) != 0))
    {
        return;
    }
    uint32_t s = SEN_COUNT, q = PAR_COUNT;
    for (uint32_t i = 0U; i < (uint32_t)SEN_COUNT; i++)
    {
        if (strcmp(col[4], k_sensor[i]) == 0) { s = i; }
    }
    for (uint32_t i = 0U; i < (uint32_t)PAR_COUNT; i++)
    {
        if (strcmp(col[5], k_param[i]) == 0) { q = i; }
    }
    char *end = NULL;
    const float v = strtof(col[7], &end);
    if ((s < (uint32_t)SEN_COUNT) && (q < (uint32_t)PAR_COUNT) && (end != col[7]))
    {
        ld->have[s][q] = true;
        ld->v[s][q] = v;
    }
}

static void loadFromCard(void)
{
    FIL f;
    static uint8_t blk[512];
    static char line[CAL_LINE_MAX];
    static CalLoad_t ld;
    uint32_t ll = 0U;
    bool overlong = false;
    memset(&ld, 0, sizeof(ld));

    const FRESULT fo = f_open(&f, CAL_PATH, FA_READ);
    if (fo != FR_OK)
    {
        s_csv_err = (fo == FR_NO_FILE) ? 0U : (0xA300U | ((uint32_t)fo & 0xFFU));
        return;
    }
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
                if (!overlong && (ll > 0U))
                {
                    takeLine(line, &ld);
                }
                ll = 0U;
                overlong = false;
            }
            else if (ll < (CAL_LINE_MAX - 1U))
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
        takeLine(line, &ld);
    }
    (void)f_close(&f);

    /* A zero learned since boot is fresher than the card's. */
    if (ld.have[SEN_PS][PAR_OFFSET] && !s_ps_learned_this_boot
        && (fabsf(ld.v[SEN_PS][PAR_OFFSET] - CAL_PS_FACTORY_OFFSET) <= CAL_PS_ZERO_MAX))
    {
        s_ps_offset = ld.v[SEN_PS][PAR_OFFSET];
        s_ps_logged = s_ps_offset;
        s_ps_src = SRC_CARD;
    }
    if (ld.have[SEN_PS][PAR_GAIN] && (ld.v[SEN_PS][PAR_GAIN] >= CAL_PS_GAIN_MIN)
        && (ld.v[SEN_PS][PAR_GAIN] <= CAL_PS_GAIN_MAX))
    {
        s_ps_gain = ld.v[SEN_PS][PAR_GAIN];
        s_gain_src = SRC_CARD;
    }
    if (ld.have[SEN_AS][PAR_OFFSET] && (fabsf(ld.v[SEN_AS][PAR_OFFSET]) <= CAL_AS_OFFSET_MAX))
    {
        s_as_offset = ld.v[SEN_AS][PAR_OFFSET];
        s_as_src = SRC_CARD;
    }
    if (ld.have[SEN_FS][PAR_ZERO] && (Flow_GetAutoZeroCount() == 0U)
        && (fabsf(ld.v[SEN_FS][PAR_ZERO] - Flow_GetZeroFactory()) <= g_flow_az_limit_mbar))
    {
        Flow_SetZeroOffset(ld.v[SEN_FS][PAR_ZERO]);
        s_fs_logged = ld.v[SEN_FS][PAR_ZERO];
    }
    DBG_I("CAL", "from CAL.CSV: ps %.3f cmH2O x%.4f, as %+.2f mbar, fs zero %.4f",
          (double)s_ps_offset, (double)s_ps_gain, (double)s_as_offset,
          (double)Flow_GetZeroOffset());
}

void Cal_Service(void)
{
    if (!SessionEdf_IsMounted())
    {
        s_loaded = false;
        return;
    }
    if (!s_loaded && (Therapy_GetState() == THERAPY_STATE_IDLE))
    {
        s_loaded = true;
        if (g_cal_restore != 0U)
        {
            loadFromCard();
        }
    }
    writePending();
}
