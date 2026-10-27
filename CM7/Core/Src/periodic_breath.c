/**
  ******************************************************************************
  * @file    periodic_breath.c
  * @brief   Periodic breathing / Cheyne-Stokes detector (see periodic_breath.h).
  ******************************************************************************
  */

#include "periodic_breath.h"
#include "session_edf_log.h"

#include <math.h>
#include <string.h>

volatile uint8_t g_pb_enable    = 1U;
volatile float   g_pb_depth_min = 0.30f;
volatile float   g_pb_corr_min  = 0.40f;

#define PB_ENV_TAU_S       2.0f      /* each of the two envelope low-passes */
#define PB_WIN_S           240U      /* window, 1 sample / s                */
#define PB_EVAL_MS         5000U
#define PB_LAG_MIN_S       30U
#define PB_LAG_MAX_S       100U
#define PB_HALF_CORR_MAX   (-0.20f)  /* half-lag anti-correlation           */
#define PB_MEAN_MIN_SLM    2.0f      /* no breathing at all is not PB       */
#define PB_ENTER_EVALS     2U
#define PB_EXIT_EVALS      3U
#define PB_MIN_SPAN_MS     60000U
#define PB_EDF_TICK_MS     40U       /* BRP sample clock (25 Hz)            */
/* Detection lag, taken off the span ends. Not symmetric: the window must be
 * mostly periodic before it is seen (~200 s on the bench, simulator CSR
 * 2026-10-27), but a minute of normal breathing breaks it (~60 s). */
#define PB_START_BACK_MS   180000U
#define PB_END_BACK_MS     60000U

static float    s_e1;
static float    s_e2;
static bool     s_env_seeded;
static float    s_env[PB_WIN_S];
static uint32_t s_head;              /* next write                          */
static uint32_t s_n;                 /* samples held, up to PB_WIN_S        */
static uint32_t s_sample_ms;
static uint32_t s_eval_ms;
static uint32_t s_t_ms;              /* scoring time since session start    */
static uint32_t s_now_ticks;         /* session ticks at the last update    */

static bool     s_active;
static uint32_t s_pos_run;
static uint32_t s_neg_run;
static uint32_t s_first_pos_ms;
static uint32_t s_first_neg_ms;
static uint32_t s_span_start_ms;
static uint32_t s_last_end_ms;
static uint32_t s_sess_ms;
static uint32_t s_episodes;

static float    s_last_corr;
static float    s_last_depth;
static float    s_last_period_s;

static float    s_x[PB_WIN_S];       /* evaluation scratch                  */

static float clampf(float v, float lo, float hi)
{
    return (v < lo) ? lo : ((v > hi) ? hi : v);
}

static void dropWindow(void)
{
    s_env_seeded = false;
    s_head = 0U;
    s_n = 0U;
    s_sample_ms = 0U;
    s_eval_ms = 0U;
    s_pos_run = 0U;
    s_neg_run = 0U;
}

void PeriodicBreath_Reset(void)
{
    dropWindow();
    s_t_ms = 0U;
    s_now_ticks = 0U;
    s_active = false;
    s_span_start_ms = 0U;
    s_last_end_ms = 0U;
    s_sess_ms = 0U;
    s_episodes = 0U;
    s_last_corr = 0.0f;
    s_last_depth = 0.0f;
    s_last_period_s = 0.0f;
}

/* Session ticks of a scoring time in the past (CSL onsets). Scoring time
 * stops through mask-off while the session clock runs on, so this is only
 * right because a span never straddles a gap: Interrupt() closes it from the
 * last update before one. */
static uint32_t ticksAt(uint32_t t_ms)
{
    const uint32_t back = (s_t_ms - t_ms) / PB_EDF_TICK_MS;
    return (s_now_ticks > back) ? (s_now_ticks - back) : 0U;
}

static void closeSpan(uint32_t end_ms)
{
    s_active = false;
    if (end_ms <= s_span_start_ms)
    {
        return;
    }
    const uint32_t dur = end_ms - s_span_start_ms;
    if (dur < PB_MIN_SPAN_MS)
    {
        return;
    }
    SessionEdf_LogEvent(SESSION_EVT_CSR_START, ticksAt(s_span_start_ms), 0.0f);
    SessionEdf_LogEvent(SESSION_EVT_CSR_END, ticksAt(end_ms), 0.0f);
    s_sess_ms += dur;
    s_episodes++;
    s_last_end_ms = end_ms;
}

/* Normalised correlation of the window with itself shifted by lag samples,
 * over the overlap only. */
static float corrAt(uint32_t lag)
{
    float sxy = 0.0f;
    float sxx = 0.0f;
    float syy = 0.0f;
    for (uint32_t i = 0U; (i + lag) < PB_WIN_S; i++)
    {
        const float a = s_x[i];
        const float b = s_x[i + lag];
        sxy += a * b;
        sxx += a * a;
        syy += b * b;
    }
    const float den = sqrtf(sxx * syy);
    return (den > 1e-6f) ? (sxy / den) : 0.0f;
}

static bool evaluate(void)
{
    float sum = 0.0f;
    for (uint32_t i = 0U; i < PB_WIN_S; i++)
    {
        s_x[i] = s_env[(s_head + i) % PB_WIN_S];   /* oldest first */
        sum += s_x[i];
    }
    const float mean = sum / (float)PB_WIN_S;
    float var = 0.0f;
    for (uint32_t i = 0U; i < PB_WIN_S; i++)
    {
        s_x[i] -= mean;
        var += s_x[i] * s_x[i];
    }
    var /= (float)PB_WIN_S;

    s_last_depth = (mean > 0.1f) ? (sqrtf(var) / mean) : 0.0f;
    s_last_corr = 0.0f;
    s_last_period_s = 0.0f;
    if (mean < PB_MEAN_MIN_SLM)
    {
        return false;
    }

    /* Highest local peak of the correlation over the cycle-length lags. */
    float prev = corrAt(PB_LAG_MIN_S - 1U);
    float cur = corrAt(PB_LAG_MIN_S);
    float best = -1.0f;
    uint32_t best_lag = 0U;
    for (uint32_t lag = PB_LAG_MIN_S; lag <= PB_LAG_MAX_S; lag++)
    {
        const float next = corrAt(lag + 1U);
        if ((cur >= prev) && (cur >= next) && (cur > best))
        {
            best = cur;
            best_lag = lag;
        }
        prev = cur;
        cur = next;
    }
    if (best_lag == 0U)
    {
        return false;
    }
    s_last_corr = best;
    s_last_period_s = (float)best_lag;

    const float depth_min = clampf(g_pb_depth_min, 0.05f, 2.0f);
    const float corr_min = clampf(g_pb_corr_min, 0.10f, 0.95f);
    return (s_last_depth >= depth_min) && (best >= corr_min)
        && (corrAt(best_lag / 2U) <= PB_HALF_CORR_MAX);
}

void PeriodicBreath_Update(float q_patient_slm, uint32_t dt_ms)
{
    if ((g_pb_enable == 0U) || (dt_ms == 0U) || (dt_ms > 100U))
    {
        return;
    }
    s_t_ms += dt_ms;
    s_now_ticks = SessionEdf_NowTicks();

    const float aq = fabsf(q_patient_slm);
    if (!s_env_seeded)
    {
        s_e1 = aq;
        s_e2 = aq;
        s_env_seeded = true;
    }
    else
    {
        const float dt_s = (float)dt_ms * 0.001f;
        const float a = dt_s / (PB_ENV_TAU_S + dt_s);
        s_e1 += a * (aq - s_e1);
        s_e2 += a * (s_e1 - s_e2);
    }

    s_sample_ms += dt_ms;
    if (s_sample_ms < 1000U)
    {
        return;
    }
    s_sample_ms -= 1000U;
    s_env[s_head] = s_e2;
    s_head = (s_head + 1U) % PB_WIN_S;
    if (s_n < PB_WIN_S) { s_n++; }

    s_eval_ms += 1000U;
    if ((s_n < PB_WIN_S) || (s_eval_ms < PB_EVAL_MS))
    {
        return;
    }
    s_eval_ms = 0U;

    if (evaluate())
    {
        s_neg_run = 0U;
        if (s_pos_run == 0U) { s_first_pos_ms = s_t_ms; }
        s_pos_run++;
        if (!s_active && (s_pos_run >= PB_ENTER_EVALS))
        {
            /* The pattern filled the window before it was seen. */
            uint32_t start = (s_first_pos_ms > PB_START_BACK_MS)
                           ? (s_first_pos_ms - PB_START_BACK_MS) : 0U;
            if (start < s_last_end_ms) { start = s_last_end_ms; }
            s_span_start_ms = start;
            s_active = true;
        }
    }
    else
    {
        s_pos_run = 0U;
        if (s_active)
        {
            if (s_neg_run == 0U) { s_first_neg_ms = s_t_ms; }
            s_neg_run++;
            if (s_neg_run >= PB_EXIT_EVALS)
            {
                s_neg_run = 0U;
                closeSpan((s_first_neg_ms > PB_END_BACK_MS)
                              ? (s_first_neg_ms - PB_END_BACK_MS) : 0U);
            }
        }
    }
}

void PeriodicBreath_Interrupt(void)
{
    if (s_active)
    {
        closeSpan(s_t_ms);    /* no sign it had stopped: ends here */
    }
    dropWindow();
}

bool PeriodicBreath_IsActive(void)
{
    return s_active;
}

uint32_t PeriodicBreath_GetSessionSeconds(void)
{
    uint32_t ms = s_sess_ms;
    if (s_active && (s_t_ms > s_span_start_ms))
    {
        ms += s_t_ms - s_span_start_ms;
    }
    return ms / 1000U;
}

uint32_t PeriodicBreath_GetEpisodes(void)
{
    return s_episodes;
}

float PeriodicBreath_GetLastCorr(void)
{
    return s_last_corr;
}

float PeriodicBreath_GetLastDepth(void)
{
    return s_last_depth;
}

float PeriodicBreath_GetLastPeriodS(void)
{
    return s_last_period_s;
}
