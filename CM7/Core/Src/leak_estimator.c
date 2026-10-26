/**
  ******************************************************************************
  * @file    leak_estimator.c
  * @brief   Implementation: total - vent, slow-LPF, threshold compare.
  ******************************************************************************
  */

#include "leak_estimator.h"
#include "leak_estimator_vivo.h"
#include "flow_sensor.h"

#include <math.h>
#include <string.h>

/* === Algorithm selection === */
volatile uint8_t g_leak_vivo_enable = 1U;
static LeakAlgo_t s_algo = LEAK_ALGO_VIVO;
static float s_therapy_sp_cmh2o = 8.0f;

/* === Public configuration (debugger-writable volatiles) === */
volatile float    g_leak_vent_k         = 8.7f;       /* slm / sqrt(cmH2O) */
volatile float    g_leak_tau_seconds    = 10.0f;       /* slow LPF time const*/
volatile float    g_leak_high_threshold = 24.0f;       /* slm                */
volatile float    g_leak_small_threshold = 12.0f;      /* slm; UI "Small"    */
volatile float    g_leak_low_threshold  = 8.0f;        /* |Q_total| blocked  */
volatile float    g_leak_low_press_min_cmh2o = 4.0f;   /* arm low-leak only above */
volatile float    g_leak_vent_p_tau_s   = 0.5f;        /* P mask LPF for vent */
volatile float    g_leak_patient_lpf_hz = 2.0f;        /* breath-band LPF    */
volatile uint8_t  g_leak_patient_quiet_lpf_enable = 0U; /* 0.35 Hz in pause   */
volatile float    g_leak_patient_quiet_lpf_hz = 0.35f; /* pause-region LPF   */
volatile float    g_leak_patient_quiet_band_slm = 4.0f; /* envelope: enter quiet */
volatile float    g_leak_patient_quiet_exit_slm = 7.0f; /* |Q_raw|: leave quiet */
volatile float    g_leak_patient_quiet_release_s = 0.25f;
/* R10 (docs/APAP_FINDINGS_2026-10-12_breathsim.md): OFF by default. "Quiet"
 * is an envelope <= 4 slm and it is only left at |Q| >= 7 slm, so a weak
 * (hypopnea) inspiration never leaves it: the pause DC then slews after the
 * breath at 4 slm/s and is subtracted from it, and it holds the result
 * through expiration. Replaying this filter on the R10 capture (total flow
 * minus a constant leak level) reproduces the CPAP's published patient flow
 * - within 0.14 slm rms in normal breathing, and the -2 slm level drop and
 * the shrunken peaks (7 -> 5 slm) in a 60 % hypopnea - so this stage, not
 * the VIVO leak level, is the per-breath offset R9/R10 found. With it off,
 * the hypopnea mean is -0.3 slm and normal breaths are unchanged.
 * 1 = previous behaviour (runtime: leak set patient_pause_zero 1). */
volatile uint8_t  g_leak_patient_pause_zero_enable = 0U;
volatile float    g_leak_patient_pause_hold_s = 0.15f;
volatile float    g_leak_patient_pause_slew_slm_s = 4.0f;
volatile float    g_leak_patient_env_slm = 0.0f;
volatile float    g_leak_patient_pause_dc_slm = 0.0f;
volatile uint8_t  g_leak_patient_quiet_now = 0U;

/* === State (debugger-readable volatiles) === */
volatile float    g_leak_total_slm      = 0.0f;
volatile float    g_leak_vent_slm       = 0.0f;
volatile float    g_leak_uninten_slm    = 0.0f;
volatile float    g_leak_instant_excess_slm = 0.0f;   /* Q_total - Q_vent   */
volatile float    g_flow_patient_raw_slm = 0.0f;       /* before breath LPF  */
volatile float    g_flow_patient_slm    = 0.0f;        /* LPF for EPR/plot   */
volatile uint8_t  g_leak_is_high        = 0U;
volatile uint8_t  g_leak_is_low         = 0U;
volatile uint8_t  g_leak_severity       = (uint8_t)LEAK_SEV_SEALED;
volatile float    g_leak_pct_of_vent    = 0.0f;
volatile uint32_t g_leak_step_count     = 0U;

/* Session / health (RAM; not persisted). */
volatile uint8_t  g_leak_session_active = 0U;
volatile float    g_leak_session_elapsed_s = 0.0f;
volatile float    g_leak_session_high_time_s = 0.0f;
volatile float    g_leak_session_low_time_s = 0.0f;
volatile float    g_leak_session_avg_slm = 0.0f;
volatile float    g_leak_session_max_slm = 0.0f;
volatile uint32_t g_leak_session_samples = 0U;
/* #57: |unintentional leak| histogram for the session median / 95th
 * percentile ResMed reports (1 L/min bins, 0..150). Mask-off time
 * (disconnect freeze) is left out, as the pressure statistics leave it out. */
#define LEAK_HIST_N  151U
static uint32_t s_leak_hist[LEAK_HIST_N];
static uint32_t s_leak_hist_n = 0U;
volatile uint32_t g_leak_health_high_samples = 0U;
volatile uint32_t g_leak_health_low_samples = 0U;
volatile uint32_t g_leak_health_reseed_count = 0U;
volatile uint32_t g_leak_health_ratio_clamp_count = 0U;

/* === Internal === */
static bool s_seeded = false;
static bool s_patient_filt_seeded = false;
static float s_mask_p_filt_cmh2o = 0.0f;
static bool s_hold_adaptation = false;
static float s_pat_env = 0.0f;
static uint8_t s_pat_env_seeded = 0U;
static float s_pat_lpf = 0.0f;
static uint8_t s_pat_lpf_seeded = 0U;
static float s_pat_pause_dc = 0.0f;
static float s_pat_quiet_s = 0.0f;
static uint8_t s_pat_quiet = 0U;

#define LEAK_TWO_PI  6.28318530717958647692f

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* === Public API === */

void Leak_Init(void)
{
    g_leak_vent_k         = 8.7f;
    g_leak_tau_seconds    = 10.0f;
    g_leak_high_threshold = 24.0f;
    g_leak_small_threshold = 12.0f;
    g_leak_low_threshold  = 8.0f;
    g_leak_low_press_min_cmh2o = 4.0f;
    g_leak_vent_p_tau_s   = 0.5f;
    g_leak_patient_lpf_hz = 2.0f;
    g_leak_patient_quiet_lpf_enable = 0U;
    g_leak_patient_quiet_lpf_hz = 0.35f;
    g_leak_patient_quiet_band_slm = 4.0f;
    g_leak_patient_quiet_exit_slm = 7.0f;
    g_leak_patient_quiet_release_s = 0.25f;
    g_leak_patient_pause_zero_enable = 0U;   /* R10, see the declaration */
    g_leak_patient_pause_hold_s = 0.15f;
    g_leak_patient_pause_slew_slm_s = 4.0f;
    Leak_PatientFiltReset();

    g_leak_total_slm   = 0.0f;
    g_leak_vent_slm    = 0.0f;
    g_leak_uninten_slm = 0.0f;
    g_leak_instant_excess_slm = 0.0f;
    g_flow_patient_raw_slm = 0.0f;
    g_flow_patient_slm = 0.0f;
    g_leak_is_high     = 0U;
    g_leak_is_low      = 0U;
    g_leak_severity    = (uint8_t)LEAK_SEV_SEALED;
    g_leak_pct_of_vent = 0.0f;
    g_leak_step_count  = 0U;
    s_seeded           = false;
    s_patient_filt_seeded = false;
    s_mask_p_filt_cmh2o = 0.0f;
    s_hold_adaptation  = false;

    s_algo = LEAK_ALGO_VIVO;
    g_leak_vivo_enable = 1U;
    s_therapy_sp_cmh2o = 8.0f;
    Leak_SessionReset();
    LeakVivo_Init();
}

void Leak_SetAlgo(LeakAlgo_t algo)
{
    s_algo = algo;
    g_leak_vivo_enable = (algo == LEAK_ALGO_VIVO) ? 1U : 0U;
}

LeakAlgo_t Leak_GetAlgo(void)
{
    return s_algo;
}

void Leak_SetVivoEnable(bool en)
{
    Leak_SetAlgo(en ? LEAK_ALGO_VIVO : LEAK_ALGO_SIMPLE);
}

bool Leak_IsVivoEnabled(void)
{
    return (s_algo == LEAK_ALGO_VIVO);
}

void Leak_SetTherapyPressureCmh2o(float cmh2o)
{
    if (cmh2o < 0.0f)
    {
        cmh2o = 0.0f;
    }
    s_therapy_sp_cmh2o = cmh2o;
}

void Leak_SetStandardRatio(int32_t ratio)
{
    LeakVivo_SetStandardRatio(ratio);
}

void Leak_SetVentCoefficient(float k)
{
    g_leak_vent_k = clampf(k, 5.0f, 20.0f);
}

void Leak_SetTimeConstant(float seconds)
{
    g_leak_tau_seconds = clampf(seconds, 1.0f, 60.0f);
}

void Leak_SetHighThreshold(float slm)
{
    g_leak_high_threshold = clampf(slm, 5.0f, 60.0f);
    if (g_leak_small_threshold > g_leak_high_threshold)
    {
        g_leak_small_threshold = g_leak_high_threshold;
    }
    LeakVivo_SetHighThreshold(g_leak_high_threshold);
}

void Leak_SetSmallThreshold(float slm)
{
    g_leak_small_threshold = clampf(slm, 1.0f, g_leak_high_threshold);
}

void Leak_SetLowThreshold(float total_slm)
{
    g_leak_low_threshold = clampf(total_slm, 1.0f, 40.0f);
}

float Leak_GetVentCoefficient(void) { return g_leak_vent_k; }
float Leak_GetTimeConstant(void)    { return g_leak_tau_seconds; }
float Leak_GetHighThreshold(void)   { return g_leak_high_threshold; }
float Leak_GetSmallThreshold(void)  { return g_leak_small_threshold; }
float Leak_GetLowThreshold(void)    { return g_leak_low_threshold; }
float Leak_GetTotalSlm(void)        { return g_leak_total_slm; }
float Leak_GetVentSlm(void)         { return g_leak_vent_slm; }
float Leak_GetUnintentionalSlm(void){ return g_leak_uninten_slm; }
float Leak_GetInstantExcessSlm(void){ return g_leak_instant_excess_slm; }
float Leak_GetPatientSlm(void)      { return g_flow_patient_slm; }
bool  Leak_IsHigh(void)             { return (g_leak_is_high != 0U); }
bool  Leak_IsLow(void)              { return (g_leak_is_low != 0U); }

LeakSeverity_t Leak_GetSeverity(void)
{
    return (LeakSeverity_t)g_leak_severity;
}

const char* Leak_GetSeverityName(void)
{
    switch ((LeakSeverity_t)g_leak_severity)
    {
    case LEAK_SEV_HIGH:  return "HIGH LEAK";
    case LEAK_SEV_SMALL: return "Small leak";
    default:             return "Sealed";
    }
}

float Leak_GetPercentOfVent(void)
{
    return g_leak_pct_of_vent;
}

void Leak_SessionReset(void)
{
    g_leak_session_elapsed_s = 0.0f;
    g_leak_session_high_time_s = 0.0f;
    g_leak_session_low_time_s = 0.0f;
    g_leak_session_avg_slm = 0.0f;
    g_leak_session_max_slm = 0.0f;
    g_leak_session_samples = 0U;
    memset(s_leak_hist, 0, sizeof(s_leak_hist));
    s_leak_hist_n = 0U;
    g_leak_health_high_samples = 0U;
    g_leak_health_low_samples = 0U;
    g_leak_health_reseed_count = 0U;
    g_leak_health_ratio_clamp_count = 0U;
}

void Leak_SetSessionActive(bool active)
{
    g_leak_session_active = active ? 1U : 0U;
}

bool Leak_IsSessionActive(void)
{
    return (g_leak_session_active != 0U);
}

float Leak_GetSessionAvgSlm(void)      { return g_leak_session_avg_slm; }
float Leak_GetSessionMaxSlm(void)      { return g_leak_session_max_slm; }
float Leak_GetSessionHighTimeS(void)   { return g_leak_session_high_time_s; }
float Leak_GetSessionLowTimeS(void)    { return g_leak_session_low_time_s; }
float Leak_GetSessionElapsedS(void)    { return g_leak_session_elapsed_s; }

float Leak_GetSessionPercentileSlm(float pct)
{
    if (s_leak_hist_n == 0U)
    {
        return 0.0f;
    }
    uint32_t target = (uint32_t)((clampf(pct, 0.0f, 100.0f) / 100.0f) * (float)s_leak_hist_n);
    if (target < 1U) { target = 1U; }
    uint32_t cum = 0U;
    for (uint32_t i = 0U; i < LEAK_HIST_N; i++)
    {
        cum += s_leak_hist[i];
        if (cum >= target)
        {
            return (float)i;
        }
    }
    return (float)(LEAK_HIST_N - 1U);
}
uint32_t Leak_GetHealthHighSamples(void) { return g_leak_health_high_samples; }
uint32_t Leak_GetHealthLowSamples(void)  { return g_leak_health_low_samples; }
uint32_t Leak_GetHealthReseedCount(void) { return g_leak_health_reseed_count; }
uint32_t Leak_GetHealthRatioClampCount(void)
{
    return g_leak_health_ratio_clamp_count;
}

void Leak_NoteRatioClamp(void)
{
    g_leak_health_ratio_clamp_count++;
}

int32_t Leak_GetMeanRatio(void)
{
    return Leak_IsVivoEnabled() ? LeakVivo_GetMeanRatio() : 0;
}

int32_t Leak_GetMeanRatio10Sec(void)
{
    return Leak_IsVivoEnabled() ? LeakVivo_GetMeanRatio10Sec() : 0;
}

int32_t Leak_GetInstantRatio(void)
{
    return Leak_IsVivoEnabled() ? LeakVivo_GetInstantRatio() : 0;
}

/* The VIVO adaptation freeze has two independent owners: the disconnect
 * logic (therapy.c) and, since 2026-10-19, the APAP during NO_BREATH / FOT
 * (R5b, docs/APAP_FINDINGS_2026-10-12_breathsim.md: with no breathing, leak
 * cannot be separated from patient flow, and FOT modulates the pressure the
 * vent model depends on - on the bench the leak estimate fell 12.4 -> 9-10
 * slm during FOT, shifting patient flow +2 slm, and a FOT peak then passed
 * as a breath and ended FOT mid-apnea). Frozen while EITHER owner asks, so
 * one cannot clear the other's freeze. */
static bool s_freeze_disconnect = false;
static bool s_freeze_apnea = false;

static void leakApplyFreeze(void)
{
    if (Leak_IsVivoEnabled())
    {
        LeakVivo_SetAdaptationFreeze(s_freeze_disconnect || s_freeze_apnea);
    }
}

void Leak_SetDisconnectFreeze(bool freeze)
{
    s_freeze_disconnect = freeze;
    leakApplyFreeze();
}

void Leak_SetApneaFreeze(bool freeze)
{
    s_freeze_apnea = freeze;
    leakApplyFreeze();
}

float Leak_GetFlowLeakSlm(void)
{
    return Leak_IsVivoEnabled() ? g_leak_vivo_flow_leak_slm
                                : (g_leak_vent_slm + g_leak_uninten_slm);
}

float Leak_GetReportedLeakSlm(void)
{
    return Leak_IsVivoEnabled() ? LeakVivo_GetReportedLeakSlm()
                                : g_leak_uninten_slm;
}

float Leak_GetRestVolumeMl(void)
{
    return Leak_IsVivoEnabled() ? g_leak_vivo_rest_volume_ml : 0.0f;
}

uint8_t Leak_GetBreathInspiration(void)
{
    return Leak_IsVivoEnabled() ? g_leak_vivo_breath_insp : 0U;
}

float Leak_GetBreathRateBpm(void)
{
    return Leak_IsVivoEnabled() ? g_leak_vivo_breath_rate_bpm : 0.0f;
}

float Leak_GetLastInspVolumeMl(void)
{
    return Leak_IsVivoEnabled() ? g_leak_vivo_last_insp_vol_ml : 0.0f;
}

float Leak_GetLastExpVolumeMl(void)
{
    return Leak_IsVivoEnabled() ? g_leak_vivo_last_exp_vol_ml : 0.0f;
}

float Leak_GetPhaseVolumeMl(void)
{
    return Leak_IsVivoEnabled() ? g_leak_vivo_phase_vol_ml : 0.0f;
}

static void leakUpdateDerivedMetrics(float mask_pressure_cmh2o, float dt_s)
{
    const float abs_unint = fabsf(g_leak_uninten_slm);
    const float vent = fabsf(g_leak_vent_slm);
    const float vent_den = (vent > 0.5f) ? vent : 0.5f;
    g_leak_pct_of_vent = 100.0f * (abs_unint / vent_den);

    if (abs_unint > g_leak_high_threshold)
    {
        g_leak_severity = (uint8_t)LEAK_SEV_HIGH;
        g_leak_is_high = 1U;
    }
    else if (abs_unint > g_leak_small_threshold)
    {
        g_leak_severity = (uint8_t)LEAK_SEV_SMALL;
        g_leak_is_high = 0U;
    }
    else
    {
        g_leak_severity = (uint8_t)LEAK_SEV_SEALED;
        g_leak_is_high = 0U;
    }

    /* Blocked-vent / insufficient flush hint: pressurized but almost no flow.
     * Flag only — does not change therapy control. */
    const float p_arm = clampf(g_leak_low_press_min_cmh2o, 2.0f, 15.0f);
    const float q_abs = fabsf(g_leak_total_slm);
    g_leak_is_low = ((mask_pressure_cmh2o >= p_arm)
                     && (q_abs < g_leak_low_threshold)) ? 1U : 0U;

    if (g_leak_is_high != 0U)
    {
        g_leak_health_high_samples++;
    }
    if (g_leak_is_low != 0U)
    {
        g_leak_health_low_samples++;
    }

    if ((g_leak_session_active != 0U) && (dt_s > 0.0f) && (dt_s < 0.1f))
    {
        g_leak_session_elapsed_s += dt_s;
        if (g_leak_is_high != 0U)
        {
            g_leak_session_high_time_s += dt_s;
        }
        if (g_leak_is_low != 0U)
        {
            g_leak_session_low_time_s += dt_s;
        }

        if (!s_freeze_disconnect)
        {
            uint32_t bin = (uint32_t)(abs_unint + 0.5f);
            if (bin >= LEAK_HIST_N) { bin = LEAK_HIST_N - 1U; }
            s_leak_hist[bin]++;
            s_leak_hist_n++;
        }

        g_leak_session_samples++;
        if (g_leak_session_samples == 1U)
        {
            g_leak_session_avg_slm = abs_unint;
            g_leak_session_max_slm = abs_unint;
        }
        else
        {
            const float n = (float)g_leak_session_samples;
            g_leak_session_avg_slm += (abs_unint - g_leak_session_avg_slm) / n;
            if (abs_unint > g_leak_session_max_slm)
            {
                g_leak_session_max_slm = abs_unint;
            }
        }
    }
}

/* Pause vs breath on an envelope, not |Q_raw| of this tick. Instantaneous
 * |Q| during an expiratory pause is a few slm of leak residual + sensor
 * hash, which kept the old blend near 2 Hz and left the plot noisy.
 * Fast exit on |Q_raw| >= exit so inspiration is not delayed.
 * Output is published Q_patient only. APAP/physio FOT must score
 * Flow_GetPatientRawSlm() (see Flow_FotBpStep); 0.35 Hz kills 4 Hz. */
float Leak_PatientLpfHz(float q_raw_slm)
{
    (void)q_raw_slm;
    const float fc_hi = clampf(g_leak_patient_lpf_hz, 0.5f, 8.0f);
    if (g_leak_patient_quiet_lpf_enable == 0U)
    {
        return fc_hi;
    }
    const float fc_lo = clampf(g_leak_patient_quiet_lpf_hz, 0.25f, fc_hi);
    return (s_pat_quiet != 0U) ? fc_lo : fc_hi;
}

void Leak_PatientFiltReset(void)
{
    s_pat_env = 0.0f;
    s_pat_env_seeded = 0U;
    s_pat_lpf = 0.0f;
    s_pat_lpf_seeded = 0U;
    s_pat_pause_dc = 0.0f;
    s_pat_quiet_s = 0.0f;
    s_pat_quiet = 0U;
    g_leak_patient_env_slm = 0.0f;
    g_leak_patient_pause_dc_slm = 0.0f;
    g_leak_patient_quiet_now = 0U;
}

float Leak_PatientFiltStep(float q_raw_slm, float dt_s)
{
    if (!(dt_s > 0.0f) || dt_s > 0.1f)
    {
        return (s_pat_lpf - s_pat_pause_dc);
    }

    const float aq = (q_raw_slm < 0.0f) ? -q_raw_slm : q_raw_slm;
    if (s_pat_env_seeded == 0U)
    {
        s_pat_env = aq;
        s_pat_env_seeded = 1U;
    }
    else
    {
        const float tau = clampf(g_leak_patient_quiet_release_s, 0.08f, 2.0f);
        const float a = dt_s / (tau + dt_s);
        s_pat_env += a * (aq - s_pat_env);
    }
    g_leak_patient_env_slm = s_pat_env;

    const float enter = clampf(g_leak_patient_quiet_band_slm, 1.0f, 15.0f);
    const float exit_slm = clampf(g_leak_patient_quiet_exit_slm, enter + 1.0f, 25.0f);
    if (s_pat_quiet != 0U)
    {
        if (aq >= exit_slm)
        {
            s_pat_quiet = 0U;
            s_pat_quiet_s = 0.0f;
        }
    }
    else if (s_pat_env <= enter)
    {
        s_pat_quiet = 1U;
    }
    g_leak_patient_quiet_now = s_pat_quiet;

    const float fc = Leak_PatientLpfHz(q_raw_slm);
    const float omega = LEAK_TWO_PI * fc;
    const float alpha = (omega * dt_s) / (1.0f + omega * dt_s);
    if (s_pat_lpf_seeded == 0U)
    {
        s_pat_lpf = q_raw_slm;
        s_pat_lpf_seeded = 1U;
    }
    else
    {
        s_pat_lpf = alpha * q_raw_slm + (1.0f - alpha) * s_pat_lpf;
    }

    if (s_pat_quiet != 0U)
    {
        s_pat_quiet_s += dt_s;
        const float hold = clampf(g_leak_patient_pause_hold_s, 0.05f, 1.5f);
        if ((g_leak_patient_pause_zero_enable != 0U) && (s_pat_quiet_s >= hold))
        {
            const float slew = clampf(g_leak_patient_pause_slew_slm_s, 0.2f, 20.0f) * dt_s;
            float err = s_pat_lpf - s_pat_pause_dc;
            if (err > slew) { err = slew; }
            if (err < -slew) { err = -slew; }
            s_pat_pause_dc += err;
            s_pat_pause_dc = clampf(s_pat_pause_dc, -8.0f, 8.0f);
        }
    }
    else
    {
        s_pat_quiet_s = 0.0f;
    }
    if (g_leak_patient_pause_zero_enable == 0U)
    {
        s_pat_pause_dc = 0.0f;   /* do not keep subtracting a held DC */
    }
    g_leak_patient_pause_dc_slm = s_pat_pause_dc;
    return s_pat_lpf - s_pat_pause_dc;
}

void Leak_Reseed(void)
{
    /* Reseed active backend. Does not wipe session accumulators. */
    s_seeded = false;
    s_patient_filt_seeded = false;
    Leak_PatientFiltReset();
    LeakVivo_Reset();
    g_leak_health_reseed_count++;
}

void Leak_SetHoldAdaptation(bool hold)
{
    s_hold_adaptation = hold;
}

void Leak_NudgeUnintentional(float delta_slm)
{
    if (!s_seeded || !(delta_slm == delta_slm))
    {
        return;
    }
    /* SIMPLE path: patient = (Q_total − vent) − uninten. Moving uninten
     * with Q_total cancels the step on Q_patient. Vivo uses a table×ratio
     * leak; leave it alone (idle AZ runs with SIMPLE by default). */
    if (s_algo == LEAK_ALGO_SIMPLE)
    {
        g_leak_uninten_slm += delta_slm;
    }
}

static void leakUpdateSimple(float total_q_slm, float mask_pressure_cmh2o, float dt_s)
{
    if (!(dt_s > 0.0f) || dt_s > 0.1f)
    {
        return;
    }

    g_leak_total_slm = total_q_slm;

    /* Flow sample was held: Q_total is frozen, so freeze the estimator too.
     * Letting the 10 s LPF converge onto a repeated sample is what turns a
     * stale-data run into a needle on Q_patient when fresh data resumes. */
    if (s_hold_adaptation && s_seeded)
    {
        return;
    }

    /* Modelled vent flow. Smooth mask pressure before sqrt() so PID
     * ripple does not modulate Q_vent and appear as noise on Q_patient. */
    const float p_raw = (mask_pressure_cmh2o > 0.0f) ? mask_pressure_cmh2o : 0.0f;
    const float p_tau = clampf(g_leak_vent_p_tau_s, 0.05f, 2.0f);
    const float p_alpha = dt_s / (p_tau + dt_s);
    if (!s_seeded)
    {
        s_mask_p_filt_cmh2o = p_raw;
    }
    else
    {
        s_mask_p_filt_cmh2o = p_alpha * p_raw
                            + (1.0f - p_alpha) * s_mask_p_filt_cmh2o;
    }
    g_leak_vent_slm = g_leak_vent_k * sqrtf(s_mask_p_filt_cmh2o)
                    * Flow_GetAltitudeFactor();

    /* Instantaneous "everything that isn't vent" flow. Over a complete
     * breath cycle this averages to Q_leak; cycle-by-cycle it swings
     * with tidal volume. The slow LPF below extracts the DC component. */
    const float instant_excess = total_q_slm - g_leak_vent_slm;
    g_leak_instant_excess_slm = instant_excess;

    /* Slow first-order IIR. tau = 10 s with 5 ms tick gives:
     *      alpha = dt / (tau + dt) = 0.005 / 10.005 = 4.998e-4
     * which is small (long memory) - exactly what we want for averaging
     * out the breath cycle. We use the bilinear form to stay accurate
     * if dt or tau is poked from the watch window. */
    const float tau = clampf(g_leak_tau_seconds, 1.0f, 60.0f);
    const float alpha = dt_s / (tau + dt_s);

    if (!s_seeded)
    {
        /* Reseed: snap to the instantaneous excess so we don't have to
         * wait ~30 s for the LPF to climb out of zero on therapy start. */
        g_leak_uninten_slm = instant_excess;
        s_seeded = true;
    }
    else
    {
        g_leak_uninten_slm = alpha * instant_excess
                           + (1.0f - alpha) * g_leak_uninten_slm;
    }

    /* Patient flow: remove vent orifice flow and the slow unintentional-
     * leak estimate from total flow. Over a full breath cycle this signal
     * is approximately zero-mean and tracks inhale/exhale shape.
     * The AC residual is high-passed sensor noise — apply a breath-band
     * LPF (2 Hz on breaths, ~0.7 Hz near zero) before EPR / telemetry. */
    const float patient_raw = instant_excess - g_leak_uninten_slm;
    g_flow_patient_raw_slm = patient_raw;
    g_flow_patient_slm = Leak_PatientFiltStep(patient_raw, dt_s);

    g_leak_step_count++;
    leakUpdateDerivedMetrics(mask_pressure_cmh2o, dt_s);
}

void Leak_Update(float total_q_slm, float mask_pressure_cmh2o, float dt_s)
{
    if (s_algo == LEAK_ALGO_VIVO)
    {
        LeakVivo_Update(total_q_slm, mask_pressure_cmh2o, s_therapy_sp_cmh2o, dt_s);

        g_leak_total_slm          = LeakVivo_GetTotalSlm();
        g_leak_vent_slm           = LeakVivo_GetVentSlm();
        g_leak_uninten_slm        = LeakVivo_GetUnintentionalSlm();
        g_leak_instant_excess_slm = LeakVivo_GetInstantExcessSlm();
        g_flow_patient_raw_slm    = LeakVivo_GetPatientRawSlm();
        g_flow_patient_slm        = LeakVivo_GetPatientSlm();
        g_leak_step_count++;
        leakUpdateDerivedMetrics(mask_pressure_cmh2o, dt_s);
        return;
    }

    leakUpdateSimple(total_q_slm, mask_pressure_cmh2o, dt_s);
}
