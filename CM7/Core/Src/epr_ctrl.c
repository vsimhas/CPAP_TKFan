/**
  ******************************************************************************
  * @file    epr_ctrl.c
  * @brief   EPR breath detection + setpoint relief.
  ******************************************************************************
  */

#include "epr_ctrl.h"
#include "therapy.h"
#include "stm32h7xx_hal.h"

#include <math.h>
#include <stddef.h>

/* === Tunables (debugger-writable volatiles) === */
volatile uint8_t g_epr_enabled                 = 0U;
volatile uint8_t g_epr_type                    = (uint8_t)EPR_TYPE_FULL_TIME;
volatile uint8_t g_epr_level                   = 1U;
volatile float   g_epr_relief_cmh2o            = 1.0f;
volatile float   g_epr_min_pressure_cmh2o      = 4.0f;

/* Relief restore (inhale rise): slewed for comfort. Expiratory apply: snap
 * for fast return to CPAP-level after inspiration ends. */
volatile float   g_epr_relief_apply_slew_cmh2o_per_s    = 12.0f;
volatile float   g_epr_relief_restore_slew_cmh2o_per_s  = 8.0f;

/* Breath detection thresholds (absolute minimums in slm). Adaptive logic
 * also scales thresholds as a fraction of the running flow baseline so a
 * high-leak bench rig and a masked patient both work without retuning. */
volatile float   g_epr_insp_thresh_slm         = 4.0f;
volatile float   g_epr_exp_enter_thresh_slm    = 3.0f;   /* enter exhale    */
volatile float   g_epr_exp_end_hyst_slm        = 0.5f;   /* end exhale      */

/* Fraction of flow baseline added to the absolute thresholds above.
 * With patient flow (~zero-mean) baseline is small; keep fractions low. */
volatile float   g_epr_insp_thresh_frac        = 0.02f;
volatile float   g_epr_exp_thresh_frac         = 0.02f;

volatile float   g_epr_flow_baseline_tau_s     = 2.5f;
volatile float   g_epr_baseline_track_deadband_slm = 5.0f;
/* R11 (trial 8, docs/APAP_FINDINGS_2026-10-12_breathsim.md): therapy started
 * on a patient who was already breathing seeded the baseline from a
 * mid-inspiration sample (+11 slm). The deadband tracker then only ever saw
 * flow within 5 slm of it - more mid-inspiration flow - so it stayed there
 * for the whole session. Each inspiration then began below the baseline,
 * the fallback end test fired at the 120 ms min hold, and every breath was
 * a ~215 ms / ~23 mL fragment that R5a rejected: no stamp, so FOT never
 * ended (replayed on the trial 8 flow with this file). Two guards:
 *   - seed at 0 (leak-corrected patient flow is zero-mean) unless the first
 *     sample is already inside the deadband;
 *   - a slow unconditional pull toward the mean of all flow, so a wrong
 *     baseline cannot hold; the deadband tracker takes over once within
 *     reach. ~+/-0.5 slm of breath ripple at 30 s. 0 = off. */
volatile float   g_epr_baseline_slow_tau_s     = 30.0f;

volatile float    g_epr_last_insp_peak_slm     = 0.0f; /* last qualified insp peak */
/* R5a (docs/APAP_FINDINGS_2026-10-12_breathsim.md, 19 Oct): the breath stamp
 * that the APAP uses (Epr_HasRecentBreath / the recovery-breath test) also
 * requires an inspiration of plausible LENGTH and VOLUME, not just a peak.
 * On the BreathSimulator, brief 6-11 slm flow transients (~0.15 s above
 * 4 slm, 10-40 mL) and single 4 Hz FOT half-cycles (~0.125 s) passed every
 * EPR test, restarted the APAP's no-breath window or ended FOT mid-apnea:
 * a 10 s apnea was missed and a 20 s one probed 8 s late. Real breaths on
 * that rig: 500 mL; the weakest tested (60 % hypopnea, ~200 mL, Ti 2.5 s)
 * still carry ~150 mL by the time the stamp is taken - note the stamp comes
 * at end of inspiration OR at the 1.2 s insp_max_hold, so these limits see
 * at most ~1.25 s of a long breath. Only the stamp is gated: EPR pressure
 * relief runs on the FSM phase and is unchanged. 0 disables a test. */
volatile uint32_t g_epr_apap_breath_min_insp_ms = 400U;
/* R8a (trial 7d): the fixed 80 mL sat right on real weak breaths. The leak
 * estimate reads ~2 slm high once breaths get small, so 207-227 mL hypopnea
 * breaths reached the APAP as 96-107 mL outside FOT and 29-73 mL in FOT
 * (normal breaths ~280 mL up to the stamp) - some passed, some failed,
 * NO_BREATH tripped and FOT then ran through the whole 60 % hypopnea. With
 * R6 removing the needles at the source and the 400 ms test rejecting the
 * 0.15 s needles and 0.125 s FOT half-cycles, volume is now a relative
 * backstop: min_vol_ratio x the median of the last qualified breath volumes,
 * never below min_insp_ml (the floor; also the limit before any history).
 * 0.15 x ~280 mL = ~42 mL here. min_vol_ratio = 0 -> fixed floor only. */
volatile float    g_epr_apap_breath_min_insp_ml = 30.0f;
volatile float    g_epr_apap_breath_min_vol_ratio = 0.15f;
volatile float    g_epr_apap_breath_vol_median_ml = 0.0f;   /* readback */
/* R9 (trial 7e + three-sensor capture, docs/APAP_FINDINGS_2026-10-12_
 * breathsim.md): when breaths shrink, the CPAP's patient flow steps DOWN
 * at once - mean -2.2 slm in a 60 % hypopnea (-1.2 at 50 %), ~0 in normal
 * breathing, back on the first normal breath. Only ~0.7 slm of that is real
 * (rig box leak); the rest forms inside the CPAP within each breath (sensor
 * path or vent/leak model - not yet split; a total-flow capture will). A
 * 60 % hypopnea breath (7.4 slm peak delivered) then peaked at +2.8 slm
 * absolute and never started an inspiration, so the APAP went NO_BREATH ->
 * FOT -> CA. With this on, inspiration start, the qualifying peak (also
 * what Epr_GetLastInspPeakSlm reports to the APAP recovery test) and the
 * R5a volume are measured against g_epr_flow_baseline_slm - like `dir`
 * already was - so a common offset cancels; and the baseline no longer
 * tracks during inspiration, so a weak breath (within the 5 slm tracking
 * deadband) cannot drag its own reference up. In normal breathing the
 * baseline sits near 0 and nothing changes. 0 = the previous absolute
 * tests.
 * R10 (trial 7f): OFF by default. The baseline only tracks inside its 5 slm
 * deadband, and a hypopnea breath's whole inspiration is inside it, so the
 * baseline rode +1..+2 slm onto weak breaths and made them smaller (55 %
 * hypopnea regressed to FOT + CA). The offset itself is fixed at its source
 * (g_leak_patient_pause_zero_enable, leak_estimator.c). */
volatile uint8_t  g_epr_rel_baseline_enable    = 0U;
volatile uint32_t g_epr_last_insp_ms           = 0U;    /* last candidate, readback */
volatile float    g_epr_last_insp_ml           = 0.0f;  /* last candidate, readback */
volatile uint32_t g_epr_apap_breath_rejects    = 0U;    /* peak OK but too short/small */
volatile float    g_epr_last_cand_peak_slm     = 0.0f;  /* R11 diag: last candidate's peak */
volatile uint8_t  g_epr_last_cand_stamped      = 0U;    /* R11 diag: 1 = it was stamped */

volatile uint32_t g_epr_insp_confirm_ms        = 50U;
volatile uint32_t g_epr_exp_confirm_ms         = 50U;
volatile uint32_t g_epr_insp_min_hold_ms       = 120U;
/* Max confirmed-inspiration time before expiratory relief is forced
 * (breath detector missed exhale). 0 = disabled. */
volatile uint32_t g_epr_insp_max_hold_ms       = 1200U;
volatile float   g_epr_min_insp_peak_slm       = 4.0f;
/* Min flow fall from insp peak (slm) before exhale can be accepted. */
volatile float   g_epr_exp_min_peak_fall_slm   = 6.0f;
/* Min time (ms) after last insp-peak update before exhale qualify. */
volatile uint32_t g_epr_exp_post_peak_ms       = 80U;
/* |Q_patient| below this (slm) = expiratory flow ended. */
volatile float   g_epr_exp_end_flow_slm        = 4.0f;
volatile uint32_t g_epr_exp_end_confirm_ms     = 120U;
/* Min quiet time after exp relief before next insp arm (ms). */
volatile uint32_t g_epr_insp_refractory_ms     = 350U;
volatile uint32_t g_epr_insp_dropout_ms        = 80U;

/* Advanced expiratory bump / dual-peak mitigation in the breath FSM
 * (post-peak gate, peak-fall, refractory, relief latch). Default off —
 * set to 1 in debugger to re-enable. Blower-side twin:
 * g_blower_ctrl_epr_exhale_mit_enable. */
volatile uint8_t  g_epr_bump_mit_enable          = 0U;
/* Drop setpoint to expiratory level on first qualified exhale flow (after
 * insp_min_hold) without waiting for exp_confirm. FSM still debounces. */
volatile uint8_t  g_epr_exp_early_relief_enable  = 1U;
/* End inspiration when Q falls from peak AND therapy pressure is reached
 * (before Q goes strongly negative). Default on. */
volatile uint8_t  g_epr_peak_fall_detect_enable  = 1U;
volatile float    g_epr_insp_pressure_band_cmh2o = 0.5f;

/* === Live state === */
volatile uint8_t g_epr_phase                   = (uint8_t)EPR_PHASE_UNKNOWN;
volatile float   g_epr_active_relief_cmh2o     = 0.0f;
volatile float   g_epr_flow_baseline_slm       = 0.0f;
volatile uint32_t g_epr_step_count             = 0U;
volatile uint8_t  g_epr_insp_armed             = 0U;
volatile float    g_epr_insp_peak_delta_slm    = 0.0f;
volatile float    g_epr_flow_delta_slm         = 0.0f;  /* Q - baseline    */
volatile uint8_t  g_epr_breath_fsm             = 0U;    /* BreathFsm_t     */
volatile int8_t   g_epr_insp_polarity          = 0;     /* +1/-1 in effect */

/* Inspiratory flow polarity ================================================
 *
 * Flow-sensor orientation is a fixed property of the board, not something to
 * rediscover each session: blower_ctrl.c already asserts it as
 * g_blower_ctrl_flow_ff_inhale_sign = +1 (inspiration is positive Q).
 *
 * The original code latched polarity from the FIRST flow excursion past
 * threshold after every reset and never revisited it. On a bench rig the
 * simulator's blower is already running when therapy starts, so that first
 * excursion is often EXPIRATORY - and polarity then locks inverted for the
 * whole session, silently.
 *
 * Measured 2026-09-04 (capture_20260904_161747.log): Epr_GetPhase() reported
 * INSPIRATION while Q_patient averaged -13.1 slm, and real inspiration (up to
 * +25 slm) sat in phase UNKNOWN. finalizeInspBreath() was handed a peak of
 * -4.5 slm, below APAP's min_peak of 4.0, so it early-returned on every
 * breath: the flow-limitation and hypopnea detectors never evaluated a single
 * one across the whole run, and Apap_GetFlPeakBaselineSlm() never left 0.
 *
 * Everything keyed on Epr_GetPhase() inherits this - EPR relief timing
 * included - so the sign is now configured, not guessed.
 *
 *   g_epr_insp_polarity_cfg = +1  inspiration is positive flow (default)
 *                             -1  inspiration is negative flow
 *                              0  restore the old first-excursion auto-detect
 */
volatile int8_t   g_epr_insp_polarity_cfg      = 1;

/* Watchdog on the configured sign. Integrates directed flow while the FSM
 * believes it is inspiring: with the sign right this sits strongly positive.
 * Persistently negative means the configured polarity is wrong for this
 * hardware - surfaced rather than silently corrected, because a sign that
 * flips itself mid-session is far harder to debug than one that complains. */
volatile float    g_epr_insp_polarity_evidence = 0.0f;
volatile uint8_t  g_epr_insp_polarity_mismatch = 0U;

/* === Internal breath FSM === */
typedef enum
{
    BREATH_IDLE = 0,
    BREATH_INSP_PENDING,
    BREATH_INSPIRATION,
    BREATH_EXP_PENDING,
    BREATH_EXPIRATION
} BreathFsm_t;

static BreathFsm_t s_breath_fsm = BREATH_IDLE;
static uint32_t    s_phase_timer_ms = 0U;
static float       s_cycle_insp_peak = 0.0f;
static float       s_cycle_insp_peak_q_slm = 0.0f;
static uint32_t    s_cycle_insp_start_ms = 0U;   /* R5a: INSP_PENDING entry */
static float       s_cycle_insp_vol_ml = 0.0f;   /* R5a: patient-directed volume */
#define EPR_VOL_HIST_N  9U
static float       s_vol_hist[EPR_VOL_HIST_N];    /* R8a: qualified breath volumes */
static uint8_t     s_vol_hist_n = 0U;
static uint8_t     s_vol_hist_idx = 0U;
static bool        s_baseline_seeded = false;
static int8_t      s_insp_polarity = 0;   /* +1: inhale = flow up; -1: inhale = flow down */
static uint32_t    s_startup_grace_ms = 0U;
static bool        s_pressurize_hold = false;
static float       s_hold_target_sp_cmh2o = 0.0f;
static uint32_t    s_hold_stable_ms = 0U;
static uint32_t    s_last_breath_ms = 0U;
static uint32_t    s_insp_enter_ms = 0U;
static uint32_t    s_insp_peak_ms = 0U;
static uint32_t    s_exp_commit_ms = 0U;
static uint32_t    s_dropout_timer_ms = 0U;
static bool        s_exp_relief_committed = false;
static bool        s_force_exp_relief = false;
static bool        s_awaiting_exp_complete = false;
static bool        s_early_exp_relief_latched = false;

#define EPR_STARTUP_GRACE_MS       5000U
#define EPR_PRESSURIZE_BAND_CMH2O  0.75f
#define EPR_PRESSURIZE_SETTLE_MS   1000U

static bool eprServiceActive(uint8_t therapy_state);
static bool eprInspirationActive(void);

static float effectiveInspThresh(void)
{
    const float base = (fabsf(g_epr_flow_baseline_slm) > 2.0f)
                         ? fabsf(g_epr_flow_baseline_slm) : 2.0f;
    return g_epr_insp_thresh_slm + base * g_epr_insp_thresh_frac;
}

static float effectiveExpEnterThresh(void)
{
    const float base = (fabsf(g_epr_flow_baseline_slm) > 2.0f)
                         ? fabsf(g_epr_flow_baseline_slm) : 2.0f;
    return g_epr_exp_enter_thresh_slm + base * g_epr_exp_thresh_frac;
}

static void syncFsmDebug(void)
{
    g_epr_breath_fsm = (uint8_t)s_breath_fsm;
    g_epr_insp_polarity = s_insp_polarity;
}

/* Map flow delta into "inspiration-positive" coordinates. Auto-detects
 * polarity on the first strong breath (handles bench test lungs where
 * inhale may appear as a flow drop instead of a rise). */
static float directedDelta(float delta)
{
    if (s_insp_polarity < 0)
    {
        return -delta;
    }
    return delta;
}

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

/* Apply the configured polarity. Called from every reset path so phase
 * decisions are correct from the first tick instead of waiting for an
 * excursion to guess from. Leaves polarity unresolved (0) only when the
 * caller has explicitly selected auto-detect. */
static void seedInspPolarity(void)
{
    s_insp_polarity = (g_epr_insp_polarity_cfg > 0) ? (int8_t)1
                    : ((g_epr_insp_polarity_cfg < 0) ? (int8_t)-1 : (int8_t)0);
    g_epr_insp_polarity = s_insp_polarity;
    g_epr_insp_polarity_evidence = 0.0f;
    g_epr_insp_polarity_mismatch = 0U;
}

static void lockPolarityFromDelta(float delta, float th)
{
    if (s_insp_polarity != 0)
    {
        return;
    }
    /* Auto-detect only when explicitly selected; see g_epr_insp_polarity_cfg. */
    if (g_epr_insp_polarity_cfg != 0)
    {
        seedInspPolarity();
        return;
    }
    if (delta > th)
    {
        s_insp_polarity = 1;
    }
    else if (delta < -th)
    {
        s_insp_polarity = -1;
    }
}

/* Accumulate evidence for/against the configured sign. q_dir is the
 * polarity-corrected flow, so during a correctly-signed inspiration it is
 * strongly positive. ~10 s memory: long enough to ignore a single odd
 * breath, short enough to flag a wrong board within a few cycles. */
static void updateInspPolarityEvidence(float q_dir, float dt_s)
{
    const float a = dt_s / (10.0f + dt_s);
    g_epr_insp_polarity_evidence =
        ((1.0f - a) * g_epr_insp_polarity_evidence) + (a * q_dir);
    g_epr_insp_polarity_mismatch =
        (g_epr_insp_polarity_evidence < -1.0f) ? 1U : 0U;
}

static float patientDirectedFlow(float q_slm)
{
    if (s_insp_polarity < 0)
    {
        return -q_slm;
    }
    return q_slm;
}

/* R9: the patient-directed baseline, 0 when the relative tests are off. */
static float baselineDirected(void)
{
    if (g_epr_rel_baseline_enable == 0U)
    {
        return 0.0f;
    }
    return patientDirectedFlow(g_epr_flow_baseline_slm);
}

static bool patientInspiratoryFlow(float q_slm, float insp_th)
{
    return (patientDirectedFlow(q_slm) - baselineDirected()) > insp_th;
}

static bool patientExpiratoryFlow(float q_slm, float exp_enter)
{
    return patientDirectedFlow(q_slm) < -exp_enter;
}

static bool patientFlowNearZero(float q_slm)
{
    const float th = clampf(g_epr_exp_end_flow_slm, 1.0f, 15.0f);
    return fabsf(q_slm) < th;
}

static void markExpiratoryReliefCommitted(uint32_t now_ms)
{
    s_exp_relief_committed = true;
    s_exp_commit_ms = now_ms;
}

static void slewToward(float *val, float target, float max_step)
{
    const float diff = target - *val;
    if (diff > max_step)
    {
        *val += max_step;
    }
    else if (diff < -max_step)
    {
        *val -= max_step;
    }
    else
    {
        *val = target;
    }
}

/* === Public API === */

void Epr_Init(void)
{
    g_epr_enabled             = 0U;
    g_epr_type                = (uint8_t)EPR_TYPE_FULL_TIME;
    g_epr_level               = 1U;
    g_epr_relief_cmh2o        = 1.0f;
    g_epr_min_pressure_cmh2o  = 4.0f;
    g_epr_relief_apply_slew_cmh2o_per_s    = 12.0f;
    g_epr_relief_restore_slew_cmh2o_per_s  = 8.0f;
    g_epr_flow_baseline_tau_s = 2.5f;
    g_epr_insp_thresh_slm     = 4.0f;
    g_epr_exp_enter_thresh_slm = 3.0f;
    g_epr_exp_end_hyst_slm    = 0.5f;
    g_epr_insp_thresh_frac    = 0.02f;
    g_epr_exp_thresh_frac     = 0.02f;
    g_epr_baseline_track_deadband_slm = 5.0f;
    g_epr_baseline_slow_tau_s = 30.0f;
    g_epr_insp_confirm_ms     = 50U;
    g_epr_exp_confirm_ms      = 50U;
    g_epr_insp_min_hold_ms    = 120U;
    g_epr_insp_max_hold_ms    = 1200U;
    g_epr_min_insp_peak_slm   = 4.0f;
    g_epr_apap_breath_min_insp_ms = 400U;
    g_epr_apap_breath_min_insp_ml = 30.0f;
    g_epr_apap_breath_min_vol_ratio = 0.15f;
    g_epr_rel_baseline_enable = 0U;   /* R10 */
    g_epr_exp_min_peak_fall_slm = 6.0f;
    g_epr_exp_post_peak_ms    = 80U;
    g_epr_exp_end_flow_slm    = 4.0f;
    g_epr_exp_end_confirm_ms  = 120U;
    g_epr_insp_refractory_ms  = 350U;
    g_epr_insp_dropout_ms     = 80U;
    g_epr_bump_mit_enable     = 0U;
    g_epr_exp_early_relief_enable = 1U;
    g_epr_peak_fall_detect_enable = 1U;
    g_epr_insp_pressure_band_cmh2o = 0.5f;

    g_epr_phase               = (uint8_t)EPR_PHASE_UNKNOWN;
    g_epr_active_relief_cmh2o = 0.0f;
    g_epr_flow_baseline_slm   = 0.0f;
    g_epr_step_count          = 0U;
    g_epr_insp_armed          = 0U;
    g_epr_insp_peak_delta_slm = 0.0f;
    g_epr_last_insp_peak_slm  = 0.0f;
    g_epr_flow_delta_slm      = 0.0f;
    g_epr_breath_fsm          = 0U;
    s_baseline_seeded         = false;
    s_breath_fsm              = BREATH_IDLE;
    s_phase_timer_ms          = 0U;
    s_cycle_insp_peak         = 0.0f;
    s_cycle_insp_peak_q_slm   = 0.0f;
    s_cycle_insp_start_ms     = 0U;
    s_cycle_insp_vol_ml       = 0.0f;
    s_vol_hist_n              = 0U;
    s_vol_hist_idx            = 0U;
    g_epr_apap_breath_vol_median_ml = 0.0f;
    seedInspPolarity();
    s_insp_enter_ms           = 0U;
    s_insp_peak_ms            = 0U;
    s_exp_commit_ms           = 0U;
    s_dropout_timer_ms        = 0U;
    s_exp_relief_committed    = false;
    s_force_exp_relief        = false;
    s_awaiting_exp_complete   = false;
    s_early_exp_relief_latched = false;
}

void  Epr_SetEnable(bool en)              { g_epr_enabled = en ? 1U : 0U;
                                            if (!en) { s_pressurize_hold = false; } }
bool  Epr_IsEnabled(void)                 { return (g_epr_enabled != 0U); }

void Epr_SetType(EprType_t type)
{
    g_epr_type = (type == EPR_TYPE_RAMP_ONLY) ? (uint8_t)EPR_TYPE_RAMP_ONLY
                                              : (uint8_t)EPR_TYPE_FULL_TIME;
}

EprType_t Epr_GetType(void)
{
    return (g_epr_type == (uint8_t)EPR_TYPE_RAMP_ONLY) ? EPR_TYPE_RAMP_ONLY
                                                       : EPR_TYPE_FULL_TIME;
}

void Epr_SetLevel(uint8_t level)
{
    if (level < 1U) { level = 1U; }
    if (level > 3U) { level = 3U; }
    g_epr_level = level;
    g_epr_relief_cmh2o = (float)level;
    if (!eprInspirationActive())
    {
        g_epr_active_relief_cmh2o = g_epr_relief_cmh2o;
    }
}

uint8_t Epr_GetLevel(void)                { return g_epr_level; }

void Epr_SetReliefCmh2o(float cmh2o)
{
    const float clamped = clampf(cmh2o, 1.0f, 3.0f);
    g_epr_relief_cmh2o = clamped;
    g_epr_level = (uint8_t)lrintf(clamped);
    if (g_epr_level < 1U) { g_epr_level = 1U; }
    if (g_epr_level > 3U) { g_epr_level = 3U; }
    if (!eprInspirationActive())
    {
        g_epr_active_relief_cmh2o = g_epr_relief_cmh2o;
    }
}

float Epr_GetReliefCmh2o(void)            { return g_epr_relief_cmh2o; }

void Epr_SetMinPressureCmh2o(float cmh2o)
{
    g_epr_min_pressure_cmh2o = clampf(cmh2o, 4.0f, 10.0f);
}

float Epr_GetMinPressureCmh2o(void)       { return g_epr_min_pressure_cmh2o; }

EprPhase_t Epr_GetPhase(void)             { return (EprPhase_t)g_epr_phase; }

const char* Epr_GetPhaseName(void)
{
    switch ((EprPhase_t)g_epr_phase)
    {
    case EPR_PHASE_INSPIRATION:  return "Inhale";
    case EPR_PHASE_EXPIRATION:   return "Exhale";
    case EPR_PHASE_UNKNOWN:
    default:                     return "---";
    }
}

float Epr_GetActiveReliefCmh2o(void)      { return g_epr_active_relief_cmh2o; }
float Epr_GetFlowBaselineSlm(void)        { return g_epr_flow_baseline_slm; }

static void resetBreathFsmCore(void)
{
    g_epr_phase               = (uint8_t)EPR_PHASE_UNKNOWN;
    g_epr_flow_baseline_slm   = 0.0f;
    g_epr_insp_armed          = 0U;
    g_epr_insp_peak_delta_slm = 0.0f;
    g_epr_flow_delta_slm      = 0.0f;
    g_epr_breath_fsm          = 0U;
    s_baseline_seeded         = false;
    s_breath_fsm              = BREATH_IDLE;
    s_phase_timer_ms          = 0U;
    s_cycle_insp_peak         = 0.0f;
    s_cycle_insp_peak_q_slm   = 0.0f;
    s_cycle_insp_start_ms     = 0U;
    s_cycle_insp_vol_ml       = 0.0f;
    s_vol_hist_n              = 0U;
    s_vol_hist_idx            = 0U;
    g_epr_apap_breath_vol_median_ml = 0.0f;
    seedInspPolarity();
    s_last_breath_ms          = 0U;
    g_epr_last_insp_peak_slm  = 0.0f;
    s_insp_enter_ms           = 0U;
    s_insp_peak_ms            = 0U;
    s_exp_commit_ms           = 0U;
    s_dropout_timer_ms        = 0U;
    s_exp_relief_committed    = false;
    s_force_exp_relief        = false;
    s_awaiting_exp_complete   = false;
    s_early_exp_relief_latched = false;
}

/* #8: switching EPR on or off mid-session changes the pressure at the next
 * breath boundary, where EPR changes it anyway, instead of at whatever moment
 * the command arrives (that was a 1-3 cmH2O step, mid-exhalation). */
#define EPR_DISENGAGE_MAX_MS  10000U   /* no inspiration by then: slew off */
static bool     s_engage_wait = false;     /* relief held off until a breath */
static bool     s_disengage_wait = false;  /* relief kept until inspiration */
static uint32_t s_disengage_ms = 0U;

void Epr_EngageAtNextBreath(void)
{
    s_engage_wait = true;
    s_disengage_wait = false;
}

void Epr_DisengageAtNextBreath(void)
{
    s_disengage_wait = true;
    s_disengage_ms = 0U;
    s_engage_wait = false;
}

bool Epr_IsTransitionPending(void)
{
    return s_engage_wait || s_disengage_wait;
}

/* A reset or a new session settles any pending switch: off stays off. */
static void settlePendingSwitch(void)
{
    if (s_disengage_wait)
    {
        g_epr_enabled = 0U;
    }
    s_disengage_wait = false;
    s_engage_wait = false;
}

void Epr_ResetBreathState(void)
{
    settlePendingSwitch();
    resetBreathFsmCore();
    g_epr_active_relief_cmh2o = 0.0f;
    s_startup_grace_ms = 0U;
    s_pressurize_hold = false;
    s_hold_stable_ms = 0U;
}

static float expiratorySetpointCmh2o(float base_setpoint_cmh2o)
{
    float sp = base_setpoint_cmh2o - g_epr_relief_cmh2o;
    if (sp < g_epr_min_pressure_cmh2o)
    {
        sp = g_epr_min_pressure_cmh2o;
    }
    return sp;
}

float Epr_GetExpiratorySetpointCmh2o(float base_setpoint_cmh2o,
                                     uint8_t therapy_state)
{
    if (!eprServiceActive(therapy_state))
    {
        return base_setpoint_cmh2o;
    }
    return expiratorySetpointCmh2o(base_setpoint_cmh2o);
}

void Epr_SetStartupHoldTarget(float expiratory_sp_cmh2o)
{
    if (g_epr_enabled == 0U)
    {
        s_pressurize_hold = false;
        s_hold_stable_ms = 0U;
        return;
    }

    s_pressurize_hold = true;
    s_hold_target_sp_cmh2o = expiratory_sp_cmh2o;
    s_hold_stable_ms = 0U;
}

void Epr_OnTherapyStart(void)
{
    settlePendingSwitch();
    resetBreathFsmCore();

    if (g_epr_enabled != 0U)
    {
        /* AirSense: therapy begins at expiratory (CPAP - level) pressure. */
        g_epr_active_relief_cmh2o = g_epr_relief_cmh2o;
        s_startup_grace_ms = EPR_STARTUP_GRACE_MS;
        s_hold_stable_ms = 0U;
    }
    else
    {
        g_epr_active_relief_cmh2o = 0.0f;
        s_startup_grace_ms = 0U;
        s_pressurize_hold = false;
        s_hold_stable_ms = 0U;
    }
}

static void mapPhaseForGui(BreathFsm_t fsm)
{
    switch (fsm)
    {
    case BREATH_INSP_PENDING:
    case BREATH_INSPIRATION:
        g_epr_phase = (uint8_t)EPR_PHASE_INSPIRATION;
        break;
    case BREATH_EXP_PENDING:
    case BREATH_EXPIRATION:
        g_epr_phase = (uint8_t)EPR_PHASE_EXPIRATION;
        break;
    case BREATH_IDLE:
    default:
        g_epr_phase = (uint8_t)EPR_PHASE_UNKNOWN;
        break;
    }
}

static bool eprServiceActive(uint8_t therapy_state)
{
    if (g_epr_enabled == 0U)
    {
        return false;
    }

    if (g_epr_type == (uint8_t)EPR_TYPE_RAMP_ONLY)
    {
        return (therapy_state == (uint8_t)THERAPY_STATE_RAMP);
    }

    return (therapy_state == (uint8_t)THERAPY_STATE_RAMP)
        || (therapy_state == (uint8_t)THERAPY_STATE_RUNNING);
}

static bool eprInspirationActive(void)
{
    if (s_startup_grace_ms > 0U)
    {
        return false;
    }

    if (s_pressurize_hold)
    {
        return false;
    }

    if (s_force_exp_relief)
    {
        return false;
    }

    /* Full CPAP only after inspiration is confirmed — pending state is
     * too sensitive to leak / spin-up flow and caused 8 cmH2O flapping. */
    if (s_breath_fsm != BREATH_INSPIRATION)
    {
        return false;
    }

    /* Latched after sustained exhale flow — do not toggle on brief flow
     * reversals (caused a double pressure peak during inspiration). */
    if ((g_epr_bump_mit_enable != 0U) && s_exp_relief_committed)
    {
        return false;
    }

    if ((g_epr_exp_early_relief_enable != 0U) && s_early_exp_relief_latched)
    {
        return false;
    }

    return true;
}

static bool expiratoryFlowQualified(float q_slm, float dir, float exp_enter,
                                    float min_peak, uint32_t now_ms)
{
    if (s_cycle_insp_peak < min_peak)
    {
        return false;
    }

    if (!patientExpiratoryFlow(q_slm, exp_enter))
    {
        return false;
    }

    if (dir >= -exp_enter)
    {
        return false;
    }

    const float min_fall = clampf(g_epr_exp_min_peak_fall_slm, 1.0f, 25.0f);
    const float fall_from_peak = s_cycle_insp_peak - dir;
    if (fall_from_peak < min_fall)
    {
        return false;
    }

    const uint32_t post_peak = (g_epr_exp_post_peak_ms > 0U)
                                 ? g_epr_exp_post_peak_ms : 1U;
    if ((now_ms - s_insp_peak_ms) < post_peak)
    {
        return false;
    }

    return true;
}

static void trackInspFlowPeak(float q_slm, uint32_t dt_ms)
{
    const float q_dir = patientDirectedFlow(q_slm);
    if (q_dir > s_cycle_insp_peak_q_slm)
    {
        s_cycle_insp_peak_q_slm = q_dir;
        s_insp_peak_ms = HAL_GetTick();
    }
    const float q_vol = q_dir - baselineDirected();   /* R9: relative */
    if (q_vol > 0.0f)
    {
        /* slm x ms -> mL: (1000 mL / 60000 ms) per slm */
        s_cycle_insp_vol_ml += q_vol * (float)dt_ms * (1.0f / 60.0f);
    }
}

static bool inspTargetPressureReached(float p_meas_cmh2o, float therapy_base_sp)
{
    if (therapy_base_sp <= 0.0f)
    {
        return false;
    }

    const float band = clampf(g_epr_insp_pressure_band_cmh2o, 0.1f, 2.0f);
    return p_meas_cmh2o >= (therapy_base_sp - band);
}

static bool flowFallingFromPeak(float q_slm, float min_peak_q)
{
    if (s_cycle_insp_peak_q_slm < min_peak_q)
    {
        return false;
    }

    const float q_dir = patientDirectedFlow(q_slm);
    const float min_fall = clampf(g_epr_exp_min_peak_fall_slm, 1.0f, 25.0f);
    return (s_cycle_insp_peak_q_slm - q_dir) >= min_fall;
}

static bool inspirationEndingDetected(float q_slm, float dir, float p_meas_cmh2o,
                                      float therapy_base_sp, float exp_enter,
                                      float min_peak, uint32_t now_ms)
{
    if (g_epr_bump_mit_enable != 0U)
    {
        return expiratoryFlowQualified(q_slm, dir, exp_enter, min_peak, now_ms);
    }

    if (g_epr_peak_fall_detect_enable != 0U)
    {
        const float min_peak_q = clampf(g_epr_min_insp_peak_slm, 2.0f, 30.0f);
        const uint32_t post_peak = (g_epr_exp_post_peak_ms > 0U)
                                     ? g_epr_exp_post_peak_ms : 1U;

        if (flowFallingFromPeak(q_slm, min_peak_q)
            && inspTargetPressureReached(p_meas_cmh2o, therapy_base_sp)
            && ((now_ms - s_insp_peak_ms) >= post_peak))
        {
            return true;
        }
    }

    /* Fallback: sustained expiratory flow (Q clearly negative). */
    return (dir < -exp_enter) && (s_cycle_insp_peak >= min_peak);
}

static void noteQualifiedBreath(uint32_t now_ms)
{
    /* Count a breath only after a finished inspiration with a real peak.
     * Stamping at 50 ms insp-confirm made obstructed gasps abort APAP FOT. */
    const float min_peak = clampf(g_epr_min_insp_peak_slm, 2.0f, 30.0f);
    /* R9: s_cycle_insp_peak is the directed peak of (q - baseline). */
    const float peak_q = (g_epr_rel_baseline_enable != 0U)
                           ? s_cycle_insp_peak : s_cycle_insp_peak_q_slm;
    g_epr_last_cand_peak_slm = peak_q;   /* R11 diag */
    g_epr_last_cand_stamped = 0U;
    if (peak_q < min_peak)
    {
        return;
    }
    /* R5a: length and volume, see g_epr_apap_breath_min_insp_ms. */
    const uint32_t insp_ms = now_ms - s_cycle_insp_start_ms;
    g_epr_last_insp_ms = insp_ms;
    g_epr_last_insp_ml = s_cycle_insp_vol_ml;
    const uint32_t min_ms = (g_epr_apap_breath_min_insp_ms > 2000U)
                              ? 2000U : g_epr_apap_breath_min_insp_ms;
    float min_ml = clampf(g_epr_apap_breath_min_insp_ml, 0.0f, 400.0f);
    {
        /* R8a: relative to the median of recent qualified breaths. */
        const float ratio = clampf(g_epr_apap_breath_min_vol_ratio, 0.0f, 0.8f);
        if ((ratio > 0.0f) && (s_vol_hist_n >= 3U))
        {
            float tmp[EPR_VOL_HIST_N];
            for (uint8_t i = 0U; i < s_vol_hist_n; i++) { tmp[i] = s_vol_hist[i]; }
            for (uint8_t i = 1U; i < s_vol_hist_n; i++)      /* insertion sort */
            {
                const float v = tmp[i];
                int8_t j = (int8_t)i - 1;
                while ((j >= 0) && (tmp[j] > v)) { tmp[j + 1] = tmp[j]; j--; }
                tmp[j + 1] = v;
            }
            const float med = tmp[s_vol_hist_n / 2U];
            g_epr_apap_breath_vol_median_ml = med;
            if ((ratio * med) > min_ml) { min_ml = ratio * med; }
        }
    }
    if (((min_ms > 0U) && (insp_ms < min_ms))
        || ((min_ml > 0.0f) && (s_cycle_insp_vol_ml < min_ml)))
    {
        g_epr_apap_breath_rejects++;
        return;
    }
    s_vol_hist[s_vol_hist_idx] = s_cycle_insp_vol_ml;
    s_vol_hist_idx = (uint8_t)((s_vol_hist_idx + 1U) % EPR_VOL_HIST_N);
    if (s_vol_hist_n < EPR_VOL_HIST_N) { s_vol_hist_n++; }
    s_last_breath_ms = now_ms;
    g_epr_last_insp_peak_slm = peak_q;
    g_epr_last_cand_stamped = 1U;
}

static void beginExpiratoryPhase(uint32_t now_ms, BreathFsm_t next_fsm)
{
    noteQualifiedBreath(now_ms);
    if (g_epr_bump_mit_enable != 0U)
    {
        markExpiratoryReliefCommitted(now_ms);
    }
    s_awaiting_exp_complete = true;
    s_breath_fsm = next_fsm;
    s_phase_timer_ms = 0U;
}

static void updatePressurizeHold(float p_meas_cmh2o, uint32_t dt_ms)
{
    if (!s_pressurize_hold)
    {
        return;
    }

    if (fabsf(p_meas_cmh2o - s_hold_target_sp_cmh2o) <= EPR_PRESSURIZE_BAND_CMH2O)
    {
        s_hold_stable_ms += dt_ms;
        if (s_hold_stable_ms >= EPR_PRESSURIZE_SETTLE_MS)
        {
            s_pressurize_hold = false;
            s_hold_stable_ms = 0U;
        }
    }
    else
    {
        s_hold_stable_ms = 0U;
    }
}

void Epr_Update(float q_slm, float p_meas_cmh2o, float therapy_base_sp_cmh2o,
                float dt_s)
{
    if (!(dt_s > 0.0f) || dt_s > 0.1f)
    {
        return;
    }

    const uint32_t dt_ms = (uint32_t)lrintf(dt_s * 1000.0f);

    if (s_startup_grace_ms > 0U)
    {
        if (dt_ms >= s_startup_grace_ms)
        {
            s_startup_grace_ms = 0U;
            /* Keep flow baseline from grace; only reset phase tracking. */
            s_breath_fsm       = BREATH_IDLE;
            s_phase_timer_ms   = 0U;
            s_cycle_insp_peak  = 0.0f;
            seedInspPolarity();
            s_insp_enter_ms    = 0U;
            g_epr_insp_armed   = 0U;
            mapPhaseForGui(BREATH_IDLE);
            syncFsmDebug();
        }
        else
        {
            s_startup_grace_ms -= dt_ms;
        }
    }

    updatePressurizeHold(p_meas_cmh2o, dt_ms);

    /* Slow baseline tracks mid-breath flow. Only update when flow is near
     * the current baseline so inspiration/exhalation excursions do not drag
     * the reference with them (major noise-immunity improvement). */
    const float tau = clampf(g_epr_flow_baseline_tau_s, 0.5f, 5.0f);
    const float alpha = dt_s / (tau + dt_s);
    const float deadband = clampf(g_epr_baseline_track_deadband_slm, 2.0f, 20.0f);

    if (!s_baseline_seeded)
    {
        /* R11: never seed from a mid-breath sample. */
        g_epr_flow_baseline_slm = (fabsf(q_slm) < deadband) ? q_slm : 0.0f;
        s_baseline_seeded = true;
    }
    else
    {
        const float slow_tau = clampf(g_epr_baseline_slow_tau_s, 0.0f, 300.0f);
        if (slow_tau > 0.0f)   /* R11: a wrong baseline cannot hold */
        {
            g_epr_flow_baseline_slm += (dt_s / (slow_tau + dt_s))
                                     * (q_slm - g_epr_flow_baseline_slm);
        }
        const float delta_for_base = q_slm - g_epr_flow_baseline_slm;
        const bool in_insp = (s_breath_fsm == BREATH_INSP_PENDING)
                          || (s_breath_fsm == BREATH_INSPIRATION);
        if ((fabsf(delta_for_base) < deadband)
            && !((g_epr_rel_baseline_enable != 0U) && in_insp))   /* R9 */
        {
            g_epr_flow_baseline_slm = alpha * q_slm
                                    + (1.0f - alpha) * g_epr_flow_baseline_slm;
        }
    }

    const float delta = q_slm - g_epr_flow_baseline_slm;
    g_epr_flow_delta_slm = delta;

    const float insp_th   = effectiveInspThresh();
    const float exp_enter = effectiveExpEnterThresh();
    const float min_peak  = clampf(g_epr_min_insp_peak_slm, 2.0f, 30.0f);
    const uint32_t insp_confirm = (g_epr_insp_confirm_ms > 0U)
                                    ? g_epr_insp_confirm_ms : 1U;
    const uint32_t exp_confirm  = (g_epr_exp_confirm_ms > 0U)
                                    ? g_epr_exp_confirm_ms : 1U;

    const float dir = directedDelta(delta);

    /* Ignore spin-up flow transients during startup grace. */
    if (s_startup_grace_ms > 0U)
    {
        s_breath_fsm = BREATH_IDLE;
        mapPhaseForGui(BREATH_IDLE);
        syncFsmDebug();
        g_epr_step_count++;
        return;
    }

    /* --- Breath FSM: confirm inhale, then accept moderate exhale dip --- */
    switch (s_breath_fsm)
    {
    case BREATH_IDLE:
    {
        g_epr_insp_armed = 0U;
        s_exp_relief_committed = false;
        s_force_exp_relief = false;
        s_cycle_insp_peak = 0.0f;
        s_cycle_insp_peak_q_slm = 0.0f;
        s_dropout_timer_ms = 0U;

        if (s_awaiting_exp_complete)
        {
            break;
        }

        const uint32_t now = HAL_GetTick();
        if (g_epr_bump_mit_enable != 0U)
        {
            /* Time-only refractory after exp relief. */
            if (s_exp_commit_ms != 0U)
            {
                const uint32_t since_commit = now - s_exp_commit_ms;
                const uint32_t refractory = (g_epr_insp_refractory_ms > 0U)
                                              ? g_epr_insp_refractory_ms : 0U;
                if ((refractory > 0U) && (since_commit < refractory))
                {
                    break;
                }
            }
        }

        if ((delta > insp_th) || (delta < -insp_th))
        {
            lockPolarityFromDelta(delta, insp_th);
        }

        if (patientInspiratoryFlow(q_slm, insp_th))
        {
            s_breath_fsm = BREATH_INSP_PENDING;
            s_phase_timer_ms = 0U;
            /* R11: a start below the baseline (dir < 0) is not a peak - with
             * fabsf it passed min_peak and armed the fallback end test. */
            s_cycle_insp_peak = (dir > 0.0f) ? dir : 0.0f;
            s_insp_peak_ms = now;
            s_cycle_insp_start_ms = now;
            s_cycle_insp_vol_ml = 0.0f;
        }
        break;
    }

    case BREATH_INSP_PENDING:
        if (patientInspiratoryFlow(q_slm, insp_th))
        {
            s_dropout_timer_ms = 0U;
            s_phase_timer_ms += dt_ms;
            if (dir > s_cycle_insp_peak)
            {
                s_cycle_insp_peak = dir;
                s_insp_peak_ms = HAL_GetTick();
            }
            trackInspFlowPeak(q_slm, dt_ms);
            if (s_phase_timer_ms >= insp_confirm)
            {
                s_breath_fsm = BREATH_INSPIRATION;
                s_force_exp_relief = false;
                s_exp_relief_committed = false;
                s_early_exp_relief_latched = false;
                g_epr_insp_armed = 1U;
                g_epr_insp_peak_delta_slm = s_cycle_insp_peak;
                s_phase_timer_ms = 0U;
                s_insp_enter_ms = HAL_GetTick();
                s_insp_peak_ms = s_insp_enter_ms;
            }
        }
        else
        {
            const uint32_t dropout = (g_epr_insp_dropout_ms > 0U)
                                       ? g_epr_insp_dropout_ms : 1U;
            s_dropout_timer_ms += dt_ms;
            s_phase_timer_ms = 0U;
            if (s_dropout_timer_ms >= dropout)
            {
                s_breath_fsm = BREATH_IDLE;
                s_dropout_timer_ms = 0U;
                s_cycle_insp_peak = 0.0f;
                s_cycle_insp_peak_q_slm = 0.0f;
            }
        }
        break;

    case BREATH_INSPIRATION:
        if (dir > s_cycle_insp_peak)
        {
            s_cycle_insp_peak = dir;
            g_epr_insp_peak_delta_slm = s_cycle_insp_peak;
            s_insp_peak_ms = HAL_GetTick();
        }
        trackInspFlowPeak(q_slm, dt_ms);

        {
            const uint32_t now = HAL_GetTick();
            const uint32_t insp_elapsed = now - s_insp_enter_ms;
            const uint32_t max_hold = g_epr_insp_max_hold_ms;

            if (max_hold > 0U && insp_elapsed >= max_hold)
            {
                s_force_exp_relief = true;
                beginExpiratoryPhase(now, BREATH_EXPIRATION);
                break;
            }

            if (inspirationEndingDetected(q_slm, dir, p_meas_cmh2o,
                                            therapy_base_sp_cmh2o, exp_enter,
                                            min_peak, now))
            {
                const uint32_t min_hold = (g_epr_insp_min_hold_ms > 0U)
                                            ? g_epr_insp_min_hold_ms : 1U;
                if (insp_elapsed >= min_hold)
                {
                    if (g_epr_exp_early_relief_enable != 0U)
                    {
                        s_early_exp_relief_latched = true;
                    }

                    s_phase_timer_ms += dt_ms;
                    if (s_phase_timer_ms >= exp_confirm)
                    {
                        beginExpiratoryPhase(now, BREATH_EXP_PENDING);
                    }
                }
                else
                {
                    s_phase_timer_ms = 0U;
                }
            }
            else
            {
                s_phase_timer_ms = 0U;
            }
        }
        break;

    case BREATH_EXP_PENDING:
        s_phase_timer_ms += dt_ms;
        if (s_phase_timer_ms >= exp_confirm)
        {
            s_breath_fsm = BREATH_EXPIRATION;
            s_phase_timer_ms = 0U;
        }
        break;

    case BREATH_EXPIRATION:
        if (patientFlowNearZero(q_slm))
        {
            s_phase_timer_ms += dt_ms;
            const uint32_t end_confirm = (g_epr_exp_end_confirm_ms > 0U)
                                           ? g_epr_exp_end_confirm_ms : 1U;
            if (s_phase_timer_ms >= end_confirm)
            {
                s_breath_fsm = BREATH_IDLE;
                g_epr_insp_armed = 0U;
                s_exp_relief_committed = false;
                s_force_exp_relief = false;
                s_exp_commit_ms = 0U;
                s_awaiting_exp_complete = false;
                s_early_exp_relief_latched = false;
                s_phase_timer_ms = 0U;
                s_cycle_insp_peak = 0.0f;
                s_cycle_insp_peak_q_slm = 0.0f;
                s_dropout_timer_ms = 0U;
            }
        }
        else
        {
            s_phase_timer_ms = 0U;
        }
        break;

    default:
        s_breath_fsm = BREATH_IDLE;
        break;
    }

    /* Sanity-check the configured sign against what the detector is actually
     * seeing. Only sampled inside a confirmed inspiration: if polarity is
     * right this is strongly positive, and a sustained negative reading means
     * the board's flow sense is opposite to g_epr_insp_polarity_cfg. */
    if (s_breath_fsm == BREATH_INSPIRATION)
    {
        updateInspPolarityEvidence(patientDirectedFlow(q_slm), dt_s);
    }

    mapPhaseForGui(s_breath_fsm);
    syncFsmDebug();
    g_epr_step_count++;
}

float Epr_ModifySetpoint(float base_setpoint_cmh2o, float dt_s,
                         uint8_t therapy_state)
{
    if (!eprServiceActive(therapy_state))
    {
        g_epr_active_relief_cmh2o = 0.0f;
        return base_setpoint_cmh2o;
    }

    if (!(dt_s > 0.0f) || dt_s > 0.1f)
    {
        if (!eprInspirationActive())
        {
            g_epr_active_relief_cmh2o = g_epr_relief_cmh2o;
            return expiratorySetpointCmh2o(base_setpoint_cmh2o);
        }
        g_epr_active_relief_cmh2o = 0.0f;
        return base_setpoint_cmh2o;
    }

    /* #8: just switched on - no relief until an inspiration has been seen,
     * so it first appears at the exhalation after it, like every breath. */
    if (s_engage_wait)
    {
        if (!eprInspirationActive())
        {
            g_epr_active_relief_cmh2o = 0.0f;
            return base_setpoint_cmh2o;
        }
        s_engage_wait = false;
    }

    float target_relief = 0.0f;
    bool  force_restore = false;
    if (s_disengage_wait)
    {
        /* #8: just switched off - keep relief until the next inspiration
         * restores it as usual (or, with no breath for 10 s, slew it off at
         * the same restore rate), then stop. */
        s_disengage_ms += (uint32_t)(dt_s * 1000.0f);
        force_restore = eprInspirationActive() || (s_disengage_ms >= EPR_DISENGAGE_MAX_MS);
        if (force_restore && (g_epr_active_relief_cmh2o <= 0.001f))
        {
            s_disengage_wait = false;
            g_epr_enabled = 0U;
            g_epr_active_relief_cmh2o = 0.0f;
            return base_setpoint_cmh2o;
        }
    }
    if (!eprInspirationActive() && !force_restore)
    {
        /* AirSense baseline: CPAP - level except during inspiration. */
        target_relief = g_epr_relief_cmh2o;
    }

    if (target_relief <= 0.0f)
    {
        /* Slewed restore during inspiration (comfort). */
        const float slew = clampf(g_epr_relief_restore_slew_cmh2o_per_s,
                                  1.0f, 40.0f);
        const float max_step = slew * dt_s;
        slewToward((float *)&g_epr_active_relief_cmh2o, 0.0f, max_step);
    }
    else
    {
        /* Snap expiratory relief when exhale is detected. */
        g_epr_active_relief_cmh2o = target_relief;
    }

    float sp = base_setpoint_cmh2o - g_epr_active_relief_cmh2o;
    if (sp < g_epr_min_pressure_cmh2o)
    {
        sp = g_epr_min_pressure_cmh2o;
    }

    if (s_pressurize_hold)
    {
        s_hold_target_sp_cmh2o = sp;
    }

    return sp;
}

bool Epr_IsInspiratoryPressureActive(void)
{
    return eprInspirationActive();
}

bool Epr_HasRecentBreath(uint32_t window_ms)
{
    if (s_last_breath_ms == 0U)
    {
        return false;
    }
    if (window_ms == 0U)
    {
        return false;
    }
    const uint32_t now = HAL_GetTick();
    return (now - s_last_breath_ms) < window_ms;
}

float Epr_GetLastInspPeakSlm(void)
{
    return g_epr_last_insp_peak_slm;
}

uint32_t Epr_GetLastInspMs(void)
{
    return g_epr_last_insp_ms;
}

float Epr_GetLastInspVolMl(void)
{
    return g_epr_last_insp_ml;
}

void Epr_GetDiag(EprDiag_t *d)
{
    if (d == NULL)
    {
        return;
    }
    d->cand_ms = g_epr_last_insp_ms;
    d->cand_ml = g_epr_last_insp_ml;
    d->cand_peak_slm = g_epr_last_cand_peak_slm;
    d->cand_stamped = g_epr_last_cand_stamped;
    d->rejects = g_epr_apap_breath_rejects;
    d->vol_median_ml = g_epr_apap_breath_vol_median_ml;
    d->since_stamp_ms = (s_last_breath_ms == 0U) ? 0xFFFFFFFFU
                                                 : (HAL_GetTick() - s_last_breath_ms);
    d->stamp_peak_slm = g_epr_last_insp_peak_slm;
    d->flow_baseline_slm = g_epr_flow_baseline_slm;
    d->fsm = (uint8_t)s_breath_fsm;
    d->polarity = g_epr_insp_polarity;
}
