/**
  ******************************************************************************
  * @file    apap_ctrl.c
  * @brief   Autoset (APAP) titration state machine.
  *
  *  Functional-equivalent prototype: FOT OSA/CA, FL, hypopnea, snore,
  *  leak-gated raises, down-titration, session AHI/RDI. All thresholds are
  *  debugger-tunable heuristics — not ResMed constants; clinical validation
  *  required before therapeutic claims.
  ******************************************************************************
  */

#include "apap_ctrl.h"
#include "therapy.h"
#include "epr_ctrl.h"
#include "blower_osc.h"
#include "blower_ctrl.h"
#include "flow_sensor.h"
#include "leak_estimator.h"
#include "stm32h7xx_hal.h"

#include <math.h>
#include <stdio.h>

/* === Tunables (debugger-writable) === */
/* Apnea timing - 2026-10-12, BreathSimulator AutoSet trials
 * (docs/APAP_FINDINGS_2026-10-12_breathsim.md, finding 1). MONITOR only
 * leaves for NO_BREATH once Epr_HasRecentBreath() has seen nothing for
 * breath_window_s; FOT starts no_breath_timeout_s after that; an event is
 * scored once s_episode_ms >= apnea_min_s. The episode clock used to start
 * at zero on NO_BREATH entry, so window + apnea_min = 12 + 10 = ~22 s with
 * no breath were needed and 10-20 s apneas were never probed or scored
 * (bench: FOT 14-17 s into the apnea; 20 s apneas scored 3/4). The clock
 * now starts at the window (that much breathless time has already passed),
 * and the defaults are 12 -> 8 s and 4 -> 2 s, so FOT starts ~10 s after
 * the last qualified breath (stamped at end of inspiration, i.e. ~7-8 s
 * into an apnea at I:E 1:1) and the first 2 s evaluation can score a
 * ~10 s apnea. The window doubles as the slow-breathing guard: breath
 * periods up to ~10 s (6 bpm) still never start FOT; slower breathing gets
 * a brief probe that the next qualified breath ends. Not yet bench-verified
 * with the new defaults - re-run the 10-25 s apnea-length sweep. */
volatile float   g_apap_no_breath_timeout_s        = 2.0f;  /* FOT start, after the window */
volatile float   g_apap_breath_window_s            = 8.0f;
/* Exit from NO_BREATH / FOT. Finding 3: a FIXED 10 slm made the outcome
 * depend on patient size - with ~20 slm normal peaks, a 60 % hypopnea
 * (peaks ~8 slm) kept FOT running and was scored as a central apnea (2/2).
 * The threshold is now resume_peak_ratio x the peak baseline the FL/Hyp
 * detector held when the episode began, never below resume_min_floor_slm
 * (the EPR breath qualifier is 4 slm). resume_min_peak_slm is only the
 * fallback when no baseline was established yet (start of therapy). */
volatile float   g_apap_resume_min_peak_slm        = 10.0f; /* fallback, no baseline */
volatile float   g_apap_resume_peak_ratio          = 0.30f; /* x baseline; 0 = fixed 10 */
volatile float   g_apap_resume_min_floor_slm       = 4.0f;
volatile float   g_apap_startup_grace_s            = 15.0f;
/* FOT is the most power-hungry state: on the bench (2026-10-12) supply
 * current peaked 1.05-1.12 A only while oscillating, vs <= 0.78 A for
 * breathing (finding 6). Brief peaks, but size the supply for them. */
volatile float   g_apap_osc_amplitude_cmh2o        = 1.0f;   /* setpoint peak; ~2.0 p-p */
volatile float   g_apap_osc_amplitude_max_cmh2o    = 1.75f; /* setpoint peak clamp */
volatile float   g_apap_osc_freq_hz                = 4.0f;
volatile float   g_apap_fot_eval_period_s          = 2.0f;   /* ~8 cycles @ 4 Hz */
volatile float   g_apap_osc_min_p_pp_cmh2o         = 0.15f;
volatile float   g_apap_osa_flow_pp_thresh_slm     = 2.0f;
volatile float   g_apap_apnea_min_s                = 10.0f; /* score/raise only after */
volatile float   g_apap_oa_raise_cmh2o_per_s       = 0.08f; /* independent duration heuristic */
volatile float   g_apap_oa_raise_max_cmh2o         = 2.0f;
volatile float   g_apap_her_oa_above_cmh2o         = 12.0f; /* For Her: milder OA above this */
volatile float   g_apap_her_oa_above_scale         = 0.50f;
volatile float   g_apap_raise_rate_standard_cmh2o_per_s = 0.050f; /* ~3/min */
volatile float   g_apap_raise_rate_soft_cmh2o_per_s     = 0.020f; /* ~1.2/min */
volatile float   g_apap_raise_rate_her_scale     = 0.75f;
volatile uint8_t g_apap_response_soft              = 0U;

/* Flow-limitation (MONITOR). */
volatile uint8_t g_apap_fl_enable                  = 1U;
volatile float   g_apap_fl_ratio                   = 0.70f;  /* peak < ratio*baseline */
volatile uint8_t g_apap_fl_breaths_needed          = 3U;
volatile uint8_t g_apap_fl_clear_breaths           = 2U;
/* 23 Oct (docs/APAP_FINDINGS_2026-10-12_breathsim.md, "Pressure lowering"):
 * on clear flow limitation ours rose ~0.5 cmH2O/min against the AirSense 11's
 * ~3. The 0.50 scale halved the 3/min standard rate, and the shape severity
 * was measured over the whole span from the threshold to 1.0, so typical FL
 * breaths sat on its 0.25 floor. Now full scale, and severity reaches 1 at
 * FL_SEV_SPAN past the threshold: plateau 90 % (flat ~0.80) ~2.4 cmH2O/min,
 * M 90 % (dip ~0.86) ~2.1, plateau 60 % (flat ~0.75, which the AirSense
 * barely raised for) ~0.9. */
volatile float   g_apap_fl_raise_scale             = 1.00f;
volatile float   g_apap_fl_min_peak_slm            = 4.0f;
volatile float   g_apap_fl_baseline_alpha          = 0.25f;
volatile float   g_apap_fl_flat_ratio              = 0.72f; /* mean/peak plateau */
/* R13 (docs/APAP_FINDINGS_2026-10-12_breathsim.md): the flatness used to be
 * accumulated only in the EPR inspiration phase, which ends at the 1.2 s
 * max-hold, so a 2.5 s inspiration was judged on its rising half: normal
 * breaths read 0.78-0.80 against the 0.78 line, and an M-shaped breath was
 * caught only by accident. The shape features now cover the whole
 * inspiration, from the flow rising through fl_seg_floor to its return
 * there. Re-tuned by replaying the 20 Oct BreathSimulator logs (CPAP patient
 * flow, whole inspiration, 5 runs): normal 0.61-0.68, hypopneas <= 0.68,
 * plateau 60 % 0.73-0.77, plateau 90 % 0.74-0.82, M 90 % 0.67-0.75.
 * The M shape is caught by its mid-inspiratory dip instead: the lowest flow
 * between the peaks of the two halves over the smaller of those peaks
 * (M 90 % median 0.87 on 20 Oct, 0.90 on 23 Oct; normal and hypopnea breaths
 * ~1.0). Below fl_dip_ratio the breath counts as flow-limited; 0 disables the
 * dip arm. 0.90 (first bench run, 23 Oct) sat on the M block's median and held
 * the FL flag only 25 % of the block; 0.93 catches 24/25 M breaths in both
 * runs, while no hypopnea or normal breathing turns FL on (at most 3/36
 * hypopnea and 1/745 normal breaths cross it, never 3 in a row). The
 * firmware's per-breath values matched the offline replay to +-0.003. */
volatile float   g_apap_fl_dip_ratio               = 0.93f;
volatile float   g_apap_fl_seg_floor_slm           = 0.0f;
/* Finding 4: the baseline only updates on non-limited breaths, so one or
 * two large arousal breaths (1.5x) lifted it until normal breaths read as
 * "peak < 0.70 x baseline", could no longer update it, and held a false FL
 * call for ~90 s (with arousals off: no FL call). Breaths above this
 * multiple of the baseline still count as non-limited but are kept out of
 * the baseline. 0 disables the rejection. */
volatile float   g_apap_fl_baseline_reject_ratio   = 1.40f;
/* R1 (re-test of 5ff3544, docs/APAP_FINDINGS_2026-10-12_breathsim.md): the
 * FL/Hyp peak baseline used to be zeroed on every NO_BREATH entry and
 * re-seeded, unchecked, from the first breath afterwards - the ~31 slm
 * arousal breath after an apnea (false FL even with the 1.4x rejection,
 * which then compared against the inflated value), or a hypopnea's own
 * 8-9 slm breath after a brief NO_BREATH visit (the 55 % hypopnea then
 * scored nothing, worse than before). The pre-episode baseline is now
 * restored when the episode ends, unless the episode lasted longer than
 * this (then the old baseline may be stale and is relearned). */
volatile float   g_apap_baseline_restore_max_s     = 120.0f;
/* R4 (re-test of 4ab7a0c): a 60 % hypopnea whose weak breaths were partly
 * missed went NO_BREATH -> FOT -> MONITOR three times and scored a central
 * apnea in each burst (every NO_BREATH entry starts the episode at the full
 * window, so the first FOT evaluation is already past apnea_min, and CA
 * scoring re-armed per burst). Now:
 *  - one apnea score per low-flow event: after a CA/OA is scored, no new
 *    one until a breath that is not hypopnea-limited (s_apnea_need_recovery);
 *  - a FOT burst ended by a breath that is still hypopnea-limited
 *    (< hyp_ratio x the pre-episode baseline) keeps the FL/Hyp progress,
 *    like a NO_BREATH visit that never reached FOT;
 *  - if that burst had scored a CA, the CA is provisional for this long:
 *    should the hypopnea detector then score the same event, the CA is
 *    withdrawn so the event counts once, as a hypopnea. If no hypopnea is
 *    scored in time, the CA stands (nothing is lost). 0 = never withdraw. */
volatile float   g_apap_ca_reclass_window_s        = 30.0f;
/* R7a (trial 7c): R5b froze an UNSEEDED leak estimate. The APAP entered
 * NO_BREATH/FOT before the patient's first breath, the start-up estimate
 * (~23 slm, vs ~12.7 once breathing) was frozen, patient flow sat ~12.7 slm
 * low, every real breath looked like an 18-57 mL fragment that R5a
 * rejected, and FOT ran ~56 s on a normally breathing patient. So the
 * apnea freeze is only applied once this many breaths have been seen since
 * therapy start, and never holds longer than leak_freeze_max_s per
 * episode. (g_leak_vivo_seed_done is NOT a breathing seed - it is the
 * start-up Q/table seed, which may precede any breath.) */
volatile uint8_t g_apap_leak_freeze_min_breaths    = 5U;
volatile float   g_apap_leak_freeze_max_s          = 60.0f;  /* 0 = no cap */
/* R7b (trial 7c): during FOT the 4 Hz ripple (+-3-6 slm) satisfies the EPR
 * peak-fall end-of-inspiration test ~0.4-1.2 s into a breath, so a real
 * 200-230 mL hypopnea breath was stamped as a 55-85 mL fragment and R5a
 * rejected it (replayed: during FOT median 720 ms / 55 mL, 13 % accepted;
 * outside FOT 97 %). The flow the EPR breath FSM sees is notched at the
 * oscillator frequency whenever the oscillator runs (APAP FOT or the
 * physiology probe). Breaths are < 1 Hz; the RBJ notch (Q 2, the same Q as
 * the FOT band-pass) passes ~99 % at 1 Hz. The APAP's FOT metrics keep
 * their own band-passed flow. */
volatile uint8_t g_apap_breath_notch_enable        = 1U;
/* R8b (trial 7d), the mirror of R4's reclassification: in a 55 % hypopnea
 * the Hyp scored at 14.2 s, then a FOT burst on the event's last weak
 * breaths scored a CA - one event, two scores. An episode that begins while
 * a Hyp is active or within this long of one, with breaths still
 * hypopnea-limited, has its CA/OA score DEFERRED to the episode's end: if it
 * ends on a still-limited breath it was the hypopnea's tail and is not
 * scored; if it ends on a normal-sized breath it was a real apnea and is
 * scored then (nothing lost). FOT treatment (OA raise) is unaffected - only
 * the count waits. 0 = off. */
volatile float   g_apap_hyp_ca_guard_s             = 30.0f;
/* R9: the FL/Hyp detector measures each breath's peak and mean against the
 * EPR flow baseline, so the CPAP's patient-flow step-down during weak
 * breathing (-2 to -3 slm) neither hides a hypopnea breath under
 * fl_min_peak nor skews the flattening index. Baseline ~0 in normal
 * breathing. 0 = absolute patient flow (previous).
 * R10: OFF by default - the EPR baseline climbs onto weak breaths (see
 * g_epr_rel_baseline_enable), and the offset is now fixed at its source. */
volatile uint8_t g_apap_rel_baseline_enable        = 0U;

/* Hypopnea (MONITOR): peak drop lasting ~>=10 s. */
volatile uint8_t g_apap_hyp_enable                 = 1U;
volatile float   g_apap_hyp_ratio                  = 0.50f;  /* peak < 50% baseline */
volatile uint8_t g_apap_hyp_breaths_needed         = 2U;
volatile float   g_apap_hyp_min_duration_s         = 10.0f;
volatile float   g_apap_hyp_raise_scale            = 0.70f;
volatile float   g_apap_hyp_hold_s                 = 20.0f;  /* raise window after detect */
/* R12 (trial 8b, docs/APAP_FINDINGS_2026-10-12_breathsim.md): R2 re-armed
 * the Hyp count on ONE breath that was not hypopnea-limited (peak >= 0.50 x
 * baseline). A 60 % hypopnea seen at 45-49 % had one breath wobble over 0.50,
 * and when the 20 s hold expired the same event was counted again. Re-arm
 * now needs hyp_rearm_breaths consecutive breaths at >= hyp_rearm_ratio x
 * baseline (the FL line), i.e. breathing clearly back to normal. */
volatile float   g_apap_hyp_rearm_ratio            = 0.70f;
volatile uint8_t g_apap_hyp_rearm_breaths          = 2U;
/* 23 Oct (AirSense 11 reference run, 20 Oct): on round, low-effort - i.e.
 * central - hypopneas the AirSense held 8.0 cmH2O and scored no FL, while
 * ours raised to ~10 through two paths:
 *  - the FL amplitude arm (peak < fl_ratio x baseline) called any reduced
 *    breath flow-limited. FL is now called on shape only (flatness / dip);
 *    the arm can be put back with fl_amp_arm 1. A reduced breath still
 *    holds the FL/Hyp baseline, exactly as before.
 *  - every scored hypopnea raised. A hypopnea now raises only when more than
 *    hyp_raise_shape_frac of its limited breaths are shape-limited
 *    (obstructive); a round one is scored and holds the pressure (no raise,
 *    and the lowering wait restarts). 0 = raise on every hypopnea. */
volatile uint8_t g_apap_fl_amp_arm_enable          = 0U;
volatile float   g_apap_hyp_raise_shape_frac       = 0.50f;

/* Snore: HF residual RMS on patient raw vs breath-band flow. */
volatile uint8_t g_apap_snore_enable               = 1U;
volatile float   g_apap_snore_rms_thresh_slm       = 0.35f;
volatile float   g_apap_snore_on_s                 = 1.5f;
volatile float   g_apap_snore_off_s                = 2.0f;
volatile float   g_apap_snore_raise_scale          = 0.35f;
volatile float   g_apap_snore_hp_hz                = 20.0f;  /* residual HP corner */

/* Down-titration + leak gate. */
volatile uint8_t g_apap_lower_enable               = 1U;
/* 23 Oct: lowering only began after 120 s with no FL / Hyp / snore, so in an
 * eventful night (gaps <= 120 s) the target could only ratchet up. The
 * AirSense 11 starts down as soon as the event clears, at ~0.3 cmH2O/min.
 * Now 20 s, then 0.3/min (was 0.48): a short hold against hunting, and a
 * slower fall in place of the long wait. */
volatile float   g_apap_lower_rate_cmh2o_per_s     = 0.005f; /* ~0.3/min */
volatile float   g_apap_stable_before_lower_s      = 20.0f;
volatile uint8_t g_apap_leak_gate_enable           = 1U;     /* block raises on HIGH */

/* === Live state === */
volatile uint8_t  g_apap_state                     = (uint8_t)APAP_STATE_IDLE;
volatile float    g_apap_target_cmh2o              = 8.0f;
volatile uint32_t g_apap_no_breath_ms              = 0U;
volatile uint32_t g_apap_osc_eval_ms               = 0U;
volatile float    g_apap_osc_p_min_cmh2o           = 0.0f;
volatile float    g_apap_osc_p_max_cmh2o           = 0.0f;
volatile float    g_apap_osc_q_min_slm             = 0.0f;
volatile float    g_apap_osc_q_max_slm             = 0.0f;
volatile uint8_t  g_apap_last_osa_detected         = 0U;
volatile uint8_t  g_apap_osa_treating              = 0U;
volatile float    g_apap_fot_p_pp_cmh2o           = 0.0f;
volatile float    g_apap_fot_q_pp_slm             = 0.0f;
volatile float    g_apap_fot_z_cmh2o_per_slm      = 0.0f;
volatile uint8_t  g_apap_fot_osa_likely           = 0U;
volatile uint8_t  g_apap_fl_active                = 0U;
volatile float    g_apap_fl_peak_baseline_slm     = 0.0f;
volatile float    g_apap_fl_last_peak_slm         = 0.0f;
volatile float    g_apap_fl_last_flat             = 0.0f; /* insp mean/peak */
volatile float    g_apap_fl_last_dip              = 1.0f; /* R13: 1 = no notch */
volatile uint8_t  g_apap_hyp_active               = 0U;
volatile uint8_t  g_apap_snore_active             = 0U;
volatile float    g_apap_snore_rms_slm            = 0.0f;
volatile uint32_t g_apap_stable_ms                = 0U;
volatile uint8_t  g_apap_lowering                 = 0U;
volatile uint8_t  g_apap_raise_blocked_leak       = 0U;

volatile uint32_t g_apap_evt_oa                   = 0U;
volatile uint32_t g_apap_evt_ca                   = 0U;
volatile uint32_t g_apap_evt_hyp                  = 0U;
volatile uint32_t g_apap_evt_fl                   = 0U;
volatile uint32_t g_apap_evt_snore                = 0U;
volatile uint32_t g_apap_therapy_ms               = 0U;

#define FOT_Q_PP_EPS_SLM  0.15f
#define FL_SEV_SPAN       0.10f   /* shape past its threshold for full FL raise */
#define FOT_Z_BLOCKED     999.0f

/* === Internal === */
static float     s_min_cmh2o = 5.0f;
static float     s_max_cmh2o = 20.0f;
static bool      s_for_her = false;
static bool      s_active = false;
static bool      s_osa_treating = false;
static FlowFotBp_t s_fot_bp_p;
static FlowFotBp_t s_fot_bp_q;
static float       s_fot_p_bp = 0.0f;
static float       s_fot_q_bp = 0.0f;
static bool       s_osa_event_armed = true;  /* count once per apnea episode */
static bool       s_ca_event_armed = true;
static uint32_t   s_running_enter_ms = 0U;
static uint32_t   s_episode_ms = 0U;
static float      s_oa_raise_applied = 0.0f;

/* FL / hypopnea share inspiratory peak tracking. */
static EprPhase_t s_fl_prev_phase = EPR_PHASE_UNKNOWN;
static float      s_fl_insp_peak_q = 0.0f;
static float      s_fl_baseline_slm = 0.0f;
static uint8_t    s_fl_streak = 0U;
static uint8_t    s_fl_good_streak = 0U;
static bool       s_fl_active = false;
static bool       s_fl_was_active = false;

static uint8_t    s_hyp_streak = 0U;
static uint32_t   s_hyp_limited_ms = 0U;
static bool       s_hyp_active = false;
static uint32_t   s_hyp_hold_ms = 0U;

/* Snore detector. */
static float      s_snore_prev_q = 0.0f;
static float      s_snore_hp = 0.0f;
static float      s_snore_rms2 = 0.0f;
static uint32_t   s_snore_on_ms = 0U;
static uint32_t   s_snore_off_ms = 0U;
static bool       s_snore_active = false;
static bool       s_snore_was_active = false;
static uint32_t   s_insp_ms = 0U;
static float      s_fl_insp_sum_q = 0.0f;
static uint32_t   s_fl_insp_n = 0U;
static float      s_fl_last_flat = 0.0f;
static uint32_t   s_since_insp_end_ms = 0U;   /* breath period for Hyp duration */
/* R13: one segment per inspiration for the shape features. */
#define FL_SEG_BIN_MS    40U
#define FL_SEG_MAX_BINS  100U                  /* 4 s */
static bool       s_fl_seg_active = false;
static bool       s_fl_seg_armed = false;      /* flow seen at/below the floor */
static bool       s_fl_seg_from_floor = false;
static bool       s_fl_seg_confirmed = false;  /* EPR inspiration onset inside */
static bool       s_fl_seg_overflow = false;
static float      s_fl_seg_bins[FL_SEG_MAX_BINS];
static uint32_t   s_fl_seg_nbins = 0U;
static float      s_fl_seg_bin_sum = 0.0f;
static uint32_t   s_fl_seg_bin_n = 0U;
static uint32_t   s_fl_seg_bin_ms = 0U;
static float      s_fl_last_dip = 1.0f;
static float      s_resume_ref_slm = 0.0f;    /* baseline held at episode start */
/* R2: one long hypopnea is one event - re-arm only after a breath that is
 * no longer hypopnea-limited. */
static bool       s_hyp_need_recovery = false;
static uint8_t    s_hyp_rearm_n = 0U;   /* R12: consecutive recovered breaths */
/* Central vs obstructive hypopnea: shape-limited share of its breaths. */
static uint8_t    s_hyp_ev_n = 0U;
static uint8_t    s_hyp_ev_shape_n = 0U;
static bool       s_hyp_obstructive = false;
/* R4 */
static bool       s_apnea_need_recovery = false;
static bool       s_episode_scored_ca = false;
static uint32_t   s_pending_ca_ms = 0U;
/* R7a / R7b */
static uint32_t   s_breaths_seen = 0U;
static bool       s_leak_frozen = false;
static uint32_t   s_leak_freeze_ms = 0U;
static FlowFotBp_t s_breath_notch_bp;
/* R8b */
static uint32_t   s_since_hyp_ms = 0xFFFFFFFFU;
static bool       s_episode_after_hyp = false;
static bool       s_deferred_ca = false;
static bool       s_deferred_oa = false;
/* R1: detector progress saved at NO_BREATH entry, restored if the episode
 * ends without ever reaching FOT (a hypopnea that briefly lost a breath). */
static bool       s_episode_reached_fot = false;
static uint8_t    s_save_fl_streak = 0U;
static uint8_t    s_save_fl_good_streak = 0U;
static bool       s_save_fl_active = false;
static uint8_t    s_save_hyp_streak = 0U;
static uint32_t   s_save_hyp_limited_ms = 0U;
static bool       s_save_hyp_active = false;
static uint32_t   s_save_hyp_hold_ms = 0U;
static bool       s_save_hyp_need_recovery = false;
static uint8_t    s_save_hyp_ev_n = 0U;
static uint8_t    s_save_hyp_ev_shape_n = 0U;
static bool       s_save_hyp_obstructive = false;

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static bool autosetMode(uint8_t mode)
{
    return (mode == (uint8_t)THERAPY_MODE_AUTOSET)
        || (mode == (uint8_t)THERAPY_MODE_AUTOSET_FOR_HER);
}

static float raiseRate(void)
{
    float rate = (g_apap_response_soft != 0U)
                     ? g_apap_raise_rate_soft_cmh2o_per_s
                     : g_apap_raise_rate_standard_cmh2o_per_s;
    if (s_for_her)
    {
        rate *= clampf(g_apap_raise_rate_her_scale, 0.25f, 1.0f);
    }
    return clampf(rate, 0.005f, 0.2f);
}

static void setState(ApapState_t st)
{
    g_apap_state = (uint8_t)st;
}

static void disableOsc(void)
{
    BlowerOsc_SetEnable(false);
}

static void configureOsc(void)
{
    /* Commanded on the PID setpoint. Mask p-p is often ~30-50% of this
     * (tube/mask compliance, leak). Tune g_apap_osc_amplitude_cmh2o. */
    const float ampl_max = clampf(g_apap_osc_amplitude_max_cmh2o, 0.5f, 2.5f);
    BlowerOsc_SetAmplitude(clampf(g_apap_osc_amplitude_cmh2o, 0.05f, ampl_max));
    BlowerOsc_SetFrequency(clampf(g_apap_osc_freq_hz, 2.0f, 8.0f));
    BlowerOsc_SetEnable(true);
}

static void resetOscStats(void)
{
    /* Restart p-p window on the last bandpassed samples. Do not reset
     * the biquads here — that would inject a transient every eval. */
    g_apap_osc_p_min_cmh2o = s_fot_p_bp;
    g_apap_osc_p_max_cmh2o = s_fot_p_bp;
    g_apap_osc_q_min_slm = s_fot_q_bp;
    g_apap_osc_q_max_slm = s_fot_q_bp;
    g_apap_osc_eval_ms = 0U;
}

static void trackOscStats(float dt_s)
{
    const float f0 = clampf(g_apap_osc_freq_hz, 2.0f, 8.0f);
    s_fot_p_bp = Flow_FotBpStep(&s_fot_bp_p, BlowerCtrl_GetMeasured(), f0, dt_s);
    s_fot_q_bp = Flow_FotBpStep(&s_fot_bp_q, Flow_GetPatientRawSlm(), f0, dt_s);
    const float p = s_fot_p_bp;
    const float q = s_fot_q_bp;
    if (p < g_apap_osc_p_min_cmh2o) { g_apap_osc_p_min_cmh2o = p; }
    if (p > g_apap_osc_p_max_cmh2o) { g_apap_osc_p_max_cmh2o = p; }
    if (q < g_apap_osc_q_min_slm)   { g_apap_osc_q_min_slm = q; }
    if (q > g_apap_osc_q_max_slm)   { g_apap_osc_q_max_slm = q; }
}

static bool startupGraceActive(void)
{
    if (g_apap_startup_grace_s <= 0.0f)
    {
        return false;
    }
    const uint32_t grace_ms = (uint32_t)(g_apap_startup_grace_s * 1000.0f);
    return (HAL_GetTick() - s_running_enter_ms) < grace_ms;
}

static bool recentBreath(void)
{
    const uint32_t window_ms = (uint32_t)(clampf(g_apap_breath_window_s,
                                                  4.0f, 30.0f) * 1000.0f);
    return Epr_HasRecentBreath(window_ms);
}

/* Leave FOT / no-breath only on a recovery-sized inspiration, not an
 * obstructed gasp. MONITOR still uses recentBreath() so hypopnea can run. */
static float recoveryNeedSlm(void)
{
    float need = clampf(g_apap_resume_min_peak_slm, 4.0f, 40.0f);
    const float ratio = clampf(g_apap_resume_peak_ratio, 0.0f, 0.8f);
    if ((ratio > 0.0f) && (s_resume_ref_slm > 0.0f))
    {
        const float floor_slm = clampf(g_apap_resume_min_floor_slm, 2.0f, 20.0f);
        need = ratio * s_resume_ref_slm;
        if (need < floor_slm) { need = floor_slm; }
    }
    return need;
}

static bool recoveryBreath(void)
{
    if (!recentBreath())
    {
        return false;
    }
    return Epr_GetLastInspPeakSlm() >= recoveryNeedSlm();
}

static bool raiseAllowed(void)
{
    if (g_apap_leak_gate_enable == 0U)
    {
        g_apap_raise_blocked_leak = 0U;
        return true;
    }
    const bool blocked = (Leak_GetSeverity() == LEAK_SEV_HIGH);
    g_apap_raise_blocked_leak = blocked ? 1U : 0U;
    return !blocked;
}

static void applyRaise(float scale, float dt_s)
{
    if (!raiseAllowed())
    {
        return;
    }
    g_apap_target_cmh2o += raiseRate() * clampf(scale, 0.05f, 1.0f) * dt_s;
    g_apap_stable_ms = 0U;
    g_apap_lowering = 0U;
}

static bool applyStepRaise(float delta_cmh2o)
{
    if (!(delta_cmh2o > 0.0f) || !raiseAllowed())
    {
        return false;
    }
    g_apap_target_cmh2o += delta_cmh2o;
    g_apap_stable_ms = 0U;
    g_apap_lowering = 0U;
    return true;
}

/* OA raise after scored apnea (~10 s): amount tracks episode length, not a
 * continuous FOT ramp. Central (s_osa_treating false) does not raise. */
static void applyOaDurationRaise(void)
{
    if (!s_osa_treating)
    {
        return;
    }

    const float apnea_min_s = clampf(g_apap_apnea_min_s, 6.0f, 30.0f);
    const float episode_s = (float)s_episode_ms / 1000.0f;
    if (episode_s < apnea_min_s)
    {
        return;
    }

    const float per_s = clampf(g_apap_oa_raise_cmh2o_per_s, 0.01f, 0.50f);
    const float cap = clampf(g_apap_oa_raise_max_cmh2o, 0.2f, 6.0f);
    float want = (episode_s - apnea_min_s) * per_s;
    if (s_for_her
        && (g_apap_target_cmh2o
            >= clampf(g_apap_her_oa_above_cmh2o, 8.0f, 18.0f)))
    {
        want *= clampf(g_apap_her_oa_above_scale, 0.10f, 1.0f);
    }
    want = clampf(want, 0.0f, cap);

    const float delta = want - s_oa_raise_applied;
    if (delta > 0.02f && applyStepRaise(delta))
    {
        s_oa_raise_applied += delta;
    }
}

static float sessionHours(void)
{
    if (g_apap_therapy_ms < 1000U)
    {
        return 0.0f;
    }
    return (float)g_apap_therapy_ms / 3600000.0f;
}

static void refreshFotMetrics(void)
{
    const float p_pp = g_apap_osc_p_max_cmh2o - g_apap_osc_p_min_cmh2o;
    const float q_pp = g_apap_osc_q_max_slm - g_apap_osc_q_min_slm;
    const float p_min = clampf(g_apap_osc_min_p_pp_cmh2o, 0.05f, 0.8f);
    const float q_thr = clampf(g_apap_osa_flow_pp_thresh_slm, 0.5f, 10.0f);

    g_apap_fot_p_pp_cmh2o = p_pp;
    g_apap_fot_q_pp_slm = q_pp;
    if (q_pp > FOT_Q_PP_EPS_SLM)
    {
        g_apap_fot_z_cmh2o_per_slm = p_pp / q_pp;
    }
    else
    {
        g_apap_fot_z_cmh2o_per_slm = FOT_Z_BLOCKED;
    }

    const bool osa = (p_pp >= p_min) && (q_pp < q_thr);
    g_apap_fot_osa_likely = osa ? 1U : 0U;
}

static bool evaluateOscForOsa(void)
{
    refreshFotMetrics();
    return (g_apap_fot_osa_likely != 0U);
}

static void enterFotProbe(void)
{
    Flow_FotBpReset(&s_fot_bp_p);
    Flow_FotBpReset(&s_fot_bp_q);
    s_fot_p_bp = 0.0f;
    s_fot_q_bp = 0.0f;
    resetOscStats();
    s_osa_treating = false;
    g_apap_osa_treating = 0U;
    g_apap_last_osa_detected = 0U;
    s_osa_event_armed = true;
    s_ca_event_armed = true;
    s_episode_reached_fot = true;
    configureOsc();
    setState(APAP_STATE_OSCILLATE);
}

static void restoreDetectorsAfterEpisode(void)
{
    const float min_peak = clampf(g_apap_fl_min_peak_slm, 1.0f, 30.0f);
    const uint32_t max_ms = (uint32_t)(clampf(g_apap_baseline_restore_max_s,
                                              0.0f, 3600.0f) * 1000.0f);
    if ((s_resume_ref_slm >= min_peak) && (s_episode_ms <= max_ms))
    {
        s_fl_baseline_slm = s_resume_ref_slm;
        g_apap_fl_peak_baseline_slm = s_fl_baseline_slm;
    }
    /* R4: a FOT burst ended by a breath that is still hypopnea-limited was
     * a weak hypopnea, not an apnea. */
    const float hyp_ratio = clampf(g_apap_hyp_ratio, 0.20f, 0.80f);
    const bool ended_limited = (s_resume_ref_slm >= min_peak)
        && (Epr_GetLastInspPeakSlm() < (hyp_ratio * s_resume_ref_slm));
    /* R8b: resolve a deferred score now that the ending breath is known. */
    if (s_deferred_oa || s_deferred_ca)
    {
        if (!ended_limited)
        {
            if (s_deferred_oa) { g_apap_evt_oa++; }
            else               { g_apap_evt_ca++; }
            s_apnea_need_recovery = true;
        }
        s_deferred_oa = false;
        s_deferred_ca = false;
    }
    if (ended_limited && s_episode_scored_ca)
    {
        s_pending_ca_ms = (uint32_t)(clampf(g_apap_ca_reclass_window_s,
                                            0.0f, 120.0f) * 1000.0f);
    }
    if (!s_episode_reached_fot || ended_limited)
    {
        /* Only a breath or two went missing (no FOT), or the episode ended
         * on a still-limited breath: carry on with the FL / hypopnea in
         * progress instead of starting it over. A FOT episode that ended on
         * a normal-sized breath was an apnea; those start afresh. */
        s_fl_streak = s_save_fl_streak;
        s_fl_good_streak = s_save_fl_good_streak;
        s_fl_active = s_save_fl_active;
        s_fl_was_active = s_save_fl_active;   /* no new FL edge to count */
        g_apap_fl_active = s_fl_active ? 1U : 0U;
        s_hyp_streak = s_save_hyp_streak;
        s_hyp_limited_ms = s_save_hyp_limited_ms;
        s_hyp_active = s_save_hyp_active;
        s_hyp_hold_ms = s_save_hyp_hold_ms;
        s_hyp_need_recovery = s_save_hyp_need_recovery;
        s_hyp_ev_n = s_save_hyp_ev_n;
        s_hyp_ev_shape_n = s_save_hyp_ev_shape_n;
        s_hyp_obstructive = s_save_hyp_obstructive;
        g_apap_hyp_active = s_hyp_active ? 1U : 0U;
    }
}

static void releaseLeakFreeze(void)
{
    Leak_SetApneaFreeze(false);
    s_leak_frozen = false;
    s_leak_freeze_ms = 0U;
}

static void exitProbeToMonitor(void)
{
    restoreDetectorsAfterEpisode();
    releaseLeakFreeze();          /* R5b */
    disableOsc();
    s_osa_treating = false;
    g_apap_osa_treating = 0U;
    g_apap_last_osa_detected = 0U;
    g_apap_no_breath_ms = 0U;
    g_apap_stable_ms = 0U;
    g_apap_lowering = 0U;
    s_osa_event_armed = true;
    s_ca_event_armed = true;
    s_episode_ms = 0U;
    s_oa_raise_applied = 0.0f;
    setState(APAP_STATE_MONITOR);
}

static void resetBreathDetectors(void)
{
    s_fl_prev_phase = EPR_PHASE_UNKNOWN;
    s_fl_insp_peak_q = 0.0f;
    s_fl_baseline_slm = 0.0f;
    s_fl_streak = 0U;
    s_fl_good_streak = 0U;
    s_fl_active = false;
    s_fl_was_active = false;
    g_apap_fl_active = 0U;
    g_apap_fl_peak_baseline_slm = 0.0f;
    g_apap_fl_last_peak_slm = 0.0f;
    g_apap_fl_last_flat = 0.0f;
    g_apap_fl_last_dip = 1.0f;
    s_fl_insp_sum_q = 0.0f;
    s_fl_insp_n = 0U;
    s_fl_last_flat = 0.0f;
    s_fl_last_dip = 1.0f;
    /* R13: a breath already under way is skipped - the next segment only
     * starts once the flow has been at or below the floor. */
    s_fl_seg_active = false;
    s_fl_seg_armed = false;
    s_fl_seg_from_floor = false;
    s_fl_seg_confirmed = false;
    s_fl_seg_overflow = false;
    s_fl_seg_nbins = 0U;
    s_fl_seg_bin_sum = 0.0f;
    s_fl_seg_bin_n = 0U;
    s_fl_seg_bin_ms = 0U;

    s_hyp_streak = 0U;
    s_hyp_limited_ms = 0U;
    s_hyp_active = false;
    s_hyp_hold_ms = 0U;
    s_hyp_need_recovery = false;
    s_hyp_rearm_n = 0U;
    s_hyp_ev_n = 0U;
    s_hyp_ev_shape_n = 0U;
    s_hyp_obstructive = false;
    g_apap_hyp_active = 0U;

    s_snore_prev_q = 0.0f;
    s_snore_hp = 0.0f;
    s_snore_rms2 = 0.0f;
    s_snore_on_ms = 0U;
    s_snore_off_ms = 0U;
    s_snore_active = false;
    s_snore_was_active = false;
    g_apap_snore_active = 0U;
    g_apap_snore_rms_slm = 0.0f;
    s_insp_ms = 0U;
    s_fl_insp_sum_q = 0.0f;
    s_fl_insp_n = 0U;
    s_since_insp_end_ms = 0U;
}

static void resetSessionEvents(void)
{
    g_apap_evt_oa = 0U;
    g_apap_evt_ca = 0U;
    g_apap_evt_hyp = 0U;
    g_apap_evt_fl = 0U;
    g_apap_evt_snore = 0U;
    g_apap_therapy_ms = 0U;
}

static void noteFlEdge(void)
{
    if (s_fl_active && !s_fl_was_active)
    {
        g_apap_evt_fl++;
    }
    s_fl_was_active = s_fl_active;
}

static void noteSnoreEdge(void)
{
    if (s_snore_active && !s_snore_was_active)
    {
        g_apap_evt_snore++;
    }
    s_snore_was_active = s_snore_active;
}

static float flDipThreshold(void)
{
    return (g_apap_fl_dip_ratio > 0.0f) ? clampf(g_apap_fl_dip_ratio, 0.50f, 0.98f)
                                        : 0.0f;   /* dip arm off */
}

static float segDipRatio(void)
{
    /* R13: M-type limitation, a mid-inspiratory notch between two humps:
     * the lowest bin between the peaks of the two halves, over the smaller
     * of those peaks. 1 = no notch (single-peaked or flat-topped). */
    const uint32_t nb = s_fl_seg_nbins;
    if (nb < 6U)
    {
        return 1.0f;
    }
    const uint32_t mid = nb / 2U;
    uint32_t i1 = 0U;
    for (uint32_t i = 1U; i < mid; i++)
    {
        if (s_fl_seg_bins[i] > s_fl_seg_bins[i1]) { i1 = i; }
    }
    uint32_t i2 = mid;
    for (uint32_t i = mid + 1U; i < nb; i++)
    {
        if (s_fl_seg_bins[i] > s_fl_seg_bins[i2]) { i2 = i; }
    }
    float valley = s_fl_seg_bins[i1];
    for (uint32_t i = i1 + 1U; i <= i2; i++)
    {
        if (s_fl_seg_bins[i] < valley) { valley = s_fl_seg_bins[i]; }
    }
    const float hump = fminf(s_fl_seg_bins[i1], s_fl_seg_bins[i2]);
    return (hump > 0.5f) ? clampf(valley / hump, 0.0f, 1.0f) : 1.0f;
}

static void finalizeInspBreath(float peak_slm, uint32_t approx_breath_ms,
                               bool shape_ok)
{
    const float min_peak = clampf(g_apap_fl_min_peak_slm, 1.0f, 30.0f);
    const float fl_ratio = clampf(g_apap_fl_ratio, 0.40f, 0.95f);
    const float hyp_ratio = clampf(g_apap_hyp_ratio, 0.20f, 0.80f);
    const float alpha = clampf(g_apap_fl_baseline_alpha, 0.05f, 0.75f);
    const float flat_thr = clampf(g_apap_fl_flat_ratio, 0.55f, 0.95f);
    const float dip_thr = flDipThreshold();
    const uint8_t fl_need = (g_apap_fl_breaths_needed < 1U) ? 1U
                          : ((g_apap_fl_breaths_needed > 10U) ? 10U
                                                             : g_apap_fl_breaths_needed);
    const uint8_t fl_clear = (g_apap_fl_clear_breaths < 1U) ? 1U
                           : ((g_apap_fl_clear_breaths > 10U) ? 10U
                                                             : g_apap_fl_clear_breaths);
    const uint8_t hyp_need = (g_apap_hyp_breaths_needed < 1U) ? 1U
                           : ((g_apap_hyp_breaths_needed > 10U) ? 10U
                                                               : g_apap_hyp_breaths_needed);
    const uint32_t hyp_need_ms =
        (uint32_t)(clampf(g_apap_hyp_min_duration_s, 5.0f, 60.0f) * 1000.0f);

    g_apap_fl_last_peak_slm = peak_slm;

    /* Independent flattening heuristic: insp mean/peak. Square/plateau
     * waveforms sit near 1; peaked breaths sit nearer 0.5. R13: over the
     * whole inspiration; a breath without a clean segment (shape_ok false)
     * is judged on its peak only, as flat 0 / dip 1. */
    {
        const float mean_q = (s_fl_insp_n > 0U)
                                 ? (s_fl_insp_sum_q / (float)s_fl_insp_n)
                                 : 0.0f;
        const float flat = (shape_ok && (peak_slm > 0.5f))
                               ? clampf(mean_q / peak_slm, 0.0f, 1.0f)
                               : 0.0f;
        const float dip = shape_ok ? segDipRatio() : 1.0f;
        s_fl_last_flat = flat;
        g_apap_fl_last_flat = flat;
        s_fl_last_dip = dip;
        g_apap_fl_last_dip = dip;
    }

    if (!(peak_slm >= min_peak))
    {
        return;
    }
    if (s_breaths_seen < 1000000U) { s_breaths_seen++; }   /* R7a */

    if (!(s_fl_baseline_slm >= min_peak))
    {
        s_fl_baseline_slm = peak_slm;
        g_apap_fl_peak_baseline_slm = s_fl_baseline_slm;
        return;
    }

    const bool fl_peak_drop = (peak_slm < (fl_ratio * s_fl_baseline_slm));
    const bool fl_flat = (s_fl_last_flat >= flat_thr);
    const bool fl_dip = (s_fl_last_dip < dip_thr);
    const bool fl_shape = fl_flat || fl_dip;
    /* FL on shape only unless the amplitude arm is enabled; a reduced breath
     * still keeps out of the baseline either way. */
    const bool fl_limited = fl_shape
                         || (fl_peak_drop && (g_apap_fl_amp_arm_enable != 0U));
    const bool baseline_hold = fl_shape || fl_peak_drop;
    const bool hyp_limited = (peak_slm < (hyp_ratio * s_fl_baseline_slm));
    if (!hyp_limited)
    {
        s_apnea_need_recovery = false;   /* R4: breathing recovered */
    }

    /* --- Flow limitation --- */
    if (g_apap_fl_enable != 0U)
    {
        if (fl_limited)
        {
            if (s_fl_streak < 255U) { s_fl_streak++; }
            s_fl_good_streak = 0U;
        }
        else
        {
            if (s_fl_good_streak < 255U) { s_fl_good_streak++; }
            s_fl_streak = 0U;
        }
        if (!baseline_hold)
        {
            const float rej = clampf(g_apap_fl_baseline_reject_ratio, 0.0f, 4.0f);
            const bool arousal_sized = (rej >= 1.05f)
                                    && (peak_slm > (rej * s_fl_baseline_slm));
            if (!arousal_sized)
            {
                s_fl_baseline_slm =
                    ((1.0f - alpha) * s_fl_baseline_slm) + (alpha * peak_slm);
            }
        }
        if (s_fl_streak >= fl_need) { s_fl_active = true; }
        if (s_fl_good_streak >= fl_clear) { s_fl_active = false; }
        g_apap_fl_active = s_fl_active ? 1U : 0U;
        noteFlEdge();
    }

    /* --- Hypopnea --- */
    if (g_apap_hyp_enable != 0U)
    {
        if (hyp_limited)
        {
            s_hyp_rearm_n = 0U;   /* R12 */
            if (s_hyp_streak < 255U) { s_hyp_streak++; }
            if (s_hyp_ev_n < 255U)
            {
                s_hyp_ev_n++;
                if (fl_shape) { s_hyp_ev_shape_n++; }
            }
            {
                /* Strictly more than the fraction: the dip arm is noisy on
                 * small breaths, and a round 60 % hypopnea with 1 notched
                 * breath of its first 2 raised 0.18 cmH2O under ">=". */
                const float need = clampf(g_apap_hyp_raise_shape_frac, 0.0f, 1.0f);
                s_hyp_obstructive = (need <= 0.0f)
                    || ((float)s_hyp_ev_shape_n > (need * (float)s_hyp_ev_n));
            }
            s_hyp_limited_ms += (approx_breath_ms > 0U) ? approx_breath_ms : 3000U;
            if ((s_hyp_streak >= hyp_need) && (s_hyp_limited_ms >= hyp_need_ms)
                && !s_hyp_active && !s_hyp_need_recovery)
            {
                s_hyp_active = true;
                s_hyp_need_recovery = true;   /* R2: no re-count until recovery */
                g_apap_evt_hyp++;
                s_since_hyp_ms = 0U;          /* R8b */
                if ((s_pending_ca_ms > 0U) && (g_apap_evt_ca > 0U))
                {
                    g_apap_evt_ca--;          /* R4: same event, now a Hyp */
                    s_pending_ca_ms = 0U;
                }
                s_hyp_hold_ms = (uint32_t)(clampf(g_apap_hyp_hold_s, 5.0f, 120.0f)
                                           * 1000.0f);
            }
        }
        else
        {
            s_hyp_streak = 0U;
            s_hyp_limited_ms = 0U;
            s_hyp_ev_n = 0U;         /* next event counts afresh; the verdict */
            s_hyp_ev_shape_n = 0U;   /* stays latched for this one's hold */
            /* R12: re-arm only once breathing is clearly back to normal. */
            const float rearm_ratio = clampf(g_apap_hyp_rearm_ratio, hyp_ratio, 1.0f);
            const uint8_t rearm_need = (g_apap_hyp_rearm_breaths < 1U) ? 1U
                                     : ((g_apap_hyp_rearm_breaths > 10U) ? 10U
                                                                         : g_apap_hyp_rearm_breaths);
            if (peak_slm >= (rearm_ratio * s_fl_baseline_slm))
            {
                if (s_hyp_rearm_n < 255U) { s_hyp_rearm_n++; }
            }
            else
            {
                s_hyp_rearm_n = 0U;
            }
            if (s_hyp_rearm_n >= rearm_need)
            {
                s_hyp_need_recovery = false;  /* breathing recovered: re-arm */
            }
            if (!s_hyp_active)
            {
                /* keep baseline updates via FL path when not limited */
            }
        }
    }

    g_apap_fl_peak_baseline_slm = s_fl_baseline_slm;
    g_apap_hyp_active = s_hyp_active ? 1U : 0U;
}

static void flSegStart(bool from_floor)
{
    s_fl_seg_active = true;
    s_fl_seg_from_floor = from_floor;
    s_fl_seg_confirmed = false;
    s_fl_seg_overflow = false;
    s_fl_seg_nbins = 0U;
    s_fl_seg_bin_sum = 0.0f;
    s_fl_seg_bin_n = 0U;
    s_fl_seg_bin_ms = 0U;
    s_fl_insp_peak_q = 0.0f;
    s_fl_insp_sum_q = 0.0f;
    s_fl_insp_n = 0U;
    s_insp_ms = 0U;
}

static void flSegAccumulate(float q, uint32_t dt_ms)
{
    if (q > s_fl_insp_peak_q)
    {
        s_fl_insp_peak_q = q;
    }
    s_insp_ms += dt_ms;
    s_fl_insp_sum_q += q;
    if (s_fl_insp_n < 100000U)
    {
        s_fl_insp_n++;
    }
    if (s_fl_seg_nbins < FL_SEG_MAX_BINS)
    {
        s_fl_seg_bin_sum += q;
        s_fl_seg_bin_n++;
        s_fl_seg_bin_ms += dt_ms;
        if (s_fl_seg_bin_ms >= FL_SEG_BIN_MS)
        {
            s_fl_seg_bins[s_fl_seg_nbins] = s_fl_seg_bin_sum / (float)s_fl_seg_bin_n;
            s_fl_seg_nbins++;
            s_fl_seg_bin_sum = 0.0f;
            s_fl_seg_bin_n = 0U;
            s_fl_seg_bin_ms = 0U;
        }
    }
    else
    {
        s_fl_seg_overflow = true;   /* > 4 s: not one inspiration */
    }
}

static void flSegFinish(void)
{
    /* Finding 2: the hypopnea's 10 s is wall-clock time, so pass the
     * breath PERIOD (end of inspiration to end of inspiration), not the
     * inspiratory time - that needed >= ~20 s of hypopnea at I:E 1:1
     * (bench: 50 % scored only at the event's end, 55 % never). The
     * first breath after a reset has no period yet (0 -> the old 3 s
     * fallback in finalizeInspBreath); a gap longer than the breath
     * window cannot be one breath, so it falls back too. */
    const uint32_t cap_ms = (uint32_t)(clampf(g_apap_breath_window_s,
                                              4.0f, 30.0f) * 1000.0f);
    uint32_t period_ms = s_since_insp_end_ms;
    if (period_ms > cap_ms) { period_ms = 0U; }
    finalizeInspBreath(s_fl_insp_peak_q, period_ms,
                       s_fl_seg_from_floor && !s_fl_seg_overflow);
    s_insp_ms = 0U;
    s_since_insp_end_ms = 0U;
    s_fl_seg_active = false;
}

static void updateFlHypDetector(uint32_t dt_ms)
{
    const EprPhase_t ph = Epr_GetPhase();
    const float q = Flow_GetPatientSlm()
                  - ((g_apap_rel_baseline_enable != 0U) ? Epr_GetFlowBaselineSlm()
                                                       : 0.0f);   /* R9 */
    const float floor_slm = clampf(g_apap_fl_seg_floor_slm, 0.0f, 5.0f);
    const bool onset = (ph == EPR_PHASE_INSPIRATION)
                    && (s_fl_prev_phase != EPR_PHASE_INSPIRATION);

    /* R13: a breath is the flow segment above the floor that contains an
     * EPR inspiration onset; it is judged when the flow returns to the
     * floor, so peak, flatness and dip see the whole inspiration. A positive
     * segment without an onset is noise, not a breath. */
    if (!s_fl_seg_active && s_fl_seg_armed && (q > floor_slm))
    {
        flSegStart(true);
    }
    if (onset)
    {
        if (s_fl_seg_active && s_fl_seg_confirmed)
        {
            /* The previous inspiration never returned to the floor (a flow
             * offset): close it here on its peak alone, and take this one
             * from its onset, as before R13. */
            flSegFinish();
        }
        if (!s_fl_seg_active)
        {
            flSegStart(false);
        }
        s_fl_seg_confirmed = true;
    }
    if (s_fl_seg_active)
    {
        flSegAccumulate(q, dt_ms);
    }
    if (q <= floor_slm)
    {
        if (s_fl_seg_active && s_fl_seg_confirmed)
        {
            flSegFinish();
        }
        s_fl_seg_active = false;
        s_fl_seg_armed = true;
    }
    if (s_since_insp_end_ms < 600000U) { s_since_insp_end_ms += dt_ms; }

    s_fl_prev_phase = ph;

    /* Hypopnea hold timer (raise window after event). */
    if (s_hyp_active)
    {
        if (s_hyp_hold_ms > dt_ms)
        {
            s_hyp_hold_ms -= dt_ms;
        }
        else
        {
            s_hyp_hold_ms = 0U;
            s_hyp_active = false;
            s_hyp_streak = 0U;
            s_hyp_limited_ms = 0U;
        }
        g_apap_hyp_active = s_hyp_active ? 1U : 0U;
    }
}

static void updateSnoreDetector(float dt_s, uint32_t dt_ms)
{
    if (g_apap_snore_enable == 0U)
    {
        s_snore_active = false;
        g_apap_snore_active = 0U;
        g_apap_snore_rms_slm = 0.0f;
        return;
    }

    const float q_raw = Flow_GetPatientRawSlm();
    const float fc = clampf(g_apap_snore_hp_hz, 5.0f, 80.0f);
    /* First-order HP: y += a*(y + x - x_prev) with a = 1/(1+2*pi*fc*dt)
     * approx via residual of raw vs breath LPF when dt small. */
    const float a = clampf((2.0f * 3.14159265f * fc * dt_s)
                               / (1.0f + 2.0f * 3.14159265f * fc * dt_s),
                           0.01f, 0.95f);
    s_snore_hp = a * (s_snore_hp + q_raw - s_snore_prev_q);
    s_snore_prev_q = q_raw;

    /* Also blend residual vs breath-band patient flow (snore = HF). */
    const float resid = q_raw - Flow_GetPatientSlm();
    const float hf = 0.5f * (s_snore_hp + resid);

    const float alpha_rms = clampf(dt_s / 0.20f, 0.01f, 0.5f); /* ~200 ms */
    s_snore_rms2 = ((1.0f - alpha_rms) * s_snore_rms2) + (alpha_rms * hf * hf);
    const float rms = sqrtf(s_snore_rms2);
    g_apap_snore_rms_slm = rms;

    const float thr = clampf(g_apap_snore_rms_thresh_slm, 0.05f, 5.0f);
    const uint32_t on_need =
        (uint32_t)(clampf(g_apap_snore_on_s, 0.3f, 10.0f) * 1000.0f);
    const uint32_t off_need =
        (uint32_t)(clampf(g_apap_snore_off_s, 0.3f, 15.0f) * 1000.0f);

    /* Prefer snore during inspiration (effort + vibration). */
    const bool insp = (Epr_GetPhase() == EPR_PHASE_INSPIRATION);
    if (insp && (rms >= thr))
    {
        s_snore_on_ms += dt_ms;
        s_snore_off_ms = 0U;
        if (s_snore_on_ms >= on_need)
        {
            s_snore_active = true;
        }
    }
    else
    {
        s_snore_off_ms += dt_ms;
        s_snore_on_ms = 0U;
        if (s_snore_off_ms >= off_need)
        {
            s_snore_active = false;
        }
    }

    g_apap_snore_active = s_snore_active ? 1U : 0U;
    noteSnoreEdge();
}

static void applyMonitorTitration(float dt_s, uint32_t dt_ms)
{
    g_apap_lowering = 0U;
    (void)raiseAllowed(); /* refresh leak flag */

    /* Priority: obstructive hypopnea > FL > snore > central-hypopnea hold >
     * stable lower. */
    const bool hyp_on = (g_apap_hyp_enable != 0U) && s_hyp_active;
    if (hyp_on && s_hyp_obstructive)
    {
        applyRaise(g_apap_hyp_raise_scale, dt_s);
        return;
    }
    if ((g_apap_fl_enable != 0U) && s_fl_active)
    {
        float fl_scale = clampf(g_apap_fl_raise_scale, 0.05f, 1.0f);
        const float flat_thr = clampf(g_apap_fl_flat_ratio, 0.55f, 0.95f);
        if (s_fl_last_flat >= flat_thr)
        {
            const float sev = clampf((s_fl_last_flat - flat_thr) / FL_SEV_SPAN,
                                     0.25f, 1.0f);
            fl_scale *= sev;
        }
        else if (s_fl_last_dip < flDipThreshold())
        {
            /* R13: M-type notch. */
            const float sev = clampf((flDipThreshold() - s_fl_last_dip) / FL_SEV_SPAN,
                                     0.25f, 1.0f);
            fl_scale *= sev;
        }
        else if ((s_fl_baseline_slm > 0.5f) && (g_apap_fl_last_peak_slm > 0.0f))
        {
            const float rel = g_apap_fl_last_peak_slm / s_fl_baseline_slm;
            const float fl_r = clampf(g_apap_fl_ratio, 0.40f, 0.95f);
            const float sev = clampf((fl_r - rel) / fl_r, 0.25f, 1.0f);
            fl_scale *= sev;
        }
        applyRaise(fl_scale, dt_s);
        return;
    }
    if ((g_apap_snore_enable != 0U) && s_snore_active)
    {
        const float thr = clampf(g_apap_snore_rms_thresh_slm, 0.05f, 5.0f);
        const float rms = g_apap_snore_rms_slm;
        const float sev = (thr > 0.05f)
                              ? clampf((rms - thr) / thr, 0.25f, 1.0f)
                              : 1.0f;
        applyRaise(g_apap_snore_raise_scale * sev, dt_s);
        return;
    }
    if (hyp_on)
    {
        /* Central (round) hypopnea: scored, but no raise - and no lowering
         * through it either; the wait starts again once it is over. */
        g_apap_stable_ms = 0U;
        return;
    }

    g_apap_stable_ms += dt_ms;
    if (g_apap_lower_enable == 0U)
    {
        return;
    }

    const uint32_t stable_need_ms =
        (uint32_t)(clampf(g_apap_stable_before_lower_s, 5.0f, 600.0f) * 1000.0f);
    if (g_apap_stable_ms < stable_need_ms)
    {
        return;
    }

    const float rate = clampf(g_apap_lower_rate_cmh2o_per_s, 0.001f, 0.05f);
    g_apap_target_cmh2o -= rate * dt_s;
    g_apap_lowering = 1U;
}

void Apap_Init(void)
{
    /* Must match the initialisers above (2026-10-12 findings 1/3) - this
     * runs at boot and would otherwise silently restore the old 4/12 s. */
    g_apap_no_breath_timeout_s = 2.0f;
    g_apap_breath_window_s = 8.0f;
    g_apap_resume_min_peak_slm = 10.0f;
    g_apap_resume_peak_ratio = 0.30f;
    g_apap_resume_min_floor_slm = 4.0f;
    g_apap_startup_grace_s = 15.0f;
    g_apap_osc_amplitude_cmh2o = 1.0f;
    g_apap_osc_amplitude_max_cmh2o = 1.75f;
    g_apap_osc_freq_hz = 4.0f;
    g_apap_fot_eval_period_s = 2.0f;
    g_apap_osc_min_p_pp_cmh2o = 0.15f;
    g_apap_osa_flow_pp_thresh_slm = 2.0f;
    g_apap_apnea_min_s = 10.0f;
    g_apap_oa_raise_cmh2o_per_s = 0.08f;
    g_apap_oa_raise_max_cmh2o = 2.0f;
    g_apap_her_oa_above_cmh2o = 12.0f;
    g_apap_her_oa_above_scale = 0.50f;
    g_apap_raise_rate_standard_cmh2o_per_s = 0.050f;
    g_apap_raise_rate_soft_cmh2o_per_s = 0.020f;
    g_apap_raise_rate_her_scale = 0.75f;
    g_apap_response_soft = 0U;
    g_apap_fl_enable = 1U;
    g_apap_fl_ratio = 0.70f;
    g_apap_fl_breaths_needed = 3U;
    g_apap_fl_clear_breaths = 2U;
    g_apap_fl_raise_scale = 1.00f;
    g_apap_fl_min_peak_slm = 4.0f;
    g_apap_fl_baseline_alpha = 0.25f;
    g_apap_fl_flat_ratio = 0.72f;     /* R13 */
    g_apap_fl_dip_ratio = 0.93f;
    g_apap_fl_seg_floor_slm = 0.0f;
    g_apap_fl_baseline_reject_ratio = 1.40f;
    g_apap_baseline_restore_max_s = 120.0f;
    g_apap_ca_reclass_window_s = 30.0f;
    g_apap_leak_freeze_min_breaths = 5U;
    g_apap_leak_freeze_max_s = 60.0f;
    g_apap_breath_notch_enable = 1U;
    g_apap_hyp_ca_guard_s = 30.0f;
    g_apap_rel_baseline_enable = 0U;   /* R10 */
    g_apap_hyp_enable = 1U;
    g_apap_hyp_ratio = 0.50f;
    g_apap_hyp_breaths_needed = 2U;
    g_apap_hyp_min_duration_s = 10.0f;
    g_apap_hyp_raise_scale = 0.70f;
    g_apap_hyp_hold_s = 20.0f;
    g_apap_hyp_rearm_ratio = 0.70f;   /* R12 */
    g_apap_hyp_rearm_breaths = 2U;
    g_apap_fl_amp_arm_enable = 0U;      /* FL on shape only */
    g_apap_hyp_raise_shape_frac = 0.50f;
    g_apap_snore_enable = 1U;
    g_apap_snore_rms_thresh_slm = 0.35f;
    g_apap_snore_on_s = 1.5f;
    g_apap_snore_off_s = 2.0f;
    g_apap_snore_raise_scale = 0.35f;
    g_apap_snore_hp_hz = 20.0f;
    g_apap_lower_enable = 1U;
    g_apap_lower_rate_cmh2o_per_s = 0.005f;
    g_apap_stable_before_lower_s = 20.0f;
    g_apap_leak_gate_enable = 1U;

    s_active = false;
    s_for_her = false;
    s_osa_treating = false;
    s_osa_event_armed = true;
    s_ca_event_armed = true;
    s_running_enter_ms = 0U;
    s_episode_ms = 0U;
    s_oa_raise_applied = 0.0f;
    g_apap_target_cmh2o = 8.0f;
    g_apap_no_breath_ms = 0U;
    g_apap_osa_treating = 0U;
    g_apap_fot_p_pp_cmh2o = 0.0f;
    g_apap_fot_q_pp_slm = 0.0f;
    g_apap_fot_z_cmh2o_per_slm = 0.0f;
    g_apap_fot_osa_likely = 0U;
    g_apap_stable_ms = 0U;
    g_apap_lowering = 0U;
    g_apap_raise_blocked_leak = 0U;
    Flow_FotBpReset(&s_fot_bp_p);
    Flow_FotBpReset(&s_fot_bp_q);
    s_fot_p_bp = 0.0f;
    s_fot_q_bp = 0.0f;
    resetBreathDetectors();
    resetSessionEvents();
    setState(APAP_STATE_IDLE);
    disableOsc();
}

void Apap_SetResponseSoft(bool soft)
{
    g_apap_response_soft = soft ? 1U : 0U;
}

bool Apap_IsResponseSoft(void)
{
    return (g_apap_response_soft != 0U);
}

void Apap_OnTherapyStart(float min_cmh2o, float max_cmh2o, uint8_t therapy_mode)
{
    if (!autosetMode(therapy_mode))
    {
        s_active = false;
        setState(APAP_STATE_IDLE);
        disableOsc();
        return;
    }

    s_active = true;
    s_for_her = (therapy_mode == (uint8_t)THERAPY_MODE_AUTOSET_FOR_HER);
    if (s_for_her)
    {
        /* AirSense for Her: gentler rises + slightly more FL-sensitive. */
        g_apap_response_soft = 1U;
        g_apap_fl_breaths_needed = 2U;
        g_apap_fl_raise_scale = 1.00f;  /* gentler via response_soft + her scale */
        g_apap_fl_flat_ratio = 0.70f;   /* R13: 0.72 - 0.02, as 0.78 - 0.04 was */
    }
    s_min_cmh2o = clampf(min_cmh2o, THERAPY_PRESS_MIN_CMH2O, THERAPY_PRESS_MAX_CMH2O);
    s_max_cmh2o = clampf(max_cmh2o, s_min_cmh2o + 0.5f, THERAPY_PRESS_MAX_CMH2O);
    if (s_max_cmh2o < s_min_cmh2o)
    {
        s_max_cmh2o = s_min_cmh2o;
    }

    g_apap_target_cmh2o = s_min_cmh2o;
    g_apap_no_breath_ms = 0U;
    g_apap_last_osa_detected = 0U;
    g_apap_osa_treating = 0U;
    g_apap_stable_ms = 0U;
    g_apap_lowering = 0U;
    g_apap_raise_blocked_leak = 0U;
    s_osa_treating = false;
    s_osa_event_armed = true;
    s_ca_event_armed = true;
    s_running_enter_ms = HAL_GetTick();
    s_episode_ms = 0U;
    s_oa_raise_applied = 0.0f;
    s_resume_ref_slm = 0.0f;
    s_episode_reached_fot = false;
    s_apnea_need_recovery = false;
    s_episode_scored_ca = false;
    s_pending_ca_ms = 0U;
    s_breaths_seen = 0U;
    s_since_hyp_ms = 0xFFFFFFFFU;
    s_episode_after_hyp = false;
    s_deferred_ca = false;
    s_deferred_oa = false;
    releaseLeakFreeze();
    Flow_FotBpReset(&s_breath_notch_bp);
    resetBreathDetectors();
    resetSessionEvents();
    disableOsc();
    setState(APAP_STATE_MONITOR);
}

void Apap_OnTherapyStop(void)
{
    s_active = false;
    s_osa_treating = false;
    g_apap_osa_treating = 0U;
    g_apap_no_breath_ms = 0U;
    g_apap_last_osa_detected = 0U;
    g_apap_stable_ms = 0U;
    g_apap_lowering = 0U;
    s_episode_ms = 0U;
    s_oa_raise_applied = 0.0f;
    Flow_FotBpReset(&s_fot_bp_p);
    Flow_FotBpReset(&s_fot_bp_q);
    s_fot_p_bp = 0.0f;
    s_fot_q_bp = 0.0f;
    releaseLeakFreeze();          /* R5b: never leave it set behind */
    resetBreathDetectors();
    setState(APAP_STATE_IDLE);
    disableOsc();
}

void Apap_OnMaskReconnect(void)
{
    if (!s_active)
    {
        return;
    }
    exitProbeToMonitor();
}

float Apap_GetTargetPressure(void)
{
    return g_apap_target_cmh2o;
}

ApapState_t Apap_GetState(void)
{
    return (ApapState_t)g_apap_state;
}

const char* Apap_GetStateName(void)
{
    switch ((ApapState_t)g_apap_state)
    {
    case APAP_STATE_MONITOR:
        if (s_hyp_active) { return "Monitor+Hyp"; }
        if (s_fl_active)  { return "Monitor+FL"; }
        if (s_snore_active) { return "Monitor+Snore"; }
        if (g_apap_lowering != 0U) { return "Monitor+Lower"; }
        return "Monitor";
    case APAP_STATE_NO_BREATH:  return "No breath";
    case APAP_STATE_OSCILLATE:
        return s_osa_treating ? "FOT+Raise" : "FOT probe";
    case APAP_STATE_RESPOND:    return "Raise P";
    case APAP_STATE_IDLE:
    default:                    return "Idle";
    }
}

bool Apap_IsOscillating(void)
{
    return ((ApapState_t)g_apap_state == APAP_STATE_OSCILLATE)
        && BlowerOsc_IsEnabled();
}

float Apap_GetFotPressurePpCmh2o(void)     { return g_apap_fot_p_pp_cmh2o; }
float Apap_GetFotFlowPpSlm(void)           { return g_apap_fot_q_pp_slm; }
float Apap_GetFotImpedanceCmh2oPerSlm(void) { return g_apap_fot_z_cmh2o_per_slm; }
float Apap_GetOsaFlowPpThreshSlm(void)     { return g_apap_osa_flow_pp_thresh_slm; }
bool  Apap_IsOsaLikely(void)               { return (g_apap_fot_osa_likely != 0U); }
bool  Apap_IsOsaTreating(void)             { return (g_apap_osa_treating != 0U); }
bool  Apap_IsFlowLimited(void)             { return (g_apap_fl_active != 0U); }
bool  Apap_IsHypopneaActive(void)          { return (g_apap_hyp_active != 0U); }
bool  Apap_IsSnoreActive(void)             { return (g_apap_snore_active != 0U); }
bool  Apap_IsRaiseBlockedByLeak(void)      { return (g_apap_raise_blocked_leak != 0U); }
float Apap_GetFlPeakBaselineSlm(void)      { return g_apap_fl_peak_baseline_slm; }
float Apap_GetFlLastPeakSlm(void)          { return g_apap_fl_last_peak_slm; }
float Apap_GetFlLastFlat(void)             { return g_apap_fl_last_flat; }
float Apap_GetFlLastDip(void)              { return g_apap_fl_last_dip; }
float Apap_GetSnoreRmsSlm(void)            { return g_apap_snore_rms_slm; }
uint32_t Apap_GetStableMs(void)            { return g_apap_stable_ms; }
bool  Apap_IsLowering(void)                { return (g_apap_lowering != 0U); }

uint32_t Apap_GetEventOaCount(void)        { return g_apap_evt_oa; }
uint32_t Apap_GetEventCaCount(void)        { return g_apap_evt_ca; }
uint32_t Apap_GetEventHypCount(void)       { return g_apap_evt_hyp; }
uint32_t Apap_GetEventFlCount(void)        { return g_apap_evt_fl; }
uint32_t Apap_GetEventSnoreCount(void)     { return g_apap_evt_snore; }
uint32_t Apap_GetTherapyMs(void)           { return g_apap_therapy_ms; }

float Apap_GetSessionAhi(void)
{
    const float hrs = sessionHours();
    if (!(hrs > 0.0f)) { return 0.0f; }
    /* Finding 5: AHI counts ALL apneas (it read 0.00 with CA 3 when it was
     * (OA + Hyp) / h). */
    return ((float)(g_apap_evt_oa + g_apap_evt_ca + g_apap_evt_hyp)) / hrs;
}

float Apap_GetSessionRdi(void)
{
    const float hrs = sessionHours();
    if (!(hrs > 0.0f)) { return 0.0f; }
    return ((float)(g_apap_evt_oa + g_apap_evt_ca + g_apap_evt_hyp
                    + g_apap_evt_fl)) / hrs;
}

float Apap_GetSessionSnoreIndex(void)
{
    const float hrs = sessionHours();
    if (!(hrs > 0.0f)) { return 0.0f; }
    return ((float)g_apap_evt_snore) / hrs;
}

int Apap_FormatDiag(char *buf, unsigned len)
{
    /* R11: what decides whether a breath ends an episode, in one place. */
    if ((buf == NULL) || (len == 0U))
    {
        return 0;
    }
    EprDiag_t e;
    Epr_GetDiag(&e);
    const bool notch_on = (g_apap_breath_notch_enable != 0U) && BlowerOsc_IsEnabled();
    const int n = snprintf(buf, len,
        "# apap diag st=%u ep_ms=%lu recent=%u recov=%u need=%.1f ref=%.1f "
        "flbase=%.1f seen=%lu lkfrz=%u apnea_nr=%u hyp_nr=%u rearm=%u "
        "notch=%u f0=%.1f\r\n"
        "# epr diag cand_ms=%lu cand_ml=%.0f cand_pk=%.1f stamped=%u rej=%lu "
        "med_ml=%.0f since_ms=%ld stamp_pk=%.1f base=%.2f fsm=%u pol=%d\r\n",
        (unsigned)g_apap_state, (unsigned long)s_episode_ms,
        (unsigned)(recentBreath() ? 1U : 0U), (unsigned)(recoveryBreath() ? 1U : 0U),
        (double)recoveryNeedSlm(), (double)s_resume_ref_slm,
        (double)s_fl_baseline_slm, (unsigned long)s_breaths_seen,
        (unsigned)(s_leak_frozen ? 1U : 0U), (unsigned)(s_apnea_need_recovery ? 1U : 0U),
        (unsigned)(s_hyp_need_recovery ? 1U : 0U), (unsigned)s_hyp_rearm_n,
        (unsigned)(notch_on ? 1U : 0U), (double)BlowerOsc_GetFrequency(),
        (unsigned long)e.cand_ms, (double)e.cand_ml, (double)e.cand_peak_slm,
        (unsigned)e.cand_stamped, (unsigned long)e.rejects, (double)e.vol_median_ml,
        (e.since_stamp_ms == 0xFFFFFFFFU) ? -1L : (long)e.since_stamp_ms,
        (double)e.stamp_peak_slm, (double)e.flow_baseline_slm,
        (unsigned)e.fsm, (int)e.polarity);
    return ((n > 0) && ((unsigned)n < len)) ? n : 0;
}

float Apap_FilterBreathFlow(float q_slm, float dt_s)
{
    /* R7b: run the band-pass every tick so switching the notch in or out
     * is transient-free; notch = input - band-pass (RBJ, unity at f0). */
    const float f0 = clampf(BlowerOsc_GetFrequency(), 2.0f, 8.0f);
    const float bp = Flow_FotBpStep(&s_breath_notch_bp, q_slm, f0, dt_s);
    if ((g_apap_breath_notch_enable != 0U) && BlowerOsc_IsEnabled())
    {
        return q_slm - bp;
    }
    return q_slm;
}

float Apap_Update(float dt_s, uint8_t therapy_state, bool mask_off)
{
    if (!s_active
        || mask_off
        || (therapy_state != (uint8_t)THERAPY_STATE_RUNNING))
    {
        if (mask_off && s_active)
        {
            disableOsc();
            s_osa_treating = false;
            g_apap_osa_treating = 0U;
            g_apap_lowering = 0U;
        }
        return g_apap_target_cmh2o;
    }

    if (!(dt_s > 0.0f) || dt_s > 0.1f)
    {
        return g_apap_target_cmh2o;
    }

    const uint32_t dt_ms = (uint32_t)lrintf(dt_s * 1000.0f);
    g_apap_therapy_ms += dt_ms;
    s_pending_ca_ms = (s_pending_ca_ms > dt_ms) ? (s_pending_ca_ms - dt_ms) : 0U;
    if (s_since_hyp_ms < 0x7FFFFFFFU) { s_since_hyp_ms += dt_ms; }
    if (s_leak_frozen)
    {
        s_leak_freeze_ms += dt_ms;
        const float cap_s = clampf(g_apap_leak_freeze_max_s, 0.0f, 600.0f);
        if ((cap_s > 0.0f) && (s_leak_freeze_ms >= (uint32_t)(cap_s * 1000.0f)))
        {
            releaseLeakFreeze();   /* R7a: a bad freeze cannot hold */
        }
    }

    if (startupGraceActive())
    {
        disableOsc();
        s_osa_treating = false;
        g_apap_osa_treating = 0U;
        g_apap_lowering = 0U;
        setState(APAP_STATE_MONITOR);
        return g_apap_target_cmh2o;
    }

    /* Recovery-sized breath ended the apnea / FOT episode. */
    if (recoveryBreath()
        && ((ApapState_t)g_apap_state == APAP_STATE_OSCILLATE
            || (ApapState_t)g_apap_state == APAP_STATE_NO_BREATH))
    {
        exitProbeToMonitor();
    }

    switch ((ApapState_t)g_apap_state)
    {
    case APAP_STATE_MONITOR:
        disableOsc();
        s_osa_treating = false;
        g_apap_osa_treating = 0U;
        if (!recentBreath())
        {
            /* Keep the breath-size reference for the recovery test before
             * the detectors (and their baseline) are reset - finding 3. */
            s_resume_ref_slm = s_fl_baseline_slm;
            s_save_fl_streak = s_fl_streak;
            s_save_fl_good_streak = s_fl_good_streak;
            s_save_fl_active = s_fl_active;
            s_save_hyp_streak = s_hyp_streak;
            s_save_hyp_limited_ms = s_hyp_limited_ms;
            s_save_hyp_active = s_hyp_active;
            s_save_hyp_hold_ms = s_hyp_hold_ms;
            s_save_hyp_need_recovery = s_hyp_need_recovery;
            s_save_hyp_ev_n = s_hyp_ev_n;
            s_save_hyp_ev_shape_n = s_hyp_ev_shape_n;
            s_save_hyp_obstructive = s_hyp_obstructive;
            s_episode_reached_fot = false;
            s_episode_scored_ca = false;
            {
                /* R8b: does this episode continue a hypopnea? */
                const uint32_t guard_ms = (uint32_t)(clampf(g_apap_hyp_ca_guard_s,
                                                            0.0f, 120.0f) * 1000.0f);
                s_episode_after_hyp = (guard_ms > 0U)
                    && (s_hyp_streak > 0U)
                    && (s_hyp_active || (s_since_hyp_ms < guard_ms));
                s_deferred_ca = false;
                s_deferred_oa = false;
            }
            /* R5b, gated by R7a: freeze only an estimate breathing seeded. */
            if (s_breaths_seen >= (uint32_t)g_apap_leak_freeze_min_breaths)
            {
                Leak_SetApneaFreeze(true);
                s_leak_frozen = true;
                s_leak_freeze_ms = 0U;
            }
            resetBreathDetectors();
            g_apap_stable_ms = 0U;
            g_apap_lowering = 0U;
            setState(APAP_STATE_NO_BREATH);
            g_apap_no_breath_ms = 0U;
            /* Finding 1: the episode began at the last qualified breath, a
             * full window ago - not now. */
            s_episode_ms = (uint32_t)(clampf(g_apap_breath_window_s,
                                             4.0f, 30.0f) * 1000.0f);
            s_oa_raise_applied = 0.0f;
            break;
        }
        updateFlHypDetector(dt_ms);
        updateSnoreDetector(dt_s, dt_ms);
        applyMonitorTitration(dt_s, dt_ms);
        break;

    case APAP_STATE_NO_BREATH:
        disableOsc();
        g_apap_no_breath_ms += dt_ms;
        s_episode_ms += dt_ms;
        g_apap_stable_ms = 0U;
        g_apap_lowering = 0U;
        if (recoveryBreath())
        {
            exitProbeToMonitor();
            break;
        }
        {
            const uint32_t timeout_ms =
                (uint32_t)(clampf(g_apap_no_breath_timeout_s, 2.0f, 30.0f)
                           * 1000.0f);
            if (g_apap_no_breath_ms >= timeout_ms)
            {
                enterFotProbe();
            }
        }
        break;

    case APAP_STATE_OSCILLATE:
        configureOsc();
        trackOscStats(dt_s);
        refreshFotMetrics();
        g_apap_osc_eval_ms += dt_ms;
        s_episode_ms += dt_ms;
        g_apap_stable_ms = 0U;
        g_apap_lowering = 0U;

        if (recoveryBreath())
        {
            exitProbeToMonitor();
            break;
        }

        {
            const uint32_t eval_ms =
                (uint32_t)(clampf(g_apap_fot_eval_period_s, 0.5f, 10.0f)
                           * 1000.0f);
            const uint32_t apnea_min_ms =
                (uint32_t)(clampf(g_apap_apnea_min_s, 6.0f, 30.0f) * 1000.0f);
            if (g_apap_osc_eval_ms >= eval_ms)
            {
                const bool osa = evaluateOscForOsa();
                g_apap_last_osa_detected = osa ? 1U : 0U;
                s_osa_treating = osa;
                g_apap_osa_treating = osa ? 1U : 0U;

                if ((s_episode_ms >= apnea_min_ms)
                    && osa && s_osa_event_armed && !s_apnea_need_recovery)
                {
                    if (s_episode_after_hyp) { s_deferred_oa = true; }   /* R8b */
                    else
                    {
                        g_apap_evt_oa++;
                        s_apnea_need_recovery = true;
                    }
                    s_osa_event_armed = false;
                    s_ca_event_armed = false;
                }
                else if ((s_episode_ms >= apnea_min_ms)
                         && !osa && s_ca_event_armed && !s_apnea_need_recovery
                         && (g_apap_fot_p_pp_cmh2o
                             >= clampf(g_apap_osc_min_p_pp_cmh2o, 0.05f, 0.8f)))
                {
                    /* Adequate FOT pressure with open-airway flow → central. */
                    if (s_episode_after_hyp) { s_deferred_ca = true; }   /* R8b */
                    else
                    {
                        g_apap_evt_ca++;
                        s_apnea_need_recovery = true;
                        s_episode_scored_ca = true;
                    }
                    s_ca_event_armed = false;
                }
                resetOscStats();
            }
        }

        applyOaDurationRaise();
        {
            const uint32_t apnea_min_ms =
                (uint32_t)(clampf(g_apap_apnea_min_s, 6.0f, 30.0f) * 1000.0f);
            if ((s_episode_ms >= apnea_min_ms)
                && s_osa_treating && s_osa_event_armed && !s_apnea_need_recovery)
            {
                if (s_episode_after_hyp) { s_deferred_oa = true; }   /* R8b */
                else
                {
                    g_apap_evt_oa++;
                    s_apnea_need_recovery = true;
                }
                s_osa_event_armed = false;
                s_ca_event_armed = false;
            }
        }
        break;

    case APAP_STATE_RESPOND:
        enterFotProbe();
        break;

    case APAP_STATE_IDLE:
    default:
        disableOsc();
        s_osa_treating = false;
        g_apap_osa_treating = 0U;
        g_apap_lowering = 0U;
        break;
    }

    g_apap_target_cmh2o = clampf(g_apap_target_cmh2o, s_min_cmh2o, s_max_cmh2o);
    return g_apap_target_cmh2o;
}
