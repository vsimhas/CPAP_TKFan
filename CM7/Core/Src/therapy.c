/**
  ******************************************************************************
  * @file    therapy.c
  * @brief   CPAP therapy state machine.
  *
  *  See therapy.h for the architecture overview. This file owns the
  *  state transitions, ramp interpolator, run-time counter and the
  *  glue calls into blower_ctrl / blower_ipc.
  ******************************************************************************
  */

#include "therapy.h"
#include "blower_ctrl.h"
#include "blower_ipc.h"
#include "leak_estimator.h"
#include "flow_sensor.h"
#include "epr_ctrl.h"
#include "apap_ctrl.h"
#include "session_edf_log.h"
#include "session_summary.h"
#include "safety_supervisor.h"
#include "telemetry_stream.h"   /* Telem_PushMaskOff - see enterMaskOff() */
#include "dbg_log.h"
#include "stm32h7xx_hal.h"

#include <math.h>
#include <stdint.h>

/* === Compile-time defaults === */
#define THERAPY_RAMP_START_CMH2O    4.0f         /* AirSense-style start    */
#define THERAPY_TARGET_DEFAULT      8.0f         /* sensible adult default  */
#define THERAPY_PRESS_MIN_CMH2O     4.0f
#define THERAPY_PRESS_MAX_CMH2O     25.0f
#define THERAPY_RAMP_MAX_MINUTES    45U
#define THERAPY_STOP_DURATION_S     2.0f         /* spin-down hold time     */

/* Safety tunables live in safety_supervisor.c (g_safety_*). */

/* === Public configuration (debugger-writable volatiles) === */
volatile float    g_therapy_target_cmh2o   = THERAPY_TARGET_DEFAULT;
volatile float    g_therapy_apap_min_cmh2o = 5.0f;
volatile float    g_therapy_apap_max_cmh2o = 20.0f;
volatile uint8_t  g_therapy_ramp_minutes   = 0U;
volatile uint8_t  g_therapy_mode           = (uint8_t)THERAPY_MODE_CPAP;

/* Mask-off (high-leak / disconnect) during RAMP/RUNNING.
 *
 * Uses RAW orifice total flow Flow_GetSlm() — NOT patient flow and NOT
 * Q_total−Q_vent "excess" (those wait on leak/vent estimates and false-clear
 * while the blower is still ramping). Enter needs high |Q_total| + pressure
 * collapse; clear needs sustained low |Q_total| vs the open-hose peak. */
volatile uint8_t  g_therapy_mask_off_enabled     = 1U;
volatile int32_t  g_therapy_mask_off_rpm         = 6000;  /* hold for reconnect sense */
/* One-shot soft decel (brake on CM4). Do not re-post while still above hold —
 * that restarts the FOC ramp and makes the drop look endless. */
volatile uint32_t g_therapy_mask_off_ramp_ms     = 1000U;
/* Fixed |Q_total| enter threshold. Kept as a FLOOR under the vent-referenced
 * threshold below, so nothing gets less sensitive at low setpoints than it
 * was before 2026-09-23. */
volatile float    g_therapy_mask_off_enter_slm   = 45.0f;  /* |Q_total| enter          */

/* Vent-referenced enter threshold.
 *
 * maskOffTotalFlowSlm() returns Flow_GetSlm() - the TOTAL device flow, vent
 * included, not patient flow. The vent flows vent_k*sqrt(P), so the share of
 * a fixed 45 slm threshold that the vent has already spent grows with the
 * setpoint, and the patient-flow headroom left over collapses:
 *
 *      setpoint   vent flow   headroom under a fixed 45
 *         4 cmH2O   17.4 slm      27.6 slm
 *         8         24.6          20.4
 *        12         30.1          14.9
 *        18         36.9           8.1
 *
 * At 18 cmH2O a perfectly ordinary inhale spends the entire budget. Measured
 * on the bench on 2026-09-23 with a human breathing at 18 cmH2O: mask-off
 * false-fired three times in 75 s, with total flow at the trigger reaching
 * 75.5 and 66.5 slm against the 45 slm threshold. Each park dropped the
 * blower to 6000 rpm mid-inspiration and pressure collapsed to ~10 cmH2O.
 *
 * So the threshold is referenced to what the vent already accounts for:
 *
 *      enter = max(vent_flow(setpoint) + margin, enter_slm)
 *
 * which gives the same patient-flow headroom at every setpoint instead of a
 * budget that runs out as pressure rises. Set enter_vent_ref to 0 to get the
 * old fixed-threshold behaviour back. */
volatile uint8_t  g_therapy_mask_off_enter_vent_ref  = 1U;
volatile float    g_therapy_mask_off_enter_margin_slm = 30.0f;
volatile float    g_therapy_mask_off_enter_thresh_now = 0.0f; /* diagnostic */
volatile float    g_therapy_mask_off_clear_slm   = 40.0f;  /* optional flow assist   */
/* At hold RPM (~6 k), sealed tip is only ~0.5–0.8 cmH2O — use that rise vs
 * open-hose baseline as the primary mask-on detector (not a 2 cmH2O floor). */
volatile float    g_therapy_mask_off_reconnect_press_delta_cmh2o = 0.35f;
volatile float    g_therapy_mask_off_reconnect_abs_cmh2o = 0.55f;
volatile float    g_therapy_mask_off_reconnect_flow_max_slm = 55.0f;
volatile float    g_therapy_mask_off_reconnect_flow_margin_slm = 25.0f;
volatile float    g_therapy_mask_off_flow_drop_frac = 0.55f;
volatile uint32_t g_therapy_mask_off_baseline_ms = 600U;   /* learn @ hold RPM only */
volatile uint32_t g_therapy_mask_off_enter_ms    = 600U;   /* fast-path confirm       */
/* Tip confirm. 300 ms was far too eager: a hand briefly over the mask, or the
 * cushion touching the face on the way past, holds the tip for that long and
 * resumed full therapy pressure. Nothing distinguishes a momentary closure
 * from a mask being put on except how long it lasts, so duration is the only
 * honest lever here - a real reconnect holds the tip indefinitely. */
volatile uint32_t g_therapy_mask_off_clear_ms    = 1200U;  /* tip confirm            */
/* Probationary resume. Clearing is a guess; this checks it. After resuming we
 * give the circuit verify_ms to actually reach verify_frac of the therapy
 * target. If it does not, the clear was wrong - go straight back to the park
 * without waiting for an enter path to re-confirm, and make the next clear
 * harder so a repeated brush cannot keep pumping the blower up to speed. */
volatile uint32_t g_therapy_mask_off_verify_ms       = 4000U;
volatile float    g_therapy_mask_off_verify_frac     = 0.50f;
volatile uint32_t g_therapy_mask_off_clear_backoff_ms = 1000U;
volatile uint32_t g_therapy_mask_off_clear_ms_max    = 5000U;
volatile uint32_t g_therapy_mask_off_false_clear_count = 0U; /* diagnostic */
volatile uint32_t g_therapy_mask_off_min_dwell_ms = 400U;
/* Short: only debounce mask-on chatter. Safety uses a separate longer grace. */
volatile float    g_therapy_mask_off_regrace_s   = 2.5f;
/* Require therapy pressure collapse before declaring mask-off (blocks breath FPs). */
volatile uint8_t  g_therapy_mask_off_require_press = 1U;
volatile float    g_therapy_mask_off_press_drop_cmh2o = 1.2f; /* vs slewed setpoint */
volatile float    g_therapy_mask_off_press_abs_cmh2o  = 4.0f; /* absolute floor    */
/* Extra floor for the DEVICE-SIDE sensor in maskOffOpenCircuitPressure(),
 * which is what lets a clipped-flow / speed-limited disconnect through the
 * fast path. The sensor sits at the blower end of the hose, so with the mask
 * off it still reads the hose drop at open-circuit flow. See that function. */
volatile float    g_therapy_mask_off_press_sensor_allow_cmh2o = 2.5f;
/* Open-circuit evidence by FLOW, 2026-10-05, for the ~150 slm flow element.
 * The fast path refuses entry at the speed ceiling / on a saturated flow
 * sample unless maskOffOpenCircuitPressure() holds. With Round Hole_2 an open
 * circuit read 3.5-5 cmH2O at the device sensor (the element's large drop is
 * upstream of it), so that held. The ~150 slm element drops far less: with
 * the mask pulled at 20 cmH2O the device sensor read 14-16 and the mask
 * estimate 11-13 at 129-155 slm (2 real pulls, 3 bench uncaps), the gate
 * never held, and only the 2.5 s rpm-ratio backstop parked the blower - at
 * the 38.5 k ceiling into an open circuit, ~1.7 A, for the whole 2.5 s.
 * Sustained flow at or above this, with both pressures still short of the
 * setpoint (maskOffPressureCollapsed, the usual 600 ms confirm), is accepted
 * as that evidence instead. On the same pulls it confirms 0.70-0.94 s after
 * the pull. In ~100 s of 20 cmH2O breathing (peaks 113-119 slm total) flow
 * >= 100 slm with both pressures short never lasted more than 20 ms: the
 * inspiratory pressure dip comes at the onset, the flow peak after the
 * pressure has recovered. Round Hole_2 clips at ~89 slm and never reaches
 * it, so its behaviour is unchanged. 0 = off. Re-check the breathing margin
 * on more (and deeper) breaths. */
volatile float    g_therapy_mask_off_open_flow_slm = 110.0f;
volatile uint8_t  g_therapy_mask_off_diag_open_flow = 0U; /* 1 = flow stood in for open-circuit pressure */
/* During fast-arm window, min |Q| must stay elevated (open hose ≠ breath). */
volatile float    g_therapy_mask_off_enter_min_frac = 0.35f;
/* Inhibit fast enter briefly after a breath phase edge if pressure is OK.
 * Not applied when |Q| already looks like open hose. */
volatile uint8_t  g_therapy_mask_off_breath_inhibit_enable = 1U;
volatile uint32_t g_therapy_mask_off_breath_inhibit_ms     = 1500U;
/* Vivo-style disconnect: 10 s mean leak ratio > limit (MaskOn_Off_Check.c). */
volatile uint8_t  g_therapy_mask_off_ratio_enable      = 1U;
volatile int32_t  g_therapy_mask_off_ratio_10s_limit   = 2000;
volatile uint32_t g_therapy_mask_off_ratio_enter_ms    = 8000U;
volatile uint8_t  g_therapy_mask_off_enter_via_ratio   = 0U; /* diagnostic     */
/* Pressure-only backstop. Both paths above need a trustworthy total-flow
 * signal; with none, a disconnect is invisible to them and the PID simply
 * runs away to the RPM ceiling until the safety supervisor latches
 * LOW_PRESS_AT_HIGH_RPM and stops the motor. The same evidence the supervisor
 * acts on - high commanded RPM with the circuit pressure on the floor - parks
 * the blower instead, on a shorter timer so it always wins that race. */
volatile uint8_t  g_therapy_mask_off_press_only_enable = 1U;
volatile int32_t  g_therapy_mask_off_press_only_rpm    = 12000;
/* Threshold is a FRACTION of the pressure being asked for, capped by the
 * absolute ceiling below. An absolute floor on its own does not work: with the
 * mask off the face, the hose and the mask vent still develop a couple of
 * cmH2O at the RPM ceiling, so a 1.5 cmH2O floor is never reached and the
 * disconnect stays invisible until RPM_SATURATED stops the motor. Measured on
 * the bench: setpoint 8, mask off, blower pinned at 30 kRPM -> ~2.5 cmH2O. */
volatile float    g_therapy_mask_off_press_only_frac   = 0.55f;
volatile float    g_therapy_mask_off_press_only_cmh2o  = 4.0f;
volatile uint32_t g_therapy_mask_off_press_only_ms     = 2000U;
volatile float    g_therapy_mask_off_press_only_thresh = 0.0f; /* diagnostic  */
/* RPM-ratio backstop: the loop is asking for far more RPM than the calibrated
 * square-law model says this pressure should need, and the pressure is still
 * short of target. Scale-invariant - it does not care HOW far the pressure
 * fell - so it covers the bottom of the therapy range, where an open circuit
 * still holds nearly the set pressure and the pressure test cannot separate
 * mask-on from mask-off.
 *
 * Headroom is what makes it work, and headroom shrinks as the setpoint rises.
 * Against the 36 kRPM ceiling (g_safety_safe_rpm_max) and the steady-state
 * model as of 2026-09-22 (K*sqrt(P) + 1820 + 157*P):
 *
 *      P      model rpm    ceiling ratio
 *      4        14730          2.44
 *      8        20445          1.76
 *     12        24977          1.44
 *     16        28896          1.25
 *     20        32423          1.11
 *
 * At the 1.25 default this path covers about 4-16 cmH2O. An earlier version
 * of this comment claimed 4-19; that was computed against the old
 * constant-offset model and was never achievable.
 *
 * Do not lower the threshold to buy back the top end. The ratio there is
 * running out because a sealed circuit at 20 cmH2O already needs 32.4 k of
 * the 36 k ceiling, and no threshold fixes that - it is the same headroom
 * limit that makes CPAP itself degrade above ~13 cmH2O on this blower. The
 * pressure-only path covers 5 cmH2O and up anyway, so the top end is not
 * this path's job. What IS its job is the bottom of the range, below ~4.6
 * cmH2O, where the open circuit still holds nearly the set pressure and the
 * pressure test cannot separate mask-on from mask-off. There the available
 * ratio is 2.0-2.4 and the margin is comfortable.
 *
 * !! STALE AS OF 2026-09-24 - RE-MEASURE BEFORE RELYING ON IT. Everything in
 * this block was computed against the 2026-09-22 model and a 36 k ceiling.
 * Since then the ceiling is 40000 and the feed-forward has been recalibrated
 * twice; the zero-flow model was then 6535*sqrt(P) + 853 (blower_ctrl.c),
 * about 2.6% LOWER than the one above, and c_fan is 0.001634 so the loop now
 * commands more RPM during inspiration. Both push this ratio UP for the same
 * circuit. Recomputed ceiling ratios, 40000 / model:
 *      P=4 2.87 | 8 2.07 | 12 1.70 | 16 1.48 | 20 1.33
 * so at 1.25 the path now reaches ~22 cmH2O.
 *
 * !! 2026-10-05: RE-MEASURE THE MASK-OFF MARGINS. The ~150 slm flow element is
 * now fitted (flow_sensor.c) and the FF refitted with it: 6211*sqrt(P) + 685,
 * ~2 % HIGHER than 6056*sqrt(P) + 773 at 20 cmH2O, so every ratio below reads
 * ~2 % lower. The element also costs ~4 cmH2O less at ~115 slm, so a given
 * open-circuit flow needs fewer rpm (ratios lower again), and total flow is
 * no longer clipped at ~89 slm (it reads to ~160), so any flow-based test
 * sees larger numbers than when these margins were taken.
 *
 * 2026-10-02: FF refitted on the resealed air path, K 6535 -> 6056 and offset
 * 853 -> 773 (blower_ctrl.c). The zero-flow model is now 6056*sqrt(P) + 773,
 * ~7 % LOWER than 6535*sqrt(P) + 853 (P=4 13.9k -> 12.9k, 16 27.0k -> 25.0k),
 * and the therapy ceiling is 38500. Ceiling ratios 38500 / model:
 *      P=4 2.99 | 8 2.15 | 12 1.77 | 16 1.54 | 20 1.38
 * so at 1.25 the path reaches ~24 cmH2O. Every ratio here reads ~7 % higher
 * for the same circuit than with the old model; the resting ratio (vent only,
 * 16 cmH2O) is ~1.04. (A 2026-10-01 refit to 1541 + 94.3*P was made on a
 * leaking air path and never held for a sealed one.) The false-trigger margin below
 * is the part that CANNOT be recomputed - it was measured, and must be
 * measured again: longest continuous run above 1.25 during sealed-circuit
 * breathing, against the 2500 ms confirm.
 *
 * PARTIALLY RE-MEASURED 2026-09-24 on the joint-fit build, HUMAN breathing
 * (harder than the simulator runs below) on the normal circuit:
 *   18 cmH2O, median peak 58 slm: peak ratio 1.40; longest continuous run
 *      above 1.25 = 1220 ms (51% margin), above 1.20 = 1360, above 1.10 = 1620.
 *   20 cmH2O, median peak 47 slm: peak ratio 1.33 (= the 40000 ceiling, cmd
 *      was clipped there on most breaths); above 1.25 = 1200 ms (52% margin),
 *      above 1.20 = 1320, above 1.10 = 2600.
 * Better than the 30% below despite the higher ratio reference. Still to do:
 * 6/10/14 cmH2O - the low setpoints are where this path matters most.
 *
 * Margin against false positives, measured 2026-09-22 over 23688 samples of
 * simulator breathing on a SEALED circuit at 6/10/14/18 cmH2O: instantaneous
 * ratio reaches 2.18 on transients, but the longest CONTINUOUS run above 1.25
 * is 1740 ms against the 2500 ms confirm - and that figure includes the
 * settling periods after each pressure step, so steady operation is quieter
 * still. The pressure gate (both signals more than press_drop below setpoint)
 * has to hold for the same 2500 ms on top of that, and during normal
 * breathing it does not. Run lengths at other thresholds: 1.10 -> 2100 ms,
 * 1.15 -> 2040, 1.20 -> 1960. So 1.25 is not arbitrary; it is roughly 30%
 * margin, and 1.10 would leave 16%.
 *
 * A deep breath transiently raises the ratio too (feed-forward adds RPM for
 * flow), which is why the confirm time is long compared with the other paths:
 * an inspiration cannot hold both the ratio and a sustained pressure shortfall
 * for 2.5 s. The sustained shortfall is what actually rejects breaths; the
 * ratio is what rejects a circuit that is merely working hard.
 *
 * Sizing the default. The obvious mistake is to assume the loop saturates on
 * every disconnect - it does not, at the bottom of the range. At a 4 cmH2O
 * setpoint an open circuit still holds ~2.5 cmH2O, so the error is only
 * ~1.5 cmH2O: that is outside the integrator deadband (1.2) so I is frozen,
 * and the loop settles at roughly FF(~16.1 k) + P(1800 x 1.5 = 2.7 k) =
 * ~18.8 kRPM against a model estimate of 14.3 k - a ratio of about 1.32, not
 * the 2.1 the ceiling would allow. A 1.35 threshold would therefore miss the
 * exact case this path exists for. 1.25 clears it with margin and still
 * reaches up to ~13 cmH2O before the ceiling runs out of headroom. */
volatile uint8_t  g_therapy_mask_off_rpm_ratio_enable = 1U;
volatile float    g_therapy_mask_off_rpm_ratio        = 1.25f;
volatile uint32_t g_therapy_mask_off_rpm_ratio_ms     = 2500U;
volatile float    g_therapy_mask_off_rpm_ratio_now    = 0.0f; /* diagnostic  */
volatile float    g_therapy_mask_off_rpm_ratio_ss     = 0.0f; /* diagnostic  */
volatile uint8_t  g_therapy_mask_off_diag_rpm_ratio   = 0U;
volatile uint8_t  g_therapy_mask_off_enter_via_rpm    = 0U; /* diagnostic  */
/* Sustained-flow path, 2026-10-05. Every other enter path needs the
 * pressure to collapse below setpoint. With the ~150 slm flow element the
 * blower can hold a moderate setpoint with the mask OFF: at 12 cmH2O, mask
 * removed (bench, patient end open through the external sensor), it held the
 * mask estimate at 12.5 cmH2O with 157 slm flowing at 36.9 krpm, and no path
 * fired. This one reads only flow: the 3 s boxcar mean of total flow at or
 * above the vent reference + margin, held for sustained_ms. Over any ~3 s
 * window (about one breath) breathing averages roughly vent + mask leak -
 * the inspiratory and expiratory halves cancel; at 20 cmH2O the highest
 * 3 s mean in five breathing runs incl. deliberately deep breaths was 38-58
 * slm against a threshold of ~87. An open circuit averages its full open
 * flow (~150 slm at 12-20 cmH2O here). Also catches a sustained
 * unintentional leak above ~margin - by design a leak that large is treated
 * as mask off. Works with Round Hole_2 as well (clips at ~89 slm, above the
 * threshold at every setpoint). Not evaluated during the start-up / reconnect
 * grace (the circuit is filling); the window restarts after them. */
volatile uint8_t  g_therapy_mask_off_sustained_enable     = 1U;
volatile float    g_therapy_mask_off_sustained_margin_slm = 50.0f;
volatile uint32_t g_therapy_mask_off_sustained_ms         = 500U;
volatile float    g_therapy_mask_off_sustained_mean_slm   = 0.0f; /* diagnostic */
volatile uint8_t  g_therapy_mask_off_enter_via_sustained  = 0U;   /* diagnostic */
volatile uint8_t  g_therapy_mask_off_enter_via_press   = 0U; /* diagnostic     */
volatile uint8_t  g_therapy_mask_off_diag_press_only   = 0U;
/* Fallback clears. The pressure-tip test cannot fire when the mask went back
 * on before the decel finished - the open-circuit reference is then the sealed
 * hold pressure and nothing ever rises above it. A sustained flow collapse
 * clears on its own (longer confirm than the tip path), and the watchdog makes
 * sure a mask-off can never stay latched for the rest of the session. */
volatile uint32_t g_therapy_mask_off_flow_clear_ms = 1500U;
volatile uint32_t g_therapy_mask_off_max_hold_ms   = 60000U;
volatile uint8_t  g_therapy_mask_off_active      = 0U;
volatile uint32_t g_therapy_mask_off_enter_count = 0U; /* session mask-off events */
volatile uint32_t g_therapy_mask_on_count        = 0U; /* session reconnects     */
volatile uint8_t  g_therapy_mask_off_reconn_flow_ok  = 0U;
volatile uint8_t  g_therapy_mask_off_reconn_press_ok = 0U;
/* Enter diagnostics (1 = that gate currently satisfied). */
volatile uint8_t  g_therapy_mask_off_diag_excess  = 0U;
volatile uint8_t  g_therapy_mask_off_diag_min     = 0U;
volatile uint8_t  g_therapy_mask_off_diag_press   = 0U;
volatile uint8_t  g_therapy_mask_off_diag_breath  = 0U; /* 1 = breath inhibit active */
/* Fast-path rejection bits: 1 flow stale beyond flow_stale_ticks_max, never
 * received, or nonfinite; 2 clipped flow and 4 upper-limited PID, each
 * without open-circuit pressure (maskOffOpenCircuitPressure).
 * Other entry paths have independent evidence / longer confirmations. */
volatile uint8_t  g_therapy_mask_off_diag_fast_block = 0U;

/* How many CONSECUTIVE held flow ticks the fast path tolerates before it
 * calls the flow invalid.
 *
 * A single held read is routine, not a dead sensor: SensorTask reads the
 * AMS5935 every FS_CONV_TICKS (2 x 5 ms), and a read that lands on a busy
 * conversion is marked stale and simply retried on the next tick. In every
 * bench log of 2026-09-24 each stale spell was exactly one tick long, yet
 * stale showed up in 10-17 % of sampled ticks, more often at high flow
 * (median 35-49 slm on stale samples against 10-29 fresh) - i.e. most often
 * during exactly the high-flow event this path exists to catch.
 *
 * Because updateMaskOff RESETS the fast confirm on any false tick, treating
 * every stale tick as invalid meant the 300/600 ms confirms rarely
 * completed: only 36 % of 600 ms windows were free of a stale sample even at
 * the 1-in-8 tick resolution the B line offers. A held value one tick old is
 * perfectly good evidence for a 300-600 ms confirm - the flow LPF and leak
 * estimator likewise freeze across stale ticks rather than resetting.
 *
 * 6 matches FS_CONV_TICKS_MAX in main.c (30 ms): past that the sensor task
 * has given up on the conversion and restarted it, and the flow really is
 * old. Longer runs still block the fast path, and the pressure-only and
 * rpm-ratio backstops, which read no flow, are unaffected either way. */
volatile uint32_t g_therapy_mask_off_flow_stale_ticks_max = 6U;

/* === Live state (debugger-readable volatiles) === */
volatile uint8_t  g_therapy_state          = (uint8_t)THERAPY_STATE_IDLE;
volatile float    g_therapy_setpoint_cmh2o = 0.0f;
volatile float    g_therapy_measured_cmh2o = 0.0f;
volatile float    g_therapy_leak_slm       = 0.0f;
volatile uint32_t g_therapy_run_ms         = 0U;     /* time in RAMP+RUNNING*/
/* #57: part of run_ms spent in mask-off (usage = run - mask-off). */
volatile uint32_t g_therapy_session_mask_off_ms = 0U;
volatile uint32_t g_therapy_ramp_total_ms  = 0U;     /* this session        */

/* Session pressure percentiles (#63). Histogram 0..30 cmH2O @ 0.2 step. */
#define THERAPY_P_HIST_STEP_CMH2O  0.2f
#define THERAPY_P_HIST_MAX_CMH2O   30.0f
#define THERAPY_P_HIST_N           151U   /* 0.0 .. 30.0 inclusive */

static uint32_t s_p_hist[THERAPY_P_HIST_N];
static uint32_t s_p_hist_count = 0U;
static float    s_p_sum_cmh2o  = 0.0f;
static float    s_p_min_cmh2o  = 0.0f;
static float    s_p_max_cmh2o  = 0.0f;
volatile float  g_therapy_session_p_avg = 0.0f;
volatile float  g_therapy_session_p_min = 0.0f;
volatile float  g_therapy_session_p_max = 0.0f;
volatile float  g_therapy_session_p50   = 0.0f;
volatile float  g_therapy_session_p90   = 0.0f;
volatile float  g_therapy_session_p95   = 0.0f;
volatile uint32_t g_therapy_session_p_samples = 0U;
volatile uint32_t g_therapy_ramp_elapsed_ms = 0U;
volatile uint32_t g_therapy_stop_elapsed_ms = 0U;

volatile uint8_t  g_therapy_fault          = (uint8_t)THERAPY_FAULT_NONE;

/* === Internal === */
static float s_session_target_cmh2o  = THERAPY_TARGET_DEFAULT;
static uint32_t s_therapy_start_ms   = 0U;
static bool     s_mask_off_active    = false;
static uint32_t s_mask_off_fast_enter_ms  = 0U;
static uint32_t s_mask_off_ratio_enter_ms = 0U;
static uint32_t s_mask_off_press_only_ms  = 0U;
static uint32_t s_mask_off_rpm_ratio_ms   = 0U;
#define MASK_OFF_SUST_BIN_MS  50U
#define MASK_OFF_SUST_BINS    60U          /* 60 x 50 ms = 3 s boxcar */
static float    s_mo_sust_bins[MASK_OFF_SUST_BINS];
static float    s_mo_sust_bin_sum = 0.0f;
static uint32_t s_mo_sust_bin_n   = 0U;
static uint32_t s_mo_sust_bin_ms  = 0U;
static uint8_t  s_mo_sust_idx     = 0U;
static uint8_t  s_mo_sust_fill    = 0U;
static uint32_t s_mask_off_sustained_ms = 0U;
static uint32_t s_mask_off_verify_ms      = 0U; /* probation after a clear */
static uint32_t s_mask_off_clear_extra_ms = 0U; /* backoff after a bad clear */
/* Set when the entry path that fired carried no flow evidence. The flow-only
 * clear is meaningless in that case - see maskOffFlowOnlyClearCondition(). */
static bool     s_mask_off_flow_blind_entry = false;
static uint32_t s_mask_off_clear_ms  = 0U;
static uint32_t s_mask_off_hold_ms   = 0U;
static uint32_t s_mask_off_active_ms = 0U;
static uint32_t s_mask_off_ramp_left_ms = 0U;
static uint32_t s_mask_off_hold_elapsed_ms = 0U; /* time at hold RPM only */
static float    s_mask_off_p_baseline  = 0.0f;
static bool     s_mask_off_p_baseline_valid = false;
static float    s_mask_off_hold_p_ref  = 0.0f;  /* learned at hold RPM only */
static bool     s_mask_off_hold_p_ref_valid = false;
static uint32_t s_mask_off_flow_clear_ms = 0U;
static float    s_mask_off_q_peak_slm  = 0.0f;  /* peak |Q| while holding */
static float    s_mask_off_arm_excess_min = 0.0f;
static uint8_t  s_mask_off_prev_breath_insp = 0U;
static uint32_t s_mask_off_last_breath_edge_ms = 0U;
static uint32_t s_reconnect_grace_until_ms = 0U;       /* mask-off re-entry block */
static uint32_t s_safety_reconnect_grace_until_ms = 0U; /* low-P safety grace     */
static uint32_t s_mask_off_resume_boost_ms = 0U;
static uint32_t s_mask_off_resume_repost_ms = 0U;

/* Last EPR settings from USB/GUI — re-applied atomically at Therapy_Start so
 * relief level cannot be left at the default 1 cmH2O if commands reorder. */
static uint8_t  s_epr_cfg_enable = 0U;
static uint8_t  s_epr_cfg_level  = 1U;
static uint8_t  s_epr_cfg_type   = 0U; /* 0=full time, 1=ramp only */

static void applyStoredEprConfig(void)
{
    Epr_SetType((s_epr_cfg_type == 1U) ? EPR_TYPE_RAMP_ONLY : EPR_TYPE_FULL_TIME);
    Epr_SetLevel(s_epr_cfg_level);
    Epr_SetEnable(s_epr_cfg_enable != 0U);
}

static bool therapyAutosetActive(void)
{
    return (g_therapy_mode == (uint8_t)THERAPY_MODE_AUTOSET)
        || (g_therapy_mode == (uint8_t)THERAPY_MODE_AUTOSET_FOR_HER);
}

static bool maskOffResumeBoostActive(void);

/* What to command right now. While a mask-on resume is still rebuilding
 * pressure, EPR gets no say: its expiratory baseline is CPAP - relief, so
 * letting it run during the resume capped recovery a full relief level short
 * of the set pressure and looked like the blower had given up. */
static float resumeAwareSetpointCmh2o(float base_sp_cmh2o, float dt_s)
{
    if (maskOffResumeBoostActive()
        && (BlowerCtrl_GetMeasured() < (base_sp_cmh2o - 0.3f)))
    {
        return base_sp_cmh2o;
    }
    return Epr_ModifySetpoint(base_sp_cmh2o, dt_s, g_therapy_state);
}

static void applyTherapySetpoint(float base_sp_cmh2o, float dt_s)
{
    const float sp = resumeAwareSetpointCmh2o(base_sp_cmh2o, dt_s);
    BlowerCtrl_SetSetpoint(sp);
}

/* === Helpers === */

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void sessionPressureReset(void)
{
    for (uint32_t i = 0U; i < THERAPY_P_HIST_N; i++)
    {
        s_p_hist[i] = 0U;
    }
    s_p_hist_count = 0U;
    s_p_sum_cmh2o  = 0.0f;
    s_p_min_cmh2o  = 0.0f;
    s_p_max_cmh2o  = 0.0f;
    g_therapy_session_p_avg = 0.0f;
    g_therapy_session_p_min = 0.0f;
    g_therapy_session_p_max = 0.0f;
    g_therapy_session_p50   = 0.0f;
    g_therapy_session_p90   = 0.0f;
    g_therapy_session_p95   = 0.0f;
    g_therapy_session_p_samples = 0U;
}

static float sessionPressurePercentile(float pct)
{
    if (s_p_hist_count == 0U)
    {
        return 0.0f;
    }
    /* pct in [0,100]; target rank is ceil(pct/100 * N) style. */
    uint32_t target = (uint32_t)((pct / 100.0f) * (float)s_p_hist_count);
    if (target < 1U)
    {
        target = 1U;
    }
    if (target > s_p_hist_count)
    {
        target = s_p_hist_count;
    }

    uint32_t cum = 0U;
    for (uint32_t i = 0U; i < THERAPY_P_HIST_N; i++)
    {
        cum += s_p_hist[i];
        if (cum >= target)
        {
            return (float)i * THERAPY_P_HIST_STEP_CMH2O;
        }
    }
    return THERAPY_P_HIST_MAX_CMH2O;
}

static void sessionPressureRefreshCaches(void)
{
    if (s_p_hist_count == 0U)
    {
        g_therapy_session_p_avg = 0.0f;
        g_therapy_session_p_min = 0.0f;
        g_therapy_session_p_max = 0.0f;
        g_therapy_session_p50   = 0.0f;
        g_therapy_session_p90   = 0.0f;
        g_therapy_session_p95   = 0.0f;
        g_therapy_session_p_samples = 0U;
        return;
    }
    g_therapy_session_p_avg = s_p_sum_cmh2o / (float)s_p_hist_count;
    g_therapy_session_p_min = s_p_min_cmh2o;
    g_therapy_session_p_max = s_p_max_cmh2o;
    g_therapy_session_p50   = sessionPressurePercentile(50.0f);
    g_therapy_session_p90   = sessionPressurePercentile(90.0f);
    g_therapy_session_p95   = sessionPressurePercentile(95.0f);
    g_therapy_session_p_samples = s_p_hist_count;
}

static void sessionPressureSample(float p_cmh2o)
{
    float p = p_cmh2o;
    if (p < 0.0f)
    {
        p = 0.0f;
    }
    if (p > THERAPY_P_HIST_MAX_CMH2O)
    {
        p = THERAPY_P_HIST_MAX_CMH2O;
    }

    uint32_t bin = (uint32_t)(p / THERAPY_P_HIST_STEP_CMH2O + 0.5f);
    if (bin >= THERAPY_P_HIST_N)
    {
        bin = THERAPY_P_HIST_N - 1U;
    }

    s_p_hist[bin]++;
    s_p_hist_count++;
    s_p_sum_cmh2o += p;
    if (s_p_hist_count == 1U)
    {
        s_p_min_cmh2o = p;
        s_p_max_cmh2o = p;
    }
    else
    {
        if (p < s_p_min_cmh2o) { s_p_min_cmh2o = p; }
        if (p > s_p_max_cmh2o) { s_p_max_cmh2o = p; }
    }

    /* Refresh caches ~1 Hz equivalent (every 200 samples @ 200 Hz). */
    if ((s_p_hist_count % 200U) == 0U)
    {
        sessionPressureRefreshCaches();
    }
}

static void setState(TherapyState_t s)
{
    if ((uint8_t)s != g_therapy_state)
    {
        DBG_I("THR", "state %u -> %u",
              (unsigned)g_therapy_state, (unsigned)s);
        g_therapy_state = (uint8_t)s;
    }
}

bool Therapy_SafetyGraceActive(void)
{
    const uint32_t now = HAL_GetTick();
    if (g_safety_grace_s > 0.0f)
    {
        const uint32_t grace_ms = (uint32_t)(g_safety_grace_s * 1000.0f);
        if ((now - s_therapy_start_ms) < grace_ms)
        {
            return true;
        }
    }
    /* Post mask-on: allow pressure rebuild without low-P-at-high-RPM fault.
     * Kept separate from mask-off re-entry grace (which is much shorter). */
    if ((s_safety_reconnect_grace_until_ms != 0U)
        && (now < s_safety_reconnect_grace_until_ms))
    {
        return true;
    }
    return false;
}

static int32_t clampMaskOffRpm(int32_t rpm)
{
    if (rpm < 1000) return 1000;
    if (rpm > 12000) return 12000;
    return rpm;
}

static bool maskOffReconnectGraceActive(void)
{
    const uint32_t now = HAL_GetTick();
    return (s_reconnect_grace_until_ms != 0U) && (now < s_reconnect_grace_until_ms);
}

/* Returns the CM4 ramp duration actually posted (ms), or 0 if skipped. */
static uint32_t maskOffApplyHoldRpm(bool soft_ramp)
{
    /* ONE IPC write only. CM4 SET_SPEED auto-starts / ack-faults when idle.
     * Never re-post a long soft ramp while mech is still above hold — that
     * restarts FOC's ramp and the blower crawls down forever. */
    const int32_t rpm = clampMaskOffRpm(g_therapy_mask_off_rpm);

    BlowerIpcStatus_t st;
    (void)BlowerIpc_CM7_GetStatus(&st);
    const int32_t mech = st.mech_speed_rpm;
    const bool faults = (st.current_faults != 0U);
    const bool lost = (!st.motor_running) || faults || (mech < (rpm - 1000));

    /* Still decelerating toward hold: leave the in-flight CM4 ramp alone. */
    if (!soft_ramp && !lost && (mech > (rpm + 500)))
    {
        return 0U;
    }

    int32_t delta = mech - rpm;
    if (delta < 0)
    {
        delta = -delta;
    }

    uint32_t ramp = 150U;
    if (soft_ramp || lost || (delta > 400))
    {
        /* ~4 kRPM/s with regen brake; cap so large drops stay under ~2.5 s. */
        ramp = g_therapy_mask_off_ramp_ms;
        const uint32_t from_delta = (uint32_t)((delta * 1000) / 4000);
        if (from_delta > ramp)
        {
            ramp = from_delta;
        }
        if (lost && (mech < rpm) && (ramp < 800U))
        {
            ramp = 800U; /* re-accel from coast */
        }
        if (ramp < 400U)
        {
            ramp = 400U;
        }
        if (ramp > 2500U)
        {
            ramp = 2500U;
        }
    }

    (void)BlowerIpc_CM7_SetSpeedRpm(rpm, (uint16_t)ramp);
    /* PID is off while parked, so Step() no longer refreshes the output
     * registers. Tell blower_ctrl what the motor is actually being asked for:
     * the safety supervisor and telemetry read GetOutputRpm(). */
    BlowerCtrl_NoteParkedRpm((float)rpm);
    return ramp;
}

static void maskOffTrackBreathEdges(void)
{
    const uint8_t insp = Leak_GetBreathInspiration();
    if (insp != s_mask_off_prev_breath_insp)
    {
        s_mask_off_last_breath_edge_ms = HAL_GetTick();
        s_mask_off_prev_breath_insp = insp;
    }
}

static bool maskOffBreathInhibitActive(void)
{
    if (g_therapy_mask_off_breath_inhibit_enable == 0U)
    {
        return false;
    }

    /* Do NOT use sustained breath-rate here — Vivo often keeps rate ≥4 while
     * the hose is already open, which blocked mask-off for many seconds. */

    if (s_mask_off_last_breath_edge_ms == 0U)
    {
        return false;
    }
    const uint32_t now = HAL_GetTick();
    const uint32_t win = (g_therapy_mask_off_breath_inhibit_ms > 0U)
                           ? g_therapy_mask_off_breath_inhibit_ms : 1U;
    return ((now - s_mask_off_last_breath_edge_ms) <= win);
}

/* Vent flow the circuit is expected to be passing at the CURRENT SETPOINT.
 *
 * Deliberately the setpoint and not the measured pressure. The leak
 * estimator's own g_leak_vent_slm tracks measured mask pressure, which dips
 * during a deep inspiratory sag - referencing that would lower the enter
 * threshold at exactly the moment a big breath is pushing total flow up, and
 * make the false trigger easier rather than harder. The setpoint is what the
 * vent should be flowing whenever the mask is on and regulation is working.
 *
 * Same model the estimator uses (leak_estimator.c): vent_k * sqrt(P), scaled
 * by the altitude factor so this tracks the flow element's own correction. */
static float maskOffVentRefSlm(void)
{
    const float sp = BlowerCtrl_GetSetpointSlewed();
    const float p  = (sp > 0.0f) ? sp : 0.0f;
    return Leak_GetVentCoefficient() * sqrtf(p) * Flow_GetAltitudeFactor();
}

/* |Q_total| at or above this means "more flow than a worn mask explains". */
static float maskOffEnterThresholdSlm(void)
{
    const float fixed = clampf(g_therapy_mask_off_enter_slm, 10.0f, 150.0f);
    float thresh = fixed;

    if (g_therapy_mask_off_enter_vent_ref != 0U)
    {
        const float margin = clampf(g_therapy_mask_off_enter_margin_slm,
                                    10.0f, 80.0f);
        const float vent_ref = maskOffVentRefSlm() + margin;
        /* Never go below the old fixed value, so this can only ever make
         * mask-off harder to trigger, not easier. With the default 30 slm
         * margin the floor binds below about 3 cmH2O (8.7*sqrt(P) + 30 < 45),
         * i.e. never at a real therapy setting - it is there to keep a small
         * hand-set margin from quietly raising sensitivity. */
        if (vent_ref > thresh)
        {
            thresh = vent_ref;
        }
    }

    g_therapy_mask_off_enter_thresh_now = thresh;
    return thresh;
}

/* Pressure on the floor in absolute terms, not merely below setpoint.
 *
 * This is the open-hose signature: with the circuit open the blower cannot
 * build pressure at all, so P goes to near zero (measured 0.33 cmH2O on a
 * genuine disconnect). A deep inhale is a completely different shape - it
 * sags 1-3 cmH2O off an 18 cmH2O setpoint and recovers. Used only to gate the
 * shortened "strong open" confirm; the ordinary relative test still governs
 * whether mask-off is considered at all. */
static bool maskOffPressureFloored(void)
{
    const float abs_floor = clampf(g_therapy_mask_off_press_abs_cmh2o,
                                   0.5f, 15.0f);
    const float drop = clampf(g_therapy_mask_off_press_drop_cmh2o, 0.3f, 10.0f);
    const float relative_floor = BlowerCtrl_GetSetpointSlewed() - drop;
    /* At a 4 cmH2O target, 3.9 is NOT a pressure collapse. Require a real
     * shortfall as well as the absolute floor for this strong evidence. */
    const float floor = fminf(abs_floor, relative_floor);
    return (BlowerCtrl_GetMeasured() < floor)
         && (BlowerCtrl_GetSensorCmh2o() < floor);
}

/* Open-circuit pressure evidence for the clipped-flow / upper-limited
 * exception in maskOffEnterFastCondition().
 *
 * maskOffPressureFloored() puts BOTH signals under the same 4 cmH2O floor.
 * That holds at low setpoints but not at 20: the device-side sensor reads the
 * hose drop at open-circuit flow, and with the blower at the 40 k ceiling
 * that drop alone is several cmH2O. Bench mask pulls 2026-09-25, from 300 ms
 * after the circuit opened until entry:
 *      sp  6/10: P_sensor <= 3.3-4.0, P_mask <= 1.4-2.1 -> fast path, ~0.5 s
 *      sp 20:    P_sensor 3.5-5.1,    P_mask 2.1-3.2    -> fast path blocked,
 *                                                         rpm-ratio at 2.5 s
 * The mask estimate did floor at 20; only the sensor did not.
 *
 * So the MASK estimate keeps the 4.0 floor, and the SENSOR gets
 * press_sensor_allow on top (default 2.5 -> 6.5), both still capped at
 * sp - press_drop so this never accepts less than a real shortfall. At 20
 * that holds 600 ms continuously by ~620 ms after the pull (was 2.5 s), with
 * 1.4 cmH2O margin over the highest sensor reading. The false trigger this
 * exception exists to reject had P_sensor 17.77, and sensor minima during
 * 20 cmH2O breathing never went below 17.9, so the gap stays wide.
 *
 * The 300 ms "strong open" shortcut keeps maskOffPressureFloored(): a
 * disconnect accepted only on the wider sensor floor gets the nominal
 * confirm. At setpoints up to ~7.7 the cap makes the sensor floor equal to
 * the relative gate; clipping or hitting the ceiling on a breath is not
 * reachable there, and the mask estimate must still be under 4.
 *
 * NOTE: the mask estimate here is tube compensation applied to a CLIPPED
 * flow, so it reads high. The bench pulls above ran on the old tube-comp pair
 * (0.002 / 0.00022, which TubeComp_Init used to force); with the calibrated
 * 0.00353 / 0.000118 the modelled drop at 89 slm is 0.67 cmH2O smaller.
 * Replaying the four 20 cmH2O pulls with it: P_mask peaks 3.52-3.71 in the
 * confirm window (was 2.84-3.03) and the 600 ms confirm still completes at
 * 620-660 ms. So the 4.0 floor holds, but the margin at 20 is now ~0.3
 * cmH2O (6/10: peaks 2.3-2.6). Recheck with real pulls. */
static bool maskOffOpenCircuitPressure(void)
{
    const float abs_floor = clampf(g_therapy_mask_off_press_abs_cmh2o,
                                   0.5f, 15.0f);
    const float allow = clampf(g_therapy_mask_off_press_sensor_allow_cmh2o,
                               0.0f, 6.0f);
    const float drop = clampf(g_therapy_mask_off_press_drop_cmh2o, 0.3f, 10.0f);
    const float relative_floor = BlowerCtrl_GetSetpointSlewed() - drop;
    const float mask_floor = fminf(abs_floor, relative_floor);
    const float sens_floor = fminf(abs_floor + allow, relative_floor);
    return (BlowerCtrl_GetMeasured() < mask_floor)
         && (BlowerCtrl_GetSensorCmh2o() < sens_floor);
}

static bool maskOffPressureCollapsed(void)
{
    const float p_meas = BlowerCtrl_GetMeasured();
    const float p_sens = BlowerCtrl_GetSensorCmh2o();
    const float sp = BlowerCtrl_GetSetpointSlewed();
    const float drop = clampf(g_therapy_mask_off_press_drop_cmh2o, 0.3f, 10.0f);
    const float abs_floor = clampf(g_therapy_mask_off_press_abs_cmh2o, 0.5f, 15.0f);

    /* BOTH signals, not either.
     *
     * p_meas is the tube-compensated mask estimate: it is p_sens minus the
     * modelled hose drop, and that drop is large at inspiratory flow. At
     * 18 cmH2O with a human breathing, the hose term runs ~1.6 cmH2O, so
     * p_meas sits well over a cmH2O below p_sens through every inhale - more
     * than the 1.2 cmH2O press_drop this test uses. With an OR here, tube
     * compensation alone armed mask-off on a perfectly intact circuit.
     *
     * Measured on the bench 2026-09-23, six false entries in 75 s; in five of
     * them the raw sensor never crossed the threshold at all, and in one it
     * read 18.20 against a setpoint of 18.00 - i.e. mask-off fired while
     * pressure was ABOVE target. Only the sixth was a real collapse (sensor
     * 15.96), caused by the RPM ceiling rather than by any disconnection.
     *
     * maskOffEnterPressureOnlyCondition() has required both signals since it
     * was written, for exactly this reason - see its comment. This test was
     * simply never given the same guard.
     *
     * A genuine disconnect costs nothing: with the circuit open neither
     * signal can hold pressure. The same capture's exit lines recorded
     * p_sens 0.21 / p_meas 0.16 and 0.07 / -0.00. */
    const bool vs_setpoint = (p_meas < (sp - drop)) && (p_sens < (sp - drop));
    const bool abs_low = (p_meas < abs_floor) && (p_sens < abs_floor);
    return vs_setpoint || abs_low;
}

static float maskOffTotalFlowSlm(void)
{
    /* Orifice / SFM path — immediate, no leak or patient-flow filtering. */
    return Flow_GetSlm();
}

static bool maskOffEnterFastCondition(void)
{
    g_therapy_mask_off_diag_fast_block = 0U;
    if (maskOffReconnectGraceActive())
    {
        g_therapy_mask_off_diag_excess = 0U;
        g_therapy_mask_off_diag_min = 0U;
        g_therapy_mask_off_diag_press = 0U;
        g_therapy_mask_off_diag_breath = 0U;
        return false;
    }

    maskOffTrackBreathEdges();

    /* Raw |Q_total| from the flow element. Open hose >> sealed breath peaks
     * at therapy RPM; still require pressure collapse so a strong inhale
     * alone cannot trip mask-off. */
    const float q_abs = fabsf(maskOffTotalFlowSlm());
    const float enter = maskOffEnterThresholdSlm();
    /* Stale with a zero run = no fresh sample since Flow_Init: invalid. A
     * short held run (<= flow_stale_ticks_max) uses the held value, which is
     * at most that many ticks old. See g_therapy_mask_off_flow_stale_ticks_max. */
    const uint32_t stale_run = Flow_GetStaleRun();
    const bool flow_fresh_enough = !Flow_IsSampleStale()
        || ((stale_run > 0U)
            && (stale_run <= g_therapy_mask_off_flow_stale_ticks_max));
    const bool flow_valid = flow_fresh_enough && isfinite(q_abs);
    const bool flow_ok = flow_valid && (q_abs >= enter);
    g_therapy_mask_off_diag_excess = flow_ok ? 1U : 0U;

    const bool press_ok = maskOffPressureCollapsed();
    g_therapy_mask_off_diag_press = press_ok ? 1U : 0U;

    /* Breath-edge inhibit only for ambiguous mid flows. Clear open-hose
     * |Q| (≥ enter) must not wait on Vivo breath state. */
    const bool breath_block = maskOffBreathInhibitActive()
                           && !press_ok
                           && (q_abs < (enter * 1.15f));
    g_therapy_mask_off_diag_breath = breath_block ? 1U : 0U;

    /* 2026-09-24: at 20 cmH2O / 38500 RPM, a large inspiration gave
     * P_sensor 17.77, P_mask 15.85 and clipped Q=89.1. That satisfied the
     * old relative-pressure gate for 600 ms and falsely parked the blower.
     * A clipped flow cannot distinguish this breath from an open circuit;
     * a pressure deficit at the speed ceiling cannot do so either.
     *
     * Reject those AMBIGUOUS fast-path cases, not all disconnects at a cap.
     * Fresh high flow (even a clipped bound) plus BOTH pressures at
     * open-circuit level (maskOffOpenCircuitPressure: mask under the floor,
     * sensor under the floor plus its hose allowance) still qualifies.
     * Flow stale past flow_stale_ticks_max, never
     * received, or nonfinite never qualifies. The
     * flow-independent pressure/RPM backstops remain unchanged, including
     * their longer confirmation timers; no safety limit is relaxed here.
     * These guards also apply when the optional relative gate is disabled. */
    if (!flow_valid)
    {
        g_therapy_mask_off_diag_fast_block |= 1U;
    }
    /* Open circuit by flow (see g_therapy_mask_off_open_flow_slm). A saturated
     * sample is the element's full scale, so it counts only if that is above
     * the threshold - never for Round Hole_2. */
    const bool open_by_flow = (g_therapy_mask_off_open_flow_slm > 0.0f)
                           && flow_valid
                           && (q_abs >= g_therapy_mask_off_open_flow_slm);
    g_therapy_mask_off_diag_open_flow = 0U;
    if (!maskOffOpenCircuitPressure() && open_by_flow
        && (Flow_IsSampleSaturated() || BlowerCtrl_IsUpperLimited()))
    {
        g_therapy_mask_off_diag_open_flow = 1U;
    }
    if (!maskOffOpenCircuitPressure() && !open_by_flow)
    {
        if (Flow_IsSampleSaturated())
        {
            g_therapy_mask_off_diag_fast_block |= 2U;
        }
        if (BlowerCtrl_IsUpperLimited())
        {
            g_therapy_mask_off_diag_fast_block |= 4U;
        }
    }
    if (g_therapy_mask_off_diag_fast_block != 0U)
    {
        g_therapy_mask_off_diag_min = 0U;
        return false; /* updateMaskOff resets, rather than pauses, fast arming */
    }

    if (!flow_ok)
    {
        g_therapy_mask_off_diag_min = 0U;
        return false;
    }

    if ((g_therapy_mask_off_require_press != 0U) && !press_ok)
    {
        g_therapy_mask_off_diag_min = 0U;
        return false;
    }

    if (breath_block)
    {
        g_therapy_mask_off_diag_min = 0U;
        return false;
    }

    return true;
}

/* Pressure-only disconnect evidence: the PID has been driven up near the RPM
 * ceiling and the circuit still will not hold pressure. Deliberately reads no
 * flow at all, so it survives a dead or under-reading flow element. */
static bool maskOffEnterPressureOnlyCondition(void)
{
    if (maskOffReconnectGraceActive()
        || (g_therapy_mask_off_press_only_enable == 0U))
    {
        g_therapy_mask_off_diag_press_only = 0U;
        return false;
    }

    /* Never during the therapy-start or post-reconnect transients: pressure is
     * legitimately on the floor there while the circuit fills. This is the
     * same window the safety supervisor honours before it will judge low
     * pressure, which also keeps the two timers aligned. */
    if (Therapy_SafetyGraceActive())
    {
        g_therapy_mask_off_diag_press_only = 0U;
        return false;
    }

    /* Only meaningful while actually asking for a real pressure. */
    const float sp_slew = BlowerCtrl_GetSetpointSlewed();
    if (sp_slew < 2.0f)
    {
        g_therapy_mask_off_press_only_thresh = 0.0f;
        g_therapy_mask_off_diag_press_only = 0U;
        return false;
    }

    const float frac = clampf(g_therapy_mask_off_press_only_frac, 0.20f, 0.80f);
    const float cap  = clampf(g_therapy_mask_off_press_only_cmh2o, 0.5f, 8.0f);
    float thresh = frac * sp_slew;
    if (thresh > cap)
    {
        thresh = cap;
    }
    g_therapy_mask_off_press_only_thresh = thresh;

    const bool rpm_high = (BlowerCtrl_GetOutputRpm()
                             >= (float)g_therapy_mask_off_press_only_rpm);
    /* Both the raw sensor and the mask estimate, so a tube-comp artefact on
     * one of them cannot arm this by itself. */
    const bool p_floor  = (BlowerCtrl_GetSensorCmh2o() < thresh)
                       && (BlowerCtrl_GetMeasured() < thresh);

    const bool ok = rpm_high && p_floor && maskOffPressureCollapsed();
    g_therapy_mask_off_diag_press_only = ok ? 1U : 0U;
    return ok;
}

/* Scale-invariant disconnect evidence: commanded RPM far above what the
 * calibrated model says this setpoint needs, with the pressure still short.
 * Reads no flow and no absolute pressure floor, so it survives both a dead
 * flow element and a circuit that holds pressure well with the mask off. */
static bool maskOffEnterRpmRatioCondition(void)
{
    if (maskOffReconnectGraceActive()
        || (g_therapy_mask_off_rpm_ratio_enable == 0U))
    {
        g_therapy_mask_off_diag_rpm_ratio = 0U;
        return false;
    }

    /* Same exclusion as the pressure path: during start-up and post-reconnect
     * the loop is legitimately commanding far more RPM than steady state. */
    if (Therapy_SafetyGraceActive())
    {
        g_therapy_mask_off_diag_rpm_ratio = 0U;
        return false;
    }

    const float sp_slew = BlowerCtrl_GetSetpointSlewed();
    const float ss_rpm  = BlowerCtrl_EstimateSteadyStateRpm(sp_slew);
    g_therapy_mask_off_rpm_ratio_ss = ss_rpm;
    if (!(ss_rpm > 1000.0f))
    {
        g_therapy_mask_off_diag_rpm_ratio = 0U;
        return false;
    }

    const float ratio = BlowerCtrl_GetOutputRpm() / ss_rpm;
    g_therapy_mask_off_rpm_ratio_now = ratio;

    /* Deliberately NOT maskOffPressureCollapsed(): its absolute-floor branch
     * (below 4 cmH2O on both sensors) is true during normal therapy at a 4
     * cmH2O setpoint, which would leave the ratio as the only real gate. Ask
     * instead for a genuine shortfall against the setpoint, on both signals. */
    const float drop = clampf(g_therapy_mask_off_press_drop_cmh2o, 0.3f, 10.0f);
    const bool  p_short = (BlowerCtrl_GetMeasured() < (sp_slew - drop))
                       && (BlowerCtrl_GetSensorCmh2o() < (sp_slew - drop));

    const float need = clampf(g_therapy_mask_off_rpm_ratio, 1.10f, 3.0f);
    const bool  ok = (ratio >= need) && p_short;
    g_therapy_mask_off_diag_rpm_ratio = ok ? 1U : 0U;
    return ok;
}

static void maskOffSustainedReset(void)
{
    s_mo_sust_bin_sum = 0.0f;
    s_mo_sust_bin_n   = 0U;
    s_mo_sust_bin_ms  = 0U;
    s_mo_sust_idx     = 0U;
    s_mo_sust_fill    = 0U;
    s_mask_off_sustained_ms = 0U;
    g_therapy_mask_off_sustained_mean_slm = 0.0f;
}

/* See g_therapy_mask_off_sustained_enable. Called every tick while not
 * parked; keeps the 3 s window of total flow. */
static bool maskOffEnterSustainedCondition(uint32_t dt_ms)
{
    if ((g_therapy_mask_off_sustained_enable == 0U)
        || maskOffReconnectGraceActive() || Therapy_SafetyGraceActive())
    {
        maskOffSustainedReset();
        return false;
    }

    const float q = maskOffTotalFlowSlm();
    if (isfinite(q))
    {
        s_mo_sust_bin_sum += q;
        s_mo_sust_bin_n++;
    }
    s_mo_sust_bin_ms += dt_ms;
    if (s_mo_sust_bin_ms >= MASK_OFF_SUST_BIN_MS)
    {
        if (s_mo_sust_bin_n > 0U)
        {
            s_mo_sust_bins[s_mo_sust_idx] = s_mo_sust_bin_sum / (float)s_mo_sust_bin_n;
            s_mo_sust_idx = (uint8_t)((s_mo_sust_idx + 1U) % MASK_OFF_SUST_BINS);
            if (s_mo_sust_fill < MASK_OFF_SUST_BINS)
            {
                s_mo_sust_fill++;
            }
        }
        s_mo_sust_bin_sum = 0.0f;
        s_mo_sust_bin_n   = 0U;
        s_mo_sust_bin_ms  = 0U;
    }
    if (s_mo_sust_fill < MASK_OFF_SUST_BINS)
    {
        return false;                   /* need a full 3 s window */
    }

    float sum = 0.0f;
    for (uint32_t i = 0U; i < MASK_OFF_SUST_BINS; i++)
    {
        sum += s_mo_sust_bins[i];
    }
    const float mean = sum / (float)MASK_OFF_SUST_BINS;
    g_therapy_mask_off_sustained_mean_slm = mean;

    if (BlowerCtrl_GetSetpointSlewed() < 2.0f)
    {
        return false;
    }
    const float margin = clampf(g_therapy_mask_off_sustained_margin_slm, 20.0f, 150.0f);
    return mean >= (maskOffVentRefSlm() + margin);
}

/* Vivo50 vCheckDisconnect: liGetMeanRatio10Sec() > DISCONNECT_LIMIT (2000). */
static bool maskOffEnterRatioCondition(void)
{
    if (maskOffReconnectGraceActive())
    {
        return false;
    }

    if (g_therapy_mask_off_ratio_enable == 0U)
    {
        return false;
    }
    if (!Leak_IsVivoEnabled())
    {
        return false;
    }

    const int32_t limit = g_therapy_mask_off_ratio_10s_limit;
    if (limit <= 0)
    {
        return false;
    }

    if (Leak_GetMeanRatio10Sec() <= limit)
    {
        return false;
    }

    /* Same pressure gate as fast path — MeanRatio can rise while breathing. */
    if ((g_therapy_mask_off_require_press != 0U) && !maskOffPressureCollapsed())
    {
        return false;
    }

    return true;
}

/* Vivo hysteresis: abort slow ratio arming when instant ratio falls well
 * below the disconnect threshold while 10 s mean is still elevated. */
static void maskOffRatioEnterHysteresis(void)
{
    if (!Leak_IsVivoEnabled())
    {
        return;
    }

    const int32_t limit = g_therapy_mask_off_ratio_10s_limit;
    if (limit <= 0)
    {
        return;
    }

    const int32_t ratio10s = Leak_GetMeanRatio10Sec();
    const int32_t ratio_inst = Leak_GetInstantRatio();

    if ((ratio10s > limit)
        && (ratio_inst > 0)
        && (ratio_inst < (limit * 10)))
    {
        s_mask_off_ratio_enter_ms = 0U;
    }
}

/* Hold-RPM pressure reference below this reads as an open circuit (an open
 * hose at hold measures ~0.07 cmH2O; a sealed mask ~0.6). Shared by the
 * pressure-tip clear and the flow-only escape hatch so they agree on what
 * "open" means. */
#define MASK_OFF_HOLD_REF_OPEN_CMH2O  0.50f

/* High-leak still present on the *instant* residual only. Do not use
 * Leak_IsHigh() here — its 10 s LPF stays elevated long after reconnect. */
static void maskOffTrackOpenCircuit(float p_sensor_cmh2o, float q_total_slm,
                                    bool learning)
{
    const float q_abs = fabsf(q_total_slm);

    if (!s_mask_off_p_baseline_valid)
    {
        s_mask_off_p_baseline = p_sensor_cmh2o;
        s_mask_off_p_baseline_valid = true;
    }
    else if (p_sensor_cmh2o < s_mask_off_p_baseline)
    {
        s_mask_off_p_baseline = p_sensor_cmh2o;
    }

    /* Reference measured at hold RPM and frozen once the learn window closes,
     * so a reconnect cannot drag it. The baseline above spans the whole decel
     * from therapy pressure, which makes it only a fallback. */
    if (learning)
    {
        if (!s_mask_off_hold_p_ref_valid)
        {
            s_mask_off_hold_p_ref = p_sensor_cmh2o;
            s_mask_off_hold_p_ref_valid = true;
        }
        else if (p_sensor_cmh2o < s_mask_off_hold_p_ref)
        {
            s_mask_off_hold_p_ref = p_sensor_cmh2o;
        }
    }

    if (q_abs > s_mask_off_q_peak_slm)
    {
        s_mask_off_q_peak_slm = q_abs;
    }
}

static bool maskOffClearCondition(void)
{
    /* Do not clear while still decelerating to hold — residual therapy
     * pressure would look like a seal and abort the 6 kRPM park. */
    BlowerIpcStatus_t st;
    (void)BlowerIpc_CM7_GetStatus(&st);
    const int32_t hold = clampMaskOffRpm(g_therapy_mask_off_rpm);
    if (st.mech_speed_rpm > (hold + 1200))
    {
        g_therapy_mask_off_reconn_press_ok = 0U;
        g_therapy_mask_off_reconn_flow_ok  = 0U;
        return false;
    }

    const float q_abs = fabsf(maskOffTotalFlowSlm());
    const float p_sensor = BlowerCtrl_GetSensorCmh2o();
    const float delta = clampf(g_therapy_mask_off_reconnect_press_delta_cmh2o,
                               0.15f, 6.0f);
    const float abs_p = clampf(g_therapy_mask_off_reconnect_abs_cmh2o, 0.35f, 10.0f);
    const float flow_frac = clampf(g_therapy_mask_off_flow_drop_frac, 0.20f, 0.90f);
    const float clear_slm = clampf(g_therapy_mask_off_clear_slm, 5.0f, 80.0f);

    /* Reference for an open circuit at hold RPM: prefer the value learned once
     * the decel settled, fall back to the ramp-wide minimum. */
    const bool ref_valid = s_mask_off_hold_p_ref_valid
                        || s_mask_off_p_baseline_valid;
    const float ref = s_mask_off_hold_p_ref_valid ? s_mask_off_hold_p_ref
                                                  : s_mask_off_p_baseline;
    /* An open hose at hold reads near 0. Only then is a ~0.7 absolute reading
     * evidence of a seal by itself. A rise above the reference is meaningful
     * whatever the reference happens to be. */
    const bool ref_open = ref_valid && (ref < MASK_OFF_HOLD_REF_OPEN_CMH2O);

    const bool tip_rise = ref_valid && (p_sensor >= (ref + delta));
    /* Bench observation: mask-on at 6 kRPM tips ~0.7 cmH2O. */
    const bool tip_abs = ref_open && (p_sensor >= abs_p);

    const bool peak_valid = (s_mask_off_q_peak_slm >= 25.0f);
    const bool flow_from_peak =
        peak_valid && (q_abs < (s_mask_off_q_peak_slm * flow_frac));
    const bool flow_low = (q_abs < clear_slm);

    g_therapy_mask_off_reconn_press_ok = (tip_rise || tip_abs) ? 1U : 0U;
    g_therapy_mask_off_reconn_flow_ok  = (flow_low || flow_from_peak) ? 1U : 0U;

    if (!ref_valid)
    {
        return false;
    }

    /* Primary: pressure tip vs open-hose baseline (no flow gate — vented
     * masks often keep |Q| high at hold RPM). */
    if (tip_rise || tip_abs)
    {
        return true;
    }

    /* Secondary assist: large flow collapse with any tip rise. */
    if ((flow_low || flow_from_peak)
        && (p_sensor >= (ref + 0.20f)))
    {
        return true;
    }
    return false;
}

/* Sealed-circuit evidence from flow alone: at hold RPM an open hose moves far
 * more gas than a mask on a face, vented or not. Only used as the fallback in
 * updateMaskOff(), with a longer confirm than the pressure path. */
static bool maskOffFlowOnlyClearCondition(void)
{
    /* Entered without flow evidence (pressure-only or RPM-ratio) means flow
     * could not see this disconnect in the first place - at hold RPM it will
     * read "sealed" whatever the circuit is doing, and clear us straight back
     * out into the runaway. The pressure tip, plus the hold watchdog, are the
     * clears in that case. */
    if (s_mask_off_flow_blind_entry)
    {
        return false;
    }

    /* Only when the hold reference is NOT an open circuit.
     *
     * This hatch exists for the mask going back on before the decel
     * finished: the pressure learned at hold is then the sealed pressure
     * itself, no tip can appear, and flow is the only evidence left. When
     * the reference reads open (< MASK_OFF_HOLD_REF_OPEN_CMH2O), a reconnect
     * DOES produce a tip and maskOffClearCondition() takes it - so this
     * hatch has nothing to add there, and flow alone is not trustworthy:
     * the thresholds below assume an open hose at hold moves > 25-40 slm,
     * but this blower at 6 krpm only pushes ~15-21 slm through the open
     * circuit.
     *
     * Measured 2026-09-25, bench mask pulls at 6/10/20 cmH2O: after every
     * flow-fast entry, with the mask still OFF, hold P_sensor ~0.07 and
     * |Q| 15.2 (peak 19-21, under the 25 slm peak_valid bar) - read as
     * "sealed" by the low-flow branch, cleared after 1.5 s, and therapy
     * resumed into the open circuit for ~3-4 s before re-parking. Mask back
     * on at hold reads ~0.6 cmH2O / 5.2 slm: a clear pressure tip. */
    {
        const bool ref_valid = s_mask_off_hold_p_ref_valid
                            || s_mask_off_p_baseline_valid;
        const float ref = s_mask_off_hold_p_ref_valid ? s_mask_off_hold_p_ref
                                                      : s_mask_off_p_baseline;
        if (ref_valid && (ref < MASK_OFF_HOLD_REF_OPEN_CMH2O))
        {
            return false;
        }
    }

    BlowerIpcStatus_t st;
    (void)BlowerIpc_CM7_GetStatus(&st);
    const int32_t hold = clampMaskOffRpm(g_therapy_mask_off_rpm);
    if (st.mech_speed_rpm > (hold + 1200))
    {
        return false;
    }

    const float q_abs = fabsf(maskOffTotalFlowSlm());
    const float clear_slm = clampf(g_therapy_mask_off_clear_slm, 5.0f, 80.0f);
    const float flow_frac = clampf(g_therapy_mask_off_flow_drop_frac, 0.20f, 0.90f);
    const bool  peak_valid = (s_mask_off_q_peak_slm >= 25.0f);

    if (peak_valid)
    {
        /* Open-hose flow was seen at hold, so only a real collapse counts. */
        return (q_abs < (s_mask_off_q_peak_slm * flow_frac))
            && (q_abs < clear_slm);
    }

    /* Open-hose flow never appeared at hold: consistent with the mask being
     * back on before the decel finished, which is the case the pressure tip
     * cannot see. Low flow is then the only evidence there is. */
    return (q_abs < clear_slm);
}

static float currentRampSetpointCmh2o(void)
{
    if (g_therapy_ramp_total_ms == 0U)
    {
        return s_session_target_cmh2o;
    }
    if (g_therapy_ramp_elapsed_ms >= g_therapy_ramp_total_ms)
    {
        return s_session_target_cmh2o;
    }
    const float frac = (float)g_therapy_ramp_elapsed_ms
                     / (float)g_therapy_ramp_total_ms;
    return THERAPY_RAMP_START_CMH2O
         + frac * (s_session_target_cmh2o - THERAPY_RAMP_START_CMH2O);
}

static float therapyBaseSetpointCmh2o(void)
{
    switch ((TherapyState_t)g_therapy_state)
    {
    case THERAPY_STATE_RAMP:
        return currentRampSetpointCmh2o();
    case THERAPY_STATE_RUNNING:
        return clampf(g_therapy_target_cmh2o,
                      THERAPY_PRESS_MIN_CMH2O,
                      THERAPY_PRESS_MAX_CMH2O);
    default:
        return s_session_target_cmh2o;
    }
}

/* AirSense-style: begin at expiratory (CPAP - level), not full CPAP. */
static void applyEprStartupSnap(float base_sp_cmh2o)
{
    const float exp_sp =
        Epr_GetExpiratorySetpointCmh2o(base_sp_cmh2o, g_therapy_state);
    Epr_SetStartupHoldTarget(exp_sp);
    BlowerCtrl_SnapSetpoint(exp_sp);
}

/* path: 0 = probation re-park, 1 = flow fast, 2 = Vivo ratio,
 *       3 = pressure-only, 4 = rpm-ratio.  See Telem_PushMaskOff(). */
static void enterMaskOff(uint8_t path)
{
    if (s_mask_off_active)
    {
        return;
    }

    /* Latched BEFORE any state is reset, so the M line records the circuit as
     * the decision saw it. The DBG_I below says the same thing but is starved
     * off the shared CDC endpoint whenever telemetry is running, which is
     * every time anyone is actually looking. */
    Telem_PushMaskOff(1U, path,
                      maskOffTotalFlowSlm(),
                      s_mask_off_arm_excess_min,
                      BlowerCtrl_GetSensorCmh2o(),
                      BlowerCtrl_GetMeasured(),
                      BlowerCtrl_GetSetpointSlewed(),
                      g_therapy_mask_off_enter_thresh_now,
                      g_therapy_mask_off_rpm_ratio_now,
                      g_therapy_mask_off_rpm_ratio_ss,
                      Leak_GetMeanRatio10Sec(),
                      Leak_GetBreathInspiration());

    s_mask_off_active = true;
    g_therapy_mask_off_active = 1U;
    g_therapy_mask_off_enter_count++;
    s_mask_off_hold_ms = 0U;
    s_mask_off_active_ms = 0U;
    s_mask_off_hold_elapsed_ms = 0U;
    s_mask_off_clear_ms = 0U;
    s_mask_off_p_baseline_valid = false;
    s_mask_off_p_baseline = 0.0f;
    s_mask_off_hold_p_ref_valid = false;
    s_mask_off_hold_p_ref = 0.0f;
    s_mask_off_flow_clear_ms = 0U;
    s_mask_off_verify_ms = 0U;
    /* Peak is learned only after hold RPM settles — not at therapy RPM. */
    s_mask_off_q_peak_slm = 0.0f;

    Leak_SetDisconnectFreeze(true);

    Epr_ResetBreathState();
    BlowerCtrl_SetEnable(false);
    /* Adaptive soft ramp (may be > default) — block clear until it finishes. */
    s_mask_off_ramp_left_ms = maskOffApplyHoldRpm(true);

    DBG_I("THR",
          "mask off: ramp to %ld rpm in %lu ms (ratio=%u press=%u rpm=%u q=%.2f)",
          (long)clampMaskOffRpm(g_therapy_mask_off_rpm),
          (unsigned long)s_mask_off_ramp_left_ms,
          (unsigned)g_therapy_mask_off_enter_via_ratio,
          (unsigned)g_therapy_mask_off_enter_via_press,
          (unsigned)g_therapy_mask_off_enter_via_rpm,
          (double)g_therapy_mask_off_rpm_ratio_now);
}

static void exitMaskOff(float dt_s)
{
    if (!s_mask_off_active)
    {
        return;
    }

    (void)dt_s;

    /* Paired with the enter line so a capture shows how long the park lasted
     * and what the circuit looked like when it was released. path is carried
     * over from the entry flags, which are cleared a few lines below. */
    Telem_PushMaskOff(0U,
                      (g_therapy_mask_off_enter_via_ratio != 0U) ? 2U :
                      (g_therapy_mask_off_enter_via_press != 0U) ? 3U :
                      (g_therapy_mask_off_enter_via_rpm   != 0U) ? 4U :
                      (g_therapy_mask_off_enter_via_sustained != 0U) ? 5U : 1U,
                      maskOffTotalFlowSlm(),
                      s_mask_off_q_peak_slm,
                      BlowerCtrl_GetSensorCmh2o(),
                      BlowerCtrl_GetMeasured(),
                      BlowerCtrl_GetSetpointSlewed(),
                      g_therapy_mask_off_enter_thresh_now,
                      g_therapy_mask_off_rpm_ratio_now,
                      g_therapy_mask_off_rpm_ratio_ss,
                      Leak_GetMeanRatio10Sec(),
                      Leak_GetBreathInspiration());

    s_mask_off_active = false;
    g_therapy_mask_off_active = 0U;
    g_therapy_mask_on_count++;
    s_mask_off_fast_enter_ms = 0U;
    s_mask_off_ratio_enter_ms = 0U;
    s_mask_off_clear_ms = 0U;
    s_mask_off_active_ms = 0U;
    s_mask_off_hold_elapsed_ms = 0U;
    s_mask_off_ramp_left_ms = 0U;
    s_mask_off_arm_excess_min = 0.0f;
    s_mask_off_flow_clear_ms = 0U;
    s_mask_off_press_only_ms = 0U;
    s_mask_off_rpm_ratio_ms = 0U;
    s_mask_off_flow_blind_entry = false;
    s_mask_off_hold_p_ref_valid = false;
    s_mask_off_hold_p_ref = 0.0f;
    g_therapy_mask_off_enter_via_ratio = 0U;
    g_therapy_mask_off_enter_via_press = 0U;
    g_therapy_mask_off_enter_via_rpm = 0U;
    g_therapy_mask_off_enter_via_sustained = 0U;
    maskOffSustainedReset();
    g_therapy_mask_off_diag_press_only = 0U;
    g_therapy_mask_off_diag_rpm_ratio = 0U;

    Leak_SetDisconnectFreeze(false);

    {
        /* Short re-entry debounce only — do not block the next mask-off for 12 s. */
        float regrace_s = g_therapy_mask_off_regrace_s;
        if (regrace_s < 0.5f)
        {
            regrace_s = 0.5f;
        }
        if (regrace_s > 6.0f)
        {
            regrace_s = 6.0f;
        }
        s_reconnect_grace_until_ms =
            HAL_GetTick() + (uint32_t)(regrace_s * 1000.0f);

        /* Longer safety grace while pressure climbs from tip (~0.7) to set CPAP. */
        s_safety_reconnect_grace_until_ms = HAL_GetTick() + 12000U;
    }

    const float base_sp = therapyBaseSetpointCmh2o();
    float resume_sp = base_sp;
    /* Deliberately no EPR startup hold or startup grace here. Both belong to
     * therapy start, where they park the blower at CPAP - level until the
     * circuit pressurises. Applied on mask-on they held the resumed pressure a
     * full relief level below set CPAP for 5 s, so the recovery looked stalled.
     * A reconnect resumes therapy that was already running: only the breath
     * state needs clearing (below). */

    DBG_I("THR", "mask on: resume pressure control -> target %.1f",
          (double)resume_sp);

    Leak_Reseed();
    Apap_OnMaskReconnect();
    Epr_ResetBreathState();

    /* Never STOP here — a STOP+SetSpeed race leaves the rotor coasting to 0. */
    BlowerCtrl_ClearManualFlag();

    const float hold_rpm = (float)clampMaskOffRpm(g_therapy_mask_off_rpm);
    const float ss_rpm = BlowerCtrl_EstimateSteadyStateRpm(resume_sp);
    float seed_rpm = ss_rpm;
    if (seed_rpm < hold_rpm)
    {
        seed_rpm = hold_rpm;
    }

    /* Assertive resume: keep PWM armed at ≥ hold RPM while pressure rebuilds. */
    Safety_Supervisor_ResetFaultTimers();

    BlowerCtrl_SetEnable(true);
    BlowerCtrl_SnapSetpoint(resume_sp);
    BlowerCtrl_ReseedForTherapyResume(resume_sp);
    BlowerCtrl_SetResumeRpmFloor(hold_rpm, 5000U);
    /* Vivo MeanRatio reseed attributes hose-fill flow to Q_patient briefly. */
    BlowerCtrl_SuppressPatientFlowFf(3000U);
    {
        int32_t tgt = (int32_t)lrintf(seed_rpm);
        BlowerIpcStatus_t st;
        (void)BlowerIpc_CM7_GetStatus(&st);
        int32_t delta = tgt - st.mech_speed_rpm;
        if (delta < 0)
        {
            delta = -delta;
        }
        /* Prefer a brisk climb back to therapy speed (accel ≠ regen OV). */
        uint32_t ramp = 500U;
        if (!st.motor_running || (st.mech_speed_rpm < 1500))
        {
            ramp = 1200U; /* soft re-accel from near-stop / after OV */
        }
        const uint32_t from_delta = (uint32_t)((delta * 1000) / 4000);
        if (from_delta > ramp)
        {
            ramp = from_delta;
        }
        if (ramp > 3000U)
        {
            ramp = 3000U;
        }
        /* ONE IPC write — CM4 SET_SPEED auto ack/start when idle/fault-cleared. */
        (void)BlowerIpc_CM7_SetSpeedRpm(tgt, (uint16_t)ramp);
    }

    s_mask_off_resume_boost_ms = 5000U;
    s_mask_off_resume_repost_ms = 0U;
    s_mask_off_verify_ms = g_therapy_mask_off_verify_ms;
    g_therapy_mask_off_reconn_flow_ok  = 0U;
    g_therapy_mask_off_reconn_press_ok = 0U;
}

/* Keep commanding therapy pressure after mask-on so RPM cannot linger at
 * hold speed if the first IPC set was lost or the PID seed was soft. */
static void maskOffResumeBoost(float dt_s, uint32_t dt_ms)
{
    if (s_mask_off_resume_boost_ms == 0U)
    {
        return;
    }

    if (dt_ms >= s_mask_off_resume_boost_ms)
    {
        s_mask_off_resume_boost_ms = 0U;
    }
    else
    {
        s_mask_off_resume_boost_ms -= dt_ms;
    }

    if (!BlowerCtrl_IsEnabled())
    {
        BlowerCtrl_SetEnable(true);
    }

    /* Keep full target during resume so pressure actually returns to set CPAP.
     * Therapy_Update() re-applies the setpoint right after this call, so both
     * paths have to agree on that - hence the shared helper. */
    const float base_sp = therapyBaseSetpointCmh2o();
    const float sp = resumeAwareSetpointCmh2o(base_sp, dt_s);
    BlowerCtrl_SnapSetpoint(sp);

    /* Only re-seed IPC if collapsing toward stop or stuck near hold. */
    s_mask_off_resume_repost_ms += dt_ms;
    if (s_mask_off_resume_repost_ms >= 250U)
    {
        s_mask_off_resume_repost_ms = 0U;
        const float meas = BlowerCtrl_GetMeasured();
        BlowerIpcStatus_t st;
        (void)BlowerIpc_CM7_GetStatus(&st);
        const int32_t hold = clampMaskOffRpm(g_therapy_mask_off_rpm);
        const int32_t ss = (int32_t)lrintf(BlowerCtrl_EstimateSteadyStateRpm(sp));
        const bool collapsing = (!st.motor_running)
            || (st.mech_speed_rpm < 2000)
            || (st.current_faults != 0U);
        const bool stuck_low = (meas < (sp - 1.0f))
            && (st.mech_speed_rpm < (hold + 800));
        if (collapsing || stuck_low)
        {
            int32_t rpm = ss;
            if (rpm < hold)
            {
                rpm = hold;
            }
            const uint16_t ramp = collapsing ? 1000U : 400U;
            (void)BlowerIpc_CM7_SetSpeedRpm(rpm, ramp);
        }
    }
}

static bool maskOffResumeBoostActive(void)
{
    return (s_mask_off_resume_boost_ms > 0U);
}

/* Returns true when mask-off is active and normal RAMP/RUNNING work should
 * be skipped for this tick. */
static bool updateMaskOff(float dt_s, uint32_t dt_ms)
{
    if ((g_therapy_mask_off_enabled == 0U)
        || ((g_therapy_state != (uint8_t)THERAPY_STATE_RAMP)
            && (g_therapy_state != (uint8_t)THERAPY_STATE_RUNNING)))
    {
        if (s_mask_off_active)
        {
            exitMaskOff(dt_s);
        }
        return false;
    }

    if (!s_mask_off_active)
    {
        /* Probation: did the circuit actually pressurise after the last clear?
         * Checked before the enter paths because this is a verdict on a
         * decision already made, not new evidence. */
        if (s_mask_off_verify_ms > 0U)
        {
            if (dt_ms >= s_mask_off_verify_ms)
            {
                s_mask_off_verify_ms = 0U;
                const float base_sp = therapyBaseSetpointCmh2o();
                const float frac = clampf(g_therapy_mask_off_verify_frac,
                                          0.20f, 0.90f);
                if (BlowerCtrl_GetMeasured() < (base_sp * frac))
                {
                    g_therapy_mask_off_false_clear_count++;
                    s_mask_off_clear_extra_ms +=
                        g_therapy_mask_off_clear_backoff_ms;
                    /* Our own re-entry debounce must not block this. */
                    s_reconnect_grace_until_ms = 0U;
                    DBG_W("THR",
                          "mask on was wrong: %.1f of %.1f after %lu ms — "
                          "re-park, clear confirm +%lu ms",
                          (double)BlowerCtrl_GetMeasured(), (double)base_sp,
                          (unsigned long)g_therapy_mask_off_verify_ms,
                          (unsigned long)s_mask_off_clear_extra_ms);
                    enterMaskOff(0U);   /* probation verdict, not a new detect */
                    return true;
                }
                /* Held the target: that reconnect was genuine. */
                s_mask_off_clear_extra_ms = 0U;
            }
            else
            {
                s_mask_off_verify_ms -= dt_ms;
            }
        }

        const bool fast_cond  = maskOffEnterFastCondition();
        const bool ratio_cond = maskOffEnterRatioCondition();
        const bool press_cond = maskOffEnterPressureOnlyCondition();
        const bool rpm_cond   = maskOffEnterRpmRatioCondition();
        const bool sust_cond  = maskOffEnterSustainedCondition(dt_ms);
        const float q_abs = fabsf(maskOffTotalFlowSlm());
        const float enter = maskOffEnterThresholdSlm();
        const float min_frac = clampf(g_therapy_mask_off_enter_min_frac, 0.15f, 0.90f);

        if (fast_cond)
        {
            if (s_mask_off_fast_enter_ms == 0U)
            {
                s_mask_off_arm_excess_min = q_abs;
            }
            else if (q_abs < s_mask_off_arm_excess_min)
            {
                s_mask_off_arm_excess_min = q_abs;
            }
            s_mask_off_fast_enter_ms += dt_ms;
        }
        else
        {
            s_mask_off_fast_enter_ms = 0U;
            s_mask_off_arm_excess_min = 0.0f;
            g_therapy_mask_off_diag_min = 0U;
        }

        if (ratio_cond)
        {
            s_mask_off_ratio_enter_ms += dt_ms;
        }
        else
        {
            s_mask_off_ratio_enter_ms = 0U;
        }

        if (press_cond)
        {
            s_mask_off_press_only_ms += dt_ms;
        }
        else
        {
            s_mask_off_press_only_ms = 0U;
        }

        if (rpm_cond)
        {
            s_mask_off_rpm_ratio_ms += dt_ms;
        }
        else
        {
            s_mask_off_rpm_ratio_ms = 0U;
        }

        if (sust_cond)
        {
            s_mask_off_sustained_ms += dt_ms;
        }
        else
        {
            s_mask_off_sustained_ms = 0U;
        }

        maskOffRatioEnterHysteresis();

        const uint32_t fast_need_nom = (g_therapy_mask_off_enter_ms > 0U)
                                         ? g_therapy_mask_off_enter_ms : 1U;
        /* Strong open-hose: high |Q| + FLOORED P → shorter confirm (~0.3 s).
         *
         * This used to accept the relative pressure test (diag_press, i.e. P
         * more than press_drop below setpoint). At 18 cmH2O that is satisfied
         * by the ordinary inspiratory sag, and tube compensation makes it
         * worse: P_mask sits ~1.3 cmH2O under P_sensor at peak flow because it
         * subtracts hose drop, so on 2026-09-23 one false trigger fired on
         * P_mask 15.99 while P_sensor never left 16.86 and would not have
         * armed it at all.
         *
         * With the confirm cut to 300 ms, two breaths cleared it - the gate
         * held 320 ms and 380 ms. Neither would have reached the nominal
         * 600 ms. The shortcut, not the threshold, is what fired.
         *
         * The absolute floor is the actual open-hose signature: an open
         * circuit cannot build pressure at all (measured 0.33 cmH2O on a real
         * disconnect), whereas a deep inhale sags a couple of cmH2O and
         * recovers. A genuine disconnect still gets the fast confirm. */
        const bool strong_open = fast_cond
            && (q_abs >= (enter * 1.25f))
            && maskOffPressureFloored();
        const uint32_t fast_need = strong_open
            ? ((fast_need_nom > 300U) ? 300U : fast_need_nom)
            : fast_need_nom;
        const uint32_t ratio_need = (g_therapy_mask_off_ratio_enter_ms > 0U)
                                      ? g_therapy_mask_off_ratio_enter_ms : 8000U;

        const bool min_ok = (s_mask_off_fast_enter_ms > 0U)
            && (s_mask_off_arm_excess_min >= (enter * min_frac));
        g_therapy_mask_off_diag_min = min_ok ? 1U : 0U;

        const bool fast_ready  = fast_cond
                              && (s_mask_off_fast_enter_ms >= fast_need)
                              && min_ok;
        const bool ratio_ready = ratio_cond && (s_mask_off_ratio_enter_ms >= ratio_need);

        /* Clamped below the supervisor's low-pressure timeout (4 s by default)
         * so the park always lands before the fault would. */
        uint32_t press_need = (g_therapy_mask_off_press_only_ms > 0U)
                                ? g_therapy_mask_off_press_only_ms : 1U;
        if (press_need < 500U)  { press_need = 500U; }
        if (press_need > 3500U) { press_need = 3500U; }
        const bool press_ready = press_cond
                              && (s_mask_off_press_only_ms >= press_need);

        uint32_t rpm_need = (g_therapy_mask_off_rpm_ratio_ms > 0U)
                              ? g_therapy_mask_off_rpm_ratio_ms : 1U;
        if (rpm_need < 1000U)  { rpm_need = 1000U; }
        if (rpm_need > 10000U) { rpm_need = 10000U; }
        const bool rpm_ready = rpm_cond
                            && (s_mask_off_rpm_ratio_ms >= rpm_need);

        uint32_t sust_need = (g_therapy_mask_off_sustained_ms > 0U)
                               ? g_therapy_mask_off_sustained_ms : 1U;
        if (sust_need < 200U)  { sust_need = 200U; }
        if (sust_need > 5000U) { sust_need = 5000U; }
        const bool sust_ready = sust_cond
                             && (s_mask_off_sustained_ms >= sust_need);

        if (fast_ready || ratio_ready || press_ready || rpm_ready || sust_ready)
        {
            g_therapy_mask_off_enter_via_ratio =
                (ratio_ready && !fast_ready && !press_ready && !rpm_ready && !sust_ready)
                  ? 1U : 0U;
            g_therapy_mask_off_enter_via_press =
                (press_ready && !fast_ready) ? 1U : 0U;
            g_therapy_mask_off_enter_via_rpm =
                (rpm_ready && !fast_ready && !press_ready) ? 1U : 0U;
            g_therapy_mask_off_enter_via_sustained =
                (sust_ready && !fast_ready && !press_ready && !rpm_ready) ? 1U : 0U;
            /* The fast and sustained paths saw the disconnect in the flow signal. */
            s_mask_off_flow_blind_entry = !(fast_ready || sust_ready);
            if (g_therapy_mask_off_enter_via_sustained != 0U)
            {
                /* M line q_excess field carries the 3 s mean for this path */
                s_mask_off_arm_excess_min = g_therapy_mask_off_sustained_mean_slm;
            }
            /* Same precedence the via_* flags above encode, as one value. */
            enterMaskOff(fast_ready  ? 1U :
                         press_ready ? 3U :
                         rpm_ready   ? 4U :
                         sust_ready  ? 5U : 2U);
            s_mask_off_fast_enter_ms = 0U;
            s_mask_off_ratio_enter_ms = 0U;
            s_mask_off_press_only_ms = 0U;
            s_mask_off_rpm_ratio_ms = 0U;
            s_mask_off_arm_excess_min = 0.0f;
            maskOffSustainedReset();
        }

        return false;
    }

    /* Soft-ramp to hold RPM — track open-hose pressure min. Never clear on
     * absolute P here: residual therapy pressure after disconnect looks
     * sealed and aborts the 6 kRPM park (RPM then runs away open-hose). */
    if (s_mask_off_ramp_left_ms > 0U)
    {
        s_mask_off_active_ms += dt_ms;
        {
            const float p_sensor = BlowerCtrl_GetSensorCmh2o();
            if (!s_mask_off_p_baseline_valid)
            {
                s_mask_off_p_baseline = p_sensor;
                s_mask_off_p_baseline_valid = true;
            }
            else if (p_sensor < s_mask_off_p_baseline)
            {
                s_mask_off_p_baseline = p_sensor;
            }
        }

        /* True loss of control only — never re-post while still decelerating. */
        {
            BlowerIpcStatus_t st;
            (void)BlowerIpc_CM7_GetStatus(&st);
            const int32_t hold = clampMaskOffRpm(g_therapy_mask_off_rpm);
            const bool lost = (!st.motor_running)
                || (st.current_faults != 0U)
                || (st.mech_speed_rpm < (hold / 2));
            if (lost)
            {
                s_mask_off_ramp_left_ms = maskOffApplyHoldRpm(true);
                s_mask_off_clear_ms = 0U;
                return true;
            }
        }

        if (dt_ms >= s_mask_off_ramp_left_ms)
        {
            s_mask_off_ramp_left_ms = 0U;
            s_mask_off_hold_ms = 0U;
            s_mask_off_hold_elapsed_ms = 0U;
            /* Keep open-hose P baseline from the ramp. Reset Q peak only so
             * hold-RPM open flow can be learned if the hose is still open. */
            s_mask_off_q_peak_slm = 0.0f;
            (void)maskOffApplyHoldRpm(false);
        }
        else
        {
            s_mask_off_ramp_left_ms -= dt_ms;
        }
        s_mask_off_clear_ms = 0U;
        return true;
    }

    /* Hold low RPM; learn open-hose Q only if still open. */
    s_mask_off_hold_ms += dt_ms;
    s_mask_off_active_ms += dt_ms;
    s_mask_off_hold_elapsed_ms += dt_ms;

    const uint32_t min_dwell = (g_therapy_mask_off_min_dwell_ms > 0U)
                                 ? g_therapy_mask_off_min_dwell_ms : 1U;
    const uint32_t learn_ms  = (g_therapy_mask_off_baseline_ms > 0U)
                                 ? g_therapy_mask_off_baseline_ms : 1U;

    maskOffTrackOpenCircuit(BlowerCtrl_GetSensorCmh2o(), maskOffTotalFlowSlm(),
                            (s_mask_off_hold_elapsed_ms <= learn_ms));
    if (s_mask_off_hold_ms >= 750U)
    {
        s_mask_off_hold_ms = 0U;
        (void)maskOffApplyHoldRpm(false);
    }

    if (s_mask_off_hold_elapsed_ms < min_dwell)
    {
        s_mask_off_clear_ms = 0U;
        s_mask_off_flow_clear_ms = 0U;
        return true;
    }
    if (s_mask_off_hold_elapsed_ms < learn_ms)
    {
        s_mask_off_clear_ms = 0U;
        s_mask_off_flow_clear_ms = 0U;
        return true;
    }

    /* Confirm times grow after a clear that did not hold, and are forgiven by
     * the first one that does. Capped so the park can always be left. */
    const uint32_t clear_cap = (g_therapy_mask_off_clear_ms_max > 0U)
                                 ? g_therapy_mask_off_clear_ms_max : 1U;

    if (maskOffClearCondition())
    {
        s_mask_off_clear_ms += dt_ms;
        uint32_t need = (g_therapy_mask_off_clear_ms > 0U)
                          ? g_therapy_mask_off_clear_ms : 1U;
        need += s_mask_off_clear_extra_ms;
        if (need > clear_cap) { need = clear_cap; }
        if (s_mask_off_clear_ms >= need)
        {
            exitMaskOff(dt_s);
            return false;
        }
    }
    else
    {
        s_mask_off_clear_ms = 0U;
    }

    /* Escape hatch 1: no pressure tip can appear when the mask went back on
     * before the decel finished - the reference is then the sealed hold
     * pressure itself. A sustained flow collapse still says sealed, so resume
     * on that alone. If the circuit is in fact still open, mask-off re-arms
     * about a second later, which is survivable; staying parked was not. */
    if (maskOffFlowOnlyClearCondition())
    {
        s_mask_off_flow_clear_ms += dt_ms;
        uint32_t flow_need = (g_therapy_mask_off_flow_clear_ms > 0U)
                               ? g_therapy_mask_off_flow_clear_ms : 1U;
        flow_need += s_mask_off_clear_extra_ms;
        if (flow_need > clear_cap) { flow_need = clear_cap; }
        if (s_mask_off_flow_clear_ms >= flow_need)
        {
            DBG_I("THR", "mask on: flow-only clear |Q|=%.1f peak=%.1f",
                  (double)fabsf(maskOffTotalFlowSlm()),
                  (double)s_mask_off_q_peak_slm);
            exitMaskOff(dt_s);
            return false;
        }
    }
    else
    {
        s_mask_off_flow_clear_ms = 0U;
    }

    /* Escape hatch 2: never stay parked for the rest of the session. Neither
     * detector can be trusted if the pressure zero has drifted, so re-probe at
     * therapy pressure and let the enter path judge the circuit again. */
    if ((g_therapy_mask_off_max_hold_ms > 0U)
        && (s_mask_off_hold_elapsed_ms >= g_therapy_mask_off_max_hold_ms))
    {
        DBG_W("THR", "mask off: hold watchdog %lu ms, re-probing",
              (unsigned long)s_mask_off_hold_elapsed_ms);
        exitMaskOff(dt_s);
        return false;
    }

    return true;
}

/* Latch a fault: log it, kill the blower, transition to FAULT. */
void Therapy_LatchFault(TherapyFault_t f, const char *why)
{
    if (g_therapy_fault != (uint8_t)THERAPY_FAULT_NONE)
    {
        return;  /* first fault wins */
    }

    DBG_E("THR", "FAULT %u: %s (rpm=%ld meas=%.2f sp=%.2f)",
          (unsigned)f, why,
          (long)BlowerCtrl_GetOutputRpm(),
          (double)g_therapy_measured_cmh2o,
          (double)g_therapy_setpoint_cmh2o);

    BlowerCtrl_SetEnable(false);
    (void)BlowerIpc_CM7_Stop();

    g_therapy_fault = (uint8_t)f;
    setState(THERAPY_STATE_FAULT);
}

void Therapy_Init(void)
{
    g_therapy_target_cmh2o    = THERAPY_TARGET_DEFAULT;
    g_therapy_apap_min_cmh2o  = 5.0f;
    g_therapy_apap_max_cmh2o  = 20.0f;
    g_therapy_ramp_minutes    = 0U;
    g_therapy_mode            = (uint8_t)THERAPY_MODE_CPAP;
    g_therapy_state           = (uint8_t)THERAPY_STATE_IDLE;
    g_therapy_setpoint_cmh2o  = 0.0f;
    g_therapy_measured_cmh2o  = 0.0f;
    g_therapy_leak_slm        = 0.0f;
    g_therapy_run_ms          = 0U;
    g_therapy_ramp_total_ms   = 0U;
    g_therapy_ramp_elapsed_ms = 0U;
    g_therapy_stop_elapsed_ms = 0U;
    sessionPressureReset();

    g_therapy_fault           = (uint8_t)THERAPY_FAULT_NONE;
    s_mask_off_active         = false;
    g_therapy_mask_off_active = 0U;
    s_mask_off_fast_enter_ms  = 0U;
    s_mask_off_ratio_enter_ms = 0U;
    s_mask_off_clear_ms       = 0U;
    s_mask_off_hold_ms        = 0U;
    s_mask_off_active_ms      = 0U;
    s_mask_off_hold_elapsed_ms = 0U;
    s_mask_off_p_baseline_valid = false;
    s_mask_off_q_peak_slm = 0.0f;
    s_mask_off_arm_excess_min = 0.0f;
    s_mask_off_prev_breath_insp = 0U;
    s_mask_off_last_breath_edge_ms = 0U;
    s_mask_off_resume_boost_ms = 0U;
    s_mask_off_resume_repost_ms = 0U;
    s_reconnect_grace_until_ms = 0U;
    s_safety_reconnect_grace_until_ms = 0U;

    Safety_Supervisor_Init();

    Epr_Init();
    Apap_Init();
    SessionEdf_Init();
}

void Therapy_SetTargetPressure(float cmh2o)
{
    g_therapy_target_cmh2o = clampf(cmh2o,
                                    THERAPY_PRESS_MIN_CMH2O,
                                    THERAPY_PRESS_MAX_CMH2O);
}

float Therapy_GetTargetPressure(void) { return g_therapy_target_cmh2o; }

void Therapy_SetRampMinutes(uint8_t minutes)
{
    if (minutes > THERAPY_RAMP_MAX_MINUTES) minutes = THERAPY_RAMP_MAX_MINUTES;
    g_therapy_ramp_minutes = minutes;
}

uint8_t Therapy_GetRampMinutes(void) { return g_therapy_ramp_minutes; }

static void therapyNormalizeApapRange(void)
{
    float min_p = clampf(g_therapy_apap_min_cmh2o,
                         THERAPY_PRESS_MIN_CMH2O,
                         THERAPY_PRESS_MAX_CMH2O);
    float max_p = clampf(g_therapy_apap_max_cmh2o,
                         THERAPY_PRESS_MIN_CMH2O,
                         THERAPY_PRESS_MAX_CMH2O);
    if (max_p < (min_p + 0.5f))
    {
        max_p = min_p + 0.5f;
        if (max_p > THERAPY_PRESS_MAX_CMH2O)
        {
            max_p = THERAPY_PRESS_MAX_CMH2O;
            min_p = max_p - 0.5f;
        }
    }
    g_therapy_apap_min_cmh2o = min_p;
    g_therapy_apap_max_cmh2o = max_p;
}

void Therapy_SetMode(TherapyMode_t m)
{
    g_therapy_mode = (uint8_t)m;
    BlowerCtrl_SyncCpapProfilesForMode((uint8_t)m);
}

TherapyMode_t Therapy_GetMode(void)   { return (TherapyMode_t)g_therapy_mode; }

const char* Therapy_GetModeName(void)
{
    switch ((TherapyMode_t)g_therapy_mode)
    {
    case THERAPY_MODE_CPAP:             return "CPAP";
    case THERAPY_MODE_AUTOSET:          return "Autoset";
    case THERAPY_MODE_AUTOSET_FOR_HER:  return "Autoset for her";
    default:                            return "?";
    }
}

void Therapy_SetApapMinPressure(float cmh2o)
{
    g_therapy_apap_min_cmh2o = clampf(cmh2o,
                                      THERAPY_PRESS_MIN_CMH2O,
                                      THERAPY_PRESS_MAX_CMH2O);
    therapyNormalizeApapRange();
}

void Therapy_SetApapMaxPressure(float cmh2o)
{
    g_therapy_apap_max_cmh2o = clampf(cmh2o,
                                      THERAPY_PRESS_MIN_CMH2O,
                                      THERAPY_PRESS_MAX_CMH2O);
    therapyNormalizeApapRange();
}

float Therapy_GetApapMinPressure(void) { return g_therapy_apap_min_cmh2o; }
float Therapy_GetApapMaxPressure(void) { return g_therapy_apap_max_cmh2o; }

const char* Therapy_GetApapStateName(void)
{
    /* AutoSet, or CPAP with event scoring (#33). */
    if (Apap_GetState() == APAP_STATE_IDLE)
    {
        return "";
    }
    return Apap_GetStateName();
}

TherapyState_t Therapy_GetState(void) { return (TherapyState_t)g_therapy_state; }

const char* Therapy_GetStateName(void)
{
    switch ((TherapyState_t)g_therapy_state)
    {
    case THERAPY_STATE_IDLE:      return "Idle";
    case THERAPY_STATE_RAMP:      return "Ramp";
    case THERAPY_STATE_RUNNING:   return "Running";
    case THERAPY_STATE_STOPPING:  return "Stopping";
    case THERAPY_STATE_FAULT:     return "Fault";
    default:                      return "?";
    }
}

TherapyFault_t Therapy_GetFault(void) { return (TherapyFault_t)g_therapy_fault; }

const char* Therapy_GetFaultName(void)
{
    switch ((TherapyFault_t)g_therapy_fault)
    {
    case THERAPY_FAULT_NONE:                 return "OK";
    case THERAPY_FAULT_LOW_PRESS_AT_HIGH_RPM:return "Sensor/Hose Lost";
    case THERAPY_FAULT_OVER_PRESSURE:        return "Over Pressure";
    case THERAPY_FAULT_SENSOR_OUT_OF_RANGE:  return "Sensor Out Of Range";
    case THERAPY_FAULT_RPM_SATURATED:        return "Cannot Reach Target";
    case THERAPY_FAULT_CM4_HEARTBEAT:        return "CM4 Heartbeat Lost";
    default:                                  return "?";
    }
}

void Therapy_ClearFault(void)
{
    g_therapy_fault = (uint8_t)THERAPY_FAULT_NONE;
    Safety_Supervisor_ResetFaultTimers();
    if (g_therapy_state == (uint8_t)THERAPY_STATE_FAULT)
    {
        setState(THERAPY_STATE_IDLE);
    }
}

float    Therapy_GetSetpointCmh2o(void) { return g_therapy_setpoint_cmh2o; }
float    Therapy_GetMeasuredCmh2o(void) { return g_therapy_measured_cmh2o; }
float    Therapy_GetLeakSlm(void)       { return g_therapy_leak_slm; }
bool     Therapy_IsLeakHigh(void)       { return Leak_IsHigh(); }
bool     Therapy_IsLeakLow(void)        { return Leak_IsLow(); }
LeakSeverity_t Therapy_GetLeakSeverity(void) { return Leak_GetSeverity(); }
const char* Therapy_GetLeakSeverityName(void) { return Leak_GetSeverityName(); }
float    Therapy_GetLeakPercentOfVent(void) { return Leak_GetPercentOfVent(); }
uint32_t Therapy_GetMaskOffEnterCount(void) { return g_therapy_mask_off_enter_count; }
uint32_t Therapy_GetMaskOnCount(void)       { return g_therapy_mask_on_count; }
uint32_t Therapy_GetRunSeconds(void)    { return g_therapy_run_ms / 1000U; }
uint32_t Therapy_GetSessionMaskOffSeconds(void) { return g_therapy_session_mask_off_ms / 1000U; }

float Therapy_GetSessionPressureAvg(void)
{
    sessionPressureRefreshCaches();
    return g_therapy_session_p_avg;
}
float Therapy_GetSessionPressureMin(void)
{
    sessionPressureRefreshCaches();
    return g_therapy_session_p_min;
}
float Therapy_GetSessionPressureMax(void)
{
    sessionPressureRefreshCaches();
    return g_therapy_session_p_max;
}
float Therapy_GetSessionPressureP50(void)
{
    sessionPressureRefreshCaches();
    return g_therapy_session_p50;
}
float Therapy_GetSessionPressureP90(void)
{
    sessionPressureRefreshCaches();
    return g_therapy_session_p90;
}
float Therapy_GetSessionPressureP95(void)
{
    sessionPressureRefreshCaches();
    return g_therapy_session_p95;
}
uint32_t Therapy_GetSessionPressureSamples(void)
{
    return s_p_hist_count;
}

uint32_t Therapy_GetRampRemainingSeconds(void)
{
    if (g_therapy_state != (uint8_t)THERAPY_STATE_RAMP) return 0U;
    if (g_therapy_ramp_elapsed_ms >= g_therapy_ramp_total_ms) return 0U;
    return (g_therapy_ramp_total_ms - g_therapy_ramp_elapsed_ms) / 1000U;
}

void Therapy_SetEprEnabled(bool en)
{
    const bool was_enabled = Epr_IsEnabled();
    s_epr_cfg_enable = en ? 1U : 0U;
    Epr_SetEnable(en);

    if (!en || was_enabled)
    {
        return;
    }

    const TherapyState_t st = (TherapyState_t)g_therapy_state;
    if ((st != THERAPY_STATE_RAMP) && (st != THERAPY_STATE_RUNNING))
    {
        return;
    }

    applyStoredEprConfig();
    Epr_OnTherapyStart();
    applyEprStartupSnap(therapyBaseSetpointCmh2o());
}

bool Therapy_IsEprEnabled(void)
{
    return Epr_IsEnabled();
}

void Therapy_SetEprType(uint8_t type)
{
    s_epr_cfg_type = (type == 1U) ? 1U : 0U;
    Epr_SetType((type == 1U) ? EPR_TYPE_RAMP_ONLY : EPR_TYPE_FULL_TIME);
}

uint8_t Therapy_GetEprType(void)
{
    return (Epr_GetType() == EPR_TYPE_RAMP_ONLY) ? 1U : 0U;
}

void Therapy_SetEprLevel(uint8_t level)
{
    if (level < 1U) { level = 1U; }
    if (level > 3U) { level = 3U; }
    s_epr_cfg_level = level;
    Epr_SetLevel(level);
}

uint8_t Therapy_GetEprLevel(void)
{
    return Epr_GetLevel();
}

void Therapy_SetEprReliefCmh2o(float cmh2o)
{
    uint8_t level = (uint8_t)lrintf(cmh2o);
    if (level < 1U) { level = 1U; }
    if (level > 3U) { level = 3U; }
    s_epr_cfg_level = level;
    Epr_SetReliefCmh2o(cmh2o);
}

float Therapy_GetEprReliefCmh2o(void)
{
    return Epr_GetReliefCmh2o();
}

bool Therapy_IsMaskOffActive(void)
{
    return s_mask_off_active;
}

void Therapy_SetMaskOffEnabled(bool en)
{
    g_therapy_mask_off_enabled = en ? 1U : 0U;
    if (!en && s_mask_off_active)
    {
        exitMaskOff(0.005f);
    }
}

bool Therapy_IsMaskOffArming(void)
{
    /* Any enter path currently counting toward its confirm time. The safety
     * supervisor holds its pressure fault timers while this is set, so a
     * disconnect that is being detected cannot be pre-empted by a fault that
     * started counting earlier - see the note in runPressureChecks(). */
    return (s_mask_off_fast_enter_ms > 0U)
        || (s_mask_off_ratio_enter_ms > 0U)
        || (s_mask_off_press_only_ms > 0U)
        || (s_mask_off_rpm_ratio_ms > 0U)
        || (s_mask_off_sustained_ms > 0U);
}

bool Therapy_IsMaskOffEnabled(void)
{
    return (g_therapy_mask_off_enabled != 0U);
}

void Therapy_ForceMaskOn(void)
{
    if (s_mask_off_active)
    {
        exitMaskOff(0.005f);
    }
}

void Therapy_Start(void)
{
    /* A pending FAULT must be cleared before we'll re-arm the blower. */
    if (g_therapy_fault != (uint8_t)THERAPY_FAULT_NONE)
    {
        Therapy_ClearFault();
    }

    /* Re-apply the safety RPM ceiling in case it was poked. */
    extern volatile int32_t g_blower_ctrl_rpm_max;
    g_blower_ctrl_rpm_max = g_safety_safe_rpm_max;

    /* Snap a self-consistent set of session parameters now, so changes
     * the GUI makes during therapy don't yank the running ramp around. */
    if (therapyAutosetActive())
    {
        Apap_OnTherapyStart(g_therapy_apap_min_cmh2o,
                            g_therapy_apap_max_cmh2o,
                            g_therapy_mode);
        s_session_target_cmh2o = Apap_GetTargetPressure();
        g_therapy_target_cmh2o = s_session_target_cmh2o;
    }
    else
    {
        s_session_target_cmh2o   = clampf(g_therapy_target_cmh2o,
                                          THERAPY_PRESS_MIN_CMH2O,
                                          THERAPY_PRESS_MAX_CMH2O);
        /* #33: event scoring at the fixed pressure (no titration). */
        Apap_OnTherapyStart(s_session_target_cmh2o, s_session_target_cmh2o,
                            g_therapy_mode);
    }
    g_therapy_ramp_total_ms  = (uint32_t)g_therapy_ramp_minutes * 60U * 1000U;
    g_therapy_ramp_elapsed_ms = 0U;
    g_therapy_stop_elapsed_ms = 0U;
    g_therapy_run_ms         = 0U;
    g_therapy_session_mask_off_ms = 0U;
    sessionPressureReset();
    Safety_Supervisor_ResetFaultTimers();
    s_therapy_start_ms       = HAL_GetTick();
    s_reconnect_grace_until_ms = 0U;
    s_safety_reconnect_grace_until_ms = 0U;
    s_mask_off_active        = false;
    g_therapy_mask_off_active = 0U;
    s_mask_off_fast_enter_ms  = 0U;
    s_mask_off_ratio_enter_ms = 0U;
    s_mask_off_clear_ms      = 0U;
    s_mask_off_hold_ms       = 0U;
    s_mask_off_active_ms     = 0U;
    s_mask_off_ramp_left_ms  = 0U;
    s_mask_off_hold_elapsed_ms = 0U;
    s_mask_off_p_baseline_valid = false;
    s_mask_off_q_peak_slm = 0.0f;
    s_mask_off_arm_excess_min = 0.0f;
    s_mask_off_prev_breath_insp = 0U;
    s_mask_off_last_breath_edge_ms = 0U;
    s_mask_off_resume_boost_ms = 0U;
    s_mask_off_resume_repost_ms = 0U;
    s_mask_off_press_only_ms = 0U;
    s_mask_off_rpm_ratio_ms = 0U;
    maskOffSustainedReset();
    s_mask_off_flow_clear_ms = 0U;
    s_mask_off_flow_blind_entry = false;
    s_mask_off_hold_p_ref_valid = false;
    s_mask_off_verify_ms = 0U;
    s_mask_off_clear_extra_ms = 0U;
    g_therapy_mask_off_enter_count = 0U;
    g_therapy_mask_on_count = 0U;
    g_therapy_mask_off_false_clear_count = 0U;

    /* Configure the PID, then enable. Set therapy state first so EPR
     * logic (Full time / Ramp only) applies on the very first setpoint. */
    if (g_therapy_ramp_total_ms > 0U)
    {
        setState(THERAPY_STATE_RAMP);
    }
    else
    {
        setState(THERAPY_STATE_RUNNING);
    }

    Leak_SessionReset();
    Leak_SetSessionActive(true);
    Leak_Reseed();
    applyStoredEprConfig();
    Epr_OnTherapyStart();

    /* CPAP Plateau / Hybrid: auto-apply preferred profile on CPAP start. */
    BlowerCtrl_SyncCpapProfilesForMode(g_therapy_mode);

    if (BlowerCtrl_IsManualActive())
    {
        BlowerCtrl_ManualStop();
    }

    const float start_sp = (g_therapy_ramp_total_ms > 0U)
                              ? THERAPY_RAMP_START_CMH2O
                              : s_session_target_cmh2o;
    if (Epr_IsEnabled())
    {
        applyEprStartupSnap(start_sp);
    }
    else
    {
        applyTherapySetpoint(start_sp, 0.005f);
        BlowerCtrl_SnapSetpoint(BlowerCtrl_GetSetpoint());
    }
    BlowerCtrl_SetEnable(true);

    /* Clear any CM4 FOC latch (e.g. over-voltage from a prior hard
     * deceleration) before START — otherwise MC_StartMotor1() refuses. */
    (void)BlowerIpc_CM7_AcknowledgeFault();
    (void)BlowerIpc_CM7_Start();

    SessionEdf_OnTherapyStart();
    SessionSummary_OnTherapyStart();

    DBG_I("THR", "start target=%.1f epr=%u lvl=%u exp=%.1f cmH2O ramp=%u min",
          (double)s_session_target_cmh2o,
          (unsigned)Epr_IsEnabled(),
          (unsigned)s_epr_cfg_level,
          (double)Epr_GetExpiratorySetpointCmh2o(s_session_target_cmh2o,
                                                 g_therapy_state),
          (unsigned)g_therapy_ramp_minutes);
}

void Therapy_Stop(void)
{
    if (g_therapy_state == (uint8_t)THERAPY_STATE_IDLE)
    {
        return;
    }

    Apap_OnTherapyStop();
    Leak_SetSessionActive(false);
    Leak_Reseed(); /* drop leftover vent/leak LPF so idle Q_patient stays quiet */
    Flow_AutoZeroArm(); /* one quiet-mean slew after stop; never snap */
    SessionEdf_OnTherapyStop();

    /* Drop PID control immediately so the motor coasts down rather than
     * fighting to maintain pressure during spin-down. */
    if (s_mask_off_active)
    {
        s_mask_off_active = false;
        g_therapy_mask_off_active = 0U;
        s_mask_off_ramp_left_ms = 0U;
        Leak_SetDisconnectFreeze(false);
    }
    BlowerCtrl_SetEnable(false);
    (void)BlowerIpc_CM7_Stop();
    /* Leave FAULT_OVER so the next Start can arm the motor again. */
    (void)BlowerIpc_CM7_AcknowledgeFault();

    g_therapy_stop_elapsed_ms = 0U;
    /* If we're in FAULT, the supervisor latched the cause and the blower
     * is already off. A user Stop is the operator acknowledging the
     * fault, so go straight to STOPPING (which then transitions to IDLE
     * after the spin-down hold) and clear the fault flag. */
    if (g_therapy_state == (uint8_t)THERAPY_STATE_FAULT)
    {
        Therapy_ClearFault();
    }
    setState(THERAPY_STATE_STOPPING);

    sessionPressureRefreshCaches();
    SessionSummary_OnTherapyStop();   /* #57: snapshot after the refresh */
    DBG_I("THR", "stop after %lu s  P50=%.1f P90=%.1f P95=%.1f avg=%.1f",
          (unsigned long)Therapy_GetRunSeconds(),
          (double)g_therapy_session_p50,
          (double)g_therapy_session_p90,
          (double)g_therapy_session_p95,
          (double)g_therapy_session_p_avg);
}

void Therapy_Update(float dt_s)
{
    if (!(dt_s > 0.0f) || dt_s > 0.5f)
    {
        return;
    }

    const uint32_t dt_ms = (uint32_t)lrintf(dt_s * 1000.0f);

    /* Always refresh observable state (so the GUI can show current
     * values regardless of state). */
    g_therapy_measured_cmh2o = BlowerCtrl_GetMeasured();
    g_therapy_leak_slm       = Leak_GetUnintentionalSlm();

    /* Breath detection for EPR runs every therapy tick. */
    /* R7b: notch the oscillator frequency out of the breath detector's
     * flow while FOT / the physiology probe runs. */
    Epr_Update(Apap_FilterBreathFlow(Flow_GetPatientSlm(), dt_s),
               BlowerCtrl_GetMeasured(), therapyBaseSetpointCmh2o(), dt_s);
    SessionEdf_Update(dt_s);

    if (Therapy_GetFault() != THERAPY_FAULT_NONE)
    {
        g_therapy_setpoint_cmh2o = BlowerCtrl_GetSetpoint();
        return;
    }

    /* CM4 defers a start while the rotor may still be coasting from a stop
     * or a drive fault (restart_holdoff_ms, up to ~10 s from 40 krpm - see
     * blower_ipc_cm4.c). The low-pressure grace must not run out while the
     * blower is not yet allowed to turn: a therapy restart just after a stop
     * from 20 cmH2O is ~10 s of hold-off plus ~5 s of rev-up, past the 10 s
     * start grace, and the supervisor would latch LOW_PRESSURE on a start
     * that is working as designed. So while a hold-off is reported, keep the
     * full grace window open from now; it then runs from the moment the
     * start is actually released. Also keeps the pressure-only and rpm-ratio
     * mask-off paths (both gated on this grace) quiet with the motor off. */
    if ((g_therapy_state == (uint8_t)THERAPY_STATE_RAMP)
        || (g_therapy_state == (uint8_t)THERAPY_STATE_RUNNING))
    {
        BlowerIpcStatus_t st;
        if (BlowerIpc_CM7_GetStatus(&st) && (st.restart_holdoff_ms > 0U)
            && (g_safety_grace_s > 0.0f))
        {
            s_safety_reconnect_grace_until_ms =
                HAL_GetTick() + (uint32_t)(g_safety_grace_s * 1000.0f);
        }
    }

    switch ((TherapyState_t)g_therapy_state)
    {
    case THERAPY_STATE_IDLE:
    {
        /* Nothing to do; waiting for Therapy_Start(). */
        break;
    }

    case THERAPY_STATE_RAMP:
    {
        if (updateMaskOff(dt_s, dt_ms))
        {
            g_therapy_run_ms += dt_ms;
            g_therapy_session_mask_off_ms += dt_ms;
            break;
        }
        maskOffResumeBoost(dt_s, dt_ms);

        /* Linear ramp from THERAPY_RAMP_START_CMH2O to session target
         * over ramp_total_ms. The PID's internal slew limiter (15
         * cmH2O/s) is far faster than this slow ramp (~0.01 cmH2O/s
         * typical) so the per-tick step is, for all practical purposes,
         * a smooth continuous trajectory. */
        g_therapy_ramp_elapsed_ms += dt_ms;
        g_therapy_run_ms          += dt_ms;

        if (g_therapy_ramp_elapsed_ms >= g_therapy_ramp_total_ms)
        {
            applyTherapySetpoint(s_session_target_cmh2o, dt_s);
            setState(THERAPY_STATE_RUNNING);
        }
        else
        {
            const float frac = (float)g_therapy_ramp_elapsed_ms
                             / (float)g_therapy_ramp_total_ms;
            const float sp   = THERAPY_RAMP_START_CMH2O
                             + frac * (s_session_target_cmh2o
                                       - THERAPY_RAMP_START_CMH2O);
            applyTherapySetpoint(sp, dt_s);
        }
        sessionPressureSample(g_therapy_measured_cmh2o);
        break;
    }

    case THERAPY_STATE_RUNNING:
    {
        if (updateMaskOff(dt_s, dt_ms))
        {
            g_therapy_run_ms += dt_ms;
            g_therapy_session_mask_off_ms += dt_ms;
            break;
        }
        maskOffResumeBoost(dt_s, dt_ms);

        /* Always apply therapy setpoint while resuming — boost only snaps SP
         * and optionally kick-starts RPM if still parked at hold. */
        if (therapyAutosetActive())
        {
            const float apap_target = Apap_Update(dt_s,
                                                  g_therapy_state,
                                                  s_mask_off_active);
            s_session_target_cmh2o = apap_target;
            g_therapy_target_cmh2o   = apap_target;
            applyTherapySetpoint(s_session_target_cmh2o, dt_s);
        }
        else
        {
            /* Hold target. The user may have edited the target via the GUI
             * during therapy - re-apply each tick so the PID tracks. */
            const float live_target = clampf(g_therapy_target_cmh2o,
                                             THERAPY_PRESS_MIN_CMH2O,
                                             THERAPY_PRESS_MAX_CMH2O);
            if (fabsf(live_target - s_session_target_cmh2o) > 0.05f)
            {
                s_session_target_cmh2o = live_target;
            }
            /* #33: detectors only - the APAP target is ignored here. */
            (void)Apap_Update(dt_s, g_therapy_state, s_mask_off_active);
            applyTherapySetpoint(s_session_target_cmh2o, dt_s);
        }
        g_therapy_run_ms += dt_ms;
        sessionPressureSample(g_therapy_measured_cmh2o);
        break;
    }

    case THERAPY_STATE_STOPPING:
    {
        g_therapy_stop_elapsed_ms += dt_ms;
        if (g_therapy_stop_elapsed_ms
              >= (uint32_t)(THERAPY_STOP_DURATION_S * 1000.0f))
        {
            setState(THERAPY_STATE_IDLE);
        }
        break;
    }

    case THERAPY_STATE_FAULT:
    {
        /* No automatic recovery yet. User has to call Stop() then Start()
         * once the offending condition (motor fault, persistent high
         * leak) is cleared. */
        BlowerCtrl_SetEnable(false);
        break;
    }

    default:
        setState(THERAPY_STATE_IDLE);
        break;
    }

    g_therapy_setpoint_cmh2o = BlowerCtrl_GetSetpoint();
}
