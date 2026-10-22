/**
  ******************************************************************************
  * @file    blower_ctrl.c
  * @brief   Gain-scheduled pressure-control PID for the CPAP blower.
  *
  *  Design notes (read these before tuning)
  *  ---------------------------------------
  *  1. Units: setpoint and measurement are in cmH2O (the unit clinicians
  *     and the device GUI use). The mask pressure sensor (hamsPS) returns
  *     mbar; we convert via 1 cmH2O = 0.980665 mbar.
  *
  *  2. Three gain zones: LOW (4-8), MID (8-15), HIGH (15-25). At low
  *     therapy pressures the loop is highly sensitive (steep dP/dRPM), so
  *     gains are small. At high pressures gains rise to keep the time
  *     constant roughly uniform across the therapy range.
  *
 *  3. Feed-forward: Physics FF is the default (hose + fan load on Q).
 *     Sqrt FF and flow FF are off; they are mutually exclusive with Physics
 *     FF. Dip FF can still run. Plateau/Hybrid profiles override these.
 *
 *  8. WM6850 reference mode: set g_blower_ctrl_wm6850_mode=1 to run the
 *     pressure_pid.c-style loop at 100 Hz (every 2nd SensorTask tick, dt=10 ms;
 *     gains scaled for cmH2O units), no FF, no setpoint slew, sensor-side
 *     measurement, I starts at 0. Speed cap uses sqrt bench calibration
 *     ×1.3 (not the dev-board interp_occ_speed table, ~6 krpm max).
 *
 *  9. CPAP Plateau preset (g_blower_ctrl_cpap_plateau_prefer): AirSense-like
 *     flat pressure platform — sqrt FF only, flow/physics/dip FF off, soft
 *     under/over-P gains, heavier meas/out filters. Auto-applied on CPAP
 *     therapy start/mode; restored when switching to Autoset or prefer=0.
 *
 * 10. CPAP Hybrid (g_blower_ctrl_cpap_hybrid_prefer): Physics FF tracks flow
 *     on inspiration; expiration holds vent-floor RPM (plateau-like). Mutex
 *     with Plateau. Toggle off if A/B shows little benefit.
 *
  *  4. Anti-windup: classic conditional integration. We only accumulate
  *     the integrator when the output is NOT saturated, and when the
  *     accumulation would not push the output further into saturation.
  *
  *  5. Derivative on measurement (not error): prevents derivative kick
  *     when the user steps the setpoint. The setpoint slew limiter would
  *     also smooth a step but D-on-M is robust regardless.
  *
  *  6. Setpoint slew: the user-facing setpoint can step instantly (e.g. a
  *     ramp profile or pressure relief), but the *internal* setpoint that
  *     feeds the PID is slew-limited so we never command the loop to do
  *     something physically impossible.
  *
  *  7. Output post-filter: the raw PID output is mildly low-passed (1st
  *     order) to remove sensor noise before posting to the IPC. Disable
  *     by setting g_blower_ctrl_out_filter_alpha = 1.0f.
  *
  *  Default gains
  *  -------------
  *  These are CONSERVATIVE starting points. Expect to tune Kp/Ki up at
  *  least 2-4x once you see the actual mask + hose + leak response. Keep
  *  Kd small initially; aggressive D-term will amplify the 200 Hz sensor
  *  noise. The recommended bring-up procedure:
  *      a. Set ki=0, kd=0; raise kp until the response shows mild
  *         oscillation, then back off ~30%.
  *      b. Raise ki until steady-state error is removed within 200-500 ms.
  *      c. Add a tiny kd (5-30) only if you see overshoot you cannot
  *         dial out by reducing kp.
  ******************************************************************************
  */

#include "blower_ctrl.h"
#include "blower_ipc.h"
#include "blower_osc.h"
#include "pressure_sensors.h"
#include "flow_sensor.h"
#include "tube_comp.h"
#include "epr_ctrl.h"
#include "leak_estimator.h"
#include "therapy.h"

#include <math.h>
#include <stdint.h>

/* === External plumbing === */
extern AMS5935_Data_t measPS;     /* mask/patient pressure sensor (mbar) */

/* PS sample freshness, published by SensorTask in main.c. SPI4 is shared
 * with the ambient sensor so measPS is not refreshed every tick; both
 * pressure derivatives below are timed against the interval the sample
 * actually spans and held across stale ticks. */
extern volatile uint8_t g_ps_sample_fresh;
extern volatile float   g_ps_sample_dt_s;

/* === Conversion === */
#define MBAR_PER_CMH2O   0.980665f
#define MBAR_TO_CMH2O(x) ((x) * (1.0f / MBAR_PER_CMH2O))

/* === Tunable gains (volatile so they live in a debugger-writable global) ===
 *
 *  Default values were chosen ASSUMING the sqrt-based feed-forward below is
 *  active and well-calibrated. With FF carrying the steady-state RPM, the
 *  PID only needs to handle disturbances (leaks, mask resistance, breath
 *  cycle) so Kp can be modest and Ki can be small. If you turn FF off
 *  (g_blower_ctrl_ff_enable=0) you will need to re-tune Kp/Ki up by 2-3x.
 */
/* Low zone: 4 - 8 cmH2O.
 *
 * Tuned from open-loop FOPDT bench fit @ ~7.8 cmH2O (fopdt_fit.py, post-
 * reprofile motor): K=0.000934 cmH2O/RPM, tau=0.068 s, theta=0.009 s.
 * SIMC @ tau_c=0.60 s, gain-scheduled by K_sqrt/K at zone (LOW@8 fit).
 * Kp raised 1500 -> 1800 during the 2026-09 AirSense waveform match; the
 * other two zones were not re-tuned and are still at their SIMC values. */
volatile float g_blower_ctrl_kp_low  = 1800.0f;
volatile float g_blower_ctrl_ki_low  = 4500.0f;
volatile float g_blower_ctrl_kd_low  = 2.0f;

/* Mid zone: 8 - 15 cmH2O (SIMC gain-scheduled @ MID@12). */
volatile float g_blower_ctrl_kp_mid  = 1200.0f;
volatile float g_blower_ctrl_ki_mid  = 4200.0f;
volatile float g_blower_ctrl_kd_mid  = 1.5f;

/* High zone: 15 - 25 cmH2O (SIMC gain-scheduled @ HIGH@20). */
volatile float g_blower_ctrl_kp_high = 1000.0f;
volatile float g_blower_ctrl_ki_high = 3900.0f;
volatile float g_blower_ctrl_kd_high = 0.8f;

/* WM6850 pressure_pid.c reference mode (see design note 8 above).
 * Gains mapped from fKP=1.2, fKI=26, fKD=0.06 with pressure in cmH2O×100
 * @ 100 Hz -> cmH2O @ 200 Hz (Kp/Ki scale; Kd ×100 for derivative units). */
volatile uint8_t g_blower_ctrl_wm6850_mode             = 0U;
volatile float   g_blower_ctrl_wm6850_kp               = 250.0f;
volatile float   g_blower_ctrl_wm6850_ki               = 3400.0f;
volatile float   g_blower_ctrl_wm6850_kd               = 1.0f;
volatile float   g_blower_ctrl_wm6850_meas_alpha       = 0.50f;
volatile float   g_blower_ctrl_wm6850_deriv_alpha      = 0.20f;
volatile float   g_blower_ctrl_wm6850_speed_lim_scale  = 1.30f;

/* === Feed-forward (open-loop steady-state RPM model) =======================
 *
 *  Centrifugal blowers obey P ~ K * w^2, so the inverse mapping that gives
 *  us "the RPM needed to hold a given pressure" is a square root. The
 *  default coefficient was extracted from a calibration sweep on this
 *  exact board (constant g_bench_blower_enable, varying g_bench_blower_speed_rpm):
 *
 *      RPM    P(mbar)  P(cmH2O)  rpm/sqrt(P_cmh2o)
 *      ----------------------------------------------
 *      12000   3.775    3.85         6116
 *      15000   5.870    5.99         6131
 *      18000   8.420    8.59         6143
 *      21000  11.450   11.68         6144
 *      24000  14.950   15.25         6147
 *      27000  18.900   19.27         6151
 *      30000  23.300   23.76         6156
 *      33000  28.200   28.76         6154
 *      36000  33.500   34.16         6160
 *
 *  Average K_sqrt ~6141 across the 4-25 cmH2O therapy range (scatter
 *  <0.5%). Below 9 krpm the linear/static-friction term dominates so the
 *  ratio falls; the therapeutic range is far above that knee.
 *
 *  Re-calibrate per-mask/per-hose if you find the integrator pegging in
 *  one direction in steady state (it means K_sqrt is off for your test
 *  setup).
 *
 *      rpm_ff = K_sqrt * sqrt(max(0, setpoint_cmh2o)) + offset_rpm
 */
volatile uint8_t g_blower_ctrl_ff_enable           = 0U;
/* 2026-09-24: 6141 -> 6731, re-derived on the bench.
 *
 * THESE FOUR MOVE TOGETHER - K, ff_offset, ff_offset_slope and
 * physics_ff_c_fan. c_fan was fitted USING this K and offset, so changing one
 * alone makes the others wrong. Re-derive them as a set or not at all.
 *
 * Method: at steady state the commanded rpm IS what the operating point
 * requires, so sweep setpoint and least-squares fit
 * rpm = K*sqrt(p_equiv) + A + B*sp. Six points, 6 to 20 cmH2O, validated by
 * pre-loading the result and confirming the integrator then had nothing left
 * to carry: |I| <= 110 rpm at every setpoint, residuals +/-24 rpm.
 *
 * The old constants over-asked by ~1300 rpm at every operating point, which
 * is why the integrator ran persistently negative in the 2026-09-22/23
 * captures.
 *
 * ff_offset_slope goes to 0. That slope was introduced when c_fan was too
 * small to carry the flow-dependent part, so the offset had to fake a
 * pressure dependence instead. With c_fan corrected below, a two-parameter
 * fit is better conditioned and fits just as well.
 *
 * Do not read precision into any ONE of these numbers: at steady state
 * sqrt(p_equiv), 1 and setpoint are nearly collinear, so the split between
 * them is poorly determined even though their sum is not. An attempt to pin K
 * separately by opening the leak failed for exactly this reason - it moved
 * p_equiv by only 0.3 cmH2O and the joint fit produced K = 10833 with a
 * NEGATIVE offset. What is well determined is the total rpm they predict.
 *
 * 2026-09-24 (later): 6731 -> 6535, offset 796 -> 853, c_fan 0.000683 ->
 * 0.001634. A JOINT fit of all three, which is what finally made the model
 * hold across flows.
 *
 * The 6731/796 fit above used closed-circuit steady points only (16-30 slm),
 * and c_fan was then fitted separately against that reference. Each was
 * right in its own regime and wrong outside it. The model this file uses is
 *
 *      P_blower = ((N - A) / K)^2 - c_fan * Q^2
 *
 * i.e. p_equiv = P_blower + c_fan*Q^2 and N = K*sqrt(p_equiv) + A - the
 * classic fan law, pressure = a*N^2 - b*Q^2, with an offset. Fitting K, A and
 * c_fan TOGETHER against 22 steady points spanning 17000-34000 rpm and 16-64
 * slm (closed and open loop, closed circuit, three leak settings, and a
 * part-open and wide-open port) gives 0.10 cmH2O rms, 0.24 worst.
 *
 * rpm error of the feed-forward (predicted - actual):
 *                          old 6731/796/0.000683     this
 *   steady, 6 families          up to 2228           within 154
 *   breathing peaks,
 *   out of sample            -2739 mean (UNDER)      +493 mean (over)
 *
 * The old set under-asked by ~2700 rpm at peak inspiration - that was the
 * sag. The residual over-ask at breath peaks is the dynamic effect measured
 * separately (peaks show ~1 cmH2O less loss than steady state at the same
 * flow coefficient); it errs early rather than late, which is the right side
 * for a feed-forward to err on.
 *
 * A fan-similarity form, psi = c*(phi - phi0)^2 with phi = Q/(N-A), was also
 * fitted. With phi0 free it improves the fit by 0.01 cmH2O rms; with phi0 = 0
 * it IS the equation above. An earlier claim that the loss "depends on speed,
 * not just flow, and no coefficient can fix it" was an artefact of measuring
 * loss against the stale closed-circuit K/A - the form was fine, the
 * constants were not fitted together.
 *
 * Knock-on: the zero-flow model steadyStateRpmForSetpoint() drops ~2.6%,
 * which moves the mask-off rpm ratio - see the note in therapy.c.
 *
 * 2026-10-02: K 6535 -> 6056 (with ff_offset 773). Refitted by the integrator
 * method on the bench (CPAP, tube + vent only, no breathing) after the
 * blower -> flow-element joint was resealed. With 6535/853 the integrator
 * settled at -1091..-2327 rpm over 4-20 cmH2O - the model asked for a nearly
 * constant 7.6-8.0 % too much speed at every pressure, i.e. a gain error
 * (this blower now makes more pressure per rpm than on 2026-09-24), not an
 * offset. Fit of K and offset together, slope 0, residuals +/-16 rpm:
 *      sp     p_equiv   required FF   (K 6056, off 773)
 *       4       4.32       13344
 *       8       8.65       18598
 *      12      13.00       22620
 *      16      17.36       26006
 *      18      19.54       27543
 *      20      21.74       28996
 * Checked live at 18 -> 6 -> 12 -> 4 cmH2O (two drops): I settled +20, 0,
 * +58, -4 rpm, mask within +/-0.02 cmH2O. K also scales the FF's flow
 * response, which is right for a stronger blower (fan law).
 *
 * Do NOT calibrate with a leaking air path: on 2026-10-01 the joint leaked
 * from ~14:30 (pressure per rpm^2 -25 %, current per rpm^2 +27 %) and a
 * refit made then (offset 1541 + 94.3*sp) was wrong for the sealed device.
 * Run scratchpad health_check.py first (healthy: P/N^2 ~0.023-0.024,
 * iq/N^2 ~0.59-0.64 at 22-38 L/min).
 *
 * 2026-10-05: 6056 -> 6211 (offset 773 -> 685) with the ~150 slm flow
 * element (flow_sensor.c kFlowCalHi, now the default). Same integrator
 * method (ff_cal.py, vent only, 4-20 cmH2O, element 1, inhale deadband 8):
 * the old values left I at +215..+638 rpm; k+offset fit residuals within
 * +/-27 rpm, check at 6/12/18 cmH2O settled I -9/+38/-9. With Round Hole_2
 * back in the path, refit (the old 6056/773 were fitted with it, and with
 * an extra tube on one dP port that has since been removed).
 *
 * 2026-10-06: 6211 -> 6102 (offset 685 -> 903). Air path changed: blower
 * box removed, mesh straightener between blower and element, element LUT
 * recalibrated. Without the box the blower makes more pressure per rpm, so
 * the old values left the integrator at -114..-392 rpm (4-20 cmH2O, vent
 * only) and at -411 at rest on the BreathSimulator. Same integrator method
 * (ff_cal.py): k+offset fit residuals within +/-24 rpm; check at 6/12/18
 * cmH2O settled I -42/+42/-7.
 *
 * 2026-10-07: 6102 -> 6159 (offset 903 -> 667). Blower back in a (new) box,
 * blower-outlet tube junction sealed, mesh + element unchanged. The box
 * draws ~60 mA more at 28 krpm for ~0.2 cmH2O less, and the no-box values
 * left I at -193 rpm at rest on the BreathSimulator. ff_cal.py, vent only:
 * k+offset residuals within +/-34 rpm; check at 6/12/18 cmH2O settled I
 * -22/+37/+6. Refit again if the box or its seals change.
 *
 * 2026-10-22: 6159 -> 5933 (offset 667 -> 628), TKFan BA7060H24B (repo
 * CPAP_TKFan). Air path: blower -> flow element -> device P tap -> 22 mm
 * hose -> vent -> external sensor -> leak closed; no box. ff_cal.py, vent
 * only: with the WM6850 values the integrator already settled at only
 * -32..-57 rpm over 4-20 cmH2O (rotor 12.8k at 4, 27.9k at 20 cmH2O);
 * k+offset residuals within +/-18 rpm; check at 12/18 cmH2O settled I
 * +11/+4 (6 cmH2O had not settled, I sd 209). */
volatile float   g_blower_ctrl_ff_k_rpm_per_sqrtcmh = 5933.0f;

/* RIG CALIBRATION - re-measure for a different mask / hose / vent.
 *
 * K_sqrt above was taken open to atmosphere, so it under-predicts the real
 * circuit: the blower is never at zero flow, it is always pushing the vent
 * leak. On the 2026-09 bench (18 slm vent) the model asked for ~17.7 krpm at
 * 8 cmH2O while ~19.7 krpm was actually needed, and the integrator carried
 * the 2 krpm difference - over half its budget spent on steady-state trim
 * instead of disturbance rejection.
 *
 * Use the OFFSET, not K, to close that gap. K is multiplicative, so raising
 * it also scales the breath excursion and every K-dependent floor together;
 * a 6141 -> 6820 attempt widened the expiratory hump badly. The error is a
 * constant (a fan-curve operating-point shift), so an additive term is the
 * right shape. c_fan cannot close it either - that would need ~0.007, and it
 * is clamped at 0.001.
 *
 * To re-calibrate: run fixed CPAP at several setpoints with the real
 * circuit, record steady-state RPM, and fit rpm = K*sqrt(P) + offset.
 * pc_tools/blower_recal.py automates exactly that sweep and fit.
 *
 * 2026-09-21 re-calibration. Three things below, and the first two are traps
 * that each produced a WRONG value before the third got it right.
 *
 * TRAP 1 - do not fit total rpm against sqrt(P). That is what
 * pc_tools/blower_recal.py does, and it gives the offset for
 * steadyStateRpmForSetpoint(), which has no flow term. Feeding it here
 * double-counts, because the physics FF ADDS K*sqrt(hose + c_fan*Q^2) on top,
 * and the sweep already contained the circuit's vent flow (22-33 slm, rising
 * with pressure). That fit asked for 3777, which measured WORSE at 8 cmH2O
 * than the 2000 it replaced.
 *
 * TRAP 2 - turn the FOT probe off before measuring anything steady state.
 * physiology_telem runs a 4 Hz probe every 30 s whenever therapy is active,
 * in fixed CPAP too, and it was on by default. Measured 1.15 cmH2O p-p bursts
 * against a 0.23 quiet baseline, i.e. 1-2 probes inside every 50 s dwell. The
 * loop is not symmetric (over_p_gain 6.0 vs under_p_gain 3.5, and the blower
 * accelerates faster than it decelerates), so that rectifies into a DC bias
 * in the integrator worth ~850 rpm at 8 cmH2O and ~1280 at 16. It is why two
 * sessions disagreed by ~1250 rpm and why 2850 looked right once and wrong
 * the next time. g_physio_fot_cpap_enable now defaults it off in CPAP.
 *
 * METHOD - calibrate from the integrator. At steady state it carries exactly
 * what the FF gets wrong, so the offset that zeroes it is
 * offset_in_force + I_settled. Allow 70 s settle + 50 s dwell: the integ
 * error LPF has tau 15 s with a 45 s warm-up, so a 45 s run is still moving.
 *
 * Sweep, FOT off, one session, one build, offset held at 2850:
 *
 *      P        I      I sd    offset needed   clip
 *      6.15   -1292      8         1558        0.0%
 *      8.21    -998     20         1852        0.0%
 *     12.33    -878     27         1972        0.0%
 *     16.41    -971    236         1879        2.5%  (at the rpm ceiling)
 *
 * Confirmation 20 min later at the chosen 1750:
 *
 *      P        I      I sd    offset needed
 *      6.15    -262     29         1488
 *      8.21    -158     15         1592
 *     12.33      -5     38         1745
 *
 * REPEATABILITY - note the two tables disagree by up to 260 rpm on the same
 * rig in the same session. Scatter WITHIN a run is 15-38 rpm; scatter BETWEEN
 * runs is ~200. Thermal drift and flow auto-zero are the suspects. Do not
 * chase residuals smaller than that: pooling both runs suggests ~1700 rather
 * than 1750, which is not a real distinction.
 *
 * 1750 keeps |I| under ~260 rpm (6.6% of the +/-4000 budget) across 6-12
 * cmH2O, against 878-1292 rpm (22-32%) at 2850. The requirement is also
 * curved, not linear - the slope runs 143 rpm/cmH2O from 6 to 8 and 29 from
 * 8 to 12 - so no single constant can flatten it. Getting below ~200 rpm
 * means fixing the shape (c_fan and the vent model), not this number.
 *
 * NOTE on the +2.71% that an earlier version of this comment called a sensor
 * span error: it is not one. P_sensor sits above the setpoint by that much
 * because the PID regulates P_mask, and P_sensor exceeds P_mask by the
 * modelled hose drop - which also scales with pressure, since vent flow goes
 * as sqrt(P). That is tube compensation working as designed, and an external
 * sensor at the mask end confirmed it on 2026-09-22 (tubecomp off put the
 * patient 0.53 cmH2O low; tubecomp on brought it to 0.13 low).
 *
 * Nothing here needs re-deriving on that account: every number in this block
 * is referenced to our own sensor, which is also what the loop controls on.
 *
 * Separately, and still unresolved: our sensor and the external one disagree
 * by 1.58% of reading at zero flow (see tube_comp.c). That is a real mismatch
 * between two instruments, but it is common-mode here and does not move this
 * offset. It does need settling with a reference manometer, because if ours
 * is the one reading high then delivered pressure is 1.6% below the display. */
/* 2026-10-02: 773, fitted together with K = 6056 - see the note at K.
 * (The 2026-10-01 refit to 1541 + 94.3*sp was made on a leaking air path and
 * is superseded.) */
volatile float   g_blower_ctrl_ff_offset_rpm        = 628.0f;   /* 2026-10-22 TKFan (was 667) - see ff_k */

/* Pressure-dependent part of the same offset: rpm added per cmH2O of setpoint.
 *
 * Why the offset needed a slope at all. c_fan was doing two incompatible jobs.
 * Vent flow goes as sqrt(P), so c_fan*Q^2 is LINEAR in pressure - which means
 * the fan term was quietly supplying the steady-state slope of the required
 * rpm, as well as the breath response it is nominally for. Those want opposite
 * values, measured on 2026-09-22 with the breath simulator:
 *
 *      c_fan    slope     worst |I|      p-p @8   p-p @15
 *     0.0003    157.2       943 rpm       0.560     0.570
 *     0.0010     62.7       376           0.810     0.830
 *     0.0015     -4.8        29           0.966     0.751
 *     0.0022    -99.3       596           1.220     0.970
 *
 * c_fan ~0.0015 flattens the steady-state requirement completely, and ~0.0003
 * gives the flattest delivered pressure. No single value is good at both, so
 * every choice was a compromise until the jobs were separated.
 *
 * Giving the offset its own slope does that: the steady-state term is now
 * offset_a + offset_b * setpoint, fitted directly, and c_fan is free to be set
 * purely for the waveform. Fit at c_fan 0.0003, 5 points over 6-18 cmH2O:
 *
 *      P      required offset   fit    resid
 *      6.15        2772        2786     -14
 *      8.19        3135        3106     +29
 *     12.26        3730        3745     -15
 *     16.35        4370        4387     -17
 *     18.31        4713        4696     +17
 *
 * +/-29 rpm, against 943 rpm of unavoidable integrator load with a constant.
 *
 * Set to 0 to get the old constant-offset behaviour back. Re-derive both terms
 * together with the integrator method described above - they are one fit. */
volatile float   g_blower_ctrl_ff_offset_slope_rpm_per_cmh2o = 0.0f;   /* 2026-10-02, see K */

/* Steady-state rpm term: constant plus slope. Takes the SETPOINT, not p_equiv
 * - the slope was fitted against setpoint, and the flow-dependent part of the
 * model is c_fan's job, not this one. */
static float ffOffsetRpm(float setpoint_cmh2o)
{
    const float sp = (setpoint_cmh2o > 0.0f) ? setpoint_cmh2o : 0.0f;
    return g_blower_ctrl_ff_offset_rpm
         + (g_blower_ctrl_ff_offset_slope_rpm_per_cmh2o * sp);
}

/* Flow disturbance feed-forward: proactive RPM boost on patient inhalation.
 * On this hardware Q_patient is positive on inhale (see telemetry plot).
 * g_blower_ctrl_flow_ff_inhale_sign: +1 = positive inhale, -1 = negative.
 * Uses the raw breath signal (fastest) for the target magnitude. */
volatile uint8_t g_blower_ctrl_flow_ff_enable     = 0U;
volatile int8_t  g_blower_ctrl_flow_ff_inhale_sign = 1;
volatile float   g_blower_ctrl_flow_ff_k_rpm_per_slm = 58.0f;
volatile float   g_blower_ctrl_flow_ff_max_rpm    = 3000.0f;
/* 2026-10-05: 3 -> 8 slm with the ~150 slm flow element. Its flow noise is
 * ~2x Round Hole_2's at low flow (same dP noise in mbar, ~3x coarser dP per
 * slm below 30 slm); at 3 slm the noise alone flipped inhale detection and,
 * in the twin, drove the pressure loop into a ~7-9 Hz oscillation with iq at
 * its limit. At 8: stable in the twin with margin at +2 ms delay, and on the
 * bench (CPAP 20<->18, vent only, 6 runs) no faults, speed-command jitter
 * ~200-250 rpm/20 ms (Round Hole_2 at 3: ~80-150). Breathing at 20 cmH2O:
 * breath-onset detection 10 ms behind true flow (Round Hole_2 at 3: 40 ms). */
volatile float   g_blower_ctrl_flow_ff_deadband_slm = 8.0f;
volatile float   g_blower_ctrl_flow_ff_rise_rpm_per_s = 10000.0f;
volatile float   g_blower_ctrl_flow_ff_fall_rpm_per_s = 7000.0f;

/* Extra flow-FF from dQ_patient/dt at inhale onset (before flow magnitude
 * peaks). Catches the breath earlier than magnitude-only FF. */
volatile uint8_t g_blower_ctrl_flow_ff_onset_enable          = 0U;
volatile float   g_blower_ctrl_flow_ff_onset_k_rpm_per_slm_per_s = 6.0f;
volatile float   g_blower_ctrl_flow_ff_onset_thresh_slm_per_s    = 60.0f;
volatile float   g_blower_ctrl_flow_ff_onset_max_rpm             = 1200.0f;

/* Immediate RPM boost when raw sensor pressure is falling quickly
 * (catches inhale dips even if patient-flow sign or leak estimate lags).
 *
 * The threshold must sit just above the measured dP/dt noise floor, NOT
 * above the breath. The old 10 cmH2O/s never fired: a 25 slm bench breath
 * only produces ~1.6 cmH2O/s. 2.0 works once the 10 Hz stale-sample
 * artefact is fixed (see g_ps_sample_fresh in main.c) - before that the
 * artefact could trip it at random. If the expiratory baseline turns hairy,
 * this threshold has dropped into the noise: raise it to ~4. */
volatile uint8_t g_blower_ctrl_dip_ff_enable              = 1U;
volatile float   g_blower_ctrl_dip_ff_k_rpm_per_cmh2o_per_s = 400.0f;
/* 2026-09-21: raised 2.0 -> 5.0 because baseline dP/dt noise at 16 cmH2O was
 * tripping this term to its 1500 rpm cap at ZERO flow (~1400 rpm of command
 * jitter between adjacent 20 ms samples on a static circuit).
 *
 * 2026-09-22: back to 1.5, with the noise rejected by err_min instead. 5.0
 * did stop the noise, but it also switched the term off for real breaths: a
 * breath at 15 cmH2O drops the pressure at only ~2.8 cmH2O/s, so nothing
 * fired, and the output is k*(rate - thresh) so even 2.0 would have yielded
 * just 320 rpm. Rate is simply the wrong discriminator - noise and breaths
 * overlap in dP/dt.
 *
 * The SAG separates them cleanly. On the same trace the baseline wanders
 * +/-0.15 cmH2O while a breath sags 2.0, so err_min at 0.30 rejects the noise
 * with margin and passes every real breath. The phase gate above does not
 * help here: physicsFfExpTailNearZero() is true on a static circuit, so
 * dip_phase is satisfied while nobody is breathing at all. */
volatile float   g_blower_ctrl_dip_ff_thresh_cmh2o_per_s   = 1.5f;
volatile float   g_blower_ctrl_dip_ff_max_rpm              = 1500.0f;
volatile float   g_blower_ctrl_dip_ff_err_min_cmh2o        = 0.30f;

/* Expiratory rise FF - the mirror of dip FF, and the piece that was missing.
 *
 * dip_ff brakes a FALLING pressure during inspiration; nothing braked a
 * RISING one. Term telemetry (capture_20260908_133537.log, 22-breath
 * ensemble) shows why that matters: through early expiration ff sits pinned
 * at -307 rpm while patient flow swings 15.2 -> 10.5 slm, because
 * physicsFfFlowInputSlm() clamps total flow at >= 0 and vent+patient is
 * about -0.9 slm there. The static plant inverse has nothing to say about
 * this - signing the clamp moves p_equiv by 0.003 cmH2O, roughly 4 rpm -
 * because the real disturbance is the patient dumping tidal volume into the
 * circuit compliance, which only bleeds out through the vent.
 *
 * So brake on the pressure RATE instead of the flow model: when pressure is
 * above setpoint AND rising faster than the threshold, subtract RPM
 * proportionally. Deliberately NOT phase-gated - the worst of the rise
 * happens at flow reversal, where patientInhaling() is still ambiguous, and
 * "over setpoint and climbing" is the condition we care about regardless of
 * which phase the detector thinks it is in.
 *
 * Default ON since 2026-09-08: A/B over 23/49 breaths gave spike
 * +0.361 -> +0.333 and p-p 0.540 -> 0.525 for a 0.012 cost on the dip.
 * Watch g_blower_ctrl_ff_rise_term on the "D," line. */
volatile uint8_t g_blower_ctrl_rise_ff_enable               = 1U;
volatile float   g_blower_ctrl_rise_ff_k_rpm_per_cmh2o_per_s = 400.0f;
/* 2026-09-21: 2.0 -> 5.0 because at 16 cmH2O with no flow this term was
 * reaching -995 rpm on noise alone.
 * 2026-09-22: back to 1.5 with the noise rejected by rise_ff_err_min instead,
 * for the reason spelled out on dip_ff_thresh above - rate does not separate
 * noise from breaths, the pressure excursion does. */
volatile float   g_blower_ctrl_rise_ff_thresh_cmh2o_per_s   = 1.5f;
volatile float   g_blower_ctrl_rise_ff_max_rpm              = 1500.0f;
volatile float   g_blower_ctrl_rise_ff_err_min_cmh2o        = 0.30f;

/* Expiratory flow FF - derived from measurement, not from the plant model.
 *
 * Fitting every expiratory sample of capture_20260908_135325 (23 breaths):
 *
 *    |Q_exp| bin    mean excess   excess/|Q|
 *      8 - 10         +0.125        0.0139
 *     10 - 12         +0.164        0.0149
 *     12 - 14         +0.186        0.0142
 *     14 - 20         +0.201        0.0134
 *
 * i.e. the residual expiratory over-pressure is proportional to expiratory
 * flow at k = 0.0137 cmH2O/slm = 22.8 rpm/slm (fit through the origin,
 * R2 = 0.46). A deadband fits WORSE (R2 0.38 at 4 slm), so the term is
 * plain proportional.
 *
 * The important part: cross-correlation puts the pressure excess 200 ms
 * AHEAD of the flow (r = 0.87 at -200 ms vs 0.69 at zero). So flow is a
 * correlate, not the cause - the blower over-speeds when inspiratory demand
 * collapses, and expiratory flow only builds afterwards. A FF driven by
 * instantaneous flow would therefore arrive 200 ms late, which is why this
 * uses a phase lead (q + T*dq/dt) rather than the hold that a
 * compliance-dump model would have suggested.
 *
 * Being explicit about the limit: at zero lag flow explains 47% of the
 * variance, 75% with perfect 200 ms prediction. This cannot remove the
 * residual entirely, and what it leaves behind is the genuinely causal part.
 *
 * Default ON since 2026-09-08: fires on 50% of samples, peak -381 rpm
 * (predicted -345), and took hump 1 from +0.140 to +0.122 with p-p
 * 0.526 -> 0.507 and sd -11%. Watch g_blower_ctrl_ff_exp_flow_term.
 *
 * Default OFF since 2026-10-06 (BreathSimulator, 2 L lung, same breaths
 * on/off, two repeats each, ~150 slm element, physics FF). The fit above
 * predates the physics FF, which now lowers the speed through expiration on
 * its own, so this counted the same thing twice. It sat at the -800 rpm cap
 * for the whole expiration and released only once |Q| fell below ~35 slm,
 * after the flow had already turned:
 *                       exp plateau   end-exp trough   rest sd
 *   20 cmH2O  on           -0.04          -0.77         0.091
 *             off          +0.19          -0.44         0.075
 *    8 cmH2O  on           -0.20          -0.41         0.02
 *             off          +0.01          -0.28         0.02
 * (cmH2O vs rest, external sensor at the lung.) Its rpm is not scaled with
 * setpoint, so at low pressure it under-pressures the expiration outright.
 * Inspiratory dip and spike unchanged. */
volatile uint8_t g_blower_ctrl_exp_flow_ff_enable         = 0U;
volatile float   g_blower_ctrl_exp_flow_ff_k_rpm_per_slm  = 23.0f;
volatile float   g_blower_ctrl_exp_flow_ff_lead_s         = 0.20f;
volatile float   g_blower_ctrl_exp_flow_ff_max_rpm        = 800.0f;
/* LPF on the dq/dt used for the lead; raw 50 Hz derivative is too noisy. */
volatile float   g_blower_ctrl_exp_flow_ff_dq_lpf_hz      = 8.0f;

/* Physics-based combined FF (mutually exclusive with sqrt + flow + dip FF):
 *      P_equiv = SP_mask + dP_hose(|Q|) + c_fan * max(Q,0)^2
 *      rpm_ff  = K_sqrt * sqrt(P_equiv) + offset
 * dP_hose uses the FF's own hose coefficients below; c_fan models blower fan
 * curve. */
volatile uint8_t g_blower_ctrl_physics_ff_enable = 1U;

/* Hose basis of the physics FF - NOT tube_comp's coefficients.
 *
 * dP_hose inside P_equiv used to read TubeComp_GetRLam/RTurb. The 2026-09-24
 * joint fit of K / offset / c_fan (6535 / 853 / 0.001634) ran with tube comp
 * on 0.002 / 0.00022 - TubeComp_Init reset it to that pair at every boot, and
 * the D-line hose term confirms it for flow_sweep_K and cfan_split. hose and
 * c_fan * Q^2 were fitted together; only their sum is calibrated. Fixing the
 * tube-comp startup values (to the 0.00353 / 0.000118 fitted at the mask on
 * 2026-09-22) would have silently cut P_equiv by ~0.5 cmH2O at 80 slm and
 * detuned that fit.
 *
 * So the FF keeps the basis it was fitted on, frozen here, while tube comp -
 * the pressure MEASUREMENT - takes the calibrated pair. When the FF is next
 * refitted, set these to whatever the fit uses (the calibrated pair is the
 * natural choice) and refit c_fan with them. */
volatile float   g_blower_ctrl_physics_ff_hose_r_lam  = 0.002f;
volatile float   g_blower_ctrl_physics_ff_hose_r_turb = 0.00022f;
/* Set by CPAP Hybrid profile while active (insp Physics FF / exp hold). */
volatile uint8_t g_blower_ctrl_cpap_hybrid_prefer = 0U;
volatile uint8_t g_blower_ctrl_cpap_hybrid_active = 0U;
/* Hybrid exp hold: only bleed RPM when P > setpoint + band; boost if below. */
volatile float   g_blower_ctrl_hybrid_exp_hold_band_cmh2o = 0.12f;
volatile float   g_blower_ctrl_hybrid_exp_fall_rpm_per_s  = 1500.0f;
volatile float   g_blower_ctrl_hybrid_exp_boost_rpm_per_cmh2o = 2500.0f;
volatile float   g_blower_ctrl_hybrid_exp_boost_rpm_per_s = 8000.0f;
volatile uint8_t g_blower_ctrl_physics_ff_use_total_flow = 1U;
volatile uint8_t g_blower_ctrl_physics_ff_exhale_mit_enable = 1U;
/* Fan-curve droop: the term that turns inspiratory flow into anticipatory
 * RPM, and the single biggest lever on the inspiratory dip. At the old
 * 0.00012 the FF anticipated only ~570 of the ~2500 rpm a breath needs, so
 * the P-term supplied the rest reactively - i.e. only after pressure had
 * already fallen. 0.0008 raises that to ~1550 rpm. Measure it properly with
 * the open-loop manual blower: hold RPM, sweep flow, and the slope of
 * P against Q^2 is c_fan. Clamped to 0.001 in
 * physicsFfRpmForSetpointAndFlow(). */
/* 2026-09-22: 0.0022 -> 0.0003, and the value now means something, which it
 * did not before. Read the history, because it explains why the older numbers
 * in this file cite c_fan as if it had been tuned.
 *
 * Until 2026-09-22 the USB setter clamped c_fan at 0.001 (see the note on
 * BLOWER_CTRL_C_FAN_MAX in blower_ctrl.h). So every "blower set
 * physics_ff_c_fan" above that was silently clipped while still acking. The
 * 0.003 -> 0.0022 change recorded here on 2026-09-21 was really 0.001 ->
 * 0.001: whatever improved that waveform, it was not this coefficient. Three
 * bench sweeps then read as "c_fan has no effect" because it never varied.
 *
 * Once the setter was fixed, two clean sweeps with the breath simulator
 * (framing corruption also fixed first - it had reached 18% of D lines, and
 * percentile metrics cannot survive that):
 *
 *   c_fan    P_sensor p-p @ 8     @ 15        P@peak @ 8    I mean @ 8
 *   0.0000        0.540          0.610          +2240         +1709
 *   0.0003        0.560          0.570          +1727         +1414
 *   0.0014        0.930          0.720           +276          +486
 *   0.0022        1.220          0.970           -704            -29
 *   0.0044        1.570             -           -1047         -1167
 *
 * Delivered pressure gets monotonically WORSE as c_fan rises - almost 3x at 8
 * cmH2O between 0 and the old 0.0022 default. The optimum is ~0 at BOTH
 * pressures, so this is not a coefficient that wants to scale with the
 * operating point; the term as formulated (c_fan * Q^2 on TOTAL flow) is net
 * harmful.
 *
 * The three criteria disagree, consistently, and that is the real finding:
 * flattest pressure at ~0, P-term-zeroed at ~0.0017, integrator-zeroed at
 * ~0.0022. One coefficient sets both the DC level the integrator must cancel
 * AND the breath modulation, and they want opposite values. Driving the FF to
 * supply what the model says the breath needs degrades what the patient gets.
 *
 * 0.0003 rather than 0.0 is deliberate: it costs nothing measurable at either
 * pressure (0.560 vs 0.540 at 8; 0.570 vs 0.610 at 15) and leaves the flow
 * path alive rather than removing the FF's flow response outright. At 0 the
 * P term carries the entire breath (+2240 rpm at peak at 8 cmH2O), which
 * works but is purely reactive.
 *
 * UNEXPLAINED: P_mask p-p has a shallow minimum near 0.0008-0.0014 at both
 * pressures, i.e. the estimate the loop controls on prefers a small fan term
 * even though the real pressure does not. Worth understanding before anyone
 * reshapes this term. P_sensor is the physical quantity and it is unambiguous.
 *
 * NOTE: ff_offset below was calibrated with c_fan effectively 0.001. The FF's
 * flow term scales with c_fan, so re-derive the offset after this change -
 * see the integrator method documented there. */
/* 2026-09-24: 0.0003 -> 0.000683, and the flow filter below drops 12/10 Hz
 * to 3 Hz. The two changes are one change and must not be separated.
 *
 * WHY THE HISTORY ABOVE SAYS THE OPPOSITE. Everything above concluded c_fan
 * wants to be near zero, because every sweep found that raising it made
 * delivered pressure monotonically worse. That was a true observation and the
 * wrong conclusion, because c_fan*Q^2 does two jobs at once:
 *
 *   the DC part  - the real circuit loss at the mean flow. Genuine physics,
 *                  slow, and the blower can follow it.
 *   the fast part - breath-rate modulation, applied through a flow estimate
 *                  that lags 40-80 ms, into a motor measured 2300-4000 rpm
 *                  behind its command during a breath.
 *
 * One coefficient raised both, so the DC benefit always arrived with a
 * dynamic penalty stapled to it, and the penalty won.
 *
 * THE DC PART IS REAL AND WAS BEING UNDER-SUPPLIED. Backing the true loss out
 * of 11 steady points over 16-45 slm (leak closed and open) gives a Q^2
 * coefficient of 0.000801. The model carried R_turb 0.000118 + c_fan 0.0003 =
 * 0.000418, i.e. 1.9x too small; at 80 slm it under-asks by several cmH2O,
 * which is the inspiratory sag. c_fan = 0.000801 - R_turb = 0.000683.
 *
 * SEPARATING THEM. Fixing the coefficient at 0.000683 and sweeping the flow
 * cutoff, 8 breaths per point at 18 cmH2O:
 *
 *      cutoff     sag (cmH2O)   SE
 *      12 Hz         0.41      0.06     <- worse than the old c_fan
 *       6 Hz         0.37      0.05
 *       3 Hz         0.24      0.04
 *     1.5 Hz         0.18      0.06     <- best
 *     0.7 Hz         0.34      0.04     <- benefit fading
 *      (old c_fan 0.0003 @ 12 Hz: 0.36; no flow comp at all: 0.35)
 *
 * A clear minimum, about 2.7 sigma below the 12 Hz point. At 12 Hz the
 * corrected coefficient IS worse, reproducing the historical result exactly;
 * filtered, the same coefficient is 33-50% better. Both prior findings were
 * true and the conflict was never about the value of c_fan.
 *
 * 3 Hz rather than the measured-best 1.5 Hz: deliberately conservative. The
 * sweep ran at 17.7 slm peak patient flow because the breath simulator cannot
 * do more - a real patient pulls 50-73 - and the fast component grows with
 * flow, so the optimum may move. 3 Hz keeps most of the gain with more of the
 * flow signal intact. Confirm with a human before going lower.
 *
 * Independent support: mean |I| during breathing fell 2036 (no comp) -> 1527
 * (c_fan 0.0003) -> ~950 (0.000683), regardless of cutoff. The DC
 * compensation works; the cutoff only decides how much dynamic harm rides
 * along with it.
 *
 * 2026-09-24 (later): 0.000683 -> 0.001634, from a joint fit with K and
 * ff_offset - see the note on K. Two corrections to the reasoning above:
 *   - "c_fan = 0.000801 - R_turb" was wrong. That loss was measured at the
 *     BLOWER sensor, where tube compensation plays no part; the hose term
 *     converts mask setpoint to blower target and is separate from fan droop.
 *   - 0.000801 itself was fitted against the closed-circuit-only K/A, so it
 *     absorbed their error. Fitted together, c_fan is 0.001634.
 * The DC/dynamic split and the 3 Hz filter below stand. The bigger
 * coefficient makes the filter MORE important, not less: the cutoff sweep
 * that chose 3 Hz ran at 0.000683, so confirm it at this value with a human
 * breathing before trusting it.
 *
 * 2026-10-02: 0.001634 -> 0.00135. With the flow-sensor lag fixed (flow_sensor.c
 * dp_max_slew) a fit of the speed actually needed for 20 cmH2O at the mask
 * against true flow gave 0.00135 with no lag; 0.001634 asked for too much
 * speed at peak flow (ramping end-inspiration overshoot) and too little in
 * expiration (baseline sag). Part of the breathing-tuned set - see over_p_gain. */
/* 2026-10-05: 0.00135 -> 0.0008 with the ~150 slm flow element (less drop in
 * the path, so less load per Q^2), together with physics_ff_fall 12000.
 * Twin re-tune on eight 20 cmH2O breath sets (element noise included,
 * deadband 8, over_p_gain 4): dip 1.25/2.35 (p90), spike 1.21, width 0.40 s,
 * p-p 2.62 vs Round Hole_2 as flashed 1.56/2.54, 2.02, 0.51, 3.67. Measured
 * once (el1tuned20_20261005_170351, fast deep breaths): dip 3.02/3.64,
 * spike 2.15, width 0.40, p-p 5.19, where the twin puts Round Hole_2 at
 * 3.03/4.08, 2.62, 0.52, 5.84 on the same breaths. */
volatile float   g_blower_ctrl_physics_ff_c_fan_cmh2o_per_slm2 = 0.0008f;

/* Expiratory side of the fan-load term, 2026-10-22 (TKFan BA7060H24B).
 * c_fan above now applies while the total flow is at or above the resting
 * flow (vent + leak, physicsFfVentFloorQSlm); BELOW it - expiration, when the
 * patient supplies part of the vent flow - the drop from the resting value
 * uses this coefficient instead:
 *     Q >= Qr: fan_load = c_fan * Q^2
 *     Q <  Qr: fan_load = c_fan * Qr^2 - c_fan_exp * (Qr^2 - Q^2)
 * Continuous at Qr; c_fan_exp == c_fan gives the previous single-coefficient
 * term exactly.
 * Why: on the BreathSimulator (10 bpm, ~70 L/min breaths, 20 cmH2O) one shared
 * c_fan traded the two halves of the breath against each other:
 *     c_fan    insp dip  insp avg  exp plateau  end-exp dip
 *     0.0008    -0.80     -0.05      -0.44        -0.59
 *     0.0005    -0.82     -0.14      -0.26        -0.47
 *     0.0003    -0.88     -0.20      -0.14        -0.51
 *     0.0001    -1.00     -0.31      -0.05        -0.51
 * The TKFan's flatter fan curve droops less with flow, so the full c_fan
 * over-reduces speed through expiration (the -0.44 plateau sag), while
 * inspiration still wants it.
 * Split sweep (c_fan 0.0008 inspiratory side, two runs, same breaths):
 *     c_fan_exp  insp dip  insp avg  exp plateau  end-exp dip
 *     0.0008      -0.80     +0.02    -0.45/-0.40     -0.49   (old model)
 *     0.0003      -0.87     -0.02    -0.12/-0.13     -0.55
 *     0.0002      -0.77     +0.01    -0.12/-0.08     -0.56
 *     0.0001      -0.77     +0.05    -0.04/-0.03     -0.50
 *     0           -0.78     +0.02       -0.00        -0.48
 * Inspiration is untouched by the split; the expiratory sag goes away.
 * 0.0001 keeps a little of the physical droop that 0 would drop. */
volatile float   g_blower_ctrl_physics_ff_c_fan_exp_cmh2o_per_slm2 = 0.0001f;
volatile float   g_blower_ctrl_physics_ff_flow_lpf_hz = 4.0f;
volatile float   g_blower_ctrl_physics_ff_flow_lpf_inhale_hz = 3.0f;
volatile float   g_blower_ctrl_physics_ff_flow_lpf_exhale_hz = 3.0f;
volatile uint8_t g_blower_ctrl_physics_ff_inhale_mit_enable = 1U;

/* Patient flow (L/min) that confirms an inhale (+) or exhale (-) before the
 * physics-FF onset handlers snap the flow filter. 0 = snap on every flip of
 * patientInhaling(), the previous behaviour.
 *
 * 2026-10-01, bench, CPAP 16 cmH2O vent only, no breathing: patient-flow
 * noise at rest is ~3 L/min sd around zero, so patientInhaling() (3 L/min
 * deadband) flipped ~1.7 times a second. Each flip snapped the FF flow filter
 * onto raw flow: q_filt jumped 3.6 L/min and the FF target 590 rpm (vs 1.0 /
 * 77 between flips) - the source of most of the rest command jitter and the
 * ~150 rpm (~0.15 cmH2O) rotor wobble at 1-6 Hz. A heavier exhale LPF made
 * the snaps bigger, not smaller. False 'breaths' at rest peak at 4.5-5 L/min
 * (max 13.5); real inhales peaked > 50 L/min and exhales > 20 in the
 * recorded sessions, and pass 8 L/min ~40 ms after 3. At 10 L/min, 98 % of
 * false inhale and 95 % of false exhale flips no longer snap; on a real
 * breath the snap comes ~40-60 ms later. Only the snaps are gated - the FF
 * branch, onset lead and every other user of patientInhaling() keep the raw
 * flag. Breathing effect not yet verified (needs the test lung).
 *
 * RESULT, same bench: no measurable effect (FF target still jumped
 * ~800-1170 rpm at the flips, rotor wobble unchanged). The jumps come from
 * the expiratory-tail hold switching in and out at |patient flow| = 3 L/min
 * (vent-estimate flow vs measured total flow, ~370 rpm apart), not from the
 * snaps - see g_blower_ctrl_physics_ff_exp_tail_flow_slm. Left OFF; kept as
 * an option.
 *      blower set physics_ff_onset_confirm 10    (try it) */
volatile float   g_blower_ctrl_physics_ff_onset_confirm_slm = 0.0f;
/* Inspiratory onset lead, gated on dQ/dt so it leads the flow magnitude.
 * Same sizing rule as dip FF: the old thresh of 80 slm/s sat above what a
 * 25 slm bench breath produces (~40 slm/s peak), so it never fired, and at
 * k=4 the boost would have been ~90 rpm even if it had. 20/20 gives a
 * useful ~460 rpm of lead. Clamped to >= 10 slm/s in physicsFfOnsetBoostRpm(). */
volatile uint8_t g_blower_ctrl_physics_ff_onset_enable = 1U;
volatile float   g_blower_ctrl_physics_ff_onset_k_rpm_per_slm_per_s = 20.0f;
volatile float   g_blower_ctrl_physics_ff_onset_thresh_slm_per_s = 20.0f;
volatile float   g_blower_ctrl_physics_ff_onset_max_rpm = 2500.0f;
/* Expiratory-recovery lead, 2026-10-06 - the onset lead's counterpart at the
 * END of expiration.
 *
 * BreathSimulator, 2 L lung, ~70 L/min breaths, 20 cmH2O: as expiratory flow
 * dies away (-47 -> -5 L/min in ~0.45 s) the flow through the blower climbs
 * from ~0 back to vent + leak, the FF follows it with no lead, the speed loop
 * lags the command by ~300 rpm, and the pressure at the lung sags 0.44 cmH2O
 * below rest (AirSense 11 on the same breaths: 0.23). Same cause as the
 * inspiratory onset dip, so the same remedy: while expiring (patient flow
 * below -exp_tail_flow) and patient flow is rising toward zero faster than
 * thresh, add k * (dQ/dt - thresh), capped. Patient flow is used rather
 * than the FF's own input because that is total flow clamped at >= 0, which
 * hides the first part of the recovery whenever expiration exceeds vent +
 * leak. Like the onset lead it rides on top of the slew. The dQ/dt filter
 * reuses physics_ff_onset_dqdt_tau. Sizing: ~93 slm/s on those breaths,
 * (93 - 20) * 5 = ~365 rpm, about the measured speed shortfall.
 *
 * Simulator result (same breaths, exp_flow_ff off, two on/off repeats per
 * setpoint), end-of-expiration trough at the lung vs rest:
 *   20 cmH2O   off -0.46 / -0.45    on (k 5) -0.31 / -0.37 / -0.29
 *    8 cmH2O   off -0.30 / -0.26    on (k 5) -0.21 / -0.20
 * Inspiratory dip, spike, expiratory plateau and rest ripple unchanged. k 3
 * to 15 all land at -0.28..-0.36: with the lead on, the controller's own mask
 * estimate no longer dips (k 3: 19.88 at worst; k 15 overshoots it to
 * +0.25 and P cancels the excess). What the lung still sees happens between
 * the mask estimate and the lung - most likely hose inertance (L*dQ/dt,
 * ~0.14 cmH2O on these breaths), which tube_comp does not model. */
volatile uint8_t g_blower_ctrl_physics_ff_exp_lead_enable = 1U;
volatile float   g_blower_ctrl_physics_ff_exp_lead_k_rpm_per_slm_per_s = 5.0f;
volatile float   g_blower_ctrl_physics_ff_exp_lead_thresh_slm_per_s = 20.0f;
volatile float   g_blower_ctrl_physics_ff_exp_lead_max_rpm = 1000.0f;
volatile float   g_blower_ctrl_physics_ff_exp_lead_rpm = 0.0f;   /* diagnostic */

/* dQ/dt for the onset lead is a raw one-tick difference at 200 Hz, so it is
 * mostly noise: measured on the end-expiratory plateau the 50 Hz derivative
 * has std 11 slm/s, which scales to ~45 slm/s at the loop tick - above the
 * 20 slm/s threshold. A one-sided threshold rectifies that into ~190 rpm of
 * permanent phantom boost. The rise slew used to smooth it away; once the
 * lead bypasses the slew it has to be filtered here instead. At tau = 30 ms
 * the tick noise drops to ~13 slm/s and the rectified term to ~12 rpm, for
 * 30 ms of added lag against a ~200 ms feature. 0 disables the filter. */
volatile float   g_blower_ctrl_physics_ff_onset_dqdt_tau_s = 0.03f;

/* Add the onset lead AFTER the rise slew instead of before it.
 *
 * The slew exists to stop sqrt(P_equiv) chasing flow noise. The onset term is
 * not flow-following - it is a bounded transient lead (onset_max), and its own
 * derivative is ~onset_k * d2Q/dt2, which dwarfs the flow-following rate at
 * inhale onset. Measured with the lead inside the slew, d(ff)/dt saturated at
 * exactly 12000 rpm/s (rise 6000 x insp_rise_boost 2) through the steepest
 * part of the inspiratory dip: the limiter was deleting the lead it was meant
 * to deliver. Set to 0 to restore the previous ordering. */
volatile uint8_t g_blower_ctrl_physics_ff_onset_bypass_slew = 1U;
volatile float   g_blower_ctrl_physics_ff_inhale_rise_boost = 2.0f;
volatile float   g_blower_ctrl_physics_ff_rise_rpm_per_s = 20000.0f;  /* 2026-10-02 (was 6000) - see over_p_gain */
/* 2026-10-05: 4000 -> 12000 rpm/s. Lets the FF come down after the inspiratory
 * peak; in the twin it cuts the end-inspiration spike and its width on
 * either element (Round Hole_2: spike 2.02 -> 1.56, width 0.51 -> 0.37 s,
 * p-p 3.67 -> 3.33, dip unchanged) - see c_fan. */
volatile float   g_blower_ctrl_physics_ff_fall_rpm_per_s = 12000.0f;
volatile float   g_blower_ctrl_physics_ff_fall_exhale_rpm_per_s = 8000.0f;
volatile uint8_t g_blower_ctrl_physics_ff_exp_tail_hold_enable = 1U;
/* 2026-10-06: 3 -> 6 slm. With the ~150 slm element, patient flow at rest
 * wanders ~0.5 slm sd (largely the BreathSimulator's own flow wobble) and
 * crossed the 3 slm band, so the FF kept switching between the tail hold and
 * flow-following and pushed pressure along with that flow. Simulator, rest
 * ripple at the lung (full-rate external sensor, sd cmH2O), two repeats:
 *   20 cmH2O   3: 0.076 / 0.077 (corr P,Q +0.50/+0.58)   6: 0.071 / 0.066 (+0.38/+0.28)
 *    8 cmH2O   3: 0.036 / 0.021                          6: 0.028 / 0.020
 * Inspiratory dip, spike, expiratory plateau and end-expiration trough
 * unchanged at both setpoints. 10 was similar on ripple, slightly worse on
 * the dip. Also the gate of the expiratory-recovery lead (q < -this). */
volatile float   g_blower_ctrl_physics_ff_exp_tail_flow_slm = 6.0f;
volatile float   g_blower_ctrl_physics_ff_exp_tail_hold_cmh2o = 0.08f;

/* Fall rate (rpm/s) allowed while in the expiratory-tail hold. 0 = hold
 * (rise only), the previous behaviour.
 *
 * 2026-10-01, bench, CPAP vent only, no breathing: after a setpoint drop
 * 18 -> 6 cmH2O the FF froze ~7000 rpm above its own target (25.8k vs
 * 18.8k) for 100 s, because at rest |patient flow| < 3 L/min ~100 % of the
 * time and the tail branch only ever slewed UP. The integrator ran into its
 * -4000 clamp, P added -2000, and the mask still sat +0.4 cmH2O high. In the
 * tail the target is the vent-flow requirement for the setpoint - the right
 * steady-state speed - so falling to it is correct; the hold was there to
 * stop a coast-down below it as patient flow passes zero, which this does not
 * allow (it stops at the target). Any pressure decrease without breathing
 * (lower setpoint, APAP step-down in a pause, apnea after a drop) hit it. */
volatile float   g_blower_ctrl_physics_ff_exp_tail_fall_rpm_per_s = 2000.0f;
volatile float   g_blower_ctrl_physics_ff_vent_floor_rpm = 0.0f;
volatile float   g_blower_ctrl_physics_ff_p_equiv_cmh2o = 0.0f;
volatile float   g_blower_ctrl_physics_ff_hose_drop_cmh2o = 0.0f;
volatile float   g_blower_ctrl_physics_ff_fan_load_cmh2o  = 0.0f;
volatile float   g_blower_ctrl_physics_ff_q_filt_slm = 0.0f;
volatile float   g_blower_ctrl_physics_ff_rpm_raw = 0.0f;

/* During a pressure sag (setpoint - raw_meas > thresh), blend the PID
 * measurement toward the unfiltered tube-comp estimate so a heavy
 * meas_alpha (e.g. 0.9) does not hide the dip from P/D for 50-100 ms. */
volatile uint8_t g_blower_ctrl_fast_meas_enable       = 1U;
volatile float   g_blower_ctrl_fast_meas_thresh_cmh2o  = 0.05f;
volatile float   g_blower_ctrl_fast_meas_full_cmh2o     = 0.20f;

/* Extra P gain when below setpoint during inhalation (1.0 = off). Raised
 * with the lead terms above; safe to push because it only acts on the
 * under-pressure side, unlike over_p_gain whose 6x jump at -0.12 was what
 * put hash on the expiratory baseline (threshold now 0.30, see below).
 * 2026-10-02: 3.5 -> 5, part of the breathing-tuned set - see over_p_gain. */
volatile float   g_blower_ctrl_under_p_gain          = 5.0f;
volatile float   g_blower_ctrl_under_p_thresh_cmh2o  = 0.08f;

/* Extra P gain when above setpoint (1.0 = off; WM6850-style needs no boost).
 * The same threshold also arms the overpressure output brake below.
 *
 * 2026-09-30: threshold 0.12 -> 0.30 cmH2O. At rest (4-6 cmH2O, vent only)
 * the PID error has ~0.07-0.09 cmH2O of sensor noise, so 0.12 sat at ~1.4
 * sigma: noise alone crossed it on 4-9% of 200 Hz ticks, each time jumping
 * P from 1800 to 10800 rpm/cmH2O and engaging the brake. That was the bulk
 * of the speed-command jitter (P-term sd 300-460 rpm vs ~120-160 without
 * the boosts; rotor sd 80-150 rpm vs ~8 open loop). 0.30 is ~3.5 sigma, so
 * noise should essentially never arm it, while end-of-inspiration overshoots
 * (1-2 cmH2O at 20) still do. Runtime: "blower set over_p_thresh <cmH2O>".
 * This is now the REST value; see the breathing-dependent note below.
 *
 * Caveat on the numbers above: they were taken on 2026-09-30 with a fault in
 * the blower-to-flow-element air path (found and fixed 2026-10-01). On the
 * restored circuit the rest error noise was 0.018-0.025 cmH2O and rotor sd
 * ~18 rpm at 4-6 cmH2O, so a fixed 0.12 may not trip on noise there. The
 * breathing-dependent logic keeps the margin for noisier real conditions.
 *
 * 2026-10-01: gain 6 -> 3. Closed-loop therapy on the bench (16 cmH2O, vent
 * only, no breathing, p_err_lpf_hz 10, speed_kp 1637), three runs including
 * one with the threshold forced to its breathing value 0.12: speed-command
 * jitter -28-30 % (~600 -> ~430 rpm per 20 ms), iq_ref sd -20-26 %, rise on
 * 16<->18 steps 230-260 -> 170-240 ms, overshoot unchanged within run-to-run
 * noise (0.60-0.65 -> 0.62-0.65 cmH2O). 2 was quieter still but slowed the
 * rise (420 ms) and overshot more. Not yet confirmed with breathing: the
 * x6 was originally for the end-of-inspiration overshoot - check that with
 * the test lung. "blower set over_p_gain 6" restores the old value.
 *
 * 2026-10-02: back to 6, as part of a set tuned on human breathing at 20
 * cmH2O (external sensor at the mask, sealed air path, flow guard fixed):
 *      c_fan 0.001634 -> 0.00135, over_p_gain 3 -> 6,
 *      physics_ff_rise 6000 -> 20000 rpm/s, under_p_gain 3.5 -> 5.
 * The x3 above was chosen on 2026-10-01 bench data from a LEAKING air path and
 * without breathing; the x6 is what catches the end-of-inspiration overshoot.
 * Ranked first in a digital twin (the real CM7 code in a fitted plant,
 * scratchpad twin/), on ~50 identical recorded breaths, then confirmed live:
 * on one run's 10 breaths (peak 77 L/min) the end-inspiration spike was
 * +1.58 cmH2O (twin with the old defaults on the same breaths: +2.56), width
 * 0.47 s (0.66), p-p 3.58 (4.25), inspiratory dip unchanged (~1.5-1.8 on
 * breaths this large). On AirSense-sized breaths (~55 L/min) the twin gives
 * dip 0.54, spike 1.02, width 0.37 s vs AirSense 11 ~0.3-0.5 / 1.3-1.5 / 0.3.
 *
 * 2026-10-02 evening: 6 -> 4. The twin above had no loop delays and called
 * speed_kp 2500/3000 and a CM4 acceleration feed-forward stable; on the bench
 * all of them oscillated at 11-12 Hz into DP_FAULT. With 5 ms pressure-path +
 * 5 ms command-path delay + 3 ms current lag (fitted to those bench results)
 * and a breathing-only damping term, the twin reproduces all five bench
 * outcomes and breathing to mean |err| 0.12 cmH2O. The x6 jump is what
 * eats the outer loop's stability margin. On eight recorded 20 cmH2O breath
 * sets, 4 vs 6: dip 1.70 -> 1.59, p90 2.63 -> 2.58, spike 1.99 -> 1.87,
 * width 0.51 -> 0.49, p-p 3.70 -> 3.59; on the AirSense 11 run's breaths
 * dip 1.26 -> 0.83 (AirSense measured 0.84). Stable with +2 ms extra delay.
 * Bench (CPAP 20<->18, vent only): no faults, jitter / iq_ref sd / rest
 * pressure sd equal to 6 once warm. Breathing (ovp420_20261002_183316, fast
 * deep breaths): no faults; twin on those breaths 4 vs 6 dip 2.46 vs 2.45,
 * spike 2.09 vs 2.19, p-p 4.55 vs 4.83 (twin predicted the run's measured
 * dip 2.39 to within 0.07). */
volatile float   g_blower_ctrl_over_p_gain          = 4.0f;
volatile float   g_blower_ctrl_over_p_thresh_cmh2o  = 0.30f;

/* 2026-10-01: breathing-dependent over-pressure threshold.
 *
 * Low threshold (over_p_thresh_exp, 0.12) while the patient is breathing,
 * rest threshold (over_p_thresh, 0.30) once no real breath has been seen for
 * over_p_exp_window seconds - i.e. on the bench, mask-on idle, long apneas.
 *
 * Why, from A/Bs at 20 cmH2O with breathing (simulator ~10 BPM, 18 L/min
 * peak, 2 x 60 s blocks each, port pressure 5 Hz low-pass):
 *     0.12 always          breath p-p 0.72, fast (0.8-5 Hz) sd 0.122
 *     0.30 always          breath p-p 0.91, fast sd 0.147
 *     0.12 for 1.5 s after each inspiration, else 0.30
 *                          breath p-p 0.97, fast sd 0.149
 * The averaged breath waveforms of all three agree within 0.1 cmH2O, so the
 * low threshold does not act on an end-of-inspiration overshoot (the first
 * version of this, a 1.5 s post-inspiration window, was based on that idea
 * and did nothing). It tightens fast wobble throughout the breath: while
 * breathing the error swings ~+/-0.3, so 0.12 is crossed by real disturbance.
 * At rest the error is only ~0.08 of sensor noise and 0.12 is crossed by
 * noise - the rotor jitter that moved the rest value to 0.30.
 *
 * The window is re-armed at the end of every real inhalation and outlasts a
 * breath period, so during breathing the low threshold is continuous.
 * Arming: the inhalation must last >= over_p_min_insp_s with patient flow
 * reaching >= over_p_min_insp_q_slm. The inhaling flag itself flickers on
 * flow-sensor noise at rest (up to 25 drops/min at 20 cmH2O, vent only);
 * replayed against the 2026-09-30/10-01 logs these two gates armed nothing
 * at rest at 4-20 cmH2O and once per breath when breathing.
 *
 * (Variable and command names keep "exp" from the first version.) */
volatile float   g_blower_ctrl_over_p_thresh_exp_cmh2o = 0.12f;
volatile float   g_blower_ctrl_over_p_exp_window_s     = 10.0f;
volatile float   g_blower_ctrl_over_p_min_insp_s       = 0.3f;
volatile float   g_blower_ctrl_over_p_min_insp_q_slm   = 8.0f;
volatile float   g_blower_ctrl_over_p_thresh_active    = 0.30f;  /* readback */

/* 2026-10-01: post-hand-over speed ramp.
 *
 * Starts at 20 cmH2O failed with DP_FAULT (STSPIN830 EN/FAULT) 0.28-0.44 s
 * after the rev-up -> sensorless hand-over, at 24-28 krpm. With pressure
 * still ~0 the loop commands the ceiling, CM4 executes that buffered target
 * at hand-over, and the drive accelerates at ~35 krpm/s with iq_ref pinned at
 * IQMAX (sampled iq 1.3-1.7 A). With the motor's ~50-70 uH in-situ inductance
 * the 20 kHz ripple rides on top of that, and the instantaneous peaks reach
 * the driver's internal over-current threshold (3 A min / 3.5 A typ,
 * STSPIN830 datasheet). 16 starts at 20 cmH2O: every fresh start faulted,
 * with speed_ki 1024 / rpm_max 43000 and with 256 / 30000 alike.
 *
 * So the speed ceiling is held at the hand-over speed while the drive is not
 * in RUN (CM4 buffers the command during rev-up and applies it at
 * switch-over), then raised at start_ramp_rpm_per_s once it is. Applies to
 * every entry into RUN: therapy start, and restarts after a fault or stop.
 * Breathing in steady therapy is unaffected - the drive stays in RUN.
 * 0 disables. Runtime: "blower set start_ramp <rpm/s>". */
volatile float   g_blower_ctrl_start_ramp_rpm_per_s = 12000.0f;
volatile float   g_blower_ctrl_start_ramp_base_rpm  = 15000.0f;  /* rev-up ends at 14405 */
volatile float   g_blower_ctrl_start_ramp_limit_rpm = 0.0f;      /* readback, 0 = inactive */

/* Overpressure output brake (0 = disabled). */
volatile float   g_blower_ctrl_over_brake_rpm_per_s = 6000.0f;

/* EPR expiratory bump mitigation: zero disturbance FF + integrator bleed on
 * exhale onset, and gate flow/dip FF while EPR is at expiratory pressure.
 * Default off — set to 1 in debugger to re-enable. */
volatile uint8_t g_blower_ctrl_epr_exhale_mit_enable = 0U;

/* Floor 2 of the four described below. When set, and EPR is off, the final
 * command is held >= the sqrt(P) steady state for the active slewed setpoint.
 * Note this scales with ff_offset as well as ff_k, so it rises one-for-one
 * with any DC correction applied to the FF. Now defaults OFF - see the
 * "Expiratory RPM floors" block. */
volatile uint8_t g_blower_ctrl_rpm_floor_enable = 0U;

/* Expiratory RPM floors — ALL FOUR NOW DEFAULT OFF =========================
 *
 *  Holding a fixed mask pressure while the patient exhales into the circuit
 *  needs the blower BELOW the speed that holds that pressure at zero flow:
 *  the returning breath is doing part of the work. Four guards used to
 *  forbid that, so an expiratory bump could only bleed off passively:
 *
 *    1. g_blower_ctrl_ff_ss_floor_enable      - physics FF target >= K*sqrt(SP)
 *    2. g_blower_ctrl_rpm_floor_enable        - final command >= the same floor
 *    3. g_blower_ctrl_exhale_integ_clamp_enable - integrator forced >= 0 on the
 *                                                inspiration -> expiration edge
 *    4. g_blower_ctrl_vent_floor_enable       - >= the speed that holds SP
 *                                               against the RESTING vent flow
 *
 *  Bench A/B (2026-09-04, 8 and 5 cmH2O against an AirSense 11 on a shared
 *  breath simulator) cleared all four. The expiratory hump went from a ~1.8 s
 *  passive decay to a ~0.36 s corrected return, and the inspiratory dip fell
 *  from -0.60 to -0.35 cmH2O. At 5 cmH2O the dip now matches the AirSense
 *  (-0.31 both) and the end-inspiratory spike is smaller (+0.50 vs +0.57).
 *
 *  PRECONDITION: clearing these is only safe while the feed-forward is
 *  DC-correct. With ff_offset at its old 0 the FF sat ~2 krpm low and
 *  dragged the output down every expiration. Do not clear the floors and
 *  reset ff_offset in the same change - calibrate the FF first (see
 *  g_blower_ctrl_ff_offset_rpm).
 *
 *  Set any back to 1 to restore the old guard if expiratory SAG appears on a
 *  different circuit. The better fix for sag is a signed, vent-aware plant
 *  inverse (the RPM that holds P_target at the actual current total flow,
 *  either direction) rather than a floor.
 */
volatile uint8_t g_blower_ctrl_ff_ss_floor_enable        = 0U;
volatile uint8_t g_blower_ctrl_exhale_integ_clamp_enable = 0U;
/* Floor 4, and the one that binds hardest during expiration: the speed that
 * holds SP against the RESTING vent flow. Unlike the other three it scales
 * with ff_k, so raising K to close a steady-state gap also raises this pin.
 * See physicsFfVentFloorRpm(). */
volatile uint8_t g_blower_ctrl_vent_floor_enable         = 0U;

/* Zero a negative integrator during expiration while under-pressure.
 *
 * MEASURED CAUSE OF THE SECOND EXPIRATORY PRESSURE HUMP - now defaults OFF.
 * Term telemetry (capture_20260908_132604.log, 18-breath ensemble from peak
 * expiratory flow) shows the mechanism directly:
 *
 *    t(s)   P_mask     ff      p       i
 *    0.80   +0.061   -265   -106   -429     I doing useful negative work
 *    1.00   +0.017   -232    -34   -257     first hump corrected
 *    1.20   +0.091   -160   -160    +55     <- I dumps 312 rpm, hump returns
 *
 * The loop corrects the expiratory bump, slightly overshoots so that
 * error > 0.04, and this guard then discards the very integral term that was
 * holding pressure down. Pressure springs back and I has to rebuild from
 * zero, which is the slow ~1 s tail after it. Across the ensemble only I
 * moves WITH the hump (r = +0.15); p and ff correlate negatively (-0.92,
 * -0.71) because they are responding to it, not causing it.
 *
 * STAYS OFF in the tuned configuration - turning it on reinstates the
 * measured fault. Set to 1 only if genuine expiratory UNDERSHOOT appears
 * - that is what it was added for. The better fix in that case is to bound
 * how negative I may go, not to erase it outright. */
volatile uint8_t g_blower_ctrl_exp_neg_i_clamp_enable    = 0U;

/* Setpoint slew rate (down / general). 6 cmH2O/s for downward steps and
 * manual ramp-up from idle. EPR recovery uses g_blower_ctrl_slew_up. */
volatile float g_blower_ctrl_slew_cmh2o_per_s = 18.0f;

/* Faster slew when the user setpoint rises (EPR restore, inhale onset).
 * 18 cmH2O/s -> 3 cmH2O step in ~170 ms. */
volatile float g_blower_ctrl_slew_up_cmh2o_per_s = 18.0f;

/* Output post-filter (1st order IIR). 1.0 = pass-through, 0.2 = heavy.
 *
 * Set to 1.0 (pass-through) so that PID output reaches the motor with
 * zero filter lag - we need every millisecond of bandwidth to fight
 * breath-cycle pressure swings. The motor's own mechanical inertia
 * provides any smoothing the system needs. Drop to 0.5-0.7 only if
 * downstream noise (audible whine) becomes a problem. */
volatile float g_blower_ctrl_out_filter_alpha = 1.0f;

/* RPM clamp. Defaults to the IPC ceiling but kept tunable in case you
 * want to limit the blower further during development. */
volatile int32_t g_blower_ctrl_rpm_max = (int32_t)BLOWER_IPC_MAX_SPEED_RPM;
volatile int32_t g_blower_ctrl_rpm_min = 0;          /* never reverse */

/* Integrator deadband and magnitude clamp (anti-windup).
 *
 *   - Deadband: |error| > deadband -> freeze integrator. This stops the
 *     I-term from winding up during the slewing phase of a step change
 *     (where P+FF should carry the load) and dedicates I purely to
 *     steady-state trim near the target. Default 2.5 cmH2O comfortably
 *     covers the breath-cycle pressure swing on any normal mask.
 *   - Magnitude clamp: hard limit on |I| in RPM. Even a very leaky
 *     circuit shouldn't need more than 4 krpm of integrator trim - past
 *     that is windup, not real correction.
 *
 * Both are tunable from the watch window. Set deadband to 1e6 to
 * recover the original "always integrate" behaviour. */
volatile float   g_blower_ctrl_integ_deadband_cmh2o = 1.2f;

/* The deadband above was documented and GUI-exposed but never read, so the
 * gains in the field were all tuned with it inactive. It is wired up now,
 * but stays OFF by default: switching it on changes integrator behaviour at
 * every setpoint step and mask event, and that must be an explicit,
 * separately-tested decision rather than a side effect of a firmware update.
 *      blower set integ_deadband_enable 1
 */
volatile uint8_t g_blower_ctrl_integ_deadband_enable = 0U;

/* Low-pass the error that drives the integrator.
 *
 * The breath cycle is a ~0.18 Hz periodic disturbance the integrator should
 * not chase: measured over 41 breaths, I winds to +623 rpm fighting the
 * inspiratory dip and is still +476 rpm when the end-inspiratory spike
 * peaks, while ff and p have each shed ~1000 rpm. That leftover push is the
 * spike (476 rpm ~ 0.29 cmH2O against a 0.317 spike).
 *
 * The deadband CANNOT fix this. It freezes I when |error| > band, so a band
 * below the breath swing deadlocks: error stays outside, I stays frozen,
 * nothing ever corrects. Measured 2026-09-08 with band 0.12 - I pinned at
 * its -4000 clamp and mask pressure sat at 3.9 against a 5.0 setpoint.
 *
 * Filtering instead of gating avoids that entirely. At tau = 15 s the breath
 * component is attenuated ~94% (omega*tau ~ 17 at a 5.5 s period) while DC
 * passes untouched, so I still trims the steady-state offset and simply
 * stops responding per-breath. No band, so no deadlock is possible.
 *
 * Anti-windup below still uses the RAW error - saturation is an
 * instantaneous condition and must not be filtered. Default OFF. */
volatile uint8_t g_blower_ctrl_integ_err_lpf_enable = 1U;
volatile float   g_blower_ctrl_integ_err_lpf_tau_s  = 15.0f;

/* Warm-up ramp on the LPF time constant.
 *
 * The lag above sits INSIDE the integral path, so error -> LPF -> integrator
 * is a double lag: the loop is second order with zeta = 1/(2*sqrt(ki*G*tau)).
 * At tau = 15 s that measured zeta ~ 0.17 - a ~32 s ring-down taking ~110 s
 * to settle, with I pinned at +integ_max for the first ~18 s because the
 * filtered error was still positive from the pressure ramp long after the
 * raw error had gone negative.
 *
 * During that transient the filter has nothing to reject: the offset dwarfs
 * the per-breath ripple it exists to remove. So start at tau_start and ramp
 * linearly to the configured tau over warmup_s, measured from the seed. The
 * loop converges near-critically damped, then slows for steady state.
 *
 * warmup_s <= 0 disables the ramp (straight to the configured tau). */
volatile float   g_blower_ctrl_integ_err_lpf_tau_start_s = 1.0f;
volatile float   g_blower_ctrl_integ_err_lpf_warmup_s    = 45.0f;

/* The warm-up clock only runs while the loop is actually settled.
 *
 * A pure wall-clock ramp grows tau while the loop is still ringing, which is
 * exactly when the extra lag hurts: measured, tau was already 7.2 s at t=20 s
 * and 13.4 s at t=40 s, so the second half of the transient ran at nearly the
 * full lag and the ring-down survived. Gating on |filtered error| holds tau at
 * tau_start until convergence, then ramps.
 *
 * Band must clear the breath ripple that leaks through at tau_start, or the
 * clock never starts: measured at tau = 1 s the filtered error still swings
 * +/-0.097 cmH2O, against 0.046 at tau = 3 s. 0.15 clears the first with
 * margin and is far below the 0.3+ excursions of the startup transient.
 *
 * The clock runs backwards when out of band, so a mid-session disturbance
 * pulls tau back down instead of leaving the loop stuck at the full lag.
 * Band <= 0 disables the gate and gives a plain wall-clock ramp. */
volatile float   g_blower_ctrl_integ_err_lpf_settle_band_cmh2o = 0.15f;
/* Diagnostics for the block above. Watch these when the BASELINE pressure
 * wanders between breaths: tau_now collapsing back toward tau_start means the
 * filtered error has left the settle band, the warm-up clock is running
 * backwards, and the integrator has become breath-reactive - which moves the
 * baseline instead of holding it. Bigger or more irregular breaths (a real
 * patient rather than the bench simulator) are what push it out of band. */
volatile float   g_blower_ctrl_integ_err_filt_cmh2o    = 0.0f;
volatile float   g_blower_ctrl_integ_err_lpf_tau_now_s = 0.0f;
volatile float   g_blower_ctrl_integ_max_abs_rpm    = 4000.0f;

/* Bench switch: freeze the integrator where it is.
 *
 * The integrator is what moves the BASELINE pressure - it is the only term
 * that can hold a steady offset, so when it reacts to breathing (see the
 * settle-band note above) the between-breath level wanders. Setting this to 1
 * stops it accumulating while leaving the trim it has already earned in
 * place, so P and the feed-forward keep working and the baseline stops
 * moving. Useful for judging the raw P + FF waveform without the integrator
 * shifting the reference underneath it.
 *
 * Not a substitute for tuning: with the integrator held, any genuine
 * steady-state error stays uncorrected. The magnitude clamp still applies to
 * the frozen value; the headroom clamp no longer drags it (see below). */
volatile uint8_t g_blower_ctrl_integ_hold           = 0U;

/* Headroom clamp: hold the integrator, don't drag it.
 *
 * The clamp to remaining RPM headroom sets I <= rpm_max - FF - P - D. When
 * FF + P ALONE exceed the ceiling - every inspiratory peak at 20 cmH2O, where
 * they reach 42-51 krpm against 40000 - that bound is negative and the clamp
 * pulled I down to its -4000 magnitude limit on nearly every breath, while
 * the pressure error was POSITIVE (under setpoint). It changed nothing at the
 * peak itself, where the output is clipped to 40000 either way. But the
 * negative I outlived the peak: measured 2026-09-24 at 20 cmH2O (human
 * breathing, normal circuit), it withheld ~1200 rpm for 0.5-1.4 s after peak
 * flow while pressure was still below setpoint, and wiped the ~+750 rpm
 * steady trim I carries between breaths, which then had to be re-earned at
 * the ki rate every breath. This was the "integrator pinned at -4000 at peak
 * flow" seen since the 2026-09-22 captures - clamp memory, not a model error.
 *
 * With this on, the clamp still stops I from growing into saturation (it
 * cannot rise above max(bound, its previous value)), but it no longer pushes
 * I below where it already was; the low side is symmetric. Genuine unwinding
 * is left to normal integration: conditional integration above still steps I
 * toward the error while saturated when that step reduces saturation, and
 * the magnitude clamp is unchanged. The output itself is still clamped to
 * [rpm_min, rpm_max] below, so no rpm limit is relaxed.
 *
 *      blower set integ_headroom_hold 0      (previous behaviour, for A/B) */
volatile uint8_t g_blower_ctrl_integ_headroom_hold_enable = 1U;

/* IIR on pressure before PID (WM6850 used alpha_meas = 0.50). */
volatile float   g_blower_ctrl_meas_filter_alpha    = 0.45f;

/* LPF on D-on-measurement (WM6850 d_alpha ~ 0.20). */
volatile float   g_blower_ctrl_deriv_filter_alpha = 0.25f;

/* Low-pass on the error the P term sees, Hz. 0 = off (previous behaviour).
 *
 * Added 2026-10-01. The speed command jittered +/-1.3-1.9 krpm tick to tick
 * on the expiratory plateau at 16 cmH2O. At that setpoint kp is 1000
 * rpm/cmH2O and jumps x6 past -over_p_thresh (0.12 while breathing) and
 * x3.5 past +under_p_thresh (0.08) - so a few hundredths of a cmH2O of
 * sensor noise near a threshold steps the command by hundreds of rpm, and
 * meas_filter_alpha 0.45 at 200 Hz is only a ~39 Hz filter. Every step goes
 * to CM4 with ramp 0; at speed_kp 4000 those steps became +/-1.5 A current
 * chatter and tripped DP_FAULT in therapy.
 *
 * The filtered error drives both the P term and the gain-switch decisions.
 * The integrator, the over-pressure brake and the FF are unchanged, so the
 * breath-rate path keeps its timing (a 10 Hz first-order adds ~16 ms to P
 * only). Bench, CM7 playback of a recorded 16 cmH2O command, leak rig: the
 * command low-passed at 10 Hz with speed_kp 4000 cut rotor over-speed on
 * the falling phase from +514 rpm (raw, kp 1637) to +7 rpm with no faults.
 *
 * Closed-loop therapy on the bench (16 cmH2O, vent only, no breathing,
 * speed_kp 1637), off -> 10 Hz: command jitter 792 -> 453 rpm per 20 ms,
 * iq_ref sd 230 -> 153 mA, 16<->18 step rise 310 -> 270 ms, overshoot
 * 0.62 -> 0.56 cmH2O; rest pressure sd 0.199 -> 0.229. Default 10 since.
 *      blower set p_err_lpf_hz 0       (previous behaviour, for A/B) */
volatile float   g_blower_ctrl_p_err_lpf_hz = 10.0f;
volatile float   g_blower_ctrl_p_err_filt_cmh2o = 0.0f;   /* readback */

/* Freeze I while internal setpoint slew is catching up at therapy start. */
volatile float   g_blower_ctrl_startup_slew_gap_cmh2o = 0.25f;

/* === Public configuration (debugger-visible) === */
volatile uint8_t g_blower_ctrl_enabled        = 0U;       /* default OFF */
volatile uint8_t g_blower_ctrl_pid_enable     = 1U;       /* P/I/D trim on */
volatile uint8_t g_blower_manual_active       = 0U;
volatile float   g_blower_ctrl_setpoint_user  = 6.0f;     /* cmH2O */

/* === State (debugger-visible) === */
volatile float   g_blower_ctrl_setpoint_slew  = 0.0f;
volatile float   g_blower_ctrl_meas_cmh2o     = 0.0f;
volatile float   g_blower_ctrl_sensor_cmh2o  = 0.0f;   /* AMS5935 at blower */
volatile float   g_blower_ctrl_meas_filt_cmh2o = 0.0f;
volatile float   g_blower_ctrl_error_cmh2o    = 0.0f;
volatile float   g_blower_ctrl_integ          = 0.0f;
volatile float   g_blower_ctrl_deriv_rpm_per_s = 0.0f;
volatile float   g_blower_ctrl_p_term         = 0.0f;
volatile float   g_blower_ctrl_i_term         = 0.0f;
volatile float   g_blower_ctrl_d_term         = 0.0f;
volatile float   g_blower_ctrl_ff_term        = 0.0f;
volatile float   g_blower_ctrl_ff_flow_term   = 0.0f;
volatile float   g_blower_ctrl_ff_dip_term    = 0.0f;
volatile float   g_blower_ctrl_ff_rise_term   = 0.0f;
volatile float   g_blower_ctrl_ff_exp_flow_term = 0.0f;
volatile float   g_blower_ctrl_out_unfilt     = 0.0f;
volatile float   g_blower_ctrl_out_filt       = 0.0f;
volatile int32_t g_blower_ctrl_out_rpm        = 0;
volatile uint8_t g_blower_ctrl_zone           = (uint8_t)BLOWER_CTRL_ZONE_LOW;
volatile uint8_t g_blower_ctrl_saturated      = 0U;
volatile uint8_t g_blower_ctrl_upper_limited  = 0U;
volatile uint32_t g_blower_ctrl_step_count    = 0U;

/* Previous-tick measurement for derivative-on-measurement. */
static float s_prev_meas_cmh2o = 0.0f;
static bool  s_prev_meas_valid = false;
static float s_d_meas_filt_per_s = 0.0f;
static float s_ff_flow_slew_rpm = 0.0f;
static float s_ff_dip_slew_rpm  = 0.0f;
static float s_ff_rise_slew_rpm = 0.0f;
static float s_ff_exp_flow_slew_rpm = 0.0f;
static float s_integ_err_filt_cmh2o = 0.0f;
static bool  s_integ_err_filt_valid = false;
static float s_integ_err_filt_age_s = 0.0f;
static float s_exp_q_prev_slm = 0.0f;
static bool  s_exp_q_prev_valid = false;
static float s_exp_dq_filt_slm_per_s = 0.0f;
static float s_prev_q_pat_slm   = 0.0f;
static bool  s_prev_q_pat_valid = false;
/* After mask reconnect, Vivo Q_patient is briefly huge (MeanRatio reseeds to
 * standard while total Q is still filling) — suppress patient-flow FF so
 * pressure does not overshoot until Q_pat returns near zero. */
static uint32_t s_patient_ff_suppress_ms = 0U;
static float    s_resume_rpm_floor = 0.0f;
static uint32_t s_resume_rpm_floor_ms = 0U;
static float s_p_drop_filt_per_s = 0.0f;
static bool  s_p_drop_filt_valid = false;
static float s_prev_p_sensor_cmh2o = 0.0f;
static bool  s_prev_p_sensor_valid = false;
static float s_meas_filt_cmh2o  = 0.0f;
static bool  s_meas_filt_valid  = false;
static float s_p_err_filt_cmh2o = 0.0f;   /* see g_blower_ctrl_p_err_lpf_hz */
static bool  s_p_err_filt_valid = false;
static float s_physics_ff_q_filt_slm = 0.0f;
static bool  s_physics_ff_q_filt_valid = false;
static float s_physics_ff_slew_rpm = 0.0f;
static bool  s_physics_ff_was_active = false;
static bool  s_physics_prev_inhaling = false;
static bool  s_physics_phase_inhaling = false;  /* confirmed phase, onset snaps only */
static float s_prev_physics_q_in_slm = 0.0f;
static bool  s_prev_physics_q_valid  = false;
static float s_physics_onset_dqdt_filt = 0.0f;
static float s_physics_exp_lead_q_prev = 0.0f;
static bool  s_physics_exp_lead_q_valid = false;
static float s_physics_exp_lead_dqdt_filt = 0.0f;
static bool  s_epr_insp_pressure_prev = false;
/* WM6850 mode: SensorTask still calls Step @ 200 Hz; PID runs every 2nd tick. */
static uint8_t s_wm6850_skip_phase = 0U;

/* Consecutive ticks without a fresh PS sample. Watched so a sensor-side
 * stall can be seen in the debugger / telemetry instead of silently
 * degrading the D path. */
#define BLOWER_CTRL_PS_STALE_MAX 10U
static uint8_t s_ps_stale_ticks = 0U;
volatile uint8_t g_blower_ctrl_ps_stale_ticks = 0U;

/* BLOWER_CTRL_C_FAN_MAX now lives in blower_ctrl.h - the USB setter has to
 * clamp to the same value and used to carry its own stale copy. */

#define BLOWER_CTRL_WM6850_DT_S 0.01f
#define BLOWER_CTRL_TWO_PI      6.28318530717958647692f

static void resetPhysicsFfState(void)
{
    s_physics_ff_q_filt_slm    = 0.0f;
    s_physics_ff_q_filt_valid  = false;
    s_physics_ff_slew_rpm      = 0.0f;
    s_prev_physics_q_in_slm    = 0.0f;
    s_prev_physics_q_valid     = false;
    s_physics_onset_dqdt_filt  = 0.0f;
    s_physics_exp_lead_q_prev    = 0.0f;
    s_physics_exp_lead_q_valid   = false;
    s_physics_exp_lead_dqdt_filt = 0.0f;
    g_blower_ctrl_physics_ff_exp_lead_rpm = 0.0f;
    g_blower_ctrl_physics_ff_q_filt_slm = 0.0f;
    g_blower_ctrl_physics_ff_rpm_raw    = 0.0f;
}

/* === Helpers === */

static float clampf(float v, float lo, float hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static int32_t clampi(int32_t v, int32_t lo, int32_t hi)
{
    if (v < lo) return lo;
    if (v > hi) return hi;
    return v;
}

static void slewTowardFloat(float *val, float target, float max_step)
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

/* Raw vs LPF patient flow in the inhale direction (earliest / largest).
 * Raw is only mixed in once filtered flow is already past the deadband;
 * pause-region sensor hash must not drive dQ/dt onset FF. */
static float inhalePatientFlowSlm(void)
{
    const float q_filt = Flow_GetPatientSlm();
    const float q_raw  = Flow_GetPatientRawSlm();
    const float db = clampf(g_blower_ctrl_flow_ff_deadband_slm, 0.0f, 15.0f);
    if (g_blower_ctrl_flow_ff_inhale_sign >= 0)
    {
        if (q_filt > db)
        {
            return (q_raw > q_filt) ? q_raw : q_filt;
        }
        return q_filt;
    }
    if (q_filt < -db)
    {
        return (q_raw < q_filt) ? q_raw : q_filt;
    }
    return q_filt;
}

static bool patientInhaling(void)
{
    const float db = clampf(g_blower_ctrl_flow_ff_deadband_slm, 0.0f, 15.0f);
    const float q  = inhalePatientFlowSlm();
    if (g_blower_ctrl_flow_ff_inhale_sign >= 0)
    {
        return q > db;
    }
    return q < -db;
}

/* Breathing-dependent over-pressure threshold - see the note at
 * g_blower_ctrl_over_p_thresh_exp_cmh2o. Call once per control tick. */
static float s_overp_insp_s    = 0.0f;   /* length of the current inhalation */
static float s_overp_insp_qpk  = 0.0f;   /* its peak patient flow, slm */
static float s_overp_window_s  = 0.0f;   /* time left before falling back to rest */
static bool  s_overp_prev_insp = false;

static void resetOverPressurePhase(void)
{
    s_overp_insp_s    = 0.0f;
    s_overp_insp_qpk  = 0.0f;
    s_overp_window_s  = 0.0f;
    s_overp_prev_insp = false;
}

static float overPressureThreshold(float dt_s)
{
    const bool inhaling = patientInhaling();

    if (inhaling)
    {
        const float q = fabsf(inhalePatientFlowSlm());
        s_overp_insp_s += dt_s;
        if (q > s_overp_insp_qpk) { s_overp_insp_qpk = q; }
    }
    else if (s_overp_prev_insp)
    {
        /* Inhalation just ended: arm only if it was a real breath. */
        if ((s_overp_insp_s >= g_blower_ctrl_over_p_min_insp_s)
            && (s_overp_insp_qpk >= g_blower_ctrl_over_p_min_insp_q_slm))
        {
            s_overp_window_s = clampf(g_blower_ctrl_over_p_exp_window_s, 0.0f, 15.0f);
        }
        s_overp_insp_s   = 0.0f;
        s_overp_insp_qpk = 0.0f;
    }
    s_overp_prev_insp = inhaling;

    float thresh = g_blower_ctrl_over_p_thresh_cmh2o;
    if (s_overp_window_s > 0.0f)
    {
        s_overp_window_s -= dt_s;
        thresh = g_blower_ctrl_over_p_thresh_exp_cmh2o;
    }
    g_blower_ctrl_over_p_thresh_active = thresh;
    return thresh;
}

/* Post-hand-over speed ramp - see the note at
 * g_blower_ctrl_start_ramp_rpm_per_s. Returns the speed ceiling to apply this
 * tick (rpm_max_f when the ramp is not active). Call once per control tick. */
#define BLOWER_CTRL_MC_STATE_RUN 6U   /* MCI_State_t RUN, CM4 mc_interface.h */
extern BlowerIpcStatus_t g_blower_status;   /* main.c, refreshed at 100 Hz */
static bool  s_start_ramp_armed = true;
static float s_start_ramp_rpm   = 0.0f;

static void resetStartRamp(void)
{
    s_start_ramp_armed = true;
    s_start_ramp_rpm   = g_blower_ctrl_start_ramp_base_rpm;
}

static float startRampCeilingRpm(float dt_s, float rpm_max_f)
{
    const float rate = g_blower_ctrl_start_ramp_rpm_per_s;
    if (!(rate > 0.0f))
    {
        s_start_ramp_armed = false;
        g_blower_ctrl_start_ramp_limit_rpm = 0.0f;
        return rpm_max_f;
    }

    if (g_blower_status.mc_state != BLOWER_CTRL_MC_STATE_RUN)
    {
        /* Not under sensorless speed control yet (idle, rev-up, switch-over,
         * stopping, fault): whatever is posted now is what CM4 applies at
         * the hand-over, so keep it at the hand-over speed. */
        s_start_ramp_armed = true;
        s_start_ramp_rpm   = g_blower_ctrl_start_ramp_base_rpm;
    }
    else if (s_start_ramp_armed)
    {
        s_start_ramp_rpm += rate * dt_s;
        if (s_start_ramp_rpm >= rpm_max_f)
        {
            s_start_ramp_armed = false;   /* ramp has reached the ceiling */
        }
    }

    if (!s_start_ramp_armed)
    {
        g_blower_ctrl_start_ramp_limit_rpm = 0.0f;
        return rpm_max_f;
    }
    g_blower_ctrl_start_ramp_limit_rpm = s_start_ramp_rpm;
    return (s_start_ramp_rpm < rpm_max_f) ? s_start_ramp_rpm : rpm_max_f;
}

/* End of expiration: patient flow returning to zero before next inhale. */
static bool physicsFfExpTailNearZero(void)
{
    if (g_blower_ctrl_physics_ff_exp_tail_hold_enable == 0U)
    {
        return false;
    }
    if (patientInhaling())
    {
        return false;
    }

    const float thresh = clampf(g_blower_ctrl_physics_ff_exp_tail_flow_slm, 0.5f, 10.0f);
    const float q_pat = Flow_GetPatientSlm();
    return fabsf(q_pat) < thresh;
}

/* EPR lowers expiratory setpoint; flow/dip FF must not fight that baseline. */
static bool eprDisturbanceFfAllowed(void)
{
    if (!Epr_IsEnabled())
    {
        return true;
    }
    if (g_blower_ctrl_epr_exhale_mit_enable == 0U)
    {
        return true;
    }
    return Epr_IsInspiratoryPressureActive();
}

static float steadyStateRpmForSetpoint(float setpoint_cmh2o)
{
    float sp = (setpoint_cmh2o > 0.0f) ? setpoint_cmh2o : 0.0f;
    return g_blower_ctrl_ff_k_rpm_per_sqrtcmh * sqrtf(sp)
         + ffOffsetRpm(sp);
}

static bool wm6850ModeActive(void);

static bool physicsFfActive(void)
{
    return (g_blower_ctrl_physics_ff_enable != 0U) && !wm6850ModeActive();
}

static void eprExhaleOnsetTrim(float setpoint_cmh2o)
{
    s_ff_flow_slew_rpm = 0.0f;
    s_ff_dip_slew_rpm  = 0.0f;

    const float ss = steadyStateRpmForSetpoint(setpoint_cmh2o);
    if (physicsFfActive())
    {
        s_physics_ff_slew_rpm = ss;
        g_blower_ctrl_physics_ff_rpm_raw = ss;
    }

    if (g_blower_ctrl_integ > ss)
    {
        g_blower_ctrl_integ = ss;
    }
}

static bool anyOpenLoopFfActive(void)
{
    return (g_blower_ctrl_ff_enable != 0U)
        || (g_blower_ctrl_flow_ff_enable != 0U)
        || (g_blower_ctrl_dip_ff_enable != 0U)
        || physicsFfActive();
}

static float hoseDropMagnitudeCmh2o(float q_slm)
{
    /* The FF's own frozen basis, not tube comp - see
     * g_blower_ctrl_physics_ff_hose_r_lam. */
    const float qabs = fabsf(q_slm);
    return g_blower_ctrl_physics_ff_hose_r_lam * qabs
         + g_blower_ctrl_physics_ff_hose_r_turb * qabs * qabs;
}

static float physicsFfRpmForSetpointAndFlow(float setpoint_cmh2o, float q_slm);
static float ffOffsetRpm(float setpoint_cmh2o);

static float physicsFfFlowInputSlm(void)
{
    if (g_blower_ctrl_physics_ff_use_total_flow != 0U)
    {
        /* Ctrl variants: fan-map estimate above the dP sensor range (flow_sensor.c) */
        const float q_filt = Flow_GetSlmCtrl();
        const float q_raw  = Flow_GetSlmRawCtrl();
        float q = (q_filt > 0.0f) ? q_filt : 0.0f;

        if (q_raw > q)
        {
            q = q_raw;
        }
        return q;
    }

    return inhalePatientFlowSlm();
}

static float physicsFfVentFloorQSlm(void)
{
    float q = Leak_GetVentSlm();
    const float reported = Leak_GetReportedLeakSlm();

    if (reported > q)
    {
        q = reported;
    }
    if (q < 10.0f)
    {
        q = 10.0f;
    }
    return q;
}

static float physicsFfVentFloorRpm(float setpoint_cmh2o)
{
    /* Floor 4: the speed that would hold the setpoint against the RESTING
     * vent flow, applied regardless of what the patient is doing. During
     * expiration the patient supplies part of that flow - total flow through
     * the blower drops toward zero - so the machine should slow below this,
     * and the floor stops it. It also scales with ff_k, so recalibrating K
     * upward raises this pin by the same proportion and widens the
     * expiratory hump. Clearing the flag removes it everywhere it is used
     * (physicsFfClampRpmTarget, rpmFloorForSetpointCmh2o, the exp-tail and
     * hybrid holds), since they all compare against this return value. */
    if (g_blower_ctrl_vent_floor_enable == 0U)
    {
        g_blower_ctrl_physics_ff_vent_floor_rpm = 0.0f;
        return 0.0f;
    }

    const float q_vent = physicsFfVentFloorQSlm();
    const float rpm = physicsFfRpmForSetpointAndFlow(setpoint_cmh2o, q_vent);

    g_blower_ctrl_physics_ff_vent_floor_rpm = rpm;
    return rpm;
}

static float physicsFfClampRpmTarget(float setpoint_cmh2o, float rpm_target)
{
    const float vent_floor = physicsFfVentFloorRpm(setpoint_cmh2o);
    float rpm = rpm_target;

    if (rpm < vent_floor)
    {
        rpm = vent_floor;
    }
    /* Floor 1: never below the zero-flow steady state. Gated so the loop can
     * be allowed to slow through expiration - see the A/B block above. */
    if (g_blower_ctrl_ff_ss_floor_enable != 0U)
    {
        const float ss = steadyStateRpmForSetpoint(setpoint_cmh2o);
        if (rpm < ss)
        {
            rpm = ss;
        }
    }
    return rpm;
}

static float physicsFfRpmForSetpointAndFlow(float setpoint_cmh2o, float q_slm)
{
    float sp = (setpoint_cmh2o > 0.0f) ? setpoint_cmh2o : 0.0f;
    const float hose_drop = hoseDropMagnitudeCmh2o(q_slm);
    /* Ceiling raised 0.001 -> BLOWER_CTRL_C_FAN_MAX on 2026-09-08: tuning had
     * reached the old limit and was being silently clipped (the GUI showed
     * 0.0011 while the loop used 0.001). Physically the whole term is the
     * blower's fan-curve droop, c_fan*(Q_hi^2 - Q_lo^2); over this rig's
     * 14 -> 37 slm swing a plausible 1.5-2.5 cmH2O droop is c_fan
     * 0.0013-0.0021, so the new ceiling leaves headroom without admitting
     * nonsense. Note the term is quadratic - at the ceiling and 37 slm it
     * adds ~6.8 cmH2O to p_equiv, i.e. several krpm of feed-forward. */
    const float c_fan = clampf(g_blower_ctrl_physics_ff_c_fan_cmh2o_per_slm2,
                               0.0f, BLOWER_CTRL_C_FAN_MAX);
    const float c_fan_exp = clampf(g_blower_ctrl_physics_ff_c_fan_exp_cmh2o_per_slm2,
                                   0.0f, BLOWER_CTRL_C_FAN_MAX);
    const float q_pos = (q_slm > 0.0f) ? q_slm : 0.0f;
    /* Split around the resting flow - see g_blower_ctrl_physics_ff_c_fan_exp. */
    const float q_rest = physicsFfVentFloorQSlm();
    float fan_load;
    if (q_pos >= q_rest)
    {
        fan_load = c_fan * q_pos * q_pos;
    }
    else
    {
        fan_load = (c_fan * q_rest * q_rest)
                 - (c_fan_exp * ((q_rest * q_rest) - (q_pos * q_pos)));
        if (fan_load < 0.0f) { fan_load = 0.0f; }
    }
    const float p_equiv  = sp + hose_drop + fan_load;

    g_blower_ctrl_physics_ff_hose_drop_cmh2o = hose_drop;
    g_blower_ctrl_physics_ff_fan_load_cmh2o  = fan_load;
    g_blower_ctrl_physics_ff_p_equiv_cmh2o   = p_equiv;

    /* Offset on the SETPOINT, not p_equiv: the flow-dependent part is already
     * inside the sqrt above via hose_drop and fan_load. */
    return g_blower_ctrl_ff_k_rpm_per_sqrtcmh * sqrtf(p_equiv)
         + ffOffsetRpm(sp);
}

static float physicsFfFilterFlow(float q_slm, float dt_s, bool inhaling, bool exhaling)
{
    float fc;
    if (exhaling)
    {
        fc = clampf(g_blower_ctrl_physics_ff_flow_lpf_exhale_hz, 0.5f, 25.0f);
    }
    else if (inhaling)
    {
        fc = clampf(g_blower_ctrl_physics_ff_flow_lpf_inhale_hz, 0.5f, 25.0f);
    }
    else
    {
        fc = clampf(g_blower_ctrl_physics_ff_flow_lpf_hz, 0.5f, 15.0f);
    }
    const float omega_dt = BLOWER_CTRL_TWO_PI * fc * dt_s;
    const float alpha = omega_dt / (1.0f + omega_dt);

    if (!s_physics_ff_q_filt_valid)
    {
        s_physics_ff_q_filt_slm   = q_slm;
        s_physics_ff_q_filt_valid = true;
    }
    else
    {
        s_physics_ff_q_filt_slm = alpha * q_slm
                              + (1.0f - alpha) * s_physics_ff_q_filt_slm;
    }

    g_blower_ctrl_physics_ff_q_filt_slm = s_physics_ff_q_filt_slm;
    return s_physics_ff_q_filt_slm;
}

static void physicsFfOnInhaleOnset(void)
{
    const float q_now = physicsFfFlowInputSlm();

    s_physics_ff_q_filt_slm = q_now;
    g_blower_ctrl_physics_ff_q_filt_slm = q_now;
    s_physics_ff_q_filt_valid = true;
    s_prev_physics_q_in_slm = q_now;
    s_prev_physics_q_valid  = true;
}

static float physicsFfOnsetBoostRpm(float q_slm, float dt_s)
{
    if (g_blower_ctrl_physics_ff_onset_enable == 0U)
    {
        return 0.0f;
    }

    float dq_dt = 0.0f;
    if (s_prev_physics_q_valid && (dt_s > 0.0f))
    {
        dq_dt = (q_slm - s_prev_physics_q_in_slm) / dt_s;
    }
    s_prev_physics_q_in_slm = q_slm;
    s_prev_physics_q_valid  = true;

    const float dq_tau = clampf(g_blower_ctrl_physics_ff_onset_dqdt_tau_s,
                                0.0f, 0.5f);
    if (dq_tau > 0.0f)
    {
        const float a = dt_s / (dq_tau + dt_s);
        s_physics_onset_dqdt_filt = (a * dq_dt)
                                  + ((1.0f - a) * s_physics_onset_dqdt_filt);
        dq_dt = s_physics_onset_dqdt_filt;
    }
    else
    {
        s_physics_onset_dqdt_filt = dq_dt;
    }

    const float thresh = clampf(g_blower_ctrl_physics_ff_onset_thresh_slm_per_s,
                                10.0f, 500.0f);
    if (dq_dt <= thresh)
    {
        return 0.0f;
    }

    float boost = g_blower_ctrl_physics_ff_onset_k_rpm_per_slm_per_s * (dq_dt - thresh);
    const float boost_max = clampf(g_blower_ctrl_physics_ff_onset_max_rpm, 0.0f, 5000.0f);
    if (boost > boost_max)
    {
        boost = boost_max;
    }
    return boost;
}

/* See g_blower_ctrl_physics_ff_exp_lead_enable. Called every physics FF
 * tick so the derivative stays continuous across phases. */
static float physicsFfExpLeadRpm(float dt_s, bool exhaling)
{
    const float q = Flow_GetPatientSlm();
    float dq_dt = 0.0f;
    if (s_physics_exp_lead_q_valid && (dt_s > 0.0f))
    {
        dq_dt = (q - s_physics_exp_lead_q_prev) / dt_s;
    }
    s_physics_exp_lead_q_prev  = q;
    s_physics_exp_lead_q_valid = true;

    const float dq_tau = clampf(g_blower_ctrl_physics_ff_onset_dqdt_tau_s,
                                0.0f, 0.5f);
    if (dq_tau > 0.0f)
    {
        const float a = dt_s / (dq_tau + dt_s);
        s_physics_exp_lead_dqdt_filt = (a * dq_dt)
                                     + ((1.0f - a) * s_physics_exp_lead_dqdt_filt);
    }
    else
    {
        s_physics_exp_lead_dqdt_filt = dq_dt;
    }

    float lead = 0.0f;
    const float tail = clampf(g_blower_ctrl_physics_ff_exp_tail_flow_slm, 0.5f, 10.0f);
    const float thresh = clampf(g_blower_ctrl_physics_ff_exp_lead_thresh_slm_per_s,
                                10.0f, 500.0f);
    if ((g_blower_ctrl_physics_ff_exp_lead_enable != 0U) && exhaling
        && (q < -tail) && (s_physics_exp_lead_dqdt_filt > thresh))
    {
        lead = g_blower_ctrl_physics_ff_exp_lead_k_rpm_per_slm_per_s
             * (s_physics_exp_lead_dqdt_filt - thresh);
        const float lead_max = clampf(g_blower_ctrl_physics_ff_exp_lead_max_rpm,
                                      0.0f, 3000.0f);
        lead = clampf(lead, 0.0f, lead_max);
    }
    g_blower_ctrl_physics_ff_exp_lead_rpm = lead;
    return lead;
}

static void physicsFfOnExhaleOnset(float setpoint_cmh2o)
{
    const float q_now = physicsFfFlowInputSlm();
    const float vent_floor = physicsFfVentFloorRpm(setpoint_cmh2o);

    /* Hybrid: freeze RPM / I-term — do not snap FF or bleed integrator.
     * Coasting here is the main source of expiratory undershoot. */
    if (g_blower_ctrl_cpap_hybrid_active != 0U)
    {
        const float q_vent = physicsFfVentFloorQSlm();
        s_physics_ff_q_filt_slm = q_vent;
        g_blower_ctrl_physics_ff_q_filt_slm = q_vent;
        s_physics_ff_q_filt_valid = true;
        if (s_physics_ff_slew_rpm < vent_floor)
        {
            s_physics_ff_slew_rpm = vent_floor;
        }
        g_blower_ctrl_physics_ff_rpm_raw = s_physics_ff_slew_rpm;
        return;
    }

    float rpm_tgt = physicsFfClampRpmTarget(
        setpoint_cmh2o,
        physicsFfRpmForSetpointAndFlow(setpoint_cmh2o, q_now));

    /* Drop the flow LPF to measured total flow so expiratory vent leak is
     * not carried over from the inspiratory peak. */
    s_physics_ff_q_filt_slm = q_now;
    g_blower_ctrl_physics_ff_q_filt_slm = q_now;
    s_physics_ff_q_filt_valid = true;

    /* Floor 3: do not carry negative integral trim into expiration.
     *
     * Note the first branch is unreachable at stock settings: vent_floor is
     * ~17 krpm in the therapy range while |integ| is capped at
     * g_blower_ctrl_integ_max_abs_rpm (4 krpm default), so only the second
     * branch runs and the net effect is integ = max(integ, 0). It becomes
     * reachable if integ_max is raised above the vent floor, so both are
     * kept as-is. Gated for the bench A/B - see the block above. */
    if (g_blower_ctrl_exhale_integ_clamp_enable != 0U)
    {
        if (g_blower_ctrl_integ > vent_floor)
        {
            g_blower_ctrl_integ = vent_floor * 0.25f;
        }
        else if (g_blower_ctrl_integ < 0.0f)
        {
            g_blower_ctrl_integ = 0.0f;
        }
    }

    if (s_physics_ff_slew_rpm > rpm_tgt)
    {
        float new_slew = 0.5f * s_physics_ff_slew_rpm + 0.5f * rpm_tgt;
        if (new_slew < vent_floor)
        {
            new_slew = vent_floor;
        }
        s_physics_ff_slew_rpm = new_slew;
        g_blower_ctrl_physics_ff_rpm_raw = s_physics_ff_slew_rpm;
    }
    else if (s_physics_ff_slew_rpm < vent_floor)
    {
        s_physics_ff_slew_rpm = vent_floor;
        g_blower_ctrl_physics_ff_rpm_raw = vent_floor;
    }
}

static float physicsFfStep(float setpoint_cmh2o, float q_slm, float dt_s,
                           bool inhaling, bool exhaling, float meas_cmh2o)
{
    const float vent_floor = physicsFfVentFloorRpm(setpoint_cmh2o);
    const bool hybrid = (g_blower_ctrl_cpap_hybrid_active != 0U);
    /* Hybrid: hold plateau-style for the whole expiration, not only |Q|→0. */
    const bool exp_tail = hybrid ? false : physicsFfExpTailNearZero();
    float q_model = physicsFfFilterFlow(q_slm, dt_s, inhaling, exhaling);
    float rpm_target;

    float rise_step = clampf(g_blower_ctrl_physics_ff_rise_rpm_per_s,
                             100.0f, 20000.0f) * dt_s;
    if (inhaling && (meas_cmh2o < (setpoint_cmh2o - 0.04f)))
    {
        rise_step *= clampf(g_blower_ctrl_physics_ff_inhale_rise_boost, 1.0f, 4.0f);
    }

    /* --- Hybrid expiration: freeze RPM; only bleed if over-pressure ------ */
    if (hybrid && exhaling)
    {
        const float hold_band =
            clampf(g_blower_ctrl_hybrid_exp_hold_band_cmh2o, 0.02f, 1.0f);
        const float err = setpoint_cmh2o - meas_cmh2o; /* >0 → under-pressure */

        q_model = physicsFfVentFloorQSlm();
        rpm_target = physicsFfClampRpmTarget(
            setpoint_cmh2o,
            physicsFfRpmForSetpointAndFlow(setpoint_cmh2o, q_model));
        if (rpm_target < vent_floor)
        {
            rpm_target = vent_floor;
        }

        /* Floor: never sit below vent-leak RPM for the therapy setpoint. */
        if (s_physics_ff_slew_rpm < vent_floor)
        {
            s_physics_ff_slew_rpm = vent_floor;
        }

        if (err > 0.02f)
        {
            /* Below setpoint: raise FF (do not fall). Extra lift vs. sag. */
            float boost = err * clampf(g_blower_ctrl_hybrid_exp_boost_rpm_per_cmh2o,
                                       0.0f, 8000.0f);
            float lift_tgt = rpm_target + boost;
            if (lift_tgt < s_physics_ff_slew_rpm)
            {
                lift_tgt = s_physics_ff_slew_rpm;
            }
            const float boost_step =
                clampf(g_blower_ctrl_hybrid_exp_boost_rpm_per_s,
                       500.0f, 20000.0f) * dt_s;
            slewTowardFloat(&s_physics_ff_slew_rpm, lift_tgt, boost_step);
            g_blower_ctrl_physics_ff_rpm_raw = s_physics_ff_slew_rpm;
            return s_physics_ff_slew_rpm;
        }

        if (err < -hold_band)
        {
            /* Clearly above setpoint: very slow bleed toward vent floor. */
            const float fall_step =
                clampf(g_blower_ctrl_hybrid_exp_fall_rpm_per_s,
                       50.0f, 4000.0f) * dt_s;
            if (s_physics_ff_slew_rpm > rpm_target)
            {
                slewTowardFloat(&s_physics_ff_slew_rpm, rpm_target, fall_step);
            }
            if (s_physics_ff_slew_rpm < vent_floor)
            {
                s_physics_ff_slew_rpm = vent_floor;
            }
            g_blower_ctrl_physics_ff_rpm_raw = s_physics_ff_slew_rpm;
            return s_physics_ff_slew_rpm;
        }

        /* Inside hold band: freeze RPM (true plateau). */
        g_blower_ctrl_physics_ff_rpm_raw = s_physics_ff_slew_rpm;
        return s_physics_ff_slew_rpm;
    }

    if (exp_tail)
    {
        /* Patient flow → 0: model vent leak, not zero-flow hose drop. */
        q_model = physicsFfVentFloorQSlm();
        rpm_target = physicsFfClampRpmTarget(
            setpoint_cmh2o,
            physicsFfRpmForSetpointAndFlow(setpoint_cmh2o, q_model));
    }
    else
    {
        rpm_target = physicsFfClampRpmTarget(
            setpoint_cmh2o,
            physicsFfRpmForSetpointAndFlow(setpoint_cmh2o, q_model));
    }

    float onset_rpm = 0.0f;
    if (inhaling)
    {
        onset_rpm = physicsFfOnsetBoostRpm(q_slm, dt_s);
    }
    const float exp_lead_rpm = physicsFfExpLeadRpm(dt_s, exhaling);
    if (g_blower_ctrl_physics_ff_onset_bypass_slew == 0U)
    {
        rpm_target += onset_rpm;
        onset_rpm   = 0.0f;
    }

    /* Reported raw target includes the lead either way. */
    g_blower_ctrl_physics_ff_rpm_raw = rpm_target + onset_rpm;

    float fall_rate = clampf(g_blower_ctrl_physics_ff_fall_rpm_per_s,
                           100.0f, 20000.0f);
    if (exhaling && !exp_tail && (meas_cmh2o > (setpoint_cmh2o + 0.04f)))
    {
        fall_rate = clampf(g_blower_ctrl_physics_ff_fall_exhale_rpm_per_s,
                           fall_rate, 25000.0f);
    }
    const float fall_step = fall_rate * dt_s;

    /* Exp tail (|Q_pat| small): hold vent RPM — no coast-down through Q→0. */
    if (exp_tail)
    {
        if (rpm_target < vent_floor)
        {
            rpm_target = vent_floor;
        }
        if (s_physics_ff_slew_rpm < rpm_target)
        {
            slewTowardFloat(&s_physics_ff_slew_rpm, rpm_target, rise_step * 2.0f);
        }
        else
        {
            /* Come down to the target too, but never below it - see
             * g_blower_ctrl_physics_ff_exp_tail_fall_rpm_per_s. */
            const float tail_fall = clampf(g_blower_ctrl_physics_ff_exp_tail_fall_rpm_per_s,
                                           0.0f, 20000.0f) * dt_s;
            if (tail_fall > 0.0f)
            {
                slewTowardFloat(&s_physics_ff_slew_rpm, rpm_target, tail_fall);
            }
        }
        if (s_physics_ff_slew_rpm < vent_floor)
        {
            s_physics_ff_slew_rpm = vent_floor;
        }
        return s_physics_ff_slew_rpm;
    }

    /* Pressure below setpoint during expiration: recover RPM, don't coast down. */
    if (exhaling && (meas_cmh2o < (setpoint_cmh2o - 0.04f)))
    {
        if (rpm_target < vent_floor)
        {
            rpm_target = vent_floor;
        }
        if (s_physics_ff_slew_rpm < vent_floor)
        {
            slewTowardFloat(&s_physics_ff_slew_rpm, vent_floor, rise_step * 2.0f);
            return s_physics_ff_slew_rpm + exp_lead_rpm;
        }
    }

    if (rpm_target > s_physics_ff_slew_rpm)
    {
        slewTowardFloat(&s_physics_ff_slew_rpm, rpm_target, rise_step);
    }
    else
    {
        float tgt_fall = rpm_target;
        if (exhaling && (tgt_fall < vent_floor))
        {
            tgt_fall = vent_floor;
        }
        slewTowardFloat(&s_physics_ff_slew_rpm, tgt_fall, fall_step);
    }

    if (s_physics_ff_slew_rpm < vent_floor)
    {
        s_physics_ff_slew_rpm = vent_floor;
    }

    /* Lead rides on top of the slewed base without entering the slew state,
     * so it cannot accumulate or bias the flow-following target. */
    return s_physics_ff_slew_rpm + onset_rpm + exp_lead_rpm;
}

void BlowerCtrl_EnforceFfMutualExclusion(void)
{
    if (g_blower_ctrl_physics_ff_enable != 0U)
    {
        g_blower_ctrl_ff_enable            = 0U;
        g_blower_ctrl_flow_ff_enable       = 0U;
        g_blower_ctrl_flow_ff_onset_enable = 0U;
        return;
    }

    if ((g_blower_ctrl_ff_enable != 0U)
        || (g_blower_ctrl_flow_ff_enable != 0U))
    {
        g_blower_ctrl_physics_ff_enable = 0U;
    }
}

/* --- CPAP Plateau + Hybrid profiles -------------------------------------- */

volatile uint8_t g_blower_ctrl_cpap_plateau_prefer = 0U;
volatile uint8_t g_blower_ctrl_cpap_plateau_active = 0U;
/* g_blower_ctrl_cpap_hybrid_prefer/active declared with Physics FF globals. */

typedef struct
{
    uint8_t ff_enable;
    uint8_t flow_ff_enable;
    uint8_t flow_ff_onset_enable;
    uint8_t dip_ff_enable;
    uint8_t physics_ff_enable;
    uint8_t physics_ff_exp_tail_hold_enable;
    float   under_p_gain;
    float   over_p_gain;
    float   out_filter_alpha;
    float   meas_filter_alpha;
    float   deriv_filter_alpha;
    float   integ_deadband_cmh2o;
} CpapProfileSnapshot_t;

static CpapProfileSnapshot_t s_plateau_snap;
static uint8_t               s_plateau_snap_valid = 0U;
static CpapProfileSnapshot_t s_hybrid_snap;
static uint8_t               s_hybrid_snap_valid = 0U;

static void profileCaptureSnapshot(CpapProfileSnapshot_t *snap, uint8_t *valid)
{
    snap->ff_enable                     = g_blower_ctrl_ff_enable;
    snap->flow_ff_enable                = g_blower_ctrl_flow_ff_enable;
    snap->flow_ff_onset_enable          = g_blower_ctrl_flow_ff_onset_enable;
    snap->dip_ff_enable                 = g_blower_ctrl_dip_ff_enable;
    snap->physics_ff_enable             = g_blower_ctrl_physics_ff_enable;
    snap->physics_ff_exp_tail_hold_enable =
        g_blower_ctrl_physics_ff_exp_tail_hold_enable;
    snap->under_p_gain                  = g_blower_ctrl_under_p_gain;
    snap->over_p_gain                   = g_blower_ctrl_over_p_gain;
    snap->out_filter_alpha              = g_blower_ctrl_out_filter_alpha;
    snap->meas_filter_alpha             = g_blower_ctrl_meas_filter_alpha;
    snap->deriv_filter_alpha            = g_blower_ctrl_deriv_filter_alpha;
    snap->integ_deadband_cmh2o          = g_blower_ctrl_integ_deadband_cmh2o;
    *valid = 1U;
}

static void profileRestoreSnapshot(const CpapProfileSnapshot_t *snap, uint8_t valid)
{
    if (valid == 0U)
    {
        return;
    }
    g_blower_ctrl_ff_enable              = snap->ff_enable;
    g_blower_ctrl_flow_ff_enable         = snap->flow_ff_enable;
    g_blower_ctrl_flow_ff_onset_enable   = snap->flow_ff_onset_enable;
    g_blower_ctrl_dip_ff_enable          = snap->dip_ff_enable;
    g_blower_ctrl_physics_ff_enable      = snap->physics_ff_enable;
    g_blower_ctrl_physics_ff_exp_tail_hold_enable =
        snap->physics_ff_exp_tail_hold_enable;
    g_blower_ctrl_under_p_gain           = snap->under_p_gain;
    g_blower_ctrl_over_p_gain            = snap->over_p_gain;
    g_blower_ctrl_out_filter_alpha       = snap->out_filter_alpha;
    g_blower_ctrl_meas_filter_alpha      = snap->meas_filter_alpha;
    g_blower_ctrl_deriv_filter_alpha     = snap->deriv_filter_alpha;
    g_blower_ctrl_integ_deadband_cmh2o   = snap->integ_deadband_cmh2o;
    BlowerCtrl_EnforceFfMutualExclusion();
}

static void plateauApplyPreset(void)
{
    /* Hold mask pressure: sqrt(P) baseline + soft PI; do not chase flow. */
    g_blower_ctrl_ff_enable              = 1U;
    g_blower_ctrl_flow_ff_enable         = 0U;
    g_blower_ctrl_flow_ff_onset_enable   = 0U;
    g_blower_ctrl_dip_ff_enable          = 0U;
    g_blower_ctrl_physics_ff_enable      = 0U;
    g_blower_ctrl_under_p_gain           = 1.0f;
    g_blower_ctrl_over_p_gain            = 1.0f;
    g_blower_ctrl_out_filter_alpha       = 0.40f;
    g_blower_ctrl_meas_filter_alpha      = 0.70f;
    g_blower_ctrl_deriv_filter_alpha     = 0.20f;
    g_blower_ctrl_integ_deadband_cmh2o   = 1.5f;
    BlowerCtrl_EnforceFfMutualExclusion();
    g_blower_ctrl_cpap_plateau_active = 1U;
    g_blower_ctrl_cpap_hybrid_active  = 0U;
}

static void plateauRestoreSnapshot(void)
{
    profileRestoreSnapshot(&s_plateau_snap, s_plateau_snap_valid);
    g_blower_ctrl_cpap_plateau_active = 0U;
}

static void hybridApplyPreset(void)
{
    /* Insp: Physics FF. Exp: freeze RPM + boost if under-pressure. */
    g_blower_ctrl_ff_enable              = 0U;
    g_blower_ctrl_flow_ff_enable         = 0U;
    g_blower_ctrl_flow_ff_onset_enable   = 0U;
    g_blower_ctrl_dip_ff_enable          = 0U;
    g_blower_ctrl_physics_ff_enable      = 1U;
    g_blower_ctrl_physics_ff_exp_tail_hold_enable = 1U;
    g_blower_ctrl_under_p_gain           = 1.6f;
    g_blower_ctrl_over_p_gain            = 1.0f;
    g_blower_ctrl_out_filter_alpha       = 0.50f;
    g_blower_ctrl_meas_filter_alpha      = 0.55f;
    g_blower_ctrl_deriv_filter_alpha     = 0.20f;
    g_blower_ctrl_integ_deadband_cmh2o   = 1.5f;
    g_blower_ctrl_hybrid_exp_hold_band_cmh2o = 0.12f;
    g_blower_ctrl_hybrid_exp_fall_rpm_per_s  = 1500.0f;
    g_blower_ctrl_hybrid_exp_boost_rpm_per_cmh2o = 2500.0f;
    g_blower_ctrl_hybrid_exp_boost_rpm_per_s = 8000.0f;
    BlowerCtrl_EnforceFfMutualExclusion();
    g_blower_ctrl_cpap_hybrid_active  = 1U;
    g_blower_ctrl_cpap_plateau_active = 0U;
}

static void hybridRestoreSnapshot(void)
{
    profileRestoreSnapshot(&s_hybrid_snap, s_hybrid_snap_valid);
    g_blower_ctrl_cpap_hybrid_active = 0U;
}

static void clearPlateauPreferInternal(void)
{
    g_blower_ctrl_cpap_plateau_prefer = 0U;
    if (g_blower_ctrl_cpap_plateau_active != 0U)
    {
        plateauRestoreSnapshot();
    }
}

static void clearHybridPreferInternal(void)
{
    g_blower_ctrl_cpap_hybrid_prefer = 0U;
    if (g_blower_ctrl_cpap_hybrid_active != 0U)
    {
        hybridRestoreSnapshot();
    }
}

void BlowerCtrl_SetCpapPlateauPrefer(bool prefer)
{
    if (prefer)
    {
        clearHybridPreferInternal();
    }
    g_blower_ctrl_cpap_plateau_prefer = prefer ? 1U : 0U;
    if (!prefer && (g_blower_ctrl_cpap_plateau_active != 0U))
    {
        plateauRestoreSnapshot();
    }
}

void BlowerCtrl_SetCpapHybridPrefer(bool prefer)
{
    if (prefer)
    {
        clearPlateauPreferInternal();
    }
    g_blower_ctrl_cpap_hybrid_prefer = prefer ? 1U : 0U;
    if (!prefer && (g_blower_ctrl_cpap_hybrid_active != 0U))
    {
        hybridRestoreSnapshot();
    }
}

static void syncPlateauForMode(uint8_t therapy_mode)
{
    const bool cpap = (therapy_mode == (uint8_t)THERAPY_MODE_CPAP);

    if (cpap && (g_blower_ctrl_cpap_plateau_prefer != 0U)
        && (g_blower_ctrl_cpap_hybrid_prefer == 0U))
    {
        if (g_blower_ctrl_cpap_plateau_active == 0U)
        {
            profileCaptureSnapshot(&s_plateau_snap, &s_plateau_snap_valid);
            plateauApplyPreset();
        }
        else
        {
            plateauApplyPreset();
        }
        return;
    }

    if ((!cpap || (g_blower_ctrl_cpap_plateau_prefer == 0U))
        && (g_blower_ctrl_cpap_plateau_active != 0U))
    {
        plateauRestoreSnapshot();
    }
}

static void syncHybridForMode(uint8_t therapy_mode)
{
    const bool cpap = (therapy_mode == (uint8_t)THERAPY_MODE_CPAP);

    if (cpap && (g_blower_ctrl_cpap_hybrid_prefer != 0U))
    {
        if (g_blower_ctrl_cpap_hybrid_active == 0U)
        {
            profileCaptureSnapshot(&s_hybrid_snap, &s_hybrid_snap_valid);
            hybridApplyPreset();
        }
        else
        {
            hybridApplyPreset();
        }
        return;
    }

    if ((!cpap || (g_blower_ctrl_cpap_hybrid_prefer == 0U))
        && (g_blower_ctrl_cpap_hybrid_active != 0U))
    {
        hybridRestoreSnapshot();
    }
}

void BlowerCtrl_SyncCpapProfilesForMode(uint8_t therapy_mode)
{
    /* Plateau first (no-op if hybrid preferred), then hybrid. */
    syncPlateauForMode(therapy_mode);
    syncHybridForMode(therapy_mode);
}

void BlowerCtrl_SyncCpapPlateauForMode(uint8_t therapy_mode)
{
    BlowerCtrl_SyncCpapProfilesForMode(therapy_mode);
}

bool BlowerCtrl_IsCpapPlateauPreferred(void)
{
    return (g_blower_ctrl_cpap_plateau_prefer != 0U);
}

bool BlowerCtrl_IsCpapPlateauActive(void)
{
    return (g_blower_ctrl_cpap_plateau_active != 0U);
}

bool BlowerCtrl_IsCpapHybridPreferred(void)
{
    return (g_blower_ctrl_cpap_hybrid_prefer != 0U);
}

bool BlowerCtrl_IsCpapHybridActive(void)
{
    return (g_blower_ctrl_cpap_hybrid_active != 0U);
}

/* With FF on, |I| is a small trim (default cap 4 krpm). With FF off, I
 * must carry the full steady-state RPM (~17 krpm @ 8 cmH2O). */
static float effectiveIntegMaxAbsRpm(float user_sp_cmh2o)
{
    if (g_blower_ctrl_wm6850_mode != 0U)
    {
        return (float)g_blower_ctrl_rpm_max;
    }
    if (anyOpenLoopFfActive())
    {
        return g_blower_ctrl_integ_max_abs_rpm;
    }
    const float ss = steadyStateRpmForSetpoint(user_sp_cmh2o);
    const float ff_off_cap = ss * 1.25f;
    return (ff_off_cap > g_blower_ctrl_integ_max_abs_rpm)
             ? ff_off_cap : g_blower_ctrl_integ_max_abs_rpm;
}

static float wm6850SpeedCapRpm(float setpoint_cmh2o)
{
    /* WM6850 dev firmware used interp_occ_speed(P)*1.3 (6 krpm class blower).
     * Same motor on the custom CPAP board uses the sqrt bench map instead. */
    float sp = (setpoint_cmh2o > 0.0f) ? setpoint_cmh2o : 0.0f;
    return steadyStateRpmForSetpoint(sp) * g_blower_ctrl_wm6850_speed_lim_scale;
}

static bool wm6850ModeActive(void)
{
    return (g_blower_ctrl_wm6850_mode != 0U);
}

static bool rpmFloorActive(void)
{
    return (g_blower_ctrl_rpm_floor_enable != 0U)
        && anyOpenLoopFfActive()
        && !wm6850ModeActive()
        && !Epr_IsEnabled();
}

static float rpmFloorForSetpointCmh2o(float setpoint_cmh2o)
{
    const float ss = steadyStateRpmForSetpoint(setpoint_cmh2o);

    if (physicsFfActive())
    {
        const float vent = physicsFfVentFloorRpm(setpoint_cmh2o);
        return (vent > ss) ? vent : ss;
    }
    return ss;
}

/* Hysteretic gain-scheduling: pick the new zone given the slewed setpoint
 * and the previously-active zone. The hysteresis band is small (0.5
 * cmH2O); this only matters if a setpoint sits exactly on a boundary. */
static BlowerCtrlZone_t selectZone(float setpoint_cmh2o, BlowerCtrlZone_t prev)
{
    const float h = BLOWER_CTRL_ZONE_HYST_CMH2O;
    const float low_hi = BLOWER_CTRL_ZONE_LOW_HI_CMH2O;
    const float mid_hi = BLOWER_CTRL_ZONE_MID_HI_CMH2O;

    switch (prev)
    {
    case BLOWER_CTRL_ZONE_LOW:
        if (setpoint_cmh2o > (low_hi + h)) return BLOWER_CTRL_ZONE_MID;
        return BLOWER_CTRL_ZONE_LOW;
    case BLOWER_CTRL_ZONE_MID:
        if (setpoint_cmh2o < (low_hi - h)) return BLOWER_CTRL_ZONE_LOW;
        if (setpoint_cmh2o > (mid_hi + h)) return BLOWER_CTRL_ZONE_HIGH;
        return BLOWER_CTRL_ZONE_MID;
    case BLOWER_CTRL_ZONE_HIGH:
    default:
        if (setpoint_cmh2o < (mid_hi - h)) return BLOWER_CTRL_ZONE_MID;
        return BLOWER_CTRL_ZONE_HIGH;
    }
}

static void getActiveGains(BlowerCtrlZone_t z, float *kp, float *ki, float *kd)
{
    switch (z)
    {
    case BLOWER_CTRL_ZONE_LOW:
        *kp = g_blower_ctrl_kp_low;
        *ki = g_blower_ctrl_ki_low;
        *kd = g_blower_ctrl_kd_low;
        break;
    case BLOWER_CTRL_ZONE_HIGH:
        *kp = g_blower_ctrl_kp_high;
        *ki = g_blower_ctrl_ki_high;
        *kd = g_blower_ctrl_kd_high;
        break;
    case BLOWER_CTRL_ZONE_MID:
    default:
        *kp = g_blower_ctrl_kp_mid;
        *ki = g_blower_ctrl_ki_mid;
        *kd = g_blower_ctrl_kd_mid;
        break;
    }
}

/* === Public API === */

void BlowerCtrl_Init(void)
{
    g_blower_ctrl_setpoint_slew  = g_blower_ctrl_setpoint_user;
    g_blower_ctrl_meas_cmh2o     = 0.0f;
    g_blower_ctrl_sensor_cmh2o  = 0.0f;
    g_blower_ctrl_error_cmh2o    = 0.0f;
    g_blower_ctrl_integ          = 0.0f;
    g_blower_ctrl_deriv_rpm_per_s = 0.0f;
    g_blower_ctrl_p_term         = 0.0f;
    g_blower_ctrl_i_term         = 0.0f;
    g_blower_ctrl_d_term         = 0.0f;
    g_blower_ctrl_ff_term        = 0.0f;
    g_blower_ctrl_ff_flow_term   = 0.0f;
    g_blower_ctrl_ff_dip_term    = 0.0f;
    g_blower_ctrl_out_unfilt     = 0.0f;
    g_blower_ctrl_out_filt       = 0.0f;
    g_blower_ctrl_out_rpm        = 0;
    g_blower_ctrl_zone           = (uint8_t)BLOWER_CTRL_ZONE_LOW;
    g_blower_ctrl_saturated      = 0U;
    g_blower_ctrl_upper_limited  = 0U;
    g_blower_ctrl_step_count     = 0U;
    g_blower_ctrl_meas_filt_cmh2o = 0.0f;

    s_prev_meas_cmh2o = 0.0f;
    s_prev_meas_valid = false;
    s_integ_err_filt_cmh2o = 0.0f;
    s_integ_err_filt_valid = false;
    s_integ_err_filt_age_s = 0.0f;
    s_d_meas_filt_per_s = 0.0f;
    s_ff_flow_slew_rpm  = 0.0f;
    s_ff_dip_slew_rpm   = 0.0f;
    s_prev_q_pat_slm    = 0.0f;
    s_prev_q_pat_valid  = false;
    s_p_drop_filt_per_s = 0.0f;
    s_p_drop_filt_valid = false;
    s_prev_p_sensor_cmh2o = 0.0f;
    s_prev_p_sensor_valid = false;
    s_meas_filt_cmh2o  = 0.0f;
    s_meas_filt_valid  = false;
    s_p_err_filt_cmh2o = 0.0f;
    s_p_err_filt_valid = false;
    s_wm6850_skip_phase = 0U;
    s_physics_ff_was_active = false;
    s_physics_prev_inhaling = false;
    s_physics_phase_inhaling = false;
    s_epr_insp_pressure_prev = false;
    resetPhysicsFfState();
    BlowerCtrl_EnforceFfMutualExclusion();

    /* Co-init the FOT oscillator. Defaults to disabled so this is
     * invisible until the user explicitly turns it on. */
    BlowerOsc_Init();
}

void BlowerCtrl_SetEnable(bool en)
{
    if (en && !g_blower_ctrl_enabled)
    {
        const float user_sp = g_blower_ctrl_setpoint_user;
        const float meas    = g_blower_ctrl_meas_cmh2o;

        /* Therapy start is the anchor for the integrator-error LPF warm-up.
         * The tick bails out early while disabled, so without this the filter
         * keeps both its value and its age across a stop/start: the first run
         * after boot warms up correctly and every later one begins at the
         * full tau with a stale filtered error, which is the ~30 s ring-down
         * with I pinned at its clamp. Reseeds on the first enabled tick. */
        s_integ_err_filt_valid = false;
        s_integ_err_filt_age_s = 0.0f;
        resetOverPressurePhase();
        resetStartRamp();

        if (wm6850ModeActive())
        {
            /* PressurePID_Reset(): zero state, no sqrt seed. */
            g_blower_ctrl_setpoint_slew   = user_sp;
            g_blower_ctrl_integ           = 0.0f;
            g_blower_ctrl_out_filt        = 0.0f;
            g_blower_ctrl_out_rpm         = 0;
            s_ff_flow_slew_rpm            = 0.0f;
            s_ff_dip_slew_rpm             = 0.0f;
            s_prev_q_pat_valid            = false;
            s_d_meas_filt_per_s           = 0.0f;
            s_meas_filt_cmh2o             = meas;
            s_meas_filt_valid               = true;
            g_blower_ctrl_meas_filt_cmh2o   = meas;
            s_prev_meas_cmh2o               = meas;
            s_prev_meas_valid               = true;
            s_wm6850_skip_phase             = 0U;
        }
        else
        {
            /* Default to the requested setpoint (e.g. EPR-adjusted therapy
             * target). Only nudge slew to measured pressure when already
             * within bumpless band — never leave a stale slew from a prior
             * session when FF is enabled. */
            const float bumpless_band = 0.75f;
            g_blower_ctrl_setpoint_slew = user_sp;
            if (fabsf(meas - user_sp) <= bumpless_band)
            {
                g_blower_ctrl_setpoint_slew = meas;
            }
            g_blower_ctrl_integ         = 0.0f;
            s_ff_flow_slew_rpm          = 0.0f;
            s_ff_dip_slew_rpm           = 0.0f;
            s_prev_q_pat_valid          = false;
            s_d_meas_filt_per_s         = 0.0f;
            s_meas_filt_cmh2o           = meas;
            s_meas_filt_valid           = (meas > 0.01f);
            g_blower_ctrl_meas_filt_cmh2o = meas;
            s_prev_meas_valid           = false;

            const float ss_rpm = steadyStateRpmForSetpoint(g_blower_ctrl_setpoint_slew);
            if (physicsFfActive())
            {
                /* I is trim only; seed Physics FF to steady-state RPM. */
                s_physics_ff_slew_rpm = ss_rpm;
                g_blower_ctrl_physics_ff_rpm_raw = ss_rpm;
                s_physics_ff_was_active = true;
                g_blower_ctrl_integ = 0.0f;
                g_blower_ctrl_out_filt = ss_rpm;
            }
            else if (g_blower_ctrl_ff_enable != 0U)
            {
                g_blower_ctrl_out_filt = ss_rpm;
            }
            else
            {
                g_blower_ctrl_integ    = ss_rpm;
                g_blower_ctrl_out_filt = ss_rpm;
            }
            g_blower_ctrl_out_rpm = (int32_t)lrintf(g_blower_ctrl_out_filt);
        }
    }
    g_blower_ctrl_enabled = en ? 1U : 0U;
}

bool BlowerCtrl_IsEnabled(void)
{
    return (g_blower_ctrl_enabled != 0U);
}

void BlowerCtrl_ManualStart(int32_t rpm, uint16_t ramp_ms)
{
    BlowerCtrl_SetEnable(false);
    (void)BlowerIpc_CM7_Start();
    (void)BlowerIpc_CM7_SetSpeedRpm(rpm, ramp_ms);
    g_blower_manual_active = 1U;
}

void BlowerCtrl_ManualSetSpeed(int32_t rpm, uint16_t ramp_ms)
{
    if (g_blower_manual_active == 0U)
    {
        (void)BlowerIpc_CM7_Start();
        g_blower_manual_active = 1U;
    }
    (void)BlowerIpc_CM7_SetSpeedRpm(rpm, ramp_ms);
}

void BlowerCtrl_ManualStop(void)
{
    (void)BlowerIpc_CM7_Stop();
    g_blower_manual_active = 0U;
}

void BlowerCtrl_ClearManualFlag(void)
{
    /* Drop manual mode without STOP — used on mask-on resume so we do not
     * race a STOP mailbox write against the therapy SetSpeed that follows. */
    g_blower_manual_active = 0U;
}

bool BlowerCtrl_IsManualActive(void)
{
    return (g_blower_manual_active != 0U);
}

void BlowerCtrl_SnapSetpoint(float cmh2o)
{
    const float sp = clampf(cmh2o,
                            BLOWER_CTRL_SETPOINT_MIN_CMH2O,
                            BLOWER_CTRL_SETPOINT_MAX_CMH2O);
    g_blower_ctrl_setpoint_user = sp;
    g_blower_ctrl_setpoint_slew = sp;
}

void BlowerCtrl_SetUserSetpointOnly(float cmh2o)
{
    g_blower_ctrl_setpoint_user = clampf(cmh2o,
                                         BLOWER_CTRL_SETPOINT_MIN_CMH2O,
                                         BLOWER_CTRL_SETPOINT_MAX_CMH2O);
}

void BlowerCtrl_ReseedForTherapyResume(float target_cmh2o)
{
    const float sp = clampf(target_cmh2o,
                            BLOWER_CTRL_SETPOINT_MIN_CMH2O,
                            BLOWER_CTRL_SETPOINT_MAX_CMH2O);
    BlowerCtrl_SnapSetpoint(sp);

    const float ss_rpm = steadyStateRpmForSetpoint(sp);
    s_ff_flow_slew_rpm = 0.0f;
    s_ff_dip_slew_rpm = 0.0f;
    s_prev_q_pat_valid = false;
    s_d_meas_filt_per_s = 0.0f;
    s_prev_meas_valid = false;
    /* Reseed on the next tick: a stale filtered error from the previous
     * session would drive the integrator before it is meaningful. */
    s_integ_err_filt_valid = false;
    s_integ_err_filt_age_s = 0.0f;
    s_physics_prev_inhaling = false;
    s_physics_phase_inhaling = false;

    if (physicsFfActive())
    {
        s_physics_ff_q_filt_valid = false;
        s_physics_ff_slew_rpm = ss_rpm;
        g_blower_ctrl_physics_ff_rpm_raw = ss_rpm;
        s_physics_ff_was_active = true;
        g_blower_ctrl_integ = 0.0f;
        g_blower_ctrl_out_filt = ss_rpm;
    }
    else if (g_blower_ctrl_ff_enable != 0U)
    {
        g_blower_ctrl_integ = 0.0f;
        g_blower_ctrl_out_filt = ss_rpm;
    }
    else
    {
        g_blower_ctrl_integ = ss_rpm;
        g_blower_ctrl_out_filt = ss_rpm;
    }

    g_blower_ctrl_out_rpm = (int32_t)lrintf(g_blower_ctrl_out_filt);
    g_blower_manual_active = 0U;
}

void BlowerCtrl_BeginMaskResume(float target_cmh2o, float start_rpm)
{
    const float sp = clampf(target_cmh2o,
                            BLOWER_CTRL_SETPOINT_MIN_CMH2O,
                            BLOWER_CTRL_SETPOINT_MAX_CMH2O);
    g_blower_ctrl_setpoint_user = sp;

    /* Start slewed SP near measured pressure so sqrt-FF tracks a rising
     * trajectory instead of the full therapy target on the first tick. */
    float slew0 = g_blower_ctrl_meas_cmh2o;
    if (slew0 < 0.5f)
    {
        slew0 = 0.5f;
    }
    if (slew0 > sp)
    {
        slew0 = sp;
    }
    g_blower_ctrl_setpoint_slew = slew0;

    float rpm = start_rpm;
    if (rpm < 1000.0f)
    {
        rpm = 1000.0f;
    }
    if (rpm > (float)g_blower_ctrl_rpm_max)
    {
        rpm = (float)g_blower_ctrl_rpm_max;
    }

    s_ff_flow_slew_rpm = 0.0f;
    s_ff_dip_slew_rpm = 0.0f;
    s_prev_q_pat_valid = false;
    s_d_meas_filt_per_s = 0.0f;
    s_prev_meas_valid = false;
    /* Reseed on the next tick: a stale filtered error from the previous
     * session would drive the integrator before it is meaningful. */
    s_integ_err_filt_valid = false;
    s_integ_err_filt_age_s = 0.0f;
    s_physics_prev_inhaling = false;
    s_physics_phase_inhaling = false;

    if (physicsFfActive())
    {
        s_physics_ff_q_filt_valid = false;
        s_physics_ff_slew_rpm = rpm;
        g_blower_ctrl_physics_ff_rpm_raw = rpm;
        s_physics_ff_was_active = true;
        g_blower_ctrl_integ = 0.0f;
        g_blower_ctrl_out_filt = rpm;
    }
    else if (g_blower_ctrl_ff_enable != 0U)
    {
        /* I trim only; FF will follow slewed SP from ~meas upward. */
        g_blower_ctrl_integ = 0.0f;
        g_blower_ctrl_out_filt = rpm;
    }
    else
    {
        g_blower_ctrl_integ = rpm;
        g_blower_ctrl_out_filt = rpm;
    }

    g_blower_ctrl_out_rpm = (int32_t)lrintf(g_blower_ctrl_out_filt);
    g_blower_manual_active = 0U;
}

void BlowerCtrl_SuppressPatientFlowFf(uint32_t duration_ms)
{
    s_patient_ff_suppress_ms = duration_ms;
    s_ff_flow_slew_rpm = 0.0f;
    s_ff_dip_slew_rpm = 0.0f;
    s_prev_q_pat_valid = false;
    g_blower_ctrl_ff_flow_term = 0.0f;
    g_blower_ctrl_ff_dip_term = 0.0f;
}

void BlowerCtrl_SetResumeRpmFloor(float rpm, uint32_t duration_ms)
{
    if (rpm < 0.0f)
    {
        rpm = 0.0f;
    }
    s_resume_rpm_floor = rpm;
    s_resume_rpm_floor_ms = duration_ms;
}

void BlowerCtrl_NoteParkedRpm(float rpm)
{
    float r = rpm;
    if (r < 0.0f)
    {
        r = 0.0f;
    }
    if (r > (float)g_blower_ctrl_rpm_max)
    {
        r = (float)g_blower_ctrl_rpm_max;
    }
    g_blower_ctrl_out_filt = r;
    g_blower_ctrl_out_rpm  = (int32_t)lrintf(r);
}

float BlowerCtrl_EstimateSteadyStateRpm(float target_cmh2o)
{
    const float sp = clampf(target_cmh2o,
                            BLOWER_CTRL_SETPOINT_MIN_CMH2O,
                            BLOWER_CTRL_SETPOINT_MAX_CMH2O);
    return steadyStateRpmForSetpoint(sp);
}

void BlowerCtrl_SetSetpoint(float cmh2o)
{
    g_blower_ctrl_setpoint_user = clampf(cmh2o,
                                         BLOWER_CTRL_SETPOINT_MIN_CMH2O,
                                         BLOWER_CTRL_SETPOINT_MAX_CMH2O);
    /* While the PID is off, keep slew aligned so therapy start (EPR, ramp)
     * does not inherit the previous session's slewed target. */
    if ((g_blower_ctrl_enabled == 0U) || Epr_IsEnabled())
    {
        /* EPR steps the therapy target each breath; do not second-slew here
         * (6 cmH2O/s down made status/pressure linger at 6–7 cmH2O). */
        g_blower_ctrl_setpoint_slew = g_blower_ctrl_setpoint_user;
    }
}

float BlowerCtrl_GetSetpoint(void)        { return g_blower_ctrl_setpoint_user; }
float BlowerCtrl_GetSetpointSlewed(void)  { return g_blower_ctrl_setpoint_slew; }
float BlowerCtrl_GetMeasured(void)        { return g_blower_ctrl_meas_cmh2o; }
float BlowerCtrl_GetSensorCmh2o(void)     { return g_blower_ctrl_sensor_cmh2o; }
float BlowerCtrl_GetOutputRpm(void)       { return (float)g_blower_ctrl_out_rpm; }

bool BlowerCtrl_IsUpperLimited(void)
{
    return BlowerCtrl_IsEnabled() && (g_blower_ctrl_upper_limited != 0U);
}

void BlowerCtrl_Step(float dt_s)
{
    /* Refuse silly dt values - bad dt is the #1 cause of PID misbehaviour. */
    if (!(dt_s > 0.0f) || dt_s > 0.1f)
    {
        return;
    }

    /* --- 1. Pull the latest measurement and convert to cmH2O ---
     *
     *  We then push the sensor reading through the tube-loss compensator
     *  so the PID closes the loop on the *mask*-side pressure estimate
     *  rather than the device-side sensor reading. When TubeComp is
     *  disabled (boot default), this is a no-op pass-through and
     *  meas_cmh2o equals the raw sensor reading. */
    const float meas_mbar       = measPS.pressure;
    const float p_sensor_cmh2o  = MBAR_TO_CMH2O(meas_mbar);

    /* Only differentiate when a new conversion landed, and divide by the
     * interval that sample spans rather than by one tick. On a stale tick
     * the previous filtered rate is held (see g_ps_sample_fresh).
     *
     * Watchdog: if the freshness flag were ever to stop arriving, the
     * derivative history below would freeze. Nothing that feeds the PID is
     * allowed to stall indefinitely, so after BLOWER_CTRL_PS_STALE_MAX
     * consecutive stale ticks the tick is treated as fresh regardless.
     * At 200 Hz that is a 50 ms ceiling on how long the D path can coast. */
    bool ps_fresh = (g_ps_sample_fresh != 0U);
    if (ps_fresh)
    {
        s_ps_stale_ticks = 0U;
    }
    else if (++s_ps_stale_ticks >= BLOWER_CTRL_PS_STALE_MAX)
    {
        s_ps_stale_ticks = 0U;
        ps_fresh = true;
    }
    g_blower_ctrl_ps_stale_ticks = s_ps_stale_ticks;

    float ps_dt_s = (g_ps_sample_dt_s > 1.0e-4f) ? g_ps_sample_dt_s : dt_s;
    if (ps_dt_s > 0.1f)
    {
        ps_dt_s = 0.1f;   /* never let a stall inflate the divisor */
    }

    float p_drop_rate_cmh2o_per_s = s_p_drop_filt_valid ? s_p_drop_filt_per_s : 0.0f;
    if (ps_fresh && s_prev_p_sensor_valid)
    {
        const float p_drop_raw = (p_sensor_cmh2o - s_prev_p_sensor_cmh2o) / ps_dt_s;
        if (!s_p_drop_filt_valid)
        {
            s_p_drop_filt_per_s = p_drop_raw;
            s_p_drop_filt_valid = true;
        }
        else
        {
            s_p_drop_filt_per_s = 0.35f * p_drop_raw + 0.65f * s_p_drop_filt_per_s;
        }
        p_drop_rate_cmh2o_per_s = s_p_drop_filt_per_s;
    }
    const float q_slm           = Flow_GetSlmCtrl();   /* tube comp: fan-map flow when saturated */
    g_blower_ctrl_sensor_cmh2o  = p_sensor_cmh2o;
    const float meas_pid_cmh2o  = wm6850ModeActive()
                                    ? p_sensor_cmh2o
                                    : TubeComp_ApplyWithInertance(p_sensor_cmh2o, q_slm, dt_s);
    g_blower_ctrl_meas_cmh2o    = meas_pid_cmh2o;

    /* WM6850: pressure_pid.c runs at 100 Hz. SensorTask stays at 200 Hz so
     * therapy/safety still see fresh g_blower_ctrl_sensor_cmh2o every tick;
     * filter/PID/output run only on alternating ticks with dt = 10 ms. */
    if (wm6850ModeActive())
    {
        s_wm6850_skip_phase ^= 1U;
        if (s_wm6850_skip_phase != 0U)
        {
            return;
        }
        dt_s = BLOWER_CTRL_WM6850_DT_S;
    }

    if (!s_meas_filt_valid)
    {
        s_meas_filt_cmh2o = meas_pid_cmh2o;
        s_meas_filt_valid = true;
    }
    else
    {
        /* NEVER gate this on sample freshness. It is the PID's feedback
         * level: if it stops tracking, the loop runs open-loop and drives
         * the blower to the rail. Re-filtering a repeated sample is a
         * harmless no-op; skipping it is not. Only the DERIVATIVES below
         * care whether the sample is new. */
        float am = clampf(g_blower_ctrl_meas_filter_alpha, 0.0f, 1.0f);
        if (wm6850ModeActive())
        {
            am = clampf(g_blower_ctrl_wm6850_meas_alpha, 0.0f, 1.0f);
        }
        s_meas_filt_cmh2o = am * s_meas_filt_cmh2o + (1.0f - am) * meas_pid_cmh2o;
    }
    g_blower_ctrl_meas_filt_cmh2o = s_meas_filt_cmh2o;

    /* --- 2. Slew the user setpoint toward the target --- */
    {
        const float target = clampf(g_blower_ctrl_setpoint_user,
                                    BLOWER_CTRL_SETPOINT_MIN_CMH2O,
                                    BLOWER_CTRL_SETPOINT_MAX_CMH2O);
        if (wm6850ModeActive())
        {
            g_blower_ctrl_setpoint_slew = target;
        }
        else
        {
        float       sp       = g_blower_ctrl_setpoint_slew;
        const float diff     = target - sp;
        const float slew_rate = (diff > 0.0f)
                                  ? g_blower_ctrl_slew_up_cmh2o_per_s
                                  : g_blower_ctrl_slew_cmh2o_per_s;
        const float max_step = slew_rate * dt_s;

        if (diff >  max_step)      sp += max_step;
        else if (diff < -max_step) sp -= max_step;
        else                       sp  = target;

        g_blower_ctrl_setpoint_slew = sp;
        }
    }

    if (Epr_IsEnabled() && (g_blower_ctrl_epr_exhale_mit_enable != 0U))
    {
        const bool epr_insp_now = Epr_IsInspiratoryPressureActive();
        if (s_epr_insp_pressure_prev && !epr_insp_now)
        {
            eprExhaleOnsetTrim(g_blower_ctrl_setpoint_slew);
        }
        s_epr_insp_pressure_prev = epr_insp_now;
    }
    else
    {
        s_epr_insp_pressure_prev = false;
    }

    /* --- 3. If disabled, just track the measurement and bail out --- */
    if (g_blower_ctrl_enabled == 0U)
    {
        s_prev_meas_cmh2o = s_meas_filt_cmh2o;
        s_prev_meas_valid = true;
        s_p_err_filt_valid = false;      /* reseed on the next enabled tick */
        return;
    }

    /* --- 4. Pick the active gain set ---
     * Schedule on the *DC* (slewed) setpoint, not on the slew+osc value:
     * otherwise a 4 Hz oscillation that crosses a zone boundary (e.g.
     * setpoint=8 cmH2O with 1 cmH2O amplitude crosses LOW/MID at 8) would
     * flap the gain set 8x/s. */
    BlowerCtrlZone_t zone = selectZone(g_blower_ctrl_setpoint_slew,
                                       (BlowerCtrlZone_t)g_blower_ctrl_zone);
    g_blower_ctrl_zone = (uint8_t)zone;

    float kp, ki, kd;
    if (wm6850ModeActive())
    {
        kp = g_blower_ctrl_wm6850_kp;
        ki = g_blower_ctrl_wm6850_ki;
        kd = g_blower_ctrl_wm6850_kd;
        g_blower_ctrl_zone = (uint8_t)BLOWER_CTRL_ZONE_LOW;
    }
    else
    {
        getActiveGains(zone, &kp, &ki, &kd);
    }

    /* --- 5. PID terms --- */
    const float osc      = wm6850ModeActive() ? 0.0f : BlowerOsc_StepAndGetSample(dt_s);
    const float setpoint = g_blower_ctrl_setpoint_slew + osc;

    /* Blend toward unfiltered pressure only during inhale sags — avoids
     * measurement chatter on expiration / overshoot recovery. */
    float meas_cmh2o = s_meas_filt_cmh2o;
    if (!wm6850ModeActive() && (g_blower_ctrl_fast_meas_enable != 0U))
    {
        const float err_raw = setpoint - meas_pid_cmh2o;
        const float thresh  = clampf(g_blower_ctrl_fast_meas_thresh_cmh2o, 0.0f, 1.0f);
        const float full    = clampf(g_blower_ctrl_fast_meas_full_cmh2o,
                                     thresh + 0.05f, 2.0f);
        if (patientInhaling() && (err_raw > thresh))
        {
            const float blend = clampf((err_raw - thresh) / (full - thresh),
                                       0.0f, 1.0f);
            meas_cmh2o = (1.0f - blend) * s_meas_filt_cmh2o
                       + blend * meas_pid_cmh2o;
        }
    }

    const float error    = setpoint - meas_cmh2o;
    g_blower_ctrl_error_cmh2o = error;

    /* P. Extra gain when above setpoint to settle overshoot faster. The
     * over-pressure threshold is low while the patient breathes, high at rest
     * (overPressureThreshold); it also arms the output brake below. */
    const float over_p_thresh = overPressureThreshold(dt_s);

    /* P sees a low-passed error when p_err_lpf_hz > 0 - both its value and
     * which gain applies (see g_blower_ctrl_p_err_lpf_hz). */
    float p_error = error;
    if (!wm6850ModeActive() && (g_blower_ctrl_p_err_lpf_hz > 0.0f))
    {
        const float fc = clampf(g_blower_ctrl_p_err_lpf_hz, 1.0f, 50.0f);
        const float w  = BLOWER_CTRL_TWO_PI * fc * dt_s;
        if (!s_p_err_filt_valid)
        {
            s_p_err_filt_cmh2o = error;
            s_p_err_filt_valid = true;
        }
        else
        {
            s_p_err_filt_cmh2o += (w / (1.0f + w)) * (error - s_p_err_filt_cmh2o);
        }
        p_error = s_p_err_filt_cmh2o;
    }
    else
    {
        /* Track so switching the filter on mid-run starts from here. */
        s_p_err_filt_cmh2o = error;
        s_p_err_filt_valid = true;
    }
    g_blower_ctrl_p_err_filt_cmh2o = p_error;

    float p_term = kp * p_error;
    if (!wm6850ModeActive()
        && patientInhaling()
        && (p_error > g_blower_ctrl_under_p_thresh_cmh2o))
    {
        p_term *= g_blower_ctrl_under_p_gain;
    }
    else if (!wm6850ModeActive()
        && (p_error < -over_p_thresh))
    {
        p_term *= g_blower_ctrl_over_p_gain;
    }

    /* D-on-measurement (negated; positive d/dt of meas pushes output down).
     * Stepped on fresh conversions only - on a stale tick the last filtered
     * rate is reused rather than a spurious zero. */
    float d_meas_per_s = s_d_meas_filt_per_s;
    if (ps_fresh)
    {
        float d_raw = 0.0f;
        if (s_prev_meas_valid)
        {
            d_raw = (meas_cmh2o - s_prev_meas_cmh2o) / ps_dt_s;
        }
        float a = clampf(g_blower_ctrl_deriv_filter_alpha, 0.05f, 1.0f);
        if (wm6850ModeActive())
        {
            a = clampf(g_blower_ctrl_wm6850_deriv_alpha, 0.05f, 1.0f);
        }
        if (wm6850ModeActive())
        {
            /* Match pressure_pid.c: d_alpha weights the previous filtered D. */
            s_d_meas_filt_per_s = a * s_d_meas_filt_per_s + (1.0f - a) * d_raw;
        }
        else
        {
            s_d_meas_filt_per_s = a * d_raw + (1.0f - a) * s_d_meas_filt_per_s;
        }
        d_meas_per_s = s_d_meas_filt_per_s;
    }
    g_blower_ctrl_deriv_rpm_per_s = d_meas_per_s;
    float d_term = -kd * d_meas_per_s;

    /* Feed-forward: legacy (sqrt + flow + dip) OR physics-based combined FF.
     * The two paths are mutually exclusive (see BlowerCtrl_EnforceFfMutualExclusion). */
    float ff_term = 0.0f;
    float ff_flow = 0.0f;
    float ff_dip  = 0.0f;
    const bool epr_ff_ok = eprDisturbanceFfAllowed();

    bool suppress_pat_ff = false;
    if (s_patient_ff_suppress_ms > 0U)
    {
        const uint32_t step_ms = (uint32_t)lrintf(dt_s * 1000.0f);
        if (step_ms >= s_patient_ff_suppress_ms)
        {
            s_patient_ff_suppress_ms = 0U;
        }
        else
        {
            s_patient_ff_suppress_ms -= step_ms;
        }
        suppress_pat_ff = true;
        s_ff_flow_slew_rpm = 0.0f;
        s_ff_dip_slew_rpm = 0.0f;
    }

    if (!wm6850ModeActive() && physicsFfActive())
    {
        const bool inhaling = suppress_pat_ff ? false : patientInhaling();

        /* The onset snaps fire on a CONFIRMED phase change (see
         * g_blower_ctrl_physics_ff_onset_confirm_slm); everything else in
         * the FF keeps the raw flag and its timing. confirm 0 = snap on
         * every raw flip, the previous behaviour. */
        bool phase_inhaling = inhaling;
        const float confirm = clampf(g_blower_ctrl_physics_ff_onset_confirm_slm, 0.0f, 30.0f);
        if (confirm > 0.0f)
        {
            const float q_pat = suppress_pat_ff ? 0.0f : inhalePatientFlowSlm();
            const float q_in  = (g_blower_ctrl_flow_ff_inhale_sign >= 0) ? q_pat : -q_pat;
            if (q_in > confirm)
            {
                s_physics_phase_inhaling = true;
            }
            else if (q_in < -confirm)
            {
                s_physics_phase_inhaling = false;
            }
            phase_inhaling = s_physics_phase_inhaling;
        }
        else
        {
            s_physics_phase_inhaling = inhaling;
        }

        if ((g_blower_ctrl_physics_ff_exhale_mit_enable != 0U)
            && s_physics_prev_inhaling
            && !phase_inhaling)
        {
            physicsFfOnExhaleOnset(setpoint);
        }
        else if ((g_blower_ctrl_physics_ff_inhale_mit_enable != 0U)
                 && !s_physics_prev_inhaling
                 && phase_inhaling)
        {
            physicsFfOnInhaleOnset();
        }
        s_physics_prev_inhaling = phase_inhaling;

        if (epr_ff_ok && !suppress_pat_ff)
        {
            const float q_ff_in = physicsFfFlowInputSlm();
            ff_term = physicsFfStep(setpoint, q_ff_in, dt_s, inhaling, !inhaling,
                                    meas_pid_cmh2o);
        }
        else
        {
            /* During reconnect suppress: plateau at sqrt SS (no patient-Q boost). */
            ff_term = steadyStateRpmForSetpoint(setpoint);
            s_physics_ff_slew_rpm = ff_term;
            g_blower_ctrl_physics_ff_rpm_raw = ff_term;
        }
        s_physics_ff_was_active = true;
        s_ff_flow_slew_rpm = 0.0f;
        g_blower_ctrl_ff_flow_term = 0.0f;
    }
    else if (!wm6850ModeActive())
    {
        if (s_physics_ff_was_active)
        {
            resetPhysicsFfState();
            s_physics_ff_was_active = false;
            s_physics_prev_inhaling = false;
            s_physics_phase_inhaling = false;
        }
    /* Square-root steady-state RPM at setpoint. */
    if (g_blower_ctrl_ff_enable != 0U)
    {
        float ff_input = (setpoint > 0.0f) ? setpoint : 0.0f;
        ff_term = steadyStateRpmForSetpoint(ff_input);
    }

    /* Flow disturbance FF: proactive RPM boost on patient inhalation. */
    float ff_flow_target = 0.0f;
    const float q_pat = inhalePatientFlowSlm();
    float q_pat_rate_slm_per_s = 0.0f;
    if (s_prev_q_pat_valid)
    {
        q_pat_rate_slm_per_s = (q_pat - s_prev_q_pat_slm) / dt_s;
    }
    s_prev_q_pat_slm   = q_pat;
    s_prev_q_pat_valid = true;

    if (suppress_pat_ff)
    {
        ff_flow_target = 0.0f;
    }
    else if (g_blower_ctrl_flow_ff_enable != 0U && epr_ff_ok)
    {
        const float db = clampf(g_blower_ctrl_flow_ff_deadband_slm, 0.0f, 15.0f);
        if (g_blower_ctrl_flow_ff_inhale_sign >= 0)
        {
            if (q_pat > db)
            {
                ff_flow_target = g_blower_ctrl_flow_ff_k_rpm_per_slm * (q_pat - db);
            }
            if (g_blower_ctrl_flow_ff_onset_enable != 0U)
            {
                const float dq_thresh = clampf(
                    g_blower_ctrl_flow_ff_onset_thresh_slm_per_s, 10.0f, 500.0f);
                if (q_pat_rate_slm_per_s > dq_thresh)
                {
                    float ff_onset = g_blower_ctrl_flow_ff_onset_k_rpm_per_slm_per_s
                                   * (q_pat_rate_slm_per_s - dq_thresh);
                    const float onset_max = clampf(
                        g_blower_ctrl_flow_ff_onset_max_rpm, 0.0f, 4000.0f);
                    if (ff_onset > onset_max)
                    {
                        ff_onset = onset_max;
                    }
                    ff_flow_target += ff_onset;
                }
            }
        }
        else if (q_pat < -db)
        {
            ff_flow_target = g_blower_ctrl_flow_ff_k_rpm_per_slm * (-q_pat - db);
            if (g_blower_ctrl_flow_ff_onset_enable != 0U)
            {
                const float dq_thresh = clampf(
                    g_blower_ctrl_flow_ff_onset_thresh_slm_per_s, 10.0f, 500.0f);
                if (q_pat_rate_slm_per_s < -dq_thresh)
                {
                    float ff_onset = g_blower_ctrl_flow_ff_onset_k_rpm_per_slm_per_s
                                   * (-q_pat_rate_slm_per_s - dq_thresh);
                    const float onset_max = clampf(
                        g_blower_ctrl_flow_ff_onset_max_rpm, 0.0f, 4000.0f);
                    if (ff_onset > onset_max)
                    {
                        ff_onset = onset_max;
                    }
                    ff_flow_target += ff_onset;
                }
            }
        }
        if (ff_flow_target > 0.0f)
        {
            const float ff_max = clampf(g_blower_ctrl_flow_ff_max_rpm, 0.0f, 8000.0f);
            if (ff_flow_target > ff_max)
            {
                ff_flow_target = ff_max;
            }
        }
    }
    else if (!epr_ff_ok)
    {
        ff_flow_target = 0.0f;
        s_ff_flow_slew_rpm = 0.0f;
    }
    {
        const float rise_step = clampf(g_blower_ctrl_flow_ff_rise_rpm_per_s,
                                       100.0f, 20000.0f) * dt_s;
        const float fall_step = clampf(g_blower_ctrl_flow_ff_fall_rpm_per_s,
                                       100.0f, 20000.0f) * dt_s;
        if (ff_flow_target > s_ff_flow_slew_rpm)
        {
            slewTowardFloat(&s_ff_flow_slew_rpm, ff_flow_target, rise_step);
        }
        else
        {
            slewTowardFloat(&s_ff_flow_slew_rpm, ff_flow_target, fall_step);
        }
    }
    ff_flow = s_ff_flow_slew_rpm;
    g_blower_ctrl_ff_flow_term = ff_flow;
    } /* legacy FF path */

    if (!wm6850ModeActive())
    {
        float ff_dip_target = 0.0f;
        if (!suppress_pat_ff
            && (g_blower_ctrl_dip_ff_enable != 0U) && epr_ff_ok)
        {
            const float press_sag = setpoint - meas_pid_cmh2o;
            const float err_min   = clampf(g_blower_ctrl_dip_ff_err_min_cmh2o, 0.0f, 1.0f);
            const bool  dip_phase = patientInhaling() || physicsFfExpTailNearZero();
            if (dip_phase && (press_sag > err_min))
            {
                const float thresh = clampf(g_blower_ctrl_dip_ff_thresh_cmh2o_per_s,
                                            1.0f, 80.0f);
                if (p_drop_rate_cmh2o_per_s < -thresh)
                {
                    ff_dip_target = g_blower_ctrl_dip_ff_k_rpm_per_cmh2o_per_s
                                  * (-p_drop_rate_cmh2o_per_s - thresh);
                    const float dip_max = clampf(g_blower_ctrl_dip_ff_max_rpm,
                                                 0.0f, 8000.0f);
                    if (ff_dip_target > dip_max)
                    {
                        ff_dip_target = dip_max;
                    }
                }
            }
        }
        else if (!epr_ff_ok)
        {
            ff_dip_target = 0.0f;
            s_ff_dip_slew_rpm = 0.0f;
        }
        {
            const float rise_step = clampf(g_blower_ctrl_flow_ff_rise_rpm_per_s,
                                           100.0f, 20000.0f) * dt_s;
            const float fall_step = clampf(g_blower_ctrl_flow_ff_fall_rpm_per_s,
                                           100.0f, 20000.0f) * dt_s;
            if (ff_dip_target > s_ff_dip_slew_rpm)
            {
                slewTowardFloat(&s_ff_dip_slew_rpm, ff_dip_target, rise_step);
            }
            else
            {
                slewTowardFloat(&s_ff_dip_slew_rpm, ff_dip_target, fall_step);
            }
        }
        ff_dip = s_ff_dip_slew_rpm;
        g_blower_ctrl_ff_dip_term = ff_dip;
    }

    /* Expiratory rise FF: negative RPM while pressure is over setpoint and
     * still climbing. See the tunables block for the measurement behind it. */
    float ff_rise = 0.0f;
    if (!wm6850ModeActive())
    {
        float ff_rise_target = 0.0f;
        if ((g_blower_ctrl_rise_ff_enable != 0U) && !suppress_pat_ff && epr_ff_ok)
        {
            const float press_over = meas_pid_cmh2o - setpoint;
            const float err_min = clampf(g_blower_ctrl_rise_ff_err_min_cmh2o,
                                         0.0f, 1.0f);
            if (press_over > err_min)
            {
                const float thresh = clampf(g_blower_ctrl_rise_ff_thresh_cmh2o_per_s,
                                            0.5f, 80.0f);
                if (p_drop_rate_cmh2o_per_s > thresh)
                {
                    const float rise_max = clampf(g_blower_ctrl_rise_ff_max_rpm,
                                                  0.0f, 8000.0f);
                    ff_rise_target = -g_blower_ctrl_rise_ff_k_rpm_per_cmh2o_per_s
                                   * (p_drop_rate_cmh2o_per_s - thresh);
                    if (ff_rise_target < -rise_max)
                    {
                        ff_rise_target = -rise_max;
                    }
                }
            }
        }
        else
        {
            s_ff_rise_slew_rpm = 0.0f;
        }
        {
            const float rise_step = clampf(g_blower_ctrl_flow_ff_rise_rpm_per_s,
                                           100.0f, 20000.0f) * dt_s;
            const float fall_step = clampf(g_blower_ctrl_flow_ff_fall_rpm_per_s,
                                           100.0f, 20000.0f) * dt_s;
            /* Target is negative: approaching it is the "rise" direction here. */
            if (ff_rise_target < s_ff_rise_slew_rpm)
            {
                slewTowardFloat(&s_ff_rise_slew_rpm, ff_rise_target, rise_step);
            }
            else
            {
                slewTowardFloat(&s_ff_rise_slew_rpm, ff_rise_target, fall_step);
            }
        }
        ff_rise = s_ff_rise_slew_rpm;
        g_blower_ctrl_ff_rise_term = ff_rise;
    }

    /* Expiratory flow FF, phase-lead compensated. See the tunables block. */
    float ff_exp_flow = 0.0f;
    if (!wm6850ModeActive())
    {
        const float q_pat_raw = Flow_GetPatientSlm();
        float dq = 0.0f;
        if (s_exp_q_prev_valid)
        {
            dq = (q_pat_raw - s_exp_q_prev_slm) / dt_s;
        }
        s_exp_q_prev_slm = q_pat_raw;
        s_exp_q_prev_valid = true;
        {
            const float fc = clampf(g_blower_ctrl_exp_flow_ff_dq_lpf_hz, 0.5f, 25.0f);
            const float w = BLOWER_CTRL_TWO_PI * fc * dt_s;
            const float a = w / (1.0f + w);
            s_exp_dq_filt_slm_per_s = (a * dq)
                                    + ((1.0f - a) * s_exp_dq_filt_slm_per_s);
        }

        float ff_exp_target = 0.0f;
        if ((g_blower_ctrl_exp_flow_ff_enable != 0U) && !suppress_pat_ff && epr_ff_ok)
        {
            const float lead = clampf(g_blower_ctrl_exp_flow_ff_lead_s, 0.0f, 0.5f);
            const float q_lead = q_pat_raw + (lead * s_exp_dq_filt_slm_per_s);
            if (q_lead < 0.0f)
            {
                const float cap = clampf(g_blower_ctrl_exp_flow_ff_max_rpm,
                                         0.0f, 4000.0f);
                ff_exp_target = g_blower_ctrl_exp_flow_ff_k_rpm_per_slm * q_lead;
                if (ff_exp_target < -cap)
                {
                    ff_exp_target = -cap;
                }
            }
        }
        else
        {
            s_ff_exp_flow_slew_rpm = 0.0f;
        }
        {
            const float rise_step = clampf(g_blower_ctrl_flow_ff_rise_rpm_per_s,
                                           100.0f, 20000.0f) * dt_s;
            const float fall_step = clampf(g_blower_ctrl_flow_ff_fall_rpm_per_s,
                                           100.0f, 20000.0f) * dt_s;
            if (ff_exp_target < s_ff_exp_flow_slew_rpm)
            {
                slewTowardFloat(&s_ff_exp_flow_slew_rpm, ff_exp_target, rise_step);
            }
            else
            {
                slewTowardFloat(&s_ff_exp_flow_slew_rpm, ff_exp_target, fall_step);
            }
        }
        ff_exp_flow = s_ff_exp_flow_slew_rpm;
        g_blower_ctrl_ff_exp_flow_term = ff_exp_flow;
    }

    const float ff_dist = ff_flow + ff_dip + ff_rise + ff_exp_flow;

    const float rpm_min_f = (float)g_blower_ctrl_rpm_min;
    const float rpm_max_f = (float)g_blower_ctrl_rpm_max;
    /* Post-hand-over ramp lowers the ceiling for the first seconds of RUN.
     * Using the ceiling (not a post-clamp) keeps anti-windup consistent; it
     * also holds upper_limited, which only blocks fast mask-off arming. */
    float active_rpm_max_f = startRampCeilingRpm(dt_s, rpm_max_f);
    if (active_rpm_max_f < rpm_min_f)
    {
        active_rpm_max_f = rpm_min_f;
    }
    if (wm6850ModeActive())
    {
        const float speed_cap = wm6850SpeedCapRpm(setpoint);
        if (speed_cap < active_rpm_max_f)
        {
            active_rpm_max_f = speed_cap;
        }
        if (active_rpm_max_f < rpm_min_f)
        {
            active_rpm_max_f = rpm_min_f;
        }
    }

    /* I-term: WM6850-style bidirectional conditional integration.
     * When pressure > setpoint (err < 0), I ramps down and pulls RPM
     * lower quickly — key to fast expiratory return. Freeze I only while
     * the internal setpoint is still slewing up at therapy start. */
    const float sp_gap = g_blower_ctrl_setpoint_user - g_blower_ctrl_setpoint_slew;
    const bool  sp_catching_up = (!wm6850ModeActive())
        && (sp_gap > g_blower_ctrl_startup_slew_gap_cmh2o);

    /* Integrator deadband. Documented in the header and exposed to the PC
     * tool as "integ_deadband", but previously never read here: |error|
     * beyond the band freezes I, so the integrator only trims near the
     * target instead of winding up through a large transient. A value of 0
     * or below disables the check, as does a very large one (the header
     * suggests 1e6) - both give the previous always-integrate behaviour.
     * Not applied in WM6850 mode, which reproduces pressure_pid.c. */
    const float integ_db = g_blower_ctrl_integ_deadband_cmh2o;
    const bool  integ_in_band = (g_blower_ctrl_integ_deadband_enable == 0U)
                             || wm6850ModeActive()
                             || !(integ_db > 0.0f)
                             || (fabsf(error) <= integ_db);

    /* Error seen by the integrator only; P and D keep the raw signal. */
    if (!s_integ_err_filt_valid)
    {
        s_integ_err_filt_cmh2o = error;
        s_integ_err_filt_age_s = 0.0f;
        s_integ_err_filt_valid = true;
        g_blower_ctrl_integ_err_filt_cmh2o = s_integ_err_filt_cmh2o;
        g_blower_ctrl_integ_err_lpf_tau_now_s =
            clampf(g_blower_ctrl_integ_err_lpf_tau_start_s, 0.05f, 120.0f);
    }
    else
    {
        const float tau_full  = clampf(g_blower_ctrl_integ_err_lpf_tau_s, 0.5f, 120.0f);
        const float tau_start = clampf(g_blower_ctrl_integ_err_lpf_tau_start_s,
                                       0.05f, tau_full);
        const float warmup    = clampf(g_blower_ctrl_integ_err_lpf_warmup_s, 0.0f, 600.0f);

        const float band = g_blower_ctrl_integ_err_lpf_settle_band_cmh2o;
        const bool  settled = !(band > 0.0f)
                           || (fabsf(s_integ_err_filt_cmh2o) <= band);

        s_integ_err_filt_age_s = clampf(
            s_integ_err_filt_age_s + (settled ? dt_s : -dt_s), 0.0f, warmup);

        float tau = tau_full;
        if (warmup > 0.0f)
        {
            const float frac = clampf(s_integ_err_filt_age_s / warmup, 0.0f, 1.0f);
            tau = tau_start + ((tau_full - tau_start) * frac);
        }

        const float a = dt_s / (tau + dt_s);
        s_integ_err_filt_cmh2o = (a * error)
                               + ((1.0f - a) * s_integ_err_filt_cmh2o);

        g_blower_ctrl_integ_err_filt_cmh2o = s_integ_err_filt_cmh2o;
        g_blower_ctrl_integ_err_lpf_tau_now_s = tau;
    }
    const float integ_err = (g_blower_ctrl_integ_err_lpf_enable != 0U)
                              ? s_integ_err_filt_cmh2o
                              : error;

    float i_step = 0.0f;
    if (g_blower_ctrl_pid_enable != 0U)
    {
        if (!sp_catching_up && integ_in_band
            && (g_blower_ctrl_integ_hold == 0U))
        {
            const float trim_try = p_term + g_blower_ctrl_integ + d_term;
            const float out_try  = ff_term + ff_dist + trim_try;
            const bool  sat_hi   = (out_try > active_rpm_max_f);
            const bool  sat_lo   = (out_try < rpm_min_f);

            if ((!sat_hi && !sat_lo)
                || (sat_hi && (error < 0.0f))
                || (sat_lo && (error > 0.0f)))
            {
                i_step = ki * integ_err * dt_s;
            }
        }
    }

    float integ_new = g_blower_ctrl_integ + i_step;

    /* Physics FF: negative I after expiratory overshoot deepens undershoot.
     * Off by default - it was measured causing a second expiratory hump.
     * See g_blower_ctrl_exp_neg_i_clamp_enable. */
    if ((g_blower_ctrl_exp_neg_i_clamp_enable != 0U)
        && physicsFfActive()
        && !patientInhaling()
        && (error > 0.04f)
        && (integ_new < 0.0f))
    {
        integ_new = 0.0f;
    }

    /* Compute provisional output with the candidate integrator. */
    float out_unfilt = ff_term + ff_dist + p_term + integ_new + d_term;

    /* Overpressure brake (0 = off; on at 6000 rpm/s by default). Shares the
     * phase-dependent threshold with the over-pressure P boost. */
    if (g_blower_ctrl_pid_enable != 0U
        && !wm6850ModeActive()
        && (g_blower_ctrl_over_brake_rpm_per_s > 0.0f)
        && (error < -over_p_thresh))
    {
        const float brake = clampf(g_blower_ctrl_over_brake_rpm_per_s, 500.0f, 15000.0f) * dt_s;
        const float floor_out = g_blower_ctrl_out_filt - brake;
        if (out_unfilt > floor_out)
        {
            out_unfilt = floor_out;
        }
    }

    /* Anti-windup layer 2: reject integrator step if it deepens saturation. */
    bool        sat_high  = (out_unfilt > active_rpm_max_f) && (i_step > 0.0f);
    bool        sat_low   = (out_unfilt < rpm_min_f) && (i_step < 0.0f);
    if (sat_high || sat_low)
    {
        integ_new = g_blower_ctrl_integ;
        out_unfilt = ff_term + ff_dist + p_term + integ_new + d_term;
    }
    else if (g_blower_ctrl_pid_enable != 0U && !sp_catching_up)
    {
        /* WM6850-style integrator clamp to remaining RPM headroom. */
        float trim_min = rpm_min_f - ff_term - ff_dist - p_term - d_term;
        float trim_max = active_rpm_max_f - ff_term - ff_dist - p_term - d_term;
        if (g_blower_ctrl_integ_headroom_hold_enable != 0U)
        {
            /* Stop I growing into saturation, never drag it past where it
             * already was. See g_blower_ctrl_integ_headroom_hold_enable. */
            const float i_prev = g_blower_ctrl_integ;
            if (trim_max < i_prev) { trim_max = i_prev; }
            if (trim_min > i_prev) { trim_min = i_prev; }
        }
        if (integ_new < trim_min) { integ_new = trim_min; }
        if (integ_new > trim_max) { integ_new = trim_max; }
        out_unfilt = ff_term + ff_dist + p_term + integ_new + d_term;
    }

    /* Anti-windup layer 3: hard magnitude clamp on the integrator. */
    if (g_blower_ctrl_pid_enable != 0U)
    {
        const float integ_max = effectiveIntegMaxAbsRpm(g_blower_ctrl_setpoint_user);
        if (integ_new > integ_max)
        {
            integ_new = integ_max;
            out_unfilt = ff_term + ff_dist + p_term + integ_new + d_term;
        }
        else if (integ_new < -integ_max)
        {
            integ_new = -integ_max;
            out_unfilt = ff_term + ff_dist + p_term + integ_new + d_term;
        }
    }

    if (g_blower_ctrl_pid_enable == 0U)
    {
        out_unfilt = ff_term + ff_dist;
        integ_new  = g_blower_ctrl_integ;
        p_term     = 0.0f;
        d_term     = 0.0f;
    }

    const float rpm_floor = rpmFloorActive()
                              ? rpmFloorForSetpointCmh2o(setpoint)
                              : 0.0f;
    if (rpm_floor > 0.0f && out_unfilt < rpm_floor)
    {
        out_unfilt = rpm_floor;
    }

    /* Commit and clamp. */
    g_blower_ctrl_integ      = integ_new;
    g_blower_ctrl_p_term     = p_term;
    g_blower_ctrl_i_term     = integ_new;
    g_blower_ctrl_d_term     = d_term;
    g_blower_ctrl_ff_term    = ff_term;
    g_blower_ctrl_out_unfilt = out_unfilt;

    const float out_clamped = clampf(out_unfilt, rpm_min_f, active_rpm_max_f);
    g_blower_ctrl_saturated  = (out_clamped != out_unfilt) ? 1U : 0U;

    float out_post_sat = out_clamped;

    /* --- 6. Output low-pass filter --- */
    float       a = g_blower_ctrl_out_filter_alpha;
    if (a < 0.05f) a = 0.05f;
    if (a > 1.0f)  a = 1.0f;
    g_blower_ctrl_out_filt = a * out_post_sat
                           + (1.0f - a) * g_blower_ctrl_out_filt;
    if (rpm_floor > 0.0f && g_blower_ctrl_out_filt < rpm_floor)
    {
        g_blower_ctrl_out_filt = rpm_floor;
    }
    const float out_filt_clamped = clampf(g_blower_ctrl_out_filt,
                                          rpm_min_f,
                                          active_rpm_max_f);
    if (out_filt_clamped != g_blower_ctrl_out_filt)
    {
        g_blower_ctrl_out_filt = out_filt_clamped;
        g_blower_ctrl_saturated = 1U;
    }

    /* --- 7. Quantise and post --- */
    int32_t out_rpm = (int32_t)lrintf(g_blower_ctrl_out_filt);
    out_rpm         = clampi(out_rpm,
                             g_blower_ctrl_rpm_min,
                             g_blower_ctrl_rpm_max);

    /* Do not confuse low-RPM clamping with exhausted inspiratory headroom.
     * Include the unclipped demand: output filtering may still be catching up.
     * 50 RPM covers quantisation / asymptotic approach to the active ceiling. */
    g_blower_ctrl_upper_limited =
        ((out_unfilt >= active_rpm_max_f)
         || ((float)out_rpm >= (active_rpm_max_f - 50.0f))) ? 1U : 0U;

    if (s_resume_rpm_floor_ms > 0U)
    {
        const uint32_t step_ms = (uint32_t)lrintf(dt_s * 1000.0f);
        if (step_ms >= s_resume_rpm_floor_ms)
        {
            s_resume_rpm_floor_ms = 0U;
            s_resume_rpm_floor = 0.0f;
        }
        else
        {
            s_resume_rpm_floor_ms -= step_ms;
        }
        /* Never above this tick's ceiling: during a restart the post-hand-
         * over ramp is holding it, and the floor must not undo that. */
        const float floor_f = (s_resume_rpm_floor < active_rpm_max_f)
                                ? s_resume_rpm_floor : active_rpm_max_f;
        const int32_t floor_i = (int32_t)lrintf(floor_f);
        if ((s_resume_rpm_floor > 0.0f) && (out_rpm < floor_i))
        {
            out_rpm = floor_i;
            g_blower_ctrl_out_filt = (float)out_rpm;
        }
    }

    g_blower_ctrl_out_rpm = out_rpm;

    (void)BlowerIpc_CM7_SetSpeedRpmSilent(out_rpm);

    /* --- 8. Bookkeeping ---
     * Only advance the derivative history on ticks that carried a new
     * conversion, so the next difference spans a real sampling interval. */
    if (ps_fresh)
    {
        s_prev_meas_cmh2o     = meas_cmh2o;
        s_prev_meas_valid     = true;
        s_prev_p_sensor_cmh2o = p_sensor_cmh2o;
        s_prev_p_sensor_valid = true;
    }
    g_blower_ctrl_step_count++;
}
