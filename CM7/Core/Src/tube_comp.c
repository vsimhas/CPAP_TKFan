/**
  ******************************************************************************
  * @file    tube_comp.c
  * @brief   Implementation: dP_hose = R_lam*|Q| + R_turb*Q*|Q|.
  ******************************************************************************
  */

#include "tube_comp.h"

#include <math.h>

/* === Tunables (debugger-writable volatiles) ================================ */
volatile uint8_t g_tubecomp_enabled       = 1U;
/* 2026-09-22: fitted against an external pressure sensor at the mask end.
 * Was R_lam 0.002 / R_turb 0.00022 (a 15 mm slimline book value, then bench
 * nudged). Those over-estimate the drop badly once flow rises:
 *
 *      flow      fitted   previous
 *      15 LPM     0.080     0.080
 *      25         0.162     0.188
 *      35         0.268     0.340
 *      45         0.398     0.536
 *      55         0.552     0.775
 *
 * They agree at 15 LPM and diverge with flow, so the old pair was too
 * quadratic: at peak inspiratory flow it subtracted 0.22 cmH2O too much, and
 * since P_mask is the PID's feedback that is a breath-synchronous error
 * injected straight into the loop. Fit residual 0.011 cmH2O against 0.097 for
 * the old pair, which also carried a -0.070 bias.
 *
 * HOW THIS WAS MEASURED, because the obvious experiment cannot work. In a
 * vent-dominated circuit the vent flow goes as sqrt(P), so Q^2 is exactly
 * proportional to P: "drop proportional to Q^2" and "drop proportional to
 * pressure" are the same curve, and no sweep driven by vent flow can tell a
 * hose drop from a sensor scale difference. Two independent fits collapsed on
 * that (basis correlation 0.9965, one returning a negative R_turb).
 *
 * It takes three measurements:
 *   1. Static sweep 6-18 cmH2O, circuit occluded downstream of the vent so
 *      the external sensor sits in a dead-end branch and reads mask pressure.
 *      Pressure varies, flow follows it.
 *   2. Breathing at a regulated 15 cmH2O. Pressure fixed, flow swings 30-54
 *      LPM. This is what decorrelates flow from pressure.
 *   3. Vent AND outlet blocked, zero flow everywhere. A hose cannot drop
 *      pressure with nothing moving through it, so the residual difference is
 *      sensor mismatch: measured +0.237 cmH2O at 15.01, i.e. 1.58% of reading,
 *      over 6000 samples.
 * Subtract that 1.58% from (1) and (2), and the flow range widens to 16-54 LPM
 * with the basis correlation down to 0.9894 - enough to fit both terms.
 *
 * OPEN: the 1.58% is a DIFFERENCE between two sensors. If it is ours reading
 * high, patients get 1.6% less than the display claims (0.3 cmH2O at 20) and
 * that matters more than this model. A reference manometer is the only way to
 * tell which unit is honest. These coefficients are unaffected either way -
 * the mismatch is common-mode and was removed before fitting.
 *
 * Re-fit per hose/mask: pc_tools has no tool for this yet; the three captures
 * above were driven by scratch scripts against PlotFlowP.py's packet format.
 *
 * 2026-09-25: these never took effect after a boot. TubeComp_Init() - called
 * from main() - reset both to the old 0.002 / 0.00022, so the fitted values
 * only applied in sessions that set them over USB ('tubecomp set R_lam ...'),
 * and the D-line hose term shows every capture of 2026-09-24 ran on the old
 * pair. One definition now feeds both the initialisers and TubeComp_Init().
 *
 * What moves: P_mask (the PID's feedback) reads higher at high inspiratory
 * flow - +0.2 cmH2O at 55 LPM, +0.67 at the 89 slm clip - so the loop
 * pushes that much less at the sensor, which is the point: the old pair made
 * the true mask pressure overshoot setpoint by that amount on every breath.
 * The physics FF does NOT move: its hose term was fitted jointly with K,
 * offset and c_fan on the old pair and now has its own frozen coefficients
 * (g_blower_ctrl_physics_ff_hose_*, blower_ctrl.c). */
#define TUBECOMP_R_LAM_DEFAULT   0.00353f
#define TUBECOMP_R_TURB_DEFAULT  0.000118f
volatile float   g_tubecomp_R_lam_cmh2o_per_lpm  = TUBECOMP_R_LAM_DEFAULT;
volatile float   g_tubecomp_R_turb_cmh2o_per_lpm2 = TUBECOMP_R_TURB_DEFAULT;

/* Hose inertance, 2026-10-06: P_mask = P_sensor - R(Q) - L * dQ/dt.
 *
 * The gas in the hose has mass; accelerating it toward the patient takes
 * pressure the resistive model above does not see. Fitted on the
 * BreathSimulator (2 L lung, external sensor at the patient end of the hose,
 * 21 capture windows at 8 and 20 cmH2O, device flow aligned to the external
 * flow by cross-correlation): L = 0.0016 cmH2O per slm/s (IQR
 * 0.0014-0.0019), stable to within +-0.0003 when the pressure signals are
 * allowed +-60 ms of relative lag, so it is not a timing artefact. Physics
 * agrees: rho*l/A for 1.8 m of 15-22 mm hose is 0.001-0.002. It took the
 * residual (external minus our mask estimate) sd from 0.24 to 0.21 at 20 and
 * from 0.17 to 0.12 cmH2O at 8. On ~70 L/min breaths dQ/dt peaks ~100-200
 * slm/s, i.e. 0.16-0.3 cmH2O of mask pressure the static model misses - at
 * inspiratory onset and as expiratory flow returns to zero (mask low), and
 * at end-inspiration (mask high).
 *
 * Only the controller's call (TubeComp_ApplyWithInertance, blower_ctrl.c)
 * adds it; main.c's leak-estimator call stays static. dQ/dt is total flow
 * through a first-order filter (L_tau). The derivative of the flow signal
 * feeds the PID's feedback, so flow noise becomes pressure noise.
 *
 * BENCH RESULT (same day, simulator, 20 cmH2O, exp lead on): it does NOT
 * reduce the end-of-expiration trough at the lung, which was the reason for
 * adding it. Trough / spike / exp plateau / rest sd, cmH2O:
 *     L 0              -0.31..-0.37   +0.96..1.08   +0.14..0.16   0.067-0.072
 *     L .0016 db 80    -0.35..-0.37   +0.91..0.97   +0.07..0.09   0.070-0.086
 *     L .0016 db 40    -0.33..-0.34   +0.86..0.97   +0.03..0.06   0.071-0.077
 *     L .0016 db 20    -0.33          +0.85         +0.05         0.082
 *     L .0016 db 0     -0.41          +0.77         +0.01         0.098
 *     L .0032 db 80    -0.35          +0.91         -0.01         0.070
 * The trough is not reachable through feedback within its ~0.4 s: with a
 * larger exp lead (k 15) the blower ran ~400 rpm faster through it and the
 * lung still dipped the same, while the mask estimate overshot afterwards.
 * What L does buy is a smaller spike and a flatter expiratory plateau, paid
 * for in rest ripple once the deadband comes down. Left OFF (L = 0); the
 * fitted value is the one to use if that trade is ever wanted.
 *      tubecomp set L 0.0016
 *      tubecomp set L_tau 0.05
 *      tubecomp set L_db 80 */
volatile float   g_tubecomp_L_cmh2o_per_slm_per_s = 0.0f;
volatile float   g_tubecomp_L_dqdt_tau_s          = 0.05f;
/* |dQ/dt| below this is treated as noise (soft deadband: the term uses
 * |dQ/dt| - db). See TubeComp_ApplyWithInertance. */
volatile float   g_tubecomp_L_dqdt_db_slm_per_s   = 80.0f;
volatile float   g_tubecomp_inert_cmh2o           = 0.0f;   /* diagnostic */
static float     s_inert_q_prev  = 0.0f;
static bool      s_inert_q_valid = false;
static float     s_inert_dqdt    = 0.0f;

/* === Live values (debugger-readable volatiles) === */
volatile float   g_tubecomp_p_sensor_cmh2o = 0.0f;
volatile float   g_tubecomp_p_mask_cmh2o   = 0.0f;
volatile float   g_tubecomp_q_slm          = 0.0f;
volatile float   g_tubecomp_drop_cmh2o     = 0.0f;
volatile uint32_t g_tubecomp_step_count    = 0U;

/* === Public API === */

void TubeComp_Init(void)
{
    g_tubecomp_enabled                = 1U;
    g_tubecomp_R_lam_cmh2o_per_lpm    = TUBECOMP_R_LAM_DEFAULT;
    g_tubecomp_R_turb_cmh2o_per_lpm2  = TUBECOMP_R_TURB_DEFAULT;
    g_tubecomp_p_sensor_cmh2o         = 0.0f;
    g_tubecomp_p_mask_cmh2o           = 0.0f;
    g_tubecomp_q_slm                  = 0.0f;
    g_tubecomp_drop_cmh2o             = 0.0f;
    g_tubecomp_step_count             = 0U;
    g_tubecomp_L_cmh2o_per_slm_per_s  = 0.0f;
    g_tubecomp_L_dqdt_tau_s           = 0.05f;
    g_tubecomp_L_dqdt_db_slm_per_s    = 80.0f;
    g_tubecomp_inert_cmh2o            = 0.0f;
    s_inert_q_prev  = 0.0f;
    s_inert_q_valid = false;
    s_inert_dqdt    = 0.0f;
}

void  TubeComp_SetEnable(bool en) { g_tubecomp_enabled = en ? 1U : 0U; }
bool  TubeComp_IsEnabled(void)    { return (g_tubecomp_enabled != 0U); }

void TubeComp_SetCoefficients(float R_lam, float R_turb)
{
    /* Defensive clamping: physically sensible adult-CPAP hose has
     * R_turb in [0, 0.001] and R_lam in [0, 0.05]. Reject obviously
     * silly numbers - they'd swamp the loop. */
    if (R_lam  < 0.0f)     R_lam  = 0.0f;
    if (R_lam  > 0.05f)    R_lam  = 0.05f;
    if (R_turb < 0.0f)     R_turb = 0.0f;
    if (R_turb > 0.001f)   R_turb = 0.001f;
    g_tubecomp_R_lam_cmh2o_per_lpm    = R_lam;
    g_tubecomp_R_turb_cmh2o_per_lpm2  = R_turb;
}

void TubeComp_SetInertance(float L_cmh2o_per_slm_per_s, float dqdt_tau_s)
{
    if (!(L_cmh2o_per_slm_per_s >= 0.0f)) L_cmh2o_per_slm_per_s = 0.0f;
    if (L_cmh2o_per_slm_per_s > 0.01f)    L_cmh2o_per_slm_per_s = 0.01f;
    if (!(dqdt_tau_s >= 0.0f))            dqdt_tau_s = 0.0f;
    if (dqdt_tau_s > 0.5f)                dqdt_tau_s = 0.5f;
    g_tubecomp_L_cmh2o_per_slm_per_s = L_cmh2o_per_slm_per_s;
    g_tubecomp_L_dqdt_tau_s          = dqdt_tau_s;
}

float TubeComp_GetInertanceL(void)       { return g_tubecomp_L_cmh2o_per_slm_per_s; }
float TubeComp_GetInertanceTau(void)     { return g_tubecomp_L_dqdt_tau_s; }
void  TubeComp_SetInertanceDeadband(float db_slm_per_s)
{
    if (!(db_slm_per_s >= 0.0f)) db_slm_per_s = 0.0f;
    if (db_slm_per_s > 300.0f)   db_slm_per_s = 300.0f;
    g_tubecomp_L_dqdt_db_slm_per_s = db_slm_per_s;
}
float TubeComp_GetDefaultRLam(void)      { return TUBECOMP_R_LAM_DEFAULT; }
float TubeComp_GetDefaultRTurb(void)     { return TUBECOMP_R_TURB_DEFAULT; }
float TubeComp_GetRLam(void)             { return g_tubecomp_R_lam_cmh2o_per_lpm; }
float TubeComp_GetRTurb(void)            { return g_tubecomp_R_turb_cmh2o_per_lpm2; }
float TubeComp_GetLastSensorCmh2o(void)  { return g_tubecomp_p_sensor_cmh2o; }
float TubeComp_GetLastMaskCmh2o(void)    { return g_tubecomp_p_mask_cmh2o; }
float TubeComp_GetLastDropCmh2o(void)    { return g_tubecomp_drop_cmh2o; }
float TubeComp_GetLastFlowSlm(void)      { return g_tubecomp_q_slm; }

float TubeComp_Apply(float p_sensor_cmh2o, float q_slm)
{
    g_tubecomp_p_sensor_cmh2o = p_sensor_cmh2o;
    g_tubecomp_q_slm          = q_slm;
    g_tubecomp_step_count++;

    if (g_tubecomp_enabled == 0U)
    {
        /* Pass-through: mask = sensor, no drop, no surprise. */
        g_tubecomp_drop_cmh2o   = 0.0f;
        g_tubecomp_p_mask_cmh2o = p_sensor_cmh2o;
        return p_sensor_cmh2o;
    }

    /* dP magnitude is always positive; sign comes from flow direction.
     * Convention: Q > 0 means flow from device toward patient (the usual
     * direction during inspiration). With positive Q, P_mask < P_sensor.
     * During exhalation Q is negative and P_mask > P_sensor for the same
     * reason - the air piles up against the closed inspiratory limb. */
    const float qabs    = fabsf(q_slm);
    const float drop_mag = g_tubecomp_R_lam_cmh2o_per_lpm   * qabs
                         + g_tubecomp_R_turb_cmh2o_per_lpm2 * qabs * qabs;
    const float drop    = (q_slm >= 0.0f) ? drop_mag : -drop_mag;

    g_tubecomp_drop_cmh2o   = drop;
    g_tubecomp_p_mask_cmh2o = p_sensor_cmh2o - drop;
    return g_tubecomp_p_mask_cmh2o;
}

float TubeComp_ApplyWithInertance(float p_sensor_cmh2o, float q_slm, float dt_s)
{
    const float p_static = TubeComp_Apply(p_sensor_cmh2o, q_slm);

    /* Keep the derivative running whatever L is, so enabling it at
     * runtime does not start from a stale filter. */
    if (isfinite(q_slm) && (dt_s > 0.0f))
    {
        float dq_dt = 0.0f;
        if (s_inert_q_valid)
        {
            dq_dt = (q_slm - s_inert_q_prev) / dt_s;
        }
        s_inert_q_prev  = q_slm;
        s_inert_q_valid = true;

        float tau = g_tubecomp_L_dqdt_tau_s;
        if (tau < 0.0f) { tau = 0.0f; }
        if (tau > 0.5f) { tau = 0.5f; }
        const float a = dt_s / (tau + dt_s);
        s_inert_dqdt = (a * dq_dt) + ((1.0f - a) * s_inert_dqdt);
    }

    float L = g_tubecomp_L_cmh2o_per_slm_per_s;
    if ((g_tubecomp_enabled == 0U) || !(L > 0.0f))
    {
        g_tubecomp_inert_cmh2o = 0.0f;
        return p_static;
    }
    if (L > 0.01f) { L = 0.01f; }

    /* Soft deadband. The ~150 slm element's flow noise (mostly pulsation)
     * differentiates into a dQ/dt the loop would otherwise chase: in the twin
     * with its measured noise, L 0.0016 raised the speed-command jitter from
     * ~220 to ~550 rpm sd (no instability - without flow noise it was
     * unchanged). Breath-scale events run 100-200 slm/s, so a deadband keeps
     * most of the term where it matters. Twin, L 0.0016, jitter rpm sd:
     *     tau 30 ms: db 0 566, 40 406, 80 320, 120 272
     *     tau 50 ms: db 0 401, 40 319, 80 260, 120 241    (L 0: 224)
     * Defaults 50 ms / 80 slm/s. The twin has no hose inertia in its plant,
     * so it can only judge the noise cost, not the benefit. */
    float db = g_tubecomp_L_dqdt_db_slm_per_s;
    if (!(db >= 0.0f)) { db = 0.0f; }
    if (db > 300.0f)   { db = 300.0f; }
    float dq_eff = 0.0f;
    if (s_inert_dqdt > db)       { dq_eff = s_inert_dqdt - db; }
    else if (s_inert_dqdt < -db) { dq_eff = s_inert_dqdt + db; }
    float inert = L * dq_eff;
    if (inert >  1.0f) { inert =  1.0f; }
    if (inert < -1.0f) { inert = -1.0f; }
    g_tubecomp_inert_cmh2o  = inert;
    g_tubecomp_p_mask_cmh2o = p_static - inert;
    return g_tubecomp_p_mask_cmh2o;
}
