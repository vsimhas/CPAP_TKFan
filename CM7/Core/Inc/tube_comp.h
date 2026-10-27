/**
  ******************************************************************************
  * @file    tube_comp.h
  * @brief   Hose / tube pressure-drop compensator.
  *
  *  Why this exists
  *  ---------------
  *  The pressure sensor (AMS5935 hamsPS) sits inside the device, not at the
  *  mask. While the patient is breathing, flow through the hose introduces
  *  a pressure drop, so
  *
  *      P_sensor  !=  P_mask
  *
  *  even when a conventional CPAP loop is "perfectly tracking setpoint".
  *  The user-visible symptom is a mask pressure that visibly oscillates
  *  with each breath when the hose has appreciable resistance (every
  *  hose >= 15 mm x 1.5 m does).
  *
  *  This module implements the standard quadratic hose-resistance model
  *
  *      dP_hose(Q)  =  R_lam * |Q|   +   R_turb * Q * |Q|       [cmH2O]
  *      P_mask      =  P_sensor      -   sign(Q) * |dP_hose|    [cmH2O]
  *
  *  with Q in L/min (slm-equivalent) and the two coefficients chosen for
  *  the specific hose. Typical adult-CPAP hoses (1.8 m, 15-22 mm ID):
  *
  *      hose                R_lam              R_turb
  *      -----------------------------------------------------
  *      22 mm corrugated    ~0.001             ~0.00005
  *      15 mm slimline      ~0.002             ~0.00018
  *      ClimateLineAir 15   ~0.002             ~0.00012
  *
  *  Numbers are from regression of the standard CPAP industry curves;
  *  recalibrate per hose if you have a flow rig.
  *
  *  Calibration procedure (no flow rig, only the device):
  *      1. Block the patient end of the hose tightly with a plug.
  *      2. Run the device at a known steady RPM. Q falls almost to 0
  *         (only the vent leak flows). Note P_sensor.
  *      3. Replace the plug with the *intentional* mask vent only (no
  *         leaks, no patient). Q rises to ~30-50 slm. Note P_sensor and
  *         Q_total at the same RPM.
  *      4. The pressure delta between (2) and (3) divided by Q_total
  *         from (3) gives R_turb (assuming R_lam ~ 0).
  *
  *  Use
  *  ---
  *  Call TubeComp_Apply() with the freshly-converted sensor pressure and
  *  flow on every PID tick. It returns the mask-pressure estimate. The
  *  PID consumes that estimate as its measurement, so closed-loop
  *  control regulates *mask* pressure, not sensor pressure. The leak
  *  estimator reads BlowerCtrl_GetMeasured() and therefore picks up the
  *  same estimate automatically.
  ******************************************************************************
  */

#ifndef TUBE_COMP_H
#define TUBE_COMP_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* Reset state to defaults (enabled, R_lam=0.002, R_turb=0.00022 for a 15 mm
 * slimline-class hose). Idempotent. */
void  TubeComp_Init(void);

/* Default is OFF on boot - your current bench rig has effectively no
 * tube between the sensor and the test lung, so compensation makes the
 * loop noisier without a benefit. Enable when you connect a real hose. */
void  TubeComp_SetEnable(bool en);
bool  TubeComp_IsEnabled(void);

/* Coefficients, in cmH2O / (L/min)  and  cmH2O / (L/min)^2  respectively. */
void  TubeComp_SetCoefficients(float R_lam, float R_turb);
float TubeComp_GetRLam(void);
float TubeComp_GetRTurb(void);
/* The built-in pair (bench hose, 2026-09-22); circuit_learn.c can replace
 * it with the attached tube's (#85). */
float TubeComp_GetDefaultRLam(void);
float TubeComp_GetDefaultRTurb(void);

/* Per-tick API: feed in the sensor pressure (cmH2O) and current flow
 * (slm; equivalent to L/min in CPAP context), receive the mask-pressure
 * estimate. When disabled, returns p_sensor unchanged. The function is
 * pure (no I/O), can be called at any rate, and is safe to call before
 * the rest of the system is initialised. */
float TubeComp_Apply(float p_sensor_cmh2o, float q_slm);

/* TubeComp_Apply plus the hose inertance term, -L * dQ/dt (see
 * g_tubecomp_L_cmh2o_per_slm_per_s). Keeps derivative state, so call it from
 * ONE place at a fixed rate - the pressure controller. */
float TubeComp_ApplyWithInertance(float p_sensor_cmh2o, float q_slm, float dt_s);
void  TubeComp_SetInertance(float L_cmh2o_per_slm_per_s, float dqdt_tau_s);
float TubeComp_GetInertanceL(void);
float TubeComp_GetInertanceTau(void);
void  TubeComp_SetInertanceDeadband(float db_slm_per_s);

/* Read-only accessors for the last applied set of values. Useful for
 * the GUI / diagnostics screen. */
float TubeComp_GetLastSensorCmh2o(void);
float TubeComp_GetLastMaskCmh2o(void);
float TubeComp_GetLastDropCmh2o(void);
float TubeComp_GetLastFlowSlm(void);

#ifdef __cplusplus
}
#endif

#endif /* TUBE_COMP_H */
