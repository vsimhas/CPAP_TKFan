/**
  ******************************************************************************
  * @file    leak_estimator.h
  * @brief   Unintentional-leak estimator for CPAP therapy.
  *
  *  Concept
  *  -------
  *  At any instant the total flow leaving the blower is
  *
  *      Q_total(t) = Q_patient(t) + Q_vent(P_mask) + Q_leak(t)
  *
  *  where:
  *      - Q_patient is the patient breathing flow. It is approximately
  *        zero-mean over multiple breath cycles (inhale equals exhale to
  *        within tidal-volume drift), so it averages out under a slow LPF.
  *      - Q_vent is the *intentional* leak through the mask exhaust port.
  *        For typical orifice-style vents this is well modelled as
  *            Q_vent = K_vent * sqrt(P_mask)
  *        with K_vent ~10-12 slm/sqrt(cmH2O) for adult masks.
  *      - Q_leak is the *unintentional* leak we want to estimate (mouth
  *        leak, mask seal breakdown, hose disconnect).
  *
  *  Therefore
  *
  *      Q_leak ~= LPF_slow( Q_total - K_vent*sqrt(P_mask) )
 *
 *  Patient breathing flow (approximately zero-mean over a breath cycle):
 *
 *      Q_patient(t) ~= Q_total(t) - Q_vent(t) - Q_leak_slow
 *
 *  exposed as g_flow_patient_slm / Leak_GetPatientSlm() /
 *  Flow_GetPatientSlm().
  *
  *  with the LPF time constant chosen long enough to suppress the
  *  ~0.2-0.5 Hz breath cycle (we use 10 s by default).
  *
  *  Outputs feed:
  *      - the THERAPY screen ("Mask: Sealed / Small leak / High leak"),
  *      - a future high-leak alarm,
  *      - therapy event logs.
  *
 *  Default thresholds match the ResMed AirSense convention:
 *      |Q_leak| > 24 slm  -> "high leak" condition (efficacy compromised).
 *
 *  Two backends (selectable at runtime):
 *      - Vivo-style (default): bench table × adaptive MeanRatio
 *        (leak_estimator_vivo.c).
 *      - Simple: slow LPF on Q_total - K*sqrt(P).
 ******************************************************************************
 */

#ifndef LEAK_ESTIMATOR_H
#define LEAK_ESTIMATOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    LEAK_ALGO_SIMPLE = 0,
    LEAK_ALGO_VIVO   = 1,
} LeakAlgo_t;

/* Display / annotation bands (THERAPY screen + telemetry). */
typedef enum
{
    LEAK_SEV_SEALED = 0,   /* |Q_unint| <= small threshold */
    LEAK_SEV_SMALL  = 1,   /* small < |Q_unint| <= high    */
    LEAK_SEV_HIGH   = 2,   /* |Q_unint| > high threshold   */
} LeakSeverity_t;

/* Reset state, load defaults (K_vent = 11, tau = 10 s, threshold = 24 slm). */
void  Leak_Init(void);

/* Algorithm selection (default: LEAK_ALGO_VIVO). */
void       Leak_SetAlgo(LeakAlgo_t algo);
LeakAlgo_t Leak_GetAlgo(void);
void       Leak_SetVivoEnable(bool en);   /* convenience: en -> VIVO, !en -> SIMPLE */
bool       Leak_IsVivoEnabled(void);

/* Therapy setpoint for Vivo reported leak / ratio reference [cmH2O]. */
void  Leak_SetTherapyPressureCmh2o(float cmh2o);
void  Leak_SetStandardRatio(int32_t ratio);

/* Live tunables. All clamp internally. */
void  Leak_SetVentCoefficient(float k_slm_per_sqrt_cmh2o);  /* 5..20    */
void  Leak_SetTimeConstant(float seconds);                   /* 1..60    */
void  Leak_SetHighThreshold(float slm);                      /* 5..60    */
void  Leak_SetSmallThreshold(float slm);                     /* 1..high  */
void  Leak_SetLowThreshold(float total_slm);                 /* 1..40; blocked-vent hint */

float Leak_GetVentCoefficient(void);
float Leak_GetTimeConstant(void);
float Leak_GetHighThreshold(void);
float Leak_GetSmallThreshold(void);
float Leak_GetLowThreshold(void);

/* Push one new measurement pair into the estimator. dt_s is the elapsed
 * time since the previous call. Designed to be called once per sensor
 * tick (5 ms in SensorTask). Both inputs should already be calibrated
 * and noise-filtered (Flow_GetSlm and BlowerCtrl_GetMeasured fit). */
void  Leak_Update(float total_q_slm, float mask_pressure_cmh2o, float dt_s);

/* Read current values (no side effect). */
float Leak_GetTotalSlm(void);          /* last total flow input             */
float Leak_GetVentSlm(void);           /* modelled vent flow at current P   */
float Leak_GetUnintentionalSlm(void);  /* slow-LPF unintentional leak       */
float Leak_GetInstantExcessSlm(void);  /* Q_total - Q_vent (no slow LPF)    */
float Leak_GetPatientSlm(void);       /* vent+leak corrected breath flow   */
bool  Leak_IsHigh(void);               /* |unint| > high threshold          */
bool  Leak_IsLow(void);                /* pressurized + very low Q_total    */
LeakSeverity_t Leak_GetSeverity(void);
const char*    Leak_GetSeverityName(void); /* "Sealed" / "Small leak" / "HIGH LEAK" */
float Leak_GetPercentOfVent(void);     /* 100*|Q_unint|/max(Q_vent,eps)     */

/* Session accumulators (RAM only). Reset on Therapy_Start via Leak_SessionReset.
 * Accumulate only while Leak_SetSessionActive(true). */
void     Leak_SessionReset(void);
void     Leak_SetSessionActive(bool active);
bool     Leak_IsSessionActive(void);
float    Leak_GetSessionAvgSlm(void);
float    Leak_GetSessionMaxSlm(void);
float    Leak_GetSessionHighTimeS(void);   /* time |unint| above high thresh */
float    Leak_GetSessionLowTimeS(void);    /* time low/blocked-vent flag set */
float    Leak_GetSessionPercentileSlm(float pct); /* |unint| percentile, L/min */
float    Leak_GetSessionElapsedS(void);

/* Estimator health counters (monotonic until SessionReset / Init). */
uint32_t Leak_GetHealthHighSamples(void);
uint32_t Leak_GetHealthLowSamples(void);
uint32_t Leak_GetHealthReseedCount(void);
uint32_t Leak_GetHealthRatioClampCount(void); /* Vivo MeanRatio hit max */
void     Leak_NoteRatioClamp(void);           /* called from Vivo backend */

/* Extended metrics (Vivo backend; zeros / simple values when SIMPLE). */
int32_t Leak_GetMeanRatio(void);
int32_t Leak_GetMeanRatio10Sec(void);
int32_t Leak_GetInstantRatio(void);    /* Vivo Q/table ratio; 0 when SIMPLE */

/* Freeze Vivo per-breath leak adaptation during mask-off / disconnect. */
void    Leak_SetDisconnectFreeze(bool freeze);
/* APAP-owned freeze during NO_BREATH / FOT; ORed with the disconnect one. */
void    Leak_SetApneaFreeze(bool freeze);
float   Leak_GetFlowLeakSlm(void);     /* total modeled leak subtracted   */
float   Leak_GetReportedLeakSlm(void); /* leak at therapy setpoint        */
float   Leak_GetRestVolumeMl(void);
uint8_t Leak_GetBreathInspiration(void);
float   Leak_GetBreathRateBpm(void);
float   Leak_GetLastInspVolumeMl(void);
float   Leak_GetLastExpVolumeMl(void);
float   Leak_GetPhaseVolumeMl(void);

/* Vent- and leak-corrected patient flow [slm], breath-band LPF applied.
 * Published Q_patient is g_leak_patient_lpf_hz (2 Hz) on breath. If
 * g_leak_patient_quiet_lpf_enable, pause uses 0.35 Hz. Off by default.
 * Plot, EPR, FL/hyp use published Q. FOT MUST use raw + Flow_FotBpStep().
 * Debugger tunables: g_leak_patient_lpf_hz, g_leak_patient_quiet_lpf_enable,
 * g_leak_patient_quiet_lpf_hz, quiet_band / quiet_exit, g_leak_vent_p_tau_s. */
extern volatile float g_flow_patient_slm;
extern volatile float g_flow_patient_raw_slm;
extern volatile float g_leak_instant_excess_slm;
extern volatile float g_leak_patient_lpf_hz;
extern volatile uint8_t g_leak_patient_quiet_lpf_enable;
extern volatile float g_leak_patient_quiet_lpf_hz;
extern volatile float g_leak_patient_quiet_band_slm;
extern volatile float g_leak_patient_quiet_exit_slm;
extern volatile uint8_t g_leak_patient_quiet_now;
extern volatile float g_leak_patient_pause_dc_slm;

float Leak_PatientLpfHz(float q_raw_slm);
float Leak_PatientFiltStep(float q_raw_slm, float dt_s);
void  Leak_PatientFiltReset(void);

/* Vivo backend tunables / state (no-op when LEAK_ALGO_SIMPLE). */
extern volatile uint8_t g_leak_vivo_enable;
extern volatile int32_t g_leak_vivo_mean_ratio;
extern volatile int32_t g_leak_vivo_mean_ratio_10s;

/* Force the slow LPF to converge instantly to the next sample. Use this
 * at therapy start so the user doesn't have to wait ~30 s for the
 * estimator to settle from zero. */
void  Leak_Reseed(void);

/* Hold leak adaptation for ticks where the flow sample was not fresh
 * (held AMS5935 conversion). Prevents the slow leak LPF from converging
 * onto a frozen Q_total and then stepping when real data resumes. */
void  Leak_SetHoldAdaptation(bool hold);

/* Keep Q_patient continuous when Q_total steps for a non-flow reason
 * (flow zero-offset trim). Adds delta to the unintentional-leak LPF so
 * patient_raw = (Q−vent) − leak does not spike. No-op before first seed. */
void  Leak_NudgeUnintentional(float delta_slm);

#ifdef __cplusplus
}
#endif

#endif /* LEAK_ESTIMATOR_H */
