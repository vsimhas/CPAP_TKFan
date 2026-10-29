/**
  ******************************************************************************
  * @file    epr_ctrl.h
  * @brief   Expiratory Pressure Relief (EPR / Flex-style).
  *
  *  During exhalation the therapy setpoint is reduced by a fixed relief
  *  amount so the patient exhales against a lower pressure. This matches
  *  ResMed EPR / Philips Flex behaviour at a basic level:
  *
  *      P_therapy_exhale = max(P_min, P_CPAP - relief)
  *
 *  Breath phase is inferred from vent/leak-corrected patient flow
 *  (Flow_GetPatientSlm) using a gated state machine:
 *
 *      IDLE -> (sustained insp) -> INSPIRATION -> (sustained exp) -> EXPIRATION
 *
 *  AirSense-style EPR types:
 *
 *    FULL_TIME  - baseline setpoint is (CPAP - level) except during
 *                 inspiration when full CPAP pressure is delivered.
 *    RAMP_ONLY  - same as FULL_TIME during the pressure ramp; after ramp
 *                 completes, EPR is off and therapy holds CPAP setpoint.
 *
 *  Relief level is 1, 2, or 3 cmH2O (AirSense EPR Level).
  ******************************************************************************
  */

#ifndef EPR_CTRL_H
#define EPR_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    EPR_PHASE_UNKNOWN = 0,
    EPR_PHASE_INSPIRATION,
    EPR_PHASE_EXPIRATION
} EprPhase_t;

typedef enum
{
    EPR_TYPE_FULL_TIME = 0,
    EPR_TYPE_RAMP_ONLY = 1,
} EprType_t;

void       Epr_Init(void);

void       Epr_SetEnable(bool en);
/* #8: mid-session switch-on / switch-off at the next breath boundary. */
void       Epr_EngageAtNextBreath(void);
void       Epr_DisengageAtNextBreath(void);
bool       Epr_IsTransitionPending(void);
bool       Epr_IsEnabled(void);

/* EPR type and level (AirSense-style). Level is 1, 2, or 3 cmH2O. */
void       Epr_SetType(EprType_t type);
EprType_t  Epr_GetType(void);
void       Epr_SetLevel(uint8_t level);
uint8_t    Epr_GetLevel(void);

/* Relief magnitude in cmH2O (mirrors level). Clamped 1..3. */
void       Epr_SetReliefCmh2o(float cmh2o);
float      Epr_GetReliefCmh2o(void);

/* Minimum pressure floor during EPR (default 4.0 cmH2O). */
void       Epr_SetMinPressureCmh2o(float cmh2o);
float      Epr_GetMinPressureCmh2o(void);

/* Per-tick update: feed patient flow (slm), mask pressure (cmH2O), and dt.
 * Call from the same task/rate as Therapy_Update (200 Hz), after
 * Leak_Update(). therapy_base_sp_cmh2o is the full CPAP target (before
 * EPR relief) used for insp-pressure-reached detection. */
void       Epr_Update(float q_slm, float p_meas_cmh2o,
                      float therapy_base_sp_cmh2o, float dt_s);

/* Expiratory setpoint (CPAP - level, floored) when EPR is active for the
 * given therapy state; otherwise returns base unchanged. */
float      Epr_GetExpiratorySetpointCmh2o(float base_setpoint_cmh2o,
                                          uint8_t therapy_state);

/* After therapy start / mask reconnect: hold expiratory pressure until
 * measured pressure is near this target (blocks false insp during spin-up). */
void       Epr_SetStartupHoldTarget(float expiratory_sp_cmh2o);

/* Apply EPR to a base CPAP setpoint (cmH2O). therapy_state is
 * TherapyState_t from therapy.h (passed as uint8_t to avoid a header
 * cycle). Exhale relief snaps in; inhale restore is slewed. */
float      Epr_ModifySetpoint(float base_setpoint_cmh2o, float dt_s,
                              uint8_t therapy_state);

EprPhase_t Epr_GetPhase(void);
const char* Epr_GetPhaseName(void);
float      Epr_GetActiveReliefCmh2o(void);   /* currently applied relief  */
float      Epr_GetFlowBaselineSlm(void);

/* Reset breath-phase state at therapy start (baseline re-seeds on next
 * flow sample). Does not change enable/relief settings. */
void       Epr_ResetBreathState(void);

/* Call when therapy resumes (start or mask reconnect). Snaps to
 * expiratory (CPAP - level) pressure and ignores brief false insp. */
void       Epr_OnTherapyStart(void);

/* True if a qualified inspiration (min hold + min peak) ended within window_ms. */
bool       Epr_HasRecentBreath(uint32_t window_ms);
float      Epr_GetLastInspPeakSlm(void); /* peak Q of that last qualified insp */
/* Length (ms, from insp onset to the stamp) and volume (mL) of the last
 * candidate inspiration, qualified or not - see g_epr_apap_breath_min_*. */
uint32_t   Epr_GetLastInspMs(void);
float      Epr_GetLastInspVolMl(void);

/* R11: read-only breath-stamp diagnostics for "apap diag". */
typedef struct
{
    uint32_t cand_ms;            /* last candidate: onset to end, ms     */
    float    cand_ml;            /* last candidate: volume, mL           */
    float    cand_peak_slm;      /* last candidate: qualifying peak      */
    uint8_t  cand_stamped;       /* 1 = it was stamped as a breath       */
    uint32_t rejects;            /* R5a/R8a rejects since reset          */
    float    vol_median_ml;      /* R8a median of stamped volumes        */
    uint32_t since_stamp_ms;     /* 0xFFFFFFFF = never stamped           */
    float    stamp_peak_slm;     /* peak of the last stamped breath      */
    float    flow_baseline_slm;  /* g_epr_flow_baseline_slm              */
    uint8_t  fsm;                /* breath FSM state                     */
    int8_t   polarity;           /* +1 / -1 in effect                    */
} EprDiag_t;
void       Epr_GetDiag(EprDiag_t *d);

/* True while EPR is delivering full CPAP (confirmed inspiration only). */
bool       Epr_IsInspiratoryPressureActive(void);

#ifdef __cplusplus
}
#endif

#endif /* EPR_CTRL_H */
