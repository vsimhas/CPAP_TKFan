/**
  ******************************************************************************
  * @file    therapy.h
  * @brief   CPAP therapy state machine.
  *
  *  Architecture
  *  ------------
  *      [GUI: START THERAPY click] -> Therapy_Start()
  *
  *      SensorTask (200 Hz, high): sensors + leak + PID + Therapy_Update
  *      SafetyTask (200 Hz, realtime): safety_supervisor (preempts SensorTask)
  *
  *      States: IDLE -> RAMP -> RUNNING -> STOPPING -> IDLE
  *              FAULT latched by SafetyTask or motor fault
  *
  *  Outputs of the state machine:
  *      - blower start/stop commands via blower_ipc_cm7,
  *      - PID enable + setpoint via blower_ctrl,
  *      - leak monitoring threshold (leak_estimator already runs free
  *        but only matters during therapy).
  *
 *  Therapy modes (AirSense-11 style)
 *  ---------------------------------
 *      - CPAP: fixed therapy pressure (4-25 cmH2O).
 *      - Autoset / Autoset for her: APAP titration between min and max
 *        pressure (see apap_ctrl.c). Starts at min; raises on OSA/FL/hyp/snore.
 *      - Optional pressure ramp from RAMP_START to target over the
 *        ramp_minutes setting (Off or 5..45 min), or AutoRamp (#38):
 *        hold the start pressure until sleep onset, then rise.
 *      - Run timer, leak status, current/target pressure available
 *        for the GUI to display.
 *
 *  Out of scope for this iteration:
 *      - Snore / flow-limitation response, CSA detection, long-term
 *        pressure reduction after stable periods (full AirSense feature set).
  *      - SmartStart/SmartStop - needs flow-based start/stop detection.
  *      - Therapy session logging to flash/SD.
  ******************************************************************************
  */

#ifndef THERAPY_H
#define THERAPY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "leak_estimator.h"

#define THERAPY_PRESS_MIN_CMH2O  4.0f
#define THERAPY_PRESS_MAX_CMH2O  25.0f

typedef enum
{
    THERAPY_STATE_IDLE = 0,    /* blower off, PID off, waiting for Start    */
    THERAPY_STATE_RAMP,        /* blower on, PID on, setpoint ramping       */
    THERAPY_STATE_RUNNING,     /* at target pressure, monitoring leak       */
    THERAPY_STATE_STOPPING,    /* user stopped, blower spinning down        */
    THERAPY_STATE_FAULT        /* error: motor fault, persistent high leak  */
} TherapyState_t;

typedef enum
{
    THERAPY_MODE_CPAP = 0,           /* fixed pressure                      */
    THERAPY_MODE_AUTOSET,            /* APAP titration (apap_ctrl.c)        */
    THERAPY_MODE_AUTOSET_FOR_HER     /* gentler APAP response             */
} TherapyMode_t;

/* === Safety supervisor =====================================================
 *
 *  Fault classes that cause the therapy supervisor to shut the blower down
 *  and transition to THERAPY_STATE_FAULT. The screen reports the reason so
 *  the user knows what went wrong.
 */
typedef enum
{
    THERAPY_FAULT_NONE = 0,

    /* Pressure stayed below `g_therapy_low_press_cmh2o` (default 1.5 cmH2O)
     * for `g_therapy_low_press_timeout_s` (default 4 s) while the PID was
     * commanding more than `g_therapy_high_rpm_threshold` (default 12 krpm).
     * Means: pressure-sensor disconnected, hose disconnected at the blower
     * outlet, or a leak so large that the controller is winding up. */
    THERAPY_FAULT_LOW_PRESS_AT_HIGH_RPM,

    /* Measured pressure exceeded `target + g_therapy_overpress_band_cmh2o`
     * (default +5 cmH2O over target) for `g_therapy_overpress_timeout_s`
     * (default 3 s). Means: sensor stuck high, controller runaway, or
     * mechanical occlusion downstream. */
    THERAPY_FAULT_OVER_PRESSURE,

    /* Measured pressure fell outside the AMS5935 plausible range
     * (-10 cmH2O .. +50 cmH2O) for 2 s. Means: sensor electrical fault,
     * SPI corruption, bad cable. */
    THERAPY_FAULT_SENSOR_OUT_OF_RANGE,

    /* RPM has been pinned at `g_therapy_safe_rpm_max` for
     * `g_therapy_saturated_timeout_s` (default 6 s) without the pressure
     * converging within `g_therapy_saturate_band_cmh2o` (default 1.5
     * cmH2O) of target. Means: the controller can't deliver setpoint -
     * usually a too-large leak or undersized blower for this circuit. */
    THERAPY_FAULT_RPM_SATURATED,

    /* CM4 `cm4_status_seq` in the blower IPC mailbox stopped advancing
     * while therapy expected the motor domain to be alive. */
    THERAPY_FAULT_CM4_HEARTBEAT
} TherapyFault_t;

/* Bring-up at boot. Idempotent; safe to call multiple times. */
void   Therapy_Init(void);

/* Tick. Call every dt_s seconds from the same task that runs the PID
 * (SensorTask, 5 ms). State transitions and ramp updates happen here. */
void   Therapy_Update(float dt_s);

/* User-driven session control. Safe to call from any thread/context. */
void   Therapy_Start(void);
void   Therapy_Stop(void);

/* Configuration (the GUI calls these from cpap_settings on edit). */
void   Therapy_SetTargetPressure(float cmh2o);   /* clamped 4..25         */
float  Therapy_GetTargetPressure(void);
void   Therapy_SetRampMinutes(uint8_t minutes);  /* 0 = no ramp           */
uint8_t Therapy_GetRampMinutes(void);
/* #38 AutoRamp: overrides the minutes. Hold the ramp start pressure until
 * sleep onset, then rise to the target at g_therapy_autoramp_rate_cmh2o_min,
 * reaching it within g_therapy_autoramp_max_min of start regardless. */
void     Therapy_SetRampAuto(bool en);
/* #5: pressure the ramp (fixed or Auto) starts from; held at 4 cmH2O ..
 * session target when therapy starts. Default 4. */
void     Therapy_SetRampStartPressure(float cmh2o);
float    Therapy_GetRampStartPressure(void);
bool     Therapy_GetRampAuto(void);
uint8_t  Therapy_GetAutoRampPhase(void);     /* 0 holding, 1 rising        */
uint8_t  Therapy_GetAutoRampCause(void);     /* 0 none yet, 1 breathing, 2 events,
                                                3 snore, 4 time limit      */
uint32_t Therapy_GetAutoRampOnsetSeconds(void);  /* start -> onset        */
void   Therapy_SetMode(TherapyMode_t m);
TherapyMode_t Therapy_GetMode(void);
const char*    Therapy_GetModeName(void);
void   Therapy_SetApapMinPressure(float cmh2o);  /* clamped 4..25       */
void   Therapy_SetApapMaxPressure(float cmh2o);  /* clamped 4..25       */
float  Therapy_GetApapMinPressure(void);
float  Therapy_GetApapMaxPressure(void);
const char* Therapy_GetApapStateName(void);

/* EPR (Expiratory Pressure Relief). AirSense-style type + level. */
void   Therapy_SetEprEnabled(bool en);
bool   Therapy_IsEprEnabled(void);
void   Therapy_SetEprType(uint8_t type);       /* 0=Full time, 1=Ramp only */
uint8_t Therapy_GetEprType(void);
void   Therapy_SetEprLevel(uint8_t level);     /* 1, 2, or 3 cmH2O       */
uint8_t Therapy_GetEprLevel(void);
void   Therapy_SetEprReliefCmh2o(float cmh2o); /* alias for level          */
float  Therapy_GetEprReliefCmh2o(void);

/* Mask-off during therapy: on sustained high leak + pressure collapse, hold
 * blower at low RPM; resume when the circuit seals again.
 *
 * Enter paths (OR), both gated against breath false-positives:
 *   - Fast: |Flow_GetSlm()| (raw orifice total) high for enter_ms, min flow
 *     during arm stays elevated, AND pressure collapsed vs setpoint.
 *     Nonfinite flow, or flow held for more than flow_stale_ticks_max
 *     consecutive ticks (default 6 = 30 ms), cannot arm; a routine one-tick
 *     held read uses the held value. Clipped flow or an upper-limited PID
 *     additionally requires open-circuit pressure on BOTH signals: the mask
 *     estimate below min(abs_floor, SP-drop) and the device-side sensor
 *     below min(abs_floor + press_sensor_allow, SP-drop) - the sensor still
 *     reads the hose drop at open-circuit flow (default allowance 2.5).
 *     This keeps disconnects detectable at saturation, including at 20
 *     cmH2O, while rejecting ambiguous inspiratory sag. These quality guards
 *     cannot be bypassed by disabling require_press. diag_fast_block bits
 *     are 1=invalid flow, 2=clipped without open-circuit pressure,
 *     4=upper-limited without open-circuit pressure.
 *   - Slow: Vivo MeanRatio10Sec > limit for ratio_enter_ms, AND pressure
 *     collapsed when require_press is set.
 *   - Pressure-only backstop: commanded RPM >= press_only_rpm with both the
 *     sensor and mask pressure below min(press_only_frac x setpoint,
 *     press_only_cmh2o) for press_only_ms, and
 *     never inside a safety grace. Reads no flow, so a dead or under-reading
 *     flow element cannot hide a disconnect; it parks the blower instead of
 *     letting the PID run away into a LOW_PRESS_AT_HIGH_RPM fault. Disable
 *     with g_therapy_mask_off_press_only_enable = 0.
 *   - RPM-ratio backstop: commanded RPM >= rpm_ratio x the square-law model's
 *     estimate for the current setpoint, with both pressures short of the
 *     setpoint, for rpm_ratio_ms. Scale-invariant, so it covers the bottom of
 *     the therapy range (~4-13 cmH2O) where an open circuit still holds
 *     nearly the set pressure; above that the sealed circuit already needs
 *     most of the RPM ceiling and there is no headroom left, which is the
 *     range the pressure-only path covers. Disable with
 *     g_therapy_mask_off_rpm_ratio_enable = 0.
 *
 * Clearing is treated as a guess and then checked. After a clear the resume
 * is probationary: if the circuit has not reached verify_frac of the therapy
 * target within verify_ms, the clear is judged wrong, the blower re-parks
 * immediately, and the clear confirm time grows by clear_backoff_ms (capped
 * at clear_ms_max) so a repeatedly brushed mask cannot keep pumping the
 * blower up to speed. The first clear that does hold forgives the backoff.
 * Count of bad clears: g_therapy_mask_off_false_clear_count.
 *
 * Clear uses the same raw |Q_total| (not patient flow / InstantExcess).
 *
 * Clear paths (OR):
 *   - Pressure tip above the open-circuit reference learned at hold RPM.
 *   - Sustained flow collapse (g_therapy_mask_off_flow_clear_ms). Needed when
 *     the mask goes back on before the decel finishes: the reference is then
 *     the sealed hold pressure, so no tip can ever appear above it.
 *   - Hold watchdog (g_therapy_mask_off_max_hold_ms): re-probe at therapy
 *     pressure so a mask-off can never latch for the rest of the session.
 *
 * Reconnect at hold RPM (default 6000): soft-ramp down (default 1000 ms)
 * to avoid bus over-voltage, then hold for sensing.
 * Debugger: g_therapy_mask_off_reconn_* / g_therapy_mask_off_diag_* /
 * g_therapy_mask_off_ramp_ms / g_therapy_mask_off_clear_slm. */
bool   Therapy_IsMaskOffActive(void);
/* True while an enter path is counting toward its confirm time. */
bool   Therapy_IsMaskOffArming(void);
void   Therapy_SetMaskOffEnabled(bool en);
bool   Therapy_IsMaskOffEnabled(void);
/* Bench/recovery: force exit from mask-off and resume therapy pressure. */
void   Therapy_ForceMaskOn(void);

/* Live state for GUI / logging. */
TherapyState_t Therapy_GetState(void);
const char*    Therapy_GetStateName(void);
float          Therapy_GetSetpointCmh2o(void);    /* what we *ask* for     */
float          Therapy_GetMeasuredCmh2o(void);    /* mask pressure         */
float          Therapy_GetLeakSlm(void);          /* unintentional leak    */
bool           Therapy_IsLeakHigh(void);
bool           Therapy_IsLeakLow(void);           /* insufficient total flow hint */
LeakSeverity_t Therapy_GetLeakSeverity(void);
const char*    Therapy_GetLeakSeverityName(void);
float          Therapy_GetLeakPercentOfVent(void);
uint32_t       Therapy_GetMaskOffEnterCount(void);
uint32_t       Therapy_GetMaskOnCount(void);
uint32_t       Therapy_GetRunSeconds(void);       /* time in RAMP+RUNNING  */
uint32_t       Therapy_GetSessionMaskOffSeconds(void); /* part of it mask-off */
uint32_t       Therapy_GetRampRemainingSeconds(void); /* 0 if not ramping  */

/* Session delivered-pressure statistics (RAM; reset on Therapy_Start).
 * Checklist #63: median / 90th / 95th percentile for Auto→fixed conversion.
 * Accumulated in RAMP+RUNNING while mask is on. */
float          Therapy_GetSessionPressureAvg(void);
float          Therapy_GetSessionPressureMin(void);
float          Therapy_GetSessionPressureMax(void);
float          Therapy_GetSessionPressureP50(void);  /* median */
float          Therapy_GetSessionPressureP90(void);
float          Therapy_GetSessionPressureP95(void);
uint32_t       Therapy_GetSessionPressureSamples(void);

/* Fault inspection / recovery. */
TherapyFault_t Therapy_GetFault(void);
const char*    Therapy_GetFaultName(void);
void           Therapy_ClearFault(void);          /* operator acknowledge   */

/* Called by safety_supervisor.c on trip. */
void           Therapy_LatchFault(TherapyFault_t f, const char *why);
bool           Therapy_SafetyGraceActive(void);

#ifdef __cplusplus
}
#endif

#endif /* THERAPY_H */
