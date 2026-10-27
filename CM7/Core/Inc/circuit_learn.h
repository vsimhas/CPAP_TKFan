/**
  ******************************************************************************
  * @file    circuit_learn.h
  * @brief   Learn circuit: measure the attached tube (checklist #85).
  *
  *  The mask-pressure estimate subtracts the tube's pressure drop from the
  *  device-side sensor (tube_comp.c, dP = R_lam*Q + R_turb*Q*|Q|). Its
  *  coefficients were fitted for one hose on the bench; a different tube, a
  *  humidifier or an added accessory changes them, with no visible symptom.
  *
  *  This run measures them on the device itself. The user takes the mask off
  *  the tube (the patient end open to the room) and starts it from the menu
  *  (MORE > Learn Tube) or over USB (`circuit learn`). With the end open the
  *  pressure there is atmospheric, so the device sensor reads the drop from
  *  the sensor to the end of the tube directly - no external reference, and
  *  flow and pressure are not tied together the way a vent ties them.
  *
  *    1. blower at rest, 1 s: sensor zero (p0) and flow zero (q0)
  *    2. open-loop speed steps, 12000 rpm up to g_circuit_learn_rpm_max
  *       (default 26000, the open-outlet bench limit) in CL_STEPS steps;
  *       each held for 0.8 s at speed, then averaged for 1 s
  *    3. controlled ramp-down to 12000 rpm, then stop
  *    4. fit dP - p0 = R_lam*Q + R_turb*Q^2 (least squares, both >= 0)
  *
  *  Checks:
  *    not_open   after the first step the pressure is more than 60 % of the
  *               fan's shut-off pressure at that speed, or over 0.0015*Q^2
  *               (map-free), or the flow is under 20 L/min: the end is not
  *               open (mask still on, or blocked).
  *               The run stops at ~3 cmH2O and the coefficients are kept.
  *    speed      a step not reached within 6 s
  *    drive      motor fault or supply voltage out of range
  *    few_points fewer than 3 usable steps
  *    low_flow   highest step under 50 L/min (not enough range to fit)
  *    fit        fit rms over max(0.10, 5 % of the largest drop) cmH2O
  *    high       drop at 60 L/min over 3 cmH2O, or a coefficient past the
  *               tube_comp clamps: a kinked / blocked tube or an accessory
  *               the compensation should not hide
  *  The climb stops early (the steps so far are kept) when the pressure
  *  passes 10 cmH2O, the flow 140 L/min or the flow sensor saturates, or the
  *  estimated supply current stays over 0.8 A for 0.5 s.
  *
  *  Warnings (the result is still used):
  *    differs    the learned drop at 60 L/min is off the coefficients in use
  *               before by more than max(0.15 cmH2O, 30 %) - a different
  *               tube from the one the settings assumed
  *    no_tube    drop at 60 L/min under 0.05 cmH2O
  *    flow_zero  flow read more than 2 L/min with the blower at rest
  *
  *  A good result is applied to tube_comp (unless g_circuit_learn_apply is 0)
  *  and every run is appended to 0:/CIRCUIT.CSV. At boot, and when a card is
  *  inserted while therapy is idle, the last applied row is loaded back, so a
  *  learned tube survives a power cycle. `circuit reset` returns to the
  *  built-in coefficients (and logs that, so it survives too).
  *
  *  Compliance is not measured: the tube's ~1 mL/cmH2O is under 10 mL over a
  *  10 cmH2O step, and the flow element's +-0.5 L/min zero integrates to that
  *  much in one second.
  ******************************************************************************
  */
#ifndef CIRCUIT_LEARN_H
#define CIRCUIT_LEARN_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

#define CL_STEPS  5U

typedef enum
{
    CL_PHASE_IDLE = 0,
    CL_PHASE_WAIT,        /* CM4 restart hold-off after a recent stop */
    CL_PHASE_ZERO,
    CL_PHASE_STEP,
    CL_PHASE_RAMPDOWN,
    CL_PHASE_DONE
} CircuitLearnPhase_t;

typedef enum
{
    CL_OK = 0,
    CL_NONE,              /* never run */
    CL_BUSY,              /* therapy not idle / blower running */
    CL_PRECHECK,          /* a sensor / drive self-test check failed */
    CL_NOT_OPEN,
    CL_SPEED,
    CL_DRIVE,
    CL_FEW_POINTS,
    CL_LOW_FLOW,
    CL_FIT,
    CL_HIGH,
    CL_ABORTED,
    CL_COUNT
} CircuitLearnFail_t;

#define CL_WARN_DIFFERS    0x01U
#define CL_WARN_NO_TUBE    0x02U
#define CL_WARN_FLOW_ZERO  0x04U

typedef struct
{
    float rpm;
    float p;              /* cmH2O, minus p0 */
    float q;              /* L/min */
    float supply_a;       /* estimated motor supply current */
} CircuitLearnPoint_t;

typedef struct
{
    CircuitLearnFail_t fail;
    uint8_t  warn;
    bool     applied;
    uint8_t  n;           /* usable steps */
    CircuitLearnPoint_t pt[CL_STEPS];
    float    p0;          /* sensor at rest, cmH2O */
    float    q0;          /* flow at rest, L/min */
    float    r_lam;
    float    r_turb;
    float    d60;         /* fitted drop at 60 / 100 L/min, cmH2O */
    float    d100;
    float    rms;
    float    prev_d60;    /* drop at 60 L/min with the coefficients before */
    char     stop_why[16];/* why the climb ended early ("" = all steps) */
} CircuitLearnResult_t;

extern volatile uint8_t g_circuit_learn_apply;     /* 1 = apply a good result */
extern volatile int32_t g_circuit_learn_rpm_max;   /* top step, <= 26000 */
extern volatile uint8_t g_circuit_learn_restore;   /* 1 = reload from CIRCUIT.CSV */

/* Start a run. Returns CL_OK, or why it was refused (nothing moves then). */
CircuitLearnFail_t CircuitLearn_Start(void);
void  CircuitLearn_Abort(void);
bool  CircuitLearn_IsBusy(void);
/* SensorTask, every loop. */
void  CircuitLearn_Update(uint32_t dt_ms);
/* SD task: CIRCUIT.CSV append and reload. */
void  CircuitLearn_Service(void);
/* Back to the built-in tube coefficients (logged). */
void  CircuitLearn_ResetToDefault(void);

CircuitLearnPhase_t CircuitLearn_GetPhase(void);
uint8_t     CircuitLearn_GetStep(void);       /* 0-based step being run */
uint8_t     CircuitLearn_GetStepCount(void);
const CircuitLearnResult_t* CircuitLearn_GetResult(void);
const char* CircuitLearn_FailName(CircuitLearnFail_t f);   /* screen text */
const char* CircuitLearn_FailToken(CircuitLearnFail_t f);  /* "not_open" */
const char* CircuitLearn_PhaseName(CircuitLearnPhase_t p);
/* Where the coefficients in use came from: "default", "learned", "card". */
const char* CircuitLearn_GetSourceName(void);
uint32_t    CircuitLearn_GetCsvError(void);
uint32_t    CircuitLearn_GetCsvRows(void);    /* rows written this boot */

#ifdef __cplusplus
}
#endif

#endif /* CIRCUIT_LEARN_H */
