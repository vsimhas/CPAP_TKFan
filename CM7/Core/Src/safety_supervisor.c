/**
  ******************************************************************************
  * @file    safety_supervisor.c
  * @brief   Independent safety supervisor task logic.
  ******************************************************************************
  */

#include "safety_supervisor.h"
#include "therapy.h"
#include "blower_ctrl.h"
#include "blower_ipc.h"
#include "dbg_log.h"
#include "stm32h7xx_hal.h"

#include <math.h>

#define SAFETY_SENSOR_MIN_CMH2O     (-10.0f)
#define SAFETY_SENSOR_MAX_CMH2O     ( 50.0f)
#define SAFETY_SENSOR_TIMEOUT_S     ( 2.0f)

/* Blower speed ceiling. Propagates to g_blower_ctrl_rpm_max (see Init and
 * Therapy_Start) and is also the threshold for THERAPY_FAULT_RPM_SATURATED.
 *
 * 2026-09-22: 30000 -> 36000. The blower itself is good for ~42500; the
 * STM32 motor profiler characterisation pinned MAX_APPLICATION_SPEED_RPM at
 * 36012, so 36000 sits just under what CM4 will accept and is the most that
 * can be asked for without re-profiling.
 *
 * Why it matters. The measured steady-state curve on the real circuit is
 * rpm ~ 7239*sqrt(P) + 274, so the deliverable pressure at a given ceiling is
 *
 *      30000 -> 16.9 cmH2O      36000 -> 24.4 cmH2O
 *
 * and the 30000 figure matched the bench exactly: asking for 18 cmH2O gave
 * 16.7 with 38% of samples clipped. Disturbance rejection needs headroom on
 * top of that, which is why CPAP was visibly degrading above 12-13 cmH2O -
 * at 13 the circuit needs ~26.4 kRPM, leaving only 14% of a 30 k ceiling.
 * At 36 k the same point keeps 36%, and 16 cmH2O keeps 23%.
 *
 * UNVERIFIED at this ceiling, check before trusting it in a long run:
 *   - Thermal. Sustained operation near 36 k is more motor power and more
 *     heat. Within the motor's rating, but the duty cycle is not characterised.
 *   - Regenerative braking. Rotor energy goes as speed squared, so a decel
 *     from 36 k dumps 1.44x what 30 k did into a bus that trips OV at 24.9 V.
 *     The mask-off park (therapy speed -> 6000) is the worst case, and its
 *     soft ramp is capped at 2500 ms, i.e. ~12 kRPM/s from 36 k against the
 *     ~4 kRPM/s the brake sizing assumed. Watch brake_active and
 *     bus_voltage_v through a mask-off from 16+ cmH2O.
 * Live override for A/B: safety set safe_rpm_max <rpm>. */
/* 2026-09-23: 36000 -> 40000, matching BLOWER_IPC_MAX_SPEED_RPM and CM4's
 * MAX_APPLICATION_SPEED_RPM. The regenerative-braking concern noted above
 * scales with the square of the speed being dumped, so a mask-off park from
 * 40 k is 1.23x the energy of one from 36 k and 1.78x the 30 k the brake
 * sizing assumed. The bench never actually reached 36 k - the drive is
 * back-EMF limited near 34-36 k - so in practice this raises the ceiling on
 * the command, not on the energy. Still worth watching brake_active and
 * bus_voltage_v through a mask-off from 16+ cmH2O before trusting it. */
/* 2026-09-25: 40000 -> 38500, together with the CM4 speed-loop KI 256 ->
 * 1024 (drive_parameters.h).
 *
 * With the old slow speed loop the rotor never reached 40 k in therapy (it
 * peaked ~38.4 k, ~1 s after the command pinned), so 40000 was only ever a
 * command. With the faster loop the rotor does reach its command, and
 * ceiling_map.py found current control breaking down at 39 krpm under load
 * (id sd 226 mA, |id| 630 mA, then a DP_FAULT at ~39.5 k in the step test).
 * 38500 is the highest speed that held clean for 4 s (id sd 108 mA).
 *
 * Bench, 20 cmH2O, human breathing, mask-end sensor, matched peak flow:
 * dip 2.53 -> 2.10 cmH2O (fitted at 66 LPM), end-insp rise unchanged, rotor
 * within ~20 rpm of 38500 at peak flow, no drive stops or faults.
 *
 * Mask-off rpm-ratio path: the ceiling ratio at 20 cmH2O drops from 1.33 to
 * 38500 / 30078 = 1.28 against its 1.25 threshold - less margin, still
 * reached (a 38500-ceiling pull at 20 was caught by rpm-ratio at 2.5 s). */
volatile int32_t  g_safety_safe_rpm_max           = 38500;
volatile float    g_safety_low_press_cmh2o        = 1.5f;
volatile int32_t  g_safety_high_rpm_threshold     = 12000;
volatile float    g_safety_low_press_timeout_s    = 4.0f;
volatile float    g_safety_overpress_band_cmh2o   = 5.0f;
volatile float    g_safety_overpress_timeout_s    = 3.0f;
volatile float    g_safety_saturate_band_cmh2o    = 1.5f;
volatile float    g_safety_saturated_timeout_s     = 6.0f;
volatile uint8_t  g_safety_enabled                = 1U;
volatile float    g_safety_grace_s                = 10.0f;

volatile uint32_t g_safety_cm4_heartbeat_timeout_ms = 100U;
volatile uint32_t g_safety_cm4_last_seq           = 0U;
volatile uint32_t g_safety_cm4_stale_ms           = 0U;

static uint32_t s_low_press_ms   = 0U;
static uint32_t s_overpress_ms   = 0U;
static uint32_t s_outrange_ms    = 0U;
static uint32_t s_saturated_ms   = 0U;
static uint32_t s_cm4_seen_seq   = 0U;
static uint32_t s_cm4_last_change_ms = 0U;
static uint8_t  s_cm4_seq_valid  = 0U;

static void resetFaultTimers(void)
{
    s_low_press_ms  = 0U;
    s_overpress_ms  = 0U;
    s_outrange_ms   = 0U;
    s_saturated_ms  = 0U;
}

static bool therapyNeedsCm4Heartbeat(void)
{
    const TherapyState_t st = Therapy_GetState();
    return (st == THERAPY_STATE_RAMP)
        || (st == THERAPY_STATE_RUNNING)
        || (st == THERAPY_STATE_STOPPING)
        || (st == THERAPY_STATE_PRECHECK);   /* #83 spin check */
}

static void updateCm4Heartbeat(uint32_t dt_ms)
{
    uint32_t seq;

    __DMB();
    seq = g_blower_ipc.cm4_status_seq;
    __DMB();

    g_safety_cm4_last_seq = seq;

    if (!s_cm4_seq_valid || (seq != s_cm4_seen_seq))
    {
        s_cm4_seen_seq = seq;
        s_cm4_last_change_ms = HAL_GetTick();
        s_cm4_seq_valid = 1U;
        g_safety_cm4_stale_ms = 0U;
        return;
    }

    if (!therapyNeedsCm4Heartbeat())
    {
        g_safety_cm4_stale_ms = 0U;
        return;
    }

    const uint32_t now = HAL_GetTick();
    g_safety_cm4_stale_ms = now - s_cm4_last_change_ms;

    const uint32_t limit = (g_safety_cm4_heartbeat_timeout_ms > 0U)
                               ? g_safety_cm4_heartbeat_timeout_ms
                               : 1U;
    if (g_safety_cm4_stale_ms >= limit)
    {
        Therapy_LatchFault(THERAPY_FAULT_CM4_HEARTBEAT,
                           "CM4 status heartbeat stale");
    }

    (void)dt_ms;
}

static bool runPressureChecks(uint32_t dt_ms)
{
    const float meas   = Therapy_GetMeasuredCmh2o();
    const float target = Therapy_GetTargetPressure();
    /* #8: a setting change reaches the blower at 0.5 cmH2O/s, so judge
     * against what is commanded now. Over-pressure: the higher of the two,
     * so a slow decrease is not a fault (and an increase still is judged on
     * the target). Saturation: what the loop is actually aiming at. */
    const float commanded = Therapy_GetCommandedPressureCmh2o();
    const float over_ref  = (commanded > target) ? commanded : target;
    const int32_t rpm  = (int32_t)BlowerCtrl_GetOutputRpm();
    const bool grace   = Therapy_SafetyGraceActive();

    if ((meas < SAFETY_SENSOR_MIN_CMH2O) || (meas > SAFETY_SENSOR_MAX_CMH2O))
    {
        s_outrange_ms += dt_ms;
        if (s_outrange_ms >= (uint32_t)(SAFETY_SENSOR_TIMEOUT_S * 1000.0f))
        {
            Therapy_LatchFault(THERAPY_FAULT_SENSOR_OUT_OF_RANGE,
                               "pressure reading outside plausible range");
            return true;
        }
    }
    else
    {
        s_outrange_ms = 0U;
    }

    if (!grace)
    {
        const float p_sensor = BlowerCtrl_GetSensorCmh2o();
        /* Mask-off intentionally holds ~6 kRPM with near-zero pressure —
         * do not treat that as a hose/sensor fault. */
        if (!Therapy_IsMaskOffActive()
            && !Therapy_IsMaskOffArming()
            && (rpm > g_safety_high_rpm_threshold)
            && (p_sensor < g_safety_low_press_cmh2o))
        {
            s_low_press_ms += dt_ms;
            if (s_low_press_ms
                  >= (uint32_t)(g_safety_low_press_timeout_s * 1000.0f))
            {
                Therapy_LatchFault(THERAPY_FAULT_LOW_PRESS_AT_HIGH_RPM,
                                   "low pressure while RPM high (sensor/hose lost?)");
                return true;
            }
        }
        else
        {
            s_low_press_ms = 0U;
        }
    }
    else
    {
        s_low_press_ms = 0U;
    }

    if (!grace && (meas > (over_ref + g_safety_overpress_band_cmh2o)))
    {
        s_overpress_ms += dt_ms;
        if (s_overpress_ms >= (uint32_t)(g_safety_overpress_timeout_s * 1000.0f))
        {
            Therapy_LatchFault(THERAPY_FAULT_OVER_PRESSURE,
                               "measured pressure exceeds target+band");
            return true;
        }
    }
    else
    {
        s_overpress_ms = 0U;
    }

    /* Mask-off parks the blower at hold RPM with the PID disabled, so the
     * output register read above stops being refreshed and keeps whatever it
     * saturated to while the circuit was open (normally the safe max). Judging
     * saturation on that stale value latched an RPM_SATURATED fault a few
     * seconds into every mask-off, which stopped the motor and left nothing to
     * resume when the mask went back on. Same reasoning as the low-pressure
     * check above.
     *
     * Also held off while an enter path is arming. This timer starts as soon
     * as the PID pins at max with pressure short of target, which a large leak
     * alone will do - well before a full disconnect puts the pressure on the
     * floor and gives mask-off something to detect. Without this, the fault
     * can arrive with a several-second head start and stop the motor while the
     * detector is still counting, which is what it did on the bench. A genuine
     * "cannot reach target" leak never arms mask-off, so it still latches. */
    if (!grace && !Therapy_IsMaskOffActive() && !Therapy_IsMaskOffArming())
    {
        const bool at_safe_max = (rpm >= g_safety_safe_rpm_max);
        const bool not_close   = (fabsf(commanded - meas) > g_safety_saturate_band_cmh2o);
        if (at_safe_max && not_close)
        {
            s_saturated_ms += dt_ms;
            if (s_saturated_ms >= (uint32_t)(g_safety_saturated_timeout_s * 1000.0f))
            {
                Therapy_LatchFault(THERAPY_FAULT_RPM_SATURATED,
                                   "RPM saturated without reaching target");
                return true;
            }
        }
        else
        {
            s_saturated_ms = 0U;
        }
    }
    else
    {
        s_saturated_ms = 0U;
    }

    return false;
}

void Safety_Supervisor_Init(void)
{
    resetFaultTimers();
    s_cm4_seq_valid = 0U;
    s_cm4_seen_seq = 0U;
    s_cm4_last_change_ms = HAL_GetTick();
    g_safety_cm4_stale_ms = 0U;

    extern volatile int32_t g_blower_ctrl_rpm_max;
    g_blower_ctrl_rpm_max = g_safety_safe_rpm_max;
}

void Safety_Supervisor_ResetFaultTimers(void)
{
    resetFaultTimers();
}

void Safety_Supervisor_Update(float dt_s)
{
    if (!(dt_s > 0.0f) || (dt_s > 0.5f))
    {
        return;
    }

    const uint32_t dt_ms = (uint32_t)lrintf(dt_s * 1000.0f);

    updateCm4Heartbeat(dt_ms);

    if (g_safety_enabled == 0U)
    {
        return;
    }

    if (Therapy_GetFault() != THERAPY_FAULT_NONE)
    {
        return;
    }

    const TherapyState_t st = Therapy_GetState();
    if ((st != THERAPY_STATE_RAMP) && (st != THERAPY_STATE_RUNNING))
    {
        resetFaultTimers();
        return;
    }

    (void)runPressureChecks(dt_ms);
}
