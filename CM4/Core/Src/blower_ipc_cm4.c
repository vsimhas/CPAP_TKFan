/**
  * @file    blower_ipc_cm4.c
  * @brief   CM4-side blower IPC command handling and regen brake control.
  */

#include "blower_ipc.h"
#include "main.h"
#include "mc_api.h"
#include "mc_config_common.h"
#include "mc_stm_types.h"
#include "mc_interface.h"
#include "bus_voltage_sensor.h"
#include "mc_config.h"
#include "pid_regulator.h"
#include "parameters_conversion.h"   /* CURRENT_CONV_FACTOR_INV */

extern TIM_HandleTypeDef htim3;

#define BRAKE_PWM_FREQ_HZ           10000U
#define BRAKE_TIM_CLK_HZ            200000000UL
/* FOC OV trips at OV_VOLTAGE_THRESHOLD_V (28.0 V since 2026-09-25; 24.9 V
 * when the text below was written). Engage brake early so a mask-off /
 * therapy soft decel does not kill PWM and coast to 0.
 *
 * 2026-09-24: ON 22 -> 26 to DISABLE the brake for now. These thresholds were
 * set against the mis-scaled bus reading, back when a real 23.9 V bus was
 * reported as 20-21 V, so 22 was comfortably above anything ever seen and the
 * brake in practice never armed. Correcting VBUS_PARTITIONING_FACTOR on
 * 2026-09-24 made the same bus report 23, which is >= 22, so the first
 * deceleration latched the brake and it then never released - release needs
 * vbus <= BRAKE_VBUS_OFF_V, and a 23.9 V supply never goes there. Measured
 * result: 250 permille continuous into R154, about 5.7 W of standing heat in
 * a 25 W resistor. Not damaging, but wrong, and it was found by the resistor
 * getting hot rather than by anything in firmware noticing.
 *
 * 26 V is above the supply, so the brake cannot arm. That restores the
 * behaviour the machine has actually had all along.
 *
 * TO RE-ENABLE PROPERLY, two things are needed and neither is this line:
 *   1. Whole-volt resolution cannot express a usable window. The bus sits at
 *      23.9 V and OV trips at 24.9 V, so the entire brake range is 1 V and
 *      VBS_GetAvBusVoltage_V() can only say 24 or 25 inside it. Use
 *      VBS_GetAvBusVoltage_d() (~0.8 mV/count, already used for
 *      g_blower_ipc.bus_voltage_mv) and thresholds in mV.
 *   2. The window has to be widened, which means knowing the DC-link
 *      capacitor rating and raising OV_VOLTAGE_THRESHOLD_V to suit. Sizing a
 *      brake against a trip point that may move is wasted work.
 *
 * Note the brake is what stops a fast decel tripping OV, so while it is
 * disabled, watch for Over Voltage on mask-off parks from high pressure.
 *
 * 2026-09-25: OV raised 24.9 -> 28.0 V (drive_parameters.h), which also
 * answers point 2 above. ON moves 26 -> 29 with it so the brake STAYS
 * disabled: at 26 it would now start switching between 26 and 28 V, i.e.
 * a silent re-enable. Re-enabling is a separate decision (R154 rating,
 * whole-volt resolution - point 1). */
#define BRAKE_VBUS_ON_V             29U
#define BRAKE_VBUS_OFF_V            21U
#define BRAKE_DECEL_RPM_PER_MS      8
#define BRAKE_MAX_PERMILLE            900U

static uint32_t s_last_cmd_seq;
static uint32_t s_last_foc_tuning_seq;
static int32_t s_prev_speed_rpm;
static bool s_brake_latched;

static int16_t BlowerIpc_RpmToSpeedUnit(int32_t rpm)
{
  /* Clamp to the drive's limit first: STC_ExecRamp REJECTS a target above
   * MAX_APPLICATION_SPEED_RPM (the ramp is not run at all) rather than
   * saturating it, so an over-range command from CM7 would leave the rotor at
   * its previous speed. TKFan: 36000 rpm (drive_parameters.h). */
  if (rpm >  (int32_t)MAX_APPLICATION_SPEED_RPM) { rpm =  (int32_t)MAX_APPLICATION_SPEED_RPM; }
  if (rpm < -(int32_t)MAX_APPLICATION_SPEED_RPM) { rpm = -(int32_t)MAX_APPLICATION_SPEED_RPM; }
  /* SPEED_UNIT/U_RPM is typically 10/60 = 1/6, so even at 36528 RPM
   * (workbench app limit) the result is ~6088, which fits comfortably in
   * an int16. We still do the math in int32 to avoid intermediate overflow. */
  return (int16_t)((rpm * (int32_t)SPEED_UNIT) / (int32_t)U_RPM);
}

static int32_t BlowerIpc_SpeedUnitToRpm(int16_t speed_unit)
{
  return ((int32_t)speed_unit * (int32_t)U_RPM) / (int32_t)SPEED_UNIT;
}

/* === Restart-while-coasting guard ========================================
 *
 * Never start the motor while the rotor may still be coasting.
 *
 * The sensorless start (charge boot cap -> offset calib -> alignment ->
 * open-loop rev-up -> switch-over) assumes a rotor at rest. Measured
 * 2026-09-25 on the bench:
 *   - therapy restarted 3.1 s after a stop from 18 krpm, rotor still at
 *     roughly 2 krpm: the drive went to RUN and reported mech tracking the
 *     command up to 32 krpm, but iq ~0 and pressure/flow ~0 - it was not
 *     driving the fan at all, and nothing faulted;
 *   - an OVER_VOLT trip at ~32 krpm switched PWM off; SET_SPEED kept
 *     arriving and each one acknowledged and restarted into the spinning
 *     rotor: six DP_FAULTs in ~1 s, then a start drawing 2.66 A.
 *
 * Once PWM is off the CM4 cannot see the rotor - the angle/speed estimator
 * is disabled in STOP/IDLE/FAULT and its output freezes - so the guard is
 * time-based. When drive stops (leaving RUN/START/SWITCH_OVER/ALIGNMENT, by
 * STOP or by a fault) it records the last driven speed and holds any start
 * off for BASE + PER_KRPM x speed.
 *
 * Sizing, from the sensor-pressure decay after therapy stops (pressure
 * tracks rotor speed, closed circuit with vent): the rotor is below ~1.5
 * krpm 3.3-3.7 s after a stop from 18 k, 3.8-4.3 s from 23 k, 4.2-5.2 s from
 * 33 k. Below that pressure cannot follow it, so the hold-off leaves a wide
 * margin for the slow bearing-friction tail: 6.7 s from 18 k (the failed
 * restart was at 3.1 s), 9.0 s from 33 k, 10 s from 40 k. Restarts that
 * waited 10-12 s all started cleanly (18 starts on 2026-09-25).
 *
 * A start that arrives inside the hold-off is not dropped: it is kept
 * pending and issued from the main loop once the hold-off has run out and
 * the state machine is in IDLE. STOP cancels it. The remaining hold-off is
 * published as restart_holdoff_ms so CM7 can hold its low-pressure grace
 * and so it shows on the B line. */
#define RESTART_HOLDOFF_BASE_MS      4000U
#define RESTART_HOLDOFF_MS_PER_KRPM   150U
/* Below this the rotor was never really spun up (alignment, a failed start
 * at crawl speed): nothing to wait for. */
#define RESTART_HOLDOFF_MIN_RPM      1000U

static bool     s_drive_on;          /* PWM was driving the rotor           */
static uint32_t s_drive_rpm;         /* |mech| while it was                 */
static uint32_t s_coast_start_ms;    /* when drive stopped                  */
static uint32_t s_coast_holdoff_ms;  /* hold-off length; 0 = none           */
static bool     s_start_pending;     /* start requested during the hold-off */
/* STOP accepted but the state machine has not left the driven states yet
 * (it acts on the next medium-frequency tick). A START in that window must
 * survive the stop, not be mistaken for "already running". */
static bool     s_stop_in_flight;

static bool BlowerIpc_StateDrives(MCI_State_t st)
{
  return (st == RUN) || (st == START) || (st == SWITCH_OVER) ||
         (st == ALIGNMENT);
}

/* Main-loop context only: it is the sole writer of the coast state. */
static void BlowerIpc_CM4_TrackCoast(void)
{
  const MCI_State_t st = MC_GetSTMStateMotor1();
  if (BlowerIpc_StateDrives(st))
  {
    int32_t mech = BlowerIpc_SpeedUnitToRpm(MC_GetMecSpeedAverageMotor1());
    if (mech < 0)
    {
      mech = -mech;
    }
    s_drive_rpm = (uint32_t)mech;
    s_drive_on = true;
    s_coast_holdoff_ms = 0U;
  }
  else if (s_drive_on)
  {
    /* Drive just stopped. The estimate froze at the last driven speed,
     * which is what the rotor is coasting down from. */
    s_drive_on = false;
    s_stop_in_flight = false;
    s_coast_start_ms = HAL_GetTick();
    s_coast_holdoff_ms = (s_drive_rpm < RESTART_HOLDOFF_MIN_RPM)
        ? 0U
        : RESTART_HOLDOFF_BASE_MS
          + ((s_drive_rpm * RESTART_HOLDOFF_MS_PER_KRPM) / 1000U);
  }
}

/* Pure read, safe from the medium-frequency hook as well. */
static uint32_t BlowerIpc_CM4_HoldoffRemainingMs(void)
{
  const uint32_t holdoff = s_coast_holdoff_ms;
  if (holdoff == 0U)
  {
    return 0U;
  }
  const uint32_t elapsed = HAL_GetTick() - s_coast_start_ms;
  return (elapsed >= holdoff) ? 0U : (holdoff - elapsed);
}

/* Replaces the direct MC_StartMotor1() calls in START and SET_SPEED. */
static void BlowerIpc_CM4_RequestStart(void)
{
  HAL_GPIO_WritePin(STSPIN_STBY_GPIO_Port, STSPIN_STBY_Pin, GPIO_PIN_SET);
  HAL_GPIO_WritePin(MOT_POW_EN_GPIO_Port, MOT_POW_EN_Pin, GPIO_PIN_SET);
  s_start_pending = true;
}

/* Issues a pending start once the rotor has had time to stop. Called every
 * main-loop pass. MCI_StartMotor only accepts IDLE, so acknowledge a latched
 * fault first and start on a later pass - the ack takes effect on the next
 * state-machine tick, and a start queued behind it would be refused. */
static void BlowerIpc_CM4_ServicePendingStart(void)
{
  if (!s_start_pending)
  {
    return;
  }
  const MCI_State_t st = MC_GetSTMStateMotor1();
  if (BlowerIpc_StateDrives(st))
  {
    if (!s_stop_in_flight)
    {
      s_start_pending = false;   /* already running (e.g. Motor Pilot) */
    }
    /* else: a STOP is about to take effect; keep the start for after it. */
    return;
  }
  if (BlowerIpc_CM4_HoldoffRemainingMs() > 0U)
  {
    return;
  }
  if (st == FAULT_OVER)
  {
    (void)MC_AcknowledgeFaultMotor1();
    return;
  }
  if (st == IDLE)
  {
    s_start_pending = false;
    (void)MC_StartMotor1();
  }
  /* STOP / WAIT_STOP_MOTOR / FAULT_NOW: settling, try again next pass. */
}

static void BlowerBrake_ApplyPermille(uint16_t permille)
{
  uint32_t period = __HAL_TIM_GET_AUTORELOAD(&htim3);
  uint32_t pulse;

  if (permille > 1000U)
  {
    permille = 1000U;
  }

  pulse = ((uint32_t)permille * (period + 1U)) / 1000U;
  __HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, pulse);
  g_blower_ipc.brake_pwm_permille = permille;
  g_blower_ipc.brake_active = (permille > 0U) ? 1U : 0U;
}

void BlowerBrake_Init(void)
{
  TIM_OC_InitTypeDef sConfigOC = {0};
  uint32_t prescaler;
  uint32_t period;

  prescaler = (BRAKE_TIM_CLK_HZ / (BRAKE_PWM_FREQ_HZ * 1000UL)) - 1U;
  if (prescaler == 0xFFFFFFFFUL)
  {
    prescaler = 0U;
  }
  period = 999U;

  htim3.Init.Prescaler = (uint16_t)prescaler;
  htim3.Init.Period = period;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;

  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }

  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0U;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }

  HAL_TIM_MspPostInit(&htim3);
  (void)HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_1);
  BlowerBrake_ApplyPermille(0U);
  s_prev_speed_rpm = 0;
  s_brake_latched = false;
}

void BlowerBrake_Update(void)
{
  uint16_t vbus_v = VBS_GetAvBusVoltage_V(&BusVoltageSensor_M1._Super);
  int32_t curr_rpm = BlowerIpc_SpeedUnitToRpm(MC_GetMecSpeedAverageMotor1());
  int32_t decel_rpm = (int32_t)s_prev_speed_rpm - curr_rpm;
  uint16_t brake_permille = 0U;

  g_blower_ipc.bus_voltage_v = vbus_v;

  if ((vbus_v >= BRAKE_VBUS_ON_V) &&
      ((decel_rpm >= BRAKE_DECEL_RPM_PER_MS) || s_brake_latched))
  {
    uint32_t over_v = (uint32_t)vbus_v - (uint32_t)BRAKE_VBUS_ON_V;

    s_brake_latched = true;
    brake_permille = (uint16_t)(over_v * 250U);
    if (brake_permille > BRAKE_MAX_PERMILLE)
    {
      brake_permille = BRAKE_MAX_PERMILLE;
    }
  }

  if (vbus_v <= BRAKE_VBUS_OFF_V)
  {
    s_brake_latched = false;
    brake_permille = 0U;
  }

  BlowerBrake_ApplyPermille(brake_permille);
  s_prev_speed_rpm = curr_rpm;
}

/* Runtime speed-PI current limit ("foc set iq_max_ma"). The upper bound is
 * a hard cap independent of the mailbox: the STSPIN830 trips at 3.0 A min.
 * On the WM6850 (~70 uH) PWM ripple put peaks ~0.7-1.0 A above the
 * controlled current; the TKFan's 0.21 mH gives about a third of that ripple,
 * but its 1.075 ohm phases dissipate ~4x the copper loss at a given current
 * (see IQMAX_A in drive_parameters.h). */
#define IQ_MAX_RUNTIME_MIN_MA  500U
#define IQ_MAX_RUNTIME_MAX_MA  1800U

/* Speed-dependent limit, 2026-10-01 (see foc_iq_boost_* in blower_ipc.h).
 * In RUN the limit is iq_boost_ma up to iq_boost_lo_rpm and tapers linearly
 * to iq_max_ma at iq_boost_hi_rpm; everywhere else it is iq_max_ma.
 *
 * Measured with a flat limit while breathing: 1650 mA cut the onset dip at
 * 16 cmH2O from 1.34 to 0.87 cmH2O with no faults (rotor <= ~36 krpm, Id
 * <= 0.3 A), but at 20 cmH2O, where the rotor spends ~17% of the time above
 * 37 krpm, Id scattered 1-2 A and one breath tripped DP_FAULT at 38.2 krpm.
 * Above ~37 krpm the three-shunt sampling window is short and current
 * control degrades, so the extra current is kept out of that region. A lower
 * speed ceiling (37000) with 1650 mA flat was worse at 20 cmH2O (dip 1.49
 * -> 2.63 cmH2O): there the blower needs the speed more than the current.
 *
 * Boot defaults; "foc set iq_boost_ma 0" turns the boost off.
 *
 * 2026-10-02: boost 1650 -> 1800 mA (the runtime ceiling, IQ_MAX_RUNTIME_MAX_MA),
 * on the sealed air path with the breathing-tuned CM7 set. Digital twin on
 * identical recorded 20 cmH2O breaths: inspiratory dip median -0.14..-0.27,
 * p90 ~-0.35 cmH2O, p-p ~-0.3, end-inspiration spike +0.03. No DP_FAULT in:
 * CM7 playback of a recorded 20 cmH2O breathing command on the leak rig
 * (~80 L/min, limit reached 1800, |Id| <= 0.24 A), closed-loop CPAP 20<->18
 * cmH2O, and ~55 s of human breathing at 20 cmH2O. Margin NOT measured: with
 * PWM ripple the phase peaks at 1.8 A are estimated ~2.5-2.8 A against the
 * STSPIN830's 3.0 A minimum trip - scope the phase current before going
 * further, and fall back with "foc set iq_boost_ma 1650" if DP_FAULT appears. */
/* TKFan, 2026-10-22: boost ON by default at 1800 mA, full up to 30 krpm and
 * tapering off by 31.5 krpm. Bench, vent-only closed circuit, open loop, true speed steps
 * (step_tk.py), against 1314 mA with no boost:
 *   10 -> 20 cmH2O (19.4 -> 27.4k): 20.0 -> 30.0 krpm/s, speed t90 0.44 ->
 *     0.32 s, pressure t90 0.25 -> 0.20 s - level with the WM6850 on its own
 *     1800 mA boost (30.4 krpm/s). Peak iq 1698 mA.
 *   4 -> 20 cmH2O (12.2 -> 27.4k): only 22.8 -> 25.3 krpm/s, because the
 *     boost then ramped in at 18-20k (the WM6850 values) - see below.
 *   Cost: ~+100 rpm / +0.25 cmH2O more step overshoot (more speed-PI
 *   integral at the higher limit). No faults; Vbus flat (23.64-23.68 V).
 * The WM6850's limits, translated:
 *   - Taper: the WM6850 kept the boost out above ~35.5-37k because current
 *     control degraded there (three-shunt sampling window shrinks as the
 *     modulation rises; Id scattered 1-2 A and one DP_FAULT at 38.2k). That
 *     is ~85-92 % of Vbus/sqrt3. The TKFan reaches the same modulation much
 *     lower - measured |Vqd| under load: 74 % at 26k, 88 % at 30k (outlet
 *     open, 1.0 A), 87 % at 32k (leak open), 83 % at 32k vent-only - so its
 *     taper sits at 30-31.5k. (A first version of this change put the taper
 *     above the speed cap, from a misread of the vq_mv telemetry field as a
 *     fraction of 32767; it is millivolts.)
 *   - Over-current: its 0.21 mH (3x the WM6850) cuts PWM ripple to about a
 *     third, so 1.8 A puts phase peaks near ~2.1-2.2 A against the
 *     STSPIN830's 3.0 A minimum trip (WM6850: ~2.5-2.8 A estimated).
 * Watch: copper loss is ~5.2 W at 1.8 A (1.5 x 1.8^2 x 1.07 ohm) vs 2.8 W at
 * 1.314 A. It is reached only in brief acceleration bursts, but check motor
 * temperature in a long 20 cmH2O breathing run, and watch for DP_FAULT.
 * Runtime override: "foc set iq_boost_ma 0" (off) or another value. */
#define IQ_BOOST_DEFAULT_MA       1800U
#define IQ_BOOST_DEFAULT_LO_RPM   30000U
#define IQ_BOOST_DEFAULT_HI_RPM   31500U
/* No boost until the rotor is clear of the sensorless hand-over. TKFan: the
 * observer takes over at 11804 rpm (OBS_MINIMUM_SPEED_RPM), so the boost now
 * ramps in over 14-15 krpm (WM6850: 18-20 krpm, above its 14.4k hand-over,
 * where full current used to trip DP_FAULT on start). Starts themselves run
 * at IQMAX as before; 4 cmH2O sits at ~12.2k, below the ramp-in. */
#define IQ_BOOST_ON_START_RPM     14000
#define IQ_BOOST_ON_FULL_RPM      15000

/* TKFan high-speed current safeguard, 2026-10-22. Above 33k the speed-PI
 * current limit is stepped down - whatever the base/boost settings, including
 * runtime "foc set" overrides - from the 1314 mA base at 33k to 1100 mA at
 * 35k and above, so over-modulation is only entered at moderate current.
 * Why: with the outlet wide open and the boost lifted for a test, the TKFan
 * held 34k at 107 % modulation (1.39 A), then tripped DP_FAULT (STSPIN830
 * OCP, 3.0 A min) at 34.1k / 108 % with only ~1.45-1.5 A sampled - in
 * over-modulation the phase current is distorted and the three-shunt window
 * shrinks, so true peaks run far above the sampled value. The WM6850 failed
 * the same way at 38.2k. The CPAP case that needs these speeds (full flow at
 * high altitude) runs at moderate current: 36k on a ~115 L/min load took
 * 947 mA at 99 % modulation, cleanly. 1100 mA keeps ~150 mA of headroom there
 * and ~25-30 % below the observed trip. Over-modulation at 1.0-1.3 A remains
 * untested on the bench. */
#define IQ_HISPEED_DERATE_START_RPM   33000
#define IQ_HISPEED_DERATE_FULL_RPM    35000
#define IQ_HISPEED_DERATE_MA          1100U

/* Written by the main loop (BlowerIpc_CM4_LatchIqLimit), read by the
 * medium-frequency hook. Each is one aligned word, so a change caught
 * mid-update mixes old and new values for at most one speed-loop tick,
 * always inside the hard clamp above. */
static volatile int16_t s_iq_base_lim;     /* speed-PI units; 0 = not latched */
static volatile int16_t s_iq_boost_lim;
static volatile int32_t s_iq_boost_lo_rpm;
static volatile int32_t s_iq_boost_hi_rpm;
static int16_t s_iq_applied_lim;           /* MF hook only */
static uint16_t s_iq_base_ma;              /* main loop only */

static uint16_t BlowerIpc_ClampIqMa(uint16_t ma)
{
  if (ma < IQ_MAX_RUNTIME_MIN_MA) { return IQ_MAX_RUNTIME_MIN_MA; }
  if (ma > IQ_MAX_RUNTIME_MAX_MA) { return IQ_MAX_RUNTIME_MAX_MA; }
  return ma;
}

static int16_t BlowerIpc_IqMaToUnits(uint16_t ma)
{
  return (int16_t)((float)ma * (float)CURRENT_CONV_FACTOR / 1000.0f);
}

/* Main loop: take the mailbox settings, clamp them, write back what is used. */
static void BlowerIpc_CM4_LatchIqLimit(void)
{
  /* iq_max 0 = leave as is (the boot default if nothing latched yet). */
  if (g_blower_ipc.foc_iq_max_ma != 0U)
  {
    s_iq_base_ma = BlowerIpc_ClampIqMa(g_blower_ipc.foc_iq_max_ma);
  }
  else if (s_iq_base_ma == 0U)
  {
    s_iq_base_ma = BlowerIpc_ClampIqMa((uint16_t)(IQMAX_A * 1000.0 + 0.5));
  }
  s_iq_base_lim = BlowerIpc_IqMaToUnits(s_iq_base_ma);
  g_blower_ipc.foc_iq_max_ma = s_iq_base_ma;

  uint16_t boost = g_blower_ipc.foc_iq_boost_ma;
  if (boost != 0U)
  {
    boost = BlowerIpc_ClampIqMa(boost);
  }
  s_iq_boost_lim = (boost != 0U) ? BlowerIpc_IqMaToUnits(boost) : 0;
  g_blower_ipc.foc_iq_boost_ma = boost;

  s_iq_boost_lo_rpm = (int32_t)g_blower_ipc.foc_iq_boost_lo_rpm;
  s_iq_boost_hi_rpm = (int32_t)g_blower_ipc.foc_iq_boost_hi_rpm;
}

/* 0 at or below lo, 1 at or above hi, linear between (a step at hi if
 * hi <= lo). */
static float BlowerIpc_Ramp01(int32_t x, int32_t lo, int32_t hi)
{
  if (x >= hi) { return 1.0f; }
  if (x <= lo) { return 0.0f; }
  return (float)(x - lo) / (float)(hi - lo);
}

/* Medium-frequency hook, right after the speed PI: set the limit for the
 * next tick from the measured speed. The PI clamps its integral term to the
 * new limit on its next run, so lowering the limit cannot leave it wound up
 * above it. */
void BlowerIpc_CM4_UpdateIqLimit(void)
{
  const int16_t base = s_iq_base_lim;
  if (base <= 0)
  {
    return;   /* MF tasks run before BlowerIpc_CM4_Init latches the limits */
  }

  int16_t lim = base;
  int32_t rpm = BlowerIpc_SpeedUnitToRpm(MC_GetMecSpeedAverageMotor1());
  if (rpm < 0) { rpm = -rpm; }
  const int32_t extra = (int32_t)s_iq_boost_lim - (int32_t)base;
  if ((extra > 0) && (MC_GetSTMStateMotor1() == RUN))
  {
    const float on  = BlowerIpc_Ramp01(rpm, IQ_BOOST_ON_START_RPM, IQ_BOOST_ON_FULL_RPM);
    const float off = 1.0f - BlowerIpc_Ramp01(rpm, s_iq_boost_lo_rpm, s_iq_boost_hi_rpm);
    const float k = (on < off) ? on : off;
    lim = (int16_t)((int32_t)base + (int32_t)(k * (float)extra));
  }

  /* High-speed safeguard (see IQ_HISPEED_DERATE_*): cap the limit, never
   * raise it, so a lower runtime iq_max still wins. */
  {
    const float k_unit = (float)CURRENT_CONV_FACTOR / 1000.0f;   /* units per mA */
    const float base_ma = (float)base / k_unit;
    const float floor_ma = (float)IQ_HISPEED_DERATE_MA;
    const float k = BlowerIpc_Ramp01(rpm, IQ_HISPEED_DERATE_START_RPM, IQ_HISPEED_DERATE_FULL_RPM);
    if ((k > 0.0f) && (base_ma > floor_ma))
    {
      const int16_t cap = (int16_t)((base_ma - k * (base_ma - floor_ma)) * k_unit);
      if (lim > cap) { lim = cap; }
    }
  }

  if (lim != s_iq_applied_lim)
  {
    PID_SetUpperOutputLimit(&PIDSpeedHandle_M1, lim);
    PID_SetLowerOutputLimit(&PIDSpeedHandle_M1, (int16_t)-lim);
    PID_SetUpperIntegralTermLimit(&PIDSpeedHandle_M1, (int32_t)lim * (int32_t)SP_KIDIV);
    PID_SetLowerIntegralTermLimit(&PIDSpeedHandle_M1, -((int32_t)lim * (int32_t)SP_KIDIV));
    s_iq_applied_lim = lim;
    g_blower_ipc.iq_lim_ma =
        (uint16_t)((float)lim * (float)CURRENT_CONV_FACTOR_INV * 1000.0f + 0.5f);
  }
}

static void BlowerIpc_CM4_SeedFocTuning(void)
{
  g_blower_ipc.foc_speed_kp = PIDSpeedHandle_M1.hKpGain;
  g_blower_ipc.foc_speed_ki = PIDSpeedHandle_M1.hKiGain;
  g_blower_ipc.foc_torque_kp = PIDIqHandle_M1.hKpGain;
  g_blower_ipc.foc_torque_ki = PIDIqHandle_M1.hKiGain;
  g_blower_ipc.foc_flux_kp = PIDIdHandle_M1.hKpGain;
  g_blower_ipc.foc_flux_ki = PIDIdHandle_M1.hKiGain;
  g_blower_ipc.foc_iq_max_ma = (uint16_t)(IQMAX_A * 1000.0 + 0.5);
  g_blower_ipc.foc_iq_boost_ma = IQ_BOOST_DEFAULT_MA;
  g_blower_ipc.foc_iq_boost_lo_rpm = IQ_BOOST_DEFAULT_LO_RPM;
  g_blower_ipc.foc_iq_boost_hi_rpm = IQ_BOOST_DEFAULT_HI_RPM;
  BlowerIpc_CM4_LatchIqLimit();
  __DMB();
  s_last_foc_tuning_seq = g_blower_ipc.cm7_foc_tuning_seq;
  g_blower_ipc.cm4_foc_tuning_ack_seq = s_last_foc_tuning_seq;
  __DMB();
}

static int16_t BlowerIpc_PickFocGain(int16_t mailbox, int16_t current)
{
  return (mailbox > 0) ? mailbox : current;
}

static void BlowerIpc_CM4_ApplyFocTuning(void)
{
  uint32_t seq = g_blower_ipc.cm7_foc_tuning_seq;
  if (seq == s_last_foc_tuning_seq)
  {
    return;
  }

  __DMB();
  PID_SetKP(&PIDSpeedHandle_M1,
            BlowerIpc_PickFocGain(g_blower_ipc.foc_speed_kp, PIDSpeedHandle_M1.hKpGain));
  PID_SetKI(&PIDSpeedHandle_M1,
            BlowerIpc_PickFocGain(g_blower_ipc.foc_speed_ki, PIDSpeedHandle_M1.hKiGain));
  PID_SetKP(&PIDIqHandle_M1,
            BlowerIpc_PickFocGain(g_blower_ipc.foc_torque_kp, PIDIqHandle_M1.hKpGain));
  PID_SetKI(&PIDIqHandle_M1,
            BlowerIpc_PickFocGain(g_blower_ipc.foc_torque_ki, PIDIqHandle_M1.hKiGain));
  PID_SetKP(&PIDIdHandle_M1,
            BlowerIpc_PickFocGain(g_blower_ipc.foc_flux_kp, PIDIdHandle_M1.hKpGain));
  PID_SetKI(&PIDIdHandle_M1,
            BlowerIpc_PickFocGain(g_blower_ipc.foc_flux_ki, PIDIdHandle_M1.hKiGain));
  BlowerIpc_CM4_LatchIqLimit();
  __DMB();

  s_last_foc_tuning_seq = seq;
  g_blower_ipc.cm4_foc_tuning_ack_seq = seq;
  __DMB();
}

void BlowerIpc_CM4_Init(void)
{
  BlowerIpc_EnableSharedRamClock();
  __HAL_RCC_HSEM_CLK_ENABLE();

  BlowerIpc_InitSharedMemory();
  s_last_cmd_seq = g_blower_ipc.cm7_cmd_seq;
  BlowerIpc_CM4_SeedFocTuning();
  BlowerBrake_Init();
}

static void BlowerIpc_CM4_HandleCommand(BlowerIpcCommand_t cmd, int32_t rpm, uint16_t ramp_ms)
{
  switch (cmd)
  {
    case BLOWER_CMD_START:
      /* Clear latched FOC faults (e.g. over-voltage) before arming — CM7
       * may post ACK then START back-to-back and only the last IPC lands.
       * The start itself waits out any coast-down hold-off. */
      (void)MC_AcknowledgeFaultMotor1();
      BlowerIpc_CM4_RequestStart();
      BlowerIpc_CM4_ServicePendingStart();
      break;

    case BLOWER_CMD_STOP:
    {
      /* In FAULT_OVER acknowledge; otherwise stop, and look at the answer.
       *
       * MCI_StopMotor refuses while any fault is latched (PastFaults or
       * CurrentFaults) and in IDLE; MCI_FaultAcknowledged only acts in
       * FAULT_OVER. This used to stop first and acknowledge second with both
       * results discarded, so a STOP arriving in FAULT_OVER was refused and
       * only the ack that followed it made the NEXT stop possible.
       *
       * What that looked like on the bench (2026-09-24, 34000 rpm open loop,
       * port wide): the drive faulted, PWM went off and the rotor coasted -
       * but the angle/speed estimator is disabled in FAULT_NOW, FAULT_OVER
       * and IDLE (AngleSpeedEstimation[] in mc_tasks_foc.c), so the reported
       * speed FROZE at 34002 rpm and the blower looked like it was still
       * being driven while repeated STOPs "did nothing". Read mc_state and
       * occurred_faults on the B line, not mech_rpm, to know whether the
       * power stage is actually on.
       *
       * In FAULT_OVER, acknowledge ONLY. The ack clears PastFaults and queues
       * MCI_ACK_FAULTS; a MC_StopMotor1() after it then sees no faults and a
       * non-IDLE state, is accepted, and overwrites the queued command with
       * MCI_STOP - which FAULT_OVER ignores (mc_tasks_foc.c). Measured
       * 2026-09-24 after a DP_FAULT at 40 krpm: the drive stayed latched in
       * FAULT_OVER through every STOP. PWM is already off there, so the ack
       * alone is the whole job: FAULT_OVER -> IDLE on the next state-machine
       * tick. The ack is refused while the fault condition is still present
       * (CurrentFaults), so this cannot clear an active fault. */
      s_start_pending = false;    /* a start held off for coast-down is void */
      if (MC_GetSTMStateMotor1() == FAULT_OVER)
      {
        (void)MC_AcknowledgeFaultMotor1();
      }
      else if (MC_StopMotor1())
      {
        s_stop_in_flight = BlowerIpc_StateDrives(MC_GetSTMStateMotor1());
      }
      else
      {
        const MCI_State_t st = MC_GetSTMStateMotor1();
        /* Refused. In IDLE, STOP, WAIT_STOP_MOTOR, ICLWAIT or FAULT_* the
         * power stage is already off or on its way off: nothing to force.
         *
         * Refused while still POWERING the motor should be impossible - a
         * latched fault forces FAULT_OVER and switches PWM off - so this is
         * defence in depth, not the fix for the bench incident. If it ever
         * happens, do not leave the rotor driven with no report: raise the
         * SDK's own software-error stop (the same call TSK_HardwareFaultTask
         * makes). The safety task then switches PWM off, the state machine
         * goes to FAULT_NOW, MC_SW_ERROR shows in occurred_faults, and the
         * next START acknowledges it and runs normally. */
        if ((st == RUN) || (st == START) || (st == SWITCH_OVER) ||
            (st == ALIGNMENT) || (st == CHARGE_BOOT_CAP) ||
            (st == OFFSET_CALIB))
        {
          MCI_FaultProcessing(&Mci[M1], MC_SW_ERROR, 0U);
        }
      }
      BlowerBrake_ApplyPermille(0U);
      s_brake_latched = false;
      break;
    }

    case BLOWER_CMD_SET_SPEED:
    {
      /* CM7 often posts Start then SetSpeed in consecutive IPC writes; only
       * the last seq is observed. Ensure the motor is armed here so a lone
       * SET_SPEED (mask-off hold, resume ramp) cannot leave the rotor coasting.
       *
       * Do not call Start while FAULT_NOW still has an *active* fault (typical
       * regen OV during soft decel) — Ack only after the condition clears
       * (FAULT_OVER / IDLE). The ramp is still programmed so it runs on Start. */
      MCI_State_t state = MC_GetSTMStateMotor1();
      const uint16_t cur_faults = MC_GetCurrentFaultsMotor1();
      const int32_t mech_now =
          BlowerIpc_SpeedUnitToRpm(MC_GetMecSpeedAverageMotor1());

      /* Large soft decels pump the bus; latch brake early (OV at OV_VOLTAGE_THRESHOLD_V). */
      if ((mech_now > (rpm + 1500)) && (ramp_ms > 0U))
      {
        s_brake_latched = true;
        BlowerBrake_ApplyPermille(400U);
      }

      if ((state == FAULT_OVER) || ((state == FAULT_NOW) && (cur_faults == 0U)))
      {
        (void)MC_AcknowledgeFaultMotor1();
        state = MC_GetSTMStateMotor1();
      }

      if ((state == IDLE) || (state == FAULT_OVER) ||
          (state == STOP) || (state == WAIT_STOP_MOTOR))
      {
        /* Deferred while the rotor may still be coasting - see the
         * restart-while-coasting guard. This is the path that restarted
         * into a 32 krpm rotor after an OV trip on 2026-09-25. */
        (void)MC_AcknowledgeFaultMotor1();
        BlowerIpc_CM4_RequestStart();
        BlowerIpc_CM4_ServicePendingStart();
      }
      MC_ProgramSpeedRampMotor1(BlowerIpc_RpmToSpeedUnit(rpm), ramp_ms);
      break;
    }

    case BLOWER_CMD_ACK_FAULT:
      (void)MC_AcknowledgeFaultMotor1();
      break;

    default:
      break;
  }
}

void BlowerIpc_CM4_ProcessCommands(void)
{
  BlowerIpc_CM4_ApplyFocTuning();
  /* Every pass, not only on new commands: a held-off start must still be
   * issued when CM7 has gone quiet. */
  BlowerIpc_CM4_TrackCoast();
  BlowerIpc_CM4_ServicePendingStart();

  uint32_t seq;
  BlowerIpcCommand_t cmd;
  int32_t rpm;
  uint16_t ramp_ms;

  seq = g_blower_ipc.cm7_cmd_seq;
  if (seq == s_last_cmd_seq)
  {
    return;
  }

  __DMB();
  cmd = (BlowerIpcCommand_t)g_blower_ipc.command;
  rpm = g_blower_ipc.target_speed_rpm;
  ramp_ms = g_blower_ipc.ramp_duration_ms;
  __DMB();

  BlowerIpc_CM4_HandleCommand(cmd, rpm, ramp_ms);
  s_last_cmd_seq = seq;
  g_blower_ipc.cm4_cmd_ack_seq = seq;
  __DMB();
}

void BlowerIpc_CM4_UpdateStatus(void)
{
  MCI_State_t state = MC_GetSTMStateMotor1();

  g_blower_ipc.mc_state = (uint16_t)state;
  g_blower_ipc.current_faults = MC_GetCurrentFaultsMotor1();
  g_blower_ipc.occurred_faults = MC_GetOccurredFaultsMotor1();
  g_blower_ipc.mech_speed_rpm = BlowerIpc_SpeedUnitToRpm(MC_GetMecSpeedAverageMotor1());
  g_blower_ipc.speed_ref_rpm = BlowerIpc_SpeedUnitToRpm(MC_GetMecSpeedReferenceMotor1());
  g_blower_ipc.motor_running = ((state == RUN) || (state == START) || (state == SWITCH_OVER) ||
                                (state == STOP) || (state == WAIT_STOP_MOTOR)) ? 1U : 0U;
  {
    /* Coast-down hold-off still to run, ms; 1 while a start is pending but
     * the state machine has not reached IDLE yet. 0 = no start held off. */
    uint32_t hold = BlowerIpc_CM4_HoldoffRemainingMs();
    if (s_start_pending && (hold == 0U))
    {
      hold = 1U;
    }
    g_blower_ipc.restart_holdoff_ms = (uint16_t)((hold > 65535U) ? 65535U : hold);
  }

  /* Electrical diagnostics - see the note in blower_ipc.h. Converted here
   * because CURRENT_CONV_FACTOR_INV exists on this core only. If iq_ref sits
   * at NOMINAL_CURRENT (1345 mA) with iq tracking it and the speed still will
   * not rise, the drive is torque limited; if the bus sags under the same
   * load instead, it is the supply. Speed alone cannot separate the two. */
  {
    const qd_t iqd     = MC_GetIqdMotor1();
    const qd_t iqd_ref = MC_GetIqdrefMotor1();
    g_blower_ipc.iq_ma =
        (int16_t)((float)iqd.q * (float)CURRENT_CONV_FACTOR_INV * 1000.0f);
    g_blower_ipc.iq_ref_ma =
        (int16_t)((float)iqd_ref.q * (float)CURRENT_CONV_FACTOR_INV * 1000.0f);
    g_blower_ipc.id_ma =
        (int16_t)((float)iqd.d * (float)CURRENT_CONV_FACTOR_INV * 1000.0f);
  }
  /* Bus voltage at real resolution.
   *
   * NOT VBS_GetAvBusVoltage_V() - that returns WHOLE VOLTS, so scaling it by
   * 1000 yields exactly 20000 mV every sample and reports a rock-steady bus
   * whether or not one exists. The first capture with this line did exactly
   * that: 2137 samples, one distinct value, and an apparent 0.00 V sag that
   * was pure quantisation.
   *
   * _d is the averaged reading in u16Volt format, i.e. full scale = the
   * divider's range, ADC_REFERENCE_VOLTAGE / VBUS_PARTITIONING_FACTOR =
   * 52.8 V. That is ~0.8 mV per count, which will resolve a sag under a
   * one-second inspiratory load. */
  {
    const uint32_t vbus_d =
        (uint32_t)VBS_GetAvBusVoltage_d(&BusVoltageSensor_M1._Super);
    const uint32_t full_scale_mv =
        (uint32_t)((ADC_REFERENCE_VOLTAGE / VBUS_PARTITIONING_FACTOR) * 1000.0f);
    g_blower_ipc.bus_voltage_mv =
        (uint16_t)((vbus_d * full_scale_mv) / 65535U);
  }

  /* Rotor-frame stator voltage - see the note in blower_ipc.h. s16V to volts
   * per mc_api.c: V = s16V * Vbus / (sqrt(3) * 32768), 0-to-peak phase.
   * This hook runs once per medium-frequency tick (1 kHz); a 1/16 IIR gives
   * a ~16 ms time constant, enough to drop the PI ripple without smearing a
   * breath. Reset while the drive is not in RUN so a stale value is never
   * reported against a stopped rotor. */
  {
    static float s_vq_mv = 0.0f;
    static float s_vd_mv = 0.0f;
    if (state == RUN)
    {
      const qd_t  vqd  = MC_GetVqdMotor1();
      const float k_mv = (float)g_blower_ipc.bus_voltage_mv / (1.7320508f * 32768.0f);
      s_vq_mv += ((float)vqd.q * k_mv - s_vq_mv) * (1.0f / 16.0f);
      s_vd_mv += ((float)vqd.d * k_mv - s_vd_mv) * (1.0f / 16.0f);
    }
    else
    {
      s_vq_mv = 0.0f;
      s_vd_mv = 0.0f;
    }
    g_blower_ipc.vq_mv = (int16_t)s_vq_mv;
    g_blower_ipc.vd_mv = (int16_t)s_vd_mv;
  }

  __DMB();
  g_blower_ipc.cm4_status_seq++;
  __DMB();
  (void)HAL_HSEM_Release(BLOWER_IPC_HSEM_STATUS_ID, 0U);
}
