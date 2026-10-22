#ifndef BLOWER_IPC_H
#define BLOWER_IPC_H

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

/* Dual-core shared mailbox in D3 SRAM4 (both cores must map the same address). */
#define BLOWER_IPC_SRAM4_BASE       0x38000000U
#define BLOWER_IPC_MAGIC            0x424C5752U /* 'BLWR' */

#define BLOWER_IPC_HSEM_CMD_ID      1U
#define BLOWER_IPC_HSEM_STATUS_ID   2U

/*
 * Hard ceiling exposed to the CM7 caller. The MCWB-generated firmware on
 * CM4 saturates the speed reference at MAX_APPLICATION_SPEED_RPM. We clamp
 * at that same value on the CM7 side so requests beyond the motor's reachable
 * speed do not behave like faults.
 *
 * KEEP THIS EQUAL TO MAX_APPLICATION_SPEED_RPM in
 * CM4/Core/Inc/drive_parameters.h. They are separate headers on separate
 * cores and nothing checks that they agree. Until 2026-09-23 this read 36528
 * while CM4 was actually set to 36012, and the comment above asserted the two
 * matched - so CM7 believed it had 516 rpm that CM4 would silently clip.
 * Both are now 43000.
 *
 * This is the DRIVE's capability clamp, not a therapy limit. The therapy
 * ceiling is g_safety_safe_rpm_max in safety_supervisor.c and is deliberately
 * lower; do not raise that one just because this moved.
 *
 * Raising this does not raise the speed the motor can actually reach: the
 * drive is back-EMF limited near 34-36 kRPM on this bus. Read the note on
 * MAX_APPLICATION_SPEED_RPM before reading anything into this number.
 */
/* TKFan BA7060H24B: 36000, = MAX_APPLICATION_SPEED_RPM in CM4's
 * drive_parameters.h (32790 before over-modulation was enabled; 43000 for the
 * WM6850). CM4 also clamps to its own limit, because the motor-control SDK
 * rejects - not saturates - a target above it. */
#define BLOWER_IPC_MAX_SPEED_RPM    36000

typedef enum
{
  BLOWER_CMD_NONE = 0,
  BLOWER_CMD_START,
  BLOWER_CMD_STOP,
  BLOWER_CMD_SET_SPEED,
  BLOWER_CMD_ACK_FAULT,
} BlowerIpcCommand_t;

typedef enum
{
  FOC_GAIN_SPEED_KP = 0,
  FOC_GAIN_SPEED_KI,
  FOC_GAIN_TORQUE_KP,
  FOC_GAIN_TORQUE_KI,
  FOC_GAIN_FLUX_KP,
  FOC_GAIN_FLUX_KI,
  FOC_GAIN_IQ_MAX_MA,     /* speed-PI current limit, mA (2026-10-01) */
  FOC_GAIN_IQ_BOOST_MA,   /* speed-dependent limit: boost current, mA */
  FOC_GAIN_IQ_BOOST_LO_RPM, /* full boost up to this speed */
  FOC_GAIN_IQ_BOOST_HI_RPM, /* back to iq_max from this speed */
} BlowerFocGainId_t;

typedef struct __attribute__((aligned(32)))
{
  uint32_t magic;

  volatile uint32_t cm7_cmd_seq;
  volatile uint32_t cm4_cmd_ack_seq;
  volatile uint32_t cm4_status_seq;

  volatile uint32_t command;
  /* RPM is carried as int32 - int16 (max 32767) was overflowing the
   * blower's operating range and tripping MC_SPEED_FDBK on CM4. */
  volatile int32_t  target_speed_rpm;
  volatile uint16_t ramp_duration_ms;
  volatile uint16_t reserved_align;

  volatile uint16_t mc_state;
  volatile uint16_t current_faults;
  volatile uint16_t occurred_faults;
  volatile int32_t  mech_speed_rpm;
  volatile int32_t  speed_ref_rpm;
  volatile uint16_t bus_voltage_v;
  volatile uint16_t brake_pwm_permille;
  volatile uint8_t  brake_active;
  volatile uint8_t  motor_running;
  /* Was `reserved` - same slot, so the layout is unchanged and a core built
   * before this still interoperates (it reads/writes 0 here). CM4's
   * restart-while-coasting hold-off still to run, ms: after drive stops, a
   * start is deferred until the rotor has had time to stop (see
   * blower_ipc_cm4.c). 1 = a start is pending but the state machine is not
   * back in IDLE yet. 0 = no start is being held off. */
  volatile uint16_t restart_holdoff_ms;

  /* CM4 FOC inner-loop PI gains (runtime tunable via CM7 / USB). */
  volatile uint32_t cm7_foc_tuning_seq;
  volatile uint32_t cm4_foc_tuning_ack_seq;
  volatile int16_t  foc_speed_kp;
  volatile int16_t  foc_speed_ki;
  volatile int16_t  foc_torque_kp;
  volatile int16_t  foc_torque_ki;
  volatile int16_t  foc_flux_kp;
  volatile int16_t  foc_flux_ki;
  /* Was foc_reserved - same slot, layout unchanged. Speed-PI output limit
   * (the q-axis current request cap) in mA; 0 = leave as is. CM4 seeds it
   * with IQMAX_A at boot and writes back the value it actually applied. */
  volatile uint16_t foc_iq_max_ma;

  /* Motor electrical diagnostics, added 2026-09-23.
   *
   * After the speed ceiling went to 40000 the command stopped being the
   * binding constraint and the motor became it: mech ran 2300-4000 rpm under
   * cmd on every breath, and on one run it reached 36400 where it had been
   * stuck at 33800 under a similar command. Whether that is the bus sagging
   * or the q-axis current saturating at NOMINAL_CURRENT cannot be told apart
   * from speed alone, and we had been arguing about it instead of measuring.
   *
   * Currents are in mA so CM7 needs none of CM4's conversion macros. Sign
   * follows the MC SDK: positive iq is motoring torque. bus_voltage_mv exists
   * because bus_voltage_v is whole volts, too coarse to see a sag under a
   * one-second inspiratory load. */
  volatile int16_t  iq_ma;
  volatile int16_t  iq_ref_ma;
  volatile int16_t  id_ma;
  volatile uint16_t bus_voltage_mv;

  /* Applied stator voltage in the rotor frame, mV phase 0-to-peak, added
   * 2026-09-29. Appended so every field above keeps its offset.
   *
   * Purpose: measure the motor's inductance as the drive sees it. The ST
   * profiler put Ls at 70 uH; an LCR meter across two windings reads ~460 uH
   * (~230 uH per phase), and Rs 0.26 vs 0.495 ohm per phase. Both feed the
   * observer and current-loop gains, and flux weakening would depend on Ld.
   * At steady speed with Id = 0, Vd = -w*Lq*Iq and Vq = Rs*Iq + w*lambda, so
   * a few steady-speed points separate Lq, Rs and the magnet flux.
   *
   * This is the FOC's commanded voltage (after circle limitation), low-pass
   * filtered on CM4 (~16 ms) because the raw PI output rides on PWM-rate
   * ripple. It equals the applied voltage only in linear modulation: above
   * ~37.7 krpm over-modulation makes the real fundamental larger than this. */
  volatile int16_t  vq_mv;
  volatile int16_t  vd_mv;

  /* Speed-dependent speed-PI current limit, added 2026-10-01. Appended so
   * every field above keeps its offset.
   *
   * foc_iq_max_ma is the limit everywhere except a boost band. In RUN, from
   * ~20 krpm up to foc_iq_boost_lo_rpm the limit is foc_iq_boost_ma; it
   * tapers linearly back to foc_iq_max_ma at foc_iq_boost_hi_rpm. Why: above
   * ~37 krpm the three-shunt sampling window gets short and current control
   * degrades (Id scatter 1-2 A, one DP_FAULT at 38.2 krpm with 1650 mA
   * flat), while below ~36 krpm 1650 mA cut the 16 cmH2O onset dip from 1.34
   * to 0.87 cmH2O with no faults. A lower speed ceiling instead was worse at
   * 20 cmH2O (dip 1.49 -> 2.63): there the blower needs the speed. See
   * BlowerIpc_CM4_UpdateIqLimit(). boost_ma 0 or <= iq_max = no boost. CM4
   * seeds these at boot and writes back what it applied.
   *
   * iq_lim_ma is status, not a setting: the limit in force this speed-loop
   * tick, mA (CM4 writes it; B line field 22). */
  volatile uint16_t foc_iq_boost_ma;
  volatile uint16_t foc_iq_boost_lo_rpm;
  volatile uint16_t foc_iq_boost_hi_rpm;
  volatile uint16_t iq_lim_ma;
} BlowerIpcShared_t;

typedef struct
{
  uint16_t mc_state;
  uint16_t current_faults;
  uint16_t occurred_faults;
  int32_t  mech_speed_rpm;
  int32_t  speed_ref_rpm;
  uint16_t bus_voltage_v;
  uint16_t brake_pwm_permille;
  bool     brake_active;
  bool     motor_running;
  uint32_t status_seq;
  uint32_t cmd_ack_seq;
  int16_t  iq_ma;
  int16_t  iq_ref_ma;
  int16_t  id_ma;
  uint16_t bus_voltage_mv;
  uint16_t restart_holdoff_ms;   /* see BlowerIpcShared_t */
  int16_t  vq_mv;                /* see BlowerIpcShared_t */
  int16_t  vd_mv;
  uint16_t iq_lim_ma;            /* see BlowerIpcShared_t */
} BlowerIpcStatus_t;

extern BlowerIpcShared_t g_blower_ipc;

void BlowerIpc_EnableSharedRamClock(void);
void BlowerIpc_InitSharedMemory(void);

#if defined(CORE_CM4)
void BlowerIpc_CM4_Init(void);
void BlowerIpc_CM4_ProcessCommands(void);
void BlowerIpc_CM4_UpdateStatus(void);
void BlowerIpc_CM4_UpdateIqLimit(void);
void BlowerBrake_Init(void);
void BlowerBrake_Update(void);
#endif

#if defined(CORE_CM7)
void BlowerIpc_CM7_Init(void);
bool BlowerIpc_CM7_Start(void);
bool BlowerIpc_CM7_Stop(void);
bool BlowerIpc_CM7_SetSpeedRpm(int32_t rpm, uint16_t ramp_duration_ms);
/* Same as SetSpeedRpm but with no DBG_I/DBG_W output. Use this from a
 * high-rate control loop (e.g. the pressure PID at 200 Hz) where the
 * logging volume of the verbose variant would flood the USB-CDC link. */
bool BlowerIpc_CM7_SetSpeedRpmSilent(int32_t rpm);
bool BlowerIpc_CM7_AcknowledgeFault(void);
bool BlowerIpc_CM7_GetStatus(BlowerIpcStatus_t *status);
bool BlowerIpc_CM7_SetFocGain(BlowerFocGainId_t id, int32_t value);
#endif

#ifdef __cplusplus
}
#endif

#endif /* BLOWER_IPC_H */
