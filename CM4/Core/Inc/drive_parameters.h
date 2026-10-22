
/**
  ******************************************************************************
  * @file    drive_parameters.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the parameters needed for the Motor Control SDK
  *          in order to configure a motor drive.
  *
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef DRIVE_PARAMETERS_H
#define DRIVE_PARAMETERS_H

/************************
 *** Motor Parameters ***
 ************************/

/* TKFan BA7060H24B (repo CPAP_TKFan, 2026-10-22). Motor-derived values were
 * first regenerated from the TKFan profile (TKFan.json) by scaling, then
 * checked against Workbench's own project for the same profile,
 * D:\Code\CPAP_TestTKFan (generated for an STM32H745ZI / X-NUCLEO-IHM16M1
 * template, so its timer and power-stage values do not apply to this board).
 * Agreed exactly: F1/F2, PLL gains, max speed, observer hand-over and rev-up
 * target speeds; current PI identical once its divider is accounted for.
 * Taken from Workbench: observer GAIN1/GAIN2 (with RS 1.07, Ke 0.5),
 * nominal current 1.314, PHASE1_DURATION 1000. Kept deliberately different:
 * speed PI, PHASE2_DURATION, dividers, and all board/power-stage values -
 * see the notes at each. Original scaling rules, for reference:
 *   - current PI:   KP ~ Ls x profiling Vdc, KI ~ Rs x profiling Vdc
 *                   (KI/KP = 2 Rs Ts / Ls, pole-zero cancellation; the
 *                   TKFan was profiled at the same 20.77 V as cfe9528)
 *   - STO gains:    double observer pole at z = 0.2265 (all three WM6850
 *                   profiles land on 0.225-0.227 once MAX_BEMF_VOLTAGE is
 *                   truncated as parameters_conversion.h does)
 *   - PLL gains:    proportional to MAX_APPLICATION_SPEED_RPM
 *   - start-up:     observer minimum 36 %, rev-up target 40 % of max speed;
 *                   currents = nominal current
 * The hand-tuned items carried over from the WM6850 (speed PI, Iq limit
 * policy) are marked as such. */

/******** MAIN AND AUXILIARY SPEED/POSITION SENSOR(S) SETTINGS SECTION ********/

/* Over-modulation. 0 = classic SVPWM, 1 = PWMC_SetPhaseVoltage_OVM.
 *
 * TKFan, 2026-10-22: ON, for the 36000 rpm maximum below (was 0, the
 * Workbench default, while the cap was 32790). Measured on the TKFan
 * (23.6 V bus), |Vqd| is 83 % of Vbus/sqrt3 at 32k vent-only and 87-88 % at
 * 30-32k under load, so linear SVPWM runs out near ~34k under load; 36k
 * needs the extra ~10 % that over-modulation gives (six-step ceiling 0.78
 * vs 0.707 Vbus line-to-line rms). Why 36k: full AirSense 11 flow (136 L/min
 * at 20 cmH2O) at 3010 m / 35 C needs ~35.1k (density 0.72 of the bench).
 * The WM6850 build runs with this on. The hazards below (observer assumes
 * linear modulation; three-shunt sampling window) apply above ~34k: verify
 * speed estimate, Id and DP_FAULT there on the bench before relying on it.
 * Set 0 (and MAX_APPLICATION_SPEED_RPM back to 32790 with GAIN2 18328 and
 * PLL 872/31) to return to the linear-only build. */
#ifndef FOC_OVERMODULATION_ENABLE
#define FOC_OVERMODULATION_ENABLE           1
#endif

/*** Speed measurement settings ***/
/* TKFan, 2026-10-22: 32790 -> 36000. 32790 was the profiler's maxRatedSpeed
 * on its 20.77 V bus. Tenkai's own listing for this exact part
 * (BA7060H24B-K260417) gives 36000 rpm with 52.8 W and 458 L/min, which is
 * the datasheet's free-air point (34217 rpm, 44.8 W, 445 L/min) scaled by
 * the fan laws, i.e. its full-power top speed. NOT yet a written maximum
 * continuous rating - confirm with Tenkai (and bearing life at 36k).
 * Above ~34k under load this needs FOC_OVERMODULATION_ENABLE (above).
 * CM4 clamps incoming speed commands to this (blower_ipc_cm4.c), because
 * the SDK rejects a ramp target above it instead of saturating; the shared
 * BLOWER_IPC_MAX_SPEED_RPM (Common/Inc/blower_ipc.h) matches it.
 * Moving this moves MAX_BEMF_VOLTAGE (16 -> 17, so observer C3 9549 ->
 * 10146) and the PLL gains; GAIN2 and PLL_K*_GAIN below are regenerated for
 * 36000. The observer hand-over (OBS_MINIMUM_SPEED_RPM 11804) and rev-up
 * target (13116) are deliberately left at their verified 32790-based values
 * rather than Workbench's 36 %/40 % of 36000 (12960/14400). */
#define MAX_APPLICATION_SPEED_RPM           36000 /*!< rpm, mechanical */
#define MIN_APPLICATION_SPEED_RPM           0 /*!< rpm, mechanical, absolute value */
#define M1_SS_MEAS_ERRORS_BEFORE_FAULTS     3 /*!< Number of speed measurement errors before main sensor goes in fault */

/****** State Observer + PLL ****/
#define VARIANCE_THRESHOLD                  0.25 /*!< Maximum accepted variance on speed estimates (percentage) */

/* State observer scaling factors F1 */
/* TKFan: F1 8192 -> 16384, the largest scale at which C1/C2/C3/C5 still fit
 * int16 (C5 = 20292 with LS 0.21 mH; with the WM6850's 0.07 mH it did not,
 * which is why that profile had 8192). */
#define F1                                  16384
#define F2                                  16384
#define F1_LOG                              LOG2((16384))
#define F2_LOG                              LOG2((16384))

/* State observer constants. TKFan: Workbench's values (CPAP_TestTKFan),
 * computed for RS 1.07, LS 0.21 mH, Ke 0.5 (MAX_BEMF_VOLTAGE 16, C1 4174,
 * C3 9549 at F1 = F2 = 16384): observer poles 0.219 +/- 0.200j, |z| 0.30.
 * They are only valid with those RS / LS / MOTOR_VOLTAGE_CONSTANT values.
 * 2026-10-22, max speed 36000: MAX_BEMF_VOLTAGE 17, C3 10146, so GAIN2
 * 18328 x 9549/10146 = 17250 keeps the product C3*GAIN2 and with it the
 * same poles (checked: 0.2186 +/- 0.2034j). GAIN1 depends only on C1 and is
 * unchanged. For the 32790 build: GAIN2 18328. Was -11146 / 5275 for the
 * WM6850. */
#define GAIN1                               -21432
#define GAIN2                               17250

/* Only in case PLL is used, PLL gains */
/* TKFan: Workbench scales both with max speed. 36000: 872/31 x 36000/32790
 * = 957/34 (the same values Workbench gave the WM6850 at 36012). For the
 * 32790 build: 872/31. */
#define PLL_KP_GAIN                         957
#define PLL_KI_GAIN                         34
#define PLL_KPDIV                           16384
#define PLL_KPDIV_LOG                       LOG2((PLL_KPDIV))
#define PLL_KIDIV                           65535
#define PLL_KIDIV_LOG                       LOG2((PLL_KIDIV))

#define STO_FIFO_DEPTH_DPP                  64 /*!< Depth of the FIFO used  to average mechanical speed in dpp format */
#define STO_FIFO_DEPTH_DPP_LOG              LOG2((64))
#define STO_FIFO_DEPTH_UNIT                 64 /*!< Depth of the FIFO used to average mechanical speed in the unit defined by #SPEED_UNIT */
#define M1_BEMF_CONSISTENCY_TOL             64 /* Parameter for B-emf amplitude-speed consistency */
#define M1_BEMF_CONSISTENCY_GAIN            64 /* Parameter for B-emf amplitude-speed consistency */

/* USER CODE BEGIN angle reconstruction M1 */
#define PARK_ANGLE_COMPENSATION_FACTOR      0
#define REV_PARK_ANGLE_COMPENSATION_FACTOR  0
/* USER CODE END angle reconstruction M1 */

/**************************    DRIVE SETTINGS SECTION   **********************/
/* PWM generation and current reading */
#define PWM_FREQUENCY                       20000
#define PWM_FREQ_SCALING                    1
#define LOW_SIDE_SIGNALS_ENABLING           ES_GPIO

/* Torque and flux regulation loops */
#define REGULATION_EXECUTION_RATE           1 /*!< FOC execution rate in number of PWM cycles */
#define ISR_FREQUENCY_HZ                    (PWM_FREQUENCY/REGULATION_EXECUTION_RATE) /*!< @brief FOC execution rate in Hz */

/* Gains values for torque and flux control loops.
 * TKFan: 2168 x 0.21/0.07 and 806 x 1.0748/0.2601 (WM6850 Workbench values
 * scaled by Ls and Rs). Identical to Workbench's TKFan project, which has
 * KP 3252 with TF_KPDIV 4096 (= 6504 / 8192) and KI 3329. The 8192 divider is
 * kept so raw "foc set torque_kp" values keep their meaning on this repo. */
#define PID_TORQUE_KP_DEFAULT               6504
#define PID_TORQUE_KI_DEFAULT               3330
#define PID_TORQUE_KD_DEFAULT               100
#define PID_FLUX_KP_DEFAULT                 6504
#define PID_FLUX_KI_DEFAULT                 3330
#define PID_FLUX_KD_DEFAULT                 100

/* Torque/Flux control loop gains dividers*/
#define TF_KPDIV                            8192
#define TF_KIDIV                            16384
#define TF_KDDIV                            8192
#define TF_KPDIV_LOG                        LOG2((8192))
#define TF_KIDIV_LOG                        LOG2((16384))
#define TF_KDDIV_LOG                        LOG2((8192))
#define TFDIFFERENTIAL_TERM_ENABLING        DISABLE

/* TKFan: speed PI KEPT at the values hand-tuned on the WM6850 (1637 / 1024),
 * not Workbench's. Workbench's TKFan project has KP 3713 with SP_KPDIV 1024
 * (= 464 on this file's /128 scale, 3.5x softer than 1637) and KI 10 (100x
 * lower); on the WM6850 its KI (307) already proved far too slow for CPAP.
 * Fallback if the speed loop oscillates on the TKFan: "foc set speed_kp 464"
 * and raise KI from there.
 * The TKFan rotor has ~1.4x the WM6850's inertia (2.45 vs 1.73), so with the
 * same gains expect a ~30 % slower speed loop. Re-tune on the bench (runtime:
 * "foc set speed_kp / speed_ki"); the WM6850 notes below explain the limits
 * (KP was capped by DP_FAULT on ref steps, not by stability).
 *
 * WM6850 history:
 * 2026-10-01: KP 1637 -> 4000 (Workbench value was 1637).
 * 1637/128 is ~0.43 mA per rpm of speed error; against the measured inertia
 * (~40 mA per krpm/s) that is a ~1.7 Hz speed loop. In breathing at 16 cmH2O
 * the rotor had to be >1000 rpm off its reference before the PI pushed hard
 * either way, and it stayed high while the command fell in the second half
 * of each inhale - most of the inspiratory over-pressure. Bench, leak rig,
 * open loop, 2026-10-01:
 *   31<->37k ramps at 15 krpm/s (kp_track.py):
 *      kp     lag      max err      overshoot    |id|max
 *      1637   55-60ms  900-1010rpm  190/440 rpm  0.23 A
 *      4000   30-35ms  510-580rpm   <= 40 rpm    0.19 A
 *      8000   unstable near 36-37k (+/-800-950 rpm, |id| 0.96 A)
 *   replay of recorded 16 cmH2O breath trajectories (traj_kp.py), ideal
 *   speed: rms error 712 -> 511 rpm, rotor-high-while-falling +591 -> +168
 *   rpm (~0.6 -> ~0.2 cmH2O), |id|max 0.70 -> 0.24 A, no faults to 38.5k.
 * Hold ripple at 31k unchanged (3 rpm, ~37 mA). 4000 keeps ~2x margin to the
 * 8000 oscillation. The 2400/512 entry below was large STEPS at 38k, which
 * are current-limited whatever kp is; this is tracking a moving reference.
 *
 * REVERTED to 1637 the same day: in THERAPY at 16 cmH2O, 4000 tripped
 * DP_FAULT (STSPIN830 OCP, 3 A min) six times in ~100 s - once while
 * settling, then repeatedly while breathing - and 1637 zero times in the
 * interleaved blocks. Faults at 28-35 krpm with sampled |iq| <= 1.76 A, each
 * right after the CM7 command jumped several krpm (breath onset, or the
 * +/-1-2 krpm per-tick jitter). CM7 posts every tick with ramp 0, so the
 * reference STEPS; at kp 4000 each step is a 2.4x larger iq_ref step, and the
 * current-loop overshoot plus PWM ripple reaches the 3 A trip. The bench
 * replay above sent 100 ms ramps, which hid exactly that. Do not raise kp
 * again without limiting how fast iq_ref (or the speed reference) can step,
 * and test it with per-tick ramp-0 commands, i.e. closed loop. */
#define PID_SPEED_KP_DEFAULT                1637/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
/* 2026-09-25: KI 256 -> 1024 (Workbench value was 256).
 * Speed-step tests at the 38 krpm top, closed circuit, open loop:
 *      kp/ki        t90 (steps)    settle       reaches target
 *      1637/256     1.16-1.70 s    0.9-2.4 s    ~500 rpm short at 2.5 s
 *      1637/512     0.46-0.82 s    0.36-1.2 s   yes
 *      2400/512     1.04-1.22 s    0.9-1.7 s    50-170 short (kp does not help)
 *      1637/1024    0.24-0.44 s    0.2-0.54 s   yes
 * At 1024 large steps are current-limited (iq_ref at IQMAX 50-58 % of the
 * rise, ~21 krpm/s), current noise at 30-38 krpm unchanged (iq sd 59-70 mA,
 * id sd 65-85 mA), no faults. Ramp overshoot 430 rpm (was 10).
 * In therapy at 20 cmH2O the rotor had been ~1.2 krpm short of its command at
 * the pressure minimum; with 1024 (and the therapy ceiling at 38500,
 * safety_supervisor.c) the mask-end dip at matched flow fell 2.53 -> 2.10
 * cmH2O with no drive stops or faults. Pair the two: with the faster loop
 * the rotor really reaches its command, and above ~39 krpm under load
 * current control breaks down. */
#define PID_SPEED_KI_DEFAULT                1024/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/
#define PID_SPEED_KD_DEFAULT                0/(SPEED_UNIT/10) /* Workbench compute the gain for 01Hz unit*/

/* Speed control loop */
#define SPEED_LOOP_FREQUENCY_HZ             (uint16_t)1000 /*!<Execution rate of speed regulation loop (Hz) */

/* Speed PID parameter dividers */
#define SP_KPDIV                            128
#define SP_KIDIV                            16384
#define SP_KDDIV                            16
#define SP_KPDIV_LOG                        LOG2((128))
#define SP_KIDIV_LOG                        LOG2((16384))
#define SP_KDDIV_LOG                        LOG2((16))

/* USER CODE BEGIN PID_SPEED_INTEGRAL_INIT_DIV */
#define PID_SPEED_INTEGRAL_INIT_DIV         0 /*  */
/* USER CODE END PID_SPEED_INTEGRAL_INIT_DIV */

#define SPD_DIFFERENTIAL_TERM_ENABLING      DISABLE
/* Speed-PI output / integral limit (boot default). 2026-10-01: can be changed
 * at runtime with "foc set iq_max_ma <mA>" (CM4 clamps to 500-1800 mA, see
 * blower_ipc_cm4.c), so a higher limit can be trialled against this one in
 * the same session and is gone on reboot.
 *
 * Why a higher limit is being tried: at 20 cmH2O, breath-onset pressure
 * dips are acceleration-limited. At
 * ~32 krpm holding speed against the airflow takes ~850-1000 mA, leaving
 * only ~450-600 mA of 1445 to accelerate (rotor ~17 krpm/s, command pinned
 * at the ceiling ~100 ms before flow onset). 1.65 A adds ~40% accelerating
 * current.
 *
 * RISK - read before raising it: the STSPIN830 trips its internal
 * over-current protection at 3.0 A min / 3.5 A typ (DP_FAULT), and with
 * this motor's ~50-70 uH the 20 kHz ripple is ~1.4-2.0 A p-p, i.e. peaks
 * ~0.7-1.0 A above the sampled current. At 1.445 A that was already enough
 * to trip during the old full-current start (fixed by the CM7 post-hand-
 * over ramp). Phase-current peaks have NOT been measured on a scope.
 * Only make a higher value the default once starts and breathing show no
 * DP_FAULT and the STSPIN830 does not run hot. Motor copper loss at the
 * limit: ~1.55 W at 1.445 A, ~2.0 W at 1.65 A.
 *
 * Result (breathing A/B, 2026-10-01): 1.65 A flat helped at 16 cmH2O but
 * tripped DP_FAULT once at 38.2 krpm at 20 cmH2O. This value is therefore
 * kept as the limit above ~37 krpm, and a speed-dependent boost (1.65 A up
 * to 35.5 krpm, tapering to this by 37 krpm) is applied on top - see
 * BlowerIpc_CM4_UpdateIqLimit() in blower_ipc_cm4.c. */
/* TKFan: the profile's nominal current (Workbench sets IQMAX to it). This is
 * the base limit; above 14-15 krpm an 1800 mA boost applies (blower_ipc_cm4.c,
 * on by default since 2026-10-22 after bench step tests). Copper loss at a
 * given current is ~4x the WM6850's (1.07 vs 0.26 ohm per phase). */
#define IQMAX_A                             1.314

/* Default settings */
#define DEFAULT_CONTROL_MODE                MCM_SPEED_MODE
#define DEFAULT_TARGET_SPEED_RPM            11804
#define DEFAULT_TARGET_SPEED_UNIT           (DEFAULT_TARGET_SPEED_RPM*SPEED_UNIT/U_RPM)
#define DEFAULT_TORQUE_COMPONENT_A          0
#define DEFAULT_FLUX_COMPONENT_A            0

/**************************    FIRMWARE PROTECTIONS SECTION   *****************/
/* 2026-09-25: 24.9 -> 28.0 V.
 *
 * 24.9 was harmless while VBUS_PARTITIONING_FACTOR was mis-scaled - the real
 * trip was then ~28.1 V. Correcting the factor on 2026-09-24 made 24.9 real,
 * leaving ~1 V above the 23.9 V supply, and with the brake resistor disabled
 * a hard end-of-inspiration decel now regenerates past it. Measured in
 * therapy at 20 cmH2O (human breathing): pressure overshoot to 23.4 cmH2O,
 * command 40000 -> 29300 rpm in 180 ms, rotor -32 krpm/s, drive stopped at
 * ~32.6 krpm with the bus at 24.6 V just after. The restart-while-coasting
 * hold-off then kept it off for 8.8 s: ~13 s without pressure.
 *
 * 28.0 restores the headroom the machine ran with before the scaling fix.
 * Ratings on the regen path (BLOWER_BOARD_FINAL_SCH): 24 V input bulk
 * 330 uF / 35 V (C5, C6, C10, C13) and 330 uF / 50 V (C1); motor bus
 * 33 uF / 50 V (C89, C90); STSPIN830 45 V max. 28 V is 80 % of the lowest
 * (35 V). The bench supply also sees the regen - check it tolerates being
 * pushed to 28 V. The brake stays disabled (BRAKE_VBUS_ON_V, blower_ipc_cm4.c
 * is kept above this trip). */
#define OV_VOLTAGE_THRESHOLD_V              28.0 /*!< Over-voltage threshold */
#define UD_VOLTAGE_THRESHOLD_V              7.0 /*!< Under-voltage threshold */
#ifdef NOT_IMPLEMENTED
#define ON_OVER_VOLTAGE                     TURN_OFF_PWM /*!< TURN_OFF_PWM, TURN_ON_R_BRAKE or TURN_ON_LOW_SIDES */
#endif /* NOT_IMPLEMENTED */
#define OV_TEMPERATURE_THRESHOLD_C          110 /*!< Celsius degrees */
#define OV_TEMPERATURE_HYSTERESIS_C         10 /*!< Celsius degrees */
#define HW_OV_CURRENT_PROT_BYPASS           DISABLE /*!< In case ON_OVER_VOLTAGE is set to TURN_ON_LOW_SIDES this
                                                         feature may be used to bypass HW over-current protection
                                                         (if supported by power stage) */
#define OVP_INVERTINGINPUT_MODE             INT_MODE
#define OVP_INVERTINGINPUT_MODE2            INT_MODE
#define OVP_SELECTION                       COMP_Selection_COMP1
#define OVP_SELECTION2                      COMP_Selection_COMP1

/******************************   START-UP PARAMETERS   **********************/
/* TKFan: rev-up to 13116 rpm (40 % of max) at the nominal 1.314 A, observer
 * hand-over from 11804 rpm (36 %) - both as Workbench generated them.
 *
 * PHASE1 (alignment) 320 -> 1000 ms, Workbench's TKFan value: the TKFan's low
 * friction (0.41 vs 1.94) means the rotor rings longer before it settles on
 * the alignment vector.
 *
 * PHASE2 (rev-up) KEPT at 4322 ms; Workbench's TKFan project has 26232 ms.
 * Workbench's figure scales with the mechanical time constant J/B (WM6850
 * 4322 x (2.45/1.73) x (1.94/0.41) = 29000), not with what the start needs:
 * 1.314 A gives ~7.7 mNm, and following 13116 rpm in 4.3 s on 2.45e-6
 * kg m^2 takes ~0.8 mNm. With 26 s the observer would only lock at ~25 s,
 * past CM7's 10 s start grace (g_safety_grace_s), so every therapy start
 * would trip. If the start fails to lock on the bench, lengthen this first
 * (in steps; 8-10 s still fits the grace). */

/* Phase 1 */
#define PHASE1_DURATION                     1000 /*milliseconds */
#define PHASE1_FINAL_SPEED_UNIT             (0*SPEED_UNIT/U_RPM)
#define PHASE1_FINAL_CURRENT_A              1.314

/* Phase 2 */
#define PHASE2_DURATION                     4322 /*milliseconds */
#define PHASE2_FINAL_SPEED_UNIT             (13116*SPEED_UNIT/U_RPM)
#define PHASE2_FINAL_CURRENT_A              1.314

/* Phase 3 */
#define PHASE3_DURATION                     0 /*milliseconds */
#define PHASE3_FINAL_SPEED_UNIT             (13116*SPEED_UNIT/U_RPM)
#define PHASE3_FINAL_CURRENT_A              1.314

/* Phase 4 */
#define PHASE4_DURATION                     0 /*milliseconds */
#define PHASE4_FINAL_SPEED_UNIT             (13116*SPEED_UNIT/U_RPM)
#define PHASE4_FINAL_CURRENT_A              1.314

/* Phase 5 */
#define PHASE5_DURATION                     0 /* milliseconds */
#define PHASE5_FINAL_SPEED_UNIT             (13116*SPEED_UNIT/U_RPM)
#define PHASE5_FINAL_CURRENT_A              1.314

#define ENABLE_SL_ALGO_FROM_PHASE           2

/* Sensor-less rev-up sequence */
#define STARTING_ANGLE_DEG                  0  /*!< degrees [0...359] */

/* Observer start-up output conditions  */
#define OBS_MINIMUM_SPEED_RPM               11804
#define NB_CONSECUTIVE_TESTS                2 /* corresponding to former
                                                 NB_CONSECUTIVE_TESTS / (TF_REGULATION_RATE / MEDIUM_FREQUENCY_TASK_RATE) */
#define SPEED_BAND_UPPER_LIMIT              17 /*!< It expresses how much estimated speed can exceed forced stator electrical
                                                 without being considered wrong. In 1/16 of forced speed */
#define SPEED_BAND_LOWER_LIMIT              15 /*!< It expresses how much estimated speed can be below forced stator electrical
                                                 without being considered wrong. In 1/16 of forced speed */

#define TRANSITION_DURATION                 25 /* Switch over duration, ms */

/******************************   BUS VOLTAGE Motor 1  **********************/
#define  M1_VBUS_SAMPLING_TIME              LL_ADC_SAMPLING_CYCLE(64)

/******************************   Temperature sensing Motor 1  **********************/
#define  M1_TEMP_SAMPLING_TIME              LL_ADC_SAMPLING_CYCLE(64)

/******************************   Current sensing Motor 1   **********************/
#define ADC_SAMPLING_CYCLES                 (8 + SAMPLING_CYCLE_CORRECTION)

/******************************   ADDITIONAL FEATURES   **********************/

/*** On the fly start-up ***/

/**************************
 *** Control Parameters ***
 **************************/

/* ##@@_USER_CODE_START_##@@ */
/* ##@@_USER_CODE_END_##@@ */

#endif /*DRIVE_PARAMETERS_H*/
/******************* (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
