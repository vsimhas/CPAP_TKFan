/**
  ******************************************************************************
  * @file    pmsm_motor_parameters.h
  * @author  Motor Control SDK Team, ST Microelectronics
  * @brief   This file contains the parameters needed for the Motor Control SDK
  *          in order to configure the motor to drive.
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
#ifndef PMSM_MOTOR_PARAMETERS_H
#define PMSM_MOTOR_PARAMETERS_H

/************************
 *** Motor Parameters ***
 ************************/

/***************** MOTOR ELECTRICAL PARAMETERS  ******************************/
/* TKFan BA7060H24B (Shenzhen Tenkai), profiled with ST Motor Control
 * Workbench on this board: profile "TKFan" (description "BA7060"),
 * ~/.st_workbench/hardware/motor/TKFan.json, 2026-10-22:
 *   rs 1.0748 ohm, ls 0.21 mH, BEmfConstant 0.4562 V rms ph-ph / krpm,
 *   polePairs 1, nominalCurrent 1.3149 A, maxRatedSpeed 32790 rpm,
 *   nominalDCVoltage 20.77 V, inertia 2.45, friction 0.41.
 * Datasheet (BA7060H24B-K260417 v02) cross-check: 3-phase, 6-slot, 2-pole
 * (1 pole pair); 1.9 ohm and 540 uH line-to-line at 1 kHz without rotor,
 * i.e. ~0.95 ohm / 270 uH per phase - consistent with the profiler.
 * Previous blower (WM6850, profile WM6850_H747_LowRes_OpnAtm): RS 0.26,
 * LS 0.00007, Ke 0.5, max 36012 (later raised to 43000), nominal 1.345 A. */
#define POLE_PAIR_NUM           1 /* Number of motor pole pairs */
#define RS                      1.07 /* Stator resistance , ohm*/
#define LS                      0.00021 /* Stator inductance, H
                                                 For I-PMSM it is equal to Lq */

/* When using Id = 0, NOMINAL_CURRENT is utilized to saturate the output of the
   PID for speed regulation (i.e. reference torque).
   Transformation of real currents (A) into int16_t format must be done accordingly with
   formula:
   Phase current (int16_t 0-to-peak) = (Phase current (A 0-to-peak)* 32767 * Rshunt *
                                   *Amplifying network gain)/(MCU supply voltage/2)
*/

/* Measured 0.4562; 0.5 is what Workbench writes (it rounds to one decimal),
 * as in the Workbench reference project D:\Code\CPAP_TestTKFan. Kept at 0.5
 * because the observer gain GAIN2 in drive_parameters.h is Workbench's, and
 * Workbench computed it for this value (through MAX_BEMF_VOLTAGE -> C3).
 * Change the two together. */
#define MOTOR_VOLTAGE_CONSTANT  0.5 /*!< Volts RMS ph-ph /kRPM */
/* TKFan BA7060H24B: 36000 per Tenkai's Alibaba listing for this part
 * (BA7060H24B-K260417: 36000 rpm, 52.8 W, 458 L/min), consistent with the
 * datasheet's free-air point (34217 rpm at 24 V on Tenkai's driver) scaled
 * by the fan laws. The datasheet itself gives no maximum; written
 * confirmation of the continuous rating is still pending. The profiler
 * measured 32790 on a 20.77 V bus. Documentation only - nothing in CM4
 * consumes this macro. */
#define MOTOR_MAX_SPEED_RPM     36000 /*!< Maximum rated speed  */
#define NOMINAL_CURRENT_A       1.314

#define ID_DEMAG_A              -1.314 /*!< Demagnetization current */

/***************** MOTOR SENSORS PARAMETERS  ******************************/
/* Motor sensors parameters are always generated but really meaningful only
   if the corresponding sensor is actually present in the motor         */

/*** Hall sensors ***/
#define HALL_SENSORS_PLACEMENT  DEGREES_120 /*!<Define here the
                                                 mechanical position of the sensors
                                                 withreference to an electrical cycle.
                                                 It can be either DEGREES_120 or
                                                 DEGREES_60 */

#define HALL_PHASE_SHIFT        300 /*!< Define here in degrees
                                                 the electrical phase shift between
                                                 the low to high transition of
                                                 signal H1 and the maximum of
                                                 the Bemf induced on phase A */
/*** Quadrature encoder ***/
#define M1_ENCODER_PPR          400  /*!< Number of pulses per
                                            revolution */

#endif /* PMSM_MOTOR_PARAMETERS_H */
/******************* (C) COPYRIGHT 2026 STMicroelectronics *****END OF FILE****/
