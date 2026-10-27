/**
  ******************************************************************************
  * @file    flow_sensor.h
  * @brief   Flow-element differential-pressure conditioning.
  *
  *  The flow element at the blower outlet generates a small differential
  *  pressure (read as `measFS.pressure`, ±5 mbar full-scale). At constant
  *  flow this signal is intrinsically noisy because:
  *      - centrifugal blowers produce ~1-30 Hz pressure pulsations,
  *      - turbulence around the orifice/Pitot adds broadband 5-100 Hz hash,
  *      - housing vibration couples mechanically into the ΔP ports.
  *
  *  This module owns the conditioning pipeline:
  *
  *      raw ΔP (measFS.pressure)
  *           |
  *           v
  *      [1st-order IIR LPF, fc = 10 Hz default]
  *           |
  *           v
  *      filtered ΔP   (LUT → Q_total; leak → Q_patient)
  *
  *  Bulk flow (leak, breath shape, published Q_patient) uses the 10 Hz ΔP
  *  LPF plus the leak-path breath/pause LPF (2 Hz / 0.35 Hz). FOT must
  *  NOT use published Q_patient: the pause LPF kills the ~4 Hz carrier.
  *  Score FOT from Flow_GetPatientRawSlm() through Flow_FotBpStep()
  *  centered on the oscillator frequency.
  *
  *  All limits and gains are debugger-writable volatile globals so you
  *  can tune the cutoff live with no rebuild.
  ******************************************************************************
  */

#ifndef FLOW_SENSOR_H
#define FLOW_SENSOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/* Cutoff clamps. Below 0.1 Hz the time constant (>1.6 s) is longer than
 * a breath cycle and you start corrupting the breathing waveform. Above
 * 50 Hz the filter starts losing meaning at our 200 Hz sample rate
 * (Nyquist = 100 Hz, but anti-aliasing margin says stay <= fs/4). */
#define FLOW_LPF_CUTOFF_MIN_HZ   0.1f
#define FLOW_LPF_CUTOFF_MAX_HZ   50.0f

/* Reset filter state and load defaults (10 Hz LPF). */
void  Flow_Init(void);

/* Live-tunable cutoff. Clamps inside; pass any value, get a safe one. */
void  Flow_SetLpfCutoff(float hz);
float Flow_GetLpfCutoff(void);

/* Push one new raw ΔP sample (mbar) into the filter. dt_s is the elapsed
 * time since the previous call. Designed to be called once per sensor
 * tick (every 5 ms in SensorTask). After the LPF runs we also recompute
 * the calibrated flow rate so Flow_GetSlm() is fresh every tick. */
/* saturated is the quality flag paired with this fresh sensor sample.
 * A clipped value is a bound, not an exact flow measurement. */
void  Flow_Update(float dp_mbar_raw, float dt_s, bool saturated);

/* Call instead of Flow_Update when the FS read failed or the driver had to
 * repeat its last good sample (AMS5935_WasSampleHeld). Keeps the LPF, the
 * LUT outputs and the auto-zero quiet statistics frozen for that tick so a
 * repeated ΔP is never mistaken for a genuinely quiet signal. */
void  Flow_NoteStaleSample(void);
bool  Flow_IsSampleStale(void);   /* true if the last tick was held */
/* Consecutive held ticks since the last fresh sample; 0 once fresh data
 * arrives. Stale with a run of 0 means no fresh sample since Flow_Init. */
uint32_t Flow_GetStaleRun(void);
/* Quality of the last fresh input; retained across held/conversion-wait ticks.
 * Check IsSampleStale too: a held clipped sample is not fresh evidence. */
bool  Flow_IsSampleSaturated(void);

/* Call once per tick with the published patient flow. Latches a full
 * snapshot on each rising edge above g_flow_spike_thresh_slm so a needle
 * can be attributed (see "flow spike"): ΔP moved = pneumatic/sensor,
 * ΔP flat but zero/leak moved = arithmetic. gap_ms gives the period. */
void  Flow_SpikeWatch(float q_patient_slm, float dt_s);

/* Read the most recent values (no side effect). */
float Flow_GetDpRaw(void);          /* last raw input,        mbar */
float Flow_GetDpFiltered(void);     /* LPF output,            mbar */
float Flow_GetDpFilteredCorr(void); /* LPF output minus zero, mbar */
float Flow_GetSlm(void);            /* active total flow (SLM or BTPS) */
float Flow_GetSlmStd(void);         /* LUT output always as SFM3300 SLM */
float Flow_GetSlmRaw(void);         /* same calc on raw ΔP (active units) */
float Flow_GetSlmCtrl(void);        /* Flow_GetSlm + fan-map estimate when saturated (controller only) */
float Flow_GetSlmRawCtrl(void);     /* Flow_GetSlmRaw, same substitution */
float Flow_GetPatientSlm(void);     /* vent+leak corrected, breath-band LPF */
float Flow_GetPatientRawSlm(void);  /* vent+leak corrected, no breath LPF */

/* Constant-Q bandpass for FOT p-p (RBJ, Q≈2, unity gain at f_center).
 * Own one instance per P and Q channel. Reset when a probe starts.
 * First sample after reset seeds history so a DC step is not a spike. */
typedef struct
{
    float   x1;
    float   x2;
    float   y1;
    float   y2;
    uint8_t seeded;
} FlowFotBp_t;

void  Flow_FotBpReset(FlowFotBp_t *s);
float Flow_FotBpStep(FlowFotBp_t *s, float x, float f_center_hz, float dt_s);

/* === Zero-offset handling ============================================
 *
 *  At zero flow the AMS5935 doesn't read exactly 0 mbar - Round Hole_2
 *  cal sheet zeros at -0.033 mbar / 0 SLM. The offset is part-and-board
 *  specific and drifts a little with temperature, so we expose two ways
 *  to manage it:
 *
 *      (a) Flow_SetZeroOffset(mbar) - apply a known offset (default is
 *          -0.033 mbar from the Round Hole_2 / SFM3300 sheet).
 *      (b) Flow_AutoZero() - start a slew of the offset toward the current
 *          quiet ΔP mean. Never a one-sample snap (near-zero LUT is
 *          ~60 slm/mbar, so a 0.1 mbar twitch is a ~6 slm Q spike).
 *
 *  After offset removal: q_slm = LUT_interp(filtered_dp - zero_offset).
 *  Optional BTPS scaling may then be applied (see below). */
void  Flow_SetZeroOffset(float mbar);
float Flow_GetZeroOffset(void);
void  Flow_AutoZero(void);

/* Idle auto-zero (#51): ON by default. While therapy/blower are idle,
 * accumulate a quiet mean of filtered ΔP (AC peak-peak gate), then SLEW
 * the offset toward that mean at ~0.3 slm/s. No instantaneous snap.
 * One-shot per idle: after settle, holds until Flow_AutoZeroArm()
 * (therapy stop) or USB "flow autozero". */
void  Flow_AutoZeroService(float dt_s, bool allow);
void  Flow_AutoZeroArm(void); /* allow one more idle trim after therapy stop */
/* #84: the zero is limited to g_flow_az_limit_mbar around the factory one. */
float Flow_GetZeroFactory(void);
bool  Flow_IsZeroRejected(void);
float Flow_GetZeroRejectedMbar(void);
void  Flow_SetAutoZeroEnable(bool en);
bool  Flow_IsAutoZeroEnabled(void);
uint32_t Flow_GetAutoZeroCount(void);
uint8_t  Flow_GetAutoZeroLatched(void); /* 1 = settled / hold-off */

/* Diagnostics for "is this needle the zero trim or the sensor?" (flow diag).
 * az_count / az_slewing move only when the offset is being trimmed;
 * stale / median counters move when the AMS5935 read path misbehaves. */
extern volatile uint8_t  g_flow_az_slewing;
extern volatile uint8_t  g_flow_az_quiet_now;
extern volatile float    g_flow_az_last_pp_mbar;
extern volatile float    g_flow_az_done_mbar;        /* settle band, mbar */
extern volatile float    g_flow_az_slew_mbar_per_s;
extern volatile float    g_flow_az_quiet_pp_mbar;
extern volatile uint8_t  g_flow_median_enable;
extern volatile uint8_t  g_flow_median_taps;   /* 3 or 5; 5 rejects 2 in a row */
extern volatile uint32_t g_flow_median_reject_count;
extern volatile float    g_flow_dp_max_slew_mbar_per_s;
extern volatile uint8_t  g_flow_slew_max_reject;
extern volatile uint32_t g_flow_slew_reject_count;
extern volatile float    g_flow_q_max_step_slm;       /* slm per FS sample */
extern volatile uint32_t g_flow_impulse_reject_count;
extern volatile uint32_t g_flow_stale_count;
extern volatile uint32_t g_flow_stale_run_max;

/* Fan-map flow estimate (flow_sensor.c): while the dP sensor is saturated,
 * flow is taken from P_dev = a*N^2 + b*N*Q + c*Q|Q| + d*Q (cmH2O, krpm, L/min).
 * Off by default; "flow set fan_est 1" enables it. */
/* Flow element / LUT: 0 = Round Hole_2 (default), 1 = ~150 slm element
 * (2026-10-05, flow_sensor.c kFlowCalHi). "flow set element 0|1". */
extern volatile uint8_t  g_flow_element;
extern volatile uint8_t  g_flow_fan_est_enable;
extern volatile float    g_flow_fan_a;
extern volatile float    g_flow_fan_b;
extern volatile float    g_flow_fan_c;
extern volatile float    g_flow_fan_d;
extern volatile float    g_flow_fan_est_slm;      /* last estimate, slm */
extern volatile uint8_t  g_flow_fan_est_active;   /* 1 while substituting */

extern volatile float    g_flow_spike_thresh_slm;
extern volatile uint32_t g_flow_spike_count;
extern volatile uint32_t g_flow_ms;
extern volatile uint32_t g_flow_spike_ms;
extern volatile uint32_t g_flow_spike_gap_ms;
extern volatile float    g_flow_spike_q_patient;
extern volatile float    g_flow_spike_dp_raw;
extern volatile float    g_flow_spike_dp_filt;
extern volatile float    g_flow_spike_q_total;
extern volatile float    g_flow_spike_vent;
extern volatile float    g_flow_spike_uninten;
extern volatile float    g_flow_spike_zero;
extern volatile uint32_t g_flow_spike_az_count;
extern volatile uint32_t g_flow_spike_stale;
extern volatile uint32_t g_flow_spike_medrej;

/* === SLM (SFM3300) ↔ BTPS ============================================
 *
 *  The Round Hole_2 LUT is referenced to SFM3300-250-D SLM
 *  (Sensirion standard: dry gas @ 20 °C / 1013.25 mbar).
 *
 *  Altitude / density (default ON):
 *  Orifice ΔP→Q is density-dependent. With ambient absolute pressure P_amb
 *  from measAS (mbar):
 *
 *      Q_SLM ≈ Q_LUT(ΔP) * sqrt(P_amb / 1013.25)
 *
 *  so reported standard flow tracks mass flow at altitude. Pressure PID
 *  still closes on gauge cmH2O and does not need this correction.
 *
 *  BTPS (optional):
 *  When enabled, active flow is then scaled to body conditions
 *  (37 °C, saturated, ambient pressure from measAS.pressure):
 *
 *      Q_BTPS = Q_SLM * (310.15/293.15) * (P_amb / (P_amb - 62.67))
 *
 *  SensorTask refreshes P_amb from measAS.pressure at ~10 Hz.
 *  Flow_SetBtpsAmbientMbar() remains as a USB/debug override between AS samples.
 *  Therapy, leak, and USB telemetry all consume Flow_GetSlm() so the
 *  toggle applies end-to-end. Flow_GetSlmStd() is density-corrected SLM. */
void  Flow_SetBtpsEnable(bool en);
bool  Flow_IsBtpsEnabled(void);
void  Flow_SetBtpsAmbientMbar(float mbar);
float Flow_GetBtpsAmbientMbar(void);
float Flow_GetBtpsFactor(void);
float Flow_SlmToBtps(float q_slm_std);

/* Altitude / density compensation on LUT→SLM (independent of BTPS). */
void  Flow_SetAltitudeEnable(bool en);
bool  Flow_IsAltitudeEnabled(void);
float Flow_GetAltitudeFactor(void);     /* sqrt(P_amb/P_ref), 1.0 if disabled */
float Flow_GetAltitudeMeters(void);     /* barometric estimate from P_amb   */
float Flow_GetAltitudeRefMbar(void);

/* Standalone calibration evaluator: pure function, no state. Useful if
 * you want to apply the cal curve to a different ΔP source (e.g., the
 * raw band-passed FOT signal once impedance computation is wired).
 * Returns SFM3300 SLM (no BTPS). */
float Flow_DpToSlm(float dp_corrected_mbar);

#ifdef __cplusplus
}
#endif

#endif /* FLOW_SENSOR_H */
