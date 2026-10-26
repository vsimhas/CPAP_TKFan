/**
  ******************************************************************************
  * @file    telemetry_stream.c
  * @brief   Live pressure / patient-flow CSV stream over USB CDC VCP.
  ******************************************************************************
  */

#include "telemetry_stream.h"

#if TELEM_ENABLED

#include <stdio.h>
#include <stdlib.h>
#include <stddef.h>
#include <string.h>

#include "stm32h7xx_hal.h"
#include "cmsis_compiler.h"
#include "therapy.h"
#include "blower_ipc.h"
#include "blower_ctrl.h"
#include "apap_ctrl.h"
#include "epr_ctrl.h"
#include "leak_estimator.h"
#include "flow_sensor.h"
#include "pressure_sensors.h"   /* hamsFS saturation flag on the B line */
#include "tube_comp.h"
#include "bench_playback.h"
#include "physiology_telem.h"
#include "session_edf_log.h"
#include "rtc_clock.h"
#include "session_summary.h"
#include "adherence.h"

#include "usbd_def.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

extern USBD_HandleTypeDef hUsbDeviceFS;
extern volatile uint8_t sd_card_present;
extern volatile uint8_t sd_fatfs_ready;
extern volatile HAL_StatusTypeDef sd_test_result;

#ifndef TELEM_BUF_SIZE
/* 2026-09-22: 2048 -> 8192. With the terms line enabled the stream is a ~40 B
 * T line plus a ~102 B D line every 20 ms, about 7.1 kB/s, so 2048 bytes held
 * less than 300 ms of slack and any USB hiccup overran it. Measured corruption
 * climbed 2.9% -> 13.4% -> 18.0% of D lines as that line grew. */
#define TELEM_BUF_SIZE   8192U
#endif

#ifndef TELEM_TX_CHUNK
#define TELEM_TX_CHUNK   64U
#endif

#ifndef TELEM_FORMAT_BUF
/* Holds one formatted T or D line. The D line grew five physics-FF fields on
 * 2026-09-22 and no longer fits in 64. Note the failure mode if this is ever
 * too small again: Telem_PushSample() checks the snprintf return against
 * sizeof(line) and simply DROPS the line, so the symptom is a silently
 * missing D stream rather than anything crashing.
 * 128 -> 160 on 2026-09-29: the B line gained vq/vd and, with its cumulative
 * counters grown, could pass 128 and vanish the same way.
 * 160 -> 176 on 2026-10-01 for the B line's iq_lim_ma. */
#define TELEM_FORMAT_BUF 176U
#endif

#if (TELEM_BUF_SIZE & (TELEM_BUF_SIZE - 1U)) != 0U
#error "TELEM_BUF_SIZE must be a power of two"
#endif

#define TELEM_BUF_MASK   (TELEM_BUF_SIZE - 1U)
#define TELEM_CMD_QUEUE_LEN  96U

volatile uint8_t  g_telem_enabled      = 1U;
/* Emit the "D," controller-term line alongside "T,". Off by default: it
 * roughly doubles the stream, and it exists for diagnosis, not routine use. */
volatile uint8_t  g_telem_terms_enable = 0U;

/* Motor electrical line. Off by default: the stream already runs ~7 kB/s with
 * the terms line on and drops D lines under that, so this emits at half rate
 * (25 Hz) and only when asked. Enable with: blower set telem_motor 1 */
volatile uint8_t  g_telem_motor_enable = 0U;
volatile uint32_t g_telem_sample_count = 0U;
volatile uint32_t g_telem_dropped_count = 0U;

extern volatile float   g_blower_ctrl_kp_low;
extern volatile float   g_blower_ctrl_ki_low;
extern volatile float   g_blower_ctrl_kd_low;
extern volatile float   g_blower_ctrl_kp_mid;
extern volatile float   g_blower_ctrl_ki_mid;
extern volatile float   g_blower_ctrl_kd_mid;
extern volatile float   g_blower_ctrl_kp_high;
extern volatile float   g_blower_ctrl_ki_high;
extern volatile float   g_blower_ctrl_kd_high;
extern volatile uint8_t g_blower_ctrl_ff_enable;
extern volatile float   g_blower_ctrl_ff_k_rpm_per_sqrtcmh;
extern volatile float   g_blower_ctrl_ff_offset_rpm;
extern volatile float   g_blower_ctrl_ff_offset_slope_rpm_per_cmh2o;
extern volatile uint8_t g_blower_ctrl_flow_ff_enable;
extern volatile int8_t  g_blower_ctrl_flow_ff_inhale_sign;
extern volatile uint8_t g_blower_ctrl_dip_ff_enable;
extern volatile uint8_t g_blower_ctrl_physics_ff_enable;
extern volatile uint8_t g_blower_ctrl_physics_ff_use_total_flow;
extern volatile uint8_t g_blower_ctrl_physics_ff_exhale_mit_enable;
extern volatile uint8_t g_blower_ctrl_integ_hold;
extern volatile uint8_t g_blower_ctrl_integ_headroom_hold_enable;
extern volatile float   g_blower_ctrl_physics_ff_c_fan_cmh2o_per_slm2;
extern volatile float   g_blower_ctrl_physics_ff_c_fan_exp_cmh2o_per_slm2;
extern volatile float   g_blower_ctrl_physics_ff_hose_r_lam;
extern volatile float   g_blower_ctrl_physics_ff_hose_r_turb;
extern volatile float   g_blower_ctrl_physics_ff_flow_lpf_hz;
extern volatile float   g_blower_ctrl_physics_ff_flow_lpf_inhale_hz;
extern volatile float   g_blower_ctrl_physics_ff_flow_lpf_exhale_hz;
extern volatile float   g_blower_ctrl_physics_ff_onset_confirm_slm;
extern volatile uint8_t g_blower_ctrl_physics_ff_inhale_mit_enable;
extern volatile uint8_t g_blower_ctrl_physics_ff_onset_enable;
extern volatile float   g_blower_ctrl_physics_ff_onset_k_rpm_per_slm_per_s;
extern volatile float   g_blower_ctrl_physics_ff_onset_thresh_slm_per_s;
extern volatile float   g_blower_ctrl_physics_ff_onset_max_rpm;
extern volatile uint8_t g_blower_ctrl_physics_ff_exp_lead_enable;
extern volatile float   g_blower_ctrl_physics_ff_exp_lead_k_rpm_per_slm_per_s;
extern volatile float   g_blower_ctrl_physics_ff_exp_lead_thresh_slm_per_s;
extern volatile float   g_blower_ctrl_physics_ff_exp_lead_max_rpm;
extern volatile float   g_blower_ctrl_physics_ff_onset_dqdt_tau_s;
extern volatile uint8_t g_blower_ctrl_physics_ff_onset_bypass_slew;
extern volatile float   g_blower_ctrl_physics_ff_inhale_rise_boost;
extern volatile float   g_blower_ctrl_physics_ff_rise_rpm_per_s;
extern volatile float   g_blower_ctrl_physics_ff_fall_rpm_per_s;
extern volatile float   g_blower_ctrl_physics_ff_fall_exhale_rpm_per_s;
extern volatile uint8_t g_blower_ctrl_physics_ff_exp_tail_hold_enable;
extern volatile float   g_blower_ctrl_physics_ff_exp_tail_flow_slm;
extern volatile float   g_blower_ctrl_physics_ff_exp_tail_fall_rpm_per_s;
extern volatile float   g_blower_ctrl_physics_ff_exp_tail_hold_cmh2o;
extern volatile float   g_blower_ctrl_hybrid_exp_hold_band_cmh2o;
extern volatile float   g_blower_ctrl_hybrid_exp_fall_rpm_per_s;
extern volatile float   g_blower_ctrl_hybrid_exp_boost_rpm_per_cmh2o;
extern volatile float   g_blower_ctrl_hybrid_exp_boost_rpm_per_s;
extern volatile float   g_blower_ctrl_under_p_gain;
extern volatile float   g_blower_ctrl_under_p_thresh_cmh2o;
extern volatile float   g_blower_ctrl_flow_ff_k_rpm_per_slm;
extern volatile float   g_blower_ctrl_flow_ff_max_rpm;
extern volatile float   g_blower_ctrl_flow_ff_deadband_slm;
extern volatile float   g_blower_ctrl_flow_ff_rise_rpm_per_s;
extern volatile float   g_blower_ctrl_flow_ff_fall_rpm_per_s;
extern volatile float   g_blower_ctrl_flow_ff_onset_k_rpm_per_slm_per_s;
extern volatile float   g_blower_ctrl_flow_ff_onset_thresh_slm_per_s;
extern volatile float   g_blower_ctrl_flow_ff_onset_max_rpm;
extern volatile uint8_t g_blower_ctrl_flow_ff_onset_enable;
extern volatile float   g_blower_ctrl_dip_ff_k_rpm_per_cmh2o_per_s;
extern volatile float   g_blower_ctrl_dip_ff_thresh_cmh2o_per_s;
extern volatile float   g_blower_ctrl_dip_ff_max_rpm;
extern volatile float   g_blower_ctrl_dip_ff_err_min_cmh2o;
extern volatile int8_t  g_blower_ctrl_flow_ff_inhale_sign;
extern volatile float   g_blower_ctrl_over_p_gain;
extern volatile float   g_blower_ctrl_over_p_thresh_cmh2o;
extern volatile float   g_blower_ctrl_over_p_thresh_exp_cmh2o;
extern volatile float   g_blower_ctrl_p_err_lpf_hz;
extern volatile float   g_blower_ctrl_over_p_exp_window_s;
extern volatile float   g_blower_ctrl_start_ramp_rpm_per_s;
extern volatile float   g_blower_ctrl_over_brake_rpm_per_s;
extern volatile float   g_blower_ctrl_slew_cmh2o_per_s;
extern volatile float   g_blower_ctrl_slew_up_cmh2o_per_s;
extern volatile float   g_blower_ctrl_out_filter_alpha;
extern volatile float   g_blower_ctrl_integ_deadband_cmh2o;
extern volatile float   g_blower_ctrl_integ_max_abs_rpm;
extern volatile float   g_blower_ctrl_meas_filter_alpha;
extern volatile float   g_blower_ctrl_deriv_filter_alpha;
extern volatile float   g_leak_patient_lpf_hz;
extern volatile uint8_t g_leak_patient_quiet_lpf_enable;
extern volatile float   g_leak_patient_quiet_lpf_hz;
extern volatile float   g_leak_patient_quiet_band_slm;
extern volatile float   g_leak_patient_quiet_exit_slm;
extern volatile uint8_t g_leak_patient_pause_zero_enable;
extern volatile float   g_leak_vivo_patient_lpf_hz;
extern volatile uint8_t g_physio_fot_src;
extern volatile float   g_physio_last_r_eff;
extern volatile float   g_physio_last_c_proxy_ml_cmh2o;
extern volatile float   g_physio_last_p_pp_cmh2o;
extern volatile float   g_physio_last_q_pp_slm;
extern volatile int32_t g_blower_ctrl_rpm_max;
extern volatile uint8_t g_blower_ctrl_enabled;
extern volatile uint8_t g_blower_ctrl_pid_enable;
extern volatile uint8_t g_blower_ctrl_rpm_floor_enable;
extern volatile uint8_t g_blower_ctrl_ff_ss_floor_enable;
extern volatile uint8_t g_blower_ctrl_exhale_integ_clamp_enable;
extern volatile uint8_t g_blower_ctrl_vent_floor_enable;
extern volatile uint8_t g_blower_ctrl_exp_neg_i_clamp_enable;
extern volatile uint8_t g_blower_ctrl_rise_ff_enable;
extern volatile float   g_blower_ctrl_rise_ff_k_rpm_per_cmh2o_per_s;
extern volatile float   g_blower_ctrl_rise_ff_thresh_cmh2o_per_s;
extern volatile float   g_blower_ctrl_rise_ff_max_rpm;
extern volatile float   g_blower_ctrl_rise_ff_err_min_cmh2o;
extern volatile uint8_t g_blower_ctrl_exp_flow_ff_enable;
extern volatile uint8_t g_blower_ctrl_integ_err_lpf_enable;
extern volatile float   g_blower_ctrl_integ_err_lpf_tau_s;
extern volatile float   g_blower_ctrl_integ_err_lpf_tau_start_s;
extern volatile float   g_blower_ctrl_integ_err_lpf_warmup_s;
extern volatile float   g_blower_ctrl_integ_err_lpf_settle_band_cmh2o;
extern volatile float   g_blower_ctrl_exp_flow_ff_k_rpm_per_slm;
extern volatile float   g_blower_ctrl_exp_flow_ff_lead_s;
extern volatile float   g_blower_ctrl_exp_flow_ff_max_rpm;
/* Controller term decomposition, for attributing pressure-waveform features
 * to the block that caused them. Emitted on a separate "D," line so the
 * existing "T," parser is untouched. */
extern volatile float   g_blower_ctrl_ff_term;
extern volatile float   g_blower_ctrl_physics_ff_p_equiv_cmh2o;
extern volatile float   g_blower_ctrl_physics_ff_hose_drop_cmh2o;
extern volatile float   g_blower_ctrl_physics_ff_fan_load_cmh2o;
extern volatile float   g_blower_ctrl_physics_ff_q_filt_slm;
extern volatile float   g_blower_ctrl_physics_ff_rpm_raw;
extern volatile float   g_blower_ctrl_ff_dip_term;
extern volatile float   g_blower_ctrl_ff_rise_term;
extern volatile float   g_blower_ctrl_ff_exp_flow_term;
extern volatile float   g_blower_ctrl_p_term;
extern volatile float   g_blower_ctrl_i_term;
extern volatile float   g_blower_ctrl_d_term;
extern volatile float   g_blower_ctrl_error_cmh2o;

extern volatile int8_t  g_epr_insp_polarity;
extern volatile int8_t  g_epr_insp_polarity_cfg;
extern volatile float   g_epr_insp_polarity_evidence;
extern volatile uint8_t g_epr_insp_polarity_mismatch;
extern volatile uint8_t g_blower_ctrl_integ_deadband_enable;

/* APAP tunables (apap_ctrl.c) */
extern volatile float   g_apap_no_breath_timeout_s;
extern volatile float   g_apap_breath_window_s;
extern volatile float   g_apap_resume_min_peak_slm;
extern volatile float   g_apap_resume_peak_ratio;
extern volatile float   g_apap_resume_min_floor_slm;
extern volatile float   g_apap_fl_baseline_reject_ratio;
extern volatile float   g_apap_baseline_restore_max_s;
extern volatile float   g_apap_ca_reclass_window_s;
extern volatile uint8_t g_apap_leak_freeze_min_breaths;
extern volatile float   g_apap_leak_freeze_max_s;
extern volatile uint8_t g_apap_breath_notch_enable;
extern volatile float   g_apap_hyp_ca_guard_s;
extern volatile float   g_apap_hyp_rearm_ratio;
extern volatile uint8_t g_apap_hyp_rearm_breaths;
extern volatile float    g_epr_apap_breath_min_vol_ratio;
extern volatile uint8_t  g_epr_rel_baseline_enable;
extern volatile float    g_epr_baseline_slow_tau_s;
extern volatile uint8_t  g_apap_rel_baseline_enable;
extern volatile uint32_t g_epr_apap_breath_min_insp_ms;
extern volatile float    g_epr_apap_breath_min_insp_ml;
extern volatile float   g_apap_startup_grace_s;
extern volatile float   g_apap_osc_amplitude_cmh2o;
extern volatile float   g_apap_osc_amplitude_max_cmh2o;
extern volatile float   g_apap_osc_freq_hz;
extern volatile float   g_apap_fot_eval_period_s;
extern volatile float   g_apap_osc_min_p_pp_cmh2o;
extern volatile float   g_apap_osa_flow_pp_thresh_slm;
extern volatile float   g_apap_apnea_min_s;
extern volatile float   g_apap_oa_raise_cmh2o_per_s;
extern volatile float   g_apap_oa_raise_max_cmh2o;
extern volatile float   g_apap_her_oa_above_cmh2o;
extern volatile float   g_apap_her_oa_above_scale;
extern volatile float   g_apap_raise_rate_standard_cmh2o_per_s;
extern volatile float   g_apap_raise_rate_soft_cmh2o_per_s;
extern volatile float   g_apap_raise_rate_her_scale;
extern volatile uint8_t g_apap_response_soft;
extern volatile uint8_t g_apap_fl_enable;
extern volatile float   g_apap_fl_ratio;
extern volatile uint8_t g_apap_fl_breaths_needed;
extern volatile uint8_t g_apap_fl_clear_breaths;
extern volatile float   g_apap_fl_raise_scale;
extern volatile float   g_apap_fl_min_peak_slm;
extern volatile float   g_apap_fl_baseline_alpha;
extern volatile float   g_apap_fl_flat_ratio;
extern volatile float   g_apap_fl_dip_ratio;
extern volatile float   g_apap_fl_seg_floor_slm;
extern volatile uint8_t g_apap_fl_amp_arm_enable;
extern volatile uint8_t g_apap_cpap_scoring_enable;
extern volatile float   g_apap_hyp_raise_shape_frac;
extern volatile uint8_t g_apap_hyp_enable;
extern volatile float   g_apap_hyp_ratio;
extern volatile uint8_t g_apap_hyp_breaths_needed;
extern volatile float   g_apap_hyp_min_duration_s;
extern volatile float   g_apap_hyp_raise_scale;
extern volatile float   g_apap_hyp_hold_s;
extern volatile uint8_t g_apap_snore_enable;
extern volatile float   g_apap_snore_rms_thresh_slm;
extern volatile float   g_apap_snore_on_s;
extern volatile float   g_apap_snore_off_s;
extern volatile float   g_apap_snore_raise_scale;
extern volatile float   g_apap_snore_hp_hz;
extern volatile uint8_t g_apap_lower_enable;
extern volatile float   g_apap_lower_rate_cmh2o_per_s;
extern volatile float   g_apap_stable_before_lower_s;
extern volatile uint8_t g_apap_leak_gate_enable;

static uint8_t            s_buf[TELEM_BUF_SIZE];
static volatile uint32_t  s_head;
static volatile uint32_t  s_tail;
static uint8_t            s_tx_chunk[TELEM_TX_CHUNK];
static volatile uint8_t   s_inited = 0U;
static char               s_rx_line[96];
static uint8_t            s_rx_len = 0U;
static char               s_cmd_queue[TELEM_CMD_QUEUE_LEN][96];
static volatile uint8_t   s_cmd_head = 0U;
static volatile uint8_t   s_cmd_tail = 0U;
/* Suppress Telem_PushStatus while draining queued therapy-set cmds. */
static uint8_t            s_telem_quiet = 0U;

static void Telem_BufPush(const uint8_t *data, uint32_t len);
static uint32_t Telem_BufPopChunk(uint8_t *dst, uint32_t maxLen);
static void Telem_QueueCommand(const char *cmd);

static void Telem_PushRaw(const char *text)
{
  size_t len;

  if (text == NULL)
  {
    return;
  }

  len = strnlen(text, 256U);
  Telem_BufPush((const uint8_t *)text, (uint32_t)len);
}

void Telem_PushNote(const char *text)
{
  Telem_PushRaw(text);
}

static void Telem_HandleCommand(const char *cmd)
{
  if (cmd == NULL) { return; }

  if ((strcmp(cmd, "telem on") == 0) || (strcmp(cmd, "telem 1") == 0))
  {
    g_telem_enabled = 1U;
    Telem_PushRaw("# telem on\r\n");
  }
  else if ((strcmp(cmd, "telem off") == 0) || (strcmp(cmd, "telem 0") == 0))
  {
    g_telem_enabled = 0U;
    Telem_PushRaw("# telem off\r\n");
  }
  else
  {
    Telem_QueueCommand(cmd);
  }
}

static void Telem_QueueCommand(const char *cmd)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  uint8_t next = (uint8_t)((s_cmd_head + 1U) % TELEM_CMD_QUEUE_LEN);
  if (next == s_cmd_tail)
  {
    if (primask == 0U)
    {
      __enable_irq();
    }
    Telem_PushRaw("# err cmd queue full\r\n");
    return;
  }

  (void)strncpy(s_cmd_queue[s_cmd_head], cmd, sizeof(s_cmd_queue[0]) - 1U);
  s_cmd_queue[s_cmd_head][sizeof(s_cmd_queue[0]) - 1U] = '\0';
  s_cmd_head = next;

  if (primask == 0U)
  {
    __enable_irq();
  }
}

void Telem_OnRx(const uint8_t *data, uint32_t len)
{
  if ((data == NULL) || (len == 0U)) { return; }

  for (uint32_t i = 0U; i < len; i++)
  {
    char c = (char)data[i];
    if ((c == '\r') || (c == '\n'))
    {
      if (s_rx_len > 0U)
      {
        s_rx_line[s_rx_len] = '\0';
        Telem_HandleCommand(s_rx_line);
        s_rx_len = 0U;
      }
    }
    else if (s_rx_len < (sizeof(s_rx_line) - 1U))
    {
      s_rx_line[s_rx_len++] = c;
    }
    else
    {
      s_rx_len = 0U;
    }
  }
}

static void Telem_PushAck(const char *cmd)
{
  char line[112];
  int n = snprintf(line, sizeof(line), "# ack %s\r\n", cmd);
  if ((n > 0) && ((size_t)n < sizeof(line)))
  {
    Telem_BufPush((const uint8_t *)line, (uint32_t)n);
  }
}

static void Telem_PushErr(const char *reason)
{
  char line[80];
  int n = snprintf(line, sizeof(line), "# err %s\r\n", reason);
  if ((n > 0) && ((size_t)n < sizeof(line)))
  {
    Telem_BufPush((const uint8_t *)line, (uint32_t)n);
  }
}

static void Telem_PushStatus(void)
{
  if (s_telem_quiet != 0U)
  {
    return;
  }
  static char line[1024];
  uint32_t now = HAL_GetTick();
  const uint8_t mode = Therapy_GetMode();
  /* APAP fields whenever its detectors run: AutoSet, or CPAP scoring (#33). */
  const bool autoset = (mode != (uint8_t)THERAPY_MODE_CPAP) || Apap_IsScoreOnly();
  int n = snprintf(line, sizeof(line),
                   "S,%lu,%u,%.2f,%.2f,%.2f,%.2f,%u,%u,%u,%u,%.2f,%.2f"
                   ",%u,%u,%.3f,%.3f,%.3f,%u,%.3f,%u,%u,%.2f,%u"
                   ",%u,%ld,%ld,%.2f,%.2f,%.2f,%.2f,%.1f,%u"
                   ",%.1f,%.0f,%.0f,%.0f"
                   ",%u,%.1f,%u,%u,%.2f,%.2f,%.1f,%.1f,%lu,%lu,%lu,%lu"
                   ",%.2f,%.4f,%.0f"
                   ",%.2f,%.2f,%.2f,%.2f,%.2f,%.2f,%lu"
                   ",%u,%.2f,%.2f,%lu,%u"
                   ",%u,%u,%u,%lu,%lu,%lu,%lu,%lu,%lu,%.2f,%.2f,%.2f"
                   ",%u,%.3f,%.1f,%.3f,%.3f\r\n",
                   (unsigned long)now,
                   (unsigned)Therapy_GetState(),
                   (double)Therapy_GetTargetPressure(),
                   (double)Therapy_GetSetpointCmh2o(),
                   (double)Therapy_GetMeasuredCmh2o(),
                   (double)Therapy_GetLeakSlm(),
                   Therapy_IsEprEnabled() ? 1U : 0U,
                   Therapy_IsMaskOffActive() ? 1U : 0U,
                   (unsigned)Therapy_GetFault(),
                   (unsigned)mode,
                   (double)Therapy_GetApapMinPressure(),
                   (double)Therapy_GetApapMaxPressure(),
                   autoset ? (unsigned)Apap_GetState() : 0U,
                   autoset && Apap_IsOscillating() ? 1U : 0U,
                   autoset ? (double)Apap_GetFotPressurePpCmh2o() : 0.0,
                   autoset ? (double)Apap_GetFotFlowPpSlm() : 0.0,
                   autoset ? (double)Apap_GetFotImpedanceCmh2oPerSlm() : 0.0,
                   autoset && Apap_IsOsaLikely() ? 1U : 0U,
                   autoset ? (double)Apap_GetOsaFlowPpThreshSlm() : 0.0,
                   autoset && Apap_IsOsaTreating() ? 1U : 0U,
                   (unsigned)Therapy_GetEprLevel(),
                   (double)Epr_GetActiveReliefCmh2o(),
                   (unsigned)Epr_GetPhase(),
                   (unsigned)Leak_GetAlgo(),
                   (long)Leak_GetMeanRatio(),
                   (long)Leak_GetMeanRatio10Sec(),
                   (double)Leak_GetVentSlm(),
                   (double)Leak_GetUnintentionalSlm(),
                   (double)Leak_GetFlowLeakSlm(),
                   (double)Leak_GetReportedLeakSlm(),
                   (double)Leak_GetRestVolumeMl(),
                   (unsigned)Leak_GetBreathInspiration(),
                   (double)Leak_GetBreathRateBpm(),
                   (double)Leak_GetLastInspVolumeMl(),
                   (double)Leak_GetLastExpVolumeMl(),
                   (double)Leak_GetPhaseVolumeMl(),
                   (unsigned)Leak_GetSeverity(),
                   (double)Leak_GetPercentOfVent(),
                   Therapy_IsLeakLow() ? 1U : 0U,
                   Leak_IsSessionActive() ? 1U : 0U,
                   (double)Leak_GetSessionAvgSlm(),
                   (double)Leak_GetSessionMaxSlm(),
                   (double)Leak_GetSessionHighTimeS(),
                   (double)Leak_GetSessionLowTimeS(),
                   (unsigned long)Leak_GetHealthHighSamples(),
                   (unsigned long)Leak_GetHealthRatioClampCount(),
                   (unsigned long)Therapy_GetMaskOffEnterCount(),
                   (unsigned long)Therapy_GetMaskOnCount(),
                   (double)Flow_GetBtpsAmbientMbar(),
                   (double)Flow_GetAltitudeFactor(),
                   (double)Flow_GetAltitudeMeters(),
                   (double)Therapy_GetSessionPressureAvg(),
                   (double)Therapy_GetSessionPressureMin(),
                   (double)Therapy_GetSessionPressureMax(),
                   (double)Therapy_GetSessionPressureP50(),
                   (double)Therapy_GetSessionPressureP90(),
                   (double)Therapy_GetSessionPressureP95(),
                   (unsigned long)Therapy_GetSessionPressureSamples(),
                   autoset && Apap_IsFlowLimited() ? 1U : 0U,
                   autoset ? (double)Apap_GetFlPeakBaselineSlm() : 0.0,
                   autoset ? (double)Apap_GetFlLastPeakSlm() : 0.0,
                   autoset ? (unsigned long)Apap_GetStableMs() : 0UL,
                   autoset && Apap_IsLowering() ? 1U : 0U,
                   autoset && Apap_IsHypopneaActive() ? 1U : 0U,
                   autoset && Apap_IsSnoreActive() ? 1U : 0U,
                   autoset && Apap_IsRaiseBlockedByLeak() ? 1U : 0U,
                   autoset ? (unsigned long)Apap_GetEventOaCount() : 0UL,
                   autoset ? (unsigned long)Apap_GetEventCaCount() : 0UL,
                   autoset ? (unsigned long)Apap_GetEventHypCount() : 0UL,
                   autoset ? (unsigned long)Apap_GetEventFlCount() : 0UL,
                   autoset ? (unsigned long)Apap_GetEventSnoreCount() : 0UL,
                   autoset ? (unsigned long)Apap_GetTherapyMs() : 0UL,
                   autoset ? (double)Apap_GetSessionAhi() : 0.0,
                   autoset ? (double)Apap_GetSessionRdi() : 0.0,
                   autoset ? (double)Apap_GetSessionSnoreIndex() : 0.0,
                   (unsigned)g_physio_fot_src,
                   (double)g_physio_last_r_eff,
                   (double)g_physio_last_c_proxy_ml_cmh2o,
                   (double)g_physio_last_p_pp_cmh2o,
                   (double)g_physio_last_q_pp_slm);
  if ((n > 0) && ((size_t)n < sizeof(line)))
  {
    Telem_BufPush((const uint8_t *)line, (uint32_t)n);
  }
}

static int Telem_ParseFloat(const char *text, float *out)
{
  char *end = NULL;
  float v = strtof(text, &end);
  if ((end == text) || (*end != '\0'))
  {
    return 0;
  }
  *out = v;
  return 1;
}

static int Telem_ParseLong(const char *text, long *out)
{
  char *end = NULL;
  long v = strtol(text, &end, 10);
  if ((end == text) || (*end != '\0'))
  {
    return 0;
  }
  *out = v;
  return 1;
}

static int Telem_SetFloatParam(const char *name, float value)
{
  if (strcmp(name, "kp_low") == 0) { g_blower_ctrl_kp_low = value; return 1; }
  if (strcmp(name, "ki_low") == 0) { g_blower_ctrl_ki_low = value; return 1; }
  if (strcmp(name, "kd_low") == 0) { g_blower_ctrl_kd_low = value; return 1; }
  if (strcmp(name, "kp_mid") == 0) { g_blower_ctrl_kp_mid = value; return 1; }
  if (strcmp(name, "ki_mid") == 0) { g_blower_ctrl_ki_mid = value; return 1; }
  if (strcmp(name, "kd_mid") == 0) { g_blower_ctrl_kd_mid = value; return 1; }
  if (strcmp(name, "kp_high") == 0) { g_blower_ctrl_kp_high = value; return 1; }
  if (strcmp(name, "ki_high") == 0) { g_blower_ctrl_ki_high = value; return 1; }
  if (strcmp(name, "kd_high") == 0) { g_blower_ctrl_kd_high = value; return 1; }

  if (strcmp(name, "ff_k") == 0) { g_blower_ctrl_ff_k_rpm_per_sqrtcmh = value; return 1; }
  /* Constant RPM term in the sqrt/physics FF. Use when the model error at
   * steady state is an offset rather than a gain (fit K and offset across
   * several setpoints before trusting either). */
  if (strcmp(name, "ff_offset") == 0) { g_blower_ctrl_ff_offset_rpm = value; return 1; }
  /* Slope of the same offset, rpm per cmH2O of setpoint. 0 restores the old
   * constant-offset behaviour. Fit it together with ff_offset. */
  if (strcmp(name, "ff_offset_slope") == 0)
  {
    if (value < 0.0f) { value = 0.0f; }
    if (value > 600.0f) { value = 600.0f; }
    g_blower_ctrl_ff_offset_slope_rpm_per_cmh2o = value;
    return 1;
  }
  /* Clamp to the model's own ceiling, not a literal - see the note on
   * BLOWER_CTRL_C_FAN_MAX in blower_ctrl.h for why this matters. */
  if (strcmp(name, "physics_ff_c_fan") == 0)
  {
    if (value < 0.0f) { value = 0.0f; }
    if (value > BLOWER_CTRL_C_FAN_MAX) { value = BLOWER_CTRL_C_FAN_MAX; }
    g_blower_ctrl_physics_ff_c_fan_cmh2o_per_slm2 = value;
    return 1;
  }
  /* Expiratory side of the fan-load term (below the resting flow); same
   * clamp. Set equal to physics_ff_c_fan for the old single-c_fan model. */
  if (strcmp(name, "physics_ff_c_fan_exp") == 0)
  {
    if (value < 0.0f) { value = 0.0f; }
    if (value > BLOWER_CTRL_C_FAN_MAX) { value = BLOWER_CTRL_C_FAN_MAX; }
    g_blower_ctrl_physics_ff_c_fan_exp_cmh2o_per_slm2 = value;
    return 1;
  }
  /* The physics FF's own hose basis (fitted together with c_fan), separate
   * from tube comp's R_lam/R_turb. Same clamps as TubeComp_SetCoefficients. */
  if (strcmp(name, "physics_ff_hose_r_lam") == 0)
  {
    if (value < 0.0f)  { value = 0.0f; }
    if (value > 0.05f) { value = 0.05f; }
    g_blower_ctrl_physics_ff_hose_r_lam = value;
    return 1;
  }
  if (strcmp(name, "physics_ff_hose_r_turb") == 0)
  {
    if (value < 0.0f)   { value = 0.0f; }
    if (value > 0.001f) { value = 0.001f; }
    g_blower_ctrl_physics_ff_hose_r_turb = value;
    return 1;
  }
  if (strcmp(name, "physics_ff_flow_lpf_hz") == 0)
  {
    if (value < 0.5f) { value = 0.5f; }
    if (value > 15.0f) { value = 15.0f; }
    g_blower_ctrl_physics_ff_flow_lpf_hz = value;
    return 1;
  }
  if (strcmp(name, "physics_ff_flow_lpf_inhale_hz") == 0)
  {
    if (value < 0.5f) { value = 0.5f; }
    if (value > 25.0f) { value = 25.0f; }
    g_blower_ctrl_physics_ff_flow_lpf_inhale_hz = value;
    return 1;
  }
  if (strcmp(name, "physics_ff_rise") == 0)
  {
    if (value < 100.0f) { value = 100.0f; }
    if (value > 20000.0f) { value = 20000.0f; }
    g_blower_ctrl_physics_ff_rise_rpm_per_s = value;
    return 1;
  }
  if (strcmp(name, "physics_ff_fall") == 0)
  {
    if (value < 100.0f) { value = 100.0f; }
    if (value > 20000.0f) { value = 20000.0f; }
    g_blower_ctrl_physics_ff_fall_rpm_per_s = value;
    return 1;
  }
  if (strcmp(name, "physics_ff_fall_exhale") == 0)
  {
    if (value < 100.0f) { value = 100.0f; }
    if (value > 25000.0f) { value = 25000.0f; }
    g_blower_ctrl_physics_ff_fall_exhale_rpm_per_s = value;
    return 1;
  }
  if (strcmp(name, "physics_ff_exp_tail_flow_slm") == 0)
  {
    if (value < 0.5f) { value = 0.5f; }
    if (value > 10.0f) { value = 10.0f; }
    g_blower_ctrl_physics_ff_exp_tail_flow_slm = value;
    return 1;
  }
  if (strcmp(name, "physics_ff_exp_tail_fall") == 0)
  {
    /* rpm/s the FF may fall to its target in the tail hold; 0 = hold. */
    if (value < 0.0f)     { value = 0.0f; }
    if (value > 20000.0f) { value = 20000.0f; }
    g_blower_ctrl_physics_ff_exp_tail_fall_rpm_per_s = value;
    return 1;
  }
  if (strcmp(name, "physics_ff_exp_tail_hold_cmh2o") == 0)
  {
    if (value < 0.02f) { value = 0.02f; }
    if (value > 0.30f) { value = 0.30f; }
    g_blower_ctrl_physics_ff_exp_tail_hold_cmh2o = value;
    return 1;
  }
  if (strcmp(name, "physics_ff_exp_tail_hold_enable") == 0)
  {
    g_blower_ctrl_physics_ff_exp_tail_hold_enable = (value != 0L) ? 1U : 0U;
    return 1;
  }
  if (strcmp(name, "physics_ff_flow_lpf_exhale_hz") == 0)
  {
    if (value < 0.5f) { value = 0.5f; }
    if (value > 25.0f) { value = 25.0f; }
    g_blower_ctrl_physics_ff_flow_lpf_exhale_hz = value;
    return 1;
  }
  if (strcmp(name, "physics_ff_onset_confirm") == 0)
  {
    /* L/min that confirms a phase change before the FF onset snaps; 0 = off. */
    if (value < 0.0f)  { value = 0.0f; }
    if (value > 30.0f) { value = 30.0f; }
    g_blower_ctrl_physics_ff_onset_confirm_slm = value;
    return 1;
  }
  if (strcmp(name, "physics_ff_use_total_flow") == 0)
  {
    g_blower_ctrl_physics_ff_use_total_flow = (value != 0L) ? 1U : 0U;
    return 1;
  }
  if (strcmp(name, "physics_ff_exhale_mit_enable") == 0)
  {
    g_blower_ctrl_physics_ff_exhale_mit_enable = (value != 0L) ? 1U : 0U;
    return 1;
  }
  if (strcmp(name, "physics_ff_inhale_mit_enable") == 0)
  {
    g_blower_ctrl_physics_ff_inhale_mit_enable = (value != 0L) ? 1U : 0U;
    return 1;
  }
  if (strcmp(name, "physics_ff_onset_enable") == 0)
  {
    g_blower_ctrl_physics_ff_onset_enable = (value != 0L) ? 1U : 0U;
    return 1;
  }
  if (strcmp(name, "physics_ff_onset_bypass_slew") == 0)
  {
    g_blower_ctrl_physics_ff_onset_bypass_slew = (value != 0L) ? 1U : 0U;
    return 1;
  }
  if (strcmp(name, "physics_ff_exp_lead_enable") == 0)
  {
    g_blower_ctrl_physics_ff_exp_lead_enable = (value != 0.0f) ? 1U : 0U;
    return 1;
  }
  if (strcmp(name, "physics_ff_exp_lead_k") == 0)
  {
    if (value < 0.0f) { value = 0.0f; }
    if (value > 50.0f) { value = 50.0f; }
    g_blower_ctrl_physics_ff_exp_lead_k_rpm_per_slm_per_s = value;
    return 1;
  }
  if (strcmp(name, "physics_ff_exp_lead_thresh") == 0)
  {
    if (value < 10.0f) { value = 10.0f; }
    if (value > 500.0f) { value = 500.0f; }
    g_blower_ctrl_physics_ff_exp_lead_thresh_slm_per_s = value;
    return 1;
  }
  if (strcmp(name, "physics_ff_exp_lead_max") == 0)
  {
    if (value < 0.0f) { value = 0.0f; }
    if (value > 3000.0f) { value = 3000.0f; }
    g_blower_ctrl_physics_ff_exp_lead_max_rpm = value;
    return 1;
  }
  if (strcmp(name, "physics_ff_onset_k") == 0)
  {
    g_blower_ctrl_physics_ff_onset_k_rpm_per_slm_per_s = value;
    return 1;
  }
  if (strcmp(name, "physics_ff_onset_thresh") == 0)
  {
    if (value < 10.0f) { value = 10.0f; }
    if (value > 500.0f) { value = 500.0f; }
    g_blower_ctrl_physics_ff_onset_thresh_slm_per_s = value;
    return 1;
  }
  if (strcmp(name, "physics_ff_onset_max") == 0)
  {
    if (value < 0.0f) { value = 0.0f; }
    if (value > 5000.0f) { value = 5000.0f; }
    g_blower_ctrl_physics_ff_onset_max_rpm = value;
    return 1;
  }
  if (strcmp(name, "physics_ff_onset_dqdt_tau") == 0)
  {
    if (value < 0.0f) { value = 0.0f; }
    if (value > 0.5f) { value = 0.5f; }
    g_blower_ctrl_physics_ff_onset_dqdt_tau_s = value;
    return 1;
  }
  if (strcmp(name, "physics_ff_inhale_rise_boost") == 0)
  {
    if (value < 1.0f) { value = 1.0f; }
    if (value > 4.0f) { value = 4.0f; }
    g_blower_ctrl_physics_ff_inhale_rise_boost = value;
    return 1;
  }
  if (strcmp(name, "hybrid_exp_hold_band") == 0)
  {
    if (value < 0.02f) { value = 0.02f; }
    if (value > 1.0f) { value = 1.0f; }
    g_blower_ctrl_hybrid_exp_hold_band_cmh2o = value;
    return 1;
  }
  if (strcmp(name, "hybrid_exp_fall") == 0)
  {
    if (value < 50.0f) { value = 50.0f; }
    if (value > 4000.0f) { value = 4000.0f; }
    g_blower_ctrl_hybrid_exp_fall_rpm_per_s = value;
    return 1;
  }
  if (strcmp(name, "hybrid_exp_boost_k") == 0)
  {
    if (value < 0.0f) { value = 0.0f; }
    if (value > 8000.0f) { value = 8000.0f; }
    g_blower_ctrl_hybrid_exp_boost_rpm_per_cmh2o = value;
    return 1;
  }
  if (strcmp(name, "hybrid_exp_boost_rise") == 0)
  {
    if (value < 500.0f) { value = 500.0f; }
    if (value > 20000.0f) { value = 20000.0f; }
    g_blower_ctrl_hybrid_exp_boost_rpm_per_s = value;
    return 1;
  }
  if (strcmp(name, "under_p_gain") == 0)
  {
    if (value < 1.0f) { value = 1.0f; }
    if (value > 8.0f) { value = 8.0f; }
    g_blower_ctrl_under_p_gain = value;
    return 1;
  }
  if (strcmp(name, "under_p_thresh") == 0)
  {
    if (value < 0.0f) { value = 0.0f; }
    if (value > 1.0f) { value = 1.0f; }
    g_blower_ctrl_under_p_thresh_cmh2o = value;
    return 1;
  }
  if (strcmp(name, "flow_ff_k") == 0) { g_blower_ctrl_flow_ff_k_rpm_per_slm = value; return 1; }
  if (strcmp(name, "flow_ff_max") == 0) { g_blower_ctrl_flow_ff_max_rpm = value; return 1; }
  if (strcmp(name, "flow_ff_deadband") == 0) { g_blower_ctrl_flow_ff_deadband_slm = value; return 1; }
  if (strcmp(name, "flow_ff_rise") == 0) { g_blower_ctrl_flow_ff_rise_rpm_per_s = value; return 1; }
  if (strcmp(name, "flow_ff_fall") == 0) { g_blower_ctrl_flow_ff_fall_rpm_per_s = value; return 1; }
  if (strcmp(name, "flow_ff_onset_k") == 0) { g_blower_ctrl_flow_ff_onset_k_rpm_per_slm_per_s = value; return 1; }
  if (strcmp(name, "flow_ff_onset_thresh") == 0) { g_blower_ctrl_flow_ff_onset_thresh_slm_per_s = value; return 1; }
  if (strcmp(name, "flow_ff_onset_max") == 0) { g_blower_ctrl_flow_ff_onset_max_rpm = value; return 1; }
  if (strcmp(name, "dip_ff_k") == 0) { g_blower_ctrl_dip_ff_k_rpm_per_cmh2o_per_s = value; return 1; }
  if (strcmp(name, "dip_ff_thresh") == 0) { g_blower_ctrl_dip_ff_thresh_cmh2o_per_s = value; return 1; }
  if (strcmp(name, "dip_ff_max") == 0) { g_blower_ctrl_dip_ff_max_rpm = value; return 1; }
  if (strcmp(name, "dip_ff_err_min") == 0) { g_blower_ctrl_dip_ff_err_min_cmh2o = value; return 1; }
  if (strcmp(name, "rise_ff_k") == 0) { g_blower_ctrl_rise_ff_k_rpm_per_cmh2o_per_s = value; return 1; }
  if (strcmp(name, "rise_ff_thresh") == 0) { g_blower_ctrl_rise_ff_thresh_cmh2o_per_s = value; return 1; }
  if (strcmp(name, "rise_ff_max") == 0) { g_blower_ctrl_rise_ff_max_rpm = value; return 1; }
  if (strcmp(name, "rise_ff_err_min") == 0) { g_blower_ctrl_rise_ff_err_min_cmh2o = value; return 1; }
  if (strcmp(name, "integ_err_lpf_tau") == 0) { g_blower_ctrl_integ_err_lpf_tau_s = value; return 1; }
  if (strcmp(name, "integ_err_lpf_tau_start") == 0) { g_blower_ctrl_integ_err_lpf_tau_start_s = value; return 1; }
  if (strcmp(name, "integ_err_lpf_warmup") == 0) { g_blower_ctrl_integ_err_lpf_warmup_s = value; return 1; }
  if (strcmp(name, "integ_err_lpf_band") == 0) { g_blower_ctrl_integ_err_lpf_settle_band_cmh2o = value; return 1; }
  if (strcmp(name, "exp_flow_ff_k") == 0) { g_blower_ctrl_exp_flow_ff_k_rpm_per_slm = value; return 1; }
  if (strcmp(name, "exp_flow_ff_lead") == 0) { g_blower_ctrl_exp_flow_ff_lead_s = value; return 1; }
  if (strcmp(name, "exp_flow_ff_max") == 0) { g_blower_ctrl_exp_flow_ff_max_rpm = value; return 1; }
  if (strcmp(name, "over_p_gain") == 0) { g_blower_ctrl_over_p_gain = value; return 1; }
  if (strcmp(name, "over_p_thresh") == 0)
  {
    /* Rest threshold; also arms the overpressure brake. Too low and sensor
     * noise trips it at rest (see blower_ctrl.c), so no value of 0. */
    if (value < 0.05f) { value = 0.05f; }
    if (value > 1.0f)  { value = 1.0f; }
    g_blower_ctrl_over_p_thresh_cmh2o = value;
    return 1;
  }
  if (strcmp(name, "over_p_thresh_exp") == 0)
  {
    /* Threshold while breathing (rest value is over_p_thresh). Same clamp. */
    if (value < 0.05f) { value = 0.05f; }
    if (value > 1.0f)  { value = 1.0f; }
    g_blower_ctrl_over_p_thresh_exp_cmh2o = value;
    return 1;
  }
  if (strcmp(name, "p_err_lpf_hz") == 0)
  {
    /* Low-pass on the P term's error, Hz; 0 = off (blower_ctrl.c). */
    if (value < 0.0f)  { value = 0.0f; }
    if (value > 50.0f) { value = 50.0f; }
    g_blower_ctrl_p_err_lpf_hz = value;
    return 1;
  }
  if (strcmp(name, "over_p_exp_window") == 0)
  {
    /* Seconds the low threshold stays on after the last real breath; 0 =
     * rest threshold everywhere. Must outlast a breath period to be
     * continuous while breathing (default 10). */
    if (value < 0.0f)  { value = 0.0f; }
    if (value > 15.0f) { value = 15.0f; }
    g_blower_ctrl_over_p_exp_window_s = value;
    return 1;
  }
  if (strcmp(name, "start_ramp") == 0)
  {
    /* Post-hand-over speed ramp, rpm/s; 0 = off (step straight to the
     * loop's command, which faulted the driver at 20 cmH2O). */
    if (value < 0.0f)     { value = 0.0f; }
    if (value > 60000.0f) { value = 60000.0f; }
    g_blower_ctrl_start_ramp_rpm_per_s = value;
    return 1;
  }
  if (strcmp(name, "over_brake") == 0) { g_blower_ctrl_over_brake_rpm_per_s = value; return 1; }
  if (strcmp(name, "slew_down") == 0) { g_blower_ctrl_slew_cmh2o_per_s = value; return 1; }
  if (strcmp(name, "slew_up") == 0) { g_blower_ctrl_slew_up_cmh2o_per_s = value; return 1; }
  if (strcmp(name, "out_alpha") == 0) { g_blower_ctrl_out_filter_alpha = value; return 1; }
  if (strcmp(name, "integ_deadband") == 0) { g_blower_ctrl_integ_deadband_cmh2o = value; return 1; }
  if (strcmp(name, "integ_max") == 0) { g_blower_ctrl_integ_max_abs_rpm = value; return 1; }
  if (strcmp(name, "meas_alpha") == 0) { g_blower_ctrl_meas_filter_alpha = value; return 1; }
  if (strcmp(name, "deriv_alpha") == 0) { g_blower_ctrl_deriv_filter_alpha = value; return 1; }
  if (strcmp(name, "patient_lpf_hz") == 0)
  {
    g_leak_patient_lpf_hz = value;
    g_leak_vivo_patient_lpf_hz = value;
    return 1;
  }
  if (strcmp(name, "patient_quiet_lpf_hz") == 0)
  {
    g_leak_patient_quiet_lpf_hz = value;
    return 1;
  }
  if (strcmp(name, "patient_quiet_band") == 0)
  {
    g_leak_patient_quiet_band_slm = value;
    return 1;
  }
  if (strcmp(name, "patient_quiet_exit") == 0)
  {
    g_leak_patient_quiet_exit_slm = value;
    return 1;
  }
  return 0;
}

static int Telem_SetLeakIntParam(const char *name, long value)
{
  if (strcmp(name, "vivo_enable") == 0)
  {
    Leak_SetVivoEnable(value != 0L);
    return 1;
  }
  if (strcmp(name, "patient_quiet_lpf_enable") == 0)
  {
    g_leak_patient_quiet_lpf_enable = (value != 0L) ? 1U : 0U;
    return 1;
  }
  /* R10: pause-DC zeroing of the patient flow (default 0). */
  if (strcmp(name, "patient_pause_zero") == 0)
  {
    g_leak_patient_pause_zero_enable = (value != 0L) ? 1U : 0U;
    return 1;
  }
  if (strcmp(name, "standard_ratio") == 0)
  {
    if (value < 100L) { value = 100L; }
    if (value > 32000L) { value = 32000L; }
    Leak_SetStandardRatio((int32_t)value);
    return 1;
  }
  return 0;
}

static int Telem_SetLeakFloatParam(const char *name, float value)
{
  if (strcmp(name, "vent_k") == 0)
  {
    Leak_SetVentCoefficient(value);
    return 1;
  }
  if (strcmp(name, "tau_seconds") == 0)
  {
    Leak_SetTimeConstant(value);
    return 1;
  }
  if (strcmp(name, "high_threshold") == 0)
  {
    Leak_SetHighThreshold(value);
    return 1;
  }
  if (strcmp(name, "small_threshold") == 0)
  {
    Leak_SetSmallThreshold(value);
    return 1;
  }
  if (strcmp(name, "low_threshold") == 0)
  {
    Leak_SetLowThreshold(value);
    return 1;
  }
  if (strcmp(name, "patient_lpf_hz") == 0)
  {
    if (value < 0.5f) { value = 0.5f; }
    if (value > 8.0f) { value = 8.0f; }
    g_leak_patient_lpf_hz = value;
    g_leak_vivo_patient_lpf_hz = value;
    return 1;
  }
  if (strcmp(name, "patient_quiet_lpf_hz") == 0)
  {
    if (value < 0.25f) { value = 0.25f; }
    if (value > 8.0f) { value = 8.0f; }
    g_leak_patient_quiet_lpf_hz = value;
    return 1;
  }
  if (strcmp(name, "patient_quiet_band") == 0)
  {
    if (value < 1.0f) { value = 1.0f; }
    if (value > 20.0f) { value = 20.0f; }
    g_leak_patient_quiet_band_slm = value;
    return 1;
  }
  if (strcmp(name, "patient_quiet_exit") == 0)
  {
    if (value < 2.0f) { value = 2.0f; }
    if (value > 25.0f) { value = 25.0f; }
    g_leak_patient_quiet_exit_slm = value;
    return 1;
  }
  return 0;
}

static int Telem_SetApapIntParam(const char *name, long value)
{
  if (strcmp(name, "response_soft") == 0)
  {
    Apap_SetResponseSoft(value != 0L);
    return 1;
  }
  if (strcmp(name, "fl_enable") == 0)
  {
    g_apap_fl_enable = (value != 0L) ? 1U : 0U;
    return 1;
  }
  if (strcmp(name, "fl_amp_arm") == 0)
  {
    g_apap_fl_amp_arm_enable = (value != 0L) ? 1U : 0U;
    return 1;
  }
  if (strcmp(name, "cpap_scoring") == 0)
  {
    g_apap_cpap_scoring_enable = (value != 0L) ? 1U : 0U;
    return 1;
  }
  if (strcmp(name, "fl_breaths_needed") == 0)
  {
    if (value < 1L) { value = 1L; }
    if (value > 10L) { value = 10L; }
    g_apap_fl_breaths_needed = (uint8_t)value;
    return 1;
  }
  if (strcmp(name, "fl_clear_breaths") == 0)
  {
    if (value < 1L) { value = 1L; }
    if (value > 10L) { value = 10L; }
    g_apap_fl_clear_breaths = (uint8_t)value;
    return 1;
  }
  if (strcmp(name, "hyp_enable") == 0)
  {
    g_apap_hyp_enable = (value != 0L) ? 1U : 0U;
    return 1;
  }
  if (strcmp(name, "hyp_breaths_needed") == 0)
  {
    if (value < 1L) { value = 1L; }
    if (value > 10L) { value = 10L; }
    g_apap_hyp_breaths_needed = (uint8_t)value;
    return 1;
  }
  if (strcmp(name, "snore_enable") == 0)
  {
    g_apap_snore_enable = (value != 0L) ? 1U : 0U;
    return 1;
  }
  if (strcmp(name, "lower_enable") == 0)
  {
    g_apap_lower_enable = (value != 0L) ? 1U : 0U;
    return 1;
  }
  if (strcmp(name, "leak_gate_enable") == 0)
  {
    g_apap_leak_gate_enable = (value != 0L) ? 1U : 0U;
    return 1;
  }
  return 0;
}

static int Telem_SetApapFloatParam(const char *name, float value)
{
  if (strcmp(name, "no_breath_timeout_s") == 0)
  {
    g_apap_no_breath_timeout_s = value;
    return 1;
  }
  if (strcmp(name, "breath_window_s") == 0)
  {
    g_apap_breath_window_s = value;
    return 1;
  }
  if (strcmp(name, "resume_min_peak") == 0)
  {
    g_apap_resume_min_peak_slm = value;
    return 1;
  }
  if (strcmp(name, "resume_peak_ratio") == 0)
  {
    g_apap_resume_peak_ratio = value;   /* clamped 0..0.8 where used */
    return 1;
  }
  if (strcmp(name, "resume_min_floor") == 0)
  {
    g_apap_resume_min_floor_slm = value;   /* clamped 2..20 where used */
    return 1;
  }
  if (strcmp(name, "fl_baseline_reject") == 0)
  {
    g_apap_fl_baseline_reject_ratio = value;   /* 0 = off, clamped 0..4 */
    return 1;
  }
  if (strcmp(name, "baseline_restore_max_s") == 0)
  {
    g_apap_baseline_restore_max_s = value;   /* 0 = never restore */
    return 1;
  }
  if (strcmp(name, "ca_reclass_window_s") == 0)
  {
    g_apap_ca_reclass_window_s = value;   /* 0 = never withdraw a CA */
    return 1;
  }
  if (strcmp(name, "leak_freeze_min_breaths") == 0)
  {
    g_apap_leak_freeze_min_breaths = (value <= 0.0f) ? 0U
                                   : ((value >= 255.0f) ? 255U : (uint8_t)value);
    return 1;
  }
  if (strcmp(name, "leak_freeze_max_s") == 0)
  {
    g_apap_leak_freeze_max_s = value;   /* 0 = no cap */
    return 1;
  }
  if (strcmp(name, "breath_notch") == 0)
  {
    g_apap_breath_notch_enable = (value != 0.0f) ? 1U : 0U;
    return 1;
  }
  if (strcmp(name, "hyp_ca_guard_s") == 0)
  {
    g_apap_hyp_ca_guard_s = value;   /* 0 = off */
    return 1;
  }
  /* R12: Hyp re-arm hysteresis (ratio x baseline, consecutive breaths). */
  if (strcmp(name, "hyp_rearm_ratio") == 0)
  {
    g_apap_hyp_rearm_ratio = value;
    return 1;
  }
  if (strcmp(name, "hyp_rearm_breaths") == 0)
  {
    g_apap_hyp_rearm_breaths = (value < 1.0f) ? 1U
                             : ((value > 10.0f) ? 10U : (uint8_t)value);
    return 1;
  }
  if (strcmp(name, "breath_min_vol_ratio") == 0)
  {
    g_epr_apap_breath_min_vol_ratio = value;   /* x median; 0 = floor only */
    return 1;
  }
  if (strcmp(name, "epr_baseline_slow_tau_s") == 0)
  {
    g_epr_baseline_slow_tau_s = value;   /* R11: 0 = off */
    return 1;
  }
  /* R9: breath detection relative to the EPR flow baseline (0/1 each). */
  if (strcmp(name, "epr_rel_baseline") == 0)
  {
    g_epr_rel_baseline_enable = (value != 0.0f) ? 1U : 0U;
    return 1;
  }
  if (strcmp(name, "rel_baseline") == 0)
  {
    g_apap_rel_baseline_enable = (value != 0.0f) ? 1U : 0U;
    return 1;
  }
  /* R5a: APAP breath qualification lives in the EPR breath detector. */
  if (strcmp(name, "breath_min_insp_ms") == 0)
  {
    g_epr_apap_breath_min_insp_ms = (value > 0.0f) ? (uint32_t)value : 0U;
    return 1;
  }
  if (strcmp(name, "breath_min_insp_ml") == 0)
  {
    g_epr_apap_breath_min_insp_ml = (value > 0.0f) ? value : 0.0f;
    return 1;
  }
  if (strcmp(name, "startup_grace_s") == 0)
  {
    g_apap_startup_grace_s = value;
    return 1;
  }
  if (strcmp(name, "osc_amplitude") == 0)
  {
    g_apap_osc_amplitude_cmh2o = value;
    return 1;
  }
  if (strcmp(name, "osc_amplitude_max") == 0)
  {
    g_apap_osc_amplitude_max_cmh2o = value;
    return 1;
  }
  if (strcmp(name, "osc_freq_hz") == 0)
  {
    g_apap_osc_freq_hz = value;
    return 1;
  }
  if (strcmp(name, "fot_eval_period_s") == 0)
  {
    g_apap_fot_eval_period_s = value;
    return 1;
  }
  if (strcmp(name, "osc_min_p_pp") == 0)
  {
    g_apap_osc_min_p_pp_cmh2o = value;
    return 1;
  }
  if (strcmp(name, "osa_flow_pp_thresh") == 0)
  {
    g_apap_osa_flow_pp_thresh_slm = value;
    return 1;
  }
  if (strcmp(name, "apnea_min_s") == 0)
  {
    g_apap_apnea_min_s = value;
    return 1;
  }
  if (strcmp(name, "oa_raise_per_s") == 0)
  {
    g_apap_oa_raise_cmh2o_per_s = value;
    return 1;
  }
  if (strcmp(name, "oa_raise_max") == 0)
  {
    g_apap_oa_raise_max_cmh2o = value;
    return 1;
  }
  if (strcmp(name, "her_oa_above_cmh2o") == 0)
  {
    g_apap_her_oa_above_cmh2o = value;
    return 1;
  }
  if (strcmp(name, "her_oa_above_scale") == 0)
  {
    g_apap_her_oa_above_scale = value;
    return 1;
  }
  if (strcmp(name, "raise_rate_standard") == 0)
  {
    g_apap_raise_rate_standard_cmh2o_per_s = value;
    return 1;
  }
  if (strcmp(name, "raise_rate_soft") == 0)
  {
    g_apap_raise_rate_soft_cmh2o_per_s = value;
    return 1;
  }
  if (strcmp(name, "raise_rate_her_scale") == 0)
  {
    g_apap_raise_rate_her_scale = value;
    return 1;
  }
  if (strcmp(name, "fl_ratio") == 0)
  {
    g_apap_fl_ratio = value;
    return 1;
  }
  if (strcmp(name, "fl_raise_scale") == 0)
  {
    g_apap_fl_raise_scale = value;
    return 1;
  }
  if (strcmp(name, "fl_min_peak") == 0)
  {
    g_apap_fl_min_peak_slm = value;
    return 1;
  }
  if (strcmp(name, "fl_baseline_alpha") == 0)
  {
    g_apap_fl_baseline_alpha = value;
    return 1;
  }
  if (strcmp(name, "fl_flat_ratio") == 0)
  {
    g_apap_fl_flat_ratio = value;
    return 1;
  }
  if (strcmp(name, "fl_dip_ratio") == 0)
  {
    g_apap_fl_dip_ratio = value;
    return 1;
  }
  if (strcmp(name, "hyp_raise_shape_frac") == 0)
  {
    g_apap_hyp_raise_shape_frac = value;
    return 1;
  }
  if (strcmp(name, "fl_seg_floor") == 0)
  {
    g_apap_fl_seg_floor_slm = value;
    return 1;
  }
  if (strcmp(name, "hyp_ratio") == 0)
  {
    g_apap_hyp_ratio = value;
    return 1;
  }
  if (strcmp(name, "hyp_min_duration_s") == 0)
  {
    g_apap_hyp_min_duration_s = value;
    return 1;
  }
  if (strcmp(name, "hyp_raise_scale") == 0)
  {
    g_apap_hyp_raise_scale = value;
    return 1;
  }
  if (strcmp(name, "hyp_hold_s") == 0)
  {
    g_apap_hyp_hold_s = value;
    return 1;
  }
  if (strcmp(name, "snore_rms_thresh") == 0)
  {
    g_apap_snore_rms_thresh_slm = value;
    return 1;
  }
  if (strcmp(name, "snore_on_s") == 0)
  {
    g_apap_snore_on_s = value;
    return 1;
  }
  if (strcmp(name, "snore_off_s") == 0)
  {
    g_apap_snore_off_s = value;
    return 1;
  }
  if (strcmp(name, "snore_raise_scale") == 0)
  {
    g_apap_snore_raise_scale = value;
    return 1;
  }
  if (strcmp(name, "snore_hp_hz") == 0)
  {
    g_apap_snore_hp_hz = value;
    return 1;
  }
  if (strcmp(name, "lower_rate") == 0)
  {
    g_apap_lower_rate_cmh2o_per_s = value;
    return 1;
  }
  if (strcmp(name, "stable_before_lower_s") == 0)
  {
    g_apap_stable_before_lower_s = value;
    return 1;
  }
  return 0;
}

static int Telem_SetTubeCompParam(const char *name, float value)
{
  if (strcmp(name, "enable") == 0)
  {
    TubeComp_SetEnable(value != 0.0f);
    return 1;
  }
  if (strcmp(name, "R_lam") == 0)
  {
    TubeComp_SetCoefficients(value, TubeComp_GetRTurb());
    return 1;
  }
  if (strcmp(name, "R_turb") == 0)
  {
    TubeComp_SetCoefficients(TubeComp_GetRLam(), value);
    return 1;
  }
  if (strcmp(name, "L") == 0)
  {
    TubeComp_SetInertance(value, TubeComp_GetInertanceTau());
    return 1;
  }
  if (strcmp(name, "L_tau") == 0)
  {
    TubeComp_SetInertance(TubeComp_GetInertanceL(), value);
    return 1;
  }
  if (strcmp(name, "L_db") == 0)
  {
    TubeComp_SetInertanceDeadband(value);
    return 1;
  }
  return 0;
}

static int Telem_SetIntParam(const char *name, long value)
{
  if (strcmp(name, "loop_enable") == 0)
  {
    BlowerCtrl_SetEnable(value != 0L);
    return 1;
  }
  if (strcmp(name, "pid_enable") == 0)
  {
    g_blower_ctrl_pid_enable = (value != 0L) ? 1U : 0U;
    return 1;
  }
  if (strcmp(name, "rpm_floor_enable") == 0)
  {
    g_blower_ctrl_rpm_floor_enable = (value != 0L) ? 1U : 0U;
    return 1;
  }
  /* Expiratory RPM floors 1 and 3 (floor 2 is rpm_floor_enable above).
   * All default to 1; clear one at a time to A/B the expiratory bump. */
  if (strcmp(name, "ff_ss_floor_enable") == 0)
  {
    g_blower_ctrl_ff_ss_floor_enable = (value != 0L) ? 1U : 0U;
    return 1;
  }
  if (strcmp(name, "exhale_integ_clamp_enable") == 0)
  {
    g_blower_ctrl_exhale_integ_clamp_enable = (value != 0L) ? 1U : 0U;
    return 1;
  }
  if (strcmp(name, "vent_floor_enable") == 0)
  {
    g_blower_ctrl_vent_floor_enable = (value != 0L) ? 1U : 0U;
    return 1;
  }
  /* Zeroing a negative I during expiration; measured cause of the second
   * expiratory hump, so default OFF. See blower_ctrl.c. */
  if (strcmp(name, "integ_err_lpf_enable") == 0)
  {
    g_blower_ctrl_integ_err_lpf_enable = (value != 0L) ? 1U : 0U;
    return 1;
  }
  /* Freeze the integral trim so the baseline stops drifting with breathing.
   * Leaves the accumulated trim in place; P and FF keep working. */
  if (strcmp(name, "integ_hold") == 0)
  {
    g_blower_ctrl_integ_hold = (value != 0L) ? 1U : 0U;
    return 1;
  }
  /* Headroom clamp holds I instead of dragging it negative when FF + P alone
   * exceed the rpm ceiling. Default ON; 0 restores the old clamp for A/B. */
  if (strcmp(name, "integ_headroom_hold") == 0)
  {
    g_blower_ctrl_integ_headroom_hold_enable = (value != 0L) ? 1U : 0U;
    return 1;
  }
  if (strcmp(name, "exp_flow_ff_enable") == 0)
  {
    g_blower_ctrl_exp_flow_ff_enable = (value != 0L) ? 1U : 0U;
    return 1;
  }
  if (strcmp(name, "rise_ff_enable") == 0)
  {
    g_blower_ctrl_rise_ff_enable = (value != 0L) ? 1U : 0U;
    return 1;
  }
  if (strcmp(name, "exp_neg_i_clamp_enable") == 0)
  {
    g_blower_ctrl_exp_neg_i_clamp_enable = (value != 0L) ? 1U : 0U;
    return 1;
  }
  /* Breath-phase polarity: +1 inspiration = positive flow (default),
   * -1 inverted board, 0 = old first-excursion auto-detect. Takes effect at
   * the next breath-FSM reset (therapy start / mask reconnect). */
  /* Controller term decomposition on the "D," line (see Telem_PushSample). */
  if (strcmp(name, "telem_terms") == 0)
  {
    g_telem_terms_enable = (value != 0L) ? 1U : 0U;
    return 1;
  }
  /* Motor electrical line at 25 Hz - vbus, iq, iq_ref, id, CM4's own rpm. */
  if (strcmp(name, "telem_motor") == 0)
  {
    g_telem_motor_enable = (value != 0L) ? 1U : 0U;
    return 1;
  }
  if (strcmp(name, "epr_insp_polarity") == 0)
  {
    g_epr_insp_polarity_cfg = (value > 0L) ? (int8_t)1
                            : ((value < 0L) ? (int8_t)-1 : (int8_t)0);
    return 1;
  }
  /* Off by default - the deadband was dormant while the gains were tuned. */
  if (strcmp(name, "integ_deadband_enable") == 0)
  {
    g_blower_ctrl_integ_deadband_enable = (value != 0L) ? 1U : 0U;
    return 1;
  }
  if (strcmp(name, "ff_enable") == 0)
  {
    g_blower_ctrl_ff_enable = (value != 0L) ? 1U : 0U;
    BlowerCtrl_EnforceFfMutualExclusion();
    return 1;
  }
  if (strcmp(name, "flow_ff_enable") == 0)
  {
    g_blower_ctrl_flow_ff_enable = (value != 0L) ? 1U : 0U;
    BlowerCtrl_EnforceFfMutualExclusion();
    return 1;
  }
  if (strcmp(name, "flow_ff_onset_enable") == 0)
  {
    g_blower_ctrl_flow_ff_onset_enable = (value != 0L) ? 1U : 0U;
    return 1;
  }
  if (strcmp(name, "dip_ff_enable") == 0)
  {
    g_blower_ctrl_dip_ff_enable = (value != 0L) ? 1U : 0U;
    BlowerCtrl_EnforceFfMutualExclusion();
    return 1;
  }
  if (strcmp(name, "physics_ff_enable") == 0)
  {
    g_blower_ctrl_physics_ff_enable = (value != 0L) ? 1U : 0U;
    BlowerCtrl_EnforceFfMutualExclusion();
    return 1;
  }
  if (strcmp(name, "cpap_plateau_enable") == 0)
  {
    BlowerCtrl_SetCpapPlateauPrefer(value != 0L);
    BlowerCtrl_SyncCpapProfilesForMode((uint8_t)Therapy_GetMode());
    return 1;
  }
  if (strcmp(name, "cpap_hybrid_enable") == 0)
  {
    BlowerCtrl_SetCpapHybridPrefer(value != 0L);
    BlowerCtrl_SyncCpapProfilesForMode((uint8_t)Therapy_GetMode());
    return 1;
  }
  if (strcmp(name, "flow_ff_sign") == 0)
  {
    g_blower_ctrl_flow_ff_inhale_sign = (value < 0L) ? (int8_t)-1 : (int8_t)1;
    return 1;
  }
  if (strcmp(name, "rpm_max") == 0)
  {
    if (value < 0L) { value = 0L; }
    if (value > 45000L) { value = 45000L; }
    g_blower_ctrl_rpm_max = (int32_t)value;
    return 1;
  }
  return 0;
}

static int Telem_SetFocParam(const char *name, long value)
{
  BlowerFocGainId_t id;

  /* Range clamping per parameter is in BlowerIpc_CM7_SetFocGain. */
  if (value < 0L) { value = 0L; }
  if (value > 45000L) { value = 45000L; }

  if (strcmp(name, "speed_kp") == 0) { id = FOC_GAIN_SPEED_KP; }
  else if (strcmp(name, "speed_ki") == 0) { id = FOC_GAIN_SPEED_KI; }
  else if (strcmp(name, "torque_kp") == 0) { id = FOC_GAIN_TORQUE_KP; }
  else if (strcmp(name, "torque_ki") == 0) { id = FOC_GAIN_TORQUE_KI; }
  else if (strcmp(name, "flux_kp") == 0) { id = FOC_GAIN_FLUX_KP; }
  else if (strcmp(name, "flux_ki") == 0) { id = FOC_GAIN_FLUX_KI; }
  else if (strcmp(name, "iq_max_ma") == 0) { id = FOC_GAIN_IQ_MAX_MA; }
  else if (strcmp(name, "iq_boost_ma") == 0) { id = FOC_GAIN_IQ_BOOST_MA; }
  else if (strcmp(name, "iq_boost_lo_rpm") == 0) { id = FOC_GAIN_IQ_BOOST_LO_RPM; }
  else if (strcmp(name, "iq_boost_hi_rpm") == 0) { id = FOC_GAIN_IQ_BOOST_HI_RPM; }
  else { return 0; }

  return BlowerIpc_CM7_SetFocGain(id, (int32_t)value) ? 1 : 0;
}

static int Telem_ParseManualSpeed(const char *args, int32_t *rpm, uint16_t *ramp_ms)
{
  long rpm_l = 0L;
  long ramp_l = 0L;
  char extra = '\0';

  if (sscanf(args, "%ld %ld%c", &rpm_l, &ramp_l, &extra) != 2)
  {
    return 0;
  }
  if (rpm_l < 0L) { rpm_l = 0L; }
  if (rpm_l > 45000L) { rpm_l = 45000L; }
  if (ramp_l < 0L) { ramp_l = 0L; }
  if (ramp_l > 65535L) { ramp_l = 65535L; }
  *rpm = (int32_t)rpm_l;
  *ramp_ms = (uint16_t)ramp_l;
  return 1;
}

static void Telem_BlowerManualStart(int32_t rpm, uint16_t ramp_ms)
{
  if (Therapy_GetState() != (uint8_t)THERAPY_STATE_IDLE)
  {
    Therapy_Stop();
  }
  BlowerCtrl_ManualStart(rpm, ramp_ms);
}

#define TELEM_CMD_BATCH_MAX  16U

static bool Telem_QueueTailStartsWith(const char *prefix, size_t len)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (s_cmd_head == s_cmd_tail)
  {
    if (primask == 0U)
    {
      __enable_irq();
    }
    return false;
  }
  const int eq = (strncmp(s_cmd_queue[s_cmd_tail], prefix, len) == 0);
  if (primask == 0U)
  {
    __enable_irq();
  }
  return (eq != 0);
}

static bool Telem_DequeueNextCommand(char *cmd, size_t cmd_len)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();
  if (s_cmd_head == s_cmd_tail)
  {
    if (primask == 0U)
    {
      __enable_irq();
    }
    return false;
  }
  (void)strncpy(cmd, s_cmd_queue[s_cmd_tail], cmd_len - 1U);
  cmd[cmd_len - 1U] = '\0';
  s_cmd_tail = (uint8_t)((s_cmd_tail + 1U) % TELEM_CMD_QUEUE_LEN);
  if (primask == 0U)
  {
    __enable_irq();
  }
  return true;
}

static void Telem_ProcessOneCommand(const char *cmd);

static void Telem_DrainQueuedTherapySetCommands(void)
{
  char pending[sizeof(s_cmd_queue[0])];

  s_telem_quiet = 1U;
  for (uint8_t n = 0U; n < TELEM_CMD_BATCH_MAX; n++)
  {
    if (!Telem_QueueTailStartsWith("therapy set ", 12U))
    {
      break;
    }
    if (!Telem_DequeueNextCommand(pending, sizeof(pending)))
    {
      break;
    }
    Telem_ProcessOneCommand(pending);
  }
  s_telem_quiet = 0U;
}

/* "bench play ..." - bench-only command replay, see bench_playback.h.
 * 'add' answers with the running sample count instead of echoing the line,
 * so the host can check nothing was dropped. */
static void Telem_HandleBenchPlay(const char *cmd, const char *arg)
{
#if BENCH_PLAYBACK
  char line[96];
  int n;

  if (strcmp(arg, "clear") == 0)
  {
    BenchPlay_Clear();
    Telem_PushAck(cmd);
    return;
  }
  if (strncmp(arg, "period ", 7U) == 0)
  {
    long ms = 0L;
    if (Telem_ParseLong(&arg[7], &ms) && (ms > 0L)
        && BenchPlay_SetPeriodMs((uint32_t)ms))
    {
      Telem_PushAck(cmd);
    }
    else
    {
      Telem_PushErr("bench play period: 5-1000 ms, multiple of 5, not while playing");
    }
    return;
  }
  if (strncmp(arg, "add ", 4U) == 0)
  {
    if (BenchPlay_Append(&arg[4]) > 0)
    {
      n = snprintf(line, sizeof(line), "# PLAY,loaded,%lu\r\n",
                   (unsigned long)BenchPlay_Count());
      if ((n > 0) && ((size_t)n < sizeof(line)))
      {
        Telem_PushRaw(line);
      }
    }
    else
    {
      Telem_PushErr("bench play add: bad value, buffer full or playing");
    }
    return;
  }
  if (strcmp(arg, "start") == 0)
  {
    if (BenchPlay_Start())
    {
      Telem_PushAck(cmd);
    }
    else
    {
      Telem_PushErr("bench play start: needs samples, manual mode in RUN, therapy idle");
    }
    return;
  }
  if (strcmp(arg, "stop") == 0)
  {
    BenchPlay_Stop();
    Telem_PushAck(cmd);
    return;
  }
  if (strcmp(arg, "status") == 0)
  {
    n = snprintf(line, sizeof(line), "# PLAY,status,n=%lu,period=%lu,active=%u,idx=%lu\r\n",
                 (unsigned long)BenchPlay_Count(), (unsigned long)BenchPlay_PeriodMs(),
                 BenchPlay_IsActive() ? 1U : 0U, (unsigned long)BenchPlay_Index());
    if ((n > 0) && ((size_t)n < sizeof(line)))
    {
      Telem_PushRaw(line);
    }
    return;
  }
  Telem_PushErr("unknown bench play command");
#else
  (void)cmd;
  (void)arg;
  Telem_PushErr("bench playback not built");
#endif
}

static void Telem_ProcessOneCommand(const char *cmd)
{
  if (strcmp(cmd, "therapy start") == 0)
  {
    Telem_DrainQueuedTherapySetCommands();
    Therapy_Start();
    Telem_PushAck(cmd);
    /* One status after start is enough — drain above stays quiet. */
    Telem_PushStatus();
    return;
  }
  if (strcmp(cmd, "therapy stop") == 0)
  {
    Therapy_Stop();
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }
  if (strcmp(cmd, "therapy status") == 0)
  {
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }

  if (strcmp(cmd, "therapy percentiles") == 0)
  {
    char line[192];
    int n = snprintf(line, sizeof(line),
                     "# P_session avg=%.2f min=%.2f max=%.2f "
                     "P50=%.2f P90=%.2f P95=%.2f n=%lu\r\n",
                     (double)Therapy_GetSessionPressureAvg(),
                     (double)Therapy_GetSessionPressureMin(),
                     (double)Therapy_GetSessionPressureMax(),
                     (double)Therapy_GetSessionPressureP50(),
                     (double)Therapy_GetSessionPressureP90(),
                     (double)Therapy_GetSessionPressureP95(),
                     (unsigned long)Therapy_GetSessionPressureSamples());
    Telem_PushAck(cmd);
    if ((n > 0) && ((size_t)n < sizeof(line)))
    {
      Telem_BufPush((const uint8_t *)line, (uint32_t)n);
    }
    return;
  }

  if (strcmp(cmd, "therapy apap events") == 0)
  {
    char line[256];
    int n = snprintf(line, sizeof(line),
                     "# APAP OA=%lu CA=%lu Hyp=%lu FL=%lu Snore=%lu "
                     "t_ms=%lu AHI=%.2f RDI=%.2f SI=%.2f "
                     "st=%s hyp=%u fl=%u sn=%u leak_blk=%u "
                     "pol=%d pol_ev=%.1f pol_bad=%u flat=%.2f dip=%.2f\r\n",
                     (unsigned long)Apap_GetEventOaCount(),
                     (unsigned long)Apap_GetEventCaCount(),
                     (unsigned long)Apap_GetEventHypCount(),
                     (unsigned long)Apap_GetEventFlCount(),
                     (unsigned long)Apap_GetEventSnoreCount(),
                     (unsigned long)Apap_GetTherapyMs(),
                     (double)Apap_GetSessionAhi(),
                     (double)Apap_GetSessionRdi(),
                     (double)Apap_GetSessionSnoreIndex(),
                     Apap_GetStateName(),
                     Apap_IsHypopneaActive() ? 1U : 0U,
                     Apap_IsFlowLimited() ? 1U : 0U,
                     Apap_IsSnoreActive() ? 1U : 0U,
                     Apap_IsRaiseBlockedByLeak() ? 1U : 0U,
                     /* Breath-phase polarity gates every detector above: a
                      * wrong sign makes them silently score nothing. */
                     (int)g_epr_insp_polarity,
                     (double)g_epr_insp_polarity_evidence,
                     (unsigned)g_epr_insp_polarity_mismatch,
                     /* R13: the last breath's whole-inspiration shape. */
                     (double)Apap_GetFlLastFlat(),
                     (double)Apap_GetFlLastDip());
    Telem_PushAck(cmd);
    if ((n > 0) && ((size_t)n < sizeof(line)))
    {
      Telem_BufPush((const uint8_t *)line, (uint32_t)n);
    }
    return;
  }

  if (strcmp(cmd, "therapy edf status") == 0)
  {
    extern volatile uint32_t g_diskio_last_hal_err;
    extern volatile uint32_t g_diskio_fail_count;
    extern volatile uint32_t sd_test_fail_step;
    extern volatile uint32_t sd_hal_error;
    extern volatile uint32_t g_session_edf_eve_dropped;
    char line[512];
    int n = snprintf(line, sizeof(line),
                     "# EDF en=%u st=%s path=%s rec=%lu fsize=%lu err=0x%lX "
                     "pld=%s pldrec=%lu plderr=0x%lX "
                     "eve=%s everec=%lu eveerr=0x%lX evedrop=%lu "
                     "mounted=%u sd=%u test=%d fatfs=%u fail_step=%lu "
                     "sd_hal=0x%lX dio_err=0x%lX dio_n=%lu\r\n",
                     SessionEdf_IsEnabled() ? 1U : 0U,
                     SessionEdf_GetStateName(),
                     SessionEdf_GetLastPath(),
                     (unsigned long)SessionEdf_GetRecords(),
                     (unsigned long)SessionEdf_GetFileSize(),
                     (unsigned long)SessionEdf_GetLastError(),
                     SessionEdf_GetPldPath(),
                     (unsigned long)SessionEdf_GetPldRecords(),
                     (unsigned long)SessionEdf_GetPldError(),
                     SessionEdf_GetEvePath(),
                     (unsigned long)SessionEdf_GetEveRecords(),
                     (unsigned long)SessionEdf_GetEveError(),
                     (unsigned long)g_session_edf_eve_dropped,
                     SessionEdf_IsMounted() ? 1U : 0U,
                     (unsigned)sd_card_present,
                     (int)sd_test_result,
                     (unsigned)sd_fatfs_ready,
                     (unsigned long)sd_test_fail_step,
                     (unsigned long)sd_hal_error,
                     (unsigned long)g_diskio_last_hal_err,
                     (unsigned long)g_diskio_fail_count);
    Telem_PushAck(cmd);
    if ((n > 0) && ((size_t)n < sizeof(line)))
    {
      Telem_BufPush((const uint8_t *)line, (uint32_t)n);
    }
    return;
  }

  /* Blower speed ceiling. Writes g_blower_ctrl_rpm_max too, so it takes
   * effect immediately rather than at the next Therapy_Start. Clamped to what
   * the CM4 FOC will accept. */
  if (strncmp(cmd, "safety set safe_rpm_max ", 24U) == 0)
  {
    extern volatile int32_t g_safety_safe_rpm_max;
    extern volatile int32_t g_blower_ctrl_rpm_max;
    long v = 0L;
    if (!Telem_ParseLong(&cmd[24], &v))
    {
      Telem_PushErr("safety set safe_rpm_max <rpm>");
      return;
    }
    if (v < 10000L) { v = 10000L; }
    if (v > (long)BLOWER_IPC_MAX_SPEED_RPM) { v = (long)BLOWER_IPC_MAX_SPEED_RPM; }
    g_safety_safe_rpm_max = (int32_t)v;
    g_blower_ctrl_rpm_max = (int32_t)v;
    Telem_PushAck(cmd);
    return;
  }

  if (strncmp(cmd, "therapy edf enable ", 19U) == 0)
  {
    long v = 0;
    if (!Telem_ParseLong(cmd + 19, &v) || ((v != 0) && (v != 1)))
    {
      Telem_PushErr("therapy edf enable 0|1");
      return;
    }
    SessionEdf_SetEnable(v != 0);
    Telem_PushAck(cmd);
    return;
  }

  if (strncmp(cmd, "therapy edf datetime ", 21U) == 0)
  {
    /* YY MM DD HH MI SS (space-separated). */
    unsigned yy = 0, mo = 0, dd = 0, hh = 0, mi = 0, ss = 0;
    if (sscanf(cmd + 21, "%u %u %u %u %u %u",
               &yy, &mo, &dd, &hh, &mi, &ss) != 6)
    {
      Telem_PushErr("therapy edf datetime YY MM DD HH MI SS");
      return;
    }
    SessionEdf_SetDateTime((uint8_t)yy, (uint8_t)mo, (uint8_t)dd,
                           (uint8_t)hh, (uint8_t)mi, (uint8_t)ss);
    Telem_PushAck(cmd);
    return;
  }

  /* RTC calendar (checklist #66). `rtc set` is the same as
   * `therapy edf datetime`; both set the RTC and the EDF fallback. */
  if (strncmp(cmd, "rtc set ", 8U) == 0)
  {
    unsigned yy = 0, mo = 0, dd = 0, hh = 0, mi = 0, ss = 0;
    if ((sscanf(cmd + 8, "%u %u %u %u %u %u",
                &yy, &mo, &dd, &hh, &mi, &ss) != 6)
        || (yy > 99U) || (mo < 1U) || (mo > 12U) || (dd < 1U) || (dd > 31U)
        || (hh > 23U) || (mi > 59U) || (ss > 59U))
    {
      Telem_PushErr("rtc set YY MM DD HH MI SS");
      return;
    }
    SessionEdf_SetDateTime((uint8_t)yy, (uint8_t)mo, (uint8_t)dd,
                           (uint8_t)hh, (uint8_t)mi, (uint8_t)ss);
    Telem_PushAck(cmd);
    return;
  }

  /* #65: adherence from the session records on the card (adherence.h). */
  if (strcmp(cmd, "therapy adherence") == 0)
  {
    char line[320];
    const int n = Adherence_Format(line, sizeof(line));
    Telem_PushAck(cmd);
    if (n > 0)
    {
      Telem_BufPush((const uint8_t *)line, (uint32_t)n);
    }
    return;
  }

  /* #57: the last session's summary record (as appended to SUMMARY.CSV). */
  if (strcmp(cmd, "therapy summary") == 0)
  {
    char line[64];
    Telem_PushAck(cmd);
    int n = snprintf(line, sizeof(line), "# SUM pending=%u written=%lu err=0x%lX\r\n",
                     SessionSummary_IsPending() ? 1U : 0U,
                     (unsigned long)SessionSummary_GetWritten(),
                     (unsigned long)SessionSummary_GetError());
    if ((n > 0) && ((size_t)n < sizeof(line)))
    {
      Telem_BufPush((const uint8_t *)line, (uint32_t)n);
    }
    Telem_BufPush((const uint8_t *)"# SUMH ", 7U);
    Telem_BufPush((const uint8_t *)SessionSummary_GetHeaderLine(),
                  (uint32_t)strlen(SessionSummary_GetHeaderLine()));
    Telem_BufPush((const uint8_t *)"\r\n# SUMR ", 9U);
    Telem_BufPush((const uint8_t *)SessionSummary_GetLastLine(),
                  (uint32_t)strlen(SessionSummary_GetLastLine()));
    Telem_BufPush((const uint8_t *)"\r\n", 2U);
    return;
  }

  if (strcmp(cmd, "rtc status") == 0)
  {
    char line[112];
    RtcDateTime_t dt = { 0U, 0U, 0U, 0U, 0U, 0U };
    const bool run = RtcClock_Get(&dt);
    const int n = snprintf(line, sizeof(line),
                           "# rtc src=%s run=%u valid=%u now=20%02u-%02u-%02u %02u:%02u:%02u\r\n",
                           RtcClock_GetSourceName(), run ? 1U : 0U,
                           RtcClock_IsTimeValid() ? 1U : 0U,
                           (unsigned)dt.yy, (unsigned)dt.mo, (unsigned)dt.dd,
                           (unsigned)dt.hh, (unsigned)dt.mi, (unsigned)dt.ss);
    Telem_PushAck(cmd);
    if ((n > 0) && ((size_t)n < sizeof(line)))
    {
      Telem_BufPush((const uint8_t *)line, (uint32_t)n);
    }
    return;
  }

  if (strcmp(cmd, "therapy set mode cpap") == 0)
  {
    Therapy_SetMode(THERAPY_MODE_CPAP);
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }
  if (strcmp(cmd, "therapy set mode autoset") == 0)
  {
    Therapy_SetMode(THERAPY_MODE_AUTOSET);
    Therapy_SetTargetPressure(Therapy_GetApapMinPressure());
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }
  if (strcmp(cmd, "therapy set mode autoset_her") == 0)
  {
    Therapy_SetMode(THERAPY_MODE_AUTOSET_FOR_HER);
    Therapy_SetTargetPressure(Therapy_GetApapMinPressure());
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }

  if (strncmp(cmd, "therapy set pressure_min ", 25U) == 0)
  {
    float v;
    if (!Telem_ParseFloat(&cmd[25], &v) || (v < 4.0f) || (v > 25.0f))
    {
      Telem_PushErr("pressure_min range 4..25");
      return;
    }
    Therapy_SetApapMinPressure(v);
    if (Therapy_GetMode() != THERAPY_MODE_CPAP)
    {
      Therapy_SetTargetPressure(Therapy_GetApapMinPressure());
    }
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }

  if (strncmp(cmd, "therapy set pressure_max ", 25U) == 0)
  {
    float v;
    if (!Telem_ParseFloat(&cmd[25], &v) || (v < 4.0f) || (v > 25.0f))
    {
      Telem_PushErr("pressure_max range 4..25");
      return;
    }
    Therapy_SetApapMaxPressure(v);
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }

  if (strncmp(cmd, "therapy set pressure ", 21U) == 0)
  {
    float v;
    if (!Telem_ParseFloat(&cmd[21], &v) || (v < 4.0f) || (v > 25.0f))
    {
      Telem_PushErr("pressure range 4..25");
      return;
    }
    Therapy_SetMode(THERAPY_MODE_CPAP);
    Therapy_SetTargetPressure(v);
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }

  if (strncmp(cmd, "therapy set ramp ", 17U) == 0)
  {
    long v;
    if (!Telem_ParseLong(&cmd[17], &v) || (v < 0L) || (v > 45L))
    {
      Telem_PushErr("ramp range 0..45");
      return;
    }
    Therapy_SetRampMinutes((uint8_t)v);
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }

  if (strncmp(cmd, "therapy set epr ", 16U) == 0)
  {
    long v;
    if (!Telem_ParseLong(&cmd[16], &v) || ((v != 0L) && (v != 1L)))
    {
      Telem_PushErr("epr must be 0 or 1");
      return;
    }
    Therapy_SetEprEnabled(v != 0L);
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }

  if (strncmp(cmd, "therapy set epr_relief ", 23U) == 0)
  {
    float v;
    if (!Telem_ParseFloat(&cmd[23], &v) || (v < 1.0f) || (v > 3.0f))
    {
      Telem_PushErr("epr_relief range 1..3");
      return;
    }
    Therapy_SetEprReliefCmh2o(v);
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }

  if (strncmp(cmd, "therapy set epr_level ", 20U) == 0)
  {
    long v;
    if (!Telem_ParseLong(&cmd[20], &v) || (v < 1L) || (v > 3L))
    {
      Telem_PushErr("epr_level must be 1, 2, or 3");
      return;
    }
    Therapy_SetEprLevel((uint8_t)v);
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }

  if (strncmp(cmd, "therapy set epr_type ", 21U) == 0)
  {
    if (strcmp(&cmd[21], "fulltime") == 0)
    {
      Therapy_SetEprType(0U);
      Telem_PushAck(cmd);
      Telem_PushStatus();
      return;
    }
    if (strcmp(&cmd[21], "ramp") == 0)
    {
      Therapy_SetEprType(1U);
      Telem_PushAck(cmd);
      Telem_PushStatus();
      return;
    }
    long v;
    if (Telem_ParseLong(&cmd[21], &v) && ((v == 0L) || (v == 1L)))
    {
      Therapy_SetEprType((uint8_t)v);
      Telem_PushAck(cmd);
      Telem_PushStatus();
      return;
    }
    Telem_PushErr("epr_type fulltime|ramp|0|1");
    return;
  }

  if (strcmp(cmd, "therapy maskon") == 0)
  {
    Therapy_ForceMaskOn();
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }

  if (strncmp(cmd, "therapy set maskoff ", 20U) == 0)
  {
    long v;
    if (!Telem_ParseLong(&cmd[20], &v) || ((v != 0L) && (v != 1L)))
    {
      Telem_PushErr("maskoff must be 0 or 1");
      return;
    }
    Therapy_SetMaskOffEnabled(v != 0L);
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }

  if (strncmp(cmd, "therapy set maskoff_", 19U) == 0)
  {
    extern volatile float    g_therapy_mask_off_enter_slm;
    extern volatile uint8_t  g_therapy_mask_off_enter_vent_ref;
    extern volatile float    g_therapy_mask_off_enter_margin_slm;
    extern volatile uint32_t g_therapy_mask_off_enter_ms;
    extern volatile uint32_t g_therapy_mask_off_flow_stale_ticks_max;
    extern volatile uint8_t  g_therapy_mask_off_require_press;
    extern volatile float    g_therapy_mask_off_press_drop_cmh2o;
    extern volatile float    g_therapy_mask_off_press_abs_cmh2o;
    extern volatile float    g_therapy_mask_off_press_sensor_allow_cmh2o;
    extern volatile float    g_therapy_mask_off_open_flow_slm;
    extern volatile uint8_t  g_therapy_mask_off_sustained_enable;
    extern volatile float    g_therapy_mask_off_sustained_margin_slm;
    extern volatile uint32_t g_therapy_mask_off_sustained_ms;
    extern volatile float    g_therapy_mask_off_enter_min_frac;
    extern volatile uint8_t  g_therapy_mask_off_breath_inhibit_enable;
    extern volatile uint32_t g_therapy_mask_off_breath_inhibit_ms;
    extern volatile uint8_t  g_therapy_mask_off_ratio_enable;
    extern volatile uint32_t g_therapy_mask_off_verify_ms;
    extern volatile float    g_therapy_mask_off_verify_frac;
    extern volatile uint32_t g_therapy_mask_off_clear_backoff_ms;
    extern volatile uint32_t g_therapy_mask_off_clear_ms_max;
    extern volatile uint8_t  g_therapy_mask_off_rpm_ratio_enable;
    extern volatile float    g_therapy_mask_off_rpm_ratio;
    extern volatile uint32_t g_therapy_mask_off_rpm_ratio_ms;
    extern volatile uint8_t  g_therapy_mask_off_press_only_enable;
    extern volatile int32_t  g_therapy_mask_off_press_only_rpm;
    extern volatile float    g_therapy_mask_off_press_only_frac;
    extern volatile float    g_therapy_mask_off_press_only_cmh2o;
    extern volatile uint32_t g_therapy_mask_off_press_only_ms;
    extern volatile uint32_t g_therapy_mask_off_flow_clear_ms;
    extern volatile uint32_t g_therapy_mask_off_max_hold_ms;

    char name[40];
    char value_text[32];
    float fv;
    long iv;
    int matched = sscanf(&cmd[19], "%39s %31s", name, value_text);
    if (matched != 2)
    {
      Telem_PushErr("therapy set maskoff_<name> <value>");
      return;
    }

    if ((strcmp(name, "enter_slm") == 0) && Telem_ParseFloat(value_text, &fv))
    {
      if (fv < 5.0f) { fv = 5.0f; }
      if (fv > 80.0f) { fv = 80.0f; }
      g_therapy_mask_off_enter_slm = fv;
      Telem_PushAck(cmd);
      return;
    }
    /* 0 restores the pre-2026-09-23 fixed enter_slm threshold. With it on,
     * the threshold is vent_flow(setpoint) + enter_margin, floored at
     * enter_slm - see the note in therapy.c. */
    if ((strcmp(name, "enter_vent_ref") == 0)
        && Telem_ParseLong(value_text, &iv))
    {
      g_therapy_mask_off_enter_vent_ref = (iv != 0L) ? 1U : 0U;
      Telem_PushAck(cmd);
      return;
    }
    /* Patient-flow headroom above the vent before mask-off will arm. The
     * clamp matches maskOffEnterThresholdSlm(). */
    if ((strcmp(name, "enter_margin") == 0)
        && Telem_ParseFloat(value_text, &fv))
    {
      if (fv < 10.0f) { fv = 10.0f; }
      if (fv > 80.0f) { fv = 80.0f; }
      g_therapy_mask_off_enter_margin_slm = fv;
      Telem_PushAck(cmd);
      return;
    }
    if ((strcmp(name, "enter_ms") == 0) && Telem_ParseLong(value_text, &iv))
    {
      if (iv < 100L) { iv = 100L; }
      if (iv > 10000L) { iv = 10000L; }
      g_therapy_mask_off_enter_ms = (uint32_t)iv;
      Telem_PushAck(cmd);
      return;
    }
    /* Consecutive held flow ticks the fast path tolerates (5 ms each). 0
     * makes any held tick invalidate the flow again - not recommended, see
     * therapy.c. */
    if ((strcmp(name, "flow_stale_ticks_max") == 0)
        && Telem_ParseLong(value_text, &iv))
    {
      if (iv < 0L) { iv = 0L; }
      if (iv > 40L) { iv = 40L; }
      g_therapy_mask_off_flow_stale_ticks_max = (uint32_t)iv;
      Telem_PushAck(cmd);
      return;
    }
    if ((strcmp(name, "require_press") == 0) && Telem_ParseLong(value_text, &iv))
    {
      g_therapy_mask_off_require_press = (iv != 0L) ? 1U : 0U;
      Telem_PushAck(cmd);
      return;
    }
    if ((strcmp(name, "press_drop") == 0) && Telem_ParseFloat(value_text, &fv))
    {
      if (fv < 0.3f) { fv = 0.3f; }
      if (fv > 10.0f) { fv = 10.0f; }
      g_therapy_mask_off_press_drop_cmh2o = fv;
      Telem_PushAck(cmd);
      return;
    }
    if ((strcmp(name, "press_abs") == 0) && Telem_ParseFloat(value_text, &fv))
    {
      if (fv < 0.5f) { fv = 0.5f; }
      if (fv > 15.0f) { fv = 15.0f; }
      g_therapy_mask_off_press_abs_cmh2o = fv;
      Telem_PushAck(cmd);
      return;
    }
    /* Sustained-flow path (therapy.c): 3 s mean total flow >= vent + margin. */
    if ((strcmp(name, "sustained_enable") == 0) && Telem_ParseLong(value_text, &iv))
    {
      g_therapy_mask_off_sustained_enable = (iv != 0L) ? 1U : 0U;
      Telem_PushAck(cmd);
      return;
    }
    if ((strcmp(name, "sustained_margin") == 0) && Telem_ParseFloat(value_text, &fv))
    {
      if (fv < 20.0f) { fv = 20.0f; }
      if (fv > 150.0f) { fv = 150.0f; }
      g_therapy_mask_off_sustained_margin_slm = fv;
      Telem_PushAck(cmd);
      return;
    }
    if ((strcmp(name, "sustained_ms") == 0) && Telem_ParseLong(value_text, &iv))
    {
      if (iv < 200L) { iv = 200L; }
      if (iv > 5000L) { iv = 5000L; }
      g_therapy_mask_off_sustained_ms = (uint32_t)iv;
      Telem_PushAck(cmd);
      return;
    }
    /* Open-circuit evidence by flow for the fast path (therapy.c); 0 = off. */
    if ((strcmp(name, "open_flow") == 0) && Telem_ParseFloat(value_text, &fv))
    {
      if (fv < 0.0f) { fv = 0.0f; }
      if ((fv > 0.0f) && (fv < 90.0f)) { fv = 90.0f; }   /* never down into breathing flows */
      if (fv > 250.0f) { fv = 250.0f; }
      g_therapy_mask_off_open_flow_slm = fv;
      Telem_PushAck(cmd);
      return;
    }
    /* Device-side sensor allowance on top of press_abs for the clipped /
     * speed-limited fast-path exception (hose drop at open-circuit flow).
     * Clamp matches maskOffOpenCircuitPressure(). */
    if ((strcmp(name, "press_sensor_allow") == 0)
        && Telem_ParseFloat(value_text, &fv))
    {
      if (fv < 0.0f) { fv = 0.0f; }
      if (fv > 6.0f) { fv = 6.0f; }
      g_therapy_mask_off_press_sensor_allow_cmh2o = fv;
      Telem_PushAck(cmd);
      return;
    }
    if ((strcmp(name, "enter_min_frac") == 0) && Telem_ParseFloat(value_text, &fv))
    {
      if (fv < 0.15f) { fv = 0.15f; }
      if (fv > 0.90f) { fv = 0.90f; }
      g_therapy_mask_off_enter_min_frac = fv;
      Telem_PushAck(cmd);
      return;
    }
    if ((strcmp(name, "breath_inhibit") == 0) && Telem_ParseLong(value_text, &iv))
    {
      g_therapy_mask_off_breath_inhibit_enable = (iv != 0L) ? 1U : 0U;
      Telem_PushAck(cmd);
      return;
    }
    if ((strcmp(name, "breath_inhibit_ms") == 0) && Telem_ParseLong(value_text, &iv))
    {
      if (iv < 0L) { iv = 0L; }
      if (iv > 30000L) { iv = 30000L; }
      g_therapy_mask_off_breath_inhibit_ms = (uint32_t)iv;
      Telem_PushAck(cmd);
      return;
    }
    if ((strcmp(name, "ratio_enable") == 0) && Telem_ParseLong(value_text, &iv))
    {
      g_therapy_mask_off_ratio_enable = (iv != 0L) ? 1U : 0U;
      Telem_PushAck(cmd);
      return;
    }
    /* 0 disables the probationary resume check entirely. */
    if ((strcmp(name, "verify_ms") == 0) && Telem_ParseLong(value_text, &iv))
    {
      if (iv < 0L) { iv = 0L; }
      if (iv > 20000L) { iv = 20000L; }
      g_therapy_mask_off_verify_ms = (uint32_t)iv;
      Telem_PushAck(cmd);
      return;
    }
    if ((strcmp(name, "verify_frac") == 0) && Telem_ParseFloat(value_text, &fv))
    {
      if (fv < 0.20f) { fv = 0.20f; }
      if (fv > 0.90f) { fv = 0.90f; }
      g_therapy_mask_off_verify_frac = fv;
      Telem_PushAck(cmd);
      return;
    }
    if ((strcmp(name, "clear_backoff_ms") == 0) && Telem_ParseLong(value_text, &iv))
    {
      if (iv < 0L) { iv = 0L; }
      if (iv > 10000L) { iv = 10000L; }
      g_therapy_mask_off_clear_backoff_ms = (uint32_t)iv;
      Telem_PushAck(cmd);
      return;
    }
    if ((strcmp(name, "clear_ms_max") == 0) && Telem_ParseLong(value_text, &iv))
    {
      if (iv < 300L) { iv = 300L; }
      if (iv > 20000L) { iv = 20000L; }
      g_therapy_mask_off_clear_ms_max = (uint32_t)iv;
      Telem_PushAck(cmd);
      return;
    }
    if ((strcmp(name, "rpm_ratio_enable") == 0) && Telem_ParseLong(value_text, &iv))
    {
      g_therapy_mask_off_rpm_ratio_enable = (iv != 0L) ? 1U : 0U;
      Telem_PushAck(cmd);
      return;
    }
    if ((strcmp(name, "rpm_ratio") == 0) && Telem_ParseFloat(value_text, &fv))
    {
      if (fv < 1.10f) { fv = 1.10f; }
      if (fv > 3.0f) { fv = 3.0f; }
      g_therapy_mask_off_rpm_ratio = fv;
      Telem_PushAck(cmd);
      return;
    }
    if ((strcmp(name, "rpm_ratio_ms") == 0) && Telem_ParseLong(value_text, &iv))
    {
      if (iv < 1000L) { iv = 1000L; }
      if (iv > 10000L) { iv = 10000L; }
      g_therapy_mask_off_rpm_ratio_ms = (uint32_t)iv;
      Telem_PushAck(cmd);
      return;
    }
    if ((strcmp(name, "press_only_enable") == 0) && Telem_ParseLong(value_text, &iv))
    {
      g_therapy_mask_off_press_only_enable = (iv != 0L) ? 1U : 0U;
      Telem_PushAck(cmd);
      return;
    }
    if ((strcmp(name, "press_only_rpm") == 0) && Telem_ParseLong(value_text, &iv))
    {
      if (iv < 3000L) { iv = 3000L; }
      if (iv > 36000L) { iv = 36000L; }
      g_therapy_mask_off_press_only_rpm = (int32_t)iv;
      Telem_PushAck(cmd);
      return;
    }
    if ((strcmp(name, "press_only_frac") == 0) && Telem_ParseFloat(value_text, &fv))
    {
      if (fv < 0.20f) { fv = 0.20f; }
      if (fv > 0.80f) { fv = 0.80f; }
      g_therapy_mask_off_press_only_frac = fv;
      Telem_PushAck(cmd);
      return;
    }
    if ((strcmp(name, "press_only_cmh2o") == 0) && Telem_ParseFloat(value_text, &fv))
    {
      if (fv < 0.5f) { fv = 0.5f; }
      if (fv > 8.0f) { fv = 8.0f; }
      g_therapy_mask_off_press_only_cmh2o = fv;
      Telem_PushAck(cmd);
      return;
    }
    /* Clamped again at use so it stays under the safety low-pressure timeout. */
    if ((strcmp(name, "press_only_ms") == 0) && Telem_ParseLong(value_text, &iv))
    {
      if (iv < 500L) { iv = 500L; }
      if (iv > 3500L) { iv = 3500L; }
      g_therapy_mask_off_press_only_ms = (uint32_t)iv;
      Telem_PushAck(cmd);
      return;
    }
    if ((strcmp(name, "flow_clear_ms") == 0) && Telem_ParseLong(value_text, &iv))
    {
      if (iv < 200L) { iv = 200L; }
      if (iv > 20000L) { iv = 20000L; }
      g_therapy_mask_off_flow_clear_ms = (uint32_t)iv;
      Telem_PushAck(cmd);
      return;
    }
    /* 0 disables the watchdog (bench only: mask-off can then latch). */
    if ((strcmp(name, "max_hold_ms") == 0) && Telem_ParseLong(value_text, &iv))
    {
      if (iv < 0L) { iv = 0L; }
      if (iv > 600000L) { iv = 600000L; }
      g_therapy_mask_off_max_hold_ms = (uint32_t)iv;
      Telem_PushAck(cmd);
      return;
    }

    Telem_PushErr("unknown maskoff_ param");
    return;
  }

  if (strncmp(cmd, "blower manual start ", 20U) == 0)
  {
    int32_t rpm = 0;
    uint16_t ramp_ms = 0U;
    if (!Telem_ParseManualSpeed(&cmd[20], &rpm, &ramp_ms))
    {
      Telem_PushErr("manual start needs rpm ramp_ms");
      return;
    }
    Telem_BlowerManualStart(rpm, ramp_ms);
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }

  if (strncmp(cmd, "blower manual speed ", 20U) == 0)
  {
    int32_t rpm = 0;
    uint16_t ramp_ms = 0U;
    if (!Telem_ParseManualSpeed(&cmd[20], &rpm, &ramp_ms))
    {
      Telem_PushErr("manual speed needs rpm ramp_ms");
      return;
    }
    BlowerCtrl_ManualSetSpeed(rpm, ramp_ms);
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }

  if (strcmp(cmd, "blower manual stop") == 0)
  {
    BlowerCtrl_ManualStop();
    Telem_PushAck(cmd);
    Telem_PushStatus();
    return;
  }

  if (strncmp(cmd, "bench play ", 11U) == 0)
  {
    Telem_HandleBenchPlay(cmd, &cmd[11]);
    return;
  }

  if (strncmp(cmd, "blower set ", 11U) == 0)
  {
    char name[32];
    char value_text[32];
    float fv;
    long iv;
    int matched = sscanf(&cmd[11], "%31s %31s", name, value_text);
    if (matched != 2)
    {
      Telem_PushErr("blower set needs name value");
      return;
    }

    if (Telem_ParseLong(value_text, &iv) && Telem_SetIntParam(name, iv))
    {
      Telem_PushAck(cmd);
      return;
    }
    if (Telem_ParseFloat(value_text, &fv) && Telem_SetFloatParam(name, fv))
    {
      Telem_PushAck(cmd);
      return;
    }

    Telem_PushErr("unknown blower param");
    return;
  }

  if (strncmp(cmd, "leak set ", 9U) == 0)
  {
    char name[32];
    char value_text[32];
    float fv;
    long iv;
    int matched = sscanf(&cmd[9], "%31s %31s", name, value_text);
    if (matched != 2)
    {
      Telem_PushErr("leak set needs name value");
      return;
    }

    if (Telem_ParseLong(value_text, &iv) && Telem_SetLeakIntParam(name, iv))
    {
      Telem_PushAck(cmd);
      Telem_PushStatus();
      return;
    }
    if (Telem_ParseFloat(value_text, &fv) && Telem_SetLeakFloatParam(name, fv))
    {
      Telem_PushAck(cmd);
      Telem_PushStatus();
      return;
    }

    Telem_PushErr("unknown leak param");
    return;
  }

  if (strncmp(cmd, "physio set ", 11U) == 0)
  {
    char name[32];
    char value_text[32];
    long iv;
    int matched = sscanf(&cmd[11], "%31s %31s", name, value_text);
    if (matched != 2)
    {
      Telem_PushErr("physio set needs name value");
      return;
    }
    if ((strcmp(name, "fot_cpap_enable") == 0) && Telem_ParseLong(value_text, &iv))
    {
      extern volatile uint8_t g_physio_fot_cpap_enable;
      g_physio_fot_cpap_enable = (iv != 0L) ? 1U : 0U;
      Telem_PushAck(cmd);
      return;
    }
    if ((strcmp(name, "fot_enable") == 0) && Telem_ParseLong(value_text, &iv))
    {
      PhysioTelem_SetFotEnable(iv != 0L);
      Telem_PushAck(cmd);
      return;
    }
    if ((strcmp(name, "telem_enable") == 0) && Telem_ParseLong(value_text, &iv))
    {
#if PHYSIO_TELEM_ENABLED
      g_physio_telem_enable = (iv != 0L) ? 1U : 0U;
      if (iv == 0L)
      {
        PhysioTelem_SetFotEnable(false);
      }
#endif
      Telem_PushAck(cmd);
      return;
    }
    Telem_PushErr("unknown physio param");
    return;
  }

  /* apap diag - read-only breath-stamp / episode-exit state (R11). */
  if (strncmp(cmd, "apap diag", 9U) == 0)
  {
    char line[384];
    const int n = Apap_FormatDiag(line, (unsigned)sizeof(line));
    if (n > 0)
    {
      Telem_BufPush((const uint8_t *)line, (uint32_t)n);
    }
    return;
  }

  if (strncmp(cmd, "apap set ", 9U) == 0)
  {
    char name[40];
    char value_text[32];
    float fv;
    long iv;
    int matched = sscanf(&cmd[9], "%39s %31s", name, value_text);
    if (matched != 2)
    {
      Telem_PushErr("apap set needs name value");
      return;
    }
    if (Telem_ParseLong(value_text, &iv) && Telem_SetApapIntParam(name, iv))
    {
      Telem_PushAck(cmd);
      return;
    }
    if (Telem_ParseFloat(value_text, &fv) && Telem_SetApapFloatParam(name, fv))
    {
      Telem_PushAck(cmd);
      return;
    }
    Telem_PushErr("unknown apap param");
    return;
  }

  /* flow set btps_enable 0|1
   * flow set btps_amb_mbar <mbar>
   * flow set alt_enable 0|1
   * flow set zero_offset <mbar>
   * flow autozero
   */
  if (strncmp(cmd, "flow autozero", 13U) == 0)
  {
    if (Therapy_GetState() != (uint8_t)THERAPY_STATE_IDLE)
    {
      Telem_PushErr("autozero refused: therapy running");
      return;
    }
    Flow_AutoZero();
    Telem_PushAck(cmd);
    return;
  }

  /* flow spike - snapshot latched at the last Q_patient needle. If dp_raw /
   * dp_filt are far from zero the event is pneumatic or sensor; if they are
   * ~0 while zero/uninten moved it is arithmetic. gap_ms = event period. */
  if (strncmp(cmd, "flow spike", 10U) == 0)
  {
    char line[256];
    int n = snprintf(line, sizeof(line),
                     "# flow spike n=%lu at=%lu gap_ms=%lu q_pat=%.2f "
                     "q_tot=%.2f vent=%.2f unint=%.2f dp_raw=%.4f "
                     "dp_filt=%.4f zero=%.4f az=%lu stale=%lu medrej=%lu\r\n",
                     (unsigned long)g_flow_spike_count,
                     (unsigned long)g_flow_spike_ms,
                     (unsigned long)g_flow_spike_gap_ms,
                     (double)g_flow_spike_q_patient,
                     (double)g_flow_spike_q_total,
                     (double)g_flow_spike_vent,
                     (double)g_flow_spike_uninten,
                     (double)g_flow_spike_dp_raw,
                     (double)g_flow_spike_dp_filt,
                     (double)g_flow_spike_zero,
                     (unsigned long)g_flow_spike_az_count,
                     (unsigned long)g_flow_spike_stale,
                     (unsigned long)g_flow_spike_medrej);
    if ((n > 0) && ((size_t)n < sizeof(line)))
    {
      Telem_BufPush((const uint8_t *)line, (uint32_t)n);
    }
    return;
  }

  /* flow diag - is a Q_patient needle from the zero trim or from stale
   * sensor samples? az_count rising at the spike means auto-zero;
   * stale/median counters rising means the AMS5935 read path. */
  if (strncmp(cmd, "flow diag", 9U) == 0)
  {
    char line[256];
    int n = snprintf(line, sizeof(line),
                     "# flow diag zero=%.4f az_en=%u az_cnt=%lu az_latch=%u "
                     "az_slew=%u az_quiet=%u pp=%.4f stale=%lu run_max=%lu "
                     "medtaps=%u medrej=%lu slewrej=%lu imprej=%lu\r\n",
                     (double)Flow_GetZeroOffset(),
                     (unsigned)(Flow_IsAutoZeroEnabled() ? 1U : 0U),
                     (unsigned long)Flow_GetAutoZeroCount(),
                     (unsigned)Flow_GetAutoZeroLatched(),
                     (unsigned)g_flow_az_slewing,
                     (unsigned)g_flow_az_quiet_now,
                     (double)g_flow_az_last_pp_mbar,
                     (unsigned long)g_flow_stale_count,
                     (unsigned long)g_flow_stale_run_max,
                     (unsigned)g_flow_median_taps,
                     (unsigned long)g_flow_median_reject_count,
                     (unsigned long)g_flow_slew_reject_count,
                     (unsigned long)g_flow_impulse_reject_count);
    if ((n > 0) && ((size_t)n < sizeof(line)))
    {
      Telem_BufPush((const uint8_t *)line, (uint32_t)n);
    }
    return;
  }

  if (strncmp(cmd, "flow set ", 9U) == 0)
  {
    char name[32];
    char value_text[32];
    float fv;
    long iv;
    int matched = sscanf(&cmd[9], "%31s %31s", name, value_text);
    if (matched != 2)
    {
      Telem_PushErr("flow set needs name value");
      return;
    }

    if ((strcmp(name, "btps_enable") == 0) && Telem_ParseLong(value_text, &iv))
    {
      Flow_SetBtpsEnable(iv != 0L);
      Telem_PushAck(cmd);
      return;
    }
    if ((strcmp(name, "btps_amb_mbar") == 0) && Telem_ParseFloat(value_text, &fv))
    {
      Flow_SetBtpsAmbientMbar(fv);
      Telem_PushAck(cmd);
      return;
    }
    if ((strcmp(name, "alt_enable") == 0) && Telem_ParseLong(value_text, &iv))
    {
      Flow_SetAltitudeEnable(iv != 0L);
      Telem_PushAck(cmd);
      return;
    }
    if ((strcmp(name, "az_enable") == 0) && Telem_ParseLong(value_text, &iv))
    {
      Flow_SetAutoZeroEnable(iv != 0L);
      Telem_PushAck(cmd);
      return;
    }
    if ((strcmp(name, "az_done") == 0) && Telem_ParseFloat(value_text, &fv))
    {
      g_flow_az_done_mbar = fv;
      Telem_PushAck(cmd);
      return;
    }
    if ((strcmp(name, "az_slew") == 0) && Telem_ParseFloat(value_text, &fv))
    {
      g_flow_az_slew_mbar_per_s = fv;
      Telem_PushAck(cmd);
      return;
    }
    if ((strcmp(name, "az_quiet_pp") == 0) && Telem_ParseFloat(value_text, &fv))
    {
      g_flow_az_quiet_pp_mbar = fv;
      Telem_PushAck(cmd);
      return;
    }
    if ((strcmp(name, "median_enable") == 0) && Telem_ParseLong(value_text, &iv))
    {
      g_flow_median_enable = (iv != 0L) ? 1U : 0U;
      Telem_PushAck(cmd);
      return;
    }
    /* Flow element LUT: 0 = Round Hole_2 (default), 1 = ~150 slm element. */
    if ((strcmp(name, "element") == 0) && Telem_ParseLong(value_text, &iv))
    {
      if ((iv != 0L) && (iv != 1L))
      {
        Telem_PushErr("flow set element 0|1");
        return;
      }
      g_flow_element = (uint8_t)iv;
      Telem_PushAck(cmd);
      return;
    }
    /* Fan-map flow estimate while the dP sensor is saturated (flow_sensor.c). */
    if ((strcmp(name, "fan_est") == 0) && Telem_ParseLong(value_text, &iv))
    {
      g_flow_fan_est_enable = (iv != 0L) ? 1U : 0U;
      Telem_PushAck(cmd);
      return;
    }
    if ((strncmp(name, "fan_", 4U) == 0) && (name[5] == '\0') && Telem_ParseFloat(value_text, &fv))
    {
      switch (name[4])
      {
        case 'a': g_flow_fan_a = fv; break;
        case 'b': g_flow_fan_b = fv; break;
        case 'c': g_flow_fan_c = fv; break;
        case 'd': g_flow_fan_d = fv; break;
        default:  Telem_PushErr("flow set fan_a|fan_b|fan_c|fan_d <coef>"); return;
      }
      Telem_PushAck(cmd);
      return;
    }
    if ((strcmp(name, "median_taps") == 0) && Telem_ParseLong(value_text, &iv))
    {
      g_flow_median_taps = (iv >= 5L) ? 5U : 3U;
      Telem_PushAck(cmd);
      return;
    }
    if ((strcmp(name, "dp_max_slew") == 0) && Telem_ParseFloat(value_text, &fv))
    {
      g_flow_dp_max_slew_mbar_per_s = fv;
      Telem_PushAck(cmd);
      return;
    }
    if ((strcmp(name, "q_max_step") == 0) && Telem_ParseFloat(value_text, &fv))
    {
      g_flow_q_max_step_slm = fv;
      Telem_PushAck(cmd);
      return;
    }
    if ((strcmp(name, "zero_offset") == 0) && Telem_ParseFloat(value_text, &fv))
    {
      Flow_SetZeroOffset(fv);
      Telem_PushAck(cmd);
      return;
    }

    Telem_PushErr("unknown flow param");
    return;
  }

  if (strncmp(cmd, "tubecomp set ", 13U) == 0)
  {
    char name[32];
    char value_text[32];
    float fv;
    long iv;
    int matched = sscanf(&cmd[13], "%31s %31s", name, value_text);
    if (matched != 2)
    {
      Telem_PushErr("tubecomp set needs name value");
      return;
    }

    if (Telem_ParseLong(value_text, &iv) && Telem_SetTubeCompParam(name, (float)iv))
    {
      Telem_PushAck(cmd);
      return;
    }
    if (Telem_ParseFloat(value_text, &fv) && Telem_SetTubeCompParam(name, fv))
    {
      Telem_PushAck(cmd);
      return;
    }

    Telem_PushErr("unknown tubecomp param");
    return;
  }

  if (strncmp(cmd, "foc set ", 8U) == 0)
  {
    char name[32];
    char value_text[32];
    long iv;
    int matched = sscanf(&cmd[8], "%31s %31s", name, value_text);
    if (matched != 2)
    {
      Telem_PushErr("foc set needs name value");
      return;
    }
    if (!Telem_ParseLong(value_text, &iv))
    {
      Telem_PushErr("foc value must be integer");
      return;
    }
    if (Telem_SetFocParam(name, iv))
    {
      Telem_PushAck(cmd);
      return;
    }
    Telem_PushErr("unknown foc param");
    return;
  }

  Telem_PushErr("unknown command");
}

void Telem_ProcessCommands(void)
{
  for (uint8_t batch = 0U; batch < TELEM_CMD_BATCH_MAX; batch++)
  {
    char cmd[sizeof(s_cmd_queue[0])];

    uint32_t primask = __get_PRIMASK();
    __disable_irq();
    if (s_cmd_head == s_cmd_tail)
    {
      if (primask == 0U)
      {
        __enable_irq();
      }
      return;
    }
    (void)strncpy(cmd, s_cmd_queue[s_cmd_tail], sizeof(cmd) - 1U);
    cmd[sizeof(cmd) - 1U] = '\0';
    s_cmd_tail = (uint8_t)((s_cmd_tail + 1U) % TELEM_CMD_QUEUE_LEN);
    if (primask == 0U)
    {
      __enable_irq();
    }

    Telem_ProcessOneCommand(cmd);
  }
}

/* Drop WHOLE lines, never bytes.
 *
 * This used to evict the oldest byte to make room, which in a line-oriented
 * protocol cannot produce anything but garbage: the reader sees two half
 * lines spliced together, parses them as one, and gets values that are
 * syntactically fine and physically impossible. A bench capture on
 * 2026-09-22 reported q_filt excursions of 27539 slm and ff_raw swings of
 * 283014 rpm from exactly this, and the numbers only looked wrong because
 * they were absurd - a subtler splice would have been believed.
 *
 * Refusing the whole push instead means every line that arrives is complete.
 * Loss shows up as a gap in the millisecond column, which any consumer can
 * see, rather than as plausible nonsense. */
static void Telem_BufPush(const uint8_t *data, uint32_t len)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  const uint32_t used = (s_head - s_tail) & TELEM_BUF_MASK;
  const uint32_t room = TELEM_BUF_MASK - used;   /* one slot stays empty */

  if (len > room)
  {
    g_telem_dropped_count++;                     /* counts LINES now */
  }
  else
  {
    for (uint32_t i = 0U; i < len; i++)
    {
      s_buf[s_head] = data[i];
      s_head = (s_head + 1U) & TELEM_BUF_MASK;
    }
  }

  if (primask == 0U)
  {
    __enable_irq();
  }
}

static uint32_t Telem_BufPopChunk(uint8_t *dst, uint32_t maxLen)
{
  uint32_t primask = __get_PRIMASK();
  __disable_irq();

  uint32_t head = s_head;
  uint32_t tail = s_tail;
  uint32_t avail;

  if (head >= tail)
  {
    avail = head - tail;
  }
  else
  {
    avail = TELEM_BUF_SIZE - tail;
  }

  if (avail > maxLen)
  {
    avail = maxLen;
  }

  for (uint32_t i = 0U; i < avail; i++)
  {
    dst[i] = s_buf[(tail + i) & TELEM_BUF_MASK];
  }
  s_tail = (tail + avail) & TELEM_BUF_MASK;

  if (primask == 0U)
  {
    __enable_irq();
  }

  return avail;
}

void Telem_Init(void)
{
  s_head = 0U;
  s_tail = 0U;
  s_rx_len = 0U;
  g_telem_sample_count = 0U;
  g_telem_dropped_count = 0U;
  s_inited = 1U;
  Telem_PushRaw("# telem ready (T,... | PH,... | R,...)\r\n");
}

void Telem_PushSample(float pressure_sensor_cmh2o, float patient_flow_slm,
                      int32_t pid_rpm_cmd, int32_t mech_rpm,
                      float pressure_mask_cmh2o)
{
  if ((s_inited == 0U) || (g_telem_enabled == 0U))
  {
    return;
  }

  char line[TELEM_FORMAT_BUF];
  uint32_t now = HAL_GetTick();
  int n = snprintf(line, sizeof(line), "T,%lu,%.2f,%.2f,%ld,%ld,%.2f\r\n",
                   (unsigned long)now,
                   (double)pressure_sensor_cmh2o,
                   (double)patient_flow_slm,
                   (long)pid_rpm_cmd,
                   (long)mech_rpm,
                   (double)pressure_mask_cmh2o);
  if ((n > 0) && ((size_t)n < sizeof(line)))
  {
    Telem_BufPush((const uint8_t *)line, (uint32_t)n);
  }

  /* Optional second line: which controller term produced the command.
   *   D,<ms>,<ff>,<dip>,<p>,<i>,<d>,<error>,<rise>,<expflow>,
   *     <p_equiv>,<hose>,<fan>,<q_filt>,<ff_raw>
   * ff+dip+p+i+d is the pre-clamp output, so a feature in P_mask can be
   * attributed to a block instead of inferred. Off by default - it roughly
   * doubles the stream - enable with:  blower set telem_terms 1
   *
   * The last five are the physics FF's own internals, added 2026-09-22 after
   * three bench sweeps (c_fan, then the FF slew rate) all failed to move the
   * FF swing at 15 cmH2O. Inferring the FF's state from its output was the
   * mistake - these show it directly:
   *   p_equiv  what the FF thinks it must generate = sp + hose + fan
   *   hose     the hose term at the current flow
   *   fan      the c_fan*Q^2 droop term - if this does not move when c_fan is
   *            swept, c_fan is not reaching the model
   *   q_filt   the flow the FF actually used, after its own filtering. If the
   *            breath is not in here, no downstream gain can compensate.
   *   ff_raw   the FF target BEFORE slew limiting. ff_raw minus ff is exactly
   *            how much the slew is holding back, which is the question the
   *            physics_ff_rise sweep could not answer. */
  if (g_telem_terms_enable != 0U)
  {
    n = snprintf(line, sizeof(line),
                 "D,%lu,%.0f,%.0f,%.0f,%.0f,%.0f,%.3f,%.0f,%.0f"
                 ",%.2f,%.3f,%.3f,%.1f,%.0f\r\n",
                 (unsigned long)now,
                 (double)g_blower_ctrl_ff_term,
                 (double)g_blower_ctrl_ff_dip_term,
                 (double)g_blower_ctrl_p_term,
                 (double)g_blower_ctrl_i_term,
                 (double)g_blower_ctrl_d_term,
                 (double)g_blower_ctrl_error_cmh2o,
                 (double)g_blower_ctrl_ff_rise_term,
                 (double)g_blower_ctrl_ff_exp_flow_term,
                 (double)g_blower_ctrl_physics_ff_p_equiv_cmh2o,
                 (double)g_blower_ctrl_physics_ff_hose_drop_cmh2o,
                 (double)g_blower_ctrl_physics_ff_fan_load_cmh2o,
                 (double)g_blower_ctrl_physics_ff_q_filt_slm,
                 (double)g_blower_ctrl_physics_ff_rpm_raw);
    if ((n > 0) && ((size_t)n < sizeof(line)))
    {
      Telem_BufPush((const uint8_t *)line, (uint32_t)n);
    }
  }

  /* Motor electrical line, 25 Hz, opt-in.
   *   B,<ms>,<vbus_mv>,<iq_ma>,<iq_ref_ma>,<id_ma>,<mech_rpm>,<ref_rpm>,
   *     <mc_state>,<faults>,<q_total>,<dp_raw>,<slew_rej>,<stale>,<stale_run>,
   *     <sat>,<sat_n>,<occ_faults>,<holdoff>,<vq_mv>,<vd_mv>,<iq_lim_ma>
   *
   * holdoff (2026-09-25): CM4's restart-while-coasting hold-off still to run,
   * ms; 1 = start pending, waiting for IDLE; 0 = none. Appended last so
   * parsers indexing the earlier fields are unaffected.
   *
   * vq_mv, vd_mv (2026-09-29): rotor-frame stator voltage, mV phase 0-to-peak,
   * ~16 ms filtered on CM4, 0 outside RUN. For measuring Lq/Rs/flux in the
   * running motor - see the note in blower_ipc.h. Commanded, not measured:
   * valid below the over-modulation region (~37.7 krpm). Appended last.
   *
   * iq_lim_ma (2026-10-01): speed-PI current limit in force, mA. Speed-
   * dependent (foc set iq_boost_ma / iq_boost_lo_rpm / iq_boost_hi_rpm),
   * so iq_ref sitting on it means the drive is current-limited at that
   * speed. 0 from a CM4 built before it. Appended last.
   *
   * Exists to settle one question: after the speed ceiling went to 40000 the
   * motor still ran 2300-4000 rpm under command, and speed alone cannot say
   * whether that is the bus sagging or the q-axis current saturating. iq_ref
   * pinned at NOMINAL_CURRENT (1345 mA) with iq tracking it means torque
   * limited; a bus that drops under the same load means supply limited. Both
   * have been argued for from rpm alone, which is why this is here.
   *
   * mech_rpm and ref_rpm are CM4's own figures, not the CM7 command in the T
   * line, so the two sides can be compared rather than assumed equal - they
   * were not: the IPC header clamped at 36528 while CM4 clamped at 36012.
   *
   * q_total (added 2026-09-24) is Flow_GetSlm(), the RAW orifice reading of
   * total device flow. Nothing else on the stream carries it: the D line's
   * q_filt is the physics feed-forward's own filtered copy, and the FF only
   * runs in therapy. In open-loop manual mode q_filt freezes at whatever it
   * last held - measured holding 45.7 slm as the rotor went from 0 to 28000
   * rpm - so a fan-curve measurement read from it is pure fiction. This one
   * is live in every mode.
   *
   * The last four (added 2026-09-24) instrument the flow sensor's own input
   * guards, because q_total was then caught LATCHED: at 34000 rpm open loop
   * with the port opened fast, it read exactly 52.6 slm with zero variance
   * for 45 s while pressure fell from 22 to 9.4 cmH2O.
   *   dp_raw    ΔP as read, BEFORE the slew guard - so it keeps moving even
   *             when every sample is being rejected. dp_raw live while q_total
   *             is flat is the signature of the guard, not the sensor.
   *   slew_rej  cumulative g_flow_slew_reject_count; its rate is the reject
   *             rate.
   *   stale     1 if this tick's sample was dropped (slew reject or failed
   *             read - slew_rej tells which).
   *   stale_run consecutive dropped samples. A latch shows as this climbing
   *             without bound.
   * The suspicion that prompted these - the slew guard latching - was WRONG.
   * Reproduced, the guard rejected nothing while dp_raw itself froze: the
   * AMS5935 is ±5 mbar (~93 slm through the LUT), past that it flags
   * overflow, and the driver held the last good sample indefinitely.
   *   sat       1 if this tick's flow sample was clipped to full scale.
   *   sat_n     cumulative clipped samples.
   * The driver now reports full scale on over-range (a true lower bound)
   * instead of holding, and sat makes it visible rather than silent.
   *
   *   occ_faults  CM4 occurred (latched) fault bits, MC_* codes from
   *               mc_type.h. Added 2026-09-24 because a "blower that would
   *               not stop" at 34000 rpm could not be diagnosed without it.
   *               Read it WITH mc_state: the speed estimator is disabled in
   *               IDLE, FAULT_NOW and FAULT_OVER, so mech_rpm freezes at its
   *               last value there. A fault at speed therefore shows a steady
   *               mech_rpm with the power stage OFF - mc_state 10/11 and a
   *               non-zero occ_faults are what tell the truth. */
  if (g_telem_motor_enable != 0U)
  {
    static uint8_t s_motor_div = 0U;
    s_motor_div ^= 1U;
    if (s_motor_div == 0U)
    {
      extern BlowerIpcStatus_t g_blower_status;
      extern volatile float    g_flow_dp_raw_mbar;
      extern volatile uint32_t g_flow_slew_reject_count;
      extern volatile uint8_t  g_flow_stale_now;
      extern volatile uint32_t g_flow_stale_run;
      extern AMS5935_HandleTypeDef hamsFS;
      n = snprintf(line, sizeof(line),
                   "B,%lu,%u,%d,%d,%d,%ld,%ld,%u,%u,%.2f,%.3f,%lu,%u,%lu,%u,%lu,%u,%u,%d,%d,%u\r\n",
                   (unsigned long)now,
                   (unsigned)g_blower_status.bus_voltage_mv,
                   (int)g_blower_status.iq_ma,
                   (int)g_blower_status.iq_ref_ma,
                   (int)g_blower_status.id_ma,
                   (long)g_blower_status.mech_speed_rpm,
                   (long)g_blower_status.speed_ref_rpm,
                   (unsigned)g_blower_status.mc_state,
                   (unsigned)g_blower_status.current_faults,
                   (double)Flow_GetSlmCtrl(),   /* = Flow_GetSlm unless fan_est substitutes */
                   (double)g_flow_dp_raw_mbar,
                   (unsigned long)g_flow_slew_reject_count,
                   (unsigned)g_flow_stale_now,
                   (unsigned long)g_flow_stale_run,
                   (unsigned)AMS5935_WasSampleSaturated(&hamsFS),
                   (unsigned long)hamsFS.saturated_count,
                   (unsigned)g_blower_status.occurred_faults,
                   (unsigned)g_blower_status.restart_holdoff_ms,
                   (int)g_blower_status.vq_mv,
                   (int)g_blower_status.vd_mv,
                   (unsigned)g_blower_status.iq_lim_ma);
      if ((n > 0) && ((size_t)n < sizeof(line)))
      {
        Telem_BufPush((const uint8_t *)line, (uint32_t)n);
      }
    }
  }

  g_telem_sample_count++;
}

/* Mask-off transitions, pushed from therapy.c at the instant of the decision.
 *
 * This exists because the DBG_I log inside enterMaskOff() is unusable in
 * practice: debug text and telemetry share one USB CDC endpoint, and with the
 * terms line on the stream saturates it. A 75 s bench capture on 2026-09-23
 * held three mask-off events and not one of their log lines. Putting the event
 * in the telemetry buffer means it is kept or dropped on the same terms as the
 * T and D lines around it, and lands in the same file with the same clock.
 *
 * Everything the decision rested on is latched HERE rather than read back
 * later, because exitMaskOff() clears the via_* flags. By the time a PC tool
 * notices mask-off went active, the reason for it is already gone.
 *
 *   M,<ms>,<event>,<path>,<q_total>,<q_excess_min>,<p_sensor>,<p_mask>,
 *     <sp_slew>,<enter_thresh>,<rpm_ratio>,<rpm_ratio_ss>,<ratio10s>,<insp>
 *
 * enter_thresh is the |Q_total| threshold actually in force at that instant,
 * which since 2026-09-23 is vent-referenced and therefore moves with the
 * setpoint - so a capture shows the bar as well as the flow that cleared it.
 *
 * event 1 = enter, 0 = exit.
 * path  0 = probation re-park (the preceding mask-on was judged wrong)
 *       1 = flow fast path      (|Q_total| >= enter_slm, plus pressure gate)
 *       2 = Vivo leak ratio
 *       3 = pressure-only backstop
 *       4 = rpm-ratio backstop
 * q_total is TOTAL device flow (Flow_GetSlm), not patient flow - which is the
 * whole point: at high setpoints the vent alone is most of the enter
 * threshold. */
void Telem_PushMaskOff(uint8_t event, uint8_t path,
                       float q_total_slm, float q_excess_min_slm,
                       float p_sensor_cmh2o, float p_mask_cmh2o,
                       float sp_slew_cmh2o, float enter_thresh_slm,
                       float rpm_ratio, float rpm_ratio_ss,
                       int32_t leak_ratio_10s, uint8_t inspiring)
{
  if ((s_inited == 0U) || (g_telem_enabled == 0U))
  {
    return;
  }

  char line[128];
  uint32_t now = HAL_GetTick();
  int n = snprintf(line, sizeof(line),
                   "M,%lu,%u,%u,%.1f,%.1f,%.2f,%.2f,%.2f,%.2f,%.3f,%.0f,%ld,%u"
                   "\r\n",
                   (unsigned long)now,
                   (unsigned)event,
                   (unsigned)path,
                   (double)q_total_slm,
                   (double)q_excess_min_slm,
                   (double)p_sensor_cmh2o,
                   (double)p_mask_cmh2o,
                   (double)sp_slew_cmh2o,
                   (double)enter_thresh_slm,
                   (double)rpm_ratio,
                   (double)rpm_ratio_ss,
                   (long)leak_ratio_10s,
                   (unsigned)inspiring);
  if ((n > 0) && ((size_t)n < sizeof(line)))
  {
    Telem_BufPush((const uint8_t *)line, (uint32_t)n);
  }
}

void Telem_PushPhysiology(float vt_ml, float r_eff, float c_proxy_ml_cmh2o,
                          float rr_bpm, uint8_t fot_src,
                          float p_pp_cmh2o, float q_pp_slm)
{
  if ((s_inited == 0U) || (g_telem_enabled == 0U))
  {
    return;
  }

  char line[96];
  uint32_t now = HAL_GetTick();
  int n = snprintf(line, sizeof(line),
                   "PH,%lu,%.0f,%.3f,%.1f,%.1f,%u,%.3f,%.3f\r\n",
                   (unsigned long)now,
                   (double)vt_ml,
                   (double)r_eff,
                   (double)c_proxy_ml_cmh2o,
                   (double)rr_bpm,
                   (unsigned)fot_src,
                   (double)p_pp_cmh2o,
                   (double)q_pp_slm);
  if ((n > 0) && ((size_t)n < sizeof(line)))
  {
    Telem_BufPush((const uint8_t *)line, (uint32_t)n);
  }
}

void Telem_PushBenchRpm(int32_t rpm_cmd, int32_t rpm_act, float pressure_cmh2o)
{
  if (s_inited == 0U)
  {
    return;
  }

  char line[56];
  uint32_t now = HAL_GetTick();
  int n = snprintf(line, sizeof(line), "R,%lu,%ld,%ld,%.2f\r\n",
                   (unsigned long)now,
                   (long)rpm_cmd,
                   (long)rpm_act,
                   (double)pressure_cmh2o);
  if ((n > 0) && ((size_t)n < sizeof(line)))
  {
    Telem_BufPush((const uint8_t *)line, (uint32_t)n);
  }
}

void Telem_Pump(void)
{
  if (s_inited == 0U) { return; }
  if (hUsbDeviceFS.dev_state != USBD_STATE_CONFIGURED) { return; }

  USBD_CDC_HandleTypeDef *hcdc =
      (USBD_CDC_HandleTypeDef *)hUsbDeviceFS.pClassData;
  if (hcdc == NULL) { return; }
  if (hcdc->TxState != 0U) { return; }

  uint32_t n = Telem_BufPopChunk(s_tx_chunk, sizeof(s_tx_chunk));
  if (n == 0U) { return; }

  (void)CDC_Transmit_FS(s_tx_chunk, (uint16_t)n);
}

#endif /* TELEM_ENABLED */
