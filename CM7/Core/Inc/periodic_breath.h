/**
  ******************************************************************************
  * @file    periodic_breath.h
  * @brief   Periodic breathing / Cheyne-Stokes detector (checklist #32).
  *
  *  Looks for the waxing and waning of ventilation that defines periodic
  *  breathing, not for individual events. Each therapy tick |patient flow|
  *  goes through two 2 s low-passes - a ventilation envelope that keeps a
  *  30-100 s cycle but not the breaths - sampled once a second into a 240 s
  *  window. Every 5 s the window is periodic when:
  *
  *    depth   std / mean of the envelope       >= g_pb_depth_min (0.30)
  *    cycle   normalised autocorrelation peak  >= g_pb_corr_min  (0.40)
  *            at a lag of 30..100 s, and <= -0.20 at half that lag (a
  *            real cycle, not a slow drift or a step)
  *
  *  Two periodic windows in a row start an episode, three non-periodic ones
  *  end it. Detection lags the pattern - about 200 s to see it (the window
  *  must be mostly periodic) but only about 60 s to lose it - so the span is
  *  moved back 180 s at the start (never before the previous span) and 60 s
  *  at the end. Spans of 60 s or more go to the CSL file as
  *  "CSR Start" / "CSR End", which is what OSCAR shows as CSR, and into the
  *  session total.
  *
  *  Scoring only - no pressure response (the AirSense does not react to CSR
  *  either). It cannot tell Cheyne-Stokes from cyclic obstructive events of
  *  the same period; the obstructive ones are scored as OA / H anyway.
  ******************************************************************************
  */
#ifndef PERIODIC_BREATH_H
#define PERIODIC_BREATH_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint8_t g_pb_enable;        /* 1 = detect (default)       */
extern volatile float   g_pb_depth_min;     /* envelope std / mean        */
extern volatile float   g_pb_corr_min;      /* autocorrelation peak       */

void     PeriodicBreath_Reset(void);         /* session start              */
/* Every scoring tick while breathing is being followed (not mask-off). */
void     PeriodicBreath_Update(float q_patient_slm, uint32_t dt_ms);
/* Mask-off or therapy stop: close an open span and drop the window. */
void     PeriodicBreath_Interrupt(void);

bool     PeriodicBreath_IsActive(void);
uint32_t PeriodicBreath_GetSessionSeconds(void);   /* closed + open span    */
uint32_t PeriodicBreath_GetEpisodes(void);
float    PeriodicBreath_GetLastCorr(void);         /* last evaluation       */
float    PeriodicBreath_GetLastDepth(void);
float    PeriodicBreath_GetLastPeriodS(void);

#ifdef __cplusplus
}
#endif

#endif /* PERIODIC_BREATH_H */
