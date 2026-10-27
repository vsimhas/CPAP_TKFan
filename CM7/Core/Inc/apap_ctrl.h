/**
  ******************************************************************************
  * @file    apap_ctrl.h
  * @brief   AirSense-style Autoset (APAP) pressure titration.
  *
  *  Algorithm (functional-equivalent prototype — not ResMed IP)
  *  ----------------------------------------------------------
  *    1. Start at min pressure; hold while breaths are detected.
  *    2. MONITOR: flow limitation (peak drop and/or insp flattening),
 *       hypopnea, snore (severity-weighted) → mild/moderate raises.
 *    3. No qualified breath for the breath window (8 s) + 2 s → FOT
 *       (~4 Hz on raw Q); the episode is timed from the last breath and
 *       scored OA/CA from ~10 s. FOT ends on a *recovery-sized* breath
 *       (30 % of the pre-episode peak baseline, floor 4 slm). Small obstructed
 *       gasps do not abort FOT. OSA if dP_pp OK and dQ_pp low → raise
 *       in duration-based steps (not a continuous ramp). Central if
 *       airway transmits flow → do not raise.
  *    4. Excessive leak (HIGH) gates all pressure raises.
  *    5. Quiet stable period → slow down-titration toward min.
  *    6. Session event counts + AHI-style indices for debug/telemetry.
  *
  *  Thresholds are debugger-tunable heuristics pending clinical validation.
  *  Call Apap_Update() from Therapy_Update() after Epr_Update().
  ******************************************************************************
  */

#ifndef APAP_CTRL_H
#define APAP_CTRL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef enum
{
    APAP_STATE_IDLE = 0,       /* therapy not running                    */
    APAP_STATE_MONITOR,        /* breathing: hold / FL / hyp / snore     */
    APAP_STATE_NO_BREATH,      /* counting time since last breath        */
    APAP_STATE_OSCILLATE,      /* continuous FOT until breath resumes    */
    APAP_STATE_RESPOND         /* legacy alias: same as OSCILLATE+raise  */
} ApapState_t;

void       Apap_Init(void);

void       Apap_OnTherapyStart(float min_cmh2o, float max_cmh2o,
                               uint8_t therapy_mode);
void       Apap_OnTherapyStop(void);
void       Apap_OnMaskReconnect(void);

/* AirSense AutoSet Response: false=Standard, true=Soft (gentler rises). */
void       Apap_SetResponseSoft(bool soft);
bool       Apap_IsResponseSoft(void);

float      Apap_Update(float dt_s, uint8_t therapy_state, bool mask_off);

/* Flow for the EPR breath FSM: the oscillator frequency is notched out
 * while the oscillator runs (FOT ripple must not chop breaths - R7b).
 * Call every tick with the patient flow; returns it unchanged otherwise. */
float      Apap_FilterBreathFlow(float q_slm, float dt_s);
/* R11: "apap diag" text (two "# ..." lines); returns length, 0 on overflow. */
int        Apap_FormatDiag(char *buf, unsigned len);

float      Apap_GetTargetPressure(void);
ApapState_t Apap_GetState(void);
const char* Apap_GetStateName(void);
bool       Apap_IsOscillating(void);

/* FOT window metrics (updated during APAP_STATE_OSCILLATE). */
float      Apap_GetFotPressurePpCmh2o(void);
float      Apap_GetFotFlowPpSlm(void);
float      Apap_GetFotImpedanceCmh2oPerSlm(void);  /* dP_pp/dQ_pp; high => blocked */
float      Apap_GetOsaFlowPpThreshSlm(void);
bool       Apap_IsOsaLikely(void);
bool       Apap_IsOsaTreating(void);

/* MONITOR detectors. */
bool       Apap_IsFlowLimited(void);
bool       Apap_IsHypopneaActive(void);
bool       Apap_IsSnoreActive(void);
bool       Apap_IsRaiseBlockedByLeak(void);
/* #33: detectors running in CPAP mode (scoring, no titration). */
bool       Apap_IsScoreOnly(void);
/* The last / current session scored events (AutoSet, or CPAP scoring on). */
bool       Apap_WasSessionScored(void);
float      Apap_GetFlPeakBaselineSlm(void);
float      Apap_GetFlLastPeakSlm(void);
float      Apap_GetFlLastFlat(void);   /* insp mean/peak, 0–1 */
float      Apap_GetFlLastDip(void);    /* R13: mid-insp notch, 1 = none */
float      Apap_GetSnoreRmsSlm(void);
uint32_t   Apap_GetStableMs(void);
bool       Apap_IsLowering(void);

/* Session event scoring (reset on therapy start). */
uint32_t   Apap_GetEventOaCount(void);
uint32_t   Apap_GetEventCaCount(void);
uint32_t   Apap_GetEventHypCount(void);
uint32_t   Apap_GetEventFlCount(void);
uint32_t   Apap_GetEventSnoreCount(void);
uint32_t   Apap_GetTherapyMs(void);
float      Apap_GetSessionAhi(void);   /* (OA + CA + hyp) / hour of therapy */
float      Apap_GetSessionRdi(void);   /* (OA + CA + hyp + FL) / hour       */
float      Apap_GetSessionSnoreIndex(void); /* snore events / hour     */

/* #60 / #61: continuous indices for the PLD file (Snore.2s, FlowLim.2s). */
float      Apap_GetFlowLimIndex(void);  /* last breath, 0..1; 0 after 10 s without one */
float      Apap_GetSnoreLevel(void);    /* snore RMS / threshold while snoring, else 0; 0..5 */
float      Apap_GetSessionFlIndexAvg(void);  /* mean over the session's breaths */
float      Apap_GetSessionFlIndexP95(void);
uint32_t   Apap_GetSessionFlSeconds(void);    /* time flow-limited (FL flag) */
uint32_t   Apap_GetSessionSnoreSeconds(void); /* time snoring */

/* #19 leak-invalidated scoring: while unintentional leak is HIGH (after a
 * short debounce, plus a settling time after it drops) no event is scored,
 * FL / snore time and the PLD indices stop, and the time is left out of the
 * AHI / RDI / SI denominator. Reported in SUMMARY.CSV (leak_excl_s / _n). */
bool       Apap_IsLeakExcluded(void);
uint32_t   Apap_GetSessionLeakExclSeconds(void);
uint32_t   Apap_GetSessionLeakExclCount(void);
uint32_t   Apap_GetLeakExclDroppedEvents(void);   /* events not scored */

/* #38 AutoRamp: sleep onset as the AirSense defines it. Latched from
 * Apap_OnTherapyStart until the next start; the detectors also run (score
 * only) while therapy is in RAMP. */
typedef enum
{
    APAP_ONSET_NONE = 0,
    APAP_ONSET_BREATHING,         /* g_apap_onset_breaths stable breaths */
    APAP_ONSET_EVENTS,            /* obstructive apneas / any hypopneas  */
    APAP_ONSET_SNORE              /* snore breaths in a row              */
} ApapOnset_t;
ApapOnset_t Apap_GetSleepOnset(void);
uint32_t   Apap_GetOnsetStableBreaths(void);  /* current run */

#ifdef __cplusplus
}
#endif

#endif /* APAP_CTRL_H */
