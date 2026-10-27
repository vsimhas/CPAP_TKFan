/**
  ******************************************************************************
  * @file    session_summary.h
  * @brief   Per-session summary record (checklist #57) -> 0:/SUMMARY.CSV.
  *
  *  One line per therapy session, appended when therapy stops: start / end
  *  time (RTC), run and mask-on time, mode and settings, pressure avg / min /
  *  max / P50 / P90 / P95, leak avg / P50 / P95 / max / time above the high
  *  threshold, scored events and indices, and the EDF file name of the
  *  session. Column 1 is the schema version: add columns only at the end and
  *  bump it, so older records stay readable.
  *
  *  The snapshot is taken in RAM at Therapy_Stop; the SD write happens in
  *  SessionSummary_Service() (SelfTestTask) once the card is mounted - it is
  *  mounted whenever one is in, EDF logging on or not. `therapy summary`
  *  prints the last record over USB whether or not it reached the card.
  ******************************************************************************
  */

#ifndef SESSION_SUMMARY_H
#define SESSION_SUMMARY_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define SESSION_SUMMARY_SCHEMA  2U

void        SessionSummary_OnTherapyStart(void);
/* Call at Therapy_Stop after the session statistics have been refreshed. */
void        SessionSummary_OnTherapyStop(void);
/* FatFs I/O - SelfTestTask only (after SessionEdf_Service). */
void        SessionSummary_Service(void);

bool        SessionSummary_HasLast(void);
const char* SessionSummary_GetLastLine(void);   /* CSV, no line ending */
const char* SessionSummary_GetHeaderLine(void); /* CSV column names */
bool        SessionSummary_IsPending(void);     /* not on the card yet */
uint32_t    SessionSummary_GetWritten(void);    /* records appended since boot */
uint32_t    SessionSummary_GetError(void);      /* 0x6Sxx: S stage, xx FRESULT */

#ifdef __cplusplus
}
#endif

#endif /* SESSION_SUMMARY_H */
