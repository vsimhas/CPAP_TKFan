/**
  ******************************************************************************
  * @file    adherence.h
  * @brief   Adherence metrics (checklist #65) from the session records in
  *          0:/SUMMARY.CSV (session_summary.h).
  *
  *  A session counts toward the therapy night it STARTED in, split at noon
  *  like ResMed: a start before 12:00 belongs to the previous date. Usage is
  *  the record's mask-on time. Records without a valid RTC (rtc_valid 0) are
  *  counted as undated and left out - their date is unknown.
  *
  *  Rebuilt by reading SUMMARY.CSV once per card mount (columns found by
  *  header name, so newer schemas still parse); each record written after
  *  that is added in RAM. Times are the RTC's local time as set - no time
  *  zone or daylight-saving handling (#66).
  *
  *  Metrics: last ADH_WINDOW_DAYS ending today (days used, days >= 4 h,
  *  their share, average usage over all days / days used); since first use;
  *  and the usual reimbursement test - the best 30-night window within the
  *  first 90 nights, met at >= 21 of 30 nights (70 %) with >= 4 h.
  ******************************************************************************
  */

#ifndef ADHERENCE_H
#define ADHERENCE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define ADH_WINDOW_DAYS     30U
#define ADH_THRESHOLD_S     (4U * 3600U)    /* compliant night */
#define ADH_INITIAL_DAYS    90U             /* reimbursement period */
#define ADH_COMPLIANT_NIGHTS 21U            /* of 30 = 70 % */

/* FatFs I/O - SelfTestTask only, after SessionSummary_Service(). */
void     Adherence_Service(void);
/* A record just appended to SUMMARY.CSV (session_summary.c). */
void     Adherence_OnRecordWritten(const char *csv_line);

bool     Adherence_IsBuilt(void);
/* "# ADH ..." status line, CRLF-terminated. Returns its length, 0 if too long. */
int      Adherence_Format(char *buf, unsigned len);

#ifdef __cplusplus
}
#endif

#endif /* ADHERENCE_H */
