/**
  ******************************************************************************
  * @file    mask_events.h
  * @brief   Timestamped mask-off / mask-on episodes (checklist #64).
  *
  *  SUMMARY.CSV has the session's mask-off total and episode count; this
  *  keeps each episode, so a night can be told apart from hours of a
  *  pressurised empty mask, and when the mask came off is known.
  *
  *  Every episode, when it closes (mask back on, or therapy stopped with the
  *  mask still off), is:
  *    - a row in 0:/MASK.CSV: session start (as in SUMMARY.CSV), off and on
  *      times (RTC and seconds into the session), duration, the detector
  *      path that entered it, and how it ended;
  *    - a "Mask Off" annotation in the session's EVE file, stamped at its
  *      end with its duration like the respiratory events (OSCAR ignores
  *      annotation names it does not know, so the AHI view is unchanged).
  *  A mask-on is held 20 s before the episode is written: a reconnect that
  *  fails therapy.c's probation re-parks (path 0), and that continues the
  *  same episode instead of starting a new one. Episodes are queued in RAM
  *  from the therapy tick and written by MaskEvents_Service() in the SD
  *  task.
  ******************************************************************************
  */
#ifndef MASK_EVENTS_H
#define MASK_EVENTS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/* therapy.c */
void     MaskEvents_SessionStart(void);
void     MaskEvents_Off(uint8_t path);           /* enterMaskOff path code */
void     MaskEvents_On(void);                    /* exitMaskOff */
void     MaskEvents_Update(uint32_t dt_ms);      /* therapy tick */
/* Therapy_Stop, before SessionEdf_OnTherapyStop closes the EVE file. */
void     MaskEvents_SessionEnd(void);

/* SD task. */
void     MaskEvents_Service(void);

uint32_t MaskEvents_GetSessionCount(void);       /* episodes this session */
uint32_t MaskEvents_GetRows(void);               /* rows written this boot */
uint32_t MaskEvents_GetError(void);
/* "# MASK n=.. last_off=.. last_on=.. dur=.. path=.. end=.." */
int      MaskEvents_FormatStatus(char *buf, unsigned len);

#ifdef __cplusplus
}
#endif

#endif /* MASK_EVENTS_H */
