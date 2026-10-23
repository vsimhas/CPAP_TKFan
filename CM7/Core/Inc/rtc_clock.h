/**
  ******************************************************************************
  * @file    rtc_clock.h
  * @brief   Real-time clock (STM32H7 RTC calendar) for session timestamps.
  *
  *  Checklist #66: there was no RTC - every session went into
  *  DATALOG/20260101 unless the date was sent over USB, and the date never
  *  advanced. This runs the RTC calendar from the LSE crystal (32.768 kHz)
  *  when one is fitted, else from the LSI (~32 kHz, about +-5 %, so it is
  *  only good for ordering sessions until the time is set again).
  *
  *  Bring-up never blocks boot: RtcClock_Init() only starts the oscillator;
  *  RtcClock_Service() (SelfTestTask, every 10 ms) waits up to 4 s for the
  *  LSE, falls back to the LSI, and then starts the calendar.
  *
  *  The calendar lives in the backup domain, so it survives a reset and -
  *  with a battery on VBAT - a power cycle. "Valid" means the time has been
  *  set (RtcClock_Set / `rtc set` / `therapy edf datetime`) since the backup
  *  domain was last lost; until then the calendar counts from 2026-01-01.
  ******************************************************************************
  */

#ifndef RTC_CLOCK_H
#define RTC_CLOCK_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

typedef struct
{
    uint8_t yy;   /* 0..99 = 2000..2099 */
    uint8_t mo;   /* 1..12 */
    uint8_t dd;   /* 1..31 */
    uint8_t hh;   /* 0..23 */
    uint8_t mi;   /* 0..59 */
    uint8_t ss;   /* 0..59 */
} RtcDateTime_t;

typedef enum
{
    RTC_SRC_NONE = 0,
    RTC_SRC_LSE,
    RTC_SRC_LSI,
    RTC_SRC_OTHER
} RtcSource_t;

void        RtcClock_Init(void);
void        RtcClock_Service(void);

bool        RtcClock_IsRunning(void);
bool        RtcClock_IsTimeValid(void);
RtcSource_t RtcClock_GetSource(void);
const char* RtcClock_GetSourceName(void);

/* False while the calendar is not running yet. */
bool        RtcClock_Get(RtcDateTime_t *dt);
/* Sets the calendar and marks the time valid. Before the calendar runs the
 * value is kept and applied as soon as it starts. False on a bad value. */
bool        RtcClock_Set(const RtcDateTime_t *dt);

#ifdef __cplusplus
}
#endif

#endif /* RTC_CLOCK_H */
