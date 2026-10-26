/**
  ******************************************************************************
  * @file    watchdog.h
  * @brief   CM7 hardware watchdog (IWDG1) with per-task check-in (checklist #80).
  *
  *  IWDG1 runs from the LSI with a ~1 s timeout. SafetyTask (realtime, 5 ms)
  *  is the only place it is refreshed, and it refreshes only while every
  *  monitored task has checked in within its own deadline - so a stuck task
  *  is not hidden by a healthy SafetyTask:
  *
  *    task       deadline   loop
  *    sensors    500 ms     5 ms   pressure PID, flow, therapy state machine
  *    system     1 s        10 ms  USB commands / log pump, blower bench cmds
  *    gui        2 s        ~16 ms TouchGFX frame
  *    climate    3 s        250 ms humidifier heater loop
  *    nfc        5 s        10 ms  RFAL worker
  *    selftest   60 s       10 ms  SD card / FatFs (HAL SD timeouts are 30 s)
  *
  *  A task not seen yet has a 20 s grace from start (one-time bring-up).
  *  A missed deadline latches: refresh stops and IWDG1 resets the system
  *  (both cores) within ~1 s. If the CPU itself is wedged - a fault handler
  *  loop, interrupts off, the scheduler dead - SafetyTask cannot run and
  *  IWDG1 fires on its own.
  *
  *  The reason is left in RTC backup registers BKP15R..BKP17R (the backup
  *  domain survives a system reset): which task stalled, or which fault
  *  handler and its PC. At the next boot it is read together with the reset
  *  flags (RCC_C1_RSR) and an abnormal reset goes into the fault log
  *  (FAULTS.CSV, source "system").
  *
  *  IWDG1 is frozen while the core is halted by the debugger.
  ******************************************************************************
  */

#ifndef WATCHDOG_H
#define WATCHDOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx.h"

typedef enum
{
    WDG_TASK_SAFETY = 0,
    WDG_TASK_SENSOR,
    WDG_TASK_SYSTEM,
    WDG_TASK_GUI,
    WDG_TASK_CLIMATE,
    WDG_TASK_NFC,
    WDG_TASK_SELFTEST,
    WDG_TASK_N
} WdgTask_t;

/* Why the last reset happened, as left in the backup registers. */
#define WDG_NOTE_MAGIC        0xA5000000UL
#define WDG_NOTE_STALL        0x00010000UL  /* low byte: task, BKP16R: age ms */
#define WDG_NOTE_FAULT        0x00020000UL  /* low byte: 'H' 'M' 'B' 'U', BKP16R: PC, BKP15R: CFSR */
#define WDG_NOTE_ERROR        0x00030000UL  /* BKP16R: caller of Error_Handler */

/* Fault handlers / Error_Handler: leave the reason for the reset that
 * follows. Plain register writes, safe in any context. */
static inline void Watchdog_Note(uint32_t kind, uint32_t detail, uint32_t a, uint32_t b)
{
    RTC->BKP15R = b;
    RTC->BKP16R = a;
    RTC->BKP17R = WDG_NOTE_MAGIC | kind | (detail & 0xFFFFUL);
}

/* main(), before RtcClock_Init(): latch and clear the reset flags and note. */
void        Watchdog_CaptureResetCause(void);
/* SafetyTask, once: log an abnormal last reset, then start IWDG1. */
void        Watchdog_Start(void);
/* SafetyTask, every tick: deadline check + refresh. */
void        Watchdog_Service(void);
/* Each monitored task, once per loop. */
void        Watchdog_Kick(WdgTask_t t);

/* Bench tests (SystemTask). A test starts ~300 ms after the request so the
 * ack drains over USB first. stall: SystemTask spins (task deadline path);
 * hang: interrupts off (hardware path); fault: undefined instruction. */
typedef enum
{
    WDG_TEST_NONE = 0,
    WDG_TEST_STALL,
    WDG_TEST_HANG,
    WDG_TEST_FAULT
} WdgTest_t;
void        Watchdog_RequestTest(WdgTest_t t);
void        Watchdog_TestPoll(void);

/* Status for `wdg`. */
bool        Watchdog_IsRunning(void);
uint32_t    Watchdog_GetTimeoutMs(void);
int         Watchdog_GetTripped(void);          /* task index, -1 if none */
uint32_t    Watchdog_GetResetFlags(void);       /* RCC_C1_RSR at boot */
const char* Watchdog_GetResetCauseName(void);
int         Watchdog_FormatResetNote(char *buf, unsigned len);
const char* Watchdog_GetTaskName(WdgTask_t t);
uint32_t    Watchdog_GetTaskDeadlineMs(WdgTask_t t);
uint32_t    Watchdog_GetTaskMaxGapMs(WdgTask_t t);
uint32_t    Watchdog_GetTaskAgeMs(WdgTask_t t);
void        Watchdog_ClearStats(void);

#ifdef __cplusplus
}
#endif

#endif /* WATCHDOG_H */
