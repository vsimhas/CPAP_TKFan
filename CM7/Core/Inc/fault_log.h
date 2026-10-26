/**
  ******************************************************************************
  * @file    fault_log.h
  * @brief   Durable fault log (checklist #86) -> 0:/FAULTS.CSV.
  *
  *  One record per fault, with the state around it, not just the code:
  *  time (RTC) and uptime, source and code, therapy state / mode / run time,
  *  target / setpoint / measured pressure, patient flow, leak, commanded and
  *  measured rpm, Iq / Id, bus voltage and the CM4 state and fault bits.
  *
  *  Sources:
  *   - therapy: every Therapy_LatchFault() (safety supervisor), captured at
  *     the moment of the latch, before the blower is stopped;
  *   - drive:   new bits in the CM4 occurred-fault word (MCSDK MC_* codes,
  *     e.g. DP_FAULT), polled from FaultLog_Service() every 10 ms;
  *   - system:  an abnormal last reset (watchdog, see watchdog.h), recorded
  *     at boot with the reason left before the reset.
  *
  *  Records go into a RAM ring (last FAULT_LOG_RING_N, `faults` over USB)
  *  and are appended to FAULTS.CSV whenever the card is mounted - pending
  *  ones wait for the next mount. Column 1 is the schema version.
  ******************************************************************************
  */

#ifndef FAULT_LOG_H
#define FAULT_LOG_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

#define FAULT_LOG_SCHEMA  1U
#define FAULT_LOG_RING_N  16U

typedef enum
{
    FAULT_SRC_THERAPY = 0,
    FAULT_SRC_DRIVE,
    FAULT_SRC_SYSTEM
} FaultSource_t;

/* Therapy_LatchFault(): snapshot now (RAM only, any task). */
void     FaultLog_RecordTherapy(uint16_t code, const char *name, const char *why);
/* Abnormal last reset (watchdog.c): code = RCC_C1_RSR >> 16. */
void     FaultLog_RecordSystem(uint16_t code, const char *name, const char *why);
/* SelfTestTask: drive-fault polling + FAULTS.CSV append. */
void     FaultLog_Service(void);

uint32_t FaultLog_GetCount(void);       /* records since boot */
uint32_t FaultLog_GetPending(void);     /* not on the card yet */
uint32_t FaultLog_GetError(void);       /* 0x8Sxx: S stage, xx FRESULT */
/* k-th most recent record (0 = newest) as "# FLT ..." CRLF line; 0 if none. */
int      FaultLog_FormatRecent(uint32_t k, char *buf, unsigned len);

#ifdef __cplusplus
}
#endif

#endif /* FAULT_LOG_H */
