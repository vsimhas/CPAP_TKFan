/**
  ******************************************************************************
  * @file    precheck.h
  * @brief   Pre-therapy self-test (checklist #83).
  *
  *  Run by Therapy_Start before the blower is enabled, from values the sensor
  *  loop already keeps - it never blocks. A HARD failure refuses the start:
  *  therapy latches THERAPY_FAULT_PRECHECK (logged to FAULTS.CSV, shown on
  *  the screen as "Self-test: <what>"). A WARNING lets therapy start and is
  *  reported over USB and in the debug log.
  *
  *    hard  mask pressure (PS)   reading frozen > 500 ms (no new conversion,
  *                               or stuck), NaN / outside -10..+50 cmH2O, or
  *                               more than 3 cmH2O off zero with the blower at
  *                               rest
  *          flow (FS)            frozen > 500 ms, NaN, or saturated
  *          ambient (AS)         frozen > 3 s, NaN / outside 500..1150 mbar
  *          motor drive (CM4)    status not advancing for 500 ms, a fault
  *                               active, or bus voltage outside 20..28 V
  *    warn  analog pressure (AMS5105) more than 2 cmH2O from PS at rest -
  *                               warn only: on the 2026-10-27 bench board it
  *                               read a fixed -26 cmH2O at rest and at
  *                               6 cmH2O (not fitted / not plumbed)
  *          SD card missing (no logging), clock not set, humidity / tube
  *          temperature sensor not reading
  *
  *  "Frozen" is time since the sensor's raw 24-bit word last changed: the
  *  AMS5935s are noisy in the last bits, so a live sensor changes on every
  *  conversion, and a dead SPI link, a sensor that stopped converting or a
  *  stuck one all look the same.
  *
  *  `therapy precheck` runs it and prints the result without starting;
  *  `therapy precheck test <what>` makes the next runs fail that check, to
  *  prove the refusal path (`therapy precheck test none` clears it).
  ******************************************************************************
  */
#ifndef PRECHECK_H
#define PRECHECK_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    PRECHECK_OK = 0,
    PRECHECK_PS_FROZEN,
    PRECHECK_PS_RANGE,
    PRECHECK_PS_OFFSET,
    PRECHECK_FS_FROZEN,
    PRECHECK_FS_BAD,
    PRECHECK_AS_FROZEN,
    PRECHECK_AS_RANGE,
    PRECHECK_CM4_STALE,
    PRECHECK_CM4_FAULT,
    PRECHECK_VBUS,
    PRECHECK_COUNT
} PrecheckFail_t;

/* Warning bits. */
#define PRECHECK_WARN_ANALOG   0x01U
#define PRECHECK_WARN_SD       0x02U
#define PRECHECK_WARN_CLOCK    0x04U
#define PRECHECK_WARN_HUMID    0x08U
#define PRECHECK_WARN_TUBE     0x10U

typedef struct
{
    PrecheckFail_t fail;
    uint8_t        warn;
    bool           injected;      /* the failure came from `precheck test` */
    float          ps_cmh2o;
    float          analog_cmh2o;
    float          flow_slm;
    float          ambient_mbar;
    uint32_t       ps_frozen_ms;
    uint32_t       fs_frozen_ms;
    uint32_t       as_frozen_ms;
    uint32_t       cm4_stale_ms;
    uint16_t       cm4_faults;
    float          vbus_v;
    int32_t        rpm;
} PrecheckResult_t;

extern volatile uint8_t g_precheck_enable;   /* 1 = run at Therapy_Start */

/* SensorTask, every loop after the sensors are harvested. */
void        Precheck_SensorTick(uint32_t dt_ms);
/* Evaluate now. Returns true if therapy may start. */
bool        Precheck_Run(PrecheckResult_t *out);
const char* Precheck_FailName(PrecheckFail_t f);   /* "mask pressure frozen" */
const char* Precheck_FailToken(PrecheckFail_t f);  /* "ps_frozen" (FAULTS.CSV) */
/* Name of the last failure, for the screen ("Self-test: ..."). */
const char* Precheck_GetLastFailText(void);
/* Bench: make the following runs fail check f (PRECHECK_OK clears). */
void        Precheck_Inject(PrecheckFail_t f);
PrecheckFail_t Precheck_ParseName(const char *s);  /* "ps_frozen" ...; COUNT if unknown */

#ifdef __cplusplus
}
#endif

#endif /* PRECHECK_H */
