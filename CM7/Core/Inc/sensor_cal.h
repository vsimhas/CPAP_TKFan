/**
  ******************************************************************************
  * @file    sensor_cal.h
  * @brief   Sensor calibration: in-service zero, field span, traceability
  *          (checklist #84).
  *
  *  Factory calibration is what the parts carry: the AMS5935s are factory
  *  calibrated to their datasheet transfer function, and the flow element
  *  has its bench LUT and zero (flow_sensor.c). This module keeps what drifts
  *  in service on top of that, and records every change.
  *
  *  Device pressure (PS), the controlled variable:
  *      corrected = (raw - offset) * gain
  *    offset  learned automatically. While therapy is idle and the blower has
  *            been stopped for 3 s with |flow| < 3 L/min, the raw reading is
  *            averaged over 1 s windows; three quiet windows in a row (peak-
  *            peak <= 0.2 cmH2O each) give a candidate. Within 1.5 cmH2O of
  *            the factory zero it is applied; past that it is NOT applied,
  *            and the pre-therapy self-test refuses to start (ps_offset) -
  *            a sensor that far off is drifting or damaged, not to be
  *            silently corrected. Learned once per idle period, again every
  *            10 min of idle.
  *    gain    field span, `cal ps gain <g>` against a reference gauge,
  *            0.95..1.05. Left at 1.0 until a reference is available.
  *
  *  Flow (FS): the existing idle auto-zero (flow_sensor.c) now refuses a zero
  *  more than 0.05 mbar (~5 L/min near zero) from the factory zero; the
  *  self-test warns (therapy still starts - a flow zero error biases leak and
  *  event figures, it does not change the delivered pressure).
  *
  *  Ambient (AS): `cal as offset <mbar>` against a reference barometer,
  *  +-10 mbar.
  *
  *  Every applied or refused change is a row in 0:/CAL.CSV (time, sensor,
  *  parameter, old, new, source auto / field / reset / factory, result). The
  *  first rows of a new file are the factory values. At boot, and when a card
  *  is inserted while therapy is idle, the last applied value of each
  *  parameter is loaded back (a zero already learned since boot is kept).
  *
  *  Bench: `cal test ps <cmH2O>` / `cal test fs <mbar>` add an offset to the
  *  raw reading, to prove the learn / refuse paths (`cal test none`).
  *
  *  Not covered: calibration data carried by a field-replaceable part (needs
  *  an ID / EEPROM on the sensor board), and the store being the SD card
  *  until there is non-volatile memory on the board (#89).
  ******************************************************************************
  */
#ifndef SENSOR_CAL_H
#define SENSOR_CAL_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

extern volatile uint8_t g_cal_ps_auto;          /* 1 = learn the PS zero */
extern volatile uint8_t g_cal_restore;          /* 1 = reload from CAL.CSV */

/* SensorTask, on every successful PS / AS read: raw mbar in, corrected out. */
float Cal_ApplyPs(float raw_mbar);
float Cal_ApplyAs(float raw_mbar);
/* Bench offset added to the raw flow dP (0 unless `cal test fs`). */
float Cal_FsInjectMbar(void);
/* SensorTask, every loop. idle = therapy idle and blower stopped. */
void  Cal_Update(uint32_t dt_ms, bool idle, float q_slm);
/* SD task: CAL.CSV append and reload. */
void  Cal_Service(void);

bool  Cal_PsZeroRejected(void);
bool  Cal_FsZeroRejected(void);

/* Field calibration; false if out of limits (nothing changes). */
bool  Cal_SetPsGain(float gain);
bool  Cal_SetAsOffset(float mbar);
void  Cal_ArmPsZero(void);                      /* learn again now */
void  Cal_ArmFsZero(void);
void  Cal_ResetToFactory(void);
void  Cal_SetTest(float ps_cmh2o, float fs_mbar);

/* `cal status` line. */
int   Cal_FormatStatus(char *buf, unsigned len);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_CAL_H */
