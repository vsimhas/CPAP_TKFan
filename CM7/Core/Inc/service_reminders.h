/**
  ******************************************************************************
  * @file    service_reminders.h
  * @brief   Supplies replacement reminders (checklist #87).
  *
  *  Five consumables, each due on usage hours OR calendar days, whichever
  *  comes first. ResMed-style intervals; the hour limit is the same period at
  *  8 h a night, so a heavy user is reminded sooner and a light one by the
  *  calendar:
  *
  *      filter          30 days    240 h
  *      mask cushion    30 days    240 h
  *      mask            90 days    720 h
  *      tube            90 days    720 h
  *      water chamber  180 days   1440 h
  *
  *  An item is SOON at 90 % of either limit and DUE at 100 %.
  *
  *  Usage hours are the blower run time of every session on the card (the
  *  run_s column of SUMMARY.CSV, read by adherence.c, dated or not) plus the
  *  session running now - so they count even when the clock was not set.
  *  Days need the RTC, both when an item was replaced and now.
  *
  *  Filter blockage, measured (the checklist's "drive from measured blockage
  *  where possible"): every therapy start runs the #83 spin check at
  *  12000 rpm, and its device pressure minus the fan map at that speed and
  *  flow is logged. A blocked inlet filter starves the blower, so that
  *  residual falls. The reference is the median of the first 3 starts after
  *  the filter was replaced; when the median of the last 3 is 0.5 cmH2O or
  *  more below it, the filter is flagged DUE ("blocked") whatever its age.
  *  Being relative to the same device and filter, it does not need an exact
  *  fan map.
  *
  *  Everything lives in 0:/SERVICE.CSV: one row per replacement, a "start"
  *  row the first time an item is seen on a card (its clock starts then), and
  *  a "spin" row per therapy start. Reloaded when a card is mounted, after
  *  SUMMARY.CSV has been read. No card: no reminders (nothing to count from).
  *
  *  GUI: MORE > Supplies lists the items with their state; click one, rotate
  *  to Yes and click to log it replaced. The welcome screen names what is
  *  due. USB: `service status`, `service replaced <item|all>`; bench:
  *  `service test hours|days|spin <value>` adds an offset to the usage
  *  hours, the calendar, or the spin residual (`service test none`).
  ******************************************************************************
  */
#ifndef SERVICE_REMINDERS_H
#define SERVICE_REMINDERS_H

#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum
{
    SVC_FILTER = 0,
    SVC_CUSHION,
    SVC_MASK,
    SVC_TUBE,
    SVC_CHAMBER,
    SVC_COUNT
} SvcItem_t;

typedef enum
{
    SVC_UNKNOWN = 0,      /* no card / not loaded yet */
    SVC_OK,
    SVC_SOON,
    SVC_DUE
} SvcState_t;

typedef struct
{
    SvcState_t state;
    float      used_h;        /* since replaced */
    float      limit_h;
    int32_t    days;          /* since replaced; -1 if the date is unknown */
    int32_t    limit_d;
    bool       blocked;       /* filter only: measured blockage */
} SvcStatus_t;

/* SensorTask (therapy tick is fine), every loop. */
void        Service_Update(uint32_t dt_ms);
/* SD task: SERVICE.CSV reload and append. */
void        Service_Service(void);

bool        Service_IsReady(void);
void        Service_GetStatus(SvcItem_t item, SvcStatus_t *out);
bool        Service_AnyDue(void);
/* "filter, tube" - the due items, for the welcome screen; "" if none. */
const char* Service_DueText(void);
void        Service_Replaced(SvcItem_t item);
const char* Service_ItemName(SvcItem_t item);     /* "Filter" (screen) */
const char* Service_ItemToken(SvcItem_t item);    /* "filter" (USB, CSV) */
SvcItem_t   Service_ParseItem(const char *s);     /* SVC_COUNT if unknown */
void        Service_SetTest(float hours, int32_t days, float spin_cmh2o);
/* One bench offset at a time; the others keep their value. */
void        Service_SetTestHours(float hours);
void        Service_SetTestDays(int32_t days);
void        Service_SetTestSpin(float spin_cmh2o);
/* `service status`: line k (0..SVC_COUNT+1), "# SVC ..."; 0 past the end. */
int         Service_FormatStatus(uint32_t k, char *buf, unsigned len);

#ifdef __cplusplus
}
#endif

#endif /* SERVICE_REMINDERS_H */
