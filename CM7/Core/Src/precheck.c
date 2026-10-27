/**
  ******************************************************************************
  * @file    precheck.c
  * @brief   Pre-therapy self-test (see precheck.h).
  ******************************************************************************
  */

#include "precheck.h"
#include "pressure_sensors.h"
#include "sensirion_th.h"
#include "flow_sensor.h"
#include "blower_ipc.h"
#include "rtc_clock.h"

#include <math.h>
#include <string.h>

volatile uint8_t g_precheck_enable = 1U;

#define PC_MBAR_PER_CMH2O     0.980665f
#define PC_PS_FROZEN_MS       500U
#define PC_FS_FROZEN_MS       500U
#define PC_AS_FROZEN_MS       3000U     /* AS is sampled at 10 Hz */
#define PC_CM4_STALE_MS       500U
#define PC_PS_MIN_CMH2O       (-10.0f)  /* as the safety supervisor */
#define PC_PS_MAX_CMH2O       50.0f
#define PC_PS_REST_MAX_CMH2O  3.0f      /* blower stopped: about zero */
#define PC_ANALOG_DIFF_CMH2O  2.0f
#define PC_AS_MIN_MBAR        500.0f
#define PC_AS_MAX_MBAR        1150.0f
#define PC_VBUS_MIN_V         20.0f
#define PC_VBUS_MAX_V         28.0f
#define PC_REST_RPM           500

/* main.c */
extern AMS5935_HandleTypeDef hamsFS;
extern AMS5935_Data_t        measFS;
extern AMS5935_Data_t        measAS;
extern AMS5935_Data_t        measPS;
extern BlowerIpcStatus_t     g_blower_status;
extern SensirionTH_Handle_t  hSht4x;
extern SensirionTH_Handle_t  hSts4x;
extern volatile uint8_t      sd_card_present;

typedef struct
{
    uint32_t last;
    uint32_t frozen_ms;
    bool     seen;
} Watch_t;

static Watch_t s_ps;
static Watch_t s_fs;
static Watch_t s_as;
static Watch_t s_cm4;
static PrecheckFail_t s_inject = PRECHECK_OK;
static PrecheckFail_t s_last_fail = PRECHECK_OK;

static const char * const k_name[PRECHECK_COUNT] = {
    "OK",
    "mask pressure frozen",
    "mask pressure out of range",
    "mask pressure not zero at rest",
    "flow sensor frozen",
    "flow sensor bad reading",
    "ambient pressure frozen",
    "ambient pressure out of range",
    "motor drive not responding",
    "motor drive fault",
    "motor supply voltage",
};

static const char * const k_token[PRECHECK_COUNT] = {
    "none", "ps_frozen", "ps_range", "ps_offset", "fs_frozen", "fs_bad",
    "as_frozen", "as_range", "cm4_stale", "cm4_fault", "vbus",
};

static void watch(Watch_t *w, uint32_t value, uint32_t dt_ms)
{
    if (!w->seen || (value != w->last))
    {
        w->last = value;
        w->frozen_ms = 0U;
        w->seen = true;
    }
    else if (w->frozen_ms < 0x7FFFFFFFU)
    {
        w->frozen_ms += dt_ms;
    }
}

void Precheck_SensorTick(uint32_t dt_ms)
{
    watch(&s_ps, measPS.pressure_raw, dt_ms);
    watch(&s_fs, measFS.pressure_raw, dt_ms);
    watch(&s_as, measAS.pressure_raw, dt_ms);
    watch(&s_cm4, g_blower_status.status_seq, dt_ms);
}

static bool isFiniteF(float v)
{
    return (v == v) && (fabsf(v) < 1.0e9f);
}

bool Precheck_Run(PrecheckResult_t *out)
{
    PrecheckResult_t r;
    memset(&r, 0, sizeof(r));

    r.ps_cmh2o     = measPS.pressure / PC_MBAR_PER_CMH2O;
    r.analog_cmh2o = g_pressure_analog_mbar / PC_MBAR_PER_CMH2O;
    r.flow_slm     = Flow_GetSlm();
    r.ambient_mbar = measAS.pressure;
    r.ps_frozen_ms = s_ps.frozen_ms;
    r.fs_frozen_ms = s_fs.frozen_ms;
    r.as_frozen_ms = s_as.frozen_ms;
    r.cm4_stale_ms = s_cm4.frozen_ms;
    r.cm4_faults   = g_blower_status.current_faults;
    r.vbus_v       = (float)g_blower_status.bus_voltage_mv * 0.001f;
    r.rpm          = g_blower_status.mech_speed_rpm;
    const bool at_rest = (r.rpm > -PC_REST_RPM) && (r.rpm < PC_REST_RPM);

    /* Hard checks, in order of what the pressure loop depends on most. */
    if (!s_ps.seen || (r.ps_frozen_ms > PC_PS_FROZEN_MS))
    {
        r.fail = PRECHECK_PS_FROZEN;
    }
    else if (!isFiniteF(r.ps_cmh2o) || (r.ps_cmh2o < PC_PS_MIN_CMH2O)
             || (r.ps_cmh2o > PC_PS_MAX_CMH2O))
    {
        r.fail = PRECHECK_PS_RANGE;
    }
    else if (at_rest && (fabsf(r.ps_cmh2o) > PC_PS_REST_MAX_CMH2O))
    {
        r.fail = PRECHECK_PS_OFFSET;
    }
    else if (!s_fs.seen || (r.fs_frozen_ms > PC_FS_FROZEN_MS))
    {
        r.fail = PRECHECK_FS_FROZEN;
    }
    else if (!isFiniteF(r.flow_slm) || !isFiniteF(measFS.pressure)
             || (AMS5935_WasSampleSaturated(&hamsFS) != 0U))
    {
        r.fail = PRECHECK_FS_BAD;
    }
    else if (!s_as.seen || (r.as_frozen_ms > PC_AS_FROZEN_MS))
    {
        r.fail = PRECHECK_AS_FROZEN;
    }
    else if (!isFiniteF(r.ambient_mbar) || (r.ambient_mbar < PC_AS_MIN_MBAR)
             || (r.ambient_mbar > PC_AS_MAX_MBAR))
    {
        r.fail = PRECHECK_AS_RANGE;
    }
    else if (!s_cm4.seen || (r.cm4_stale_ms > PC_CM4_STALE_MS))
    {
        r.fail = PRECHECK_CM4_STALE;
    }
    else if (r.cm4_faults != 0U)
    {
        r.fail = PRECHECK_CM4_FAULT;
    }
    else if ((r.vbus_v < PC_VBUS_MIN_V) || (r.vbus_v > PC_VBUS_MAX_V))
    {
        r.fail = PRECHECK_VBUS;
    }

    if ((r.fail == PRECHECK_OK) && (s_inject != PRECHECK_OK))
    {
        r.fail = s_inject;
        r.injected = true;
    }

    /* Warnings: therapy still starts. */
    if (at_rest && isFiniteF(r.analog_cmh2o)
        && (fabsf(r.analog_cmh2o - r.ps_cmh2o) > PC_ANALOG_DIFF_CMH2O))
    {
        r.warn |= PRECHECK_WARN_ANALOG;
    }
    if (sd_card_present == 0U)        { r.warn |= PRECHECK_WARN_SD; }
    if (!RtcClock_IsTimeValid())      { r.warn |= PRECHECK_WARN_CLOCK; }
    if (hSht4x.last_crc_ok == 0U)     { r.warn |= PRECHECK_WARN_HUMID; }
    if (hSts4x.last_crc_ok == 0U)     { r.warn |= PRECHECK_WARN_TUBE; }

    s_last_fail = r.fail;
    if (out != NULL)
    {
        *out = r;
    }
    return (r.fail == PRECHECK_OK);
}

const char* Precheck_FailName(PrecheckFail_t f)
{
    return ((uint32_t)f < (uint32_t)PRECHECK_COUNT) ? k_name[f] : "?";
}

const char* Precheck_FailToken(PrecheckFail_t f)
{
    return ((uint32_t)f < (uint32_t)PRECHECK_COUNT) ? k_token[f] : "?";
}

const char* Precheck_GetLastFailText(void)
{
    return Precheck_FailName(s_last_fail);
}

void Precheck_Inject(PrecheckFail_t f)
{
    s_inject = ((uint32_t)f < (uint32_t)PRECHECK_COUNT) ? f : PRECHECK_OK;
}

PrecheckFail_t Precheck_ParseName(const char *s)
{
    for (uint32_t i = 0U; i < (uint32_t)PRECHECK_COUNT; i++)
    {
        if (strcmp(s, k_token[i]) == 0)
        {
            return (PrecheckFail_t)i;
        }
    }
    return PRECHECK_COUNT;
}
