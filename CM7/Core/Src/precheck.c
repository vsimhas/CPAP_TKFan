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
#include "sensor_cal.h"

#include <math.h>
#include <string.h>

volatile uint8_t g_precheck_enable = 1U;
volatile uint8_t g_precheck_spin_enable = 1U;
/* TKFan: off - g_flow_fan_a..d are still the WM6850's fit (flow_sensor.c),
 * so the map would refuse a healthy BA7060H24B. Refit the map for this
 * blower, then set it back to 1 (`therapy precheck map 1` to try it). */
volatile uint8_t g_precheck_spin_map_enable = 0U;

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
#define PC_SPIN_RPM           12000     /* ~3 cmH2O on the WM6850 map */
#define PC_SPIN_RAMP_MS       1000U
#define PC_SPIN_SAMPLE_MS     500U      /* at speed this long, then sample */
#define PC_SPIN_END_MS        1500U     /* ... until this long at speed    */
#define PC_SPIN_TIMEOUT_MS    6000U     /* never got there: spin_speed     */
#define PC_SPIN_SPEED_TOL     1500.0f
/* Map rms 0.38 cmH2O. Bench 2026-10-27 at 12 krpm: healthy +0.51 / +0.26
 * cmH2O off the map; 1.0 + 25 % let a 57 % fan_a error (1.78 off) pass,
 * 0.8 + 15 % (1.16 here) catches it and a ~30 % pressure gain error. */
#define PC_SPIN_MAP_ABS       0.8f
#define PC_SPIN_MAP_REL       0.15f
#define PC_SPIN_P_STD_MIN     0.002f    /* cmH2O: a live sensor never sits flat */
#define PC_SPIN_Q_STD_MIN     0.02f     /* L/min */
#define PC_SPIN_P_STD_MAX     3.0f
#define PC_SPIN_Q_STD_MAX     60.0f

/* main.c */
extern AMS5935_HandleTypeDef hamsFS;
extern AMS5935_Data_t        measFS;
extern AMS5935_Data_t        measAS;
extern AMS5935_Data_t        measPS;
extern BlowerIpcStatus_t     g_blower_status;
extern SensirionTH_Handle_t  hSht4x;
extern SensirionTH_Handle_t  hSts4x;
extern volatile uint8_t      sd_card_present;
/* flow_sensor.c fan map (cmH2O; krpm; L/min). */
extern volatile float        g_flow_fan_a;
extern volatile float        g_flow_fan_b;
extern volatile float        g_flow_fan_c;
extern volatile float        g_flow_fan_d;

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
/* Spin accumulators. */
static uint32_t s_sp_n;
static double   s_sp_rpm;
static double   s_sp_p;
static double   s_sp_p2;
static double   s_sp_q;
static double   s_sp_q2;
static PrecheckSpin_t s_last_spin;

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
    "blower did not reach speed",
    "pressure / flow off the blower curve",
    "sensor flat while the blower runs",
    "sensor noise while the blower runs",
};

static const char * const k_token[PRECHECK_COUNT] = {
    "none", "ps_frozen", "ps_range", "ps_offset", "fs_frozen", "fs_bad",
    "as_frozen", "as_range", "cm4_stale", "cm4_fault", "vbus",
    "spin_speed", "spin_map", "spin_flat", "spin_noisy",
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
    else if (at_rest && ((fabsf(r.ps_cmh2o) > PC_PS_REST_MAX_CMH2O)
                         || Cal_PsZeroRejected()))
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

    if ((r.fail == PRECHECK_OK) && (s_inject != PRECHECK_OK)
        && (s_inject < PRECHECK_SPIN_SPEED))      /* spin ones: at the verdict */
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
    if (Cal_FsZeroRejected())         { r.warn |= PRECHECK_WARN_FLOW_ZERO; }

    s_last_fail = r.fail;
    if (out != NULL)
    {
        *out = r;
    }
    return (r.fail == PRECHECK_OK);
}

int32_t  Precheck_SpinRpm(void)          { return PC_SPIN_RPM; }
uint16_t Precheck_SpinRampMs(void)       { return (uint16_t)PC_SPIN_RAMP_MS; }
uint32_t Precheck_SpinSampleFromMs(void) { return PC_SPIN_SAMPLE_MS; }
uint32_t Precheck_SpinEndMs(void)        { return PC_SPIN_END_MS; }
uint32_t Precheck_SpinTimeoutMs(void)    { return PC_SPIN_TIMEOUT_MS; }

void Precheck_SpinBegin(void)
{
    s_sp_n = 0U;
    s_sp_rpm = 0.0;
    s_sp_p = 0.0;
    s_sp_p2 = 0.0;
    s_sp_q = 0.0;
    s_sp_q2 = 0.0;
}

void Precheck_SpinSample(void)
{
    const double p = (double)(measPS.pressure / PC_MBAR_PER_CMH2O);
    const double q = (double)Flow_GetSlm();
    s_sp_rpm += (double)g_blower_status.mech_speed_rpm;
    s_sp_p += p;
    s_sp_p2 += p * p;
    s_sp_q += q;
    s_sp_q2 += q * q;
    s_sp_n++;
}

static float stdOf(double sum, double sum2, uint32_t n)
{
    const double m = sum / (double)n;
    const double v = (sum2 / (double)n) - (m * m);
    return (v > 0.0) ? (float)sqrt(v) : 0.0f;
}

bool Precheck_SpinVerdict(PrecheckSpin_t *out)
{
    PrecheckSpin_t r;
    memset(&r, 0, sizeof(r));
    r.n = s_sp_n;
    if (s_sp_n == 0U)
    {
        r.fail = PRECHECK_SPIN_SPEED;
    }
    else
    {
        r.rpm_mean = (float)(s_sp_rpm / (double)s_sp_n);
        r.p_mean = (float)(s_sp_p / (double)s_sp_n);
        r.q_mean = (float)(s_sp_q / (double)s_sp_n);
        r.p_std = stdOf(s_sp_p, s_sp_p2, s_sp_n);
        r.q_std = stdOf(s_sp_q, s_sp_q2, s_sp_n);
        const float n_k = r.rpm_mean * 0.001f;
        const float q = r.q_mean;
        r.p_model = (g_flow_fan_a * n_k * n_k) + (g_flow_fan_b * n_k * q)
                  + (g_flow_fan_c * q * fabsf(q)) + (g_flow_fan_d * q);

        if (fabsf(r.rpm_mean - (float)PC_SPIN_RPM) > PC_SPIN_SPEED_TOL)
        {
            r.fail = PRECHECK_SPIN_SPEED;
        }
        else if ((g_precheck_spin_map_enable != 0U)
                 && (fabsf(r.p_mean - r.p_model)
                     > (PC_SPIN_MAP_ABS + (PC_SPIN_MAP_REL * fabsf(r.p_model)))))
        {
            r.fail = PRECHECK_SPIN_MAP;
        }
        else if ((r.p_std < PC_SPIN_P_STD_MIN) || (r.q_std < PC_SPIN_Q_STD_MIN))
        {
            r.fail = PRECHECK_SPIN_FLAT;
        }
        else if ((r.p_std > PC_SPIN_P_STD_MAX) || (r.q_std > PC_SPIN_Q_STD_MAX))
        {
            r.fail = PRECHECK_SPIN_NOISY;
        }
    }
    if ((r.fail == PRECHECK_OK) && (s_inject >= PRECHECK_SPIN_SPEED))
    {
        r.fail = s_inject;
        r.injected = true;
    }
    r.seq = s_last_spin.seq + 1U;
    s_last_spin = r;
    s_last_fail = r.fail;
    if (out != NULL)
    {
        *out = r;
    }
    return (r.fail == PRECHECK_OK);
}

const PrecheckSpin_t* Precheck_GetLastSpin(void)
{
    return &s_last_spin;
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
