/**
  ******************************************************************************
  * @file    circuit_learn.c
  * @brief   Learn circuit: measure the attached tube (see circuit_learn.h).
  ******************************************************************************
  */

#include "circuit_learn.h"
#include "tube_comp.h"
#include "precheck.h"
#include "therapy.h"
#include "blower_ctrl.h"
#include "blower_ipc.h"
#include "flow_sensor.h"
#include "pressure_sensors.h"
#include "session_edf_log.h"
#include "rtc_clock.h"
#include "dbg_log.h"
#include "ff.h"
#include "main.h"

#include <math.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

volatile uint8_t g_circuit_learn_apply   = 1U;
volatile int32_t g_circuit_learn_rpm_max = 26000;
volatile uint8_t g_circuit_learn_restore = 1U;

#define CL_PATH              "0:/CIRCUIT.CSV"
#define CL_SCHEMA            1U
#define CL_MBAR_PER_CMH2O    0.980665f
#define CL_RPM_FIRST         12000
#define CL_RPM_CAP           26000     /* open-outlet bench limit */
#define CL_REST_RPM          500
#define CL_WAIT_MAX_MS       15000U    /* CM4 restart hold-off */
#define CL_ZERO_MS           1000U
#define CL_SETTLE_MS         800U      /* at speed before sampling */
#define CL_SAMPLE_MS         1000U
#define CL_STEP_TIMEOUT_MS   6000U
#define CL_SPEED_TOL         800.0f
#define CL_FIRST_RAMP_MS     1000U
#define CL_STEP_RAMP_MS      800U
#define CL_DOWN_RPM          12000
#define CL_DOWN_RAMP_MS      2000U
#define CL_DOWN_STOP_RPM     13500
#define CL_DOWN_MAX_MS       5000U
/* Open-end test after the first step: an open tube passes 40-60 L/min at
 * 12 krpm with well under half the fan's shut-off pressure; mask on (vent
 * only) passes ~12 L/min at nearly all of it. */
#define CL_OPEN_P_FRAC       0.6f
#define CL_OPEN_Q_MIN        20.0f
/* Map-free version of the same test, P / Q^2 at the first step (cmH2O per
 * (L/min)^2). Bench 2026-10-27 at 12 krpm: open tube 0.78 / 60.1 L/min =
 * 0.00022; external sensor body left on the end 0.0011; circuit connected
 * (vent + lung) 2.82 / 28.3 = 0.0035. The worst tube accepted (3 cmH2O at
 * 60 L/min, CL_D60_MAX) is 0.00083. Needed where the fan map is not the
 * blower's (TKFan): there the shut-off fraction alone can pass a mask. */
#define CL_OPEN_PQ2_MAX      0.0015f
/* Climb limits. */
#define CL_P_MAX             10.0f
#define CL_Q_MAX             140.0f
#define CL_SUPPLY_A_MAX      0.8f
#define CL_SUPPLY_MS         500U
#define CL_VBUS_MIN_V        20.0f
#define CL_VBUS_MAX_V        28.0f
/* Verdict. */
#define CL_MIN_POINTS        3U
#define CL_MIN_QMAX          50.0f
#define CL_RMS_ABS           0.10f
#define CL_RMS_REL           0.05f
#define CL_D60_MAX           3.0f
#define CL_RLAM_MAX          0.05f     /* tube_comp clamps */
#define CL_RTURB_MAX         0.001f
#define CL_DIFF_ABS          0.15f
#define CL_DIFF_REL          0.30f
#define CL_NO_TUBE_D60       0.05f
#define CL_Q0_MAX            2.0f
#define CL_LINE_MAX          320U
#define CL_RING_N            4U

/* main.c */
extern AMS5935_HandleTypeDef hamsFS;
extern AMS5935_Data_t        measPS;
extern BlowerIpcStatus_t     g_blower_status;
/* flow_sensor.c fan map (cmH2O; krpm). */
extern volatile float        g_flow_fan_a;

typedef struct
{
    uint8_t              event;      /* 0 learn, 1 reset */
    RtcDateTime_t        dt;
    uint8_t              dt_valid;
    uint32_t             uptime_ms;
    int32_t              rpm_max;
    CircuitLearnResult_t r;
} ClRow_t;

static CircuitLearnPhase_t s_phase = CL_PHASE_IDLE;
static CircuitLearnResult_t s_res = { .fail = CL_NONE };
static uint8_t  s_step;
static uint32_t s_ph_ms;          /* time in the phase / step */
static uint32_t s_at_ms;          /* time at the step speed */
static uint32_t s_sup_ms;         /* time over the supply limit */
static bool     s_started;        /* blower commanded */
static bool     s_sat;
static int32_t  s_rpm_max;
static uint32_t s_acc_n;
static double   s_acc_rpm;
static double   s_acc_p;
static double   s_acc_q;
static double   s_acc_a;
static const char *s_source = "default";

static ClRow_t           s_ring[CL_RING_N];
static volatile uint32_t s_rows = 0U;
static volatile uint32_t s_written = 0U;
static uint32_t          s_csv_err = 0U;
static bool              s_loaded = false;

static const char * const k_name[CL_COUNT] = {
    "tube learned",
    "not run",
    "therapy or blower running",
    "sensor self-test failed",
    "tube end not open",
    "blower did not reach speed",
    "motor drive fault",
    "too few steps",
    "flow too low to fit",
    "inconsistent readings",
    "tube resistance too high",
    "stopped",
};

static const char * const k_token[CL_COUNT] = {
    "ok", "none", "busy", "precheck", "not_open", "speed", "drive",
    "few_points", "low_flow", "fit", "high", "aborted",
};

static const char * const k_phase[] = {
    "idle", "wait", "zero", "step", "rampdown", "done",
};

static float psCmh2o(void)
{
    return measPS.pressure / CL_MBAR_PER_CMH2O;
}

static float supplyAmps(void)
{
    const float v = (float)g_blower_status.bus_voltage_mv * 0.001f;
    if (v < 1.0f)
    {
        return 0.0f;
    }
    const float p_w = 1.5e-6f * (((float)g_blower_status.vq_mv * (float)g_blower_status.iq_ma)
                               + ((float)g_blower_status.vd_mv * (float)g_blower_status.id_ma));
    return p_w / v;
}

static int32_t stepRpm(uint8_t k)
{
    return CL_RPM_FIRST + (int32_t)(((s_rpm_max - CL_RPM_FIRST) * (int32_t)k)
                                    / (int32_t)(CL_STEPS - 1U));
}

static float dropAt(float r_lam, float r_turb, float q)
{
    return (r_lam * q) + (r_turb * q * q);
}

static void accReset(void)
{
    s_acc_n = 0U;
    s_acc_rpm = 0.0;
    s_acc_p = 0.0;
    s_acc_q = 0.0;
    s_acc_a = 0.0;
    s_sat = false;
}

static void queueRow(uint8_t event)
{
    ClRow_t row;
    memset(&row, 0, sizeof(row));
    row.event = event;
    row.uptime_ms = HAL_GetTick();
    row.dt_valid = (RtcClock_IsTimeValid() && RtcClock_Get(&row.dt)) ? 1U : 0U;
    if (event == 0U)
    {
        row.rpm_max = s_rpm_max;
        row.r = s_res;
    }
    else
    {
        row.r.r_lam = TubeComp_GetRLam();
        row.r.r_turb = TubeComp_GetRTurb();
        row.r.d60 = dropAt(row.r.r_lam, row.r.r_turb, 60.0f);
        row.r.d100 = dropAt(row.r.r_lam, row.r.r_turb, 100.0f);
        row.r.applied = true;
        row.r.fail = CL_OK;
    }
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    s_ring[s_rows % CL_RING_N] = row;
    s_rows++;
    if ((s_rows - s_written) > CL_RING_N)
    {
        s_written = s_rows - CL_RING_N;
    }
    __set_PRIMASK(primask);
}

/* Least squares dP = R_lam*Q + R_turb*Q^2 through the origin, both >= 0. */
static void fit(void)
{
    CircuitLearnResult_t *r = &s_res;
    double s11 = 0.0, s12 = 0.0, s22 = 0.0, b1 = 0.0, b2 = 0.0;
    float qmax = 0.0f, dmax = 0.0f;
    for (uint8_t i = 0U; i < r->n; i++)
    {
        const double q = (double)r->pt[i].q;
        const double d = (double)r->pt[i].p;
        s11 += q * q;
        s12 += q * q * q;
        s22 += q * q * q * q;
        b1  += d * q;
        b2  += d * q * q;
        if (r->pt[i].q > qmax) { qmax = r->pt[i].q; }
        if (r->pt[i].p > dmax) { dmax = r->pt[i].p; }
    }
    if (r->n < CL_MIN_POINTS)
    {
        r->fail = CL_FEW_POINTS;
        return;
    }
    if (qmax < CL_MIN_QMAX)
    {
        r->fail = CL_LOW_FLOW;
        return;
    }
    double rl = 0.0, rt = 0.0;
    const double det = (s11 * s22) - (s12 * s12);
    if (det > 0.0)
    {
        rl = ((b1 * s22) - (b2 * s12)) / det;
        rt = ((s11 * b2) - (s12 * b1)) / det;
    }
    if ((det <= 0.0) || (rl < 0.0))
    {
        rl = 0.0;
        rt = (s22 > 0.0) ? (b2 / s22) : 0.0;
    }
    if (rt < 0.0)
    {
        rt = 0.0;
        rl = (s11 > 0.0) ? (b1 / s11) : 0.0;
    }
    if (rl < 0.0) { rl = 0.0; }
    r->r_lam = (float)rl;
    r->r_turb = (float)rt;
    double e2 = 0.0;
    for (uint8_t i = 0U; i < r->n; i++)
    {
        const double e = (double)r->pt[i].p
                       - (double)dropAt(r->r_lam, r->r_turb, r->pt[i].q);
        e2 += e * e;
    }
    r->rms = (float)sqrt(e2 / (double)r->n);
    r->d60 = dropAt(r->r_lam, r->r_turb, 60.0f);
    r->d100 = dropAt(r->r_lam, r->r_turb, 100.0f);

    if (r->rms > fmaxf(CL_RMS_ABS, CL_RMS_REL * dmax))
    {
        r->fail = CL_FIT;
    }
    else if ((r->d60 > CL_D60_MAX) || (r->r_lam > CL_RLAM_MAX)
             || (r->r_turb > CL_RTURB_MAX))
    {
        r->fail = CL_HIGH;
    }
    if (r->fail != CL_OK)
    {
        return;
    }
    if (fabsf(r->d60 - r->prev_d60) > fmaxf(CL_DIFF_ABS, CL_DIFF_REL * r->prev_d60))
    {
        r->warn |= CL_WARN_DIFFERS;
    }
    if (r->d60 < CL_NO_TUBE_D60)
    {
        r->warn |= CL_WARN_NO_TUBE;
    }
}

/* Leave the speed steps: verdict now (the screen shows it during the
 * ramp-down), then bring the blower down. */
static void endSteps(CircuitLearnFail_t fail, const char *why)
{
    if ((why != NULL) && (s_res.stop_why[0] == '\0'))
    {
        (void)snprintf(s_res.stop_why, sizeof(s_res.stop_why), "%s", why);
    }
    s_res.fail = fail;
    if (fail == CL_OK)
    {
        fit();
    }
    if ((s_res.fail == CL_OK) && (g_circuit_learn_apply != 0U))
    {
        TubeComp_SetCoefficients(s_res.r_lam, s_res.r_turb);
        s_res.applied = true;
        s_source = "learned";
    }
    DBG_I("CIR", "learn %s: n=%u R_lam=%.5f R_turb=%.6f d60=%.2f (was %.2f) rms=%.3f %s",
          k_token[s_res.fail], (unsigned)s_res.n, (double)s_res.r_lam,
          (double)s_res.r_turb, (double)s_res.d60, (double)s_res.prev_d60,
          (double)s_res.rms, s_res.applied ? "applied" : "kept");
    queueRow(0U);
    s_phase = CL_PHASE_RAMPDOWN;
    s_ph_ms = 0U;
    if (s_started && (g_blower_status.mech_speed_rpm > CL_DOWN_STOP_RPM))
    {
        BlowerCtrl_ManualSetSpeed(CL_DOWN_RPM, (uint16_t)CL_DOWN_RAMP_MS);
    }
}

static void beginStep(uint8_t k)
{
    s_step = k;
    s_ph_ms = 0U;
    s_at_ms = 0U;
    s_sup_ms = 0U;
    accReset();
    if (k == 0U)
    {
        BlowerCtrl_ManualStart(stepRpm(0U), (uint16_t)CL_FIRST_RAMP_MS);
        s_started = true;
    }
    else
    {
        BlowerCtrl_ManualSetSpeed(stepRpm(k), (uint16_t)CL_STEP_RAMP_MS);
    }
    s_phase = CL_PHASE_STEP;
}

CircuitLearnFail_t CircuitLearn_Start(void)
{
    if (CircuitLearn_IsBusy())
    {
        return CL_BUSY;
    }
    const int32_t rpm = g_blower_status.mech_speed_rpm;
    if ((Therapy_GetState() != THERAPY_STATE_IDLE) || BlowerCtrl_IsManualActive()
        || (rpm > CL_REST_RPM) || (rpm < -CL_REST_RPM))
    {
        return CL_BUSY;
    }
    PrecheckResult_t pc;
    if (!Precheck_Run(&pc))
    {
        memset(&s_res, 0, sizeof(s_res));
        s_res.fail = CL_PRECHECK;
        (void)snprintf(s_res.stop_why, sizeof(s_res.stop_why), "%s",
                       Precheck_FailToken(pc.fail));
        return CL_PRECHECK;
    }
    memset(&s_res, 0, sizeof(s_res));
    s_res.fail = CL_OK;
    s_res.prev_d60 = dropAt(TubeComp_GetRLam(), TubeComp_GetRTurb(), 60.0f);
    int32_t top = g_circuit_learn_rpm_max;
    if (top > CL_RPM_CAP)                   { top = CL_RPM_CAP; }
    if (top < (CL_RPM_FIRST + 4000))        { top = CL_RPM_FIRST + 4000; }
    s_rpm_max = top;
    s_started = false;
    s_step = 0U;
    s_ph_ms = 0U;
    accReset();
    s_phase = (g_blower_status.restart_holdoff_ms > 0U) ? CL_PHASE_WAIT : CL_PHASE_ZERO;
    DBG_I("CIR", "learn start: 12000..%ld rpm", (long)s_rpm_max);
    return CL_OK;
}

void CircuitLearn_Abort(void)
{
    if ((s_phase == CL_PHASE_IDLE) || (s_phase == CL_PHASE_DONE)
        || (s_phase == CL_PHASE_RAMPDOWN))
    {
        return;
    }
    s_res.n = 0U;                             /* nothing from a stopped run */
    endSteps(CL_ABORTED, "abort");
}

bool CircuitLearn_IsBusy(void)
{
    return (s_phase != CL_PHASE_IDLE) && (s_phase != CL_PHASE_DONE);
}

void CircuitLearn_Update(uint32_t dt_ms)
{
    const int32_t mech = g_blower_status.mech_speed_rpm;
    const float vbus = (float)g_blower_status.bus_voltage_mv * 0.001f;

    switch (s_phase)
    {
    case CL_PHASE_WAIT:
        s_ph_ms += dt_ms;
        if (g_blower_status.restart_holdoff_ms == 0U)
        {
            s_phase = CL_PHASE_ZERO;
            s_ph_ms = 0U;
            accReset();
        }
        else if (s_ph_ms > CL_WAIT_MAX_MS)
        {
            endSteps(CL_SPEED, "holdoff");
        }
        break;

    case CL_PHASE_ZERO:
        s_ph_ms += dt_ms;
        s_acc_p += (double)psCmh2o();
        s_acc_q += (double)Flow_GetSlm();
        s_acc_n++;
        if (s_ph_ms >= CL_ZERO_MS)
        {
            s_res.p0 = (float)(s_acc_p / (double)s_acc_n);
            s_res.q0 = (float)(s_acc_q / (double)s_acc_n);
            if (fabsf(s_res.q0) > CL_Q0_MAX)
            {
                s_res.warn |= CL_WARN_FLOW_ZERO;
            }
            beginStep(0U);
        }
        break;

    case CL_PHASE_STEP:
    {
        s_ph_ms += dt_ms;
        if ((g_blower_status.current_faults != 0U)
            || (vbus < CL_VBUS_MIN_V) || (vbus > CL_VBUS_MAX_V))
        {
            endSteps(CL_DRIVE, "drive");
            break;
        }
        const float p = psCmh2o() - s_res.p0;
        const float amps = supplyAmps();
        s_sup_ms = (amps > CL_SUPPLY_A_MAX) ? (s_sup_ms + dt_ms) : 0U;
        if ((p > CL_P_MAX) || (s_sup_ms >= CL_SUPPLY_MS))
        {
            /* Keep the steps already done; this one is not finished. */
            endSteps(CL_OK, (p > CL_P_MAX) ? "p_max" : "current");
            break;
        }
        const float cmd = (float)stepRpm(s_step);
        if (s_at_ms < CL_SETTLE_MS)
        {
            if (fabsf((float)mech - cmd) <= CL_SPEED_TOL)
            {
                s_at_ms += dt_ms;
            }
            else
            {
                s_at_ms = 0U;
            }
            if ((s_at_ms < CL_SETTLE_MS) && (s_ph_ms > CL_STEP_TIMEOUT_MS))
            {
                endSteps(CL_SPEED, "speed");
            }
            break;
        }
        s_at_ms += dt_ms;
        s_acc_rpm += (double)mech;
        s_acc_p += (double)p;
        s_acc_q += (double)Flow_GetSlm();
        s_acc_a += (double)amps;
        s_acc_n++;
        if (AMS5935_WasSampleSaturated(&hamsFS) != 0U)
        {
            s_sat = true;
        }
        if (s_at_ms < (CL_SETTLE_MS + CL_SAMPLE_MS))
        {
            break;
        }
        if (s_sat)
        {
            endSteps(CL_OK, "q_sat");          /* flow unusable at this step */
            break;
        }
        CircuitLearnPoint_t *pt = &s_res.pt[s_res.n];
        pt->rpm = (float)(s_acc_rpm / (double)s_acc_n);
        pt->p = (float)(s_acc_p / (double)s_acc_n);
        pt->q = (float)(s_acc_q / (double)s_acc_n);
        pt->supply_a = (float)(s_acc_a / (double)s_acc_n);
        s_res.n++;
        if (s_step == 0U)
        {
            const float n_k = pt->rpm * 0.001f;
            const float shut = g_flow_fan_a * n_k * n_k;
            if ((pt->p > (CL_OPEN_P_FRAC * shut)) || (pt->q < CL_OPEN_Q_MIN)
                || (pt->p > (CL_OPEN_PQ2_MAX * pt->q * pt->q)))
            {
                endSteps(CL_NOT_OPEN, NULL);
                break;
            }
        }
        if ((uint8_t)(s_step + 1U) >= CL_STEPS)
        {
            endSteps(CL_OK, NULL);
        }
        else if (pt->q > CL_Q_MAX)
        {
            endSteps(CL_OK, "q_max");
        }
        else
        {
            beginStep((uint8_t)(s_step + 1U));
        }
        break;
    }

    case CL_PHASE_RAMPDOWN:
        s_ph_ms += dt_ms;
        if (!s_started)
        {
            s_phase = CL_PHASE_DONE;
        }
        else if ((mech < CL_DOWN_STOP_RPM) || (s_ph_ms > CL_DOWN_MAX_MS))
        {
            BlowerCtrl_ManualStop();
            s_started = false;
            s_phase = CL_PHASE_DONE;
        }
        break;

    case CL_PHASE_IDLE:
    case CL_PHASE_DONE:
    default:
        break;
    }
}

void CircuitLearn_ResetToDefault(void)
{
    TubeComp_SetCoefficients(TubeComp_GetDefaultRLam(), TubeComp_GetDefaultRTurb());
    s_source = "default";
    queueRow(1U);
    DBG_I("CIR", "tube coefficients back to default");
}

/* === CIRCUIT.CSV ========================================================== */

static const char k_header[] =
    "schema,time,rtc_valid,uptime_s,event,result,warn,applied,r_lam,r_turb,"
    "d60,d100,rms,n,p0,q0,prev_d60,rpm_max,stop,points";

static int formatRow(const ClRow_t *w, char *buf, unsigned len)
{
    char t[24];
    if (w->dt_valid != 0U)
    {
        (void)snprintf(t, sizeof(t), "20%02u-%02u-%02u %02u:%02u:%02u",
                       (unsigned)w->dt.yy, (unsigned)w->dt.mo, (unsigned)w->dt.dd,
                       (unsigned)w->dt.hh, (unsigned)w->dt.mi, (unsigned)w->dt.ss);
    }
    else
    {
        t[0] = '\0';
    }
    const CircuitLearnResult_t *r = &w->r;
    int n = snprintf(buf, len,
        "%u,%s,%u,%lu,%s,%s,%u,%u,%.6f,%.7f,%.3f,%.3f,%.3f,%u,%.3f,%.2f,%.3f,%ld,%s,",
        (unsigned)CL_SCHEMA, t, (unsigned)w->dt_valid,
        (unsigned long)(w->uptime_ms / 1000U),
        (w->event != 0U) ? "reset" : "learn", k_token[r->fail], (unsigned)r->warn,
        r->applied ? 1U : 0U, (double)r->r_lam, (double)r->r_turb, (double)r->d60,
        (double)r->d100, (double)r->rms, (unsigned)r->n, (double)r->p0, (double)r->q0,
        (double)r->prev_d60, (long)w->rpm_max, r->stop_why);
    if ((n <= 0) || ((unsigned)n >= len))
    {
        return 0;
    }
    for (uint8_t i = 0U; i < r->n; i++)
    {
        const int m = snprintf(&buf[n], len - (unsigned)n, "%s%.0f/%.1f/%.3f",
                               (i > 0U) ? ";" : "", (double)r->pt[i].rpm,
                               (double)r->pt[i].q, (double)r->pt[i].p);
        if ((m <= 0) || ((unsigned)m >= (len - (unsigned)n)))
        {
            return 0;
        }
        n += m;
    }
    if (((unsigned)n + 3U) > len)
    {
        return 0;
    }
    buf[n++] = '\r';
    buf[n++] = '\n';
    buf[n] = '\0';
    return n;
}

static void writePending(void)
{
    if ((s_written == s_rows) || !SessionEdf_IsMounted())
    {
        return;
    }
    FIL f;
    UINT bw = 0U;
    FRESULT fr = f_open(&f, CL_PATH, FA_OPEN_APPEND | FA_WRITE);
    if (fr != FR_OK)
    {
        s_csv_err = 0x9100U | ((uint32_t)fr & 0xFFU);
        return;
    }
    if (f_size(&f) == 0U)
    {
        char hdr[sizeof(k_header) + 64U];
        const int hn = snprintf(hdr, sizeof(hdr), "# SomnaLink circuit log, schema %u\r\n%s\r\n",
                                (unsigned)CL_SCHEMA, k_header);
        if (hn > 0) { fr = f_write(&f, hdr, (UINT)hn, &bw); }
    }
    while ((fr == FR_OK) && (s_written != s_rows))
    {
        static char line[CL_LINE_MAX];
        ClRow_t w;
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        w = s_ring[s_written % CL_RING_N];
        __set_PRIMASK(primask);
        const int n = formatRow(&w, line, sizeof(line));
        if (n > 0)
        {
            fr = f_write(&f, line, (UINT)n, &bw);
        }
        if (fr == FR_OK)
        {
            s_written++;
        }
    }
    const FRESULT cr = f_close(&f);
    s_csv_err = ((fr == FR_OK) && (cr == FR_OK)) ? 0U
              : (0x9200U | ((uint32_t)((fr != FR_OK) ? fr : cr) & 0xFFU));
}

/* A data row with applied=1: take its coefficients. Columns per k_header;
 * only schema 1 rows are read. */
static void takeLine(char *line, float *rl, float *rt, bool *found)
{
    if ((line[0] < '0') || (line[0] > '9'))
    {
        return;                              /* comment / header */
    }
    char *col[10];
    uint32_t nc = 0U;
    char *p = line;
    col[nc++] = p;
    while ((*p != '\0') && (nc < 10U))
    {
        if (*p == ',')
        {
            *p = '\0';
            col[nc++] = p + 1;
        }
        p++;
    }
    if ((nc < 10U) || (strtoul(col[0], NULL, 10) != CL_SCHEMA)
        || (strcmp(col[7], "1") != 0))
    {
        return;
    }
    char *end1 = NULL;
    char *end2 = NULL;
    const float a = strtof(col[8], &end1);
    const float b = strtof(col[9], &end2);
    if ((end1 != col[8]) && (end2 != col[9]) && (a >= 0.0f) && (b >= 0.0f)
        && (a <= CL_RLAM_MAX) && (b <= CL_RTURB_MAX))
    {
        *rl = a;
        *rt = b;
        *found = true;
    }
}

static void loadFromCard(void)
{
    FIL f;
    static uint8_t blk[512];
    static char line[CL_LINE_MAX];
    uint32_t ll = 0U;
    bool overlong = false;
    float rl = 0.0f, rt = 0.0f;
    bool found = false;

    const FRESULT fo = f_open(&f, CL_PATH, FA_READ);
    if (fo != FR_OK)
    {
        s_csv_err = (fo == FR_NO_FILE) ? 0U : (0x9300U | ((uint32_t)fo & 0xFFU));
        return;
    }
    for (;;)
    {
        UINT br = 0U;
        const FRESULT fr = f_read(&f, blk, sizeof(blk), &br);
        if ((fr != FR_OK) || (br == 0U))
        {
            break;
        }
        for (UINT i = 0U; i < br; i++)
        {
            const char ch = (char)blk[i];
            if ((ch == '\n') || (ch == '\r'))
            {
                line[ll] = '\0';
                if (!overlong && (ll > 0U))
                {
                    takeLine(line, &rl, &rt, &found);
                }
                ll = 0U;
                overlong = false;
            }
            else if (ll < (CL_LINE_MAX - 1U))
            {
                line[ll++] = ch;
            }
            else
            {
                overlong = true;
            }
        }
    }
    if ((ll > 0U) && !overlong)
    {
        line[ll] = '\0';
        takeLine(line, &rl, &rt, &found);
    }
    (void)f_close(&f);
    if (found)
    {
        TubeComp_SetCoefficients(rl, rt);
        s_source = "card";
        DBG_I("CIR", "tube from CIRCUIT.CSV: R_lam=%.5f R_turb=%.6f d60=%.2f",
              (double)rl, (double)rt, (double)dropAt(rl, rt, 60.0f));
    }
}

void CircuitLearn_Service(void)
{
    if (!SessionEdf_IsMounted())
    {
        s_loaded = false;                    /* reload for whichever card is next */
        return;
    }
    /* Once per card, and only while idle: never swap the tube mid-session. */
    if (!s_loaded && (Therapy_GetState() == THERAPY_STATE_IDLE)
        && !CircuitLearn_IsBusy())
    {
        s_loaded = true;
        if (g_circuit_learn_restore != 0U)
        {
            loadFromCard();
        }
    }
    writePending();
}

/* === Accessors ============================================================ */

CircuitLearnPhase_t CircuitLearn_GetPhase(void) { return s_phase; }
uint8_t CircuitLearn_GetStep(void)              { return s_step; }
uint8_t CircuitLearn_GetStepCount(void)         { return (uint8_t)CL_STEPS; }
const CircuitLearnResult_t* CircuitLearn_GetResult(void) { return &s_res; }
const char* CircuitLearn_GetSourceName(void)    { return s_source; }
uint32_t CircuitLearn_GetCsvError(void)         { return s_csv_err; }
uint32_t CircuitLearn_GetCsvRows(void)          { return s_written; }

const char* CircuitLearn_FailName(CircuitLearnFail_t f)
{
    return ((uint32_t)f < (uint32_t)CL_COUNT) ? k_name[f] : "?";
}

const char* CircuitLearn_FailToken(CircuitLearnFail_t f)
{
    return ((uint32_t)f < (uint32_t)CL_COUNT) ? k_token[f] : "?";
}

const char* CircuitLearn_PhaseName(CircuitLearnPhase_t p)
{
    return ((uint32_t)p <= (uint32_t)CL_PHASE_DONE) ? k_phase[p] : "?";
}
