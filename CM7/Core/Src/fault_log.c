/**
  ******************************************************************************
  * @file    fault_log.c
  * @brief   Durable fault log -> 0:/FAULTS.CSV (see fault_log.h).
  ******************************************************************************
  */

#include "fault_log.h"
#include "session_edf_log.h"
#include "rtc_clock.h"
#include "therapy.h"
#include "flow_sensor.h"
#include "blower_ctrl.h"
#include "blower_ipc.h"
#include "ff.h"
#include "main.h"

#include <stdio.h>
#include <string.h>

#define FAULTS_PATH  "0:/FAULTS.CSV"

extern BlowerIpcStatus_t g_blower_status;   /* main.c, refreshed at 100 Hz */

typedef struct
{
    uint32_t      seq;
    uint32_t      uptime_ms;
    RtcDateTime_t dt;
    uint8_t       dt_valid;
    uint8_t       source;
    uint16_t      code;
    char          name[32];
    char          why[48];
    uint8_t       th_state;
    uint8_t       mode;
    uint32_t      run_s;
    float         target;
    float         setpoint;
    float         meas;
    float         flow;
    float         leak;
    int32_t       rpm_cmd;
    int32_t       rpm_mech;
    int16_t       iq_ma;
    int16_t       id_ma;
    uint16_t      vbus_mv;
    uint16_t      mc_state;
    uint16_t      cur_faults;
    uint16_t      occ_faults;
} FaultRec_t;

static const char k_header[] =
    "schema,seq,time,rtc_valid,uptime_s,source,code,name,detail,"
    "therapy_state,mode,run_s,target,setpoint,meas,flow,leak,"
    "rpm_cmd,rpm_mech,iq_ma,id_ma,vbus_mv,mc_state,cur_faults,occ_faults";

static FaultRec_t        s_ring[FAULT_LOG_RING_N];
static volatile uint32_t s_count = 0U;        /* records ever made (seq) */
static volatile uint32_t s_written = 0U;      /* records on the card */
static uint32_t          s_error = 0U;
static uint16_t          s_prev_occ = 0U;

/* Copy text into a CSV field: commas would split it, so they become ';'. */
static void copyField(char *dst, size_t len, const char *src)
{
    size_t i = 0U;
    if (src != NULL)
    {
        for (; (src[i] != '\0') && (i < (len - 1U)); i++)
        {
            dst[i] = (src[i] == ',') ? ';' : src[i];
        }
    }
    dst[i] = '\0';
}

static void record(uint8_t source, uint16_t code, const char *name, const char *why)
{
    FaultRec_t r;
    memset(&r, 0, sizeof(r));
    r.uptime_ms = HAL_GetTick();
    r.dt_valid = (RtcClock_IsTimeValid() && RtcClock_Get(&r.dt)) ? 1U : 0U;
    r.source = source;
    r.code = code;
    copyField(r.name, sizeof(r.name), name);
    copyField(r.why, sizeof(r.why), why);
    r.th_state = (uint8_t)Therapy_GetState();
    r.mode = (uint8_t)Therapy_GetMode();
    r.run_s = Therapy_GetRunSeconds();
    r.target = Therapy_GetTargetPressure();
    r.setpoint = Therapy_GetSetpointCmh2o();
    r.meas = Therapy_GetMeasuredCmh2o();
    r.flow = Flow_GetPatientSlm();
    r.leak = Therapy_GetLeakSlm();
    r.rpm_cmd = (int32_t)BlowerCtrl_GetOutputRpm();
    r.rpm_mech = g_blower_status.mech_speed_rpm;
    r.iq_ma = g_blower_status.iq_ma;
    r.id_ma = g_blower_status.id_ma;
    r.vbus_mv = g_blower_status.bus_voltage_mv;
    r.mc_state = g_blower_status.mc_state;
    r.cur_faults = g_blower_status.current_faults;
    r.occ_faults = g_blower_status.occurred_faults;

    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    r.seq = s_count + 1U;
    s_ring[s_count % FAULT_LOG_RING_N] = r;
    s_count++;
    if ((s_count - s_written) > FAULT_LOG_RING_N)
    {
        s_written = s_count - FAULT_LOG_RING_N;   /* oldest unsaved overwritten */
    }
    __set_PRIMASK(primask);
}

void FaultLog_RecordTherapy(uint16_t code, const char *name, const char *why)
{
    record((uint8_t)FAULT_SRC_THERAPY, code, name, why);
}

/* MCSDK fault bits (CM4 mc_type.h) as names joined with '+'. */
static void driveFaultNames(uint16_t bits, char *buf, size_t len)
{
    static const struct { uint16_t bit; const char *name; } k[] = {
        { 0x0001U, "DURATION" }, { 0x0002U, "OVER_VOLT" }, { 0x0004U, "UNDER_VOLT" },
        { 0x0008U, "OVER_TEMP" }, { 0x0010U, "START_UP" }, { 0x0020U, "SPEED_FDBK" },
        { 0x0040U, "OVER_CURR" }, { 0x0080U, "SW_ERROR" }, { 0x0400U, "DP_FAULT" },
    };
    size_t n = 0U;
    buf[0] = '\0';
    for (size_t i = 0U; i < (sizeof(k) / sizeof(k[0])); i++)
    {
        if ((bits & k[i].bit) != 0U)
        {
            const int m = snprintf(&buf[n], len - n, "%s%s", (n > 0U) ? "+" : "", k[i].name);
            if ((m < 0) || ((size_t)m >= (len - n))) { break; }
            n += (size_t)m;
        }
    }
    if (n == 0U)
    {
        (void)snprintf(buf, len, "MC_0x%04X", (unsigned)bits);
    }
}

static int formatRecord(const FaultRec_t *r, char *buf, unsigned len, bool csv)
{
    char t[24];
    if (r->dt_valid != 0U)
    {
        (void)snprintf(t, sizeof(t), "20%02u-%02u-%02u %02u:%02u:%02u",
                       (unsigned)r->dt.yy, (unsigned)r->dt.mo, (unsigned)r->dt.dd,
                       (unsigned)r->dt.hh, (unsigned)r->dt.mi, (unsigned)r->dt.ss);
    }
    else
    {
        t[0] = '\0';
    }
    const int n = snprintf(buf, len,
        "%s%u,%lu,%s,%u,%lu,%s,%u,%s,%s,%u,%u,%lu,%.2f,%.2f,%.2f,%.1f,%.1f,"
        "%ld,%ld,%d,%d,%u,%u,0x%04X,0x%04X%s",
        csv ? "" : "# FLT ",
        (unsigned)FAULT_LOG_SCHEMA, (unsigned long)r->seq, t, (unsigned)r->dt_valid,
        (unsigned long)(r->uptime_ms / 1000U),
        (r->source == (uint8_t)FAULT_SRC_DRIVE) ? "drive" : "therapy",
        (unsigned)r->code, r->name, r->why,
        (unsigned)r->th_state, (unsigned)r->mode, (unsigned long)r->run_s,
        (double)r->target, (double)r->setpoint, (double)r->meas,
        (double)r->flow, (double)r->leak,
        (long)r->rpm_cmd, (long)r->rpm_mech, (int)r->iq_ma, (int)r->id_ma,
        (unsigned)r->vbus_mv, (unsigned)r->mc_state,
        (unsigned)r->cur_faults, (unsigned)r->occ_faults, "\r\n");
    return ((n > 0) && ((unsigned)n < len)) ? n : 0;
}

/* Append records not yet on the card. Errors are kept and retried on the
 * next mount - the RAM ring still holds the last FAULT_LOG_RING_N. */
static void writePending(void)
{
    if ((s_written == s_count) || !SessionEdf_IsMounted())
    {
        return;
    }
    FIL f;
    UINT bw = 0U;
    FRESULT fr = f_open(&f, FAULTS_PATH, FA_OPEN_APPEND | FA_WRITE);
    if (fr != FR_OK)
    {
        s_error = 0x8100U | ((uint32_t)fr & 0xFFU);
        return;
    }
    if (f_size(&f) == 0U)
    {
        char hdr[sizeof(k_header) + 64U];
        const int hn = snprintf(hdr, sizeof(hdr), "# SomnaLink fault log, schema %u\r\n%s\r\n",
                                (unsigned)FAULT_LOG_SCHEMA, k_header);
        if (hn > 0) { fr = f_write(&f, hdr, (UINT)hn, &bw); }
    }
    while ((fr == FR_OK) && (s_written != s_count))
    {
        char line[320];
        FaultRec_t r;
        const uint32_t primask = __get_PRIMASK();
        __disable_irq();
        r = s_ring[s_written % FAULT_LOG_RING_N];
        __set_PRIMASK(primask);
        const int n = formatRecord(&r, line, sizeof(line), true);
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
    s_error = ((fr == FR_OK) && (cr == FR_OK)) ? 0U
            : (0x8200U | ((uint32_t)((fr != FR_OK) ? fr : cr) & 0xFFU));
}

void FaultLog_Service(void)
{
    /* Drive faults: bits newly set in the CM4 occurred-fault word (it stays
     * latched until the next start acknowledges it, so polling is enough). */
    const uint16_t occ = g_blower_status.occurred_faults;
    const uint16_t fresh = (uint16_t)(occ & (uint16_t)~s_prev_occ);
    s_prev_occ = occ;
    if (fresh != 0U)
    {
        char name[32];
        driveFaultNames(fresh, name, sizeof(name));
        record((uint8_t)FAULT_SRC_DRIVE, fresh, name, "CM4 occurred-fault bits");
    }
    writePending();
}

uint32_t FaultLog_GetCount(void)   { return s_count; }
uint32_t FaultLog_GetPending(void) { return s_count - s_written; }
uint32_t FaultLog_GetError(void)   { return s_error; }

int FaultLog_FormatRecent(uint32_t k, char *buf, unsigned len)
{
    if ((buf == NULL) || (k >= s_count) || (k >= FAULT_LOG_RING_N))
    {
        return 0;
    }
    const FaultRec_t r = s_ring[(s_count - 1U - k) % FAULT_LOG_RING_N];
    return formatRecord(&r, buf, len, false);
}
