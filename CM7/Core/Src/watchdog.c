/**
  ******************************************************************************
  * @file    watchdog.c
  * @brief   CM7 hardware watchdog (IWDG1) with per-task check-in (see watchdog.h).
  *
  *  Direct register access (RM0399 IWDG / RCC_RSR) rather than the HAL IWDG
  *  driver, so stm32h7xx_hal_conf.h - which CubeMX regenerates - stays as it
  *  is. Backup registers: boot trace owns BKP18R / BKP19R, the RTC BKP0R,
  *  this module BKP15R..BKP17R.
  ******************************************************************************
  */

#include "watchdog.h"
#include "fault_log.h"
#include "rtc_clock.h"
#include "dbg_log.h"
#include "main.h"

#include <stdio.h>

/* IWDG1 counts LSI / 32 (~1 kHz): 1000 counts = ~1.0 s, 0.94..1.07 s over
 * the LSI range (30..34 kHz). */
#define WDG_IWDG_PR           3U
#define WDG_IWDG_RLR          1000U
#define WDG_TIMEOUT_MS        1000U
#define WDG_SR_SPINS          100000U
#define WDG_STARTUP_GRACE_MS  20000U
#define WDG_TEST_DELAY_MS     300U
#define WDG_RESET_LOG_WAIT_MS 10000U

/* RCC_C1_RSR flags that mean the last reset was not a normal power-up,
 * reset button or software reset. */
#define WDG_RSR_ABNORMAL  (RCC_RSR_IWDG1RSTF | RCC_RSR_IWDG2RSTF | RCC_RSR_WWDG1RSTF \
                           | RCC_RSR_WWDG2RSTF | RCC_RSR_LPWR1RSTF | RCC_RSR_LPWR2RSTF)

static const struct
{
    const char *name;
    uint32_t    deadline_ms;   /* 0: refreshes IWDG1 itself (hardware only) */
} k_task[WDG_TASK_N] = {
    [WDG_TASK_SAFETY]   = { "safety",   0U     },
    [WDG_TASK_SENSOR]   = { "sensors",  500U   },   /* 4 SPI xfers x 100 ms HAL timeout */
    [WDG_TASK_SYSTEM]   = { "system",   1000U  },
    [WDG_TASK_GUI]      = { "gui",      2000U  },
    [WDG_TASK_CLIMATE]  = { "climate",  3000U  },
    [WDG_TASK_NFC]      = { "nfc",      5000U  },
    [WDG_TASK_SELFTEST] = { "selftest", 60000U },
};

static volatile uint32_t s_last[WDG_TASK_N];
static volatile uint32_t s_max_gap[WDG_TASK_N];
static volatile uint8_t  s_seen[WDG_TASK_N];
static uint32_t          s_start_ms = 0U;
static volatile bool     s_running = false;
static volatile int      s_tripped = -1;

static uint32_t          s_boot_rsr = 0U;
static uint32_t          s_boot_note = 0U;
static uint32_t          s_boot_a = 0U;
static uint32_t          s_boot_b = 0U;

static bool              s_reset_log_pending = false;

static volatile WdgTest_t s_test = WDG_TEST_NONE;
static volatile uint32_t  s_test_at_ms = 0U;

void Watchdog_CaptureResetCause(void)
{
    __HAL_RCC_RTC_CLK_ENABLE();
    HAL_PWR_EnableBkUpAccess();
    s_boot_rsr = RCC_C1->RSR;
    s_boot_note = RTC->BKP17R;
    s_boot_a = RTC->BKP16R;
    s_boot_b = RTC->BKP15R;
    RTC->BKP17R = 0U;
    RTC->BKP16R = 0U;
    RTC->BKP15R = 0U;
    RCC_C1->RSR |= RCC_RSR_RMVF;
}

const char* Watchdog_GetResetCauseName(void)
{
    const uint32_t f = s_boot_rsr;
    if ((f & RCC_RSR_IWDG1RSTF) != 0U)  { return "watchdog"; }
    if ((f & RCC_RSR_IWDG2RSTF) != 0U)  { return "cm4-watchdog"; }
    if ((f & (RCC_RSR_WWDG1RSTF | RCC_RSR_WWDG2RSTF)) != 0U) { return "window-watchdog"; }
    if ((f & (RCC_RSR_LPWR1RSTF | RCC_RSR_LPWR2RSTF)) != 0U) { return "low-power"; }
    if ((f & (RCC_RSR_SFT1RSTF | RCC_RSR_SFT2RSTF)) != 0U)   { return "software"; }
    if ((f & RCC_RSR_PORRSTF) != 0U)    { return "power-on"; }
    if ((f & RCC_RSR_BORRSTF) != 0U)    { return "brown-out"; }
    if ((f & RCC_RSR_PINRSTF) != 0U)    { return "reset-pin"; }
    return "?";
}

/* The note left before the reset, as text for the fault record; "-" after
 * a normal power-up, reset button or software reset. */
int Watchdog_FormatResetNote(char *buf, unsigned len)
{
    int n;
    const uint32_t kind = s_boot_note & 0x00FF0000UL;
    const uint32_t detail = s_boot_note & 0xFFFFUL;
    if ((s_boot_rsr & WDG_RSR_ABNORMAL) == 0U)
    {
        n = snprintf(buf, len, "-");
    }
    else if ((s_boot_note & 0xFF000000UL) != WDG_NOTE_MAGIC)
    {
        n = snprintf(buf, len, "no note - CPU or interrupt lock-up");
    }
    else if (kind == WDG_NOTE_STALL)
    {
        n = snprintf(buf, len, "%s task stalled %lu ms",
                     (detail < (uint32_t)WDG_TASK_N) ? k_task[detail].name : "?",
                     (unsigned long)s_boot_a);
    }
    else if (kind == WDG_NOTE_FAULT)
    {
        const char *exc = (detail == 'H') ? "HardFault" : (detail == 'M') ? "MemManage"
                        : (detail == 'B') ? "BusFault"  : (detail == 'U') ? "UsageFault" : "Fault";
        n = snprintf(buf, len, "%s pc=0x%08lX cfsr=0x%08lX", exc,
                     (unsigned long)s_boot_a, (unsigned long)s_boot_b);
    }
    else if (kind == WDG_NOTE_ERROR)
    {
        n = snprintf(buf, len, "Error_Handler from 0x%08lX", (unsigned long)s_boot_a);
    }
    else
    {
        n = snprintf(buf, len, "note 0x%08lX", (unsigned long)s_boot_note);
    }
    return ((n > 0) && ((unsigned)n < len)) ? n : 0;
}

static void logLastReset(void)
{
    char why[48];
    if (Watchdog_FormatResetNote(why, sizeof(why)) == 0)
    {
        why[0] = '\0';
    }
    FaultLog_RecordSystem((uint16_t)(s_boot_rsr >> 16U), Watchdog_GetResetCauseName(), why);
    DBG_E("WDG", "last reset: %s (rsr=0x%08lX) %s", Watchdog_GetResetCauseName(),
          (unsigned long)s_boot_rsr, why);
}

void Watchdog_Start(void)
{
    if (s_running)
    {
        return;
    }

    /* #86 fault log: the last reset, if it was abnormal - from
     * Watchdog_Service() once the RTC is up, so the record is timestamped. */
    s_reset_log_pending = ((s_boot_rsr & WDG_RSR_ABNORMAL) != 0U);

    /* Freeze IWDG1 while the debugger halts CM7. */
    DBGMCU->APB4FZ1 |= DBGMCU_APB4FZ1_DBG_IWDG1;

    IWDG1->KR = 0xCCCCU;                     /* start (forces the LSI on) */
    IWDG1->KR = 0x5555U;                     /* unlock PR / RLR */
    IWDG1->PR = WDG_IWDG_PR;
    IWDG1->RLR = WDG_IWDG_RLR;
    for (uint32_t i = 0U; (i < WDG_SR_SPINS) && (IWDG1->SR != 0U); i++)
    {
    }
    IWDG1->KR = 0xAAAAU;                     /* reload, relocks */

    s_start_ms = HAL_GetTick();
    s_running = true;
    DBG_I("WDG", "IWDG1 started, timeout ~%lu ms", (unsigned long)WDG_TIMEOUT_MS);
}

void Watchdog_Kick(WdgTask_t t)
{
    if ((uint32_t)t >= (uint32_t)WDG_TASK_N)
    {
        return;
    }
    const uint32_t now = HAL_GetTick();
    if (s_seen[t] != 0U)
    {
        const uint32_t gap = now - s_last[t];
        if (gap > s_max_gap[t])
        {
            s_max_gap[t] = gap;
        }
    }
    s_last[t] = now;
    s_seen[t] = 1U;
}

void Watchdog_Service(void)
{
    if (!s_running)
    {
        return;
    }
    Watchdog_Kick(WDG_TASK_SAFETY);

    /* RtcClock_Service() brings the calendar back within a few ms of boot
     * after a reset; without a working RTC, log it undated after 10 s. */
    if (s_reset_log_pending
        && (RtcClock_IsRunning() || ((HAL_GetTick() - s_start_ms) > WDG_RESET_LOG_WAIT_MS)))
    {
        s_reset_log_pending = false;
        logLastReset();
    }

    if (s_tripped < 0)
    {
        const uint32_t now = HAL_GetTick();
        for (uint32_t t = 0U; t < (uint32_t)WDG_TASK_N; t++)
        {
            if (k_task[t].deadline_ms == 0U)
            {
                continue;
            }
            const uint32_t age = (s_seen[t] != 0U) ? (now - s_last[t]) : (now - s_start_ms);
            const uint32_t limit = (s_seen[t] != 0U) ? k_task[t].deadline_ms : WDG_STARTUP_GRACE_MS;
            if (age > limit)
            {
                /* Latch: no more refreshes, IWDG1 resets within ~1 s. */
                s_tripped = (int)t;
                Watchdog_Note(WDG_NOTE_STALL, t, age, 0U);
                DBG_E("WDG", "%s task stalled %lu ms - reset", k_task[t].name,
                      (unsigned long)age);
                break;
            }
        }
    }

    if (s_tripped < 0)
    {
        IWDG1->KR = 0xAAAAU;
    }
}

void Watchdog_RequestTest(WdgTest_t t)
{
    s_test_at_ms = HAL_GetTick() + WDG_TEST_DELAY_MS;
    s_test = t;
}

void Watchdog_TestPoll(void)
{
    const WdgTest_t t = s_test;
    if ((t == WDG_TEST_NONE) || ((int32_t)(HAL_GetTick() - s_test_at_ms) < 0))
    {
        return;
    }
    s_test = WDG_TEST_NONE;
    if (t == WDG_TEST_STALL)
    {
        for (;;)
        {
            __NOP();                          /* SystemTask stops checking in */
        }
    }
    else if (t == WDG_TEST_HANG)
    {
        __disable_irq();
        for (;;)
        {
            __NOP();                          /* SafetyTask cannot run */
        }
    }
    else if (t == WDG_TEST_FAULT)
    {
        __builtin_trap();                     /* UDF -> UsageFault handler loop */
    }
    else
    {
        /* unknown: ignore */
    }
}

bool        Watchdog_IsRunning(void)      { return s_running; }
uint32_t    Watchdog_GetTimeoutMs(void)   { return WDG_TIMEOUT_MS; }
int         Watchdog_GetTripped(void)     { return s_tripped; }
uint32_t    Watchdog_GetResetFlags(void)  { return s_boot_rsr; }

const char* Watchdog_GetTaskName(WdgTask_t t)
{
    return ((uint32_t)t < (uint32_t)WDG_TASK_N) ? k_task[t].name : "?";
}

uint32_t Watchdog_GetTaskDeadlineMs(WdgTask_t t)
{
    return ((uint32_t)t < (uint32_t)WDG_TASK_N) ? k_task[t].deadline_ms : 0U;
}

uint32_t Watchdog_GetTaskMaxGapMs(WdgTask_t t)
{
    return ((uint32_t)t < (uint32_t)WDG_TASK_N) ? s_max_gap[t] : 0U;
}

uint32_t Watchdog_GetTaskAgeMs(WdgTask_t t)
{
    if (((uint32_t)t >= (uint32_t)WDG_TASK_N) || (s_seen[t] == 0U))
    {
        return 0xFFFFFFFFUL;
    }
    return HAL_GetTick() - s_last[t];
}

void Watchdog_ClearStats(void)
{
    for (uint32_t t = 0U; t < (uint32_t)WDG_TASK_N; t++)
    {
        s_max_gap[t] = 0U;
    }
}
