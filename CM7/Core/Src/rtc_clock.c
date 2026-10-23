/**
  ******************************************************************************
  * @file    rtc_clock.c
  * @brief   RTC calendar bring-up, read and set (see rtc_clock.h).
  *
  *  Direct register access (RM0399 RTC / RCC_BDCR) rather than the HAL RTC
  *  driver, so stm32h7xx_hal_conf.h - which CubeMX regenerates - stays as it
  *  is. The backup-domain write access (PWR_CR1.DBP) and the RTC APB clock
  *  are already enabled at boot by BootMarkerWrite(); they are set again here
  *  so this module does not depend on that.
  *
  *  Boot trace owns RTC->BKP18R / BKP19R. This module uses BKP0R only.
  ******************************************************************************
  */

#include "rtc_clock.h"
#include "main.h"

#define RTC_VALID_MAGIC     0x52544331UL      /* "RTC1" in BKP0R */
#define RTC_LSE_WAIT_MS     4000U             /* LSE start-up: tSU ~2 s typ. */
#define RTC_LSI_WAIT_MS     50U
#define RTC_INITF_SPINS     200000U           /* INITF: <= 2 RTCCLK, ~61 us */

/* ck_spre = RTCCLK / ((PREDIV_A + 1) x (PREDIV_S + 1)) = 1 Hz. */
#define RTC_PREDIV_A        127U
#define RTC_PREDIV_S_LSE    255U              /* 32768 / 128 / 256 */
#define RTC_PREDIV_S_LSI    249U              /* 32000 / 128 / 250 */

/* Calendar start when the backup domain was lost: matches the EDF logger's
 * old fixed default, 2026-01-01 00:00:00. */
static const RtcDateTime_t k_default = { 26U, 1U, 1U, 0U, 0U, 0U };

typedef enum
{
    RTC_ST_OFF = 0,
    RTC_ST_WAIT_LSE,
    RTC_ST_WAIT_LSI,
    RTC_ST_RUNNING,
    RTC_ST_FAULT
} RtcState_t;

static volatile RtcState_t s_state = RTC_ST_OFF;
static uint32_t s_wait_t0 = 0U;
static volatile bool s_pending = false;
static RtcDateTime_t s_pending_dt;

static uint8_t toBcd(uint8_t v)  { return (uint8_t)(((v / 10U) << 4) | (v % 10U)); }
static uint8_t fromBcd(uint32_t v) { return (uint8_t)(((v >> 4) & 0xFU) * 10U + (v & 0xFU)); }

static bool leapYear(uint8_t yy) { return ((yy % 4U) == 0U); }   /* 2000..2099 */

static uint8_t daysInMonth(uint8_t yy, uint8_t mo)
{
    static const uint8_t k[12] = { 31U, 28U, 31U, 30U, 31U, 30U,
                                   31U, 31U, 30U, 31U, 30U, 31U };
    if ((mo == 2U) && leapYear(yy)) { return 29U; }
    return k[mo - 1U];
}

static bool validDateTime(const RtcDateTime_t *dt)
{
    return (dt != NULL) && (dt->yy <= 99U)
        && (dt->mo >= 1U) && (dt->mo <= 12U)
        && (dt->dd >= 1U) && (dt->dd <= daysInMonth(dt->yy, dt->mo))
        && (dt->hh <= 23U) && (dt->mi <= 59U) && (dt->ss <= 59U);
}

/* RTC weekday: 1 = Monday .. 7 = Sunday (Sakamoto). */
static uint8_t weekday(const RtcDateTime_t *dt)
{
    static const uint8_t t[12] = { 0U, 3U, 2U, 5U, 0U, 3U, 5U, 1U, 4U, 6U, 2U, 4U };
    uint32_t y = 2000U + dt->yy;
    if (dt->mo < 3U) { y--; }
    const uint32_t w = (y + y / 4U - y / 100U + y / 400U + t[dt->mo - 1U] + dt->dd) % 7U;
    return (w == 0U) ? 7U : (uint8_t)w;    /* 0 = Sunday */
}

static void backupAccess(void)
{
    __HAL_RCC_RTC_CLK_ENABLE();            /* RTC APB register clock */
    HAL_PWR_EnableBkUpAccess();
}

static uint32_t rtcSel(void)
{
    return (RCC->BDCR & RCC_BDCR_RTCSEL_Msk);
}

static bool enterInit(void)
{
    RTC->WPR = 0xCAU;
    RTC->WPR = 0x53U;
    RTC->ISR = 0xFFFFFFFFU;                /* INIT = 1; rc_w0 flags untouched */
    for (uint32_t i = 0U; i < RTC_INITF_SPINS; i++)
    {
        if ((RTC->ISR & RTC_ISR_INITF) != 0U)
        {
            return true;
        }
    }
    RTC->ISR &= ~RTC_ISR_INIT;
    RTC->WPR = 0xFFU;
    return false;
}

static void exitInit(void)
{
    RTC->ISR &= ~RTC_ISR_INIT;
    RTC->WPR = 0xFFU;
}

static void writeCalendar(const RtcDateTime_t *dt)
{
    RTC->TR = ((uint32_t)toBcd(dt->hh) << 16)     /* HT:HU, 24 h (PM = 0) */
            | ((uint32_t)toBcd(dt->mi) << 8)      /* MNT:MNU */
            | ((uint32_t)toBcd(dt->ss));          /* ST:SU */
    RTC->DR = ((uint32_t)toBcd(dt->yy) << 16)     /* YT:YU */
            | ((uint32_t)weekday(dt) << RTC_DR_WDU_Pos)
            | ((uint32_t)toBcd(dt->mo) << 8)      /* MT:MU */
            | ((uint32_t)toBcd(dt->dd));          /* DT:DU */
}

/* Prescalers, 24 h format, direct (unshadowed) reads, and a calendar. */
static bool startCalendar(uint32_t prediv_s, const RtcDateTime_t *dt)
{
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const bool ok = enterInit();
    if (ok)
    {
        RTC->PRER = prediv_s;
        RTC->PRER |= (RTC_PREDIV_A << RTC_PRER_PREDIV_A_Pos);
        RTC->CR = (RTC->CR & ~RTC_CR_FMT) | RTC_CR_BYPSHAD;
        writeCalendar(dt);
        exitInit();
    }
    __set_PRIMASK(primask);
    return ok;
}

/* Clock selected and enabled: start the calendar if the backup domain was
 * lost, otherwise keep counting from where it is. */
static void becomeRunning(uint32_t prediv_s)
{
    RCC->BDCR |= RCC_BDCR_RTCEN;
    if ((RTC->ISR & RTC_ISR_INITS) == 0U)
    {
        RTC->BKP0R = 0U;                   /* time not set yet */
        if (!startCalendar(prediv_s, &k_default))
        {
            s_state = RTC_ST_FAULT;
            return;
        }
    }
    else if ((RTC->CR & RTC_CR_BYPSHAD) == 0U)
    {
        RTC->WPR = 0xCAU;
        RTC->WPR = 0x53U;
        RTC->CR |= RTC_CR_BYPSHAD;
        RTC->WPR = 0xFFU;
    }
    s_state = RTC_ST_RUNNING;
}

void RtcClock_Init(void)
{
    backupAccess();
    s_wait_t0 = HAL_GetTick();
    switch (rtcSel())
    {
    case RCC_BDCR_RTCSEL_0:                /* LSE kept in the backup domain */
        s_state = RTC_ST_WAIT_LSE;
        break;
    case RCC_BDCR_RTCSEL_1:                /* LSI is reset with the core */
        RCC->CSR |= RCC_CSR_LSION;
        s_state = RTC_ST_WAIT_LSI;
        break;
    case 0U:                               /* backup domain was lost */
        if ((RCC->BDCR & RCC_BDCR_LSEON) == 0U)
        {
            /* LSEDRV may only change with the LSE off: medium-high drive,
             * since the crystal on this board is not characterised. */
            RCC->BDCR = (RCC->BDCR & ~RCC_BDCR_LSEDRV_Msk) | RCC_BDCR_LSEDRV_1;
            RCC->BDCR |= RCC_BDCR_LSEON;
        }
        s_state = RTC_ST_WAIT_LSE;
        break;
    default:                               /* HSE/RTCPRE: not ours, keep it */
        s_state = RTC_ST_RUNNING;
        break;
    }
}

void RtcClock_Service(void)
{
    const uint32_t now = HAL_GetTick();
    switch (s_state)
    {
    case RTC_ST_WAIT_LSE:
        if ((RCC->BDCR & RCC_BDCR_LSERDY) != 0U)
        {
            if (rtcSel() == 0U)
            {
                RCC->BDCR |= RCC_BDCR_RTCSEL_0;
            }
            becomeRunning(RTC_PREDIV_S_LSE);
        }
        else if ((now - s_wait_t0) >= RTC_LSE_WAIT_MS)
        {
            if (rtcSel() == RCC_BDCR_RTCSEL_0)
            {
                /* Selected on an earlier boot but not oscillating now; the
                 * source cannot change without a backup-domain reset, which
                 * would also wipe the boot trace - leave it to a power cycle. */
                s_state = RTC_ST_FAULT;
                break;
            }
            RCC->BDCR &= ~RCC_BDCR_LSEON;  /* no crystal: use the LSI */
            RCC->CSR |= RCC_CSR_LSION;
            s_wait_t0 = now;
            s_state = RTC_ST_WAIT_LSI;
        }
        break;

    case RTC_ST_WAIT_LSI:
        if ((RCC->CSR & RCC_CSR_LSIRDY) != 0U)
        {
            if (rtcSel() == 0U)
            {
                RCC->BDCR |= RCC_BDCR_RTCSEL_1;
            }
            becomeRunning(RTC_PREDIV_S_LSI);
        }
        else if ((now - s_wait_t0) >= RTC_LSI_WAIT_MS)
        {
            s_state = RTC_ST_FAULT;
        }
        break;

    case RTC_ST_RUNNING:
        if (s_pending)
        {
            RtcDateTime_t dt = s_pending_dt;
            s_pending = false;
            (void)RtcClock_Set(&dt);
        }
        break;

    default:
        break;
    }
}

bool RtcClock_IsRunning(void)
{
    return (s_state == RTC_ST_RUNNING);
}

bool RtcClock_IsTimeValid(void)
{
    return RtcClock_IsRunning() && (RTC->BKP0R == RTC_VALID_MAGIC);
}

RtcSource_t RtcClock_GetSource(void)
{
    switch (rtcSel())
    {
    case RCC_BDCR_RTCSEL_0: return RTC_SRC_LSE;
    case RCC_BDCR_RTCSEL_1: return RTC_SRC_LSI;
    case 0U:                return RTC_SRC_NONE;
    default:                return RTC_SRC_OTHER;
    }
}

const char* RtcClock_GetSourceName(void)
{
    if (s_state == RTC_ST_FAULT) { return "FAULT"; }
    switch (RtcClock_GetSource())
    {
    case RTC_SRC_LSE:   return "LSE";
    case RTC_SRC_LSI:   return "LSI";
    case RTC_SRC_OTHER: return "OTHER";
    default:            return "NONE";
    }
}

bool RtcClock_Get(RtcDateTime_t *dt)
{
    if ((dt == NULL) || !RtcClock_IsRunning())
    {
        return false;
    }
    /* BYPSHAD: TR and DR are read straight from the counters, so read twice
     * and retry if a second (or midnight) rolled over in between. */
    uint32_t tr = 0U;
    uint32_t dr = 0U;
    for (uint32_t i = 0U; i < 4U; i++)
    {
        tr = RTC->TR;
        dr = RTC->DR;
        if ((RTC->TR == tr) && (RTC->DR == dr))
        {
            break;
        }
    }
    dt->hh = fromBcd((tr >> 16) & 0x3FU);
    dt->mi = fromBcd((tr >> 8) & 0x7FU);
    dt->ss = fromBcd(tr & 0x7FU);
    dt->yy = fromBcd((dr >> 16) & 0xFFU);
    dt->mo = fromBcd((dr >> 8) & 0x1FU);
    dt->dd = fromBcd(dr & 0x3FU);
    return true;
}

bool RtcClock_Set(const RtcDateTime_t *dt)
{
    if (!validDateTime(dt))
    {
        return false;
    }
    if (!RtcClock_IsRunning())
    {
        s_pending_dt = *dt;
        s_pending = true;
        return true;
    }
    const uint32_t primask = __get_PRIMASK();
    __disable_irq();
    const bool ok = enterInit();
    if (ok)
    {
        writeCalendar(dt);
        exitInit();
        RTC->BKP0R = RTC_VALID_MAGIC;
    }
    __set_PRIMASK(primask);
    return ok;
}
