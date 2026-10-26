/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "cm7_cpu_config.h"
#include "app_touchgfx.h"
#include "usb_device.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>
#include "lcd_test.h"
#include "lcd_panel.h"
#include "lcd_drv.h"
#include "boot_trace.h"
#include "stm32h7xx_hal_rcc_ex.h"
#include "pressure_sensors.h"
#include "blower_ipc.h"
#include "nfc_test.h"
#include "sensirion_th.h"
#include "dbg_log.h"
#include "telemetry_stream.h"
#include "esp_bridge.h"
#include "humidifier.h"
#include "rotary_input.h"
#include "blower_ctrl.h"
#include "flow_sensor.h"
#include "tube_comp.h"
#include "bench_playback.h"
#include "leak_estimator.h"
#include "therapy.h"
#include "session_edf_log.h"
#include "rtc_clock.h"
#include "session_summary.h"
#include "adherence.h"
#include "fault_log.h"
#include "watchdog.h"
#include "physiology_telem.h"
#include "safety_supervisor.h"
/* USER CODE END Includes */

#define MAIN_MBAR_PER_CMH2O   0.980665f
#define MAIN_MBAR_TO_CMH2O(x) ((x) * (1.0f / MAIN_MBAR_PER_CMH2O))

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SDRAM_DEVICE_ADDR           ((uint32_t)0xC0000000U)
#define SDRAM_DEVICE_SIZE           (32U * 1024U * 1024U)
#define SDRAM_TIMEOUT                 ((uint32_t)0xFFFFU)
#define SDRAM_REFRESH_COUNT           ((uint32_t)(((64U * 1000U * 100U) / 8192U) - 20U))
#define SDRAM_MODEREG_BURST_LENGTH_1  ((uint16_t)0x0000U)
#define SDRAM_MODEREG_BURST_TYPE_SEQ  ((uint16_t)0x0000U)
#define SDRAM_MODEREG_CAS_LATENCY_3   ((uint16_t)0x0030U)
#define SDRAM_MODEREG_OPERATING_STD   ((uint16_t)0x0000U)
#define SDRAM_MODEREG_WRITEBURST_SINGLE ((uint16_t)0x0200U)
#define SDRAM_TEST_SCRATCH_ADDR       (SDRAM_DEVICE_ADDR + 0x00200000U)

/* DUAL_CORE_BOOT_SYNC_SEQUENCE: Define for dual core boot synchronization    */
/*                             demonstration code based on hardware semaphore */
/* This define is present in both CM7/CM4 projects                            */
/* To comment when developping/debugging on a single core                     */
/* CM7: always-on dual-core prep in main (not #ifdef) — CM4 must reach STOP before RCC. */
/* #define DUAL_CORE_BOOT_SYNC_SEQUENCE */

#if defined(DUAL_CORE_BOOT_SYNC_SEQUENCE)
#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U) /* HW semaphore 0*/
#endif
#endif /* DUAL_CORE_BOOT_SYNC_SEQUENCE */

#ifndef HSEM_ID_0
#define HSEM_ID_0 (0U)
#endif

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
ADC_HandleTypeDef hadc3;

CRC_HandleTypeDef hcrc;

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c2;

LTDC_HandleTypeDef hltdc;

QSPI_HandleTypeDef hqspi;

SD_HandleTypeDef hsd1;

SPI_HandleTypeDef hspi1;
SPI_HandleTypeDef hspi2;
SPI_HandleTypeDef hspi4;

TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart2;

SDRAM_HandleTypeDef hsdram1;

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
/* USER CODE BEGIN PV */
volatile uint32_t sd_test_step = 0U;
volatile uint32_t sd_test_fail_step = 0U;
volatile HAL_StatusTypeDef sd_test_result = HAL_BUSY;
volatile uint8_t sd_card_present = 0U;
volatile uint8_t sd_fatfs_ready = 0U; /* 1 after self-test OK + settle */
volatile uint32_t sd_hal_error = 0U;
volatile uint32_t sd_sdmmc_ker_clk_hz = 0U;
volatile uint32_t sd_sdmmc_clkcr = 0U;
volatile uint32_t sd_sdmmc_power = 0U;
volatile uint32_t sd_sdmmc_sta = 0U;
volatile uint32_t sd_sdmmc_resp1 = 0U;
volatile uint32_t sd_cmd_idle = 0U;
volatile uint32_t sd_d0_idle = 0U;
volatile uint32_t sd_init_attempt = 0U;
HAL_SD_CardInfoTypeDef sd_card_info;

/* SD dummy-file writer (raw-pattern, off-line PC verification).
 *
 *  After the basic SD self-test passes, SD_WriteDummyFile() drops a 512-byte
 *  ASCII payload at sector SD_DUMMY_SECTOR_NUM. To check the pattern on a
 *  Windows PC: pull the SD card out, open HxD as Administrator -> Tools ->
 *  Open disk -> Physical disk for the SD reader. Press Ctrl+G and enter the
 *  byte offset SD_DUMMY_SECTOR_OFFSET. The header text (incl. boot count and
 *  card capacity) should appear verbatim. */
#define SD_DUMMY_SECTOR_NUM    0x10000U                            /* sector index    */
#define SD_DUMMY_SECTOR_OFFSET (SD_DUMMY_SECTOR_NUM * 512U)         /* = 0x02000000     */

volatile uint32_t sd_dummy_step       = 0U;     /* 0 idle, 1..4 in flight, 100 done */
volatile uint32_t sd_dummy_status     = 0U;     /* 0 ok, !=0 last failure code        */
volatile uint32_t sd_dummy_write_count = 0U;    /* incremented per successful write   */

__attribute__((aligned(32))) static uint8_t sd_tx_buf[512];
__attribute__((aligned(32))) static uint8_t sd_rx_buf[512];

volatile uint32_t qspi_test_step = 0U;
volatile HAL_StatusTypeDef qspi_test_result = HAL_BUSY;
volatile uint8_t qspi_jedec_id[3] = {0U, 0U, 0U};
volatile uint32_t qspi_hal_error = 0U;
volatile uint32_t qspi_read_substep = 0U;

__attribute__((aligned(32))) static uint8_t qspi_tx_buf[256];
__attribute__((aligned(32))) static uint8_t qspi_rx_buf[256];

volatile uint32_t sdram_test_step = 0U;
volatile HAL_StatusTypeDef sdram_test_result = HAL_BUSY;
volatile uint32_t sdram_test_fail_index = 0U;
volatile uint32_t sdram_test_fail_expected = 0U;
volatile uint32_t sdram_test_fail_actual = 0U;
volatile uint32_t clock_config_step = 0U;
extern volatile uint32_t boot_sync_step;
volatile uint32_t ltdc_cfbbar = 0U;
volatile uint32_t ltdc_gcr = 0U;
BlowerIpcStatus_t g_blower_status;

/* ---------------------------------------------------------------------------
 * FreeRTOS application tasks.
 *
 * Feature split (period / priority chosen by CPU profile of each job):
 *  - GuiTask      : TouchGFX render loop. Blocks on LTDC VSYNC (~60 Hz), so
 *                   it consumes CPU only while drawing. Polls rotary input
 *                   once per frame before rendering.
 *  - SensorTask   : Fast sensor acquisition (3x AMS5935 SPI + ADC), leak,
 *                   outer pressure PID (200 Hz), and Therapy_Update.
 *                   osPriorityAboveNormal — below SafetyTask.
 *  - SafetyTask   : Independent safety_supervisor @ 200 Hz (pressure faults,
 *                   CM4 IPC heartbeat). osPriorityRealtime — preempts all
 *                   other CM7 application tasks.
 *  - NfcTask      : RFAL discovery worker + secure tag verify. Needs regular
 *                   servicing (10 ms) but each call is short.
 *  - ClimateTask  : SHT4x/STS4x I2C reads (~10 ms blocking each) + 1 Hz
 *                   humidifier PI loop. Slow plant -> 250 ms period at low
 *                   priority.
 *  - SystemTask   : Blower start/stop/speed edge commands from the debugger
 *                   globals, fault/state edge logging, USB CDC log pump.
 *  - SelfTestTask : SD card detect/self-test state machine + deferred
 *                   SDRAM/QSPI one-shot tests. Lowest priority: QSPI erase
 *                   can block for seconds and must not disturb anything.
 * ------------------------------------------------------------------------ */
static osThreadId_t guiTaskHandle;
static const osThreadAttr_t guiTask_attributes = {
  .name = "gui",
  .stack_size = 4096,
  .priority = (osPriority_t) osPriorityNormal,
};
static osThreadId_t sensorTaskHandle;
static const osThreadAttr_t sensorTask_attributes = {
  .name = "sensors",
  .stack_size = 3072,
  .priority = (osPriority_t) osPriorityAboveNormal,
};
static osThreadId_t safetyTaskHandle;
static const osThreadAttr_t safetyTask_attributes = {
  .name = "safety",
  .stack_size = 1024,
  .priority = (osPriority_t) osPriorityRealtime,
};
static osThreadId_t nfcTaskHandle;
static const osThreadAttr_t nfcTask_attributes = {
  .name = "nfc",
  .stack_size = 2048,
  .priority = (osPriority_t) osPriorityNormal,
};
static osThreadId_t climateTaskHandle;
static const osThreadAttr_t climateTask_attributes = {
  .name = "climate",
  .stack_size = 1536,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
static osThreadId_t systemTaskHandle;
static const osThreadAttr_t systemTask_attributes = {
  .name = "system",
  .stack_size = 4096,
  .priority = (osPriority_t) osPriorityBelowNormal,
};
static osThreadId_t selfTestTaskHandle;
static const osThreadAttr_t selfTestTask_attributes = {
  .name = "selftest",
  .stack_size = 4096,
  .priority = (osPriority_t) osPriorityLow,
};
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
void PeriphCommonClock_Config(void);
static void MPU_Config(void);
static void MX_DMA_Init(void);
static void MX_GPIO_Init(void);
static void MX_FMC_Init(void);
static void MX_QUADSPI_Init(void);
static void MX_LTDC_Init(void);
static void MX_CRC_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC3_Init(void);
static void MX_SPI2_Init(void);
static void MX_SPI4_Init(void);
static void MX_TIM4_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C2_Init(void);
static void MX_SPI1_Init(void);
static void MX_SDMMC1_SD_Init(void);
static void MX_USART2_UART_Init(void);
void StartDefaultTask(void *argument);

/* USER CODE BEGIN PFP */
static int SD_CardPresent(void);
static HAL_StatusTypeDef SD_WaitReady(void);
static HAL_StatusTypeDef SD_RunBasicTest(void);
static HAL_StatusTypeDef SD_WriteDummyFile(void);
static void QSPI_PrepareCommand(QSPI_CommandTypeDef *cmd);
static HAL_StatusTypeDef QSPI_ResetMemory(void);
static HAL_StatusTypeDef QSPI_AutoPollReady(void);
static HAL_StatusTypeDef QSPI_WriteEnable(void);
static HAL_StatusTypeDef QSPI_ReadJedecId(uint8_t id[3]);
static HAL_StatusTypeDef QSPI_EraseSubsector(uint32_t addr);
static HAL_StatusTypeDef QSPI_PageProgram(uint32_t addr, uint8_t *data, uint32_t len);
static HAL_StatusTypeDef QSPI_ReadData(uint32_t addr, uint8_t *data, uint32_t len);
static int QSPI_IsKnownFlashId(const uint8_t id[3]);
static HAL_StatusTypeDef QSPI_RunBasicTest(void);
static HAL_StatusTypeDef SDRAM_InitializationSequence(SDRAM_HandleTypeDef *hsdram);
static HAL_StatusTypeDef SDRAM_RunBasicTest(void);
static void CM7_WakeCm4(void);
static void CM7_PrepareDualCoreBoot(void);
static void BootMarkerWrite(uint32_t marker);
static void LCD_UpdateDebugRegs(void);
static void CM7_EnableFpu(void);
static void GuiTask(void *argument);
static void SensorTask(void *argument);
static void SafetyTask(void *argument);
static void NfcTask(void *argument);
static void ClimateTask(void *argument);
static void SystemTask(void *argument);
static void SelfTestTask(void *argument);
/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void BootMarkerWrite(uint32_t marker)
{
  boot_trace_sync = marker;
  __HAL_RCC_RTC_CLK_ENABLE();
  HAL_PWR_EnableBkUpAccess();
  RTC->BKP19R = marker;
}

static void CM7_WakeCm4(void)
{
  __HAL_RCC_HSEM_CLK_ENABLE();
  (void)HAL_HSEM_FastTake(HSEM_ID_0);
  HAL_HSEM_Release(HSEM_ID_0, 0U);
  boot_sync_step = 6U;
  BootMarkerWrite((BOOT_TRACE_CM7_BASE | 0x0060U) | boot_sync_step);
}

/**
  * On debug connect Keil often leaves CM4 halted at reset while CM7 runs.
  * Kick CM4 boot and wait (bounded) for D2 STOP before changing RCC.
  */
static void CM7_PrepareDualCoreBoot(void)
{
  int32_t timeout;

  boot_sync_step = 1U;
  BootMarkerWrite((BOOT_TRACE_CM7_BASE | 0x0010U) | boot_sync_step);
  __HAL_RCC_HSEM_CLK_ENABLE();
  HAL_RCCEx_EnableBootCore(RCC_BOOT_C2);

  /* Let CM4 reach STOP before CM7 reconfigures RCC (cold boot + debug). */
  timeout = 0x00800000;
  while ((__HAL_RCC_GET_FLAG(RCC_FLAG_D2CKRDY) != RESET) && (timeout-- > 0))
  {
  }

  if (timeout < 0)
  {
    boot_sync_step = 2U;
  }
  else
  {
    boot_sync_step = 3U;
  }
  BootMarkerWrite((BOOT_TRACE_CM7_BASE | 0x0020U) | boot_sync_step);
}

static void LCD_UpdateDebugRegs(void)
{
  ltdc_cfbbar = LTDC_Layer1->CFBAR;
  ltdc_gcr = LTDC->GCR;
}

static void CM7_EnableFpu(void)
{
  CM7_ConfigureCpu();
}

static int SD_CardPresent(void)
{
  /* Board wiring: SDIO_DET high when microSD card is inserted */
  return (HAL_GPIO_ReadPin(SDIO_DET_GPIO_Port, SDIO_DET_Pin) == GPIO_PIN_SET);
}

static HAL_StatusTypeDef SD_WaitReady(void)
{
  uint32_t tickstart = HAL_GetTick();

  while (HAL_SD_GetCardState(&hsd1) != HAL_SD_CARD_TRANSFER)
  {
    if ((HAL_GetTick() - tickstart) > 5000U)
    {
      return HAL_TIMEOUT;
    }
  }

  return HAL_OK;
}

static void SD_PrepareSdmmcHandle(void)
{
  hsd1.Instance = SDMMC1;
  hsd1.Init.ClockEdge = SDMMC_CLOCK_EDGE_RISING;
  hsd1.Init.ClockPowerSave = SDMMC_CLOCK_POWER_SAVE_DISABLE;
  hsd1.Init.BusWide = SDMMC_BUS_WIDE_1B;
  /* Flow control stalls SDMMC_CK when the FIFO cannot keep up. Without it,
   * RTOS/motor ISRs preempting the polling loop caused SDMMC_ERROR_TX_UNDERRUN
   * (0x10) on write blocks, which failed the SD self-test and EDF logging. */
  hsd1.Init.HardwareFlowControl = SDMMC_HARDWARE_FLOW_CONTROL_ENABLE;
  /* Post-init transfer speed; HAL_SD_InitCard() forces <=400 kHz for enumeration. */
  hsd1.Init.ClockDiv = 4U;
}

static void SD_UpdateDebugRegs(void)
{
  sd_sdmmc_ker_clk_hz = HAL_RCCEx_GetPeriphCLKFreq(RCC_PERIPHCLK_SDMMC);
  sd_sdmmc_clkcr = SDMMC1->CLKCR;
  sd_sdmmc_power = SDMMC1->POWER;
  sd_sdmmc_sta = SDMMC1->STA;
  sd_sdmmc_resp1 = SDMMC1->RESP1;
}

static void SD_SampleBusIdle(void)
{
  GPIO_InitTypeDef gpio = {0};

  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  gpio.Mode = GPIO_MODE_INPUT;
  gpio.Pull = GPIO_NOPULL;
  gpio.Pin = GPIO_PIN_8;
  HAL_GPIO_Init(GPIOC, &gpio);

  gpio.Pin = GPIO_PIN_2;
  HAL_GPIO_Init(GPIOD, &gpio);

  sd_d0_idle = (uint32_t)HAL_GPIO_ReadPin(GPIOC, GPIO_PIN_8);
  sd_cmd_idle = (uint32_t)HAL_GPIO_ReadPin(GPIOD, GPIO_PIN_2);
}

static void SD_DelayMs(uint32_t ms)
{
  if (osKernelGetState() == osKernelRunning)
  {
    (void)osDelay(ms);
  }
  else
  {
    HAL_Delay(ms);
  }
}

static HAL_StatusTypeDef SD_TryInit(void)
{
  HAL_StatusTypeDef status;

  SD_PrepareSdmmcHandle();
  if (hsd1.State != HAL_SD_STATE_RESET)
  {
    (void)HAL_SD_DeInit(&hsd1);
    SD_DelayMs(50);
  }

  status = HAL_SD_Init(&hsd1);
  SD_UpdateDebugRegs();
  if (status != HAL_OK)
  {
    sd_hal_error = hsd1.ErrorCode;
    return status;
  }

  if (HAL_SD_ConfigWideBusOperation(&hsd1, SDMMC_BUS_WIDE_4B) != HAL_OK)
  {
    sd_hal_error = hsd1.ErrorCode;
    SD_UpdateDebugRegs();
    return HAL_ERROR;
  }

  SD_UpdateDebugRegs();
  return HAL_OK;
}

static HAL_StatusTypeDef SD_RunBasicTest(void)
{
  sd_test_step = 1U;
  sd_card_present = (uint8_t)SD_CardPresent();

  if (!SD_CardPresent())
  {
    sd_test_fail_step = sd_test_step;
    return HAL_ERROR;
  }

  SD_DelayMs(500);
  SD_SampleBusIdle();

  sd_test_step = 2U;
  sd_init_attempt = 0U;
  SD_UpdateDebugRegs();

  {
    HAL_StatusTypeDef init_status = HAL_ERROR;

    for (sd_init_attempt = 1U; sd_init_attempt <= 3U; sd_init_attempt++)
    {
      init_status = SD_TryInit();
      if (init_status == HAL_OK)
      {
        break;
      }

      sd_test_fail_step = sd_test_step;
      if (sd_init_attempt < 3U)
      {
        SD_DelayMs(100);
      }
    }

    if (init_status != HAL_OK)
    {
      sd_test_fail_step = sd_test_step;
      return HAL_ERROR;
    }
  }

  sd_test_step = 3U;
  if (HAL_SD_GetCardInfo(&hsd1, &sd_card_info) != HAL_OK)
  {
    sd_hal_error = hsd1.ErrorCode;
    sd_test_fail_step = sd_test_step;
    return HAL_ERROR;
  }

  sd_test_step = 4U;
  SCB_CleanInvalidateDCache_by_Addr((uint32_t *)sd_rx_buf, sizeof(sd_rx_buf));
  if (HAL_SD_ReadBlocks(&hsd1, sd_rx_buf, 0U, 1U, 5000U) != HAL_OK)
  {
    sd_hal_error = hsd1.ErrorCode;
    sd_test_fail_step = sd_test_step;
    return HAL_ERROR;
  }
  if (SD_WaitReady() != HAL_OK)
  {
    sd_hal_error = hsd1.ErrorCode;
    sd_test_fail_step = sd_test_step;
    return HAL_TIMEOUT;
  }
  SCB_InvalidateDCache_by_Addr((uint32_t *)sd_rx_buf, sizeof(sd_rx_buf));

  sd_test_step = 5U;
  if (sd_card_info.LogBlockNbr == 0U)
  {
    sd_test_fail_step = sd_test_step;
    return HAL_ERROR;
  }

  for (uint32_t i = 0U; i < 512U; i++)
  {
    sd_tx_buf[i] = (uint8_t)(i ^ 0xA5U);
  }

  {
    uint32_t test_block = sd_card_info.LogBlockNbr - 1U;

    SCB_CleanDCache_by_Addr((uint32_t *)sd_tx_buf, sizeof(sd_tx_buf));
    if (HAL_SD_WriteBlocks(&hsd1, sd_tx_buf, test_block, 1U, 5000U) != HAL_OK)
    {
      sd_hal_error = hsd1.ErrorCode;
      sd_test_fail_step = sd_test_step;
      return HAL_ERROR;
    }
    if (SD_WaitReady() != HAL_OK)
    {
      sd_hal_error = hsd1.ErrorCode;
      sd_test_fail_step = sd_test_step;
      return HAL_TIMEOUT;
    }

    memset(sd_rx_buf, 0, sizeof(sd_rx_buf));
    SCB_CleanInvalidateDCache_by_Addr((uint32_t *)sd_rx_buf, sizeof(sd_rx_buf));

    if (HAL_SD_ReadBlocks(&hsd1, sd_rx_buf, test_block, 1U, 5000U) != HAL_OK)
    {
      sd_hal_error = hsd1.ErrorCode;
      sd_test_fail_step = sd_test_step;
      return HAL_ERROR;
    }
    if (SD_WaitReady() != HAL_OK)
    {
      sd_hal_error = hsd1.ErrorCode;
      sd_test_fail_step = sd_test_step;
      return HAL_TIMEOUT;
    }
    SCB_InvalidateDCache_by_Addr((uint32_t *)sd_rx_buf, sizeof(sd_rx_buf));
  }

  sd_test_step = 6U;
  if (memcmp(sd_tx_buf, sd_rx_buf, 512U) != 0)
  {
    sd_test_fail_step = sd_test_step;
    return HAL_ERROR;
  }

  sd_test_step = 100U;
  sd_test_fail_step = 0U;
  return HAL_OK;
}

/* Drops a recognizable 512-byte ASCII payload at SD_DUMMY_SECTOR_NUM and
 * reads it back to confirm. See the comment block above the SD_DUMMY_*
 * macros for the verification procedure on a Windows PC. */
static HAL_StatusTypeDef SD_WriteDummyFile(void)
{
  if ((sd_card_present == 0U) || (sd_test_result != HAL_OK))
  {
    sd_dummy_status = 0xCAFEFACEU;     /* card / self-test not ready  */
    return HAL_ERROR;
  }

  sd_dummy_step = 1U;
  memset(sd_tx_buf, 0, sizeof(sd_tx_buf));

  /* Build the human-readable payload. snprintf return is intentionally
   * unchecked: we just want as much info as fits in 512 bytes. */
  int n = 0;
  n += snprintf((char *)sd_tx_buf + n, sizeof(sd_tx_buf) - (size_t)n,
                "==== CPAP InitialCode SD dummy file ====\r\n");
  n += snprintf((char *)sd_tx_buf + n, sizeof(sd_tx_buf) - (size_t)n,
                "Sector  : 0x%08lX (decimal %lu)\r\n",
                (unsigned long)SD_DUMMY_SECTOR_NUM,
                (unsigned long)SD_DUMMY_SECTOR_NUM);
  n += snprintf((char *)sd_tx_buf + n, sizeof(sd_tx_buf) - (size_t)n,
                "Offset  : 0x%08lX bytes  (= sector * 512)\r\n",
                (unsigned long)SD_DUMMY_SECTOR_OFFSET);
  n += snprintf((char *)sd_tx_buf + n, sizeof(sd_tx_buf) - (size_t)n,
                "Sysclk  : %lu Hz\r\n",
                (unsigned long)HAL_RCC_GetSysClockFreq());
  n += snprintf((char *)sd_tx_buf + n, sizeof(sd_tx_buf) - (size_t)n,
                "Tick    : %lu ms\r\n",
                (unsigned long)HAL_GetTick());
  n += snprintf((char *)sd_tx_buf + n, sizeof(sd_tx_buf) - (size_t)n,
                "CardSize: %lu blocks (%lu MB)\r\n",
                (unsigned long)sd_card_info.LogBlockNbr,
                (unsigned long)((uint64_t)sd_card_info.LogBlockNbr *
                                (uint64_t)sd_card_info.LogBlockSize /
                                (1024ULL * 1024ULL)));
  n += snprintf((char *)sd_tx_buf + n, sizeof(sd_tx_buf) - (size_t)n,
                "WriteCnt: %lu\r\n",
                (unsigned long)(sd_dummy_write_count + 1U));
  n += snprintf((char *)sd_tx_buf + n, sizeof(sd_tx_buf) - (size_t)n,
                "----------------------------------------\r\n"
                "Hello from STM32H747 CM7!\r\n"
                "Open the SD card with HxD on Windows:\r\n"
                "  Tools -> Open disk -> Physical disk\r\n"
                "  (run HxD as Administrator).\r\n"
                "Then Ctrl+G and enter offset 0x02000000.\r\n"
                "You should see this exact text.\r\n"
                "----------------------------------------\r\n");
  (void)n;

  sd_dummy_step = 2U;
  SCB_CleanDCache_by_Addr((uint32_t *)sd_tx_buf, sizeof(sd_tx_buf));
  if (HAL_SD_WriteBlocks(&hsd1, sd_tx_buf, SD_DUMMY_SECTOR_NUM, 1U, 5000U) != HAL_OK)
  {
    sd_dummy_status = 0xE0000000U | (uint32_t)hsd1.ErrorCode;
    return HAL_ERROR;
  }
  if (SD_WaitReady() != HAL_OK)
  {
    sd_dummy_status = 0xE1000000U | (uint32_t)hsd1.ErrorCode;
    return HAL_TIMEOUT;
  }

  /* Read-back verify so we don't claim success on cards that ACK the write
   * but quietly drop the payload. */
  sd_dummy_step = 3U;
  memset(sd_rx_buf, 0, sizeof(sd_rx_buf));
  SCB_CleanInvalidateDCache_by_Addr((uint32_t *)sd_rx_buf, sizeof(sd_rx_buf));
  if (HAL_SD_ReadBlocks(&hsd1, sd_rx_buf, SD_DUMMY_SECTOR_NUM, 1U, 5000U) != HAL_OK)
  {
    sd_dummy_status = 0xE2000000U | (uint32_t)hsd1.ErrorCode;
    return HAL_ERROR;
  }
  if (SD_WaitReady() != HAL_OK)
  {
    sd_dummy_status = 0xE3000000U | (uint32_t)hsd1.ErrorCode;
    return HAL_TIMEOUT;
  }
  SCB_InvalidateDCache_by_Addr((uint32_t *)sd_rx_buf, sizeof(sd_rx_buf));

  sd_dummy_step = 4U;
  if (memcmp(sd_tx_buf, sd_rx_buf, 512U) != 0)
  {
    sd_dummy_status = 0xE4000000U;
    return HAL_ERROR;
  }

  sd_dummy_step = 100U;
  sd_dummy_status = 0U;
  sd_dummy_write_count++;
  return HAL_OK;
}

#define MT25Q_MANUFACTURER_ID         0x20U
#define MT25Q_MEMORY_TYPE             0xBAU
#define MT25Q_MEMORY_CAPACITY_256MBIT 0x19U

#define MX25L_MANUFACTURER_ID         0xC2U
#define MX25L_MEMORY_TYPE             0x20U
#define MX25L_MEMORY_CAPACITY_256MBIT 0x19U

#define MT25Q_CMD_RESET_ENABLE          0x66U
#define MT25Q_CMD_RESET_MEMORY          0x99U
#define MT25Q_CMD_READ_ID               0x9FU
#define MT25Q_CMD_QUAD_OUT_FAST_READ    0x6BU
#define MT25Q_CMD_WRITE_ENABLE          0x06U
#define MT25Q_CMD_READ_STATUS           0x05U
#define MT25Q_CMD_READ                  0x03U
#define MT25Q_CMD_PAGE_PROGRAM          0x02U
#define MT25Q_CMD_SUBSECTOR_ERASE_4K    0x20U

#define MT25Q_SR_WIP                    0x01U
#define MT25Q_TEST_ADDRESS              0x00100000U
#define MT25Q_PAGE_SIZE                 256U

static void QSPI_PrepareCommand(QSPI_CommandTypeDef *cmd)
{
  cmd->InstructionMode = QSPI_INSTRUCTION_1_LINE;
  cmd->AddressMode = QSPI_ADDRESS_NONE;
  cmd->AlternateByteMode = QSPI_ALTERNATE_BYTES_NONE;
  cmd->Address = 0U;
  cmd->AddressSize = QSPI_ADDRESS_24_BITS;
  cmd->DummyCycles = 0U;
  cmd->Instruction = 0U;
  cmd->NbData = 0U;
  cmd->DataMode = QSPI_DATA_NONE;
  cmd->DdrMode = QSPI_DDR_MODE_DISABLE;
  cmd->DdrHoldHalfCycle = QSPI_DDR_HHC_ANALOG_DELAY;
  cmd->SIOOMode = QSPI_SIOO_INST_EVERY_CMD;
}

static HAL_StatusTypeDef QSPI_ResetMemory(void)
{
  QSPI_CommandTypeDef cmd = {0};

  QSPI_PrepareCommand(&cmd);
  cmd.Instruction = MT25Q_CMD_RESET_ENABLE;
  if (HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    qspi_hal_error = hqspi.ErrorCode;
    return HAL_ERROR;
  }

  QSPI_PrepareCommand(&cmd);
  cmd.Instruction = MT25Q_CMD_RESET_MEMORY;
  if (HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    qspi_hal_error = hqspi.ErrorCode;
    return HAL_ERROR;
  }

  HAL_Delay(1);
  return HAL_OK;
}

static HAL_StatusTypeDef QSPI_AutoPollReady(void)
{
  QSPI_CommandTypeDef cmd = {0};
  uint8_t status_byte = 0U;
  uint32_t tickstart = HAL_GetTick();

  if (hqspi.State != HAL_QSPI_STATE_READY)
  {
    (void)HAL_QSPI_Abort(&hqspi);
  }

  while ((HAL_GetTick() - tickstart) < 30000U)
  {
    QSPI_PrepareCommand(&cmd);
    cmd.Instruction = MT25Q_CMD_READ_STATUS;
    cmd.DataMode = QSPI_DATA_1_LINE;
    cmd.NbData = 1U;

    if (HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      qspi_hal_error = hqspi.ErrorCode;
      return HAL_ERROR;
    }

    if (HAL_QSPI_Receive(&hqspi, &status_byte, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
    {
      qspi_hal_error = hqspi.ErrorCode;
      return HAL_ERROR;
    }

    if ((status_byte & MT25Q_SR_WIP) == 0U)
    {
      return HAL_OK;
    }
  }

  qspi_hal_error = HAL_QSPI_ERROR_TIMEOUT;
  return HAL_TIMEOUT;
}

static HAL_StatusTypeDef QSPI_WriteEnable(void)
{
  QSPI_CommandTypeDef cmd = {0};

  QSPI_PrepareCommand(&cmd);
  cmd.Instruction = MT25Q_CMD_WRITE_ENABLE;

  return HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
}

static HAL_StatusTypeDef QSPI_ReadJedecId(uint8_t id[3])
{
  QSPI_CommandTypeDef cmd = {0};
  uint8_t rx[3] = {0U, 0U, 0U};
  HAL_StatusTypeDef status;

  if (QSPI_ResetMemory() != HAL_OK)
  {
    qspi_read_substep = 1U;
    return HAL_ERROR;
  }

  QSPI_PrepareCommand(&cmd);
  cmd.Instruction = MT25Q_CMD_READ_ID;
  cmd.DataMode = QSPI_DATA_1_LINE;
  cmd.NbData = 3U;

  qspi_read_substep = 2U;
  status = HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
  if (status != HAL_OK)
  {
    qspi_hal_error = hqspi.ErrorCode;
    return status;
  }

  qspi_read_substep = 3U;
  status = HAL_QSPI_Receive(&hqspi, rx, HAL_QSPI_TIMEOUT_DEFAULT_VALUE);
  if (status != HAL_OK)
  {
    qspi_hal_error = hqspi.ErrorCode;
    return status;
  }

  id[0] = rx[0];
  id[1] = rx[1];
  id[2] = rx[2];
  qspi_jedec_id[0] = rx[0];
  qspi_jedec_id[1] = rx[1];
  qspi_jedec_id[2] = rx[2];

  return HAL_OK;
}

static HAL_StatusTypeDef QSPI_EraseSubsector(uint32_t addr)
{
  QSPI_CommandTypeDef cmd = {0};

  if (QSPI_WriteEnable() != HAL_OK)
  {
    return HAL_ERROR;
  }

  QSPI_PrepareCommand(&cmd);
  cmd.Instruction = MT25Q_CMD_SUBSECTOR_ERASE_4K;
  cmd.AddressMode = QSPI_ADDRESS_1_LINE;
  cmd.AddressSize = QSPI_ADDRESS_24_BITS;
  cmd.Address = addr;

  if (HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return QSPI_AutoPollReady();
}

static HAL_StatusTypeDef QSPI_PageProgram(uint32_t addr, uint8_t *data, uint32_t len)
{
  QSPI_CommandTypeDef cmd = {0};

  if ((len == 0U) || (len > MT25Q_PAGE_SIZE))
  {
    return HAL_ERROR;
  }

  if (QSPI_WriteEnable() != HAL_OK)
  {
    return HAL_ERROR;
  }

  QSPI_PrepareCommand(&cmd);
  cmd.Instruction = MT25Q_CMD_PAGE_PROGRAM;
  cmd.AddressMode = QSPI_ADDRESS_1_LINE;
  cmd.AddressSize = QSPI_ADDRESS_24_BITS;
  cmd.Address = addr;
  cmd.DataMode = QSPI_DATA_1_LINE;
  cmd.NbData = len;

  if (HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_QSPI_Transmit(&hqspi, data, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return HAL_ERROR;
  }

  return QSPI_AutoPollReady();
}

static HAL_StatusTypeDef QSPI_ReadData(uint32_t addr, uint8_t *data, uint32_t len)
{
  QSPI_CommandTypeDef cmd = {0};

  QSPI_PrepareCommand(&cmd);
  cmd.Instruction = MT25Q_CMD_READ;
  cmd.AddressMode = QSPI_ADDRESS_1_LINE;
  cmd.AddressSize = QSPI_ADDRESS_24_BITS;
  cmd.Address = addr;
  cmd.DataMode = QSPI_DATA_1_LINE;
  cmd.NbData = len;

  if (HAL_QSPI_Command(&hqspi, &cmd, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_QSPI_Receive(&hqspi, data, HAL_QSPI_TIMEOUT_DEFAULT_VALUE) != HAL_OK)
  {
    return HAL_ERROR;
  }

#if defined(__DCACHE_PRESENT) && (__DCACHE_PRESENT == 1U)
  SCB_InvalidateDCache_by_Addr((uint32_t *)data, (int32_t)len);
#endif

  return HAL_OK;
}

static int QSPI_IsKnownFlashId(const uint8_t id[3])
{
  if ((id[0] == MT25Q_MANUFACTURER_ID) &&
      (id[1] == MT25Q_MEMORY_TYPE) &&
      (id[2] == MT25Q_MEMORY_CAPACITY_256MBIT))
  {
    return 1;
  }

  if ((id[0] == MX25L_MANUFACTURER_ID) &&
      (id[1] == MX25L_MEMORY_TYPE) &&
      (id[2] == MX25L_MEMORY_CAPACITY_256MBIT))
  {
    return 1;
  }

  return 0;
}

volatile uint8_t g_qspi_test_err_loc = 0U;
static HAL_StatusTypeDef QSPI_RunBasicTest(void)
{
  qspi_test_step = 1U;
  if (QSPI_ReadJedecId((uint8_t *)qspi_jedec_id) != HAL_OK)
  {
    g_qspi_test_err_loc = 1U;
    return HAL_ERROR;
  }

  qspi_test_step = 2U;
  if (!QSPI_IsKnownFlashId((const uint8_t *)qspi_jedec_id))
  {
    g_qspi_test_err_loc = 2U;
    return HAL_ERROR;
  }

  qspi_test_step = 3U;
  if (QSPI_EraseSubsector(MT25Q_TEST_ADDRESS) != HAL_OK)
  {
    g_qspi_test_err_loc = 3U;
    return HAL_ERROR;
  }

  qspi_test_step = 4U;
  for (uint32_t i = 0U; i < MT25Q_PAGE_SIZE; i++)
  {
    qspi_tx_buf[i] = (uint8_t)(i ^ 0x5AU);
  }

  if (QSPI_PageProgram(MT25Q_TEST_ADDRESS, qspi_tx_buf, MT25Q_PAGE_SIZE) != HAL_OK)
  {
    g_qspi_test_err_loc = 4U;
    return HAL_ERROR;
  }

  qspi_test_step = 5U;
  memset(qspi_rx_buf, 0, sizeof(qspi_rx_buf));
  if (QSPI_ReadData(MT25Q_TEST_ADDRESS, qspi_rx_buf, MT25Q_PAGE_SIZE) != HAL_OK)
  {
    g_qspi_test_err_loc = 5U;
    return HAL_ERROR;
  }

  qspi_test_step = 6U;
  if (memcmp(qspi_tx_buf, qspi_rx_buf, MT25Q_PAGE_SIZE) != 0)
  {
    g_qspi_test_err_loc = 6U;
    return HAL_ERROR;
  }

  qspi_test_step = 100U;
  return HAL_OK;
}

static HAL_StatusTypeDef SDRAM_InitializationSequence(SDRAM_HandleTypeDef *hsdram)
{
  FMC_SDRAM_CommandTypeDef command = {0};
  uint32_t mode_register;

  command.CommandMode = FMC_SDRAM_CMD_CLK_ENABLE;
  command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
  command.AutoRefreshNumber = 1U;
  command.ModeRegisterDefinition = 0U;
  if (HAL_SDRAM_SendCommand(hsdram, &command, SDRAM_TIMEOUT) != HAL_OK)
  {
    return HAL_ERROR;
  }

  HAL_Delay(100);

  command.CommandMode = FMC_SDRAM_CMD_PALL;
  command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
  command.AutoRefreshNumber = 1U;
  command.ModeRegisterDefinition = 0U;
  if (HAL_SDRAM_SendCommand(hsdram, &command, SDRAM_TIMEOUT) != HAL_OK)
  {
    return HAL_ERROR;
  }

  command.CommandMode = FMC_SDRAM_CMD_AUTOREFRESH_MODE;
  command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
  command.AutoRefreshNumber = 8U;
  command.ModeRegisterDefinition = 0U;
  if (HAL_SDRAM_SendCommand(hsdram, &command, SDRAM_TIMEOUT) != HAL_OK)
  {
    return HAL_ERROR;
  }

  mode_register = (uint32_t)(SDRAM_MODEREG_BURST_LENGTH_1 |
                             SDRAM_MODEREG_BURST_TYPE_SEQ |
                             SDRAM_MODEREG_CAS_LATENCY_3 |
                             SDRAM_MODEREG_OPERATING_STD |
                             SDRAM_MODEREG_WRITEBURST_SINGLE);
  command.CommandMode = FMC_SDRAM_CMD_LOAD_MODE;
  command.CommandTarget = FMC_SDRAM_CMD_TARGET_BANK1;
  command.AutoRefreshNumber = 1U;
  command.ModeRegisterDefinition = mode_register;
  if (HAL_SDRAM_SendCommand(hsdram, &command, SDRAM_TIMEOUT) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_SDRAM_ProgramRefreshRate(hsdram, SDRAM_REFRESH_COUNT) != HAL_OK)
  {
    return HAL_ERROR;
  }

  HAL_Delay(10);

  return HAL_OK;
}

static HAL_StatusTypeDef SDRAM_RunBasicTest(void)
{
  volatile uint16_t *mem16 = (volatile uint16_t *)SDRAM_DEVICE_ADDR;
  volatile uint32_t *mem32 = (volatile uint32_t *)SDRAM_TEST_SCRATCH_ADDR;
  uint32_t index;

  sdram_test_fail_index = 0U;
  sdram_test_fail_expected = 0U;
  sdram_test_fail_actual = 0U;

  sdram_test_step = 1U;
  for (index = 0U; index < 512U; index++)
  {
    mem16[index] = (uint16_t)(0xB500U + index);
  }
  SCB_CleanDCache_by_Addr((uint32_t *)SDRAM_DEVICE_ADDR, 512U * sizeof(uint16_t));

  sdram_test_step = 2U;
  SCB_InvalidateDCache_by_Addr((uint32_t *)SDRAM_DEVICE_ADDR, 512U * sizeof(uint16_t));
  for (index = 0U; index < 512U; index++)
  {
    uint16_t expected = (uint16_t)(0xB500U + index);
    uint16_t actual = mem16[index];

    if (actual != expected)
    {
      sdram_test_fail_index = index;
      sdram_test_fail_expected = expected;
      sdram_test_fail_actual = actual;
      return HAL_ERROR;
    }
  }

  sdram_test_step = 3U;
  for (index = 0U; index < 32U; index++)
  {
    mem32[index] = (1UL << index);
  }

  sdram_test_step = 4U;
  for (index = 0U; index < 32U; index++)
  {
    uint32_t expected = (1UL << index);
    uint32_t actual = mem32[index];

    if (actual != expected)
    {
      sdram_test_fail_index = index;
      sdram_test_fail_expected = expected;
      sdram_test_fail_actual = actual;
      return HAL_ERROR;
    }
  }

  sdram_test_step = 5U;
  mem32 = (volatile uint32_t *)(SDRAM_DEVICE_ADDR + SDRAM_DEVICE_SIZE - 4U);
  *mem32 = 0xDEADBEEFU;
  if (*mem32 != 0xDEADBEEFU)
  {
    sdram_test_fail_index = (SDRAM_DEVICE_SIZE - 4U) >> 2;
    sdram_test_fail_expected = 0xDEADBEEFU;
    sdram_test_fail_actual = *mem32;
    return HAL_ERROR;
  }

  sdram_test_step = 100U;
  return HAL_OK;
}

AMS5935_HandleTypeDef hamsFS;
AMS5935_Data_t measFS;

AMS5935_HandleTypeDef hamsAS;
AMS5935_Data_t measAS;

AMS5935_HandleTypeDef hamsPS;
AMS5935_Data_t measPS;

/* PS sample freshness for the outer pressure loop.
 *
 * SPI4 is shared between PS and AS, so measPS is NOT refreshed on every
 * 5 ms SensorTask tick: one tick in AS_DECIMATE goes to the ambient
 * sensor, and a busy/torn conversion can hold the previous sample too.
 * BlowerCtrl_Step differentiates the pressure signal (D-on-measurement and
 * the dip-FF drop rate); dividing an unchanged sample by one tick period
 * gives a derivative of zero followed by a doubled one - a periodic
 * artefact at 200 Hz / AS_DECIMATE straight into the RPM command.
 *
 * g_ps_sample_fresh is 1 only on ticks where a new conversion landed;
 * g_ps_sample_dt_s is the interval that sample actually spans. */
volatile uint8_t g_ps_sample_fresh = 0U;
volatile float   g_ps_sample_dt_s  = 0.0f;

volatile uint8_t g_bench_blower_enable = 0U;
volatile uint8_t g_bench_blower_enable_prev = 0U;
/* int32 to span the full motor speed range without wrapping the IPC wire. */
volatile int32_t g_bench_blower_speed_rpm = 29000;
volatile int32_t g_bench_blower_speed_prev_rpm = 29000;

/* Sensirion T/RH sensors */
SensirionTH_Handle_t hSht4x;  /* SHT4x on I2C2 — humidity + temperature */
SensirionTH_Handle_t hSts4x;  /* STS4x on I2C1 — temperature only       */
volatile HAL_StatusTypeDef sht4x_init_status = HAL_ERROR;
volatile HAL_StatusTypeDef sts4x_init_status = HAL_ERROR;

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  boot_stage = 10U;
  BootMarkerWrite(boot_stage);
  BootTraceWrite(BOOT_TRACE_CM7_BASE | 10U);
  /* USER CODE END 1 */
/* USER CODE BEGIN Boot_Mode_Sequence_0 */
/* USER CODE END Boot_Mode_Sequence_0 */

/* USER CODE BEGIN Boot_Mode_Sequence_1 */
/* USER CODE END Boot_Mode_Sequence_1 */
  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  CM7_EnableFpu();
  SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk;
  boot_stage = 13U;
  BootMarkerWrite(boot_stage);
  CM7_PrepareDualCoreBoot();
  boot_stage = 14U;
  BootMarkerWrite(boot_stage);
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();
  boot_stage = 15U;
  BootMarkerWrite(boot_stage);

  /* Configure the peripherals common clocks */
  PeriphCommonClock_Config();
  boot_stage = 16U;
  BootMarkerWrite(boot_stage);

  /* MPU must be configured after system clock (ST H7 recommendation) */
  MPU_Config();

  /* USER CODE BEGIN SysInit */
  SCB_EnableICache();
  SCB_EnableDCache();
  CM7_ConfigureCpu();
  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_DMA_Init();
  MX_GPIO_Init();
  HAL_Delay(50);

  /* Wake CM4 here, exactly like the proven-working bare-metal project
   * (CPAP_TestBoard_V0_4_LowRes). CM4's MX_MotorControl_Init() runs its
   * TIM8 sync-start (startTimers in pwm_common.c) while CM7 has not yet
   * touched TIM2, so it takes the clean "TIM2 clock off" path. Waking CM4
   * late (after CM7's TIM2 encoder is running) puts startTimers in an
   * untested code path that pokes the live encoder timer. */
  CM7_WakeCm4();
  BlowerIpc_CM7_Init();

  /* SDRAM must be ready before LTDC/TouchGFX use the external framebuffer. */
  MX_FMC_Init();
  boot_stage = 21U;
  BootMarkerWrite(boot_stage);

  /* Configure LTDC (panel off until TouchGFX renders first frame). */
  MX_LTDC_Init();
  boot_stage = 23U;
  BootMarkerWrite(boot_stage);
  MX_QUADSPI_Init();

  sdram_test_result = HAL_BUSY;
  sdram_test_step = 0U;
  sd_test_result = HAL_BUSY;
  sd_test_step = 0U;

  MX_CRC_Init();
  MX_TIM2_Init();
  MX_ADC3_Init();
  MX_SPI2_Init();
  MX_SPI4_Init();
  MX_TIM4_Init();
  MX_I2C1_Init();
  MX_I2C2_Init();
  MX_SPI1_Init();
  MX_SDMMC1_SD_Init();
  MX_USART2_UART_Init();
  MX_USB_DEVICE_Init();
  MX_TouchGFX_Init();
  /* USER CODE BEGIN 2 */
  /* Bring up the USB-CDC debug log as early as possible so subsequent
   * init failures are visible on the host. The logger is non-blocking;
   * lines are buffered until a terminal is attached. */
  Dbg_Init();
  Telem_Init();
  /* #80: why the last reset happened - before anything else uses the
   * backup registers. Logged and IWDG1 started from SafetyTask. */
  Watchdog_CaptureResetCause();
  /* RTC calendar (checklist #66): starts the LSE here; SelfTestTask's
   * RtcClock_Service() finishes the bring-up without blocking boot. */
  RtcClock_Init();
  EspBridge_Init();
  DBG_I("SYS", "CM7 boot reached USER CODE 2, sysclk=%lu Hz",
        (unsigned long)HAL_RCC_GetSysClockFreq());

  /* NOTE: MX_TouchGFX_Init() above creates FreeRTOS objects, which leaves
   * BASEPRI masked (priority >= configMAX_SYSCALL) until osKernelStart().
   * From this point until the scheduler starts, EXTI/SPI/I2C interrupts at
   * RTOS-managed priorities will NOT fire, so any bring-up that needs
   * HAL_Delay() timeouts or peripheral IRQs (NFC, AMS5935, Sensirion) is
   * deferred into the owning task's entry code below. */

  /* QSPI test is deferred ~10 s into SelfTestTask via qspi_test_delay so it
   * doesn't block TouchGFX bring-up. Step must start at 0 here: the task's
   * gate is `if (qspi_test_step != 100U)`, so leaving step at 100 would
   * make the task treat the test as already-complete and skip it. */
  qspi_test_result = HAL_BUSY;
  qspi_test_step = 0U;
	
  boot_stage = 25U;
  BootMarkerWrite(boot_stage);
  /* MX_TouchGFX_Init() already ran above (generated init list); calling it
   * twice would recreate the OSWrappers semaphore/queue and leak them. */
  LCD_LatchController(&hltdc);
  boot_stage = 26U;
  BootMarkerWrite(boot_stage);
  BootTraceWrite(BOOT_TRACE_CM7_BASE | 26U);
  boot_stage = 27U;
  BootMarkerWrite(boot_stage);
  boot_stage = 28U;
  BootMarkerWrite(boot_stage);
  boot_stage = 11U;
  BootMarkerWrite(boot_stage);
  /* LCD_RunBasicTest disabled: TouchGFX owns the SDRAM framebuffer */
  lcd_test_result = HAL_OK;
  lcd_test_step = 100U;
  HAL_TIM_Encoder_Start(&htim2, TIM_CHANNEL_ALL);
  RotaryInput_Init();

  /* Sensor / actuator / NFC bring-up moved into the owning tasks' entry
   * code (SensorTask / ClimateTask / NfcTask): it relies on HAL_Delay()
   * and peripheral IRQs which are masked between MX_TouchGFX_Init() and
   * osKernelStart() - see note above. */
  /* USER CODE END 2 */

  /* Init scheduler */
  osKernelInitialize();

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask — unused; project tasks created below. */

  /* USER CODE BEGIN RTOS_THREADS */
  guiTaskHandle      = osThreadNew(GuiTask,      NULL, &guiTask_attributes);
  sensorTaskHandle   = osThreadNew(SensorTask,   NULL, &sensorTask_attributes);
  safetyTaskHandle   = osThreadNew(SafetyTask,   NULL, &safetyTask_attributes);
  nfcTaskHandle      = osThreadNew(NfcTask,      NULL, &nfcTask_attributes);
  climateTaskHandle  = osThreadNew(ClimateTask,  NULL, &climateTask_attributes);
  systemTaskHandle   = osThreadNew(SystemTask,   NULL, &systemTask_attributes);
  selfTestTaskHandle = osThreadNew(SelfTestTask, NULL, &selfTestTask_attributes);
  if ((guiTaskHandle == NULL) || (sensorTaskHandle == NULL) ||
      (nfcTaskHandle == NULL) || (climateTaskHandle == NULL) ||
      (systemTaskHandle == NULL) || (selfTestTaskHandle == NULL))
  {
    Error_Handler(); /* osThreadNew failed: raise configTOTAL_HEAP_SIZE */
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

  /* Start scheduler */
  osKernelStart();

  /* We should never get here as control is now taken by the scheduler */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    /* Never reached - the scheduler owns the CPU. All periodic work lives
     * in the FreeRTOS tasks implemented in USER CODE 4 below. */
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  clock_config_step = 20U;

  /** Supply configuration update enable
  */
  if (HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY) != HAL_OK)
  {
    clock_config_step = 0xE021U;
    Error_Handler();
  }

  clock_config_step = 21U;

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_SYSCFG_CLK_ENABLE();
  if (HAL_PWREx_ControlVoltageScaling(PWR_REGULATOR_VOLTAGE_SCALE1) != HAL_OK)
  {
    clock_config_step = 0xE022U;
    Error_Handler();
  }

  clock_config_step = 22U;

  /* Allow external HSE/BYPASS clock to stabilize on cold power-up. */
  {
    volatile uint32_t hse_settle;
    for (hse_settle = 0U; hse_settle < 8000000U; hse_settle++)
    {
    }
  }

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 2;
  RCC_OscInitStruct.PLL.PLLN = 64;
  RCC_OscInitStruct.PLL.PLLP = 2;
  RCC_OscInitStruct.PLL.PLLQ = 16;
  RCC_OscInitStruct.PLL.PLLR = 2;
  RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
  RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
  RCC_OscInitStruct.PLL.PLLFRACN = 0;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    clock_config_step = 0xE023U;
    Error_Handler();
  }

  clock_config_step = 23U;

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                              |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV2;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
  RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    clock_config_step = 0xE024U;
    Error_Handler();
  }

  clock_config_step = 24U;
}

/**
  * @brief Peripherals Common Clock Configuration
  * @retval None
  */
void PeriphCommonClock_Config(void)
{
  RCC_PeriphCLKInitTypeDef PeriphClkInitStruct = {0};

  /** Initializes the peripherals clock
  */
  PeriphClkInitStruct.PeriphClockSelection = RCC_PERIPHCLK_ADC|RCC_PERIPHCLK_SDMMC
                              |RCC_PERIPHCLK_SPI2|RCC_PERIPHCLK_SPI1
                              |RCC_PERIPHCLK_LTDC;
  PeriphClkInitStruct.PLL2.PLL2M = 25;
  PeriphClkInitStruct.PLL2.PLL2N = 240;
  PeriphClkInitStruct.PLL2.PLL2P = 10;
  PeriphClkInitStruct.PLL2.PLL2Q = 2;
  PeriphClkInitStruct.PLL2.PLL2R = 4;
  PeriphClkInitStruct.PLL2.PLL2RGE = RCC_PLL2VCIRANGE_0;
  PeriphClkInitStruct.PLL2.PLL2VCOSEL = RCC_PLL2VCOWIDE;
  PeriphClkInitStruct.PLL2.PLL2FRACN = 0;
  PeriphClkInitStruct.PLL3.PLL3M = 25;
  PeriphClkInitStruct.PLL3.PLL3N = 258;
  PeriphClkInitStruct.PLL3.PLL3P = 2;
  PeriphClkInitStruct.PLL3.PLL3Q = 16;
  PeriphClkInitStruct.PLL3.PLL3R = 2;
  PeriphClkInitStruct.PLL3.PLL3RGE = RCC_PLL3VCIRANGE_0;
  PeriphClkInitStruct.PLL3.PLL3VCOSEL = RCC_PLL3VCOWIDE;
  PeriphClkInitStruct.PLL3.PLL3FRACN = 0;
  PeriphClkInitStruct.SdmmcClockSelection = RCC_SDMMCCLKSOURCE_PLL2;
  PeriphClkInitStruct.Spi123ClockSelection = RCC_SPI123CLKSOURCE_PLL3;
  PeriphClkInitStruct.AdcClockSelection = RCC_ADCCLKSOURCE_PLL2;
  clock_config_step = 30U;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInitStruct) != HAL_OK)
  {
    clock_config_step = 0xE025U;
    Error_Handler();
  }

  clock_config_step = 31U;
}

/**
  * @brief ADC3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC3_Init(void)
{

  /* USER CODE BEGIN ADC3_Init 0 */

  /* USER CODE END ADC3_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC3_Init 1 */

  /* USER CODE END ADC3_Init 1 */

  /** Common config
  */
  hadc3.Instance = ADC3;
  hadc3.Init.Resolution = ADC_RESOLUTION_16B;
  hadc3.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc3.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc3.Init.LowPowerAutoWait = DISABLE;
  hadc3.Init.ContinuousConvMode = DISABLE;
  hadc3.Init.NbrOfConversion = 1;
  hadc3.Init.DiscontinuousConvMode = DISABLE;
  hadc3.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc3.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc3.Init.ConversionDataManagement = ADC_CONVERSIONDATA_DR;
  hadc3.Init.Overrun = ADC_OVR_DATA_PRESERVED;
  hadc3.Init.LeftBitShift = ADC_LEFTBITSHIFT_NONE;
  hadc3.Init.OversamplingMode = DISABLE;
  hadc3.Init.Oversampling.Ratio = 1;
  if (HAL_ADC_Init(&hadc3) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_13;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SamplingTime = ADC_SAMPLETIME_64CYCLES_5;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  sConfig.OffsetSignedSaturation = DISABLE;
  if (HAL_ADC_ConfigChannel(&hadc3, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC3_Init 2 */
  if (HAL_ADCEx_Calibration_Start(&hadc3, ADC_CALIB_OFFSET, ADC_SINGLE_ENDED) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END ADC3_Init 2 */

}

/**
  * @brief CRC Initialization Function
  * @param None
  * @retval None
  */
static void MX_CRC_Init(void)
{

  /* USER CODE BEGIN CRC_Init 0 */

  /* USER CODE END CRC_Init 0 */

  /* USER CODE BEGIN CRC_Init 1 */

  /* USER CODE END CRC_Init 1 */
  hcrc.Instance = CRC;
  hcrc.Init.DefaultPolynomialUse = DEFAULT_POLYNOMIAL_ENABLE;
  hcrc.Init.DefaultInitValueUse = DEFAULT_INIT_VALUE_ENABLE;
  hcrc.Init.InputDataInversionMode = CRC_INPUTDATA_INVERSION_NONE;
  hcrc.Init.OutputDataInversionMode = CRC_OUTPUTDATA_INVERSION_DISABLE;
  hcrc.InputDataFormat = CRC_INPUTDATA_FORMAT_BYTES;
  if (HAL_CRC_Init(&hcrc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CRC_Init 2 */

  /* USER CODE END CRC_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x10C0ECFF;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief I2C2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C2_Init(void)
{

  /* USER CODE BEGIN I2C2_Init 0 */

  /* USER CODE END I2C2_Init 0 */

  /* USER CODE BEGIN I2C2_Init 1 */

  /* USER CODE END I2C2_Init 1 */
  hi2c2.Instance = I2C2;
  hi2c2.Init.Timing = 0x10C0ECFF;
  hi2c2.Init.OwnAddress1 = 0;
  hi2c2.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c2.Init.OwnAddress2 = 0;
  hi2c2.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c2.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c2, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c2, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C2_Init 2 */

  /* USER CODE END I2C2_Init 2 */

}

/**
  * @brief LTDC Initialization Function
  * @param None
  * @retval None
  */
static void MX_LTDC_Init(void)
{

  /* USER CODE BEGIN LTDC_Init 0 */

  /* USER CODE END LTDC_Init 0 */

  /* USER CODE BEGIN LTDC_Init 1 */
  hltdc.Instance = LTDC;
  if (LCD_Init(&hltdc) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END LTDC_Init 1 */

  /* USER CODE BEGIN LTDC_Init 2 */

  /* USER CODE END LTDC_Init 2 */

}

/**
  * @brief QUADSPI Initialization Function
  * @param None
  * @retval None
  */
static void MX_QUADSPI_Init(void)
{

  /* USER CODE BEGIN QUADSPI_Init 0 */

  /* USER CODE END QUADSPI_Init 0 */

  /* USER CODE BEGIN QUADSPI_Init 1 */

  /* USER CODE END QUADSPI_Init 1 */
  /* QUADSPI parameter configuration*/
  hqspi.Instance = QUADSPI;
  hqspi.Init.ClockPrescaler = 50;
  hqspi.Init.FifoThreshold = 1;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_NONE;
  hqspi.Init.FlashSize = 1;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_1_CYCLE;
  hqspi.Init.ClockMode = QSPI_CLOCK_MODE_0;
  hqspi.Init.FlashID = QSPI_FLASH_ID_1;
  hqspi.Init.DualFlash = QSPI_DUALFLASH_DISABLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN QUADSPI_Init 2 */
  hqspi.Init.ClockPrescaler = 255;
  hqspi.Init.SampleShifting = QSPI_SAMPLE_SHIFTING_HALFCYCLE;
  hqspi.Init.FlashSize = 24;
  hqspi.Init.ChipSelectHighTime = QSPI_CS_HIGH_TIME_6_CYCLE;
  if (HAL_QSPI_Init(&hqspi) != HAL_OK)
  {
    Error_Handler();
  }
  HAL_Delay(10);
  /* USER CODE END QUADSPI_Init 2 */

}

/**
  * @brief SDMMC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SDMMC1_SD_Init(void)
{

  /* USER CODE BEGIN SDMMC1_Init 0 */

  /* USER CODE END SDMMC1_Init 0 */

  /* USER CODE BEGIN SDMMC1_Init 1 */

  /* USER CODE END SDMMC1_Init 1 */
  /* SD card is initialized later in SD_RunBasicTest (1-bit, card detect). */
  SD_PrepareSdmmcHandle();
  /* USER CODE BEGIN SDMMC1_Init 2 */

  /* USER CODE END SDMMC1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  /* ST25R3920B requires SPI mode 1 (CPOL=0, CPHA=1) per datasheet 4.3.3. */
  hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 0x0;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_DISABLE;
  hspi1.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi1.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi1.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi1.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi1.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi1.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi1.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi1.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief SPI2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI2_Init(void)
{

  /* USER CODE BEGIN SPI2_Init 0 */

  /* USER CODE END SPI2_Init 0 */

  /* USER CODE BEGIN SPI2_Init 1 */

  /* USER CODE END SPI2_Init 1 */
  /* SPI2 parameter configuration*/
  hspi2.Instance = SPI2;
  hspi2.Init.Mode = SPI_MODE_MASTER;
  hspi2.Init.Direction = SPI_DIRECTION_2LINES;
  hspi2.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi2.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi2.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi2.Init.NSS = SPI_NSS_SOFT;
  hspi2.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_64;
  hspi2.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi2.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi2.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi2.Init.CRCPolynomial = 0x0;
  hspi2.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi2.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi2.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi2.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi2.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi2.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi2.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi2.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi2.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI2_Init 2 */

  /* USER CODE END SPI2_Init 2 */

}

/**
  * @brief SPI4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI4_Init(void)
{

  /* USER CODE BEGIN SPI4_Init 0 */

  /* USER CODE END SPI4_Init 0 */

  /* USER CODE BEGIN SPI4_Init 1 */

  /* USER CODE END SPI4_Init 1 */
  /* SPI4 parameter configuration*/
  hspi4.Instance = SPI4;
  hspi4.Init.Mode = SPI_MODE_MASTER;
  hspi4.Init.Direction = SPI_DIRECTION_2LINES;
  hspi4.Init.DataSize = SPI_DATASIZE_8BIT;
  hspi4.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi4.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi4.Init.NSS = SPI_NSS_SOFT;
  hspi4.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
  hspi4.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi4.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi4.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi4.Init.CRCPolynomial = 0x0;
  hspi4.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  hspi4.Init.NSSPolarity = SPI_NSS_POLARITY_LOW;
  hspi4.Init.FifoThreshold = SPI_FIFO_THRESHOLD_01DATA;
  hspi4.Init.TxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.RxCRCInitializationPattern = SPI_CRC_INITIALIZATION_ALL_ZERO_PATTERN;
  hspi4.Init.MasterSSIdleness = SPI_MASTER_SS_IDLENESS_00CYCLE;
  hspi4.Init.MasterInterDataIdleness = SPI_MASTER_INTERDATA_IDLENESS_00CYCLE;
  hspi4.Init.MasterReceiverAutoSusp = SPI_MASTER_RX_AUTOSUSP_DISABLE;
  hspi4.Init.MasterKeepIOState = SPI_MASTER_KEEP_IO_STATE_DISABLE;
  hspi4.Init.IOSwap = SPI_IO_SWAP_DISABLE;
  if (HAL_SPI_Init(&hspi4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI4_Init 2 */

  /* USER CODE END SPI4_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_Encoder_InitTypeDef sConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 0;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 80;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  sConfig.EncoderMode = TIM_ENCODERMODE_TI12;
  sConfig.IC1Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC1Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC1Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC1Filter = 8;
  sConfig.IC2Polarity = TIM_ICPOLARITY_RISING;
  sConfig.IC2Selection = TIM_ICSELECTION_DIRECTTI;
  sConfig.IC2Prescaler = TIM_ICPSC_DIV1;
  sConfig.IC2Filter = 8;
  if (HAL_TIM_Encoder_Init(&htim2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 230400;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  huart2.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart2.Init.ClockPrescaler = UART_PRESCALER_DIV1;
  huart2.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetTxFifoThreshold(&huart2, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_SetRxFifoThreshold(&huart2, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_UARTEx_DisableFifoMode(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}

/**
  * Enable DMA controller clock
  */
static void MX_DMA_Init(void)
{

  /* DMA controller clock enable */
  __HAL_RCC_DMA1_CLK_ENABLE();

}

/* FMC initialization function */
void MX_FMC_Init(void)
{

  /* USER CODE BEGIN FMC_Init 0 */

  /* USER CODE END FMC_Init 0 */

  FMC_SDRAM_TimingTypeDef SdramTiming = {0};

  /* USER CODE BEGIN FMC_Init 1 */

  /* USER CODE END FMC_Init 1 */

  /** Perform the SDRAM1 memory initialization sequence
  */
  hsdram1.Instance = FMC_SDRAM_DEVICE;
  /* hsdram1.Init */
  hsdram1.Init.SDBank = FMC_SDRAM_BANK1;
  hsdram1.Init.ColumnBitsNumber = FMC_SDRAM_COLUMN_BITS_NUM_9;
  hsdram1.Init.RowBitsNumber = FMC_SDRAM_ROW_BITS_NUM_13;
  hsdram1.Init.MemoryDataWidth = FMC_SDRAM_MEM_BUS_WIDTH_16;
  hsdram1.Init.InternalBankNumber = FMC_SDRAM_INTERN_BANKS_NUM_4;
  hsdram1.Init.CASLatency = FMC_SDRAM_CAS_LATENCY_3;
  hsdram1.Init.WriteProtection = FMC_SDRAM_WRITE_PROTECTION_DISABLE;
  hsdram1.Init.SDClockPeriod = FMC_SDRAM_CLOCK_PERIOD_2;
  hsdram1.Init.ReadBurst = FMC_SDRAM_RBURST_DISABLE;
  hsdram1.Init.ReadPipeDelay = FMC_SDRAM_RPIPE_DELAY_0;
  /* SdramTiming */
  SdramTiming.LoadToActiveDelay = 2;
  SdramTiming.ExitSelfRefreshDelay = 8;
  SdramTiming.SelfRefreshTime = 7;
  SdramTiming.RowCycleDelay = 7;
  SdramTiming.WriteRecoveryTime = 4;
  SdramTiming.RPDelay = 3;
  SdramTiming.RCDDelay = 3;

  if (HAL_SDRAM_Init(&hsdram1, &SdramTiming) != HAL_OK)
  {
    Error_Handler( );
  }

  /* USER CODE BEGIN FMC_Init 2 */
  if (SDRAM_InitializationSequence(&hsdram1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE END FMC_Init 2 */
}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};
  /* USER CODE BEGIN MX_GPIO_Init_1 */

  /* USER CODE END MX_GPIO_Init_1 */

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI4_CS1_GPIO_Port, SPI4_CS1_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(OTG_FS_PWR_ON_GPIO_Port, OTG_FS_PWR_ON_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOB, BL_CTL_Pin|SPI2_CS_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(SPI4_CS2_GPIO_Port, SPI4_CS2_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin : SDIO_DET_Pin */
  GPIO_InitStruct.Pin = SDIO_DET_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(SDIO_DET_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI4_CS1_Pin */
  GPIO_InitStruct.Pin = SPI4_CS1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI4_CS1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : NFC_INT_Pin */
  GPIO_InitStruct.Pin = NFC_INT_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(NFC_INT_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : OTG_FS_PWR_ON_Pin */
  GPIO_InitStruct.Pin = OTG_FS_PWR_ON_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(OTG_FS_PWR_ON_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI1_CS_Pin */
  GPIO_InitStruct.Pin = SPI1_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
  HAL_GPIO_Init(SPI1_CS_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : BL_CTL_Pin SPI2_CS_Pin */
  GPIO_InitStruct.Pin = BL_CTL_Pin|SPI2_CS_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

  /*Configure GPIO pin : SPI4_CS2_Pin */
  GPIO_InitStruct.Pin = SPI4_CS2_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(SPI4_CS2_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : ROT_SW_Pin */
  GPIO_InitStruct.Pin = ROT_SW_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(ROT_SW_GPIO_Port, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(ROT_SW_EXTI_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(ROT_SW_EXTI_IRQn);

  /* USER CODE BEGIN MX_GPIO_Init_2 */

  /* USER CODE END MX_GPIO_Init_2 */
}

/* USER CODE BEGIN 4 */

/**
  * @brief GUI task: TouchGFX render loop.
  *
  * MX_TouchGFX_Process() blocks on the VSYNC message queue (fed by the LTDC
  * line interrupt) and renders exactly one frame per call, so this task
  * sleeps between frames and only consumes CPU while drawing (~60 Hz).
  * The rotary encoder is polled once per frame so Model::tick() always sees
  * the freshest detents/presses.
  */
static void GuiTask(void *argument)
{
  (void)argument;
  boot_stage = 12U;
  for (;;)
  {
    Watchdog_Kick(WDG_TASK_GUI);
    RotaryInput_Poll();
    MX_TouchGFX_Process();
  }
}

/**
  * @brief Fast sensor acquisition: 3x AMS5935 (SPI) + analog pressure + blower
  *        IPC status snapshot, plus the outer-loop pressure PID.
  *
  *  Timing strategy
  *  ---------------
  *  Each AMS5935 single-shot conversion takes ~4 ms (datasheet) and the
  *  HAL transaction itself is sub-100us. So a naive
  *      Start -> HAL_Delay(6) -> Read
  *  pattern can never run faster than ~6 ms per sensor, which is what the
  *  previous implementation did at a 10 ms period.
  *
  *  To get the patient pressure (PS) up to 200 Hz for the pressure PID we
  *  run a *pipelined* scheme:
  *      tick N  : start conversions
  *      tick N+1: read results (4 ms conversion fits inside one 5 ms tick)
  *  AS and PS share SPI4, so PS gets every tick except 1-in-20 where AS
  *  borrows the slot (PS effective rate = ~190 Hz; AS = 10 Hz, plenty for
  *  weather/altitude tracking).
  *
  *  FS (flow, SPI2) deliberately runs a *2-tick* pipeline (100 Hz). One tick
  *  left only ~0.6 ms of margin over the 4 ms conversion, and with no EOC
  *  line and no shared clock that margin was periodically crossed - the read
  *  landed on the register update and returned a torn value. See
  *  FS_CONV_TICKS below. 100 Hz is still ~7x the 15 Hz ΔP LPF.
  *
  *  After reading the new pressure, we run BlowerCtrl_Step() so the PID
  *  acts on the freshest sample with minimum loop latency.
  */
static void SensorTask(void *argument)
{
  (void)argument;

  /* One-time bring-up (runs with the scheduler active so HAL_Delay()
   * timeouts inside the SPI transactions work). */

  /* Flow Sensor Handle Init */
  hamsFS.hspi = &hspi2;
  hamsFS.cs_port = GPIOB;
  hamsFS.cs_pin = GPIO_PIN_12;
  hamsFS.use_eoc = 0;              /* set 1 if EOC line connected */
  hamsFS.p_min = -5.0f;
  hamsFS.p_max = 5.0f;
  /* Over-range reads full scale, not the last good sample. ±5 mbar through
   * the flow LUT tops out at ~93 slm total, and a mask disconnect at high
   * pressure goes past it. Holding froze the published flow at whatever it
   * had reached on the way up (26-53 slm on the bench) - under the mask-off
   * enter threshold, so the flow path could not see the disconnect. Only
   * the flow sensor opts in; hamsAS / hamsPS keep hold-on-bad-read. */
  hamsFS.saturate_on_overflow = 1U;
  AMS5935_Init(&hamsFS);

  /* Atmospheric Pressure Sensor Handle Init */
  hamsAS.hspi = &hspi4;
  hamsAS.cs_port = GPIOE;
  hamsAS.cs_pin = GPIO_PIN_4;
  hamsAS.use_eoc = 0;
  hamsAS.p_min = 700.0f;
  hamsAS.p_max = 1200.0f;
  AMS5935_Init(&hamsAS);

  /* Pressure Sensor Handle Init */
  hamsPS.hspi = &hspi4;
  hamsPS.cs_port = GPIOG;
  hamsPS.cs_pin = GPIO_PIN_3;
  hamsPS.use_eoc = 0;
  hamsPS.p_min = -50.0f;
  hamsPS.p_max = 50.0f;
  AMS5935_Init(&hamsPS);

  /* PID state. Defaults to disabled in BlowerCtrl_Init() - flip
   * g_blower_ctrl_enabled = 1 (and a sensible setpoint) from the watch
   * window, the GUI, or boot config when you want to use it. */
  BlowerCtrl_Init();

  /* Flow-sensor conditioning. Default is a 10 Hz LPF on the raw FS
   * differential pressure - smooths the blower-induced pulsation while
   * preserving respiratory features. Tune via g_flow_lpf_cutoff_hz. */
  Flow_Init();

  /* Tube-loss compensator. Defaults disabled - on the current bench rig
   * the sensor sits adjacent to the load so the hose drop is negligible.
   * When you connect a real 1.8 m hose, set g_tubecomp_enabled = 1 and
   * (optionally) tune the R coefficients to your hose. With it enabled,
   * the PID and leak estimator both automatically work off the
   * mask-pressure estimate instead of the device-side sensor reading. */
  TubeComp_Init();

  /* Unintentional-leak estimator. Always-on; meaningful values appear
   * within ~1 second of therapy start (Therapy_Start reseeds the LPF). */
  Leak_Init();

  /* CPAP therapy state machine. Starts in IDLE; the GUI flips it to
   * RAMP/RUNNING via Therapy_Start when the user clicks START THERAPY. */
  Therapy_Init();
  PhysioTelem_Init();

  /* Tick period of the loop. 5 ms = 200 Hz, matched to the BlowerCtrl
   * PID. Changing this requires re-tuning the PID gains. */
  const uint32_t period_ms = 5U;
  const float    dt_s      = (float)period_ms * 0.001f;

  /* Period 1-in-N for atmospheric (AS) sampling. AS borrows one PS slot
   * every AS_DECIMATE ticks. 20 ticks @ 5 ms = 100 ms = 10 Hz. */
  const uint32_t AS_DECIMATE = 20U;

  /* Period 1-in-N for analog ADC pressure sampling. ADC is cheap (~10us)
   * but every tick is overkill - 100 Hz is plenty for the AMS5105. */
  const uint32_t ADC_DECIMATE = 2U;

  /* Period 1-in-N for IPC status snapshot. CM4 only updates the mailbox
   * at ~100 Hz, so polling at 100 Hz is enough; saves a few cycles per
   * tick that we'd rather give to the PID. */
  const uint32_t IPC_DECIMATE = 2U;

  /* Ticks allowed for one FS conversion. The AMS5935 needs ~4 ms and the
   * EOC line is not wired (hamsFS.use_eoc = 0), so a 1-tick pipeline left
   * only ~0.6 ms of margin. With no clock shared between the sensor and our
   * tick, that phase creeps and periodically the read lands on the register
   * update, returning a torn 24-bit value - a ~0.1 mbar error, which the
   * near-zero LUT (~100 slm/mbar) prints as a ~10 slm needle on Q_patient
   * roughly every 18 s. 2 ticks = 10 ms gives ~6 ms of margin on the happy
   * path. If the sensor is still busy we wait further (do NOT issue a new
   * 0xAA - that aborts and recreates the race) up to FS_CONV_TICKS_MAX. */
  const uint32_t FS_CONV_TICKS     = 2U;
  const uint32_t FS_CONV_TICKS_MAX = 6U; /* 30 ms: give up and restart */

  /* Kick off the first set of conversions BEFORE the loop so the very
   * first tick has data to harvest. FS is on SPI2; PS goes on SPI4 (AS
   * will get its turn after AS_DECIMATE ticks). */
  (void)AMS5935_StartSingleMeasurement(&hamsFS);
  (void)AMS5935_StartSingleMeasurement(&hamsPS);
  uint8_t ps_in_flight = 1U;
  uint8_t as_in_flight = 0U;
  uint32_t fs_wait_ticks = 0U;
  float    fs_dt_s = 0.0f;
  /* Elapsed time since the last fresh PS conversion (see g_ps_sample_fresh). */
  float    ps_dt_s = 0.0f;

  uint32_t tick_n    = 0U;
  uint32_t next_wake = osKernelGetTickCount() + period_ms;

  for (;;)
  {
    (void)osDelayUntil(next_wake);
    next_wake += period_ms;
    Watchdog_Kick(WDG_TASK_SENSOR);

    /* ---------- Harvest results from previous tick's conversions ---------- */

    /* Assume no new PS sample this tick; the SPI4 harvest below sets the
     * flag if one actually lands. BlowerCtrl_Step reads both. */
    g_ps_sample_fresh = 0U;
    ps_dt_s += dt_s;

    /* SPI2: FS (flow). Read only once the conversion has had FS_CONV_TICKS
     * of settling, then restart immediately so the next conversion owns the
     * whole window (rather than starting mid-tick further down). */
    fs_dt_s += dt_s;
    fs_wait_ticks++;
    if (fs_wait_ticks >= FS_CONV_TICKS)
    {
      /* Only feed genuinely fresh conversions into the ~100 slm/mbar
       * near-zero LUT. A held (busy/overflow/torn) sample is flagged stale
       * so the LPF, leak estimator and auto-zero quiet detector all freeze.
       * Do not start a new conversion while the previous one is still busy:
       * a fresh 0xAA aborts the in-flight ADC and the next read can land
       * on a torn register. */
      if ((AMS5935_ReadRaw(&hamsFS, &measFS) == HAL_OK)
          && (AMS5935_WasSampleHeld(&hamsFS) == 0U))
      {
        Flow_Update(measFS.pressure, fs_dt_s,
                    AMS5935_WasSampleSaturated(&hamsFS) != 0U);
        fs_dt_s = 0.0f;
        (void)AMS5935_StartSingleMeasurement(&hamsFS);
        fs_wait_ticks = 0U;
      }
      else
      {
        Flow_NoteStaleSample();
        if (fs_wait_ticks >= FS_CONV_TICKS_MAX)
        {
          fs_dt_s = 0.0f;
          (void)AMS5935_StartSingleMeasurement(&hamsFS);
          fs_wait_ticks = 0U;
        }
      }
    }

    /* Flow auto-zero: idle + blower stopped. Quiet ΔP mean, then slew. */
    const bool therapy_idle =
        (Therapy_GetState() == THERAPY_STATE_IDLE);
    const bool blower_idle =
        (!BlowerCtrl_IsEnabled())
        && (!BlowerCtrl_IsManualActive())
        && (g_bench_blower_enable == 0U)
        && (g_blower_status.mech_speed_rpm < 500);
    Flow_AutoZeroService(dt_s, therapy_idle && blower_idle);

    /* SPI4: AS or PS - mutually exclusive each tick. */
    if (as_in_flight)
    {
      (void)AMS5935_ReadRaw(&hamsAS, &measAS);
      as_in_flight = 0U;
      /* Ambient absolute P (mbar) from AS drives orifice density altitude
       * compensation and optional BTPS scaling. */
      Flow_SetBtpsAmbientMbar(measAS.pressure);
    }
    else if (ps_in_flight)
    {
      /* "Fresh" here means the PS slot was serviced this tick, NOT that the
       * conversion was pristine. PS runs a 1-tick pipeline (5 ms) against a
       * ~4 ms conversion with no EOC line, so AMS5935_ReadRaw often returns
       * a held sample - it still carries the last good pressure, which is
       * what the loop must keep tracking. Rejecting those here would stall
       * the derivative timing for long stretches; the only thing the flag
       * needs to catch is the AS decimation hole, where measPS is not
       * refreshed at all. */
      if (AMS5935_ReadRaw(&hamsPS, &measPS) == HAL_OK)
      {
        g_ps_sample_dt_s  = ps_dt_s;
        g_ps_sample_fresh = 1U;
        ps_dt_s = 0.0f;
      }
      ps_in_flight = 0U;
    }

    /* ---------- Start next-tick conversions ---------- */

    /* SPI4: every AS_DECIMATE-th tick we devote the slot to AS, skipping
     * PS for that one tick. PS effective rate = (AS_DECIMATE-1)/AS_DECIMATE
     * * 200 Hz = 190 Hz with AS_DECIMATE=20. */
    if ((tick_n % AS_DECIMATE) == 0U)
    {
      (void)AMS5935_StartSingleMeasurement(&hamsAS);
      as_in_flight = 1U;
    }
    else
    {
      (void)AMS5935_StartSingleMeasurement(&hamsPS);
      ps_in_flight = 1U;
    }

    /* ---------- Lower-priority bookkeeping ---------- */

    if ((tick_n % ADC_DECIMATE) == 0U)
    {
      (void)pressure_sensor_read_analog_adc_raw();
    }

    if ((tick_n % IPC_DECIMATE) == 0U)
    {
      (void)BlowerIpc_CM7_GetStatus(&g_blower_status);
    }

    /* ---------- Leak / patient flow (before PID so flow FF is current) --- */
    const float q_total_slm = Flow_GetSlm();
    const float p_sensor_cmh2o = MAIN_MBAR_TO_CMH2O(measPS.pressure);
    const float p_mask_est_cmh2o = TubeComp_Apply(p_sensor_cmh2o, q_total_slm);
    Leak_SetTherapyPressureCmh2o(BlowerCtrl_GetSetpoint());
    Leak_SetHoldAdaptation(Flow_IsSampleStale());
    Leak_Update(q_total_slm, p_mask_est_cmh2o, dt_s);
    Flow_SpikeWatch(Flow_GetPatientSlm(), dt_s);

    /* ---------- Therapy management (before PID so EPR setpoint is current) --- */
    Therapy_Update(dt_s);
    PhysioTelem_Update(dt_s);

    /* ---------- Outer-loop pressure PID ---------- */
    BlowerCtrl_Step(dt_s);

    /* Bench-only replay of a recorded speed command at this tick, ramp 0
     * (manual mode only; no-op otherwise). See bench_playback.h. */
    BenchPlay_Tick(dt_s);

    /* USB telemetry: P_sensor, P_mask, patient flow, PID cmd, CM4 mech RPM at 50 Hz. */
    if ((tick_n % 4U) == 0U)
    {
      Telem_PushSample(BlowerCtrl_GetSensorCmh2o(),
                       Flow_GetPatientSlm(),//Flow_GetDpFiltered(),
                       BenchPlay_IsActive() ? BenchPlay_CurrentRpm()
                                            : (int32_t)BlowerCtrl_GetOutputRpm(),
                       g_blower_status.mech_speed_rpm,
                       BlowerCtrl_GetMeasured());
    }

    /* USART2 → ESP32 → MQTT at 100 Hz / 10 ms (200 Hz loop, every 2nd tick).
     * ~60 B/frame = 6 kB/s, ~26% of the 230400-baud line; drained by the
     * low-priority SystemTask so it cannot stall SensorTask/GUI/Safety. */
    if ((tick_n % 2U) == 0U)
    {
      EspBridge_PushSample(BlowerCtrl_GetSensorCmh2o(),
                           BlowerCtrl_GetMeasured(),
                           Flow_GetPatientSlm(),
                           Flow_GetSlm(),
                           BlowerCtrl_GetSetpoint(),
                           (int32_t)BlowerCtrl_GetOutputRpm(),
                           g_blower_status.mech_speed_rpm,
                           Leak_GetReportedLeakSlm(),
                           Leak_GetVentSlm(),
                           (uint8_t)Therapy_GetState(),
                           (uint8_t)Therapy_GetMode(),
                           Therapy_IsMaskOffActive() ? 1U : 0U);
    }

    tick_n++;
  }
}

/**
  * @brief Safety supervisor: pressure faults + CM4 IPC heartbeat.
  *        Runs at osPriorityRealtime so it can preempt SensorTask/GUI.
  */
static void SafetyTask(void *argument)
{
  (void)argument;

  const uint32_t period_ms = 5U;
  const float dt_s = (float)period_ms * 0.001f;

  /* #80: IWDG1 - refreshed only from here, and only while every task
   * checks in on time (watchdog.h). */
  Watchdog_Start();
  uint32_t next_wake = osKernelGetTickCount() + period_ms;

  for (;;)
  {
    (void)osDelayUntil(next_wake);
    next_wake += period_ms;
    Safety_Supervisor_Update(dt_s);
    Watchdog_Service();
  }
}

/**
  * @brief NFC task: RFAL discovery worker + secure component verification.
  *        Each NfcTest_Poll() call is short; the RFAL state machine just
  *        needs regular servicing.
  */
static void NfcTask(void *argument)
{
  (void)argument;

  /* One-time bring-up: RFAL init needs HAL_Delay() and the ST25R EXTI
   * interrupt, both unavailable before the scheduler starts. */
  {
    ReturnCode rc = NfcTest_Init();
    if (rc == RFAL_ERR_NONE)
    {
      DBG_I("NFC", "init ok, chip_id=0x%02X rev=0x%02X",
            (unsigned)g_nfc_chip_id, (unsigned)g_nfc_chip_rev);
    }
    else
    {
      DBG_E("NFC", "init failed rc=%d status=%u raw_id=0x%02X",
            (int)rc, (unsigned)g_nfc_test_status,
            (unsigned)g_nfc_raw_chip_id);
    }
  }

  for (;;)
  {
    Watchdog_Kick(WDG_TASK_NFC);
    NfcTest_Poll();
    osDelay(10U);
  }
}

/**
  * @brief Climate task: Sensirion T/RH reads (~10 ms blocking I2C each) and
  *        the humidifier closed loop. The plant time constant is minutes, so
  *        250 ms sampling is more than enough; Humidifier_Update() decimates
  *        these calls down to its 1 Hz control tick.
  */
static void ClimateTask(void *argument)
{
  (void)argument;

  /* One-time bring-up: Sensirion init does blocking I2C with HAL timeouts. */
  /* Sensirion SHT4x (RH+T) on I2C2, STS4x (T-only) on I2C1, both at 0x44. */
  sht4x_init_status = SensirionTH_Init(&hSht4x, &hi2c2, SENSIRION_I2C_ADDR_44, 1U);
  sts4x_init_status = SensirionTH_Init(&hSts4x, &hi2c1, SENSIRION_I2C_ADDR_44, 0U);

  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
  HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_2);

  /* CH2 is unrelated to the humidifier; leave the existing test value. */
  TIM4->CCR2 = 65000;
  /* CH1 is owned by the humidifier closed loop. Init forces CCR1 = 0. */
  Humidifier_Init();

  for (;;)
  {
    Watchdog_Kick(WDG_TASK_CLIMATE);
    (void)SensirionTH_ReadMeasurement(&hSht4x);
    (void)SensirionTH_ReadMeasurement(&hSts4x);
    Humidifier_Update();
    osDelay(250U);
  }
}

/**
  * @brief System task: blower start/stop/speed commands driven from the
  *        debugger globals (g_bench_blower_enable / g_bench_blower_speed_rpm),
  *        edge-triggered blower
  *        fault/state logging, and the USB CDC debug-log pump.
  */
static void SystemTask(void *argument)
{
  (void)argument;
  uint16_t s_blr_prev_state  = 0xFFFFU;
  uint16_t s_blr_prev_faults = 0U;
  int32_t  s_blr_log_decim   = 0;

  for (;;)
  {
    Watchdog_Kick(WDG_TASK_SYSTEM);
    Watchdog_TestPoll();        /* `wdg test`, after the ack has drained */
    Telem_ProcessCommands();

    if ((g_bench_blower_enable == 1U) && (g_bench_blower_enable_prev == 0U))
    {
      BlowerIpc_CM7_Start();
    }
    else if ((g_bench_blower_enable == 0U) && (g_bench_blower_enable_prev == 1U))
    {
      BlowerIpc_CM7_Stop();
    }

    if (g_bench_blower_speed_rpm != g_bench_blower_speed_prev_rpm)
    {
      BlowerIpc_CM7_SetSpeedRpm(g_bench_blower_speed_rpm, 0U);
      Telem_PushBenchRpm(g_bench_blower_speed_rpm,
                         g_blower_status.mech_speed_rpm,
                         BlowerCtrl_GetSensorCmh2o());
    }

    g_bench_blower_speed_prev_rpm = g_bench_blower_speed_rpm;
    g_bench_blower_enable_prev = g_bench_blower_enable;

    /* Edge-trigger blower fault / state logs from the latest IPC status. */
    if (g_blower_status.mc_state != s_blr_prev_state)
    {
      DBG_I("BLR", "state %u -> %u (ref=%ld act=%ld vbus=%u)",
        (unsigned)s_blr_prev_state, (unsigned)g_blower_status.mc_state,
        (long)g_blower_status.speed_ref_rpm,
        (long)g_blower_status.mech_speed_rpm,
        (unsigned)g_blower_status.bus_voltage_v);
      s_blr_prev_state = g_blower_status.mc_state;
    }
    if (g_blower_status.current_faults != s_blr_prev_faults)
    {
      if (g_blower_status.current_faults != 0U)
      {
        DBG_E("BLR", "fault 0x%04X (occurred=0x%04X) ref=%ld act=%ld",
          (unsigned)g_blower_status.current_faults,
          (unsigned)g_blower_status.occurred_faults,
          (long)g_blower_status.speed_ref_rpm,
          (long)g_blower_status.mech_speed_rpm);
      }
      else
      {
        DBG_I("BLR", "fault cleared (occurred=0x%04X)",
          (unsigned)g_blower_status.occurred_faults);
      }
      s_blr_prev_faults = g_blower_status.current_faults;
    }
    /* Periodic speed snapshot (~1 Hz at 10 ms task period). */
    if (++s_blr_log_decim >= 100)
    {
      s_blr_log_decim = 0;
      if (g_blower_status.motor_running)
      {
        DBG_D("BLR", "ref=%ld act=%ld vbus=%u brake=%u",
          (long)g_blower_status.speed_ref_rpm,
          (long)g_blower_status.mech_speed_rpm,
          (unsigned)g_blower_status.bus_voltage_v,
          (unsigned)g_blower_status.brake_pwm_permille);
      }
    }

    /* Drain USB CDC — up to 8 x 64-byte chunks per tick while endpoint idle. */
    for (uint8_t drain = 0U; drain < 8U; drain++)
    {
      Telem_Pump();
      Dbg_Pump();
    }
    EspBridge_Pump();

    osDelay(10U);
  }
}

/**
  * @brief Self-test task: SD card detect/self-test state machine plus the
  *        deferred SDRAM/QSPI one-shot tests. Lowest priority because the
  *        QSPI erase/program test can block for seconds; nothing else may
  *        be disturbed by it.
  *
  * Tick period is 10 ms (the old bare-metal loop ran at 5 ms, so the delay
  * thresholds are halved to preserve the original real-time behaviour:
  * 2.5 s settle after insertion, 25 s between failed-test retries, 10 s
  * before the deferred SDRAM/QSPI tests).
  */
static void SelfTestTask(void *argument)
{
  (void)argument;
  uint32_t sd_test_delay = 0U;
  uint8_t  sd_prev_present = 0xFFU;
  HAL_StatusTypeDef sd_prev_result = HAL_BUSY;
  uint32_t sdram_test_delay = 0U;
  uint32_t qspi_test_delay = 0U;
  uint8_t  sd_dummy_done = 0U;
  uint32_t sd_fatfs_settle = 0U;

  for (;;)
  {
    Watchdog_Kick(WDG_TASK_SELFTEST);
    if (!SD_CardPresent())
    {
      if (hsd1.State != HAL_SD_STATE_RESET)
      {
        (void)HAL_SD_DeInit(&hsd1);
      }
      sd_card_present = 0U;
      sd_test_step = 0U;
      sd_test_fail_step = 0U;
      sd_test_result = HAL_BUSY;
      sd_test_delay = 0U;
      sd_fatfs_ready = 0U;
      sd_fatfs_settle = 0U;
      sd_dummy_done = 0U;
      sd_dummy_step = 0U;
    }
    else if (sd_test_result == HAL_BUSY)
    {
      sd_card_present = 1U;
      sd_fatfs_ready = 0U;
      sd_fatfs_settle = 0U;
      if (sd_test_delay < 250U)
      {
        sd_test_delay++;
      }
      else
      {
        sd_test_result = SD_RunBasicTest();
      }
    }
    else if (sd_test_result != HAL_OK)
    {
      sd_card_present = 1U;
      sd_fatfs_ready = 0U;
      if (sd_test_delay < 2500U)
      {
        sd_test_delay++;
      }
      else
      {
        sd_test_step = 0U;
        sd_test_fail_step = 0U;
        sd_hal_error = 0U;
        sd_test_result = HAL_BUSY;
        sd_test_delay = 0U;
      }
    }
    else
    {
      /* Self-test OK: settle ~500 ms before FatFs mounts. */
      sd_card_present = 1U;
      if (sd_fatfs_ready == 0U)
      {
        if (sd_fatfs_settle < 50U)
        {
          sd_fatfs_settle++;
        }
        else
        {
          sd_fatfs_ready = 1U;
        }
      }
    }

    /* Edge-trigger debug logs only - keeps output clean. */
    if (sd_card_present != sd_prev_present)
    {
      if (sd_card_present != 0U) { DBG_I("SD", "card inserted"); }
      else                       { DBG_I("SD", "card removed"); }
      sd_prev_present = sd_card_present;
    }
    if (sd_test_result != sd_prev_result)
    {
      if (sd_test_result == HAL_OK)
      {
        DBG_I("SD", "self-test ok");
      }
      else if (sd_test_result != HAL_BUSY)
      {
        DBG_E("SD", "self-test fail rc=%d step=%u hal_err=0x%lX",
              (int)sd_test_result, (unsigned)sd_test_fail_step,
              (unsigned long)sd_hal_error);
      }
      sd_prev_result = sd_test_result;
    }

    /* Once the basic SD self-test has passed for the currently-inserted
     * card, drop a dummy ASCII payload at SD_DUMMY_SECTOR_NUM so the user
     * can verify the write off-line on a Windows PC (HxD or similar). The
     * payload includes uptime/clock/capacity so each run is distinguishable
     * by inspection. Done once per insertion - sd_dummy_done is cleared
     * when the card is removed, above. */
    /* Never raw-write a fixed LBA after FatFs is in the build — that can
     * corrupt a FAT volume. Use session EDF files for SD verification. */
    if ((sd_test_result == HAL_OK) && (sd_dummy_done == 0U))
    {
      sd_dummy_done = 1U;
    }

    RtcClock_Service();

    /* FatFs session logger — never runs in SensorTask / Therapy_Start. */
    SessionEdf_Service();
    SessionSummary_Service();   /* #57: SUMMARY.CSV */
    Adherence_Service();        /* #65: from SUMMARY.CSV */
    FaultLog_Service();         /* #86: drive faults + FAULTS.CSV */

    if (sdram_test_step != 100U)
    {
      if (sdram_test_delay < 1000U)
      {
        sdram_test_delay++;
      }
      else
      {
        sdram_test_result = SDRAM_RunBasicTest();
      }
    }

    if (qspi_test_step != 100U)
    {
      if (qspi_test_delay < 1000U)
      {
        qspi_test_delay++;
      }
      else
      {
        qspi_test_result = QSPI_RunBasicTest();
      }
    }

    osDelay(10U);
  }
}

/* USER CODE END 4 */

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* init code for USB_DEVICE */
  MX_USB_DEVICE_Init();
  /* USER CODE BEGIN 5 */
  /* Infinite loop */
  for(;;)
  {
    osDelay(1);
  }
  /* USER CODE END 5 */
}

 /* MPU Configuration */

void MPU_Config(void)
{
  MPU_Region_InitTypeDef MPU_InitStruct = {0};

  /* Disables the MPU */
  HAL_MPU_Disable();

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Enable = MPU_REGION_ENABLE;
  MPU_InitStruct.Number = MPU_REGION_NUMBER0;
  MPU_InitStruct.BaseAddress = 0x0;
  MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
  MPU_InitStruct.SubRegionDisable = 0xC7;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER1;
  MPU_InitStruct.BaseAddress = SDRAM_DEVICE_ADDR;
  MPU_InitStruct.Size = MPU_REGION_SIZE_32MB;
  MPU_InitStruct.SubRegionDisable = 0x0;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** Initializes and configures the Region and the memory to be protected
  */
  MPU_InitStruct.Number = MPU_REGION_NUMBER2;
  MPU_InitStruct.BaseAddress = LCD_FRAME_BUFFER_ADDR;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512KB;
  MPU_InitStruct.SubRegionDisable = 0x0U;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** CM7 AXI SRAM (512 KB at 0x24000000). */
  MPU_InitStruct.Number = MPU_REGION_NUMBER3;
  MPU_InitStruct.BaseAddress = 0x24000000U;
  MPU_InitStruct.Size = MPU_REGION_SIZE_512KB;
  MPU_InitStruct.SubRegionDisable = 0x0U;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);

  /** D3 SRAM4 shared blower IPC mailbox — non-cacheable for CM4 visibility. */
  MPU_InitStruct.Number = MPU_REGION_NUMBER4;
  MPU_InitStruct.BaseAddress = 0x38000000U;
  MPU_InitStruct.Size = MPU_REGION_SIZE_1KB;
  MPU_InitStruct.SubRegionDisable = 0x0U;
  MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
  MPU_InitStruct.AccessPermission = MPU_REGION_FULL_ACCESS;
  MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
  MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
  MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
  MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

  HAL_MPU_ConfigRegion(&MPU_InitStruct);
  /* Enables the MPU */
  HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);

}

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM1 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  /* USER CODE BEGIN Callback 0 */

  /* USER CODE END Callback 0 */
  if (htim->Instance == TIM1)
  {
    HAL_IncTick();
  }
  /* USER CODE BEGIN Callback 1 */

  /* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  Watchdog_Note(WDG_NOTE_ERROR, boot_stage, (uint32_t)__builtin_return_address(0), 0U);
  boot_stage = 0xEEU;
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
