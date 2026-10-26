/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    stm32h7xx_it.c
  * @brief   Interrupt Service Routines.
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
#include "stm32h7xx_it.h"
#include "cm7_cpu_config.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "watchdog.h"   /* #80: leave the reason before IWDG1 resets */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN TD */

/* USER CODE END TD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN PV */
volatile uint32_t hf_cfsr = 0U;
volatile uint32_t hf_hfsr = 0U;
volatile uint32_t hf_bfar = 0U;
volatile uint32_t hf_mmfar = 0U;
volatile uint32_t hf_ufsr = 0U;
volatile uint32_t hf_boot_stage = 0U;
volatile uint32_t hf_pc = 0U;
volatile uint32_t hf_lr = 0U;
volatile uint32_t hf_cpacr = 0U;
volatile uint32_t hf_ccr = 0U;
volatile uint32_t hf_this = 0U;
volatile uint32_t hf_sp = 0U;
volatile uint32_t hf_fault_addr = 0U;
volatile uint32_t hf_unalign_recoveries = 0U;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
uint32_t ulRotSwIntrCnt = 0;

#define CM7_STACK_BASE 0x20000000U
#define CM7_STACK_LIMIT 0x20020000U

/* FreeRTOS task stacks come from ucHeap in AXI SRAM. */
#define CM7_AXI_SRAM_BASE 0x24000000U
#define CM7_AXI_SRAM_LIMIT 0x24080000U

static int FaultStackPointerValid(uint32_t sp)
{
  return (sp >= (CM7_STACK_BASE + 32U)) && (sp < CM7_STACK_LIMIT) && ((sp & 3U) == 0U);
}

/* Wider than FaultStackPointerValid (which still gates the unaligned-access
 * emulation): only for reading the stacked PC / LR of a fault in a task. */
static int FaultStackReadable(uint32_t sp)
{
  return FaultStackPointerValid(sp)
      || ((sp >= CM7_AXI_SRAM_BASE) && (sp <= (CM7_AXI_SRAM_LIMIT - 32U)) && ((sp & 3U) == 0U));
}

static void FaultCaptureStack(void)
{
  uint32_t *sp;

  __asm volatile(
      "tst lr, #4 \n"
      "ite eq \n"
      "mrseq %0, msp \n"
      "mrsne %0, psp \n"
      : "=r"(sp)
      :
      : "cc");

  hf_sp = (uint32_t)sp;
  hf_cpacr = SCB->CPACR;
  hf_ccr = SCB->CCR;

  if (FaultStackReadable(hf_sp))
  {
    hf_pc = sp[6];
    hf_lr = sp[5];
  }
  else
  {
    hf_pc = 0U;
    hf_lr = 0U;
    boot_stage = 0xFCU;
  }
}

static void FaultCaptureCoreRegs(uint32_t *regs)
{
  __asm volatile(
      "mov %0, r4 \n"
      "mov %1, r5 \n"
      "mov %2, r6 \n"
      "mov %3, r7 \n"
      "mov %4, r8 \n"
      "mov %5, r9 \n"
      "mov %6, r10\n"
      "mov %7, r11\n"
      : "=r"(regs[4]), "=r"(regs[5]), "=r"(regs[6]), "=r"(regs[7]),
        "=r"(regs[8]), "=r"(regs[9]), "=r"(regs[10]), "=r"(regs[11])
      :
      : "memory");
}

static int FaultEmulateUnalignedHalfword(uint32_t *exc_stack, const uint32_t *core_regs)
{
  uint32_t pc;
  uint16_t hw1;
  uint16_t hw2;
  uint32_t rn;
  uint32_t rt;
  uint32_t imm12;
  uint32_t addr;
  uint32_t val;
  volatile uint8_t *p;
  uint32_t regs[13];

  pc = exc_stack[6];
  hw2 = *(uint16_t const *)pc;
  hw1 = *(uint16_t const *)(pc + 2U);

  /* STRH.W Rt, [Rn, #imm12] (Thumb-2, T3) */
  if ((hw1 & 0xFFF0U) != 0xF8A0U)
  {
    return 0;
  }

  rn = hw1 & 0xFU;
  rt = (hw2 >> 12U) & 0xFU;
  imm12 = hw2 & 0xFFFU;
  if (rt >= 13U)
  {
    return 0;
  }

  regs[0] = exc_stack[0];
  regs[1] = exc_stack[1];
  regs[2] = exc_stack[2];
  regs[3] = exc_stack[3];
  regs[4] = core_regs[4];
  regs[5] = core_regs[5];
  regs[6] = core_regs[6];
  regs[7] = core_regs[7];
  regs[8] = core_regs[8];
  regs[9] = core_regs[9];
  regs[10] = core_regs[10];
  regs[11] = core_regs[11];
  regs[12] = exc_stack[4];

  if (rn == 13U)
  {
    addr = (uint32_t)exc_stack + 32U + imm12;
  }
  else if (rn < 13U)
  {
    addr = regs[rn] + imm12;
  }
  else
  {
    return 0;
  }

  val = regs[rt] & 0xFFFFU;
  p = (volatile uint8_t *)addr;
  p[0] = (uint8_t)(val & 0xFFU);
  p[1] = (uint8_t)((val >> 8) & 0xFFU);

  hf_this = regs[6];
  hf_fault_addr = addr;
  exc_stack[6] = pc + 4U;
  return 1;
}
/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern PCD_HandleTypeDef hpcd_USB_OTG_FS;
extern TIM_HandleTypeDef htim1;

/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
  * @brief This function handles Non maskable interrupt.
  */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  /* USER CODE BEGIN NonMaskableInt_IRQn 1 */
   while (1)
  {
  }
  /* USER CODE END NonMaskableInt_IRQn 1 */
}

/**
  * @brief This function handles Hard fault interrupt.
  */
void HardFault_Handler(void)
{
  /* USER CODE BEGIN HardFault_IRQn 0 */
  FaultCaptureStack();
  hf_cfsr = SCB->CFSR;
  hf_hfsr = SCB->HFSR;
  hf_bfar = SCB->BFAR;
  hf_mmfar = SCB->MMFAR;
  hf_ufsr = (SCB->CFSR >> 16U) & 0xFFFFU;
  hf_boot_stage = boot_stage;
  boot_stage = 0xFFU;
  Watchdog_Note(WDG_NOTE_FAULT, 'H', hf_pc, hf_cfsr);
  /* USER CODE END HardFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_HardFault_IRQn 0 */
    /* USER CODE END W1_HardFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Memory management fault.
  */
void MemManage_Handler(void)
{
  /* USER CODE BEGIN MemoryManagement_IRQn 0 */
  hf_cfsr = SCB->CFSR;
  hf_mmfar = SCB->MMFAR;
  hf_boot_stage = boot_stage;
  boot_stage = 0xFEU;
  Watchdog_Note(WDG_NOTE_FAULT, 'M', 0U, hf_cfsr);
  /* USER CODE END MemoryManagement_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_MemoryManagement_IRQn 0 */
    /* USER CODE END W1_MemoryManagement_IRQn 0 */
  }
}

/**
  * @brief This function handles Pre-fetch fault, memory access fault.
  */
void BusFault_Handler(void)
{
  /* USER CODE BEGIN BusFault_IRQn 0 */
  Watchdog_Note(WDG_NOTE_FAULT, 'B', 0U, SCB->CFSR);
  /* USER CODE END BusFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_BusFault_IRQn 0 */
    /* USER CODE END W1_BusFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Undefined instruction or illegal state.
  */
void UsageFault_Handler_C(void)
{
  uint32_t *exc_stack;
  uint32_t core_regs[13] = {0U};

  /* USER CODE BEGIN UsageFault_IRQn 0 */
  __asm volatile(
      "tst lr, #4 \n"
      "ite eq \n"
      "mrseq %0, msp \n"
      "mrsne %0, psp \n"
      : "=r"(exc_stack)
      :
      : "cc");
  FaultCaptureCoreRegs(core_regs);

  hf_sp = (uint32_t)exc_stack;
  hf_cpacr = SCB->CPACR;
  hf_ccr = SCB->CCR;
  if (FaultStackReadable(hf_sp))
  {
    hf_pc = exc_stack[6];
    hf_lr = exc_stack[5];
  }
  else
  {
    hf_pc = 0U;
    hf_lr = 0U;
    boot_stage = 0xFCU;
  }
  hf_cfsr = SCB->CFSR;
  hf_hfsr = SCB->HFSR;
  hf_bfar = SCB->BFAR;
  hf_mmfar = SCB->MMFAR;
  hf_ufsr = (SCB->CFSR >> 16U) & 0xFFFFU;
  hf_boot_stage = boot_stage;

  if ((hf_ufsr & 0x0100U) != 0U)
  {
    if (FaultStackPointerValid((uint32_t)exc_stack) &&
        (FaultEmulateUnalignedHalfword(exc_stack, core_regs) != 0))
    {
      hf_unalign_recoveries++;
      SCB->CFSR = (0xFFFFUL << 16U);
      __DSB();
      __ISB();
      return;
    }
  }

  boot_stage = 0xFDU;
  Watchdog_Note(WDG_NOTE_FAULT, 'U', hf_pc, hf_cfsr);
  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_IRQn 0 */
  }
}

/**
  * @brief This function handles Debug monitor.
  */
void DebugMon_Handler(void)
{
  /* USER CODE BEGIN DebugMonitor_IRQn 0 */

  /* USER CODE END DebugMonitor_IRQn 0 */
  /* USER CODE BEGIN DebugMonitor_IRQn 1 */

  /* USER CODE END DebugMonitor_IRQn 1 */
}

/******************************************************************************/
/* STM32H7xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32h7xx.s).                    */
/******************************************************************************/

/**
  * @brief This function handles EXTI line[9:5] interrupts.
  */
void EXTI9_5_IRQHandler(void)
{
  /* USER CODE BEGIN EXTI9_5_IRQn 0 */
  /* Do NOT increment ulRotSwIntrCnt here - EXTI9_5 is shared between
   * ROT_SW (PB5, line 5) and NFC_INT (PI8, line 8). Incrementing here
   * also counts every NFC IRQ as a knob press. The per-pin counter
   * lives in HAL_GPIO_EXTI_Callback below where we can gate on
   * GPIO_Pin == ROT_SW_Pin. */
  /* USER CODE END EXTI9_5_IRQn 0 */
  HAL_GPIO_EXTI_IRQHandler(ROT_SW_Pin);
  HAL_GPIO_EXTI_IRQHandler(NFC_INT_Pin);
  /* USER CODE BEGIN EXTI9_5_IRQn 1 */

  /* USER CODE END EXTI9_5_IRQn 1 */
}

/**
  * @brief This function handles TIM1 update interrupt.
  */
void TIM1_UP_IRQHandler(void)
{
  /* USER CODE BEGIN TIM1_UP_IRQn 0 */

  /* USER CODE END TIM1_UP_IRQn 0 */
  HAL_TIM_IRQHandler(&htim1);
  /* USER CODE BEGIN TIM1_UP_IRQn 1 */

  /* USER CODE END TIM1_UP_IRQn 1 */
}

/**
  * @brief This function handles USB OTG FS global interrupt.
  */
void OTG_FS_IRQHandler(void)
{
  /* USER CODE BEGIN OTG_FS_IRQn 0 */

  /* USER CODE END OTG_FS_IRQn 0 */
  HAL_PCD_IRQHandler(&hpcd_USB_OTG_FS);
  /* USER CODE BEGIN OTG_FS_IRQn 1 */

  /* USER CODE END OTG_FS_IRQn 1 */
}

/* USER CODE BEGIN 1 */
#include "rfal_platform.h"

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if (GPIO_Pin == NFC_INT_Pin)
  {
    platformNfcIrqHandler();
  }
  else if (GPIO_Pin == ROT_SW_Pin)
  {
    /* The rotary push-switch generates two edges per physical click
     * (press-down + release), so this counter advances by 2 per click.
     * rotary_input.c divides by 2 to derive the press count. */
    ulRotSwIntrCnt = ulRotSwIntrCnt + 1U;
  }
}

void LTDC_IRQHandler(void)
{
  HAL_LTDC_IRQHandler(&hltdc);
}

/* USER CODE END 1 */
