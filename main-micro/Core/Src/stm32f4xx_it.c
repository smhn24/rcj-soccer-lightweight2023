/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    stm32f4xx_it.c
 * @brief   Interrupt Service Routines.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
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
#include "stm32f4xx_it.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "tssp_helper.h"
#include "robot_movement.h"
#include "keys.h"
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
// extern volatile Robot robot;
extern Robot robot;
extern TSSP sensors[16];
extern uint16_t width_temp[16][AVERAGE_DATA_NUMBER];
extern uint16_t Task1ms, Task4ms, Task10ms, Task50ms;
// extern uint8_t Task4ms;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/* External variables --------------------------------------------------------*/
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;
extern TIM_HandleTypeDef htim5;
extern TIM_HandleTypeDef htim7;
extern TIM_HandleTypeDef htim8;
extern TIM_HandleTypeDef htim12;
extern DMA_HandleTypeDef hdma_uart4_tx;
extern DMA_HandleTypeDef hdma_uart5_rx;
extern UART_HandleTypeDef huart4;
extern UART_HandleTypeDef huart5;
/* USER CODE BEGIN EV */

/* USER CODE END EV */

/******************************************************************************/
/*           Cortex-M4 Processor Interruption and Exception Handlers          */
/******************************************************************************/
/**
 * @brief This function handles Non maskable interrupt.
 */
void NMI_Handler(void)
{
  /* USER CODE BEGIN NonMaskableInt_IRQn 0 */

  /* USER CODE END NonMaskableInt_IRQn 0 */
  HAL_RCC_NMI_IRQHandler();
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
  TurnOnLED();
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
void UsageFault_Handler(void)
{
  /* USER CODE BEGIN UsageFault_IRQn 0 */

  /* USER CODE END UsageFault_IRQn 0 */
  while (1)
  {
    /* USER CODE BEGIN W1_UsageFault_IRQn 0 */
    /* USER CODE END W1_UsageFault_zIRQn 0 */
  }
}

/**
 * @brief This function handles System service call via SWI instruction.
 */
void SVC_Handler(void)
{
  /* USER CODE BEGIN SVCall_IRQn 0 */

  /* USER CODE END SVCall_IRQn 0 */
  /* USER CODE BEGIN SVCall_IRQn 1 */

  /* USER CODE END SVCall_IRQn 1 */
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

/**
 * @brief This function handles Pendable request for system service.
 */
void PendSV_Handler(void)
{
  /* USER CODE BEGIN PendSV_IRQn 0 */

  /* USER CODE END PendSV_IRQn 0 */
  /* USER CODE BEGIN PendSV_IRQn 1 */

  /* USER CODE END PendSV_IRQn 1 */
}

/**
 * @brief This function handles System tick timer.
 */
void SysTick_Handler(void)
{
  /* USER CODE BEGIN SysTick_IRQn 0 */

  /* USER CODE END SysTick_IRQn 0 */
  HAL_IncTick();
  /* USER CODE BEGIN SysTick_IRQn 1 */

  /* USER CODE END SysTick_IRQn 1 */
}

/******************************************************************************/
/* STM32F4xx Peripheral Interrupt Handlers                                    */
/* Add here the Interrupt Handlers for the used peripherals.                  */
/* For the available peripheral interrupt handler names,                      */
/* please refer to the startup file (startup_stm32f4xx.s).                    */
/******************************************************************************/

/**
 * @brief This function handles DMA1 stream0 global interrupt.
 */
void DMA1_Stream0_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream0_IRQn 0 */

  /* USER CODE END DMA1_Stream0_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart5_rx);
  /* USER CODE BEGIN DMA1_Stream0_IRQn 1 */

  /* USER CODE END DMA1_Stream0_IRQn 1 */
}

/**
 * @brief This function handles DMA1 stream4 global interrupt.
 */
void DMA1_Stream4_IRQHandler(void)
{
  /* USER CODE BEGIN DMA1_Stream4_IRQn 0 */

  /* USER CODE END DMA1_Stream4_IRQn 0 */
  HAL_DMA_IRQHandler(&hdma_uart4_tx);
  /* USER CODE BEGIN DMA1_Stream4_IRQn 1 */

  /* USER CODE END DMA1_Stream4_IRQn 1 */
}

/**
 * @brief This function handles TIM2 global interrupt.
 */
void TIM2_IRQHandler(void)
{
  /* USER CODE BEGIN TIM2_IRQn 0 */
  if (TIM2->SR & TIM_SR_CC1IF) //! Timer 2 channel 1 (TSSP7)
  {
    if (TIM2->CCER & TIM_CCER_CC1P)
    {
      sensors[7].start_time = TIM2->CCR1;
      sensors[7].timeout = 0;
      TIM2->CCER &= ~TIM_CCER_CC1P;
    }
    else
    {
      sensors[7].end_time = TIM2->CCR1;
      update_sensor(7, &sensors[7]);
      TIM2->CCER |= TIM_CCER_CC1P;
    }
    TIM2->SR &= ~TIM_SR_CC1IF; //? Clear CC1P interrupt flag
  }

  if (TIM2->SR & TIM_SR_CC2IF) //! Timer 2 channel 2 (TSSP8)
  {
    if (TIM2->CCER & TIM_CCER_CC2P)
    {
      sensors[8].start_time = TIM2->CCR2;
      sensors[8].timeout = 0;
      TIM2->CCER &= ~TIM_CCER_CC2P;
    }
    else
    {
      sensors[8].end_time = TIM2->CCR2;
      update_sensor(8, &sensors[8]);
      TIM2->CCER |= TIM_CCER_CC2P;
    }
    TIM2->SR &= ~TIM_SR_CC2IF; //? Clear CC2P interrupt flag
  }

  if (TIM2->SR & TIM_SR_CC3IF) //! Timer 2 channel 3 (TSSP4)
  {
    if (TIM2->CCER & TIM_CCER_CC3P)
    {
      sensors[4].start_time = TIM2->CCR3;
      sensors[4].timeout = 0;
      TIM2->CCER &= ~TIM_CCER_CC3P;
    }
    else
    {
      sensors[4].end_time = TIM2->CCR3;
      update_sensor(4, &sensors[4]);
      TIM2->CCER |= TIM_CCER_CC3P;
    }
    TIM2->SR &= ~TIM_SR_CC3IF; //? Clear CC3P interrupt flag
  }

  if (TIM2->SR & TIM_SR_CC4IF) //! Timer 2 channel 4 (TSSP11)
  {
    if (TIM2->CCER & TIM_CCER_CC4P)
    {
      sensors[11].start_time = TIM2->CCR4;
      sensors[11].timeout = 0;
      TIM2->CCER &= ~TIM_CCER_CC4P;
    }
    else
    {
      sensors[11].end_time = TIM2->CCR4;
      update_sensor(11, &sensors[11]);
      TIM2->CCER |= TIM_CCER_CC4P;
    }
    TIM2->SR &= ~TIM_SR_CC4IF; //? Clear CC3P interrupt flag
  }

  TIM2->SR &= ~TIM_SR_UIF; //? Clear timer interrupt flag
  /* USER CODE END TIM2_IRQn 0 */
  HAL_TIM_IRQHandler(&htim2);
  /* USER CODE BEGIN TIM2_IRQn 1 */

  /* USER CODE END TIM2_IRQn 1 */
}

/**
 * @brief This function handles TIM3 global interrupt.
 */
void TIM3_IRQHandler(void)
{
  /* USER CODE BEGIN TIM3_IRQn 0 */
  if (TIM3->SR & TIM_SR_CC1IF) //! Timer 3 channel 1 (TSSP6)
  {
    if (TIM3->CCER & TIM_CCER_CC1P)
    {
      sensors[6].start_time = TIM3->CCR1;
      sensors[6].timeout = 0;
      TIM3->CCER &= ~TIM_CCER_CC1P;
    }
    else
    {
      sensors[6].end_time = TIM3->CCR1;
      update_sensor(6, &sensors[6]);
      TIM3->CCER |= TIM_CCER_CC1P;
    }
    TIM3->SR &= ~TIM_SR_CC1IF; //? Clear CC1P interrupt flag
  }

  if (TIM3->SR & TIM_SR_CC2IF) //! Timer 3 channel 2 (TSSP9)
  {
    if (TIM3->CCER & TIM_CCER_CC2P)
    {
      sensors[9].start_time = TIM3->CCR2;
      sensors[9].timeout = 0;
      TIM3->CCER &= ~TIM_CCER_CC2P;
    }
    else
    {
      sensors[9].end_time = TIM3->CCR2;
      update_sensor(9, &sensors[9]);
      TIM3->CCER |= TIM_CCER_CC2P;
    }
    TIM3->SR &= ~TIM_SR_CC2IF; //? Clear CC2P interrupt flag
  }

  if (TIM3->SR & TIM_SR_CC3IF) //! Timer 3 channel 3 (TSSP3)
  {
    if (TIM3->CCER & TIM_CCER_CC3P)
    {
      sensors[3].start_time = TIM3->CCR3;
      sensors[3].timeout = 0;
      TIM3->CCER &= ~TIM_CCER_CC3P;
    }
    else
    {
      sensors[3].end_time = TIM3->CCR3;
      update_sensor(3, &sensors[3]);
      TIM3->CCER |= TIM_CCER_CC3P;
    }
    TIM3->SR &= ~TIM_SR_CC3IF; //? Clear CC3P interrupt flag
  }

  if (TIM3->SR & TIM_SR_CC4IF) //! Timer 3 channel 4 (TSSP12)
  {
    if (TIM3->CCER & TIM_CCER_CC4P)
    {
      sensors[12].start_time = TIM3->CCR4;
      sensors[12].timeout = 0;
      TIM3->CCER &= ~TIM_CCER_CC4P;
    }
    else
    {
      sensors[12].end_time = TIM3->CCR4;
      update_sensor(12, &sensors[12]);
      TIM3->CCER |= TIM_CCER_CC4P;
    }
    TIM3->SR &= ~TIM_SR_CC4IF; //? Clear CC4P interrupt flag
  }

  TIM3->SR &= ~TIM_SR_UIF; //? Clear timer interrupt flag
  /* USER CODE END TIM3_IRQn 0 */
  HAL_TIM_IRQHandler(&htim3);
  /* USER CODE BEGIN TIM3_IRQn 1 */

  /* USER CODE END TIM3_IRQn 1 */
}

/**
 * @brief This function handles TIM4 global interrupt.
 */
void TIM4_IRQHandler(void)
{
  /* USER CODE BEGIN TIM4_IRQn 0 */
  for (uint8_t i = 0; i < 16; i++)
  {
    sensors[i].timeout++;
    if (sensors[i].timeout > TSSP_MAX_TIMEOUT)
    {
      sensors[i].start_time = 0;
      sensors[i].end_time = 0;
      update_sensor(i, &sensors[i]);
      sensors[i].timeout = 0;
    }
  }
  /* USER CODE END TIM4_IRQn 0 */
  HAL_TIM_IRQHandler(&htim4);
  /* USER CODE BEGIN TIM4_IRQn 1 */

  /* USER CODE END TIM4_IRQn 1 */
}

/**
 * @brief This function handles TIM8 break interrupt and TIM12 global interrupt.
 */
void TIM8_BRK_TIM12_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 0 */
  if (TIM12->SR & TIM_SR_CC1IF) //! Timer 12 channel 1 (TSSP2)
  {
    if (TIM12->CCER & TIM_CCER_CC1P)
    {
      sensors[2].start_time = TIM12->CCR1;
      sensors[2].timeout = 0;
      TIM12->CCER &= ~TIM_CCER_CC1P;
    }
    else
    {
      sensors[2].end_time = TIM12->CCR1;
      update_sensor(2, &sensors[2]);
      TIM12->CCER |= TIM_CCER_CC1P;
    }
    TIM12->SR &= ~TIM_SR_CC1IF; //? Clear CC1P interrupt flag
  }

  if (TIM12->SR & TIM_SR_CC2IF) //! Timer 12 channel 2 (TSSP13)
  {
    if (TIM12->CCER & TIM_CCER_CC2P)
    {
      sensors[13].start_time = TIM12->CCR2;
      sensors[13].timeout = 0;
      TIM12->CCER &= ~TIM_CCER_CC2P;
    }
    else
    {
      sensors[13].end_time = TIM12->CCR2;
      update_sensor(13, &sensors[13]);
      TIM12->CCER |= TIM_CCER_CC2P;
    }
    TIM12->SR &= ~TIM_SR_CC2IF; //? Clear CC2P interrupt flag
  }

  TIM12->SR &= ~TIM_SR_UIF; //? Clear timer interrupt flag
  /* USER CODE END TIM8_BRK_TIM12_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  HAL_TIM_IRQHandler(&htim12);
  /* USER CODE BEGIN TIM8_BRK_TIM12_IRQn 1 */

  /* USER CODE END TIM8_BRK_TIM12_IRQn 1 */
}

/**
 * @brief This function handles TIM8 update interrupt and TIM13 global interrupt.
 */
void TIM8_UP_TIM13_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 0 */

  /* USER CODE END TIM8_UP_TIM13_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  /* USER CODE BEGIN TIM8_UP_TIM13_IRQn 1 */

  /* USER CODE END TIM8_UP_TIM13_IRQn 1 */
}

/**
 * @brief This function handles TIM8 capture compare interrupt.
 */
void TIM8_CC_IRQHandler(void)
{
  /* USER CODE BEGIN TIM8_CC_IRQn 0 */
  if (TIM8->SR & TIM_SR_CC1IF) //! Timer 8 channel 1 (TSSP1)
  {
    if (TIM8->CCER & TIM_CCER_CC1P)
    {
      sensors[1].start_time = TIM8->CCR1;
      sensors[1].timeout = 0;
      TIM8->CCER &= ~TIM_CCER_CC1P;
    }
    else
    {
      sensors[1].end_time = TIM8->CCR1;
      update_sensor(1, &sensors[1]);
      TIM8->CCER |= TIM_CCER_CC1P;
    }
    TIM8->SR &= ~TIM_SR_CC1IF; //? Clear CC1P interrupt flag
  }

  if (TIM8->SR & TIM_SR_CC2IF) //! Timer 8 channel 2 (TSSP14)
  {
    if (TIM8->CCER & TIM_CCER_CC2P)
    {
      sensors[14].start_time = TIM8->CCR2;
      sensors[14].timeout = 0;
      TIM8->CCER &= ~TIM_CCER_CC2P;
    }
    else
    {
      sensors[14].end_time = TIM8->CCR2;
      update_sensor(14, &sensors[14]);
      TIM8->CCER |= TIM_CCER_CC2P;
    }
    TIM8->SR &= ~TIM_SR_CC2IF; //? Clear CC2P interrupt flag
  }

  if (TIM8->SR & TIM_SR_CC3IF) //! Timer 8 channel 3 (TSSP0)
  {
    if (TIM8->CCER & TIM_CCER_CC3P)
    {
      sensors[0].start_time = TIM8->CCR3;
      sensors[0].timeout = 0;
      TIM8->CCER &= ~TIM_CCER_CC3P;
    }
    else
    {
      sensors[0].end_time = TIM8->CCR3;
      update_sensor(0, &sensors[0]);
      TIM8->CCER |= TIM_CCER_CC3P;
    }
    TIM8->SR &= ~TIM_SR_CC3IF; //? Clear CC3P interrupt flag
  }

  if (TIM8->SR & TIM_SR_CC4IF) //! Timer 8 channel 4 (TSSP15)
  {
    if (TIM8->CCER & TIM_CCER_CC4P)
    {
      sensors[15].start_time = TIM8->CCR4;
      sensors[15].timeout = 0;
      TIM8->CCER &= ~TIM_CCER_CC4P;
    }
    else
    {
      sensors[15].end_time = TIM8->CCR4;
      update_sensor(15, &sensors[15]);
      TIM8->CCER |= TIM_CCER_CC4P;
    }
    TIM8->SR &= ~TIM_SR_CC4IF; //? Clear CC4P interrupt flag
  }

  TIM8->SR &= ~TIM_SR_UIF; //? Clear timer interrupt flag
  /* USER CODE END TIM8_CC_IRQn 0 */
  HAL_TIM_IRQHandler(&htim8);
  /* USER CODE BEGIN TIM8_CC_IRQn 1 */

  /* USER CODE END TIM8_CC_IRQn 1 */
}

/**
 * @brief This function handles TIM5 global interrupt.
 */
void TIM5_IRQHandler(void)
{
  /* USER CODE BEGIN TIM5_IRQn 0 */
  if (TIM5->SR & TIM_SR_CC1IF) //! Timer 5 channel 1 (TSSP5)
  {
    if (TIM5->CCER & TIM_CCER_CC1P)
    {
      sensors[5].start_time = TIM5->CCR1;
      sensors[5].timeout = 0;
      TIM5->CCER &= ~TIM_CCER_CC1P;
    }
    else
    {
      sensors[5].end_time = TIM5->CCR1;
      update_sensor(5, &sensors[5]);
      TIM5->CCER |= TIM_CCER_CC1P;
    }
    TIM5->SR &= ~TIM_SR_CC1IF; //? Clear CC1P interrupt flag
  }

  if (TIM5->SR & TIM_SR_CC2IF) //! Timer 5 channel 2 (TSSP10)
  {
    if (TIM5->CCER & TIM_CCER_CC2P)
    {
      sensors[10].start_time = TIM5->CCR2;
      sensors[10].timeout = 0;
      TIM5->CCER &= ~TIM_CCER_CC2P;
    }
    else
    {
      sensors[10].end_time = TIM5->CCR2;
      update_sensor(10, &sensors[10]);
      TIM5->CCER |= TIM_CCER_CC2P;
    }
    TIM5->SR &= ~TIM_SR_CC2IF; //? Clear CC2P interrupt flag
  }

  TIM5->SR &= ~TIM_SR_UIF; //? Clear timer interrupt flag
  /* USER CODE END TIM5_IRQn 0 */
  HAL_TIM_IRQHandler(&htim5);
  /* USER CODE BEGIN TIM5_IRQn 1 */

  /* USER CODE END TIM5_IRQn 1 */
}

/**
 * @brief This function handles UART4 global interrupt.
 */
void UART4_IRQHandler(void)
{
  /* USER CODE BEGIN UART4_IRQn 0 */

  /* USER CODE END UART4_IRQn 0 */
  HAL_UART_IRQHandler(&huart4);
  /* USER CODE BEGIN UART4_IRQn 1 */

  /* USER CODE END UART4_IRQn 1 */
}

/**
 * @brief This function handles UART5 global interrupt.
 */
void UART5_IRQHandler(void)
{
  /* USER CODE BEGIN UART5_IRQn 0 */

  /* USER CODE END UART5_IRQn 0 */
  HAL_UART_IRQHandler(&huart5);
  /* USER CODE BEGIN UART5_IRQn 1 */

  /* USER CODE END UART5_IRQn 1 */
}

/**
 * @brief This function handles TIM7 global interrupt.
 */
void TIM7_IRQHandler(void)
{
  /* USER CODE BEGIN TIM7_IRQn 0 */
  if (robot.on_line_sensors == 0)
  {
    if (robot.green_time < 10000)
    {
      robot.green_time++;
    }
  }
  else if (robot.line_detect)
  {
    robot.green_time = 0;
  }

  robot.camera_connection = (robot.camera_refresh_time < 1000) ? true : false;
  if (robot.camera_refresh_time < 10000)
  {
    robot.camera_refresh_time++;
  }

  if (!CAPTURED_BALL_STATUS() && robot.captured_ball_time < 10000)
  {
    robot.captured_ball_time++;
  }

  Task1ms++;
  Task4ms++;
  Task10ms++;
  Task50ms++;
  /* USER CODE END TIM7_IRQn 0 */
  HAL_TIM_IRQHandler(&htim7);
  /* USER CODE BEGIN TIM7_IRQn 1 */

  /* USER CODE END TIM7_IRQn 1 */
}

/* USER CODE BEGIN 1 */

/* USER CODE END 1 */
