/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
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
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>
#include <stdlib.h>

#include "keys.h"
#include "robot_movement.h"
#include "bno055.h"
#include "tssp_helper.h"
#include "line_sensor.h"
#include "helpers.h"
#include "ssd1306.h"
#include "I2Cdev.h"
#include "MPU6050.h"
#include "camera.h"
#include "srf_helper.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
volatile TSSP sensors[16];
volatile SRF left_srf, right_srf, back_srf;
volatile BALL ball;
GOAL goal;
Robot robot;
uint16_t width_temp[16][AVERAGE_DATA_NUMBER] = {0}; //? This array use for avarage filter from pulses
uint8_t tx_buff[100];                               //? Data to send with uart
bool line_sensors[20] = {0};                        //? NJL sensors status that sees the line(1) or not(0)

uint8_t openmv_data[OPENMV_DATA_LENGTH] = {0};

float GyroZ_Filtered = 0, HeadPID_i = 0, HeadPIDLastError = 0, HeadPID_Out = 0;

//* Tasks *//
volatile uint16_t Task1ms = 0, Task4ms = 0, Task10ms = 0, Task25ms = 0, Task50ms = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/**
 * @brief  The application entry point.
 * @retval int
 */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_I2C2_Init();
  MX_UART4_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_TIM3_Init();
  MX_TIM5_Init();
  MX_TIM8_Init();
  MX_TIM12_Init();
  MX_TIM4_Init();
  MX_TIM7_Init();
  MX_UART5_Init();
  MX_SPI1_Init();
  MX_TIM13_Init();
  /* USER CODE BEGIN 2 */
  robot.role = attacker;

  // if (robot.role == attacker)
  // {
  HAL_UART_Receive_DMA(&huart5, openmv_data, OPENMV_DATA_LENGTH);
  // }

  LL_mDelay(100);
  MPU6050_Init();
  LL_mDelay(100);
  MPU6050_Calibration();
  start_timers();
  LL_mDelay(500); //! Wait for BNO055 to be ready(Boot time)
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  TurnOnLED();
  MOTORS_DISABLE();

  //! Wait to start the task
  while (1)
  {
    if (START_KEY_STATUS())
    {
      LL_mDelay(5);
      while (START_KEY_STATUS())
      {
        TurnOnLED();
      }
      TurnOffLED();
      break;
    }
  }

  MOTORS_ENABLE();

  while (1)
  {
    uart_error_handler();

    measure_ball_data(sensors, &ball);
    get_ball(&ball);

    if (CAPTURED_BALL_STATUS())
    {
      robot.captured_ball = true;
      robot.captured_ball_time = 0;
    }
    else if (robot.captured_ball_time > CAPTURE_BALL_TIMEOUT)
    {
      robot.captured_ball = false;
    }

    //* Out code
    update_out_data();

    //? Update robot movement
    // TODO: It must change
    //! Shit condition
    if (!robot.line_detect && (!robot.in_out_area || abs(robot.out_angle - robot.get_ball_move_angle) > 45))
    {
      robot.in_out_area = false;
      robot.move_angle = robot.get_ball_move_angle;
      robot.percent_speed = robot.get_ball_percent_speed;
    }

    if (Task1ms > 0)
    {
      update_robot_angle();

      Task1ms -= 1;
    }

    if (Task4ms > 3)
    {
      int16_t GyroZ = Read_MPU6050();
      GyroZ_Filtered = (GyroZ_Filtered * 0.70) + (((float)GyroZ / 65.5) * 0.30);

      //? Head PID
      float HeadPID_Setpoint = -robot.angle + robot.head_angle;
      float HeadPIDError;
      HeadPIDError = -HeadPID_Setpoint;
      HeadPIDError += (GyroZ_Filtered / 5);
      HeadPID_i += HeadPIDError * HEAD_KI;
      if (HeadPID_i > HEAD_PID_I_MAX)
        HeadPID_i = HEAD_PID_I_MAX;
      else if (HeadPID_i < HEAD_PID_I_MAX * -1)
        HeadPID_i = HEAD_PID_I_MAX * -1;
      else
        HeadPID_i = 0;

      float hpo = 0;
      hpo = (HeadPIDError * HEAD_KP) + (HeadPID_i) + ((HeadPIDError - HeadPIDLastError) * HEAD_KD);
      if (HeadPID_Out > HEAD_PID_MAX)
        hpo = HEAD_PID_MAX;
      else if (HeadPID_Out < HEAD_PID_MAX * -1)
        hpo = HEAD_PID_MAX * -1;
      HeadPID_Out = hpo;
      HeadPIDLastError = HeadPIDError;

      if (!robot.must_brake)
      {
        robot_move(robot.move_angle, robot.percent_speed);
      }
      else
      {
        robot_brake(25);
      }

      Task4ms -= 4;
    }

    if (Task10ms > 9)
    {
      if (robot.captured_ball)
      {
        update_head_angle();
      }
      else
      {
        robot.head_angle = 0;
      }

      Task10ms -= 10;
    }

    if (Task50ms > 49)
    {
      if (robot.captured_ball)
      {
        ToggleLED();
      }
      else
      {
        TurnOffLED();
      }

      Task50ms -= 50;
    }
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
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

  /** Configure the main internal regulator output voltage
   */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
   * in the RCC_OscInitTypeDef structure.
   */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
   */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }

  /** Enables the Clock Security System
   */
  HAL_RCC_EnableCSS();
}

/* USER CODE BEGIN 4 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == UART5)
  {
    robot.camera_refresh_time = 0;
    if (robot.role == attacker && robot.camera_connection)
    {
      read_openmv();
    }
  }
}
/* USER CODE END 4 */

/**
 * @brief  This function is executed in case of error occurrence.
 * @retval None
 */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
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
