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
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "keys.h"
#include "robot_movement.h"
#include "bno055.h"
#include "tssp_helper.h"
#include "line_sensor.h"
#include "helpers.h"
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
volatile uint8_t tx_buff[100]; //? Data to send with uart
volatile TSSP sensors[16];
volatile BALL ball;
volatile Robot robot;
volatile uint16_t width_temp[16][AVERAGE_DATA_NUMBER] = {0};
volatile uint8_t on_line_sensors = 0; //? Number of sensors sees the line
volatile int out_angle;
volatile bool line_sensors[20] = {0}; //? NJL sensors status that sees the line(1) or not(0)
volatile bool symmetry[20] = {0};
volatile bool out_detect; //? A flag to detect line(true) or not(false)
volatile out_direction_t out_direction;
volatile uint8_t first_out_sensor = 0;
volatile uint8_t out_edges[2];
volatile uint16_t brake_time = 0;
volatile bool is_braking = false;

//* Tasks *//
volatile uint8_t Task1ms = 0, Task5ms = 0, Task10ms = 0, Task50ms = 0;

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
  /* USER CODE BEGIN 2 */

  start_timers();

  LL_mDelay(2000); //! Wait for BNO055 to be ready
  BNO055_Config();
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
      TurnOffLED();
      while (START_KEY_STATUS())
      {
      }
      break;
    }
  }

  // MOTORS_ENABLE();

  Task1ms = 0;
  Task5ms = 0;
  Task10ms = 0;
  Task50ms = 0;

  while (1)
  {
    robot.angle = BNO055_read();

    if (Task1ms)
    {
      Task1ms = 0;
      measure_ball_data(sensors, &ball);
    }

    if (Task5ms > 4)
    {
      Task5ms = 0;
      get_ball(&ball);
      if (is_braking)
      {
        robot_brake(out_angle, 0.5, 20);
      }
      else
      {
        robot_move(robot.move_angle, robot.percent_speed);
      }
    }

    if (Task10ms > 9)
    {
      Task10ms = 0;

      read_line_sensors(line_sensors); //? Update line_sensors array
      on_line_sensors = on_line_sensors_number(line_sensors);

      if (on_line_sensors != 0 && !out_detect) //? At least one sensor detected line for the first time
      {
        for (uint8_t i = 0; i < 20; i++)
        {
          if (line_sensors[i])
          {
            first_out_sensor = i;
            if (i > 8 && i < 14) //? Left
            {
              out_direction = left;
              sprintf(tx_buff, "Left: %d\r\n", i);
            }
            else if (i >= 14 && i <= 18) //? Backward
            {
              out_direction = backward;
              sprintf(tx_buff, "Back: %d\r\n", i);
            }
            else if (i > 18 || i < 4) //? Right
            {
              out_direction = right;
              sprintf(tx_buff, "Right: %d\r\n", i);
            }
            else if (i >= 4 && i <= 8) //? Forward
            {
              out_direction = forward;
              sprintf(tx_buff, "Forward: %d\r\n", i);
            }
            // PRINT_BUFFER();
            out_detect = true;
            break;
          }
        }
      }
      else if (on_line_sensors != 0 && out_detect)
      {
        switch (out_direction)
        {
        case left:
          for (uint8_t i = 16; i >= 11; i--)
          {
            if (line_sensors[i])
            {
              out_edges[0] = i;
              break;
            }
          }
          for (uint8_t i = 6; i <= 11; i++)
          {
            if (line_sensors[i])
            {
              out_edges[1] = i;
              break;
            }
          }
          break;
        case right:
          for (uint8_t i = 16; i <= 20; i++)
          {
            if (i == 20 && line_sensors[0])
            {
              out_edges[0] = 20;
              break;
            }
            else if (line_sensors[i])
            {
              out_edges[0] = i;
              break;
            }
          }
          for (uint8_t i = 6; i >= 1; i--)
          {
            if (line_sensors[i])
            {
              out_edges[1] = i;
              break;
            }
          }
          break;
        case forward:
          for (uint8_t i = 1; i <= 6; i++)
          {
            if (line_sensors[i])
            {
              out_edges[0] = i;
              break;
            }
          }
          for (uint8_t i = 11; i > 6; i--)
          {
            if (line_sensors[i])
            {
              out_edges[1] = i;
              break;
            }
          }
          break;
        case backward:
          for (uint8_t i = 11; i < 16; i++)
          {
            if (line_sensors[i])
            {
              out_edges[0] = i;
              break;
            }
          }

          for (uint8_t i = 16; i < 21; i++)
          {
            if (i == 20 && line_sensors[0])
            {
              out_edges[1] = 20;
              break;
            }

            else if (line_sensors[i])
            {
              out_edges[1] = i;
              break;
            }
          }
          break;

        default:
          break;
        }

        out_angle = (out_edges[0] + out_edges[1]) / 2;
        out_angle *= 18;
        if (out_direction == right)
        {
          out_angle = fabs(out_angle - 180);
        }

        out_angle -= 90;
        if (out_angle < 0)
          out_angle += 360;
        out_angle *= -1;
        out_angle += 360;

        is_braking = true;

        // sprintf(tx_buff, "%d      %d    Angle: %d\r\n", out_edges[0], out_edges[1], out_angle);
        // sprintf(tx_buff, "%d     %d\r\n", out_edges[0], out_edges[1]);
        // PRINT_BUFFER();
      }
      else if (on_line_sensors == 0) //? No sensor sees the line
      {
        out_detect = false;
        out_direction = not_out;
      }
    }

    if (Task50ms > 49)
    {
      Task50ms = 0;

      sprintf(tx_buff, "%d\r\n", robot.angle);
      PRINT_BUFFER();
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
