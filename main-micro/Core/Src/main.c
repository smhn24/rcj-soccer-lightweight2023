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
volatile BALL ball;
Robot robot;
uint16_t width_temp[16][AVERAGE_DATA_NUMBER] = {0}; //? This array use for avarage filter from pulses
uint8_t tx_buff[100];                               //? Data to send with uart
bool line_sensors[20] = {0};                        //? NJL sensors status that sees the line(1) or not(0)

/*
  0: 90, 1: 72, 2: 54, 3: 36, 4: 18, 5: 0, 6: 342, 7: 324, 8: 306, 9: 288, 10: 270
  11: 252, 12: 234, 13: 216, 14: 198, 15: 180, 16: 162, 17: 144, 18: 126, 19: 108
*/
int16_t njl_angle[20] = {90, 72, 54, 36, 18, 0, 342, 324, 306, 288, 270, 252, 234, 216, 198, 180, 162, 144, 126, 108};
float njl_x[20] = {1, 0.95, 0.81, 0.58, 0.31, 0, -0.31, -0.58, -0.81, -0.95, -1, -0.95, -0.81, -0.58, -0.31, 0, 0.31, 0.58, 0.81, 0.95};
float njl_y[20] = {0, 0.31, 0.58, 0.81, 0.95, 1, 0.95, 0.81, 0.58, 0.31, 0, -0.31, -0.58, -0.81, -0.95, -1, -0.95, -0.81, -0.58, -0.31};

//* Tasks *//
volatile uint16_t Task1ms = 0, Task5ms = 0, Task10ms = 0, Task50ms = 0;
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
void update_robot_angle()
{
  static bool ignore = false;
  int angle;
  uint8_t data[2];
  HAL_GPIO_WritePin(SPI1_BNO_SS_GPIO_Port, SPI1_BNO_SS_Pin, 0);
  HAL_SPI_Transmit(&hspi1, "L", 1, 100);
  HAL_Delay(1);
  HAL_SPI_Receive(&hspi1, &data[0], 1, 100);
  __NOP();
  HAL_SPI_Transmit(&hspi1, "H", 1, 100);
  HAL_Delay(1);
  HAL_SPI_Receive(&hspi1, &data[1], 1, 100);
  __NOP();
  HAL_GPIO_WritePin(SPI1_BNO_SS_GPIO_Port, SPI1_BNO_SS_Pin, 1);
  HAL_Delay(1);

  angle = (data[1] << 8) | data[0];
  if (angle >= 360)
  {
    angle = 0;
  }

  if (abs(angle - robot.angle) < 15 || ignore)
  {
    robot.angle = angle;
    ignore = false;
  }
  else
  {
    ignore = true;
  }
}
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
  /* USER CODE BEGIN 2 */
  LL_mDelay(5);
  start_timers();
  LL_mDelay(3000); //! Wait for BNO055 to be ready(Boot time)
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
      }
      TurnOffLED();
      break;
    }
  }

  // MOTORS_ENABLE();

  while (1)
  {
    update_robot_angle();
    measure_ball_data(sensors, &ball);
    get_ball(&ball);

    //* Out code
    read_line_sensors(line_sensors); //? Update line_sensors array
    robot.on_line_sensors = on_line_sensors_number(line_sensors);

    if (robot.on_line_sensors > 0)
    {
      // robot.out_angle = 0;
      robot.njl_sum_x = robot.njl_sum_y = 0;
      for (uint8_t i = 0; i < 20; i++)
      {
        if (line_sensors[i])
        {
          robot.njl_sum_x += njl_x[i];
          robot.njl_sum_y += njl_y[i];
        }
      }

      if (!robot.line_detect)
      {
        for (uint8_t i = 0; i < 20; i++)
        {
          if (line_sensors[i] && robot.green_time > 500)
          {
            robot.first_out_sensor = i;
            break;
          }
        }
        robot.line_detect = true;
        robot.in_out_area = true;
      }

      robot.current_out_angle = (int)(atan2f(robot.njl_sum_y, robot.njl_sum_x) * RADIAN_TO_DEGREE);
      //? Convert grading system
      robot.current_out_angle -= 90;
      while (robot.current_out_angle < 0)
        robot.current_out_angle += 360;
      robot.current_out_angle *= -1;
      robot.current_out_angle += 360;

      robot.invert_out_angle = robot.current_out_angle - 180;
      if (robot.invert_out_angle < 0)
      {
        robot.invert_out_angle += 360;
      }

      if (abs(robot.current_out_angle - njl_angle[robot.first_out_sensor]) < abs(robot.invert_out_angle - njl_angle[robot.first_out_sensor]))
      {
        robot.out_angle = robot.current_out_angle;
      }
      else
      {
        robot.out_angle = robot.invert_out_angle;
      }
      robot.move_angle = robot.out_angle - 180;
      if (robot.move_angle < 0)
      {
        robot.move_angle += 360;
      }
      // robot.percent_speed = 0.5;
      // robot.percent_speed = 0.7;
      robot.percent_speed = 1;
    }
    else
    {
      robot.line_detect = false;
      robot.percent_speed = 0;
    }

    if (!robot.line_detect && (!robot.in_out_area || abs(robot.out_angle - robot.get_ball_move_angle) > 90))
    {
      robot.in_out_area = false;
      robot.move_angle = robot.get_ball_move_angle;
      robot.percent_speed = robot.get_ball_percent_speed;
    }

    if (Task1ms > 0)
    {
      Task1ms = 0;
    }

    if (Task5ms > 4)
    {
      if (!robot.must_brake)
      {
        robot_move(robot.move_angle, robot.percent_speed);
      }
      else
      {
        robot_brake(25);
      }

      Task5ms -= 5;
    }

    if (Task10ms > 9)
    {
      // sprintf(tx_buff, "oAng: %d  FOS: %d  mA: %d  T: %d  N: %d\r\n", robot.out_angle, robot.first_out_sensor, robot.move_angle, robot.green_time, robot.on_line_sensors);
      // sprintf(tx_buff, "Oa: %d    lD: %d   io: %d   MA: %d    FOS: %d  On: %d\r\n", robot.out_angle, robot.line_detect, robot.in_out_area, robot.move_angle, robot.first_out_sensor, robot.on_line_sensors);
      // PRINT_BUFFER();

      for (uint8_t i = 0; i < 20; i++)
      {
        sprintf(tx_buff, "%d", line_sensors[i]);
        HAL_UART_Transmit(&huart4, tx_buff, strlen(tx_buff), 100);
      }
      sprintf(tx_buff, "     %d   OA: %d\r\n", robot.on_line_sensors, robot.out_angle);
      PRINT_BUFFER();

      Task10ms -= 10;
    }

    if (Task50ms > 49)
    {
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
