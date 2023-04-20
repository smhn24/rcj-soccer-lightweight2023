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

  MOTORS_ENABLE();
  int temp_angle;
  while (1)
  {
    update_robot_angle();
    measure_ball_data(sensors, &ball);
    get_ball(&ball);

    if (robot.in_out_area)
    {
      if (robot.line_detect && (robot.out_error_x > MAX_OUT_ERROR || robot.out_error_y > MAX_OUT_ERROR))
      {
        robot.move_angle = robot.out_angle - 180;
        if (robot.move_angle < 0)
        {
          robot.move_angle += 360;
        }
        robot.percent_speed = LINE_KP * (robot.out_error_x + robot.out_error_y);
      }
      else if (robot.out_direction == E)
      {
        if (robot.get_ball_move_angle < 180)
        {
          robot.percent_speed = 0;
          if (robot.get_ball_move_angle < 70)
          {
            robot.move_angle = 0;
            robot.percent_speed = abs(cosf((robot.get_ball_move_angle + 20) * DEGREE_TO_RADIAN));
          }
          else if (robot.get_ball_move_angle > 110)
          {
            robot.move_angle = 180;
            robot.percent_speed = abs(cosf((robot.get_ball_move_angle - 20) * DEGREE_TO_RADIAN));
          }
        }
        else if (!robot.must_brake)
        {
          temp_angle = robot.move_angle;
          robot.in_out_area = false;
        }
        else
        {
          robot.percent_speed = 0;
        }
      }

      else
      {
        robot.percent_speed = 0;
      }
    }
    else
    {
      robot.move_angle = robot.get_ball_move_angle;
      robot.percent_speed = robot.get_ball_percent_speed;
    }

    sprintf(tx_buff, "eX: %d   eY: %d   io: %d   D: %d  mA: %d  get: %d   p: %d\r\n", robot.out_error_x, robot.out_error_y, robot.in_out_area, robot.out_direction, robot.move_angle, robot.get_ball_move_angle, robot.percent_speed);
    PRINT_BUFFER();
    //* Out code
    read_line_sensors(line_sensors); //? Update line_sensors array
    robot.on_line_sensors = on_line_sensors_number(line_sensors);

    //? Get out main direction
    if (robot.on_line_sensors > 0 && !robot.line_detect)
    {
      get_out_direction(line_sensors);
      robot.line_detect = true;
      if (robot.brake_done > BRAKE_TIME_LIMIT)
      {
        robot.must_brake = true;
      }
      robot.in_out_area = true;
    }
    else if (robot.on_line_sensors > 1)
    {
      get_edges(line_sensors);
      get_out_direction_edge(line_sensors);
      get_out_error();
    }

    else if (robot.on_line_sensors < 1)
    {
      robot.line_detect = false;
      robot.out_error_x = 0;
      robot.out_error_y = 0;
      // TODO reset out direction
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
      // sprintf(tx_buff, "N: %d   e0: %d   e1: %d  D: %d   A: %d\r\n", robot.on_line_sensors, robot.out_edges[0], robot.out_edges[1], robot.out_direction, robot.out_angle);
      // sprintf(tx_buff, "ODir: %d  ErrX: %d   ErrY: %d  OAng: %d  BAng: %d\r\n", robot.out_direction, robot.out_error_x, robot.out_error_y, robot.out_angle, ball.angle);
      // PRINT_BUFFER();

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
