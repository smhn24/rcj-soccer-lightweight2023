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
#include "ssd1306.h"
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
volatile uint8_t tx_buff[100];  //? Data to send with uart
volatile uint8_t oled_buff[20]; //? Data to show in oled
volatile TSSP sensors[16];
volatile BALL ball;
volatile Robot robot;
volatile uint16_t width_temp[16][AVERAGE_DATA_NUMBER] = {0}; //? This array use for avarage filter from pulses
volatile bool line_sensors[20] = {0};                        //? NJL sensors status that sees the line(1) or not(0)

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
  /* USER CODE BEGIN 2 */
  start_timers();
  ssd1306_Init(&hi2c2);
  LL_mDelay(2000); //! Wait for BNO055 to be ready(Boot time)
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
      while (START_KEY_STATUS())
      {
      }
      TurnOffLED();
      break;
    }
  }

  MOTORS_ENABLE();

  // while (1)
  // {
  //   set_motors(0, 0, 0, 0);
  //   HAL_Delay(5000);
  //   for (int i = 0; i < 85; i += 1)
  //   {
  //     set_motors(i, i, i, i);

  //     HAL_Delay(25);
  //   }

  //   for (int i = 85; i > 0; i -= 1)
  //   {
  //     set_motors(i, i, i, i);

  //     HAL_Delay(25);
  //   }

  //   for (int i = 0; i > -85; i -= 1)
  //   {
  //     set_motors(i, i, i, i);

  //     HAL_Delay(25);
  //   }

  //   for (int i = -85; i < 0; i += 1)
  //   {
  //     set_motors(i, i, i, i);

  //     HAL_Delay(25);
  //   }
  // }

  while (1)
  {
    measure_ball_data(sensors, &ball);

    if (!robot.is_braking && !robot.in_out) //! Needs check
    {
      if (robot.out_detect && abs(robot.out_angle - ball.angle) < 45) //! Needs to change with distance of the ball
      {
        robot.move_angle = 0;
        robot.percent_speed = 0;
      }
      else
      {
        robot.out_detect = false;
        get_ball(&ball);
      }
    }

    if (!robot.is_braking)
    {
      robot_move(robot.move_angle, robot.percent_speed);
    }
    else
    {
      robot_brake(robot.move_angle, BRAKE_PERCENT_SPEED, 10);
      // robot_brake(robot.move_angle, BRAKE_PERCENT_SPEED, 200);
    }

    //* Out code
    read_line_sensors(line_sensors); //? Update line_sensors array
    robot.on_line_sensors = on_line_sensors_number(line_sensors);

    if ((robot.on_line_sensors > 0 && robot.on_line_sensors < 10) && !robot.line_detect) //? At least one sensor detected line for the first time
    {
      robot.line_detect = true;
      get_out_direction(line_sensors);
    }
    else if (robot.line_detect && robot.on_line_sensors > 1) //? At least 2 sensors see the line for gettting out error & out angle
    {
      get_edges(line_sensors, robot.out_direction, robot.out_edges);
      get_out_angle();
      get_out_error();

      // robot.percent_speed = LINE_KP * robot.out_error;
      // robot.percent_speed = robot.out_error > 3 ? LINE_KP * robot.out_error : 0;

      if (robot.out_error > 5)
      {
        robot.in_out = true;

        //? Back inside
        robot.move_angle = robot.out_angle - 180;
        if (robot.move_angle < 0)
        {
          robot.move_angle += 360;
        }
        robot.percent_speed = LINE_KP * robot.out_error;
      }
      else
      {
        robot.is_braking = robot.in_out ? true : false;
        robot.in_out = false;
        robot.out_error = 0;
        robot.out_detect = true;
      }

      // robot.percent_speed = LINE_KP * robot.out_error;
    }
    else if (robot.line_detect && robot.on_line_sensors == 0)
    {
      robot.line_detect = false;
      robot.out_direction = NOTHING;
      robot.in_out = false;
      robot.out_detect = true;
    }
    else if (robot.line_detect)
    {
      robot.out_detect = true;
      robot.in_out = false;
    }

    //! Debug
    // if (on_line_sensors_number > 1)
    // {
    //   TurnOnLED();
    // }
    // else
    // {
    //   TurnOffLED();
    // }

    // TODO: Out angle offset must check

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
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
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

#ifdef  USE_FULL_ASSERT
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
