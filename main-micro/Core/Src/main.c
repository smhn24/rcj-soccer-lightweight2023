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
#include "tssp_helper.h"
#include "line_sensor.h"
#include "helpers.h"
#include "camera.h"
#include "srf_helper.h"
#include "attacker_strategy.h"
#include "goal_keeper_strategy.h"
#include "robot_connection.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define MAXON_MOTORS
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
SECOND_ROBOT second_robot;
uint16_t width_temp[16][AVERAGE_DATA_NUMBER] = {0}; //? This array use for avarage filter from pulses
uint8_t tx_buff[100];                               //? Data to send with uart
bool line_sensors[20] = {0};                        //? NJL sensors status that sees the line(1) or not(0)

uint8_t openmv_data[OPENMV_DATA_LENGTH] = {0};
uint8_t second_robot_pocket[POCKET_LENGTH] = {0};

float AyFilt = 0;
float I_sigma = 0;
float last_error_line = 0;

float pidout_line = 0;
int X_srf = 90;
float Y_srf = 50;
uint8_t TrueSRF = 0;

uint16_t Y_timer = 0;

uint16_t go_ahead_flag = 0;
//******************************************

volatile SRF left_srf, right_srf, back_srf;

extern float average_x;
extern float average_y;

//* Tasks *//
volatile uint16_t Task1ms = 0, Task4ms = 0, Task10ms = 0, Task25ms = 0, Task30ms = 0, Task50ms = 0, Task250ms = 0;
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
  // robot.role = attacker;
  robot.role = goal_keeper;
  robot.started = false;

  HAL_UART_Receive_DMA(&huart5, openmv_data, OPENMV_DATA_LENGTH);    //? Openmv camera
  HAL_UART_Receive_DMA(&huart4, second_robot_pocket, POCKET_LENGTH); //? Robots connection

  LL_mDelay(100);
  MPU6050_Init();
  LL_mDelay(100);
  MPU6050_Calibration();
  start_timers();
  LL_mDelay(100); //! Wait for BNO055 to be ready(Boot time)
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
  robot.started = true;
  MOTORS_ENABLE();

  while (1)
  {
    uart_error_handler(); //? Error handler for uart dma(openmv)

    measure_ball_data(sensors, &ball);
    if (robot.role == attacker)
    {
      get_ball(&ball);
    }

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
    if (robot.role == attacker)
    {
      if (!robot.line_detect && (!robot.in_out_area || abs(robot.out_angle - robot.get_ball_move_angle) > 45))
      {
        robot.in_out_area = false;
        // robot.move_angle = robot.get_ball_move_angle;
        if (robot.captured_ball)
        {
          robot.move_angle = -goal.width * 0.45; //? Change move angle with goal location
        }
        else
        {
          robot.move_angle = robot.get_ball_move_angle;
        }

        robot.percent_speed = robot.get_ball_percent_speed;
      }
    }

    if (Task1ms > 0)
    {
      update_robot_angle();

      Task1ms -= 1;
    }

    if (Task4ms > 3)
    {
      //? Head PID
      update_robot_head_pid();

      if (!robot.must_brake)
      {
        robot_move(robot.move_angle, robot.percent_speed);
      }
      else
      {
        robot_brake(25);
      }

      if (robot.role == attacker)
      {
        attacker_strategy();
      }
      // else
      // {
      //   goal_keeper_strategy();
      // }

      Task4ms -= 4;
    }

    if (Task10ms > 9)
    {
      if (robot.role == goal_keeper)
      {
        // ################################### Line PID
        //! Must call every 10ms
        if (robot.on_line_sensors > 0)
        {
          // AyFilt = (AyFilt * 0.9) + (average_y * 0.1);
          AyFilt = (AyFilt * 0.9) + (average_y * 0.1) + 0.025;
          I_sigma += AyFilt;

          if (I_sigma > 100)
          {
            I_sigma = 100;
          }
          else if (I_sigma < -100)
          {
            I_sigma = -100;
          }
          if (fabs(AyFilt) < 0.05)
          {
            I_sigma = 0;
          }

          pidout_line = LINE_KP * AyFilt + I_sigma * LINE_KI + (AyFilt - last_error_line) * LINE_KD;
          last_error_line = AyFilt;
          if (pidout_line > 1)
          {
            pidout_line = 1;
          }
          if (pidout_line < -1)
          {
            pidout_line = -1;
          }
        }

        float MoveSpeed = 0;
        float MoveAngle = 0;

        //? False flag
        // if (ball.angle > 45 || ball.angle < -45 || ball.distance < 15)
        // {
        // 	go_ahead_flag = false;
        // }

        if ((ball.angle > 330 || ball.angle < 30) && ball.distance > 10 && Y_srf < 50)
        {
          MoveAngle = 0;
          MoveSpeed = 1;
        }
        else if (X_srf > RIGHT_GOAL_POS || X_srf < LEFT_GOAL_POS || Y_srf > (PENALTY_LINE_POS + 15) || (Y_srf < PENALTY_LINE_POS - 6))
        {
          MoveSpeed = GO_TO_PENALTY_SPEED;
          if (Y_srf < PENALTY_LINE_POS - 5)
          {
            MoveAngle = 0;
            pidout_line = 1;
            // MoveSpeed = 0.5;
          }
          else
          {
            MoveAngle = 180 - atan2f((X_srf - 91), (Y_srf - PENALTY_LINE_POS)) * RADIAN_TO_DEGREE;
          }
        }

        else
        {
          //? True flag
          // else if (ball.angle < 45 && ball.angle > -45 && ball.distance > 15 && !go_ahead_flag)
          // go_ahead_flag = true;
          // // Y_srf = 0;
          // Y_timer = 0;

          // ################################### Ball PID
          int16_t BallAngle = ball.angle;
          BallAngle *= -1;
          BallAngle += 450;
          while (BallAngle > 360)
          {
            BallAngle -= 360;
          }

          float ball_error = 0;
          if (ball.distance > 0)
          {
            // ball_error = cos(BallAngle * DEGREE_TO_RADIAN);
            ball_error = cosf(BallAngle * DEGREE_TO_RADIAN);
          }
          float pidout_ball = BALL_KP * ball_error;
          if (pidout_ball > 1)
          {
            pidout_ball = 1;
          }
          if (pidout_ball < -1)
          {
            pidout_ball = -1;
          }

          // ################################### Set Percent Speed
          MoveSpeed = ((fabs(pidout_ball * ball_error)) + (fabs(pidout_line * AyFilt))) / (fabs(ball_error) + fabs(AyFilt));

          // ################################### Set Move Angle
          if (fabs(ball_error) < 0.05 || robot.green_time > 200)
          {
            if (pidout_line > 0)
            {
              MoveAngle = 0;
            }
            else if (pidout_line < 0)
            {
              MoveAngle = -180;
            }
          }
          else
          {
            MoveAngle = atan2f(pidout_line, ball_error) * RADIAN_TO_DEGREE;

            MoveAngle -= 90;
          }

          if (MoveAngle < -180)
          {
            MoveAngle += 360;
          }
          else if (MoveAngle > 180)
          {
            MoveAngle -= 360;
          }

          if (X_srf > RIGHT_GOAL_POS - 2 && MoveAngle < 0)
            MoveSpeed = 0;
          if (X_srf < LEFT_GOAL_POS + 2 && MoveAngle > 0)
            MoveSpeed = 0;
        }

        if (MoveSpeed > 1)
        {
          MoveSpeed = 1;
        }
        else if (MoveSpeed < 0)
        {
          MoveSpeed = 0;
        }

        robot.percent_speed = MoveSpeed;
        robot.move_angle = MoveAngle;
      }

      Task10ms -= 10;
    }

    if (Task30ms > 29)
    {
      update_srf_data();
      Task30ms -= 30;
    }

    if (Task50ms > 49)
    {
      if (robot.captured_ball)
      {
        TurnOnLED();
      }
      else
      {
        TurnOffLED();
      }

      Task50ms -= 50;
    }

    if (Task250ms > 249) //? This task is for connection of the robots
    {
      //? Send pocket to second robot
      sprintf(tx_buff, "%c%03d", robot.role == attacker ? 'A' : 'G', ball.distance);
      SEND_BUFFER();

      //? Change task
      if (!second_robot.exist)
      {
        robot.role = attacker;
      }
      else if (robot.role == second_robot.role)
      {
        robot.role = (ball.distance > second_robot.ball_distance) ? goal_keeper : attacker;
      }

      // sprintf(tx_buff, "H: %d    W: %d    D: %d\n", goal.height, goal.width, goal.detection);
      // SEND_BUFFER();

      Task250ms -= 250;
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
  else if (huart->Instance == UART4)
  {
    second_robot.refresh_time = 0;
    read_second_robot_pocket();
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
