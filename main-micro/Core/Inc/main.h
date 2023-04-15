/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.h
  * @brief          : Header for main.c file.
  *                   This file contains the common defines of the application.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx_hal.h"

#include "stm32f4xx_ll_system.h"
#include "stm32f4xx_ll_gpio.h"
#include "stm32f4xx_ll_exti.h"
#include "stm32f4xx_ll_bus.h"
#include "stm32f4xx_ll_cortex.h"
#include "stm32f4xx_ll_rcc.h"
#include "stm32f4xx_ll_utils.h"
#include "stm32f4xx_ll_pwr.h"
#include "stm32f4xx_ll_dma.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Exported types ------------------------------------------------------------*/
/* USER CODE BEGIN ET */

/* USER CODE END ET */

/* Exported constants --------------------------------------------------------*/
/* USER CODE BEGIN EC */

/* USER CODE END EC */

/* Exported macro ------------------------------------------------------------*/
/* USER CODE BEGIN EM */

/* USER CODE END EM */

/* Exported functions prototypes ---------------------------------------------*/
void Error_Handler(void);

/* USER CODE BEGIN EFP */

/* USER CODE END EFP */

/* Private defines -----------------------------------------------------------*/
#define Key_Pin LL_GPIO_PIN_13
#define Key_GPIO_Port GPIOC
#define LED_Pin LL_GPIO_PIN_14
#define LED_GPIO_Port GPIOC
#define StartKey_Pin LL_GPIO_PIN_15
#define StartKey_GPIO_Port GPIOC
#define TSSP5_Pin LL_GPIO_PIN_0
#define TSSP5_GPIO_Port GPIOA
#define TSSP10_Pin LL_GPIO_PIN_1
#define TSSP10_GPIO_Port GPIOA
#define TSSP4_Pin LL_GPIO_PIN_2
#define TSSP4_GPIO_Port GPIOA
#define TSSP11_Pin LL_GPIO_PIN_3
#define TSSP11_GPIO_Port GPIOA
#define SPI1_BNO_SS_Pin LL_GPIO_PIN_4
#define SPI1_BNO_SS_GPIO_Port GPIOA
#define MOTORS_ENABLE_Pin LL_GPIO_PIN_4
#define MOTORS_ENABLE_GPIO_Port GPIOC
#define MOTOR2_DIRECTION_Pin LL_GPIO_PIN_5
#define MOTOR2_DIRECTION_GPIO_Port GPIOC
#define TSSP3_Pin LL_GPIO_PIN_0
#define TSSP3_GPIO_Port GPIOB
#define TSSP12_Pin LL_GPIO_PIN_1
#define TSSP12_GPIO_Port GPIOB
#define MOTOR3_DIRECTION_Pin LL_GPIO_PIN_2
#define MOTOR3_DIRECTION_GPIO_Port GPIOB
#define MOTOR4_DIRECTION_Pin LL_GPIO_PIN_12
#define MOTOR4_DIRECTION_GPIO_Port GPIOB
#define MOTOR1_DIRECTION_Pin LL_GPIO_PIN_13
#define MOTOR1_DIRECTION_GPIO_Port GPIOB
#define TSSP2_Pin LL_GPIO_PIN_14
#define TSSP2_GPIO_Port GPIOB
#define TSSP13_Pin LL_GPIO_PIN_15
#define TSSP13_GPIO_Port GPIOB
#define TSSP1_Pin LL_GPIO_PIN_6
#define TSSP1_GPIO_Port GPIOC
#define TSSP14_Pin LL_GPIO_PIN_7
#define TSSP14_GPIO_Port GPIOC
#define TSSP0_Pin LL_GPIO_PIN_8
#define TSSP0_GPIO_Port GPIOC
#define TSSP15_Pin LL_GPIO_PIN_9
#define TSSP15_GPIO_Port GPIOC
#define MOTOR2_PWM_Pin LL_GPIO_PIN_8
#define MOTOR2_PWM_GPIO_Port GPIOA
#define MOTOR3_PWM_Pin LL_GPIO_PIN_9
#define MOTOR3_PWM_GPIO_Port GPIOA
#define MOTOR4_PWM_Pin LL_GPIO_PIN_10
#define MOTOR4_PWM_GPIO_Port GPIOA
#define MOTOR1_PWM_Pin LL_GPIO_PIN_11
#define MOTOR1_PWM_GPIO_Port GPIOA
#define TSSP7_Pin LL_GPIO_PIN_15
#define TSSP7_GPIO_Port GPIOA
#define TSSP8_Pin LL_GPIO_PIN_3
#define TSSP8_GPIO_Port GPIOB
#define TSSP6_Pin LL_GPIO_PIN_4
#define TSSP6_GPIO_Port GPIOB
#define TSSP9_Pin LL_GPIO_PIN_5
#define TSSP9_GPIO_Port GPIOB

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
}
#endif

#endif /* __MAIN_H */
