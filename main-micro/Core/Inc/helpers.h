#ifndef HELPER_FUNCTIONS_H
#define HELPER_FUNCTIONS_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "tim.h"

#define DEGREE_TO_RADIAN 0.01744
#define RADIAN_TO_DEGREE 57.2957
#define Pi 3.141592

#define TurnOnLED() LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin)
#define TurnOffLED() LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin)

#define PRINT_BUFFER() HAL_UART_Transmit(&huart4, (uint8_t *)tx_buff, strlen(tx_buff), 500)

void start_timers();

#endif
