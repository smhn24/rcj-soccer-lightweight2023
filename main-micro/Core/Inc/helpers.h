#ifndef HELPER_FUNCTIONS_H
#define HELPER_FUNCTIONS_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "tim.h"
#include "usart.h"

#define DEGREE_TO_RADIAN 0.01744
#define RADIAN_TO_DEGREE 57.2957
#define Pi 3.141592

#define TurnOnLED() LL_GPIO_SetOutputPin(LED_GPIO_Port, LED_Pin)
#define TurnOffLED() LL_GPIO_ResetOutputPin(LED_GPIO_Port, LED_Pin)
#define ToggleLED() LL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin)

#define PRINT_BUFFER() HAL_UART_Transmit_DMA(&huart4, tx_buff, strlen(tx_buff))

typedef enum __attribute__((packed)) _direction
{
    N = 0,
    NE = 1,
    E = 2,
    SE = 3,
    S = 4,
    SW = 5,
    W = 6,
    NW = 7,
    NOTHING = 8,
} direction_t;

void start_timers();

#endif
