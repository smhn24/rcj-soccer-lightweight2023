#ifndef HELPER_FUNCTIONS_H
#define HELPER_FUNCTIONS_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#include "tim.h"
#include "usart.h"
#include "spi.h"

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

typedef enum __attribute__((packed)) _robot_role
{
    goal_keeper = 0,
    attacker = 1,
} robot_role_t;

typedef struct
{
    volatile float get_ball_percent_speed;
    volatile float percent_speed;
    volatile float njl_sum_x;
    volatile float njl_sum_y;
    volatile int16_t angle;
    volatile int16_t head_angle;
    volatile int16_t get_ball_move_angle;
    volatile int16_t brake_move_angle;
    volatile int16_t move_angle;
    volatile int16_t current_out_angle;
    volatile int16_t first_out_angle;
    volatile int16_t out_angle;
    volatile int16_t invert_out_angle;
    volatile uint16_t green_time;
    volatile uint16_t captured_ball_time;
    volatile uint16_t camera_refresh_time;
    volatile uint8_t on_line_sensors;
    volatile uint8_t first_out_sensor;
    volatile robot_role_t role;
    volatile bool in_out_area;
    volatile bool line_detect;
    volatile bool must_brake;
    volatile bool camera_connection;
    volatile bool captured_ball;
} Robot;

void start_timers();
void uart_error_handler();

#endif
