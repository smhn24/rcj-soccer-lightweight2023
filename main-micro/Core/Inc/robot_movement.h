#ifndef ROBOT_MOVEMENT_H
#define ROBOT_MOVEMENT_H

#include <stdlib.h>

#include "tim.h"
#include "tssp_helper.h"
#include "helpers.h"
#include "bno055.h"
#include "line_sensor.h"

// #define MAX_VELOCITY 2800
#define MAX_VELOCITY 100
#define MAX_PWM_VALUE 2800
#define GET_BALL_DISTANCE 10
#define LEFT_TOLERANCE_ANGLE 335
#define RIGHT_TOLERANCE_ANGLE 25
#define MAX_DISTANCE 29.1 //? 11 + maxmimum distance
#define MAX_SPEED_PERCENT 0.8
// #define MAX_SPEED_PERCENT 0.65
#define MAX_GET_BALL_SPEED_PERCENT 0.5
#define BRAKE_PERCENT_SPEED 0.6
// #define BRAKE_PERCENT_SPEED 0.85

// #define KP 17
#define KP 0.6
#define KD 0

#define MOTORS_ENABLE() LL_GPIO_SetOutputPin(MOTORS_ENABLE_GPIO_Port, MOTORS_ENABLE_Pin)
#define MOTORS_DISABLE() LL_GPIO_ResetOutputPin(MOTORS_ENABLE_GPIO_Port, MOTORS_ENABLE_Pin)
#define MOTOR1_FORWARD() LL_GPIO_ResetOutputPin(MOTOR1_DIRECTION_GPIO_Port, MOTOR1_DIRECTION_Pin)
#define MOTOR2_FORWARD() LL_GPIO_ResetOutputPin(MOTOR2_DIRECTION_GPIO_Port, MOTOR2_DIRECTION_Pin)
#define MOTOR3_FORWARD() LL_GPIO_ResetOutputPin(MOTOR3_DIRECTION_GPIO_Port, MOTOR3_DIRECTION_Pin)
#define MOTOR4_FORWARD() LL_GPIO_ResetOutputPin(MOTOR4_DIRECTION_GPIO_Port, MOTOR4_DIRECTION_Pin)
#define MOTOR1_BACKWARD() LL_GPIO_SetOutputPin(MOTOR1_DIRECTION_GPIO_Port, MOTOR1_DIRECTION_Pin)
#define MOTOR2_BACKWARD() LL_GPIO_SetOutputPin(MOTOR2_DIRECTION_GPIO_Port, MOTOR2_DIRECTION_Pin)
#define MOTOR3_BACKWARD() LL_GPIO_SetOutputPin(MOTOR3_DIRECTION_GPIO_Port, MOTOR3_DIRECTION_Pin)
#define MOTOR4_BACKWARD() LL_GPIO_SetOutputPin(MOTOR4_DIRECTION_GPIO_Port, MOTOR4_DIRECTION_Pin)

#define SET_MOTOR_1(pwm) TIM1->CCR4 = pwm
#define SET_MOTOR_2(pwm) TIM1->CCR1 = pwm
#define SET_MOTOR_3(pwm) TIM1->CCR2 = pwm
#define SET_MOTOR_4(pwm) TIM1->CCR3 = pwm

typedef struct
{
    volatile float percent_speed;
    volatile int angle;
    volatile int move_angle;
    volatile int out_angle;
    volatile uint8_t out_edges[2]; //? NJL edges for out angle
    volatile uint8_t on_line_sensors;
    volatile uint8_t first_out_sensor;
    volatile uint8_t out_error;
    volatile direction_t out_direction;
    volatile bool out_detect;  //? Robot detects out
    volatile bool in_out;      //? Robot is in the out
    volatile bool line_detect; //? Robot sees the line
    volatile bool is_braking;
} Robot;

void get_ball(BALL *ball);
void set_motors(int motor_1, int motor_2, int motor_3, int motor_4);
void robot_move(int angle, float percent_speed);
int pid_calculator(int error);
void robot_brake(int angle, float percent_speed, uint16_t time);
// void motors_enable();
// void motors_disable();
// uint16_t get_motor_value(int8_t percent_velocity);

#endif
