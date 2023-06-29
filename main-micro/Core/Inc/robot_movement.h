#ifndef ROBOT_MOVEMENT_H
#define ROBOT_MOVEMENT_H

#include <stdlib.h>

#include "tim.h"
#include "tssp_helper.h"
#include "helpers.h"
#include "line_sensor.h"
#include "camera.h"

#define MAX_VELOCITY 100
#define MAX_PWM_VALUE 2800

#define GET_BALL_DISTANCE 8
#define LEFT_TOLERANCE_ANGLE 335
#define RIGHT_TOLERANCE_ANGLE 25
// #define MAX_DISTANCE 29.1 //? 11 + maxmimum distance
// #define MAX_DISTANCE 35.1 //? 17 + maxmimum distance
#define MAX_DISTANCE 72.1 //? 55 + maxmimum distance

#ifndef MAXON_MOTORS
#define MAX_SPEED_PERCENT 0.85
#define MAX_GET_BALL_SPEED_PERCENT 0.75
#define ENTERING_PERCENT_SPEED 1.0
#define HEAD_KI 0.3
#define HEAD_KD 0.0
#define HEAD_KP 1.3
#endif

#ifdef MAXON_MOTORS
#define MAX_SPEED_PERCENT 0.65
#define MAX_GET_BALL_SPEED_PERCENT 0.55
#define ENTERING_PERCENT_SPEED 0.9
#define HEAD_KI 0.2
#define HEAD_KD 0.0
#define HEAD_KP 0.9
#endif

#define MIN_VERTICAL_DISTANCE 12
#define CAPTURE_BALL_TIMEOUT 500

#define HEAD_PID_I_MAX 10
#define HEAD_PID_MAX 50
#define HEAD_ROTATION_KP 1.3
#define HEAD_ROTATION_I_MAX 100
#define HEAD_ROTATION_KI 0

#define KP 0.65
#define KI 0.0
#define KD 0.05

// ################### Keeper #####################
#define LINE_KP 0.25
#define LINE_KI 0.0025
#define LINE_KD 6

#define BALL_KP 12.5

#define GO_TO_PENALTY_SPEED 0.46

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

void get_ball(BALL *ball);
void set_motors(int motor_1, int motor_2, int motor_3, int motor_4);
void robot_move(int angle, float percent_speed);
int pid_calculator(int error);
void robot_brake(uint16_t time);
void update_head_angle();
void update_robot_head_pid();

#endif
