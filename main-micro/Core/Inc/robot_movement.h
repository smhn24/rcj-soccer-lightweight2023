#ifndef ROBOT_MOVEMENT_H
#define ROBOT_MOVEMENT_H

#include <stdlib.h>

#include "tim.h"
#include "tssp_helper.h"
#include "helpers.h"
#include "bno055.h"
#include "line_sensor.h"
#include "camera.h"

#define MAX_VELOCITY 100
#define MAX_PWM_VALUE 2800
#define GET_BALL_DISTANCE 10
#define LEFT_TOLERANCE_ANGLE 335
#define RIGHT_TOLERANCE_ANGLE 25
#define MAX_DISTANCE 29.1 //? 11 + maxmimum distance
#define MAX_SPEED_PERCENT (robot.role == attacker ? 0.7 : 0.8)
#define MAX_GET_BALL_SPEED_PERCENT (robot.role == attacker ? 0.55 : 0.75)
#define BRAKE_PERCENT_SPEED (robot.role == attacker ? 0.85 : 1)
#define MIN_VERTICAL_DISTANCE 12
#define CAPTURE_BALL_TIMEOUT 500

#define HEAD_PID_I_MAX 10
#define HEAD_PID_MAX 50
#define HEAD_KI (robot.role == attacker ? 0.2 : 0.3)
#define HEAD_KP (robot.role == attacker ? 0.9 : 1.3)
#define HEAD_KD 0.0
#define HEAD_ROTATION_KP 1.2
#define HEAD_ROTATION_I_MAX 200
#define HEAD_ROTATION_KI 0.04

#define KP 0.65
#define KI 0.0
#define KD 0.05

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
    volatile robot_role_t role;
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
    volatile bool in_out_area;
    volatile bool line_detect;
    volatile bool must_brake;
    volatile bool camera_connection;
    volatile bool captured_ball;
} Robot;

void get_ball(BALL *ball);
void set_motors(int motor_1, int motor_2, int motor_3, int motor_4);
void robot_move(int angle, float percent_speed);
int pid_calculator(int error);
void robot_brake(uint16_t time);
void update_head_angle();

#endif
