#include "robot_movement.h"

/*  Motors number
    2/         \1


    3\         /4
*/

extern Robot robot;
extern GOAL goal;

//? Lookup table for omni-directional movement with 5 degrees resolution(Generated by lookup_generator.py available on this repository)
const int8_t motor_speed_table[72][2] = {
    {70, 70},
    {75, 65},
    {80, 55},
    {85, 50},
    {90, 40},
    {95, 35},
    {95, 25},
    {100, 15},
    {100, 10},
    {100, 0},
    {100, -10},
    {100, -15},
    {95, -25},
    {95, -35},
    {90, -40},
    {85, -50},
    {80, -55},
    {75, -65},
    {70, -70},
    {65, -75},
    {60, -80},
    {50, -85},
    {40, -90},
    {35, -95},
    {25, -95},
    {20, -100},
    {10, -100},
    {0, -100},
    {-10, -100},
    {-15, -100},
    {-25, -95},
    {-35, -95},
    {-40, -90},
    {-50, -85},
    {-55, -80},
    {-65, -75},
    {-70, -70},
    {-75, -65},
    {-80, -60},
    {-85, -50},
    {-90, -45},
    {-95, -35},
    {-95, -25},
    {-100, -20},
    {-100, -10},
    {-100, 0},
    {-100, 10},
    {-100, 15},
    {-95, 25},
    {-95, 35},
    {-90, 40},
    {-85, 50},
    {-80, 55},
    {-75, 65},
    {-70, 70},
    {-65, 75},
    {-60, 80},
    {-50, 85},
    {-45, 90},
    {-35, 95},
    {-25, 95},
    {-20, 100},
    {-10, 100},
    {0, 100},
    {10, 100},
    {15, 100},
    {25, 95},
    {35, 95},
    {40, 90},
    {50, 85},
    {55, 80},
    {65, 75},
};

void set_motors(int motor_1, int motor_2, int motor_3, int motor_4)
{
    //***********Set range of values************
    motor_1 = motor_1 > MAX_VELOCITY ? MAX_VELOCITY : motor_1;
    motor_2 = motor_2 > MAX_VELOCITY ? MAX_VELOCITY : motor_2;
    motor_3 = motor_3 > MAX_VELOCITY ? MAX_VELOCITY : motor_3;
    motor_4 = motor_4 > MAX_VELOCITY ? MAX_VELOCITY : motor_4;

    motor_1 = motor_1 < -MAX_VELOCITY ? -MAX_VELOCITY : motor_1;
    motor_2 = motor_2 < -MAX_VELOCITY ? -MAX_VELOCITY : motor_2;
    motor_3 = motor_3 < -MAX_VELOCITY ? -MAX_VELOCITY : motor_3;
    motor_4 = motor_4 < -MAX_VELOCITY ? -MAX_VELOCITY : motor_4;
    //*******Convert percent to pwm value*******
    motor_1 *= 28;
    motor_2 *= 28;
    motor_3 *= 28;
    motor_4 *= 28;
    //******************************************
    if (motor_1 >= 0)
    {
        SET_MOTOR_1(motor_1);
        MOTOR1_FORWARD();
    }
    else
    {
        SET_MOTOR_1(abs(motor_1));
        MOTOR1_BACKWARD();
    }
    //******************************************
    if (motor_2 >= 0)
    {
        SET_MOTOR_2(motor_2);
        MOTOR2_FORWARD();
    }
    else
    {
        SET_MOTOR_2(abs(motor_2));
        MOTOR2_BACKWARD();
    }
    //******************************************
    if (motor_3 >= 0)
    {
        SET_MOTOR_3(motor_3);
        MOTOR3_FORWARD();
    }
    else
    {
        SET_MOTOR_3(abs(motor_3));
        MOTOR3_BACKWARD();
    }
    //******************************************
    if (motor_4 >= 0)
    {
        SET_MOTOR_4(motor_4);
        MOTOR4_FORWARD();
    }
    else
    {
        SET_MOTOR_4(abs(motor_4));
        MOTOR4_BACKWARD();
    }
    //******************************************
}

//? Robot angle is our error because target is 0
int pid_calculator(int error)
{
    static int previous_error = 0;
    static int sigma = 0;
    int pid, p, d;

    error = abs(error) < 2 ? 0 : error;

    // if (error > 180 && error < 360)
    // {
    //     error -= 360;
    // }

    // p = -error;
    p = error;
    sigma += p;
    if (p == 0)
    {
        sigma = 0;
    }
    else if (sigma > 1000)
    {
        sigma = 1000;
    }
    else if (sigma < -1000)
    {
        sigma = -1000;
    }
    d = p - previous_error;

    pid = (KP * p) + (KI * sigma) + (KD * d);
    previous_error = error;

    return pid;
}

extern float HeadPID_Out;
void robot_move(int angle, float percent_speed)
{
    int m1 = 0, m2 = 0, m3 = 0, m4 = 0; //? motors value
    int pid_value;

    pid_value = HeadPID_Out;
    // pid_value = pid_calculator(robot.angle);

    //* Add pid value to omni-directional
    m1 = -pid_value;
    m2 = pid_value;
    m3 = pid_value;
    m4 = -pid_value;

    if (angle > 180)
    {
        angle -= 360;
    }
    else if (angle < -180)
    {
        angle += 360;
    }

    angle *= -1;
    if (angle < 0)
    {
        angle += 360;
    }

    while (angle > 359)
    {
        angle -= 360;
    }

    while (angle < 0)
    {
        angle += 360;
    }

    angle /= 5;

    if (percent_speed > 1)
    {
        percent_speed = 1;
    }
    else if (percent_speed < 0)
    {
        percent_speed = 0;
    }

    //* omni-directional
    m1 += motor_speed_table[angle][1] * percent_speed;
    m2 += motor_speed_table[angle][0] * percent_speed;
    m3 += motor_speed_table[angle][1] * percent_speed;
    m4 += motor_speed_table[angle][0] * percent_speed;

    set_motors(m1, m2, m3, m4);
}

inline void get_ball(BALL *ball)
{
    if (ball->distance > GET_BALL_DISTANCE)
    {
        if (ball->angle > LEFT_TOLERANCE_ANGLE || ball->angle < RIGHT_TOLERANCE_ANGLE)
        {
            ball->get_ball_offset = 0;
            robot.get_ball_percent_speed = MAX_SPEED_PERCENT;
        }
        else if (ball->angle >= RIGHT_TOLERANCE_ANGLE && ball->angle <= 180) //? Ball is right of the robot
        {
            ball->get_ball_offset = (int)(asinf((float)11 / (float)(MAX_DISTANCE - ball->distance)) * RADIAN_TO_DEGREE);
            robot.get_ball_percent_speed = MAX_GET_BALL_SPEED_PERCENT;
        }
        else if (ball->angle > 180 && ball->angle <= LEFT_TOLERANCE_ANGLE) //? Ball is left of the robot
        {
            ball->get_ball_offset = (int)(-asinf((float)11 / (float)(MAX_DISTANCE - ball->distance)) * RADIAN_TO_DEGREE);
            robot.get_ball_percent_speed = MAX_GET_BALL_SPEED_PERCENT;
        }
        robot.get_ball_move_angle = ball->get_ball_offset + ball->angle;
    }
    else
    {
        robot.get_ball_move_angle = ball->angle;
        robot.get_ball_percent_speed = MAX_SPEED_PERCENT;
    }

    if (ball->distance < 2)
    {
        robot.get_ball_move_angle = 0;
        robot.get_ball_percent_speed = 0;
    }

    robot.get_ball_move_angle *= -1;
    if (robot.get_ball_move_angle < -180)
    {
        robot.get_ball_move_angle += 360;
    }
}

void robot_brake(uint16_t time)
{
    static uint16_t start_time = 0;
    robot.brake_move_angle -= 180;
    if (robot.brake_move_angle < 0)
    {
        robot.brake_move_angle += 360;
    }

    if (start_time > time)
    {
        start_time = 0;
        robot.must_brake = false;
    }
    else
    {
        robot_move(robot.move_angle, BRAKE_PERCENT_SPEED);
        start_time++;
    }
}

void update_head_angle()
{
    //!!! Must call every 10ms
    static uint32_t sum_i = 0;
    if (robot.camera_connection && goal.detection)
    {
        sum_i += goal.width;
        if (sum_i > HEAD_ROTATION_I_MAX)
        {
            sum_i = HEAD_ROTATION_I_MAX;
        }
        robot.head_angle = -goal.width * HEAD_ROTATION_KP;
        robot.head_angle += sum_i * HEAD_ROTATION_KI;
        // robot.head_angle = -goal.width;
    }
    else
    {
        robot.head_angle = 0;
    }

    // uint8_t tx[100];
    // sprintf(tx, "ha: %d\r\n", robot.head_angle);
    // HAL_UART_Transmit(&huart4, tx, strlen(tx), 200);
}
