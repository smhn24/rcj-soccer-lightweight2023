#include "robot_movement.h"

/*  Motors number
    2/         \1


    3\         /4
*/

extern Robot robot;
extern uint16_t brake_time;

//? Lookup table for omni-directional movement with 5 degrees resolution
const int motor_speed_table[72][2] = {
    {1980, 1980},
    {2145, 1800},
    {2290, 1610},
    {2425, 1400},
    {2535, 1185},
    {2630, 960},
    {2705, 725},
    {2755, 490},
    {2790, 245},
    {2800, 5},
    {2790, -240},
    {2760, -485},
    {2705, -720},
    {2635, -955},
    {2540, -1180},
    {2425, -1395},
    {2295, -1600},
    {2150, -1795},
    {1985, -1975},
    {1805, -2140},
    {1610, -2290},
    {1405, -2420},
    {1190, -2535},
    {965, -2630},
    {730, -2705},
    {490, -2755},
    {250, -2790},
    {5, -2800},
    {-235, -2790},
    {-480, -2760},
    {-720, -2705},
    {-950, -2635},
    {-1175, -2540},
    {-1395, -2430},
    {-1600, -2300},
    {-1795, -2150},
    {-1975, -1985},
    {-2140, -1805},
    {-2290, -1615},
    {-2420, -1410},
    {-2535, -1190},
    {-2630, -965},
    {-2700, -735},
    {-2755, -495},
    {-2790, -255},
    {-2800, -10},
    {-2790, 235},
    {-2760, 475},
    {-2705, 715},
    {-2635, 950},
    {-2540, 1175},
    {-2430, 1390},
    {-2300, 1595},
    {-2150, 1790},
    {-1990, 1970},
    {-1810, 2135},
    {-1615, 2285},
    {-1410, 2420},
    {-1195, 2530},
    {-970, 2625},
    {-735, 2700},
    {-500, 2755},
    {-255, 2790},
    {-15, 2800},
    {245, 2790},
    {485, 2760},
    {725, 2705},
    {955, 2630},
    {1180, 2540},
    {1400, 2425},
    {1605, 2295},
    {1800, 2145},
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
    //******************************************
    if (motor_1 >= 0)
    {
        SET_MOTOR_1(motor_1);
        MOTOR1_FORWARD();
    }
    else if (motor_1 < 0)
    {
        SET_MOTOR_1((int)fabs(motor_1));
        MOTOR1_BACKWARD();
    }
    //******************************************
    if (motor_2 >= 0)
    {
        SET_MOTOR_2(motor_2);
        MOTOR2_FORWARD();
    }
    else if (motor_2 < 0)
    {
        SET_MOTOR_2((int)fabs(motor_2));
        MOTOR2_BACKWARD();
    }
    //******************************************
    if (motor_3 >= 0)
    {
        SET_MOTOR_3(motor_3);
        MOTOR3_FORWARD();
    }
    else if (motor_3 < 0)
    {
        SET_MOTOR_3((int)fabs(motor_3));
        MOTOR3_BACKWARD();
    }
    //******************************************
    if (motor_4 >= 0)
    {
        SET_MOTOR_4(motor_4);
        MOTOR4_FORWARD();
    }
    else if (motor_4 < 0)
    {
        SET_MOTOR_4((int)fabs(motor_4));
        MOTOR4_BACKWARD();
    }
    //******************************************
}

//? Robot angle is our error because target is 0
int pid_calculator(int error)
{
    static int previous_error = 0;
    int pid, p, d;

    error = (int)fabs((float)error) < 2 ? 0 : error;

    if (error > 180 && error < 360)
    {
        error -= 360;
    }

    p = -error;
    d = error - previous_error;

    pid = (KP * p) + (KD * d);
    previous_error = error;

    return pid;
}

void robot_move(int angle, float percent_speed)
{
    int m1 = 0, m2 = 0, m3 = 0, m4 = 0; //? motors value
    int pid_value;

    pid_value = pid_calculator(robot.angle);

    //* Add pid value to omni-directional
    m1 = -pid_value;
    m2 = pid_value;
    m3 = pid_value;
    m4 = -pid_value;

    angle -= 1;
    if (angle < 0)
        angle += 360;
    angle /= 5;

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
            robot.percent_speed = MAX_SPEED_PERCENT;
        }
        else if (ball->angle >= RIGHT_TOLERANCE_ANGLE && ball->angle <= 180) //? Ball is right of the robot
        {
            ball->get_ball_offset = (int)(asinf((float)11 / (float)(MAX_DISTANCE - ball->distance)) * RADIAN_TO_DEGREE);
            robot.percent_speed = MAX_GET_BALL_SPEED_PERCENT;
        }
        else if (ball->angle > 180 && ball->angle <= LEFT_TOLERANCE_ANGLE) //? Ball is left of the robot
        {
            ball->get_ball_offset = (int)(-asinf((float)11 / (float)(MAX_DISTANCE - ball->distance)) * RADIAN_TO_DEGREE);
            robot.percent_speed = MAX_GET_BALL_SPEED_PERCENT;
        }
        robot.move_angle = ball->get_ball_offset + ball->angle;
    }
    else
    {
        robot.move_angle = ball->angle;
        robot.percent_speed = MAX_SPEED_PERCENT;
    }

    if (ball->distance < 2)
    {
        robot.move_angle = 0;
        robot.percent_speed = 0;
    }
}

inline void robot_brake(int angle, float percent_speed, uint16_t time)
{
    brake_time = 0;
    while (brake_time < time)
    {
        robot_move(angle, percent_speed);
        LL_mDelay(2);
    }
}
