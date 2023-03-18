#include "robot_movement.h"

extern const int motor_speed_table[72][2];

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

// Robot angle is our error because target is 0
int pid_calculator(int error)
{
    static int previous_error = 0;
    static int32_t i = 0;
    int pid, p, d;

    error = (int)fabs((float)error) < 2 ? 0 : error;

    if (error > 180 && error < 360)
    {
        error -= 360;
    }

    p = -error;
    i += p;
    d = error - previous_error;

    if ((int)fabs(error) < 4)
    {
        i = 0;
    }

    pid = (KP * p) + (KI * i) + (KD * d);
    previous_error = error;

    return pid;
}

void robot_move(int angle, float percent_speed)
{
    int m1 = 0, m2 = 0, m3 = 0, m4 = 0; // motors value
    int pid_value;

    pid_value = pid_calculator(BNO055_read());

    // Add pid value to omni-directional
    m1 = -pid_value;
    m2 = pid_value;
    m3 = pid_value;
    m4 = -pid_value;

    angle -= 1;
    if (angle < 0)
        angle += 360;
    angle /= 5;

    // omni-directional
    m1 += motor_speed_table[angle][1] * percent_speed;
    m2 += motor_speed_table[angle][0] * percent_speed;
    m3 += motor_speed_table[angle][1] * percent_speed;
    m4 += motor_speed_table[angle][0] * percent_speed;

    set_motors(m1, m2, m3, m4);
}
