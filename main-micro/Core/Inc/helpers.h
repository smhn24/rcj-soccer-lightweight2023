#ifndef HELPER_FUNCTIONS_H
#define HELPER_FUNCTIONS_H

#include <stdint.h>
#include <stdbool.h>
#include <math.h>

#define DEGREE_TO_RADIAN 0.01744
#define RADIAN_TO_DEGREE 57.2957
#define Pi 3.1415

typedef struct
{
    volatile bool start_status;
    volatile int angle;
    volatile int get_ball_angle;
    volatile int ball_angle;
    volatile int ball_distance;
    volatile int offset;
} Robot;

uint16_t combine_int(uint8_t high_byte, uint8_t low_byte);

#endif