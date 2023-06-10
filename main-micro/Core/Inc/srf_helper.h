#ifndef _SRF_HELPER_H
#define _SRF_HELPER_H

#define LEFT_GOAL_POS 65
#define RIGHT_GOAL_POS 115
// #define PENALTY_LINE_POS 55 // 37 for competition
#define PENALTY_LINE_POS 37 // 37 for competition

#include <stdint.h>

#include "robot_movement.h"

typedef struct
{
    uint32_t start_time; //? Pulse start time
    uint32_t end_time;   //? Pulse end time
    uint16_t width;      //? Pulse width
    int dis;
    int pre_dis;
} SRF;

void update_srf_data();

#endif