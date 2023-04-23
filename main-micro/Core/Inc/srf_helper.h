#ifndef _SRF_HELPER_H
#define _SRF_HELPER_H

#include <stdint.h>

typedef struct
{
    uint32_t start_time; //? Pulse start time
    uint32_t end_time;   //? Pulse end time
    uint16_t width;      //? Pulse width
} SRF;

#endif