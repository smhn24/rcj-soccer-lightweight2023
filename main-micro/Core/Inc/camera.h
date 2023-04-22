#ifndef CAMERA_H
#define CAMERA_H

#include <stdio.h>
#include <string.h>
#include <stdbool.h>

#include "main.h"
#include "usart.h"

#define OPENMV_DATA_LENGTH 9
#define OPENMV_START_CHAR '/'
#define PIXYCAM_DATA_LENGTH 100

#define PIXY_RX_DMA_CNT (PIXYCAM_DATA_LENGTH - DMA1_Stream0->NDTR)

typedef struct
{
    volatile int width;
    volatile int height;
    volatile bool detection;
} GOAL;

void read_openmv();
void read_pixy();

#endif