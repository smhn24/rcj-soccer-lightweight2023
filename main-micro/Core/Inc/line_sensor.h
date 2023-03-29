#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdbool.h>
#include "i2c.h"

#define LINE_SENSORS_ADDRESS 0xA0
// #define LINE_KP 0.08
#define LINE_KP 0.045

typedef enum __attribute__((packed)) _out_direction
{
    not_out = 0,
    right = 1,
    left = 2,
    backward = 3,
    forward = 4
} out_direction_t;

void read_line_sensors(bool *line_sensors);
uint8_t on_line_sensors_number(bool *line_sensors);
void get_edges(bool *line_sensors, out_direction_t out_direction, uint8_t *out_edges);

#endif
