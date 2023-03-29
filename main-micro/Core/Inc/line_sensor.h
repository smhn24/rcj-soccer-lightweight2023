#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdbool.h>

#include "i2c.h"
#include "helpers.h"

#define LINE_SENSORS_ADDRESS 0xA0
// #define LINE_KP 0.08
#define LINE_KP 0.045

void read_line_sensors(bool *line_sensors);
uint8_t on_line_sensors_number(bool *line_sensors);
void get_edges(bool *line_sensors, direction_t out_direction, uint8_t *out_edges);

#endif
