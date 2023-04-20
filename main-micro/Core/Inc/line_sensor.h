#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdbool.h>

#include "i2c.h"
#include "robot_movement.h"
#include "helpers.h"

#define LINE_SENSORS_ADDRESS 0xA0
#define LINE_KP 0.06

void read_line_sensors(bool *line_sensors);
uint8_t on_line_sensors_number(bool *line_sensors);

#endif
