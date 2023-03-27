#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdbool.h>
#include "i2c.h"

#define LINE_SENSORS_ADDRESS 0xA0

void read_line_sensors(bool *line_sensors, uint8_t *out_data);
#endif