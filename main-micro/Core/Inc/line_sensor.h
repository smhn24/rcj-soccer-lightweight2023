#ifndef LINE_SENSOR_H
#define LINE_SENSOR_H

#include <stdbool.h>

#include "i2c.h"
#include "robot_movement.h"
#include "helpers.h"

#define LINE_SENSORS_ADDRESS 0xA0
// #define LINE_KP 0.035
// #define LINE_KP 0.03
#define LINE_KP 0.06
// #define MIN_NJL_SENSORS 3
#define MIN_NJL_SENSORS 2

void read_line_sensors(bool *line_sensors);
uint8_t on_line_sensors_number(bool *line_sensors);
void get_edges(bool *line_sensors, direction_t out_direction, uint8_t *out_edges);
void get_out_direction(bool *line_sensors);
void get_out_direction_edge(bool *line_sensors, uint8_t first_sensor);
void get_out_error();
// void get_out_angle();
// inline bool is_quad_1(uint8_t sensor_number);
// inline bool is_quad_2(uint8_t sensor_number);
// inline bool is_quad_3(uint8_t sensor_number);
// inline bool is_quad_4(uint8_t sensor_number);
// inline bool is_online_quad_1(bool *line_sensors);
// inline bool is_online_quad_2(bool *line_sensors);
// inline bool is_online_quad_3(bool *line_sensors);
// inline bool is_online_quad_4(bool *line_sensors);
// inline uint8_t sensor_quad(uint8_t sensor_number);
// inline bool is_online_left(bool *line_sensors);
// inline bool is_online_right(bool *line_sensors);
// inline bool is_online_front(bool *line_sensors);
// inline bool is_online_back(bool *line_sensors);
// inline bool is_quad_1_empty(bool *line_sensors);
// inline bool is_quad_2_empty(bool *line_sensors);
// inline bool is_quad_3_empty(bool *line_sensors);
// inline bool is_quad_4_empty(bool *line_sensors);
void update_range(int8_t *start, int8_t *end, int8_t s, int8_t e);

uint8_t online_quad(uint8_t quad, bool *line_sensors);

int njl_offset(int angle);

#endif
