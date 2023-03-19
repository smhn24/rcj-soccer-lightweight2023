#ifndef TSSP_HELPER_H
#define TSSP_HELPER_H

#include <stdint.h>
#include <stdbool.h>

#include "helpers.h"

#define TSSP_MAX_TIMEOUT 7
#define Pi 3.14
#define MEDIAN_READINGS 5
#define AVERAGE_DATA_NUMBER 30
#define DISTANCE_OFFSET 100

typedef struct
{
    volatile uint32_t start_time; // Pulse start time
    volatile uint32_t end_time;   // Pulse end time
    volatile uint16_t width;      // Pulse width
    volatile uint16_t filtered_width;
    volatile uint8_t timeout; // Read timeout(It uses to set pulse 0)
    volatile uint8_t read_index;
    volatile float Ax; // Decomposing the sensor width with x-axis
    volatile float Ay; // Decomposing the sensor width with y-axis
} TSSP;

typedef struct
{
    volatile float sigma_x;         // Sum of the vector sizes on x-axis
    volatile float sigma_y;         // Sum of the vector sizes on y-axis
    volatile int distance;          // Distance between the robot and the ball
    volatile int angle;             // Angle between the robot and the ball
    volatile int get_ball_offset;   // Offset of Get ball
    volatile int get_ball_angle;    // The angel the robot should go to go behind the ball
    volatile float get_ball_speed;  // Speed of Get ball
    volatile int max_value;         // Value of the nearest robot sensor to the ball
    volatile uint8_t sensor_nember; // Number of sensors that see the ball
    volatile uint8_t max_sensor;    // Number of the nearest sensor to the ball
} BALL;

extern inline void update_sensor(uint8_t sensor_index, TSSP *sensor);
extern inline void filter_width(uint8_t sensor_index, TSSP *sensor);
extern inline void decompose_sensor_width(uint8_t sensor_index, TSSP *sensor);
extern inline void measure_ball_data(TSSP sensors[16], BALL *ball);

#endif
