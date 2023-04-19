#include "tssp_helper.h"

extern TSSP sensors[16];
extern uint16_t width_temp[16][AVERAGE_DATA_NUMBER];

const float sensor_sin[16] = {0, 0.382, 0.707, 0.923, 1, 0.923, 0.707, 0.382, 0, -0.382, -0.707, -0.923, -1, -0.923, -0.707, -0.382};
const float sensor_cos[16] = {1, 0.923, 0.707, 0.382, 0, -0.382, -0.707, -0.923, -1, -0.923, -0.707, -0.382, 0, 0.382, 0.707, 0.923};

inline void update_sensor(uint8_t sensor_index, TSSP *sensor)
{
    if (sensor->end_time < sensor->start_time)
        sensor->end_time += 799;
    sensor->width = sensor->end_time - sensor->start_time;
    sensor->read_index++;
    sensor->read_index %= AVERAGE_DATA_NUMBER;
    filter_width(sensor_index, sensor);
}

inline void filter_width(uint8_t sensor_index, TSSP *sensor)
{
    width_temp[sensor_index][sensor->read_index] = sensor->width;
    uint32_t avg_sum = 0;
    for (uint8_t i = 0; i < AVERAGE_DATA_NUMBER; i++)
    {
        avg_sum += width_temp[sensor_index][i];
    }
    sensor->filtered_width = avg_sum / AVERAGE_DATA_NUMBER;
}

inline void decompose_sensor_width(uint8_t sensor_index, TSSP *sensor)
{
    sensor->Ax = (float)sensor->filtered_width * sensor_cos[sensor_index];
    sensor->Ay = (float)sensor->filtered_width * sensor_sin[sensor_index];
}

inline void measure_ball_data(TSSP sensors[16], BALL *ball)
{
    for (uint8_t i = 0; i < 16; i++)
    {
        decompose_sensor_width(i, &sensors[i]);
        ball->sigma_x += sensors[i].Ax;
        ball->sigma_y += sensors[i].Ay;

        if (sensors[i].width >= ball->max_value)
        {
            ball->max_value = sensors[i].width;
            ball->max_sensor = i;
        }
    }

    if (ball->max_sensor < 3 || ball->max_sensor > 13)
    {
        ball->direction = E;
    }
    else if (ball->max_sensor >= 3 && ball->max_sensor < 6)
    {
        ball->direction = N;
    }
    else if (ball->max_sensor >= 6 && ball->max_sensor <= 10)
    {
        ball->direction = W;
    }
    else
    {
        ball->direction = S;
    }

    ball->angle = (int)(atan2(ball->sigma_y, ball->sigma_x) * RADIAN_TO_DEGREE); // Get the angle & convert it from radian to degree
    ball->angle -= 90;
    if (ball->angle < 0)
        ball->angle += 360;
    ball->angle *= -1;
    ball->angle += 360;

    ball->distance = (int)(sqrt((ball->sigma_x * ball->sigma_x) + (ball->sigma_y * ball->sigma_y)) / DISTANCE_OFFSET);

    ball->sigma_x = 0;
    ball->sigma_y = 0;

    ball->max_value = 0;
    // ball->max_sensor = 0;
}
