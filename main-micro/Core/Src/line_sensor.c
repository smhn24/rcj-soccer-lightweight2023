#include "line_sensor.h"

extern Robot robot;

void read_line_sensors(bool *line_sensors)
{
    uint8_t out_data[3];
    HAL_I2C_Master_Receive(&hi2c2, LINE_SENSORS_ADDRESS, (uint8_t *)out_data, 3, 200);
    for (uint8_t i = 0; i < 8; i++)
    {
        line_sensors[i] = (out_data[2] & 0x01) == 1;
        out_data[2] >>= 1;
    }
    for (uint8_t i = 8; i < 16; i++)
    {
        line_sensors[i] = (out_data[1] & 0x01) == 1;
        out_data[1] >>= 1;
    }
    for (uint8_t i = 16; i < 20; i++)
    {
        line_sensors[i] = (out_data[0] & 0x01) == 1;
        out_data[0] >>= 1;
    }
}

uint8_t on_line_sensors_number(bool *line_sensors)
{
    uint8_t on_line_sensors = 0;
    for (uint8_t i = 0; i < 20; i++)
    {
        if (line_sensors[i])
        {
            on_line_sensors++;
        }
    }
    return on_line_sensors;
}

void get_edges(bool *line_sensors, direction_t out_direction, uint8_t *out_edges)
{
    switch (out_direction)
    {
    case LEFT:
        for (uint8_t i = 16; i >= 11; i--)
        {
            if (line_sensors[i])
            {
                out_edges[0] = i;
                break;
            }
        }
        for (uint8_t i = 6; i <= 11; i++)
        {
            if (line_sensors[i])
            {
                out_edges[1] = i;
                break;
            }
        }
        break;
    case RIGHT:
        for (uint8_t i = 16; i <= 20; i++)
        {
            if (i == 20 && line_sensors[0])
            {
                out_edges[0] = 20;
                break;
            }
            else if (line_sensors[i])
            {
                out_edges[0] = i;
                break;
            }
        }
        for (uint8_t i = 6; i >= 1; i--)
        {
            if (line_sensors[i])
            {
                out_edges[1] = i;
                break;
            }
        }
        break;
    case FRONT:
        for (uint8_t i = 1; i <= 6; i++)
        {
            if (line_sensors[i])
            {
                out_edges[0] = i;
                break;
            }
        }
        for (uint8_t i = 11; i > 6; i--)
        {
            if (line_sensors[i])
            {
                out_edges[1] = i;
                break;
            }
        }
        break;
    case BACK:
        for (uint8_t i = 10; i <= 16; i++)
        {
            if (line_sensors[i])
            {
                out_edges[0] = i;
                break;
            }
        }

        for (uint8_t i = 16; i < 21; i++)
        {
            if (i == 20 && line_sensors[0])
            {
                out_edges[1] = 20;
                break;
            }

            else if (line_sensors[i])
            {
                out_edges[1] = i;
                break;
            }
        }
        break;

    default:
        break;
    }
}

void get_out_direction(bool *line_sensors)
{
    for (uint8_t i = 0; i < 20; i++)
    {
        if (line_sensors[i])
        {
            robot.first_out_sensor = i;
            if (i > 8 && i < 14) //? Left
            {
                robot.out_direction = LEFT;
            }
            else if (i >= 14 && i <= 18) //? Back
            {
                robot.out_direction = BACK;
            }
            else if (i > 18 || i < 4) //? Right
            {
                robot.out_direction = RIGHT;
            }
            else if (i >= 4 && i <= 8) //? Front
            {
                robot.out_direction = FRONT;
            }
            break;
        }
    }
}

void get_out_angle()
{
    robot.out_angle = (robot.out_edges[0] + robot.out_edges[1]) / 2;
    robot.out_angle *= 18;
    // robot.out_angle -= 18; //! Needs to check the offset
    if (robot.out_direction == RIGHT)
    {
        robot.out_angle = abs(robot.out_angle - 180);
    }

    //? Convert grading system
    robot.out_angle -= 90;
    if (robot.out_angle < 0)
        robot.out_angle += 360;
    robot.out_angle *= -1;
    robot.out_angle += 360;
}

void get_out_error()
{
    robot.out_error = robot.out_direction != RIGHT ? abs(robot.out_edges[0] - robot.out_edges[1]) : abs(robot.out_edges[0] - (robot.out_edges[1] + 20));
}
