#include "line_sensor.h"

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

void detect_out(bool *line_sensors)
{
    uint8_t on_line_sensors;
    bool out_detect = false;
    uint8_t out_edges[2];
    static out_direction_t out_direction = not_out;

    read_line_sensors(line_sensors); //? Update line_sensors array
    on_line_sensors = on_line_sensors_number(line_sensors);

    if (on_line_sensors != 0 && !out_detect) //? At least one sensor detected line for the first time
    {
        for (uint8_t i = 0; i < 20; i++)
        {
            if (line_sensors[i])
            {
                if (i > 8 && i < 14) //? Left
                {
                    out_direction = left;
                }
                else if (i >= 14 && i <= 18) //? Backward
                {
                    out_direction = backward;
                }
                else if (i > 18 || i < 4) //? Right
                {
                    out_direction = right;
                }
                else if (i >= 4 && i <= 8) //? Forward
                {
                    out_direction = forward;
                }
                out_detect = true;
                break;
            }
        }
    }
    else if (out_detect)
    {
        switch (out_direction)
        {
        case left:
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
        case right:
            for (uint8_t i = 16; i <= 20; i++)
            {
                if (i == 20 && line_sensors[0])
                {
                    out_edges[0] = 0;
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
        case forward:
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
        case backward:
            for (uint8_t i = 11; i < 16; i++)
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
                    out_edges[1] = 0;
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
    else if (on_line_sensors == 0) //? No sensor sees the line
    {
        out_detect = false;
        out_direction = not_out;
    }
}
