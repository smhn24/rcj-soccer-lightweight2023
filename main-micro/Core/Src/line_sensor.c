#include "line_sensor.h"

extern Robot robot;

volatile uint8_t out_data[4] = {0};

void read_line_sensors(bool *line_sensors)
{
    // HAL_I2C_Master_Receive(&hi2c2, LINE_SENSORS_ADDRESS, (uint8_t *)out_data, 3, 100);
    HAL_I2C_Master_Receive(&hi2c2, LINE_SENSORS_ADDRESS, out_data, 3, 100);
    // HAL_I2C_Master_Receive(&hi2c2, LINE_SENSORS_ADDRESS, &out_data, 4, 50);
    // HAL_I2C_Master_Receive(&hi2c2, LINE_SENSORS_ADDRESS, &out_data[0], 1, 20);
    // HAL_I2C_Master_Receive(&hi2c2, LINE_SENSORS_ADDRESS, &out_data[1], 1, 20);
    // HAL_I2C_Master_Receive(&hi2c2, LINE_SENSORS_ADDRESS, &out_data[2], 1, 20);

    // uint32_t data = 0;
    uint8_t tx[100];
    // data = out_data[2];
    // data |= out_data[1] << 8;
    // data |= out_data[0] << 16;

    // sprintf(tx, "%d   %d   %d   %d\r\n", out_data[0], out_data[1], out_data[2], out_data[3]);
    // HAL_UART_Transmit(&huart4, tx, strlen(tx), 100);

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

    bool temp = line_sensors[0];
    for (uint8_t i = 0; i < 19; i++)
    {
        line_sensors[i] = line_sensors[i + 1];
    }
    line_sensors[19] = temp;
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
