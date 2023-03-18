#include "bno055.h"

void BNO055_Config(void)
{
    uint8_t configData[2];
    configData[0] = 0x3E;
    configData[1] = 0x00;

    HAL_I2C_Master_Transmit(&hI2C, BNO055_ADDRESS, configData, 2, 100);
    HAL_Delay(50);

    configData[0] = 0x3D;
    configData[1] = 0x0C;

    HAL_I2C_Master_Transmit(&hI2C, BNO055_ADDRESS, configData, 2, 100);
    HAL_Delay(50);
}

int16_t BNO055_read(void)
{
    uint8_t angleData[2];
    int16_t angle;

    uint8_t regAddr = 0x1A;
    HAL_I2C_Master_Transmit(&hI2C, BNO055_ADDRESS, &regAddr, 1, 100);

    HAL_I2C_Master_Receive(&hI2C, BNO055_ADDRESS, angleData, 2, 100);

    angle = ((int16_t)(angleData[0] | angleData[1] << 8) / 16);
    return angle;
}
