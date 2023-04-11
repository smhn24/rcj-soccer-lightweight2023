#include "bno055.h"
#include "helpers.h"

// void BNO055_Config()
// {
//     i2c_start();
//     i2c_write_address(BNO055_ADDRESS);
//     i2c_write(0x3E);
//     i2c_write(0x00);
//     i2c_stop();
//     LL_mDelay(50);

//     i2c_start();
//     i2c_write_address(BNO055_ADDRESS);
//     i2c_write(0x3D);
//     i2c_write(0x0C);
//     i2c_stop();
//     LL_mDelay(50);
// }

// int16_t BNO055_read()
// {
//     int16_t angle;

//     i2c_start();
//     i2c_write_address(BNO055_ADDRESS);
//     i2c_write(0x1A);
//     i2c_start();
//     i2c_write_address(BNO055_ADDRESS | 0x01);
//     angle = (int16_t)((int16_t)(i2c_read(true) | i2c_read(false) << 8) / 16.00);
//     i2c_stop();
//     return angle;
// }

// void BNO055_Config(void)
// {
//     uint8_t configData[2];

//     //? Use external crystal
//     // configData[0] = 0x3F;
//     // configData[1] = 0xC1;
//     // HAL_I2C_Master_Transmit(&hi2c2, BNO055_ADDRESS, configData, 2, 100);
//     // HAL_Delay(50);

//     configData[0] = 0x3E;
//     configData[1] = 0x00;
//     HAL_I2C_Master_Transmit(&hI2C, BNO055_ADDRESS, configData, 2, 100);
//     HAL_Delay(50);

//     configData[0] = 0x3D;
//     // configData[1] = 0x0C;
//     configData[1] = 0x0B;

//     HAL_I2C_Master_Transmit(&hI2C, BNO055_ADDRESS, configData, 2, 100);
//     HAL_Delay(50);
// }

void BNO055_Config(void)
{
    uint8_t configData[2];
    configData[0] = 0x3F;
    configData[1] = 0x20;
    // HAL_I2C_Master_Transmit(&hi2c2, BNO055_ADDRESS, configData, 2, 100);
    I2Cdev_writeByte(BNO055_ADDRESS, 0x3F, 0x20);
    HAL_Delay(700);

    configData[0] = 0x3E;
    configData[1] = 0x00;
    // HAL_I2C_Master_Transmit(&hi2c2, BNO055_ADDRESS, configData, 2, 100);
    I2Cdev_writeByte(BNO055_ADDRESS, 0x3E, 0x00);
    HAL_Delay(50);

    configData[0] = 0x3D;
    configData[1] = 0x0C;
    // HAL_I2C_Master_Transmit(&hi2c2, BNO055_ADDRESS, configData, 2, 100);
    I2Cdev_writeByte(BNO055_ADDRESS, 0x3D, 0x0C);
    HAL_Delay(50);
}

int16_t BNO055_read(void)
{
    uint8_t angleData[2];
    int16_t angle;

    uint8_t regAddr = 0x1A;
    // uint16_t DevAddress = 0x29 << 1;
    // HAL_I2C_Master_Transmit(&hI2C, BNO055_ADDRESS, &regAddr, 1, 100);
    // HAL_I2C_Master_Receive(&hI2C, BNO055_ADDRESS, angleData, 2, 100);

    I2Cdev_readBytes(BNO055_ADDRESS, regAddr, 2, angleData, 100);

    angle = ((int16_t)(angleData[0] | angleData[1] << 8) / 16);
    return angle;
}
