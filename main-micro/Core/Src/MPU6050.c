#include "MPU6050.h"

uint8_t read_i2c(unsigned char BusAddres, unsigned char Reg, unsigned char Ack)
{
    uint8_t data[2], Data;
    data[0] = Reg;

    HAL_I2C_Master_Transmit(&hi2c2, BusAddres, data, 1, 100000);
    HAL_I2C_Master_Receive(&hi2c2, BusAddres, &Data, 1, 100000);

    return Data;
}

void write_i2c(unsigned char BusAddres, unsigned char Reg, unsigned char Data)
{
    uint8_t data[3];
    data[0] = Reg;
    data[1] = Data;

    HAL_I2C_Master_Transmit(&hi2c2, BusAddres, data, 2, 100000);
}

unsigned char MPU6050_Test_I2C()
{
    unsigned char Data = 0x00;
    Data = read_i2c(MPU6050_ADDRESS, RA_WHO_AM_I, 0);
    if (Data == 0x68)
        return 1; // Means Comunication With MPU6050 is Corect
    else
        return 0; // Means ERROR, Stopping
}

// This function can move MPU6050 to sleep
void MPU6050_Sleep(char ON_or_OFF)
{
    if (ON_or_OFF == on)
        write_i2c(MPU6050_ADDRESS, RA_PWR_MGMT_1, (1 << 6) | (CYCLE << 5) | (TEMP_DIS << 3) | CLKSEL);
    else if (ON_or_OFF == off)
        write_i2c(MPU6050_ADDRESS, RA_PWR_MGMT_1, (0) | (CYCLE << 5) | (TEMP_DIS << 3) | CLKSEL);
}

// This function can restor MPU6050 to default
void MPU6050_Reset()
{
    // When set to 1, DEVICE_RESET bit in RA_PWR_MGMT_1 resets all internal registers to their default values.
    // The bit automatically clears to 0 once the reset is done.
    // The default values for each register can be found in RA_MPU6050.h
    write_i2c(MPU6050_ADDRESS, RA_PWR_MGMT_1, 0x80);
    // Now all reg reset to default values
}

// MPU6050 sensor initialization
void MPU6050_Init()
{
    write_i2c(MPU6050_ADDRESS, RA_PWR_MGMT_1, 0x00); ////Set the register PWR_MGMT_1  bits as 00000000 to activate the gyro.

    write_i2c(MPU6050_ADDRESS, RA_GYRO_CONFIG, 0x08); ////Set the register GYRO_CONFIG bits as 00001000 (500dps full scale).

    write_i2c(MPU6050_ADDRESS, RA_ACCEL_CONFIG, 0x10); ////Set the register ACCEL_CONFIG bits as 00010000 (+/- 8g full scale range).

    write_i2c(MPU6050_ADDRESS, RA_CONFIG, 0x03); ////Set the register CONFIG bits as 00000011 (Set Digital Low Pass Filter to ~43Hz).

    ////  MPU6050 Setup Complete
}