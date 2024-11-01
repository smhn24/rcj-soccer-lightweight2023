#ifndef MPU6050_H
#define MPU6050_H

#include "stm32f4xx.h"
#include "Reg.h"
#include "Val.h"

extern I2C_HandleTypeDef hi2c2;

uint8_t read_i2c(unsigned char BusAddres, unsigned char Reg, unsigned char Ack);
void write_i2c(unsigned char BusAddres, unsigned char Reg, unsigned char Data);
unsigned char MPU6050_Test_I2C(void); // TODO: Test with uint8_t
void MPU6050_Sleep(char ON_or_OFF);
void MPU6050_Reset(void);
void MPU6050_Init(void);
void MPU6050_Calibration();
int16_t Read_MPU6050();

#endif
