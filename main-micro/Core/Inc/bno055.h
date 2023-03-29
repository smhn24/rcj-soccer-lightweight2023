#ifndef BNO055_H
#define BNO055_H

#include "i2c.h"

// #define BNO055_ADDRESS 0x29 << 1
#define BNO055_ADDRESS 0x28 << 1
#define hI2C hi2c2

void BNO055_Config(void);
int16_t BNO055_read(void);

#endif
