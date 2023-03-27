#include "helpers.h"

uint32_t convert24to32(uint8_t msb, uint8_t mid, uint8_t lsb)
{
    uint32_t result = 0;
    result |= ((uint32_t)msb << 16);
    result |= ((uint32_t)mid << 8);
    result |= (uint32_t)lsb;
    // result <<= 8;
    return result;
}