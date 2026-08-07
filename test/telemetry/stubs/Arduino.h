#pragma once

#include <stdint.h>

inline uint32_t nuraTestMillis = 0UL;

inline uint32_t millis()
{
    return nuraTestMillis;
}

inline void delay(uint32_t durationMs)
{
    nuraTestMillis += durationMs;
}

class SPIClass
{
};

inline SPIClass SPI1;
