#pragma once

#include <Arduino.h>

namespace BoardPinMap
{
struct StatusIndicator final
{
    static constexpr uint8_t pin = LED_BUILTIN;
};

struct LSM6DSO32 final
{
    static constexpr uint8_t sdaPin = 18U;
    static constexpr uint8_t sclPin = 19U;
    static constexpr uint8_t i2cAddress = 0x6AU;
};

struct LSM6DSOX final
{
    static constexpr uint8_t sdaPin = 18U;
    static constexpr uint8_t sclPin = 19U;
    static constexpr uint8_t i2cAddress = 0x6AU;
};

struct H3LIS331DL final
{
    static constexpr uint8_t sdaPin = 18U;
    static constexpr uint8_t sclPin = 19U;
    static constexpr uint8_t i2cAddress = 0x18U;
};

struct ADXL377 final
{
    static constexpr uint8_t xPin = A0;
    static constexpr uint8_t yPin = A1;
    static constexpr uint8_t zPin = A2;
};

struct LIS3MDL final
{
    static constexpr uint8_t sdaPin = 18U;
    static constexpr uint8_t sclPin = 19U;
    static constexpr uint8_t i2cAddress = 0x1CU;
};

struct MS5611 final
{
    static constexpr uint8_t sdaPin = 18U;
    static constexpr uint8_t sclPin = 19U;
    static constexpr uint8_t i2cAddress = 0x77U;
};

struct MPL3115A2 final
{
    static constexpr uint8_t sdaPin = 18U;
    static constexpr uint8_t sclPin = 19U;
    static constexpr uint8_t i2cAddress = 0x60U;
};

struct UbloxM6 final
{
    static decltype(Serial1) &serial()
    {
        return Serial1;
    }

    static constexpr uint8_t rxPin = 0U;
    static constexpr uint8_t txPin = 1U;
    static constexpr uint32_t baud = 9600UL;
};

struct Ra01DevelopmentLoRa final
{
    static constexpr uint8_t ssPin = 10U;
    static constexpr uint8_t resetPin = 9U;
    static constexpr int8_t libraryResetPin = -1;
    static constexpr uint8_t dio0Pin = 2U;
};
} // namespace BoardPinMap
