#pragma once

#include <Arduino.h>
#include <Wire.h>

namespace BoardPinMap
{
constexpr uint8_t kUnassignedPin = 255U;

struct StatusIndicator final
{
    static constexpr uint8_t pin = 33U;
    static constexpr uint8_t led1Pin = 34U;
    static constexpr uint8_t led2Pin = 33U;
};

struct Buzzer final
{
    static constexpr uint8_t pin = 2U;
};

struct SpiBus final
{
    static constexpr uint8_t mosiPin = 11U;
    static constexpr uint8_t misoPin = 12U;
    static constexpr uint8_t sckPin = 13U;
};

struct Spi1Bus final
{
    static constexpr uint8_t misoPin = 1U;
    static constexpr uint8_t mosiPin = 26U;
    static constexpr uint8_t sckPin = 27U;
};

struct I2c0Bus final
{
    static TwoWire &wire()
    {
        return Wire;
    }

    static constexpr uint8_t sdaPin = 18U;
    static constexpr uint8_t sclPin = 19U;
    static constexpr uint32_t clockHz = 100000UL;
};

struct I2c1Bus final
{
    static TwoWire &wire()
    {
        return Wire1;
    }

    static constexpr uint8_t sdaPin = 17U;
    static constexpr uint8_t sclPin = 16U;
    static constexpr uint32_t clockHz = 100000UL;
};

struct MPL3115A2 final
{
    static TwoWire &wire()
    {
        return I2c0Bus::wire();
    }

    static constexpr uint8_t sdaPin = I2c0Bus::sdaPin;
    static constexpr uint8_t sclPin = I2c0Bus::sclPin;
    static constexpr uint8_t i2cAddress = 0x60U;
};

struct BMP180 final
{
    static TwoWire &wire()
    {
        return I2c0Bus::wire();
    }

    static constexpr uint8_t sdaPin = I2c0Bus::sdaPin;
    static constexpr uint8_t sclPin = I2c0Bus::sclPin;
    static constexpr uint8_t i2cAddress = 0x77U;
};

struct LIS3MDL final
{
    static TwoWire &wire()
    {
        return I2c1Bus::wire();
    }

    static constexpr uint8_t sdaPin = I2c1Bus::sdaPin;
    static constexpr uint8_t sclPin = I2c1Bus::sclPin;
    static constexpr uint8_t i2cAddress = 0x1CU;
};

struct LSM6DSO32 final
{
    static constexpr uint8_t csPin = 10U;
};

struct H3LIS331DL final
{
    static constexpr uint8_t csPin = 0U;
};

struct UbloxM6 final
{
    static decltype(Serial3) &serial()
    {
        return Serial3;
    }

    static constexpr uint8_t rxPin = 15U;
    static constexpr uint8_t txPin = 14U;
    static constexpr uint32_t baud = 38400UL;
};

struct MicroSD final
{
#if defined(BUILTIN_SDCARD)
    static constexpr uint8_t csPin = BUILTIN_SDCARD;
#else
    static constexpr uint8_t csPin = 10U;
#endif
};

struct Ra01DevelopmentLoRa final
{
    static constexpr uint8_t ssPin = 9U;
    static constexpr uint8_t resetPin = 30U;
    static constexpr int8_t libraryResetPin = -1;
    static constexpr uint8_t dio0Pin = 31U;
    static constexpr uint8_t busyPin = 32U;
};

struct Sx1262LoRa final
{
    static constexpr uint8_t ssPin = 9U;
    static constexpr uint8_t rxEnablePin = 30U;
    static constexpr int8_t resetPin = -1;
    static constexpr uint8_t dio1Pin = 31U;
    static constexpr uint8_t busyPin = 32U;
};

struct Pyro1 final
{
    static constexpr uint8_t gpio1Pin = 28U;
    static constexpr uint8_t gpio2Pin = 29U;
    static constexpr uint8_t sensePin = 25U;
};

struct Pyro2 final
{
    static constexpr uint8_t gpio1Pin = 38U;
    static constexpr uint8_t gpio2Pin = 35U;
    static constexpr uint8_t sensePin = 41U;
};

using DroguePyro = Pyro1;
using MainPyro = Pyro2;

struct PowerSense final
{
    static constexpr uint8_t voltagePin = 22U;
    static constexpr bool conflictsWithPyroOutput =
        voltagePin == Pyro1::gpio1Pin ||
        voltagePin == Pyro1::gpio2Pin ||
        voltagePin == Pyro2::gpio1Pin ||
        voltagePin == Pyro2::gpio2Pin;
};
} // namespace BoardPinMap
