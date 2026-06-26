#include <Arduino.h>

#include "board_pinmap.h"
#include "nura_constants.h"

namespace
{
constexpr uint8_t kPins[] = {
    BoardPinMap::Pyro2::gpio1Pin,
    BoardPinMap::Pyro2::gpio2Pin,
    BoardPinMap::Pyro1::gpio1Pin,
    BoardPinMap::Pyro1::gpio2Pin,
};

void drivePinsHigh()
{
    for (const uint8_t pin : kPins)
    {
        pinMode(pin, OUTPUT);
        digitalWrite(pin, HIGH);
    }
}

void printPins()
{
    Serial.print("PYRO_PINS_HOLD_HIGH pins=");
    for (size_t i = 0; i < (sizeof(kPins) / sizeof(kPins[0])); ++i)
    {
        if (i != 0U)
        {
            Serial.print(',');
        }
        Serial.print(kPins[i]);
    }
    Serial.println();
}
} // namespace

void setup()
{
    Serial.begin(NuraConstants::App::kSerialBaudRate);
    delay(1000);
    drivePinsHigh();
    printPins();
}

void loop()
{
    while (true)
    {
        drivePinsHigh();
        delay(100);
    }
}
