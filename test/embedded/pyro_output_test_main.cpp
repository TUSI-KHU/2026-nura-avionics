#include <Arduino.h>

#include "board_pinmap.h"
#include "hal/mosfet_pyro_hal.h"
#include "nura_constants.h"

namespace
{
constexpr uint32_t kSerialWaitMs = 60000UL;
constexpr uint32_t kTestPulseMs = 5000UL;
constexpr uint32_t kOffGapMs = 3000UL;

MosfetPyroHAL pyro;
uint32_t pulseEndMs = 0U;
uint32_t nextAutoStepMs = 0U;
bool pyro1Active = false;
bool pyro2Active = false;
uint8_t autoStep = 0U;

void printHelp()
{
    Serial.println();
    Serial.println("NURA PYRO OUTPUT TEST");
    Serial.println("AUTO CYCLE ENABLED:");
    Serial.println("  OFF 3000ms -> Pyro1 5000ms -> OFF 3000ms -> Pyro2 5000ms -> repeat");
    Serial.println("Commands:");
    Serial.println("  1: Pyro1/Drogue ON for 5000ms");
    Serial.println("  2: Pyro2/Main   ON for 5000ms");
    Serial.println("  b: Both         ON for 5000ms");
    Serial.println("  o: All OFF");
    Serial.println("  s: Print sense/input states");
    Serial.println("  h: Help");
    Serial.print("Pyro1 gpio1=");
    Serial.print(BoardPinMap::Pyro1::gpio1Pin);
    Serial.print(" gpio2=");
    Serial.print(BoardPinMap::Pyro1::gpio2Pin);
    Serial.print(" sense=");
    Serial.println(BoardPinMap::Pyro1::sensePin);
    Serial.print("Pyro2 gpio1=");
    Serial.print(BoardPinMap::Pyro2::gpio1Pin);
    Serial.print(" gpio2=");
    Serial.print(BoardPinMap::Pyro2::gpio2Pin);
    Serial.print(" sense=");
    Serial.println(BoardPinMap::Pyro2::sensePin);
}

void setOutputs(bool pyro1, bool pyro2)
{
    pyro.setDrogue(pyro1);
    pyro.setMain(pyro2);
    pyro1Active = pyro1;
    pyro2Active = pyro2;
}

void allOff(const char *reason)
{
    pyro.allOff();
    pyro1Active = false;
    pyro2Active = false;
    pulseEndMs = 0U;
    Serial.print("PYRO_ALL_OFF reason=");
    Serial.println(reason);
}

void startPulse(bool pyro1, bool pyro2, const char *reason)
{
    setOutputs(pyro1, pyro2);
    pulseEndMs = millis() + kTestPulseMs;
    Serial.print("PYRO_PULSE_BEGIN pyro1=");
    Serial.print(pyro1 ? "ON" : "OFF");
    Serial.print(" pyro2=");
    Serial.print(pyro2 ? "ON" : "OFF");
    Serial.print(" duration_ms=");
    Serial.print(kTestPulseMs);
    Serial.print(" reason=");
    Serial.println(reason);
}

void printSense()
{
    Serial.print("PYRO_STATUS pyro1_active=");
    Serial.print(pyro1Active ? "true" : "false");
    Serial.print(" pyro2_active=");
    Serial.print(pyro2Active ? "true" : "false");
    Serial.print(" p1_gpio1=");
    Serial.print(digitalRead(BoardPinMap::Pyro1::gpio1Pin));
    Serial.print(" p1_gpio2=");
    Serial.print(digitalRead(BoardPinMap::Pyro1::gpio2Pin));
    Serial.print(" p1_sense=");
    Serial.print(digitalRead(BoardPinMap::Pyro1::sensePin));
    Serial.print(" p2_gpio1=");
    Serial.print(digitalRead(BoardPinMap::Pyro2::gpio1Pin));
    Serial.print(" p2_gpio2=");
    Serial.print(digitalRead(BoardPinMap::Pyro2::gpio2Pin));
    Serial.print(" p2_sense=");
    Serial.println(digitalRead(BoardPinMap::Pyro2::sensePin));
}
} // namespace

void setup()
{
    Serial.begin(NuraConstants::App::kSerialBaudRate);
    const uint32_t startMs = millis();
    while (!Serial && (millis() - startMs) < kSerialWaitMs)
    {
        delay(10);
    }

    if (!pyro.begin())
    {
        Serial.println("PYRO_INIT_FAIL");
        while (true)
        {
            delay(1000);
        }
    }

    allOff("boot");
    nextAutoStepMs = millis() + kOffGapMs;
    printHelp();
}

void loop()
{
    const uint32_t nowMs = millis();
    if (pulseEndMs != 0U && static_cast<int32_t>(nowMs - pulseEndMs) >= 0)
    {
        allOff("pulse_timeout");
        printSense();
        nextAutoStepMs = nowMs + kOffGapMs;
    }

    if (pulseEndMs == 0U && nextAutoStepMs != 0U && static_cast<int32_t>(nowMs - nextAutoStepMs) >= 0)
    {
        if ((autoStep % 2U) == 0U)
        {
            startPulse(true, false, "auto_pyro1");
        }
        else
        {
            startPulse(false, true, "auto_pyro2");
        }
        ++autoStep;
        printSense();
    }

    if (Serial.available() <= 0)
    {
        delay(5);
        return;
    }

    const char command = static_cast<char>(Serial.read());
    switch (command)
    {
    case '1':
        startPulse(true, false, "command");
        printSense();
        break;
    case '2':
        startPulse(false, true, "command");
        printSense();
        break;
    case 'b':
    case 'B':
        startPulse(true, true, "command");
        printSense();
        break;
    case 'o':
    case 'O':
        allOff("command");
        printSense();
        break;
    case 's':
    case 'S':
        printSense();
        break;
    case 'h':
    case 'H':
        printHelp();
        break;
    default:
        break;
    }
}
