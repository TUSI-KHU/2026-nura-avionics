#include <Arduino.h>
#include <Wire.h>
#include "../include/board_pinmap.h"
#include <Adafruit_MPL3115A2.h>
#include <math.h>

// ==================== PIN MAP / USER CONFIG ====================
#define SERIAL_BAUD 115200
#define I2C_SDA_PIN BoardPinMap::MPL3115A2::sdaPin
#define I2C_SCL_PIN BoardPinMap::MPL3115A2::sclPin
#define MPL3115A2_SAMPLE_COUNT 40
#define MPL3115A2_CONVERSION_TIMEOUT_MS 150
#define MPL3115A2_SEA_LEVEL_HPA 1013.25f
// MPL3115A2 fixed I2C address is 0x60.
// ================================================================

Adafruit_MPL3115A2 baro;
bool baroReady = false;

static bool waitForConversion()
{
    const uint32_t startMs = millis();
    while ((millis() - startMs) <= MPL3115A2_CONVERSION_TIMEOUT_MS)
    {
        if (baro.conversionComplete())
        {
            return true;
        }
        delay(1);
    }
    return false;
}

static bool pressureLooksValid(float pressureHpa)
{
    const float pressurePa = pressureHpa * 100.0f;
    return isfinite(pressurePa) && pressurePa >= 20000.0f && pressurePa <= 110000.0f;
}

static bool temperatureLooksValid(float temperatureC)
{
    return isfinite(temperatureC) && temperatureC >= -40.0f && temperatureC <= 85.0f;
}

static bool readSample(float &pressureHpa, float &temperatureC)
{
    baro.setMode(MPL3115A2_BAROMETER);
    baro.startOneShot();
    if (!waitForConversion())
    {
        return false;
    }

    pressureHpa = baro.getLastConversionResults(MPL3115A2_PRESSURE);
    temperatureC = baro.getLastConversionResults(MPL3115A2_TEMPERATURE);
    return pressureLooksValid(pressureHpa) && temperatureLooksValid(temperatureC);
}

static void printSample(uint16_t index, float pressureHpa, float temperatureC)
{
    Serial.print("sample=");
    Serial.print(index);
    Serial.print(" pressure_hpa=");
    Serial.print(pressureHpa, 3);
    Serial.print(" pressure_pa=");
    Serial.print(pressureHpa * 100.0f, 2);
    Serial.print(" temp_c=");
    Serial.println(temperatureC, 2);
}

static bool runDefectTest()
{
    bool anyInvalid = false;

    for (uint16_t i = 0; i < MPL3115A2_SAMPLE_COUNT; ++i)
    {
        float pressureHpa = 0.0f;
        float temperatureC = 0.0f;
        if (!readSample(pressureHpa, temperatureC))
        {
            anyInvalid = true;
            Serial.println("bad_read");
            continue;
        }

        if ((i % 5U) == 0U)
        {
            printSample(i, pressureHpa, temperatureC);
        }
    }

    if (anyInvalid)
    {
        Serial.println("FAIL: MPL3115A2 produced invalid sample or conversion timeout");
        return false;
    }

    Serial.println("PASS: MPL3115A2 responded and pressure reads are plausible");
    return true;
}

void setup()
{
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 4000)
    {
    }

    Serial.println();
    Serial.println("MPL3115A2 barometer defect test");

    Wire.setSDA(I2C_SDA_PIN);
    Wire.setSCL(I2C_SCL_PIN);
    Wire.begin();

    if (!baro.begin(&Wire))
    {
        Serial.println("FAIL: MPL3115A2 not found on fixed I2C address 0x60");
        return;
    }

    baroReady = true;
    baro.setSeaPressure(MPL3115A2_SEA_LEVEL_HPA);
    Serial.println("PASS: WHOAMI/init OK");
    runDefectTest();
}

void loop()
{
    if (!baroReady)
    {
        Serial.println("FAIL: MPL3115A2 not initialized; check VIN/3V3, GND, SDA, SCL, and I2C address 0x60");
        delay(1000);
        return;
    }

    float pressureHpa = 0.0f;
    float temperatureC = 0.0f;
    if (readSample(pressureHpa, temperatureC))
    {
        printSample(0, pressureHpa, temperatureC);
    }
    else
    {
        Serial.println("FAIL: read timeout or invalid pressure/temperature");
    }
    delay(1000);
}
