#include <Arduino.h>
#include <Wire.h>
#include <MS5611.h>
#include <math.h>

// ==================== PIN MAP / USER CONFIG ====================
#define SERIAL_BAUD 115200
#define I2C_SDA_PIN 18
#define I2C_SCL_PIN 19
#define MS5611_I2C_ADDR 0x77
#define MS5611_SAMPLE_COUNT 40
#define MS5611_SAMPLE_DELAY_MS 50
#define MS5611_SEA_LEVEL_MBAR 1013.25f
// ================================================================

MS5611 baro(MS5611_I2C_ADDR, &Wire);

static bool pressureLooksValid(float pressurePa)
{
    return isfinite(pressurePa) && pressurePa >= 1000.0f && pressurePa <= 120000.0f;
}

static bool temperatureLooksValid(float temperatureC)
{
    return isfinite(temperatureC) && temperatureC >= -40.0f && temperatureC <= 85.0f;
}

static bool promLooksValid()
{
    bool ok = true;
    Serial.print("prom=");
    for (uint8_t i = 0; i <= 7U; ++i)
    {
        const uint16_t word = baro.getProm(i);
        Serial.print(word, HEX);
        if (i < 7U)
        {
            Serial.print(",");
        }
        if (i >= 1U && i <= 6U && word == 0U)
        {
            ok = false;
        }
    }
    Serial.print(" crc=");
    Serial.println(baro.getCRC(), HEX);
    return ok;
}

static void printSample(uint16_t index)
{
    Serial.print("sample=");
    Serial.print(index);
    Serial.print(" pressure_pa=");
    Serial.print(baro.getPressurePascal(), 2);
    Serial.print(" temp_c=");
    Serial.print(baro.getTemperature(), 2);
    Serial.print(" altitude_m=");
    Serial.println(baro.getAltitude(MS5611_SEA_LEVEL_MBAR), 2);
}

static bool runDefectTest()
{
    bool anyInvalid = false;

    for (uint16_t i = 0; i < MS5611_SAMPLE_COUNT; ++i)
    {
        const int result = baro.read();
        if (result != MS5611_READ_OK ||
            !pressureLooksValid(baro.getPressurePascal()) ||
            !temperatureLooksValid(baro.getTemperature()))
        {
            anyInvalid = true;
            Serial.print("bad_read_result=");
            Serial.println(result);
            continue;
        }

        if ((i % 5U) == 0U)
        {
            printSample(i);
        }

        delay(MS5611_SAMPLE_DELAY_MS);
    }

    if (anyInvalid)
    {
        Serial.println("FAIL: MS5611 produced invalid read/status");
        return false;
    }

    Serial.println("PASS: MS5611 responded, PROM looks sane, and pressure reads are plausible");
    return true;
}

void setup()
{
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 4000)
    {
    }

    Serial.println();
    Serial.println("MS5611 barometer defect test");

    Wire.setSDA(I2C_SDA_PIN);
    Wire.setSCL(I2C_SCL_PIN);
    Wire.begin();

    if (!baro.begin())
    {
        Serial.println("FAIL: MS5611 not found or reset/PROM load failed");
        return;
    }

    baro.setOversampling(OSR_HIGH);

    if (!promLooksValid())
    {
        Serial.println("FAIL: PROM calibration words contain zero");
        return;
    }

    Serial.println("PASS: init/PROM OK");
    runDefectTest();
}

void loop()
{
    if (baro.read() == MS5611_READ_OK)
    {
        printSample(0);
    }
    else
    {
        Serial.println("FAIL: read error");
    }
    delay(1000);
}
