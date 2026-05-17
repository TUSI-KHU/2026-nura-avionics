#include <Arduino.h>
#include "board_pinmap.h"
#include <math.h>

// ==================== PIN MAP / USER CONFIG ====================
#define SERIAL_BAUD 115200
#define ADXL377_PIN_X BoardPinMap::ADXL377::xPin
#define ADXL377_PIN_Y BoardPinMap::ADXL377::yPin
#define ADXL377_PIN_Z BoardPinMap::ADXL377::zPin
#define ADC_RESOLUTION_BITS 12
#define ADC_REFERENCE_VOLTAGE 3.3f
#define ADXL377_ZERO_G_VOLTAGE (ADC_REFERENCE_VOLTAGE * 0.5f)
#define ADXL377_SENSITIVITY_MV_PER_G 7.15f
#define ADXL377_SAMPLE_COUNT 200
#define ADXL377_SAMPLE_DELAY_MS 5
// ================================================================

struct AxisStats
{
    uint16_t minRaw;
    uint16_t maxRaw;
    uint32_t sumRaw;
};

static float rawToVoltage(uint16_t raw)
{
    const float adcMax = static_cast<float>((1UL << ADC_RESOLUTION_BITS) - 1UL);
    return (static_cast<float>(raw) / adcMax) * ADC_REFERENCE_VOLTAGE;
}

static float voltageToG(float voltage)
{
    return (voltage - ADXL377_ZERO_G_VOLTAGE) / (ADXL377_SENSITIVITY_MV_PER_G * 0.001f);
}

static void resetStats(AxisStats &stats)
{
    stats.minRaw = 65535U;
    stats.maxRaw = 0U;
    stats.sumRaw = 0U;
}

static void addSample(AxisStats &stats, uint16_t raw)
{
    stats.minRaw = min(stats.minRaw, raw);
    stats.maxRaw = max(stats.maxRaw, raw);
    stats.sumRaw += raw;
}

static void printAxis(const char *name, const AxisStats &stats)
{
    const float avgRaw = static_cast<float>(stats.sumRaw) / static_cast<float>(ADXL377_SAMPLE_COUNT);
    const float voltage = rawToVoltage(static_cast<uint16_t>(avgRaw));
    const float accelG = voltageToG(voltage);

    Serial.print(name);
    Serial.print(" raw_avg=");
    Serial.print(avgRaw, 2);
    Serial.print(" raw_range=");
    Serial.print(stats.minRaw);
    Serial.print("..");
    Serial.print(stats.maxRaw);
    Serial.print(" voltage=");
    Serial.print(voltage, 4);
    Serial.print(" accel_g=");
    Serial.println(accelG, 2);
}

static bool axisLooksConnected(const AxisStats &stats)
{
    const uint16_t railMargin = static_cast<uint16_t>(((1UL << ADC_RESOLUTION_BITS) - 1UL) / 50UL);
    return stats.maxRaw > railMargin &&
           stats.minRaw < static_cast<uint16_t>(((1UL << ADC_RESOLUTION_BITS) - 1UL) - railMargin);
}

static bool runDefectTest()
{
    AxisStats x;
    AxisStats y;
    AxisStats z;
    resetStats(x);
    resetStats(y);
    resetStats(z);

    for (uint16_t i = 0; i < ADXL377_SAMPLE_COUNT; ++i)
    {
        addSample(x, static_cast<uint16_t>(analogRead(ADXL377_PIN_X)));
        addSample(y, static_cast<uint16_t>(analogRead(ADXL377_PIN_Y)));
        addSample(z, static_cast<uint16_t>(analogRead(ADXL377_PIN_Z)));
        delay(ADXL377_SAMPLE_DELAY_MS);
    }

    printAxis("x", x);
    printAxis("y", y);
    printAxis("z", z);

    if (!axisLooksConnected(x) || !axisLooksConnected(y) || !axisLooksConnected(z))
    {
        Serial.println("FAIL: at least one analog axis is railed or disconnected");
        return false;
    }

    Serial.println("PASS: ADXL377 analog outputs are readable and not railed");
    Serial.println("WARN: this does not calibrate scale/offset; rotate axes to verify response direction");
    return true;
}

void setup()
{
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 4000)
    {
    }

    Serial.println();
    Serial.println("ADXL377 high-g analog accelerometer defect test");
    analogReadResolution(ADC_RESOLUTION_BITS);
    runDefectTest();
}

void loop()
{
    const uint16_t rawX = static_cast<uint16_t>(analogRead(ADXL377_PIN_X));
    const uint16_t rawY = static_cast<uint16_t>(analogRead(ADXL377_PIN_Y));
    const uint16_t rawZ = static_cast<uint16_t>(analogRead(ADXL377_PIN_Z));

    Serial.print("raw=");
    Serial.print(rawX);
    Serial.print(",");
    Serial.print(rawY);
    Serial.print(",");
    Serial.print(rawZ);
    Serial.print(" accel_g=");
    Serial.print(voltageToG(rawToVoltage(rawX)), 2);
    Serial.print(",");
    Serial.print(voltageToG(rawToVoltage(rawY)), 2);
    Serial.print(",");
    Serial.println(voltageToG(rawToVoltage(rawZ)), 2);

    delay(500);
}
