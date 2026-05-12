#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_H3LIS331.h>
#include <Adafruit_Sensor.h>
#include <math.h>

// ==================== PIN MAP / USER CONFIG ====================
#define SERIAL_BAUD 115200
#define I2C_SDA_PIN 18
#define I2C_SCL_PIN 19
#define H3LIS331DL_I2C_ADDR 0x18
#define H3LIS331DL_SAMPLE_COUNT 80
#define H3LIS331DL_SAMPLE_DELAY_MS 10
// ================================================================

Adafruit_H3LIS331 accel;

static bool validAccel(const sensors_event_t &event)
{
    return isfinite(event.acceleration.x) &&
           isfinite(event.acceleration.y) &&
           isfinite(event.acceleration.z);
}

static void printSample(uint16_t index, const sensors_event_t &event)
{
    Serial.print("sample=");
    Serial.print(index);
    Serial.print(" accel_mps2=");
    Serial.print(event.acceleration.x, 4);
    Serial.print(",");
    Serial.print(event.acceleration.y, 4);
    Serial.print(",");
    Serial.print(event.acceleration.z, 4);
    Serial.print(" raw=");
    Serial.print(accel.x);
    Serial.print(",");
    Serial.print(accel.y);
    Serial.print(",");
    Serial.println(accel.z);
}

static bool runDefectTest()
{
    bool anyInvalid = false;
    bool anyNonZeroRaw = false;

    for (uint16_t i = 0; i < H3LIS331DL_SAMPLE_COUNT; ++i)
    {
        sensors_event_t event;
        if (!accel.getEvent(&event) || !validAccel(event))
        {
            anyInvalid = true;
            continue;
        }

        if (accel.x != 0 || accel.y != 0 || accel.z != 0)
        {
            anyNonZeroRaw = true;
        }

        if ((i % 10U) == 0U)
        {
            printSample(i, event);
        }

        delay(H3LIS331DL_SAMPLE_DELAY_MS);
    }

    if (anyInvalid)
    {
        Serial.println("FAIL: invalid or non-finite sample detected");
        return false;
    }

    if (!anyNonZeroRaw)
    {
        Serial.println("FAIL: all raw axes stayed zero");
        return false;
    }

    Serial.println("PASS: H3LIS331DL responded and produced usable acceleration samples");
    return true;
}

void setup()
{
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 4000)
    {
    }

    Serial.println();
    Serial.println("H3LIS331DL high-g accelerometer defect test");

    Wire.setSDA(I2C_SDA_PIN);
    Wire.setSCL(I2C_SCL_PIN);
    Wire.begin();

    if (!accel.begin_I2C(H3LIS331DL_I2C_ADDR, &Wire))
    {
        Serial.println("FAIL: H3LIS331DL not found on configured I2C address");
        return;
    }

    accel.setRange(H3LIS331_RANGE_200_G);
    accel.setDataRate(LIS331_DATARATE_1000_HZ);

    Serial.println("PASS: WHOAMI/init OK");
    runDefectTest();
}

void loop()
{
    sensors_event_t event;
    if (accel.getEvent(&event))
    {
        printSample(0, event);
    }
    delay(1000);
}
