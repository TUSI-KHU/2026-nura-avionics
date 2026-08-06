#include "live_sensor_monitor_output.h"

#include <Arduino.h>
#include <math.h>

#include "nura_constants.h"

namespace
{
float vectorNorm(float x, float y, float z)
{
    return sqrtf((x * x) + (y * y) + (z * z));
}
}

void printSensorInitStatus(const SensorInitStatus &status)
{
    Serial.println("NURA LIVE SENSOR MONITOR");
    Serial.println("FSM=OFF PYRO=NOT_COMPILED LORA=NOT_COMPILED STORAGE=OFF");
    Serial.print("INIT low_imu=");
    Serial.print(status.lowImu ? "ok" : "fail");
    Serial.print(" high_imu=");
    Serial.print(status.highImu ? "ok" : "fail");
    Serial.print(" mag=");
    Serial.print(status.magnetometer ? "ok" : "fail");
    Serial.print(" baro=");
    Serial.print(status.barometer ? "ok" : "fail");
    Serial.print(" gps_uart=");
    Serial.print(status.gnss ? "ok" : "fail");
    Serial.print(" battery_adc=");
    Serial.println(status.battery ? "ok" : "fail");
}

void printLowImu(const Lsm6dso32Reading *reading)
{
    Serial.print("LOW ok=");
    Serial.print(reading != nullptr ? 1 : 0);
    if (reading != nullptr)
    {
        Serial.print(" accel_mps2=");
        Serial.print(reading->accelXMps2, 4);
        Serial.print(',');
        Serial.print(reading->accelYMps2, 4);
        Serial.print(',');
        Serial.print(reading->accelZMps2, 4);
        Serial.print(" norm_g=");
        Serial.print(vectorNorm(reading->accelXMps2, reading->accelYMps2, reading->accelZMps2) /
                         NuraConstants::Physics::kGravityMps2,
                     4);
        Serial.print(" gyro_dps=");
        Serial.print(reading->gyroXDps, 3);
        Serial.print(',');
        Serial.print(reading->gyroYDps, 3);
        Serial.print(',');
        Serial.print(reading->gyroZDps, 3);
        Serial.print(" temp_c=");
        Serial.print(reading->temperatureC, 2);
        Serial.print(" raw_accel=");
        Serial.print(reading->rawAccelX);
        Serial.print(',');
        Serial.print(reading->rawAccelY);
        Serial.print(',');
        Serial.print(reading->rawAccelZ);
    }
    Serial.println();
}

void printHighImu(const H3LIS331DLReading *reading)
{
    Serial.print("HIGH ok=");
    Serial.print(reading != nullptr ? 1 : 0);
    if (reading != nullptr)
    {
        Serial.print(" whoami=0x");
        Serial.print(reading->whoAmI, HEX);
        Serial.print(" accel_g=");
        Serial.print(reading->accelXG, 4);
        Serial.print(',');
        Serial.print(reading->accelYG, 4);
        Serial.print(',');
        Serial.print(reading->accelZG, 4);
        Serial.print(" norm_g=");
        Serial.print(vectorNorm(reading->accelXG, reading->accelYG, reading->accelZG), 4);
        Serial.print(" raw=");
        Serial.print(reading->rawX);
        Serial.print(',');
        Serial.print(reading->rawY);
        Serial.print(',');
        Serial.print(reading->rawZ);
    }
    Serial.println();
}

void printMagnetometer(const Lis3mdlReading *reading)
{
    Serial.print("MAG ok=");
    Serial.print(reading != nullptr ? 1 : 0);
    if (reading != nullptr)
    {
        Serial.print(" field_ut=");
        Serial.print(reading->magXuT, 3);
        Serial.print(',');
        Serial.print(reading->magYuT, 3);
        Serial.print(',');
        Serial.print(reading->magZuT, 3);
        Serial.print(" norm_ut=");
        Serial.print(vectorNorm(reading->magXuT, reading->magYuT, reading->magZuT), 3);
        Serial.print(" raw=");
        Serial.print(reading->rawX);
        Serial.print(',');
        Serial.print(reading->rawY);
        Serial.print(',');
        Serial.print(reading->rawZ);
    }
    Serial.println();
}

void printBarometer(const Mpl3115a2Reading *reading, uint32_t nowMs)
{
    Serial.print("BARO ok=");
    Serial.print(reading != nullptr ? 1 : 0);
    if (reading != nullptr)
    {
        Serial.print(" pressure_pa=");
        Serial.print(reading->pressurePa, 1);
        Serial.print(" pressure_hpa=");
        Serial.print(reading->pressureHpa, 2);
        Serial.print(" temp_c=");
        Serial.print(reading->temperatureC, 2);
        Serial.print(" relative_alt_m=");
        Serial.print(reading->relativeAltitudeM, 2);
        Serial.print(" age_ms=");
        Serial.print(nowMs - reading->sampleMs);
    }
    Serial.println();
}

void printGnss(const UbloxM6GnssReading &reading, bool ready)
{
    Serial.print("GPS uart=");
    Serial.print(ready ? 1 : 0);
    Serial.print(" chars=");
    Serial.print(reading.charsProcessed);
    Serial.print(" checksum_ok=");
    Serial.print(reading.passedChecksum);
    Serial.print(" checksum_fail=");
    Serial.print(reading.failedChecksum);
    Serial.print(" fix=");
    Serial.print(reading.hasFix ? 1 : 0);
    Serial.print(" sats=");
    Serial.print(reading.satellites);
    Serial.print(" hdop=");
    Serial.print(reading.hdop, 2);
    if (reading.hasFix)
    {
        Serial.print(" lat=");
        Serial.print(reading.latitudeDeg, 7);
        Serial.print(" lon=");
        Serial.print(reading.longitudeDeg, 7);
        Serial.print(" alt_m=");
        Serial.print(reading.altitudeM, 2);
    }
    Serial.println();
}

void printBattery(const BatteryVoltageReading *reading)
{
    Serial.print("POWER read_ok=");
    Serial.print(reading != nullptr ? 1 : 0);
    if (reading != nullptr)
    {
        Serial.print(" valid=");
        Serial.print(reading->valid ? 1 : 0);
        Serial.print(" raw_adc=");
        Serial.print(reading->rawAdc);
        Serial.print(" sense_mv=");
        Serial.print(reading->senseMv);
        Serial.print(" battery_mv=");
        Serial.print(reading->batteryMv);
    }
    Serial.println();
}
