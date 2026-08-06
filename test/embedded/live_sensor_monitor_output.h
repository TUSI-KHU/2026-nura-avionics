#pragma once

#include <stdint.h>

#include "hal/battery_voltage_hal.h"
#include "hal/h3lis331dl_hal.h"
#include "hal/lis3mdl_hal.h"
#include "hal/lsm6dso32_hal.h"
#include "hal/mpl3115a2_hal.h"
#include "hal/ublox_m6_gnss_hal.h"

struct SensorInitStatus
{
    bool lowImu;
    bool highImu;
    bool magnetometer;
    bool barometer;
    bool gnss;
    bool battery;
};

void printSensorInitStatus(const SensorInitStatus &status);
void printLowImu(const Lsm6dso32Reading *reading);
void printHighImu(const H3LIS331DLReading *reading);
void printMagnetometer(const Lis3mdlReading *reading);
void printBarometer(const Mpl3115a2Reading *reading, uint32_t nowMs);
void printGnss(const UbloxM6GnssReading &reading, bool ready);
void printBattery(const BatteryVoltageReading *reading);
