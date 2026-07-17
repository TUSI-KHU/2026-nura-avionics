#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>

#include "board_pinmap.h"
#include "hal/battery_voltage_hal.h"
#include "hal/h3lis331dl_hal.h"
#include "hal/lis3mdl_hal.h"
#include "hal/lsm6dso32_hal.h"
#include "hal/mpl3115a2_hal.h"
#include "hal/ublox_m6_gnss_hal.h"
#include "nura_constants.h"

namespace
{
constexpr uint32_t kPrintPeriodMs = 1000UL;

LSM6DSO32HAL lowImu;
H3LIS331DLHAL highG;
LIS3MDLHAL mag;
MPL3115A2HAL baro;
UbloxM6GNSSHAL gps;
BatteryVoltageHAL battery;

Lsm6dso32Reading lowImuReading;
H3LIS331DLReading highGReading;
Lis3mdlReading magReading;
Mpl3115a2Reading baroReading;
UbloxM6GnssReading gpsReading;
BatteryVoltageReading batteryReading;

bool lowImuOk = false;
bool highGOk = false;
bool magOk = false;
bool baroOk = false;
bool gpsOk = false;
bool batteryOk = false;
uint32_t lastPrintMs = 0UL;

void printBool(const bool value)
{
    Serial.print(value ? "true" : "false");
}

void initializeBuses()
{
    SPI.setMOSI(BoardPinMap::SpiBus::mosiPin);
    SPI.setMISO(BoardPinMap::SpiBus::misoPin);
    SPI.setSCK(BoardPinMap::SpiBus::sckPin);
    SPI.begin();

    auto &i2c0 = BoardPinMap::I2c0Bus::wire();
    i2c0.setSDA(BoardPinMap::I2c0Bus::sdaPin);
    i2c0.setSCL(BoardPinMap::I2c0Bus::sclPin);
    i2c0.begin();
    i2c0.setClock(BoardPinMap::I2c0Bus::clockHz);

    auto &i2c1 = BoardPinMap::I2c1Bus::wire();
    i2c1.setSDA(BoardPinMap::I2c1Bus::sdaPin);
    i2c1.setSCL(BoardPinMap::I2c1Bus::sclPin);
    i2c1.begin();
    i2c1.setClock(BoardPinMap::I2c1Bus::clockHz);
}

void initializeSensors()
{
    lowImuOk = lowImu.begin(BoardPinMap::LSM6DSO32::csPin, SPI);
    highGOk = highG.begin(BoardPinMap::H3LIS331DL::csPin, SPI, H3LIS331DLRange::RANGE_200G);
    magOk = mag.begin(BoardPinMap::LIS3MDL::i2cAddress, BoardPinMap::LIS3MDL::wire());
    baroOk = baro.begin(BoardPinMap::MPL3115A2::wire(), 700U);

    auto &gpsSerial = BoardPinMap::UbloxM6::serial();
    gpsSerial.setRX(BoardPinMap::UbloxM6::rxPin);
    gpsSerial.setTX(BoardPinMap::UbloxM6::txPin);
    gpsOk = gps.begin(gpsSerial, BoardPinMap::UbloxM6::baud, NuraConstants::Sensors::kGnssMaxFixAgeMs);

    batteryOk = battery.begin(BoardPinMap::PowerSense::voltagePin,
                              NuraConstants::Sensors::kPowerSenseAdcReferenceMv,
                              NuraConstants::Sensors::kPowerSenseAdcResolutionBits,
                              NuraConstants::Sensors::kPowerSenseDividerRatioNumerator,
                              NuraConstants::Sensors::kPowerSenseDividerRatioDenominator,
                              NuraConstants::Sensors::kPowerSenseMinValidBatteryMv,
                              NuraConstants::Sensors::kPowerSenseMaxValidBatteryMv);
}

void printInitStatus()
{
    Serial.print("{\"src\":\"sensor_display_init\",\"low_imu\":");
    printBool(lowImuOk);
    Serial.print(",\"high_g\":");
    printBool(highGOk);
    Serial.print(",\"mag\":");
    printBool(magOk);
    Serial.print(",\"baro\":");
    printBool(baroOk);
    Serial.print(",\"gps\":");
    printBool(gpsOk);
    Serial.print(",\"battery\":");
    printBool(batteryOk);
    Serial.println("}");
}

void updateReadings(const uint32_t nowMs)
{
    if (lowImuOk)
    {
        (void)lowImu.read(lowImuReading, nowMs);
    }
    if (highGOk)
    {
        (void)highG.read(highGReading, nowMs);
    }
    if (magOk)
    {
        (void)mag.read(magReading, nowMs);
    }
    if (baroOk)
    {
        (void)baro.read(baroReading, nowMs);
    }
    if (gpsOk)
    {
        (void)gps.poll(gpsReading, nowMs, NuraConstants::Sensors::kGnssPollByteBudget);
    }
    if (batteryOk)
    {
        (void)battery.read(batteryReading, nowMs);
    }
}

void printReadings(const uint32_t nowMs)
{
    Serial.print("{\"src\":\"sensor_display\",\"t_ms\":");
    Serial.print(nowMs);

    Serial.print(",\"low_imu\":{\"ok\":");
    printBool(lowImuOk);
    Serial.print(",\"ax_mps2\":");
    Serial.print(lowImuReading.accelXMps2, 3);
    Serial.print(",\"ay_mps2\":");
    Serial.print(lowImuReading.accelYMps2, 3);
    Serial.print(",\"az_mps2\":");
    Serial.print(lowImuReading.accelZMps2, 3);
    Serial.print(",\"gx_dps\":");
    Serial.print(lowImuReading.gyroXDps, 3);
    Serial.print(",\"gy_dps\":");
    Serial.print(lowImuReading.gyroYDps, 3);
    Serial.print(",\"gz_dps\":");
    Serial.print(lowImuReading.gyroZDps, 3);
    Serial.print(",\"temp_c\":");
    Serial.print(lowImuReading.temperatureC, 2);
    Serial.print("}");

    Serial.print(",\"high_g\":{\"ok\":");
    printBool(highGOk);
    Serial.print(",\"x_g\":");
    Serial.print(highGReading.accelXG, 3);
    Serial.print(",\"y_g\":");
    Serial.print(highGReading.accelYG, 3);
    Serial.print(",\"z_g\":");
    Serial.print(highGReading.accelZG, 3);
    Serial.print(",\"whoami\":");
    Serial.print(highGReading.whoAmI);
    Serial.print("}");

    Serial.print(",\"mag\":{\"ok\":");
    printBool(magOk);
    Serial.print(",\"x_ut\":");
    Serial.print(magReading.magXuT, 2);
    Serial.print(",\"y_ut\":");
    Serial.print(magReading.magYuT, 2);
    Serial.print(",\"z_ut\":");
    Serial.print(magReading.magZuT, 2);
    Serial.print("}");

    Serial.print(",\"baro\":{\"ok\":");
    printBool(baroOk);
    Serial.print(",\"pressure_pa\":");
    Serial.print(baroReading.pressurePa, 1);
    Serial.print(",\"temp_c\":");
    Serial.print(baroReading.temperatureC, 2);
    Serial.print(",\"rel_alt_m\":");
    Serial.print(baroReading.relativeAltitudeM, 2);
    Serial.print("}");

    Serial.print(",\"gps\":{\"ok\":");
    printBool(gpsOk);
    Serial.print(",\"has_fix\":");
    printBool(gpsReading.hasFix);
    Serial.print(",\"lat_deg\":");
    if (gpsReading.hasFix)
    {
        Serial.print(gpsReading.latitudeDeg, 6);
    }
    else
    {
        Serial.print("null");
    }
    Serial.print(",\"lon_deg\":");
    if (gpsReading.hasFix)
    {
        Serial.print(gpsReading.longitudeDeg, 6);
    }
    else
    {
        Serial.print("null");
    }
    Serial.print(",\"alt_m\":");
    Serial.print(gpsReading.altitudeM, 2);
    Serial.print(",\"speed_mps\":");
    Serial.print(gpsReading.speedMps, 2);
    Serial.print(",\"course_deg\":");
    Serial.print(gpsReading.courseDeg, 2);
    Serial.print(",\"sats\":");
    Serial.print(gpsReading.satellites);
    Serial.print(",\"hdop\":");
    Serial.print(gpsReading.hdop, 2);
    Serial.print(",\"chars\":");
    Serial.print(gpsReading.charsProcessed);
    Serial.print(",\"pass_checksum\":");
    Serial.print(gpsReading.passedChecksum);
    Serial.print(",\"fail_checksum\":");
    Serial.print(gpsReading.failedChecksum);
    Serial.print("}");

    Serial.print(",\"battery\":{\"ok\":");
    printBool(batteryOk);
    Serial.print(",\"valid\":");
    printBool(batteryReading.valid);
    Serial.print(",\"raw_adc\":");
    Serial.print(batteryReading.rawAdc);
    Serial.print(",\"sense_mv\":");
    Serial.print(batteryReading.senseMv);
    Serial.print(",\"battery_mv\":");
    Serial.print(batteryReading.batteryMv);
    Serial.println("}}");
}
} // namespace

void setup()
{
    Serial.begin(NuraConstants::App::kSerialBaudRate);
    while (!Serial && millis() < 4000U)
    {
    }

    Serial.println();
    Serial.println("# NURA sensor display test");
    Serial.println("# Pyro outputs are not initialized by this firmware.");

    initializeBuses();
    initializeSensors();
    printInitStatus();
}

void loop()
{
    const uint32_t nowMs = millis();
    updateReadings(nowMs);

    if (nowMs - lastPrintMs >= kPrintPeriodMs)
    {
        lastPrintMs = nowMs;
        printReadings(nowMs);
    }
}
