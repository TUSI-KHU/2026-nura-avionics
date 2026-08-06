#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include "board_pinmap.h"
#include "hal/battery_voltage_hal.h"
#include "hal/h3lis331dl_hal.h"
#include "hal/lis3mdl_hal.h"
#include "hal/lsm6dso32_hal.h"
#include "hal/mpl3115a2_hal.h"
#include "hal/ublox_m6_gnss_hal.h"
#include "live_sensor_monitor_output.h"
#include "nura_constants.h"

namespace
{
constexpr uint32_t kPrintPeriodMs = 250UL;

LSM6DSO32HAL lowImu;
H3LIS331DLHAL highImu;
LIS3MDLHAL magnetometer;
MPL3115A2HAL barometer;
UbloxM6GNSSHAL gnss;
BatteryVoltageHAL battery;

bool lowReady = false;
bool highReady = false;
bool magReady = false;
bool baroReady = false;
bool gnssReady = false;
bool batteryReady = false;

Mpl3115a2Reading baroReading;
UbloxM6GnssReading gnssReading;
bool baroSampleValid = false;
uint32_t lastPrintMs = 0UL;

void initializeBuses()
{
    pinMode(BoardPinMap::LSM6DSO32::csPin, OUTPUT);
    pinMode(BoardPinMap::H3LIS331DL::csPin, OUTPUT);
    digitalWrite(BoardPinMap::LSM6DSO32::csPin, HIGH);
    digitalWrite(BoardPinMap::H3LIS331DL::csPin, HIGH);

    SPI.setMOSI(BoardPinMap::SpiBus::mosiPin);
    SPI.setMISO(BoardPinMap::SpiBus::misoPin);
    SPI.setSCK(BoardPinMap::SpiBus::sckPin);
    SPI.begin();

    TwoWire &i2c0 = BoardPinMap::I2c0Bus::wire();
    i2c0.setSDA(BoardPinMap::I2c0Bus::sdaPin);
    i2c0.setSCL(BoardPinMap::I2c0Bus::sclPin);
    i2c0.begin();
    i2c0.setClock(BoardPinMap::I2c0Bus::clockHz);

    TwoWire &i2c1 = BoardPinMap::I2c1Bus::wire();
    i2c1.setSDA(BoardPinMap::I2c1Bus::sdaPin);
    i2c1.setSCL(BoardPinMap::I2c1Bus::sclPin);
    i2c1.begin();
    i2c1.setClock(BoardPinMap::I2c1Bus::clockHz);

    auto &gpsSerial = BoardPinMap::UbloxM6::serial();
    gpsSerial.setRX(BoardPinMap::UbloxM6::rxPin);
    gpsSerial.setTX(BoardPinMap::UbloxM6::txPin);
}

}

void setup()
{
    Serial.begin(NuraConstants::App::kSerialBaudRate);
    const uint32_t serialStartMs = millis();
    while (!Serial && (millis() - serialStartMs) < 5000UL)
    {
        yield();
    }

    initializeBuses();
    delay(NuraConstants::App::kBusSettleDelayMs);

    lowReady = lowImu.begin(BoardPinMap::LSM6DSO32::csPin);
    highReady = highImu.begin(BoardPinMap::H3LIS331DL::csPin, SPI, H3LIS331DLRange::RANGE_200G);
    magReady = magnetometer.begin(BoardPinMap::LIS3MDL::i2cAddress, BoardPinMap::LIS3MDL::wire());
    baroReady = barometer.begin(BoardPinMap::MPL3115A2::wire());
    if (baroReady)
    {
        baroReady = barometer.calibrateGroundBaseline(32U, 10U, 0.0f);
    }
    gnssReady = gnss.begin(BoardPinMap::UbloxM6::serial(),
                           BoardPinMap::UbloxM6::baud,
                           NuraConstants::Sensors::kGnssMaxFixAgeMs);
    batteryReady = battery.begin(BoardPinMap::PowerSense::voltagePin,
                                 NuraConstants::Sensors::kPowerSenseAdcReferenceMv,
                                 NuraConstants::Sensors::kPowerSenseAdcResolutionBits,
                                 NuraConstants::Sensors::kPowerSenseDividerRatioNumerator,
                                 NuraConstants::Sensors::kPowerSenseDividerRatioDenominator,
                                 NuraConstants::Sensors::kPowerSenseMinValidBatteryMv,
                                 NuraConstants::Sensors::kPowerSenseMaxValidBatteryMv);
    printSensorInitStatus({lowReady, highReady, magReady, baroReady, gnssReady, batteryReady});
}

void loop()
{
    const uint32_t nowMs = millis();
    if (gnssReady)
    {
        (void)gnss.poll(gnssReading, nowMs, NuraConstants::Sensors::kGnssPollByteBudget);
    }
    if (baroReady)
    {
        Mpl3115a2Reading sample;
        const Mpl3115a2PollResult result = barometer.poll(sample, nowMs);
        if (result == Mpl3115a2PollResult::READY)
        {
            baroReading = sample;
            baroSampleValid = true;
        }
    }

    if ((nowMs - lastPrintMs) >= kPrintPeriodMs)
    {
        Lsm6dso32Reading lowReading;
        H3LIS331DLReading highReading;
        Lis3mdlReading magReading;
        BatteryVoltageReading batteryReading;
        const bool lowOk = lowReady && lowImu.read(lowReading, nowMs);
        const bool highOk = highReady && highImu.read(highReading, nowMs);
        const bool magOk = magReady && magnetometer.read(magReading, nowMs);
        const bool batteryOk = batteryReady && battery.read(batteryReading, nowMs);

        lastPrintMs = nowMs;
        Serial.print("SAMPLE t_ms=");
        Serial.println(nowMs);
        printLowImu(lowOk ? &lowReading : nullptr);
        printHighImu(highOk ? &highReading : nullptr);
        printMagnetometer(magOk ? &magReading : nullptr);
        printBarometer(baroReady && baroSampleValid ? &baroReading : nullptr, nowMs);
        printGnss(gnssReading, gnssReady);
        printBattery(batteryOk ? &batteryReading : nullptr);
        Serial.println("---");
    }
    yield();
}
