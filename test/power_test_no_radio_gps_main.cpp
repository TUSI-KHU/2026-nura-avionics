#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include "../src/app/app_config.h"
#include "board_pinmap.h"
#include "../src/core/logger/log_output.h"
#include "../src/core/logger/logger.h"
#include "../src/hal/h3lis331dl_hal.h"
#include "../src/hal/lis3mdl_hal.h"
#include "../src/hal/lsm6dso32_hal.h"
#include "../src/hal/mpl3115a2_hal.h"
#if defined(NURA_POWER_TEST_ENABLE_GPS)
#include "hal/ublox_m6_gnss_hal.h"
#endif
#include "nura_constants.h"
#include "../src/sensors/barometer_task.h"
#if defined(NURA_POWER_TEST_ENABLE_GPS)
#include "sensors/gnss_task.h"
#endif
#include "../src/sensors/high_g_imu_task.h"
#include "../src/sensors/imu_task.h"
#include "../src/sensors/magnetometer_task.h"
#if defined(NURA_POWER_TEST_ENABLE_GPS)
#include "state/gps_state.h"
#endif
#include "../src/state/high_g_imu_state.h"
#include "../src/state/imu_state.h"
#include "../src/state/magnetometer_state.h"
#include "../src/state/telemetry_state.h"

namespace
{
class SerialPowerLogOutput : public ILogOutput
{
public:
    bool write(const LogEntry &entry) override
    {
        if (!Serial)
        {
            return false;
        }

        Serial.print("[");
        Serial.print(entry.ts);
        Serial.print("] ");
        Serial.print(logToString(entry.level));
        Serial.print(" ");
        Serial.print(entry.src);
        Serial.print(": ");
        Serial.println(entry.msg);
        return true;
    }
};

DefaultAppConfig config;
Logger logger;
SerialPowerLogOutput logOutput;

ImuState imuState;
HighGImuState highGState;
MagnetometerState magState;
TelemetryState telemetryState;
#if defined(NURA_POWER_TEST_ENABLE_GPS)
GpsState gpsState;
#endif

LSM6DSO32HAL lowImuHal;
H3LIS331DLHAL highGImuHal;
LIS3MDLHAL magHal;
MPL3115A2HAL baroHal;
#if defined(NURA_POWER_TEST_ENABLE_GPS)
UbloxM6GNSSHAL gnssHal;
#endif

IMUTask imuTask(lowImuHal, imuState, logger, config);
HighGImuTask highGTask(highGImuHal,
                       highGState,
                       telemetryState,
                       logger,
                       config,
                       BoardPinMap::H3LIS331DL::csPin,
                       H3LIS331DLRange::RANGE_200G);
MagnetometerTask magTask(magHal, magState, telemetryState, logger, config);
BarometerTask baroTask(baroHal, telemetryState, logger, config);
#if defined(NURA_POWER_TEST_ENABLE_GPS)
GNSSTask gnssTask(gnssHal, gpsState, config);
#endif

uint32_t lastImuMs = 0UL;
uint32_t lastHighGMs = 0UL;
uint32_t lastMagMs = 0UL;
uint32_t lastBaroMs = 0UL;
#if defined(NURA_POWER_TEST_ENABLE_GPS)
uint32_t lastGnssMs = 0UL;
#endif
uint32_t lastHeartbeatMs = 0UL;
uint32_t lowImuReads = 0UL;
uint32_t highGReads = 0UL;
uint32_t magReads = 0UL;
uint32_t baroReads = 0UL;
#if defined(NURA_POWER_TEST_ENABLE_GPS)
uint32_t gnssReads = 0UL;
#endif
bool initOk = false;

void flushLogs()
{
    while (!logger.empty())
    {
        const LogFlushResult flushed = logger.flushTo(logOutput, Logger::kMaxBufferSize);
        if (flushed.drained == 0U)
        {
            break;
        }
    }
}

void initializeBusesNoRadio()
{
    pinMode(BoardPinMap::LSM6DSO32::csPin, OUTPUT);
    pinMode(BoardPinMap::H3LIS331DL::csPin, OUTPUT);
    pinMode(BoardPinMap::Ra01DevelopmentLoRa::ssPin, OUTPUT);
    pinMode(BoardPinMap::Ra01DevelopmentLoRa::resetPin, OUTPUT);
    pinMode(BoardPinMap::Ra01DevelopmentLoRa::dio0Pin, INPUT);
    digitalWrite(BoardPinMap::LSM6DSO32::csPin, HIGH);
    digitalWrite(BoardPinMap::H3LIS331DL::csPin, HIGH);
    digitalWrite(BoardPinMap::Ra01DevelopmentLoRa::ssPin, HIGH);
    digitalWrite(BoardPinMap::Ra01DevelopmentLoRa::resetPin, HIGH);

#if !defined(NURA_POWER_TEST_ENABLE_GPS)
    pinMode(BoardPinMap::UbloxM6::rxPin, INPUT);
    pinMode(BoardPinMap::UbloxM6::txPin, INPUT);
#endif

    SPI.setMOSI(BoardPinMap::SpiBus::mosiPin);
    SPI.setMISO(BoardPinMap::SpiBus::misoPin);
    SPI.setSCK(BoardPinMap::SpiBus::sckPin);
    SPI.begin();

    TwoWire &i2c = BoardPinMap::I2cBus::wire();
    i2c.setSDA(BoardPinMap::I2cBus::sdaPin);
    i2c.setSCL(BoardPinMap::I2cBus::sclPin);
    i2c.begin();
    i2c.setClock(BoardPinMap::I2cBus::clockHz);
}

void printHeartbeat(uint32_t nowMs)
{
    if (!Serial || (nowMs - lastHeartbeatMs) < 2000UL)
    {
        return;
    }

    lastHeartbeatMs = nowMs;
#if defined(NURA_POWER_TEST_ENABLE_GPS)
    Serial.print("POWER_GPS_NO_LORA t=");
#else
    Serial.print("POWER_NO_GPS_LORA t=");
#endif
    Serial.print(nowMs);
    Serial.print(" low=");
    Serial.print(lowImuReads);
    Serial.print(" high=");
    Serial.print(highGReads);
    Serial.print(" mag=");
    Serial.print(magReads);
    Serial.print(" baro=");
    Serial.print(baroReads);
#if defined(NURA_POWER_TEST_ENABLE_GPS)
    Serial.print(" gps=");
    Serial.print(gnssReads);
    Serial.print(" gps_chars=");
    Serial.print(gpsState.data.charsProcessed);
    Serial.print(" gps_pass=");
    Serial.print(gpsState.data.passedChecksum);
    Serial.print(" gps_fix=");
    Serial.print(gpsState.data.hasFix ? "yes" : "no");
#endif
    Serial.print(" baro_fault=");
    Serial.print(telemetryState.barometer.fault ? "yes" : "no");
    Serial.print(" high_ok=");
    Serial.print(telemetryState.health.highAccelOk ? "yes" : "no");
    Serial.print(" mag_ok=");
    Serial.println(telemetryState.health.magOk ? "yes" : "no");
}
} // namespace

void setup()
{
    Serial.begin(NuraConstants::App::kSerialBaudRate);
    pinMode(BoardPinMap::StatusIndicator::pin, OUTPUT);
    initializeBusesNoRadio();

    const bool lowOk = imuTask.init();
    const bool highOk = highGTask.init();
    const bool magOk = magTask.init();
    const bool baroOk = baroTask.init();
#if defined(NURA_POWER_TEST_ENABLE_GPS)
    const bool gnssOk = gnssTask.init();
    initOk = lowOk && highOk && magOk && baroOk && gnssOk;
#else
    initOk = lowOk && highOk && magOk && baroOk;
#endif

    if (Serial)
    {
#if defined(NURA_POWER_TEST_ENABLE_GPS)
        Serial.println("NURA power test: low/high IMU + mag + baro + GPS; LoRa software disabled");
#else
        Serial.println("NURA power test: low/high IMU + mag + baro only; GPS and LoRa software disabled");
#endif
        Serial.print("INIT low=");
        Serial.print(lowOk ? "ok" : "fail");
        Serial.print(" high=");
        Serial.print(highOk ? "ok" : "fail");
        Serial.print(" mag=");
        Serial.print(magOk ? "ok" : "fail");
        Serial.print(" baro=");
        Serial.print(baroOk ? "ok" : "fail");
#if defined(NURA_POWER_TEST_ENABLE_GPS)
        Serial.print(" gps=");
        Serial.print(gnssOk ? "ok" : "fail");
#endif
        Serial.println();
    }
    flushLogs();
}

void loop()
{
    const uint32_t nowMs = millis();
    digitalWrite(BoardPinMap::StatusIndicator::pin, initOk ? ((nowMs / 500UL) & 1U) : HIGH);

    if ((nowMs - lastImuMs) >= imuTask.periodMs())
    {
        imuTask.tick(nowMs);
        lastImuMs = nowMs;
        ++lowImuReads;
    }
    if ((nowMs - lastHighGMs) >= highGTask.periodMs())
    {
        highGTask.tick(nowMs);
        lastHighGMs = nowMs;
        ++highGReads;
    }
    if ((nowMs - lastMagMs) >= magTask.periodMs())
    {
        magTask.tick(nowMs);
        lastMagMs = nowMs;
        ++magReads;
    }
    if ((nowMs - lastBaroMs) >= baroTask.periodMs())
    {
        baroTask.tick(nowMs);
        lastBaroMs = nowMs;
        ++baroReads;
    }
#if defined(NURA_POWER_TEST_ENABLE_GPS)
    if ((nowMs - lastGnssMs) >= gnssTask.periodMs())
    {
        gnssTask.tick(nowMs);
        lastGnssMs = nowMs;
        ++gnssReads;
    }
#endif

    printHeartbeat(nowMs);
    flushLogs();
}
