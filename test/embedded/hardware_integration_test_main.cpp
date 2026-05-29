#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <math.h>

#include "app/app_config.h"
#include "board_pinmap.h"
#include "core/logger/log_output.h"
#include "core/logger/logger.h"
#include "hal/h3lis331dl_hal.h"
#include "hal/lis3mdl_hal.h"
#include "hal/lsm6dso32_hal.h"
#include "hal/mpl3115a2_hal.h"
#include "hal/sx127x_lora_hal.h"
#include "hal/ublox_m6_gnss_hal.h"
#include "nura_constants.h"
#include "nura_protocol_v1_lite.h"
#include "sensors/barometer_task.h"
#include "sensors/gnss_task.h"
#include "sensors/high_g_imu_task.h"
#include "sensors/imu_task.h"
#include "sensors/magnetometer_task.h"
#include "state/gps_state.h"
#include "state/high_g_imu_state.h"
#include "state/imu_state.h"
#include "state/magnetometer_state.h"
#include "state/telemetry_state.h"

namespace
{
    struct TestStats
    {
        uint8_t total = 0;
        uint8_t failed = 0;
    };

    class SerialTestLogOutput : public ILogOutput
    {
    public:
        bool write(const LogEntry &entry) override
        {
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

    void flushLogs(Logger &logger, SerialTestLogOutput &output)
    {
        while (!logger.empty())
        {
            const LogFlushResult result = logger.flushTo(output, Logger::kMaxBufferSize);
            if (result.drained == 0U)
            {
                break;
            }
        }
    }

    void printResult(TestStats &stats, const char *name, bool pass)
    {
        ++stats.total;
        if (!pass)
        {
            ++stats.failed;
        }

        Serial.print("TEST ");
        Serial.print(name);
        Serial.print(" ");
        Serial.println(pass ? "PASS" : "FAIL");
    }

    bool finite3(float x, float y, float z)
    {
        return isfinite(x) && isfinite(y) && isfinite(z);
    }

    bool pressurePlausible(float pressurePa)
    {
        return isfinite(pressurePa) &&
               pressurePa >= NuraConstants::Diagnostics::kHardwareIntegrationMinGroundPressurePa &&
               pressurePa <= NuraConstants::Diagnostics::kHardwareIntegrationMaxGroundPressurePa;
    }

    uint8_t readSpiRegisterRaw(uint8_t csPin, uint32_t spiFrequency, uint8_t spiMode, uint8_t address)
    {
        SPISettings settings(spiFrequency, MSBFIRST, spiMode);
        pinMode(csPin, OUTPUT);
        digitalWrite(csPin, HIGH);
        SPI.beginTransaction(settings);
        digitalWrite(csPin, LOW);
        delayMicroseconds(20);
        SPI.transfer(address | 0x80U);
        const uint8_t value = SPI.transfer(0x00U);
        delayMicroseconds(20);
        digitalWrite(csPin, HIGH);
        SPI.endTransaction();
        return value;
    }

    void printLsm6dso32WhoAmI()
    {
        const uint8_t modes[4] = {SPI_MODE0, SPI_MODE1, SPI_MODE2, SPI_MODE3};

        Serial.print("LSM6DSO32_SPI_WHOAMI");
        for (uint8_t i = 0U; i < 4U; ++i)
        {
            const uint8_t value = readSpiRegisterRaw(BoardPinMap::LSM6DSO32::csPin,
                                                     NuraConstants::LSM6DSO32::kProbeSpiHz,
                                                     modes[i],
                                                     NuraConstants::LSM6DSO32::kWhoAmIRegister);
            Serial.print(" m");
            Serial.print(i);
            Serial.print("=0x");
            if (value < 0x10U)
            {
                Serial.print("0");
            }
            Serial.print(value, HEX);
        }
        Serial.println();
    }

    void initializeBuses()
    {
        pinMode(BoardPinMap::LSM6DSO32::csPin, OUTPUT);
        pinMode(BoardPinMap::H3LIS331DL::csPin, OUTPUT);
        pinMode(BoardPinMap::Ra01DevelopmentLoRa::ssPin, OUTPUT);
        digitalWrite(BoardPinMap::LSM6DSO32::csPin, HIGH);
        digitalWrite(BoardPinMap::H3LIS331DL::csPin, HIGH);
        digitalWrite(BoardPinMap::Ra01DevelopmentLoRa::ssPin, HIGH);

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

    Sx127xLoRaConfig makeRadioConfig(const DefaultAppConfig &config)
    {
        Sx127xLoRaConfig radio;
        radio.frequencyHz = config.loraFrequencyHz();
        radio.spiFrequency = config.loraSpiFrequencyHz();
        radio.txPowerDbm = config.loraTxPowerDbm();
        radio.spreadingFactor = config.loraSpreadingFactor();
        radio.signalBandwidthHz = config.loraSignalBandwidthHz();
        radio.codingRateDenominator = config.loraCodingRateDenominator();
        radio.preambleLength = config.loraPreambleLength();
        radio.syncWord = config.loraSyncWord();
        radio.initAttempts = config.loraInitAttempts();
        radio.spiMode = config.loraSpiMode();
        radio.probeSpiMode = config.loraProbeSpiMode();
        return radio;
    }

    void resetRadioForProbe(const Sx127xLoRaConfig &config)
    {
        pinMode(config.ssPin, OUTPUT);
        digitalWrite(config.ssPin, HIGH);
        if (config.resetPin < 0)
        {
            return;
        }

        pinMode(config.resetPin, OUTPUT);
        digitalWrite(config.resetPin, LOW);
        delay(50);
        digitalWrite(config.resetPin, HIGH);
        delay(500);
    }

    uint8_t readLoraRegisterRaw(const Sx127xLoRaConfig &config, uint8_t spiMode, uint8_t address)
    {
        SPISettings settings(config.spiFrequency, MSBFIRST, spiMode);
        pinMode(config.ssPin, OUTPUT);
        digitalWrite(config.ssPin, HIGH);
        SPI.beginTransaction(settings);
        digitalWrite(config.ssPin, LOW);
        delayMicroseconds(20);
        SPI.transfer(address & 0x7FU);
        const uint8_t value = SPI.transfer(0x00U);
        delayMicroseconds(20);
        digitalWrite(config.ssPin, HIGH);
        SPI.endTransaction();
        return value;
    }

    bool probeLoraVersion(const Sx127xLoRaConfig &config)
    {
        const uint8_t modes[4] = {SPI_MODE0, SPI_MODE1, SPI_MODE2, SPI_MODE3};
        bool ok = false;

        resetRadioForProbe(config);
        Serial.print("LORA_SPI_VERSION");
        for (uint8_t i = 0U; i < 4U; ++i)
        {
            const uint8_t version = readLoraRegisterRaw(config, modes[i], NuraConstants::LoRa::kRegVersion);
            Serial.print(" m");
            Serial.print(i);
            Serial.print("=0x");
            if (version < 0x10U)
            {
                Serial.print("0");
            }
            Serial.print(version, HEX);
            ok = ok || version == NuraConstants::LoRa::kExpectedVersion;
        }
        Serial.println();
        return ok;
    }

    bool sendRadioProbeFrame(Sx127xLoRaHAL &radio)
    {
        nura::FastTelemetry fast;
        fast.statusWord = nura::statusWithFlightState(nura::STATUS_RADIO_OK, nura::FLIGHT_BOOT);
        fast.bootMs = millis();

        uint8_t payload[nura::kFastPayloadLen];
        uint8_t frame[nura::kMaxFrameLen];
        if (!nura::encodeFastPayload(fast, payload, sizeof(payload)))
        {
            return false;
        }

        const size_t frameLen = nura::encodeFrame(nura::MESSAGE_FAST_TLM,
                                                  0U,
                                                  payload,
                                                  nura::kFastPayloadLen,
                                                  frame,
                                                  sizeof(frame));
        if (frameLen == 0U)
        {
            return false;
        }

        return radio.send(frame, frameLen, true);
    }

    void printSensorSnapshot(const ImuState &imuState,
                             const HighGImuState &highGState,
                             const MagnetometerState &magState,
                             const TelemetryState &telemetryState,
                             const GpsState &gpsState)
    {
        Serial.print("SNAP low_acc_mps2=");
        Serial.print(imuState.data.accelXMps2, 3);
        Serial.print(",");
        Serial.print(imuState.data.accelYMps2, 3);
        Serial.print(",");
        Serial.print(imuState.data.accelZMps2, 3);
        Serial.print(" gyro_dps=");
        Serial.print(imuState.data.gyroXDps, 3);
        Serial.print(",");
        Serial.print(imuState.data.gyroYDps, 3);
        Serial.print(",");
        Serial.print(imuState.data.gyroZDps, 3);
        Serial.print(" rpy_deg=");
        Serial.print(imuState.data.rollDeg, 2);
        Serial.print(",");
        Serial.print(imuState.data.pitchDeg, 2);
        Serial.print(",");
        Serial.print(imuState.data.yawDeg, 2);
        Serial.print(" tilt_deg=");
        Serial.print(imuState.data.tiltAngleDeg, 2);
        Serial.print(" high_g=");
        Serial.print(highGState.accelXG, 3);
        Serial.print(",");
        Serial.print(highGState.accelYG, 3);
        Serial.print(",");
        Serial.print(highGState.accelZG, 3);
        Serial.print(" mag_uT=");
        Serial.print(magState.magXuT, 3);
        Serial.print(",");
        Serial.print(magState.magYuT, 3);
        Serial.print(",");
        Serial.print(magState.magZuT, 3);
        Serial.print(" pressure_pa=");
        Serial.print(telemetryState.barometer.pressurePa, 2);
        Serial.print(" gps_chars=");
        Serial.print(gpsState.data.charsProcessed);
        Serial.print(" gps_pass_checksum=");
        Serial.print(gpsState.data.passedChecksum);
        Serial.print(" gps_fix=");
        Serial.println(gpsState.data.hasFix ? "yes" : "no");
    }

    void runHardwareIntegrationTest()
    {
        TestStats stats;
        DefaultAppConfig config;
        Logger logger;
        SerialTestLogOutput logOutput;

        ImuState imuState;
        HighGImuState highGState;
        MagnetometerState magState;
        TelemetryState telemetryState;
        GpsState gpsState;

        LSM6DSO32HAL lowImuHal;
        H3LIS331DLHAL highGImuHal;
        LIS3MDLHAL magHal;
        MPL3115A2HAL baroHal;
        UbloxM6GNSSHAL gnssHal;
        Sx127xLoRaHAL radioHal;

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
        GNSSTask gnssTask(gnssHal, gpsState, config);

        initializeBuses();

        Serial.println("NURA hardware integration test");
        Serial.print("I2C=");
        Serial.print(BoardPinMap::I2cBus::name());
        Serial.println(" SPI=11/12/13 GPS=Serial3 15/14");
        printLsm6dso32WhoAmI();

        printResult(stats, "low_imu_task_init", imuTask.init());
        printResult(stats, "high_g_task_init", highGTask.init());
        printResult(stats, "mag_task_init", magTask.init());
        printResult(stats, "barometer_task_init", baroTask.init());
        printResult(stats, "gnss_task_init", gnssTask.init());
        flushLogs(logger, logOutput);

        uint32_t lastImuMs = 0UL;
        uint32_t lastHighGMs = 0UL;
        uint32_t lastMagMs = 0UL;
        uint32_t lastBaroMs = 0UL;
        uint32_t lastGnssMs = 0UL;
        uint32_t lastSnapMs = 0UL;
        const uint32_t startMs = millis();

        while ((millis() - startMs) < NuraConstants::Diagnostics::kHardwareIntegrationExerciseMs)
        {
            const uint32_t nowMs = millis();
            if ((nowMs - lastImuMs) >= imuTask.periodMs())
            {
                imuTask.tick(nowMs);
                lastImuMs = nowMs;
            }
            if ((nowMs - lastHighGMs) >= highGTask.periodMs())
            {
                highGTask.tick(nowMs);
                lastHighGMs = nowMs;
            }
            if ((nowMs - lastMagMs) >= magTask.periodMs())
            {
                magTask.tick(nowMs);
                lastMagMs = nowMs;
            }
            if ((nowMs - lastBaroMs) >= baroTask.periodMs())
            {
                baroTask.tick(nowMs);
                lastBaroMs = nowMs;
            }
            if ((nowMs - lastGnssMs) >= gnssTask.periodMs())
            {
                gnssTask.tick(nowMs);
                lastGnssMs = nowMs;
            }
            if ((nowMs - lastSnapMs) >= 1000UL)
            {
                printSensorSnapshot(imuState, highGState, magState, telemetryState, gpsState);
                lastSnapMs = nowMs;
            }
            flushLogs(logger, logOutput);
            delay(2);
        }

        const bool lowImuOk = imuState.data.lastUpdatedMs != 0UL &&
                              finite3(imuState.data.accelXMps2,
                                      imuState.data.accelYMps2,
                                      imuState.data.accelZMps2) &&
                              finite3(imuState.data.gyroXDps,
                                      imuState.data.gyroYDps,
                                      imuState.data.gyroZDps);
        const bool highGOk = highGState.connected &&
                             highGState.hasNewData &&
                             highGState.whoAmI == 0x32U &&
                             telemetryState.health.highAccelOk &&
                             finite3(highGState.accelXMps2,
                                     highGState.accelYMps2,
                                     highGState.accelZMps2);
        const bool magOk = magState.connected &&
                           magState.hasNewData &&
                           telemetryState.health.magOk &&
                           finite3(magState.magXuT, magState.magYuT, magState.magZuT);
        const bool baroOk = telemetryState.barometer.valid &&
                            !telemetryState.barometer.fault &&
                            pressurePlausible(telemetryState.barometer.pressurePa);
        const bool gpsElectricalOk = gpsState.data.charsProcessed > 0UL &&
                                     gpsState.data.passedChecksum > 0UL;

        printSensorSnapshot(imuState, highGState, magState, telemetryState, gpsState);
        printResult(stats, "low_imu_task_read", lowImuOk);
        printResult(stats, "high_g_task_read", highGOk);
        printResult(stats, "mag_task_read", magOk);
        printResult(stats, "barometer_task_read", baroOk);
        printResult(stats, "gps_nmea_checksum", gpsElectricalOk);

        const Sx127xLoRaConfig radioConfig = makeRadioConfig(config);
        const bool radioProbeOk = probeLoraVersion(radioConfig);
        bool radioInitOk = false;
        bool radioTxOk = false;
        if (radioProbeOk)
        {
            Serial.println("LORA_HAL_BEGIN_START");
            radioInitOk = radioHal.begin(radioConfig, SPI);
            Serial.println("LORA_HAL_BEGIN_DONE");
            if (radioInitOk)
            {
                Serial.println("LORA_HAL_TX_START");
                radioTxOk = sendRadioProbeFrame(radioHal);
                Serial.println("LORA_HAL_TX_DONE");
                delay(100);
                radioHal.end();
            }
        }

        printResult(stats, "lora_spi_version", radioProbeOk);
        printResult(stats, "lora_hal_init", radioInitOk);
        printResult(stats, "lora_hal_tx", radioTxOk);

        Serial.print("SUMMARY ");
        Serial.print(stats.failed == 0U ? "PASS" : "FAIL");
        Serial.print(" total=");
        Serial.print(stats.total);
        Serial.print(" failed=");
        Serial.println(stats.failed);
    }
}

void setup()
{
    Serial.begin(NuraConstants::App::kSerialBaudRate);
    while (!Serial && millis() < 4000UL)
    {
    }

    runHardwareIntegrationTest();
}

void loop()
{
}
