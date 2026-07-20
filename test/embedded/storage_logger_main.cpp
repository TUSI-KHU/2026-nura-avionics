#include <Arduino.h>
#include <SD.h>
#include <string.h>

#include "board_pinmap.h"
#include "core/logger/logger.h"
#include "logging/flight_log_mirror_storage.h"
#include "logging/flight_log_record.h"
#include "logging/program_flash_flight_log_storage.h"
#include "logging/sd_flight_log_storage.h"
#include "hal/w25q128_qspi_hal.h"
#include "missions/logging/flight_log_task.h"
#include "nura_constants.h"

namespace
{
constexpr uint32_t kSerialWaitMs = 60000UL;
constexpr uint16_t kExpectedMinFrames = 8U;
constexpr uint16_t kVerifyStreamBytes = 16U * 1024U;
constexpr uint32_t kMaxAllowedRuntimeTickUs = 5000UL;
uint32_t maxRuntimeTickUs = 0U;

W25Q128QspiHAL programFlashHal;
ProgramFlashFlightLogStorage programStorage(programFlashHal);
SdFlightLogStorage sdStorage(BoardPinMap::MicroSD::csPin, "/NURA_LOG");
FlightLogMirrorStorage mirrorStorage(programStorage, sdStorage);

FlightState flightState;
ImuState imuState;
HighGImuState highGImuState;
MagnetometerState magnetometerState;
GpsState gpsState;
BarometerState barometerState;
PowerState powerState;
SystemHealthState healthState;
TelemetrySnapshot telemetryState{barometerState, powerState, healthState};
FlightTraceBuffer flightTrace;
Logger logger;
FlightLogTask flightLogTask(flightState,
                            imuState,
                            highGImuState,
                            magnetometerState,
                            gpsState,
                            telemetryState,
                            flightTrace,
                            mirrorStorage,
                            logger);

void tickFlightLog(uint32_t nowMs)
{
    const uint32_t startUs = micros();
    flightLogTask.tick(nowMs);
    const uint32_t durationUs = micros() - startUs;
    if (durationUs > maxRuntimeTickUs)
    {
        maxRuntimeTickUs = durationUs;
    }
}

bool verifyLogFile(FS &fs,
                   const char *path,
                   const char *label,
                   uint16_t minFrames,
                   uint32_t logicalBytes,
                   uint32_t sessionId)
{
    File file = fs.open(path, FILE_READ);
    if (!file)
    {
        Serial.print(label);
        Serial.print(" RESULT FAIL reopen path=");
        Serial.println(path);
        return false;
    }

    static uint8_t stream[kVerifyStreamBytes] = {};
    uint32_t streamBytes = 0U;
    uint8_t block[NuraConstants::Logger::kFlightLogSdSectorBytes] = {};
    while (streamBytes < logicalBytes)
    {
        if (file.read(block, sizeof(block)) != static_cast<int>(sizeof(block)))
        {
            file.close();
            return false;
        }
        nura_sd_log::BlockHeader header = {};
        memcpy(&header, block, sizeof(header));
        const uint16_t storedHeaderCrc = header.headerCrc16;
        header.headerCrc16 = 0U;
        const uint16_t headerCrc = nura_log::crc16Ccitt(
            reinterpret_cast<const uint8_t *>(&header), sizeof(header));
        if (header.magic != nura_sd_log::kBlockMagic ||
            header.version != nura_sd_log::kBlockVersion ||
            header.sessionId != sessionId ||
            header.payloadLength == 0U ||
            header.payloadLength > sizeof(block) - sizeof(header) ||
            storedHeaderCrc != headerCrc ||
            header.payloadCrc16 != nura_log::crc16Ccitt(
                                       block + sizeof(header), header.payloadLength) ||
            streamBytes + header.payloadLength > sizeof(stream))
        {
            file.close();
            return false;
        }
        memcpy(stream + streamBytes, block + sizeof(header), header.payloadLength);
        streamBytes += header.payloadLength;
    }
    file.close();

    uint8_t frame[nura_log::kMaxEncodedFrameBytes] = {};
    uint16_t frames = 0U;
    bool ok = true;
    uint32_t offset = 0U;
    while (offset < streamBytes)
    {
        if (streamBytes - offset < sizeof(nura_log::FrameHeader))
        {
            ok = false;
            break;
        }
        memcpy(frame, stream + offset, sizeof(nura_log::FrameHeader));

        nura_log::FrameHeader header;
        memcpy(&header, frame, sizeof(header));
        const size_t frameLength = sizeof(nura_log::FrameHeader) +
                                   header.payloadLength +
                                   sizeof(uint16_t);
        if (header.magic != nura_log::kFrameMagic ||
            header.version != nura_log::kFrameVersion ||
            frameLength > sizeof(frame))
        {
            ok = false;
            break;
        }

        const size_t remaining = frameLength - sizeof(nura_log::FrameHeader);
        if (offset + frameLength > streamBytes)
        {
            ok = false;
            break;
        }
        memcpy(frame + sizeof(nura_log::FrameHeader),
               stream + offset + sizeof(nura_log::FrameHeader),
               remaining);

        uint16_t storedCrc = 0U;
        memcpy(&storedCrc, frame + frameLength - sizeof(storedCrc), sizeof(storedCrc));
        if (storedCrc != nura_log::crc16Ccitt(frame, frameLength - sizeof(storedCrc)))
        {
            ok = false;
            break;
        }

        ++frames;
        offset += frameLength;
    }

    const uint32_t fileSize = logicalBytes;

    Serial.print(label);
    Serial.print(" RESULT ");
    Serial.print(ok && frames >= minFrames ? "OK" : "FAIL verify");
    Serial.print(" path=");
    Serial.print(path);
    Serial.print(" bytes=");
    Serial.print(fileSize);
    Serial.print(" frames=");
    Serial.println(frames);
    return ok && frames >= minFrames;
}

void seedTelemetry(uint32_t nowMs)
{
    flightState.state = State::SAFE;
    flightState.stateEnteredMs = nowMs;

    imuState.data.accelXMps2 = 0.1f;
    imuState.data.accelYMps2 = 0.2f;
    imuState.data.accelZMps2 = NuraConstants::Physics::kGravityMps2;
    imuState.data.gyroXDps = 1.0f;
    imuState.data.gyroYDps = 2.0f;
    imuState.data.gyroZDps = 3.0f;
    imuState.data.attitudeValid = true;
    imuState.data.tiltValid = true;
    imuState.data.rollDeg = 1.0f;
    imuState.data.pitchDeg = 2.0f;
    imuState.data.yawDeg = 3.0f;
    imuState.data.tiltAngleDeg = 4.0f;
    imuState.data.lastUpdatedMs = nowMs;

    highGImuState.rawX = 10;
    highGImuState.rawY = 20;
    highGImuState.rawZ = 30;
    highGImuState.accelXG = 0.01f;
    highGImuState.accelYG = 0.02f;
    highGImuState.accelZG = 1.00f;
    highGImuState.connected = true;
    highGImuState.hasNewData = true;
    highGImuState.lastUpdatedMs = nowMs;

    magnetometerState.rawX = 100;
    magnetometerState.rawY = 200;
    magnetometerState.rawZ = 300;
    magnetometerState.magXuT = 10.0f;
    magnetometerState.magYuT = 20.0f;
    magnetometerState.magZuT = 30.0f;
    magnetometerState.connected = true;
    magnetometerState.hasNewData = true;
    magnetometerState.lastUpdatedMs = nowMs;

    gpsState.data.hasFix = true;
    gpsState.data.latitudeDeg = 37.1234567;
    gpsState.data.longitudeDeg = 127.1234567;
    gpsState.data.altitudeM = 55.5;
    gpsState.data.speedMps = 12.3;
    gpsState.data.courseDeg = 84.0;
    gpsState.data.hdop = 1.1;
    gpsState.data.satellites = 9U;
    gpsState.data.lastUpdatedMs = nowMs;

    telemetryState.barometer.valid = true;
    telemetryState.barometer.referenceValid = true;
    telemetryState.barometer.pressurePa = 101325.0f;
    telemetryState.barometer.rawAltitudeM = 0.0f;
    telemetryState.barometer.altitudeM = 0.0f;
    telemetryState.barometer.lastUpdatedMs = nowMs;
    telemetryState.health.highAccelOk = true;
    telemetryState.health.magOk = true;
    telemetryState.power.valid = true;
    telemetryState.power.batteryMv = 12000U;
    telemetryState.power.lastUpdatedMs = nowMs;
}

bool runMirrorLoggerTest()
{
    const uint32_t startMs = millis();
    seedTelemetry(startMs);
    maxRuntimeTickUs = 0U;

    if (!flightLogTask.init())
    {
        Serial.println("MIRROR LOGGER RESULT FAIL init");
        return false;
    }

    const bool programStarted = mirrorStorage.primaryHealthy();
    const bool sdStarted = mirrorStorage.mirrorHealthy();

    for (uint8_t i = 0U; i < 60U; ++i)
    {
        const uint32_t nowMs = startMs + (static_cast<uint32_t>(i) * 20UL);
        imuState.data.lastUpdatedMs = nowMs;
        highGImuState.lastUpdatedMs = nowMs;
        telemetryState.barometer.lastUpdatedMs = nowMs;
        telemetryState.barometer.rawAltitudeM = static_cast<float>(i) * 0.5f;
        telemetryState.barometer.altitudeM = telemetryState.barometer.rawAltitudeM;

        if (i == 10U)
        {
            flightState.state = State::ARMED;
            flightState.stateEnteredMs = nowMs;
        }
        else if (i == 20U)
        {
            flightState.state = State::LAUNCH;
            flightState.stateEnteredMs = nowMs;
        }
        else if (i == 30U)
        {
            flightState.state = State::COAST;
            flightState.stateEnteredMs = nowMs;
        }

        tickFlightLog(nowMs);
        delay(20U);
    }

    const uint32_t groundMs = millis();
    flightState.state = State::GROUND;
    flightState.stateEnteredMs = groundMs;
    tickFlightLog(groundMs);

    const uint32_t drainStartMs = millis();
    while (!mirrorStorage.idle() && (millis() - drainStartMs) < 5000UL)
    {
        tickFlightLog(millis());
        delay(20U);
    }

    uint32_t validFlashPages = 0U;
    uint32_t flashPayloadBytes = 0U;
    const bool programOk = programStarted &&
                           programStorage.verifyJournal(validFlashPages, flashPayloadBytes) &&
                           validFlashPages > 0U && flashPayloadBytes > 0U;
    const bool sdOk = sdStarted &&
                      verifyLogFile(SD,
                                    sdStorage.path(),
                                    "SD",
                                    kExpectedMinFrames,
                                    sdStorage.logicalBytesWritten(),
                                    sdStorage.sessionId());

    Serial.print("PROGRAM FLASH JOURNAL ");
    Serial.print(programOk ? "OK" : "FAIL");
    Serial.print(" pages=");
    Serial.print(validFlashPages);
    Serial.print(" payload_bytes=");
    Serial.println(flashPayloadBytes);

    const bool timingOk = maxRuntimeTickUs <= kMaxAllowedRuntimeTickUs;
    Serial.print("LOGGER RUNTIME MAX_US ");
    Serial.print(maxRuntimeTickUs);
    Serial.print(" RESULT ");
    Serial.println(timingOk ? "OK" : "FAIL");

    Serial.print("MIRROR LOGGER RESULT ");
    Serial.print(programOk && sdOk ? "OK" : "FAIL");
    Serial.print(" program_started=");
    Serial.print(programStarted ? "true" : "false");
    Serial.print(" sd_started=");
    Serial.println(sdStarted ? "true" : "false");

    return programOk && sdOk && timingOk;
}
} // namespace

void setup()
{
    Serial.begin(NuraConstants::App::kSerialBaudRate);
    const uint32_t startMs = millis();
    while (!Serial && (millis() - startMs) < kSerialWaitMs)
    {
        delay(10);
    }

    Serial.println();
    Serial.println("NURA STORAGE LOGGER READY");
    Serial.println("SEND D TO START");
    while (true)
    {
        if (Serial.available() > 0)
        {
            const char command = static_cast<char>(Serial.read());
            if (command == 'D' || command == 'd')
            {
                break;
            }
        }
        delay(10);
    }

    Serial.println("NURA STORAGE LOGGER END-TO-END TEST");
    const bool ok = runMirrorLoggerTest();
    Serial.print("NURA STORAGE LOGGER END-TO-END ");
    Serial.println(ok ? "PASS" : "FAIL");
}

void loop()
{
    delay(1000);
}
