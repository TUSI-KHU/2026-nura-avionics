#include "gnss_task.h"

#include "board_pinmap.h"
#include "nura_constants.h"

GNSSTask::GNSSTask(IGnss &gnss,
                   GpsState &gpsState,
                   const IAppConfig &config,
                   HardwareSerial &serial,
                   uint32_t baudRate)
    : gnss_(gnss),
      gpsState_(gpsState),
      config_(config),
      serial_(serial),
      baudRate_(baudRate) {}

const char *GNSSTask::name() const
{
    return "gnss";
}

bool GNSSTask::init()
{
    gpsState_.data = GpsData{};
    lastStatusLogMs_ = 0U;
    lastLoggedCharsProcessed_ = 0U;
    for (uint8_t attempt = 0U; attempt < NuraConstants::Sensors::kSensorInitRetryAttempts; ++attempt)
    {
        if (gnss_.begin(serial_, baudRate_, config_.gnssMaxFixAgeMs()))
        {
            if (Serial)
            {
                Serial.print("[0] gps uart initialized baud=");
                Serial.print(baudRate_);
                Serial.print(" rx=");
                Serial.print(BoardPinMap::UbloxM6::rxPin);
                Serial.print(" tx=");
                Serial.println(BoardPinMap::UbloxM6::txPin);
            }
            return true;
        }
        if ((attempt + 1U) < NuraConstants::Sensors::kSensorInitRetryAttempts)
        {
            delay(NuraConstants::Sensors::kSensorInitRetryDelayMs);
        }
    }
    return false;
}

bool GNSSTask::tick(uint32_t nowMs)
{
    UbloxM6GnssReading sample;
    const bool readOk = gnss_.poll(sample, nowMs, config_.gnssPollByteBudget());

    if (readOk)
    {
        updateState(sample);
    }

    logStatus(sample, nowMs);

    return true;
}

uint32_t GNSSTask::periodMs() const
{
    return config_.gnssTaskPeriodMs();
}

void GNSSTask::updateState(const UbloxM6GnssReading &sample)
{
    gpsState_.data.hasFix = sample.hasFix;
    gpsState_.data.latitudeDeg = sample.latitudeDeg;
    gpsState_.data.longitudeDeg = sample.longitudeDeg;
    gpsState_.data.altitudeM = sample.altitudeM;
    gpsState_.data.speedMps = sample.speedMps;
    gpsState_.data.courseDeg = sample.courseDeg;
    gpsState_.data.hdop = sample.hdop;
    gpsState_.data.satellites = sample.satellites;
    gpsState_.data.locationAgeMs = sample.locationAgeMs;
    gpsState_.data.charsProcessed = sample.charsProcessed;
    gpsState_.data.passedChecksum = sample.passedChecksum;
    gpsState_.data.failedChecksum = sample.failedChecksum;
    gpsState_.data.lastUpdatedMs = sample.sampleMs;
}

void GNSSTask::logStatus(const UbloxM6GnssReading &sample, uint32_t nowMs)
{
    if ((nowMs - lastStatusLogMs_) < NuraConstants::Diagnostics::kGnssPrintPeriodMs)
    {
        return;
    }

    const bool receivedSinceLastLog = sample.charsProcessed > lastLoggedCharsProcessed_;
    lastStatusLogMs_ = nowMs;
    lastLoggedCharsProcessed_ = sample.charsProcessed;

    if (!Serial)
    {
        return;
    }

    const char *status = "UART_SILENT";
    if (sample.charsProcessed > 0U && !receivedSinceLastLog)
    {
        status = "UART_STALLED";
    }
    else if (sample.charsProcessed > 0U && sample.passedChecksum == 0U)
    {
        status = "BYTES_NO_VALID_NMEA";
    }
    else if (sample.hasFix)
    {
        status = "FIX";
    }
    else if (sample.passedChecksum > 0U)
    {
        status = "NMEA_OK_NO_FIX";
    }

    Serial.print("[");
    Serial.print(nowMs);
    Serial.print("] gps status=");
    Serial.print(status);
    Serial.print(" chars=");
    Serial.print(sample.charsProcessed);
    Serial.print(" nmea_ok=");
    Serial.print(sample.passedChecksum);
    Serial.print(" nmea_bad=");
    Serial.print(sample.failedChecksum);
    Serial.print(" fix=");
    Serial.print(sample.hasFix ? "true" : "false");
    Serial.print(" sats=");
    Serial.print(sample.satellites);

    if (sample.hasFix)
    {
        Serial.print(" age_ms=");
        Serial.print(sample.locationAgeMs);
        Serial.print(" lat=");
        Serial.print(sample.latitudeDeg, 7);
        Serial.print(" lon=");
        Serial.print(sample.longitudeDeg, 7);
        Serial.print(" alt_m=");
        Serial.print(sample.altitudeM, 1);
    }

    Serial.println();
}
