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
    for (uint8_t attempt = 0U; attempt < NuraConstants::Sensors::kSensorInitRetryAttempts; ++attempt)
    {
        if (gnss_.begin(serial_, baudRate_, config_.gnssMaxFixAgeMs()))
        {
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
