#include "gnss_task.h"

#include "board_pinmap.h"

namespace
{
    constexpr uint32_t kGnssMaxFixAgeMs = 2000U;
    constexpr uint32_t kGnssTaskPeriodMs = 100U;
    constexpr uint16_t kGnssPollByteBudget = 128U;
}

GNSSTask::GNSSTask(UbloxM6GNSSHAL &gnss, GpsState &gpsState)
    : gnss_(gnss),
      gpsState_(gpsState) {}

const char *GNSSTask::name() const
{
    return "gnss";
}

bool GNSSTask::init()
{
    gpsState_.data = GpsData{};
    auto &serial = BoardPinMap::UbloxM6::serial();
    serial.setRX(BoardPinMap::UbloxM6::rxPin);
    serial.setTX(BoardPinMap::UbloxM6::txPin);
    return gnss_.begin(serial,
                       BoardPinMap::UbloxM6::baud,
                       kGnssMaxFixAgeMs);
}

bool GNSSTask::tick(uint32_t nowMs)
{
    UbloxM6GnssReading sample;
    const bool readOk = gnss_.poll(sample, nowMs, kGnssPollByteBudget);

    if (readOk)
    {
        updateState(sample);
    }

    return true;
}

uint32_t GNSSTask::periodMs() const
{
    return kGnssTaskPeriodMs;
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
