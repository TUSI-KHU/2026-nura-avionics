#include "ublox_m6_gnss_hal.h"

bool UbloxM6GNSSHAL::begin(HardwareSerial &serial,
                           uint32_t baudRate,
                           uint32_t maxFixAgeMs)
{
    serial.begin(baudRate);
    return attach(serial, maxFixAgeMs);
}

bool UbloxM6GNSSHAL::attach(Stream &stream, uint32_t maxFixAgeMs)
{
    stream_ = &stream;
    maxFixAgeMs_ = maxFixAgeMs;
    return true;
}

bool UbloxM6GNSSHAL::poll(UbloxM6GnssReading &out, uint32_t nowMs, uint16_t maxBytes)
{
    if (stream_ == nullptr)
    {
        return false;
    }

    uint16_t consumed = 0U;
    bool parsedSentence = false;
    while (stream_->available() > 0 && consumed < maxBytes)
    {
        const int c = stream_->read();
        if (c < 0)
        {
            break;
        }

        parsedSentence = gps_.encode(static_cast<char>(c)) || parsedSentence;
        ++consumed;
    }

    fillReading(out, nowMs);
    return parsedSentence || consumed > 0U;
}

void UbloxM6GNSSHAL::fillReading(UbloxM6GnssReading &out, uint32_t nowMs)
{
    const bool locationValid = gps_.location.isValid();
    const uint32_t locationAge = gps_.location.age();

    out.hasFix = locationValid && locationAge <= maxFixAgeMs_;
    out.latitudeDeg = out.hasFix ? gps_.location.lat() : 0.0;
    out.longitudeDeg = out.hasFix ? gps_.location.lng() : 0.0;
    out.altitudeM = gps_.altitude.isValid() ? gps_.altitude.meters() : 0.0;
    out.speedMps = gps_.speed.isValid() ? gps_.speed.mps() : 0.0;
    out.courseDeg = gps_.course.isValid() ? gps_.course.deg() : 0.0;
    out.hdop = gps_.hdop.isValid() ? gps_.hdop.hdop() : 0.0;
    out.satellites = gps_.satellites.isValid() ? gps_.satellites.value() : 0UL;
    out.locationAgeMs = locationAge;
    out.charsProcessed = gps_.charsProcessed();
    out.passedChecksum = gps_.passedChecksum();
    out.failedChecksum = gps_.failedChecksum();
    out.sampleMs = nowMs;
}
