#pragma once

#include <stdint.h>

#include <Arduino.h>
#include <TinyGPS++.h>

struct UbloxM6GnssReading
{
    bool hasFix = false;
    double latitudeDeg = 0.0;
    double longitudeDeg = 0.0;
    double altitudeM = 0.0;
    double speedMps = 0.0;
    double courseDeg = 0.0;
    double hdop = 0.0;
    uint32_t satellites = 0;
    uint32_t locationAgeMs = 0;
    uint32_t charsProcessed = 0;
    uint32_t passedChecksum = 0;
    uint32_t failedChecksum = 0;
    uint32_t sampleMs = 0;
};

class UbloxM6GNSSHAL
{
public:
    bool begin(HardwareSerial &serial,
               uint32_t baudRate = 9600UL,
               uint32_t maxFixAgeMs = 2000UL);
    bool attach(Stream &stream, uint32_t maxFixAgeMs = 2000UL);
    bool poll(UbloxM6GnssReading &out, uint32_t nowMs, uint16_t maxBytes = 128U);

private:
    void fillReading(UbloxM6GnssReading &out, uint32_t nowMs);

    Stream *stream_ = nullptr;
    TinyGPSPlus gps_;
    uint32_t maxFixAgeMs_ = 2000UL;
};
