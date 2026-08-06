#include <Arduino.h>
#include <TinyGPS++.h>
#include <stdlib.h>
#include <string.h>

#include "board_pinmap.h"
#include "nura_constants.h"

namespace
{
constexpr uint32_t kCaptureMs = 120000UL;
constexpr uint32_t kStatusPeriodMs = 10000UL;
constexpr size_t kLineBytes = 128U;

TinyGPSPlus gps;
TinyGPSCustom gnGsaFix(gps, "GNGSA", 2);
TinyGPSCustom gpGsaFix(gps, "GPGSA", 2);
TinyGPSCustom gnGsvView(gps, "GNGSV", 3);
TinyGPSCustom gpGsvView(gps, "GPGSV", 3);

char lineBuffer[kLineBytes] = {};
size_t lineLength = 0U;
uint32_t bytesReceived = 0UL;
uint32_t ggaCount = 0UL;
uint32_t rmcCount = 0UL;
uint32_t gsaCount = 0UL;
uint32_t gsvCount = 0UL;
uint32_t lastStatusMs = 0UL;

void countSentence()
{
    if (lineLength < 6U || lineBuffer[0] != '$')
    {
        return;
    }

    const char *type = lineBuffer + 3U;
    if (strncmp(type, "GGA", 3U) == 0)
    {
        ++ggaCount;
    }
    else if (strncmp(type, "RMC", 3U) == 0)
    {
        ++rmcCount;
    }
    else if (strncmp(type, "GSA", 3U) == 0)
    {
        ++gsaCount;
    }
    else if (strncmp(type, "GSV", 3U) == 0)
    {
        ++gsvCount;
    }
}

uint32_t customValue(TinyGPSCustom &gnValue, TinyGPSCustom &gpValue)
{
    if (gnValue.isValid())
    {
        return strtoul(gnValue.value(), nullptr, 10);
    }
    if (gpValue.isValid())
    {
        return strtoul(gpValue.value(), nullptr, 10);
    }
    return 0UL;
}

void printStatus(uint32_t nowMs, bool final)
{
    Serial.print(final ? "GPS_FINAL" : "GPS_STATUS");
    Serial.print(" elapsed_ms=");
    Serial.print(nowMs);
    Serial.print(" bytes=");
    Serial.print(bytesReceived);
    Serial.print(" checksum_ok=");
    Serial.print(gps.passedChecksum());
    Serial.print(" checksum_fail=");
    Serial.print(gps.failedChecksum());
    Serial.print(" gga=");
    Serial.print(ggaCount);
    Serial.print(" rmc=");
    Serial.print(rmcCount);
    Serial.print(" gsa=");
    Serial.print(gsaCount);
    Serial.print(" gsv=");
    Serial.print(gsvCount);
    Serial.print(" fix_type=");
    Serial.print(customValue(gnGsaFix, gpGsaFix));
    Serial.print(" sats_used=");
    Serial.print(gps.satellites.isValid() ? gps.satellites.value() : 0UL);
    Serial.print(" sats_view=");
    Serial.print(customValue(gnGsvView, gpGsvView));
    Serial.print(" hdop=");
    Serial.print(gps.hdop.isValid() ? gps.hdop.hdop() : 99.99, 2);
    Serial.print(" location_valid=");
    Serial.println(gps.location.isValid() ? 1 : 0);
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

    auto &gpsSerial = BoardPinMap::UbloxM6::serial();
    gpsSerial.setRX(BoardPinMap::UbloxM6::rxPin);
    gpsSerial.setTX(BoardPinMap::UbloxM6::txPin);
    gpsSerial.begin(BoardPinMap::UbloxM6::baud);
    Serial.println("NURA OUTDOOR GPS FIX MONITOR BEGIN");
}

void loop()
{
    const uint32_t nowMs = millis();
    auto &gpsSerial = BoardPinMap::UbloxM6::serial();
    while (gpsSerial.available() > 0)
    {
        const int value = gpsSerial.read();
        if (value < 0)
        {
            continue;
        }

        const char character = static_cast<char>(value);
        gps.encode(character);
        ++bytesReceived;
        if (character == '\n')
        {
            lineBuffer[lineLength] = '\0';
            countSentence();
            lineLength = 0U;
        }
        else if (character != '\r' && lineLength < (kLineBytes - 1U))
        {
            lineBuffer[lineLength++] = character;
        }
    }

    if ((nowMs - lastStatusMs) >= kStatusPeriodMs && nowMs < kCaptureMs)
    {
        lastStatusMs = nowMs;
        printStatus(nowMs, false);
    }
    if (nowMs >= kCaptureMs)
    {
        printStatus(nowMs, true);
        Serial.println("NURA OUTDOOR GPS FIX MONITOR END");
        while (true)
        {
            yield();
        }
    }
    yield();
}
