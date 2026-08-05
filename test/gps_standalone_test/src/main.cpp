#include <Arduino.h>
#include <TinyGPS++.h>
#include <Wire.h>

#include "board_pinmap.h"
#include "nura_constants.h"

namespace
{
constexpr uint32_t kConsoleBaud = 115200UL;
constexpr uint32_t kStatusPeriodMs = 1000UL;
constexpr uint32_t kLinkDecisionMs = 5000UL;

TinyGPSPlus gps;
uint8_t gpsRxBuffer[NuraConstants::Sensors::kGnssSerialRxBufferBytes] = {0U};
uint32_t startedMs = 0UL;
uint32_t lastStatusMs = 0UL;
uint32_t lastCharsProcessed = 0UL;
bool linkResultPrinted = false;
bool fixResultPrinted = false;

HardwareSerialIMXRT &gpsSerial()
{
    return BoardPinMap::UbloxM6::serial();
}

bool hasFreshFix()
{
    return gps.location.isValid() &&
           gps.location.age() <= NuraConstants::Sensors::kGnssMaxFixAgeMs;
}

void printStatus(uint32_t nowMs)
{
    if ((nowMs - lastStatusMs) < kStatusPeriodMs)
    {
        return;
    }

    const uint32_t chars = gps.charsProcessed();
    const uint32_t passed = gps.passedChecksum();
    const uint32_t failed = gps.failedChecksum();
    const bool receivedSinceLastStatus = chars > lastCharsProcessed;
    const bool fix = hasFreshFix();

    const char *status = "UART_SILENT";
    if (chars > 0UL && !receivedSinceLastStatus)
    {
        status = "UART_STALLED";
    }
    else if (chars > 0UL && passed == 0UL)
    {
        status = "BYTES_NO_VALID_NMEA";
    }
    else if (fix)
    {
        status = "FIX";
    }
    else if (passed > 0UL)
    {
        status = "NMEA_OK_NO_FIX";
    }

    Serial.print("[");
    Serial.print(nowMs);
    Serial.print("] GPS status=");
    Serial.print(status);
    Serial.print(" chars=");
    Serial.print(chars);
    Serial.print(" nmea_ok=");
    Serial.print(passed);
    Serial.print(" nmea_bad=");
    Serial.print(failed);
    Serial.print(" fix=");
    Serial.print(fix ? "true" : "false");
    Serial.print(" sats=");
    Serial.print(gps.satellites.isValid() ? gps.satellites.value() : 0UL);

    if (fix)
    {
        Serial.print(" age_ms=");
        Serial.print(gps.location.age());
        Serial.print(" lat=");
        Serial.print(gps.location.lat(), 7);
        Serial.print(" lon=");
        Serial.print(gps.location.lng(), 7);
        Serial.print(" alt_m=");
        Serial.print(gps.altitude.isValid() ? gps.altitude.meters() : 0.0, 1);
        Serial.print(" speed_mps=");
        Serial.print(gps.speed.isValid() ? gps.speed.mps() : 0.0, 2);
        Serial.print(" hdop=");
        Serial.print(gps.hdop.isValid() ? gps.hdop.hdop() : 0.0, 2);
    }

    Serial.println();
    lastStatusMs = nowMs;
    lastCharsProcessed = chars;

    if (!linkResultPrinted && passed > 0UL)
    {
        Serial.println("RESULT GPS_NMEA_LINK PASS");
        linkResultPrinted = true;
    }
    else if (!linkResultPrinted && (nowMs - startedMs) >= kLinkDecisionMs)
    {
        Serial.print("RESULT GPS_NMEA_LINK FAIL reason=");
        Serial.println(chars == 0UL ? "UART_SILENT" : "NO_VALID_NMEA");
        linkResultPrinted = true;
    }

    if (!fixResultPrinted && fix)
    {
        Serial.println("RESULT GPS_FIX PASS");
        fixResultPrinted = true;
    }
}
} // namespace

void setup()
{
    Serial.begin(kConsoleBaud);
    const uint32_t waitStartedMs = millis();
    while (!Serial && (millis() - waitStartedMs) < 2000UL)
    {
    }

    HardwareSerialIMXRT &serial = gpsSerial();
    serial.setRX(BoardPinMap::UbloxM6::rxPin);
    serial.setTX(BoardPinMap::UbloxM6::txPin);
    serial.addMemoryForRead(gpsRxBuffer, sizeof(gpsRxBuffer));
    serial.begin(BoardPinMap::UbloxM6::baud);

    startedMs = millis();
    lastStatusMs = startedMs;

    Serial.println("GPS_STANDALONE_TEST_BEGIN");
    Serial.print("config uart=Serial3 rx=");
    Serial.print(BoardPinMap::UbloxM6::rxPin);
    Serial.print(" tx=");
    Serial.print(BoardPinMap::UbloxM6::txPin);
    Serial.print(" baud=");
    Serial.print(BoardPinMap::UbloxM6::baud);
    Serial.print(" rx_buffer=");
    Serial.println(sizeof(gpsRxBuffer));
    Serial.println("GPS_NMEA_LINK passes without a satellite fix; GPS_FIX requires sky view.");
}

void loop()
{
    HardwareSerialIMXRT &serial = gpsSerial();
    while (serial.available() > 0)
    {
        const int value = serial.read();
        if (value >= 0)
        {
            gps.encode(static_cast<char>(value));
        }
    }

    printStatus(millis());
}
