#include <Arduino.h>
#include "board_pinmap.h"
#include <TinyGPS++.h>

// ==================== PIN MAP / USER CONFIG ====================
#define SERIAL_BAUD 115200
#define GNSS_SERIAL BoardPinMap::UbloxM6::serial()
#define GNSS_BAUD BoardPinMap::UbloxM6::baud
#define GNSS_RX_PIN BoardPinMap::UbloxM6::rxPin
#define GNSS_TX_PIN BoardPinMap::UbloxM6::txPin
#define GNSS_TEST_WINDOW_MS 60000UL
// ================================================================

TinyGPSPlus gps;
uint32_t lastPrintMs = 0;

static void printStats()
{
    Serial.print("chars=");
    Serial.print(gps.charsProcessed());
    Serial.print(" pass_checksum=");
    Serial.print(gps.passedChecksum());
    Serial.print(" fail_checksum=");
    Serial.print(gps.failedChecksum());
    Serial.print(" sats=");
    if (gps.satellites.isValid())
    {
        Serial.print(gps.satellites.value());
    }
    else
    {
        Serial.print("NA");
    }
    Serial.print(" hdop=");
    if (gps.hdop.isValid())
    {
        Serial.print(gps.hdop.hdop(), 2);
    }
    else
    {
        Serial.print("NA");
    }
    Serial.print(" fix=");
    Serial.print(gps.location.isValid() ? "yes" : "no");
    Serial.print(" lat=");
    if (gps.location.isValid())
    {
        Serial.print(gps.location.lat(), 6);
    }
    else
    {
        Serial.print("NA");
    }
    Serial.print(" lon=");
    if (gps.location.isValid())
    {
        Serial.print(gps.location.lng(), 6);
    }
    else
    {
        Serial.print("NA");
    }
    Serial.println();
}

static bool runDefectTest()
{
    const uint32_t startMs = millis();
    uint32_t lastPassed = 0;

    while ((millis() - startMs) < GNSS_TEST_WINDOW_MS)
    {
        while (GNSS_SERIAL.available() > 0)
        {
            gps.encode(static_cast<char>(GNSS_SERIAL.read()));
        }

        if ((millis() - lastPrintMs) >= 1000UL)
        {
            lastPrintMs = millis();
            printStats();
        }

        if (gps.passedChecksum() > lastPassed)
        {
            lastPassed = gps.passedChecksum();
            Serial.println("PASS: valid NMEA sentence received");
            return true;
        }
    }

    if (gps.charsProcessed() == 0UL)
    {
        Serial.println("FAIL: no bytes received from GNSS UART");
        return false;
    }

    Serial.println("FAIL: GNSS bytes received but no valid NMEA checksum passed");
    return false;
}

void setup()
{
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 4000)
    {
    }

    Serial.println();
    Serial.println("u-blox M6 / NEO-6M GNSS defect test");
    Serial.println("A GPS fix is not required for electrical PASS; at least one valid NMEA sentence is required.");

    GNSS_SERIAL.setRX(GNSS_RX_PIN);
    GNSS_SERIAL.setTX(GNSS_TX_PIN);
    GNSS_SERIAL.begin(GNSS_BAUD);

    runDefectTest();
}

void loop()
{
    while (GNSS_SERIAL.available() > 0)
    {
        gps.encode(static_cast<char>(GNSS_SERIAL.read()));
    }

    if ((millis() - lastPrintMs) >= 1000UL)
    {
        lastPrintMs = millis();
        printStats();
    }
}
