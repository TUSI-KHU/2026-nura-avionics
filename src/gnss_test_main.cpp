#include <Arduino.h>

#include "hal/ublox_m6_gnss_hal.h"
#include "sensors/gnss_task.h"
#include "state/gps_state.h"

namespace
{
    UbloxM6GNSSHAL gnssHal;
    GpsState gpsState;
    GNSSTask gnssTask{gnssHal, gpsState};

    constexpr uint32_t kPrintPeriodMs = 1000U;
    uint32_t lastPrintMs = 0;

    void printGpsState(uint32_t nowMs)
    {
        const GpsData &gps = gpsState.data;

        Serial.print("{\"src\":\"gnss_state\",\"t_ms\":");
        Serial.print(nowMs);
        Serial.print(",\"has_fix\":");
        Serial.print(gps.hasFix ? "true" : "false");
        Serial.print(",\"lat_deg\":");
        if (gps.hasFix)
        {
            Serial.print(gps.latitudeDeg, 6);
        }
        else
        {
            Serial.print("null");
        }
        Serial.print(",\"lon_deg\":");
        if (gps.hasFix)
        {
            Serial.print(gps.longitudeDeg, 6);
        }
        else
        {
            Serial.print("null");
        }
        Serial.print(",\"alt_m\":");
        Serial.print(gps.altitudeM, 2);
        Serial.print(",\"speed_mps\":");
        Serial.print(gps.speedMps, 2);
        Serial.print(",\"course_deg\":");
        Serial.print(gps.courseDeg, 2);
        Serial.print(",\"sats\":");
        Serial.print(gps.satellites);
        Serial.print(",\"hdop\":");
        Serial.print(gps.hdop, 2);
        Serial.print(",\"location_age_ms\":");
        Serial.print(gps.locationAgeMs);
        Serial.print(",\"chars\":");
        Serial.print(gps.charsProcessed);
        Serial.print(",\"pass_checksum\":");
        Serial.print(gps.passedChecksum);
        Serial.print(",\"fail_checksum\":");
        Serial.print(gps.failedChecksum);
        Serial.print(",\"last_update_ms\":");
        Serial.print(gps.lastUpdatedMs);
        Serial.println("}");
    }
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 4000U)
    {
    }

    Serial.println();
    Serial.println("# GNSS HAL -> Task -> State test");
    Serial.println("# GPS TX should be wired to Teensy pin 15 RX3.");
    Serial.println("# PASS means pass_checksum increases; GPS fix may need sky view.");

    if (gnssTask.init())
    {
        Serial.println("# init: ok");
    }
    else
    {
        Serial.println("# init: failed");
    }
}

void loop()
{
    const uint32_t nowMs = millis();
    gnssTask.tick(nowMs);

    if (nowMs - lastPrintMs >= kPrintPeriodMs)
    {
        lastPrintMs = nowMs;
        printGpsState(nowMs);
    }
}
