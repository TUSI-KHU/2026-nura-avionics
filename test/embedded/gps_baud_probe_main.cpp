#include <Arduino.h>
#include <TinyGPS++.h>

#include "board_pinmap.h"
#include "nura_constants.h"

namespace
{
constexpr uint32_t kProbeWindowMs = 6000UL;
constexpr uint32_t kBaudRates[] = {4800UL, 9600UL, 38400UL, 57600UL, 115200UL};
constexpr size_t kPreviewBytes = 120U;

void probeBaud(uint32_t baudRate)
{
    auto &gpsSerial = BoardPinMap::UbloxM6::serial();
    gpsSerial.end();
    delay(100);
    gpsSerial.begin(baudRate);
    delay(250);
    while (gpsSerial.available() > 0)
    {
        (void)gpsSerial.read();
    }

    TinyGPSPlus parser;
    uint32_t bytes = 0UL;
    uint32_t printable = 0UL;
    uint32_t dollars = 0UL;
    uint32_t newlines = 0UL;
    char preview[kPreviewBytes + 1U] = {};
    size_t previewLength = 0U;
    const uint32_t startMs = millis();

    while ((millis() - startMs) < kProbeWindowMs)
    {
        while (gpsSerial.available() > 0)
        {
            const int value = gpsSerial.read();
            if (value < 0)
            {
                continue;
            }

            const char character = static_cast<char>(value);
            parser.encode(character);
            ++bytes;
            if (character >= 32 && character <= 126)
            {
                ++printable;
            }
            if (character == '$')
            {
                ++dollars;
            }
            if (character == '\n')
            {
                ++newlines;
            }
            if (previewLength < kPreviewBytes)
            {
                preview[previewLength++] = character >= 32 && character <= 126 ? character : '.';
            }
        }
        yield();
    }

    Serial.print("GPS_PROBE baud=");
    Serial.print(baudRate);
    Serial.print(" bytes=");
    Serial.print(bytes);
    Serial.print(" printable_pct=");
    Serial.print(bytes > 0UL ? (printable * 100UL) / bytes : 0UL);
    Serial.print(" dollars=");
    Serial.print(dollars);
    Serial.print(" newlines=");
    Serial.print(newlines);
    Serial.print(" checksum_ok=");
    Serial.print(parser.passedChecksum());
    Serial.print(" checksum_fail=");
    Serial.print(parser.failedChecksum());
    Serial.print(" chars=");
    Serial.println(parser.charsProcessed());
    Serial.print("PREVIEW ");
    Serial.println(preview);
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

    Serial.println("NURA GPS BAUD PROBE BEGIN");
    Serial.println("FSM=OFF PYRO=NOT_COMPILED LORA=NOT_COMPILED STORAGE=OFF");
    for (const uint32_t baudRate : kBaudRates)
    {
        probeBaud(baudRate);
    }
    Serial.println("NURA GPS BAUD PROBE END");
}

void loop()
{
    delay(1000);
}
