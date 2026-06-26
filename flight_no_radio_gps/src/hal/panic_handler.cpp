#include "panic_handler.h"

#include <Arduino.h>
#include <string.h>

#include "board_pinmap.h"
#include "hal/digital_output_hal.h"
#include "nura_constants.h"

namespace
{
uint8_t panicBeepCount(const char *reason)
{
    if (reason == nullptr)
    {
        return NuraConstants::Panic::kDefaultFailureBeeps;
    }

    if (strcmp(reason, "telemetry") == 0)
    {
        return NuraConstants::Panic::kLoRaFailureBeeps;
    }

    if (strstr(reason, "flight_log") != nullptr ||
        strstr(reason, "flash") != nullptr ||
        strstr(reason, "sd") != nullptr ||
        strstr(reason, "storage") != nullptr)
    {
        return NuraConstants::Panic::kStorageFailureBeeps;
    }

    return NuraConstants::Panic::kDefaultFailureBeeps;
}
} // namespace

BlinkingPanicHandler::BlinkingPanicHandler(const IAppConfig &config)
    : config_(config) {}

void BlinkingPanicHandler::panic(const char *reason)
{
    DigitalOutputHAL statusIndicator(config_.statusIndicatorPin());
    const uint8_t beepCount = panicBeepCount(reason);

    pinMode(BoardPinMap::Buzzer::pin, OUTPUT);
    noTone(BoardPinMap::Buzzer::pin);
    digitalWrite(BoardPinMap::Buzzer::pin, LOW);

    if (Serial)
    {
        Serial.print("PANIC init_failure=");
        Serial.println(reason == nullptr ? "unknown" : reason);
    }

    // 치명적 오류 시 부저/LED 패턴을 반복하되 delay()로 USB 처리 시간을 남긴다.
    while (true)
    {
        if (Serial)
        {
            Serial.print("PANIC init_failure=");
            Serial.println(reason == nullptr ? "unknown" : reason);
        }

        for (uint8_t i = 0; i < beepCount; ++i)
        {
            statusIndicator.write(true);
            tone(BoardPinMap::Buzzer::pin, NuraConstants::Panic::kFailureToneFrequencyHz);
            delay(NuraConstants::Panic::kFailureBeepMs);
            noTone(BoardPinMap::Buzzer::pin);
            digitalWrite(BoardPinMap::Buzzer::pin, LOW);
            statusIndicator.write(false);
            delay(NuraConstants::Panic::kFailureGapMs);
        }

        while (Serial.available() > 0)
        {
            (void)Serial.read();
        }
        delay(NuraConstants::Panic::kFailureRepeatPauseMs);
    }
}
