#include "panic_handler.h"

#include <Arduino.h>

#include "hal/digital_output_hal.h"

BlinkingPanicHandler::BlinkingPanicHandler(const IAppConfig &config)
    : config_(config) {}

void BlinkingPanicHandler::panic()
{
    DigitalOutputHAL statusIndicator(config_.statusIndicatorPin());

    // 치명적 오류 시 LED를 점멸하며 무한 대기한다.
    while (true)
    {
        statusIndicator.write(true);
        delay(config_.faultBlinkIntervalMs());
        statusIndicator.write(false);
        delay(config_.faultBlinkIntervalMs());
    }
}
