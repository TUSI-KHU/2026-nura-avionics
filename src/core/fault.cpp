#include "fault.h"

#include <Arduino.h>

#include "hal/digital_output_hal.h"

[[noreturn]] void hang()
{
    static DigitalOutputHAL statusIndicator;
    // 치명적 오류 시 LED를 점멸하며 무한 대기한다.
    while (true)
    {
        statusIndicator.write(true);
        delay(1000);
        statusIndicator.write(false);
        delay(1000);
    }
}
