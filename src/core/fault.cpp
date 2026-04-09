#include "fault.h"

#include <Arduino.h>

#include "hal/digital_output_hal.h"

[[noreturn]] void hang()
{
    static DigitalOutputHAL statusIndicator;
    while (true)
    {
        statusIndicator.write(true);
        delay(1000);
        statusIndicator.write(false);
        delay(1000);
    }
}
