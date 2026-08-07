#pragma once

#include "Arduino.h"

namespace BoardPinMap
{
struct SparkFunSx1276_1W final
{
    static constexpr int ssPin = 9;
    static constexpr int resetPin = 24;
    static constexpr int libraryResetPin = -1;
    static constexpr int dio0Pin = 32;
    static constexpr int rxEnablePin = 30;
    static constexpr int txEnablePin = 31;
};
} // namespace BoardPinMap
