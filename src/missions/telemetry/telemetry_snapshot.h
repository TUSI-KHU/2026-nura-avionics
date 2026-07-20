#pragma once

#include "state/barometer_state.h"
#include "state/power_state.h"
#include "state/system_health_state.h"

using BarometerTelemetryData = BarometerState;
using PowerTelemetryData = PowerState;

// Read/write view shared by telemetry, flight logging, and flight mission
// orchestration. Physical sensor states remain independent POD types.
struct TelemetrySnapshot
{
    BarometerState &barometer;
    PowerState &power;
    SystemHealthState &health;
};
