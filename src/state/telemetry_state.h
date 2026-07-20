#pragma once

#include "state/barometer_state.h"
#include "state/power_state.h"
#include "state/system_health_state.h"

#if defined(NURA_LEGACY_TEST_TELEMETRY_STATE)
#include "missions/flight/flight_trace.h"
#include "missions/telemetry/telemetry_snapshot.h"

// Test-only adapter for legacy replay fixtures. Production code uses the three
// independent POD states and TelemetrySnapshot directly.
struct TelemetryState : public TelemetrySnapshot
{
    BarometerState barometer;
    PowerState power;
    SystemHealthState health;
    FlightTraceBuffer flightTrace;

    TelemetryState()
        : TelemetrySnapshot{barometer, power, health}
    {
    }
};
#endif
