#include "nura/apps/input_aggregator_app.h"

namespace nura::apps
{
namespace c = nura::contracts;
namespace core = nura::core;

c::AppId InputAggregatorApp::id() const { return c::AppId::INPUT_AGGREGATOR; }

core::AppRunResult InputAggregatorApp::run(uint32_t now_ms, uint32_t cycle_id)
{
    const uint64_t now_us = clock_.nowUs();
    bool updated = false;
    updated = bus_.readLowG(low_g_, id(), cycle_id, now_us) || updated;
    updated = bus_.readHighG(high_g_, id(), cycle_id, now_us) || updated;
    updated = bus_.readBarometer(barometer_, id(), cycle_id, now_us) || updated;
    updated = bus_.readSensorHealth(health_, id(), cycle_id, now_us) || updated;
    updated = bus_.readSafetyStatus(safety_, id(), cycle_id, now_us) || updated;
    have_input_ = have_input_ || updated;
    if (!have_input_)
    {
        return core::AppRunResult::NO_INPUT;
    }

    c::FlightInputs inputs{};
    inputs.header.schema_version = c::kContractSchemaVersion;
    inputs.header.sequence = ++sequence_;
    inputs.header.sample_time_ms = now_ms;
    inputs.header.publish_time_us = clock_.nowUs();
    inputs.header.status_flags = c::SAMPLE_STATUS_VALID;
    inputs.header.producer = id();
    inputs.low_g = low_g_;
    inputs.high_g = high_g_;
    inputs.barometer = barometer_;
    inputs.health = health_;
    inputs.abort_active = safety_.abort_active;
    return bus_.publishFlightInputs(inputs, cycle_id) ? core::AppRunResult::OK
                                                      : core::AppRunResult::DEGRADED;
}

} // namespace nura::apps
