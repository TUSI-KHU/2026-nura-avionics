#include "nura/apps/sensor_apps.h"

namespace nura::apps
{
namespace c = nura::contracts;
namespace core = nura::core;
namespace p = nura::platform;

namespace
{
core::AppRunResult portResult(p::PortResult result)
{
    switch (result)
    {
    case p::PortResult::OK: return core::AppRunResult::OK;
    case p::PortResult::NO_DATA: return core::AppRunResult::NO_INPUT;
    case p::PortResult::UNAVAILABLE: return core::AppRunResult::DEGRADED;
    case p::PortResult::FAULT:
    default: return core::AppRunResult::FAILED;
    }
}

template <typename Port>
core::AppRunResult ensureInitialized(Port &port, bool &initialized)
{
    if (initialized)
    {
        return core::AppRunResult::OK;
    }
    const p::PortResult result = port.initialize();
    if (result == p::PortResult::OK)
    {
        initialized = true;
    }
    return portResult(result);
}

template <typename Sample>
void stamp(Sample &sample, c::AppId producer, uint32_t sequence,
           uint32_t now_ms, uint64_t now_us)
{
    sample.header.schema_version = c::kContractSchemaVersion;
    sample.header.sequence = sequence;
    if (sample.header.sample_time_ms == 0U)
    {
        sample.header.sample_time_ms = now_ms;
    }
    sample.header.publish_time_us = now_us;
    sample.header.producer = producer;
}
} // namespace

c::AppId LowGSensorApp::id() const { return c::AppId::LOW_G_SENSOR; }
core::AppRunResult LowGSensorApp::run(uint32_t now_ms, uint32_t cycle_id)
{
    const core::AppRunResult init = ensureInitialized(port_, initialized_);
    if (init != core::AppRunResult::OK) return init;
    c::LowGImuSample sample{};
    const p::PortResult result = port_.read(now_ms, sample);
    if (result != p::PortResult::OK) return portResult(result);
    stamp(sample, id(), ++sequence_, now_ms, clock_.nowUs());
    return bus_.publishLowG(sample, cycle_id) ? core::AppRunResult::OK
                                              : core::AppRunResult::DEGRADED;
}

c::AppId HighGSensorApp::id() const { return c::AppId::HIGH_G_SENSOR; }
core::AppRunResult HighGSensorApp::run(uint32_t now_ms, uint32_t cycle_id)
{
    const core::AppRunResult init = ensureInitialized(port_, initialized_);
    if (init != core::AppRunResult::OK) return init;
    c::HighGImuSample sample{};
    const p::PortResult result = port_.read(now_ms, sample);
    if (result != p::PortResult::OK) return portResult(result);
    stamp(sample, id(), ++sequence_, now_ms, clock_.nowUs());
    return bus_.publishHighG(sample, cycle_id) ? core::AppRunResult::OK
                                               : core::AppRunResult::DEGRADED;
}

c::AppId BarometerSensorApp::id() const { return c::AppId::BAROMETER_SENSOR; }
core::AppRunResult BarometerSensorApp::run(uint32_t now_ms, uint32_t cycle_id)
{
    const core::AppRunResult init = ensureInitialized(port_, initialized_);
    if (init != core::AppRunResult::OK) return init;
    c::BarometerSample sample{};
    const p::PortResult result = port_.read(now_ms, sample);
    if (result != p::PortResult::OK) return portResult(result);
    stamp(sample, id(), ++sequence_, now_ms, clock_.nowUs());
    return bus_.publishBarometer(sample, cycle_id) ? core::AppRunResult::OK
                                                   : core::AppRunResult::DEGRADED;
}

c::AppId MagnetometerSensorApp::id() const
{
    return c::AppId::MAGNETOMETER_SENSOR;
}
core::AppRunResult MagnetometerSensorApp::run(uint32_t now_ms, uint32_t cycle_id)
{
    const core::AppRunResult init = ensureInitialized(port_, initialized_);
    if (init != core::AppRunResult::OK) return init;
    c::MagnetometerSample sample{};
    const p::PortResult result = port_.read(now_ms, sample);
    if (result != p::PortResult::OK) return portResult(result);
    stamp(sample, id(), ++sequence_, now_ms, clock_.nowUs());
    return bus_.publishMagnetometer(sample, cycle_id) ? core::AppRunResult::OK
                                                      : core::AppRunResult::DEGRADED;
}

c::AppId GnssSensorApp::id() const { return c::AppId::GNSS_SENSOR; }
core::AppRunResult GnssSensorApp::run(uint32_t now_ms, uint32_t cycle_id)
{
    const core::AppRunResult init = ensureInitialized(port_, initialized_);
    if (init != core::AppRunResult::OK) return init;
    c::GnssSample sample{};
    const p::PortResult result = port_.read(now_ms, sample);
    if (result != p::PortResult::OK) return portResult(result);
    stamp(sample, id(), ++sequence_, now_ms, clock_.nowUs());
    return bus_.publishGnss(sample, cycle_id) ? core::AppRunResult::OK
                                              : core::AppRunResult::DEGRADED;
}

c::AppId PowerSensorApp::id() const { return c::AppId::POWER_SENSOR; }
core::AppRunResult PowerSensorApp::run(uint32_t now_ms, uint32_t cycle_id)
{
    const core::AppRunResult init = ensureInitialized(port_, initialized_);
    if (init != core::AppRunResult::OK) return init;
    c::PowerSample sample{};
    const p::PortResult result = port_.read(now_ms, sample);
    if (result != p::PortResult::OK) return portResult(result);
    stamp(sample, id(), ++sequence_, now_ms, clock_.nowUs());
    return bus_.publishPower(sample, cycle_id) ? core::AppRunResult::OK
                                               : core::AppRunResult::DEGRADED;
}

c::AppId SafetyInputApp::id() const { return c::AppId::SAFETY_INPUT; }
core::AppRunResult SafetyInputApp::run(uint32_t now_ms, uint32_t cycle_id)
{
    const core::AppRunResult init = ensureInitialized(port_, initialized_);
    if (init != core::AppRunResult::OK) return init;
    c::SafetyStatus status{};
    const p::PortResult result = port_.read(now_ms, status);
    if (result != p::PortResult::OK) return portResult(result);
    stamp(status, id(), ++sequence_, now_ms, clock_.nowUs());
    return bus_.publishSafetyStatus(status, cycle_id) ? core::AppRunResult::OK
                                                      : core::AppRunResult::DEGRADED;
}

} // namespace nura::apps
