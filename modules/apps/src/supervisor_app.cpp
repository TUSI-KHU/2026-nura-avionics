#include "nura/apps/supervisor_app.h"

#include "nura/config/runtime_profile.h"

namespace nura::apps
{
namespace c = nura::contracts;
namespace core = nura::core;
namespace p = nura::platform;

c::AppId SupervisorApp::id() const { return c::AppId::SUPERVISOR; }

core::AppRunResult SupervisorApp::run(uint32_t now_ms, uint32_t cycle_id)
{
    const uint64_t now_us_64 = clock_.nowUs();
    const uint32_t now_us = static_cast<uint32_t>(now_us_64);
    const bool mission_healthy =
        appHealthy(c::AppId::FLIGHT_COORDINATOR, now_us,
                   nura::config::kRuntimeProfile.critical_health_max_age_us);
    const bool recovery_healthy =
        appHealthy(c::AppId::RECOVERY_ACTUATION, now_us,
                   nura::config::kRuntimeProfile.critical_health_max_age_us);

    c::SensorHealthSnapshot health{};
    health.header.schema_version = c::kContractSchemaVersion;
    health.header.sequence = ++sequence_;
    health.header.sample_time_ms = now_ms;
    health.header.publish_time_us = now_us_64;
    health.header.status_flags = c::SAMPLE_STATUS_VALID;
    health.header.producer = id();
    health.low_g_ok =
        appHealthy(c::AppId::LOW_G_SENSOR, now_us,
                   nura::config::kRuntimeProfile.sensor_health_max_age_us);
    health.high_accel_ok =
        appHealthy(c::AppId::HIGH_G_SENSOR, now_us,
                   nura::config::kRuntimeProfile.sensor_health_max_age_us);
    health.magnetometer_ok =
        appHealthy(c::AppId::MAGNETOMETER_SENSOR, now_us,
                   nura::config::kRuntimeProfile.sensor_health_max_age_us);
    health.barometer_ok =
        appHealthy(c::AppId::BAROMETER_SENSOR, now_us,
                   nura::config::kRuntimeProfile.sensor_health_max_age_us);
    health.gnss_ok =
        appHealthy(c::AppId::GNSS_SENSOR, now_us,
                   nura::config::kRuntimeProfile.sensor_health_max_age_us);
    health.power_ok =
        appHealthy(c::AppId::POWER_SENSOR, now_us,
                   nura::config::kRuntimeProfile.sensor_health_max_age_us);
    health.safety_input_ok =
        appHealthy(c::AppId::SAFETY_INPUT, now_us,
                   nura::config::kRuntimeProfile.sensor_health_max_age_us);

    const core::BusQueueMetrics queues = bus_.queueMetrics();
    const bool critical_drop = queues.command_dropped != 0U ||
                               queues.actuation_dropped != 0U ||
                               queues.transition_dropped != 0U;
    const bool event_drop = queues.decision_dropped != 0U ||
                            queues.sensor_fault_dropped != 0U;
    const bool critical_sensor_healthy = health.low_g_ok &&
                                         health.high_accel_ok &&
                                         health.barometer_ok &&
                                         health.safety_input_ok;
    if (!mission_healthy || !recovery_healthy || !critical_sensor_healthy ||
        critical_drop || event_drop)
    {
        health.header.status_flags |= c::SAMPLE_STATUS_DEGRADED;
    }

    if (!bus_.publishSensorHealth(health, cycle_id))
    {
        return core::AppRunResult::DEGRADED;
    }

    if (watchdog_ != nullptr)
    {
        if (!watchdog_initialized_)
        {
            watchdog_initialized_ =
                watchdog_->initialize(
                    nura::config::kRuntimeProfile.watchdog_timeout_ms) ==
                p::PortResult::OK;
        }
        if (watchdog_initialized_ && mission_healthy && recovery_healthy &&
            !critical_drop && watchdog_->feed() != p::PortResult::OK)
        {
            return core::AppRunResult::FAILED;
        }
    }

    if (critical_drop)
    {
        return core::AppRunResult::FAILED;
    }
    return (!mission_healthy || !recovery_healthy || event_drop)
               ? core::AppRunResult::DEGRADED
               : core::AppRunResult::OK;
}

bool SupervisorApp::appHealthy(c::AppId app, uint32_t now_us,
                               uint32_t max_age_us) const
{
    core::AppExecutionHealth health{};
    if (!executive_.health(app, health) || health.run_count == 0U ||
        health.last_result == core::AppRunResult::FAILED)
    {
        return false;
    }
    const uint32_t completed_us = static_cast<uint32_t>(health.last_completed_us);
    return (now_us - completed_us) <= max_age_us;
}

} // namespace nura::apps
