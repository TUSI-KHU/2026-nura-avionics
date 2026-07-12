#include "nura/core/software_bus.h"

namespace nura::core
{
namespace c = nura::contracts;

void SoftwareBus::traceBus(c::TraceEvent event, c::AppId app, c::AppId peer,
                           c::TopicId topic, uint32_t cycle_id,
                           uint32_t correlation_id, uint64_t now_us,
                           int32_t result, uint32_t detail)
{
    c::TraceRecord record{};
    record.cycle_id = cycle_id;
    record.correlation_id = correlation_id;
    record.timestamp_us = now_us;
    record.app = app;
    record.peer = peer;
    record.topic = topic;
    record.event = event;
    record.result = result;
    record.detail = detail;
    (void)trace_map_.tryRecord(record);
}

bool SoftwareBus::publishLowG(const c::LowGImuSample &value, uint32_t cycle_id)
{
    const bool ok = low_g_.tryPublish(value);
    traceBus(ok ? c::TraceEvent::BUS_PUBLISH : c::TraceEvent::BUS_DROP,
             value.header.producer, c::AppId::UNKNOWN, c::TopicId::LOW_G_LATEST,
             cycle_id, value.header.sequence, value.header.publish_time_us, ok ? 0 : -1);
    return ok;
}

bool SoftwareBus::readLowG(c::LowGImuSample &value, c::AppId consumer,
                           uint32_t cycle_id, uint64_t now_us)
{
    const bool ok = low_g_.tryRead(value);
    if (ok)
    {
        traceBus(c::TraceEvent::BUS_CONSUME, consumer, value.header.producer,
                 c::TopicId::LOW_G_LATEST, cycle_id, value.header.sequence, now_us, 0);
    }
    return ok;
}

bool SoftwareBus::publishHighG(const c::HighGImuSample &value, uint32_t cycle_id)
{
    const bool ok = high_g_.tryPublish(value);
    traceBus(ok ? c::TraceEvent::BUS_PUBLISH : c::TraceEvent::BUS_DROP,
             value.header.producer, c::AppId::UNKNOWN, c::TopicId::HIGH_G_LATEST,
             cycle_id, value.header.sequence, value.header.publish_time_us, ok ? 0 : -1);
    return ok;
}

bool SoftwareBus::readHighG(c::HighGImuSample &value, c::AppId consumer,
                            uint32_t cycle_id, uint64_t now_us)
{
    const bool ok = high_g_.tryRead(value);
    if (ok)
    {
        traceBus(c::TraceEvent::BUS_CONSUME, consumer, value.header.producer,
                 c::TopicId::HIGH_G_LATEST, cycle_id, value.header.sequence, now_us, 0);
    }
    return ok;
}

bool SoftwareBus::publishBarometer(const c::BarometerSample &value, uint32_t cycle_id)
{
    const bool ok = barometer_.tryPublish(value);
    traceBus(ok ? c::TraceEvent::BUS_PUBLISH : c::TraceEvent::BUS_DROP,
             value.header.producer, c::AppId::UNKNOWN, c::TopicId::BAROMETER_LATEST,
             cycle_id, value.header.sequence, value.header.publish_time_us, ok ? 0 : -1);
    return ok;
}

bool SoftwareBus::readBarometer(c::BarometerSample &value, c::AppId consumer,
                                uint32_t cycle_id, uint64_t now_us)
{
    const bool ok = barometer_.tryRead(value);
    if (ok)
    {
        traceBus(c::TraceEvent::BUS_CONSUME, consumer, value.header.producer,
                 c::TopicId::BAROMETER_LATEST, cycle_id, value.header.sequence, now_us, 0);
    }
    return ok;
}

bool SoftwareBus::publishMagnetometer(const c::MagnetometerSample &value,
                                      uint32_t cycle_id)
{
    const bool ok = magnetometer_.tryPublish(value);
    traceBus(ok ? c::TraceEvent::BUS_PUBLISH : c::TraceEvent::BUS_DROP,
             value.header.producer, c::AppId::UNKNOWN,
             c::TopicId::MAGNETOMETER_LATEST, cycle_id, value.header.sequence,
             value.header.publish_time_us, ok ? 0 : -1);
    return ok;
}

bool SoftwareBus::readMagnetometer(c::MagnetometerSample &value,
                                   c::AppId consumer, uint32_t cycle_id,
                                   uint64_t now_us)
{
    const bool ok = magnetometer_.tryRead(value);
    if (ok)
    {
        traceBus(c::TraceEvent::BUS_CONSUME, consumer, value.header.producer,
                 c::TopicId::MAGNETOMETER_LATEST, cycle_id,
                 value.header.sequence, now_us, 0);
    }
    return ok;
}

bool SoftwareBus::publishGnss(const c::GnssSample &value, uint32_t cycle_id)
{
    const bool ok = gnss_.tryPublish(value);
    traceBus(ok ? c::TraceEvent::BUS_PUBLISH : c::TraceEvent::BUS_DROP,
             value.header.producer, c::AppId::UNKNOWN, c::TopicId::GNSS_LATEST,
             cycle_id, value.header.sequence, value.header.publish_time_us,
             ok ? 0 : -1);
    return ok;
}

bool SoftwareBus::readGnss(c::GnssSample &value, c::AppId consumer,
                           uint32_t cycle_id, uint64_t now_us)
{
    const bool ok = gnss_.tryRead(value);
    if (ok)
    {
        traceBus(c::TraceEvent::BUS_CONSUME, consumer, value.header.producer,
                 c::TopicId::GNSS_LATEST, cycle_id, value.header.sequence,
                 now_us, 0);
    }
    return ok;
}

bool SoftwareBus::publishPower(const c::PowerSample &value, uint32_t cycle_id)
{
    const bool ok = power_.tryPublish(value);
    traceBus(ok ? c::TraceEvent::BUS_PUBLISH : c::TraceEvent::BUS_DROP,
             value.header.producer, c::AppId::UNKNOWN, c::TopicId::POWER_LATEST,
             cycle_id, value.header.sequence, value.header.publish_time_us,
             ok ? 0 : -1);
    return ok;
}

bool SoftwareBus::readPower(c::PowerSample &value, c::AppId consumer,
                            uint32_t cycle_id, uint64_t now_us)
{
    const bool ok = power_.tryRead(value);
    if (ok)
    {
        traceBus(c::TraceEvent::BUS_CONSUME, consumer, value.header.producer,
                 c::TopicId::POWER_LATEST, cycle_id, value.header.sequence,
                 now_us, 0);
    }
    return ok;
}

bool SoftwareBus::publishSensorHealth(const c::SensorHealthSnapshot &value,
                                      uint32_t cycle_id)
{
    const bool ok = sensor_health_.tryPublish(value);
    traceBus(ok ? c::TraceEvent::BUS_PUBLISH : c::TraceEvent::BUS_DROP,
             value.header.producer, c::AppId::UNKNOWN,
             c::TopicId::SENSOR_HEALTH_LATEST, cycle_id, value.header.sequence,
             value.header.publish_time_us, ok ? 0 : -1);
    return ok;
}

bool SoftwareBus::readSensorHealth(c::SensorHealthSnapshot &value, c::AppId consumer,
                                   uint32_t cycle_id, uint64_t now_us)
{
    const bool ok = sensor_health_.tryRead(value);
    if (ok)
    {
        traceBus(c::TraceEvent::BUS_CONSUME, consumer, value.header.producer,
                 c::TopicId::SENSOR_HEALTH_LATEST, cycle_id, value.header.sequence,
                 now_us, 0);
    }
    return ok;
}

bool SoftwareBus::publishSafetyStatus(const c::SafetyStatus &value,
                                      uint32_t cycle_id)
{
    const bool ok = safety_status_.tryPublish(value);
    traceBus(ok ? c::TraceEvent::BUS_PUBLISH : c::TraceEvent::BUS_DROP,
             value.header.producer, c::AppId::FLIGHT_COORDINATOR,
             c::TopicId::SAFETY_STATUS_LATEST, cycle_id, value.header.sequence,
             value.header.publish_time_us, ok ? 0 : -1);
    return ok;
}

bool SoftwareBus::readSafetyStatus(c::SafetyStatus &value, c::AppId consumer,
                                   uint32_t cycle_id, uint64_t now_us)
{
    const bool ok = safety_status_.tryRead(value);
    if (ok)
    {
        traceBus(c::TraceEvent::BUS_CONSUME, consumer, value.header.producer,
                 c::TopicId::SAFETY_STATUS_LATEST, cycle_id,
                 value.header.sequence, now_us, 0);
    }
    return ok;
}

bool SoftwareBus::publishFlightInputs(const c::FlightInputs &value, uint32_t cycle_id)
{
    const bool ok = flight_inputs_.tryPublish(value);
    traceBus(ok ? c::TraceEvent::BUS_PUBLISH : c::TraceEvent::BUS_DROP,
             value.header.producer, c::AppId::FLIGHT_COORDINATOR,
             c::TopicId::FLIGHT_INPUTS_LATEST, cycle_id, value.header.sequence,
             value.header.publish_time_us, ok ? 0 : -1);
    return ok;
}

bool SoftwareBus::readFlightInputs(c::FlightInputs &value, c::AppId consumer,
                                   uint32_t cycle_id, uint64_t now_us)
{
    const bool ok = flight_inputs_.tryRead(value);
    if (ok)
    {
        traceBus(c::TraceEvent::BUS_CONSUME, consumer, value.header.producer,
                 c::TopicId::FLIGHT_INPUTS_LATEST, cycle_id, value.header.sequence,
                 now_us, 0);
    }
    return ok;
}

bool SoftwareBus::publishFlightStatus(const c::FlightStatus &value, uint32_t cycle_id)
{
    const bool ok = flight_status_.tryPublish(value);
    traceBus(ok ? c::TraceEvent::BUS_PUBLISH : c::TraceEvent::BUS_DROP,
             value.header.producer, c::AppId::UNKNOWN, c::TopicId::FLIGHT_STATUS_LATEST,
             cycle_id, value.header.sequence, value.header.publish_time_us, ok ? 0 : -1);
    return ok;
}

bool SoftwareBus::readFlightStatus(c::FlightStatus &value, c::AppId consumer,
                                   uint32_t cycle_id, uint64_t now_us)
{
    const bool ok = flight_status_.tryRead(value);
    if (ok)
    {
        traceBus(c::TraceEvent::BUS_CONSUME, consumer, value.header.producer,
                 c::TopicId::FLIGHT_STATUS_LATEST, cycle_id, value.header.sequence,
                 now_us, 0);
    }
    return ok;
}

bool SoftwareBus::publishAppEnable(const c::AppEnableSet &value, uint32_t cycle_id)
{
    const bool ok = app_enable_.tryPublish(value);
    traceBus(ok ? c::TraceEvent::BUS_PUBLISH : c::TraceEvent::BUS_DROP,
             value.header.producer, c::AppId::SYSTEM, c::TopicId::APP_ENABLE_LATEST,
             cycle_id, value.header.sequence, value.header.publish_time_us, ok ? 0 : -1);
    return ok;
}

bool SoftwareBus::readAppEnable(c::AppEnableSet &value, c::AppId consumer,
                                uint32_t cycle_id, uint64_t now_us)
{
    const bool ok = app_enable_.tryRead(value);
    if (ok)
    {
        traceBus(c::TraceEvent::BUS_CONSUME, consumer, value.header.producer,
                 c::TopicId::APP_ENABLE_LATEST, cycle_id, value.header.sequence,
                 now_us, 0);
    }
    return ok;
}

bool SoftwareBus::pushCommand(const c::CommandRequest &value, uint32_t cycle_id,
                              uint64_t now_us)
{
    const bool ok = command_queue_.tryPush(value);
    traceBus(ok ? c::TraceEvent::BUS_PUBLISH : c::TraceEvent::BUS_DROP,
             value.source, c::AppId::FLIGHT_COORDINATOR, c::TopicId::COMMAND_REQUEST_QUEUE,
             cycle_id, value.sequence, now_us, ok ? 0 : -1,
             static_cast<uint32_t>(command_queue_.depth()));
    return ok;
}

bool SoftwareBus::popCommand(c::CommandRequest &value, c::AppId consumer,
                             uint32_t cycle_id, uint64_t now_us)
{
    const bool ok = command_queue_.tryPop(value);
    if (ok)
    {
        traceBus(c::TraceEvent::BUS_CONSUME, consumer, value.source,
                 c::TopicId::COMMAND_REQUEST_QUEUE, cycle_id, value.sequence, now_us, 0,
                 static_cast<uint32_t>(command_queue_.depth()));
    }
    return ok;
}

bool SoftwareBus::pushActuation(const c::ActuationIntent &value, uint32_t cycle_id,
                                uint64_t now_us)
{
    const bool ok = actuation_queue_.tryPush(value);
    traceBus(ok ? c::TraceEvent::ACTUATION_INTENT : c::TraceEvent::BUS_DROP,
             value.source, c::AppId::RECOVERY_ACTUATION,
             c::TopicId::ACTUATION_INTENT_QUEUE, cycle_id, value.sequence, now_us,
             ok ? 0 : -1, static_cast<uint32_t>(actuation_queue_.depth()));
    return ok;
}

bool SoftwareBus::popActuation(c::ActuationIntent &value, c::AppId consumer,
                               uint32_t cycle_id, uint64_t now_us)
{
    const bool ok = actuation_queue_.tryPop(value);
    if (ok)
    {
        traceBus(c::TraceEvent::BUS_CONSUME, consumer, value.source,
                 c::TopicId::ACTUATION_INTENT_QUEUE, cycle_id, value.sequence, now_us, 0,
                 static_cast<uint32_t>(actuation_queue_.depth()));
    }
    return ok;
}

bool SoftwareBus::pushTransition(const c::TransitionEvent &value, uint32_t cycle_id,
                                 uint64_t now_us)
{
    const bool ok = transition_queue_.tryPush(value);
    traceBus(ok ? c::TraceEvent::BUS_PUBLISH : c::TraceEvent::BUS_DROP,
             c::AppId::FLIGHT_COORDINATOR, c::AppId::EVENT_RECORDER,
             c::TopicId::TRANSITION_EVENT_QUEUE, cycle_id, value.sequence, now_us,
             ok ? 0 : -1, static_cast<uint32_t>(transition_queue_.depth()));
    return ok;
}

bool SoftwareBus::popTransition(c::TransitionEvent &value, c::AppId consumer,
                                uint32_t cycle_id, uint64_t now_us)
{
    const bool ok = transition_queue_.tryPop(value);
    if (ok)
    {
        traceBus(c::TraceEvent::BUS_CONSUME, consumer, c::AppId::FLIGHT_COORDINATOR,
                 c::TopicId::TRANSITION_EVENT_QUEUE, cycle_id, value.sequence, now_us, 0,
                 static_cast<uint32_t>(transition_queue_.depth()));
    }
    return ok;
}

bool SoftwareBus::pushDecision(const c::DecisionTrace &value, uint32_t cycle_id,
                               uint64_t now_us)
{
    const bool ok = decision_queue_.tryPush(value);
    traceBus(ok ? c::TraceEvent::BUS_PUBLISH : c::TraceEvent::BUS_DROP,
             c::AppId::FLIGHT_COORDINATOR, c::AppId::EVENT_RECORDER,
             c::TopicId::DECISION_EVENT_QUEUE, cycle_id, value.sequence, now_us,
             ok ? 0 : -1, static_cast<uint32_t>(decision_queue_.depth()));
    return ok;
}

bool SoftwareBus::popDecision(c::DecisionTrace &value, c::AppId consumer,
                              uint32_t cycle_id, uint64_t now_us)
{
    const bool ok = decision_queue_.tryPop(value);
    if (ok)
    {
        traceBus(c::TraceEvent::BUS_CONSUME, consumer, c::AppId::FLIGHT_COORDINATOR,
                 c::TopicId::DECISION_EVENT_QUEUE, cycle_id, value.sequence, now_us, 0,
                 static_cast<uint32_t>(decision_queue_.depth()));
    }
    return ok;
}

bool SoftwareBus::pushSensorFault(const c::SensorFaultEvent &value, uint32_t cycle_id,
                                  uint64_t now_us)
{
    const bool ok = sensor_fault_queue_.tryPush(value);
    traceBus(ok ? c::TraceEvent::HEALTH_CHANGE : c::TraceEvent::BUS_DROP,
             value.source, c::AppId::SUPERVISOR, c::TopicId::SENSOR_FAULT_QUEUE,
             cycle_id, value.sequence, now_us, ok ? 0 : -1,
             static_cast<uint32_t>(sensor_fault_queue_.depth()));
    return ok;
}

bool SoftwareBus::popSensorFault(c::SensorFaultEvent &value, c::AppId consumer,
                                 uint32_t cycle_id, uint64_t now_us)
{
    const bool ok = sensor_fault_queue_.tryPop(value);
    if (ok)
    {
        traceBus(c::TraceEvent::BUS_CONSUME, consumer, value.source,
                 c::TopicId::SENSOR_FAULT_QUEUE, cycle_id, value.sequence, now_us, 0,
                 static_cast<uint32_t>(sensor_fault_queue_.depth()));
    }
    return ok;
}

BusQueueMetrics SoftwareBus::queueMetrics() const
{
    BusQueueMetrics metrics{};
    metrics.command_high_water = command_queue_.highWater();
    metrics.command_dropped = command_queue_.dropped();
    metrics.actuation_high_water = actuation_queue_.highWater();
    metrics.actuation_dropped = actuation_queue_.dropped();
    metrics.transition_high_water = transition_queue_.highWater();
    metrics.transition_dropped = transition_queue_.dropped();
    metrics.decision_high_water = decision_queue_.highWater();
    metrics.decision_dropped = decision_queue_.dropped();
    metrics.sensor_fault_high_water = sensor_fault_queue_.highWater();
    metrics.sensor_fault_dropped = sensor_fault_queue_.dropped();
    return metrics;
}

BusLatestMetrics SoftwareBus::latestMetrics() const
{
    BusLatestMetrics metrics{};
#define NURA_ACCUMULATE_LATEST(topic)                                                \
    metrics.publish_contention += topic.publishContention();                         \
    metrics.read_contention += topic.readContention()
    NURA_ACCUMULATE_LATEST(low_g_);
    NURA_ACCUMULATE_LATEST(high_g_);
    NURA_ACCUMULATE_LATEST(barometer_);
    NURA_ACCUMULATE_LATEST(magnetometer_);
    NURA_ACCUMULATE_LATEST(gnss_);
    NURA_ACCUMULATE_LATEST(power_);
    NURA_ACCUMULATE_LATEST(sensor_health_);
    NURA_ACCUMULATE_LATEST(safety_status_);
    NURA_ACCUMULATE_LATEST(flight_inputs_);
    NURA_ACCUMULATE_LATEST(flight_status_);
    NURA_ACCUMULATE_LATEST(app_enable_);
#undef NURA_ACCUMULATE_LATEST
    return metrics;
}

} // namespace nura::core
