#pragma once

#include <cstdint>

#include "nura/contracts/common.h"
#include "nura/contracts/flight_topics.h"

namespace nura::contracts
{

enum class TraceEvent : uint8_t
{
    TASK_BEGIN = 0U,
    TASK_END,
    TASK_DEADLINE_MISS,
    TASK_SKIPPED,
    BUS_PUBLISH,
    BUS_CONSUME,
    BUS_DROP,
    STATE_APP_BEGIN,
    STATE_APP_END,
    TRANSITION_REQUEST,
    TRANSITION_COMMIT,
    ACTUATION_INTENT,
    ACTUATION_RESULT,
    HEALTH_CHANGE,
};

struct TraceRecord
{
    uint32_t sequence = 0U;
    uint32_t cycle_id = 0U;
    uint32_t correlation_id = 0U;
    uint64_t timestamp_us = 0U;
    uint32_t duration_us = 0U;
    AppId app = AppId::UNKNOWN;
    AppId peer = AppId::UNKNOWN;
    TopicId topic = TopicId::NONE;
    TraceEvent event = TraceEvent::TASK_BEGIN;
    FlightState state = FlightState::INIT;
    int32_t result = 0;
    uint32_t detail = 0U;
};

inline constexpr const char *appName(AppId id)
{
    switch (id)
    {
    case AppId::LOW_G_SENSOR: return "low_g_sensor";
    case AppId::HIGH_G_SENSOR: return "high_g_sensor";
    case AppId::BAROMETER_SENSOR: return "barometer_sensor";
    case AppId::MAGNETOMETER_SENSOR: return "magnetometer_sensor";
    case AppId::GNSS_SENSOR: return "gnss_sensor";
    case AppId::POWER_SENSOR: return "power_sensor";
    case AppId::SAFETY_INPUT: return "safety_input";
    case AppId::INPUT_AGGREGATOR: return "input_aggregator";
    case AppId::FLIGHT_COORDINATOR: return "flight_coordinator";
    case AppId::LAUNCH_DETECTOR: return "launch_detector";
    case AppId::BURNOUT_DETECTOR: return "burnout_detector";
    case AppId::APOGEE_DETECTOR: return "apogee_detector";
    case AppId::DROGUE_SEQUENCE: return "drogue_sequence";
    case AppId::MAIN_DEPLOY_DETECTOR: return "main_deploy_detector";
    case AppId::LANDING_SEQUENCE: return "landing_sequence";
    case AppId::RECOVERY_ACTUATION: return "recovery_actuation";
    case AppId::ANNUNCIATION: return "annunciation";
    case AppId::TELEMETRY: return "telemetry";
    case AppId::FLIGHT_RECORDER: return "flight_recorder";
    case AppId::EVENT_RECORDER: return "event_recorder";
    case AppId::SUPERVISOR: return "supervisor";
    case AppId::TRACE_EXPORTER: return "trace_exporter";
    case AppId::SYSTEM: return "system";
    case AppId::UNKNOWN:
    default: return "unknown";
    }
}

inline constexpr const char *traceEventName(TraceEvent event)
{
    switch (event)
    {
    case TraceEvent::TASK_BEGIN: return "task_begin";
    case TraceEvent::TASK_END: return "task_end";
    case TraceEvent::TASK_DEADLINE_MISS: return "deadline_miss";
    case TraceEvent::TASK_SKIPPED: return "task_skipped";
    case TraceEvent::BUS_PUBLISH: return "bus_publish";
    case TraceEvent::BUS_CONSUME: return "bus_consume";
    case TraceEvent::BUS_DROP: return "bus_drop";
    case TraceEvent::STATE_APP_BEGIN: return "state_app_begin";
    case TraceEvent::STATE_APP_END: return "state_app_end";
    case TraceEvent::TRANSITION_REQUEST: return "transition_request";
    case TraceEvent::TRANSITION_COMMIT: return "transition_commit";
    case TraceEvent::ACTUATION_INTENT: return "actuation_intent";
    case TraceEvent::ACTUATION_RESULT: return "actuation_result";
    case TraceEvent::HEALTH_CHANGE: return "health_change";
    default: return "unknown";
    }
}

} // namespace nura::contracts
