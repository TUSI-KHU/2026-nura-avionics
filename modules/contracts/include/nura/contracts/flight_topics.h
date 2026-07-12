#pragma once

#include <cstdint>

#include "nura/contracts/common.h"
#include "nura/contracts/sensor_topics.h"

namespace nura::contracts
{

enum class FlightState : uint8_t
{
    INIT = 0U,
    SAFE = 1U,
    ARMED = 2U,
    LAUNCH = 3U,
    COAST = 4U,
    APOGEE = 5U,
    DROGUE = 6U,
    DEPLOY = 7U,
    GROUND = 8U,
    FAULT = 9U,
};

enum class DecisionKind : uint8_t
{
    NONE = 0U,
    LAUNCH_ACCEL,
    BURNOUT_ACCEL,
    APOGEE_PREDICTION,
    APOGEE_DESCENT,
    APOGEE_TIMER,
    BAROMETER_FAULT_TILT,
    FORCE_DEPLOY,
    MAIN_DEPLOY,
    LANDING,
};

enum class DecisionResult : uint8_t
{
    OBSERVE = 0U,
    REJECT,
    ACCEPT,
};

enum DecisionReason : uint16_t
{
    DECISION_REASON_NONE = 0U,
    DECISION_REASON_PRIMARY_SENSOR = 1U << 0,
    DECISION_REASON_FALLBACK_SENSOR = 1U << 1,
    DECISION_REASON_THRESHOLD_MET = 1U << 2,
    DECISION_REASON_THRESHOLD_NOT_MET = 1U << 3,
    DECISION_REASON_CONFIRMATION_MET = 1U << 4,
    DECISION_REASON_TOO_EARLY = 1U << 5,
    DECISION_REASON_SENSOR_FAULT = 1U << 6,
    DECISION_REASON_TIMEOUT = 1U << 7,
    DECISION_REASON_QUALITY_REJECT = 1U << 8,
    DECISION_REASON_FORCED = 1U << 9,
};

struct FlightInputs
{
    SampleHeader header{};
    LowGImuSample low_g{};
    HighGImuSample high_g{};
    BarometerSample barometer{};
    SensorHealthSnapshot health{};
    bool abort_active = false;
};

struct SafetyStatus
{
    SampleHeader header{};
    bool abort_active = false;
};

struct FlightStatus
{
    SampleHeader header{};
    FlightState state = FlightState::INIT;
    uint32_t state_entered_ms = 0U;
    uint32_t launch_ms = 0U;
    uint32_t coast_ms = 0U;
    uint32_t apogee_ms = 0U;
    uint32_t drogue_ms = 0U;
    uint32_t deploy_ms = 0U;
    uint32_t transition_sequence = 0U;
    uint32_t decision_sequence = 0U;
    bool drogue_sequence_complete = false;
    bool main_sequence_complete = false;
    bool recovery_deploy_started = false;
    bool force_recovery_executed = false;
    uint16_t force_recovery_executed_sequence = 0U;
    bool barometer_stuck_fault_latched = false;
};

struct DecisionTrace
{
    uint32_t sequence = 0U;
    uint32_t timestamp_ms = 0U;
    FlightState state = FlightState::INIT;
    DecisionKind kind = DecisionKind::NONE;
    DecisionResult result = DecisionResult::OBSERVE;
    uint16_t reason = DECISION_REASON_NONE;
    float value0 = 0.0f;
    float value1 = 0.0f;
    float value2 = 0.0f;
    float value3 = 0.0f;
    uint8_t count0 = 0U;
    uint8_t count1 = 0U;
};

struct TransitionEvent
{
    uint32_t sequence = 0U;
    uint32_t timestamp_ms = 0U;
    FlightState previous = FlightState::INIT;
    FlightState current = FlightState::INIT;
    AppId requested_by = AppId::FLIGHT_COORDINATOR;
};

enum class CommandType : uint8_t
{
    NONE = 0U,
    FORCE_RECOVERY_DEPLOY = 1U,
};

struct CommandRequest
{
    uint32_t sequence = 0U;
    uint32_t timestamp_ms = 0U;
    CommandType type = CommandType::NONE;
    uint16_t command_sequence = 0U;
    AppId source = AppId::TELEMETRY;
};

enum class ActuationOperation : uint8_t
{
    ALL_OFF = 0U,
    SET_CHANNEL = 1U,
};

enum class RecoveryChannel : uint8_t
{
    DROGUE_PRIMARY = 0U,
    MAIN_PRIMARY = 1U,
};

struct ActuationIntent
{
    uint32_t sequence = 0U;
    uint32_t timestamp_ms = 0U;
    uint32_t transition_sequence = 0U;
    ActuationOperation operation = ActuationOperation::ALL_OFF;
    RecoveryChannel channel = RecoveryChannel::DROGUE_PRIMARY;
    bool enabled = false;
    FlightState authorized_state = FlightState::SAFE;
    AppId source = AppId::FLIGHT_COORDINATOR;
};

struct SensorFaultEvent
{
    uint32_t sequence = 0U;
    uint32_t timestamp_ms = 0U;
    TopicId sensor_topic = TopicId::NONE;
    uint32_t fault_flags = 0U;
    AppId source = AppId::UNKNOWN;
};

struct AppEnableSet
{
    SampleHeader header{};
    FlightState state = FlightState::INIT;
    uint64_t enabled_mask = 0U;
};

inline constexpr bool stateAllowsForceRecoveryDeploy(FlightState state)
{
    return state == FlightState::LAUNCH || state == FlightState::COAST;
}

inline constexpr bool recoverySequenceActive(FlightState state)
{
    return state == FlightState::APOGEE || state == FlightState::DROGUE ||
           state == FlightState::DEPLOY;
}

static_assert(static_cast<uint8_t>(FlightState::FAULT) == 9U,
              "flight-state wire/log IDs must remain stable");
static_assert(payload_fits_v<FlightInputs, 512U>, "flight input snapshot exceeds copy budget");
static_assert(payload_fits_v<FlightStatus, 160U>, "flight status topic exceeds copy budget");

} // namespace nura::contracts
