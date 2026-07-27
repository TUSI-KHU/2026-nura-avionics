#pragma once

#include <stdint.h>

#include "core/states.h"
#include "nura_constants.h"

enum class FlightDecisionKind : uint8_t
{
    NONE = 0U,
    LAUNCH_ACCEL,
    BURNOUT_ACCEL,
    APOGEE_PREDICTION,
    APOGEE_DESCENT,
    APOGEE_TIMER,
    BARO_FAULT_TILT,
    FORCE_DEPLOY,
    BENCH_RESET,
    MAIN_DEPLOY,
    LANDING,
};

enum class FlightDecisionResult : uint8_t
{
    OBSERVE = 0U,
    REJECT,
    ACCEPT,
};

enum FlightDecisionReason : uint16_t
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

struct FlightDecisionTrace
{
    uint32_t seq = 0U;
    uint32_t timestampMs = 0U;
    State state = State::INIT;
    FlightDecisionKind kind = FlightDecisionKind::NONE;
    FlightDecisionResult result = FlightDecisionResult::OBSERVE;
    uint16_t reason = DECISION_REASON_NONE;
    float value0 = 0.0f;
    float value1 = 0.0f;
    float value2 = 0.0f;
    float value3 = 0.0f;
    uint8_t count0 = 0U;
    uint8_t count1 = 0U;
};

struct FlightStateTransitionTrace
{
    State previous = State::INIT;
    State current = State::INIT;
    uint32_t timestampMs = 0U;
};

class FlightTraceBuffer
{
public:
    void clear();
    void pushDecision(const FlightDecisionTrace &trace);
    bool popDecision(FlightDecisionTrace &trace);
    void pushTransition(State previous, State current, uint32_t timestampMs);
    bool popTransition(FlightStateTransitionTrace &trace);

    const FlightDecisionTrace &latestDecision() const;
    uint32_t droppedDecisionCount() const;
    uint32_t droppedTransitionCount() const;

private:
    FlightDecisionTrace latestDecision_;
    FlightDecisionTrace decisions_[NuraConstants::Logger::kFlightDecisionTraceQueueDepth];
    FlightStateTransitionTrace transitions_[NuraConstants::Logger::kFlightStateTransitionQueueDepth];
    uint8_t decisionHead_ = 0U;
    uint8_t decisionTail_ = 0U;
    uint8_t decisionCount_ = 0U;
    uint8_t transitionHead_ = 0U;
    uint8_t transitionTail_ = 0U;
    uint8_t transitionCount_ = 0U;
    uint32_t droppedDecisions_ = 0U;
    uint32_t droppedTransitions_ = 0U;
};

inline bool stateAllowsForceRecoveryDeploy(State state)
{
    return state == State::LAUNCH || state == State::COAST;
}

inline bool recoverySequenceActive(State state)
{
    return state == State::APOGEE || state == State::DROGUE || state == State::DEPLOY;
}
