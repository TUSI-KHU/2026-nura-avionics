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
    uint32_t seq = 0;
    uint32_t timestampMs = 0;
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

struct FlightState
{
    State state = State::INIT;
    uint32_t stateEnteredMs = 0;
    uint32_t launchMs = 0;
    uint32_t coastMs = 0;
    uint32_t apogeeMs = 0;
    uint32_t drogueMs = 0;
    uint32_t deployMs = 0;
    bool drogueSequenceComplete = false;
    bool mainSequenceComplete = false;
    bool forceRecoveryDeployRequested = false;
    uint16_t forceRecoveryDeployRequestSeq = 0;
    bool forceRecoveryDeployExecuted = false;
    uint16_t forceRecoveryDeployExecutedSeq = 0;
    FlightDecisionTrace decisionTrace;
    FlightDecisionTrace decisionTraceQueue[NuraConstants::Logger::kFlightDecisionTraceQueueDepth];
    uint8_t decisionTraceHead = 0U;
    uint8_t decisionTraceTail = 0U;
    uint8_t decisionTraceCount = 0U;
    uint32_t droppedDecisionTraces = 0U;
    FlightStateTransitionTrace transitionTraceQueue[NuraConstants::Logger::kFlightStateTransitionQueueDepth];
    uint8_t transitionTraceHead = 0U;
    uint8_t transitionTraceTail = 0U;
    uint8_t transitionTraceCount = 0U;
    uint32_t droppedTransitionTraces = 0U;

    void clearDecisionTraceQueue()
    {
        decisionTraceHead = 0U;
        decisionTraceTail = 0U;
        decisionTraceCount = 0U;
        droppedDecisionTraces = 0U;
    }

    void pushDecisionTrace(const FlightDecisionTrace &trace)
    {
        decisionTraceQueue[decisionTraceHead] = trace;
        decisionTraceHead = static_cast<uint8_t>((decisionTraceHead + 1U) %
                                                 NuraConstants::Logger::kFlightDecisionTraceQueueDepth);
        if (decisionTraceCount < NuraConstants::Logger::kFlightDecisionTraceQueueDepth)
        {
            ++decisionTraceCount;
            return;
        }

        decisionTraceTail = static_cast<uint8_t>((decisionTraceTail + 1U) %
                                                 NuraConstants::Logger::kFlightDecisionTraceQueueDepth);
        ++droppedDecisionTraces;
    }

    bool popDecisionTrace(FlightDecisionTrace &trace)
    {
        if (decisionTraceCount == 0U)
        {
            return false;
        }

        trace = decisionTraceQueue[decisionTraceTail];
        decisionTraceTail = static_cast<uint8_t>((decisionTraceTail + 1U) %
                                                 NuraConstants::Logger::kFlightDecisionTraceQueueDepth);
        --decisionTraceCount;
        return true;
    }

    void clearTransitionTraceQueue()
    {
        transitionTraceHead = 0U;
        transitionTraceTail = 0U;
        transitionTraceCount = 0U;
        droppedTransitionTraces = 0U;
    }

    void pushTransitionTrace(State previous, State current, uint32_t timestampMs)
    {
        transitionTraceQueue[transitionTraceHead] = FlightStateTransitionTrace{previous, current, timestampMs};
        transitionTraceHead = static_cast<uint8_t>((transitionTraceHead + 1U) %
                                                   NuraConstants::Logger::kFlightStateTransitionQueueDepth);
        if (transitionTraceCount < NuraConstants::Logger::kFlightStateTransitionQueueDepth)
        {
            ++transitionTraceCount;
            return;
        }

        transitionTraceTail = static_cast<uint8_t>((transitionTraceTail + 1U) %
                                                   NuraConstants::Logger::kFlightStateTransitionQueueDepth);
        ++droppedTransitionTraces;
    }

    bool popTransitionTrace(FlightStateTransitionTrace &trace)
    {
        if (transitionTraceCount == 0U)
        {
            return false;
        }

        trace = transitionTraceQueue[transitionTraceTail];
        transitionTraceTail = static_cast<uint8_t>((transitionTraceTail + 1U) %
                                                   NuraConstants::Logger::kFlightStateTransitionQueueDepth);
        --transitionTraceCount;
        return true;
    }
};

inline bool stateAllowsForceRecoveryDeploy(State state)
{
    return state == State::LAUNCH || state == State::COAST;
}

inline bool recoverySequenceActive(State state)
{
    return state == State::APOGEE ||
           state == State::DROGUE ||
           state == State::DEPLOY;
}
