#pragma once

#include <array>
#include <cstddef>
#include <cstdint>

#include "nura/contracts/flight_topics.h"

namespace nura::flight
{

struct TransitionRequest
{
    bool requested = false;
    nura::contracts::FlightState next = nura::contracts::FlightState::FAULT;
    uint32_t timestamp_ms = 0U;
    nura::contracts::AppId requested_by = nura::contracts::AppId::UNKNOWN;
};

struct StateAppOutput
{
    static constexpr size_t kMaxDecisions = 4U;
    static constexpr size_t kMaxActuationIntents = 3U;

    TransitionRequest transition{};
    std::array<nura::contracts::DecisionTrace, kMaxDecisions> decisions{};
    std::array<nura::contracts::ActuationIntent, kMaxActuationIntents> actuation{};
    size_t decision_count = 0U;
    size_t actuation_count = 0U;
    bool latch_barometer_stuck_fault = false;
    bool drogue_sequence_complete = false;
    bool main_sequence_complete = false;

    bool addDecision(const nura::contracts::DecisionTrace &decision)
    {
        if (decision_count >= decisions.size())
        {
            return false;
        }
        decisions[decision_count++] = decision;
        return true;
    }

    bool addActuation(const nura::contracts::ActuationIntent &intent)
    {
        if (actuation_count >= actuation.size())
        {
            return false;
        }
        actuation[actuation_count++] = intent;
        return true;
    }

    void requestTransition(nura::contracts::FlightState next, uint32_t timestamp_ms,
                           nura::contracts::AppId requested_by)
    {
        transition.requested = true;
        transition.next = next;
        transition.timestamp_ms = timestamp_ms;
        transition.requested_by = requested_by;
    }
};

class IStateApp
{
public:
    virtual ~IStateApp() = default;
    virtual nura::contracts::AppId id() const = 0;
    virtual nura::contracts::FlightState state() const = 0;
    virtual StateAppOutput onEnter(const nura::contracts::FlightStatus &status,
                                   const nura::contracts::FlightInputs &inputs,
                                   uint32_t now_ms) = 0;
    virtual StateAppOutput step(const nura::contracts::FlightStatus &status,
                                const nura::contracts::FlightInputs &inputs,
                                uint32_t now_ms) = 0;
};

class IStateAppRunner
{
public:
    virtual ~IStateAppRunner() = default;
    virtual StateAppOutput enter(IStateApp &app,
                                 const nura::contracts::FlightStatus &status,
                                 const nura::contracts::FlightInputs &inputs,
                                 uint32_t now_ms, uint32_t cycle_id) = 0;
    virtual StateAppOutput step(IStateApp &app,
                                const nura::contracts::FlightStatus &status,
                                const nura::contracts::FlightInputs &inputs,
                                uint32_t now_ms, uint32_t cycle_id) = 0;
};

inline nura::contracts::DecisionTrace makeDecision(
    nura::contracts::FlightState state,
    nura::contracts::DecisionKind kind,
    nura::contracts::DecisionResult result,
    uint16_t reason,
    uint32_t timestamp_ms,
    float value0,
    float value1,
    float value2,
    float value3,
    uint8_t count0,
    uint8_t count1)
{
    nura::contracts::DecisionTrace decision{};
    decision.timestamp_ms = timestamp_ms;
    decision.state = state;
    decision.kind = kind;
    decision.result = result;
    decision.reason = reason;
    decision.value0 = value0;
    decision.value1 = value1;
    decision.value2 = value2;
    decision.value3 = value3;
    decision.count0 = count0;
    decision.count1 = count1;
    return decision;
}

} // namespace nura::flight
