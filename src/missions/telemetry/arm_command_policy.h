#pragma once

#include <stdint.h>

#include "nura_protocol_v1_lite.h"
#include "state/abort_state.h"
#include "state/flight_state.h"

struct ArmCommandValidation
{
    uint8_t result = nura::RESULT_OK;
    uint8_t reason = nura::REJECT_NONE;

    bool accepted() const
    {
        return result == nura::RESULT_OK;
    }
};

inline ArmCommandValidation validateArmFlightCommand(const nura::ControlPayload &command,
                                                      const FlightState &flightState,
                                                      const AbortState &abortState,
                                                      bool transitionAckPending)
{
    if (command.validUntilMs == 0UL ||
        command.param0 != static_cast<int16_t>(nura::FLIGHT_SAFE) ||
        command.param1 != static_cast<int16_t>(nura::FLIGHT_ARMED))
    {
        return {nura::RESULT_BAD_FORMAT, nura::REJECT_NONE};
    }

    if (flightState.state != State::SAFE ||
        abortState.status.active ||
        flightState.armRequested ||
        flightState.benchResetRequested ||
        transitionAckPending)
    {
        return {nura::RESULT_BAD_STATE, nura::REJECT_STATE_REJECTED};
    }

    return {};
}
