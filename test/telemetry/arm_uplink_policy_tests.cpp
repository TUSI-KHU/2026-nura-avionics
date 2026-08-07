#include <cstdio>

#include "missions/telemetry/arm_command_policy.h"

namespace
{
nura::ControlPayload validArmCommand()
{
    nura::ControlPayload command;
    command.subtype = nura::CONTROL_CMD;
    command.commandId = nura::COMMAND_ARM_FLIGHT;
    command.validUntilMs = 5000UL;
    command.param0 = nura::FLIGHT_SAFE;
    command.param1 = nura::FLIGHT_ARMED;
    return command;
}

bool expectDecision(const char *name,
                    const ArmCommandValidation &actual,
                    uint8_t expectedResult,
                    uint8_t expectedReason)
{
    if (actual.result != expectedResult || actual.reason != expectedReason)
    {
        std::fprintf(stderr,
                     "FAIL %s result=%u reason=%u expected_result=%u expected_reason=%u\n",
                     name,
                     actual.result,
                     actual.reason,
                     expectedResult,
                     expectedReason);
        return false;
    }
    return true;
}
} // namespace

int main()
{
    bool ok = true;
    FlightState flight;
    flight.state = State::SAFE;
    AbortState abort;
    nura::ControlPayload command = validArmCommand();

    ok = expectDecision("valid",
                        validateArmFlightCommand(command, flight, abort, false),
                        nura::RESULT_OK,
                        nura::REJECT_NONE) &&
         ok;

    command.validUntilMs = 0UL;
    ok = expectDecision("zero_expiry",
                        validateArmFlightCommand(command, flight, abort, false),
                        nura::RESULT_BAD_FORMAT,
                        nura::REJECT_NONE) &&
         ok;
    command = validArmCommand();
    command.param0 = nura::FLIGHT_ARMED;
    ok = expectDecision("bad_source",
                        validateArmFlightCommand(command, flight, abort, false),
                        nura::RESULT_BAD_FORMAT,
                        nura::REJECT_NONE) &&
         ok;
    command = validArmCommand();
    command.param1 = nura::FLIGHT_LAUNCH;
    ok = expectDecision("bad_target",
                        validateArmFlightCommand(command, flight, abort, false),
                        nura::RESULT_BAD_FORMAT,
                        nura::REJECT_NONE) &&
         ok;

    const State rejectedStates[] = {
        State::INIT,
        State::ARMED,
        State::LAUNCH,
        State::COAST,
        State::APOGEE,
        State::DROGUE,
        State::DEPLOY,
        State::GROUND,
        State::FAULT,
    };
    for (State state : rejectedStates)
    {
        flight = FlightState{};
        flight.state = state;
        ok = expectDecision("non_safe_state",
                            validateArmFlightCommand(validArmCommand(), flight, abort, false),
                            nura::RESULT_BAD_STATE,
                            nura::REJECT_STATE_REJECTED) &&
             ok;
    }

    flight = FlightState{};
    flight.state = State::SAFE;
    abort.status.active = true;
    ok = expectDecision("abort_active",
                        validateArmFlightCommand(validArmCommand(), flight, abort, false),
                        nura::RESULT_BAD_STATE,
                        nura::REJECT_STATE_REJECTED) &&
         ok;
    abort = AbortState{};

    flight.armRequested = true;
    ok = expectDecision("arm_pending",
                        validateArmFlightCommand(validArmCommand(), flight, abort, false),
                        nura::RESULT_BAD_STATE,
                        nura::REJECT_STATE_REJECTED) &&
         ok;
    flight.armRequested = false;
    flight.benchResetRequested = true;
    ok = expectDecision("reset_pending",
                        validateArmFlightCommand(validArmCommand(), flight, abort, false),
                        nura::RESULT_BAD_STATE,
                        nura::REJECT_STATE_REJECTED) &&
         ok;
    flight.benchResetRequested = false;
    ok = expectDecision("ack_pending",
                        validateArmFlightCommand(validArmCommand(), flight, abort, true),
                        nura::RESULT_BAD_STATE,
                        nura::REJECT_STATE_REJECTED) &&
         ok;

    if (!ok)
    {
        return 1;
    }
    std::puts("ARM uplink policy tests passed");
    return 0;
}
