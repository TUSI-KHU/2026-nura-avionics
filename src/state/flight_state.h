#pragma once

#include <stdint.h>

#include "core/states.h"

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
