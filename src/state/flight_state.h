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
    bool armRequested = false;
    uint16_t armRequestSeq = 0;
    bool armExecuted = false;
    uint16_t armExecutedSeq = 0;
    bool armRejected = false;
    uint16_t armRejectedSeq = 0;
    bool forceRecoveryDeployRequested = false;
    uint16_t forceRecoveryDeployRequestSeq = 0;
    bool forceRecoveryDeployExecuted = false;
    uint16_t forceRecoveryDeployExecutedSeq = 0;
    bool benchResetRequested = false;
    uint16_t benchResetRequestSeq = 0;
    bool benchResetExecuted = false;
    uint16_t benchResetExecutedSeq = 0;
};
