#include "missions/flight/flight_trace.h"

void FlightTraceBuffer::clear()
{
    latestDecision_ = FlightDecisionTrace{};
    decisionHead_ = 0U;
    decisionTail_ = 0U;
    decisionCount_ = 0U;
    transitionHead_ = 0U;
    transitionTail_ = 0U;
    transitionCount_ = 0U;
    droppedDecisions_ = 0U;
    droppedTransitions_ = 0U;
}

void FlightTraceBuffer::pushDecision(const FlightDecisionTrace &trace)
{
    latestDecision_ = trace;
    decisions_[decisionHead_] = trace;
    decisionHead_ = static_cast<uint8_t>((decisionHead_ + 1U) % NuraConstants::Logger::kFlightDecisionTraceQueueDepth);
    if (decisionCount_ < NuraConstants::Logger::kFlightDecisionTraceQueueDepth)
    {
        ++decisionCount_;
        return;
    }
    decisionTail_ = static_cast<uint8_t>((decisionTail_ + 1U) % NuraConstants::Logger::kFlightDecisionTraceQueueDepth);
    ++droppedDecisions_;
}

bool FlightTraceBuffer::popDecision(FlightDecisionTrace &trace)
{
    if (decisionCount_ == 0U)
    {
        return false;
    }
    trace = decisions_[decisionTail_];
    decisionTail_ = static_cast<uint8_t>((decisionTail_ + 1U) % NuraConstants::Logger::kFlightDecisionTraceQueueDepth);
    --decisionCount_;
    return true;
}

void FlightTraceBuffer::pushTransition(State previous, State current, uint32_t timestampMs)
{
    transitions_[transitionHead_] = FlightStateTransitionTrace{previous, current, timestampMs};
    transitionHead_ = static_cast<uint8_t>((transitionHead_ + 1U) % NuraConstants::Logger::kFlightStateTransitionQueueDepth);
    if (transitionCount_ < NuraConstants::Logger::kFlightStateTransitionQueueDepth)
    {
        ++transitionCount_;
        return;
    }
    transitionTail_ = static_cast<uint8_t>((transitionTail_ + 1U) % NuraConstants::Logger::kFlightStateTransitionQueueDepth);
    ++droppedTransitions_;
}

bool FlightTraceBuffer::popTransition(FlightStateTransitionTrace &trace)
{
    if (transitionCount_ == 0U)
    {
        return false;
    }
    trace = transitions_[transitionTail_];
    transitionTail_ = static_cast<uint8_t>((transitionTail_ + 1U) % NuraConstants::Logger::kFlightStateTransitionQueueDepth);
    --transitionCount_;
    return true;
}

const FlightDecisionTrace &FlightTraceBuffer::latestDecision() const
{
    return latestDecision_;
}

uint32_t FlightTraceBuffer::droppedDecisionCount() const
{
    return droppedDecisions_;
}

uint32_t FlightTraceBuffer::droppedTransitionCount() const
{
    return droppedTransitions_;
}
