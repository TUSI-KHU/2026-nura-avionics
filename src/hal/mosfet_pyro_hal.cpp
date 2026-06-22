#include "hal/mosfet_pyro_hal.h"

MosfetPyroHAL::MosfetPyroHAL(const MosfetPyroConfig &config)
    : config_(config) {}

bool MosfetPyroHAL::begin()
{
    initialized_ = false;
    faulted_ = false;

#if defined(NURA_ENABLE_PYRO_OUTPUTS)
    outputsEnabled_ = true;
#else
    outputsEnabled_ = false;
#endif

#if defined(NURA_ENABLE_PYRO_OUTPUTS) && !defined(NURA_ALLOW_PYRO_POWER_SENSE_PIN_CONFLICT)
    if (knownUnsafePinConflict())
    {
        faulted_ = true;
        return false;
    }
#endif

    if (outputsEnabled_)
    {
        configureChannel(config_.drogue);
        configureChannel(config_.main);
        allOff();
    }

    initialized_ = true;
    return true;
}

void MosfetPyroHAL::allOff()
{
    if (!outputsEnabled_)
    {
        return;
    }

    writeChannel(config_.drogue, false);
    writeChannel(config_.main, false);
}

void MosfetPyroHAL::setDrogue(bool enabled)
{
    if (!initialized_ || !outputsEnabled_)
    {
        return;
    }

    writeChannel(config_.drogue, enabled);
}

void MosfetPyroHAL::setMain(bool enabled)
{
    if (!initialized_ || !outputsEnabled_)
    {
        return;
    }

    writeChannel(config_.main, enabled);
}

void MosfetPyroHAL::configureChannel(const MosfetPyroChannelPins &pins) const
{
    pinMode(pins.gpio1Pin, OUTPUT);
    pinMode(pins.gpio2Pin, OUTPUT);
    digitalWrite(pins.gpio1Pin, config_.activeHigh ? LOW : HIGH);
    digitalWrite(pins.gpio2Pin, config_.activeHigh ? LOW : HIGH);
    pinMode(pins.sensePin, INPUT);
}

void MosfetPyroHAL::writeChannel(const MosfetPyroChannelPins &pins, bool enabled) const
{
    const uint8_t activeLevel = config_.activeHigh ? HIGH : LOW;
    const uint8_t inactiveLevel = config_.activeHigh ? LOW : HIGH;
    const uint8_t level = enabled ? activeLevel : inactiveLevel;
    digitalWrite(pins.gpio1Pin, level);
    digitalWrite(pins.gpio2Pin, level);
}

bool MosfetPyroHAL::knownUnsafePinConflict() const
{
    const uint8_t voltagePin = BoardPinMap::PowerSense::voltagePin;
    return config_.drogue.gpio1Pin == voltagePin ||
           config_.drogue.gpio2Pin == voltagePin ||
           config_.main.gpio1Pin == voltagePin ||
           config_.main.gpio2Pin == voltagePin;
}
