#pragma once

#include <stdint.h>

#include <Arduino.h>

#include "board_pinmap.h"
#include "hal/pyro_output.h"

struct MosfetPyroChannelPins
{
    uint8_t gpio1Pin;
    uint8_t gpio2Pin;
    uint8_t sensePin;
};

struct MosfetPyroConfig
{
    MosfetPyroChannelPins drogue{
        BoardPinMap::DroguePyro::gpio1Pin,
        BoardPinMap::DroguePyro::gpio2Pin,
        BoardPinMap::DroguePyro::sensePin};
    MosfetPyroChannelPins main{
        BoardPinMap::MainPyro::gpio1Pin,
        BoardPinMap::MainPyro::gpio2Pin,
        BoardPinMap::MainPyro::sensePin};
    bool activeHigh = true;
};

class MosfetPyroHAL final : public IPyroOutput
{
public:
    explicit MosfetPyroHAL(const MosfetPyroConfig &config = MosfetPyroConfig{});

    bool begin() override;
    void allOff() override;
    void setDrogue(bool enabled) override;
    void setMain(bool enabled) override;

    bool initialized() const { return initialized_; }
    bool outputsEnabled() const { return outputsEnabled_; }
    bool faulted() const { return faulted_; }

private:
    void configureChannel(const MosfetPyroChannelPins &pins) const;
    void writeChannel(const MosfetPyroChannelPins &pins, bool enabled) const;
    bool knownUnsafePinConflict() const;

    MosfetPyroConfig config_;
    bool initialized_ = false;
    bool outputsEnabled_ = false;
    bool faulted_ = false;
};
