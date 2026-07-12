#include "h3lis331dl_hal.h"

#include <Arduino.h>
#include <math.h>

#include "nura_constants.h"

bool H3LIS331DLHAL::begin(uint8_t csPin,
                          SPIClass &spi,
                          H3LIS331DLRange range)
{
    initialized_ = false;
    whoAmI_ = 0U;

    pinMode(csPin, OUTPUT);
    digitalWrite(csPin, HIGH);
    spi.begin();

    if (!sensor_.begin_SPI(csPin, &spi))
    {
        return false;
    }

    whoAmI_ = sensor_.getDeviceID();
    if (whoAmI_ != NuraConstants::H3LIS331DL::kExpectedWhoAmI)
    {
        return false;
    }

    sensor_.setRange(toAdafruitRange(range));
    sensor_.setDataRate(LIS331_DATARATE_1000_HZ);

    initialized_ = true;
    return true;
}

bool H3LIS331DLHAL::read(H3LIS331DLReading &out, uint32_t nowMs)
{
    if (!initialized_)
    {
        return false;
    }

    sensors_event_t event;
    if (!sensor_.getEvent(&event) || !validEvent(event))
    {
        return false;
    }

    out.rawX = sensor_.x;
    out.rawY = sensor_.y;
    out.rawZ = sensor_.z;
    out.accelXMps2 = event.acceleration.x;
    out.accelYMps2 = event.acceleration.y;
    out.accelZMps2 = event.acceleration.z;
    out.accelXG = out.accelXMps2 / NuraConstants::Physics::kGravityMps2;
    out.accelYG = out.accelYMps2 / NuraConstants::Physics::kGravityMps2;
    out.accelZG = out.accelZMps2 / NuraConstants::Physics::kGravityMps2;
    out.whoAmI = whoAmI_;
    out.sampleMs = nowMs;

    return true;
}

uint8_t H3LIS331DLHAL::readWhoAmI()
{
    return initialized_ ? whoAmI_ : 0U;
}

bool H3LIS331DLHAL::validEvent(const sensors_event_t &event)
{
    return isfinite(event.acceleration.x) &&
           isfinite(event.acceleration.y) &&
           isfinite(event.acceleration.z);
}

h3lis331dl_range_t H3LIS331DLHAL::toAdafruitRange(H3LIS331DLRange range)
{
    switch (range)
    {
    case H3LIS331DLRange::RANGE_200G:
        return H3LIS331_RANGE_200_G;
    case H3LIS331DLRange::RANGE_400G:
        return H3LIS331_RANGE_400_G;
    case H3LIS331DLRange::RANGE_100G:
    default:
        return H3LIS331_RANGE_100_G;
    }
}
