#include "flight_log_mirror_storage.h"

FlightLogMirrorStorage::FlightLogMirrorStorage(IFlightLogStorage &primary, IFlightLogStorage &mirror)
    : primary_(primary),
      mirror_(mirror)
{
}

bool FlightLogMirrorStorage::begin()
{
    if (fullyHealthy())
    {
        return true;
    }

    stopped_ = false;
    primaryActive_ = primary_.begin() && primary_.healthy();
    mirrorActive_ = mirror_.begin() && mirror_.healthy();
    return fullyHealthy();
}

bool FlightLogMirrorStorage::canAppend(uint16_t length) const
{
    if (stopped_ || length == 0U || (!primaryActive_ && !mirrorActive_))
    {
        return false;
    }
    return (!primaryActive_ || primary_.canAppend(length)) &&
           (!mirrorActive_ || mirror_.canAppend(length));
}

bool FlightLogMirrorStorage::append(const uint8_t *data, uint16_t length)
{
    if (stopped_ || data == nullptr || !canAppend(length))
    {
        return false;
    }

    if (primaryActive_)
    {
        primaryActive_ = primary_.append(data, length) && primary_.healthy();
    }

    if (mirrorActive_)
    {
        mirrorActive_ = mirror_.append(data, length) && mirror_.healthy();
    }

    return primaryActive_ || mirrorActive_;
}

bool FlightLogMirrorStorage::service(uint32_t nowMs)
{
    if (primaryActive_)
    {
        primaryActive_ = primary_.service(nowMs) && primary_.healthy();
    }
    if (mirrorActive_)
    {
        mirrorActive_ = mirror_.service(nowMs) && mirror_.healthy();
    }
    return healthy();
}

bool FlightLogMirrorStorage::requestFlush()
{
    if (primaryActive_)
    {
        primaryActive_ = primary_.requestFlush() && primary_.healthy();
    }
    if (mirrorActive_)
    {
        mirrorActive_ = mirror_.requestFlush() && mirror_.healthy();
    }
    return healthy();
}

bool FlightLogMirrorStorage::idle() const
{
    return (!primaryActive_ || primary_.idle()) &&
           (!mirrorActive_ || mirror_.idle());
}

void FlightLogMirrorStorage::stop()
{
    primary_.stop();
    mirror_.stop();
    primaryActive_ = false;
    mirrorActive_ = false;
    stopped_ = true;
}

bool FlightLogMirrorStorage::healthy() const
{
    return !stopped_ && (primaryActive_ || mirrorActive_);
}

bool FlightLogMirrorStorage::primaryHealthy() const
{
    return primaryActive_;
}

bool FlightLogMirrorStorage::mirrorHealthy() const
{
    return mirrorActive_;
}

bool FlightLogMirrorStorage::fullyHealthy() const
{
    return !stopped_ && primaryActive_ && mirrorActive_;
}
