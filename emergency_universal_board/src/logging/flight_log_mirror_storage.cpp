#include "flight_log_mirror_storage.h"

FlightLogMirrorStorage::FlightLogMirrorStorage(IFlightLogStorage &primary, IFlightLogStorage &mirror)
    : primary_(primary),
      mirror_(mirror)
{
}

bool FlightLogMirrorStorage::begin()
{
    if (healthy())
    {
        return true;
    }

    stopped_ = false;
    primaryActive_ = primary_.begin() && primary_.healthy();
    mirrorActive_ = mirror_.begin() && mirror_.healthy();
    return healthy();
}

bool FlightLogMirrorStorage::append(const uint8_t *data, uint16_t length)
{
    if (stopped_ || data == nullptr || length == 0U)
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

    return healthy();
}

bool FlightLogMirrorStorage::flush()
{
    if (primaryActive_)
    {
        primaryActive_ = primary_.flush() && primary_.healthy();
    }
    if (mirrorActive_)
    {
        mirrorActive_ = mirror_.flush() && mirror_.healthy();
    }
    return healthy();
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
    return !stopped_ && primaryActive_ && mirrorActive_;
}

bool FlightLogMirrorStorage::primaryHealthy() const
{
    return primaryActive_;
}

bool FlightLogMirrorStorage::mirrorHealthy() const
{
    return mirrorActive_;
}
