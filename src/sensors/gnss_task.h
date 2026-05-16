#pragma once

#include <stdint.h>

#include "core/tasks.h"
#include "hal/ublox_m6_gnss_hal.h"
#include "state/gps_state.h"

class GNSSTask : public Task
{
public:
    GNSSTask(UbloxM6GNSSHAL &gnss, GpsState &gpsState);

    const char *name() const override;
    bool init() override;
    bool tick(uint32_t nowMs) override;
    uint32_t periodMs() const override;

private:
    void updateState(const UbloxM6GnssReading &sample);

    UbloxM6GNSSHAL &gnss_;
    GpsState &gpsState_;
};
