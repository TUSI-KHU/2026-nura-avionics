#pragma once

#include <stdint.h>

#include "app/app_config.h"
#include "core/tasks.h"
#include "hal/ublox_m6_gnss_hal.h"
#include "state/gps_state.h"

class GNSSTask : public Task
{
public:
    GNSSTask(IGnss &gnss,
             GpsState &gpsState,
             const IAppConfig &config,
             HardwareSerial &serial,
             uint32_t baudRate);

    const char *name() const override;
    bool init() override;
    bool tick(uint32_t nowMs) override;
    uint32_t periodMs() const override;

private:
    void updateState(const UbloxM6GnssReading &sample);

    IGnss &gnss_;
    GpsState &gpsState_;
    const IAppConfig &config_;
    HardwareSerial &serial_;
    uint32_t baudRate_;
};
