#pragma once

#include <stdint.h>

#include "app/app_config.h"
#include "core/logger/logger.h"
#include "core/tasks.h"
#include "hal/mock_flight_data_hal.h"
#include "state/gps_state.h"
#include "state/imu_state.h"
#include "state/telemetry_state.h"

class MockTelemetrySourceTask : public Task
{
public:
    MockTelemetrySourceTask(MockFlightDataHAL &mockData,
                            ImuState &imuState,
                            GpsState &gpsState,
                            TelemetryState &telemetryState,
                            Logger &logger,
                            const IAppConfig &config);

    const char *name() const override;
    bool init() override;
    bool tick(uint32_t nowMs) override;
    uint32_t periodMs() const override;

private:
    MockFlightDataHAL &mockData_;
    ImuState &imuState_;
    GpsState &gpsState_;
    TelemetryState &telemetryState_;
    Logger &logger_;
    const IAppConfig &config_;
};
