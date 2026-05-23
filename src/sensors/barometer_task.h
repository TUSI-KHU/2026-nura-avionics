#pragma once

#include <stdint.h>

#include "app/app_config.h"
#include "core/logger/logger.h"
#include "core/tasks.h"
#include "hal/mpl3115a2_hal.h"
#include "state/telemetry_state.h"

class BarometerTask : public Task
{
public:
    BarometerTask(MPL3115A2HAL &barometer, TelemetryState &telemetryState, Logger &logger, const IAppConfig &config);

    const char *name() const override;
    bool init() override;
    bool tick(uint32_t nowMs) override;
    uint32_t periodMs() const override;

private:
    bool initialize(uint32_t nowMs);
    void clearReading(uint32_t nowMs);
    float filterAltitude(float rawAltitudeM);

    MPL3115A2HAL &barometer_;
    TelemetryState &telemetryState_;
    Logger &logger_;
    const IAppConfig &config_;
    bool initialized_ = false;
    uint32_t lastInitAttemptMs_ = 0;
    float altitudeWindowM_[3] = {0.0f, 0.0f, 0.0f};
    uint8_t altitudeWindowHead_ = 0U;
    uint8_t altitudeWindowCount_ = 0U;
    float filteredAltitudeM_ = 0.0f;
    bool filterReady_ = false;
};
