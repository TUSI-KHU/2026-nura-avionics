#pragma once

#include <stdint.h>

#include "app/app_config.h"
#include "core/logger/logger.h"
#include "core/tasks.h"
#include "hal/bmp390_hal.h"
#include "nura_constants.h"
#include "state/telemetry_state.h"

class BarometerTask : public Task
{
public:
    BarometerTask(BMP390HAL &barometer, TelemetryState &telemetryState, Logger &logger, const IAppConfig &config);

    const char *name() const override;
    bool init() override;
    bool tick(uint32_t nowMs) override;
    uint32_t periodMs() const override;

private:
    bool initialize(uint32_t nowMs);
    void clearReading(uint32_t nowMs);
    void resetHealth();
    void recordReadFailure(uint32_t nowMs);
    bool sampleAltitudeValid(float altitudeM) const;
    void recordBadValue(uint32_t nowMs, uint16_t faultFlag);
    void publishValidSample(const Bmp390Reading &sample, float rawAltitudeM);
    void markFault(uint32_t nowMs, uint16_t faultFlag);
    float filterAltitude(float rawAltitudeM);

    BMP390HAL &barometer_;
    TelemetryState &telemetryState_;
    Logger &logger_;
    const IAppConfig &config_;
    bool initialized_ = false;
    uint32_t lastInitAttemptMs_ = 0;
    uint32_t lastValidSampleMs_ = 0;
    uint8_t consecutiveReadFailCount_ = 0U;
    uint8_t consecutiveBadValueCount_ = 0U;
    uint8_t totalBadValueCount_ = 0U;
    float altitudeWindowM_[NuraConstants::Sensors::kBarometerMedianWindowSamples] = {0.0f, 0.0f, 0.0f};
    uint8_t altitudeWindowHead_ = 0U;
    uint8_t altitudeWindowCount_ = 0U;
    float filteredAltitudeM_ = 0.0f;
    bool filterReady_ = false;
};
