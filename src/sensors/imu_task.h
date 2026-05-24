#pragma once

#include <stdint.h>

#include "app/app_config.h"
#include "core/logger/logger.h"
#include "core/recoverable_task/recoverable_task.h"
#include "state/imu_state.h"
#include "hal/lsm6dso32_hal.h"

class IMUTask : public RecoverableTask
{
public:
    // SPI low-g IMU를 주기적으로 읽고 recoverable 정책을 적용하는 센서 태스크다.
    IMUTask(LSM6DSO32HAL &imu, ImuState &imuState, Logger &logger, const IAppConfig &config);

    const char *name() const override;
    bool init() override;
    bool tick(uint32_t nowMs) override;
    uint32_t periodMs() const override;

    bool recover(uint32_t nowMs) override;

private:
    bool initializeDevice(uint32_t logTs);
    void resetAttitudeEstimate();
    void updateAttitudeEstimate(const Lsm6dso32Reading &sample);
    void initializeAttitudeReference(float unitX, float unitY, float unitZ, uint32_t sampleMs);
    void integrateGyro(const Lsm6dso32Reading &sample, float dtS);
    void applyAccelCorrection(float unitX, float unitY, float unitZ, float normG, float dtS);
    void publishAttitude();
    void normalizeQuaternion();
    void rotateBodyToWorld(float x, float y, float z, float &outX, float &outY, float &outZ) const;
    LSM6DSO32HAL &imu_;
    ImuState &imuState_;
    Logger &logger_;
    const IAppConfig &config_;
    float attitudeReferenceX_ = 0.0f;
    float attitudeReferenceY_ = 0.0f;
    float attitudeReferenceZ_ = 1.0f;
    float qW_ = 1.0f;
    float qX_ = 0.0f;
    float qY_ = 0.0f;
    float qZ_ = 0.0f;
    uint32_t lastAttitudeSampleMs_ = 0U;
    bool attitudeReferenceValid_ = false;
};
