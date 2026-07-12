#include "imu_task.h"

#include <Arduino.h>
#include <math.h>

#include "nura_constants.h"

namespace
{
    constexpr float kTwo = 2.0f;

    float clampUnit(float value)
    {
        if (value > 1.0f)
        {
            return 1.0f;
        }
        if (value < -1.0f)
        {
            return -1.0f;
        }
        return value;
    }

    bool finiteQuaternion(float w, float x, float y, float z)
    {
        return isfinite(w) && isfinite(x) && isfinite(y) && isfinite(z);
    }
}

IMUTask::IMUTask(LSM6DSO32HAL &imu, ImuState &imuState, Logger &logger, const IAppConfig &config)
    : RecoverableTask(TaskCriticality::CRITICAL,
                      config.imuReadFailureThreshold(),
                      config.imuMaxRecoveryAttempts(),
                      config.imuRecoveryIntervalMs()),
      imu_(imu),
      imuState_(imuState),
      logger_(logger),
      config_(config) {}

const char *IMUTask::name() const
{
    return "imu";
}

bool IMUTask::init()
{
    // 초기화 전 IMU 컨텍스트를 안전한 기본값으로 리셋한다.
    imuState_.data.accelXMps2 = 0.0f;
    imuState_.data.accelYMps2 = 0.0f;
    imuState_.data.accelZMps2 = 0.0f;
    imuState_.data.gyroXDps = 0.0f;
    imuState_.data.gyroYDps = 0.0f;
    imuState_.data.gyroZDps = 0.0f;
    imuState_.data.attitudeValid = false;
    imuState_.data.rollDeg = 0.0f;
    imuState_.data.pitchDeg = 0.0f;
    imuState_.data.yawDeg = 0.0f;
    imuState_.data.tiltValid = false;
    imuState_.data.tiltAngleDeg = 0.0f;
    imuState_.data.lastUpdatedMs = 0U;
    resetAttitudeEstimate();

    if (!initializeDevice(0U))
    {
        LOGE(logger_, 0U, "imu", "lsm6dso32 spi init failed");
        return false;
    }

    markInitialized();
    LOGI(logger_, 0U, "imu", "lsm6dso32 spi initialized");

    return true;
}

bool IMUTask::tick(uint32_t nowMs)
{
    // 센서 태스크는 읽기 성공/실패 관측만 기록하고 health 전이는 watchdog에 맡긴다.
    Lsm6dso32Reading sample;
    const bool readOk = imu_.read(sample, nowMs);

    if (readOk)
    {
        imuState_.data.accelXMps2 = sample.accelXMps2;
        imuState_.data.accelYMps2 = sample.accelYMps2;
        imuState_.data.accelZMps2 = sample.accelZMps2;
        imuState_.data.gyroXDps = sample.gyroXDps;
        imuState_.data.gyroYDps = sample.gyroYDps;
        imuState_.data.gyroZDps = sample.gyroZDps;
        updateAttitudeEstimate(sample);
        imuState_.data.lastUpdatedMs = sample.sampleMs;
        logSample(nowMs);

        markReadSuccess();
    }
    else
    {
        markReadFailure();
    }

    return true;
}

uint32_t IMUTask::periodMs() const
{
    return config_.imuTaskPeriodMs();
}

bool IMUTask::recover(uint32_t nowMs)
{
    (void)nowMs;
    return imu_.begin(config_.imuCsPin());
}

bool IMUTask::initializeDevice(uint32_t logTs)
{
    (void)logTs;

    for (uint8_t attempt = 0U; attempt < NuraConstants::Sensors::kSensorInitRetryAttempts; ++attempt)
    {
        if (imu_.begin(config_.imuCsPin()))
        {
            return true;
        }
        if ((attempt + 1U) < NuraConstants::Sensors::kSensorInitRetryAttempts)
        {
            delay(NuraConstants::Sensors::kSensorInitRetryDelayMs);
        }
    }

    return false;
}

void IMUTask::resetAttitudeEstimate()
{
    attitudeReferenceX_ = 0.0f;
    attitudeReferenceY_ = 0.0f;
    attitudeReferenceZ_ = 1.0f;
    qW_ = 1.0f;
    qX_ = 0.0f;
    qY_ = 0.0f;
    qZ_ = 0.0f;
    lastAttitudeSampleMs_ = 0U;
    attitudeReferenceValid_ = false;
    lastSampleLogMs_ = 0U;
}

void IMUTask::updateAttitudeEstimate(const Lsm6dso32Reading &sample)
{
    const float normMps2 = sqrtf((sample.accelXMps2 * sample.accelXMps2) +
                                 (sample.accelYMps2 * sample.accelYMps2) +
                                 (sample.accelZMps2 * sample.accelZMps2));
    const float normG = normMps2 / NuraConstants::Physics::kGravityMps2;
    if (!isfinite(normG) ||
        normG < NuraConstants::Sensors::kTiltMinAccelNormG ||
        normG > NuraConstants::Sensors::kTiltMaxAccelNormG)
    {
        imuState_.data.attitudeValid = false;
        imuState_.data.tiltValid = false;
        return;
    }

    const float unitX = sample.accelXMps2 / normMps2;
    const float unitY = sample.accelYMps2 / normMps2;
    const float unitZ = sample.accelZMps2 / normMps2;
    if (!attitudeReferenceValid_)
    {
        initializeAttitudeReference(unitX, unitY, unitZ, sample.sampleMs);
        publishAttitude();
        return;
    }

    if (sample.sampleMs <= lastAttitudeSampleMs_)
    {
        publishAttitude();
        return;
    }

    const uint32_t dtMs = sample.sampleMs - lastAttitudeSampleMs_;
    lastAttitudeSampleMs_ = sample.sampleMs;
    if (dtMs > NuraConstants::Sensors::kAttitudeMaxDeltaMs)
    {
        imuState_.data.attitudeValid = false;
        imuState_.data.tiltValid = false;
        return;
    }

    const float dtS = static_cast<float>(dtMs) * 0.001f;
    integrateGyro(sample, dtS);
    applyAccelCorrection(unitX, unitY, unitZ, normG, dtS);
    publishAttitude();
}

void IMUTask::initializeAttitudeReference(float unitX, float unitY, float unitZ, uint32_t sampleMs)
{
    attitudeReferenceX_ = unitX;
    attitudeReferenceY_ = unitY;
    attitudeReferenceZ_ = unitZ;
    qW_ = 1.0f;
    qX_ = 0.0f;
    qY_ = 0.0f;
    qZ_ = 0.0f;
    lastAttitudeSampleMs_ = sampleMs;
    attitudeReferenceValid_ = true;
}

void IMUTask::integrateGyro(const Lsm6dso32Reading &sample, float dtS)
{
    const float gx = sample.gyroXDps * NuraConstants::Physics::kDegToRad;
    const float gy = sample.gyroYDps * NuraConstants::Physics::kDegToRad;
    const float gz = sample.gyroZDps * NuraConstants::Physics::kDegToRad;

    const float halfDt = 0.5f * dtS;
    const float nextW = qW_ + ((-qX_ * gx - qY_ * gy - qZ_ * gz) * halfDt);
    const float nextX = qX_ + ((qW_ * gx + qY_ * gz - qZ_ * gy) * halfDt);
    const float nextY = qY_ + ((qW_ * gy - qX_ * gz + qZ_ * gx) * halfDt);
    const float nextZ = qZ_ + ((qW_ * gz + qX_ * gy - qY_ * gx) * halfDt);

    qW_ = nextW;
    qX_ = nextX;
    qY_ = nextY;
    qZ_ = nextZ;
    normalizeQuaternion();
}

void IMUTask::applyAccelCorrection(float unitX, float unitY, float unitZ, float normG, float dtS)
{
    if (normG < NuraConstants::Sensors::kAttitudeAccelCorrectionMinNormG ||
        normG > NuraConstants::Sensors::kAttitudeAccelCorrectionMaxNormG)
    {
        return;
    }

    float measuredWorldX = 0.0f;
    float measuredWorldY = 0.0f;
    float measuredWorldZ = 0.0f;
    rotateBodyToWorld(unitX, unitY, unitZ, measuredWorldX, measuredWorldY, measuredWorldZ);

    const float errorX = (measuredWorldY * attitudeReferenceZ_) - (measuredWorldZ * attitudeReferenceY_);
    const float errorY = (measuredWorldZ * attitudeReferenceX_) - (measuredWorldX * attitudeReferenceZ_);
    const float errorZ = (measuredWorldX * attitudeReferenceY_) - (measuredWorldY * attitudeReferenceX_);
    const float gainDt = NuraConstants::Sensors::kAttitudeAccelCorrectionGain * dtS;
    const float halfX = 0.5f * errorX * gainDt;
    const float halfY = 0.5f * errorY * gainDt;
    const float halfZ = 0.5f * errorZ * gainDt;

    const float deltaW = 1.0f;
    const float deltaX = halfX;
    const float deltaY = halfY;
    const float deltaZ = halfZ;

    const float nextW = (deltaW * qW_) - (deltaX * qX_) - (deltaY * qY_) - (deltaZ * qZ_);
    const float nextX = (deltaW * qX_) + (deltaX * qW_) + (deltaY * qZ_) - (deltaZ * qY_);
    const float nextY = (deltaW * qY_) - (deltaX * qZ_) + (deltaY * qW_) + (deltaZ * qX_);
    const float nextZ = (deltaW * qZ_) + (deltaX * qY_) - (deltaY * qX_) + (deltaZ * qW_);

    qW_ = nextW;
    qX_ = nextX;
    qY_ = nextY;
    qZ_ = nextZ;
    normalizeQuaternion();
}

void IMUTask::publishAttitude()
{
    if (!attitudeReferenceValid_ || !finiteQuaternion(qW_, qX_, qY_, qZ_))
    {
        imuState_.data.attitudeValid = false;
        imuState_.data.tiltValid = false;
        return;
    }

    const float sinRollCosPitch = kTwo * ((qW_ * qX_) + (qY_ * qZ_));
    const float cosRollCosPitch = 1.0f - (kTwo * ((qX_ * qX_) + (qY_ * qY_)));
    const float sinPitch = clampUnit(kTwo * ((qW_ * qY_) - (qZ_ * qX_)));
    const float sinYawCosPitch = kTwo * ((qW_ * qZ_) + (qX_ * qY_));
    const float cosYawCosPitch = 1.0f - (kTwo * ((qY_ * qY_) + (qZ_ * qZ_)));

    imuState_.data.rollDeg = atan2f(sinRollCosPitch, cosRollCosPitch) * NuraConstants::Physics::kRadToDeg;
    imuState_.data.pitchDeg = asinf(sinPitch) * NuraConstants::Physics::kRadToDeg;
    imuState_.data.yawDeg = atan2f(sinYawCosPitch, cosYawCosPitch) * NuraConstants::Physics::kRadToDeg;

    float currentAxisWorldX = 0.0f;
    float currentAxisWorldY = 0.0f;
    float currentAxisWorldZ = 0.0f;
    rotateBodyToWorld(attitudeReferenceX_, attitudeReferenceY_, attitudeReferenceZ_,
                      currentAxisWorldX, currentAxisWorldY, currentAxisWorldZ);
    const float dot = clampUnit((currentAxisWorldX * attitudeReferenceX_) +
                                (currentAxisWorldY * attitudeReferenceY_) +
                                (currentAxisWorldZ * attitudeReferenceZ_));
    imuState_.data.tiltAngleDeg = acosf(dot) * NuraConstants::Physics::kRadToDeg;

    const bool attitudeFinite = isfinite(imuState_.data.rollDeg) &&
                                isfinite(imuState_.data.pitchDeg) &&
                                isfinite(imuState_.data.yawDeg) &&
                                isfinite(imuState_.data.tiltAngleDeg);
    imuState_.data.attitudeValid = attitudeFinite;
    imuState_.data.tiltValid = attitudeFinite;
}

void IMUTask::normalizeQuaternion()
{
    const float norm = sqrtf((qW_ * qW_) + (qX_ * qX_) + (qY_ * qY_) + (qZ_ * qZ_));
    if (!isfinite(norm) || norm <= 0.0f)
    {
        resetAttitudeEstimate();
        imuState_.data.attitudeValid = false;
        imuState_.data.tiltValid = false;
        return;
    }

    qW_ /= norm;
    qX_ /= norm;
    qY_ /= norm;
    qZ_ /= norm;
}

void IMUTask::rotateBodyToWorld(float x, float y, float z, float &outX, float &outY, float &outZ) const
{
    const float tx = kTwo * ((qY_ * z) - (qZ_ * y));
    const float ty = kTwo * ((qZ_ * x) - (qX_ * z));
    const float tz = kTwo * ((qX_ * y) - (qY_ * x));

    outX = x + (qW_ * tx) + ((qY_ * tz) - (qZ_ * ty));
    outY = y + (qW_ * ty) + ((qZ_ * tx) - (qX_ * tz));
    outZ = z + (qW_ * tz) + ((qX_ * ty) - (qY_ * tx));
}

void IMUTask::logSample(uint32_t nowMs)
{
    if ((nowMs - lastSampleLogMs_) < NuraConstants::Sensors::kLowGSampleLogIntervalMs)
    {
        return;
    }

    lastSampleLogMs_ = nowMs;
    if (!Serial)
    {
        return;
    }

    const float accelXG = imuState_.data.accelXMps2 / NuraConstants::Physics::kGravityMps2;
    const float accelYG = imuState_.data.accelYMps2 / NuraConstants::Physics::kGravityMps2;
    const float accelZG = imuState_.data.accelZMps2 / NuraConstants::Physics::kGravityMps2;
    const float normG = sqrtf((accelXG * accelXG) + (accelYG * accelYG) + (accelZG * accelZG));

    Serial.print("[");
    Serial.print(nowMs);
    Serial.print("] low_g_imu g=");
    Serial.print(accelXG, 3);
    Serial.print(",");
    Serial.print(accelYG, 3);
    Serial.print(",");
    Serial.print(accelZG, 3);
    Serial.print(" norm=");
    Serial.print(normG, 3);
    Serial.print(" gyro_dps=");
    Serial.print(imuState_.data.gyroXDps, 3);
    Serial.print(",");
    Serial.print(imuState_.data.gyroYDps, 3);
    Serial.print(",");
    Serial.print(imuState_.data.gyroZDps, 3);
    Serial.print(" rpy=");
    Serial.print(imuState_.data.rollDeg, 2);
    Serial.print(",");
    Serial.print(imuState_.data.pitchDeg, 2);
    Serial.print(",");
    Serial.print(imuState_.data.yawDeg, 2);
    Serial.print(" tilt=");
    Serial.print(imuState_.data.tiltAngleDeg, 2);
    Serial.print(" attitude=");
    Serial.print(imuState_.data.attitudeValid ? "true" : "false");
    Serial.print(" tilt_valid=");
    Serial.println(imuState_.data.tiltValid ? "true" : "false");
}
