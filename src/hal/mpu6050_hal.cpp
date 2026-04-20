#include "mpu6050_hal.h"

// Adafruit MPU6050 library does not return failure when I2C connection fails
// Reimplemented the read and readBurst function to return false when I2C reading fails

namespace
{
    constexpr float kGravity = 9.80665f;
    constexpr float kTempOffsetC = 36.53f;
    constexpr float kTempScale = 340.0f;

    constexpr uint8_t kRegAccelXoutH = 0x3B;
    constexpr uint8_t kBurstLength = 14;

    int16_t makeInt16(uint8_t hi, uint8_t lo)
    {
        return static_cast<int16_t>(
            (static_cast<uint16_t>(hi) << 8) |
            static_cast<uint16_t>(lo));
    }
}

bool MPU6050HAL::begin(uint8_t i2cAddress,
                       TwoWire &wire,
                       mpu6050_accel_range_t accelRange,
                       mpu6050_gyro_range_t gyroRange)
{
    // begin 성공 후에는 read가 사용할 I2C 주소와 스케일 값을 모두 준비한다.
    i2cAddress_ = i2cAddress;
    wire_ = &wire;
    accelRange_ = accelRange;
    gyroRange_ = gyroRange;

    if (!sensor_.begin(i2cAddress_, wire_))
    {
        return false;
    }

    if (accelRange_ == MPU6050_RANGE_16_G)
        accelScale_ = 2048.0f;
    if (accelRange_ == MPU6050_RANGE_8_G)
        accelScale_ = 4096.0f;
    if (accelRange_ == MPU6050_RANGE_4_G)
        accelScale_ = 8192.0f;
    if (accelRange_ == MPU6050_RANGE_2_G)
        accelScale_ = 16384.0f;

    if (gyroRange_ == MPU6050_RANGE_250_DEG)
        gyroScale_ = 131.0f;
    if (gyroRange_ == MPU6050_RANGE_500_DEG)
        gyroScale_ = 65.5f;
    if (gyroRange_ == MPU6050_RANGE_1000_DEG)
        gyroScale_ = 32.8f;
    if (gyroRange_ == MPU6050_RANGE_2000_DEG)
        gyroScale_ = 16.4f;

    sensor_.setAccelerometerRange(accelRange_);
    sensor_.setGyroRange(gyroRange_);
    sensor_.setFilterBandwidth(MPU6050_BAND_21_HZ);

    return true;
}

bool MPU6050HAL::read(Mpu6050Reading &out, uint32_t nowMs)
{
    // 필요한 14바이트를 한 번에 읽어 부분 읽기로 인한 불일치를 줄인다.
    if (wire_ == nullptr)
    {
        return false;
    }

    uint8_t buffer[kBurstLength];
    if (!readBurst(kRegAccelXoutH, buffer, sizeof(buffer)))
    {
        return false;
    }

    const int16_t rawAccX = makeInt16(buffer[0], buffer[1]);
    const int16_t rawAccY = makeInt16(buffer[2], buffer[3]);
    const int16_t rawAccZ = makeInt16(buffer[4], buffer[5]);
    const int16_t rawTemp = makeInt16(buffer[6], buffer[7]);
    const int16_t rawGyroX = makeInt16(buffer[8], buffer[9]);
    const int16_t rawGyroY = makeInt16(buffer[10], buffer[11]);
    const int16_t rawGyroZ = makeInt16(buffer[12], buffer[13]);

    out.accelXMps2 = (static_cast<float>(rawAccX) / accelScale_) * kGravity;
    out.accelYMps2 = (static_cast<float>(rawAccY) / accelScale_) * kGravity;
    out.accelZMps2 = (static_cast<float>(rawAccZ) / accelScale_) * kGravity;

    out.gyroXDps = static_cast<float>(rawGyroX) / gyroScale_;
    out.gyroYDps = static_cast<float>(rawGyroY) / gyroScale_;
    out.gyroZDps = static_cast<float>(rawGyroZ) / gyroScale_;

    out.temperatureC = rawTempToC(rawTemp);
    out.sampleMs = nowMs;

    return true;
}

bool MPU6050HAL::readBurst(uint8_t startReg, uint8_t *buffer, size_t length)
{
    // 주소 전송과 데이터 읽기 단계별 실패를 직접 확인해 false를 반환한다.
    if ((wire_ == nullptr) || (buffer == nullptr) || (length == 0U))
    {
        return false;
    }

    wire_->beginTransmission(i2cAddress_);
    wire_->write(startReg);

    const uint8_t txErr = wire_->endTransmission(false);
    if (txErr != 0U)
    {
        return false;
    }

    const size_t received = wire_->requestFrom(
        static_cast<int>(i2cAddress_),
        static_cast<int>(length),
        static_cast<int>(true));

    if (received != length)
    {
        while (wire_->available())
        {
            wire_->read();
        }
        return false;
    }

    for (size_t i = 0; i < length; ++i)
    {
        if (!wire_->available())
        {
            return false;
        }
        buffer[i] = static_cast<uint8_t>(wire_->read());
    }

    return true;
}

float MPU6050HAL::rawTempToC(int16_t raw)
{
    return (static_cast<float>(raw) / kTempScale) + kTempOffsetC;
}
