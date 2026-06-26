 #include "bmp180_hal.h"

#include <Arduino.h>
#include <math.h>

namespace
{
constexpr uint8_t kRegCalibration = 0xAAU;
constexpr uint8_t kRegChipId = 0xD0U;
constexpr uint8_t kRegControl = 0xF4U;
constexpr uint8_t kRegData = 0xF6U;
constexpr uint8_t kCmdTemperature = 0x2EU;
constexpr uint8_t kCmdPressure = 0x34U;
constexpr uint8_t kExpectedChipId = 0x55U;
constexpr uint8_t kOversampling = NuraConstants::BMP180::kOversampling;
} // namespace

bool BMP180HAL::begin(TwoWire &wire, uint8_t i2cAddress, uint16_t conversionTimeoutMs)
{
    wire_ = &wire;
    address_ = i2cAddress;
    conversionTimeoutMs_ = conversionTimeoutMs;
    initialized_ = false;
    pending_ = Conversion::NONE;
    pressureSamplesSinceTemp_ = 0U;
    b5_ = 0;
    lastTemperatureC_ = 0.0f;

    uint8_t chipId = 0U;
    if (!readU8(kRegChipId, chipId) || chipId != kExpectedChipId)
    {
        return false;
    }
    if (!readCalibration())
    {
        return false;
    }

    initialized_ = startTemperature(millis());
    return initialized_;
}

Bmp180PollResult BMP180HAL::poll(Bmp180Reading &out, uint32_t nowMs)
{
    out = Bmp180Reading{};
    if (!initialized_)
    {
        return Bmp180PollResult::ERROR;
    }

    if (pending_ == Conversion::NONE)
    {
        return startTemperature(nowMs) ? Bmp180PollResult::PENDING : Bmp180PollResult::ERROR;
    }

    if ((nowMs - conversionStartMs_) > conversionTimeoutMs_)
    {
        pending_ = Conversion::NONE;
        return Bmp180PollResult::ERROR;
    }

    if (!conversionReady(nowMs))
    {
        return Bmp180PollResult::PENDING;
    }

    if (pending_ == Conversion::TEMPERATURE)
    {
        if (!readTemperature() || !startPressure(nowMs))
        {
            pending_ = Conversion::NONE;
            return Bmp180PollResult::ERROR;
        }
        return Bmp180PollResult::PENDING;
    }

    if (!readPressure(out, nowMs))
    {
        pending_ = Conversion::NONE;
        return Bmp180PollResult::ERROR;
    }

    const bool refreshTemperature =
        pressureSamplesSinceTemp_ >= NuraConstants::BMP180::kTemperatureRefreshPressureSamples;
    const bool started = refreshTemperature ? startTemperature(nowMs) : startPressure(nowMs);
    if (!started)
    {
        pending_ = Conversion::NONE;
        return Bmp180PollResult::ERROR;
    }

    return Bmp180PollResult::READY;
}

bool BMP180HAL::read(Bmp180Reading &out, uint32_t nowMs)
{
    if (!initialized_)
    {
        return false;
    }

    pending_ = Conversion::NONE;
    if (!startTemperature(nowMs))
    {
        return false;
    }
    delay(NuraConstants::BMP180::kTemperatureConversionDelayMs);
    if (!readTemperature())
    {
        return false;
    }
    if (!startPressure(millis()))
    {
        return false;
    }
    delay(NuraConstants::BMP180::kPressureConversionDelayMs);
    return readPressure(out, millis());
}

bool BMP180HAL::readCalibration()
{
    return readS16(kRegCalibration + 0U, ac1_) &&
           readS16(kRegCalibration + 2U, ac2_) &&
           readS16(kRegCalibration + 4U, ac3_) &&
           readU16(kRegCalibration + 6U, ac4_) &&
           readU16(kRegCalibration + 8U, ac5_) &&
           readU16(kRegCalibration + 10U, ac6_) &&
           readS16(kRegCalibration + 12U, b1_) &&
           readS16(kRegCalibration + 14U, b2_) &&
           readS16(kRegCalibration + 16U, mb_) &&
           readS16(kRegCalibration + 18U, mc_) &&
           readS16(kRegCalibration + 20U, md_);
}

bool BMP180HAL::startTemperature(uint32_t nowMs)
{
    if (!writeU8(kRegControl, kCmdTemperature))
    {
        return false;
    }
    conversionStartMs_ = nowMs;
    pending_ = Conversion::TEMPERATURE;
    return true;
}

bool BMP180HAL::startPressure(uint32_t nowMs)
{
    if (!writeU8(kRegControl, static_cast<uint8_t>(kCmdPressure + (kOversampling << 6U))))
    {
        return false;
    }
    conversionStartMs_ = nowMs;
    pending_ = Conversion::PRESSURE;
    return true;
}

bool BMP180HAL::readTemperature()
{
    int32_t ut = 0;
    if (!readRawTemperature(ut))
    {
        return false;
    }

    const int32_t x1 = ((ut - static_cast<int32_t>(ac6_)) * static_cast<int32_t>(ac5_)) >> 15;
    const int32_t denominator = x1 + static_cast<int32_t>(md_);
    if (denominator == 0)
    {
        return false;
    }
    const int32_t x2 = (static_cast<int32_t>(mc_) << 11) / denominator;
    b5_ = x1 + x2;
    lastTemperatureC_ = static_cast<float>((b5_ + 8) >> 4) / 10.0f;
    pressureSamplesSinceTemp_ = 0U;
    return isfinite(lastTemperatureC_);
}

bool BMP180HAL::readPressure(Bmp180Reading &out, uint32_t nowMs)
{
    int32_t up = 0;
    if (!readRawPressure(up))
    {
        return false;
    }

    const int32_t b6 = b5_ - 4000;
    int32_t x1 = (static_cast<int32_t>(b2_) * ((b6 * b6) >> 12)) >> 11;
    int32_t x2 = (static_cast<int32_t>(ac2_) * b6) >> 11;
    int32_t x3 = x1 + x2;
    const int32_t b3 = (((static_cast<int32_t>(ac1_) * 4 + x3) << kOversampling) + 2) >> 2;

    x1 = (static_cast<int32_t>(ac3_) * b6) >> 13;
    x2 = (static_cast<int32_t>(b1_) * ((b6 * b6) >> 12)) >> 16;
    x3 = ((x1 + x2) + 2) >> 2;
    const uint32_t b4 = (static_cast<uint32_t>(ac4_) * static_cast<uint32_t>(x3 + 32768)) >> 15;
    if (b4 == 0U)
    {
        return false;
    }

    const uint32_t b7 = static_cast<uint32_t>(up - b3) * static_cast<uint32_t>(50000UL >> kOversampling);
    int32_t pressurePa = (b7 < 0x80000000UL) ? static_cast<int32_t>((b7 * 2UL) / b4)
                                             : static_cast<int32_t>((b7 / b4) * 2UL);
    x1 = (pressurePa >> 8) * (pressurePa >> 8);
    x1 = (x1 * 3038) >> 16;
    x2 = (-7357 * pressurePa) >> 16;
    pressurePa += (x1 + x2 + 3791) >> 4;

    const float pressure = static_cast<float>(pressurePa);
    if (!validPressure(pressure) || !isfinite(lastTemperatureC_))
    {
        return false;
    }

    out.pressurePa = pressure;
    out.pressureHpa = pressure / 100.0f;
    out.temperatureC = lastTemperatureC_;
    out.sampleMs = nowMs;
    if (pressureSamplesSinceTemp_ < 255U)
    {
        ++pressureSamplesSinceTemp_;
    }
    return true;
}

bool BMP180HAL::conversionReady(uint32_t nowMs) const
{
    const uint32_t elapsedMs = nowMs - conversionStartMs_;
    if (pending_ == Conversion::TEMPERATURE)
    {
        return elapsedMs >= NuraConstants::BMP180::kTemperatureConversionDelayMs;
    }
    return elapsedMs >= NuraConstants::BMP180::kPressureConversionDelayMs;
}

bool BMP180HAL::readU8(uint8_t reg, uint8_t &value) const
{
    if (wire_ == nullptr)
    {
        return false;
    }
    wire_->beginTransmission(address_);
    wire_->write(reg);
    if (wire_->endTransmission(false) != 0)
    {
        return false;
    }
    if (wire_->requestFrom(static_cast<int>(address_), 1) != 1)
    {
        return false;
    }
    value = wire_->read();
    return true;
}

bool BMP180HAL::readS16(uint8_t reg, int16_t &value) const
{
    uint16_t raw = 0U;
    if (!readU16(reg, raw))
    {
        return false;
    }
    value = static_cast<int16_t>(raw);
    return true;
}

bool BMP180HAL::readU16(uint8_t reg, uint16_t &value) const
{
    if (wire_ == nullptr)
    {
        return false;
    }
    wire_->beginTransmission(address_);
    wire_->write(reg);
    if (wire_->endTransmission(false) != 0)
    {
        return false;
    }
    if (wire_->requestFrom(static_cast<int>(address_), 2) != 2)
    {
        return false;
    }
    value = static_cast<uint16_t>((wire_->read() << 8) | wire_->read());
    return true;
}

bool BMP180HAL::readRawTemperature(int32_t &value) const
{
    uint16_t raw = 0U;
    if (!readU16(kRegData, raw))
    {
        return false;
    }
    value = raw;
    return true;
}

bool BMP180HAL::readRawPressure(int32_t &value) const
{
    if (wire_ == nullptr)
    {
        return false;
    }
    wire_->beginTransmission(address_);
    wire_->write(kRegData);
    if (wire_->endTransmission(false) != 0)
    {
        return false;
    }
    if (wire_->requestFrom(static_cast<int>(address_), 3) != 3)
    {
        return false;
    }
    const int32_t msb = wire_->read();
    const int32_t lsb = wire_->read();
    const int32_t xlsb = wire_->read();
    value = ((msb << 16) + (lsb << 8) + xlsb) >> (8U - kOversampling);
    return true;
}

bool BMP180HAL::writeU8(uint8_t reg, uint8_t value) const
{
    if (wire_ == nullptr)
    {
        return false;
    }
    wire_->beginTransmission(address_);
    wire_->write(reg);
    wire_->write(value);
    return wire_->endTransmission() == 0;
}

bool BMP180HAL::validPressure(float pressurePa)
{
    return isfinite(pressurePa) &&
           pressurePa >= NuraConstants::BMP180::kMinDatasheetPressurePa &&
           pressurePa <= NuraConstants::BMP180::kMaxDatasheetPressurePa;
}
