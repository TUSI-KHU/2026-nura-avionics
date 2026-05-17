#include "h3lis331dl_hal.h"

namespace
{
    constexpr uint8_t kRegWhoAmI = 0x0FU;
    constexpr uint8_t kRegCtrl1 = 0x20U;
    constexpr uint8_t kRegCtrl4 = 0x23U;
    constexpr uint8_t kRegOutXL = 0x28U;

    constexpr uint8_t kSpiRead = 0x80U;
    constexpr uint8_t kSpiAutoIncrement = 0x40U;
    constexpr uint8_t kExpectedWhoAmI = 0x32U;

    constexpr uint8_t kCtrlReg1EnableXyz50Hz = 0x27U;
    constexpr uint8_t kCtrlReg4BlockDataUpdate = 0x80U;
    constexpr uint8_t kCtrlReg4Range100G = 0x00U;
    constexpr uint8_t kCtrlReg4Range200G = 0x10U;
    constexpr uint8_t kCtrlReg4Range400G = 0x30U;

    constexpr size_t kAccelBurstLength = 6U;
    constexpr float kGravity = 9.80665f;
    constexpr float kScale100G = 0.049f;
    constexpr float kScale200G = 0.098f;
    constexpr float kScale400G = 0.195f;

    SPISettings h3lis331dlSpiSettings()
    {
        return SPISettings(1000000UL, MSBFIRST, SPI_MODE3);
    }

    int16_t makeRaw12(uint8_t lo, uint8_t hi)
    {
        const int16_t raw16 = static_cast<int16_t>(
            (static_cast<uint16_t>(hi) << 8) |
            static_cast<uint16_t>(lo));
        return static_cast<int16_t>(raw16 >> 4);
    }
}

bool H3LIS331DLHAL::begin(uint8_t csPin,
                          SPIClass &spi,
                          H3LIS331DLRange range)
{
    csPin_ = csPin;
    spi_ = &spi;
    range_ = range;
    initialized_ = false;

    pinMode(csPin_, OUTPUT);
    deselect();
    spi_->begin();

    whoAmI_ = readWhoAmI();
    if (whoAmI_ != kExpectedWhoAmI)
    {
        return false;
    }

    writeRegister(kRegCtrl1, kCtrlReg1EnableXyz50Hz);
    writeRegister(kRegCtrl4, ctrlReg4Value());

    initialized_ = true;
    return true;
}

bool H3LIS331DLHAL::read(H3LIS331DLReading &out, uint32_t nowMs)
{
    if ((spi_ == nullptr) || !initialized_)
    {
        return false;
    }

    uint8_t buffer[kAccelBurstLength];
    if (!readBurst(kRegOutXL, buffer, sizeof(buffer)))
    {
        return false;
    }

    out.rawX = makeRaw12(buffer[0], buffer[1]);
    out.rawY = makeRaw12(buffer[2], buffer[3]);
    out.rawZ = makeRaw12(buffer[4], buffer[5]);

    const float scale = scaleGPerDigit();
    out.accelXG = static_cast<float>(out.rawX) * scale;
    out.accelYG = static_cast<float>(out.rawY) * scale;
    out.accelZG = static_cast<float>(out.rawZ) * scale;

    out.accelXMps2 = out.accelXG * kGravity;
    out.accelYMps2 = out.accelYG * kGravity;
    out.accelZMps2 = out.accelZG * kGravity;

    out.whoAmI = whoAmI_;
    out.sampleMs = nowMs;

    return true;
}

uint8_t H3LIS331DLHAL::readWhoAmI()
{
    return readRegister(kRegWhoAmI);
}

void H3LIS331DLHAL::select()
{
    digitalWrite(csPin_, LOW);
}

void H3LIS331DLHAL::deselect()
{
    digitalWrite(csPin_, HIGH);
}

uint8_t H3LIS331DLHAL::readRegister(uint8_t reg)
{
    if (spi_ == nullptr)
    {
        return 0U;
    }

    spi_->beginTransaction(h3lis331dlSpiSettings());
    select();
    spi_->transfer(reg | kSpiRead);
    const uint8_t value = spi_->transfer(0x00U);
    deselect();
    spi_->endTransaction();

    return value;
}

void H3LIS331DLHAL::writeRegister(uint8_t reg, uint8_t value)
{
    if (spi_ == nullptr)
    {
        return;
    }

    spi_->beginTransaction(h3lis331dlSpiSettings());
    select();
    spi_->transfer(reg);
    spi_->transfer(value);
    deselect();
    spi_->endTransaction();
}

bool H3LIS331DLHAL::readBurst(uint8_t startReg, uint8_t *buffer, size_t length)
{
    if ((spi_ == nullptr) || (buffer == nullptr) || (length == 0U))
    {
        return false;
    }

    spi_->beginTransaction(h3lis331dlSpiSettings());
    select();
    spi_->transfer(startReg | kSpiRead | kSpiAutoIncrement);

    for (size_t i = 0; i < length; ++i)
    {
        buffer[i] = spi_->transfer(0x00U);
    }

    deselect();
    spi_->endTransaction();

    return true;
}

float H3LIS331DLHAL::scaleGPerDigit() const
{
    switch (range_)
    {
    case H3LIS331DLRange::RANGE_200G:
        return kScale200G;
    case H3LIS331DLRange::RANGE_400G:
        return kScale400G;
    case H3LIS331DLRange::RANGE_100G:
    default:
        return kScale100G;
    }
}

uint8_t H3LIS331DLHAL::ctrlReg4Value() const
{
    uint8_t rangeBits = kCtrlReg4Range100G;

    switch (range_)
    {
    case H3LIS331DLRange::RANGE_200G:
        rangeBits = kCtrlReg4Range200G;
        break;
    case H3LIS331DLRange::RANGE_400G:
        rangeBits = kCtrlReg4Range400G;
        break;
    case H3LIS331DLRange::RANGE_100G:
    default:
        rangeBits = kCtrlReg4Range100G;
        break;
    }

    return static_cast<uint8_t>(kCtrlReg4BlockDataUpdate | rangeBits);
}
