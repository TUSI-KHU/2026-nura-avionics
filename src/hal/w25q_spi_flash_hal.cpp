#include "w25q_spi_flash_hal.h"

bool W25qSpiFlashHal::begin(SPIClass &spi, uint8_t csPin, uint32_t spiHz)
{
    spi_ = &spi;
    csPin_ = csPin;
    spiHz_ = spiHz;
    capacityBytes_ = 0U;
    healthy_ = false;

    pinMode(csPin_, OUTPUT);
    deselect();

    W25qJedecId id;
    if (!readJedecId(id) || id.manufacturer == 0x00U || id.manufacturer == 0xFFU ||
        id.capacityCode < 20U || id.capacityCode > 31U)
    {
        return false;
    }

    capacityBytes_ = 1UL << id.capacityCode;
    healthy_ = true;
    return true;
}

bool W25qSpiFlashHal::readJedecId(W25qJedecId &id)
{
    if (spi_ == nullptr)
    {
        return false;
    }

    SPISettings settings(spiHz_, MSBFIRST, SPI_MODE0);
    spi_->beginTransaction(settings);
    select();
    spi_->transfer(kCmdReadJedecId);
    id.manufacturer = spi_->transfer(0x00U);
    id.memoryType = spi_->transfer(0x00U);
    id.capacityCode = spi_->transfer(0x00U);
    deselect();
    spi_->endTransaction();
    return true;
}

uint32_t W25qSpiFlashHal::capacityBytes() const
{
    return capacityBytes_;
}

bool W25qSpiFlashHal::read(uint32_t address, uint8_t *data, size_t length)
{
    if (!healthy_ || data == nullptr || (address + length) > capacityBytes_)
    {
        return false;
    }

    SPISettings settings(spiHz_, MSBFIRST, SPI_MODE0);
    spi_->beginTransaction(settings);
    select();
    spi_->transfer(kCmdReadData);
    transferAddress(address);
    for (size_t i = 0U; i < length; ++i)
    {
        data[i] = spi_->transfer(0x00U);
    }
    deselect();
    spi_->endTransaction();
    return true;
}

bool W25qSpiFlashHal::pageProgram(uint32_t address, const uint8_t *data, size_t length)
{
    if (!healthy_ || data == nullptr || length == 0U || length > kPageSize ||
        (address + length) > capacityBytes_ ||
        ((address & (kPageSize - 1U)) + length) > kPageSize)
    {
        return false;
    }

    if (!writeEnable())
    {
        return false;
    }

    SPISettings settings(spiHz_, MSBFIRST, SPI_MODE0);
    spi_->beginTransaction(settings);
    select();
    spi_->transfer(kCmdPageProgram);
    transferAddress(address);
    for (size_t i = 0U; i < length; ++i)
    {
        spi_->transfer(data[i]);
    }
    deselect();
    spi_->endTransaction();

    return waitWhileBusy(10U);
}

bool W25qSpiFlashHal::eraseSector(uint32_t address)
{
    if (!healthy_ || address >= capacityBytes_)
    {
        return false;
    }

    if (!writeEnable())
    {
        return false;
    }

    SPISettings settings(spiHz_, MSBFIRST, SPI_MODE0);
    spi_->beginTransaction(settings);
    select();
    spi_->transfer(kCmdSectorErase4k);
    transferAddress(address);
    deselect();
    spi_->endTransaction();

    return waitWhileBusy(500U);
}

bool W25qSpiFlashHal::waitWhileBusy(uint32_t timeoutMs)
{
    const uint32_t startMs = millis();
    while ((readStatus1() & kStatusBusyMask) != 0U)
    {
        if ((millis() - startMs) > timeoutMs)
        {
            healthy_ = false;
            return false;
        }
        yield();
    }
    return true;
}

bool W25qSpiFlashHal::healthy() const
{
    return healthy_;
}

bool W25qSpiFlashHal::writeEnable()
{
    if (!healthy_)
    {
        return false;
    }

    SPISettings settings(spiHz_, MSBFIRST, SPI_MODE0);
    spi_->beginTransaction(settings);
    select();
    spi_->transfer(kCmdWriteEnable);
    deselect();
    spi_->endTransaction();
    return true;
}

uint8_t W25qSpiFlashHal::readStatus1()
{
    if (spi_ == nullptr)
    {
        return 0xFFU;
    }

    SPISettings settings(spiHz_, MSBFIRST, SPI_MODE0);
    spi_->beginTransaction(settings);
    select();
    spi_->transfer(kCmdReadStatus1);
    const uint8_t value = spi_->transfer(0x00U);
    deselect();
    spi_->endTransaction();
    return value;
}

void W25qSpiFlashHal::select()
{
    digitalWrite(csPin_, LOW);
}

void W25qSpiFlashHal::deselect()
{
    digitalWrite(csPin_, HIGH);
}

void W25qSpiFlashHal::transferAddress(uint32_t address)
{
    spi_->transfer(static_cast<uint8_t>((address >> 16U) & 0xFFU));
    spi_->transfer(static_cast<uint8_t>((address >> 8U) & 0xFFU));
    spi_->transfer(static_cast<uint8_t>(address & 0xFFU));
}
