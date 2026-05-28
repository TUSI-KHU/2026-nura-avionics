#include "spi_flash_circular_log_storage.h"

#include <string.h>

SpiFlashCircularLogStorage::SpiFlashCircularLogStorage(W25qSpiFlashHal &flash, SPIClass &spi, uint8_t csPin)
    : flash_(flash),
      spi_(spi),
      csPin_(csPin)
{
}

bool SpiFlashCircularLogStorage::begin()
{
    stopped_ = false;
    healthy_ = flash_.begin(spi_, csPin_, NuraConstants::Logger::kFlightLogSpiFrequencyHz) &&
               flash_.capacityBytes() >= kCapacityBytes;
    currentSector_ = 0U;
    sessionId_ = 1U;
    nextSectorSequence_ = 0U;
    sectorOffset_ = kPayloadOffset;
    sectorOpen_ = false;
    memset(sectorBuffer_, 0xFF, sizeof(sectorBuffer_));
    if (healthy_ && !scanExistingLog())
    {
        healthy_ = false;
    }
    return healthy_;
}

bool SpiFlashCircularLogStorage::append(const uint8_t *data, uint16_t length)
{
    if (!healthy_ || stopped_ || data == nullptr || length == 0U)
    {
        return false;
    }

    const uint16_t maxPayloadBytes = static_cast<uint16_t>(kSectorBytes - kPayloadOffset);
    if (length > maxPayloadBytes)
    {
        return false;
    }

    if (!sectorOpen_ && !enterSector(currentSector_))
    {
        healthy_ = false;
        return false;
    }

    if ((sectorOffset_ + length) > kSectorBytes)
    {
        currentSector_ = nextSector(currentSector_);
        sectorOpen_ = false;
        if (!enterSector(currentSector_))
        {
            healthy_ = false;
            return false;
        }
    }

    if (!writeBuffered(data, length))
    {
        healthy_ = false;
        return false;
    }

    if (sectorOffset_ >= kSectorBytes)
    {
        currentSector_ = nextSector(currentSector_);
        sectorOffset_ = kPayloadOffset;
        sectorOpen_ = false;
    }

    return true;
}

bool SpiFlashCircularLogStorage::flush()
{
    return healthy_;
}

void SpiFlashCircularLogStorage::stop()
{
    stopped_ = true;
}

bool SpiFlashCircularLogStorage::healthy() const
{
    return healthy_ && !stopped_;
}

bool SpiFlashCircularLogStorage::scanExistingLog()
{
    bool found = false;
    uint32_t newestSector = 0U;
    uint32_t newestSessionId = 0U;
    uint32_t newestSequence = 0U;

    nura_log::FlashSectorHeader header;
    for (uint32_t sector = 0U; sector < sectorCount(); ++sector)
    {
        if (!flash_.read(sectorAddress(sector), reinterpret_cast<uint8_t *>(&header), sizeof(header)))
        {
            return false;
        }

        if (!nura_log::validFlashSectorHeader(header, kSectorBytes, kPayloadOffset))
        {
            continue;
        }

        if (!found || static_cast<int32_t>(header.sectorSequence - newestSequence) > 0)
        {
            found = true;
            newestSector = sector;
            newestSessionId = header.sessionId;
            newestSequence = header.sectorSequence;
        }
    }

    if (found)
    {
        sessionId_ = nura_log::nextFlashSessionId(newestSessionId);
        nextSectorSequence_ = newestSequence + 1UL;
        currentSector_ = nextSector(newestSector);
        return true;
    }

    sessionId_ = 1U;
    nextSectorSequence_ = 0U;
    currentSector_ = 0U;
    return true;
}

bool SpiFlashCircularLogStorage::enterSector(uint32_t sectorIndex)
{
    if (!flash_.eraseSector(sectorAddress(sectorIndex)))
    {
        return false;
    }

    memset(sectorBuffer_, 0xFF, sizeof(sectorBuffer_));
    const nura_log::FlashSectorHeader header = nura_log::makeFlashSectorHeader(sessionId_,
                                                                              nextSectorSequence_,
                                                                              kSectorBytes,
                                                                              kPayloadOffset);
    memcpy(sectorBuffer_, &header, sizeof(header));
    if (!programBytes(sectorAddress(sectorIndex), reinterpret_cast<const uint8_t *>(&header), sizeof(header)))
    {
        return false;
    }

    sectorOffset_ = kPayloadOffset;
    sectorOpen_ = true;
    ++nextSectorSequence_;

    return true;
}

bool SpiFlashCircularLogStorage::writeBuffered(const uint8_t *data, uint16_t length)
{
    if (length == 0U || (sectorOffset_ + length) > kSectorBytes)
    {
        return false;
    }

    memcpy(sectorBuffer_ + sectorOffset_, data, length);
    const uint32_t address = sectorAddress(currentSector_) + sectorOffset_;
    if (!programBytes(address, data, length))
    {
        return false;
    }

    sectorOffset_ = static_cast<uint16_t>(sectorOffset_ + length);
    return true;
}

bool SpiFlashCircularLogStorage::programBytes(uint32_t address, const uint8_t *data, uint16_t length)
{
    uint16_t offset = 0U;
    while (offset < length)
    {
        const uint16_t pageRoom = static_cast<uint16_t>(kPageBytes - ((address + offset) % kPageBytes));
        const uint16_t remaining = static_cast<uint16_t>(length - offset);
        const uint16_t chunk = remaining < pageRoom ? remaining : pageRoom;
        if (!flash_.pageProgram(address + offset, data + offset, chunk))
        {
            return false;
        }
        offset = static_cast<uint16_t>(offset + chunk);
    }
    return true;
}

uint32_t SpiFlashCircularLogStorage::sectorAddress(uint32_t sectorIndex) const
{
    return sectorIndex * static_cast<uint32_t>(kSectorBytes);
}

uint32_t SpiFlashCircularLogStorage::nextSector(uint32_t sectorIndex) const
{
    ++sectorIndex;
    if (sectorIndex >= sectorCount())
    {
        sectorIndex = 0U;
    }
    return sectorIndex;
}

uint32_t SpiFlashCircularLogStorage::sectorCount() const
{
    return kCapacityBytes / static_cast<uint32_t>(kSectorBytes);
}
