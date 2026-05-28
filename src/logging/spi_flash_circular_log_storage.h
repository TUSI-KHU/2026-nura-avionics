#pragma once

#include <stddef.h>
#include <stdint.h>

#include "flight_log_storage.h"
#include "logging/flight_log_flash_format.h"
#include "hal/w25q_spi_flash_hal.h"
#include "nura_constants.h"

class SpiFlashCircularLogStorage : public IFlightLogStorage
{
public:
    SpiFlashCircularLogStorage(W25qSpiFlashHal &flash, SPIClass &spi, uint8_t csPin);

    bool begin() override;
    bool append(const uint8_t *data, uint16_t length) override;
    bool flush() override;
    void stop() override;
    bool healthy() const override;

private:
    static constexpr uint32_t kCapacityBytes = NuraConstants::Logger::kFlightLogFlashBytes;
    static constexpr uint16_t kSectorBytes = NuraConstants::Logger::kFlightLogFlashSectorBytes;
    static constexpr uint16_t kPageBytes = 256U;
    static constexpr uint16_t kPayloadOffset = 64U;

    bool scanExistingLog();
    bool enterSector(uint32_t sectorIndex);
    bool writeBuffered(const uint8_t *data, uint16_t length);
    bool programBytes(uint32_t address, const uint8_t *data, uint16_t length);
    uint32_t sectorAddress(uint32_t sectorIndex) const;
    uint32_t nextSector(uint32_t sectorIndex) const;
    uint32_t sectorCount() const;

    W25qSpiFlashHal &flash_;
    SPIClass &spi_;
    uint8_t csPin_;
    uint8_t sectorBuffer_[kSectorBytes] = {};
    uint32_t currentSector_ = 0U;
    uint32_t sessionId_ = 1U;
    uint32_t nextSectorSequence_ = 0U;
    uint16_t sectorOffset_ = 0U;
    bool sectorOpen_ = false;
    bool healthy_ = false;
    bool stopped_ = false;
};
