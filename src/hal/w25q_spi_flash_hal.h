#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <stddef.h>
#include <stdint.h>

struct W25qJedecId
{
    uint8_t manufacturer = 0U;
    uint8_t memoryType = 0U;
    uint8_t capacityCode = 0U;
};

class W25qSpiFlashHal
{
public:
    bool begin(SPIClass &spi, uint8_t csPin, uint32_t spiHz);
    bool readJedecId(W25qJedecId &id);
    uint32_t capacityBytes() const;
    bool read(uint32_t address, uint8_t *data, size_t length);
    bool pageProgram(uint32_t address, const uint8_t *data, size_t length);
    bool eraseSector(uint32_t address);
    bool waitWhileBusy(uint32_t timeoutMs);
    bool healthy() const;

private:
    static constexpr uint8_t kCmdReadJedecId = 0x9FU;
    static constexpr uint8_t kCmdReadStatus1 = 0x05U;
    static constexpr uint8_t kCmdWriteEnable = 0x06U;
    static constexpr uint8_t kCmdReadData = 0x03U;
    static constexpr uint8_t kCmdPageProgram = 0x02U;
    static constexpr uint8_t kCmdSectorErase4k = 0x20U;
    static constexpr uint8_t kStatusBusyMask = 0x01U;
    static constexpr size_t kPageSize = 256U;

    bool writeEnable();
    uint8_t readStatus1();
    void select();
    void deselect();
    void transferAddress(uint32_t address);

    SPIClass *spi_ = nullptr;
    uint8_t csPin_ = 0U;
    uint32_t spiHz_ = 0U;
    uint32_t capacityBytes_ = 0U;
    bool healthy_ = false;
};
