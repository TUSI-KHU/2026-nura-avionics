#pragma once

#include <stddef.h>
#include <stdint.h>

class W25Q128QspiHAL
{
public:
    bool begin();
    bool read(uint32_t address, void *data, uint16_t length);
    bool ready(bool &isReady);
    bool startPageProgram(uint32_t address, const uint8_t *data, uint16_t length);
    bool startSectorErase(uint32_t address);

    bool waitUntilReady(uint32_t timeoutUs);
    bool eraseSectorForInit(uint32_t address, uint32_t timeoutUs);
    bool programForInit(uint32_t address,
                        const uint8_t *data,
                        uint16_t length,
                        uint32_t timeoutUs);

    uint32_t capacityBytes() const;
    const uint8_t *jedecId() const;

private:
    bool configureLut();
    bool command(uint8_t lutIndex, uint32_t address);
    bool readCommand(uint8_t lutIndex, uint32_t address, void *data, uint16_t length);
    bool writeCommand(uint8_t lutIndex,
                      uint32_t address,
                      const uint8_t *data,
                      uint16_t length);
    bool writeEnable();

    uint8_t jedecId_[3] = {};
    bool initialized_ = false;
};
