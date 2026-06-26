#pragma once

#include <Arduino.h>
#include <LittleFS.h>
#include <stdint.h>

#include "flight_log_storage.h"

class ProgramFlashFlightLogStorage : public IFlightLogStorage
{
public:
    explicit ProgramFlashFlightLogStorage(LittleFS_QSPIFlash &fs,
                                          uint32_t sizeBytes,
                                          const char *directory = "/NURA_LOG");

    bool begin() override;
    bool append(const uint8_t *data, uint16_t length) override;
    bool flush() override;
    void stop() override;
    bool healthy() const override;

    const char *path() const;
    uint64_t totalBytes() const;
    uint64_t usedBytes() const;

private:
    bool openNextFile();
    bool rotateFile();
    bool reclaimSpace(uint32_t bytesNeeded);
    bool removeOldestLogFile();
    void makePath(uint16_t index, char *buffer, size_t bufferLength) const;

    LittleFS_QSPIFlash &fs_;
    uint32_t sizeBytes_;
    const char *directory_;
    File file_;
    char path_[32] = {};
    uint32_t activeFileBytes_ = 0U;
    bool healthy_ = false;
    bool stopped_ = false;
};
