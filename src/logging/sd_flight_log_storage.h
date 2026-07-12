#pragma once

#include <Arduino.h>
#include <SD.h>
#include <stdint.h>

#include "flight_log_storage.h"

class SdFlightLogStorage : public IFlightLogStorage
{
public:
    SdFlightLogStorage(uint8_t csPin, const char *directory = "/NURA_LOG");

    bool begin() override;
    bool append(const uint8_t *data, uint16_t length) override;
    bool flush() override;
    void stop() override;
    bool healthy() const override;
    const char *path() const;

private:
    bool openNextFile();

    uint8_t csPin_;
    const char *directory_;
    File file_;
    char path_[32] = {};
    bool healthy_ = false;
    bool stopped_ = false;
};
