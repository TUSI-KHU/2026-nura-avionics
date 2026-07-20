#pragma once

#include <stdint.h>

#include "flight_log_storage.h"

class FlightLogMirrorStorage : public IFlightLogStorage
{
public:
    FlightLogMirrorStorage(IFlightLogStorage &primary, IFlightLogStorage &mirror);

    bool begin() override;
    bool canAppend(uint16_t length) const override;
    bool append(const uint8_t *data, uint16_t length) override;
    bool service(uint32_t nowMs) override;
    bool requestFlush() override;
    bool idle() const override;
    void stop() override;
    bool healthy() const override;

    bool primaryHealthy() const;
    bool mirrorHealthy() const;
    bool fullyHealthy() const;

private:
    IFlightLogStorage &primary_;
    IFlightLogStorage &mirror_;
    bool primaryActive_ = false;
    bool mirrorActive_ = false;
    bool stopped_ = false;
};
