#pragma once
#include <stdint.h>

#include "core/tasks.h"

enum class DeviceHealth : uint8_t
{
    NORMAL,
    DEGRADED,
    FAILED
};

struct DeviceHealthInfo
{
    DeviceHealth healthState = DeviceHealth::FAILED;
    uint8_t readFails = 0;
    uint32_t lastOkMs = 0;
    uint32_t lastRecoveryAttemptMs = 0;
};

class RecoverableDevice
{
public:
    RecoverableDevice(uint8_t readFailureThreshold,
                      uint32_t recoveryIntervalMs,
                      uint32_t maxRetryBeforeFailureMs);
    
    virtual ~RecoverableDevice() = default;

    virtual const char *deviceName() const = 0;
    virtual DeviceHealthInfo &health(SystemContext &ctx) const = 0;
    virtual bool recover(SystemContext &ctx, uint32_t nowMs) = 0;

    void markInitialized(SystemContext &ctx, uint32_t nowMs) const;
    void markReadSuccess(SystemContext &ctx, uint32_t nowMs) const;
    void markReadFailure(SystemContext &ctx, uint32_t nowMs) const;
    bool shouldRecover(SystemContext &ctx, uint32_t nowMs) const;
    void markRecoveryAttempt(SystemContext &ctx, uint32_t nowMs) const;
    void markRecoverySuccess(SystemContext &ctx, uint32_t nowMs) const;
    void markRecoveryFailure(SystemContext &ctx, uint32_t nowMs) const;
    bool isAvailable(SystemContext &ctx) const;
    bool isFailed(SystemContext &ctx) const;
    
    uint8_t readFailureThreshold() const { return readFailureThreshold_; }
    uint32_t recoveryIntervalMs() const { return recoveryIntervalMs_; }
    uint32_t maxRetryBeforeFailureMs() const { return maxRetryBeforeFailureMs_; }

private:
    static uint32_t elapsedSince(uint32_t startMs, uint32_t nowMs);
    bool hasExceededRecoveryBudget(const DeviceHealthInfo &info, uint32_t nowMs) const;

    uint8_t readFailureThreshold_;
    uint32_t recoveryIntervalMs_;
    uint32_t maxRetryBeforeFailureMs_;
};
