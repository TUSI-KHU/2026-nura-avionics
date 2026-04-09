// #pragma once

// #include <stddef.h>

// #include "core/contexts.h"
// #include "core/recoverable_device/recoverable_device.h"
// #include "core/tasks.h"

// class WatchdogTask : public Task
// {
// public:
//     WatchdogTask(RecoverableDevice *const *devices, size_t deviceCount);

//     const char *name() const override;
//     bool init(SystemContext &ctx) override;
//     bool tick(SystemContext &ctx, uint32_t nowMs) override;
//     uint32_t periodMs() const override;

// private:
//     void handleRecovery(SystemContext &ctx, RecoverableDevice &device, uint32_t nowMs) const;
//     void handleAbort(SystemContext &ctx, RecoverableDevice &device, uint32_t nowMs) const;

//     RecoverableDevice *const *devices_;
//     size_t deviceCount_;
// };
