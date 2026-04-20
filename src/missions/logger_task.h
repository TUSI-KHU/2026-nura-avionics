#pragma once

#include "app/app_config.h"
#include "core/logger/log_output.h"
#include "core/logger/logger.h"
#include "core/tasks.h"

class LoggerTask : public Task
{
public:
    // 로그 버퍼에 쌓인 로그를 실제 출력 장치로 배출하는 태스크다.
    LoggerTask(Logger &logger, ILogOutput &output, const IAppConfig &config);

    const char *name() const override;
    bool init() override;
    bool tick(uint32_t nowMs) override;
    uint32_t periodMs() const override;

private:
    uint32_t lastQueueDroppedCount_ = 0;
    uint32_t outputDroppedCount_ = 0;
    uint8_t consecutiveOutputFails_ = 0;

    Logger &logger_;
    ILogOutput &output_;
    const IAppConfig &config_;
};
