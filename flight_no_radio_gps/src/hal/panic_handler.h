#pragma once

#include "app/app_config.h"

class IPanicHandler
{
public:
    virtual ~IPanicHandler() = default;
    virtual void panic(const char *reason = nullptr) = 0;
};

class BlinkingPanicHandler : public IPanicHandler
{
public:
    explicit BlinkingPanicHandler(const IAppConfig &config);

    void panic(const char *reason = nullptr) override;

private:
    const IAppConfig &config_;
};
