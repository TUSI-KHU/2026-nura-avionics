#pragma once

#include "app/app_config.h"

class IPanicHandler
{
public:
    virtual ~IPanicHandler() = default;
    virtual void panic() = 0;
};

class BlinkingPanicHandler : public IPanicHandler
{
public:
    explicit BlinkingPanicHandler(const IAppConfig &config);

    void panic() override;

private:
    const IAppConfig &config_;
};
