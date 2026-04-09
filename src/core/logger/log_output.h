#pragma once

#include "core/logger/logger.h"

class ILogOutput
{
public:
    virtual ~ILogOutput() = default;
    virtual void write(const LogEntry &entry) = 0;
};
