#pragma once

#include "logger.h"

class ILogOutput
{
public:
    virtual ~ILogOutput() = default;
    virtual bool write(const LogEntry &entry) = 0;
};
