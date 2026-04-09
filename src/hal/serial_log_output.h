#pragma once

#include <Arduino.h>

#include "core/logger/log_output.h"

class SerialLogOutput : public ILogOutput
{
public:
    void begin(unsigned long baudRate);
    void write(const LogEntry &entry) override;
};
