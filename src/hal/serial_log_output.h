#pragma once

#include <Arduino.h>

#include "core/logger/log_output.h"

class SerialLogOutput : public ILogOutput
{
public:
    // Arduino Serial을 이용하는 로그 출력 백엔드다.
    void begin(unsigned long baudRate);
    bool write(const LogEntry &entry) override;
};
