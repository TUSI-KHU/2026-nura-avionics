#pragma once

#include "logger.h"

class ILogOutput
{
public:
    // LoggerTask가 실제 출력 장치에 로그를 기록할 때 사용하는 인터페이스다.
    virtual ~ILogOutput() = default;
    virtual bool write(const LogEntry &entry) = 0;
};
