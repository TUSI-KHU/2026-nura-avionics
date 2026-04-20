#include <Arduino.h>

#include "core/fault.h"
#include "core/scheduler.h"

#include "hal/mpu6050_hal.h"
#include "hal/serial_log_output.h"

#include "missions/fsm_task.h"
#include "missions/logger_task.h"
#include "missions/watchdog_task.h"

#include "sensors/imu_task.h"

namespace
{
    // 전역 컨텍스트와 태스크 인스턴스를 생성한다.
    SystemContext g_ctx;
    Scheduler g_scheduler;
    MPU6050HAL g_imuHal;
    SerialLogOutput g_logOutput;

    IMUTask g_imuTask(g_imuHal);

    // RecoverableTask의 경우 별도 처리가 필요하다.
    RecoverableTask *const g_recoverableDevices[] = {
        &g_imuTask,
    };
    WatchdogTask g_watchdogTask(g_recoverableDevices, sizeof(g_recoverableDevices) / sizeof(g_recoverableDevices[0]));
    FlightStateMachineTask g_fsmTask;
    LoggerTask g_loggerTask(g_logOutput);
}

void setup()
{
    // 로그 출력 채널을 먼저 연다.
    g_logOutput.begin(115200);

    // 태스크 등록 순서는 실제 실행 순서에도 영향을 준다.
    g_scheduler.add(g_imuTask);
    g_scheduler.add(g_watchdogTask);
    g_scheduler.add(g_fsmTask);
    g_scheduler.add(g_loggerTask);

    if (!g_scheduler.init(g_ctx, millis()))
    {
        // logger task가 돌기 전일 수 있으므로 치명적 실패는 즉시 정지한다.
        hang();
    }
}

void loop()
{
    // 메인 루프는 현재 시각만 넘기고, 주기 제어는 스케줄러가 담당한다.
    g_scheduler.tick(g_ctx, millis());
}
