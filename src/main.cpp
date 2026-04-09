#include <Arduino.h>

#include "core/scheduler.h"

#include "hal/mpu6050_hal.h"
#include "hal/serial_log_output.h"

#include "missions/fsm_task.h"
#include "missions/logger_task.h"

#include "sensors/imu_task.h"


namespace
{
    SystemContext g_ctx;
    Scheduler g_scheduler;
    MPU6050HAL g_imuHal;
    SerialLogOutput g_logOutput;

    IMUTask g_imuTask(g_imuHal);
    FlightStateMachineTask g_fsmTask;
    LoggerTask g_loggerTask(g_logOutput);
}

void setup()
{
    g_logOutput.begin(115200);

    g_scheduler.add(g_imuTask);
    g_scheduler.add(g_fsmTask);
    g_scheduler.add(g_loggerTask);

    g_scheduler.init(g_ctx, millis());
}

void loop()
{
    g_scheduler.tick(g_ctx, millis());
}
