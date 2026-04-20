#pragma once

#include "app/app_config.h"
#include "core/logger/logger.h"
#include "core/recoverable_task/recoverable_task.h"
#include "core/scheduler.h"
#include "state/abort_state.h"
#include "state/flight_state.h"
#include "state/imu_state.h"
#include "hal/mpu6050_hal.h"
#include "hal/panic_handler.h"
#include "hal/serial_log_output.h"
#include "missions/fsm_task.h"
#include "missions/logger_task.h"
#include "missions/watchdog_task.h"
#include "sensors/imu_task.h"

class FlightControllerApp
{
public:
    bool setup(uint32_t nowMs);
    void loop(uint32_t nowMs);

private:
    void flushBootLogs();

    DefaultAppConfig config_;
    FlightState flightState_;
    ImuState imuState_;
    AbortState abortState_;
    Logger logger_;
    Scheduler scheduler_;
    MPU6050HAL imuHal_;
    BlinkingPanicHandler panicHandler_{config_};
    SerialLogOutput logOutput_;
    IMUTask imuTask_{imuHal_, imuState_, logger_, config_};
    RecoverableTask *const recoverableDevices_[1] = {
        &imuTask_,
    };
    WatchdogTask watchdogTask_{recoverableDevices_, sizeof(recoverableDevices_) / sizeof(recoverableDevices_[0]), abortState_, logger_, config_};
    FlightStateMachineTask fsmTask_{flightState_, abortState_, logger_, config_, panicHandler_};
    LoggerTask loggerTask_{logger_, logOutput_, config_};
};
