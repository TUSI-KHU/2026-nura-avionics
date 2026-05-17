#pragma once

#include "app/app_config.h"
#include "core/logger/logger.h"
#include "core/recoverable_task/recoverable_task.h"
#include "core/scheduler.h"
#include "state/abort_state.h"
#include "state/flight_state.h"
#include "state/gps_state.h"
#include "state/imu_state.h"
#include "state/telemetry_state.h"
#if defined(NURA_MOCK_TELEMETRY)
#include "hal/mock_flight_data_hal.h"
#include "missions/mock_telemetry_source_task.h"
#else
#include "hal/lsm6dso32_hal.h"
#include "hal/ms5611_hal.h"
#include "hal/ublox_m6_gnss_hal.h"
#include "sensors/barometer_task.h"
#include "sensors/gnss_task.h"
#include "sensors/imu_task.h"
#endif
#include "hal/panic_handler.h"
#include "hal/serial_log_output.h"
#include "hal/sx127x_lora_hal.h"
#include "missions/fsm_task.h"
#include "missions/logger_task.h"
#include "missions/telemetry_task.h"
#include "missions/watchdog_task.h"

class FlightControllerApp
{
public:
    bool setup(uint32_t nowMs);
    void loop(uint32_t nowMs);

private:
    void flushBootLogs();

    DefaultAppConfig config_;
    FlightState flightState_;
    GpsState gpsState_;
    ImuState imuState_;
    AbortState abortState_;
    TelemetryState telemetryState_;
    Logger logger_;
    Scheduler scheduler_;
#if defined(NURA_MOCK_TELEMETRY)
    MockFlightDataHAL mockDataHal_;
#else
    LSM6DSO32HAL imuHal_;
    MS5611HAL barometerHal_;
    UbloxM6GNSSHAL gnssHal_;
#endif
    Sx127xLoRaHAL loraHal_;
    BlinkingPanicHandler panicHandler_{config_};
    SerialLogOutput logOutput_;
#if defined(NURA_MOCK_TELEMETRY)
    MockTelemetrySourceTask mockTelemetrySourceTask_{mockDataHal_, imuState_, gpsState_, telemetryState_, logger_, config_};
    RecoverableTask *const recoverableDevices_[1] = {
        0,
    };
#else
    IMUTask imuTask_{imuHal_, imuState_, logger_, config_};
    BarometerTask barometerTask_{barometerHal_, telemetryState_, logger_, config_};
    GNSSTask gnssTask_{gnssHal_, gpsState_};
    RecoverableTask *const recoverableDevices_[1] = {
        &imuTask_,
    };
#endif
    WatchdogTask watchdogTask_{recoverableDevices_, sizeof(recoverableDevices_) / sizeof(recoverableDevices_[0]), abortState_, logger_, config_};
    FlightStateMachineTask fsmTask_{flightState_, abortState_, logger_, config_, panicHandler_};
    TelemetryTask telemetryTask_{loraHal_, imuState_, gpsState_, telemetryState_, flightState_, abortState_, logger_, config_};
    LoggerTask loggerTask_{logger_, logOutput_, config_};
};
