#pragma once

#include "app/app_config.h"
#include "board_pinmap.h"
#include "core/logger/logger.h"
#include "core/recoverable_task/recoverable_task.h"
#include "core/scheduler.h"
#include "nura_constants.h"
#include "state/abort_state.h"
#include "state/flight_state.h"
#include "state/gps_state.h"
#include "state/high_g_imu_state.h"
#include "state/imu_state.h"
#include "state/magnetometer_state.h"
#include "state/telemetry_state.h"
#if defined(NURA_MOCK_TELEMETRY)
#include "hal/mock_flight_data_hal.h"
#include "missions/mock_telemetry_source_task.h"
#else
#include "hal/bmp390_hal.h"
#include "hal/bno085_hal.h"
#include "sensors/barometer_task.h"
#include "sensors/bno085_task.h"
#endif
#include "hal/panic_handler.h"
#include "hal/buzzer_hal.h"
#include "hal/mosfet_pyro_hal.h"
#include "hal/serial_log_output.h"
#include "logging/flight_log_mirror_storage.h"
#include "logging/flight_log_storage.h"
#if !defined(NURA_MOCK_TELEMETRY) && !defined(NURA_DISABLE_PROGRAM_FLASH_LOG)
#include "logging/program_flash_flight_log_storage.h"
#endif
#include "logging/sd_flight_log_storage.h"
#include "missions/flight_log_task.h"
#include "missions/fsm_task.h"
#include "missions/logger_task.h"
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
    HighGImuState highGImuState_;
    MagnetometerState magnetometerState_;
    AbortState abortState_;
    TelemetryState telemetryState_;
    Logger logger_;
    Scheduler scheduler_;
#if defined(NURA_MOCK_TELEMETRY)
    MockFlightDataHAL mockDataHal_;
#else
    BNO085HAL imuHal_{BoardPinMap::BNO085::resetPin};
    BMP390HAL barometerHal_;
#endif
    MosfetPyroHAL pyroHal_;
    BuzzerHAL buzzerHal_{BoardPinMap::Buzzer::pin};
#if defined(NURA_MOCK_TELEMETRY) || defined(NURA_DISABLE_PROGRAM_FLASH_LOG)
    NullFlightLogStorage programFlashLogStorage_;
#else
    LittleFS_QSPIFlash programFlashFs_;
    ProgramFlashFlightLogStorage programFlashLogStorage_{programFlashFs_, NuraConstants::Logger::kFlightLogProgramFlashBytes};
#endif
    SdFlightLogStorage sdLogStorage_{BoardPinMap::MicroSD::csPin};
    FlightLogMirrorStorage flightLogStorage_{programFlashLogStorage_, sdLogStorage_};
    BlinkingPanicHandler panicHandler_{config_};
    SerialLogOutput logOutput_;
#if defined(NURA_MOCK_TELEMETRY)
    MockTelemetrySourceTask mockTelemetrySourceTask_{mockDataHal_, imuState_, highGImuState_, gpsState_, telemetryState_, logger_, config_};
    RecoverableTask *const recoverableDevices_[1] = {
        nullptr,
    };
#else
    BNO085Task imuTask_{imuHal_, imuState_, highGImuState_, telemetryState_, logger_, config_};
    BarometerTask barometerTask_{barometerHal_, telemetryState_, logger_, config_};
    RecoverableTask *const recoverableDevices_[1] = {
        &imuTask_,
    };
#endif
    WatchdogTask watchdogTask_{recoverableDevices_, sizeof(recoverableDevices_) / sizeof(recoverableDevices_[0]), abortState_, logger_, config_};
    FlightStateMachineTask fsmTask_{flightState_, abortState_, highGImuState_, imuState_, telemetryState_, logger_, config_, panicHandler_, &pyroHal_, nullptr};
    FlightLogTask flightLogTask_{flightState_, imuState_, highGImuState_, magnetometerState_, gpsState_, telemetryState_, flightLogStorage_, logger_};
    LoggerTask loggerTask_{logger_, logOutput_, config_};
};
