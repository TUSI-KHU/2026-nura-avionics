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
#include "hal/h3lis331dl_hal.h"
#include "hal/lis3mdl_hal.h"
#include "hal/lsm6dso32_hal.h"
#include "hal/mpl3115a2_hal.h"
#include "hal/ublox_m6_gnss_hal.h"
#include "sensors/barometer_task.h"
#include "sensors/gnss_task.h"
#include "sensors/high_g_imu_task.h"
#include "sensors/imu_task.h"
#include "sensors/magnetometer_task.h"
#endif
#include "hal/panic_handler.h"
#include "hal/serial_log_output.h"
#include "hal/sx127x_lora_hal.h"
#include "logging/flight_log_mirror_storage.h"
#include "logging/flight_log_storage.h"
#if !defined(NURA_MOCK_TELEMETRY)
#include "logging/program_flash_flight_log_storage.h"
#endif
#include "logging/sd_flight_log_storage.h"
#include "missions/flight_log_task.h"
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
    HighGImuState highGImuState_;
    MagnetometerState magnetometerState_;
    AbortState abortState_;
    TelemetryState telemetryState_;
    Logger logger_;
    Scheduler scheduler_;
#if defined(NURA_MOCK_TELEMETRY)
    MockFlightDataHAL mockDataHal_;
#else
    LSM6DSO32HAL imuHal_;
    H3LIS331DLHAL highGImuHal_;
    LIS3MDLHAL magnetometerHal_;
    MPL3115A2HAL barometerHal_;
    UbloxM6GNSSHAL gnssHal_;
#endif
    Sx127xLoRaHAL loraHal_;
#if defined(NURA_MOCK_TELEMETRY)
    NullFlightLogStorage spiFlashLogStorage_;
#else
    LittleFS_Program programFlashFs_;
    ProgramFlashFlightLogStorage spiFlashLogStorage_{programFlashFs_, NuraConstants::Logger::kFlightLogProgramFlashBytes};
#endif
    SdFlightLogStorage sdLogStorage_{BoardPinMap::MicroSD::csPin};
    FlightLogMirrorStorage flightLogStorage_{spiFlashLogStorage_, sdLogStorage_};
    BlinkingPanicHandler panicHandler_{config_};
    SerialLogOutput logOutput_;
#if defined(NURA_MOCK_TELEMETRY)
    MockTelemetrySourceTask mockTelemetrySourceTask_{mockDataHal_, imuState_, highGImuState_, gpsState_, telemetryState_, logger_, config_};
    RecoverableTask *const recoverableDevices_[1] = {
        nullptr,
    };
#else
    IMUTask imuTask_{imuHal_, imuState_, logger_, config_};
    HighGImuTask highGImuTask_{highGImuHal_,
                               highGImuState_,
                               telemetryState_,
                               logger_,
                               config_,
                               BoardPinMap::H3LIS331DL::csPin,
                               H3LIS331DLRange::RANGE_200G};
    MagnetometerTask magnetometerTask_{magnetometerHal_, magnetometerState_, telemetryState_, logger_, config_};
    BarometerTask barometerTask_{barometerHal_, telemetryState_, logger_, config_};
    GNSSTask gnssTask_{gnssHal_, gpsState_, config_};
    RecoverableTask *const recoverableDevices_[3] = {
        &imuTask_,
        &highGImuTask_,
        &magnetometerTask_,
    };
#endif
    WatchdogTask watchdogTask_{recoverableDevices_, sizeof(recoverableDevices_) / sizeof(recoverableDevices_[0]), abortState_, logger_, config_};
    FlightStateMachineTask fsmTask_{flightState_, abortState_, highGImuState_, imuState_, telemetryState_, logger_, config_, panicHandler_};
    FlightLogTask flightLogTask_{flightState_, imuState_, highGImuState_, magnetometerState_, gpsState_, telemetryState_, flightLogStorage_, logger_};
    TelemetryTask telemetryTask_{loraHal_, imuState_, gpsState_, telemetryState_, flightState_, abortState_, logger_, config_};
    LoggerTask loggerTask_{logger_, logOutput_, config_};
};
