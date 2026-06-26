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
#include "hal/battery_voltage_hal.h"
#include "hal/h3lis331dl_hal.h"
#if !defined(NURA_DISABLE_MAGNETOMETER)
#include "hal/lis3mdl_hal.h"
#endif
#if !defined(NURA_DISABLE_LOWG_IMU)
#include "hal/lsm6dso32_hal.h"
#endif
#include "hal/mpl3115a2_hal.h"
#if !defined(NURA_DISABLE_GPS)
#include "hal/ublox_m6_gnss_hal.h"
#endif
#include "sensors/barometer_task.h"
#if !defined(NURA_DISABLE_GPS)
#include "sensors/gnss_task.h"
#endif
#include "sensors/high_g_imu_task.h"
#if !defined(NURA_DISABLE_LOWG_IMU)
#include "sensors/imu_task.h"
#endif
#if !defined(NURA_DISABLE_MAGNETOMETER)
#include "sensors/magnetometer_task.h"
#endif
#include "sensors/power_sense_task.h"
#endif
#include "hal/panic_handler.h"
#include "hal/buzzer_hal.h"
#include "hal/mosfet_pyro_hal.h"
#include "hal/serial_log_output.h"
#if !defined(NURA_DISABLE_LORA)
#include "hal/sx1262_lora_hal.h"
#endif
#include "logging/flight_log_mirror_storage.h"
#include "logging/flight_log_storage.h"
#if !defined(NURA_MOCK_TELEMETRY) && !defined(NURA_DISABLE_PROGRAM_FLASH_LOG)
#include "logging/program_flash_flight_log_storage.h"
#endif
#include "logging/sd_flight_log_storage.h"
#include "missions/flight_log_task.h"
#include "missions/fsm_task.h"
#include "missions/logger_task.h"
#if !defined(NURA_DISABLE_LORA)
#include "missions/telemetry_task.h"
#endif
#include "missions/watchdog_task.h"

class FlightControllerApp
{
public:
    bool setup(uint32_t nowMs);
    void loop(uint32_t nowMs);

private:
    void flushBootLogs();
#if defined(NURA_BENCH_SD_SERIAL_DUMP)
    void dumpBenchSdLogIfReady(uint32_t nowMs);
    void dumpLatestSdLogToSerial();
    bool benchSdDumpDone_ = false;
#endif

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
#if !defined(NURA_DISABLE_LOWG_IMU)
    LSM6DSO32HAL imuHal_;
#endif
    H3LIS331DLHAL highGImuHal_;
#if !defined(NURA_DISABLE_MAGNETOMETER)
    LIS3MDLHAL magnetometerHal_;
#endif
    MPL3115A2HAL barometerHal_;
#if !defined(NURA_DISABLE_GPS)
    UbloxM6GNSSHAL gnssHal_;
#endif
    BatteryVoltageHAL batteryVoltageHal_;
#endif
#if !defined(NURA_DISABLE_LORA)
    Sx1262LoRaHAL loraHal_;
#endif
    MosfetPyroHAL pyroHal_;
    BuzzerHAL buzzerHal_{BoardPinMap::Buzzer::pin};
#if defined(NURA_MOCK_TELEMETRY) || defined(NURA_DISABLE_PROGRAM_FLASH_LOG)
    NullFlightLogStorage programFlashLogStorage_;
#else
    LittleFS_Program programFlashFs_;
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
#if !defined(NURA_DISABLE_LOWG_IMU)
    IMUTask imuTask_{imuHal_, imuState_, logger_, config_};
#endif
    HighGImuTask highGImuTask_{highGImuHal_,
                               highGImuState_,
                               telemetryState_,
                               logger_,
                               config_,
                               BoardPinMap::H3LIS331DL::csPin,
                               H3LIS331DLRange::RANGE_200G};
#if !defined(NURA_DISABLE_MAGNETOMETER)
    MagnetometerTask magnetometerTask_{magnetometerHal_, magnetometerState_, telemetryState_, logger_, config_};
#endif
    BarometerTask barometerTask_{barometerHal_, telemetryState_, logger_, config_};
#if !defined(NURA_DISABLE_GPS)
    GNSSTask gnssTask_{gnssHal_, gpsState_, config_};
#endif
    PowerSenseTask powerSenseTask_{batteryVoltageHal_, telemetryState_, logger_};
#if defined(NURA_DISABLE_LOWG_IMU) && defined(NURA_DISABLE_MAGNETOMETER)
    RecoverableTask *const recoverableDevices_[1] = {
        &highGImuTask_,
    };
#elif defined(NURA_DISABLE_MAGNETOMETER)
    RecoverableTask *const recoverableDevices_[2] = {
        &imuTask_,
        &highGImuTask_,
    };
#elif defined(NURA_DISABLE_LOWG_IMU)
    RecoverableTask *const recoverableDevices_[2] = {
        &highGImuTask_,
        &magnetometerTask_,
    };
#else
    RecoverableTask *const recoverableDevices_[3] = {
        &imuTask_,
        &highGImuTask_,
        &magnetometerTask_,
    };
#endif
#endif
    WatchdogTask watchdogTask_{recoverableDevices_, sizeof(recoverableDevices_) / sizeof(recoverableDevices_[0]), abortState_, logger_, config_};
    FlightStateMachineTask fsmTask_{flightState_, abortState_, highGImuState_, imuState_, telemetryState_, logger_, config_, panicHandler_, &pyroHal_, &buzzerHal_};
    FlightLogTask flightLogTask_{flightState_, imuState_, highGImuState_, magnetometerState_, gpsState_, telemetryState_, flightLogStorage_, logger_};
#if !defined(NURA_DISABLE_LORA)
    TelemetryTask telemetryTask_{loraHal_, imuState_, gpsState_, telemetryState_, flightState_, abortState_, logger_, config_};
#endif
    LoggerTask loggerTask_{logger_, logOutput_, config_};
};
