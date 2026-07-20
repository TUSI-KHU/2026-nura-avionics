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
#include "state/barometer_state.h"
#include "state/power_state.h"
#include "state/system_health_state.h"
#include "missions/flight/flight_trace.h"
#include "missions/telemetry/telemetry_snapshot.h"
#if defined(NURA_MOCK_TELEMETRY)
#include "hal/mock_flight_data_hal.h"
#include "test_support/mock_telemetry_source_task.h"
#else
#include "hal/battery_voltage_hal.h"
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
#include "sensors/power_sense_task.h"
#endif
#include "hal/panic_handler.h"
#include "hal/buzzer_hal.h"
#include "hal/mosfet_pyro_hal.h"
#include "hal/serial_log_output.h"
#if defined(NURA_USE_SX127X_LORA)
#include "hal/sx127x_lora_hal.h"
#else
#include "hal/sx1262_lora_hal.h"
#endif
#include "hal/w25q128_qspi_hal.h"
#include "logging/flight_log_mirror_storage.h"
#include "logging/flight_log_storage.h"
#if !defined(NURA_MOCK_TELEMETRY) && !defined(NURA_DISABLE_PROGRAM_FLASH_LOG)
#include "logging/program_flash_flight_log_storage.h"
#endif
#include "logging/sd_flight_log_storage.h"
#include "missions/logging/flight_log_task.h"
#include "missions/flight/fsm_task.h"
#include "missions/system/logger_task.h"
#include "missions/telemetry/telemetry_task.h"
#include "missions/system/watchdog_task.h"

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
    BarometerState barometerState_;
    PowerState powerState_;
    SystemHealthState healthState_;
    TelemetrySnapshot telemetryState_{barometerState_, powerState_, healthState_};
    FlightTraceBuffer flightTrace_;
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
    BatteryVoltageHAL batteryVoltageHal_;
#endif
    TelemetryLoRaHAL loraHal_;
    MosfetPyroHAL pyroHal_;
    BuzzerHAL buzzerHal_{BoardPinMap::Buzzer::pin};
#if defined(NURA_MOCK_TELEMETRY) || defined(NURA_DISABLE_PROGRAM_FLASH_LOG)
    NullFlightLogStorage programFlashLogStorage_;
#else
    W25Q128QspiHAL programFlashHal_;
    ProgramFlashFlightLogStorage programFlashLogStorage_{programFlashHal_};
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
    IMUTask imuTask_{imuHal_, imuState_, logger_, config_, BoardPinMap::LSM6DSO32::csPin, SPI};
    HighGImuTask highGImuTask_{highGImuHal_,
                               highGImuState_,
                               healthState_,
                               logger_,
                               config_,
                               BoardPinMap::H3LIS331DL::csPin,
                               H3LIS331DLRange::RANGE_100G,
                               SPI};
    MagnetometerTask magnetometerTask_{magnetometerHal_,
                                       magnetometerState_,
                                       healthState_,
                                       logger_,
                                       config_,
                                       BoardPinMap::LIS3MDL::i2cAddress,
                                       BoardPinMap::LIS3MDL::wire()};
    BarometerTask barometerTask_{barometerHal_,
                                 barometerState_,
                                 logger_,
                                 config_,
                                 BoardPinMap::MPL3115A2::wire()};
    GNSSTask gnssTask_{gnssHal_,
                       gpsState_,
                       config_,
                       BoardPinMap::UbloxM6::serial(),
                       BoardPinMap::UbloxM6::baud};
    PowerSenseTask powerSenseTask_{batteryVoltageHal_, powerState_, logger_, BoardPinMap::PowerSense::voltagePin};
    RecoverableTask *const recoverableDevices_[3] = {
        &imuTask_,
        &highGImuTask_,
        &magnetometerTask_,
    };
#endif
    WatchdogTask watchdogTask_{recoverableDevices_, sizeof(recoverableDevices_) / sizeof(recoverableDevices_[0]), abortState_, logger_, config_};
    FlightStateMachineTask fsmTask_{flightState_, abortState_, highGImuState_, imuState_, telemetryState_, logger_, config_, panicHandler_, &pyroHal_, &buzzerHal_, &flightTrace_};
    FlightLogTask flightLogTask_{flightState_, imuState_, highGImuState_, magnetometerState_, gpsState_, telemetryState_, flightTrace_, flightLogStorage_, logger_};
    TelemetryTask telemetryTask_{loraHal_, imuState_, gpsState_, telemetryState_, flightState_, abortState_, logger_, config_};
    LoggerTask loggerTask_{logger_, logOutput_, config_};
};
