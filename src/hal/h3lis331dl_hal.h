#pragma once

#include <stdint.h>
#include <stddef.h>

#include <Arduino.h>
#include <SPI.h>

enum class H3LIS331DLRange : uint8_t
{
    RANGE_100G,
    RANGE_200G,
    RANGE_400G
};

struct H3LIS331DLReading
{
    int16_t rawX = 0;
    int16_t rawY = 0;
    int16_t rawZ = 0;

    float accelXG = 0.0f;
    float accelYG = 0.0f;
    float accelZG = 0.0f;

    float accelXMps2 = 0.0f;
    float accelYMps2 = 0.0f;
    float accelZMps2 = 0.0f;

    uint8_t whoAmI = 0;
    uint32_t sampleMs = 0;
};

class H3LIS331DLHAL
{
public:
    // FlightControllerApp integration note:
    // 1. Include "hal/h3lis331dl_hal.h", "state/high_g_imu_state.h",
    //    and "sensors/high_g_imu_task.h" in flight_controller_app.h.
    // 2. Add H3LIS331DLHAL, HighGImuState, and HighGImuTask members to
    //    FlightControllerApp.
    // 3. Register the task with scheduler_.add(highGImuTask_) in
    //    FlightControllerApp::setup().
    // 4. Add highGImuTask_ to recoverableDevices_ only if the team wants
    //    high-g IMU failures to affect watchdog recovery/abort behavior.
    bool begin(uint8_t csPin,
               SPIClass &spi = SPI,
               H3LIS331DLRange range = H3LIS331DLRange::RANGE_100G);
    bool read(H3LIS331DLReading &out, uint32_t nowMs);
    uint8_t readWhoAmI();

private:
    void select();
    void deselect();
    uint8_t readRegister(uint8_t reg);
    void writeRegister(uint8_t reg, uint8_t value);
    bool readBurst(uint8_t startReg, uint8_t *buffer, size_t length);
    float scaleGPerDigit() const;
    uint8_t ctrlReg4Value() const;

    SPIClass *spi_ = &SPI;
    uint8_t csPin_ = 10U;
    H3LIS331DLRange range_ = H3LIS331DLRange::RANGE_100G;
    uint8_t whoAmI_ = 0;
    bool initialized_ = false;
};
