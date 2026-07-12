#include "mock_telemetry_source_task.h"

MockTelemetrySourceTask::MockTelemetrySourceTask(MockFlightDataHAL &mockData,
                                                 ImuState &imuState,
                                                 HighGImuState &highGImuState,
                                                 GpsState &gpsState,
                                                 TelemetryState &telemetryState,
                                                 Logger &logger,
                                                 const IAppConfig &config)
    : mockData_(mockData),
      imuState_(imuState),
      highGImuState_(highGImuState),
      gpsState_(gpsState),
      telemetryState_(telemetryState),
      logger_(logger),
      config_(config)
{
}

const char *MockTelemetrySourceTask::name() const
{
    return "mock";
}

bool MockTelemetrySourceTask::init()
{
    const bool ok = mockData_.begin();
    if (ok)
    {
        LOGI(logger_, 0U, "mock", "mock telemetry source initialized");
        LOGI(logger_, 0U, "mock", mockData_.scenarioName());
    }
    return ok;
}

bool MockTelemetrySourceTask::tick(uint32_t nowMs)
{
    MockFlightDataReading sample;
    if (!mockData_.read(sample, nowMs))
    {
        return true;
    }

    imuState_.data.accelXMps2 = sample.accelXMps2;
    imuState_.data.accelYMps2 = sample.accelYMps2;
    imuState_.data.accelZMps2 = sample.accelZMps2;
    imuState_.data.gyroXDps = sample.gyroXDps;
    imuState_.data.gyroYDps = sample.gyroYDps;
    imuState_.data.gyroZDps = sample.gyroZDps;
    imuState_.data.attitudeValid = sample.attitudeValid;
    imuState_.data.rollDeg = sample.rollDeg;
    imuState_.data.pitchDeg = sample.pitchDeg;
    imuState_.data.yawDeg = sample.yawDeg;
    imuState_.data.tiltValid = sample.tiltValid;
    imuState_.data.tiltAngleDeg = sample.tiltAngleDeg;
    imuState_.data.lastUpdatedMs = sample.sampleMs;

    highGImuState_.accelXG = sample.highAccelXG;
    highGImuState_.accelYG = sample.highAccelYG;
    highGImuState_.accelZG = sample.highAccelZG;
    highGImuState_.accelXMps2 = sample.accelXMps2;
    highGImuState_.accelYMps2 = sample.accelYMps2;
    highGImuState_.accelZMps2 = sample.accelZMps2;
    highGImuState_.connected = true;
    highGImuState_.hasNewData = true;
    highGImuState_.lastUpdatedMs = sample.sampleMs;

    BarometerTelemetryData &baro = telemetryState_.barometer;
    if (sample.barometerUpdated)
    {
        baro.valid = true;
        baro.pressurePa = sample.pressurePa;
        if (!baro.referenceValid)
        {
            baro.referencePressurePa = sample.pressurePa;
            baro.referenceValid = true;
        }
        baro.rawAltitudeM = sample.rawAltitudeM;
        baro.altitudeM = sample.filteredAltitudeM;
        baro.lastUpdatedMs = sample.sampleMs;
    }

    GnssTelemetryData &gnss = telemetryState_.gnss;
    gnss.valid = true;
    gnss.hasFix = true;
    gnss.latitudeDeg = sample.latitudeDeg;
    gnss.longitudeDeg = sample.longitudeDeg;
    gnss.altitudeM = sample.altitudeM;
    gnss.speedMps = sample.speedMps;
    gnss.courseDeg = sample.courseDeg;
    gnss.hdop = sample.hdop;
    gnss.satellites = sample.satellites;
    gnss.locationAgeMs = 100U;
    gnss.lastUpdatedMs = sample.sampleMs;

    gpsState_.data.hasFix = true;
    gpsState_.data.latitudeDeg = sample.latitudeDeg;
    gpsState_.data.longitudeDeg = sample.longitudeDeg;
    gpsState_.data.altitudeM = sample.altitudeM;
    gpsState_.data.speedMps = sample.speedMps;
    gpsState_.data.courseDeg = sample.courseDeg;
    gpsState_.data.hdop = sample.hdop;
    gpsState_.data.satellites = sample.satellites;
    gpsState_.data.locationAgeMs = 100U;
    gpsState_.data.charsProcessed = 1U;
    gpsState_.data.passedChecksum = 1U;
    gpsState_.data.failedChecksum = 0U;
    gpsState_.data.lastUpdatedMs = sample.sampleMs;

    telemetryState_.power.valid = true;
    telemetryState_.power.batteryMv = sample.batteryMv;
    telemetryState_.power.lastUpdatedMs = sample.sampleMs;
    telemetryState_.health.highAccelOk = true;
    telemetryState_.health.magOk = true;
    telemetryState_.health.storageOk = true;
    telemetryState_.health.pyroContinuityOk = true;
    return true;
}

uint32_t MockTelemetrySourceTask::periodMs() const
{
    return config_.imuTaskPeriodMs();
}
