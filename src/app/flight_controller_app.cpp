#include "app/flight_controller_app.h"

#include <SPI.h>
#include <Wire.h>

#include "board_pinmap.h"

bool FlightControllerApp::setup(uint32_t nowMs)
{
    // 로그 출력용 시리얼을 먼저 연다.
    logOutput_.begin(config_.serialBaudRate());

    pinMode(BoardPinMap::LSM6DSO32::csPin, OUTPUT);
    pinMode(BoardPinMap::H3LIS331DL::csPin, OUTPUT);
    pinMode(BoardPinMap::Ra01DevelopmentLoRa::ssPin, OUTPUT);
    digitalWrite(BoardPinMap::LSM6DSO32::csPin, HIGH);
    digitalWrite(BoardPinMap::H3LIS331DL::csPin, HIGH);
    digitalWrite(BoardPinMap::Ra01DevelopmentLoRa::ssPin, HIGH);
    SPI.setMOSI(BoardPinMap::SpiBus::mosiPin);
    SPI.setMISO(BoardPinMap::SpiBus::misoPin);
    SPI.setSCK(BoardPinMap::SpiBus::sckPin);
    SPI.begin();
#if !defined(NURA_MOCK_TELEMETRY)
    TwoWire &i2c = BoardPinMap::I2cBus::wire();
    i2c.setSDA(BoardPinMap::I2cBus::sdaPin);
    i2c.setSCL(BoardPinMap::I2cBus::sclPin);
    i2c.begin();
    i2c.setClock(BoardPinMap::I2cBus::clockHz);
#endif

    // 태스크 등록 순서는 실제 실행 순서에도 영향을 준다.
#if defined(NURA_MOCK_TELEMETRY)
    scheduler_.add(mockTelemetrySourceTask_);
#else
    scheduler_.add(imuTask_);
    scheduler_.add(highGImuTask_);
    scheduler_.add(magnetometerTask_);
    scheduler_.add(barometerTask_);
    scheduler_.add(gnssTask_);
#endif
    scheduler_.add(watchdogTask_);
    scheduler_.add(fsmTask_);
    scheduler_.add(telemetryTask_);
    scheduler_.add(loggerTask_);

    if (!scheduler_.init(nowMs))
    {
        // logger task가 돌기 전일 수 있으므로 치명적 실패는 즉시 정지하고 로그 버퍼를 flush한다.
        LOGE(logger_, nowMs, "fsm", "Initialization Failed");
        flushBootLogs();
        panicHandler_.panic();
    }

#if defined(NURA_MOCK_TELEMETRY) && defined(NURA_MOCK_AUTO_ARM)
    flightState_.state = State::ARMED;
    flightState_.stateEnteredMs = nowMs;
    LOGW(logger_, nowMs, "mock", "auto armed for mock flight replay");
#endif

    flushBootLogs();

    return true;
}

void FlightControllerApp::loop(uint32_t nowMs)
{
    // 메인 루프는 현재 시각만 넘기고, 주기 제어는 스케줄러가 담당한다.
    scheduler_.tick(nowMs);
}

void FlightControllerApp::flushBootLogs()
{
    // 로그 버퍼가 빌때까지 로그를 flush한다.
    while (!logger_.empty())
    {
        const LogFlushResult flushed = logger_.flushTo(logOutput_, Logger::kMaxBufferSize);
        if (flushed.drained == 0U)
        {
            break;
        }
    }
}
