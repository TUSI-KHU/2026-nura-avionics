#include "app/flight_controller_app.h"

#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>

#include "board_pinmap.h"
#include "nura_constants.h"

bool FlightControllerApp::setup(uint32_t nowMs)
{
    // 로그 출력용 시리얼을 먼저 연다.
    logOutput_.begin(config_.serialBaudRate());
    delay(NuraConstants::App::kBoardPowerSettleDelayMs);
    nowMs = millis();

    // 필수 로깅 저장소는 외부 센서 버스 초기화보다 먼저 잡아둔다. 현재 PCB에서
    // SDIO mount가 SPI/I2C 장치 bring-up 이후 간헐 실패하는 증상이 있어 순서를 고정한다.
#if !defined(NURA_BENCH_DISABLE_FLIGHT_LOG_TASK)
    (void)flightLogStorage_.begin();
#endif

    pinMode(BoardPinMap::LSM6DSO32::csPin, OUTPUT);
    pinMode(BoardPinMap::H3LIS331DL::csPin, OUTPUT);
    pinMode(BoardPinMap::Sx1262LoRa::ssPin, OUTPUT);
    digitalWrite(BoardPinMap::LSM6DSO32::csPin, HIGH);
    digitalWrite(BoardPinMap::H3LIS331DL::csPin, HIGH);
    digitalWrite(BoardPinMap::Sx1262LoRa::ssPin, HIGH);
    SPI.setMOSI(BoardPinMap::SpiBus::mosiPin);
    SPI.setMISO(BoardPinMap::SpiBus::misoPin);
    SPI.setSCK(BoardPinMap::SpiBus::sckPin);
    SPI.begin();
#if !defined(NURA_DISABLE_LORA)
    SPI1.setMISO(BoardPinMap::Spi1Bus::misoPin);
    SPI1.setMOSI(BoardPinMap::Spi1Bus::mosiPin);
    SPI1.setSCK(BoardPinMap::Spi1Bus::sckPin);
    SPI1.begin();
#if defined(NURA_BENCH_SX1262_RXE_LOW)
    if (BoardPinMap::Sx1262LoRa::rxEnablePin != BoardPinMap::kUnassignedPin)
    {
        pinMode(BoardPinMap::Sx1262LoRa::rxEnablePin, OUTPUT);
        digitalWrite(BoardPinMap::Sx1262LoRa::rxEnablePin, LOW);
    }
#endif
#endif
#if !defined(NURA_MOCK_TELEMETRY)
    TwoWire &i2c0 = BoardPinMap::I2c0Bus::wire();
    i2c0.setSDA(BoardPinMap::I2c0Bus::sdaPin);
    i2c0.setSCL(BoardPinMap::I2c0Bus::sclPin);
    i2c0.begin();
    i2c0.setClock(BoardPinMap::I2c0Bus::clockHz);

    TwoWire &i2c1 = BoardPinMap::I2c1Bus::wire();
    i2c1.setSDA(BoardPinMap::I2c1Bus::sdaPin);
    i2c1.setSCL(BoardPinMap::I2c1Bus::sclPin);
    i2c1.begin();
    i2c1.setClock(BoardPinMap::I2c1Bus::clockHz);
#endif
    delay(NuraConstants::App::kBusSettleDelayMs);
    nowMs = millis();

    // 태스크 등록 순서는 실제 실행 순서에도 영향을 준다.
#if defined(NURA_MOCK_TELEMETRY)
    scheduler_.add(mockTelemetrySourceTask_);
#else
    scheduler_.add(imuTask_);
    scheduler_.add(highGImuTask_);
    scheduler_.add(magnetometerTask_);
    scheduler_.add(barometerTask_);
    scheduler_.add(gnssTask_);
    scheduler_.add(powerSenseTask_);
#endif
    scheduler_.add(watchdogTask_);
    scheduler_.add(fsmTask_);
#if !defined(NURA_BENCH_DISABLE_FLIGHT_LOG_TASK)
    scheduler_.add(flightLogTask_);
#endif
#if !defined(NURA_DISABLE_LORA)
    scheduler_.add(telemetryTask_);
#endif
    scheduler_.add(loggerTask_);

    if (!scheduler_.init(nowMs))
    {
        // logger task가 돌기 전일 수 있으므로 치명적 실패는 즉시 정지하고 로그 버퍼를 flush한다.
        const char *failedTask = scheduler_.lastInitFailureTaskName();
        LOGE(logger_, nowMs, "fsm", "Initialization Failed");
        if (failedTask != nullptr)
        {
            LOGE(logger_, nowMs, "fsm", failedTask);
        }
        flushBootLogs();
        panicHandler_.panic(failedTask);
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
