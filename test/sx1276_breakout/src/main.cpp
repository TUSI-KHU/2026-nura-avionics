#include <Arduino.h>
#include <RadioLib.h>
#include <SPI.h>

namespace
{
constexpr uint8_t kMiso = 1U;
constexpr uint8_t kMosi = 26U;
constexpr uint8_t kSck = 27U;
constexpr uint8_t kNss = 9U;
constexpr uint8_t kRst = 24U;
constexpr uint8_t kRxe = 30U;
constexpr uint8_t kTxe = 31U;
constexpr uint8_t kDio0 = 32U;

// Match these settings on the second radio used for the RF test.
constexpr float kFrequencyMHz = 920.9F;
constexpr uint8_t kSpreadingFactor = 7U;
constexpr uint8_t kCodingRate = 5U;
constexpr uint8_t kSyncWord = 0x12U;
constexpr int8_t kTxPowerDbm = 2;

Module radioModule(kNss,
                   kDio0,
                   kRst,
                   RADIOLIB_NC,
                   SPI1,
                   SPISettings(1000000UL, MSBFIRST, SPI_MODE0));
SX1276 radio(&radioModule);

volatile bool receivedFlag = false;
bool radioReady = false;
uint32_t txCount = 0UL;
uint32_t rxCount = 0UL;

void onPacketReceived()
{
    receivedFlag = true;
}

void clearReceivedFlag()
{
    noInterrupts();
    receivedFlag = false;
    interrupts();
}

bool startReceive()
{
    clearReceivedFlag();
    radio.setPacketReceivedAction(onPacketReceived);
    const int16_t state = radio.startReceive();
    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.print("FAIL: startReceive state=");
        Serial.println(state);
        return false;
    }
    Serial.println("PASS: RX mode (RXE=1 TXE=0)");
    return true;
}

void sendOne()
{
    if (!radioReady)
    {
        Serial.println("FAIL: radio not initialized");
        return;
    }

    char message[40];
    snprintf(message, sizeof(message), "sx1276-test-%lu", static_cast<unsigned long>(txCount));
    radio.clearPacketReceivedAction();
    clearReceivedFlag();

    Serial.print("tx_start data=");
    Serial.println(message);
    const int16_t state = radio.transmit(message);
    if (state == RADIOLIB_ERR_NONE)
    {
        ++txCount;
        Serial.println("PASS: TX complete (RXE=0 TXE=1 during TX)");
    }
    else
    {
        Serial.print("FAIL: transmit state=");
        Serial.println(state);
    }
    if (!startReceive())
    {
        radioReady = false;
    }
}

void serviceReceive()
{
    if (!receivedFlag)
    {
        return;
    }
    clearReceivedFlag();

    String payload;
    const int16_t state = radio.readData(payload);
    if (state == RADIOLIB_ERR_NONE)
    {
        ++rxCount;
        Serial.print("PASS: RX data=");
        Serial.print(payload);
        Serial.print(" rssi_dbm=");
        Serial.print(radio.getRSSI(), 1);
        Serial.print(" snr_db=");
        Serial.println(radio.getSNR(), 1);
    }
    else if (state == RADIOLIB_ERR_CRC_MISMATCH)
    {
        Serial.println("FAIL: RX CRC mismatch");
    }
    else
    {
        Serial.print("FAIL: readData state=");
        Serial.println(state);
    }
}

void printHelp()
{
    Serial.println("commands: t=one TX, s=status, h=help");
}

void serviceSerial()
{
    while (Serial.available() > 0)
    {
        const char command = static_cast<char>(Serial.read());
        if (command == 't' || command == 'T')
        {
            sendOne();
        }
        else if (command == 's' || command == 'S')
        {
            Serial.print("status ready=");
            Serial.print(radioReady ? 1 : 0);
            Serial.print(" tx=");
            Serial.print(txCount);
            Serial.print(" rx=");
            Serial.print(rxCount);
            Serial.print(" rxe=");
            Serial.print(digitalRead(kRxe));
            Serial.print(" txe=");
            Serial.println(digitalRead(kTxe));
        }
        else if (command == 'h' || command == 'H' || command == '?')
        {
            printHelp();
        }
    }
}
} // namespace

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 3000UL)
    {
    }

    Serial.println("SparkFun SPX-18572 / E19-915M30S SX1276 1W test / Teensy 4.1");
    Serial.println("SPI1: MISO=1 MOSI=26 SCK=27; NSS=9 RST=24 DIO0=32 RXE=30 TXE=31");
    Serial.println("RF power rail: 5 V; Teensy SPI/GPIO logic: 3.3 V; bench TX drive: 2 dBm");
    Serial.println("RXE/TXE are RF-switch enables, not UART pins.");
    Serial.println("Antenna and active-high RXE/TXE polarity must match the breakout schematic.");

    pinMode(kRxe, OUTPUT);
    pinMode(kTxe, OUTPUT);
    digitalWrite(kRxe, LOW);
    digitalWrite(kTxe, LOW);

    SPI1.setMISO(kMiso);
    SPI1.setMOSI(kMosi);
    SPI1.setSCK(kSck);
    SPI1.begin();

    ConfigLoRa_t config;
    config.frequency = kFrequencyMHz;
    config.bandwidth = 125.0F;
    config.spreadingFactor = kSpreadingFactor;
    config.codingRate = kCodingRate;
    config.syncWord = kSyncWord;
    config.power = kTxPowerDbm;
    config.preambleLength = 8U;

    const int16_t state = radio.begin(config);
    if (state != RADIOLIB_ERR_NONE)
    {
        Serial.print("FAIL: SX1276 init state=");
        Serial.println(state);
        if (state == RADIOLIB_ERR_CHIP_NOT_FOUND)
        {
            Serial.println("CHECK: 3.3V/GND and SPI1/NSS/RST wiring");
        }
        printHelp();
        return;
    }

    // RadioLib assumes active-high RXE/TXE: RX=(1,0), TX=(0,1), idle=(0,0).
    radio.setRfSwitchPins(kRxe, kTxe);
    radioReady = startReceive();
    Serial.println("RF: 920.9 MHz BW125 SF7 CR4/5 sync=0x12 CRC on TX=2 dBm");
    printHelp();
}

void loop()
{
    serviceReceive();
    serviceSerial();
}
