#include <Arduino.h>
#include <SPI.h>
#include "board_pinmap.h"
#define private public
#include <LoRa.h>
#undef private

// ==================== PIN MAP / USER CONFIG ====================
#define SERIAL_BAUD 115200
// Teensy 4.1 default SPI pins:
// MOSI=D11, MISO=D12, SCK=D13. Wire the LoRa module to these pins.
#define LORA_MOSI_PIN BoardPinMap::SpiBus::mosiPin
#define LORA_MISO_PIN BoardPinMap::SpiBus::misoPin
#define LORA_SCK_PIN BoardPinMap::SpiBus::sckPin
#define LORA_SS_PIN BoardPinMap::Ra01DevelopmentLoRa::ssPin
#define LORA_RESET_PIN BoardPinMap::Ra01DevelopmentLoRa::resetPin
#define LORA_LIBRARY_RESET_PIN BoardPinMap::Ra01DevelopmentLoRa::libraryResetPin
#define LORA_DIO0_PIN BoardPinMap::Ra01DevelopmentLoRa::dio0Pin
// RA-01 / SX1278 development modules are commonly 433 MHz.
// Use 915000000L for RFM95W/RFM96W 915 MHz hardware.
#define LORA_FREQUENCY_HZ 433000000L
#define LORA_SPI_FREQUENCY_HZ 125000UL
#define LORA_SPI_MODE SPI_MODE1
#define LORA_TX_POWER_DBM 17
#define LORA_SPREADING_FACTOR 7
#define LORA_SIGNAL_BANDWIDTH_HZ 125000L
#define LORA_CODING_RATE_DENOMINATOR 5
#define LORA_SYNC_WORD 0x12
#define LORA_SEND_TEST_PACKET 1
#define LORA_DIAG_SPI_FREQUENCY_HZ 125000UL
// ================================================================

#define LORA_REG_VERSION 0x42
#define LORA_EXPECTED_VERSION 0x12
#define LORA_INIT_ATTEMPTS 5

uint32_t packetCounter = 0;
uint32_t lastTxMs = 0;
bool radioReady = false;
uint8_t lastMode0Version = 0;
uint8_t lastMode1Version = 0;

uint8_t readLoraRegisterRaw(uint8_t address, uint8_t spiMode)
{
    SPISettings settings(LORA_DIAG_SPI_FREQUENCY_HZ, MSBFIRST, spiMode);

    pinMode(LORA_SS_PIN, OUTPUT);
    digitalWrite(LORA_SS_PIN, HIGH);

    SPI.beginTransaction(settings);
    digitalWrite(LORA_SS_PIN, LOW);
    delayMicroseconds(20);
    SPI.transfer(address & 0x7F);
    uint8_t value = SPI.transfer(0x00);
    delayMicroseconds(20);
    digitalWrite(LORA_SS_PIN, HIGH);
    SPI.endTransaction();

    return value;
}

void printHexByte(uint8_t value)
{
    Serial.print("0x");
    if (value < 16)
    {
        Serial.print('0');
    }
    Serial.print(value, HEX);
}

void runSpiModeDiagnostic()
{
    for (uint8_t attempt = 0; attempt < 5; ++attempt)
    {
        lastMode0Version = readLoraRegisterRaw(LORA_REG_VERSION, SPI_MODE0);
        lastMode1Version = readLoraRegisterRaw(LORA_REG_VERSION, SPI_MODE1);
        if (lastMode0Version == LORA_EXPECTED_VERSION || lastMode1Version == LORA_EXPECTED_VERSION)
        {
            break;
        }
        delay(50);
    }

    Serial.print("diag_reg_version_mode0=");
    printHexByte(lastMode0Version);
    Serial.print(" mode1=");
    printHexByte(lastMode1Version);
    Serial.println();

    if (lastMode0Version != LORA_EXPECTED_VERSION && lastMode1Version == LORA_EXPECTED_VERSION)
    {
        Serial.println("DIAG: radio answers only with SPI_MODE1; check level shifting/wiring/SCK-MISO timing");
    }
    else if (lastMode0Version != LORA_EXPECTED_VERSION)
    {
        Serial.println("DIAG: expected SX127x RegVersion 0x12; 0x00/0xFF usually means NSS/MISO/MOSI/SCK or power wiring");
    }
}

void resetRadioForDiagnostic()
{
    pinMode(LORA_SS_PIN, OUTPUT);
    digitalWrite(LORA_SS_PIN, HIGH);
    pinMode(LORA_RESET_PIN, OUTPUT);
    digitalWrite(LORA_RESET_PIN, LOW);
    delay(50);
    digitalWrite(LORA_RESET_PIN, HIGH);
    delay(500);
}

void primeLoraSpiSettings()
{
    SPISettings settings(LORA_SPI_FREQUENCY_HZ, MSBFIRST, SPI_MODE0);

    digitalWrite(LORA_SS_PIN, HIGH);
    SPI.beginTransaction(settings);
    SPI.endTransaction();
}

bool configureRadio()
{
    if (LORA_SPREADING_FACTOR < 6 || LORA_SPREADING_FACTOR > 12)
    {
        Serial.println("FAIL: invalid spreading factor macro");
        return false;
    }

    if (LORA_CODING_RATE_DENOMINATOR < 5 || LORA_CODING_RATE_DENOMINATOR > 8)
    {
        Serial.println("FAIL: invalid coding rate denominator macro");
        return false;
    }

    LoRa.setPins(LORA_SS_PIN, LORA_LIBRARY_RESET_PIN, LORA_DIO0_PIN);
    LoRa.setSPIFrequency(LORA_SPI_FREQUENCY_HZ);
    LoRa._spiSettings = SPISettings(LORA_SPI_FREQUENCY_HZ, MSBFIRST, LORA_SPI_MODE);

    for (uint8_t attempt = 1; attempt <= LORA_INIT_ATTEMPTS; ++attempt)
    {
        Serial.print("init_attempt=");
        Serial.println(attempt);

        SPI.begin();
        LoRa._spiSettings = SPISettings(LORA_SPI_FREQUENCY_HZ, MSBFIRST, LORA_SPI_MODE);
        resetRadioForDiagnostic();
        runSpiModeDiagnostic();
        primeLoraSpiSettings();
        LoRa._spiSettings = SPISettings(LORA_SPI_FREQUENCY_HZ, MSBFIRST, LORA_SPI_MODE);

        Serial.print("library_reg_version=");
        printHexByte(LoRa.readRegister(LORA_REG_VERSION));
        Serial.println();

        if (lastMode1Version == LORA_EXPECTED_VERSION && LoRa.begin(LORA_FREQUENCY_HZ))
        {
            break;
        }

        LoRa.end();
        delay(250);

        if (attempt == LORA_INIT_ATTEMPTS)
        {
            Serial.println("FAIL: SX127x/RFM9x/RA-01 not found over SPI");
            Serial.println("CHECK: Teensy 4.1 wiring must be NSS=D10, RST=D9, DIO0=D2, MOSI=D11, MISO=D12, SCK=D13");
            return false;
        }
    }

    LoRa.setTxPower(LORA_TX_POWER_DBM);
    LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
    LoRa.setSignalBandwidth(LORA_SIGNAL_BANDWIDTH_HZ);
    LoRa.setCodingRate4(LORA_CODING_RATE_DENOMINATOR);
    LoRa.setSyncWord(LORA_SYNC_WORD);
    LoRa.enableCrc();

    Serial.println("PASS: LoRa radio init OK");
    Serial.print("idle_rssi=");
    Serial.println(LoRa.rssi());
    return true;
}

void printPinMap()
{
    Serial.print("pinmap: nss=");
    Serial.print(LORA_SS_PIN);
    Serial.print(" rst=");
    Serial.print(LORA_RESET_PIN);
    Serial.print(" dio0=");
    Serial.print(LORA_DIO0_PIN);
    Serial.print(" mosi=");
    Serial.print(LORA_MOSI_PIN);
    Serial.print(" miso=");
    Serial.print(LORA_MISO_PIN);
    Serial.print(" sck=");
    Serial.print(LORA_SCK_PIN);
    Serial.print(" freq_hz=");
    Serial.println(LORA_FREQUENCY_HZ);
}

void receivePacketIfAvailable()
{
    const int packetSize = LoRa.parsePacket();
    if (packetSize <= 0)
    {
        return;
    }

    Serial.print("rx_len=");
    Serial.print(packetSize);
    Serial.print(" rssi=");
    Serial.print(LoRa.packetRssi());
    Serial.print(" snr=");
    Serial.print(LoRa.packetSnr());
    Serial.print(" data=");

    while (LoRa.available())
    {
        Serial.write(static_cast<uint8_t>(LoRa.read()));
    }
    Serial.println();
}

#if LORA_SEND_TEST_PACKET
bool sendPacket()
{
    char message[48];
    snprintf(message, sizeof(message), "nura-lora-test %lu", static_cast<unsigned long>(packetCounter++));

    if (!LoRa.beginPacket())
    {
        Serial.println("FAIL: beginPacket failed");
        return false;
    }

    LoRa.print(message);
    if (LoRa.endPacket() != 1)
    {
        Serial.println("FAIL: endPacket failed");
        return false;
    }

    Serial.print("tx=");
    Serial.println(message);
    return true;
}
#endif

void setup()
{
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 4000)
    {
    }

    Serial.println();
    Serial.println("SX1278 / RA-01 LoRa defect test on Teensy 4.1");
    printPinMap();

    if (!configureRadio())
    {
        return;
    }

    radioReady = true;

#if LORA_SEND_TEST_PACKET
    sendPacket();
#else
    Serial.println("PASS: register-level radio bring-up complete");
    Serial.println("INFO: set LORA_SEND_TEST_PACKET to 1 to transmit heartbeat packets");
#endif
}

void loop()
{
    if (!radioReady)
    {
        return;
    }

    receivePacketIfAvailable();

#if LORA_SEND_TEST_PACKET
    if ((millis() - lastTxMs) >= 2000UL)
    {
        lastTxMs = millis();
        sendPacket();
    }
#endif
}
