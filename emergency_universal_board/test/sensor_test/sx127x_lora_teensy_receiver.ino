#include <Arduino.h>
#include <SPI.h>
#include "board_pinmap.h"
#define private public
#include <LoRa.h>
#undef private

#define SERIAL_BAUD 115200
#define LORA_MOSI_PIN BoardPinMap::SpiBus::mosiPin
#define LORA_MISO_PIN BoardPinMap::SpiBus::misoPin
#define LORA_SCK_PIN BoardPinMap::SpiBus::sckPin
#define LORA_SS_PIN BoardPinMap::Ra01DevelopmentLoRa::ssPin
#define LORA_RESET_PIN BoardPinMap::Ra01DevelopmentLoRa::resetPin
#define LORA_LIBRARY_RESET_PIN BoardPinMap::Ra01DevelopmentLoRa::libraryResetPin
#define LORA_DIO0_PIN BoardPinMap::Ra01DevelopmentLoRa::dio0Pin

#define LORA_FREQUENCY_HZ 433000000L
#define LORA_SPI_FREQUENCY_HZ 125000UL
#define LORA_TX_POWER_DBM 17
#define LORA_SPREADING_FACTOR 7
#define LORA_SIGNAL_BANDWIDTH_HZ 125000L
#define LORA_CODING_RATE_DENOMINATOR 5
#define LORA_SYNC_WORD 0x12

#define LORA_REG_VERSION 0x42
#define LORA_EXPECTED_VERSION 0x12
#define LORA_INIT_ATTEMPTS 5

uint8_t selectedSpiMode = SPI_MODE0;
uint8_t selectedSpiModeNumber = 0;
bool radioReady = false;

static void printPinmap()
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

void beginSpi()
{
#if defined(CORE_TEENSY)
    SPI.setMOSI(LORA_MOSI_PIN);
    SPI.setMISO(LORA_MISO_PIN);
    SPI.setSCK(LORA_SCK_PIN);
#endif
    SPI.begin();
}

uint8_t readLoraRegisterRaw(uint8_t address, uint8_t spiMode)
{
    SPISettings settings(LORA_SPI_FREQUENCY_HZ, MSBFIRST, spiMode);

    pinMode(LORA_SS_PIN, OUTPUT);
    digitalWrite(LORA_SS_PIN, HIGH);
    SPI.beginTransaction(settings);
    digitalWrite(LORA_SS_PIN, LOW);
    delayMicroseconds(20);
    SPI.transfer(address & 0x7F);
    const uint8_t value = SPI.transfer(0x00);
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

void resetRadio()
{
    pinMode(LORA_SS_PIN, OUTPUT);
    digitalWrite(LORA_SS_PIN, HIGH);
    pinMode(LORA_RESET_PIN, OUTPUT);
    digitalWrite(LORA_RESET_PIN, LOW);
    delay(50);
    digitalWrite(LORA_RESET_PIN, HIGH);
    delay(500);
}

bool beginRadio()
{
    LoRa.setPins(LORA_SS_PIN, LORA_LIBRARY_RESET_PIN, LORA_DIO0_PIN);
    LoRa.setSPIFrequency(LORA_SPI_FREQUENCY_HZ);

    for (uint8_t attempt = 1; attempt <= LORA_INIT_ATTEMPTS; ++attempt)
    {
        beginSpi();
        resetRadio();

        const uint8_t mode0Version = readLoraRegisterRaw(LORA_REG_VERSION, SPI_MODE0);
        const uint8_t mode1Version = readLoraRegisterRaw(LORA_REG_VERSION, SPI_MODE1);
        const uint8_t mode2Version = readLoraRegisterRaw(LORA_REG_VERSION, SPI_MODE2);
        const uint8_t mode3Version = readLoraRegisterRaw(LORA_REG_VERSION, SPI_MODE3);

        Serial.print("init_attempt=");
        Serial.print(attempt);
        Serial.print(" m0=");
        printHexByte(mode0Version);
        Serial.print(" m1=");
        printHexByte(mode1Version);
        Serial.print(" m2=");
        printHexByte(mode2Version);
        Serial.print(" m3=");
        printHexByte(mode3Version);
        Serial.println();

        if (mode1Version == LORA_EXPECTED_VERSION)
        {
            selectedSpiMode = SPI_MODE1;
            selectedSpiModeNumber = 1;
        }
        else if (mode0Version == LORA_EXPECTED_VERSION)
        {
            selectedSpiMode = SPI_MODE0;
            selectedSpiModeNumber = 0;
        }
        else if (mode2Version == LORA_EXPECTED_VERSION)
        {
            selectedSpiMode = SPI_MODE2;
            selectedSpiModeNumber = 2;
        }
        else if (mode3Version == LORA_EXPECTED_VERSION)
        {
            selectedSpiMode = SPI_MODE3;
            selectedSpiModeNumber = 3;
        }
        else
        {
            delay(250);
            continue;
        }

        LoRa._spiSettings = SPISettings(LORA_SPI_FREQUENCY_HZ, MSBFIRST, selectedSpiMode);
        Serial.print("selected_spi_mode=");
        Serial.println(selectedSpiModeNumber);
        Serial.print("library_reg_version=");
        printHexByte(LoRa.readRegister(LORA_REG_VERSION));
        Serial.println();

        if (LoRa.begin(LORA_FREQUENCY_HZ))
        {
            LoRa._spiSettings = SPISettings(LORA_SPI_FREQUENCY_HZ, MSBFIRST, selectedSpiMode);
            LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
            LoRa.setSignalBandwidth(LORA_SIGNAL_BANDWIDTH_HZ);
            LoRa.setCodingRate4(LORA_CODING_RATE_DENOMINATOR);
            LoRa.setSyncWord(LORA_SYNC_WORD);
            LoRa.enableCrc();
            LoRa.receive();
            return true;
        }

        LoRa.end();
        delay(250);
    }

    return false;
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

void setup()
{
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 4000)
    {
    }

    Serial.println();
    Serial.println("SX1278 LoRa receiver on Teensy 4.1");
    Serial.println("role=rx board=teensy41");
    printPinmap();

    radioReady = beginRadio();
    if (!radioReady)
    {
        Serial.println("FAIL: receiver radio init failed");
        return;
    }

    Serial.println("PASS: receiver radio init OK");
}

void loop()
{
    if (!radioReady)
    {
        return;
    }

    receivePacketIfAvailable();
}
