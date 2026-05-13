#include <Arduino.h>
#include <SPI.h>
#define private public
#include <LoRa.h>
#undef private

#define SERIAL_BAUD 115200
#define LORA_MOSI_PIN 11
#define LORA_MISO_PIN 12
#define LORA_SCK_PIN 13
#define LORA_SS_PIN 10
#define LORA_RESET_PIN 9
#define LORA_LIBRARY_RESET_PIN -1
#define LORA_DIO0_PIN 2

#define LORA_FREQUENCY_HZ 433000000L
#define LORA_SPI_FREQUENCY_HZ 125000UL
#define LORA_TX_POWER_DBM 10
#define LORA_SPREADING_FACTOR 7
#define LORA_SIGNAL_BANDWIDTH_HZ 125000L
#define LORA_CODING_RATE_DENOMINATOR 5
#define LORA_SYNC_WORD 0x12
#define LORA_TX_INTERVAL_MS 2000UL

#define LORA_REG_VERSION 0x42
#define LORA_EXPECTED_VERSION 0x12
#define LORA_INIT_ATTEMPTS 5
#define LORA_TX_ATTEMPTS 3

uint32_t packetCounter = 0;
uint32_t lastTxMs = 0;
bool radioReady = false;
uint8_t selectedSpiMode = SPI_MODE0;
uint8_t selectedSpiModeNumber = 0;

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
            LoRa.setTxPower(LORA_TX_POWER_DBM);
            LoRa.setSpreadingFactor(LORA_SPREADING_FACTOR);
            LoRa.setSignalBandwidth(LORA_SIGNAL_BANDWIDTH_HZ);
            LoRa.setCodingRate4(LORA_CODING_RATE_DENOMINATOR);
            LoRa.setSyncWord(LORA_SYNC_WORD);
            LoRa.enableCrc();
            return true;
        }

        LoRa.end();
        delay(250);
    }

    return false;
}

bool transmitWithLibrary(const char *message)
{
    LoRa._spiSettings = SPISettings(LORA_SPI_FREQUENCY_HZ, MSBFIRST, selectedSpiMode);
    LoRa.idle();
    delay(5);

    if (!LoRa.beginPacket())
    {
        return false;
    }

    LoRa.print(message);
    if (LoRa.endPacket() != 1)
    {
        return false;
    }

    LoRa.idle();
    return true;
}

bool recoverRadio()
{
    Serial.println("WARN: sender tx busy/stuck, resetting radio");
    LoRa.end();
    delay(50);
    radioReady = beginRadio();
    if (!radioReady)
    {
        Serial.println("FAIL: sender radio recovery failed");
        return false;
    }
    Serial.println("PASS: sender radio recovery OK");
    return true;
}

bool sendPacket()
{
    char message[48];
    snprintf(message, sizeof(message), "nura-lora %lu", static_cast<unsigned long>(packetCounter));

    for (uint8_t attempt = 1; attempt <= LORA_TX_ATTEMPTS; ++attempt)
    {
        if (transmitWithLibrary(message))
        {
            ++packetCounter;
            Serial.print("tx=");
            Serial.println(message);
            return true;
        }

        Serial.print("WARN: tx attempt failed attempt=");
        Serial.print(attempt);
        Serial.print(" message=");
        Serial.println(message);

        if (attempt < LORA_TX_ATTEMPTS && !recoverRadio())
        {
            break;
        }
    }

    Serial.print("FAIL: sender tx failed message=");
    Serial.println(message);
    return false;
}

void setup()
{
    Serial.begin(SERIAL_BAUD);
    while (!Serial && millis() < 4000)
    {
    }

    Serial.println();
    Serial.println("SX1278 LoRa sender on Teensy 4.1");
    Serial.println("role=tx board=teensy41");
    Serial.println("pinmap: nss=10 rst=9 dio0=2 mosi=11 miso=12 sck=13 freq_hz=433000000");

    radioReady = beginRadio();
    if (!radioReady)
    {
        Serial.println("FAIL: sender radio init failed");
        return;
    }

    Serial.println("PASS: sender radio init OK");
    sendPacket();
}

void loop()
{
    if (!radioReady)
    {
        return;
    }

    if ((millis() - lastTxMs) >= LORA_TX_INTERVAL_MS)
    {
        lastTxMs = millis();
        sendPacket();
    }
}
