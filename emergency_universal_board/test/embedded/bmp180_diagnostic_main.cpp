#include <Arduino.h>
#include <Wire.h>

#include "board_pinmap.h"
#include "hal/bmp180_hal.h"

namespace
{
constexpr uint8_t kBmp180ChipIdReg = 0xD0U;
constexpr uint8_t kBmp180ExpectedChipId = 0x55U;

bool readRegister(TwoWire &wire, uint8_t address, uint8_t reg, uint8_t &value)
{
    wire.beginTransmission(address);
    wire.write(reg);
    const uint8_t txStatus = wire.endTransmission(false);
    if (txStatus != 0U)
    {
        Serial.print("REG_TX_FAIL addr=0x");
        Serial.print(address, HEX);
        Serial.print(" reg=0x");
        Serial.print(reg, HEX);
        Serial.print(" status=");
        Serial.println(txStatus);
        return false;
    }

    if (wire.requestFrom(static_cast<int>(address), 1) != 1)
    {
        Serial.print("REG_RX_FAIL addr=0x");
        Serial.print(address, HEX);
        Serial.print(" reg=0x");
        Serial.println(reg, HEX);
        return false;
    }

    value = static_cast<uint8_t>(wire.read());
    return true;
}

void scanBus(TwoWire &wire)
{
    Serial.println("SCAN_BEGIN Wire 18/19");
    uint8_t count = 0U;
    for (uint8_t address = 1U; address < 127U; ++address)
    {
        wire.beginTransmission(address);
        const uint8_t error = wire.endTransmission();
        if (error == 0U)
        {
            Serial.print("I2C_FOUND addr=0x");
            if (address < 16U)
            {
                Serial.print('0');
            }
            Serial.println(address, HEX);
            ++count;
        }
        else if (error == 4U)
        {
            Serial.print("I2C_UNKNOWN_ERROR addr=0x");
            Serial.println(address, HEX);
        }
    }
    Serial.print("SCAN_DONE count=");
    Serial.println(count);
}

void probeChipIds(TwoWire &wire)
{
    constexpr uint8_t addresses[] = {0x76U, 0x77U};
    for (const uint8_t address : addresses)
    {
        uint8_t chipId = 0U;
        Serial.print("PROBE addr=0x");
        Serial.print(address, HEX);
        Serial.print(' ');
        if (readRegister(wire, address, kBmp180ChipIdReg, chipId))
        {
            Serial.print("chip_id=0x");
            Serial.print(chipId, HEX);
            if (chipId == kBmp180ExpectedChipId)
            {
                Serial.print(" BMP180_OK");
            }
            else if (chipId == 0x58U)
            {
                Serial.print(" LOOKS_LIKE_BMP280");
            }
            else if (chipId == 0x60U)
            {
                Serial.print(" LOOKS_LIKE_BME280");
            }
            Serial.println();
        }
        else
        {
            Serial.println("no_reg_read");
        }
    }
}

void readBmp180(TwoWire &wire)
{
    BMP180HAL bmp;
    const uint8_t address = BoardPinMap::BMP180::i2cAddress;
    Serial.print("BMP180_BEGIN addr=0x");
    Serial.println(address, HEX);
    if (!bmp.begin(wire, address))
    {
        Serial.println("BMP180_BEGIN_FAIL");
        return;
    }

    Serial.println("BMP180_BEGIN_OK");
    for (uint8_t i = 0U; i < 20U; ++i)
    {
        Bmp180Reading reading;
        if (bmp.read(reading, millis()))
        {
            Serial.print("BMP180_READ_OK pressure_pa=");
            Serial.print(reading.pressurePa, 2);
            Serial.print(" pressure_hpa=");
            Serial.print(reading.pressureHpa, 2);
            Serial.print(" temp_c=");
            Serial.print(reading.temperatureC, 2);
            Serial.print(" sample_ms=");
            Serial.println(reading.sampleMs);
        }
        else
        {
            Serial.println("BMP180_READ_FAIL");
        }
        delay(500);
    }
}
} // namespace

void setup()
{
    Serial.begin(115200);
    const uint32_t serialStartMs = millis();
    while (!Serial && (millis() - serialStartMs) < 3000UL)
    {
    }

    Serial.println();
    Serial.println("BMP180_DIAGNOSTIC_BEGIN");
    Serial.print("SDA=");
    Serial.print(BoardPinMap::BMP180::sdaPin);
    Serial.print(" SCL=");
    Serial.print(BoardPinMap::BMP180::sclPin);
    Serial.print(" EXPECTED_ADDR=0x");
    Serial.println(BoardPinMap::BMP180::i2cAddress, HEX);

    TwoWire &wire = BoardPinMap::BMP180::wire();
    wire.setSDA(BoardPinMap::BMP180::sdaPin);
    wire.setSCL(BoardPinMap::BMP180::sclPin);
    wire.begin();
    wire.setClock(BoardPinMap::I2c0Bus::clockHz);
    delay(250);

    scanBus(wire);
    probeChipIds(wire);
    readBmp180(wire);
    Serial.println("BMP180_DIAGNOSTIC_SETUP_DONE");
}

void loop()
{
    static uint32_t lastRunMs = 0UL;
    const uint32_t nowMs = millis();
    if (lastRunMs == 0UL || (nowMs - lastRunMs) >= 5000UL)
    {
        lastRunMs = nowMs;
        Serial.println("BMP180_DIAGNOSTIC_REPEAT");
        TwoWire &wire = BoardPinMap::BMP180::wire();
        scanBus(wire);
        probeChipIds(wire);
        readBmp180(wire);
        Serial.println("BMP180_DIAGNOSTIC_REPEAT_DONE");
    }
    delay(100);
}
