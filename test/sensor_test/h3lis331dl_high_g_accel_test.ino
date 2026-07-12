/*
  H3LIS331DL high-g accelerometer SPI standalone test

  Pin map:
    H3LIS331DL 3.3V     -> 3.3V
    H3LIS331DL GND      -> GND
    H3LIS331DL SCL/SPC  -> SPI SCK  (Teensy/Nano D13)
    H3LIS331DL SDA/SDI  -> SPI MOSI (Teensy/Nano D11)
    H3LIS331DL SA0/SDO  -> SPI MISO (Teensy/Nano D12)
    H3LIS331DL CS       -> D10

  Voltage note:
    H3LIS331DL is a 2.2V to 3.6V device. Do not power it from 5V.
    With a 5V Arduino Nano, level shifting may be required on SCK, MOSI, and CS.
*/

#include <Arduino.h>
#include <SPI.h>

namespace
{
const uint8_t kCsPin = 10U;

const uint8_t kRegWhoAmI = 0x0FU;
const uint8_t kRegCtrl1 = 0x20U;
const uint8_t kRegCtrl4 = 0x23U;
const uint8_t kRegOutXL = 0x28U;

const uint8_t kSpiRead = 0x80U;
const uint8_t kSpiAutoIncrement = 0x40U;
const uint8_t kExpectedWhoAmI = 0x32U;

const uint8_t kCtrlReg1EnableXyz50Hz = 0x27U;
const uint8_t kCtrlReg4BlockUpdateRange100G = 0x80U;

const float kScale100G = 0.049f;

SPISettings h3lisSettings(1000000UL, MSBFIRST, SPI_MODE3);

void selectSensor()
{
    digitalWrite(kCsPin, LOW);
}

void deselectSensor()
{
    digitalWrite(kCsPin, HIGH);
}

uint8_t readRegister(uint8_t reg)
{
    SPI.beginTransaction(h3lisSettings);
    selectSensor();
    SPI.transfer(reg | kSpiRead);
    const uint8_t value = SPI.transfer(0x00U);
    deselectSensor();
    SPI.endTransaction();
    return value;
}

void writeRegister(uint8_t reg, uint8_t value)
{
    SPI.beginTransaction(h3lisSettings);
    selectSensor();
    SPI.transfer(reg);
    SPI.transfer(value);
    deselectSensor();
    SPI.endTransaction();
}

void readBurst(uint8_t startReg, uint8_t *buffer, size_t length)
{
    SPI.beginTransaction(h3lisSettings);
    selectSensor();
    SPI.transfer(startReg | kSpiRead | kSpiAutoIncrement);
    for (size_t i = 0; i < length; ++i)
    {
        buffer[i] = SPI.transfer(0x00U);
    }
    deselectSensor();
    SPI.endTransaction();
}

int16_t makeRaw12(uint8_t lo, uint8_t hi)
{
    const int16_t raw16 = static_cast<int16_t>(
        (static_cast<uint16_t>(hi) << 8) | static_cast<uint16_t>(lo));
    return static_cast<int16_t>(raw16 >> 4);
}
}

void setup()
{
    Serial.begin(115200);
    while (!Serial && millis() < 4000)
    {
    }

    pinMode(kCsPin, OUTPUT);
    deselectSensor();
    SPI.begin();

    const uint8_t whoAmI = readRegister(kRegWhoAmI);
    Serial.print("WHO_AM_I = 0x");
    if (whoAmI < 0x10U)
    {
        Serial.print('0');
    }
    Serial.println(whoAmI, HEX);

    if (whoAmI != kExpectedWhoAmI)
    {
        Serial.println("WARNING: WHO_AM_I mismatch. Check wiring, CS pin, SPI mode, and sensor voltage/level shifting.");
    }

    writeRegister(kRegCtrl1, kCtrlReg1EnableXyz50Hz);
    writeRegister(kRegCtrl4, kCtrlReg4BlockUpdateRange100G);
}

void loop()
{
    uint8_t buffer[6];
    readBurst(kRegOutXL, buffer, sizeof(buffer));

    const int16_t rawX = makeRaw12(buffer[0], buffer[1]);
    const int16_t rawY = makeRaw12(buffer[2], buffer[3]);
    const int16_t rawZ = makeRaw12(buffer[4], buffer[5]);

    const float accelXG = static_cast<float>(rawX) * kScale100G;
    const float accelYG = static_cast<float>(rawY) * kScale100G;
    const float accelZG = static_cast<float>(rawZ) * kScale100G;

    Serial.print("raw X/Y/Z = ");
    Serial.print(rawX);
    Serial.print(", ");
    Serial.print(rawY);
    Serial.print(", ");
    Serial.print(rawZ);
    Serial.print(" | g X/Y/Z = ");
    Serial.print(accelXG, 3);
    Serial.print(", ");
    Serial.print(accelYG, 3);
    Serial.print(", ");
    Serial.println(accelZG, 3);

    delay(200);
}
