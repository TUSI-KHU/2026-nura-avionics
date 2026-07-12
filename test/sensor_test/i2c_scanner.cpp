// --------------------------------------
// i2c_scanner
//
// Version 1
//    This program (or code that looks like it)
//    can be found in many places.
//    For example on the Arduino.cc forum.
//    The original author is not know.
// Version 2, Juni 2012, Using Arduino 1.0.1
//     Adapted to be as simple as possible by Arduino.cc user Krodal
// Version 3, Feb 26  2013
//    V3 by louarnold
// Version 4, March 3, 2013, Using Arduino 1.0.3
//    by Arduino.cc user Krodal.
//    Changes by louarnold removed.
//    Scanning addresses changed from 0...127 to 1...119,
//    according to the i2c scanner by Nick Gammon
//    http://www.gammon.com.au/forum/?id=10896
// Version 5, March 28, 2013
//    As version 4, but address scans now to 127.
//    A sensor seems to use address 120.
// Version 6, November 27, 2015.
//    Added waiting for the Leonardo serial communication.
//
//
// This sketch tests the standard 7-bit addresses
// Devices with higher bit address might not be seen properly.
//

#include <Wire.h>

#include "board_pinmap.h"

struct ScanBus
{
  TwoWire *wire;
  const char *name;
  uint8_t sdaPin;
  uint8_t sclPin;
};

static ScanBus buses[] = {
    {&Wire, "Wire 18/19 MPL3115A2", BoardPinMap::I2c0Bus::sdaPin, BoardPinMap::I2c0Bus::sclPin},
    {&Wire1, "Wire1 17/16 LIS3MDL", BoardPinMap::I2c1Bus::sdaPin, BoardPinMap::I2c1Bus::sclPin},
};

static void scanBus(ScanBus &bus)
{
  byte error, address;
  int nDevices = 0;

  Serial.print("Scanning ");
  Serial.print(bus.name);
  Serial.print(" SDA=");
  Serial.print(bus.sdaPin);
  Serial.print(" SCL=");
  Serial.println(bus.sclPin);

  for (address = 1; address < 127; address++)
  {
    bus.wire->beginTransmission(address);
    error = bus.wire->endTransmission();

    if (error == 0)
    {
      Serial.print("I2C device found at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.print(address, HEX);
      Serial.println("  !");

      nDevices++;
    }
    else if (error == 4)
    {
      Serial.print("Unknown error at address 0x");
      if (address < 16)
        Serial.print("0");
      Serial.println(address, HEX);
    }
  }
  if (nDevices == 0)
    Serial.println("No I2C devices found\n");
  else
    Serial.println("done\n");
}

void setup()
{
  for (ScanBus &bus : buses)
  {
    bus.wire->setSDA(bus.sdaPin);
    bus.wire->setSCL(bus.sclPin);
    bus.wire->begin();
    bus.wire->setClock(100000UL);
  }

  Serial.begin(9600);
  while (!Serial);             // Leonardo: wait for serial monitor
  Serial.println("\nI2C Scanner Wire 18/19 + Wire1 17/16");
}


void loop()
{
  for (ScanBus &bus : buses)
  {
    scanBus(bus);
  }

  delay(5000);           // wait 5 seconds for next scan
}
