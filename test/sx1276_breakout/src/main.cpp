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
constexpr bool kRawSpiProbeOnly = false;

Module radioModule(kNss,
                   kDio0,
                   kRst,
                   RADIOLIB_NC,
                   SPI1,
                   SPISettings(1000000UL, MSBFIRST, SPI_MODE0));
SX1276 radio(&radioModule);

volatile bool receivedFlag = false;
bool radioReady = false;
int16_t initState = RADIOLIB_ERR_UNKNOWN;
uint32_t txCount = 0UL;
uint32_t rxCount = 0UL;
uint32_t lastDiagnosticMs = 0UL;

uint16_t countValidVersionReads(uint32_t spiFrequency, uint8_t spiMode, uint16_t samples);

void setRfPath(bool receivePath, bool transmitPath)
{
    digitalWrite(kRxe, receivePath ? HIGH : LOW);
    digitalWrite(kTxe, transmitPath ? HIGH : LOW);
}

uint8_t outputLatch(uint8_t pin)
{
    return ((*portOutputRegister(pin) & digitalPinToBitMask(pin)) != 0U) ? 1U : 0U;
}

uint8_t outputEnabled(uint8_t pin)
{
    return ((*(portOutputRegister(pin) + 1) & digitalPinToBitMask(pin)) != 0U) ? 1U : 0U;
}

void probeRfPath(const char *name, bool receivePath, bool transmitPath)
{
    setRfPath(receivePath, transmitPath);
    delay(50);
    const uint16_t passed = countValidVersionReads(250000UL, SPI_MODE0, 100U);
    Serial.print("RF_PATH_TEST name=");
    Serial.print(name);
    Serial.print(" commanded=");
    Serial.print(receivePath ? 1 : 0);
    Serial.print(',');
    Serial.print(transmitPath ? 1 : 0);
    Serial.print(" read=");
    Serial.print(digitalRead(kRxe));
    Serial.print(',');
    Serial.print(digitalRead(kTxe));
    Serial.print(" latch=");
    Serial.print(outputLatch(kRxe));
    Serial.print(',');
    Serial.print(outputLatch(kTxe));
    Serial.print(" output=");
    Serial.print(outputEnabled(kRxe));
    Serial.print(',');
    Serial.print(outputEnabled(kTxe));
    Serial.print(" nss=");
    Serial.print(digitalRead(kNss));
    Serial.print(" rst=");
    Serial.print(digitalRead(kRst));
    Serial.print(" dio0=");
    Serial.print(digitalRead(kDio0));
    Serial.print(" spi=");
    Serial.print(passed);
    Serial.println("/100");
}

void runRfPathProbe()
{
    probeRfPath("idle", false, false);
    probeRfPath("rx", true, false);
    probeRfPath("tx", false, true);
    probeRfPath("idle", false, false);
}

void printRfCouplingState(const char *name)
{
    delay(20);
    Serial.print("RF_COUPLING_TEST name=");
    Serial.print(name);
    Serial.print(" read=");
    Serial.print(digitalRead(kRxe));
    Serial.print(',');
    Serial.println(digitalRead(kTxe));
}

void runRfCouplingProbe()
{
    pinMode(kRxe, INPUT);
    pinMode(kTxe, INPUT);
    printRfCouplingState("both_hiz");

    pinMode(kRxe, OUTPUT);
    digitalWrite(kRxe, HIGH);
    printRfCouplingState("rxe_high_txe_hiz");
    digitalWrite(kRxe, LOW);

    pinMode(kRxe, INPUT);
    pinMode(kTxe, OUTPUT);
    digitalWrite(kTxe, HIGH);
    printRfCouplingState("rxe_hiz_txe_high");

    pinMode(kRxe, OUTPUT);
    pinMode(kTxe, OUTPUT);
    setRfPath(false, false);
    printRfCouplingState("restored_idle");
}

uint8_t readVersionRegister(uint32_t spiFrequency = 250000UL, uint8_t spiMode = SPI_MODE0)
{
    SPI1.beginTransaction(SPISettings(spiFrequency, MSBFIRST, spiMode));
    digitalWrite(kNss, LOW);
    delayMicroseconds(20);
    (void)SPI1.transfer(0x42U);
    const uint8_t version = SPI1.transfer(0x00U);
    delayMicroseconds(20);
    digitalWrite(kNss, HIGH);
    SPI1.endTransaction();
    return version;
}

void printDiagnosticStatus()
{
    Serial.print("DIAG init_state=");
    Serial.print(initState);
    Serial.print(" reg_version=0x");
    const uint8_t version = readVersionRegister();
    if (version < 0x10U)
    {
        Serial.print('0');
    }
    Serial.print(version, HEX);
    Serial.print(" rst=");
    Serial.print(digitalRead(kRst));
    Serial.print(" nss=");
    Serial.print(digitalRead(kNss));
    Serial.print(" dio0=");
    Serial.print(digitalRead(kDio0));
    Serial.print(" rxe=");
    Serial.print(digitalRead(kRxe));
    Serial.print(" txe=");
    Serial.println(digitalRead(kTxe));
}

uint16_t countValidVersionReads(uint32_t spiFrequency, uint8_t spiMode, uint16_t samples)
{
    uint16_t passed = 0U;
    for (uint16_t sample = 0U; sample < samples; ++sample)
    {
        if (readVersionRegister(spiFrequency, spiMode) == 0x12U)
        {
            ++passed;
        }
        delay(1);
    }
    return passed;
}

bool runSpiStabilityMatrix(bool includeModeMatrix)
{
    constexpr uint16_t kSamples = 200U;
    constexpr uint32_t kFrequencies[] = {
        25000UL,
        50000UL,
        100000UL,
        250000UL,
        500000UL,
        1000000UL,
    };
    constexpr uint8_t kModes[] = {SPI_MODE0, SPI_MODE1, SPI_MODE2, SPI_MODE3};

    setRfPath(false, false);
    Serial.println("RAW_SPI_FREQUENCY_MATRIX mode=0");
    bool stable = true;
    for (const uint32_t frequency : kFrequencies)
    {
        const uint16_t passed = countValidVersionReads(frequency, SPI_MODE0, kSamples);
        Serial.print("SPI_TEST hz=");
        Serial.print(frequency);
        Serial.print(" passed=");
        Serial.print(passed);
        Serial.print('/');
        Serial.println(kSamples);
        if (passed != kSamples)
        {
            stable = false;
        }
    }

    if (includeModeMatrix)
    {
        Serial.println("RAW_SPI_MODE_MATRIX hz=250000");
        for (uint8_t index = 0U; index < 4U; ++index)
        {
            const uint8_t mode = kModes[index];
            const uint16_t passed = countValidVersionReads(250000UL, mode, kSamples);
            Serial.print("SPI_TEST mode=");
            Serial.print(index);
            Serial.print(" passed=");
            Serial.print(passed);
            Serial.print('/');
            Serial.println(kSamples);
        }
    }

    Serial.println(stable ? "RAW_SPI_STABILITY_PASS" : "RAW_SPI_STABILITY_FAIL");
    return stable;
}

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
        setRfPath(false, false);
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
        Serial.println("PASS: TX complete");
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
    Serial.println("commands: m=raw SPI matrix, g=RF-path GPIO probe, c=RF coupling probe, s=status, h=help");
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
        else if (command == 'm' || command == 'M')
        {
            const bool stable = runSpiStabilityMatrix(kRawSpiProbeOnly);
            initState = stable ? RADIOLIB_ERR_NONE : RADIOLIB_ERR_CHIP_NOT_FOUND;
        }
        else if (command == 'g' || command == 'G')
        {
            runRfPathProbe();
        }
        else if (command == 'c' || command == 'C')
        {
            runRfCouplingProbe();
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
    Serial.println("Power: radio=5 V, CS/RST pull-ups=3.3 V; SPI/GPIO logic=3.3 V");
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
    pinMode(kNss, OUTPUT);
    digitalWrite(kNss, HIGH);
    pinMode(kRst, OUTPUT);
    digitalWrite(kRst, LOW);
    delay(50);
    digitalWrite(kRst, HIGH);
    delay(500);

    Serial.print("PRE_INIT_REG_VERSION=0x");
    Serial.println(readVersionRegister(), HEX);
    const bool rawSpiStable = runSpiStabilityMatrix(kRawSpiProbeOnly);
    if (kRawSpiProbeOnly)
    {
        initState = rawSpiStable ? RADIOLIB_ERR_NONE : RADIOLIB_ERR_CHIP_NOT_FOUND;
        Serial.println("RAW_SPI_PROBE_ONLY: RadioLib and RF modes skipped");
        printHelp();
        return;
    }
    if (!rawSpiStable)
    {
        initState = RADIOLIB_ERR_CHIP_NOT_FOUND;
        Serial.println("FAIL: raw SPI is unstable; RadioLib initialization skipped");
        printHelp();
        return;
    }

    ConfigLoRa_t config;
    config.frequency = kFrequencyMHz;
    config.bandwidth = 125.0F;
    config.spreadingFactor = kSpreadingFactor;
    config.codingRate = kCodingRate;
    config.syncWord = kSyncWord;
    config.power = kTxPowerDbm;
    config.preambleLength = 8U;

    initState = radio.begin(config);
    if (initState != RADIOLIB_ERR_NONE)
    {
        Serial.print("FAIL: SX1276 init state=");
        Serial.println(initState);
        if (initState == RADIOLIB_ERR_CHIP_NOT_FOUND)
        {
            Serial.println("CHECK: stable 5 V/3.3 V/GND, then SPI1/NSS/RST");
        }
        printDiagnosticStatus();
        printHelp();
        return;
    }

    radio.setRfSwitchPins(kRxe, kTxe);
    radioReady = startReceive();
    Serial.println("RF: 920.9 MHz BW125 SF7 CR4/5 sync=0x12 CRC on TX=2 dBm");
    printHelp();
}

void loop()
{
    serviceReceive();
    serviceSerial();
    const uint32_t nowMs = millis();
    if ((nowMs - lastDiagnosticMs) >= 1000UL)
    {
        lastDiagnosticMs = nowMs;
        printDiagnosticStatus();
    }
}
