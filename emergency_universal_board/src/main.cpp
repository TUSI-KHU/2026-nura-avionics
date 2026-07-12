#include <Arduino.h>
#include <SD.h>

#include "app/flight_controller_app.h"
#include "nura_constants.h"

#if !defined(UNIT_TEST) && !defined(PIO_UNIT_TESTING)
namespace
{
    FlightControllerApp *g_app = nullptr;
}

void setup()
{
#if !defined(NURA_BENCH_DISABLE_FLIGHT_LOG_TASK)
    // The built-in SDIO bus must be claimed before the larger app object is
    // constructed on the current Teensy 4.1 PCB stack. FlightLogTask still owns
    // the final init-fatal decision and file creation.
    delay(NuraConstants::App::kBoardPowerSettleDelayMs);
    (void)SD.sdfs.begin(SdioConfig(FIFO_SDIO));
#endif
    static FlightControllerApp app;
    g_app = &app;
    g_app->setup(millis());
}

void loop()
{
    if (g_app != nullptr)
    {
        g_app->loop(millis());
    }
}
#endif
