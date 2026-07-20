#include <Arduino.h>

#include "app/flight_controller_app.h"

#if !defined(UNIT_TEST) && !defined(PIO_UNIT_TESTING)
namespace
{
    FlightControllerApp *g_app = nullptr;
}

void setup()
{
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
