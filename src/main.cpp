#include <Arduino.h>

#include "app/flight_controller_app.h"

#if !defined(UNIT_TEST) && !defined(PIO_UNIT_TESTING)
namespace
{
    FlightControllerApp g_app;
}

void setup()
{
    g_app.setup(millis());
}

void loop()
{
    g_app.loop(millis());
}
#endif
