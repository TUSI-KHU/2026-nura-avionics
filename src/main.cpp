#include <Arduino.h>

#include "app/flight_controller_app.h"

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
