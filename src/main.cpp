#include <Arduino.h>
#include <core/scheduler.h>
#include <missions/fsm_task.h>

namespace
{
    SystemContext g_ctx;
    Scheduler g_scheduler;

    FlightStateMachineTask g_fsm;
}

void setup()
{
    Serial.begin(115200);

    g_scheduler.add(g_fsm);

    g_scheduler.init(g_ctx, millis());
}

void loop()
{
    g_scheduler.tick(g_ctx, millis());
}
