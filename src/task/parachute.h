#pragma once
#include "../modules.h"

class Parachute : public Task
{
public:
    Parachute(MOTOR *motor)
    {
        _motor = motor;
    }

    void run()
    {
        _motor->rotateToAngle(90);
        delay(100);
        _motor->rotateToAngle(45);
        delay(100);
    }

private:
    MOTOR *_motor;
};