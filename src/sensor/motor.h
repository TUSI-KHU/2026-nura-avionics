#pragma once
#include <Servo.h>
#include "../lib/sensor.h"
#include <Arduino.h>
// 서보모터는 pwm 핀에 연결해야한다. -> 3, 5, 6, 9, 10, 11

class MOTOR : public Sensor<bool>
{
public:
    MOTOR(int8_t sensorMode, int16_t maxAngle, int pin)
        : Sensor<bool>(sensorMode), _maxAngle(maxAngle), _pin(pin) {}

    void init()
    { // setup() 함수의 기능
        ms.attach(_pin);
        setToZero();
    }

    void init2()
    {
        ms.attach(_pin);
    }

    void setToZero()
    {
        _currentAngle = 90;
        ms.write(_currentAngle);
    }

    void rotateToAngle(int number)
    { // loop() 함수의 기능
        _currentAngle = number;
        if (_currentAngle < -_maxAngle)
        {
            // 각도가 허용 범위를 벗어났을 경우 오류 처리
            _currentAngle = -_maxAngle;
        }
        else if (_currentAngle > _maxAngle)
        {
            _currentAngle = _maxAngle;
        }
        ms.write(_currentAngle + 90);
    }

private:
    Servo ms;
    int16_t _currentAngle;
    int16_t _maxAngle;
    int16_t _pin;
};
