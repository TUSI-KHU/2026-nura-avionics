#pragma once
#include "../modules.h"
#include <Arduino.h>

class Carnard : public Task
{
public:
    Carnard(MOTOR *pitch_motor1, MOTOR *pitch_motor2, MOTOR *yaw_motor1, MOTOR *yaw_motor2, IMU *imu)
    {
        _pitch_motor[0] = pitch_motor1;
        _pitch_motor[1] = pitch_motor2;
        _yaw_motor[0] = yaw_motor1;
        _yaw_motor[1] = yaw_motor2;
        _imu = imu;

        _kp = 0.98, _ki = 0.01, _kd = 0.35;
        _desiredAngle = 0;
        _lastTime = 0;

        _pitch_previous_error = 0;
        _yaw_previous_error = 0;

        _pitch_error_integral = 0;
        _yaw_error_integral = 0;
    }

    void setGain(float kp, float ki, float kd)
    {
        _kp = kp;
        _ki = ki;
        _kd = kd;
    }

    void run()
    {
        unsigned long currentTime = millis();
        float deltaTime = (currentTime - _lastTime) / 1000.0; // 초 단위 시간 차이
        _lastTime = currentTime;

        float pitch_error = _desiredAngle - _imu->_data[1];
        float yaw_error = _desiredAngle - _imu->_data[2];

        Serial.print("pitch error: ");
        Serial.print(pitch_error);
        Serial.print(" yaw error: ");
        Serial.print(yaw_error);
        // pitch 제어
        float pitch_derivative = (pitch_error - _pitch_previous_error) / deltaTime;
        _pitch_previous_error = pitch_error;
        _pitch_error_integral += pitch_error * deltaTime;
        float pitch_control = _kp * pitch_error + _ki * _pitch_error_integral + _kd * pitch_derivative;

        // yaw 제어
        float yaw_derivative = (yaw_error - _yaw_previous_error) / deltaTime;
        _yaw_previous_error = yaw_error;
        _yaw_error_integral += yaw_error * deltaTime;
        float yaw_control = _kp * yaw_error + _ki * _yaw_error_integral + _kd * yaw_derivative;

        Serial.print(" pitch_control: ");
        Serial.print(pitch_control);
        Serial.print(" yaw_control: ");
        Serial.println(yaw_control);

        _pitch_motor[0]->rotateToAngle(pitch_control);
        _pitch_motor[1]->rotateToAngle(-pitch_control);

        // 현재 각도에서 더해 줘야 할지 확인 필요
        _yaw_motor[0]->rotateToAngle(yaw_control);
        _yaw_motor[1]->rotateToAngle(-yaw_control);
    }

private:
    MOTOR *_pitch_motor[2];
    MOTOR *_yaw_motor[2];
    IMU *_imu;

    float _kp, _ki, _kd;

    float _desiredAngle;

    float _pitch_error_integral;
    float _yaw_error_integral;

    float _pitch_previous_error;
    float _yaw_previous_error;

    unsigned long _lastTime;
};