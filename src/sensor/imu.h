#pragma once

#include "../lib/sensor.h"
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Wire.h>
#include <Arduino.h>

class IMU : public Sensor<float>
{
public:
    IMU(int8_t sensorMode, uint8_t dataLength)
        : Sensor(sensorMode, dataLength) {} // lastTime은 deltaTime 계산을 위해 사용.

    void init()
    { // setup() 함수의 기능
        Serial.println("Bno start");
        if (!bno.begin())
        {
            while (1)
                ;
        }
        bno.setAxisRemap(Adafruit_BNO055::adafruit_bno055_axis_remap_config_t(0x09));
        delay(1000);
        Serial.println("Bno done");
        // 내부 클럭 대신 외부 크리스탈을 사용하도록 설정합니다.
        bno.setExtCrystalUse(true);
        // 센서를 통해 얻은 데이터를 event 구조체에 저장

        // bno.setAxisSign(Adafruit_BNO055::adafruit_bno055_axis_remap_sign_t(0x00));

        for (int i = 0; i < 10; i++)
        {
            readData();
            delay(100);
        }

        delay(1000);

        for (int i = 0; i < 6; i++)
        {
            _data[i] = 0.0;
        }
    }

    void readData()
    { // loop() 함수의 기능
        // Serial.println("bno 업데이트 시작");
        if (!bno.getEvent(&event, Adafruit_BNO055::VECTOR_LINEARACCEL))
        {
            return;
        }
        // Serial.println("bno 가속도 데이터 업데이트 완료");
        //  가속도 데이터
        float accelX = (float)event.acceleration.x;
        float accelY = (float)event.acceleration.y;
        float accelZ = (float)event.acceleration.z;

        // Serial.println("bno 가속도 데이터 변수에 업데이트 완료");

        // 자이로 데이터 (초기 각도 보정)
        if (!bno.getEvent(&event, Adafruit_BNO055::VECTOR_EULER))
        {
            return;
        }

        //.println("bno 자이로 데이터 업데이트 완료");

        float gyroX = (float)event.orientation.x;
        float gyroY = (float)event.orientation.y;
        float gyroZ = (float)event.orientation.z;
        // Serial.println("bno 자이로 데이터 변수에 업데이트 완료");
        //  데이터를 센서 클래스의 data 배열에 저장

        _data[0] = gyroX;
        _data[1] = gyroY;
        _data[2] = gyroZ;

        _data[3] = accelX;
        _data[4] = accelY;
        _data[5] = accelZ;

        // Serial.println("bno 데이터 배열에 업데이트 완료")
    }

private:
    // BNO055 default I2C address is 0X28.
    Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);
    // 가속도계, 자이로스코프, 지자기계 등에서 측정된 데이터를 담고 있는 구조체
    // BNO055 라이브러리에서 제공
    sensors_event_t event;
    float initialOrientation[3];
};
