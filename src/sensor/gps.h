#pragma once
#include <SoftwareSerial.h>
#include <TinyGPS++.h>
#include "../lib/sensor.h"

class GPS : public Sensor<float>
{
public:
    // 생성자
    GPS(int8_t sensorMode, uint8_t datalength) : Sensor<float>(sensorMode, datalength) {} // RX: 3번 핀, TX: 2번 핀

    // 센서 초기화
    void init()
    {                        // setup() 함수 기능
        Serial2.begin(9600); // GPS 모듈과의 시리얼 통신 초기화

        // GPS 모듈의 초기화가 별도로 필요하지 않으므로, 통신 설정만으로 초기화 완료
        delay(1000); // 센서 안정화를 위해 잠시 대기

        lastTime = millis(); // 현재 시간을 lastTime에 저장
        _data[0] = 0.0;
        _data[1] = 0.0;
    }

    // 데이터 읽기
    void readData()
    { // loop() 함수 기능
        while (Serial2.available() > 0)
        {
            gps.encode(Serial2.read());
            if (gps.location.isUpdated())
            {
                // 위도와 경도 값을 추출하여 data 배열에 저장
                _data[0] = gps.location.lat(); // 위도
                _data[1] = gps.location.lng(); // 경도
            }
        }
        delay(10);
    }

private:
    TinyGPSPlus gps;
    unsigned long lastTime;
};
