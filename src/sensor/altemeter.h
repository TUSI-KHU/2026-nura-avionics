#pragma once
#include "../lib/sensor.h"
#include <Wire.h>
#include <SPI.h> // 이거 왜 쓰는거지?
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>

// 핀 배열
// - VCC : 3.3v
// - GND : GND
// - SCL : A5(pin 20)
// - SDA : A4(pin 21)

class ALTEMETER : public Sensor<float>
{
public:
    ALTEMETER(int8_t sensorMode, int dataLength) : Sensor(sensorMode, dataLength), bmp() {}

    // 센서 사용을 위한 초기 세팅
    void init()
    {
        if (!bmp.begin(0x76))
        { // I2C 통신을 위한 주소(datasheet 찾아보면 나옴)
            Serial.print("NO!");
            // while (1);
        }
        bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,     /* 운영 모드 */
                        Adafruit_BMP280::SAMPLING_X2,     /* 온도 샘플링 */
                        Adafruit_BMP280::SAMPLING_X16,    /* 압력 샘플링 */
                        Adafruit_BMP280::FILTER_X16,      /* 필터 */
                        Adafruit_BMP280::STANDBY_MS_500); /* 대기 시간 */
        initialAltitude = resetInitialAltitude();
        _data[0] = 0.0;
    }

    // 현재 초기 고도를 0m로 간주하도록 세팅
    float resetInitialAltitude()
    {
        float mean_init_altitude = 0;
        for (int i = 0; i < 10; i++)
        {
            mean_init_altitude += bmp.readAltitude(SEALEVELPRESSURE_HPA);
            delay(100);
        }
        mean_init_altitude /= 10;
        return mean_init_altitude;
    }

    // 고도 데이터 추출
    void readData()
    {
        float altitude = bmp.readAltitude(SEALEVELPRESSURE_HPA) - initialAltitude;
        // Serial.println("고도계 시작");
        //  추출한 데이터를 센서 클래스의 _data 변수에 저장
        _data[0] = altitude;
        // createPacket(altitude);
        // delay(2000);
        // Serial.println("고도계 종료");
        delay(10);
    }

private:
    Adafruit_BMP280 bmp; // BMP280 센서 객체
    float initialAltitude;
    const float SEALEVELPRESSURE_HPA = 1006.0; // 해수면 기준 고흥 기압 -> 장소가 달라지면 해당 지역 해면 기압 입력
};
