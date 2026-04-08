#pragma once

#include <string.h>
#include <stdint.h>

#define READ_ONLY 0
#define WRITE_ONLY 1
#define READ_AND_WRITE 2

template <class T>
class Sensor
{
public:
    Sensor(int8_t sensorMode)
    {
        _sensorMode = sensorMode;
        _dataLength = 0;
        _data = NULL;
    }

    Sensor(int8_t sensorMode, uint8_t dataLength)
    {
        _sensorMode = sensorMode;
        _dataLength = dataLength;
        _data = new T[_dataLength]; // 가공된 데이터가 저장되는 배열
    }

    void writeData(); // 센서마다 파라메터 달라서 사실상 오버라이딩이 아닌 오버로딩을 해야 함. 즉 사실 이 줄은 없어도 됨.
    void readData();  // 센서로부터 데이터를 읽어서 _data에 저장하는 함수

    void init(); // READY 단계에서 수행할 작업 (생성자랑 다름)

    int getDataLength()
    {
        return _dataLength;
    }

    int getSensorMode()
    {
        return _sensorMode;
    }

    T *_data;

protected:
    uint8_t _dataLength;
    int8_t _sensorMode;
};