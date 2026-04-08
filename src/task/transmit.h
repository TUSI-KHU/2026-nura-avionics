#pragma once
#include "../modules.h"
#include <Arduino.h>

class Transmit : public Task
{
public:
    Transmit(IMU *imu, ALTEMETER *alt, GPS *gps, Xbee *xbee, LOGGER *logger)
    {
        _imu = imu;
        _alt = alt;
        _gps = gps;
        _logger = logger;
        _xbee = xbee;
        _D_Len = (_imu->getDataLength() + _alt->getDataLength() + _gps->getDataLength()) * 4;
        _data = new uint8_t[_D_Len];
        _cnt = 0;
        log = "";
    }

    /*
        데이터 구조
        [gyrox, gyroy, gyroz, accx, accy, accz, alt, lat, lon]
        모두 float형태로 4byte씩 할당
    */

    void sendSensorData()
    {
        _updateData();
        Packet packet(SENSOR, _D_Len, _data);
        packet.transmitPacket();
        _xbee->writePacket(packet);

        String data = String((*_imu)._data[0]) + "," + String((*_imu)._data[1]) + "," + String((*_imu)._data[2]) + "," + String((*_imu)._data[3]) + "," + String((*_imu)._data[4]) + "," + String((*_imu)._data[5]) + "," + String((*_alt)._data[0]) + "," + String((*_gps)._data[0], 8) + "," + String((*_gps)._data[1], 8);

        Serial.println(data);

        unsigned long currentTime = millis();

        String sensorlog = "1," + String(currentTime) + "," + data + "\n";
        log += sensorlog;
        _cnt += 1;
        if (_cnt >= 30)
        {
            _cnt = 0;
            _logger->writeData(log);
            log = "";
        }
        // Serial3.println(data);
    }

    // 로켓 상태 전송 함수
    void sendRocketState(Stage stage)
    {
        uint8_t *temp_data = new uint8_t;
        temp_data[0] = stage;
        Packet packet(ROCKETSTATE, 1, temp_data);
        packet.transmitPacket();
        _xbee->writePacket(packet);

        // p
        delay(50);

        Recieve recieve();
        Packet *packet = new Packet;
        packet = &recieve.readPacket();

        if (packet.data == ACK)
        {
            return;
        }
        else
        {
            sendRocketState(stage);
            return;
        }
        //
    }

private:
    IMU *_imu;
    ALTEMETER *_alt;
    GPS *_gps;
    Xbee *_xbee;
    LOGGER *_logger;
    uint8_t *_data;
    uint8_t _D_Len;

    uint8_t _cnt;

    String log;
    void _updateData()
    {
        int idx = 0;
        // IMU
        for (int i = 0; i < _imu->getDataLength(); i++)
        {
            uint8_t temp[4];
            memcpy(temp, &((*_imu)._data[i]), sizeof(float));
            for (int j = 0; j < 4; j++)
            {
                _data[idx] = temp[j];
                idx++;
            }
        }

        // ALTEMETER
        for (int i = 0; i < _alt->getDataLength(); i++)
        {
            uint8_t temp[4];
            memcpy(temp, &(*_alt)._data[i], sizeof(float));
            for (int j = 0; j < 4; j++)
            {
                _data[idx] = temp[j];
                idx++;
            }
        }

        // GPS
        for (int i = 0; i < _gps->getDataLength(); i++)
        {
            uint8_t temp[4];
            memcpy(temp, &(*_gps)._data[i], sizeof(float));
            for (int j = 0; j < 4; j++)
            {
                _data[idx] = temp[j];
                idx++;
            }
        }
    }
};