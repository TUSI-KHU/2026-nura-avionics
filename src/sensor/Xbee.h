#pragma once
#include "../lib/sensor.h"
#include "../lib/packet.h"
// 이 클래스는 약간 손 봐야 할 듯

class Xbee : public Sensor<bool>
{
public:
    Xbee(int8_t sensorMode) : Sensor(sensorMode) {}

    void init()
    {
        Serial3.begin(9600);
    }

    // 1바이트 데이터씩 읽고 보내는 코드
    uint8_t readData()
    {
        if (Serial3.available() > 0)
        {
            uint8_t data = Serial3.read();
            return data;
        }

        else
        {
            return 0;
        }
    }

    void writeData(uint8_t data)
    {
        Serial3.write(data);
    }

    void writePacket(Packet &packet)
    {
        /*float dataArray[9];
        for(int i=0;i<9;i++){
            dataArray[i] = packet.dataToNum<float>(i*4);
            Serial.print(dataArray[i]);
            Serial.print(" ");
        }*/
        Serial.println(packet.getPacketSize());
        for (uint8_t i = 0; i < packet.getPacketSize(); i++)
        {
            uint8_t c = packet.getPacketData(i);
            Serial.write(c);
            Serial3.write(packet.getPacketData(i));
        }
        Serial.println();
    }

    bool IsAvailable()
    {
        return Serial3.available();
    }
};