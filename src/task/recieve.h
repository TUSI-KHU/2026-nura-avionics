#pragma once
#include "../modules.h"

/*
필요 기능
1. 버퍼에 있는 데이터를 패킷 단위로 짤라서 보관한다.
2. 오류가 있는 패킷이나 중간에 노이즈가 낀 패킷은 버린다.
3. 받은 패킷의 데이터만 뽑아서 리턴한다.
*/

class Recieve : public Task
{
public:
    Recieve(Xbee *xbee)
    {
        _xbee = xbee;
        _ExpectedPacketSize = 0;
    }

    // PACKET 다 쓴다음 delete 무조건 해 줄 것!
    // 정상적인 패킷이면 해당 패킷을 반환
    // 그 외 아직 처리중인 패킷이거나 노이즈가 발생한 패킷이면 빈 패킷 반환

    Packet *readPacket()
    {
        if (_xbee->IsAvailable() == false)
        {
            Packet *tempPacket = new Packet();
            return tempPacket;
        }
        uint8_t read_byte;
        if (q.size() == 0)
        {
            // 제일 앞의 값이 노이즈인 경우
            read_byte = _xbee->readData();
            if (read_byte != STX)
            {
                while (_xbee->IsAvailable() != 0 && read_byte != STX)
                {
                    read_byte = _xbee->readData();
                }
                // 여전히 노이즈인 경우
                if (read_byte != STX)
                {
                    Packet *tempPacket = new Packet();
                    return tempPacket;
                }
            }
            q.push(read_byte);
        }

        while (_xbee->IsAvailable() != 0)
        {
            read_byte = _xbee->readData();
            q.push(read_byte);
            if (q.size() == 3)
            {
                _ExpectedPacketSize = read_byte;
            }

            //
            if (read_byte == ETX)
            {
                // 실제 ETX 자리에 ETX로 사용된 경우
                if (q.size() == _ExpectedPacketSize)
                {
                    uint8_t *data = new uint8_t[_ExpectedPacketSize];
                    int idx = 0;
                    while (!q.IsEmpty())
                    {
                        data[idx] = q.front();
                        q.pop();
                        idx++;
                    }
                    Packet *tempPacket = new Packet(data, _ExpectedPacketSize);
                    return tempPacket;
                }

                // ETX와 같은 값의 data인 경우(즉 아직 패킷이 안끝남)
                else if (q.size() < _ExpectedPacketSize)
                {
                    continue;
                }
                // 예상 패킷 사이즈보다 클때 나온 경우
                else
                {
                    while (!q.IsEmpty())
                    {
                        q.pop();
                    }
                    Packet *tempPacket = new Packet();
                    return tempPacket;
                }
            }

            // 패킷 대기 큐가 예상 사이즈보다 크면 무조건 오류
            if (q.size() > _ExpectedPacketSize)
            {
                while (!q.IsEmpty())
                {
                    q.pop();
                }
                Packet *tempPacket = new Packet();
                return tempPacket;
            }
        }

        Packet *tempPacket = new Packet();

        // p
        if (*tempPacket.data == ROCKETSTATE || *tempPacket.data == CMD)
        {
            Transmit trasmit;
            transmit.sendSensorData("ACK");
        }
        //

        return tempPacket;
    }

private:
    Xbee *_xbee;
    Queue<uint8_t> q;
    int _ExpectedPacketSize;
};