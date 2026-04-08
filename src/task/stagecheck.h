#pragma once
#include "../modules.h"
#include <Arduino.h>

class StageChecker : public Task
{
public:
    StageChecker(IMU *imu, ALTEMETER *alt, GPS *gps, Stage *stage, LOGGER *logger)
    {
        _imu = imu;
        _alt = alt;
        _gps = gps;
        _stage = stage;
        _logger = logger;
    }

    void run()
    {
        if (*_stage == READY)
        {
            if (_imu->_data[5] >= 10)
            {
                Serial3.println("Stage,ASCENDING");
                *_stage = ASCENDING;
                ascendingTime = millis();
                String log = "ASCENDING\n";
                _logger->writeData(log);
                return;
            }
        }
        else if (*_stage == ASCENDING)
        {
            // 낙하산 사출 조건 확인

            // 1. pitch 혹은 yaw 값이 75도 이상 기울었을 때
            if (abs(int(_imu->_data[1])) >= 75 || abs(int(_imu->_data[2])) >= 75)
            {
                Serial3.println("Stage,APOGEE");
                *_stage = APOGEE;
                String log = "APOGEE1\n";
                _logger->writeData(log);
            }

            // 2. 로켓의 하강이 감지 될 때
            /*else if(_imu->_data[5]<=-5){
                Serial3.println("Stage,APOGEE");
                *_stage = APOGEE;
                String log = "APOGEE2\n";
                _logger->writeData(log);
            }*/

            // 3. 타이머의 값이 발사 후 4초가 지났을때 (발사 시뮬레이터 기반으로 값 조정 예정)
            else if ((millis() - ascendingTime) / 1000 > 8)
            {
                Serial3.println("Stage,APOGEE");
                *_stage = APOGEE;
                String log = "APOGEE3\n";
                _logger->writeData(log);
            }

            return;
        }
        else if (*_stage == APOGEE)
        {
            Serial3.println("Stage,DESCENDING");
            *_stage = DESCENDING;
            String log = "DESCENDING\n";
            _logger->writeData(log);
            return;
        }
    }

    void setAscendingTime(unsigned long Ascendingtime)
    {
        ascendingTime = Ascendingtime;
    }

private:
    IMU *_imu;
    ALTEMETER *_alt;
    GPS *_gps;
    Stage *_stage;
    LOGGER *_logger;

    unsigned long ascendingTime;
};
