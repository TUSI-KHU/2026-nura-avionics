#pragma once
#include <SPI.h>
#include <SD.h>
#include "../lib/sensor.h"

class LOGGER : public Sensor<bool>
{
public:
    LOGGER(int chipSelect) : Sensor<bool>(READ_AND_WRITE)
    {
        _chipSelect = chipSelect;
        filename = "flightLogger.txt";
    }

    void init()
    {
        if (!SD.begin(_chipSelect))
        {
            // while(1);
        }

        if (!SD.exists("logcnt.txt"))
        {
            File myFile = SD.open("logcnt.txt", FILE_WRITE);
            myFile.println("1");
            myFile.close();
        }

        File myFile = SD.open("logcnt.txt", FILE_READ);
        SD.remove("flightLogger.txt");
        if (myFile)
        {
            // 파일에서 한 줄 읽기
            String numberString = myFile.readStringUntil('\n');
            myFile.close();
            // 문자열을 정수로 변환
            int number = numberString.toInt();
            Serial.print("파일에서 읽은 숫자: ");
            Serial.println(number);

            // 1을 더함
            number += 1;

            // 파일을 쓰기 모드로 다시 열기 (기존 내용 덮어쓰기)
            SD.remove("logcnt.txt");
            myFile = SD.open("logcnt.txt", FILE_WRITE);

            if (myFile)
            {
                myFile.println(number);
                myFile.close();
                Serial.print("파일에 기록한 숫자: ");
                Serial.println(number);
            }
            else
            {
                Serial.println("파일을 쓰기 모드로 열 수 없습니다.");
            }
            myFile.close();
        }
        else
        {
            Serial.println("파일을 열 수 없습니다.");
        }
        return;
    }

    void writeData(String data)
    {
        File dataFile = SD.open("log.txt", FILE_WRITE);
        if (dataFile)
        {
            dataFile.println(data);
            dataFile.close();
        }
        else
        {
            Serial.println("fail");
            // delay(10000);
            return;
        }
    }

    void readData()
    {
        File dataFile = SD.open("flightLogger.txt", FILE_READ);
        if (dataFile)
        {
            Serial.println("Reading from file:");
            while (dataFile.available())
            {
                Serial.write(dataFile.read());
            }
            dataFile.close();
        }
        else
        {
            Serial.println("Error opening file for reading.");
        }
    }

private:
    int _chipSelect;
    String filename;
};