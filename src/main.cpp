#include "modules.h"

ALTEMETER alt(READ_ONLY, 1);
GPS gps(READ_ONLY, 2);
IMU bno(READ_ONLY, 6);
Xbee xbee(READ_AND_WRITE);
MOTOR pitch_motor1(WRITE_ONLY, 15, 7);
MOTOR pitch_motor2(WRITE_ONLY, 15, 6);
MOTOR yaw_motor1(WRITE_ONLY, 15, 5);
MOTOR yaw_motor2(WRITE_ONLY, 15, 4);
MOTOR parachute_motor(WRITE_ONLY, 180, 3);
LOGGER logger(53);
Stage stage;

Carnard canard(&pitch_motor1, &pitch_motor2, &yaw_motor1, &yaw_motor2, &bno);
Parachute parachute(&parachute_motor);
Transmit transmit(&bno, &alt, &gps, &xbee, &logger);
UpdateSensor updatesensor(&bno, &alt, &gps);
StageChecker stagecheck(&bno, &alt, &gps, &stage, &logger);

// 지상국 코드

// Recieve recieve(&xbee);

unsigned long initTime, currentTime;

void setup()
{
    initTime = millis();
    // 초기 stage INIT으로 초기화
    stage = INIT;

    pinMode(13, HIGH);
    digitalWrite(13, HIGH);

    // 센서들 연결 및 초기화
    Serial.begin(115200);
    Serial3.begin(9600);
    Serial.println("INIT");
    gps.init();
    Serial.println("GPS DONE");
    bno.init();
    Serial.println("IMU DONE");
    alt.init();
    Serial.println("ALT DONE");

    logger.init();
    Serial.println("SD DONE");

    pitch_motor1.init();
    pitch_motor2.init();
    yaw_motor1.init();
    yaw_motor2.init();
    parachute_motor.init2();
    Serial.println("MOTOR");

    // 지상국으로 초기화 완료 되었다고 데이터 전송
    Serial.println("INIT");
    // 잠시 대기
    delay(100);
    Serial3.println("Stage,INIT");
    String log = "INIT\n";
    logger.writeData(log);
    digitalWrite(13, LOW);
}

void loop()
{
    // 지상국으로 부터 데이터 오는 지 확인, 있으면 해당 명령 수행
    Serial.print("Stage: ");
    Serial.println(stage);

    if (Serial3.available())
    {
        Serial.println("Recieve");
        String command = Serial3.readStringUntil('\n');
        Serial.print("command: ");
        Serial.print(command);
        command.trim();
        // delay(5000);
        delay(100);
        String logging = "command,";
        logging += command + "\n";
        logger.writeData(logging);
        if (command == "Ready")
        {
            // 센서 초기화

            // stage 변경
            stage = READY;

            // 지상국으로 상태 변경 알림
            Serial3.println("Stage,Ready");
        }

        else if (command == "Injection")
        {
            // 낙하산 강제 사출
            // parachute.run();

            // stage 변경
            stage = APOGEE;

            // 지상국으로 상태 변경 알림
            Serial3.println("Stage,APOGEE");
        }

        else if (command == "StageChange")
        {
            String log;
            switch (stage)
            {
            case INIT:
                stage = READY;
                Serial3.println("Stage,Ready");
                log = "READY\n";
                logger.writeData(log);
                break;
            case READY:
                stage = ASCENDING;
                stagecheck.setAscendingTime(millis());
                Serial3.println("Stage,Ascending");
                log = "ASCENDING\n";
                logger.writeData(log);
                break;
            case ASCENDING:
                stage = APOGEE;
                Serial3.println("Stage,Apogee");
                log = "APOGEE\n";
                logger.writeData(log);
                break;
            case APOGEE:
                stage = DESCENDING;
                Serial3.println("Stage,Descending");
                log = "DESCENDING\n";
                logger.writeData(log);
                break;
            default:
                break;
            }
        }

        else
        {
            Serial3.println("Undefined Command");
        }
    }
    if (stage == INIT)
    {
        updatesensor.run();
        transmit.sendSensorData();
    }
    if (stage == READY)
    {
        // 텔레메트리 값 읽기 시작
        // Serial.println("READY START");
        updatesensor.run();
        // 읽은 텔레메트리 값 전송 및 저장
        // Serial.println("updateSensor");
        transmit.sendSensorData();
        // Serial.println("Transmit");
        //  만약 가속도 값이 크게 변화하면 상태 ASCENDING으로 바꿈 및 상태 변경 패킷 전송 및 저장
        stagecheck.run();
        // Serial.println("REady finish");
    }
    else if (stage == ASCENDING)
    {
        // 텔레메트리 값 읽기 시작
        updatesensor.run();
        // 텔레메트리 값 전송 및 저장

        // 읽은 텔레메트리 값 전송 및 저장
        transmit.sendSensorData();
        // PID 제어
        canard.run();

        stagecheck.run();
    }
    else if (stage == APOGEE)
    {
        // 텔레메트리 값 읽기 시작
        updatesensor.run();
        // 낙하산 사출
        parachute.run();
        delay(100);
        transmit.sendSensorData();

        stagecheck.run();
    }
    else if (stage == DESCENDING)
    {
        // 텔레메트리 값 읽기 시작
        updatesensor.run();
        // 텔레메트리 값 전송 및 저장
        transmit.sendSensorData();
    }
    return;
}
