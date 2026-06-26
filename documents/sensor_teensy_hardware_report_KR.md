# NURA 2026 센서 및 Teensy 하드웨어 정리 보고서

작성일: 2026-06-26  
대상 코드 기준: `include/board_pinmap.h`, `include/nura_constants.h`, `src/hal/*`, `platformio.ini`, `documents/pinmap_summary.txt`

> 이 문서는 부품 데이터시트/공식 가이드와 현재 펌웨어 핀맵을 대조한 하드웨어 정리 보고서이다. 비행 승인 문서가 아니며, PCB 리비전/실물 회로/커넥터 방향/전원 레일/납땜 상태는 별도 검증이 필요하다.

## 1. 현재 프로젝트 하드웨어 요약

| 분류 | 부품 / 모듈 | 프로젝트 역할 | 현재 인터페이스 | 현재 핀 |
| --- | --- | --- | --- | --- |
| MCU | Teensy 4.1 | 비행 컴퓨터 | USB, GPIO, SPI, I2C, UART, ADC, SDIO/QSPI | PlatformIO `teensy41` |
| 저-g IMU | LSM6DSO32 / LSM6DSOX | 주 IMU, 가속도/자이로 | SPI0 | MOSI D11, MISO D12, SCK D13, CS D10 |
| 고-g 가속도계 | H3LIS331DL | 발사/충격/고가속 로깅 후보 | SPI0 | MOSI D11, MISO D12, SCK D13, CS D0 |
| 자기계 | LIS3MDL | 자기장/방위 후보 | I2C1 | SDA1 D17, SCL1 D16, 주소 `0x1C` |
| 기압계 | MPL3115A2 | 고도/압력 | I2C0 | SDA0 D18, SCL0 D19, 주소 `0x60` |
| GNSS | u-blox M6 / NEO-6M 계열 | 위치/시간/회수 지원 | UART Serial3 | GPS TX -> D15/RX3, GPS RX -> D14/TX3, 9600 baud |
| 비행 LoRa | SX1262 | 비행체 텔레메트리 | SPI1 + IRQ/BUSY | MISO1 D1, MOSI1 D26, SCK1 D27, NSS D9, DIO1 D31, BUSY D32 |
| 지상/개발 LoRa | SX1276/SX127x, RA-01 계열 | 지상 수신/개발 테스트 | SPI0 + DIO0 | SS D9, RST/RXE D30, DIO0 D31 |
| 전압 센스 | 3S 배터리 분압 | 배터리 전압 텔레메트리 | ADC | D21, 분압비 5.545 |
| 저장장치 | Teensy 내장 microSD | 비행 로그 미러 | SDIO | `BUILTIN_SDCARD` |
| 저장장치 | W25Q128 QSPI NOR flash 후보 | 온보드 로그/검증 저장 | Teensy 4.1 하단 QSPI pads | 16 MB / 128 Mbit 후보 |
| 알림 | Buzzer, LED | 상태/오류 피드백 | GPIO/PWM | Buzzer D2, LED2 D33 |

중요: `test/sensor_test/README.md`에는 Pyro 1이 D20/D21로 적힌 오래된 기록이 남아 있다. 현재 소스의 기준은 `include/board_pinmap.h`와 `documents/pinmap_summary.txt`이며, 현재 Pyro 1은 D28/D29, 배터리 전압 센스는 D21로 충돌하지 않는다.

## 2. Teensy 4.1

### 핵심 특징

- MCU: NXP i.MX RT1062, 600 MHz급 ARM Cortex-M7 계열 보드.
- I/O: PJRC 공식 페이지 기준 총 55개 신호 핀, 브레드보드에서 쉽게 접근 가능한 핀은 42개.
- 로직 레벨: 디지털 핀은 3.3 V 신호용이며 5 V tolerant가 아니다. VIN/VUSB를 제외한 일반 I/O에 3.3 V 초과를 넣으면 손상 위험이 있다.
- 내장 기능: USB device, USB host pads, Ethernet pads, microSD socket, 하단 QSPI 메모리 패드.
- 프로젝트 빌드: `platformio.ini`에서 `board = teensy41`, `framework = arduino`, 시리얼 모니터 115200 baud.

### 프로젝트에서 쓰는 버스

| 버스 | Teensy 핀 | 프로젝트 사용 |
| --- | --- | --- |
| SPI0 | MOSI D11, MISO D12, SCK D13 | LSM6DSO32/LSM6DSOX, H3LIS331DL, 일부 SX127x 개발 테스트 |
| SPI1 | MISO D1, MOSI D26, SCK D27 | 비행 SX1262 |
| I2C0 / Wire | SDA D18, SCL D19 | MPL3115A2 |
| I2C1 / Wire1 | SDA D17, SCL D16 | LIS3MDL |
| UART3 / Serial3 | RX3 D15, TX3 D14 | u-blox M6 GNSS |
| ADC | D21 | 3S 배터리 전압 분압 |
| SDIO | `BUILTIN_SDCARD` | microSD 비행 로그 |

### 전원 관련 주의사항

- USB와 외부 전원을 동시에 연결할 경우, Teensy 하단의 VIN-VUSB 연결 trace를 자르는 것이 일반적인 보호 방법이다. 이렇게 하면 USB 5 V가 VIN 쪽 회로를 먹이지 않고, 반대로 외부 5 V/VIN이 노트북 USB 포트로 역류하는 위험도 줄어든다.
- 단점: VIN-VUSB trace를 자른 뒤에는 USB 케이블만 꽂아서는 보드가 켜지지 않는다. USB 데이터는 연결되지만, 별도 VIN 전원이 있어야 업로드/시리얼이 동작한다.
- 되돌리려면 잘랐던 pads를 납으로 다시 bridge해야 한다. 현장에서는 trace cut 후 멀티미터 continuity로 VIN-VUSB가 정말 분리됐는지 확인해야 한다.
- 외부 전원 운용 시 권장 방식은 `USB 데이터 + 외부 VIN 전원`을 분리 운용하거나, 쇼트키 다이오드/전원 OR-ing 회로를 사용하는 것이다. 단순 병렬 연결은 노트북과 비행 전원 모두에 나쁘다.
- Teensy 3.3 V 핀은 보드 레귤레이터 출력이다. 많은 센서 breakout에 전원을 공급할 수는 있지만, RF 송신/SD write/센서 inrush가 겹치면 전압 강하와 노이즈가 생길 수 있다. RF/아날로그/센서 전원은 디커플링을 실물에서 확인해야 한다.

### 핀 레벨 주의사항

- Teensy 4.1 GPIO는 3.3 V만 안전하다.
- “Adafruit breakout은 5 V tolerant” 같은 말은 breakout 보드에 level shifter/regulator가 있을 때의 이야기이다. 센서 IC 자체는 대부분 1.8~3.6 V 계열이다.
- 현재 NURA PCB는 모든 센서 VCC를 Teensy 3.3 V rail 기준으로 보는 문서가 있다. 외부 5 V 센서 모듈을 섞으면 I2C/SPI 라인 level이 즉시 문제가 될 수 있다.

## 3. LSM6DSO32 / LSM6DSOX 저-g IMU

### 부품 특징

- 3축 가속도 + 3축 자이로.
- LSM6DSO32 데이터시트 기준 가속도 full-scale: ±4/±8/±16/±32 g.
- 자이로 full-scale: ±125/±250/±500/±1000/±2000 dps.
- I2C, SPI, I3C 계열 인터페이스를 지원한다.
- Adafruit LSM6DSO32 breakout은 VIN regulator와 level shifting이 있어 3 V/5 V MCU와 쓰기 쉽게 만든 보드이다.

### 프로젝트 연결

| 항목 | 값 |
| --- | --- |
| 프로젝트 역할 | 저-g IMU, 자세/가속도/자이로 |
| 현재 통신 | SPI0 |
| SPI 핀 | MOSI D11, MISO D12, SCK D13 |
| CS | D10 |
| INT1/INT2 | 미할당 |
| WHO_AM_I | LSM6DSO32 기대값 `0x6C` |
| HAL | `src/hal/lsm6dso32_hal.cpp`, `src/hal/lsm6dsox_hal.cpp` |
| 현재 task 주기 | 10 ms |

### 펌웨어 설정/동작

- `LSM6DSO32HAL::begin()`은 SPI mode 0~3을 직접 probe해서 WHO_AM_I가 맞는 mode를 찾는다.
- 이후 Adafruit LSM6DS 기반 init을 수행하고 accel/gyro range와 data rate를 설정한다.
- 읽기값은 `m/s^2`, `dps`, 온도 C로 저장된다.
- stationary calibration 함수가 있으며, 기대 중력 방향을 기준으로 accel offset과 gyro bias를 계산한다.

### 실무 주의사항

- LSM6DSO32는 ±32 g까지 가능하지만, 프로젝트 코드/테스트에서 LSM6DSOX bring-up은 ±16 g 설정 예제가 있다. 실제 flight range가 무엇인지 코드 경로별로 확인해야 한다.
- CS가 D10이므로 SPI0의 일반 SS 핀과 겹친다. microSD는 Teensy 내장 SDIO라 괜찮지만, 외부 SPI 장치 추가 시 CS 충돌을 피해야 한다.
- Adafruit breakout은 I2C 주소 jumper가 있다. I2C로 쓸 경우 기본 주소는 보통 `0x6A`, address pin high/jumper 시 `0x6B`이다. 현재 프로젝트는 SPI라 I2C 주소는 사용하지 않는다.
- IMU 축 방향은 breakout 실크와 PCB 장착 방향에 따라 바뀐다. “센서가 정상 응답한다”와 “로켓 좌표계와 축이 맞다”는 다른 문제다. 발사 감지/자세 추정에 쓰려면 X/Y/Z 부호와 로켓 body frame 변환을 별도 문서화해야 한다.
- 보드가 정지 상태일 때 가속도 norm은 약 1 g가 나와야 한다. 하지만 각 축 중 어디에 +1 g가 나타나는지는 장착 방향에 따라 달라진다.
- 자이로 bias는 온도와 시간에 따라 변한다. 발사대 정지 상태에서 boot calibration을 하고, arming 직전 재확인하는 쪽이 안전하다.

## 4. H3LIS331DL 고-g 가속도계

### 부품 특징

- 3축 high-g digital accelerometer.
- ST 데이터시트 기준 full-scale: ±100/±200/±400 g.
- ODR: 0.5 Hz ~ 1 kHz.
- 인터페이스: I2C/SPI.
- 공급 전압: 2.16 V ~ 3.6 V, IO는 저전압 호환.
- 충격 내성은 데이터시트상 10000 g class로 표기된다.

### 프로젝트 연결

| 항목 | 값 |
| --- | --- |
| 프로젝트 역할 | 고가속/충격/발사 이벤트 후보 |
| 현재 통신 | SPI0 |
| SPI 핀 | MOSI D11, MISO D12, SCK D13 |
| CS | D0 |
| INT1/INT2 | 미할당 |
| WHO_AM_I | 기대값 `0x32` |
| HAL | `src/hal/h3lis331dl_hal.cpp` |
| 현재 data rate | Adafruit driver에서 1000 Hz로 설정 |

### 실무 주의사항

- ±100/200/400 g 센서는 작은 흔들림/정지 1 g 측정에서는 저-g IMU보다 거칠고 noisy하다. “발사 충격을 안 잘라먹는 센서”이지 “정밀 자세 센서”가 아니다.
- SPI0를 LSM6DSO32와 공유한다. 각 센서 CS는 idle HIGH가 유지되어야 한다. 부팅 중 한 CS가 floating이면 두 SPI 장치가 동시에 MISO를 물고 버스가 깨질 수 있다.
- 현재 INT 핀이 미할당이라 data-ready interrupt 기반 샘플링은 하지 않는다. 고속 이벤트를 놓치지 않으려면 ODR, polling 주기, FIFO/interrupt 사용 여부를 별도 검증해야 한다.
- 고-g 센서를 발사 감지에 쓰려면 threshold 출처가 반드시 필요하다. 임의 g값을 넣으면 flight logic 안전 규칙에 위배된다.

## 5. LIS3MDL 자기계

### 부품 특징

- 3축 digital magnetometer.
- ST 데이터시트 기준 full-scale: ±4/±8/±12/±16 gauss.
- I2C standard/fast mode와 SPI를 지원한다.
- Adafruit LIS3MDL breakout은 STEMMA QT/Qwiic I2C 연결과 level shifting/support 회로를 제공한다.

### 프로젝트 연결

| 항목 | 값 |
| --- | --- |
| 프로젝트 역할 | 자기장/방위 후보 |
| 현재 통신 | I2C1 / Wire1 |
| 핀 | SDA1 D17, SCL1 D16 |
| 주소 | `0x1C` |
| 주소 설정 | DO/SDO 또는 AD1 계열 address pin을 GND/open 쪽으로 둔 상태 |
| HAL | `src/hal/lis3mdl_hal.cpp` |
| task 주기 | 100 ms |

### 실무 주의사항

- LIS3MDL Adafruit guide 기준 address jumper를 high/bridge하면 주소가 `0x1C`에서 `0x1E`로 바뀐다. 현재 펌웨어는 `0x1C` 고정이므로 jumper 상태가 중요하다.
- I2C mode 강제를 위해 breakout의 CS를 3.3 V에 묶는 구성이 문서에 적혀 있다. CS가 떠 있으면 SPI/I2C mode 판단이 불안정해질 수 있다.
- 자기계는 로켓 내부에서 가장 쉽게 오염되는 센서다. 배터리, 전류 루프, 모터 케이스, 나사, pyro 배선, RF coax 근처에서 hard-iron/soft-iron error가 커진다.
- boot-only calibration으로는 부족하다. 완성된 avionics stack을 실제 장착 상태로 돌려서 hard/soft iron calibration을 해야 한다.
- 발사 중 큰 전류가 흐르는 순간에는 자기계 heading이 튈 수 있다. 비행 상태 전이에 직접 쓰기보다 로그/후처리/보조 heading으로 보는 것이 안전하다.

## 6. MPL3115A2 기압계

### 부품 특징

- 절대압 기반 pressure/altimeter sensor.
- NXP 데이터시트 기준 operating pressure range: 20 kPa ~ 110 kPa.
- calibrated pressure range: 50 kPa ~ 110 kPa.
- 온도 출력 포함, I2C digital interface.
- altitude output resolution은 0.1 m class로 홍보되지만, 로켓에서는 pressure port, 동압, 진동, 온도, 필터링이 실제 성능을 좌우한다.
- 2026년 기준 NXP 페이지는 MPL3115A2/MPL3150A2를 discontinued로 표시하고, 신규 설계에는 MPL3115A2S/MPL3150A2S를 권장한다.

### 프로젝트 연결

| 항목 | 값 |
| --- | --- |
| 프로젝트 역할 | 고도/압력 |
| 현재 통신 | I2C0 / Wire |
| 핀 | SDA0 D18, SCL0 D19 |
| 주소 | `0x60` only |
| HAL | `src/hal/mpl3115a2_hal.cpp` |
| task 주기 | 50 ms |
| conversion timeout | 50 ms |
| 유효 압력 범위 | 20000 Pa ~ 110000 Pa |

### 펌웨어 설정/동작

- HAL은 BAROMETER mode를 사용한다.
- `CTRL_REG1`은 프로젝트 상수 `kFastBarometerCtrlReg1 = 0x00`으로 BAR mode, OS1 쪽 빠른 변환을 사용한다.
- non-blocking `poll()` 경로는 conversion 시작, 완료 확인, timeout 처리를 분리한다.
- pad ground baseline을 평균 압력으로 잡고, 상대 고도는 기준 압력 대비 barometric formula로 계산한다.

### 실무 주의사항

- 주소가 `0x60` 하나뿐이라 같은 버스에 `0x60` 고정 장치를 추가하면 충돌한다. 현재 MPL3115A2는 I2C0 단독이라 괜찮다.
- I2C pull-up은 버스당 한 쌍만 적절한 값으로 두는 것이 좋다. 여러 breakout의 10 k pull-up이 병렬로 많이 붙으면 effective resistance가 낮아져 rise time/전류가 바뀐다.
- 로켓 기압계는 센서 자체보다 pressure port 설계가 중요하다. 동압을 직접 맞으면 apogee detection이 흔들린다. 정압 포트 위치, 포트 수, 내부 volume, vent hole burr를 검사해야 한다.
- MPL3115A2 온도값은 보조 참고로만 써야 한다. 센서 내부 보상용 온도 성격이 강하며, 외기 온도계처럼 믿으면 안 된다.
- 부품 수급상 discontinued 이슈가 있으므로 새 PCB 리비전에서는 대체품 검토가 필요하다. 펌웨어 HAL은 `MPL3115A2S` 호환 여부를 확인하기 전까지 가정하면 안 된다.

## 7. u-blox M6 / NEO-6M GNSS

### 부품 특징

- u-blox 6 positioning engine 기반 GPS 모듈.
- 프로젝트 문서상 GY-GPS6MV2 / NEO-6M 계열 breakout 사용.
- u-blox 공식 문서는 NEO-6 시리즈가 miniature NEO form factor의 GPS module이며 NMEA/UBX protocol을 지원한다고 설명한다.
- 흔한 GY-NEO6MV2 모듈은 ceramic patch antenna, backup battery, EEPROM이 붙어 있지만, clone/제조사별 회로가 달라 전원/level 조건을 실물 확인해야 한다.

### 프로젝트 연결

| 항목 | 값 |
| --- | --- |
| 프로젝트 역할 | 위치/고도/속도/course/HDOP/satellites 텔레메트리 |
| 현재 통신 | UART Serial3 |
| 핀 | GPS TX -> Teensy RX3 D15, GPS RX -> Teensy TX3 D14 |
| baud | 9600 |
| parser | TinyGPSPlus |
| task 주기 | 50 ms |
| poll byte budget | 128 bytes |
| fix fresh 기준 | 2000 ms |

### 실무 주의사항

- GPS TX와 MCU RX는 교차 연결이다. GPS TX -> Teensy RX3 D15, GPS RX -> Teensy TX3 D14가 맞다.
- Teensy는 3.3 V UART라 NEO-6M IC와 전압 레벨이 맞는다. 5 V Arduino 예제처럼 막 연결하는 습관을 가져오면 GPS RX에 과전압이 들어갈 수 있다.
- GY-NEO6MV2 breakout은 “VCC 3~5 V 가능”으로 팔리는 경우가 많지만, 그것은 보드 regulator 이야기다. UART RX가 5 V tolerant인지 여부는 모듈 회로에 따라 다르다.
- patch antenna는 하늘을 봐야 한다. avionics bay 내부, carbon fiber, 금속 bulkhead, 배터리 아래에서는 fix 시간이 크게 늘거나 fix가 안 잡힌다.
- LED blinking은 보통 fix indication이지만 모듈마다 의미가 다를 수 있다. 펌웨어에서는 NMEA checksum/fix age/satellites/HDOP를 보고 판단해야 한다.
- 비행 중 GNSS altitude는 barometer보다 느리고 지연/점프가 있으므로 apogee 같은 빠른 상태 전이에 직접 쓰면 안 된다.

## 8. SX1262 비행 LoRa

### 부품 특징

- Semtech SX1262는 sub-GHz LoRa/(G)FSK transceiver.
- SX1261/2 계열은 150~960 MHz ISM band에서 동작하며, SX1262는 최대 +22 dBm class PA를 가진다.
- SPI command interface를 사용하고, BUSY pin이 LOW일 때 새 명령을 받을 수 있다.
- DIO1/DIO2/DIO3는 interrupt, RF switch, TCXO control 등으로 설정될 수 있다.

### 프로젝트 연결

| 항목 | 값 |
| --- | --- |
| 프로젝트 역할 | 비행 텔레메트리 |
| 현재 통신 | SPI1 |
| SPI 핀 | MISO1 D1, MOSI1 D26, SCK1 D27 |
| NSS | D9 |
| DIO1 | D31 |
| BUSY | D32 |
| RXE | 현재 `kUnassignedPin`; 과거/bench 문서에는 D30 검증 메모 |
| reset | `-1`, MCU-controlled NRESET 없음 |
| 주파수 | 920.9 MHz |
| SPI clock | 현재 상수 250 kHz, pinmap summary에는 bench 2 MHz 검증 기록 존재 |
| LoRa 설정 | SF7, BW 125 kHz, CR 4/5, preamble 8, sync word `0x12`, TX power 17 dBm |
| TCXO | `0.0 V`, XTAL 가정 |

### 펌웨어 설정/동작

- `src/hal/sx1262_lora_hal.cpp`는 BUSY LOW를 기다린 뒤 RadioLib `begin()`을 호출한다.
- PCB에 MCU-controlled NRESET net이 기록되어 있지 않아 no-reset mode가 의도적으로 사용된다.
- DIO1이 HIGH가 되면 TX complete/IRQ 처리 경로로 들어간다.
- 현재 `downlinkOnly = true`가 기본이라 RX/uplink 안정화는 후속 단계로 남아 있다.

### 실무 주의사항

- SX1262는 SX1276보다 BUSY pin 의존성이 강하다. BUSY가 연결되지 않거나 floating이면 SPI 명령 타이밍이 깨진다.
- DIO2로 RF switch를 자동 제어하는 모듈도 있고, 별도 RXEN/TXEN 핀이 필요한 모듈도 있다. 현재 PCB의 RXE/D30 의미는 schematic으로 확정해야 한다.
- TCXO가 있는 모듈이면 DIO3 TCXO voltage/startup delay 설정이 필요할 수 있다. 현재 코드는 `tcxoVoltage = 0.0 V`라 XTAL 보드 가정이다.
- NRESET이 없으면 radio가 이상 상태에 빠졌을 때 MCU가 하드 리셋을 못 한다. 전원 cycle 또는 sleep/standby command만으로 복구되는지 bench에서 확인해야 한다.
- 920.9 MHz 운용은 지역 규정/대회 규정/안테나 대역과 일치해야 한다. 915 MHz 안테나가 920.9 MHz에서 충분히 맞는지도 VNA 또는 range test로 확인해야 한다.
- RF 송신 중 전류 피크가 생긴다. 센서 3.3 V rail과 RF PA 전원이 같은 경로면 IMU/baro 노이즈와 brownout을 확인해야 한다.

## 9. SX1276 / SX127x 지상 및 개발 LoRa

### 부품 특징

- Semtech SX1276/77/78/79 계열은 137~1020 MHz LoRa transceiver.
- 데이터시트는 LoRa 장거리/고감도, +20 dBm class PA, 최대 link budget 168 dB급 특징을 설명한다.
- SX127x 계열은 SPI + reset + DIO0 interrupt 형태 예제가 많다.

### 프로젝트 연결

| 항목 | 값 |
| --- | --- |
| 프로젝트 역할 | 지상 수신기 / 개발 테스트 |
| receiver env | `sx1276_ground`에서 920.9 MHz ground radio |
| 개발 RA-01 계열 핀 | SS D9, reset/RXE D30, DIO0 D31, BUSY/reserved D32 |
| 라이브러리 | sender/receiver 일부는 Sandeep Mistry LoRa, 메인은 RadioLib |

### 실무 주의사항

- RA-01/SX1278 433 MHz 모듈과 SX1276 915/920 MHz 모듈은 같은 LoRa 계열이라도 RF front-end와 matching이 다르다. 주파수만 코드로 바꿔서 쓰면 안 된다.
- SX127x DIO0는 RX done/TX done interrupt로 자주 쓰인다. DIO0 미연결이면 polling 또는 timeout이 필요하다.
- 3.3 V logic 장치다. 5 V Arduino와 직접 연결하는 예제는 Teensy에는 필요 없지만, 반대로 Teensy 주변에 5 V 장치가 있으면 LoRa 핀 과전압을 조심해야 한다.
- 안테나 없이 송신하면 PA 손상 가능성이 있다. 아주 낮은 출력 bench라도 dummy load/안테나 연결을 기본으로 해야 한다.

## 10. 배터리 전압 센스

### 프로젝트 연결

| 항목 | 값 |
| --- | --- |
| 역할 | 3S pack voltage telemetry |
| 핀 | D21 analog input |
| ADC 기준 | 3.3 V |
| ADC resolution | 10 bit |
| 분압비 | pack voltage / 5.545 |
| 12.6 V 예상 ADC 입력 | 약 2.2723 V |
| 11.1 V 예상 ADC 입력 | 약 2.0018 V |
| 정상 raw 범위 | 약 620~704 |
| 유효 pack 범위 | 6000~14000 mV |

### 실무 주의사항

- 3S LiPo full charge 12.6 V가 ADC에 2.27 V로 들어가므로 3.3 V 기준에서는 여유가 있다.
- 저항 tolerance 때문에 실제 분압비는 5.545에서 벗어난다. 멀티미터로 pack voltage와 ADC raw를 동시에 찍어 calibration해야 한다.
- ADC 입력에 너무 큰 source impedance를 쓰면 Teensy ADC sample-and-hold가 충분히 충전되지 않아 값이 흔들릴 수 있다. 분압 저항값과 필터 capacitor를 회로에서 확인해야 한다.
- 현재 펌웨어는 raw를 mV로 환산하고 6~14 V 밖이면 invalid로 본다. invalid/stale은 telemetry에서 0으로 내려갈 수 있다.
- 배터리 음극과 Teensy GND가 확실히 공통이어야 한다. 공통 GND 없이 분압 신호만 넣으면 ADC 입력 보호 다이오드로 이상 전류가 흐를 수 있다.

## 11. microSD 및 W25Q128 QSPI flash

### microSD

- Teensy 4.1 내장 microSD socket은 SDIO 경로를 사용한다.
- 프로젝트는 flight log mirror storage에 microSD를 사용한다.
- D34~D39 계열은 Teensy SDIO 관련 핀으로 쓰일 수 있으므로, 하단/내장 SD 관련 핀과 pyro/main 출력 핀의 충돌 여부를 PCB 리비전마다 확인해야 한다.

### W25Q128 QSPI NOR flash 후보

- Winbond W25Q128JV는 128 Mbit, 즉 16 MB class 3 V serial flash이다.
- Dual/Quad SPI/QPI 계열을 지원하며, Teensy 4.1 하단 QSPI flash pads에 납땜하는 후보로 테스트 코드가 있다.
- 프로젝트 테스트는 LittleFS QSPIFlash로 mount/format/write/read 검증을 수행하고, 예상 용량을 16 MB로 둔다.

### 실무 주의사항

- Teensy 4.1 하단에는 PSRAM/flash용 QSPI pads가 있다. PJRC 공식 설명에 따르면 작은 pads는 PSRAM 위치, 큰 pads는 특정 flash chip에 사용할 수 있다.
- QSPI flash 납땜은 육안상 붙어 보여도 corner pin open/bridge가 흔하다. JEDEC ID read, LittleFS format, 반복 write/read로 검증해야 한다.
- NOR flash erase/write는 읽기보다 훨씬 느리고 blocking 구간이 생긴다. flight scheduler에 직접 write하면 sensor task jitter가 생길 수 있다.
- microSD와 QSPI flash를 동시에 쓰는 구조에서는 “RAM buffer -> 낮은 우선순위 drain” 패턴이 맞다. 현재 프로젝트도 RAM buffer/drain 상수를 둔다.

## 12. Buzzer, LED, Pyro 관련 하드웨어 메모

센서는 아니지만 bring-up과 안전 확인에서 중요하다.

| 항목 | 현재 핀 / 상태 |
| --- | --- |
| Buzzer | D2 |
| LED2 / status | D33 |
| Pyro 1 / Drogue | D28, D29, sense D25 |
| Pyro 2 / Main | D37, D36, sense D40 |
| Pyro real output | `NURA_ENABLE_PYRO_OUTPUTS` 없으면 dry-run |

주의:

- Pyro 출력은 패킷 수신, 디버그 명령, raw state assignment에서 직접 energize하면 안 된다.
- MOSFET/TC4452 driver 주변은 bench에서 LED/저항 dummy load/scope로 먼저 확인하고, e-match는 최후 단계에 연결해야 한다.
- D37/D36/D40 같은 Teensy 하단/특수 핀은 soldering 접근성과 SDIO/QSPI/보드 리비전 영향을 반드시 확인해야 한다.

## 13. 통합 bring-up 체크리스트

### 전원/보호

- VIN-VUSB cut 여부 결정: 외부 전원 + USB 디버그를 같이 쓸 비행체라면 cut 권장.
- cut 후 continuity 확인: VIN과 VUSB 사이가 open인지 멀티미터로 확인.
- USB만으로 업로드해야 하는 상황이면 VIN 외부 전원을 같이 공급하거나 cut을 복구해야 한다.
- 모든 센서 VCC가 3.3 V인지, 5 V tolerant breakout인지, IC 직접 연결인지 구분.
- 공통 GND 확인.
- RF 송신/SD write/pyro dummy load 시 3.3 V rail droop 측정.

### I2C

- I2C0 scanner: MPL3115A2 `0x60`.
- I2C1 scanner: LIS3MDL `0x1C`.
- 각 버스 pull-up이 과도하게 병렬로 붙지 않았는지 확인.
- LIS3MDL address jumper가 `0x1C` 위치인지 확인.

### SPI

- SPI0: LSM6DSO32 CS D10, H3LIS331DL CS D0 각각 WHO_AM_I 확인.
- SPI1: SX1262 NSS D9, DIO1 D31, BUSY D32 level 확인.
- 모든 SPI CS는 boot/reset 중 idle HIGH 유지.
- LoRa 송신 전 안테나/dummy load 연결.

### 센서 sanity

- LSM6DSO32: 정지 상태 accel norm 약 1 g, gyro near zero, WHO_AM_I `0x6C`.
- H3LIS331DL: WHO_AM_I `0x32`, 정지 1 g 방향 확인, range 설정 확인.
- MPL3115A2: 지상 압력 80~110 kPa 범위, baseline 생성 후 고도 0 m 근처.
- LIS3MDL: 값 finite, 주변 자석/전류에 따른 saturation 확인.
- GNSS: NMEA checksum pass 증가, satellites/HDOP/fix age 확인.
- 전압 센스: 실제 pack voltage와 telemetry mV 비교.

## 14. 권장 후속 작업

1. `test/sensor_test/README.md`의 오래된 Pyro pin 기록을 현재 핀맵에 맞게 수정한다.
2. 실제 PCB schematic에서 SX1262 `RXE`, `TXEN`, `NRESET`, `DIO2`, `DIO3/TCXO` 연결 여부를 확정한다.
3. LSM6DSO32/H3LIS331DL 축 방향을 로켓 body frame 기준으로 문서화한다.
4. MPL3115A2 discontinued 이슈 때문에 MPL3115A2S 또는 다른 기압계 후보를 부품 수급 관점에서 검토한다.
5. LIS3MDL hard/soft iron calibration 절차와 저장 위치를 정한다.
6. VIN-VUSB cut 적용 여부를 avionics assembly 절차서에 넣는다.
7. LoRa 920.9 MHz 안테나 matching/range test 결과를 별도 RF 검증 문서로 남긴다.

## 15. 주요 웹 출처

- PJRC Teensy 4.1 공식 페이지: https://www.pjrc.com/store/teensy41.html
- PJRC 외부 전원/USB 분리 가이드: https://www.pjrc.com/teensy/external_power.html
- Adafruit LSM6DSOX/ISM330DHC/LSM6DSO32 guide: https://learn.adafruit.com/lsm6dsox-and-ism330dhc-6-dof-imu
- ST LSM6DSO32 datasheet: https://www.st.com/resource/en/datasheet/lsm6dso32.pdf
- Adafruit H3LIS331/LIS331 guide: https://learn.adafruit.com/adafruit-h3lis331-and-lis331hh-high-g-3-axis-accelerometers
- ST H3LIS331DL datasheet: https://www.st.com/resource/en/datasheet/h3lis331dl.pdf
- Adafruit LIS3MDL guide: https://learn.adafruit.com/lis3mdl-triple-axis-magnetometer
- ST LIS3MDL datasheet: https://www.st.com/resource/en/datasheet/lis3mdl.pdf
- NXP MPL3115A2 product page: https://www.nxp.com/products/MPL3115A2
- NXP MPL3115A2 datasheet: https://www.nxp.com/docs/en/data-sheet/MPL3115A2.pdf
- u-blox NEO-6 datasheet: https://content.u-blox.com/sites/default/files/products/documents/NEO-6_DataSheet_%28GPS.G6-HW-09005%29.pdf
- u-blox 6 Receiver Description / Protocol Specification: https://content.u-blox.com/sites/default/files/products/documents/u-blox6_ReceiverDescrProtSpec_%28GPS.G6-SW-10018%29_Public.pdf
- Semtech SX1261/2 datasheet mirror: https://cdn.sparkfun.com/assets/6/b/5/1/4/SX1262_datasheet.pdf
- Semtech SX1276/77/78/79 datasheet mirror: https://www.mouser.com/datasheet/2/761/sx1276-1278113.pdf
- Winbond W25Q128JV product page: https://www.winbond.com/hq/new-online-purchasing-guide/index.html?__locale=en&pLine=%2Fproduct%2Fcode-storage-flash%2Fqspi-nor%2F&pNo=W25Q128JV
