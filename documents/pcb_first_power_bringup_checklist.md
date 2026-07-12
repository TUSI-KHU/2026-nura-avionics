# NURA 에비오닉스 PCB 최초 전원 인가 Bring-Up 체크리스트

상태: 하드웨어 bring-up 체크리스트  
대상: 모든 센서가 실장된 신규 PCB 조립체  
작성일: 2026-06-26  

## 절대 안전 규칙

이 체크리스트를 수행하는 동안 e-match, 니크롬선, 사출 화약, 기타
에너지를 가진 부하를 절대 연결하지 않는다.

`J_PYRO1`과 `J_PYRO2`는 전원, 도통, 펌웨어, 오실로스코프, 더미 부하
테스트가 모두 통과할 때까지 반드시 비워둔다. 최초 전원 인가 테스트는
보드 상태 확인 테스트이지, 사출 테스트가 아니다.

## 현재 보드에 대한 전제

### Pyro 커넥터 및 MOSFET 구조

Pyro 커넥터 구조는 다음과 같이 본다.

| 커넥터 | 핀 / 넷 | 예상 역할 |
| --- | --- | --- |
| `J_PYRO1` pin 1 | `+BATT_ARMED` | Pyro 부하로 들어가는 arming 이후 배터리 전원 |
| `J_PYRO1` pin 2 | `/PYRO1_2` | Pyro 1 MOSFET drain 쪽 |
| `J_PYRO2` pin 1 | `+BATT_ARMED` | Pyro 부하로 들어가는 arming 이후 배터리 전원 |
| `J_PYRO2` pin 2 | `/PYRO2_2` | Pyro 2 MOSFET drain 쪽 |

관측/추정되는 MOSFET 묶음은 다음과 같다.

| 채널 | 회수 역할 | MOSFET | MCU 출력 쌍 |
| --- | --- | --- | --- |
| Pyro 1 | Drogue | `Q1`, `Q2` 병렬 구동 | `D28`, `D29` |
| Pyro 2 | Main | `Q3`, `Q4` 병렬 구동 | `D35`, `D38`로 이동됨 |

중요: Pyro 2는 기존 `D36`/`D37` 쌍에서 이동했다. 신규 보드 bring-up
기준은 다음과 같다.

```text
Pyro 2 output pair: D35 and D38
Pyro 2 sense: D40
```

Pyro 2의 정확한 `gpio1`/`gpio2` 순서는 펌웨어에서 실제 출력을 켜기 전,
Teensy 핀에서 MOSFET gate/driver 입력 넷까지 도통을 찍어서 확인해야
한다. 이 확인이 끝나기 전까지는 다음과 같이 기록한다.

```text
Pyro2 GPIO pair = {D35, D38}
Pyro2 gpio1/gpio2 order = 하드웨어 도통 확인 TODO
```

### LoRa Reset/RXE 구조

현재 PCB 이해 기준으로 SX1262 LoRa reset net은 Teensy가 제어하지 않는다.

확인된 구조:

- `U7` LoRa `RST`는 `R2`를 통해 `+3V3_LORA`로 pull-up된다.
- `JP2`는 `LORA_RST`를 GND로 당길 수 있다.
- `JP2`가 실수로 쇼트되어 있으면 SX1262는 영원히 reset 상태에 머문다.
- `RXE`는 Teensy `D30`에 연결된 별도의 RF 제어 넷이다.

펌웨어 관점의 의미:

| Net | PCB | 현재 코드 리스크 |
| --- | --- | --- |
| `LORA_RST` | R2 pull-up, JP2-to-GND 옵션 | 실제 MCU reset net이 발견되지 않는 한 코드에서는 no-reset 상태로 유지해야 함 |
| `RXE` | Teensy `D30` | 전체 LoRa 테스트 전 `board_pinmap.h`에서 `rxEnablePin = 30`으로 지정해야 함 |

### LoRa SPI1 배선

| SX1262 net | Teensy pin |
| --- | --- |
| `SO` / MISO | `D1` |
| `SI` / MOSI | `D26` |
| `SCK` | `D27` |
| `NSS` | `D9` |
| `RXE` | `D30` |
| `DIO1` | `D31` |
| `BUSY` | `D32` |

과거 실패 모드: LoRa MISO가 GND와 쇼트된 적이 있다. `U7` SO/MISO와 GND
사이가 0옴에 가깝게 측정되면 보드에 전원을 넣지 않는다.

### LIS3MDL 실장 관련 주의

현재 보드는 LIS3MDL이 미실장된 상태로도 테스트할 수 있다. 나중에
LIS3MDL을 실장한다면, `0x1C`로 응답할 것이라고 가정하기 전에 I2C
strap net을 반드시 확인한다.

알려진 우려:

- 이전 수작업 배선에서는 `CS = 3.3 V`, `DO/SDO = GND`가 필요했다.
- Gerber 검토상 `DO`, `CS`, 위쪽 GND pad 하나가 unconnected일 가능성이
  있었다.
- 이 strap들이 실제로 연결되어 있지 않으면 LIS3MDL이 기대한 I2C
  mode/address로 고정되지 않을 수 있다.

I2C1 버스 자체에는 pull-up(`R23`, `R24`)이 있으므로, LIS3MDL이 없어도
SDA/SCL floating 문제는 줄어들 가능성이 있다.

## 전체 기능 테스트 전 수정해야 할 펌웨어/PCB 불일치

모든 하드웨어를 실제로 사용하는 빌드를 돌리기 전에 다음 코드 매핑을
수정하거나 확인해야 한다.

| 기능 | PCB / 수정된 보드 | 현재 리스크 |
| --- | --- | --- |
| Pyro 2 output pair | `D35`, `D38` | `include/board_pinmap.h`에서 Pyro2 GPIO가 아직 unassigned일 수 있음 |
| Pyro 2 sense | `D40` | 도통 확인 시 `D40`으로 유지해야 함 |
| LoRa RXE | `D30` | `Sx1262LoRa::rxEnablePin`이 아직 `kUnassignedPin`일 수 있음 |
| LoRa reset | MCU reset 없음; R2 pull-up / JP2 GND | `D30`을 reset으로 매핑하면 안 됨 |

Pyro2 순서 확인과 dummy-load 테스트가 끝날 때까지
`NURA_ENABLE_PYRO_OUTPUTS`를 정의하지 않는다.

## 필요 장비

- 도통/저항 모드와 DC 전압 모드가 있는 디지털 멀티미터.
- 최초 외부 전원 테스트용 전류 제한 가능 bench supply.
- 오실로스코프. 가능하면 2채널 이상.
- Teensy serial/programming용 USB 케이블.
- 이후 pyro bench test용 dummy load. 최초 전원 인가에는 사용하지 않음.
- 에너지 부하 없음.

최초 보드 단독 전원 테스트의 권장 bench supply 제한:

```text
시작 전압: 의도한 low-side safe input 또는 동등한 실험실 전원을 사용
전류 제한: 손상 전 쇼트를 잡을 수 있을 만큼 낮게 시작
전류 제한 상향: rail 검증이 끝난 뒤에만 수행
```

팀의 실제 regulator/current budget이 있으면 그 값을 사용한다. 모르면
보수적으로 시작하고, 전압을 올리는 동안 전류 거동을 계속 관찰한다.

## Phase 0: 계측 전 육안 검사

확인 항목:

- Teensy 핀, LoRa 핀, regulator 핀, MOSFET, pyro connector pad에 납브릿지
  없음.
- Teensy, SX1262, GPS, MPL3115A2, LSM6DSO32, H3LIS331DL, LIS3MDL 실장 시
  방향이 맞음.
- Pyro2 D35/D38 reroute 주변에 copper debris 없음.
- JP2가 실수로 `LORA_RST`를 GND에 쇼트시키고 있지 않음.
- Pyro connector는 비어 있음.
- 배터리/arming switch는 OFF.
- 저항 측정 중 USB는 분리.

실패 기준:

- 전원 rail, LoRa MISO, pyro drain, MOSFET gate 주변에 눈에 보이는 bridge가
  있으면 테스트 중단.
- Pyro2 D35/D38 수정 상태가 불확실하면, 도통으로 net이 증명될 때까지
  pyro-output 관련 테스트 중단.

## Phase 1: 무전원 도통 및 저항 검사

전원은 완전히 분리되어 있어야 한다. USB도 분리한다. 배터리도 분리한다.
특정 단계에서 ON이라고 명시하지 않는 한 arming switch는 OFF 상태여야 한다.

### 전체 전원 rail

저항/도통을 측정한다.

| Probe A | Probe B | 기대 결과 | 중단 조건 |
| --- | --- | --- | --- |
| `+BATT` | GND | 쇼트 아님 | 0옴 근처 |
| `+5V6` | GND | 쇼트 아님 | 0옴 근처 |
| `+5V` | GND | 쇼트 아님 | 0옴 근처 |
| `+3V3` | GND | 쇼트 아님 | 0옴 근처 |
| `+3V3_LORA` | GND | 쇼트 아님 | 0옴 근처 |
| `+BATT` | `+BATT_ARMED`, switch OFF | open / 매우 높은 저항 | 낮은 저항 |
| `+BATT` | `+BATT_ARMED`, switch ON | F1/switch 경로로 연결 | 연결되어야 할 때 open |

해석:

- capacitor가 멀티미터 전류로 잠깐 충전되면서 저항값이 변할 수 있다.
  안정적으로 0옴에 가까운 값이 유지되는 것이 위험 신호다.
- 단순 pass/fail만 쓰지 말고 대략적인 저항값도 기록한다.

### Pyro connector 쇼트 검사

e-match/니크롬은 연결하지 않는다.

| Probe A | Probe B | 기대 결과 |
| --- | --- | --- |
| `J_PYRO1` pin 1 / `+BATT_ARMED` | GND | 쇼트 없음 |
| `J_PYRO1` pin 2 / `/PYRO1_2` | GND | hard 0옴 쇼트 없음 |
| `J_PYRO2` pin 1 / `+BATT_ARMED` | GND | 쇼트 없음 |
| `J_PYRO2` pin 2 / `/PYRO2_2` | GND | hard 0옴 쇼트 없음 |
| `J_PYRO1` pin 1 | `J_PYRO1` pin 2 | 부하 없을 때 open |
| `J_PYRO2` pin 1 | `J_PYRO2` pin 2 | 부하 없을 때 open |

어떤 pyro connector pin이라도 GND나 반대쪽 pyro connector pin과 예상치 않게
hard-short되어 있으면 즉시 중단한다.

### Pyro MOSFET gate/MCU 매핑

펌웨어 핀맵을 수정하거나 출력을 enable하기 전에 수행한다.

| Net / point | 예상 매핑 |
| --- | --- |
| Pyro 1 gate/input A | Teensy `D28` |
| Pyro 1 gate/input B | Teensy `D29` |
| Pyro 1 sense | Teensy `D25` |
| Pyro 2 gate/input A | `D35` 또는 `D38` 중 하나 |
| Pyro 2 gate/input B | `D35` 또는 `D38` 중 나머지 하나 |
| Pyro 2 sense | Teensy `D40` |

정확한 결과를 다음 형식으로 기록한다.

```text
Pyro2 gpio1 = D__
Pyro2 gpio2 = D__
Pyro2 sense = D40 확인됨 / 미확인
```

layout 모양만 보고 gpio1/gpio2 순서를 추측하지 않는다. Teensy pad/header에서
MOSFET gate/driver net까지 도통으로 확인한다.

### LoRa reset 및 SPI net

| Probe A | Probe B | 기대 결과 | 중단 조건 |
| --- | --- | --- | --- |
| `LORA_RST` | GND | 쇼트 없음 | 0옴 근처 |
| `LORA_RST` | `+3V3_LORA` | `R2`를 통한 저항 경로 | hard short 또는 R2가 있어야 하는데 open |
| `U7` SO/MISO pin | GND | 쇼트 없음 | 0옴 근처 |
| `U7` SO/MISO pin | Teensy `D1` | 도통 | open |
| `U7` SI/MOSI pin | Teensy `D26` | 도통 | open |
| `U7` SCK pin | Teensy `D27` | 도통 | open |
| `U7` NSS pin | Teensy `D9` | 도통 | open |
| `U7` BUSY pin | Teensy `D32` | 도통 | open |
| `U7` DIO1 pin | Teensy `D31` | 도통 | open |
| `RXE` net | Teensy `D30` | 도통 | open |

반드시 반복 확인해야 할 핵심 항목:

```text
LoRa MISO / U7 SO / Teensy D1은 GND와 쇼트되어 있으면 안 된다.
```

### I2C net

| Bus | Net | 기대 결과 |
| --- | --- | --- |
| I2C0 | SDA `D18` to GND | 쇼트 없음 |
| I2C0 | SCL `D19` to GND | 쇼트 없음 |
| I2C1 | SDA `D17` to GND | 쇼트 없음 |
| I2C1 | SCL `D16` to GND | 쇼트 없음 |
| I2C0 | SDA/SCL to `+3V3` | `R21/R22`를 통한 pull-up 저항 경로 |
| I2C1 | SDA/SCL to `+3V3` | `R23/R24`를 통한 pull-up 저항 경로 |

### SPI0 센서 net

| Net | 예상 매핑 |
| --- | --- |
| SPI0 MISO | Teensy `D12`, LSM/H3L 공유 |
| SPI0 MOSI | Teensy `D11` |
| SPI0 SCK | Teensy `D13` |
| LSM CS | Teensy `D10` |
| H3L CS | Teensy `D0` |

모든 SPI0 data line과 두 CS line은 GND 또는 `+3V3`에 쇼트되어 있으면 안 된다.

### UART/GPS net

| Net | 예상 매핑 |
| --- | --- |
| GPS TXD | Teensy RX3 `D15` |
| GPS RXD | Teensy TX3 `D14` |

두 UART line 모두 GND 또는 3.3 V와 쇼트되어 있지 않은지 확인한다.

## Phase 2: 펌웨어 하드웨어 구동 없이 최초 전원 인가

조건:

- Pyro connector는 비어 있음.
- `+BATT_ARMED` 확인을 제외하면 arming switch는 OFF.
- Bench supply는 current limit 설정.
- 최초 DC rail 측정 시 USB는 분리해도 된다.

전원 인가 후 관찰:

| 테스트 | 기대 결과 |
| --- | --- |
| Bench supply current | 갑작스러운 current-limit hit 없음 |
| Regulator 온도 | 빠른 발열 없음 |
| `+5V6` | 예상 regulator 출력, 안정적 |
| `+5V` | 예상 5 V rail, 안정적 |
| `+3V3` | 약 3.3 V, 안정적 |
| `+3V3_LORA` | LoRa rail이 enable된 경우 약 3.3 V |
| `+BATT_ARMED`, switch OFF | `+BATT`와 연결되지 않음 |
| `+BATT_ARMED`, switch ON | 보호/arming battery path를 따라감 |

중단 조건:

- 어떤 rail이라도 collapse.
- 전원이 current limit에 걸림.
- regulator, MOSFET, LoRa module, 센서 중 하나라도 뜨거워짐.
- switch OFF 상태여야 하는데 `+BATT_ARMED`가 살아 있음.

## Phase 3: 전원 인가 후 DC 핀 전압 검사

보드 GND 기준으로 측정한다.

### LoRa

| Net | 펌웨어 toggle 전 기대 DC level |
| --- | --- |
| `+3V3_LORA` | 약 3.3 V |
| `LORA_RST` | R2 pull-up을 통해 약 3.3 V |
| `JP2` GND side | 0 V |
| `NSS` / D9 | firmware init 이후 보통 idle high, init 전에는 미정 |
| `BUSY` / D32 | idle 시 low, radio busy 구간에서 high |
| `DIO1` / D31 | 보통 idle low |
| `RXE` / D30 | 펌웨어에 따라 다름. LoRa 테스트 중 반드시 확인 |
| MISO / D1 | 0 V에 붙어 있지 않음. 쇼트 없음 |

`LORA_RST`가 0 V라면 JP2 쇼트 또는 납브릿지를 의심한다.

### I2C

| Net | 기대 idle 전압 |
| --- | --- |
| I2C0 SDA D18 | idle 약 3.3 V |
| I2C0 SCL D19 | idle 약 3.3 V |
| I2C1 SDA D17 | idle 약 3.3 V |
| I2C1 SCL D16 | idle 약 3.3 V |

idle이 0 V 근처이면 hard short 또는 stuck device를 의심한다. idle이 중간
전압이면 약한 pull-up, 잘못된 전원 rail, partial short, 전원 미인가 device
leakage를 의심한다.

### SPI0

센서 펌웨어가 돌기 전 SPI line은 조용할 수 있다. 펌웨어 probe 중에는:

| Net | 기대 결과 |
| --- | --- |
| D13 SCK | SPI transfer 중 burst |
| D11 MOSI | transfer 중 burst |
| D12 MISO | WHOAMI/read 중 응답 activity |
| D10 LSM CS | idle high, LSM transaction 중 low |
| D0 H3L CS | idle high, H3L transaction 중 low |

### 부하 없는 Pyro 출력

Pyro load를 연결하지 않는다.

펌웨어에서 physical pyro output이 disabled인 경우:

| Net | 기대 결과 |
| --- | --- |
| Pyro1 D28/D29 | flight logic 때문에 능동 pulse가 나오면 안 됨 |
| Pyro2 D35/D38 | flight logic 때문에 능동 pulse가 나오면 안 됨 |
| J_PYRO1 pin 2 | 예상치 못한 low-side hard-on 없음 |
| J_PYRO2 pin 2 | 예상치 못한 low-side hard-on 없음 |

이후 명시적인 bench-only GPIO test build를 사용하고 에너지 부하가 없는 경우:

- D28/D29와 D35/D38을 scope로 본다.
- 먼저 idle 상태를 확인한다.
- 제어된 bench test에서만 pulse width가 `PYRO_FIRE_DURATION_MS`와 일치하는지
  확인한다.
- 이 단계에서는 battery-fed pyro load를 연결하지 않는다.

## Phase 4: 오실로스코프 테스트

보드 GND spring 또는 짧은 ground lead를 사용한다. 긴 ground clip은 빠른
SPI edge를 실제보다 나쁘게 보이게 만들 수 있다.

### 전원 rail

Scope 대상:

| Rail | 확인 항목 |
| --- | --- |
| `+3V3` | sensor init 및 SD write 중 ripple |
| `+3V3_LORA` | LoRa init/TX 중 droop/ripple |
| `+5V` | SD write 및 buzzer 중 droop |
| `+BATT_ARMED` | arming switch OFF일 때 예상치 못한 pulse 없음 |

정량 acceptance target은 프로젝트별로 정해야 하지만, 정성적으로는:

- MCU reset을 유발할 큰 dip 없음.
- regulator rail oscillation 없음.
- RF TX로 인한 brownout 없음.

### I2C0 / I2C1

I2C scanner 또는 sensor init을 돌리면서 SDA/SCL을 scope로 본다.

기대 결과:

- Idle high가 3.3 V 근처.
- Low pulse가 0 V 근처까지 깨끗하게 내려감.
- 펌웨어가 바꾸지 않는 한 SCL은 설정된 100 kHz 근처.
- 기대 address에서 ACK bit가 존재:
  - MPL3115A2: I2C0의 `0x60`.
  - LIS3MDL: 실장되어 있고 strap이 맞으면 I2C1의 `0x1C`.

실패 패턴:

- SDA stuck low: device가 bus를 잡고 있음, short, 잘못된 power sequencing.
- SCL stuck low: short 또는 device stretch/fault.
- 둘 다 중간 전압: pull-up 또는 leakage 문제.

### SPI0 센서

센서 펌웨어 실행 중 D13, D11, D12, D10, D0를 scope로 본다.

기대 결과:

- D13 SCK burst.
- 한 번에 하나의 CS만 low.
- read 중 MISO D12가 변화.
- LSM WHOAMI가 기대 LSM6DSO32 값으로 확인.
- H3L WHOAMI가 `0x32`로 확인.

실패 패턴:

- 두 CS가 동시에 low: firmware/pin fault.
- MISO가 0 V flat: short 또는 unpowered sensor.
- MISO가 3.3 V flat: device response 없음, pull-up/leakage, wrong mode.
- MISO가 중간 전압: contention 또는 partial short.

### SPI1 LoRa

LoRa init 중 D27, D26, D1, D9, D30, D31, D32를 scope로 본다.

기대 결과:

- NSS D9는 idle high, command 중 low.
- SCK D27은 command 중 burst.
- MOSI D26은 command byte를 전달.
- MISO D1은 register/status read 중 constant가 아닌 데이터를 반환.
- BUSY D32는 SX1262 command timing에 따라 전이.
- DIO1 D31은 IRQ 전까지 low 유지.
- RXE D30은 firmware가 매핑된 뒤 설정된 RF path 동작을 따라감.

LoRa 세부 acceptance check:

1. `LORA_RST`가 high 유지.
2. `BUSY`가 영원히 high에 stuck되지 않음.
3. MISO가 0 V가 아니고, 쇼트가 아니며, 모든 transfer에서 flat하지 않음.
4. SX1262 init이 최소 10/10 cold 또는 reset cycle에서 반복 성공.
5. Downlink packet이 동일 frequency/BW/SF/CR/sync word를 쓰는 SX1276 지상국에
   수신됨.

### Pyro GPIO 오실로스코프 테스트

모든 rail, 센서, 로깅, LoRa 확인이 끝난 뒤에만 수행한다.

준비:

- e-match 없음.
- 니크롬 없음.
- 최초 GPIO-only scope test에서는 pyro battery feed도 없는 쪽을 권장.
- bench-only firmware build 사용.
- 명시적인 테스트에서만 `NURA_ENABLE_PYRO_OUTPUTS` 정의.

Scope 대상:

| 채널 | 오실로스코프로 볼 핀 |
| --- | --- |
| Pyro 1 / Drogue | D28, D29, J_PYRO1 pin 2 |
| Pyro 2 / Main | D35, D38, J_PYRO2 pin 2 |

기대 결과:

- Boot 이후 출력 idle OFF.
- `SAFE`, `GROUND`, `FAULT`는 모든 출력을 OFF로 강제.
- Pyro1 sequence는 제어된 drogue test path에서만 D28/D29 pulse.
- Pyro2 sequence는 제어된 main test path에서만 D35/D38 pulse.
- telemetry, USB, SD dump, packet parsing만으로 어떤 채널도 fire되지 않음.

중단 조건:

- 어떤 pyro output이라도 init 중 예상치 않게 pulse.
- Pyro2 새 D35/D38이 아니라 기존 D36/D37이 pulse.
- D21 battery sense가 pyro output 상태와 상호작용.

## Phase 5: 권장 펌웨어 테스트 순서

### 1. 전원 전용 smoke firmware

목표: pyro를 건드리지 않고 USB serial과 기본 보드 전원을 확인.

기대 결과:

- Teensy가 USB serial로 enumerate.
- 필요하면 buzzer/LED 테스트 가능.
- LoRa 또는 pyro output이 구동되지 않음.

### 2. I2C scanner

기대 결과:

| Bus | 기대 device |
| --- | --- |
| I2C0 D18/D19 | MPL3115A2 at `0x60` |
| I2C1 D17/D16 | LIS3MDL이 실장된 경우에만 `0x1C` |

### 3. SPI0 sensor probe

기대 결과:

- H3LIS331DL WHOAMI `0x32`.
- LSM6DSO32 WHOAMI 기대값.
- CS line은 idle high이고 서로 overlap되지 않음.

### 4. microSD logging

기대 결과:

- SD가 안정적으로 초기화.
- `/NURA_LOG/FLxxx.NLG` 생성.
- Parsed log에 BOOT, state event, FAST/SLOW frame 포함.

### 5. Program/QSPI flash logging

기대 결과:

- W25Q128 또는 선택된 flash read/write verify가 반복 통과.
- Flight logger primary storage가 append/rotate를 수행해도 SD mirror를
  corrupt하지 않음.

### 6. LoRa init and downlink

이 테스트 전 firmware pinmap 수정:

```cpp
Sx1262LoRa::rxEnablePin = 30U
Sx1262LoRa::resetPin = -1
```

기대 결과:

- `LORA_RST` high.
- SX1262 init 10/10 성공.
- 지상 SX1276이 유효한 NURA packet 수신.
- SPI1 timing이 sensor/FSM task period를 밀지 않음.

### 7. Pyro GPIO-only bench test

이 테스트 전 도통으로 순서를 확인한 뒤 firmware pinmap 수정:

```cpp
Pyro2::gpio1Pin = D35 또는 D38, 도통 확인 결과 기준
Pyro2::gpio2Pin = 나머지 핀
Pyro2::sensePin = D40
```

기대 결과:

- 부하 없음.
- 먼저 scope-only 또는 LED/dummy logic load 사용.
- `NURA_ENABLE_PYRO_OUTPUTS`는 bench test build에서만 사용.
- Pyro1과 Pyro2는 각자의 명령된 state-machine sequence에서만 pulse.

## Phase 6: Acceptance 기록 양식

모든 보드 테스트를 다음 형식으로 기록한다.

```text
보드 serial / 식별자:
날짜:
작업자:
전원 source:
전류 제한:
배터리 / bench 전압:

무전원 저항:
  +BATT-GND:
  +5V6-GND:
  +5V-GND:
  +3V3-GND:
  +3V3_LORA-GND:
  +BATT to +BATT_ARMED switch OFF:
  +BATT to +BATT_ARMED switch ON:

부하 없는 Pyro connector:
  J_PYRO1 pin1-GND:
  J_PYRO1 pin2-GND:
  J_PYRO1 pin1-pin2:
  J_PYRO2 pin1-GND:
  J_PYRO2 pin2-GND:
  J_PYRO2 pin1-pin2:

Pyro2 reroute 도통:
  D35 연결 대상:
  D38 연결 대상:
  D40 sense 확인:

LoRa:
  LORA_RST-GND:
  LORA_RST-3V3_LORA:
  U7 MISO-GND:
  U7 MISO-D1:
  BUSY-D32:
  DIO1-D31:
  RXE-D30:

전원 인가 후 rail:
  +5V6:
  +5V:
  +3V3:
  +3V3_LORA:
  +BATT_ARMED OFF:
  +BATT_ARMED ON:

오실로스코프:
  I2C0 idle/speed/ACK:
  I2C1 idle/speed/ACK:
  SPI0 LSM/H3L:
  SPI1 LoRa:
  SD write rail droop:
  LoRa TX rail droop:

펌웨어:
  I2C scanner:
  SPI0 sensor probe:
  SD log parse:
  Flash verify:
  LoRa init 10/10:
  LoRa packet RX:
  Pyro GPIO-only scope:

결과:
남은 이슈:
해결 전 비행 금지:
```

## 중단 후 수정 조건

다음 중 하나라도 참이면 다음 phase로 넘어가지 않는다.

- 어떤 power rail이라도 GND와 쇼트.
- 어떤 pyro connector pin이라도 예상치 않게 쇼트.
- Pyro2 D35/D38 mapping을 모르는 상태에서 output firmware를 enable하려 함.
- `LORA_RST` stuck low.
- LoRa MISO가 GND와 쇼트.
- I2C SDA/SCL stuck low.
- SPI MISO가 확인된 short 또는 bus contention 때문에 flat.
- USB/external power 상호작용으로 reset 또는 reverse-power 증상 발생.
- 테스트에서 microSD logging이 필요한데 실패.
- 명시적인 bench GPIO test 밖에서 어떤 pyro output이라도 상태 변화.
