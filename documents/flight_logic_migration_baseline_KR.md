# 비행 로직 Zephyr 마이그레이션 기준선

문서 상태: 구현 기준, 하드웨어 검증 전 비행 금지, 2026-07-11

목적: RTOS 포팅 중 기존 executable behavior를 보존했음을 검토할 기준

## 1. 변경 범위

이번 변경은 거대 `FlightStateMachineTask`를 Coordinator와 상태 앱으로 분리하고
실행/통신/하드웨어 경계를 바꾼다. 전이식, threshold, confirmation count, fallback을
튜닝하는 작업이 아니다.

현재 실행 기준:

- `modules/domain/flight/include/nura/flight/flight_policy.h`
- `modules/domain/flight/src/flight_coordinator.cpp`
- `modules/domain/flight/src/apps/*.cpp`

이전 bare-metal 기준 구현은 이관 동등성 검증 후 저장소에서 제거했다. 따라서 이
문서는 과거 소스의 재실행 방법이 아니라, 현재 Zephyr 코드의 판단 정책과 검증
책임을 정의하는 source of truth다.

## 2. 입력과 단위

| 입력 | 단위 | 계약 | 사용 상태 |
|---|---|---|---|
| low-g acceleration XYZ | m/s² | `LowGImuSample` | ARMED, LAUNCH; baro fault tilt fallback |
| high-g acceleration XYZ | g | `HighGImuSample` | low-g unusable 시 ARMED/LAUNCH fallback |
| filtered barometer AGL | m | `BarometerSample` | COAST, DROGUE, DEPLOY |
| tilt angle | degree | `LowGImuSample` | COAST barometer fault fallback만 |
| abort active | bool | `SafetyStatus -> FlightInputs` | SAFE 외 모든 상태 |
| force recovery command | sequence | `CommandRequest` | LAUNCH/COAST만 |

센서 adapter는 axis mapping, scale, calibration, reference AGL, filtering을 완료한 뒤
contract를 publish해야 한다. Domain은 sensor register/range/board orientation을 모른다.
현재 contract schema version은 `2`이며 전체 sensor application health field를 포함한다.

## 3. 상태별 허용/금지

| 상태 | 실행 판단 | 금지 사항 |
|---|---|---|
| INIT | SAFE 전이 | recovery ON |
| SAFE | abort 해제 시 ARMED | state detector와 recovery ON |
| ARMED | launch detector | pyro ON, burnout/apogee 판단 |
| LAUNCH | burnout detector, force recovery 허용 | main ON |
| COAST | apogee detector, force recovery 허용 | main ON |
| APOGEE | drogue sequence | main ON |
| DROGUE | main deploy detector | drogue 새 판단 시작 |
| DEPLOY | main pulse와 landing detector | drogue/main 재점화 정책 추가 |
| GROUND | all off | recovery ON |
| FAULT | all off | recovery ON |

abort가 SAFE 외 상태에서 active이면 가장 먼저 SAFE로 전이하고 `ALL_OFF` intent를
발행한다.

## 4. 전이 기준과 출처

모든 값은 이관 전에 고정한 executable policy를 `FlightPolicy`에 옮긴 것이다.
물리 검증이나 team 승인 출처가 별도로 없는 값은 이번 포팅에서 새로 정당화하지
않았다.

| 판단 | 현재 값 | 현재 출처 |
|---|---:|---|
| launch threshold | >= 2.0 g, 4 fresh samples | 기존 code/test |
| burnout threshold | < 1.0 g, 4 fresh samples | 기존 code/test |
| acceleration max age | 50 ms | 기존 code/test |
| apogee min flight time | 8000 ms | 기존 code/test |
| apogee fit window | 9 samples | 기존 code/test |
| prediction history/confirm | 5 / 3 samples | 기존 code/test |
| apogee fallback drop | 4.0 m, 4 samples | 기존 code/test |
| apogee timeout from COAST | 12000 ms | 기존 code/test |
| baro-fault tilt fallback | >= 70 deg, 5 samples, after 8000 ms | 기존 code/test |
| main altitude | <= 200 m AGL | 기존 code/test |
| main timeout from DROGUE | 15000 ms | 기존 code/test |
| landing stable window | 20 samples, range <= 0.5 m | 기존 code/test |
| barometer max sample gap | 150 ms | 기존 code/test |
| barometer stuck | 5000 ms window, range <= 0.2 m | 기존 code/test |
| drogue backup delay | 2000 ms | 기존 code/test |
| pyro pulse | **1000 ms** | 팀이 확정한 최신 mission policy |

## 5. Pyro pulse 정책과 하드웨어 검증

Pyro pulse는 **1000 ms**가 최신 확정값이며 `FlightPolicy`가 유일한 코드 기준이다.
이 값은 포팅 중 변경하지 않는다. 실제 보드에 올리기 전에는 아래의 출력 경로
검증이 필요하다.

다음 자료 없이 pulse를 변경하거나 비행 승인하지 않는다.

1. igniter/pyro channel 전기 특성 및 1000 ms pulse 적합성
2. battery worst-case voltage와 wiring resistance
3. MOSFET/PCB thermal 및 current measurement
4. disconnected dummy-load scope test
5. 실제 igniter ground test
6. team safety review와 승인 기록

## 6. Fallback과 failure mode

| Failure | 현재 동작 |
|---|---|
| low-g unusable | health/connected/fresh high-g fallback |
| barometer fault in COAST | 최소 비행시간 이후 tilt confirmation 또는 apogee timeout |
| barometer stuck | fault latch; COAST는 tilt/timeout, DROGUE는 main timeout |
| barometer gap | apogee/landing sample window reset |
| main altitude 판단 불가 | 15 s timeout |
| force recovery wrong state | reject decision, 전이 없음 |
| stale/wrong-state ON intent | RecoveryActuationApp 거부 |
| output write fail | best-effort all off, app FAILED |
| abort | SAFE + all off |

prediction은 품질 gate, curvature/RMSE/jump/sigma 제한과 descent/timeout fallback 뒤에
있다. 예측만으로 fallback을 제거하지 않는다.

## 7. 검증 계획과 현재 결과

완료:

- legacy 기준과의 이관 동등성 검토 완료 후 legacy 소스 제거
- 새 Domain/Core 엄격 compiler warning 통과
- 새 host full-flight replay `INIT -> GROUND` 통과
- abort, force command state guard, stale actuation guard test 통과
- recovery pulse intent 순서와 final all-off 확인
- TraceMap 8개 전이와 sequence gap 0 확인
- Zephyr native_sim 실제 thread boot/smoke 통과
- Teensy 4.1 cross-build 통과

미완료:

- 실제 sensor log 기반 tick-by-tick replay corpus
- 실제 Teensy sensor adapter와 calibration/filter 동등성
- hardware recovery output scope/current test
- HIL, stack/WCET, watchdog reset/recovery
- storage/radio/uplink Zephyr port

따라서 현재 결과는 architecture와 host behavior migration 검증이며 flight
qualification이 아니다.
