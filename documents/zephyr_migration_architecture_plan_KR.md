# Zephyr 이관 및 비행 자격 검증 로드맵

문서 상태: 활성 계획, 2026-07-11

## 1. 완료한 이관 범위

저장소는 Zephyr RTOS 구조로 정리되었다. 현재 구현은 다음 계층을 갖는다.

```text
Contracts -> Domain Flight -> Core (SB/Executive/TraceMap) -> Applications
          -> PSP ports -> Zephyr adapters -> board and sensors
```

- Domain은 MCU, Zephyr, GPIO, sensor driver에 의존하지 않는다.
- Software Bus는 latest snapshot과 fixed SPSC queue만 제공하며, mission path에서
  dynamic allocation과 blocking I/O를 사용하지 않는다.
- 거대 FSM은 `FlightCoordinator`와 state application으로 분리했다.
- task 시작/종료, deadline, SB drop, FSM transition은 bounded TraceMap에 기록한다.
- App Catalog 하나에서 실제 scheduler gate와 state별 application 표를 생성한다.
- 각 sensor와 event/trace recorder는 독립 static thread와 stack을 가진다.
- state app 실제 호출 경계와 transition request/commit을 별도로 추적한다.
- `native_sim`과 `teensy41` Zephyr build, host replay, stale output guard 검증을
  완료했다.
- Arduino/PlatformIO source, tests, radio bench, receiver/sender, 그리고 해당
  문서는 cutover 후 제거했다.

## 2. 설계 결정

PSP는 보드와 모든 hardware adapter의 교체 경계다. 상위 계층은 `ILowGImu`,
`IBarometer`, `IRecoveryOutput`처럼 필요한 port만 의존한다. RAM/flash/queue/trace
용량은 Devicetree와 Kconfig profile에서 설정하며, mission code에 보드 메모리 숫자를
하드코딩하지 않는다.

Application은 mutable pointer나 driver handle을 Bus에 publish하지 않는다. 센서
snapshot은 copy-only value contract, 명령/전이/actuation은 bounded ordered queue를
쓴다. queue full 또는 trace contention은 mission loop를 대기시키지 않고 계수와
TraceMap event로 관측한다.

`FlightCoordinator`는 상태 전이, transition sequence, app enable policy만 소유한다.
각 state application은 입력 snapshot으로 decision 또는 actuation intent를 만들며,
GPIO는 `RecoveryActuationApp`의 state/sequence guard를 통과한 경우에만 PSP로 간다.

## 3. 단계별 남은 작업

### Phase A: Board and Sensor PSP

1. Teensy 4.1에서 pinctrl, clock, boot-safe GPIO default를 HIL로 검증한다.
2. low-g, high-g, barometer, magnetometer, GNSS, power monitor의 Zephyr PSP
   adapter를 구현한다.
3. adapter별 unit, scale, axis mapping, calibration, sample age를 bench fixture로
   확인한다.
4. 동일 sensor log를 host replay에 넣어 policy input이 기대 단위인지 비교한다.

Success criteria: every adapter returns explicit `OK`, `NO_DATA`, `UNAVAILABLE`, or
`FAULT`; no driver object or board-specific type reaches Domain.

### Phase B: Recovery Output Safety

1. disconnected dummy load로 boot glitch, active polarity, all-off, duration을
   oscilloscope로 측정한다.
2. battery worst-case voltage, wiring resistance, MOSFET current/thermal을
   측정한다.
3. current `1000 ms` pyro pulse가 ignition channel과 PCB limit에 맞는지 team
   safety review로 승인한다.
4. output write failure, stale intent, wrong state, abort, reset 중 all-off를 HIL로
   재현한다.

Success criteria: no direct GPIO route outside `RecoveryActuationApp`; all relevant
fault cases leave physical outputs off.

### Phase C: TraceMap and Real-Time Evidence

1. TraceMap sink를 persistent storage로 연결하고 power-loss behavior를 검증한다.
2. mission, recovery, sensor, recorder thread의 WCET, deadline miss, stack
   high-water, queue high-water를 측정한다.
3. scheduler overload, sensor dropout, storage stall을 주입하고 TraceMap으로
   execution order와 fault containment를 확인한다.

Success criteria: trace loss, SB drops, deadline misses are explicit in an exported
report; logging stalls do not block the mission path.

### Phase D: Radio, Watchdog, and HIL

1. telemetry/uplink parser를 typed contract ingress로 이식한다.
2. command authentication and state guards are replay-tested; no packet directly
   changes state or energizes outputs.
3. task/hardware watchdog, reset reason persistence, and safe boot policy를
   검증한다.
4. HIL full-flight sequence와 representative failure corpus를 replay한다.

Success criteria: a hardware run produces a TraceMap that can explain every FSM
transition, enabled application, output decision, deadline, and Bus loss.

## 4. Flight Release Gate

The project remains bench-only until Phases A-D complete and the team records
approval for sensor calibration, recovery electrical behavior, real-time budget,
watchdog behavior, storage durability, radio command authorization, and HIL
evidence. A successful cross-build is not a flight release.
