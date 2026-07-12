# TraceMap 실행 추적 및 디버깅 기준

문서 상태: 구현 기준, 2026-07-11

## 1. TraceMap은 logger가 아니다

세 종류의 기록은 목적과 failure policy가 다르다.

| 종류 | 목적 | 예 | 손실 정책 |
|---|---|---|---|
| Debug log | 사람이 즉시 읽는 진단 text | init error, driver message | best effort |
| Flight recorder | 비행 재구성과 sensor/command 보존 | sensor sample, telemetry frame | 저장 지연/전원손실 정책 필요 |
| TraceMap | 실제 실행 순서와 latency/통신/FSM 복원 | task end, SB publish, transition | bounded ring, drop/gap 계측 |

TraceMap은 `LOGI("task done")` 호출 모음이 아니다. 모든 record가 고정 schema,
sequence, monotonic timestamp, cycle/correlation ID, App/Topic/State ID를 가진다.

## 2. 계측 지점

| Event | 생성 위치 | 의미 |
|---|---|---|
| `TASK_BEGIN`, `TASK_END` | `Executive` | 모든 Application 실행 경계와 결과 |
| `TASK_DEADLINE_MISS` | `Executive` | 측정 duration이 budget 초과 |
| `TASK_SKIPPED` | `Executive` | App Catalog enable mask가 실행을 차단 |
| `BUS_PUBLISH`, `BUS_CONSUME` | `SoftwareBus` | 실제 topic producer/consumer 경로 |
| `BUS_DROP` | `SoftwareBus` | latest contention 또는 queue full |
| `STATE_APP_BEGIN/END` | `TracingStateAppRunner` | `onEnter/step` 실제 호출 직전/직후 |
| `TRANSITION_REQUEST` | `TracingStateAppRunner` | state app이 요청한 전이 |
| `TRANSITION_COMMIT` | `FlightCoordinatorApp` | Coordinator가 실제 반영한 전이 |
| `ACTUATION_INTENT/RESULT` | SB/recovery adapter | 논리 의도와 물리 adapter 결과 |
| `HEALTH_CHANGE` | fault queue | sensor/app 상태 변화 |

사용자 요구대로 task 종료 기록은 `Executive`가 자동 생성한다. 개별 Application
개발자가 매번 logger 호출을 넣지 않아도 되며 누락 가능성이 작다.

## 3. 비차단 동작

`TraceMap<N>`은 fixed ring이다.

- 기록 시 동적 할당 없음
- single-attempt atomic guard
- lock 경합 시 기다리지 않고 `dropped` 증가
- ring wrap 시 `overwritten` 증가
- exporter는 snapshot을 복사한 뒤 lock을 놓고 sink에 쓴다
- exporter 지연으로 sequence가 사라지면 `export_gap_count` 증가

`overwritten`은 ring retention window가 순환한 횟수이고, exporter가 이미 읽은
record도 포함한다. 실제 export 손실 판정은 CSV sequence gap과
`export_gap_count`를 사용한다.

## 4. 산출물

Host replay:

```bash
build/host/nura_host_sim build/host/traces/flight_trace.csv
python3 tools/tracemap_export.py build/host/traces/flight_trace.csv \
  --out-dir build/host/traces/report
```

Runtime schedule/FSM enable 표:

```bash
build/host/nura_app_catalog build/host/traces/app_catalog.md
```

생성 파일:

- `flight_trace.csv`: source-of-truth record stream
- `timeline.json`: Chrome `chrome://tracing`/Perfetto 형식 timeline
- `fsm.dot`, `fsm.svg`: 실제 관측 전이 graph
- `runtime.dot`, `runtime.svg`: App/Topic 통신 graph
- `summary.md`: sequence gap, drop, deadline, app별 관측 max duration

SB instant event까지 timeline에 포함하려면 `--include-bus`를 추가한다.

## 5. 정상 판정

Host acceptance에서 다음을 자동 검사한다.

- 최종 state `GROUND`
- 8개 정상 전이 관측
- recovery output 최종 OFF
- command/actuation/transition queue drop 0
- TraceMap exporter gap 0
- deadline miss 0

2026-07-11 deterministic replay 결과:

- Trace record: 269,498
- sequence gap: 0
- FSM transition: 8
- bus drop: 0
- deadline miss: 0

Host `duration_us`는 deterministic clock의 구조 검증 값이며 MCU WCET가 아니다.
Teensy 실측값은 DWT/cycle counter 기반 clock과 실제 driver/storage 부하에서 다시
측정해야 한다.

## 6. 디버깅 절차

전이가 누락된 경우:

1. `fsm.svg`에서 마지막 commit state를 찾는다.
2. 해당 state의 `STATE_APP_END`가 계속 존재하는지 확인한다.
3. 관련 sensor topic의 publish/consume sequence를 확인한다.
4. `DecisionTrace.reason`, count, value를 기존 비행 문서와 비교한다.
5. `BUS_DROP`, stale sample, deadline miss를 확인한다.
6. 작동 문제면 intent와 result의 correlation ID를 연결한다.

task stall인 경우:

1. 마지막 `TASK_BEGIN` 뒤 `TASK_END`가 없는 App을 찾는다.
2. 그 App의 port/driver 호출과 priority를 확인한다.
3. 동일 시점의 SB contention과 recorder 부하를 본다.
4. watchdog reset reason과 직전 retained trace를 함께 보존한다.

## 7. 새 Application 계측 규칙

1. 모든 주기/이벤트 앱은 `IExecutableApp`을 구현하고 `Executive::run()`으로 실행한다.
2. SB 밖의 공유 mutable state를 만들지 않는다.
3. 중요 외부 동작은 intent/result 두 record로 나눈다.
4. blocking sink를 app 내부에서 직접 호출하지 않는다.
5. 새 Trace event가 필요하면 contract schema version과 exporter를 함께 갱신한다.
6. test에서 sequence gap, queue drop, expected transition을 assertion한다.
