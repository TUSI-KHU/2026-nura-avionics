# NURA Standard Avionics RTOS

Zephyr RTOS 기반 NURA 에비오닉스 펌웨어입니다. 비행 판단은 하드웨어와 분리된
Domain에 두고, 보드·센서·recovery 출력은 PSP(Platform Support Package) adapter로
교체합니다. 모든 task는 bounded Software Bus로 통신하며, 실행 순서와 FSM 전이는
TraceMap으로 재구성합니다.

현재 상태: `native_sim` 및 `teensy41` cross-build 검증 완료. 실제 Teensy 4.1 및
센서 보드가 없는 상태에서 작성되었으므로 **하드웨어 검증 전 비행 금지**입니다.

## Layout

```text
modules/contracts/       Typed message contracts
modules/domain/flight/   Pure FSM policy and state applications
modules/core/            Software Bus, Executive, TraceMap
modules/apps/            Input, coordination, recovery, recorder apps
modules/platform/        PSP ports and host fakes
firmware/zephyr/         Zephyr board integration and adapters
test/zephyr_port/        Host replay and fault-guard tests
tools/host_sim/          Deterministic flight simulator
tools/app_catalog/       Generated schedule and state-app matrix
documents/               Architecture, safety policy, TraceMap usage
avionics_project/        Current KiCad hardware design
```

## Build And Validate

```bash
scripts/setup-zephyr.sh
scripts/validate-migration.sh
scripts/build-zephyr.sh native_sim
scripts/build-zephyr.sh teensy41
```

`scripts/validate-migration.sh` builds the host tests, produces a complete
TraceMap flight trace, and runs Zephyr builds when the local Zephyr workspace
is installed.

현재 thread schedule과 FSM별 application은
[`documents/runtime_app_catalog.generated.md`](documents/runtime_app_catalog.generated.md)에서
확인할 수 있으며 검증 시 App Catalog와 자동 비교됩니다.

## Safety Boundary

- `modules/domain/flight/` must not include Zephyr, MCU, sensor driver, or GPIO
  headers.
- `FlightPolicy::kPyroFireDurationMs` is **1000 ms**, the current mission
  policy. Hardware output qualification remains required before flight.
- Recovery ON requests are accepted only through the Coordinator and guarded
  Recovery application; command parsing and debug paths cannot drive GPIO.

Detailed operational guidance is in [documents/README.md](documents/README.md).
