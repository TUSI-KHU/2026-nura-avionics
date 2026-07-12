# Documentation Map

This directory contains only active Zephyr RTOS documentation.

| Document | Purpose |
|---|---|
| `zephyr_port_implementation_KR.md` | Implemented layer boundaries, PSP, Bus, recovery guards, memory, and hardware qualification gates |
| `flight_logic_migration_baseline_KR.md` | Current FSM policy, inputs, thresholds, fallbacks, and safety verification scope |
| `tracemap_debugging_KR.md` | TraceMap event model, capture, export, and replay-debugging procedure |
| `runtime_app_catalog.generated.md` | App Catalog에서 생성한 thread schedule과 FSM state별 실행 application 표 |
| `zephyr_migration_architecture_plan_KR.md` | Remaining hardware integration and flight-qualification roadmap |

The current executable policy is defined in
`modules/domain/flight/include/nura/flight/flight_policy.h`. The current pyro
pulse policy is **1000 ms**.
