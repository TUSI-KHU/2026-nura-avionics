# NURA Zephyr Avionics Contributor Guide

## Scope

This repository is a Zephyr RTOS avionics migration. New firmware belongs in
`modules/` or `firmware/zephyr/`; Arduino, PlatformIO, and bare-metal
compatibility layers are intentionally out of scope.

## Layer Rules

- `modules/contracts/`: versioned POD messages, units, topic IDs only.
- `modules/config/`: the single App Catalog for period, deadline, priority,
  stack, execution domain, and state-enable policy.
- `modules/domain/flight/`: pure flight decisions. No Zephyr, board, driver,
  filesystem, logging, or GPIO includes.
- `modules/core/`: bounded Software Bus, Executive, and TraceMap only.
- `modules/apps/`: periodic application orchestration and output guards.
- `modules/platform/`: narrow PSP interfaces and host fakes.
- `firmware/zephyr/`: Zephyr threads, devicetree, Kconfig, and concrete PSP
  adapters.

Do not add a broad HAL singleton. Hardware dependencies enter through the
narrowest PSP port required by an application.

## Flight Safety

- Do not alter transitions, thresholds, arming, abort, recovery, or sensor
  fusion without an explicit request, updated `documents/`, and replay/bench
  evidence.
- `FlightPolicy` is the executable source of truth for migrated constants.
- Do not place scheduling literals in `main.cpp`; update the App Catalog and
  regenerate `documents/runtime_app_catalog.generated.md`.
- The current pyro pulse is **1000 ms**. Changing it requires a documented
  flight-logic change and hardware qualification evidence.
- No command parser or debug path may energize recovery hardware directly.
  Recovery outputs must pass through `FlightCoordinator` and
  `RecoveryActuationApp` guards.
- This repository is not flight-qualified. Hardware validation, HIL, watchdog,
  and output electrical verification remain mandatory.

## Verification

Run after firmware changes:

```bash
scripts/validate-migration.sh
scripts/build-zephyr.sh native_sim
scripts/build-zephyr.sh teensy41
```

Use `TraceMap` rather than console text as the primary execution evidence.
See `documents/tracemap_debugging_KR.md` and
`documents/flight_logic_migration_baseline_KR.md`.
