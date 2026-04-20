# 2026-nura-avionics
Embedded flight controller for the 2026 NURA competition.

## Overview
This project is designed as an easy entry point for beginners in embedded systems who are learning scheduler and HAL concepts.

It currently includes:
- cooperative task scheduler
- basic flight state machine
- focused domain stores for flight state, abort state, and sensor data
- constructor-injected task dependencies
- interface-based application configuration for board and runtime constants
- HAL-owned panic handling for fatal faults

## Why no FreeRTOS?
FreeRTOS and preemptive scheduling are powerful and widely used. However, we intentionally use a simple scheduler on bare Arduino so beginners can understand the essentials first.

The goal is not to build a perfect production-grade flight controller yet, but to create a bridge to advanced embedded firmware concepts such as:
- periodic task execution
- deterministic timing
- explicit state transitions
- shared data flow via small domain stores
- non-blocking flow (instead of `delay()`, unlike parts of the 2024 codebase)

Our next step will definitely involve a tested and proven RTOS, but later in the learning path.

## Structure
- `app/`: composition root and application configuration.
- `core/`: scheduler, logger, recoverable-task policy, and common task interfaces.
- `state/`: focused stores for flight state, abort state, and sensor data.
- `missions/`: mission-level tasks such as watchdog, logger drain, and flight state machine (FSM).
- `sensors/`: sensor acquisition tasks.
- `hal/`: hardware adapters such as IMU access, serial logging, digital output, and panic handling.
- `main.cpp`: thin Arduino entry point that forwards to `FlightControllerApp`.

## Runtime Model
- `main.cpp` constructs a single `FlightControllerApp` and forwards `setup()` and `loop()` calls to it.
- `FlightControllerApp` owns the scheduler, stores, HAL objects, config, and task instances.
- Tasks are registered in a fixed order and executed cooperatively by `Scheduler` based on each task's `periodMs()`.
- Shared state is stored in small domain structs instead of a single system-wide context object.

## Configuration
- `IAppConfig` defines board and runtime constants such as serial baud rate, IMU I2C address, task periods, retry limits, and panic LED settings.
- `DefaultAppConfig` provides the current board configuration.
- Tasks and HAL components receive config through constructor injection instead of using scattered literals.

## Fault Handling
- Fatal faults are handled by a HAL-owned `IPanicHandler` implementation.
- The current implementation blinks the configured status LED forever using `BlinkingPanicHandler`.

## Build and Upload
### Build
```bash
pio run
```

### Upload
```bash
pio run -t upload
```

### Serial Monitor
```bash
pio device monitor -b 115200
```
