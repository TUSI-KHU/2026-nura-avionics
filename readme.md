# 2026-nura-avionics
Embedded flight controller for the 2026 NURA competition.

## Overview
This project is designed as an easy entry point for beginners in embedded systems who are learning scheduler and HAL concepts.

It currently includes:
- cooperative task scheduler
- basic flight state machine
- shared system context for state, health, and sensor data

## Why no FreeRTOS?
FreeRTOS and preemptive scheduling are powerful and widely used. However, we intentionally use a simple scheduler on bare Arduino so beginners can understand the essentials first.

The goal is not to build a perfect production-grade flight controller yet, but to create a bridge to advanced embedded firmware concepts such as:
- periodic task execution
- deterministic timing
- explicit state transitions
- shared data flow via global context
- non-blocking flow (instead of `delay()`, unlike parts of the 2024 codebase)

Our next step will definitely involve a tested and proven RTOS, but later in the learning path.

## Structure
- `core/`: flight states, global context, scheduler, and common task interfaces.
- `missions/`: mission-level tasks such as communication and flight state machine (FSM).
- `sensors/` : sensor acquisition and filtering tasks (for example, IMU fusion).
- `hal/` : Hardware Access Layer between drivers and higher-level tasks.
- `main.cpp`: Arduino entry point that initializes the scheduler and tasks.

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
