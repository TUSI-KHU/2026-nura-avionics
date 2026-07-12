cmake_minimum_required(VERSION 3.20.0)
find_package(Zephyr REQUIRED HINTS $ENV{ZEPHYR_BASE})
project(nura_avionics_zephyr LANGUAGES CXX)

set(NURA_ROOT ${CMAKE_CURRENT_LIST_DIR}/../..)

target_sources(app PRIVATE
    src/main.cpp
    src/zephyr_psp.cpp
    ${NURA_ROOT}/modules/core/src/executive.cpp
    ${NURA_ROOT}/modules/core/src/software_bus.cpp
    ${NURA_ROOT}/modules/domain/flight/src/acceleration_selector.cpp
    ${NURA_ROOT}/modules/domain/flight/src/flight_coordinator.cpp
    ${NURA_ROOT}/modules/domain/flight/src/apps/apogee_detector_app.cpp
    ${NURA_ROOT}/modules/domain/flight/src/apps/burnout_detector_app.cpp
    ${NURA_ROOT}/modules/domain/flight/src/apps/drogue_sequence_app.cpp
    ${NURA_ROOT}/modules/domain/flight/src/apps/landing_sequence_app.cpp
    ${NURA_ROOT}/modules/domain/flight/src/apps/launch_detector_app.cpp
    ${NURA_ROOT}/modules/domain/flight/src/apps/main_deploy_detector_app.cpp
    ${NURA_ROOT}/modules/apps/src/event_recorder_app.cpp
    ${NURA_ROOT}/modules/apps/src/flight_coordinator_app.cpp
    ${NURA_ROOT}/modules/apps/src/input_aggregator_app.cpp
    ${NURA_ROOT}/modules/apps/src/recovery_actuation_app.cpp
    ${NURA_ROOT}/modules/apps/src/sensor_apps.cpp
    ${NURA_ROOT}/modules/apps/src/state_app_runner.cpp
    ${NURA_ROOT}/modules/apps/src/supervisor_app.cpp
    ${NURA_ROOT}/modules/apps/src/trace_exporter_app.cpp)

target_include_directories(app PRIVATE
    src
    ${NURA_ROOT}/modules/apps/include
    ${NURA_ROOT}/modules/config/include
    ${NURA_ROOT}/modules/contracts/include
    ${NURA_ROOT}/modules/core/include
    ${NURA_ROOT}/modules/domain/flight/include
    ${NURA_ROOT}/modules/platform/include)

target_compile_definitions(app PRIVATE
    NURA_TRACE_CAPACITY_RECORDS=CONFIG_NURA_TRACE_CAPACITY_RECORDS
    NURA_TRACE_EXPORT_BATCH_RECORDS=CONFIG_NURA_TRACE_EXPORT_BATCH_RECORDS
    NURA_COMMAND_QUEUE_SLOTS=CONFIG_NURA_COMMAND_QUEUE_SLOTS
    NURA_ACTUATION_QUEUE_SLOTS=CONFIG_NURA_ACTUATION_QUEUE_SLOTS
    NURA_TRANSITION_QUEUE_SLOTS=CONFIG_NURA_TRANSITION_QUEUE_SLOTS
    NURA_DECISION_QUEUE_SLOTS=CONFIG_NURA_DECISION_QUEUE_SLOTS
    NURA_SENSOR_FAULT_QUEUE_SLOTS=CONFIG_NURA_SENSOR_FAULT_QUEUE_SLOTS)
