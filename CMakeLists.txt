cmake_minimum_required(VERSION 3.20)
project(nura_avionics_port LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_library(nura_build_options INTERFACE)
target_compile_options(nura_build_options INTERFACE
    -Wall -Wextra -Wpedantic -Werror)

add_library(nura_core
    modules/core/src/executive.cpp
    modules/core/src/software_bus.cpp)
target_include_directories(nura_core PUBLIC
    modules/config/include
    modules/contracts/include
    modules/core/include)
target_link_libraries(nura_core PUBLIC nura_build_options)

add_library(nura_flight_domain
    modules/domain/flight/src/acceleration_selector.cpp
    modules/domain/flight/src/flight_coordinator.cpp
    modules/domain/flight/src/apps/apogee_detector_app.cpp
    modules/domain/flight/src/apps/burnout_detector_app.cpp
    modules/domain/flight/src/apps/drogue_sequence_app.cpp
    modules/domain/flight/src/apps/landing_sequence_app.cpp
    modules/domain/flight/src/apps/launch_detector_app.cpp
    modules/domain/flight/src/apps/main_deploy_detector_app.cpp)
target_include_directories(nura_flight_domain PUBLIC
    modules/config/include
    modules/contracts/include
    modules/domain/flight/include)
target_link_libraries(nura_flight_domain PUBLIC nura_build_options)

add_library(nura_apps
    modules/apps/src/event_recorder_app.cpp
    modules/apps/src/flight_coordinator_app.cpp
    modules/apps/src/input_aggregator_app.cpp
    modules/apps/src/recovery_actuation_app.cpp
    modules/apps/src/sensor_apps.cpp
    modules/apps/src/state_app_runner.cpp
    modules/apps/src/supervisor_app.cpp
    modules/apps/src/trace_exporter_app.cpp)
target_include_directories(nura_apps PUBLIC
    modules/apps/include
    modules/config/include
    modules/contracts/include
    modules/core/include
    modules/domain/flight/include
    modules/platform/include)
target_link_libraries(nura_apps PUBLIC
    nura_core nura_flight_domain nura_build_options)

add_library(nura_host_psp
    modules/platform/host/src/fake_platform.cpp)
target_include_directories(nura_host_psp PUBLIC
    modules/config/include
    modules/contracts/include
    modules/core/include
    modules/domain/flight/include
    modules/platform/include
    modules/platform/host/include)
target_link_libraries(nura_host_psp PUBLIC
    nura_flight_domain nura_build_options)

add_executable(nura_host_sim tools/host_sim/main.cpp)
target_link_libraries(nura_host_sim PRIVATE
    nura_apps nura_host_psp nura_build_options)

add_executable(nura_app_catalog tools/app_catalog/main.cpp)
target_include_directories(nura_app_catalog PRIVATE
    modules/config/include
    modules/contracts/include
    modules/domain/flight/include)
target_link_libraries(nura_app_catalog PRIVATE nura_build_options)

include(CTest)
if(BUILD_TESTING)
    add_executable(nura_port_core_tests test/zephyr_port/core_tests.cpp)
    target_link_libraries(nura_port_core_tests PRIVATE
        nura_apps nura_host_psp nura_build_options)
    add_test(NAME nura_port_core_tests COMMAND nura_port_core_tests)
    add_test(NAME nura_host_full_flight
             COMMAND nura_host_sim ${CMAKE_CURRENT_BINARY_DIR}/traces/flight_trace.csv)
endif()
