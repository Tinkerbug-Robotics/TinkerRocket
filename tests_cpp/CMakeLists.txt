cmake_minimum_required(VERSION 3.16)
project(tinkerrocket_tests CXX)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# ---------- GoogleTest via FetchContent ----------
include(FetchContent)
FetchContent_Declare(
    googletest
    URL https://github.com/google/googletest/archive/refs/tags/v1.14.0.tar.gz
)
set(gtest_force_shared_crt ON CACHE BOOL "" FORCE)
FetchContent_MakeAvailable(googletest)

include(GoogleTest)
enable_testing()

# ---------- Path shortcuts ----------
# Component sources now live under the ESP-IDF tree. The libraries/
# mirror is being removed in a follow-up; all paths below point at
# the canonical components/ tree.
set(LIB_DIR "${CMAKE_CURRENT_SOURCE_DIR}/../tinkerrocket-idf/components")
set(SHIM_DIR "${CMAKE_CURRENT_SOURCE_DIR}/host_shim")

# Every target needs ${SHIM_DIR} on its include path because the
# component headers now #include <compat.h> (previously <Arduino.h>).
# The shim resolves both to the host-side Arduino.h stub.

# ---------- test_pid ----------
add_executable(test_pid test_pid.cpp
    ${LIB_DIR}/TR_PID/TR_PID.cpp
)
target_include_directories(test_pid PRIVATE
    ${SHIM_DIR}
    ${LIB_DIR}/TR_PID
)
target_link_libraries(test_pid GTest::gtest_main)
gtest_discover_tests(test_pid)

# ---------- test_rocket_computer_types ----------
add_executable(test_rocket_computer_types test_rocket_computer_types.cpp)
target_include_directories(test_rocket_computer_types PRIVATE
    ${SHIM_DIR}
    ${LIB_DIR}/TR_RocketComputerTypes
)
target_link_libraries(test_rocket_computer_types GTest::gtest_main)
gtest_discover_tests(test_rocket_computer_types)

# ---------- test_crc16_compat ----------
# CRC library needs Arduino.h shim (via CrcDefines.h).
# Note: the IDF components/CRC/ layout is flat — sources live directly
# under CRC/, not CRC/src/ like the old Arduino libraries/ layout.
add_executable(test_crc16_compat test_crc16_compat.cpp
    ${LIB_DIR}/CRC/CRC16.cpp
    ${LIB_DIR}/CRC/CRC.cpp
    ${LIB_DIR}/CRC/CrcFastReverse.cpp
    # Need CRC8/12/32/64 because CRC.h pulls them all in
    ${LIB_DIR}/CRC/CRC8.cpp
    ${LIB_DIR}/CRC/CRC12.cpp
    ${LIB_DIR}/CRC/CRC32.cpp
    ${LIB_DIR}/CRC/CRC64.cpp
    ${LIB_DIR}/CRC/FastCRC32.cpp
)
target_include_directories(test_crc16_compat PRIVATE
    ${SHIM_DIR}   # Must come first so Arduino.h resolves to our shim
    ${LIB_DIR}/CRC
)
target_link_libraries(test_crc16_compat GTest::gtest_main)
gtest_discover_tests(test_crc16_compat)

# ============================================================
# Phase 2: Safety-Critical Tests
# ============================================================

# ---------- test_ekf ----------
add_executable(test_ekf test_ekf.cpp
    ${LIB_DIR}/TR_GpsInsEKF/TR_GpsInsEKF.cpp
)
target_include_directories(test_ekf PRIVATE
    ${SHIM_DIR}
    ${LIB_DIR}/TR_GpsInsEKF
)
target_link_libraries(test_ekf GTest::gtest_main)
gtest_discover_tests(test_ekf)

# ---------- test_kinematic_checks ----------
add_executable(test_kinematic_checks test_kinematic_checks.cpp
    ${LIB_DIR}/TR_KinematicChecks/TR_KinematicChecks.cpp
)
target_include_directories(test_kinematic_checks PRIVATE
    ${SHIM_DIR}
    ${LIB_DIR}/TR_KinematicChecks
)
target_link_libraries(test_kinematic_checks GTest::gtest_main)
gtest_discover_tests(test_kinematic_checks)

# ---------- test_control_mixer ----------
add_executable(test_control_mixer test_control_mixer.cpp
    ${LIB_DIR}/TR_ControlMixer/TR_ControlMixer.cpp
    ${LIB_DIR}/TR_PID/TR_PID.cpp
)
target_include_directories(test_control_mixer PRIVATE
    ${SHIM_DIR}
    ${LIB_DIR}/TR_ControlMixer
    ${LIB_DIR}/TR_PID
)
target_link_libraries(test_control_mixer GTest::gtest_main)
gtest_discover_tests(test_control_mixer)

# ---------- test_guidance_pn ----------
# TR_GuidancePN is a private submodule (optional). Only build the test if
# the submodule is initialized. Contributors without access to the private
# upstream can work on every other feature; guidance tests are skipped.
if(EXISTS "${LIB_DIR}/TR_GuidancePN/TR_GuidancePN.cpp")
    add_executable(test_guidance_pn test_guidance_pn.cpp
        ${LIB_DIR}/TR_GuidancePN/TR_GuidancePN.cpp
    )
    target_include_directories(test_guidance_pn PRIVATE
        ${SHIM_DIR}
        ${LIB_DIR}/TR_GuidancePN
    )
    target_link_libraries(test_guidance_pn GTest::gtest_main)
    gtest_discover_tests(test_guidance_pn)
else()
    message(STATUS "TR_GuidancePN submodule not initialized — skipping test_guidance_pn")
    message(STATUS "  (to enable: git submodule update --init tinkerrocket-idf/components/TR_GuidancePN)")
endif()

# ============================================================
# Phase 3: Data Path Tests (Coordinates, Sensor Converter, LoRa)
# ============================================================

# ---------- test_coordinates ----------
add_executable(test_coordinates test_coordinates.cpp
    ${LIB_DIR}/TR_Coordinates/TR_Coordinates.cpp
)
target_include_directories(test_coordinates PRIVATE
    ${SHIM_DIR}
    ${LIB_DIR}/TR_Coordinates
)
target_link_libraries(test_coordinates GTest::gtest_main)
gtest_discover_tests(test_coordinates)

# ---------- test_sensor_data_converter ----------
add_executable(test_sensor_data_converter test_sensor_data_converter.cpp
    ${LIB_DIR}/TR_Sensor_Data_Converter/TR_Sensor_Data_Converter.cpp
)
target_include_directories(test_sensor_data_converter PRIVATE
    ${SHIM_DIR}
    ${LIB_DIR}/TR_Sensor_Data_Converter
    ${LIB_DIR}/TR_RocketComputerTypes
)
target_link_libraries(test_sensor_data_converter GTest::gtest_main)
gtest_discover_tests(test_sensor_data_converter)

# ---------- test_lora_roundtrip ----------
# This tests the OutComputer -> BaseStation -> iOS App data pipeline
add_executable(test_lora_roundtrip test_lora_roundtrip.cpp
    ${LIB_DIR}/TR_Sensor_Data_Converter/TR_Sensor_Data_Converter.cpp
)
target_include_directories(test_lora_roundtrip PRIVATE
    ${SHIM_DIR}
    ${LIB_DIR}/TR_Sensor_Data_Converter
    ${LIB_DIR}/TR_RocketComputerTypes
)
target_link_libraries(test_lora_roundtrip GTest::gtest_main)
gtest_discover_tests(test_lora_roundtrip)

# ============================================================
# Phase 4: Flight-log layer (issue #50)
# ============================================================

# ---------- test_tr_flightlog_core ----------
# Scaffold + FakeNandBackend sanity. Additional executables
# (recovery, wire_format, property, timing) land as later steps in the
# issue #50 implementation plan fill in behavior.
add_executable(test_tr_flightlog_core
    test_tr_flightlog_core.cpp
    fakes/fake_nand_backend.cpp
    ${LIB_DIR}/TR_FlightLog/TR_FlightLog.cpp
    ${LIB_DIR}/TR_FlightLog/BlockStateBitmap.cpp
    ${LIB_DIR}/TR_FlightLog/FlightIndex.cpp
    ${LIB_DIR}/TR_FlightLog/TR_NandBackend_esp.cpp
    ${LIB_DIR}/TR_FlightLog/NvsBitmapStore.cpp
    ${LIB_DIR}/CRC/CRC.cpp
    ${LIB_DIR}/CRC/CRC8.cpp
    ${LIB_DIR}/CRC/CRC12.cpp
    ${LIB_DIR}/CRC/CRC16.cpp
    ${LIB_DIR}/CRC/CRC32.cpp
    ${LIB_DIR}/CRC/CRC64.cpp
    ${LIB_DIR}/CRC/CrcFastReverse.cpp
    ${LIB_DIR}/CRC/FastCRC32.cpp
)
target_include_directories(test_tr_flightlog_core PRIVATE
    ${SHIM_DIR}
    ${LIB_DIR}/TR_FlightLog/include
    ${LIB_DIR}/CRC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(test_tr_flightlog_core GTest::gtest_main)
gtest_discover_tests(test_tr_flightlog_core)

# ---------- test_tr_flightlog_wire_format ----------
# Golden-file regression guard on the BLE wire formats iOS depends on.
# Fixtures under tests_cpp/fixtures/ble_wire_format/ are the source of truth.
add_executable(test_tr_flightlog_wire_format
    test_tr_flightlog_wire_format.cpp
    ${LIB_DIR}/TR_FlightLog/WireFormat.cpp
)
target_include_directories(test_tr_flightlog_wire_format PRIVATE
    ${SHIM_DIR}
    ${LIB_DIR}/TR_FlightLog/include
)
target_compile_definitions(test_tr_flightlog_wire_format PRIVATE
    TR_FLIGHTLOG_FIXTURES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures/ble_wire_format"
)
target_link_libraries(test_tr_flightlog_wire_format GTest::gtest_main)
gtest_discover_tests(test_tr_flightlog_wire_format)
