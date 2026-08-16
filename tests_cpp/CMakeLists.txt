cmake_minimum_required(VERSION 3.16)
project(tinkerrocket_tests C CXX)

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

# TR_FlightLog's cross-core lock (#388) is a std::mutex on the host build, which
# needs pthread on Linux. Targets that compile TR_FlightLog.cpp link this.
find_package(Threads REQUIRED)

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

# ---------- test_servo_control ----------
# TR_ServoControl's LEDC calls resolve to the host_shim no-ops; the
# controller math (controlAngle cascade, gain schedule, #372 setpoint
# semantics) runs as on target.
add_executable(test_servo_control test_servo_control.cpp
    ${LIB_DIR}/TR_ServoControl_ledc_mult/TR_ServoControl_ledc_mult.cpp
    ${LIB_DIR}/TR_PID/TR_PID.cpp
)
target_include_directories(test_servo_control PRIVATE
    ${SHIM_DIR}
    ${LIB_DIR}/TR_ServoControl_ledc_mult
    ${LIB_DIR}/TR_PID
)
target_link_libraries(test_servo_control GTest::gtest_main)
gtest_discover_tests(test_servo_control)

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

# ---------- test_uart_link_codec ----------
# UART radio-modem link (#409): tr_msg codec + deframer + protocol layouts.
# TR_MsgCodec.cpp is deliberately IDF-free so it compiles on the host; the
# CRC sources come along for calcCRC16 (same shim note as test_crc16_compat).
add_executable(test_uart_link_codec test_uart_link_codec.cpp
    ${LIB_DIR}/TR_UART_Link/TR_MsgCodec.cpp
    ${LIB_DIR}/CRC/CRC16.cpp
    ${LIB_DIR}/CRC/CRC.cpp
    ${LIB_DIR}/CRC/CrcFastReverse.cpp
    ${LIB_DIR}/CRC/CRC8.cpp
    ${LIB_DIR}/CRC/CRC12.cpp
    ${LIB_DIR}/CRC/CRC32.cpp
    ${LIB_DIR}/CRC/CRC64.cpp
    ${LIB_DIR}/CRC/FastCRC32.cpp
)
target_include_directories(test_uart_link_codec PRIVATE
    ${SHIM_DIR}   # Must come first so Arduino.h resolves to our shim
    ${LIB_DIR}/TR_UART_Link
    ${LIB_DIR}/CRC
)
target_link_libraries(test_uart_link_codec GTest::gtest_main)
gtest_discover_tests(test_uart_link_codec)

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

# ---------- test_geomag ----------
# WMM2025 magnetic-declination model (TR_GeoMag), validated against the
# official NOAA WMM2025_TestValues.txt reference declinations.
add_executable(test_geomag test_geomag.cpp
    ${LIB_DIR}/TR_GeoMag/TR_GeoMag.cpp
)
target_include_directories(test_geomag PRIVATE
    ${SHIM_DIR}
    ${LIB_DIR}/TR_GeoMag
)
target_link_libraries(test_geomag GTest::gtest_main)
gtest_discover_tests(test_geomag)

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
    ${LIB_DIR}/TR_Orientation/TR_Orientation.cpp
)
target_include_directories(test_sensor_data_converter PRIVATE
    ${SHIM_DIR}
    ${LIB_DIR}/TR_Sensor_Data_Converter
    ${LIB_DIR}/TR_RocketComputerTypes
    ${LIB_DIR}/TR_Orientation
)
target_link_libraries(test_sensor_data_converter GTest::gtest_main)
gtest_discover_tests(test_sensor_data_converter)

# ---------- test_orientation ----------
# Board→rocket mounting orientation utilities (discrete 24-code set,
# quaternion round-trips, gravity nose snap, pad attitude init).
add_executable(test_orientation test_orientation.cpp
    ${LIB_DIR}/TR_Orientation/TR_Orientation.cpp
)
target_include_directories(test_orientation PRIVATE
    ${SHIM_DIR}
    ${LIB_DIR}/TR_Orientation
)
target_link_libraries(test_orientation GTest::gtest_main)
gtest_discover_tests(test_orientation)

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
target_link_libraries(test_tr_flightlog_core GTest::gtest_main Threads::Threads)
gtest_discover_tests(test_tr_flightlog_core)

# ---------- test_nand_bitmap_store (#398) ----------
# NAND-backed dual-copy bitmap store — replaces the NVS store on the OC so the
# frequent bitmap persists never touch internal flash (whose compaction erase
# cache-disabled core 1). Uses FakeNandBackend + CRC (calcCRC32).
add_executable(test_nand_bitmap_store
    test_nand_bitmap_store.cpp
    fakes/fake_nand_backend.cpp
    ${LIB_DIR}/TR_FlightLog/NandBitmapStore.cpp
    ${LIB_DIR}/CRC/CRC.cpp
    ${LIB_DIR}/CRC/CRC8.cpp
    ${LIB_DIR}/CRC/CRC12.cpp
    ${LIB_DIR}/CRC/CRC16.cpp
    ${LIB_DIR}/CRC/CRC32.cpp
    ${LIB_DIR}/CRC/CRC64.cpp
    ${LIB_DIR}/CRC/CrcFastReverse.cpp
    ${LIB_DIR}/CRC/FastCRC32.cpp
)
target_include_directories(test_nand_bitmap_store PRIVATE
    ${SHIM_DIR}
    ${LIB_DIR}/TR_FlightLog/include
    ${LIB_DIR}/CRC
    ${CMAKE_CURRENT_SOURCE_DIR}
)
target_link_libraries(test_nand_bitmap_store GTest::gtest_main Threads::Threads)
gtest_discover_tests(test_nand_bitmap_store)

# ---------- test_tr_ota (#8 phase 2) ----------
# Host coverage for TR_OTA_Receiver: state machine, offset/size validation,
# SHA-256 verification, callback fan-out. Real esp_ota_* is replaced with
# FakeOTABackend (tests_cpp/fakes/) and PSA Crypto (psa/crypto.h, used since the
# 6.0 migration) is replaced with the portable SHA-256 shim in host_shim/
# (psa_crypto_impl.c over sha256_impl.c).
add_executable(test_tr_ota
    test_tr_ota.cpp
    ${LIB_DIR}/TR_OTA/TR_OTA_Receiver.cpp
    ${SHIM_DIR}/sha256_impl.c
    ${SHIM_DIR}/psa_crypto_impl.c
)
target_include_directories(test_tr_ota PRIVATE
    ${SHIM_DIR}
    ${LIB_DIR}/TR_OTA/include
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/fakes
)
target_link_libraries(test_tr_ota GTest::gtest_main)
gtest_discover_tests(test_tr_ota)

# ---------- test_bs_log_policy ----------
# Host-side test for the base-station LoRa CSV logging policy helpers
# (silence-close gating + sequential-filename parser).  Header lives under
# projects/base_station/main/ so the test adds that to the include path
# directly — the helpers are pure and don't need any IDF stubs beyond the
# host_shim that other tests already use.  See test file + bs_log_policy.h
# for the #137 backstory.
add_executable(test_bs_log_policy test_bs_log_policy.cpp)
target_include_directories(test_bs_log_policy PRIVATE
    ${SHIM_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../tinkerrocket-idf/projects/base_station/main
    # bs_log_policy.h pulls the rocket-state enum + computeFreqLockForFlight
    # from the shared types header (#381 multi-rocket transitions)
    ${LIB_DIR}/TR_RocketComputerTypes
)
target_link_libraries(test_bs_log_policy GTest::gtest_main)
gtest_discover_tests(test_bs_log_policy)

# ---------- test_bs_uplink_policy ----------
# Host-side test for the base-station uplink transmit gate (#379): an uplink TX
# during a LoRa frequency scan must be deferred, not sent on the scan channel.
# bs_uplink_policy.h is pure, so the test just adds the base_station/main dir.
add_executable(test_bs_uplink_policy test_bs_uplink_policy.cpp)
target_include_directories(test_bs_uplink_policy PRIVATE
    ${SHIM_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../tinkerrocket-idf/projects/base_station/main
)
target_link_libraries(test_bs_uplink_policy GTest::gtest_main)
gtest_discover_tests(test_bs_uplink_policy)

# ---------- test_bs_uplink_queue ----------
# Host-side test for the base-station uplink command FIFO (#502): the uplink was
# a single slot, so a new command discarded the pending one's remaining retries —
# and the uplink is blind, so those retries ARE the delivery mechanism.
# bs_uplink_queue.h is pure, so the test just adds the base_station/main dir.
add_executable(test_bs_uplink_queue test_bs_uplink_queue.cpp)
target_include_directories(test_bs_uplink_queue PRIVATE
    ${SHIM_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../tinkerrocket-idf/projects/base_station/main
)
target_link_libraries(test_bs_uplink_queue GTest::gtest_main)
gtest_discover_tests(test_bs_uplink_queue)

# ---------- test_bs_uplink_txwin ----------
# Host-side test for the base-station uplink TX-window policy (#506): the radio is
# half-duplex, and a downlink packet (~82 ms on air) cannot fit in the ~49 ms gap
# between blind retries — so a burst lost essentially every packet that arrived
# during it. bs_uplink_txwin.h is pure, so the test just adds base_station/main.
add_executable(test_bs_uplink_txwin test_bs_uplink_txwin.cpp)
target_include_directories(test_bs_uplink_txwin PRIVATE
    ${SHIM_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../tinkerrocket-idf/projects/base_station/main
    # bs_uplink_txwin.h delegates its time-on-air to the shared
    # loraTimeOnAirMs (#150) so BS window math and hop-dwell math agree
    ${LIB_DIR}/TR_RocketComputerTypes
)
target_link_libraries(test_bs_uplink_txwin GTest::gtest_main)
gtest_discover_tests(test_bs_uplink_txwin)

# ---------- test_bs_battery_soc ----------
# Host-side test for the base-station voltage-based SoC fallback (#501): the
# BQ27Z746 reported 0% on a 4.17 V pack because its coulomb counter was never
# calibrated. bs_battery_soc.h is pure, so the test just adds base_station/main.
add_executable(test_bs_battery_soc test_bs_battery_soc.cpp)
target_include_directories(test_bs_battery_soc PRIVATE
    ${SHIM_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../tinkerrocket-idf/projects/base_station/main
)
target_link_libraries(test_bs_battery_soc GTest::gtest_main)
gtest_discover_tests(test_bs_battery_soc)

# ---------- test_ble_command_ring ----------
# Host-side test for the shared BLE command intake ring (#517): the intake was a
# single latch, so a back-to-back command burst (the app's ~13-command connect
# sync, #366) collapsed to whichever command arrived last. BleCommandRing.h is
# pure — TR_BLE_To_APP owns the NimBLE locking around it — so the test just adds
# the component dir.
add_executable(test_ble_command_ring test_ble_command_ring.cpp)
target_include_directories(test_ble_command_ring PRIVATE
    ${SHIM_DIR}
    ${LIB_DIR}/TR_BLE_To_APP
)
target_link_libraries(test_ble_command_ring GTest::gtest_main)
gtest_discover_tests(test_ble_command_ring)

# ---------- test_ble_chunk_size ----------
# #524: BLE file downloads ran at ~17 kB/s because chunks were sized MTU-maximal
# (502 B), which puts 516 octets on the wire and fragments as 251 + 251 + 14 — a
# whole link-layer packet to carry fourteen bytes. Throughput here is bounded by
# packets per connection event, so that stub cost ~a third of the download.
# BleChunkSize.h is pure integer arithmetic over (ATT MTU, LL TX octets), so the
# alignment — and the degenerate links it has to survive — are tested on the host.
add_executable(test_ble_chunk_size test_ble_chunk_size.cpp)
target_include_directories(test_ble_chunk_size PRIVATE
    ${SHIM_DIR}
    ${LIB_DIR}/TR_BLE_To_APP
)
target_link_libraries(test_ble_chunk_size GTest::gtest_main)
gtest_discover_tests(test_ble_chunk_size)

# ---------- test_bs_download_policy ----------
# Host-side test for the base-station cooperative BLE download (#380): chunk
# planning bounded by open-time file size + the 15 ms notify-drain pacing gate.
# bs_download_policy.h is pure, so the test just adds the base_station/main dir.
add_executable(test_bs_download_policy test_bs_download_policy.cpp)
target_include_directories(test_bs_download_policy PRIVATE
    ${SHIM_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../tinkerrocket-idf/projects/base_station/main
)
target_link_libraries(test_bs_download_policy GTest::gtest_main)
gtest_discover_tests(test_bs_download_policy)

# ---------- test_bs_storage_policy ----------
# Host-side test for the NAND bring-up retry policy (#761): mountExternalFlashFat
# was single-shot, so one transient error demoted the whole boot from a 512 MB
# NAND to a ~1.9 MB SPIFFS partition and truncated the flight. Two of these
# decisions are destructive if off by one (formatting on the wrong attempt wipes
# every stored flight), so they are pinned here. bs_storage_policy.h is pure;
# the test also pulls in config.h, which needs RocketComputerTypes on the path.
add_executable(test_bs_storage_policy test_bs_storage_policy.cpp)
target_include_directories(test_bs_storage_policy PRIVATE
    ${SHIM_DIR}
    ${LIB_DIR}/TR_RocketComputerTypes
    ${CMAKE_CURRENT_SOURCE_DIR}/../tinkerrocket-idf/projects/base_station/main
)
target_link_libraries(test_bs_storage_policy GTest::gtest_main)
gtest_discover_tests(test_bs_storage_policy)

# ---------- test_burnout_detector (#197/#256) ----------
# Detector logic is shared with the firmware via
# components/TR_KinematicChecks/BurnoutDetector.h — the test exercises the
# real code, not a mirror.
add_executable(test_burnout_detector test_burnout_detector.cpp)
target_include_directories(test_burnout_detector PRIVATE
    ${SHIM_DIR}
    ${LIB_DIR}/TR_KinematicChecks
)
target_link_libraries(test_burnout_detector GTest::gtest_main)
gtest_discover_tests(test_burnout_detector)

# ---------- test_deployment_detector ----------
# Recovery-deployment detector shared with the firmware via
# components/TR_KinematicChecks/DeploymentDetector.h — the test exercises the
# real code, not a mirror.  Drives the dynamic logging rate (IMU_RATE_DYNAMIC).
add_executable(test_deployment_detector test_deployment_detector.cpp)
target_include_directories(test_deployment_detector PRIVATE
    ${SHIM_DIR}
    ${LIB_DIR}/TR_KinematicChecks
)
target_link_libraries(test_deployment_detector GTest::gtest_main)
gtest_discover_tests(test_deployment_detector)

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

# ---------- test_mag_calibrator ----------
# Issue #206 — exercises the post-accept verification state machine
# (REVIEW → VERIFYING → APPLIED on pass, → REVIEW + reject_code on fail).
add_executable(test_mag_calibrator test_mag_calibrator.cpp
    ${LIB_DIR}/TR_MagCalibrator/TR_MagCalibrator.cpp
)
target_include_directories(test_mag_calibrator PRIVATE
    ${SHIM_DIR}
    ${LIB_DIR}/TR_MagCalibrator
    ${LIB_DIR}/TR_RocketComputerTypes
)
target_link_libraries(test_mag_calibrator GTest::gtest_main)
gtest_discover_tests(test_mag_calibrator)

# ---------- test_baro_gate_policy ----------
# Host-side test for the FC EKF barometer admission gate (#450): consume-on-use
# freshness (the old bmp_new_for_kf flag dropped ~50% of samples under the
# 487 Hz BMP / 479.5 Hz decimated-EKF beat) and the rate-aware spike test.
# baro_gate_policy.h is pure, so the test just adds the flight_computer/main dir.
add_executable(test_baro_gate_policy test_baro_gate_policy.cpp)
target_include_directories(test_baro_gate_policy PRIVATE
    ${SHIM_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../tinkerrocket-idf/projects/flight_computer/main
)
target_link_libraries(test_baro_gate_policy GTest::gtest_main)
gtest_discover_tests(test_baro_gate_policy)

# ---------- test_dedup_reboot_policy ----------
# Host-side test for the OC's FC-reboot-vs-replay discriminator (#468): a
# reboot verdict requires the timestamp backstep to persist for the confirm
# window; replayed stale I2S TX descriptors (once per DMA ring revolution,
# interleaved with fresh frames) must be dropped without resetting the dedup
# baselines. dedup_reboot_policy.h is pure, so the test just adds the
# out_computer/main dir.
add_executable(test_dedup_reboot_policy test_dedup_reboot_policy.cpp)
target_include_directories(test_dedup_reboot_policy PRIVATE
    ${SHIM_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../tinkerrocket-idf/projects/out_computer/main
)
target_link_libraries(test_dedup_reboot_policy GTest::gtest_main)
gtest_discover_tests(test_dedup_reboot_policy)

# ---------- test_mram_dirty_policy ----------
# Host-side test for the #417 sink-mode dirty-marker gate: the marker means
# "logging was activated" (launch detect / manual cmd-23 start), not "a log
# file was pre-created at PRELAUNCH", so a bench power-cut from PRELAUNCH no
# longer spawns a bogus flight_mram_recovered_*.bin — while a genuine in-flight
# brownout (#274) still recovers. mram_dirty_policy.h is pure (no SPI/MRAM/LFS),
# so the test just adds the TR_LogToFlash component dir.
add_executable(test_mram_dirty_policy test_mram_dirty_policy.cpp)
target_include_directories(test_mram_dirty_policy PRIVATE
    ${SHIM_DIR}
    ${LIB_DIR}/TR_LogToFlash
)
target_link_libraries(test_mram_dirty_policy GTest::gtest_main)
gtest_discover_tests(test_mram_dirty_policy)

# ---------- test_bad_block_scan_policy (#511) ----------
# Host-side test for the boot bad-block-scan gate: the full 2048-block scan
# (~2.7 s of the cmd-8 Power-On stall) runs only until one scan has completed
# for the current chip — recorded by the NVS "scanned" marker written strictly
# AFTER the scan (the "chip" key is written BEFORE it and must not be trusted).
# A dead RDID bus skips the scan so 2048 failing reads can't poison the map.
# bad_block_scan_policy.h is pure, so the test just adds the component dir.
add_executable(test_bad_block_scan_policy test_bad_block_scan_policy.cpp)
target_include_directories(test_bad_block_scan_policy PRIVATE
    ${SHIM_DIR}
    ${LIB_DIR}/TR_LogToFlash
)
target_link_libraries(test_bad_block_scan_policy GTest::gtest_main)
gtest_discover_tests(test_bad_block_scan_policy)

# ---------- test_flush_iter_ledger (#510) ----------
# Host-side test for the flush-task per-iteration accounting: section sums,
# clamped unaccounted/drain-other arithmetic, and the WARN/INFO classification
# (a long iteration the section timers explain — arm-time erase, activation
# ring drain — logs INFO; a genuine blind spot stays WARN with a breakdown).
# flush_iter_ledger.h is pure arithmetic, so the test just adds the dir.
add_executable(test_flush_iter_ledger test_flush_iter_ledger.cpp)
target_include_directories(test_flush_iter_ledger PRIVATE
    ${SHIM_DIR}
    ${LIB_DIR}/TR_LogToFlash
)
target_link_libraries(test_flush_iter_ledger GTest::gtest_main)
gtest_discover_tests(test_flush_iter_ledger)

# ---------- test_sim_pad_align (#508) ----------
# Header-only pure math (no ESP/Arduino deps), so there is no component .cpp to
# compile — the alignment solve was extracted out of SensorCollectorSim precisely
# so it could be tested on the host.
add_executable(test_sim_pad_align test_sim_pad_align.cpp)
target_include_directories(test_sim_pad_align PRIVATE
    ${SHIM_DIR}
    ${LIB_DIR}/TR_Sensor_Collector_Sim
)
target_link_libraries(test_sim_pad_align GTest::gtest_main)
gtest_discover_tests(test_sim_pad_align)

# ---------- test_sim_sensor_model (#512) ----------
# Header-only pure math. Pins the rigid-body kinematic identity that the sim's
# three synthetic sensors must jointly satisfy — the guard that was missing.
add_executable(test_sim_sensor_model test_sim_sensor_model.cpp)
target_include_directories(test_sim_sensor_model PRIVATE
    ${SHIM_DIR}
    ${LIB_DIR}/TR_Sensor_Collector_Sim
)
target_link_libraries(test_sim_sensor_model GTest::gtest_main)
gtest_discover_tests(test_sim_sensor_model)

# ---------- test_lc86_parser ----------
# Host-side test for the Quectel LC86G NMEA/PQTM stream parser — the protocol
# half of the mini's GNSS driver (TR_GNSSReceiverLC86_Serial). Lc86Parser.cpp
# is deliberately IDF-free (bytes in, GNSSData out), so the checksum rules
# (anchored on the protocol doc's published examples), the $PQTMPVT mapping
# incl. the vel_u = -VelD sign the EKF hard-fuses, the Quality==0 fix-trust
# gate, the $PQTMEPE accuracy merge, and junk/partial-line resilience are all
# pinned on the host. The ESP UART glue lives in TR_GNSSReceiverLC86_Serial.cpp
# and is not compiled here.
add_executable(test_lc86_parser test_lc86_parser.cpp
    ${LIB_DIR}/TR_GNSSReceiverLC86_Serial/Lc86Parser.cpp
)
target_include_directories(test_lc86_parser PRIVATE
    ${SHIM_DIR}
    ${LIB_DIR}/TR_GNSSReceiverLC86_Serial
    ${LIB_DIR}/TR_RocketComputerTypes
)
target_link_libraries(test_lc86_parser GTest::gtest_main)
gtest_discover_tests(test_lc86_parser)

# ---------- wire fixture corpus (docs/plans/android-port.md §2.1) ----------
# Golden-vector generation from the REAL packed structs: wire_fixture_gen
# writes tests_cpp/fixtures/wire/ (committed), test_wire_fixture_freshness
# fails CI whenever the committed bytes drift from what the current header
# would generate.  The Swift and Kotlin golden walks consume the same files.
set(WIRE_FIXTURES_SRCS
    wire_fixtures_lib.cpp
    ${LIB_DIR}/CRC/CRC16.cpp
    ${LIB_DIR}/CRC/CRC.cpp
    ${LIB_DIR}/CRC/CrcFastReverse.cpp
    ${LIB_DIR}/CRC/CRC8.cpp
    ${LIB_DIR}/CRC/CRC12.cpp
    ${LIB_DIR}/CRC/CRC32.cpp
    ${LIB_DIR}/CRC/CRC64.cpp
    ${LIB_DIR}/CRC/FastCRC32.cpp
)
set(WIRE_FIXTURES_INCLUDES
    ${SHIM_DIR}   # Must come first so Arduino.h resolves to our shim
    ${LIB_DIR}/TR_RocketComputerTypes
    ${LIB_DIR}/CRC
)

add_executable(wire_fixture_gen wire_fixture_gen_main.cpp ${WIRE_FIXTURES_SRCS})
target_include_directories(wire_fixture_gen PRIVATE ${WIRE_FIXTURES_INCLUDES})
target_compile_definitions(wire_fixture_gen PRIVATE
    TR_WIRE_FIXTURES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures/wire"
)

# Convenience: cmake --build <build> --target regen-wire-fixtures
add_custom_target(regen-wire-fixtures
    COMMAND wire_fixture_gen
    DEPENDS wire_fixture_gen
    COMMENT "Regenerating tests_cpp/fixtures/wire/ from RocketComputerTypes.h"
)

add_executable(test_wire_fixture_freshness
    test_wire_fixture_freshness.cpp ${WIRE_FIXTURES_SRCS})
target_include_directories(test_wire_fixture_freshness PRIVATE ${WIRE_FIXTURES_INCLUDES})
target_compile_definitions(test_wire_fixture_freshness PRIVATE
    TR_WIRE_FIXTURES_DIR="${CMAKE_CURRENT_SOURCE_DIR}/fixtures/wire"
)
target_link_libraries(test_wire_fixture_freshness GTest::gtest_main)
gtest_discover_tests(test_wire_fixture_freshness)
