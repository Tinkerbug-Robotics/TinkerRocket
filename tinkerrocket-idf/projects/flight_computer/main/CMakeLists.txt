# Pick the real TR_GuidancePN submodule when it's checked out, otherwise
# the local no-op stub. Project CMakeLists.txt does the matching
# EXCLUDE_COMPONENTS dance so only one of the two is actually in the
# build. Both expose `TR_GuidancePN.h`, so main.cpp compiles unchanged.
if(EXISTS "${CMAKE_CURRENT_LIST_DIR}/../../../components/TR_GuidancePN/TR_GuidancePN.cpp")
    set(_guidance_req "TR_GuidancePN")
else()
    set(_guidance_req "TR_GuidancePN_Stub")
endif()

idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        esp_driver_gpio esp_driver_spi esp_driver_uart TR_NVS
        TR_RocketComputerTypes
        TR_Sensor_Collector
        TR_Sensor_Collector_Sim
        TR_I2C_Interface
        TR_I2S_Stream
        TR_Sensor_Data_Converter
        TR_Orientation
        TR_GpsInsEKF
        TR_GeoMag
        TR_KinematicChecks
        TR_MagCalibrator
        TR_ServoControl_ledc_mult
        ${_guidance_req}
        TR_ControlMixer
        TR_GNSSReceiverUBlox_Serial
        TR_IMU_Int_V2
        TR_OTA
        app_update
        esp_app_format
        driver
        esp_timer
        nvs_flash
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-format)

# Record the firmware revision in each flight's settings snapshot (#165).
# Captured at CMake configure time; reconfigure (touch this file or clean
# build) to refresh.  git describe is unusable here (repo has no tags), so
# use the raw short SHA plus a dirty flag for uncommitted tracked changes.
execute_process(
    COMMAND git -C ${CMAKE_CURRENT_LIST_DIR} rev-parse --short HEAD
    OUTPUT_VARIABLE FW_GIT_SHA
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
if(NOT FW_GIT_SHA)
    set(FW_GIT_SHA "unknown")
endif()
execute_process(
    COMMAND git -C ${CMAKE_CURRENT_LIST_DIR} diff --quiet HEAD --
    RESULT_VARIABLE FW_GIT_DIFF_RC)
if(FW_GIT_DIFF_RC EQUAL 1)
    set(FW_GIT_DIRTY 1)
else()
    set(FW_GIT_DIRTY 0)
endif()
target_compile_definitions(${COMPONENT_LIB} PRIVATE
    FW_GIT_SHA="${FW_GIT_SHA}"
    FW_GIT_DIRTY=${FW_GIT_DIRTY})

# Board revision selection (#411). Exactly one flag, and it is MANDATORY:
#
#   idf.py -B build_v7 -DTR_BOARD_V7=1 build   # legacy V7 board
#   idf.py -B build_v8 -DTR_BOARD_V8=1 build   # V8 bench boards
#   idf.py -B build_v9 -DTR_BOARD_V9=1 build   # V9/V10 (hardware/rocket-computer)
#   idf.py -B build_m1 -DTR_BOARD_M1=1 build   # rocket-computer-mini rev1 (ESP32-S3 FC)
#
# Unlike the other projects there is no default here, because the failure is
# silent and pyrotechnic: V8 and V9 disagree on PYRO_ARM (5 vs 16) and swap
# PYRO2/PYRO3 FIRE, while all four continuity pins stay put. A mis-flashed
# board therefore boots clean, reports the right channel armed and continuous,
# and then fires the wrong connector (or, with ARM undriven, none at all).
# Better to fail here than to find out on the pad.
#
# The build-dir name is cosmetic — the board comes ONLY from the flag. Use a
# separate -B dir per variant anyway, since CMake caches it.
set(_tr_board_flags_set "")
foreach(_tr_flag TR_BOARD_V7 TR_BOARD_V8 TR_BOARD_V9 TR_BOARD_M1)
    if(${_tr_flag})
        list(APPEND _tr_board_flags_set ${_tr_flag})
    endif()
endforeach()
list(LENGTH _tr_board_flags_set _tr_board_flag_count)
if(_tr_board_flag_count EQUAL 0)
    message(FATAL_ERROR
        "flight_computer: no board revision selected. Pass exactly one of "
        "-DTR_BOARD_V7=1 / -DTR_BOARD_V8=1 / -DTR_BOARD_V9=1 / -DTR_BOARD_M1=1 "
        "(V9 is the hardware/rocket-computer V9/V10 board; M1 is "
        "hardware/rocket-computer-mini rev1, whose flight computer is an "
        "ESP32-S3 rather than a P4). There is no default: the pyro maps all "
        "differ and a wrong map fires the wrong channel with no runtime "
        "symptom.")
elseif(NOT _tr_board_flag_count EQUAL 1)
    message(FATAL_ERROR
        "flight_computer: more than one board revision selected "
        "(${_tr_board_flags_set}). Pass exactly one, and use a separate build "
        "dir per variant — a stale CMake cache is the usual cause.")
endif()
target_compile_definitions(${COMPONENT_LIB} PRIVATE ${_tr_board_flags_set}=1)
# The board flag and the silicon must agree. M1's flight computer is an
# ESP32-S3; every other revision is an ESP32-P4. Getting this wrong builds and
# links cleanly — the pin map is just integers — and produces an image for the
# wrong architecture, so check it rather than trusting the overlay.
if(TR_BOARD_M1 AND NOT "${CONFIG_IDF_TARGET}" STREQUAL "esp32s3")
    message(FATAL_ERROR
        "flight_computer: -DTR_BOARD_M1=1 needs an esp32s3 target, but this build dir "
        "is configured for '${CONFIG_IDF_TARGET}'. rocket-computer-mini's flight "
        "computer is an ESP32-S3, not a P4. Use a FRESH build dir, or run "
        "`idf.py -B <dir> set-target esp32s3` first — idf.py fixes the target before "
        "sdkconfig.defaults.m1 is applied, so a reused P4 build dir silently keeps "
        "esp32p4 and you get a RISC-V image carrying the S3 pin map.")
endif()
if(NOT TR_BOARD_M1 AND CONFIG_IDF_TARGET AND NOT "${CONFIG_IDF_TARGET}" STREQUAL "esp32p4")
    message(FATAL_ERROR
        "flight_computer: V7/V8/V9 boards are ESP32-P4, but this build dir is "
        "configured for '${CONFIG_IDF_TARGET}'. Use a fresh build dir per board.")
endif()
if(TR_BOARD_M1)
    message(STATUS "flight_computer: board pin map M1 (main/board/board_m1.h, rocket-computer-mini rev1, ESP32-S3)")
elseif(TR_BOARD_V9)
    message(STATUS "flight_computer: board pin map V9/V10 (main/board/board_v9.h)")
elseif(TR_BOARD_V8)
    message(STATUS "flight_computer: board pin map V8 (main/board/board_v8.h)")
else()
    message(STATUS "flight_computer: board pin map V7 (main/board/board_v7.h)")
endif()
