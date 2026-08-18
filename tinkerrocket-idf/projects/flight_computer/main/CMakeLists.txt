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
foreach(_tr_flag TR_BOARD_V7 TR_BOARD_V8 TR_BOARD_V9)
    if(${_tr_flag})
        list(APPEND _tr_board_flags_set ${_tr_flag})
    endif()
endforeach()
list(LENGTH _tr_board_flags_set _tr_board_flag_count)
if(_tr_board_flag_count EQUAL 0)
    message(FATAL_ERROR
        "flight_computer: no board revision selected. Pass exactly one of "
        "-DTR_BOARD_V7=1 / -DTR_BOARD_V8=1 / -DTR_BOARD_V9=1 (V9 is the "
        "hardware/rocket-computer V9/V10 board). There is no default: the V8 "
        "and V9 pyro maps differ and a wrong map fires the wrong channel with "
        "no runtime symptom.")
elseif(NOT _tr_board_flag_count EQUAL 1)
    message(FATAL_ERROR
        "flight_computer: more than one board revision selected "
        "(${_tr_board_flags_set}). Pass exactly one, and use a separate build "
        "dir per variant — a stale CMake cache is the usual cause.")
endif()
target_compile_definitions(${COMPONENT_LIB} PRIVATE ${_tr_board_flags_set}=1)
if(TR_BOARD_V9)
    message(STATUS "flight_computer: board pin map V9/V10 (main/board/board_v9.h)")
elseif(TR_BOARD_V8)
    message(STATUS "flight_computer: board pin map V8 (main/board/board_v8.h)")
else()
    message(STATUS "flight_computer: board pin map V7 (main/board/board_v7.h)")
endif()
