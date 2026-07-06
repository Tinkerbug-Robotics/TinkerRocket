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

# Board revision selection (#411): `idf.py -B build_v8 -DTR_BOARD_V8=1 build`
# compiles the V8 pin map (main/board/board_v8.h); default is V7. Use a
# separate build dir per variant — the flag is cached by CMake.
if(TR_BOARD_V8)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE TR_BOARD_V8=1)
endif()
