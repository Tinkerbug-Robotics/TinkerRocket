idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        TR_Compat
        TR_NVS
        TR_RocketComputerTypes
        TR_Sensor_Collector
        TR_Sensor_Collector_Sim
        TR_I2C_Interface
        TR_I2S_Stream
        TR_Sensor_Data_Converter
        TR_GpsInsEKF
        TR_KinematicChecks
        TR_MagCalibrator
        TR_ServoControl_ledc_mult
        TR_GuidancePN
        TR_ControlMixer
        TR_GNSSReceiverUBlox_Serial
        TR_IMU_Int_V2
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
