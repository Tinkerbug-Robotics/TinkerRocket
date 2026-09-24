# Modeled on projects/out_computer/main/CMakeLists.txt; the REQUIRES list is
# the union of both sides of the merge.
#
# Deliberately ABSENT vs the two parents (report + spec):
#   TR_I2S_Stream / TR_UART_Link  — no second MCU, no radio daughterboard
#   TR_Servo_Control / TR_Control_Mixer / TR_GuidancePN(_Stub) / TR_RunCam /
#   TR_Piezo                      — hardware gone (guidance is also excluded
#                                   project-wide, see the parent CMakeLists)
# TR_Compat stays: TR_LogToFlash::begin() takes a SPIClass& (comms.cpp uses
# it exactly as the OC did); new code is otherwise IDF-native.
# TR_I2C_Interface stays for packMessage() framing only — no bus is created.
idf_component_register(
    SRCS "main.cpp" "comms.cpp" "flight.cpp" "mini_link.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        # shared
        TR_Compat
        TR_NVS
        driver
        esp_driver_gpio esp_driver_spi esp_driver_uart esp_driver_i2c
        esp_timer
        nvs_flash
        TR_RocketComputerTypes
        TR_I2C_Interface
        TR_Sensor_Data_Converter
        TR_Orientation
        TR_OTA
        app_update
        esp_app_format
        # comms side (out_computer descendants)
        TR_LogToFlash
        TR_FlightLog
        TR_LoRa_Comms
        TR_RadioLink
        TR_Coordinates
        TR_BLE_To_APP
        TR_INA230
        # flight side (flight_computer descendants; the collector's own
        # CMakeLists pulls both GNSS driver components — the LC86 is
        # selected by TR_GNSS_DRIVER_LC86 in the parent CMakeLists)
        TR_Sensor_Collector
        TR_Sensor_Collector_Sim
        TR_GpsInsEKF
        TR_GeoMag
        TR_KinematicChecks
        TR_MagCalibrator
        TR_IMU_Int_V2
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-format -Wno-narrowing)

# Record the firmware revision in each flight's settings snapshot (#165) —
# same block as flight_computer/main/CMakeLists.txt so the ported
# FLIGHT_SETTINGS code compiles unchanged.  (PROJECT_VER in the parent
# CMakeLists carries the sha for identity/OTA; this one feeds the log.)
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

# Bench-debug image: `idf.py -B build_bench -DTR_MINI_BENCH_DEBUG=1 build`.
#
# Same trap as the out_computer's TR_OC_BENCH_DEBUG (see that CMakeLists for
# the full #627/#519 history): the console is USB Serial/JTAG ONLY, and light
# sleep powers that peripheral down ~11 s after boot once the #541 grace
# lock releases.  This flag holds ESP_PM_NO_LIGHT_SLEEP for the whole session
# so serial keeps flowing, at the cost of the entire #519 power win (~22 mA
# vs ~1 mA idle).  NOTE: unlike the OC, the mini's top-level CMakeLists does
# not stamp a "-bench" marker into PROJECT_VER — identify a bench image by
# the boot-log warning.  Never fly one.
if(TR_MINI_BENCH_DEBUG)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE TR_MINI_BENCH_DEBUG=1)
    message(WARNING "rocket_computer_mini: BENCH DEBUG build — light sleep "
                    "disabled so the USB console survives; idle current "
                    "~22 mA instead of ~1 mA. Do NOT fly this.")
endif()
