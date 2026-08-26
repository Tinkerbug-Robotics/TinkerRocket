idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        TR_Compat
        TR_NVS
        driver
        esp_timer
        TR_RocketComputerTypes
        TR_I2C_Interface
        TR_I2S_Stream
        TR_LogToFlash
        TR_FlightLog
        TR_LoRa_Comms
        TR_RadioLink
        TR_Sensor_Data_Converter
        TR_Orientation
        TR_Coordinates
        TR_BLE_To_APP
        TR_INA230
        TR_OTA
        app_update
        esp_app_format
        nvs_flash
        efuse            # #822: ESP_EFUSE_PSRAM_CAP board-vs-silicon check
        esp_adc          # #850: camera/servo IMON reads (adc_oneshot + adc_cali)
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-format -Wno-narrowing)

# Board revision selection (#411). Exactly one flag, and it is MANDATORY:
#
#   idf.py -B build_v7 -DTR_BOARD_V7=1 build   # legacy V7 board
#   idf.py -B build_v8 -DTR_BOARD_V8=1 build   # V8 bench boards (MRAM fitted)
#   idf.py -B build_v9 -DTR_BOARD_V9=1 build   # V9/V10 (no MRAM, PSRAM ring)
#   idf.py -B build_m1 -DTR_BOARD_M1=1 build   # rocket-computer-mini rev1
#
# There used to be a V7 default, i.e. `idf.py build` compiled the map of a
# revision nobody owns, and did it silently. That was already a trap — V7's
# PWR_PIN is GPIO6, which is ESP_SCL on V8 and later, so a V7 image toggles the
# FC's I2C clock on every power on/off — and #822 made it worse: the maps now
# differ in what is FITTED, not just in pin numbering. A flagless build gets the
# V7 map, no PSRAM overlay, and an MRAM_CS pointing wherever V7 put it.
#
# The failure has no runtime symptom to catch it downstream, which is the same
# argument flight_computer used to make its flag mandatory. Matching it here.
# The build-dir name stays cosmetic — the board comes ONLY from the flag — but
# use a separate -B dir per variant anyway, since CMake caches it.
set(_tr_oc_board_flags "")
foreach(_tr_flag TR_BOARD_V7 TR_BOARD_V8 TR_BOARD_V9 TR_BOARD_M1)
    if(${_tr_flag})
        list(APPEND _tr_oc_board_flags ${_tr_flag})
    endif()
endforeach()
list(LENGTH _tr_oc_board_flags _tr_oc_board_flag_count)
if(_tr_oc_board_flag_count EQUAL 0)
    message(FATAL_ERROR
        "out_computer: no board revision selected. Pass exactly one of "
        "-DTR_BOARD_V7=1 / -DTR_BOARD_V8=1 / -DTR_BOARD_V9=1 / -DTR_BOARD_M1=1 "
        "(V9 is the hardware/rocket-computer V9/V10 board; M1 is "
        "hardware/rocket-computer-mini rev1). There is no default: the old "
        "one was V7, whose PWR_PIN (GPIO6) is ESP_SCL on every later board, and "
        "V9/V10 additionally needs the sdkconfig overlay that enables its PSRAM "
        "log ring.")
elseif(NOT _tr_oc_board_flag_count EQUAL 1)
    message(FATAL_ERROR
        "out_computer: more than one board revision selected "
        "(${_tr_oc_board_flags}). Pass exactly one, and use a separate build "
        "dir per variant — a stale CMake cache is the usual cause.")
endif()
target_compile_definitions(${COMPONENT_LIB} PRIVATE ${_tr_oc_board_flags}=1)
# Every out_computer board is an ESP32-S3; assert it so a stray build dir
# cannot produce an image for the wrong architecture.
if(CONFIG_IDF_TARGET AND NOT "${CONFIG_IDF_TARGET}" STREQUAL "esp32s3")
    message(FATAL_ERROR
        "out_computer: every board revision is an ESP32-S3, but this build dir is "
        "configured for '${CONFIG_IDF_TARGET}'. Use a fresh build dir.")
endif()
if(TR_BOARD_M1)
    message(STATUS "out_computer: board pin map M1 (main/board/board_m1.h, rocket-computer-mini rev1, on-board SPI radio)")
elseif(TR_BOARD_V9)
    message(STATUS "out_computer: board pin map V9/V10 (main/board/board_v9.h, no MRAM, PSRAM ring)")
elseif(TR_BOARD_V8)
    message(STATUS "out_computer: board pin map V8 (main/board/board_v8.h, MRAM fitted)")
else()
    message(STATUS "out_computer: board pin map V7 (main/board/board_v7.h)")
endif()

# Bench-debug image: `idf.py -B build_bench -DTR_OC_BENCH_DEBUG=1 build`.
#
# The OC's console is USB Serial/JTAG ONLY (ESP_CONSOLE_UART_NUM=-1, secondary
# none), and light sleep powers that peripheral down — so a normal image goes
# permanently silent to a host ~11 s after boot, when the #541 USB-enumeration
# grace lock releases. Before #519 the BT controller's own no_light_sleep lock
# on MAIN_XTAL kept the OC awake whenever BLE was up, which masked this; #519
# moved the BT LP clock to the 32k crystal and took that lifeline with it.
# Found 2026-07-28 while chasing OC-side evidence for #627.
#
# This flag holds ESP_PM_NO_LIGHT_SLEEP for the whole session so serial keeps
# flowing. It costs the entire #519 power win (~22 mA vs ~1 mA), so it is
# opt-in AND stamped "-bench" into the app version string (see the parent
# CMakeLists) — an image built this way is identifiable in the boot log, the
# BLE config_identity "fw" field, and its OTA metadata. Never fly it.
#
# Same trap as TR_BS_BOARD: the build-dir name is cosmetic, the flag is what
# counts, so surface the choice at configure time.
if(TR_OC_BENCH_DEBUG)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE TR_OC_BENCH_DEBUG=1)
    message(WARNING "out_computer: BENCH DEBUG build — light sleep disabled so "
                    "the USB console survives; idle current ~22 mA instead of "
                    "~1 mA. Version string carries '-bench'. Do NOT fly this.")
endif()
