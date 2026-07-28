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
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-format -Wno-narrowing)

# Board revision selection (#411): `idf.py -DTR_BOARD_V8=1 build` compiles the
# V8 pin map (main/board/board_v8.h); default is V7. Use a separate build dir
# per variant (e.g. -B build_v8) — the flag is cached by CMake.
if(TR_BOARD_V8)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE TR_BOARD_V8=1)
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
