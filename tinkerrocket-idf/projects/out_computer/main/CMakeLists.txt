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
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-format -Wno-narrowing)

# Board revision selection (#411). Use a separate build dir per variant — the
# flag is cached by CMake:
#
#   idf.py -B build_v8 -DTR_BOARD_V8=1 build   # V8 bench boards (MRAM fitted)
#   idf.py -B build_v9 -DTR_BOARD_V9=1 build   # V9/V10 (no MRAM)
#
# V8 and V9/V10 share every S3 GPIO, but they are NOT interchangeable: #822
# split board_v9.h out because V9 deleted the MRAM (U12). Building V9/V10
# hardware with -DTR_BOARD_V8=1 puts the log ring and the reboot snapshot on an
# unfitted chip, and nothing at runtime says so — the writes go to a floating
# pad.
#
# Default is still V7 (no flag), unlike the FC where the flag is mandatory.
# That default is itself a trap — V7's PWR_PIN (6) is the V8+ ESP_SCL net — but
# changing it is out of scope here; CI now builds V8 and V9 explicitly so
# neither map can rot.
set(_tr_oc_board_flags "")
foreach(_tr_flag TR_BOARD_V8 TR_BOARD_V9)
    if(${_tr_flag})
        list(APPEND _tr_oc_board_flags ${_tr_flag})
    endif()
endforeach()
list(LENGTH _tr_oc_board_flags _tr_oc_board_flag_count)
if(_tr_oc_board_flag_count GREATER 1)
    # Previously V8 silently won this race. Harmless while the two flags
    # selected the same header; now it picks a board.
    message(FATAL_ERROR
        "out_computer: more than one board revision selected "
        "(${_tr_oc_board_flags}). Pass exactly one, and use a separate build "
        "dir per variant — a stale CMake cache is the usual cause.")
endif()
if(TR_BOARD_V8)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE TR_BOARD_V8=1)
    message(STATUS "out_computer: board pin map V8 (main/board/board_v8.h, MRAM fitted)")
elseif(TR_BOARD_V9)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE TR_BOARD_V9=1)
    message(STATUS "out_computer: board pin map V9/V10 (main/board/board_v9.h, no MRAM)")
else()
    message(STATUS "out_computer: board pin map V7 (default; pass -DTR_BOARD_V8=1 or -DTR_BOARD_V9=1 for newer hardware)")
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
