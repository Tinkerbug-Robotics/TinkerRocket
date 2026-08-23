idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        esp_driver_i2c esp_driver_spi TR_Compat
        TR_NVS
        TR_RocketComputerTypes
        TR_Sensor_Data_Converter
        TR_Coordinates
        TR_BLE_To_APP
        TR_LoRa_Comms
        TR_RadioLink
        TR_MAX17205G
        TR_MAX17303
        TR_MP2672
        TR_BQ27Z746
        TR_OTA
        app_update
        driver
        esp_adc              # #835 item 2: own-battery divider (adc_oneshot + adc_cali)
        esp_app_format
        esp_timer
        fatfs
        sdmmc
        spiffs
        vfs
        spi_nand_flash
        spi_nand_flash_fatfs
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-format)

# Board revision selection (OC #411 pattern): `idf.py -DTR_BS_BOARD=<1|2|3>
# build` compiles the matching main/board/board_v*.h pin map; default is V2.
# Use a separate build dir per variant (e.g. -B build_v3) — the flag is
# cached by CMake.
#
# NOTE the trap: the build-dir name is cosmetic — the board comes ONLY from
# this flag. `idf.py -B build_v3 build` (no -DTR_BS_BOARD=3) silently builds
# V2. Surface the choice at configure time so a missing/wrong flag is visible
# in the build log, not just from the runtime boot banner.
if(TR_BS_BOARD)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE TR_BS_BOARD=${TR_BS_BOARD})
    message(STATUS "base_station: building for board V${TR_BS_BOARD} (TR_BS_BOARD=${TR_BS_BOARD})")
else()
    message(WARNING "base_station: TR_BS_BOARD not set — defaulting to V2. "
                    "For another board pass -DTR_BS_BOARD=<1|3> (the build-dir "
                    "name alone does NOT select the board).")
endif()
