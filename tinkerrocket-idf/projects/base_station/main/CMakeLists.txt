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
        TR_BQ27Z746
        TR_OTA
        app_update
        driver
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
if(TR_BS_BOARD)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE TR_BS_BOARD=${TR_BS_BOARD})
endif()
