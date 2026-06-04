idf_component_register(SRCS "TR_BMP585.cpp" "bmp5.c"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_gpio esp_driver_spi driver esp_timer esp_rom)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-reorder -Wno-format-truncation -Wno-format)
