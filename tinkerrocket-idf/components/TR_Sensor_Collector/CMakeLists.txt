# gnss_detect.cpp is IDF-free (bytes in, the receiver on J1 out) so it is
# host-testable: tests_cpp/test_gnss_detect.cpp compiles it directly. The
# UART, NVS and the two drivers around it are gnss_auto_driver.cpp's, which is
# empty unless TR_GNSS_DRIVER_AUTO.
idf_component_register(SRCS "TR_Sensor_Collector.cpp" "mag_auto_driver.cpp" "gnss_detect.cpp" "gnss_auto_driver.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_gpio esp_driver_i2c TR_RocketComputerTypes TR_ISM6HG256 TR_BMP585 TR_MMC5983MA TR_IIS2MDC TR_QMC5883P TR_GNSSReceiverUBlox_Serial TR_GNSSReceiverLC86_Serial TR_GNSSReceiverMosaic_Serial
                                driver esp_timer esp_rom esp_driver_uart
                       PRIV_REQUIRES TR_NVS)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-reorder -Wno-format-truncation -Wno-format)
