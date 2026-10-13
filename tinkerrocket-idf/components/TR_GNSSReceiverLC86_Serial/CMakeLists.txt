# Lc86Parser.cpp, Rtcm3Msm.cpp and GnssTimeSync.cpp are deliberately IDF-free (bytes in,
# GNSSData, MSM7 epochs and the clock model out) so they are host-testable —
# tests_cpp/test_lc86_parser.cpp and test_gnss_time_sync.cpp compile them directly. Keep ESP-only code
# in TR_GNSSReceiverLC86_Serial.cpp.
idf_component_register(SRCS "TR_GNSSReceiverLC86_Serial.cpp" "Lc86Parser.cpp" "Rtcm3Msm.cpp" "GnssTimeSync.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_uart TR_RocketComputerTypes driver esp_timer esp_ringbuf
                       PRIV_REQUIRES esp_driver_gpio)
