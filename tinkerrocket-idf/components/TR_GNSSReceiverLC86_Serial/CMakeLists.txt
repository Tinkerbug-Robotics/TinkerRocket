# Lc86Parser.cpp and Rtcm3Msm.cpp are deliberately IDF-free (bytes in,
# GNSSData and MSM7 epochs out) so the protocol parsing is host-testable —
# tests_cpp/test_lc86_parser.cpp compiles them directly. Keep ESP-only code
# in TR_GNSSReceiverLC86_Serial.cpp.
idf_component_register(SRCS "TR_GNSSReceiverLC86_Serial.cpp" "Lc86Parser.cpp" "Rtcm3Msm.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_uart TR_RocketComputerTypes driver esp_timer esp_ringbuf
                       PRIV_REQUIRES esp_driver_gpio)
