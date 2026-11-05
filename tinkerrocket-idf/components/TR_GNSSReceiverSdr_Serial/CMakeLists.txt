# SdrRtcm.cpp is deliberately IDF-free (bytes in, RTCM3 frames out) so it is
# host-testable: tests_cpp/test_sdr_rtcm.cpp and test_gnss_detect.cpp compile
# it directly. Keep ESP-only code in TR_GNSSReceiverSdr_Serial.cpp. The CRC-24Q
# is the LC86G driver's (Rtcm3Msm, pure), hence that component.
idf_component_register(SRCS "TR_GNSSReceiverSdr_Serial.cpp" "SdrRtcm.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_uart TR_RocketComputerTypes TR_GNSSReceiverLC86_Serial
                                driver esp_timer esp_ringbuf
                       PRIV_REQUIRES esp_driver_gpio)
