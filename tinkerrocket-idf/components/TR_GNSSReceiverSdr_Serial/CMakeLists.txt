# SdrRtcm.cpp and SdrAid.cpp are deliberately IDF-free (bytes in, RTCM3
# frames out) so they are host-testable: tests_cpp/test_sdr_rtcm.cpp,
# test_gnss_detect.cpp and test_sdr_aid.cpp compile them directly. Keep
# ESP-only code in TR_GNSSReceiverSdr_Serial.cpp. The CRC-24Q is the LC86G
# driver's (Rtcm3Msm, pure), and so is the clock line (GnssTimeSync), hence
# that component. SdrRawEpoch.h and SdrRawFeed.h (pure, header-only) make the
# raw-GNSS filter's epochs (TR_GNSSRaw's RawEpoch) on the SDR raw-nav image
# (TR_GNSS_DRIVER_SDR), hence TR_GNSSRaw; every other image leaves them out.
idf_component_register(SRCS "TR_GNSSReceiverSdr_Serial.cpp" "SdrRtcm.cpp" "SdrAid.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_uart TR_RocketComputerTypes TR_GNSSReceiverLC86_Serial TR_GNSSRaw
                                driver esp_timer esp_ringbuf
                       PRIV_REQUIRES esp_driver_gpio)
# The 4002 frames' bytes are the board's and the reference codec's only with
# no fused multiply-add (SdrAid.h); tests_cpp builds SdrAid.cpp the same way.
set_source_files_properties(SdrAid.cpp PROPERTIES COMPILE_OPTIONS -ffp-contract=off)
