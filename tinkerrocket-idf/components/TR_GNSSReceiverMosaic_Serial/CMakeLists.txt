# SbfParser.cpp and MosaicLink.cpp are deliberately IDF-free (bytes in, SBF
# blocks, reply lines and the setup's actions out) so they are host-testable:
# tests_cpp/test_sbf_parser.cpp and test_mosaic_link.cpp compile them directly.
# Keep ESP-only code in TR_GNSSReceiverMosaic_Serial.cpp. The PPS clock line is
# the LC86G driver's GnssTimeSync (pure), hence that component.
# SbfMeas.cpp, SbfNav.cpp and MosaicRawEpoch.h turn the raw measurement and
# navigation blocks into what the raw-GNSS filter takes (TR_GNSSRaw's RawEpoch
# and gps_eph_t); also IDF-free (tests_cpp/v9raw/sbf*.cmake). Every board
# builds this component, but only the mosaic raw-filter build calls them, so
# their objects are left out of every other image at link.
idf_component_register(SRCS "TR_GNSSReceiverMosaic_Serial.cpp" "SbfParser.cpp" "MosaicLink.cpp"
                            "SbfMeas.cpp" "SbfNav.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_uart TR_RocketComputerTypes TR_GNSSReceiverLC86_Serial TR_GNSSRaw
                                driver esp_timer esp_ringbuf
                       PRIV_REQUIRES esp_driver_gpio)
