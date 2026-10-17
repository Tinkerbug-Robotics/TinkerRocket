# SbfParser.cpp and MosaicLink.cpp are deliberately IDF-free (bytes in, SBF
# blocks, reply lines and the setup's actions out) so they are host-testable:
# tests_cpp/test_sbf_parser.cpp and test_mosaic_link.cpp compile them directly.
# Keep ESP-only code in TR_GNSSReceiverMosaic_Serial.cpp. The PPS clock line is
# the LC86G driver's GnssTimeSync (pure), hence that component.
# SbfMeas.cpp reads the raw measurement blocks for the raw-GNSS filter
# (TR_GNSSRaw); also IDF-free (tests_cpp/v9raw/sbf*.cmake). Every board
# builds this component, but only the mosaic raw-filter build calls it, so
# its objects are left out of every other image at link.
idf_component_register(SRCS "TR_GNSSReceiverMosaic_Serial.cpp" "SbfParser.cpp" "MosaicLink.cpp"
                            "SbfMeas.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_uart TR_RocketComputerTypes TR_GNSSReceiverLC86_Serial TR_GNSSRaw
                                driver esp_timer esp_ringbuf
                       PRIV_REQUIRES esp_driver_gpio)
