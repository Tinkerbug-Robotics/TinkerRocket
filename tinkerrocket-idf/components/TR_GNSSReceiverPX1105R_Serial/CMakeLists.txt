# SkyTraqBinary.cpp, SkyTraqLink.cpp, SkyTraqNav.cpp and Px1105rRawEpoch.h are
# deliberately IDF-free (bytes in, frames, the setup's actions, ephemerides and
# raw epochs out) so they are host-testable: tests_cpp/test_skytraq_*.cpp and
# test_px1105r_raw_epoch.cpp compile them directly. Keep ESP-only code in
# TR_GNSSReceiverPX1105R_Serial.cpp. The arrival-time clock line is the LC86G
# driver's GnssTimeSync and the Doppler pairs' rule the mosaic driver's
# MosaicRawEpoch.h (both pure), hence those components. Every board builds this
# component, but only the PX1105R image (TR_GNSS_DRIVER_PX1105R) uses it, so its
# objects are left out of every other image at link.
idf_component_register(SRCS "TR_GNSSReceiverPX1105R_Serial.cpp" "SkyTraqBinary.cpp" "SkyTraqLink.cpp"
                            "SkyTraqNav.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_uart TR_RocketComputerTypes TR_GNSSReceiverLC86_Serial
                                TR_GNSSReceiverMosaic_Serial TR_GNSSRaw driver esp_timer esp_ringbuf
                       PRIV_REQUIRES esp_driver_gpio esp_hw_support heap)
