# Rtcm3Msm.cpp and GnssTimeSync.cpp are deliberately IDF-free (bytes in, MSM7
# epochs and the clock model out) so they are host-testable:
# tests_cpp/test_gnss_wire_rtcm.cpp and test_gnss_time_sync.cpp compile them
# directly. The component keeps its name from the Tinker-Beetle V1's LC86G
# driver, retired with the V1: what is left is shared by the other receivers'
# drivers (the clock line GnssTimeSync.h and GnssClockRecord.h, the RTCM3
# decoding in Rtcm3Msm.h). Lc86Parser.cpp and Lc86RawEpoch.h, that driver's
# stream parser and its MSM7 hand-over, stay as host-only decoders of the V1's
# logged frames (tools/raw_replay, tinkerrocket-sim, the raw-GNSS host tests)
# and are not built into any image.
idf_component_register(SRCS "Rtcm3Msm.cpp" "GnssTimeSync.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_uart TR_RocketComputerTypes TR_GNSSRaw driver esp_timer esp_ringbuf
                       PRIV_REQUIRES esp_driver_gpio)
