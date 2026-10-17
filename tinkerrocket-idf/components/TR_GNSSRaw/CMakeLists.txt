# TR_GNSSRaw: what a raw-measurement (tightly coupled) GNSS/INS filter needs
# beside the receiver's measurements -- the broadcast ephemeris (decoded from
# the receiver's RTCM 1019/1046/1042), satellite positions and clocks, and the
# ionosphere and troposphere corrections, and the per-satellite measurements
# built from them (RawMeasBuilder). The C core (eph.c, atmo.c) is the GNSS
# receiver's own (gnss repo receiver/core); everything here is IDF-free, so
# tests_cpp/test_gnss_raw.cpp and test_raw_gnss_meas.cpp compile it directly.
idf_component_register(SRCS "eph.c" "atmo.c" "RtcmEphemeris.cpp" "EphemerisStore.cpp" "RawMeasBuilder.cpp"
                            "RawMeasCarrier.cpp"
                       INCLUDE_DIRS "include" ".")
