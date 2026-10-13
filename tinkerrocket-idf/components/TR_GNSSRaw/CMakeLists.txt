# TR_GNSSRaw: what a raw-measurement (tightly coupled) GNSS/INS filter needs
# beside the receiver's measurements -- the broadcast ephemeris (decoded from
# the receiver's RTCM 1019/1046/1042), satellite positions and clocks, and the
# ionosphere and troposphere corrections. The C core (eph.c, atmo.c) is the
# GNSS receiver's own (gnss repo receiver/core); everything here is IDF-free,
# so tests_cpp/test_gnss_raw.cpp compiles it directly.
idf_component_register(SRCS "eph.c" "atmo.c" "RtcmEphemeris.cpp" "EphemerisStore.cpp"
                       INCLUDE_DIRS "include" ".")
