# TR_RawGnssEKF: the tightly coupled GNSS/INS filter (#1600) -- the flight
# filter's error states plus the receiver clock, updated by each satellite's
# pseudorange and range rate (TR_GNSSRaw's RawMeasBuilder makes them). Pure
# C++, IDF-free: tests_cpp/test_raw_gnss_ekf.cpp and the tinkerrocket-sim
# bindings compile it directly.
idf_component_register(SRCS "TR_RawGnssEKF.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES TR_GNSSRaw)
