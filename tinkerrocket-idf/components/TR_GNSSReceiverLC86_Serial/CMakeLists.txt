# Lc86Parser.cpp, Rtcm3Msm.cpp and GnssTimeSync.cpp are deliberately IDF-free (bytes in,
# GNSSData, MSM7 epochs and the clock model out) so they are host-testable —
# tests_cpp/test_lc86_parser.cpp and test_gnss_time_sync.cpp compile them directly. Keep ESP-only code
# in TR_GNSSReceiverLC86_Serial.cpp. Lc86RawEpoch.h hands an MSM7 epoch to
# TR_GNSSRaw's measurement builder (#1600).
idf_component_register(SRCS "TR_GNSSReceiverLC86_Serial.cpp" "Lc86Parser.cpp" "Rtcm3Msm.cpp" "GnssTimeSync.cpp"
                       INCLUDE_DIRS "."
                       REQUIRES esp_driver_uart TR_RocketComputerTypes TR_GNSSRaw driver esp_timer esp_ringbuf
                       PRIV_REQUIRES esp_driver_gpio)

# #1600 the rig replay (TR_Sensor_Collector_Sim/CMakeLists.txt): its receiver
# cold-starts at begin(), so it meets the rig's sky with no time, position or
# ephemeris of its own, as a pad power-up does.
if(TR_SIM_REPLAY_BLOB)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE TR_GNSS_COLD_START=1)
endif()
