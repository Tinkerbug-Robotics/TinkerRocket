# ---------- test_mosaic_doppler_pair ----------
# The mosaic-G5's Dopplers in pairs (TR_GNSSReceiverMosaic_Serial/
# MosaicRawEpoch.h, exp/v9-dop-avg): the off-grid read is the adapter's cells
# and follows the clock as noteMeasEpochClock does; the pairing rule, each
# refusal counted once and its Doppler left as it came; and the harness's
# feed (tools/raw_replay/mosaic_feed.h) pairing as the driver does.
add_executable(test_mosaic_doppler_pair test_mosaic_doppler_pair.cpp
    ${LIB_DIR}/TR_GNSSReceiverMosaic_Serial/SbfNav.cpp
    ${LIB_DIR}/TR_GNSSReceiverMosaic_Serial/SbfMeas.cpp
    ${LIB_DIR}/TR_GNSSReceiverMosaic_Serial/SbfParser.cpp
    ${LIB_DIR}/TR_GNSSRaw/eph.c
    ${LIB_DIR}/TR_GNSSRaw/RtcmEphemeris.cpp
)
target_include_directories(test_mosaic_doppler_pair PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../tools/raw_replay
    ${SHIM_DIR}
    ${LIB_DIR}/TR_GNSSReceiverMosaic_Serial
    ${LIB_DIR}/TR_GNSSRaw
    ${LIB_DIR}/TR_GNSSRaw/include
    ${LIB_DIR}/TR_RawGnssShadow
    ${LIB_DIR}/TR_RocketComputerTypes
    ${CMAKE_CURRENT_SOURCE_DIR}/../tinkerrocket-idf/projects/flight_computer/main   # raw_iono_rule.h, where it is
)
target_link_libraries(test_mosaic_doppler_pair GTest::gtest_main)
gtest_discover_tests(test_mosaic_doppler_pair)
