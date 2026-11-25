# ---------- test_mosaic_raw_epoch ----------
# The mosaic-G5's MeasEpoch as the raw-GNSS filter's RawEpoch
# (TR_GNSSReceiverMosaic_Serial/MosaicRawEpoch.h): the L1 picks, units and the
# range rate's sign through RawMeasBuilder, clock jumps left in with the lock
# reset, and the time tag (epochGpsS, dtrMs, ppsResidualUs) on the PPS clock
# line (GnssTimeSync) across a -1 ms jump and a week rollover, with planted
# slips caught. With TR_SBF_PRIVATE_CAPTURE set, the 10/16 V9 captures' picks
# per epoch too (statistics only).
add_executable(test_mosaic_raw_epoch test_mosaic_raw_epoch.cpp
    ${LIB_DIR}/TR_GNSSReceiverMosaic_Serial/SbfMeas.cpp
    ${LIB_DIR}/TR_GNSSReceiverMosaic_Serial/SbfParser.cpp
    ${LIB_DIR}/TR_GNSSReceiverLC86_Serial/GnssTimeSync.cpp
    ${LIB_DIR}/TR_GNSSRaw/eph.c
    ${LIB_DIR}/TR_GNSSRaw/atmo.c
    ${LIB_DIR}/TR_GNSSRaw/RtcmEphemeris.cpp
    ${LIB_DIR}/TR_GNSSRaw/EphemerisStore.cpp
    ${LIB_DIR}/TR_GNSSRaw/RawMeasBuilder.cpp
    ${LIB_DIR}/TR_GNSSRaw/RawMeasCarrier.cpp
    ${LIB_DIR}/TR_GNSSRaw/RawMeasBuilderLite.cpp
)
target_include_directories(test_mosaic_raw_epoch PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${SHIM_DIR}
    ${LIB_DIR}/TR_GNSSReceiverMosaic_Serial
    ${LIB_DIR}/TR_GNSSReceiverLC86_Serial
    ${LIB_DIR}/TR_GNSSRaw
    ${LIB_DIR}/TR_GNSSRaw/include
    ${LIB_DIR}/TR_RocketComputerTypes
)
target_link_libraries(test_mosaic_raw_epoch GTest::gtest_main)
gtest_discover_tests(test_mosaic_raw_epoch)
