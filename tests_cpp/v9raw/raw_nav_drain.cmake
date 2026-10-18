# ---------- test_raw_nav_drain (mosaic-only build) ----------
# When the mosaic-only image's GNSS task files the receiver's navigation
# blocks (flight_computer/main/raw_nav_drain.h): not until an epoch has named
# the receiver's week, so the stage refuses a stored copy weeks old instead
# of placing it as today's; through the real SbfNav decoder and
# RawGnssMeasStage::addEphemeris. Pure header in the flight_computer project.
add_executable(test_raw_nav_drain test_raw_nav_drain.cpp
    ${LIB_DIR}/TR_GNSSReceiverMosaic_Serial/SbfNav.cpp
    ${LIB_DIR}/TR_GNSSReceiverMosaic_Serial/SbfMeas.cpp
    ${LIB_DIR}/TR_GNSSReceiverMosaic_Serial/SbfParser.cpp
    ${LIB_DIR}/TR_RawGnssShadow/RawGnssMeasStage.cpp
    ${LIB_DIR}/TR_GNSSRaw/eph.c
    ${LIB_DIR}/TR_GNSSRaw/atmo.c
    ${LIB_DIR}/TR_GNSSRaw/RtcmEphemeris.cpp
    ${LIB_DIR}/TR_GNSSRaw/EphemerisStore.cpp
    ${LIB_DIR}/TR_GNSSRaw/RawMeasBuilder.cpp
    ${LIB_DIR}/TR_GNSSRaw/RawMeasCarrier.cpp
    ${LIB_DIR}/TR_GNSSRaw/RawMeasBuilderLite.cpp
)
target_include_directories(test_raw_nav_drain PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${SHIM_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/../tinkerrocket-idf/projects/flight_computer/main
    ${LIB_DIR}/TR_GNSSReceiverMosaic_Serial
    ${LIB_DIR}/TR_RawGnssShadow
    ${LIB_DIR}/TR_GNSSRaw
    ${LIB_DIR}/TR_GNSSRaw/include
    ${LIB_DIR}/TR_RocketComputerTypes
)
target_link_libraries(test_raw_nav_drain GTest::gtest_main)
gtest_discover_tests(test_raw_nav_drain)
