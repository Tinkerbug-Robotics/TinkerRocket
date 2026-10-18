# ---------- test_raw_gnss_stage_v9 ----------
# RawGnssMeasStage's mosaic-G5 additions (fc/v9raw-stage): an ephemeris the
# driver decoded itself filed as an RTCM one is, whether a satellite has one
# (BeiDou in its own time), the satellite cap in offer() -- GPS first, C/N0
# with a 3 dB hold, a pick the elevation mask drops picked last, every
# satellite's lock age kept, nothing changed with the cap off or above the
# sky -- what addEphemeris refuses, and the store's default size, the
# Beetle's, with no TR_EPH_CAPACITY macro of its own.
add_executable(test_raw_gnss_stage_v9 test_raw_gnss_stage_v9.cpp
    ${LIB_DIR}/TR_RawGnssShadow/RawGnssMeasStage.cpp
    ${LIB_DIR}/TR_GNSSRaw/eph.c
    ${LIB_DIR}/TR_GNSSRaw/atmo.c
    ${LIB_DIR}/TR_GNSSRaw/RtcmEphemeris.cpp
    ${LIB_DIR}/TR_GNSSRaw/EphemerisStore.cpp
    ${LIB_DIR}/TR_GNSSRaw/RawMeasBuilder.cpp
    ${LIB_DIR}/TR_GNSSRaw/RawMeasCarrier.cpp
    ${LIB_DIR}/TR_GNSSRaw/RawMeasBuilderLite.cpp
    ${LIB_DIR}/TR_GNSSReceiverLC86_Serial/Lc86Parser.cpp
    ${LIB_DIR}/TR_GNSSReceiverLC86_Serial/Rtcm3Msm.cpp
)
target_include_directories(test_raw_gnss_stage_v9 PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${SHIM_DIR}
    ${LIB_DIR}/TR_RawGnssShadow
    ${LIB_DIR}/TR_GNSSRaw
    ${LIB_DIR}/TR_GNSSRaw/include
    ${LIB_DIR}/TR_GNSSReceiverLC86_Serial
    ${LIB_DIR}/TR_RocketComputerTypes
)
target_link_libraries(test_raw_gnss_stage_v9 GTest::gtest_main)
gtest_discover_tests(test_raw_gnss_stage_v9)

# ---------- test_eph_store_pad_wait ----------
# The ephemeris store at the mosaic-G5 build's 128 (TR_EPH_CAPACITY, which
# that build sets project-wide; this target alone here, every source of it):
# a 2-4 h pad wait of ~50 satellites keeps two ephemerides for every
# satellite up, and a full pool drops a set satellite's first; the stage's
# addEphemeris refuses none of the wait's; the components agree on the size.
add_executable(test_eph_store_pad_wait test_eph_store_pad_wait.cpp
    ${LIB_DIR}/TR_RawGnssShadow/RawGnssMeasStage.cpp
    ${LIB_DIR}/TR_GNSSRaw/eph.c
    ${LIB_DIR}/TR_GNSSRaw/atmo.c
    ${LIB_DIR}/TR_GNSSRaw/RtcmEphemeris.cpp
    ${LIB_DIR}/TR_GNSSRaw/EphemerisStore.cpp
    ${LIB_DIR}/TR_GNSSRaw/RawMeasBuilder.cpp
    ${LIB_DIR}/TR_GNSSRaw/RawMeasCarrier.cpp
    ${LIB_DIR}/TR_GNSSRaw/RawMeasBuilderLite.cpp
)
target_compile_definitions(test_eph_store_pad_wait PRIVATE TR_EPH_CAPACITY=128)
target_include_directories(test_eph_store_pad_wait PRIVATE
    ${SHIM_DIR}
    ${LIB_DIR}/TR_RawGnssShadow
    ${LIB_DIR}/TR_GNSSRaw
    ${LIB_DIR}/TR_GNSSRaw/include
    ${LIB_DIR}/TR_RocketComputerTypes
)
target_link_libraries(test_eph_store_pad_wait GTest::gtest_main)
gtest_discover_tests(test_eph_store_pad_wait)
