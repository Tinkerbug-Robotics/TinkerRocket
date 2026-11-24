# ---------- test_l5_band ----------
# The L5 band from the PX1105R (TR_RAW_L5): its 0xE5 cells as the filter's
# band-1 cells (Px1105rRawEpoch.h l5), the L5-band codes moved onto the L1
# band's (TR_GNSSRaw/CodeBridge.h), the Dopplers in pairs kept to one band
# (MosaicRawEpoch.h), the profiles' L5-band signals (ReceiverProfile.h) and
# the raw structs' size. The filter's own L5 rows: test_raw_gnss_ekf; the
# builder's: test_raw_gnss_meas.
add_executable(test_l5_band test_l5_band.cpp
    ${LIB_DIR}/TR_GNSSReceiverPX1105R_Serial/SkyTraqBinary.cpp
)
target_include_directories(test_l5_band PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${SHIM_DIR}
    ${LIB_DIR}/TR_GNSSReceiverPX1105R_Serial
    ${LIB_DIR}/TR_GNSSReceiverMosaic_Serial
    ${LIB_DIR}/TR_GNSSRaw
    ${LIB_DIR}/TR_GNSSRaw/include
    ${LIB_DIR}/TR_RawGnssEKF
    ${LIB_DIR}/TR_RocketComputerTypes
)
target_link_libraries(test_l5_band GTest::gtest_main)
gtest_discover_tests(test_l5_band)
