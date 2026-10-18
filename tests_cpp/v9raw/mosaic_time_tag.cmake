# ---------- test_mosaic_time_tag ----------
# The mosaic-G5's raw-epoch time tag across a lost and regained sky
# (TR_GNSSReceiverMosaic_Serial/MosaicTimeTag.h) on a simulated bench through
# the driver's pure pieces in its order: the 10/17 RF-rig underrun reproduced
# with the rule before (-6855 us at every whole second) and right or untimed
# with MosaicTimeTag and the PPS line that follows its edges; the ways a
# receiver may come back, a real sky's dropout, a receiver reset, a late edge
# and the boot's line; the review's cases (the 1PPS late to the step, runs of
# late edges, a sky back early by k * 256 ms, a backed-up link); and the rule
# piece by piece.
add_executable(test_mosaic_time_tag test_mosaic_time_tag.cpp
    ${LIB_DIR}/TR_GNSSReceiverMosaic_Serial/SbfMeas.cpp
    ${LIB_DIR}/TR_GNSSReceiverMosaic_Serial/SbfParser.cpp
    ${LIB_DIR}/TR_GNSSReceiverLC86_Serial/GnssTimeSync.cpp
)
target_include_directories(test_mosaic_time_tag PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${SHIM_DIR}
    ${LIB_DIR}/TR_GNSSReceiverMosaic_Serial
    ${LIB_DIR}/TR_GNSSReceiverLC86_Serial
    ${LIB_DIR}/TR_GNSSRaw
    ${LIB_DIR}/TR_GNSSRaw/include
    ${LIB_DIR}/TR_RocketComputerTypes
)
target_link_libraries(test_mosaic_time_tag GTest::gtest_main)
gtest_discover_tests(test_mosaic_time_tag)
