# ---------- test_sbf_meas ----------
# The mosaic-G5's raw-measurement readers (TR_GNSSReceiverMosaic_Serial/
# SbfMeas.cpp): MeasEpoch's Type1/Type2 levels folded to whole signals, every
# do-not-use pair, the layout checked against Length, and MeasExtra. With
# TR_SBF_PRIVATE_CAPTURE naming .sbf files outside the repository, the 10/16
# V9 captures are read end to end too (statistics only); unset, that skips.
add_executable(test_sbf_meas test_sbf_meas.cpp
    ${LIB_DIR}/TR_GNSSReceiverMosaic_Serial/SbfMeas.cpp
    ${LIB_DIR}/TR_GNSSReceiverMosaic_Serial/SbfParser.cpp
)
target_include_directories(test_sbf_meas PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${SHIM_DIR}
    ${LIB_DIR}/TR_GNSSReceiverMosaic_Serial
    ${LIB_DIR}/TR_RocketComputerTypes
)
target_link_libraries(test_sbf_meas GTest::gtest_main)
gtest_discover_tests(test_sbf_meas)
