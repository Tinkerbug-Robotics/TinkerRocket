# ---------- test_raw_nav_adapter_mosaic (mosaic-only build) ----------
# What the mosaic-only image adds to raw_nav_adapter.h (TR_RAW_NAV_MOSAIC):
# the snapshot's geoid separation, hasRun, a restore's start height rebased to
# the barometer and its accelerometer bias, and a stop in flight held over as
# the last good state, carried forward and restarted from.
set(_raw_nav_adapter_dirs
    ${SHIM_DIR}
    ${LIB_DIR}/TR_GpsInsEKF
    ${LIB_DIR}/TR_RawGnssEKF
    ${LIB_DIR}/TR_RawGnssShadow
    ${LIB_DIR}/TR_GNSSRaw
    ${LIB_DIR}/TR_GNSSRaw/include
    ${LIB_DIR}/TR_RocketComputerTypes
    ${CMAKE_CURRENT_SOURCE_DIR}/../tinkerrocket-idf/projects/flight_computer/main
)
add_executable(test_raw_nav_adapter_mosaic test_raw_nav_adapter_mosaic.cpp
    ${LIB_DIR}/TR_RawGnssEKF/TR_RawGnssEKF.cpp
    ${LIB_DIR}/TR_RawGnssEKF/TR_RawGnssCarrier.cpp
)
target_include_directories(test_raw_nav_adapter_mosaic PRIVATE ${_raw_nav_adapter_dirs})
target_compile_definitions(test_raw_nav_adapter_mosaic PRIVATE TR_RAW_NAV_MOSAIC=1)
target_link_libraries(test_raw_nav_adapter_mosaic GTest::gtest_main)
gtest_discover_tests(test_raw_nav_adapter_mosaic)

# ---------- test_raw_nav_adapter_mosaic_base ----------
# The Beetle's adapter tests (test_raw_nav_adapter.cpp) against the mosaic
# build of the adapter: with its additions in and the hold off, it answers as
# the Beetle's does.
add_executable(test_raw_nav_adapter_mosaic_base test_raw_nav_adapter.cpp
    ${LIB_DIR}/TR_RawGnssEKF/TR_RawGnssEKF.cpp
    ${LIB_DIR}/TR_RawGnssEKF/TR_RawGnssCarrier.cpp
)
target_include_directories(test_raw_nav_adapter_mosaic_base PRIVATE ${_raw_nav_adapter_dirs})
target_compile_definitions(test_raw_nav_adapter_mosaic_base PRIVATE TR_RAW_NAV_MOSAIC=1)
target_link_libraries(test_raw_nav_adapter_mosaic_base GTest::gtest_main)
gtest_discover_tests(test_raw_nav_adapter_mosaic_base TEST_PREFIX "mosaic_build.")
