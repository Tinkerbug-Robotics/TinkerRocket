# ---------- the mosaic-only image on the V8 as well as the V9 ----------
# The Goblin flies a V8 on 2026-10-24 (-DTR_BOARD_V8=1 -DTR_GNSS_MOSAIC=1).
# Each board's config.h is built with the mosaic driver: the receiver's pins,
# its 1PPS and its rate (test_mosaic_board_config), and the V8's link budget
# with the receiver's SBF stream, the twin of test_i2s_link_budget_V9_MOSAIC.
foreach(BOARD V8 V9)
    add_executable(test_mosaic_board_config_${BOARD}
        ${CMAKE_CURRENT_SOURCE_DIR}/test_mosaic_board_config.cpp)
    target_include_directories(test_mosaic_board_config_${BOARD} PRIVATE
        ${CMAKE_CURRENT_SOURCE_DIR}
        ${SHIM_DIR}
        ${LIB_DIR}/../projects/flight_computer/main
    )
    target_compile_definitions(test_mosaic_board_config_${BOARD} PRIVATE
        TR_BOARD_${BOARD}=1 TR_GNSS_DRIVER_MOSAIC=1)
    target_link_libraries(test_mosaic_board_config_${BOARD} GTest::gtest_main)
    gtest_discover_tests(test_mosaic_board_config_${BOARD} TEST_PREFIX "${BOARD}_MOSAIC.")
endforeach()

add_executable(test_i2s_link_budget_V8_MOSAIC ${CMAKE_CURRENT_SOURCE_DIR}/test_i2s_link_budget.cpp)
target_include_directories(test_i2s_link_budget_V8_MOSAIC PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${SHIM_DIR}
    ${LIB_DIR}/TR_RocketComputerTypes
    ${LIB_DIR}/../projects/flight_computer/main
)
target_compile_definitions(test_i2s_link_budget_V8_MOSAIC PRIVATE TR_BOARD_V8=1 TR_GNSS_DRIVER_MOSAIC=1)
target_link_libraries(test_i2s_link_budget_V8_MOSAIC GTest::gtest_main)
gtest_discover_tests(test_i2s_link_budget_V8_MOSAIC TEST_PREFIX "V8_MOSAIC.")
