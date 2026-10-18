# ---------- test_cap_governor ----------
# The satellite cap's governor (TR_RawGnssShadow/CapGovernor.h,
# exp/v9-cap-adapt), pure: off unless a ceiling above the floor; a satellite
# at a time up under slack, never past the ceiling; a cut on lag, the floor
# on an epoch worked at once, closed too old or lost; the floor through the
# boost and its hold; no cascade from epochs built before a change; no
# thrashing (the band, the backoff and its doubling, a load whose ages grow
# with the cap). Its wiring through the filter and the stage is in
# test_raw_gnss_shadow_v9 and test_raw_gnss_stage_v9.
add_executable(test_cap_governor test_cap_governor.cpp)
target_include_directories(test_cap_governor PRIVATE
    ${LIB_DIR}/TR_RawGnssShadow
)
target_link_libraries(test_cap_governor GTest::gtest_main)
gtest_discover_tests(test_cap_governor)
