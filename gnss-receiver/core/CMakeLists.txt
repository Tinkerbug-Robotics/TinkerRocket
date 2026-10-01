# The P4 firmware-to-be: portable C99, no OS calls, no malloc after init.
# A host library here; under ESP-IDF this file will call idf_component_register().
add_library(gnss_core STATIC
    acq/acq.c
    acq/fft.c
    nav/eph.c
    nav/lnav.c
    pvt/pvt.c
    rx.c
    sig/codes.c
    trk/trk.c
    util/gmath.c
)
target_include_directories(gnss_core PUBLIC include)
target_link_libraries(gnss_core PUBLIC m)
target_compile_options(gnss_core PRIVATE ${GNSS_C_WARNINGS} -Wdouble-promotion)
