# File-based host side: IQ files, the front-end emulation, the CLI tools.
add_library(gnss_host STATIC
    iq_file.c
    ini.c
    manifest.c
    mix.c
    resamp.c
    iir.c
    rng.c
    quant.c
    fe_emul.c
    corr_float.c
    source.c
)
target_include_directories(gnss_host PUBLIC .)
target_link_libraries(gnss_host PUBLIC gnss_core gnss_fpga_model m)
target_compile_options(gnss_host PRIVATE ${GNSS_C_WARNINGS})
target_compile_definitions(gnss_host PRIVATE GNSS_DATA_DIR="${PROJECT_SOURCE_DIR}/data")

add_executable(iqtool iqtool.c)
target_link_libraries(iqtool PRIVATE gnss_host)
target_compile_options(iqtool PRIVATE ${GNSS_C_WARNINGS})

add_executable(gnssrx gnssrx.c)
target_link_libraries(gnssrx PRIVATE gnss_host)
target_compile_options(gnssrx PRIVATE ${GNSS_C_WARNINGS})
