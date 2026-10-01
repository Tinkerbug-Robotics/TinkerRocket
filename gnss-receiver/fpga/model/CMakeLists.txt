# Bit-exact C model of the FPGA. Host only: on the target the FPGA itself
# replaces it, and the HDL (fpga/hdl/, later) is verified against it.
add_library(gnss_fpga_model STATIC decim.c)
target_include_directories(gnss_fpga_model PUBLIC .)
target_compile_options(gnss_fpga_model PRIVATE ${GNSS_C_WARNINGS})
