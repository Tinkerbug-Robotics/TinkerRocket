idf_component_register(
    SRCS "main.cpp" "bench_selftest.cpp"
    INCLUDE_DIRS "."
    REQUIRES
        TR_Compat
        driver
        esp_timer
        esp_app_format
        TR_UART_Link
        TR_LoRa_Comms
)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-format)

# Bench self-test (#409): `idf.py -B build_bench -DTR_BENCH_SELFTEST=1 build`.
# Kept out of production images on purpose — the UART tests run the peripheral
# in internal loopback, and the TX pin keeps driving while they do, so a host
# on J6 would see the test traffic.
if(TR_BENCH_SELFTEST)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE TR_BENCH_SELFTEST=1)
    message(STATUS "radio_board: BENCH SELF-TEST build — not a production image")
endif()
