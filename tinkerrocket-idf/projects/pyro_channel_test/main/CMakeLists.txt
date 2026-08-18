idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
)

# Board revision selection (#411): idf.py -B build_v9 -DTR_BOARD_V9=1 build
# (or -DTR_BOARD_V8=1; default is V7). This tool fires squib drivers by hand,
# so the map it was built with is printed at startup — check it before arming.
if(TR_BOARD_V9)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE TR_BOARD_V9=1)
    message(STATUS "pyro_channel_test: board pin map V9/V10")
elseif(TR_BOARD_V8)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE TR_BOARD_V8=1)
    message(STATUS "pyro_channel_test: board pin map V8")
else()
    message(STATUS "pyro_channel_test: board pin map V7 (default; pass -DTR_BOARD_V8=1 or -DTR_BOARD_V9=1)")
endif()
