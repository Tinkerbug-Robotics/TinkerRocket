idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
)

# Board revision selection (#411): idf.py -B build_v8 -DTR_BOARD_V8=1 build
if(TR_BOARD_V8)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE TR_BOARD_V8=1)
endif()
