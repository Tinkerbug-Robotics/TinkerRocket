idf_component_register(SRCS "TR_Sensor_Collector_Sim.cpp" INCLUDE_DIRS "." REQUIRES TR_Compat TR_RocketComputerTypes TR_Sensor_Collector)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-reorder -Wno-format-truncation -Wno-format)

# #556 bench validation: `idf.py -B build_deadimu -DTR_SIM_DEAD_IMU=1 build`
# makes the sim stop delivering IMU samples from the boost->coast boundary on,
# so burnout never latches and only the baro-only apogee backstop can fire.
# Sim-only + compile-guarded; the default build never defines it.
if(TR_SIM_DEAD_IMU)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE TR_SIM_DEAD_IMU=1)
    message(STATUS "TR_Sensor_Collector_Sim: DEAD-IMU sim mode ENABLED (#556 bench)")
endif()
