idf_component_register(SRCS "TR_Sensor_Collector_Sim.cpp" "SimReplay.cpp" INCLUDE_DIRS "." REQUIRES TR_Compat TR_RocketComputerTypes TR_Sensor_Collector TR_GeoMag)
target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-error -Wno-reorder -Wno-format-truncation -Wno-format)

# #556 bench validation: `idf.py -B build_deadimu -DTR_SIM_DEAD_IMU=1 build`
# makes the sim stop delivering IMU samples from the boost->coast boundary on,
# so burnout never latches and only the baro-only apogee backstop can fire.
# Sim-only + compile-guarded; the default build never defines it.
if(TR_SIM_DEAD_IMU)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE TR_SIM_DEAD_IMU=1)
    message(STATUS "TR_Sensor_Collector_Sim: DEAD-IMU sim mode ENABLED (#556 bench)")
endif()

# #1600 the rig replay: `idf.py -B build_replay -DTR_BOARD_M1=1
# -DTR_SIM_REPLAY_BLOB=/abs/path/flight.bin build` carries a flight's IMU,
# barometer and attitude (tools/sim_replay_blob.py writes the file) and plays
# them from boot against the receiver's GPS time while the COCOM rig plays the
# same flight's GNSS signals (SimReplay.h). Bench-only: the default build
# carries nothing and startReplay() is false.
if(TR_SIM_REPLAY_BLOB)
    if(NOT EXISTS "${TR_SIM_REPLAY_BLOB}")
        message(FATAL_ERROR "TR_SIM_REPLAY_BLOB: no file ${TR_SIM_REPLAY_BLOB}")
    endif()
    target_add_binary_data(${COMPONENT_LIB} "${TR_SIM_REPLAY_BLOB}" BINARY RENAME_TO sim_replay_bin)
    target_compile_definitions(${COMPONENT_LIB} PRIVATE TR_SIM_REPLAY=1)
    message(STATUS "TR_Sensor_Collector_Sim: RIG REPLAY build, carrying ${TR_SIM_REPLAY_BLOB}")
endif()
