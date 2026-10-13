idf_component_register(
    SRCS "main.cpp"
    INCLUDE_DIRS "."
)

# Board revision selection (#411). Exactly one flag, and it is MANDATORY (#834):
#
#   idf.py -B build_v7 -DTR_BOARD_V7=1 build
#   idf.py -B build_v8 -DTR_BOARD_V8=1 build
#   idf.py -B build_v9 -DTR_BOARD_V9=1 build
#
# There was a V7 default here, which is the worst possible default for a tool
# that fires squib drivers by hand: on a V9 board, V7's "PYRO1 FIRE" (GPIO16)
# is PYRO_ARM — the U9 arming-FET gate through R21 — while the ARM indicator
# follows GPIO14, which is PYRO4_CONT on V9 and never moves. `idf.py build
# flash` with no flag, on a rocket with squibs on J2, arms the board on a
# keypress meant to pulse channel 1.
#
# flight_computer made this mandatory for the same reason; this is its bench
# sibling and had the opposite behaviour. The map is still printed at startup —
# keep checking it before arming — but it is no longer the only defence.
# The Tinker-Mantis V10 is refused by name rather than left to the "no board"
# error below: its arm is a two-processor vote (FC_ARM on GPIO33 AND the out
# computer's OC_ARM_EN), and this flight-computer-only tool cannot raise the
# out computer's half, so U9 never closes and every channel reads as a dead
# igniter. A V9 build on a V10 is the same thing with the arm on an unconnected
# pad (#1593).
if(TR_BOARD_V10 OR TR_BOARD_M1)
    message(FATAL_ERROR
        "pyro_channel_test: the V10 and the Beetle are not supported. Their arm "
        "needs the out computer's consent (OC_ARM_EN), which this flight-computer-"
        "only tool cannot raise. Use the app's pyro fire test, which the out "
        "computer gives consent for (out_computer/main/arm_consent_policy.h).")
endif()
set(_tr_pyro_board_flags "")
foreach(_tr_flag TR_BOARD_V7 TR_BOARD_V8 TR_BOARD_V9)
    if(${_tr_flag})
        list(APPEND _tr_pyro_board_flags ${_tr_flag})
    endif()
endforeach()
list(LENGTH _tr_pyro_board_flags _tr_pyro_board_flag_count)
if(_tr_pyro_board_flag_count EQUAL 0)
    message(FATAL_ERROR
        "pyro_channel_test: no board revision selected. Pass exactly one of "
        "-DTR_BOARD_V7=1 / -DTR_BOARD_V8=1 / -DTR_BOARD_V9=1. There is no "
        "default: this tool drives squib FETs by hand, and V7's PYRO1 FIRE pin "
        "(GPIO16) is PYRO_ARM on V9 — a flagless build arms a V9 board when you "
        "ask it to pulse channel 1.")
elseif(NOT _tr_pyro_board_flag_count EQUAL 1)
    message(FATAL_ERROR
        "pyro_channel_test: more than one board revision selected "
        "(${_tr_pyro_board_flags}). Pass exactly one, and use a separate build "
        "dir per variant — a stale CMake cache is the usual cause.")
endif()
target_compile_definitions(${COMPONENT_LIB} PRIVATE ${_tr_pyro_board_flags}=1)
if(TR_BOARD_V9)
    message(STATUS "pyro_channel_test: board pin map V9")
elseif(TR_BOARD_V8)
    message(STATUS "pyro_channel_test: board pin map V8")
else()
    message(STATUS "pyro_channel_test: board pin map V7")
endif()
