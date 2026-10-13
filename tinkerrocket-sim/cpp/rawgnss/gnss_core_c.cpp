// The GNSS receiver's C core (TR_GNSSRaw eph.c, atmo.c) compiled as C++ for
// the pybind11 extension, whose flags (-std=c++17) a .c file cannot take.
// Their headers declare every function extern "C", so the linkage is C's.
#include "eph.c"
#include "atmo.c"
