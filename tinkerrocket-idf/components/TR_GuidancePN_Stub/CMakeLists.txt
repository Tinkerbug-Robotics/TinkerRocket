# Header-only no-op stand-in for the private TR_GuidancePN submodule.
# See TR_GuidancePN.h in this directory for rationale.
#
# This component exposes the same include path (`#include <TR_GuidancePN.h>`)
# as the real TR_GuidancePN submodule, so consumer code compiles unchanged.
# The flight_computer project's CMakeLists.txt picks one or the other via
# EXCLUDE_COMPONENTS at configure time.
idf_component_register(INCLUDE_DIRS ".")
