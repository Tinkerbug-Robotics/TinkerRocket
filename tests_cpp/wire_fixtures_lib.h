#pragma once
//
// Golden-vector fixture generation for the cross-platform wire-format corpus
// (docs/plans/android-port.md §2.1).
//
// generate() builds every fixture IN MEMORY from the real packed structs in
// RocketComputerTypes.h — the C compiler's layout IS the wire layout, so there
// is no transcription step to get wrong.  Two consumers:
//
//   wire_fixture_gen (CLI)          — writes the map to tests_cpp/fixtures/wire/
//                                     (cmake --build build --target regen-wire-fixtures)
//   test_wire_fixture_freshness.cpp — regenerates and byte-compares against the
//                                     committed files, so a header change that
//                                     alters any layout fails C++ CI until the
//                                     fixtures are regenerated and re-committed.
//
// Swift (GoldenVectorTests) and Kotlin (:core:protocol golden walk) consume the
// same committed files; every .bin ships with an .expected.json sidecar so the
// app-side tests are data-driven manifest walks, not hand-written per fixture.
//
// NOTE: byte images assume a little-endian host (x86_64/arm64 — both dev and
// CI).  A static_assert in the .cpp guards this.

#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace tr_wire_fixtures {

// Relative path under fixtures/wire/ -> file content.
using FixtureMap = std::map<std::string, std::vector<uint8_t>>;

FixtureMap generate();

}  // namespace tr_wire_fixtures
