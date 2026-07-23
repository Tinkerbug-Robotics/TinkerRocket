// CLI writer for the golden-vector corpus.  Usage:
//   wire_fixture_gen [output_dir]        (default: TR_WIRE_FIXTURES_DIR)
// Normally invoked via:  cmake --build <build> --target regen-wire-fixtures
//
// Output is COMMITTED; test_wire_fixture_freshness.cpp fails C++ CI whenever
// the committed files drift from what this program would generate.

#include "wire_fixtures_lib.h"

#include <cstdio>
#include <filesystem>
#include <fstream>

namespace fs = std::filesystem;

int main(int argc, char** argv) {
#ifndef TR_WIRE_FIXTURES_DIR
#error "TR_WIRE_FIXTURES_DIR must be defined by the build"
#endif
    const fs::path outDir = (argc > 1) ? fs::path(argv[1]) : fs::path(TR_WIRE_FIXTURES_DIR);

    const auto fixtures = tr_wire_fixtures::generate();
    if (fixtures.empty()) {
        std::fprintf(stderr, "ERROR: generation produced nothing (big-endian host?)\n");
        return 1;
    }

    size_t written = 0;
    for (const auto& [rel, bytes] : fixtures) {
        const fs::path p = outDir / rel;
        fs::create_directories(p.parent_path());
        std::ofstream f(p, std::ios::binary | std::ios::trunc);
        if (!f) {
            std::fprintf(stderr, "ERROR: cannot write %s\n", p.string().c_str());
            return 1;
        }
        f.write(reinterpret_cast<const char*>(bytes.data()),
                static_cast<std::streamsize>(bytes.size()));
        ++written;
    }
    std::printf("wire_fixture_gen: wrote %zu files to %s\n", written, outDir.string().c_str());
    return 0;
}
