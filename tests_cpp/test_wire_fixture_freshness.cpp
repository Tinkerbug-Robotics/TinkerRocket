// Freshness guard for the golden-vector corpus (docs/plans/android-port.md §2.1).
//
// Regenerates every fixture in memory from the REAL packed structs and
// byte-compares against the committed files under tests_cpp/fixtures/wire/.
// A change to RocketComputerTypes.h that alters any wire layout fails here
// until the fixtures are regenerated (target: regen-wire-fixtures), reviewed
// in the diff, and re-committed — the same discipline the ble_wire_format
// fixtures established, extended to the whole corpus that the Swift and
// Kotlin golden walks consume.

#include <gtest/gtest.h>

#include "wire_fixtures_lib.h"

#include <filesystem>
#include <fstream>
#include <set>
#include <vector>

namespace fs = std::filesystem;

namespace {

#ifndef TR_WIRE_FIXTURES_DIR
#error "TR_WIRE_FIXTURES_DIR must be defined by the build"
#endif

std::vector<uint8_t> readFile(const fs::path& p) {
    std::ifstream in(p, std::ios::binary);
    return std::vector<uint8_t>((std::istreambuf_iterator<char>(in)),
                                std::istreambuf_iterator<char>());
}

}  // namespace

TEST(WireFixtureFreshness, GenerationIsNonEmpty) {
    ASSERT_FALSE(tr_wire_fixtures::generate().empty())
        << "generation produced nothing — big-endian host?";
}

TEST(WireFixtureFreshness, CommittedFilesMatchGenerator) {
    const auto generated = tr_wire_fixtures::generate();
    const fs::path root(TR_WIRE_FIXTURES_DIR);
    ASSERT_TRUE(fs::exists(root))
        << root << " missing — run the regen-wire-fixtures target once and commit.";

    for (const auto& [rel, bytes] : generated) {
        const fs::path p = root / rel;
        ASSERT_TRUE(fs::exists(p))
            << rel << " is generated but not committed — regen + commit.";
        const auto onDisk = readFile(p);
        EXPECT_EQ(onDisk, bytes)
            << rel << " differs from generator output.  A wire layout or "
            << "canonical value changed: regen-wire-fixtures, review the diff, "
            << "bump notes/format in manifest.json if the layout moved, commit.";
    }
}

TEST(WireFixtureFreshness, NoOrphanFilesOnDisk) {
    const auto generated = tr_wire_fixtures::generate();
    const fs::path root(TR_WIRE_FIXTURES_DIR);
    ASSERT_TRUE(fs::exists(root));

    std::set<std::string> known;
    for (const auto& [rel, bytes] : generated) known.insert(rel);

    for (const auto& entry : fs::recursive_directory_iterator(root)) {
        if (!entry.is_regular_file()) continue;
        const std::string rel = fs::relative(entry.path(), root).generic_string();
        EXPECT_TRUE(known.count(rel))
            << rel << " exists on disk but the generator does not produce it — "
            << "stale fixture (delete it) or a hand-added file (add it to the "
            << "generator instead; hand-authored fixtures defeat the freshness "
            << "guarantee).";
    }
}
