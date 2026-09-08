// Host tests for TR_OTA_Receiver. Uses FakeOTABackend to track flash-side
// state without touching real esp_ota_*, and a portable mbedtls/sha256.h
// shim (host_shim/sha256_impl.c) so the receiver's SHA computation runs
// unchanged on the host.

#include <gtest/gtest.h>

#include <array>
#include <cstring>
#include <vector>

#include "TR_OTA_Receiver.h"
#include "fake_ota_backend.h"
#include "mbedtls/sha256.h"

namespace {

using R = TR_OTA_Receiver;
using S = R::State;
using E = R::Error;

std::array<uint8_t, 32> sha256_of(const std::vector<uint8_t>& data)
{
    mbedtls_sha256_context ctx;
    mbedtls_sha256_init(&ctx);
    mbedtls_sha256_starts(&ctx, 0);
    mbedtls_sha256_update(&ctx, data.data(), data.size());
    std::array<uint8_t, 32> out{};
    mbedtls_sha256_finish(&ctx, out.data());
    mbedtls_sha256_free(&ctx);
    return out;
}

std::vector<uint8_t> make_image(size_t n, uint8_t seed = 0x42)
{
    std::vector<uint8_t> v(n);
    for (size_t i = 0; i < n; ++i) v[i] = (uint8_t)(seed + i * 31);
    return v;
}

// ----------- happy path -----------

TEST(TrOta, EndToEndHappyPath)
{
    FakeOTABackend be;
    R rx(be);
    auto img = make_image(1024);
    auto hash = sha256_of(img);

    EXPECT_EQ(E::Ok, rx.begin((uint32_t)img.size(), hash.data()));
    EXPECT_EQ(S::Writing, rx.state());

    // Push in 3 chunks
    EXPECT_EQ(E::Ok, rx.writeChunk(0,   img.data(),       400));
    EXPECT_EQ(E::Ok, rx.writeChunk(400, img.data() + 400, 400));
    EXPECT_EQ(E::Ok, rx.writeChunk(800, img.data() + 800, 224));
    EXPECT_EQ(1024u, rx.bytesWritten());

    EXPECT_EQ(E::Ok, rx.finish());
    EXPECT_EQ(S::ReadyToBoot, rx.state());
    EXPECT_TRUE(be.boot_set);
    EXPECT_TRUE(be.ended_once);
    EXPECT_EQ(0, be.abort_calls);
    EXPECT_EQ(img, be.bytes);
}

TEST(TrOta, SingleChunkImage)
{
    FakeOTABackend be;
    R rx(be);
    auto img = make_image(64);
    auto hash = sha256_of(img);

    EXPECT_EQ(E::Ok, rx.begin((uint32_t)img.size(), hash.data()));
    EXPECT_EQ(E::Ok, rx.writeChunk(0, img.data(), img.size()));
    EXPECT_EQ(E::Ok, rx.finish());
    EXPECT_EQ(S::ReadyToBoot, rx.state());
}

// ----------- protocol errors -----------

TEST(TrOta, BadOffsetAborts)
{
    FakeOTABackend be;
    R rx(be);
    auto img = make_image(512);
    auto hash = sha256_of(img);
    rx.begin((uint32_t)img.size(), hash.data());
    rx.writeChunk(0, img.data(), 200);

    // Next chunk should start at offset 200; sending 300 is a protocol error
    EXPECT_EQ(E::BadOffset, rx.writeChunk(300, img.data() + 200, 200));
    EXPECT_EQ(S::VerifyFailed, rx.state());
    EXPECT_EQ(1, be.abort_calls);
}

TEST(TrOta, SizeOverflowAborts)
{
    FakeOTABackend be;
    R rx(be);
    auto img = make_image(100);
    auto hash = sha256_of(img);
    rx.begin((uint32_t)img.size(), hash.data());
    rx.writeChunk(0, img.data(), 50);

    // Try to write more bytes than total_size allows
    EXPECT_EQ(E::SizeOverflow, rx.writeChunk(50, img.data() + 50, 100));
    EXPECT_EQ(S::VerifyFailed, rx.state());
    EXPECT_EQ(1, be.abort_calls);
}

TEST(TrOta, FinishWithShortImageIsSizeMismatch)
{
    FakeOTABackend be;
    R rx(be);
    auto img = make_image(200);
    auto hash = sha256_of(img);
    rx.begin((uint32_t)img.size(), hash.data());
    rx.writeChunk(0, img.data(), 100);   // only half

    EXPECT_EQ(E::SizeMismatch, rx.finish());
    EXPECT_EQ(S::VerifyFailed, rx.state());
    EXPECT_EQ(1, be.abort_calls);
}

TEST(TrOta, ShaMismatchAbortsBeforeSetBoot)
{
    FakeOTABackend be;
    R rx(be);
    auto img = make_image(300);
    auto hash = sha256_of(img);

    // Flip one byte of the expected hash so verification fails
    hash[0] ^= 0xFF;

    rx.begin((uint32_t)img.size(), hash.data());
    rx.writeChunk(0, img.data(), img.size());
    EXPECT_EQ(E::ShaMismatch, rx.finish());
    EXPECT_EQ(S::VerifyFailed, rx.state());
    EXPECT_FALSE(be.boot_set);
    EXPECT_EQ(1, be.abort_calls);
}

TEST(TrOta, WriteChunkWithoutBeginReturnsSessionNotActive)
{
    FakeOTABackend be;
    R rx(be);
    EXPECT_EQ(E::SessionNotActive, rx.writeChunk(0, nullptr, 0));
    EXPECT_EQ(S::Idle, rx.state());
}

TEST(TrOta, FinishWithoutBeginReturnsSessionNotActive)
{
    FakeOTABackend be;
    R rx(be);
    EXPECT_EQ(E::SessionNotActive, rx.finish());
    EXPECT_EQ(S::Idle, rx.state());
}

TEST(TrOta, ZeroSizeImageRejectedAtBegin)
{
    FakeOTABackend be;
    R rx(be);
    std::array<uint8_t, 32> z{};
    EXPECT_EQ(E::BeginFailed, rx.begin(0, z.data()));
}

// ----------- backend-injected failures -----------

TEST(TrOta, BackendBeginFailure)
{
    FakeOTABackend be;
    be.begin_rc = -5;
    R rx(be);
    auto img = make_image(64);
    auto hash = sha256_of(img);
    EXPECT_EQ(E::BeginFailed, rx.begin((uint32_t)img.size(), hash.data()));
    EXPECT_EQ(S::VerifyFailed, rx.state());
}

TEST(TrOta, BackendWriteFailure)
{
    FakeOTABackend be;
    be.fail_write_after_n = true;
    be.write_fail_threshold = 100;
    R rx(be);
    auto img = make_image(300);
    auto hash = sha256_of(img);
    rx.begin((uint32_t)img.size(), hash.data());
    EXPECT_EQ(E::Ok, rx.writeChunk(0, img.data(), 50));
    EXPECT_EQ(E::WriteFailed, rx.writeChunk(50, img.data() + 50, 100));
    EXPECT_EQ(S::VerifyFailed, rx.state());
    EXPECT_EQ(1, be.abort_calls);
}

TEST(TrOta, BackendEndFailure)
{
    FakeOTABackend be;
    be.end_rc = -7;
    R rx(be);
    auto img = make_image(128);
    auto hash = sha256_of(img);
    rx.begin((uint32_t)img.size(), hash.data());
    rx.writeChunk(0, img.data(), img.size());
    EXPECT_EQ(E::EndFailed, rx.finish());
    EXPECT_EQ(S::VerifyFailed, rx.state());
    EXPECT_FALSE(be.boot_set);
}

TEST(TrOta, BackendSetBootFailure)
{
    FakeOTABackend be;
    be.set_boot_rc = -8;
    R rx(be);
    auto img = make_image(128);
    auto hash = sha256_of(img);
    rx.begin((uint32_t)img.size(), hash.data());
    rx.writeChunk(0, img.data(), img.size());
    EXPECT_EQ(E::SetBootFailed, rx.finish());
    EXPECT_EQ(S::VerifyFailed, rx.state());
}

// ----------- session lifecycle -----------

TEST(TrOta, BeginWhileWritingIsAlreadyActive)
{
    FakeOTABackend be;
    R rx(be);
    auto img = make_image(64);
    auto hash = sha256_of(img);
    rx.begin((uint32_t)img.size(), hash.data());
    rx.writeChunk(0, img.data(), 32);

    EXPECT_EQ(E::AlreadyActive, rx.begin((uint32_t)img.size(), hash.data()));
    EXPECT_EQ(S::Writing, rx.state());   // still in original session
}

TEST(TrOta, BeginAfterVerifyFailedRestartsCleanly)
{
    FakeOTABackend be;
    R rx(be);
    auto img1 = make_image(64);
    auto h1 = sha256_of(img1);
    h1[0] ^= 0xFF;                       // force mismatch
    rx.begin((uint32_t)img1.size(), h1.data());
    rx.writeChunk(0, img1.data(), img1.size());
    rx.finish();                          // -> VerifyFailed
    ASSERT_EQ(S::VerifyFailed, rx.state());

    // Now restart cleanly with the right hash
    auto img2 = make_image(128, 0x99);
    auto h2 = sha256_of(img2);
    EXPECT_EQ(E::Ok, rx.begin((uint32_t)img2.size(), h2.data()));
    EXPECT_EQ(S::Writing, rx.state());
    EXPECT_EQ(E::Ok, rx.writeChunk(0, img2.data(), img2.size()));
    EXPECT_EQ(E::Ok, rx.finish());
    EXPECT_EQ(S::ReadyToBoot, rx.state());
}

TEST(TrOta, AbortReturnsToIdle)
{
    FakeOTABackend be;
    R rx(be);
    auto img = make_image(64);
    auto hash = sha256_of(img);
    rx.begin((uint32_t)img.size(), hash.data());
    rx.writeChunk(0, img.data(), 32);
    EXPECT_EQ(E::Ok, rx.abort());
    EXPECT_EQ(S::Idle, rx.state());
    EXPECT_EQ(1, be.abort_calls);
    EXPECT_EQ(0u, rx.bytesWritten());
}

TEST(TrOta, AbortIdleSessionIsNoopButSafe)
{
    FakeOTABackend be;
    R rx(be);
    EXPECT_EQ(E::Ok, rx.abort());
    EXPECT_EQ(S::Idle, rx.state());
    EXPECT_EQ(0, be.abort_calls);   // nothing to abort
}

// ----------- status callback -----------

struct CbCapture {
    int calls = 0;
    S last_state = S::Idle;
    E last_err = E::Ok;
    size_t last_bytes = 0;
};

void cb(void* user, S s, E e, size_t b)
{
    auto* c = static_cast<CbCapture*>(user);
    ++c->calls;
    c->last_state = s;
    c->last_err = e;
    c->last_bytes = b;
}

TEST(TrOta, StatusCallbackFiresOnStateTransitions)
{
    FakeOTABackend be;
    R rx(be);
    CbCapture c;
    rx.setStatusCallback(&cb, &c);

    auto img = make_image(64);
    auto hash = sha256_of(img);

    rx.begin((uint32_t)img.size(), hash.data());
    EXPECT_EQ(1, c.calls);
    EXPECT_EQ(S::Writing, c.last_state);

    rx.writeChunk(0, img.data(), img.size());
    // writeChunk is intentionally silent (rate-limited at the caller)
    EXPECT_EQ(1, c.calls);

    rx.finish();
    EXPECT_EQ(2, c.calls);
    EXPECT_EQ(S::ReadyToBoot, c.last_state);
    EXPECT_EQ(64u, c.last_bytes);
}

// ----------- known-vector SHA-256 sanity (validates the host shim itself) ----

TEST(TrOta, ShaShimMatchesKnownVector)
{
    // SHA-256("abc") = ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad
    const uint8_t expected[32] = {
        0xba,0x78,0x16,0xbf, 0x8f,0x01,0xcf,0xea, 0x41,0x41,0x40,0xde, 0x5d,0xae,0x22,0x23,
        0xb0,0x03,0x61,0xa3, 0x96,0x17,0x7a,0x9c, 0xb4,0x10,0xff,0x61, 0xf2,0x00,0x15,0xad,
    };
    auto got = sha256_of({'a','b','c'});
    EXPECT_EQ(0, std::memcmp(got.data(), expected, 32));
}

}  // namespace

// ---------------------------------------------------------------------------
// #1125 — image identity.
//
// Nothing in the OTA path inspected the incoming image: begin() takes a size
// and a SHA, writeChunk() checks ordering, and esp_ota_end() validates the
// header, segment layout and chip id — none of which distinguishes a
// flight-computer image built for V8 from one built for V9. They differ where
// it matters most: PYRO_ARM is GPIO5 on V8 and GPIO16 on V9, and on V9 GPIO5 is
// the rail latch, so a -v8 image on a V9 board arms the power-hold pin and
// never asserts the real ARM line. Continuity still reads normal on the pad and
// no channel can conduct at apogee.
// ---------------------------------------------------------------------------

// Build an image whose app descriptor names `project` and `version`, at the
// real offset (image header + first segment header = byte 32).
std::vector<uint8_t> make_image_with_desc(size_t n,
                                          const char* project,
                                          const char* version,
                                          uint32_t magic = 0xABCD5432u)
{
    std::vector<uint8_t> v = make_image(n);
    const size_t off = R::APP_DESC_OFFSET;
    // magic @0, version @16, project_name @48 within the descriptor
    std::memcpy(v.data() + off, &magic, sizeof(magic));
    std::memset(v.data() + off + 16, 0, 32);
    std::memset(v.data() + off + 48, 0, 32);
    std::strncpy((char*)v.data() + off + 16, version, 31);
    std::strncpy((char*)v.data() + off + 48, project, 31);
    return v;
}

// Push the whole image in one chunk and report what the receiver said.
E push_all(R& rx, const std::vector<uint8_t>& img)
{
    return rx.writeChunk(0, img.data(), img.size());
}

TEST(TrOtaIdentity, MatchingProjectAndBoardIsAccepted)
{
    FakeOTABackend be;
    R rx(be);
    rx.setExpectedIdentity("out_computer", "-v9");
    auto img = make_image_with_desc(1024, "out_computer", "abc1234-v9+20260907-1200");
    auto hash = sha256_of(img);

    ASSERT_EQ(E::Ok, rx.begin((uint32_t)img.size(), hash.data()));
    EXPECT_EQ(E::Ok, push_all(rx, img));
    EXPECT_EQ(E::Ok, rx.finish());
    EXPECT_EQ(S::ReadyToBoot, rx.state());
}

TEST(TrOtaIdentity, WrongBoardSuffixIsRefusedAndNothingIsFlashed)
{
    FakeOTABackend be;
    R rx(be);
    rx.setExpectedIdentity("flight_computer", "-v9");
    // The exact scenario: a -v8 flight-computer image offered to a V9 board.
    auto img = make_image_with_desc(1024, "flight_computer", "abc1234-v8+20260907-1200");
    auto hash = sha256_of(img);

    ASSERT_EQ(E::Ok, rx.begin((uint32_t)img.size(), hash.data()));
    EXPECT_EQ(E::ImageIdentityMismatch, push_all(rx, img));
    EXPECT_EQ(S::VerifyFailed, rx.state());
    EXPECT_EQ(1, be.abort_calls);
    EXPECT_FALSE(be.boot_set);
    EXPECT_TRUE(be.bytes.empty()) << "a refused image must not reach the flash at all";
}

TEST(TrOtaIdentity, WrongProjectIsRefused)
{
    FakeOTABackend be;
    R rx(be);
    rx.setExpectedIdentity("flight_computer", "-v9");
    // An out-computer image offered to the flight computer: same board, wrong app.
    auto img = make_image_with_desc(1024, "out_computer", "abc1234-v9+20260907-1200");
    auto hash = sha256_of(img);

    ASSERT_EQ(E::Ok, rx.begin((uint32_t)img.size(), hash.data()));
    EXPECT_EQ(E::ImageIdentityMismatch, push_all(rx, img));
    EXPECT_FALSE(be.boot_set);
}

TEST(TrOtaIdentity, MissingDescriptorMagicIsRefused)
{
    FakeOTABackend be;
    R rx(be);
    rx.setExpectedIdentity("out_computer", "-v9");
    auto img = make_image_with_desc(1024, "out_computer", "abc1234-v9+x", 0xDEADBEEFu);
    auto hash = sha256_of(img);

    ASSERT_EQ(E::Ok, rx.begin((uint32_t)img.size(), hash.data()));
    EXPECT_EQ(E::ImageIdentityMismatch, push_all(rx, img))
        << "an image with no identifiable app descriptor must not be installed";
}

TEST(TrOtaIdentity, CheckSurvivesChunkingAcrossTheDescriptor)
{
    // The descriptor spans bytes 32..288, so a small first chunk must not let
    // a wrong image through — the check waits until enough bytes have arrived.
    FakeOTABackend be;
    R rx(be);
    rx.setExpectedIdentity("flight_computer", "-v9");
    auto img = make_image_with_desc(1024, "flight_computer", "abc1234-v8+20260907-1200");
    auto hash = sha256_of(img);

    ASSERT_EQ(E::Ok, rx.begin((uint32_t)img.size(), hash.data()));
    EXPECT_EQ(E::Ok, rx.writeChunk(0, img.data(), 20));    // header not complete yet
    EXPECT_EQ(E::Ok, rx.writeChunk(20, img.data() + 20, 100));
    // this chunk completes the descriptor
    EXPECT_EQ(E::ImageIdentityMismatch, rx.writeChunk(120, img.data() + 120, 400));
    EXPECT_FALSE(be.boot_set);
}

TEST(TrOtaIdentity, BenchSuffixBetweenBoardAndDateStillMatches)
{
    // out_computer stamps "<sha><board><bench>+<date>", so the board suffix is
    // not necessarily last.
    FakeOTABackend be;
    R rx(be);
    rx.setExpectedIdentity("out_computer", "-v9");
    auto img = make_image_with_desc(1024, "out_computer", "abc1234-v9-bench+20260907-1200");
    auto hash = sha256_of(img);

    ASSERT_EQ(E::Ok, rx.begin((uint32_t)img.size(), hash.data()));
    EXPECT_EQ(E::Ok, push_all(rx, img));
    EXPECT_EQ(E::Ok, rx.finish());
}

TEST(TrOtaIdentity, UnsetIdentityLeavesBehaviourUnchanged)
{
    // Every existing caller and test predates the check; with no expected
    // identity set, an image with no descriptor at all must still install.
    FakeOTABackend be;
    R rx(be);
    auto img = make_image(1024);
    auto hash = sha256_of(img);

    ASSERT_EQ(E::Ok, rx.begin((uint32_t)img.size(), hash.data()));
    EXPECT_EQ(E::Ok, push_all(rx, img));
    EXPECT_EQ(E::Ok, rx.finish());
    EXPECT_TRUE(be.boot_set);
}

TEST(TrOtaIdentity, ASecondSessionRechecksIdentity)
{
    // identity_done_ is per-session: a refused image must not leave the next
    // session believing it has already been checked.
    FakeOTABackend be;
    R rx(be);
    rx.setExpectedIdentity("flight_computer", "-v9");

    auto bad = make_image_with_desc(1024, "flight_computer", "abc-v8+d");
    auto bad_hash = sha256_of(bad);
    ASSERT_EQ(E::Ok, rx.begin((uint32_t)bad.size(), bad_hash.data()));
    ASSERT_EQ(E::ImageIdentityMismatch, push_all(rx, bad));

    auto good = make_image_with_desc(1024, "flight_computer", "abc-v9+d");
    auto good_hash = sha256_of(good);
    ASSERT_EQ(E::Ok, rx.begin((uint32_t)good.size(), good_hash.data()));
    EXPECT_EQ(E::Ok, push_all(rx, good));
    EXPECT_EQ(E::Ok, rx.finish());
    EXPECT_TRUE(be.boot_set);
}

// ── #1142 item 2: abort() from ReadyToBoot must put otadata back ──
//
// finish() calls backend_.setBootPartition() and then enters ReadyToBoot, so by
// that point the next reset boots the NEW image. abort() skipped the backend
// entirely outside Writing/VerifyFailed, reset to Idle and told the app the
// session was gone -- while otadata still pointed at the cancelled update. The
// header even promised "Always safe; resets to Idle."
//
// The window is reachable: handleOtaFinish() arms the restart 500 ms out, and
// BLE cmd 72 is dispatched in-place on the NimBLE host task, cancelling it. The
// vehicle then keeps running the old image with otadata pointing at the new one
// until anything resets it -- a watchdog, a brownout, or the operator power
// cycling precisely because the app said "idle".

TEST(OtaAbortFromReadyToBoot, RestoresTheBootPartition) {
    FakeOTABackend backend;
    TR_OTA_Receiver rx(backend);

    auto img  = make_image(256);
    auto hash = sha256_of(img);
    ASSERT_EQ(rx.begin((uint32_t)img.size(), hash.data()),
              TR_OTA_Receiver::Error::Ok);
    ASSERT_EQ(rx.writeChunk(0, img.data(), (uint16_t)img.size()),
              TR_OTA_Receiver::Error::Ok);
    ASSERT_EQ(rx.finish(), TR_OTA_Receiver::Error::Ok);
    ASSERT_EQ(rx.state(), TR_OTA_Receiver::State::ReadyToBoot);
    ASSERT_TRUE(backend.boot_set) << "finish() should have committed the switch";

    EXPECT_EQ(rx.abort(), TR_OTA_Receiver::Error::Ok);
    EXPECT_EQ(backend.restore_boot_calls, 1);
    EXPECT_FALSE(backend.boot_set) << "otadata still points at the cancelled image";
    EXPECT_EQ(rx.state(), TR_OTA_Receiver::State::Idle);
}

TEST(OtaAbortFromReadyToBoot, SaysSoWhenItCannotRestore) {
    // If the restore fails the session must NOT claim to be idle -- the reboot
    // really is still coming, and the operator needs to know that rather than
    // being shown a cancelled update.
    FakeOTABackend backend;
    TR_OTA_Receiver rx(backend);

    auto img  = make_image(256);
    auto hash = sha256_of(img);
    ASSERT_EQ(rx.begin((uint32_t)img.size(), hash.data()),
              TR_OTA_Receiver::Error::Ok);
    ASSERT_EQ(rx.writeChunk(0, img.data(), (uint16_t)img.size()),
              TR_OTA_Receiver::Error::Ok);
    ASSERT_EQ(rx.finish(), TR_OTA_Receiver::Error::Ok);

    backend.restore_boot_rc = -1;
    EXPECT_EQ(rx.abort(), TR_OTA_Receiver::Error::BootAlreadyCommitted);
    EXPECT_EQ(rx.state(), TR_OTA_Receiver::State::ReadyToBoot)
        << "reported idle while otadata still points at the new image";
}

TEST(OtaAbortFromReadyToBoot, OtherStatesAreUnchanged) {
    // The Writing path must keep calling backend.abort() and must NOT touch the
    // boot partition -- nothing has been committed there.
    FakeOTABackend backend;
    TR_OTA_Receiver rx(backend);

    auto img  = make_image(256);
    auto hash = sha256_of(img);
    ASSERT_EQ(rx.begin((uint32_t)img.size(), hash.data()),
              TR_OTA_Receiver::Error::Ok);
    ASSERT_EQ(rx.writeChunk(0, img.data(), 128), TR_OTA_Receiver::Error::Ok);

    EXPECT_EQ(rx.abort(), TR_OTA_Receiver::Error::Ok);
    EXPECT_EQ(backend.abort_calls, 1);
    EXPECT_EQ(backend.restore_boot_calls, 0);
    EXPECT_EQ(rx.state(), TR_OTA_Receiver::State::Idle);
}
