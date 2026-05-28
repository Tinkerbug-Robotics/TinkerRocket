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
