#ifndef TR_OTA_RECEIVER_H
#define TR_OTA_RECEIVER_H

#include <cstdint>
#include <cstddef>

#include "TR_OTA_Backend.h"

// State-machine + SHA-256 accumulator on top of a TR_OTA_Backend.
//
// Chunk ordering: writeChunk() requires the offset to equal bytesWritten().
// The BLE transport already preserves order via write-with-response, so
// out-of-order is treated as a protocol error rather than buffered.
//
// Thread model: not thread-safe. Caller serializes on a single task.
class TR_OTA_Receiver
{
public:
    enum class State : uint8_t {
        Idle,           // No session
        Writing,        // begin() succeeded, chunks accepted
        ReadyToBoot,    // finish() OK; caller may esp_restart()
        VerifyFailed,   // terminal — abort() to return to Idle
    };

    enum class Error : uint8_t {
        Ok = 0,
        AlreadyActive,          // begin() called twice
        SessionNotActive,       // writeChunk/finish without begin
        BeginFailed,            // backend->begin() returned non-zero
        BadOffset,              // chunk offset != bytes_written_
        SizeOverflow,           // chunk would push past total_size_
        WriteFailed,            // backend->write() returned non-zero
        SizeMismatch,           // finish() called before total bytes received
        ShaMismatch,            // computed SHA != expected
        EndFailed,              // backend->end() returned non-zero
        SetBootFailed,          // backend->setBootPartition() returned non-zero
        ImageIdentityMismatch,  // #1125: the incoming image's app descriptor names a
                                // different project or a different board revision
    };

    // Optional status callback. Fired synchronously inside begin/writeChunk/
    // finish/abort whenever state_ or last_error_ changes. detail may be a
    // brief string (machine-stable token, not human prose) or nullptr.
    using StatusCb = void(*)(void* user_ctx, State state, Error err, size_t bytes_written);

    explicit TR_OTA_Receiver(TR_OTA_Backend& backend);

    // #1125: refuse an image that was not built for THIS project and board.
    //
    // Nothing in the OTA path inspected the incoming image: begin() takes a
    // size and a SHA, writeChunk() checks ordering, and esp_ota_end() validates
    // the header, segment layout and chip id — none of which distinguishes a
    // flight-computer image built for V8 from one built for V9. They differ
    // where it matters most: PYRO_ARM is GPIO5 on V8 and GPIO16 on V9, and on
    // V9 GPIO5 is the rail latch. A -v8 image on a V9 board therefore arms the
    // power-hold pin and never asserts the real ARM line, continuity still
    // reads normal on the pad, and no channel can conduct at apogee.
    //
    // `project_name` must match exactly. `board_suffix` is the CMake
    // TR_BOARD_SUFFIX ("-v9", "-m1", ...) and must appear in the image's
    // version string. Pass nullptr or "" for either to skip that half; with
    // both unset the check is disabled and behaviour is unchanged, which is
    // what keeps existing callers and the host tests valid.
    void setExpectedIdentity(const char* project_name, const char* board_suffix);

    // Offset of esp_app_desc_t within an ESP-IDF app image: the image header
    // plus the first segment header. The descriptor is the first thing in that
    // segment, so it is fully present once this many bytes have arrived.
    static constexpr size_t APP_DESC_OFFSET = 32;
    static constexpr size_t APP_DESC_SIZE   = 256;
    static constexpr uint32_t APP_DESC_MAGIC = 0xABCD5432u;
    ~TR_OTA_Receiver();

    TR_OTA_Receiver(const TR_OTA_Receiver&) = delete;
    TR_OTA_Receiver& operator=(const TR_OTA_Receiver&) = delete;

    void setStatusCallback(StatusCb cb, void* user_ctx);

    // Open a session. total_size: full image bytes; sha256: expected SHA-256
    // of the full image, exactly 32 bytes.
    Error begin(uint32_t total_size, const uint8_t sha256[32]);

    // Append the next chunk. offset must equal bytesWritten().
    Error writeChunk(uint32_t offset, const uint8_t* data, size_t len);

    // Verify SHA, end backend write, set boot partition. State becomes
    // ReadyToBoot on success (caller schedules reboot). On failure the
    // session is aborted internally — state becomes VerifyFailed.
    Error finish();

    // Cancel an in-flight session. Always safe; resets to Idle.
    Error abort();

    State  state()         const { return state_; }
    Error  lastError()     const { return last_error_; }
    size_t bytesWritten()  const { return bytes_written_; }
    size_t expectedSize()  const { return total_size_; }

private:
    TR_OTA_Backend& backend_;

    State state_ = State::Idle;
    Error last_error_ = Error::Ok;
    size_t total_size_ = 0;
    size_t bytes_written_ = 0;
    uint8_t expected_sha_[32] = {};

    // mbedtls SHA-256 context is opaque so we keep it via a void* + heap
    // allocation to avoid pulling mbedtls into this header.
    void* sha_ctx_ = nullptr;

    // #1125 identity check
    char     exp_project_[32] = {0};
    char     exp_suffix_[16]  = {0};
    bool     identity_done_   = false;   // checked (or skipped) for this session
    uint8_t  hdr_[APP_DESC_OFFSET + APP_DESC_SIZE] = {0};
    size_t   hdr_len_         = 0;
    // Returns false when the buffered header names a different project/board.
    bool checkImageIdentity();

    StatusCb status_cb_ = nullptr;
    void* status_cb_user_ = nullptr;

    void resetSession();
    void notify();
    void initShaCtx();
    void freeShaCtx();
    void shaUpdate(const uint8_t* data, size_t len);
    bool shaFinalAndCompare(const uint8_t expected[32]);
};

#endif // TR_OTA_RECEIVER_H
