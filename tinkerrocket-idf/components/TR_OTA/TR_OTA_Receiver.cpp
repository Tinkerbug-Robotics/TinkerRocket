#include "TR_OTA_Receiver.h"

#include <cstdlib>
#include <cstring>

// ESP-IDF 6.0 (TF-PSA-Crypto) made mbedtls' low-level <mbedtls/sha256.h>
// private. Use the public PSA Crypto hashing API for the OTA image digest.
#include <psa/crypto.h>

TR_OTA_Receiver::TR_OTA_Receiver(TR_OTA_Backend& backend)
    : backend_(backend)
{
}

TR_OTA_Receiver::~TR_OTA_Receiver()
{
    if (state_ != State::Idle && state_ != State::ReadyToBoot)
    {
        backend_.abort();
    }
    freeShaCtx();
}

void TR_OTA_Receiver::setStatusCallback(StatusCb cb, void* user_ctx)
{
    status_cb_ = cb;
    status_cb_user_ = user_ctx;
}

void TR_OTA_Receiver::resetSession()
{
    state_ = State::Idle;
    total_size_ = 0;
    bytes_written_ = 0;
    last_error_ = Error::Ok;
    std::memset(expected_sha_, 0, sizeof(expected_sha_));
    identity_done_ = false;          // #1125: per-session
    hdr_len_ = 0;
    freeShaCtx();
}

void TR_OTA_Receiver::notify()
{
    if (status_cb_)
    {
        status_cb_(status_cb_user_, state_, last_error_, bytes_written_);
    }
}

void TR_OTA_Receiver::initShaCtx()
{
    freeShaCtx();
    // psa_crypto_init() is idempotent; safe to call on each session start.
    if (psa_crypto_init() != PSA_SUCCESS) return;
    auto* ctx = static_cast<psa_hash_operation_t*>(std::malloc(sizeof(psa_hash_operation_t)));
    if (!ctx) return;
    static const psa_hash_operation_t kInit = PSA_HASH_OPERATION_INIT;
    *ctx = kInit;
    if (psa_hash_setup(ctx, PSA_ALG_SHA_256) != PSA_SUCCESS)
    {
        std::free(ctx);
        return;
    }
    sha_ctx_ = ctx;
}

void TR_OTA_Receiver::freeShaCtx()
{
    if (sha_ctx_)
    {
        auto* ctx = static_cast<psa_hash_operation_t*>(sha_ctx_);
        psa_hash_abort(ctx);  // safe on an already-finished/inactive op
        std::free(ctx);
        sha_ctx_ = nullptr;
    }
}

void TR_OTA_Receiver::shaUpdate(const uint8_t* data, size_t len)
{
    if (!sha_ctx_) return;
    auto* ctx = static_cast<psa_hash_operation_t*>(sha_ctx_);
    psa_hash_update(ctx, data, len);
}

bool TR_OTA_Receiver::shaFinalAndCompare(const uint8_t expected[32])
{
    if (!sha_ctx_) return false;
    auto* ctx = static_cast<psa_hash_operation_t*>(sha_ctx_);
    uint8_t computed[32];
    size_t computed_len = 0;
    if (psa_hash_finish(ctx, computed, sizeof(computed), &computed_len) != PSA_SUCCESS)
    {
        return false;
    }
    return (computed_len == 32) && (std::memcmp(computed, expected, 32) == 0);
}

TR_OTA_Receiver::Error TR_OTA_Receiver::begin(uint32_t total_size, const uint8_t sha256[32])
{
    if (state_ == State::Writing)
    {
        last_error_ = Error::AlreadyActive;
        notify();
        return Error::AlreadyActive;
    }

    // Clean restart from VerifyFailed / ReadyToBoot
    if (state_ != State::Idle)
    {
        backend_.abort();
        resetSession();
    }

    if (total_size == 0)
    {
        last_error_ = Error::BeginFailed;
        notify();
        return Error::BeginFailed;
    }

    int rc = backend_.begin(total_size);
    if (rc != 0)
    {
        last_error_ = Error::BeginFailed;
        state_ = State::VerifyFailed;
        notify();
        return Error::BeginFailed;
    }

    total_size_ = total_size;
    bytes_written_ = 0;
    std::memcpy(expected_sha_, sha256, sizeof(expected_sha_));
    initShaCtx();

    identity_done_ = false;          // #1125: a fresh session re-checks
    hdr_len_ = 0;
    state_ = State::Writing;
    last_error_ = Error::Ok;
    notify();
    return Error::Ok;
}

void TR_OTA_Receiver::setExpectedIdentity(const char* project_name, const char* board_suffix)
{
    exp_project_[0] = '\0';
    exp_suffix_[0]  = '\0';
    if (project_name && project_name[0])
    {
        std::strncpy(exp_project_, project_name, sizeof(exp_project_) - 1);
        exp_project_[sizeof(exp_project_) - 1] = '\0';
    }
    if (board_suffix && board_suffix[0])
    {
        std::strncpy(exp_suffix_, board_suffix, sizeof(exp_suffix_) - 1);
        exp_suffix_[sizeof(exp_suffix_) - 1] = '\0';
    }
}

// #1125. Layout of esp_app_desc_t, which we parse by offset rather than by
// including esp_app_format.h so the component stays host-testable:
//   0  magic (u32)   4  secure_version (u32)   8  reserv1[2] (u32)
//   16 version[32]   48 project_name[32]       80 time[16]  96 date[16]
//   112 idf_ver[32]  144 app_elf_sha256[32]    176 reserv2[20]
bool TR_OTA_Receiver::checkImageIdentity()
{
    if (exp_project_[0] == '\0' && exp_suffix_[0] == '\0') return true;  // disabled

    const uint8_t* d = hdr_ + APP_DESC_OFFSET;
    uint32_t magic = 0;
    std::memcpy(&magic, d, sizeof(magic));
    if (magic != APP_DESC_MAGIC) return false;   // not an app image we can identify

    char version[33]      = {0};
    char project_name[33] = {0};
    std::memcpy(version,      d + 16, 32);
    std::memcpy(project_name, d + 48, 32);
    version[32] = '\0';
    project_name[32] = '\0';

    if (exp_project_[0] && std::strncmp(project_name, exp_project_, sizeof(exp_project_)) != 0)
    {
        return false;
    }
    // The version is "<sha><board suffix>[<bench suffix>]+<date>"; the board
    // suffixes in use (-v7 -v8 -v9 -m1) are not substrings of one another, so a
    // plain search is unambiguous and survives the optional bench suffix.
    if (exp_suffix_[0] && std::strstr(version, exp_suffix_) == nullptr)
    {
        return false;
    }
    return true;
}

TR_OTA_Receiver::Error TR_OTA_Receiver::writeChunk(uint32_t offset, const uint8_t* data, size_t len)
{
    if (state_ != State::Writing)
    {
        // #1156 item 3: the StatusCb contract says the callback fires whenever
        // state_ OR last_error_ changes, and onFileTransferWrite relies on it
        // ("writeChunk pushes its own status via the receiver callback on
        // failure"). This was the one error path that changed last_error_ and
        // returned without notifying, so chunks arriving with no session open
        // produced no BLE status at all.
        last_error_ = Error::SessionNotActive;
        notify();
        return Error::SessionNotActive;
    }

    if (offset != bytes_written_)
    {
        last_error_ = Error::BadOffset;
        state_ = State::VerifyFailed;
        backend_.abort();
        notify();
        return Error::BadOffset;
    }

    if (bytes_written_ + len > total_size_)
    {
        last_error_ = Error::SizeOverflow;
        state_ = State::VerifyFailed;
        backend_.abort();
        notify();
        return Error::SizeOverflow;
    }

    // #1125: snapshot the head of the image so the app descriptor can be
    // identified. Runs before the backend write, so a wrong-board image is
    // refused rather than half-flashed.
    if (!identity_done_ && hdr_len_ < sizeof(hdr_))
    {
        const size_t want = sizeof(hdr_) - hdr_len_;
        const size_t take = (len < want) ? len : want;
        std::memcpy(hdr_ + hdr_len_, data, take);
        hdr_len_ += take;
        if (hdr_len_ >= sizeof(hdr_))
        {
            identity_done_ = true;
            if (!checkImageIdentity())
            {
                last_error_ = Error::ImageIdentityMismatch;
                state_ = State::VerifyFailed;
                backend_.abort();
                notify();
                return Error::ImageIdentityMismatch;
            }
        }
    }

    int rc = backend_.write(data, len);
    if (rc != 0)
    {
        last_error_ = Error::WriteFailed;
        state_ = State::VerifyFailed;
        backend_.abort();
        notify();
        return Error::WriteFailed;
    }

    shaUpdate(data, len);
    bytes_written_ += len;
    // Don't notify on every chunk — the BLE layer rate-limits its own
    // status pushes. Caller can poll bytesWritten().
    return Error::Ok;
}

TR_OTA_Receiver::Error TR_OTA_Receiver::finish()
{
    if (state_ != State::Writing)
    {
        last_error_ = Error::SessionNotActive;
        notify();
        return Error::SessionNotActive;
    }

    if (bytes_written_ != total_size_)
    {
        last_error_ = Error::SizeMismatch;
        state_ = State::VerifyFailed;
        backend_.abort();
        notify();
        return Error::SizeMismatch;
    }

    if (!shaFinalAndCompare(expected_sha_))
    {
        last_error_ = Error::ShaMismatch;
        state_ = State::VerifyFailed;
        backend_.abort();
        notify();
        return Error::ShaMismatch;
    }

    int rc = backend_.end();
    if (rc != 0)
    {
        last_error_ = Error::EndFailed;
        state_ = State::VerifyFailed;
        notify();
        return Error::EndFailed;
    }

    rc = backend_.setBootPartition();
    if (rc != 0)
    {
        last_error_ = Error::SetBootFailed;
        state_ = State::VerifyFailed;
        notify();
        return Error::SetBootFailed;
    }

    state_ = State::ReadyToBoot;
    last_error_ = Error::Ok;
    freeShaCtx();
    notify();
    return Error::Ok;
}

TR_OTA_Receiver::Error TR_OTA_Receiver::abort()
{
    if (state_ == State::Writing || state_ == State::VerifyFailed)
    {
        backend_.abort();
    }
    else if (state_ == State::ReadyToBoot)
    {
        // #1142 item 2: finish() has already called setBootPartition(), so
        // otadata points at the new image RIGHT NOW.  Resetting to Idle here
        // told the app the update was cancelled while the next reset — a
        // watchdog, a brownout, the operator power-cycling because the app
        // said "idle" — would still boot the image they just cancelled.  On a
        // flight computer that is a different firmware than the one the
        // operator believes is loaded.
        //
        // The window is real and reachable: handleOtaFinish() arms the restart
        // 500 ms out, and BLE cmd 72 (abort) is dispatched in-place on the
        // NimBLE host task, which cancels that restart — so the vehicle keeps
        // running the old image with otadata pointing at the new one until
        // something resets it.
        if (backend_.restoreBootPartition() != 0)
        {
            // Could not put it back.  Stay in ReadyToBoot and say so — the
            // reboot IS still coming, and reporting Idle would be a lie.
            notify();
            return Error::BootAlreadyCommitted;
        }
    }
    resetSession();
    notify();
    return Error::Ok;
}
