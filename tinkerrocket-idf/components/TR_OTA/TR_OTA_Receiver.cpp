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

    state_ = State::Writing;
    last_error_ = Error::Ok;
    notify();
    return Error::Ok;
}

TR_OTA_Receiver::Error TR_OTA_Receiver::writeChunk(uint32_t offset, const uint8_t* data, size_t len)
{
    if (state_ != State::Writing)
    {
        last_error_ = Error::SessionNotActive;
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
    resetSession();
    notify();
    return Error::Ok;
}
