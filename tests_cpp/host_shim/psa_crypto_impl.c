// Host PSA Crypto hashing shim backing psa/crypto.h, implemented over the
// portable SHA-256 in sha256_impl.c (via the mbedtls/sha256.h shim). Lets
// TR_OTA_Receiver's PSA code path compile and produce a correct SHA-256 in
// host unit tests. ESP-IDF 6.0+ builds use the real TF-PSA-Crypto.
#include "psa/crypto.h"

psa_status_t psa_crypto_init(void)
{
    return PSA_SUCCESS;  // nothing to initialize for the host shim
}

psa_status_t psa_hash_setup(psa_hash_operation_t* op, psa_algorithm_t alg)
{
    (void)alg;  // shim only does SHA-256
    if (!op) return (psa_status_t)-1;
    mbedtls_sha256_init(&op->ctx);
    mbedtls_sha256_starts(&op->ctx, 0);  // 0 = SHA-256 (not 224)
    op->active = 1;
    return PSA_SUCCESS;
}

psa_status_t psa_hash_update(psa_hash_operation_t* op,
                             const uint8_t* input, size_t input_length)
{
    if (!op || !op->active) return (psa_status_t)-1;
    mbedtls_sha256_update(&op->ctx, input, input_length);
    return PSA_SUCCESS;
}

psa_status_t psa_hash_finish(psa_hash_operation_t* op,
                             uint8_t* hash, size_t hash_size, size_t* hash_length)
{
    if (!op || !op->active) return (psa_status_t)-1;
    if (hash_size < 32) return (psa_status_t)-1;
    mbedtls_sha256_finish(&op->ctx, hash);
    if (hash_length) *hash_length = 32;
    op->active = 0;
    return PSA_SUCCESS;
}

psa_status_t psa_hash_abort(psa_hash_operation_t* op)
{
    if (op && op->active) {
        mbedtls_sha256_free(&op->ctx);
        op->active = 0;
    }
    return PSA_SUCCESS;
}
