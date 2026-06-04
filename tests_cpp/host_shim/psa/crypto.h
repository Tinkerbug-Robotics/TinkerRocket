// Host-side psa/crypto.h shim. Mirrors the subset of the PSA Crypto hashing
// API used by TR_OTA_Receiver (ESP-IDF 6.0+ uses the real TF-PSA-Crypto;
// mbedtls/sha256.h became private). Backed by the portable SHA-256 in
// sha256_impl.c via the mbedtls shim, so host unit tests exercise the real
// PSA code path against a correct digest. Host tests only.
#ifndef HOST_SHIM_PSA_CRYPTO_H
#define HOST_SHIM_PSA_CRYPTO_H

#include <stddef.h>
#include <stdint.h>
#include "mbedtls/sha256.h"   // reuse the host SHA-256 implementation

#ifdef __cplusplus
extern "C" {
#endif

typedef int32_t  psa_status_t;
typedef uint32_t psa_algorithm_t;

#define PSA_SUCCESS         ((psa_status_t)0)
#define PSA_ALG_SHA_256     ((psa_algorithm_t)0x02000009)  // real PSA value (shim ignores it)

typedef struct {
    mbedtls_sha256_context ctx;
    int active;
} psa_hash_operation_t;

#define PSA_HASH_OPERATION_INIT { {0}, 0 }

psa_status_t psa_crypto_init(void);
psa_status_t psa_hash_setup(psa_hash_operation_t* operation, psa_algorithm_t alg);
psa_status_t psa_hash_update(psa_hash_operation_t* operation,
                             const uint8_t* input, size_t input_length);
psa_status_t psa_hash_finish(psa_hash_operation_t* operation,
                             uint8_t* hash, size_t hash_size, size_t* hash_length);
psa_status_t psa_hash_abort(psa_hash_operation_t* operation);

#ifdef __cplusplus
}
#endif

#endif  // HOST_SHIM_PSA_CRYPTO_H
