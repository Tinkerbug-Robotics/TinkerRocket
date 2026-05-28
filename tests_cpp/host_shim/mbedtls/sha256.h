// Host-side mbedtls/sha256.h shim. Mirrors the subset of the real mbedtls
// API used by TR_OTA_Receiver so the production source compiles unchanged
// on the host. Implementation is a small portable SHA-256 (sha256_impl.c).
#ifndef HOST_SHIM_MBEDTLS_SHA256_H
#define HOST_SHIM_MBEDTLS_SHA256_H

#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t state[8];
    uint64_t bit_count;
    uint8_t  buf[64];
    size_t   buf_len;
} mbedtls_sha256_context;

void mbedtls_sha256_init(mbedtls_sha256_context* ctx);
void mbedtls_sha256_free(mbedtls_sha256_context* ctx);
int  mbedtls_sha256_starts(mbedtls_sha256_context* ctx, int is224);
int  mbedtls_sha256_update(mbedtls_sha256_context* ctx,
                           const unsigned char* input, size_t ilen);
int  mbedtls_sha256_finish(mbedtls_sha256_context* ctx,
                           unsigned char output[32]);

#ifdef __cplusplus
}
#endif

#endif  // HOST_SHIM_MBEDTLS_SHA256_H
