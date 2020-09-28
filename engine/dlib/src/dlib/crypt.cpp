// Copyright 2020 The Defold Foundation
// Licensed under the Defold License version 1.0 (the "License"); you may not use
// this file except in compliance with the License.
//
// You may obtain a copy of the License, together with FAQs at
// https://www.defold.com/license
//
// Unless required by applicable law or agreed to in writing, software distributed
// under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>
#include "shared_library.h"
#include "crypt.h"

#include <dlib/endian.h>
#include <mbedtls/md5.h>
#include <mbedtls/base64.h>
#include <mbedtls/sha1.h>
#include <mbedtls/sha256.h>
#include <mbedtls/sha512.h>
#include <mbedtls/pk.h>
#include <mbedtls/pk_internal.h>
#include <mbedtls/rsa.h>
#include <mbedtls/md.h>
#include <mbedtls/entropy.h>
#include <mbedtls/ctr_drbg.h>


#include <dlib/log.h> // For debugging the manifest verification issue

namespace dmCrypt
{
    const uint32_t NUM_ROUNDS = 32;

    static inline uint64_t EncryptXTea(uint64_t v, uint32_t* key)
    {
        uint32_t v0 = (uint32_t) (v >> 32);
        uint32_t v1 = (uint32_t) (v & 0xffffffff);

        uint32_t sum = 0, delta = 0x9e3779b9;
        for (uint32_t i = 0; i < NUM_ROUNDS; i++) {
            v0 += (((v1 << 4) ^ (v1 >> 5)) + v1) ^ (sum + dmEndian::ToHost(key[sum & 3]));
            sum += delta;
            v1 += (((v0 << 4) ^ (v0 >> 5)) + v0) ^ (sum + dmEndian::ToHost(key[(sum>>11) & 3]));
        }
        uint64_t ret = dmEndian::ToHost((((uint64_t) v0) << 32 | v1));
        return ret;
    }

    static void EncryptXTeaCTR(uint8_t* data, uint32_t datalen, const uint8_t* key, uint32_t keylen)
    {
        assert(keylen <= 16);
        const uint32_t block_len = 8;
        uint8_t paddedkey[16] = {0};
        memcpy(paddedkey, key, keylen);

        uint64_t counter = 0;

        uint32_t i = 0;
        uint64_t* d = (uint64_t*) data;
        for (i = 0; i < datalen / block_len; i++) {
            uint64_t enc_counter = EncryptXTea(counter, (uint32_t*) paddedkey);
            d[i] ^= enc_counter;
            data += block_len;
            counter++;
        }

        uint64_t enc_counter = EncryptXTea(counter, (uint32_t*) paddedkey);
        uint32_t rest = datalen & (block_len - 1);
        uint8_t* ec = (uint8_t*) &enc_counter;
        for (uint32_t j = 0; j < rest; j++) {
            data[j] ^= ec[j];
        }
    }

    Result Encrypt(Algorithm algo, uint8_t* data, uint32_t datalen, const uint8_t* key, uint32_t keylen)
    {
        EncryptXTeaCTR(data, datalen, key, keylen);
        return RESULT_OK;
    }

    Result Decrypt(Algorithm algo, uint8_t* data, uint32_t datalen, const uint8_t* key, uint32_t keylen)
    {
        EncryptXTeaCTR(data, datalen, key, keylen);
        return RESULT_OK;
    }

    // Same as rsa_alt_decrypt_wrap() except with a MBEDTLS_RSA_PUBLIC
    static int rsa_alt_decrypt_public_wrap( void *ctx,
                        const unsigned char *input, size_t ilen,
                        unsigned char *output, size_t *olen, size_t osize,
                        int (*f_rng)(void *, unsigned char *, size_t), void *p_rng )
    {
        mbedtls_rsa_context * rsa = (mbedtls_rsa_context *) ctx;

        if( ilen != mbedtls_rsa_get_len( rsa ) )
            return( MBEDTLS_ERR_RSA_BAD_INPUT_DATA );

        return( mbedtls_rsa_pkcs1_decrypt( rsa, f_rng, p_rng,
                    MBEDTLS_RSA_PUBLIC, olen, input, output, osize ) );
    }

    Result Decrypt(const uint8_t* key, uint32_t keylen, const uint8_t* data, uint32_t datalen, uint8_t** output, uint32_t* outputlen)
    {
        // https://tls.mbed.org/discussions/generic/parsing-public-key-from-memory
        Result result = RESULT_OK;

        const char* pers = "defold_pk_decrypt";
        mbedtls_pk_context pk;
        mbedtls_entropy_context entropy;
        mbedtls_ctr_drbg_context ctr_drbg;
        mbedtls_pk_init(&pk);
        mbedtls_ctr_drbg_init( &ctr_drbg );
        mbedtls_entropy_init( &entropy );

        uint32_t signature_hash_len = MBEDTLS_MD_MAX_SIZE;

        int ret;
        if( ( ret = mbedtls_ctr_drbg_seed( &ctr_drbg, mbedtls_entropy_func, &entropy, (const unsigned char *) pers, strlen(pers) ) ) != 0 )
        {
            dmLogError("Decrypt: mbedtls_ctr_drbg_seed failed: %d", ret);
            result = RESULT_ERROR;
            goto exit;
        }

        if ((ret = mbedtls_pk_parse_public_key(&pk, key, keylen) != 0))
        {
            dmLogError("Decrypt: mbedtls_pk_parse_public_key failed: %d", ret);
            result = RESULT_ERROR;
            goto exit;
        }

        *output = (uint8_t*)malloc(signature_hash_len);
        size_t _outputlen;
        if ((ret = rsa_alt_decrypt_public_wrap(pk.pk_ctx,
                    data, datalen,
                    (uint8_t*)*output, &_outputlen, signature_hash_len,
                    mbedtls_ctr_drbg_random, &ctr_drbg )) != 0)
        {
            dmLogError("Decrypt: rsa_alt_decrypt_public_wrap failed: %d", ret);
            free(*output);
            result = RESULT_ERROR;
            goto exit;
        }

        *outputlen = (uint32_t)_outputlen;

    exit:
        mbedtls_ctr_drbg_free( &ctr_drbg );
        mbedtls_entropy_free( &entropy );
        mbedtls_pk_free(&pk);
        return result;
    }

    void HashSha1(const uint8_t* buf, uint32_t buflen, uint8_t* digest)
    {
        mbedtls_sha1_context ctx;
        mbedtls_sha1_init(&ctx);
        mbedtls_sha1_starts_ret(&ctx);
        mbedtls_sha1_update_ret(&ctx, (const unsigned char*)buf, (size_t)buflen);
        int ret = mbedtls_sha1_finish_ret(&ctx, (unsigned char*)digest);
        mbedtls_sha1_free(&ctx);
        if (ret != 0) {
            memset(digest, 0, 20);
        }
    }

    void HashSha256(const uint8_t* buf, uint32_t buflen, uint8_t* digest)
    {
        int ret = mbedtls_sha256_ret((const unsigned char*)buf, (size_t)buflen, (unsigned char*)digest, 0);
        if (ret != 0) {
            memset(digest, 0, 20);
        }
    }

    void HashSha512(const uint8_t* buf, uint32_t buflen, uint8_t* digest)
    {
        int ret = mbedtls_sha512_ret((const unsigned char*)buf, (size_t)buflen, (unsigned char*)digest, 0);
        if (ret != 0) {
            memset(digest, 0, 20);
        }
    }

    void HashMd5(const uint8_t* buf, uint32_t buflen, uint8_t* digest)
    {
        int ret = mbedtls_md5_ret((const unsigned char*)buf, (size_t)buflen, (unsigned char*)digest);
        if (ret != 0) {
            memset(digest, 0, 20);
        }
    }

    bool Base64Encode(const uint8_t* src, uint32_t src_len, uint8_t* dst, uint32_t* dst_len)
    {
        size_t out_len = 0;
        int r = mbedtls_base64_encode(dst, *dst_len, &out_len, src, src_len);
        if (r != 0)
        {
            if (*dst_len == 0)
                *dst_len = (uint32_t)out_len; // Seems to return 1 more than necessary, but better to err on the safe side! (see test_crypt.cpp)
            else
                *dst_len = 0xFFFFFFFF;
            return false;
        }
        *dst_len = (uint32_t)out_len;
        return true;
    }

    bool Base64Decode(const uint8_t* src, uint32_t src_len, uint8_t* dst, uint32_t* dst_len)
    {
        size_t out_len = 0;
        int r = mbedtls_base64_decode(dst, *dst_len, &out_len, src, src_len);
        if (r != 0)
        {
            if (*dst_len == 0)
                *dst_len = (uint32_t)out_len;
            else
                *dst_len = 0xFFFFFFFF;
            return false;
        }
        *dst_len = (uint32_t)out_len;
        return true;
    }

    unsigned char * RS256SignKey( unsigned char * signing_content, unsigned char * private_key )
    {
        int ret = 1;
        mbedtls_pk_context pk;
        mbedtls_entropy_context entropy;
        mbedtls_ctr_drbg_context ctr_drbg;
        
        mbedtls_pk_init( &pk );
        mbedtls_entropy_init( &entropy );
        mbedtls_ctr_drbg_init( &ctr_drbg );

        unsigned char hash[32];
        unsigned char buf[MBEDTLS_MPI_MAX_SIZE]; // As [1024]
        
        size_t  olen = 0, 
                dlen = 344+1, 
                buflen = 256;
        unsigned char dst[344+1]; 

        const char * pers = "rsa_sign_pss";
        size_t pkey_len = strlen((char*)private_key)+1;
        /* 
         * Key Sign need a Random Generator Function, so here's one : 
         */
        if(( ret = mbedtls_ctr_drbg_seed( &ctr_drbg,  mbedtls_entropy_func,  &entropy, (const unsigned char *) pers, strlen( pers ))) != 0 )
        {
             printf( "\ncrypt -- error: mbedtls_ctr_drbg_seed returned %d", ret ); // goto exit;
        }
        /* 
         * Parse private key, wonder why pkey_len had to +1 ? 
         */
        if(( ret = mbedtls_pk_parse_key( &pk, private_key, pkey_len, NULL, NULL)) != 0)
        {
            printf( "  ! mbedtls_pk_parse_public_keyfile returned %d", ret );      // goto exit;
        }
        /* 
         * Check for valid RSA key. 
         */
        if( !mbedtls_pk_can_do( &pk, MBEDTLS_PK_RSA )) 
        {
            printf( "\ncrypt -- error: Key is not an RSA key: %s\n", private_key); // goto exit;
        }
        /* 
         * Important: MBEDTLS_RSA_PKCS_V21 won't work for Google OAuth v2, but V15. 
         */
        mbedtls_rsa_set_padding( mbedtls_pk_rsa(pk), MBEDTLS_RSA_PKCS_V15, MBEDTLS_MD_SHA256 );
        /*
         * Compute the SHA-256 hash of the input file.
         */
        if(( ret = mbedtls_md(mbedtls_md_info_from_type(MBEDTLS_MD_SHA256), signing_content, strlen((char*)signing_content), hash)) != 0 )
        {
            printf( "\ncrypt -- mbedtls_md_info_from_type\n" );                    // goto exit;
        }
        /*
         * then calculate the RSA signature of the hash.
         */
        if(( ret = mbedtls_pk_sign(&pk, MBEDTLS_MD_SHA256, hash, 0, buf, &olen, mbedtls_ctr_drbg_random, &ctr_drbg)) != 0 )
        {
            printf( "\ncrypt -- error: mbedtls_pk_sign returned %d\n", ret );      // goto exit;
        }
        /* 
         * encode given signature > base64
         */
        mbedtls_base64_encode(dst, dlen, &olen, buf, buflen);

    exit:
        /*
         * free resources.
         */
        mbedtls_ctr_drbg_free( &ctr_drbg );
        mbedtls_entropy_free( &entropy );
        mbedtls_pk_free(&pk);
        
        // return buf;
        return dst;
    }
}

extern "C" {
    DM_DLLEXPORT int EncryptXTeaCTR(uint8_t* data, uint32_t datalen, const uint8_t* key, uint32_t keylen)
    {
        return dmCrypt::Encrypt(dmCrypt::ALGORITHM_XTEA, data, datalen, key, keylen);
    }

    DM_DLLEXPORT int DecryptXTeaCTR(uint8_t* data, uint32_t datalen, const uint8_t* key, uint32_t keylen)
    {
        return dmCrypt::Decrypt(dmCrypt::ALGORITHM_XTEA, data, datalen, key, keylen);
    }

}
