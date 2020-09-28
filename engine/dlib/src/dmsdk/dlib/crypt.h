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

#ifndef DMSDK_CRYPT_H
#define DMSDK_CRYPT_H

#include <stdint.h>

/*# SDK Crypt API documentation
 * [file:<dmsdk/dlib/crypt.h>]
 *
 * Various hash and encode/decode functions.
 *
 * @document
 * @name Crypt
 * @namespace dmCrypt
 */

namespace dmCrypt
{
    /*# Hash buffer using SHA1
     * @name dmCrypt::HashSha1
     * @param buf       The source data to hash
     * @param buflen    The length of source data in bytes
     * @param digest    The destination buffer (20 bytes)
     */
    void HashSha1(const uint8_t* buf, uint32_t buflen, uint8_t* digest);      // output is 20 bytes

    /*# Hash buffer using SHA256
     * @name dmCrypt::HashSha256
     * @param buf       The source data to hash
     * @param buflen    The length of source data in bytes
     * @param digest    The destination buffer (32 bytes)
     */
    void HashSha256(const uint8_t* buf, uint32_t buflen, uint8_t* digest);    // output is 32 bytes

    /*# Hash buffer using SHA512
     * @name dmCrypt::HashSha512
     * @param buf       The source data to hash
     * @param buflen    The length of source data in bytes
     * @param digest    The destination buffer (64 bytes)
     */
    void HashSha512(const uint8_t* buf, uint32_t buflen, uint8_t* digest);    // output is 64 bytes

    /*# Hash buffer using MD5
     * @name dmCrypt::HashMd5
     * @param buf       The source data to hash
     * @param buflen    The length of source data in bytes
     * @param digest    The destination buffer (16 bytes)
     */
    void HashMd5(const uint8_t* buf, uint32_t buflen, uint8_t* digest);       // output is 16 bytes

    /*# Base64 encode a buffer
     * @name dmCrypt::Base64Encode
     * @param src               The source data to encode
     * @param src_len           The length of source data in bytes
     * @param dst               The destination buffer
     * @param dst_len[in,out]   In: The length of the destination in bytes. Out: The length of the encoded string.
     * @return true if the encoding went ok
     * @note                    Call this function with *dst_len = 0 to obtain the required buffer size in *dst_len
     */
    bool Base64Encode(const uint8_t* src, uint32_t src_len, uint8_t* dst, uint32_t* dst_len);


    /*# Base64 decode a buffer
     * @name dmCrypt::Base64Decode
     * @param src               The source data to encode
     * @param src_len           The length of source data in bytes
     * @param dst               The destination buffer
     * @param dst_len[in,out]   In: The length of the destination in bytes. Out: The length of the decoded string.
     * @return true if the decoding went ok
     * @note                    Call this function with *dst_len = 0 to obtain the required buffer size in *dst_len
     */
    bool Base64Decode(const uint8_t* src, uint32_t src_len, uint8_t* dst, uint32_t* dst_len);
}

#endif /* DMSDK_CRYPT_H */
