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

#include <stdint.h>
#include <stdlib.h>
#define JC_TEST_IMPLEMENTATION
#include <jc_test/jc_test.h>
#include "../liveupdate.h"
#include "../liveupdate_private.h"

TEST(dmLiveUpdate, HexDigestLength)
{
    uint32_t actual = 0;

    actual = dmLiveUpdate::HexDigestLength(dmLiveUpdateDDF::HASH_MD5);
    ASSERT_EQ((uint32_t)(128 / 8 * 2), actual);

    actual = dmLiveUpdate::HexDigestLength(dmLiveUpdateDDF::HASH_SHA1);
    ASSERT_EQ((uint32_t)(160 / 8 * 2), actual);

    actual = dmLiveUpdate::HexDigestLength(dmLiveUpdateDDF::HASH_SHA256);
    ASSERT_EQ((uint32_t)(256 / 8 * 2), actual);

    actual = dmLiveUpdate::HexDigestLength(dmLiveUpdateDDF::HASH_SHA512);
    ASSERT_EQ((uint32_t)(512 / 8 * 2), actual);
}

TEST(dmLiveUpdate, BytesToHexString)
{
    uint8_t instance[16] = { 0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};

    char buffer_short[6];
    dmResource::BytesToHexString(instance, dmResource::HashLength(dmLiveUpdateDDF::HASH_MD5), buffer_short, 6);
    ASSERT_STREQ("00010", buffer_short);

    char buffer_fitted[33];
    dmResource::BytesToHexString(instance, dmResource::HashLength(dmLiveUpdateDDF::HASH_MD5), buffer_fitted, 33);
    ASSERT_STREQ("000102030405060708090a0b0c0d0e0f", buffer_fitted);

    char buffer_long[513];
    dmResource::BytesToHexString(instance, dmResource::HashLength(dmLiveUpdateDDF::HASH_MD5), buffer_long, 513);
    ASSERT_STREQ("000102030405060708090a0b0c0d0e0f", buffer_long);
}

int main(int argc, char **argv)
{
    jc_test_init(&argc, argv);
    int ret = jc_test_run_all();
    return ret;
}
