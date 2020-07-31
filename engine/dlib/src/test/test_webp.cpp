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
#define JC_TEST_IMPLEMENTATION
#include <jc_test/jc_test.h>
#include "../dlib/webp.h"

/*
 * Tests based on libwebp 0.5.0. Webp image files created with the cwebp tool
 * RGB images are 16x3 Line1=R255 Line2=G255 Line3=B255
 * RGBA images are equal with A127
 */

extern unsigned char WEBP_RGB_LOSSLESS_WEBP[];
extern uint32_t WEBP_RGB_LOSSLESS_WEBP_SIZE;
extern unsigned char WEBP_RGBA_LOSSLESS_WEBP[];
extern uint32_t WEBP_RGBA_LOSSLESS_WEBP_SIZE;
extern unsigned char WEBP_RGB_LOSSY_WEBP[];
extern uint32_t WEBP_RGB_LOSSY_WEBP_SIZE;
extern unsigned char WEBP_RGBA_LOSSY_WEBP[];
extern uint32_t WEBP_RGBA_LOSSY_WEBP_SIZE;

static const uint32_t image_width = 16;

static bool CheckLineRGB(uint8_t* buffer, uint32_t line, uint8_t r, uint8_t g, uint8_t b)
{
    buffer += image_width*3*line;
    for(uint32_t i = 0; i < image_width; ++i)
    {
        if(*buffer++ != r)
            return false;
        if(*buffer++ != g)
            return false;
        if(*buffer++ != b)
            return false;
    }
    return true;
}

static bool CheckLineRGBA(uint8_t* buffer, uint32_t line, uint8_t r, uint8_t g, uint8_t b, uint8_t a)
{
    buffer += image_width*4*line;
    for(uint32_t i = 0; i < image_width; ++i)
    {
        if(*buffer++ != r)
            return false;
        if(*buffer++ != g)
            return false;
        if(*buffer++ != b)
            return false;
        if(*buffer++ != a)
            return false;
    }
    return true;
}

TEST(dmWebP, DecompressLossless)
{
    dmWebP::Result r;
    uint8_t buffer[image_width*4*3];

    r = dmWebP::DecodeRGB(WEBP_RGB_LOSSLESS_WEBP, WEBP_RGB_LOSSLESS_WEBP_SIZE, &buffer, image_width*3*3, image_width*3);
    ASSERT_EQ(dmWebP::RESULT_OK, r);
    ASSERT_TRUE(CheckLineRGB(buffer, 0, 255, 0, 0));
    ASSERT_TRUE(CheckLineRGB(buffer, 1, 0, 255, 0));
    ASSERT_TRUE(CheckLineRGB(buffer, 2, 0, 0, 255));

    r = dmWebP::DecodeRGBA(WEBP_RGBA_LOSSLESS_WEBP, WEBP_RGBA_LOSSLESS_WEBP_SIZE, &buffer, image_width*3*4, image_width*4);
    ASSERT_EQ(dmWebP::RESULT_OK, r);
    ASSERT_TRUE(CheckLineRGBA(buffer, 0, 255, 0, 0, 127));
    ASSERT_TRUE(CheckLineRGBA(buffer, 1, 0, 255, 0, 127));
    ASSERT_TRUE(CheckLineRGBA(buffer, 2, 0, 0, 255, 127));
}

TEST(dmWebP, DecompressLossy)
{
    dmWebP::Result r;
    uint8_t buffer[image_width*4*3];
    r = dmWebP::DecodeRGB(WEBP_RGB_LOSSY_WEBP, WEBP_RGB_LOSSY_WEBP_SIZE, &buffer, image_width*3*3, image_width*3);
    ASSERT_EQ(dmWebP::RESULT_OK, r);
    r = dmWebP::DecodeRGBA(WEBP_RGBA_LOSSY_WEBP, WEBP_RGBA_LOSSY_WEBP_SIZE, &buffer, image_width*3*4, image_width*4);
    ASSERT_EQ(dmWebP::RESULT_OK, r);
}

int main(int argc, char **argv)
{
    jc_test_init(&argc, argv);
    return jc_test_run_all();
}
