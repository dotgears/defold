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

#ifndef DM_IMAGE_H
#define DM_IMAGE_H

#include <stdint.h>

namespace dmImage
{
    enum Result
    {
        RESULT_OK                   = 0,
        RESULT_UNSUPPORTED_FORMAT   = -1,
        RESULT_IMAGE_ERROR          = -2,
    };

    enum Type
    {
        TYPE_RGB        = 0,
        TYPE_RGBA       = 1,
        TYPE_LUMINANCE  = 2,
    };

    struct Image
    {
        Image() : m_Width(0), m_Height(0), m_Type(TYPE_RGB), m_Buffer(0) {}
        uint32_t m_Width;
        uint32_t m_Height;
        Type     m_Type;
        void*    m_Buffer;
    };

    /**
     * Load image from buffer.
     *
     * <pre>
     *   Supported formats:
     *   png gray, gray + alpha, rgb and rgba
     *   jpg
     * </pre>
     * 16-bit (or higher) channels are not supported.
     *
     * @param buffer image buffer
     * @param buffer_size image buffer size
     * @param premult premultiply alpha or not
     * @param image output
     * @return RESULT_OK on success
     */
    Result Load(const void* buffer, uint32_t buffer_size, bool premult, Image* image);

    /**
     * Free loaded image
     * @param image image to free
     */
    void Free(Image* image);

    /**
     * Get bytes per pixel
     * @param type
     * @return bytes per pixel. zero if the type is unknown
     */
    uint32_t BytesPerPixel(Type type);

}

#endif // #ifndef DM_IMAGE_H
