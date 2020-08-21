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

#include "glsl_uniform_parser.h"

#include <string.h>

namespace dmGraphics
{
    static bool IsWS(const char c)
    {
        return c == ' ' || c == '\n' || c == '\t' || c == '\r';
    }

    static const char* SkipLine(const char* cursor)
    {
        while (*cursor != '\n' && *cursor != '\0')
        {
            ++cursor;
        }
        return cursor;
    }

    static const char* SkipWS(const char* cursor)
    {
        while (IsWS(*cursor) && *cursor != '\0')
        {
            ++cursor;
        }
        return cursor;
    }

    static const char* FindWS(const char* cursor)
    {
        while (!IsWS(*cursor) && *cursor != '\0')
        {
            ++cursor;
        }
        return cursor;
    }

#define STRNCMP(s1, s2, count)\
    strncmp(s1, s2, strnlen(s1, 64)) == 0

    static bool ParseType(const char* string, uint32_t count, Type* out_type)
    {
        if (STRNCMP("int", string, count))
        {
            *out_type = TYPE_INT;
            return true;
        }
        else if (STRNCMP("uint", string, count))
        {
            *out_type = TYPE_UNSIGNED_INT;
            return true;
        }
        else if (STRNCMP("float", string, count))
        {
            *out_type = TYPE_FLOAT;
            return true;
        }
        else if (STRNCMP("vec4", string, count))
        {
            *out_type = TYPE_FLOAT_VEC4;
            return true;
        }
        else if (STRNCMP("mat4", string, count))
        {
            *out_type = TYPE_FLOAT_MAT4;
            return true;
        }
        else if (STRNCMP("sampler2D", string, count))
        {
            *out_type = TYPE_SAMPLER_2D;
            return true;
        }
        else if (STRNCMP("samplerCube", string, count))
        {
            *out_type = TYPE_SAMPLER_CUBE;
            return true;
        }
        return false;
    }

    static void NextWord(const char** word_start, const char** word_end, uint32_t* size)
    {
        *word_start = SkipWS(*word_end);
        *word_end = FindWS(*word_start);
        *size = (uint32_t) (*word_end - *word_start);
    }

    bool GLSLUniformParse(const char* buffer, UniformCallback cb, uintptr_t userdata)
    {
        if (buffer == 0x0)
            return true;
        const char* word_end = buffer;
        const char* word_start = buffer;
        uint32_t size = 0;
        while (*word_end != '\0')
        {
            NextWord(&word_start, &word_end, &size);

            if (size > 0)
            {
                if (STRNCMP("uniform", word_start, size))
                {
                    Type type;

                    NextWord(&word_start, &word_end, &size);

                    bool found_type = ParseType(word_start, size, &type);
                    // Assume precision, look for type again
                    if (!found_type)
                    {
                        NextWord(&word_start, &word_end, &size);
                        found_type = ParseType(word_start, size, &type);
                    }

                    if (found_type)
                    {
                        // Check name
                        NextWord(&word_start, &word_end, &size);
                        cb(word_start, size-1, type, userdata);
                    }
                    else
                    {
                        return false;
                    }
                }
                else
                {
                    word_start = SkipWS(SkipLine(word_end));
                    word_end = word_start;
                }
            }
        }
        return true;
    }

#undef STRNCMP

}
