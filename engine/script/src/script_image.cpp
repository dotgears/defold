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

#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>

#include <dlib/log.h>
#include <dlib/image.h>
#include "script.h"

extern "C"
{
#include <lua/lua.h>
#include <lua/lauxlib.h>
}

#include "script_private.h"

namespace dmScript
{
    /*# Image API documentation
     *
     * Functions for creating image objects.
     *
     * @document
     * @name Image
     * @namespace image
     */

    #define LIB_NAME "image"

    /*# RGB image type
     *
     * @name image.TYPE_RGB
     * @variable
     */

    /*# RGBA image type
     *
     * @name image.TYPE_RGBA
     * @variable
     */

    /*# luminance image type
     *
     * @name image.TYPE_LUMINANCE
     * @variable
     */

    /*# load image from buffer
    * Load image (PNG or JPEG) from buffer.
    *
    * @name image.load
    * @param buffer [type:string] image data buffer
    * @param [premult] [type:boolean] optional flag if alpha should be premultiplied. Defaults to `false`
    * @return image [type:table] object or `nil` if loading fails. The object is a table with the following fields:
    *
    * - [type:number] `width`: image width
    * - [type:number] `height`: image height
    * - [type:constant] `type`: image type
    *     - `image.TYPE_RGB`
    *     - `image.TYPE_RGBA`
    *     - `image.TYPE_LUMINANCE`
    * - [type:string] `buffer`: the raw image data
    *
    * @examples
    *
    * How to load an image from an URL and create a GUI texture from it:
    *
    * ```lua
    * local imgurl = "http://www.site.com/image.png"
    * http.request(imgurl, "GET", function(self, id, response)
    *         local img = image.load(response.response)
    *         local tx = gui.new_texture("image_node", img.width, img.height, img.type, img.buffer)
    *     end)
    * ```
    */
    int Image_Load(lua_State* L)
    {
        int top = lua_gettop(L);
        luaL_checktype(L, 1, LUA_TSTRING);
        size_t buffer_len = 0;
        const char* buffer = lua_tolstring(L, 1, &buffer_len);

        bool premult = false;
        if (top == 2) {
            premult = lua_toboolean(L, 2);
        }

        dmImage::Image image;
        dmImage::Result r = dmImage::Load(buffer, buffer_len, premult, &image);
        if (r == dmImage::RESULT_OK) {

            int bytes_per_pixel = dmImage::BytesPerPixel(image.m_Type);
            if (bytes_per_pixel == 0) {
                dmImage::Free(&image);
                luaL_error(L, "unknown image type %d", image.m_Type);
            }

            lua_newtable(L);

            lua_pushliteral(L, "width");
            lua_pushinteger(L, image.m_Width);
            lua_rawset(L, -3);

            lua_pushliteral(L, "height");
            lua_pushinteger(L, image.m_Height);
            lua_rawset(L, -3);

            lua_pushliteral(L, "type");
            switch (image.m_Type) {
                case dmImage::TYPE_RGB:
                    lua_pushliteral(L, "rgb");
                    break;
                case dmImage::TYPE_RGBA:
                    lua_pushliteral(L, "rgba");
                    break;
                case dmImage::TYPE_LUMINANCE:
                    lua_pushliteral(L, "l");
                    break;
                default:
                    assert(false);
            }
            lua_rawset(L, -3);

            lua_pushliteral(L, "buffer");
            lua_pushlstring(L, (const char*) image.m_Buffer, bytes_per_pixel * image.m_Width * image.m_Height);
            lua_rawset(L, -3);

            dmImage::Free(&image);

        } else {
            dmLogWarning("failed to load image (%d)", r);
            lua_pushnil(L);
        }

        assert(top + 1 == lua_gettop(L));
        return 1;
    }

    static const luaL_reg ScriptImage_methods[] =
    {
        {"load", Image_Load},
        {0, 0}
    };

    void InitializeImage(lua_State* L)
    {
        int top = lua_gettop(L);

        luaL_register(L, LIB_NAME, ScriptImage_methods);

#define SETCONSTANT(name, val) \
        lua_pushliteral(L, val); \
        lua_setfield(L, -2, #name);\

        SETCONSTANT(TYPE_RGB, "rgb")
        SETCONSTANT(TYPE_RGBA, "rgba")
        SETCONSTANT(TYPE_LUMINANCE, "l")

#undef SETCONSTANT


        lua_pop(L, 1);

        assert(top == lua_gettop(L));
    }
}
