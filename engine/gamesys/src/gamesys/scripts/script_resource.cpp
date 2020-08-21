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

#include "script_resource.h"
#include <dlib/buffer.h>
#include <dlib/log.h>
#include <dlib/dstrings.h>
#include <resource/resource.h>
#include <graphics/graphics_ddf.h>
#include "mesh_ddf.h"
#include <script/script.h>
#include <liveupdate/liveupdate.h>
#include "../gamesys.h"
#include "script_resource_liveupdate.h"
#include "../resources/res_buffer.h"


namespace dmGameSystem
{

/*# Resource API documentation
 *
 * Functions and constants to access resources.
 *
 * @document
 * @name Resource
 * @namespace resource
 */

/*# reference to material resource
 *
 * Constructor-like function with two purposes:
 *
 * - Load the specified resource as part of loading the script
 * - Return a hash to the run-time version of the resource
 *
 * [icon:attention] This function can only be called within [ref:go.property] function calls.
 *
 * @name resource.material
 * @return path [type:hash] a path hash to the binary version of the resource
 * @examples
 *
 * Load a material and set it to a sprite:
 *
 * ```lua
 * go.property("my_material", resource.material("/material.material"))
 * function init(self)
 *   go.set("#sprite", "material", self.my_material)
 * end
 * ```
 */

/*# reference to font resource
 *
 * Constructor-like function with two purposes:
 *
 * - Load the specified resource as part of loading the script
 * - Return a hash to the run-time version of the resource
 *
 * [icon:attention] This function can only be called within [ref:go.property] function calls.
 *
 * @name resource.font
 * @return path [type:hash] a path hash to the binary version of the resource
 * @examples
 *
 * Load a font and set it to a label:
 *
 * ```lua
 * go.property("my_font", resource.font("/font.font"))
 * function init(self)
 *   go.set("#label", "font", self.my_font)
 * end
 * ```
 */

/*# reference to texture resource
 *
 * Constructor-like function with two purposes:
 *
 * - Load the specified resource as part of loading the script
 * - Return a hash to the run-time version of the resource
 *
 * [icon:attention] This function can only be called within [ref:go.property] function calls.
 *
 * @name resource.texture
 * @return path [type:hash] a path hash to the binary version of the resource
 * @examples
 *
 * Load a texture and set it to a model:
 *
 * ```lua
 * go.property("my_texture", resource.texture("/texture.png"))
 * function init(self)
 *   go.set("#model", "texture0", self.my_texture)
 * end
 * ```
 */

/*# reference to atlas resource
 *
 * Constructor-like function with two purposes:
 *
 * - Load the specified resource as part of loading the script
 * - Return a hash to the run-time version of the resource
 *
 * [icon:attention] This function can only be called within [ref:go.property] function calls.
 *
 * @name resource.atlas
 * @return path [type:hash] a path hash to the binary version of the resource
 * @examples
 *
 * Load an atlas and set it to a sprite:
 *
 * ```lua
 * go.property("my_atlas", resource.atlas("/atlas.atlas"))
 * function init(self)
 *   go.set("#sprite", "image", self.my_atlas)
 * end
 * ```
 */

/*# reference to buffer resource
 *
 * Constructor-like function with two purposes:
 *
 * - Load the specified resource as part of loading the script
 * - Return a hash to the run-time version of the resource
 *
 * [icon:attention] This function can only be called within [ref:go.property] function calls.
 *
 * @name resource.buffer
 * @return path [type:hash] a path hash to the binary version of the resource
 * @examples
 *
 * Set a unique buffer it to a sprite:
 *
 * ```lua
 * go.property("my_buffer", resource.buffer("/cube.buffer"))
 * function init(self)
 *   go.set("#mesh", "vertices", self.my_buffer)
 * end
 * ```
 */

/*# reference to tile source resource
 *
 * Constructor-like function with two purposes:
 *
 * - Load the specified resource as part of loading the script
 * - Return a hash to the run-time version of the resource
 *
 * [icon:attention] This function can only be called within [ref:go.property] function calls.
 *
 * @name resource.tile_source
 * @return path [type:hash] a path hash to the binary version of the resource
 * @examples
 *
 * Load tile source and set it to a tile map:
 *
 * ```lua
 * go.property("my_tile_source", resource.tile_source("/tilesource.tilesource"))
 * function init(self)
 *   go.set("#tilemap", "tile_source", self.my_tile_source)
 * end
 * ```
 */

struct ResourceModule
{
    dmResource::HFactory m_Factory;
} g_ResourceModule;


static int ReportPathError(lua_State* L, dmResource::Result result, dmhash_t path_hash)
{
    char msg[256];
    const char* format = 0;
    switch(result)
    {
    case dmResource::RESULT_RESOURCE_NOT_FOUND: format = "The resource was not found (%d): %llu, %s"; break;
    case dmResource::RESULT_NOT_SUPPORTED:      format = "The resource type does not support this operation (%d): %llu, %s"; break;
    default:                                    format = "The resource was not updated (%d): %llu, %s"; break;
    }
    dmSnPrintf(msg, sizeof(msg), format, result, (unsigned long long)path_hash, dmHashReverseSafe64(path_hash));
    return luaL_error(L, "%s", msg);
}

/*# Set a resource
 * Sets the resource data for a specific resource
 *
 * @name resource.set
 *
 * @param path [type:string|hash] The path to the resource
 * @param buffer [type:buffer] The buffer of precreated data, suitable for the intended resource type
 *
 * @examples
 *
 * Assuming the folder "/res" is added to the project custom resources:
 *
 * ```lua
 * -- load a texture resource and set it on a sprite
 * local buffer = resource.load("/res/new.texturec")
 * resource.set(go.get("#sprite", "texture0"), buffer)
 * ```
 */
static int Set(lua_State* L)
{
    int top = lua_gettop(L);

    dmhash_t path_hash = dmScript::CheckHashOrString(L, 1);
    dmScript::LuaHBuffer* buffer = dmScript::CheckBuffer(L, 2);

    uint32_t datasize = 0;
    void* data = 0;
    dmBuffer::GetBytes(buffer->m_Buffer, &data, &datasize);

    dmResource::Result r = dmResource::SetResource(g_ResourceModule.m_Factory, path_hash, data, datasize);
    if( r != dmResource::RESULT_OK )
    {
        assert(top == lua_gettop(L));
        return ReportPathError(L, r, path_hash);
    }
    assert(top == lua_gettop(L));
    return 0;
}

/*# load a resource
 * Loads the resource data for a specific resource.
 *
 * @name resource.load
 *
 * @param path [type:string] The path to the resource
 * @return buffer [type:buffer] Returns the buffer stored on disc
 *
 * @examples
 *
 * ```lua
 * -- read custom resource data into buffer
 * local buffer = resource.load("/resources/datafile")
 * ```
 *
 * In order for the engine to include custom resources in the build process, you need
 * to specify them in the "game.project" settings file:
 *
 * ```ini
 * [project]
 * title = My project
 * version = 0.1
 * custom_resources = resources/,assets/level_data.json
 * ```
 */
static int Load(lua_State* L)
{
    int top = lua_gettop(L);
    const char* name = luaL_checkstring(L, 1);

    void* resource = 0;
    uint32_t resourcesize = 0;
    dmResource::Result r = dmResource::GetRaw(g_ResourceModule.m_Factory, name, &resource, &resourcesize);

    if( r != dmResource::RESULT_OK )
    {
        assert(top == lua_gettop(L));
        return ReportPathError(L, r, dmHashString64(name));
    }

    dmBuffer::StreamDeclaration streams_decl[] = {
        {dmHashString64("data"), dmBuffer::VALUE_TYPE_UINT8, 1}
    };

    dmBuffer::HBuffer buffer = 0;
    dmBuffer::Create(resourcesize, streams_decl, 1, &buffer);

    uint8_t* data = 0;
    uint32_t datasize = 0;
    dmBuffer::GetBytes(buffer, (void**)&data, &datasize);

    memcpy(data, resource, resourcesize);

    dmScript::LuaHBuffer luabuf = {buffer, dmScript::OWNER_LUA};
    dmScript::PushBuffer(L, luabuf);
    assert(top + 1 == lua_gettop(L));
    return 1;
}

static int CheckTableNumber(lua_State* L, int index, const char* name)
{
    int result = -1;
    lua_pushstring(L, name);
    lua_gettable(L, index);
    if (lua_isnumber(L, -1)) {
        result = lua_tointeger(L, -1);
    } else {
        char msg[256];
        dmSnPrintf(msg, sizeof(msg), "Wrong type for table attribute '%s'. Expected number, got %s", name, luaL_typename(L, -1) );
        return luaL_error(L, "%s", msg);
    }
    lua_pop(L, 1);
    return result;
}

static int GraphicsTextureFormatToImageFormat(int textureformat)
{
    switch(textureformat)
    {
        case dmGraphics::TEXTURE_FORMAT_LUMINANCE:          return dmGraphics::TextureImage::TEXTURE_FORMAT_LUMINANCE;
        case dmGraphics::TEXTURE_FORMAT_RGB:                return dmGraphics::TextureImage::TEXTURE_FORMAT_RGB;
        case dmGraphics::TEXTURE_FORMAT_RGBA:               return dmGraphics::TextureImage::TEXTURE_FORMAT_RGBA;
        case dmGraphics::TEXTURE_FORMAT_RGB_PVRTC_2BPPV1:   return dmGraphics::TextureImage::TEXTURE_FORMAT_RGB_PVRTC_2BPPV1;
        case dmGraphics::TEXTURE_FORMAT_RGB_PVRTC_4BPPV1:   return dmGraphics::TextureImage::TEXTURE_FORMAT_RGB_PVRTC_4BPPV1;
        case dmGraphics::TEXTURE_FORMAT_RGBA_PVRTC_2BPPV1:  return dmGraphics::TextureImage::TEXTURE_FORMAT_RGBA_PVRTC_2BPPV1;
        case dmGraphics::TEXTURE_FORMAT_RGBA_PVRTC_4BPPV1:  return dmGraphics::TextureImage::TEXTURE_FORMAT_RGBA_PVRTC_4BPPV1;
        case dmGraphics::TEXTURE_FORMAT_RGB_ETC1:           return dmGraphics::TextureImage::TEXTURE_FORMAT_RGB_ETC1;
    };
    assert(false);
    return -1;
}

static int GraphicsTextureTypeToImageType(int texturetype)
{
    if (texturetype == dmGraphics::TEXTURE_TYPE_2D)
    {
        return dmGraphics::TextureImage::TYPE_2D;
    }
    else if (texturetype == dmGraphics::TEXTURE_TYPE_CUBE_MAP)
    {
        return dmGraphics::TextureImage::TYPE_CUBEMAP;
    }
    assert(false);
    return -1;
}


/*# set a texture
 * Sets the pixel data for a specific texture.
 *
 * @name resource.set_texture
 *
 * @param path [type:hash|string] The path to the resource
 * @param table [type:table] A table containing info about the texture. Supported entries:
 *
 * `type`
 * : [type:number] The texture type. Supported values:
 *
 * - `resource.TEXTURE_TYPE_2D`
 *
 * `width`
 * : [type:number] The width of the texture (in pixels)
 *
 * `height`
 * : [type:number] The width of the texture (in pixels)
 *
 * `format`
 * : [type:number] The texture format. Supported values:
 *
 * - `resource.TEXTURE_FORMAT_LUMINANCE`
 * - `resource.TEXTURE_FORMAT_RGB`
 * - `resource.TEXTURE_FORMAT_RGBA`
 *
 * @param buffer [type:buffer] The buffer of precreated pixel data
 *
 * [icon:attention] Currently, only 1 mipmap is generated.
 *
 * @examples
 * How to set all pixels of an atlas
 *
 * ```lua
 * function init(self)
 *   self.height = 128
 *   self.width = 128
 *   self.buffer = buffer.create(self.width * self.height, { {name=hash("rgb"), type=buffer.VALUE_TYPE_UINT8, count=3} } )
 *   self.stream = buffer.get_stream(self.buffer, hash("rgb"))
 *
 *   for y=1,self.height do
 *       for x=1,self.width do
 *           local index = (y-1) * self.width * 3 + (x-1) * 3 + 1
 *           self.stream[index + 0] = 0xff
 *           self.stream[index + 1] = 0x80
 *           self.stream[index + 2] = 0x10
 *       end
 *   end

 *   local resource_path = go.get("#sprite", "texture0")
 *   local header = { width=self.width, height=self.height, type=resource.TEXTURE_TYPE_2D, format=resource.TEXTURE_FORMAT_RGB, num_mip_maps=1 }
 *   resource.set_texture( resource_path, header, self.buffer )
 * end
 * ```
 */
static int SetTexture(lua_State* L)
{
    int top = lua_gettop(L);

    dmhash_t path_hash = dmScript::CheckHashOrString(L, 1);

    luaL_checktype(L, 2, LUA_TTABLE);
    uint32_t type = (uint32_t)CheckTableNumber(L, 2, "type");
    uint32_t width = (uint32_t)CheckTableNumber(L, 2, "width");
    uint32_t height = (uint32_t)CheckTableNumber(L, 2, "height");
    uint32_t format = (uint32_t)CheckTableNumber(L, 2, "format");

    uint32_t num_mip_maps = 1;

    dmScript::LuaHBuffer* buffer = dmScript::CheckBuffer(L, 3);

    uint8_t* data = 0;
    uint32_t datasize = 0;
    dmBuffer::GetBytes(buffer->m_Buffer, (void**)&data, &datasize);

    dmGraphics::TextureImage* texture_image = new dmGraphics::TextureImage;
    texture_image->m_Alternatives.m_Data = new dmGraphics::TextureImage::Image[1];
    texture_image->m_Alternatives.m_Count = 1;
    texture_image->m_Type = (dmGraphics::TextureImage::Type)GraphicsTextureTypeToImageType(type);

    for (uint32_t i = 0; i < texture_image->m_Alternatives.m_Count; ++i)
    {
        dmGraphics::TextureImage::Image* image = &texture_image->m_Alternatives[i];
        image->m_Width = width;
        image->m_Height = height;
        image->m_OriginalWidth = width;
        image->m_OriginalHeight = height;
        image->m_Format = (dmGraphics::TextureImage::TextureFormat)GraphicsTextureFormatToImageFormat(format);
        image->m_CompressionType = dmGraphics::TextureImage::COMPRESSION_TYPE_DEFAULT;
        image->m_CompressionFlags = 0;
        image->m_Data.m_Data = data;
        image->m_Data.m_Count = datasize;
        image->m_MipMapOffset.m_Data = new uint32_t[num_mip_maps];
        image->m_MipMapOffset.m_Count = num_mip_maps;

        image->m_MipMapSize.m_Data = new uint32_t[num_mip_maps];
        image->m_MipMapSize.m_Count = num_mip_maps;

        for( uint32_t mip = 0; mip < num_mip_maps; ++mip )
        {
            image->m_MipMapOffset[mip] = 0; // TODO: Fix mip offsets
            image->m_MipMapSize[mip] = datasize;
        }
    }

    dmResource::Result r = dmResource::SetResource(g_ResourceModule.m_Factory, path_hash, (void*)texture_image);

    // cleanup memory

    for (uint32_t i = 0; i < texture_image->m_Alternatives.m_Count; ++i)
    {
        dmGraphics::TextureImage::Image* image = &texture_image->m_Alternatives[i];
        delete[] image->m_MipMapSize.m_Data;
        delete[] image->m_MipMapOffset.m_Data;
    }
    delete[] texture_image->m_Alternatives.m_Data;
    delete texture_image;

    if( r != dmResource::RESULT_OK )
    {
        assert(top == lua_gettop(L));
        return ReportPathError(L, r, path_hash);
    }

    assert(top == lua_gettop(L));
    return 0;
}

/*# get resource buffer
 * gets the buffer from a resource
 *
 * @name resource.get_buffer
 *
 * @param path [type:hash|string] The path to the resource
 * @return buffer [type:buffer] The resource buffer
 *
 * @examples
 * How to get the data from a buffer
 *
 * ```lua
 * function init(self)
 *
 *     local res_path = go.get("#mesh", "vertices")
 *     local buf = resource.get_buffer(res_path)
 *     local stream_positions = buffer.get_stream(self.buffer, "position")
 *
 *     for i=1,#stream_positions do
 *         print(i, stream_positions[i])
 *     end
 * end
 * ```
 */
static int GetBuffer(lua_State* L)
{
    int top = lua_gettop(L);
    dmhash_t path_hash = dmScript::CheckHashOrString(L, 1);

    dmResource::SResourceDescriptor* rd = dmResource::FindByHash(g_ResourceModule.m_Factory, path_hash);
    if (!rd) {
        return luaL_error(L, "Could not get buffer resource: %s", dmHashReverseSafe64(path_hash));
    }

    dmResource::ResourceType resource_type;
    dmResource::Result r = dmResource::GetType(g_ResourceModule.m_Factory, rd->m_Resource, &resource_type);
    assert(r == dmResource::RESULT_OK);

    dmResource::ResourceType buffer_resource_type;
    r = dmResource::GetTypeFromExtension(g_ResourceModule.m_Factory, "bufferc", &buffer_resource_type);
    assert(r == dmResource::RESULT_OK);

    if (resource_type != buffer_resource_type) {
        return luaL_error(L, "Resource %s is not of bufferc type.", dmHashReverseSafe64(path_hash));
    }

    dmGameSystem::BufferResource* buffer_resource = (dmGameSystem::BufferResource*)rd->m_Resource;
    dmResource::IncRef(g_ResourceModule.m_Factory, buffer_resource);
    dmScript::LuaHBuffer luabuf;
    luabuf.m_BufferRes = (void*)buffer_resource;
    luabuf.m_Owner = dmScript::OWNER_RES;
    PushBuffer(L, luabuf);

    assert(top + 1 == lua_gettop(L));
    return 1;
}

/*# set resource buffer
 * sets the buffer of a resource
 *
 * @name resource.set_buffer
 *
 * @param path [type:hash|string] The path to the resource
 * @param buffer [type:buffer] The resource buffer
 *
 * @examples
 * How to set the data from a buffer
 *
 * ```lua
 * local function fill_stream(stream, verts)
 *     for key, value in ipairs(verts) do
 *         stream[key] = verts[key]
 *     end
 * end
 *
 * function init(self)
 *
 *     local res_path = go.get("#mesh", "vertices")
 *
 *     local positions = {
 *          1, -1, 0,
 *          1,  1, 0,
 *          -1, -1, 0
 *     }
 *
 *     local num_verts = #positions / 3
 *
 *     -- create a new buffer
 *     local buf = buffer.create(num_verts, {
 *         { name = hash("position"), type=buffer.VALUE_TYPE_FLOAT32, count = 3 }
 *     })
 *
 *     local buf = resource.get_buffer(res_path)
 *     local stream_positions = buffer.get_stream(buf, "position")
 *
 *     fill_stream(stream_positions, positions)
 *
 *     resource.set_buffer(res_path, buf)
 * end
 * ```
 */
static int SetBuffer(lua_State* L)
{
    int top = lua_gettop(L);
    dmhash_t path_hash = dmScript::CheckHashOrString(L, 1);
    dmScript::LuaHBuffer* luabuf = dmScript::CheckBuffer(L, 2);
    dmBuffer::HBuffer src_buffer = luabuf->m_Buffer;
    if (luabuf->m_Owner == dmScript::OWNER_RES) {
        src_buffer = ((BufferResource*)luabuf->m_BufferRes)->m_Buffer;
    }

    dmResource::SResourceDescriptor* rd = dmResource::FindByHash(g_ResourceModule.m_Factory, path_hash);
    if (!rd) {
        return luaL_error(L, "Could not get buffer resource: %s", dmHashReverseSafe64(path_hash));
    }

    dmResource::ResourceType resource_type;
    dmResource::Result r = dmResource::GetType(g_ResourceModule.m_Factory, rd->m_Resource, &resource_type);
    assert(r == dmResource::RESULT_OK);

    dmResource::ResourceType buffer_resource_type;
    r = dmResource::GetTypeFromExtension(g_ResourceModule.m_Factory, "bufferc", &buffer_resource_type);
    assert(r == dmResource::RESULT_OK);

    if (resource_type != buffer_resource_type) {
        return luaL_error(L, "Resource %s is not of bufferc type.", dmHashReverseSafe64(path_hash));
    }

    dmGameSystem::BufferResource* buffer_resource = (dmGameSystem::BufferResource*)rd->m_Resource;
    dmBuffer::HBuffer dst_buffer = buffer_resource->m_Buffer;

    // Make sure the destination buffer has enough size (otherwise, resize it).
    // TODO: Check if incoming buffer size is smaller than current size -> don't allocate new dmbuffer,
    //       but copy smaller data and change "size".
    uint32_t dst_count = 0;
    dmBuffer::Result br = dmBuffer::GetCount(dst_buffer, &dst_count);
    if (br != dmBuffer::RESULT_OK) {
        return luaL_error(L, "Unable to get buffer size for %s: %s (%d).", dmHashReverseSafe64(path_hash), dmBuffer::GetResultString(br), br);
    }
    uint32_t src_count = 0;
    br = dmBuffer::GetCount(src_buffer, &src_count);
    if (br != dmBuffer::RESULT_OK) {
        return luaL_error(L, "Unable to get buffer size for source buffer: %s (%d).", dmBuffer::GetResultString(br), br);
    }

    bool new_buffer_needed = dst_count != src_count;
    if (new_buffer_needed) {
        // Need to create a new buffer to copy data to.

        // Copy stream declaration
        uint32_t stream_count = buffer_resource->m_BufferDDF->m_Streams.m_Count;
        dmBuffer::StreamDeclaration* streams_decl = (dmBuffer::StreamDeclaration*)malloc(stream_count * sizeof(dmBuffer::StreamDeclaration));
        for (uint32_t i = 0; i < stream_count; ++i)
        {
            const dmBufferDDF::StreamDesc& ddf_stream = buffer_resource->m_BufferDDF->m_Streams[i];
            streams_decl[i].m_Name = dmHashString64(ddf_stream.m_Name);
            streams_decl[i].m_Type = (dmBuffer::ValueType)ddf_stream.m_ValueType;
            streams_decl[i].m_Count = ddf_stream.m_ValueCount;
        }

        br = dmBuffer::Create(src_count, streams_decl, stream_count, &dst_buffer);
        free(streams_decl);

        if (br != dmBuffer::RESULT_OK) {
            return luaL_error(L, "Unable to create copy buffer: %s (%d).", dmBuffer::GetResultString(br), br);
        }
    }

    // Copy supplied data to buffer
    br = dmBuffer::Copy(dst_buffer, src_buffer);
    if (br != dmBuffer::RESULT_OK) {
        if (new_buffer_needed) {
            dmBuffer::Destroy(dst_buffer);
        }
        return luaL_error(L, "Could not copy data from buffer: %s (%d).", dmBuffer::GetResultString(br), br);
    }

    // If we created a new buffer, make sure to destroy the old one.
    if (new_buffer_needed) {
        dmBuffer::Destroy(buffer_resource->m_Buffer);
        buffer_resource->m_Buffer = dst_buffer;
        buffer_resource->m_ElementCount = src_count;
    }

    assert(top == lua_gettop(L));
    return 0;
}

static const luaL_reg Module_methods[] =
{
    {"set", Set},
    {"load", Load},
    {"set_texture", SetTexture},
    {"get_buffer", GetBuffer},
    {"set_buffer", SetBuffer},

    // LiveUpdate functionality in resource namespace
    {"get_current_manifest", dmLiveUpdate::Resource_GetCurrentManifest},
    {"store_resource", dmLiveUpdate::Resource_StoreResource},
    {"store_manifest", dmLiveUpdate::Resource_StoreManifest},

    {0, 0}
};

/*# 2D texture type
 *
 * @name resource.TEXTURE_TYPE_2D
 * @variable
 */

/*# luminance type texture format
 *
 * @name resource.TEXTURE_FORMAT_LUMINANCE
 * @variable
 */

/*# RGB type texture format
 *
 * @name resource.TEXTURE_FORMAT_RGB
 * @variable
 */

/*# RGBA type texture format
 *
 * @name resource.TEXTURE_FORMAT_RGBA
 * @variable
 */

 /*# LIVEUPDATE_OK
 *
 * @name resource.LIVEUPDATE_OK
 * @variable
 */

 /*# LIVEUPDATE_INVALID_RESOURCE
 * The handled resource is invalid.
 *
 * @name resource.LIVEUPDATE_INVALID_RESOURCE
 * @variable
 */

 /*# LIVEUPDATE_VERSION_MISMATCH
 * Mismatch between manifest expected version and actual version.
 *
 * @name resource.LIVEUPDATE_VERSION_MISMATCH
 * @variable
 */

 /*# LIVEUPDATE_ENGINE_VERSION_MISMATCH
 * Mismatch between running engine version and engine versions supported by manifest.
 *
 * @name resource.LIVEUPDATE_ENGINE_VERSION_MISMATCH
 * @variable
 */

 /*# LIVEUPDATE_SIGNATURE_MISMATCH
 * Mismatch between manifest expected signature and actual signature.
 *
 * @name resource.LIVEUPDATE_SIGNATURE_MISMATCH
 * @variable
 */

 /*# LIVEUPDATE_SCHEME_MISMATCH
 * Mismatch between scheme used to load resources. Resources are loaded with a different scheme than from manifest, for example over HTTP or directly from file. This is typically the case when running the game directly from the editor instead of from a bundle.
 *
 * @name resource.LIVEUPDATE_SCHEME_MISMATCH
 * @variable
 */

 /*# LIVEUPDATE_BUNDLED_RESOURCE_MISMATCH
 * Mismatch between between expected bundled resources and actual bundled resources. The manifest expects a resource to be in the bundle, but it was not found in the bundle. This is typically the case when a non-excluded resource was modified between publishing the bundle and publishing the manifest.
 *
 * @name resource.LIVEUPDATE_BUNDLED_RESOURCE_MISMATCH
 * @variable
 */

 /*# LIVEUPDATE_FORMAT_ERROR
 * Failed to parse manifest data buffer. The manifest was probably produced by a different engine version.
 *
 * @name resource.LIVEUPDATE_FORMAT_ERROR
 * @variable
 */
static void LuaInit(lua_State* L)
{
    int top = lua_gettop(L);
    luaL_register(L, "resource", Module_methods);

#define SETGRAPHICSCONSTANT(name) \
    lua_pushnumber(L, (lua_Number) dmGraphics:: name); \
    lua_setfield(L, -2, #name); \

    SETGRAPHICSCONSTANT(TEXTURE_TYPE_2D);
    SETGRAPHICSCONSTANT(TEXTURE_TYPE_CUBE_MAP);

    SETGRAPHICSCONSTANT(TEXTURE_FORMAT_LUMINANCE);
    SETGRAPHICSCONSTANT(TEXTURE_FORMAT_RGB);
    SETGRAPHICSCONSTANT(TEXTURE_FORMAT_RGBA);
    SETGRAPHICSCONSTANT(TEXTURE_FORMAT_DEPTH);
    SETGRAPHICSCONSTANT(TEXTURE_FORMAT_STENCIL);
    SETGRAPHICSCONSTANT(TEXTURE_FORMAT_RGB_PVRTC_2BPPV1);
    SETGRAPHICSCONSTANT(TEXTURE_FORMAT_RGB_PVRTC_4BPPV1);
    SETGRAPHICSCONSTANT(TEXTURE_FORMAT_RGBA_PVRTC_2BPPV1);
    SETGRAPHICSCONSTANT(TEXTURE_FORMAT_RGBA_PVRTC_4BPPV1);
    SETGRAPHICSCONSTANT(TEXTURE_FORMAT_RGB_ETC1);
    /* DEF-994 We don't support these internally yet
    SETGRAPHICSCONSTANT(TEXTURE_FORMAT_RGB_DXT1);
    SETGRAPHICSCONSTANT(TEXTURE_FORMAT_RGBA_DXT1);
    SETGRAPHICSCONSTANT(TEXTURE_FORMAT_RGBA_DXT3);
    SETGRAPHICSCONSTANT(TEXTURE_FORMAT_RGBA_DXT5);
    */

#undef SETGRAPHICSCONSTANT

#define SETCONSTANT(name, val) \
        lua_pushnumber(L, (lua_Number) val); \
        lua_setfield(L, -2, #name);\

    SETCONSTANT(LIVEUPDATE_OK, dmLiveUpdate::RESULT_OK);
    SETCONSTANT(LIVEUPDATE_INVALID_RESOURCE, dmLiveUpdate::RESULT_INVALID_RESOURCE);
    SETCONSTANT(LIVEUPDATE_VERSION_MISMATCH, dmLiveUpdate::RESULT_VERSION_MISMATCH);
    SETCONSTANT(LIVEUPDATE_ENGINE_VERSION_MISMATCH, dmLiveUpdate::RESULT_ENGINE_VERSION_MISMATCH);
    SETCONSTANT(LIVEUPDATE_SIGNATURE_MISMATCH, dmLiveUpdate::RESULT_SIGNATURE_MISMATCH);
    SETCONSTANT(LIVEUPDATE_SCHEME_MISMATCH, dmLiveUpdate::RESULT_SCHEME_MISMATCH);
    SETCONSTANT(LIVEUPDATE_BUNDLED_RESOURCE_MISMATCH, dmLiveUpdate::RESULT_BUNDLED_RESOURCE_MISMATCH);
    SETCONSTANT(LIVEUPDATE_FORMAT_ERROR, dmLiveUpdate::RESULT_FORMAT_ERROR);

#undef SETCONSTANT

    lua_pop(L, 1);
    assert(top == lua_gettop(L));
}

void ScriptResourceRegister(const ScriptLibContext& context)
{
    LuaInit(context.m_LuaState);
    g_ResourceModule.m_Factory = context.m_Factory;
}

void ScriptResourceFinalize(const ScriptLibContext& context)
{
}

} // namespace dmGameSystem
