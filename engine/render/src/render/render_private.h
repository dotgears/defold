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

#ifndef RENDERINTERNAL_H
#define RENDERINTERNAL_H

#include <dmsdk/vectormath/cpp/vectormath_aos.h>

#include <dlib/array.h>
#include <dlib/message.h>
#include <dlib/hashtable.h>

#include "render.h"

extern "C"
{
#include <lua/lua.h>
#include <lua/lauxlib.h>
#include <lua/lualib.h>
}

namespace dmRender
{
    using namespace Vectormath::Aos;

#define DEBUG_3D_NAME "_debug3d"
#define DEBUG_2D_NAME "_debug2d"

    struct Sampler
    {
        dmhash_t m_NameHash;
        int32_t  m_Location : 24;
        int32_t  m_Unit     : 8;

        dmGraphics::TextureFilter m_MinFilter;
        dmGraphics::TextureFilter m_MagFilter;
        dmGraphics::TextureWrap m_UWrap;
        dmGraphics::TextureWrap m_VWrap;

        Sampler(int32_t unit)
            : m_NameHash(0)
            , m_Location(-1)
            , m_Unit(unit)
            , m_MinFilter(dmGraphics::TEXTURE_FILTER_LINEAR_MIPMAP_NEAREST)
            , m_MagFilter(dmGraphics::TEXTURE_FILTER_LINEAR)
            , m_UWrap(dmGraphics::TEXTURE_WRAP_CLAMP_TO_EDGE)
            , m_VWrap(dmGraphics::TEXTURE_WRAP_CLAMP_TO_EDGE)
        {
        }
    };

    struct Material
    {

        Material()
        : m_RenderContext(0)
        , m_Program(0)
        , m_VertexProgram(0)
        , m_FragmentProgram(0)
        , m_TagMask(0)
        , m_UserData1(0)
        , m_UserData2(0)
        , m_VertexSpace(dmRenderDDF::MaterialDesc::VERTEX_SPACE_LOCAL)
        {
        }

        dmRender::HRenderContext                m_RenderContext;
        dmGraphics::HProgram                    m_Program;
        dmGraphics::HVertexProgram              m_VertexProgram;
        dmGraphics::HFragmentProgram            m_FragmentProgram;
        dmHashTable64<int32_t>                  m_NameHashToLocation;
        dmArray<MaterialConstant>               m_Constants;
        dmArray<Sampler>                        m_Samplers;
        uint32_t                                m_TagMask;
        uint64_t                                m_UserData1;
        uint64_t                                m_UserData2;
        dmRenderDDF::MaterialDesc::VertexSpace  m_VertexSpace;
    };

    // The order of this enum also defines the order in which the corresponding ROs should be rendered
    enum DebugRenderType
    {
        DEBUG_RENDER_TYPE_FACE_3D,
        DEBUG_RENDER_TYPE_LINE_3D,
        DEBUG_RENDER_TYPE_FACE_2D,
        DEBUG_RENDER_TYPE_LINE_2D,
        MAX_DEBUG_RENDER_TYPE_COUNT
    };

    struct DebugRenderTypeData
    {
        dmRender::RenderObject  m_RenderObject;
        void*                   m_ClientBuffer;
    };

    struct DebugRenderer
    {
        DebugRenderTypeData             m_TypeData[MAX_DEBUG_RENDER_TYPE_COUNT];
        Predicate                       m_3dPredicate;
        Predicate                       m_2dPredicate;
        dmRender::HRenderContext        m_RenderContext;
        dmGraphics::HVertexBuffer       m_VertexBuffer;
        dmGraphics::HVertexDeclaration  m_VertexDeclaration;
        uint32_t                        m_MaxVertexCount;
        uint32_t                        m_RenderBatchVersion;
    };

    const int MAX_TEXT_RENDER_CONSTANTS = 16;

    struct TextEntry
    {
        StencilTestParams   m_StencilTestParams;
        Matrix4             m_Transform;
        dmRender::Constant  m_RenderConstants[MAX_TEXT_RENDER_CONSTANTS];
        HFontMap            m_FontMap;
        HMaterial           m_Material;
        dmGraphics::BlendFactor m_SourceBlendFactor;
        dmGraphics::BlendFactor m_DestinationBlendFactor;
        uint64_t            m_BatchKey;
        uint32_t            m_FaceColor;
        uint32_t            m_StringOffset;
        uint32_t            m_OutlineColor;
        uint32_t            m_ShadowColor;
        uint16_t            m_RenderOrder;
        uint8_t             m_NumRenderConstants;
        bool                m_LineBreak;
        float               m_Width;
        float               m_Height;
        float               m_Leading;
        float               m_Tracking;
        int32_t             m_Next;
        int32_t             m_Tail;
        uint32_t            m_Align : 2;
        uint32_t            m_VAlign : 2;
        uint32_t            m_StencilTestParamsSet : 1;
    };

    struct TextContext
    {
        dmArray<dmRender::RenderObject>     m_RenderObjects;
        dmGraphics::HVertexBuffer           m_VertexBuffer;
        void*                               m_ClientBuffer;
        dmGraphics::HVertexDeclaration      m_VertexDecl;
        uint32_t                            m_RenderObjectIndex;
        uint32_t                            m_VertexIndex;
        uint32_t                            m_MaxVertexCount;
        uint32_t                            m_VerticesFlushed;
        dmArray<char>                       m_TextBuffer;
        // Map from batch id (hash of font-map etc) to index into m_TextEntries
        dmArray<TextEntry>                  m_TextEntries;
        uint32_t                            m_TextEntriesFlushed;
        uint32_t                            m_Frame;
    };

    struct RenderScriptContext
    {
        RenderScriptContext();

        lua_State*                  m_LuaState;
        uint32_t                    m_CommandBufferSize;
    };

    struct RenderListDispatch
    {
        RenderListDispatchFn m_Fn;
        void *m_UserData;
    };

    struct RenderListSortValue
    {
        union
        {
            struct
            {
                uint32_t m_BatchKey:24;
                uint32_t m_Dispatch:8;
                uint32_t m_Order:24;
                uint32_t m_MajorOrder:4;        // currently only 2 bits used (dmRender::RenderOrder)
                uint32_t m_MinorOrder:4;
            };
            // only temporarily used
            float m_ZW;
            // final sort value
            uint64_t m_SortKey;
        };
    };

    struct RenderListRange
    {
        uint32_t m_TagMask;
        uint32_t m_Start;   // Index into the renderlist
        uint32_t m_Count;
    };

    struct RenderContext
    {
        dmGraphics::HTexture        m_Textures[RenderObject::MAX_TEXTURE_COUNT];
        DebugRenderer               m_DebugRenderer;
        TextContext                 m_TextContext;
        dmScript::HContext          m_ScriptContext;
        RenderScriptContext         m_RenderScriptContext;
        dmArray<RenderObject*>      m_RenderObjects;
        dmScript::ScriptWorld*      m_ScriptWorld;

        dmArray<RenderListEntry>    m_RenderList;
        dmArray<RenderListDispatch> m_RenderListDispatch;
        dmArray<RenderListSortValue>m_RenderListSortValues;
        dmArray<uint32_t>           m_RenderListSortBuffer;
        dmArray<uint32_t>           m_RenderListSortIndices;
        dmArray<RenderListRange>    m_RenderListRanges;         // Maps tagmask to a range in the (sorted) render list

        HFontMap                    m_SystemFontMap;

        Matrix4                     m_View;
        Matrix4                     m_Projection;
        Matrix4                     m_ViewProj;

        dmGraphics::HContext        m_GraphicsContext;

        HMaterial                   m_Material;

        dmMessage::HSocket          m_Socket;

        uint32_t                    m_OutOfResources : 1;
        uint32_t                    m_StencilBufferCleared : 1;
    };

    void RenderTypeTextBegin(HRenderContext rendercontext, void* user_context);
    void RenderTypeTextDraw(HRenderContext rendercontext, void* user_context, RenderObject* ro_, uint32_t count);

    void RenderTypeDebugBegin(HRenderContext rendercontext, void* user_context);
    void RenderTypeDebugDraw(HRenderContext rendercontext, void* user_context, RenderObject* ro, uint32_t count);

    Result GenerateKey(HRenderContext render_context, const Matrix4& view_matrix);

    void ApplyRenderObjectConstants(HRenderContext render_context, HMaterial material, const struct RenderObject* ro);


    // Exposed here for unit testing
    struct RenderListEntrySorter
    {
        bool operator()(int a, int b) const
        {
            // Sort them on tag mask first, then render order (due to costly z calculations)
            const RenderListEntry& ea = m_Base[a];
            const RenderListEntry& eb = m_Base[b];
            return ea.m_TagMask < eb.m_TagMask;
        }
        RenderListEntry* m_Base;
    };

    struct FindRangeComparator
    {
        RenderListEntry* m_Entries;
        bool operator() (const uint32_t& a, const uint32_t& b) const
        {
            return m_Entries[a].m_TagMask < m_Entries[b].m_TagMask;
        }
    };

    typedef void (*RangeCallback)(void* ctx, uint32_t val, size_t start, size_t count);

    // Invokes the callback for each range. Two ranges are not guaranteed to preceed/succeed one another.
    void FindRenderListRanges(uint32_t* first, size_t offset, size_t size, RenderListEntry* entries, FindRangeComparator& comp, void* ctx, RangeCallback callback );

    bool FindTagMaskRange(RenderListRange* ranges, uint32_t num_ranges, uint32_t tag_mask, RenderListRange& range);
}

#endif

