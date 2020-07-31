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

#include <dlib/hash.h>
#include <dlib/math.h>
#include <script/script.h>

#include "render/render.h"
#include "render/render_private.h"

using namespace Vectormath::Aos;
namespace dmGraphics
{
    extern const Vector4& GetConstantV4Ptr(dmGraphics::HContext context, int base_register);
}

static inline dmGraphics::ShaderDesc::Shader MakeDDFShader(const char* data, uint32_t count)
{
    dmGraphics::ShaderDesc::Shader ddf;
    memset(&ddf,0,sizeof(ddf));
    ddf.m_Source.m_Data  = (uint8_t*)data;
    ddf.m_Source.m_Count = count;
    return ddf;
}

TEST(dmMaterialTest, TestTags)
{
    dmGraphics::Initialize();
    dmGraphics::HContext context = dmGraphics::NewContext(dmGraphics::ContextParams());
    dmRender::RenderContextParams params;
    params.m_ScriptContext = dmScript::NewContext(0, 0, true);
    dmRender::HRenderContext render_context = dmRender::NewRenderContext(context, params);

    dmGraphics::ShaderDesc::Shader shader = MakeDDFShader("foo", 3);
    dmGraphics::HVertexProgram vp = dmGraphics::NewVertexProgram(context, &shader);
    dmGraphics::HFragmentProgram fp = dmGraphics::NewFragmentProgram(context, &shader);

    dmRender::HMaterial material = dmRender::NewMaterial(render_context, vp, fp);

    dmhash_t tags[] = {dmHashString64("tag1"), dmHashString64("tag2")};
    uint32_t mask = dmRender::ConvertMaterialTagsToMask(tags, 1);
    ASSERT_NE(0u, mask);
    dmRender::AddMaterialTag(material, tags[0]);
    dmRender::AddMaterialTag(material, tags[1]);
    mask = dmRender::ConvertMaterialTagsToMask(tags, 2);
    ASSERT_EQ(mask, dmRender::GetMaterialTagMask(material));

    dmGraphics::DeleteVertexProgram(vp);
    dmGraphics::DeleteFragmentProgram(fp);

    dmRender::DeleteMaterial(render_context, material);

    dmRender::DeleteRenderContext(render_context, 0);
    dmGraphics::DeleteContext(context);
    dmScript::DeleteContext(params.m_ScriptContext);
}

TEST(dmMaterialTest, TestMaterialConstants)
{
    dmGraphics::Initialize();
    dmGraphics::HContext context = dmGraphics::NewContext(dmGraphics::ContextParams());
    dmRender::RenderContextParams params;
    params.m_ScriptContext = dmScript::NewContext(0, 0, true);
    dmRender::HRenderContext render_context = dmRender::NewRenderContext(context, params);

    // create default material
    dmGraphics::ShaderDesc::Shader vp_shader = MakeDDFShader("uniform vec4 tint;\n", 19);
    dmGraphics::HVertexProgram vp = dmGraphics::NewVertexProgram(context, &vp_shader);

    dmGraphics::ShaderDesc::Shader fp_shader = MakeDDFShader("foo", 3);
    dmGraphics::HFragmentProgram fp = dmGraphics::NewFragmentProgram(context, &fp_shader);
    dmRender::HMaterial material = dmRender::NewMaterial(render_context, vp, fp);

    // renderobject default setup
    dmRender::RenderObject ro;
    ro.m_Material = material;
    dmRender::EnableRenderObjectConstant(&ro, dmHashString64("tint"), Vector4(1.0f, 0.0f, 0.0f, 0.0f));

    // test setting constant
    dmGraphics::HProgram program = dmRender::GetMaterialProgram(material);
    dmGraphics::EnableProgram(context, program);
    uint32_t tint_loc = dmGraphics::GetUniformLocation(program, "tint");
    ASSERT_EQ(0, tint_loc);
    dmRender::ApplyRenderObjectConstants(render_context, 0, &ro);
    const Vector4& v = dmGraphics::GetConstantV4Ptr(context, tint_loc);
    ASSERT_EQ(1.0f, v.getX());
    ASSERT_EQ(0.0f, v.getY());
    ASSERT_EQ(0.0f, v.getZ());
    ASSERT_EQ(0.0f, v.getW());

    dmGraphics::DisableProgram(context);
    dmGraphics::DeleteVertexProgram(vp);
    dmGraphics::DeleteFragmentProgram(fp);
    dmRender::DeleteMaterial(render_context, material);
    dmRender::DeleteRenderContext(render_context, 0);
    dmGraphics::DeleteContext(context);
    dmScript::DeleteContext(params.m_ScriptContext);
}

TEST(dmMaterialTest, TestMaterialConstantsOverride)
{
    dmGraphics::Initialize();
    dmGraphics::HContext context = dmGraphics::NewContext(dmGraphics::ContextParams());
    dmRender::RenderContextParams params;
    params.m_ScriptContext = dmScript::NewContext(0, 0, true);
    dmRender::HRenderContext render_context = dmRender::NewRenderContext(context, params);

    // create default material
    dmGraphics::ShaderDesc::Shader vp_shader = MakeDDFShader("uniform vec4 tint;\n", 19);
    dmGraphics::HVertexProgram vp = dmGraphics::NewVertexProgram(context, &vp_shader);
    dmGraphics::ShaderDesc::Shader fp_shader = MakeDDFShader("foo", 3);
    dmGraphics::HFragmentProgram fp = dmGraphics::NewFragmentProgram(context, &fp_shader);
    dmRender::HMaterial material = dmRender::NewMaterial(render_context, vp, fp);
    dmGraphics::HProgram program = dmRender::GetMaterialProgram(material);

    // create override material which contains tint, but at a different location
    vp_shader = MakeDDFShader("uniform vec4 dummy;\nuniform vec4 tint;\n", 40);
    dmGraphics::HVertexProgram vp_ovr = dmGraphics::NewVertexProgram(context, &vp_shader);
    dmGraphics::HFragmentProgram fp_ovr = dmGraphics::NewFragmentProgram(context, &fp_shader);
    dmRender::HMaterial material_ovr = dmRender::NewMaterial(render_context, vp_ovr, fp_ovr);
    dmGraphics::HProgram program_ovr = dmRender::GetMaterialProgram(material_ovr);

    // renderobject default setup
    dmRender::RenderObject ro;
    ro.m_Material = material;
    dmRender::EnableRenderObjectConstant(&ro, dmHashString64("tint"), Vector4(1.0f, 0.0f, 0.0f, 0.0f));

    // using the null graphics device, constant locations are assumed to be in declaration order.
    // test setting constant, no override material
    uint32_t tint_loc = dmGraphics::GetUniformLocation(program, "tint");
    ASSERT_EQ(0, tint_loc);
    dmGraphics::EnableProgram(context, program);
    dmRender::ApplyRenderObjectConstants(render_context, 0, &ro);
    const Vector4& v = dmGraphics::GetConstantV4Ptr(context, tint_loc);
    ASSERT_EQ(1.0f, v.getX());
    ASSERT_EQ(0.0f, v.getY());
    ASSERT_EQ(0.0f, v.getZ());
    ASSERT_EQ(0.0f, v.getW());

    // test setting constant, override material
    dmRender::EnableRenderObjectConstant(&ro, dmHashString64("tint"), Vector4(2.0f, 1.0f, 1.0f, 1.0f));
    ASSERT_EQ(0,  ro.m_Constants[0].m_Location);
    ASSERT_EQ(-1, ro.m_Constants[1].m_Location);
    ASSERT_EQ(-1, ro.m_Constants[2].m_Location);
    ASSERT_EQ(-1, ro.m_Constants[3].m_Location);
    uint32_t tint_loc_ovr = dmGraphics::GetUniformLocation(program_ovr, "tint");
    ASSERT_EQ(1, tint_loc_ovr);
    dmGraphics::EnableProgram(context, program_ovr);
    dmRender::ApplyRenderObjectConstants(render_context, material_ovr, &ro);
    const Vector4& v_ovr = dmGraphics::GetConstantV4Ptr(context, tint_loc_ovr);
    ASSERT_EQ(2.0f, v_ovr.getX());
    ASSERT_EQ(1.0f, v_ovr.getY());
    ASSERT_EQ(1.0f, v_ovr.getZ());
    ASSERT_EQ(1.0f, v_ovr.getW());

    dmGraphics::DisableProgram(context);
    dmGraphics::DeleteVertexProgram(vp_ovr);
    dmGraphics::DeleteFragmentProgram(fp_ovr);
    dmRender::DeleteMaterial(render_context, material_ovr);
    dmGraphics::DeleteVertexProgram(vp);
    dmGraphics::DeleteFragmentProgram(fp);
    dmRender::DeleteMaterial(render_context, material);
    dmRender::DeleteRenderContext(render_context, 0);
    dmGraphics::DeleteContext(context);
    dmScript::DeleteContext(params.m_ScriptContext);
}

int main(int argc, char **argv)
{
    jc_test_init(&argc, argv);
    return jc_test_run_all();
}
