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

package com.dynamo.bob.pipeline;

import static org.junit.Assert.assertEquals;
import java.util.List;

import org.junit.Before;
import org.junit.Test;

import com.dynamo.model.proto.ModelProto.Model;
import com.dynamo.rig.proto.Rig.RigScene;
import com.google.protobuf.Message;

public class ModelBuilderTest extends AbstractProtoBuilderTest {

    @Before
    public void setup() {
        addTestFiles();
    }

    @Test
    public void testModel() throws Exception {
        addFile("/test_meshset.dae", "");
        addFile("/test_skeleton.dae", "");
        addFile("/test_animationset.dae", "");
        addFile("/test.material", "");

        StringBuilder src = new StringBuilder();
        src.append(" mesh: \"/test_meshset.dae\"");
        src.append(" material: \"/test.material\"");
        src.append(" skeleton: \"/test_skeleton.dae\"");
        src.append(" animations: \"/test_animationset.dae\"");
        src.append(" default_animation: \"test\"");
        List<Message> outputs = build("/test.model", src.toString());

        Model model = (Model)outputs.get(0);
        assertEquals("/test.rigscenec", model.getRigScene());
        assertEquals("/test.materialc", model.getMaterial());
        assertEquals("test", model.getDefaultAnimation());

        RigScene rigScene = (RigScene)outputs.get(1);
        assertEquals("/test_meshset.meshsetc", rigScene.getMeshSet());
        assertEquals("/test_skeleton.skeletonc", rigScene.getSkeleton());
        assertEquals("/test_animationset_generated_0.animationsetc", rigScene.getAnimationSet());
    }
}
