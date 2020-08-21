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

import java.io.ByteArrayInputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.util.EnumSet;

import com.dynamo.bob.Builder;
import com.dynamo.bob.BuilderParams;
import com.dynamo.bob.CompileExceptionError;
import com.dynamo.bob.Task;
import com.dynamo.bob.Task.TaskBuilder;
import com.dynamo.bob.fs.IResource;
import com.dynamo.bob.util.TextureUtil;
import com.dynamo.bob.TexcLibrary.FlipAxis;
import com.dynamo.graphics.proto.Graphics.Cubemap;
import com.dynamo.graphics.proto.Graphics.TextureImage;
import com.dynamo.graphics.proto.Graphics.TextureImage.Image;
import com.dynamo.graphics.proto.Graphics.TextureImage.Type;
import com.dynamo.graphics.proto.Graphics.TextureProfile;
import com.google.protobuf.ByteString;

@BuilderParams(name = "Cubemap", inExts = {".cubemap"}, outExt = ".texturec")
public class CubemapBuilder extends Builder<Void> {

    @Override
    public Task<Void> create(IResource input) throws IOException, CompileExceptionError {
        Cubemap.Builder builder = Cubemap.newBuilder();
        ProtoUtil.merge(input, builder);
        Cubemap cubemap = builder.build();

        TaskBuilder<Void> taskBuilder = Task.<Void>newBuilder(this)
                .setName(params.name())
                .addInput(input)
                .addInput(input.getResource(cubemap.getRight()))
                .addInput(input.getResource(cubemap.getLeft()))
                .addInput(input.getResource(cubemap.getTop()))
                .addInput(input.getResource(cubemap.getBottom()))
                .addInput(input.getResource(cubemap.getFront()))
                .addInput(input.getResource(cubemap.getBack()))
                .addOutput(input.changeExt(params.outExt()));

        // If there is a texture profiles file, we need to make sure
        // it has been read before building this tile set, add it as an input.
        String textureProfilesPath = this.project.getProjectProperties().getStringValue("graphics", "texture_profiles");
        if (textureProfilesPath != null) {
            taskBuilder.addInput(this.project.getResource(textureProfilesPath));
        }

        return taskBuilder.build();
    }

    @Override
    public void build(Task<Void> task) throws CompileExceptionError,
            IOException {

        TextureProfile texProfile = TextureUtil.getTextureProfileByPath(this.project.getTextureProfiles(), task.input(0).getPath());

        TextureImage[] textures = new TextureImage[6];
        try {
            for (int i = 0; i < 6; i++) {
                ByteArrayInputStream is = new ByteArrayInputStream(task.input(i + 1).getContent());
                boolean compress = project.option("texture-compression", "false").equals("true");
                // NOTE: Cubemap sides should not have a flipped Y axis (as opposed to any other texture).
                // I could only find tidbits of information regarding this online, as far as I understand
                // it is not explained in the OGL spec or cubemap extension either.
                // However, most suggest that the origin of cubemap sides are upper left as opposed to lower left
                // like this SO answer suggest;
                //   "Cube Maps have been specified to follow the RenderMan specification (for whatever reason),
                //    and RenderMan assumes the images' origin being in the upper left, contrary to the usual
                //    OpenGL behaviour of having the image origin in the lower left."
                // Source: https://stackoverflow.com/a/11690553/129360
                //
                // So for cube map textures we don't flip on any axis, meaning the texture data begin at the
                // upper left corner of the input image.
                TextureImage texture = TextureGenerator.generate(is, texProfile, compress, EnumSet.noneOf(FlipAxis.class));
                textures[i] = texture;
            }
            validate(task, textures);
        } catch (TextureGeneratorException e) {
            throw new CompileExceptionError(task.input(0), -1, e.getMessage(), e);
        }

        TextureImage.Builder builder = TextureImage.newBuilder(textures[0]);

        for (int i = 0; i < builder.getAlternativesCount(); i++) {
            Image.Builder imageBuilder = TextureImage.Image.newBuilder(textures[0].getAlternatives(i));

            ByteArrayOutputStream os = new ByteArrayOutputStream(1024 * 4);
            for (int j = 0; j < imageBuilder.getMipMapSizeCount(); j++) {
                int mipSize = imageBuilder.getMipMapSize(j);
                byte[] buf = new byte[mipSize];
                for (int k = 0; k < 6; k++) {
                    ByteString data = textures[k].getAlternatives(i).getData();
                    int mipOffset = imageBuilder.getMipMapOffset(j);
                    data.copyTo(buf, mipOffset, 0, mipSize);
                    os.write(buf);
                }
            }
            os.flush();
            imageBuilder.setData(ByteString.copyFrom(os.toByteArray()));
            for (int j = 0; j < imageBuilder.getMipMapSizeCount(); j++) {
                imageBuilder.setMipMapOffset(j, imageBuilder.getMipMapOffset(j) * 6);
            }
            builder.setAlternatives(i, imageBuilder);
        }

        builder.setCount(6);
        builder.setType(Type.TYPE_CUBEMAP);

        TextureImage texture = builder.build();
        ByteArrayOutputStream out = new ByteArrayOutputStream(1024 * 1024);
        texture.writeTo(out);
        out.close();
        task.output(0).setContent(out.toByteArray());
    }

    private void validate(Task<Void> task, TextureImage[] textures) throws CompileExceptionError {
        for (int i = 1; i < 6; i++) {
            TextureImage t = textures[i];
            for (int j = 0; j < t.getAlternativesCount(); j++) {
                Image im = t.getAlternatives(j);

                Image compareTo = textures[0].getAlternatives(j);
                if (im.getWidth() != compareTo.getWidth() || im.getHeight() != compareTo.getHeight()) {
                    throw new CompileExceptionError(task.input(0), -1, "Cubemap sides must be of identical size", null);
                }
            }
        }
    }
}
