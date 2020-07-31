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

import java.io.IOException;
import java.util.HashMap;
import java.util.Map;

import org.apache.commons.io.FilenameUtils;

import com.dynamo.bob.fs.IResource;
import com.dynamo.gameobject.proto.GameObject.CollectionDesc;
import com.dynamo.gameobject.proto.GameObject.PrototypeDesc;
import com.dynamo.graphics.proto.Graphics;
import com.dynamo.graphics.proto.Graphics.TextureImage;
import com.dynamo.gui.proto.Gui;
import com.dynamo.lua.proto.Lua.LuaModule;
import com.dynamo.model.proto.ModelProto;
import com.dynamo.render.proto.Font;
import com.dynamo.rig.proto.Rig;
import com.dynamo.spine.proto.Spine;
import com.dynamo.sprite.proto.Sprite.SpriteDesc;
import com.dynamo.textureset.proto.TextureSetProto.TextureSet;
import com.google.protobuf.InvalidProtocolBufferException;
import com.google.protobuf.Message;
import com.google.protobuf.TextFormat;
import com.google.protobuf.TextFormat.ParseException;

public class ParseUtil {

    private interface IParser {
        Message parse(byte[] content) throws InvalidProtocolBufferException;
    }

    private static Map<String, IParser> parseMap = new HashMap<String, IParser>();

    static {
        parseMap.put("collectionc", new IParser() {
            @Override
            public Message parse(byte[] content) throws InvalidProtocolBufferException {
                return CollectionDesc.parseFrom(content);
            }
        });
        parseMap.put("scriptc", new IParser() {
            @Override
            public Message parse(byte[] content) throws InvalidProtocolBufferException {
                return LuaModule.parseFrom(content);
            }
        });
        parseMap.put("goc", new IParser() {
            @Override
            public Message parse(byte[] content) throws InvalidProtocolBufferException {
                return PrototypeDesc.parseFrom(content);
            }
        });
        parseMap.put("spritec", new IParser() {
            @Override
            public Message parse(byte[] content) throws InvalidProtocolBufferException {
                return SpriteDesc.parseFrom(content);
            }
        });
        parseMap.put("texturec", new IParser() {
            @Override
            public Message parse(byte[] content) throws InvalidProtocolBufferException {
                return TextureImage.parseFrom(content);
            }
        });
        parseMap.put("texturesetc", new IParser() {
            @Override
            public Message parse(byte[] content) throws InvalidProtocolBufferException {
                return TextureSet.parseFrom(content);
            }
        });
        parseMap.put("goc", new IParser() {
            @Override
            public Message parse(byte[] content) throws InvalidProtocolBufferException {
                return PrototypeDesc.parseFrom(content);
            }
        });
        parseMap.put("go", new IParser() {
            @Override
            public Message parse(byte[] content) throws InvalidProtocolBufferException {
                PrototypeDesc.Builder builder = PrototypeDesc.newBuilder();
                try {
                    TextFormat.merge(new String(content), builder);
                } catch (ParseException e) {
                    throw new RuntimeException(e);
                }
                return builder.build();
            }
        });
        parseMap.put("sprite", new IParser() {
            @Override
            public Message parse(byte[] content) throws InvalidProtocolBufferException {
                SpriteDesc.Builder builder = SpriteDesc.newBuilder();
                try {
                    TextFormat.merge(new String(content), builder);
                } catch (ParseException e) {
                    throw new RuntimeException(e);
                }
                return builder.build();
            }
        });
        parseMap.put("skeletonc", new IParser() {
            @Override
            public Message parse(byte[] content) throws InvalidProtocolBufferException {
                return Rig.Skeleton.parseFrom(content);
            }
        });
        parseMap.put("meshsetc", new IParser() {
            @Override
            public Message parse(byte[] content) throws InvalidProtocolBufferException {
                return Rig.MeshSet.parseFrom(content);
            }
        });
        parseMap.put("animationsetc", new IParser() {
            @Override
            public Message parse(byte[] content) throws InvalidProtocolBufferException {
                return Rig.AnimationSet.parseFrom(content);
            }
        });
        parseMap.put("rigscenec", new IParser() {
            @Override
            public Message parse(byte[] content) throws InvalidProtocolBufferException {
                return Rig.RigScene.parseFrom(content);
            }
        });
        parseMap.put("modelc", new IParser() {
            @Override
            public Message parse(byte[] content) throws InvalidProtocolBufferException {
                return ModelProto.Model.parseFrom(content);
            }
        });
        parseMap.put("vpc", new IParser() {
            @Override
            public Message parse(byte[] content) throws InvalidProtocolBufferException {
                return Graphics.ShaderDesc.parseFrom(content);
            }
        });
        parseMap.put("fpc", new IParser() {
            @Override
            public Message parse(byte[] content) throws InvalidProtocolBufferException {
                return Graphics.ShaderDesc.parseFrom(content);
            }
        });
        parseMap.put("spinemodelc", new IParser() {
            @Override
            public Message parse(byte[] content) throws InvalidProtocolBufferException {
                return Spine.SpineModelDesc.parseFrom(content);
            }
        });
        parseMap.put("fontc", new IParser() {
            @Override
            public Message parse(byte[] content) throws InvalidProtocolBufferException {
                return Font.FontMap.parseFrom(content);
            }
        });
        parseMap.put("guic", new IParser() {
            @Override
            public Message parse(byte[] content) throws InvalidProtocolBufferException {
                return Gui.SceneDesc.parseFrom(content);
            }
        });
    }

    public static Message parse(IResource resource) throws IOException, InvalidProtocolBufferException {
        String ext = FilenameUtils.getExtension(resource.getPath());
        IParser parser = parseMap.get(ext);
        if (parser == null) {
            throw new RuntimeException("No parser registered for extension " + ext);
        }
        return parser.parse(resource.getContent());
    }
}
