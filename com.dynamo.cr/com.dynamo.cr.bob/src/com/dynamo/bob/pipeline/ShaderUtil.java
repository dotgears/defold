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

import java.util.ArrayList;
import java.util.Iterator;
import java.util.regex.Matcher;
import java.util.regex.Pattern;
import java.io.IOException;

import org.codehaus.jackson.JsonNode;
import org.codehaus.jackson.map.ObjectMapper;

import com.dynamo.bob.CompileExceptionError;


public class ShaderUtil {

    public static class SPIRVReflector {
        private static JsonNode root;

        public SPIRVReflector(String json) throws IOException
        {
            this.root = (new ObjectMapper()).readTree(json);
        }

        public static class Resource
        {
            public String name;
            public String type;
            public int    binding;
            public int    set;
        }

        public static class UniformBlock extends Resource
        {
            public ArrayList<Resource> uniforms;
        }

        public static ArrayList<UniformBlock> getUniformBlocks()
        {
            ArrayList<UniformBlock> uniformBlocks = new ArrayList<UniformBlock>();

            JsonNode uboNode   = root.get("ubos");
            JsonNode typesNode = root.get("types");

            if (uboNode == null || typesNode == null) {
                return uniformBlocks;
            }

            Iterator<JsonNode> uniformBlockNodeIt = uboNode.getElements();
            while (uniformBlockNodeIt.hasNext()) {
                JsonNode uniformBlockNode = uniformBlockNodeIt.next();

                UniformBlock ubo = new UniformBlock();
                ubo.name         = uniformBlockNode.get("name").asText();
                ubo.set          = uniformBlockNode.get("set").asInt();
                ubo.binding      = uniformBlockNode.get("binding").asInt();
                ubo.uniforms     = new ArrayList<Resource>();

                JsonNode typeNode    = typesNode.get(uniformBlockNode.get("type").asText());
                JsonNode membersNode = typeNode.get("members");

                for (Iterator<JsonNode> membersNodeIt = membersNode.getElements(); membersNodeIt.hasNext();) {
                    JsonNode uniformNode   = membersNodeIt.next();
                    String uniformNodeName = uniformNode.get("name").asText();
                    String uniformNodeType = uniformNode.get("type").asText();

                    Resource res = new Resource();
                    res.name     = uniformNode.get("name").asText();
                    res.type     = uniformNode.get("type").asText();
                    res.binding  = 0;
                    res.set      = 0;
                    ubo.uniforms.add(res);
                }

                uniformBlocks.add(ubo);
            }

            return uniformBlocks;
        }

        public static ArrayList<Resource> getTextures() {
            ArrayList<Resource> textures = new ArrayList<Resource>();

            JsonNode texturesNode = root.get("textures");

            if (texturesNode == null) {
                return textures;
            }

            for (Iterator<JsonNode> iter = texturesNode.getElements(); iter.hasNext();) {
                JsonNode textureNode = iter.next();
                Resource res = new Resource();
                res.name     = textureNode.get("name").asText();
                res.type     = textureNode.get("type").asText();
                res.binding  = textureNode.get("binding").asInt();
                res.set      = textureNode.get("set").asInt();
                textures.add(res);
            }

            return textures;
        }

        public static ArrayList<Resource> getInputs() {
            ArrayList<Resource> inputs = new ArrayList<Resource>();

            JsonNode inputsNode = root.get("inputs");

            if (inputsNode == null) {
                return inputs;
            }

            for (Iterator<JsonNode> iter = inputsNode.getElements(); iter.hasNext();) {
                JsonNode inputNode = iter.next();
                Resource res = new Resource();
                res.name     = inputNode.get("name").asText();
                res.type     = inputNode.get("type").asText();
                res.binding  = inputNode.get("location").asInt();
                inputs.add(res);
            }

            return inputs;
        }
    }

    public static class ES2ToES3Converter {

        /*
         * ES2ToES3Converter is converting shaders using old shader language syntax into GLES version 310 compliant shaders or GL version 140 compliant shaders depending on target (GLES or GL)
         *
         * The following rules apply
         *
         * * Shader version and profile (#version xxx xx) is overridden if declared in the shader (default is minimum per target, see above)
         *
         * Vertex Shaders:
         * * "attribute" keyword is changed to "in"
         * * "varying" keyword is changed to "out"
         * * Uniform declarations are wrapped to global space uniform buffers
         *   - Unless the type is opaque (sampler, image or atomic_uint)
         *
         * Fragment Shaders:
         * * "varying" keyword is changed to "out"
         * * Uniform declarations are wrapped to global space uniform buffers
         *   - Unless the type is opaque (sampler, image or atomic_uint)
         * * Precision mediump float is added to ES shaders if not existing
         * * If exists, gl_FragColor or gl_FragData are converted to a generated out attribute.
         *   - If they exist (otherwise, one can assume this is already a compliant shader)
         *   - On ES targets only (GL targets accepts old style)
         *   - Placed after the first precision statement (if ES)
         *
         * Note: This covers known cases, but if the shader has reserved variable names of (newer) keywords as "in", "out" or "texture" they will have to be modified by the writer.
         * This would have to be done in any case upgrading shaders from GLES2 and is nothing we can patch as changing those members names would instead mean the run-time access (get/set uniform)
         * will fail which is even worse.
         *
         */

        public static class Result {
            public String shaderVersion = "";
            public String shaderProfile = "";
            public String output = "";
        }

        public static enum ShaderType {
            VERTEX_SHADER, FRAGMENT_SHADER
        };

        private static final String[] opaqueUniformTypesPrefix = { "sampler", "image", "atomic_uint" };

        private static final String  regexCommentRemovePattern = "(?:/\\*(?:[^*]|(?:\\*+[^*/]))*\\*+/)|(?://.*)"; // Ref http://blog.ostermiller.org/find-comment
        private static final Pattern regexLineBreakPattern = Pattern.compile("(?<=;)|(?<=\\{)|(?<=\\})|(?<=(#(.{0,1024}\\n)))");
        private static final Pattern regexVersionStringPattern = Pattern.compile("^\\h*#\\h*version\\h+(?<version>\\d+)(\\h+(?<profile>\\S+))?\\h*\\n");
        private static final Pattern regexPrecisionKeywordPattern = Pattern.compile("(?<keyword>precision)\\s+(?<precision>lowp|mediump|highp)\\s+(?<type>float|int)\\s*;");
        private static final Pattern regexUniformKeywordPattern = Pattern.compile("((?<keyword>uniform)\\s+|(?<layout>layout\\s*\\(.*\\n*.*\\)\\s*)\\s+|(?<precision>lowp|mediump|highp)\\s+)*(?<type>\\S+)\\s+(?<identifier>\\S+)\\s*(?<any>.*)\\s*;");

        private static final String[][] vsKeywordReps = {{"varying", "out"}, {"attribute", "in"}};
        private static final String[][] fsKeywordReps = {{"varying", "in"}, {"texture2D", "texture"}, {"textureCube", "texture"}};

        private static final String dmEngineGeneratedRep = "_DMENGINE_GENERATED_";

        private static final String glUBRep = dmEngineGeneratedRep + "UB_";
        private static final String glFragColorKeyword = "gl_FragColor";
        private static final String glFragDataKeyword = "gl_FragData";
        private static final String glFragColorRep = dmEngineGeneratedRep + glFragColorKeyword;
        private static final String glFragColorAttrRep = "\nout vec4 " + glFragColorRep + ";\n";
        private static final String floatPrecisionAttrRep = "precision mediump float;\n";

        public static Result transform(String input, ShaderType shaderType, String targetProfile) throws CompileExceptionError {
            Result result = new Result();

            // Remove comments, early bail if zero code
            input = input.replaceAll(regexCommentRemovePattern,"");
            if(input.isEmpty()) {
                return result;
            }

            int layoutSet = shaderType == ShaderType.VERTEX_SHADER ? 0 : 1;

            // Index to output used for post patching tasks
            int floatPrecisionIndex = -1;
            int patchLineIndex = 0;

            // Try get version and profile. Override targetProfile if version is set in shader
            Matcher versionMatcher = regexVersionStringPattern.matcher(input.substring(0, Math.min(input.length(), 128)));
            if (versionMatcher.find()) {
                patchLineIndex = 1;
                result.shaderVersion = versionMatcher.group("version");
                result.shaderProfile = versionMatcher.group("profile");
                result.shaderProfile = result.shaderProfile == null ? "" : result.shaderProfile;
                // override targetProfile if version is set in shader
                targetProfile = result.shaderProfile;
            }

            // Patch qualifiers (reserved keywords so word boundary replacement is safe)
            String[][] keywordReps = (shaderType == ShaderType.VERTEX_SHADER) ? vsKeywordReps : fsKeywordReps;
            for ( String[] keywordRep : keywordReps) {
                input = input.replaceAll("\\b" + keywordRep[0] + "\\b", keywordRep[1]);
            }

            // on ES shaders, replace glFragColor if exists
            if(targetProfile.equals("es")) {
                input = input.replaceAll("\\b" + glFragColorKeyword + "\\b", glFragColorRep);
                input = input.replaceAll("\\b" + glFragDataKeyword + "\\b", glFragColorRep);
            }

            // Split into slices separated by semicolon, curly bracket scopes and preprocessor definition lines: ";", "{", "}" and "#..<\n>"
            // This to reduce parsing complexity
            String[] inputLines = regexLineBreakPattern.split(input);

           // Preallocate array of resulting slices. This makes patching in specific positions less complex
           ArrayList<String> output = new ArrayList<String>(input.length());

            // Multi-instance patching
            int ubIndex = 0;
            for(String line : inputLines ) {

                if(line.contains("uniform") && !line.contains("{"))
                {
                    // Transform non-opaque uniforms into uniform blocks (UB's). Do not process existing UB's
                    Matcher uniformMatcher = regexUniformKeywordPattern.matcher(line);
                    if(uniformMatcher.find()) {
                        String keyword = uniformMatcher.group("keyword");
                        if(keyword != null) {
                            String layout = uniformMatcher.group("layout");
                            String precision = uniformMatcher.group("precision");
                            String type = uniformMatcher.group("type");
                            String identifier = uniformMatcher.group("identifier");
                            String any = uniformMatcher.group("any");

                            boolean isOpaque = false;
                            for( String opaqueTypePrefix : opaqueUniformTypesPrefix) {
                                if(type.startsWith(opaqueTypePrefix)) {
                                    isOpaque = true;
                                    break;
                                }
                            }

                            if (layout == null) {
                                layout = "layout(set=" + layoutSet + ")";
                            }

                            if (isOpaque){
                                line = layout + " " + line;
                            } else {
                                line = "\n" + layout + " " + keyword + " " + glUBRep + ubIndex++ + " { " +
                                (precision == null ? "" : (precision + " ")) + type + " " + identifier + " " + (any == null ? "" : (any + " ")) + "; };";
                            }
                        }
                    }
                }
                else if (line.contains("precision")) {
                    // Check if precision keyword present and store index if so, for post patch tasks
                    Matcher precisionMatcher = regexPrecisionKeywordPattern.matcher(line);
                    if(precisionMatcher.find()) {
                        if(precisionMatcher.group("type").equals("float")) {
                            patchLineIndex = output.size();
                            floatPrecisionIndex = output.size();
                        }
                    }
                }

                output.add(line);
            }

            // Post patching
            if(targetProfile.equals("es")) {
                // Currently only required on fragment shaders for ES profiles..
                if (shaderType == ShaderType.FRAGMENT_SHADER) {
                    // if we have patched glFragColor
                    if(input.contains(glFragColorRep)) {
                        // insert precision if not found, as it is mandatory for out attributes
                        if(floatPrecisionIndex < 0) {
                            output.add(patchLineIndex++, floatPrecisionAttrRep);
                        }
                        // insert fragcolor out attr
                        output.add(patchLineIndex++, glFragColorAttrRep);
                    }
                }
            }

            result.output = String.join("", output);
            return result;
        }
    }

}
