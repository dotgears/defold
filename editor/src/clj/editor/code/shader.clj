;; Copyright 2020 The Defold Foundation
;; Licensed under the Defold License version 1.0 (the "License"); you may not use
;; this file except in compliance with the License.
;; 
;; You may obtain a copy of the License, together with FAQs at
;; https://www.defold.com/license
;; 
;; Unless required by applicable law or agreed to in writing, software distributed
;; under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
;; CONDITIONS OF ANY KIND, either express or implied. See the License for the
;; specific language governing permissions and limitations under the License.

(ns editor.code.shader
  (:require [clojure.string :as string]
            [dynamo.graph :as g]
            [editor.build-target :as bt]
            [editor.code.resource :as r]
            [editor.gl.shader :as shader]
            [editor.protobuf :as protobuf]
            [editor.resource :as resource]
            [editor.workspace :as workspace])
  (:import (com.dynamo.bob.pipeline ShaderProgramBuilder ShaderUtil$ES2ToES3Converter$ShaderType ShaderUtil$SPIRVReflector$Resource)
           (com.dynamo.graphics.proto Graphics$ShaderDesc)
           (com.google.protobuf ByteString)))

(set! *warn-on-reflection* true)
(set! *unchecked-math* :warn-on-boxed)

(def glsl-grammar
  {:name "GLSL"
   :scope-name "source.glsl"
   :indent {:begin #"^.*\{[^}\"\']*$|^.*\([^\)\"\']*$|^\s*\{\}$"
            :end #"^\s*(\s*/[*].*[*]/\s*)*\}|^\s*(\s*/[*].*[*]/\s*)*\)"}
   :line-comment "//"
   :patterns [{:captures {1 {:name "storage.type.glsl"}
                          2 {:name "entity.name.function.glsl"}}
               :match #"^([a-zA-Z_][\w\s]*)\s+([a-zA-Z_]\w*)(?=\s*\()"
               :name "meta.function.glsl"}
              {:name #"comment.line.block.glsl"
               :begin #"/\*"
               :end #"\*/"}
              {:name "comment.line.double-slash.glsl"
               :begin #"//"
               :end #"$"}
              {:match #"<=|>=|!=|[<>=|&+-]{1,2}|[*/!^~?:]|\bdefined\b"
               :name "keyword.operator.glsl"}
              {:match #"\b(break|case|continue|default|discard|do|else|for|if|return|switch|while)\b"
               :name "keyword.control.glsl"}
              {:match #"^\s*#\s*(define|elif|else|endif|error|extension|if|ifdef|ifndef|line|pragma|undef|version)\b"
               :name "keyword.preprocessor.glsl"}
              {:match #"\b(bool|bvec[2-4]|dmat[2-4]|dmat[2-4]x[2-4]|double|dvec[2-4]|float|int|isampler2DMS|isampler2DMSArray|isampler2DRect|isamplerBuffer|isamplerCube|isampler[1-3]D|isampler[12]DArray|ivec[2-4]|mat[2-4]|mat[2-4]x[2-4]|sampler2DMS|sampler2DMSArray|sampler2DRect|sampler2DRectShadow|samplerBuffer|samplerCube|sampler[1-3]D|sampler[12]DArray|sampler[12]DArrayShadow|sampler[12]DShadow|struct|uint|usampler2DMS|usampler2DMSArray|usampler2DRect|usamplerBuffer|usamplerCube|usampler[1-3]D|usampler[12]DArray|uvec[2-4]|vec[2-4]|void)\b"
               :name "storage.type.glsl"}
              {:match #"\b(attribute|buffer|centroid|const|flat|highp|in|inout|invariant|lowp|mediump|noperspective|out|patch|precision|sample|shared|smooth|uniform|varying)\b"
               :name "storage.modifier.glsl"}
              {:match #"\b(gl_BackColor|gl_BackLightModelProduct|gl_BackLightProduct|gl_BackMaterial|gl_BackSecondaryColor|gl_ClipDistance|gl_ClipPlane|gl_ClipVertex|gl_Color|gl_DepthRange|gl_DepthRangeParameters|gl_EyePlaneQ|gl_EyePlaneR|gl_EyePlaneS|gl_EyePlaneT|gl_Fog|gl_FogCoord|gl_FogFragCoord|gl_FogParameters|gl_FragColor|gl_FragCoord|gl_FragData|gl_FragDepth|gl_FrontColor|gl_FrontFacing|gl_FrontLightModelProduct|gl_FrontLightProduct|gl_FrontMaterial|gl_FrontSecondaryColor|gl_InstanceID|gl_Layer|gl_LightModel|gl_LightModelParameters|gl_LightModelProducts|gl_LightProducts|gl_LightSource|gl_LightSourceParameters|gl_MaterialParameters|gl_ModelViewMatrix|gl_ModelViewMatrixInverse|gl_ModelViewMatrixInverseTranspose|gl_ModelViewMatrixTranspose|gl_ModelViewProjectionMatrix|gl_ModelViewProjectionMatrixInverse|gl_ModelViewProjectionMatrixInverseTranspose|gl_ModelViewProjectionMatrixTranspose|gl_MultiTexCoord[0-7]|gl_Normal|gl_NormalMatrix|gl_NormalScale|gl_ObjectPlaneQ|gl_ObjectPlaneR|gl_ObjectPlaneS|gl_ObjectPlaneT|gl_Point|gl_PointCoord|gl_PointParameters|gl_PointSize|gl_Position|gl_PrimitiveIDIn|gl_ProjectionMatrix|gl_ProjectionMatrixInverse|gl_ProjectionMatrixInverseTranspose|gl_ProjectionMatrixTranspose|gl_SecondaryColor|gl_TexCoord|gl_TextureEnvColor|gl_TextureMatrix|gl_TextureMatrixInverse|gl_TextureMatrixInverseTranspose|gl_TextureMatrixTranspose|gl_Vertex|gl_VertexID)\b"
               :name "support.variable.glsl"}
              {:match #"\b(gl_MaxClipPlanes|gl_MaxCombinedTextureImageUnits|gl_MaxDrawBuffers|gl_MaxFragmentUniformComponents|gl_MaxLights|gl_MaxTextureCoords|gl_MaxTextureImageUnits|gl_MaxTextureUnits|gl_MaxVaryingFloats|gl_MaxVertexAttribs|gl_MaxVertexTextureImageUnits|gl_MaxVertexUniformComponents)\b"
               :name "support.constant.glsl"}
              {:match #"\b(abs|acos|all|any|asin|atan|ceil|clamp|cos|cross|dFdx|dFdy|degrees|distance|dot|equal|exp|exp2|faceforward|floor|fract|ftransform|fwidth|greaterThan|greaterThanEqual|inversesqrt|length|lessThan|lessThanEqual|log|log2|matrixCompMult|max|min|mix|mod|noise[1-4]|normalize|not|notEqual|outerProduct|pow|radians|reflect|refract|shadow1D|shadow1DLod|shadow1DProj|shadow1DProjLod|shadow2D|shadow2DLod|shadow2DProj|shadow2DProjLod|sign|sin|smoothstep|sqrt|step|tan|texture1D|texture1DLod|texture1DProj|texture1DProjLod|texture2D|texture2DLod|texture2DProj|texture2DProjLod|texture3D|texture3DLod|texture3DProj|texture3DProjLod|textureCube|textureCubeLod|transpose)(?=\s*\()"
               :name "support.function.glsl"}
              {:match #"\b([A-Za-z_]\w*)(?=\s*\()"
               :name "support.function.any.glsl"}
              {:match #"\b(false|true)\b"
               :name "constant.language.glsl"}
              {:match #"\b(0x[0-9A-Fa-f]+)\b"
               :name "constant.numeric.hex.glsl"}
              {:match #"\b[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?(lf|LF|[fFuU])?\b"
               :name "constant.numeric.glsl"}
              {:match #"\b(asm|enum|extern|goto|inline|long|short|sizeof|static|typedef|union|unsigned|volatile)\b"
               :name "invalid.illegal.glsl"}]})

(def ^:private glsl-opts {:code {:grammar glsl-grammar}})

(def shader-defs [{:ext "vp"
                   :label "Vertex Program"
                   :icon "icons/32/Icons_32-Vertex-shader.png"
                   :view-types [:code :default]
                   :view-opts glsl-opts}
                  {:ext "fp"
                   :label "Fragment Program"
                   :icon "icons/32/Icons_33-Fragment-shader.png"
                   :view-types [:code :default]
                   :view-opts glsl-opts}])

(defn- make-full-source ^String [resource-ext lines]
  (string/join "\n" (if-some [compat-directive-lines (get shader/compat-directives resource-ext)]
                      (shader/insert-directives lines compat-directive-lines)
                      lines)))

(defn- shader-type-from-str [^String shader-type-in]
  (case shader-type-in
    "int" :shader-type-int
    "uint" :shader-type-uint
    "float" :shader-type-float
    "vec2" :shader-type-vec2
    "vec3" :shader-type-vec3
    "vec4" :shader-type-vec4
    "mat2" :shader-type-mat2
    "mat3" :shader-type-mat3
    "mat4" :shader-type-mat4
    "sampler2D" :shader-type-sampler2d
    "sampler3D" :shader-type-sampler3d
    "samplerCube" :shader-type-sampler-cube
    :shader-type-unknown))

(defn- shader-stage-from-ext
  ^ShaderUtil$ES2ToES3Converter$ShaderType [^String resource-ext]
  (case resource-ext
    "fp" ShaderUtil$ES2ToES3Converter$ShaderType/FRAGMENT_SHADER
    "vp" ShaderUtil$ES2ToES3Converter$ShaderType/VERTEX_SHADER))

(defn- error-string->error-value [^String error-string]
  (g/error-fatal (string/trim error-string)))

(defn- shader-resource->map [^ShaderUtil$SPIRVReflector$Resource shader-resource]
  {:name (.name shader-resource)
   :type (shader-type-from-str (.type shader-resource))
   :set (.set shader-resource)
   :binding (.binding shader-resource)})

(defn- make-glsl-shader [^String glsl-source]
  {:language :language-glsl
   :source (ByteString/copyFrom (.getBytes glsl-source "UTF-8"))})

(defn- make-spirv-shader [^String glsl-source resource-ext resource-path]
  (let [shader-stage (shader-stage-from-ext resource-ext)
        spirv-compile-result (ShaderProgramBuilder/compileGLSLToSPIRV glsl-source shader-stage resource-path "" false true)
        compile-warnings (. spirv-compile-result compile-warnings)]
    (if (seq compile-warnings)
      (mapv error-string->error-value compile-warnings)
      {:language :language-spirv
       :source (ByteString/copyFrom (. spirv-compile-result source))
       :uniforms (mapv shader-resource->map (. spirv-compile-result resource-list))
       :attributes (mapv shader-resource->map (. spirv-compile-result attributes))})))

(defn- build-shader [resource _dep-resources user-data]
  (let [{:keys [compile-spirv resource-ext lines]} user-data
        full-source (make-full-source resource-ext lines)
        resource-path (resource/path resource)
        spirv-shader-or-errors-or-nil (when compile-spirv
                                        (make-spirv-shader full-source resource-ext resource-path))]
    (g/precluding-errors spirv-shader-or-errors-or-nil
      (let [glsl-shader (make-glsl-shader full-source)
            shaders (filterv some? [glsl-shader spirv-shader-or-errors-or-nil])
            shader-desc {:shaders shaders}
            content (protobuf/map->bytes Graphics$ShaderDesc shader-desc)]
        {:resource resource
         :content content}))))

(g/defnk produce-build-targets [_node-id compile-spirv lines resource]
  [(bt/with-content-hash
     {:node-id _node-id
      :resource (workspace/make-build-resource resource)
      :build-fn build-shader
      :user-data {:compile-spirv compile-spirv
                  :lines lines
                  :resource-ext (resource/type-ext resource)}})])

(g/defnk produce-full-source [resource lines]
  (make-full-source (resource/type-ext resource) lines))

(g/defnode ShaderNode
  (inherits r/CodeEditorResourceNode)

  (input project-settings g/Any)

  (output compile-spirv g/Bool (g/fnk [project-settings]
                                 (get project-settings ["shader" "output_spirv"] false)))

  (output build-targets g/Any :cached produce-build-targets)
  (output full-source g/Str :cached produce-full-source))

(defn- additional-load-fn [project self _resource]
  (g/connect project :settings self :project-settings))

(defn register-resource-types [workspace]
  (for [def shader-defs
        :let [args (assoc def
                     :node-type ShaderNode
                     :eager-loading? true
                     :additional-load-fn additional-load-fn)]]
    (apply r/register-code-resource-type workspace (mapcat identity args))))
