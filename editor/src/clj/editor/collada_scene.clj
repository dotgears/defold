(ns editor.collada-scene
  (:require [clojure.java.io :as io]
            [clojure.xml :as xml]
            [clojure.string :as str]
            [editor.protobuf :as protobuf]
            [dynamo.graph :as g]
            [clojure.data.json :as json]
            [editor.gl :as gl]
            [editor.gl.shader :as shader]
            [editor.gl.vertex :as vtx]
            [editor.gl.texture :as texture]
            [editor.defold-project :as project]
            [editor.scene :as scene]
            [editor.workspace :as workspace]
            [editor.gl.pass :as pass]
            [editor.geom :as geom]
            [editor.render :as render]
            [editor.rig :as rig]
            [editor.collada :as collada])
  (:import [com.dynamo.rig.proto Rig$MeshSet]
           [editor.types AABB]
           [com.jogamp.opengl GL GL2]))

(set! *warn-on-reflection* true)

(def mesh-icon "icons/32/Icons_27-AT-Mesh.png")

(vtx/defvertex vtx-pos-nrm-tex
  (vec3 position)
  (vec3 normal)
  (vec2 texcoord0))

(shader/defshader shader-ver-pos-nrm-tex
  (attribute vec4 position)
  (attribute vec3 normal)
  (attribute vec2 texcoord0)
  (varying vec3 var_normal)
  (varying vec2 var_texcoord0)
  (defn void main []
    (setq gl_Position (* gl_ModelViewProjectionMatrix position))
    (setq var_texcoord0 texcoord0)
    (setq var_normal normal)))

(shader/defshader shader-frag-pos-nrm-tex
  (varying vec3 var_normal)
  (varying vec2 var_texcoord0)
  (uniform sampler2D texture)
  (defn void main []
    (setq gl_FragColor (vec4 (* (.xyz (texture2D texture var_texcoord0.xy)) var_normal.z) 1.0))))

(shader/defshader shader-frag-pos-nrm-tex-passthrough
  (varying vec3 var_normal)
  (varying vec2 var_texcoord0)
  (defn void main []
    (setq gl_FragColor (vec4 1.0 1.0 1.0 1.0))))

(def shader-pos-nrm-tex (shader/make-shader ::shader shader-ver-pos-nrm-tex shader-frag-pos-nrm-tex))
(def shader-pos-nrm-tex-passthrough (shader/make-shader ::shader shader-ver-pos-nrm-tex shader-frag-pos-nrm-tex-passthrough))

(defn render-mesh [^GL2 gl render-args renderables rcount]
  (let [pass (:pass render-args)
        renderable (first renderables)
        node-id (:node-id renderable)
        user-data (:user-data renderable)
        vbs (:vbs user-data)
        shader (:shader user-data)
        textures (:textures user-data)]
    (cond
      (= pass pass/outline)
      (let [outline-vertex-binding (vtx/use-with [node-id ::outline] (render/gen-outline-vb renderables rcount) render/shader-outline)]
        (gl/with-gl-bindings gl render-args [render/shader-outline outline-vertex-binding]
          (gl/gl-draw-arrays gl GL/GL_LINES 0 (* rcount 8))))

      (= pass pass/opaque)
      (doseq [vb vbs]
        (let [vertex-binding (vtx/use-with [node-id ::mesh] vb shader)]
          (gl/with-gl-bindings gl render-args [shader vertex-binding]
            (doseq [[name t] textures]
              (gl/bind gl t render-args)
              (shader/set-uniform shader gl name (- (:unit t) GL/GL_TEXTURE0)))
            (gl/gl-enable gl GL/GL_CULL_FACE)
            (gl/gl-cull-face gl GL/GL_BACK)
            (.glBlendFunc gl GL/GL_ONE GL/GL_ONE_MINUS_SRC_ALPHA)
            (gl/gl-draw-arrays gl GL/GL_TRIANGLES 0 (count vb))
            (gl/gl-disable gl GL/GL_CULL_FACE)
            (.glBlendFunc gl GL/GL_SRC_ALPHA GL/GL_ONE_MINUS_SRC_ALPHA)
            (doseq [[name t] textures]
              (gl/unbind gl t)))))

      (= pass pass/selection)
      (doseq [vb vbs]
        (let [vertex-binding (vtx/use-with [node-id ::mesh] vb shader-pos-nrm-tex-passthrough)]
          (gl/with-gl-bindings gl render-args [shader-pos-nrm-tex-passthrough vertex-binding]
            (gl/gl-enable gl GL/GL_CULL_FACE)
            (gl/gl-cull-face gl GL/GL_BACK)
            (gl/gl-draw-arrays gl GL/GL_TRIANGLES 0 (count vb))
            (gl/gl-disable gl GL/GL_CULL_FACE)))))))

(defn- build-mesh [self basis resource dep-resources user-data]
  (let [content (:content user-data)]
    {:resource resource :content (protobuf/map->bytes Rig$MeshSet content)}))

(g/defnk produce-build-targets [_node-id resource content]
  [{:node-id _node-id
    :resource (workspace/make-build-resource resource)
    :build-fn build-mesh
    :user-data {:content content}}])

(g/defnk produce-content [resource]
  (collada/->mesh-set (io/input-stream resource)))

(g/defnk produce-scene [_node-id aabb vbs]
  {:node-id _node-id
   :aabb aabb
   :renderable {:render-fn render-mesh
                :batch-key _node-id
                :select-batch-key _node-id
                :user-data {:vbs vbs
                            :shader shader-pos-nrm-tex
                            :textures {"texture" texture/white-pixel}}
                :passes [pass/opaque pass/selection pass/outline]}})

(defn- mesh->vb [m]
  (let [vcount (count (:positions m))]
    (when (> vcount 0)
      (loop [vb (->vtx-pos-nrm-tex vcount)
             ps (partition 3 (:positions m))
             ns (partition 3 (:normals m))
             ts (partition 2 (:texcoord0 m))]
        (if-let [[x y z] (first ps)]
          (let [[nx ny nz] (first ns)
                [tu tv] (first ts)]
            (recur (conj! vb [x y z nx ny nz tu tv]) (rest ps) (rest ns) (rest ts)))
          (persistent! vb))))))

(g/defnode ColladaSceneNode
  (inherits project/ResourceNode)

  (output content g/Any :cached produce-content)
  (output aabb AABB :cached (g/fnk [content]
                                   (reduce (fn [aabb [x y z]]
                                             (geom/aabb-incorporate aabb x y z))
                                           (geom/null-aabb)
                                           (partition 3 (get-in content [:components 0 :positions])))))
  (output build-targets g/Any :cached produce-build-targets)
  (output vbs g/Any :cached (g/fnk [content]
                                   (loop [meshes (mapcat :meshes (:mesh-entries content))
                                          vbs []]
                                     (if-let [c (first meshes)]
                                       (let [vb (mesh->vb c)]
                                         (recur (rest meshes) (if vb (conj vbs vb) vbs)))
                                       vbs))))
  (output scene g/Any :cached produce-scene))

(defn register-resource-types [workspace]
  (workspace/register-resource-type workspace
                                    :ext "dae"
                                    :label "Collada Scene"
                                    :build-ext "meshsetc"
                                    :node-type ColladaSceneNode
                                    :icon mesh-icon
                                    :view-types [:scene :text]))