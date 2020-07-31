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

(ns editor.pipeline.tex-gen
  (:require [editor.protobuf :as protobuf]
            [internal.util :as util])
  (:import [com.dynamo.bob TexcLibrary$FlipAxis]
           [com.dynamo.bob.pipeline TextureGenerator]
           [com.dynamo.bob.util TextureUtil]
           [com.dynamo.graphics.proto Graphics$TextureImage Graphics$TextureProfile Graphics$TextureProfiles]
           [com.google.protobuf ByteString]
           [java.awt.image BufferedImage]
           [java.io ByteArrayOutputStream]
           [java.util EnumSet]))

(set! *warn-on-reflection* true)

(def ^:private texture-format->editor-format
  {:texture-format-luminance         :texture-format-luminance
   :texture-format-rgb               :texture-format-rgb
   :texture-format-rgba              :texture-format-rgba
   :texture-format-rgb-pvrtc-2bppv1  :texture-format-rgb
   :texture-format-rgb-pvrtc-4bppv1  :texture-format-rgb
   :texture-format-rgba-pvrtc-2bppv1 :texture-format-rgba
   :texture-format-rgba-pvrtc-4bppv1 :texture-format-rgba
   :texture-format-rgb-etc1          :texture-format-rgb
   :texture-format-rgb-16bpp         :texture-format-rgb
   :texture-format-rgba-16bpp        :texture-format-rgba

   ;; This is incorrect, but it seems like jogl does not define or
   ;; support a pixelformat of L8A8. So we use RGBA instead to at
   ;; least previewing with alpha.
   :texture-format-luminance-alpha   :texture-format-rgba})

(defn match-texture-profile
  [texture-profiles ^String path]
  (let [texture-profiles-data (some->> texture-profiles (protobuf/map->pb Graphics$TextureProfiles))
        path (if (.startsWith path "/") (subs path 1) path)]
    (when-some [texture-profile (TextureUtil/getTextureProfileByPath texture-profiles-data path)]
      (protobuf/pb->map texture-profile))))

(defn make-texture-image
  (^Graphics$TextureImage [^BufferedImage image texture-profile]
   (make-texture-image image texture-profile false))
  (^Graphics$TextureImage [^BufferedImage image texture-profile compress?]
   (make-texture-image image texture-profile compress? true))
  (^Graphics$TextureImage [^BufferedImage image texture-profile compress? flip-y?]
   (let [^Graphics$TextureProfile texture-profile-data (some->> texture-profile (protobuf/map->pb Graphics$TextureProfile))]
     (TextureGenerator/generate image texture-profile-data ^boolean compress? (if ^boolean flip-y? (EnumSet/of TexcLibrary$FlipAxis/FLIP_AXIS_Y) (EnumSet/noneOf TexcLibrary$FlipAxis))))))

(defn- make-preview-profile
  "Given a texture-profile, return a simplified texture-profile that can be used
  for previewing purposes in editor. Will only produce data for one texture
  format."
  [{:keys [name platforms] :as texture-profile}]
  (let [platform-profile (or (first (filter #(= :os-id-generic (:os %)) (:platforms texture-profile)))
                             (first (:platforms texture-profile)))
        texture-format   (first (:formats platform-profile))]
    (when (and platform-profile texture-format)
      {:name      "editor"
       :platforms [{:os                :os-id-generic
                    :formats           [(update texture-format :format texture-format->editor-format)]
                    :mipmaps           (:mipmaps platform-profile)
                    :max-texture-size  (:max-texture-size platform-profile)
                    :premultiply-alpha (:premultiply-alpha platform-profile)}]})))

(defn make-preview-texture-image
  ^Graphics$TextureImage [^BufferedImage image texture-profile]
  (let [preview-profile (make-preview-profile texture-profile)]
    (make-texture-image image preview-profile false)))

(defn make-cubemap-texture-images
  ^Graphics$TextureImage [images texture-profile compress?]
  (let [^Graphics$TextureProfile texture-profile-data (some->> texture-profile (protobuf/map->pb Graphics$TextureProfile))]
    (util/map-vals #(make-texture-image % texture-profile compress? false) images)))

(defn- make-cubemap-texture-image-alternatives [textures]
  ;; Cube map textures are in the order px nx py ny nz pz.
  ;; Size of texture[i].alternative[k] == texture[j].alternative[k]
  ;; So mip map sizes & offsets also equal
  ;; Format of alternative data n is:
  ;; [<cube map textures' alternative n at mip level 0> <cube map textures' alternative n at mip level 1> ... <... at final mip level>]
  (for [alt (range 0 (count (:alternatives (first textures))))]
    (let [template-alternative (-> textures first :alternatives (nth alt))
          data (ByteArrayOutputStream.)]
      (doseq [mipmap (range 0 (count (:mip-map-size template-alternative))) ; mip-map-size always uncompressed
              :let [mip-size (-> template-alternative :mip-map-size (nth mipmap))
                    mip-offset (-> template-alternative :mip-map-offset (nth mipmap))
                    mip-buf (byte-array mip-size)]
              texture (range 0 (count textures))
              :let [^ByteString alt-data (-> textures (nth texture) :alternatives (nth alt) :data)]]
        (.copyTo alt-data mip-buf mip-offset 0 mip-size)
        (.write data mip-buf))
      (-> template-alternative
          (assoc :data (ByteString/copyFrom (.toByteArray data)))
          ;; thought we would have to update mip-map-size also, but CubemapBuilder doesn't so...
          (update :mip-map-offset (fn [offsets] (map #(* 6 %) offsets)))))))

(defn assemble-cubemap-texture-images
  ^Graphics$TextureImage
  [texture-images]
  ;; FIXME: in graphics_opengl.cpp(1870ish) in the engine, we set the cubemap textures
  ;; using glTexSubImage2D in the order +x, -x, +y, -y, and then ***-z***, ***+z***
  ;; until this is fixed, if ever, we flip the order as below
  (let [textures (mapv protobuf/pb->map ((juxt :px :nx :py :ny :nz :pz) texture-images))
        alternatives (make-cubemap-texture-image-alternatives textures)
        template-texture (first textures)]
    (-> template-texture
        (assoc :count 6)
        (assoc :type :type-cubemap)
        (assoc :alternatives alternatives)
        (#(protobuf/map->pb Graphics$TextureImage %)))))

(defn make-preview-cubemap-texture-images
  ^Graphics$TextureImage [images texture-profile]
  (let [preview-profile (make-preview-profile texture-profile)]
    (make-cubemap-texture-images images preview-profile false)))

