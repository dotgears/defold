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

(ns integration.sprite-test
  (:require [clojure.test :refer :all]
            [dynamo.graph :as g]
            [editor.workspace :as workspace]
            [editor.defold-project :as project]
            [editor.tile-source :as tile-source]
            [editor.types :as types]
            [editor.properties :as properties]
            [integration.test-util :as test-util]))

(deftest replacing-sprite-image-replaces-dep-build-targets
  (test-util/with-loaded-project
    (let [node-id (project/get-resource-node project "/logic/session/pow.sprite")
          old-image (g/node-value node-id :image)]
      (let [old-sources (g/sources-of node-id :dep-build-targets)]
        (g/transact (g/set-property node-id :image (workspace/find-resource workspace "/switcher/switcher.atlas")))
        (is (= (count old-sources) (count (g/sources-of node-id :dep-build-targets))))
        (is (not (= (set old-sources) (set (g/sources-of node-id :dep-build-targets)))))
        (g/transact (g/set-property node-id :image old-image))
        (is (= (count old-sources) (count (g/sources-of node-id :dep-build-targets))))
        (is (= (set old-sources) (set (g/sources-of node-id :dep-build-targets))))))))

(deftest sprite-validation
  (test-util/with-loaded-project
    (let [node-id (project/get-resource-node project "/sprite/atlas.sprite")]
      (testing "unknown atlas"
               (test-util/with-prop [node-id :image (workspace/resolve-workspace-resource workspace "/graphics/unknown_atlas.atlas")]
                 (is (g/error? (test-util/prop-error node-id :image)))))
      (testing "invalid atlas"
               (test-util/with-prop [node-id :image (workspace/resolve-workspace-resource workspace "/graphics/img_not_found.atlas")]
                 (is (g/error? (test-util/prop-error node-id :image))))))))

(deftest sprite-scene
  (test-util/with-loaded-project
    (let [node-id (project/get-resource-node project "/sprite/atlas.sprite")]
      (test-util/test-uses-assigned-material workspace project node-id
                                             :material
                                             [:renderable :user-data :shader]
                                             [:renderable :user-data :gpu-texture]))))
