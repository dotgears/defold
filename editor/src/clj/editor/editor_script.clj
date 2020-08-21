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

(ns editor.editor-script
  (:require [clojure.string :as string]
            [dynamo.graph :as g]
            [editor.code.resource :as r]
            [editor.code.script :as script]
            [editor.luart :as luart]
            [editor.resource :as resource]))

(g/defnk produce-prototype [_node-id lines resource]
  (try
    (luart/read (string/join "\n" lines) (resource/resource->proj-path resource))
    (catch Exception e
      (g/->error _node-id :prototype :fatal e "Could not compile editor extension"))))

(g/defnode EditorScript
  (inherits r/CodeEditorResourceNode)
  (input globals g/Any)
  (output prototype g/Any :cached produce-prototype))

(defn register-resource-types [workspace]
  (r/register-code-resource-type workspace
                                 :ext "editor_script"
                                 :label "Editor Script"
                                 :icon "icons/32/Icons_29-AT-Unknown.png"
                                 :view-types [:code :default]
                                 :view-opts script/lua-code-opts
                                 :node-type EditorScript
                                 :eager-loading? true
                                 :additional-load-fn
                                 (fn [project self resource]
                                   (let [extensions (g/node-value project :editor-extensions)
                                         target (if (resource/file-resource? resource)
                                                  :project-prototypes
                                                  :library-prototypes)]
                                     (g/connect self :prototype extensions target)))))
