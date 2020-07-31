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

(ns editor.form
  (:require [editor.util :as util]))

(set! *warn-on-reflection* true)

(defn set-value! [{:keys [user-data set]} path value]
  (set user-data path value))

(defn can-clear? [{:keys [clear]}]
  (not (nil? clear)))

(defn clear-value! [{:keys [user-data clear]} path]
  (clear user-data path))

(def ^:private type-defaults
  {:table []
   :string ""
   :resource nil
   :boolean false
   :integer 0
   :number 0.0
   :vec4 [0.0 0.0 0.0 0.0]
   :2panel [],
   :list []})

(defn has-default? [field-info]
  (or (contains? field-info :default)
      (contains? type-defaults (:type field-info))))

(defn field-default [field-info]
  (if (contains? field-info :default)
    (:default field-info)
    (type-defaults (:type field-info))))

(defn optional-field?
  "Whether this field can be cleared"
  [field-info]
  (:optional field-info))

(defn field-defaults [fields]
  (let [required-fields (remove optional-field? fields)]
    (when (every? has-default? required-fields)
      (reduce (fn [val field]
                (assoc-in val (:path field) (field-default field)))
              {}
              required-fields))))

(defn section-defaults [section]
  (field-defaults (:fields section)))

(defn table-row-defaults [table-field-info]
  (field-defaults (:columns table-field-info)))

(defn form-defaults [form]
  (let [sections-defaults (map section-defaults (:sections form))]
    (when-not (some nil? sections-defaults)
      (reduce merge
              {}
              sections-defaults))))

(defn two-panel-defaults [panel-field-info]
  (let [panel-key-defaults (field-defaults [(:panel-key panel-field-info)])
        panel-form-defaults (form-defaults (:panel-form panel-field-info))]
    (when (and panel-key-defaults panel-form-defaults)
      (merge panel-key-defaults panel-form-defaults))))

(defn update-section-setting [section path f]
  (if-let [index (first (util/positions #(= path (:path %)) (get section :fields)))]
    (update-in section [:fields index] f)
    section))

(defn update-form-setting [form-data path f]
  (update form-data :sections (fn [section] (mapv #(update-section-setting % path f) section))))
