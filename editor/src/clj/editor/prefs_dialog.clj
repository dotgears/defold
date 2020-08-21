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

(ns editor.prefs-dialog
  (:require [clojure.java.io :as io]
            [service.log :as log]
            [editor.ui :as ui]
            [editor.prefs :as prefs]
            [editor.system :as system]
            [editor.engine :as engine]
            [editor.engine.native-extensions :as native-extensions])
  (:import [com.defold.control LongField]
           [javafx.scene Parent Scene]
           [javafx.scene.paint Color]
           [javafx.scene.control Button ColorPicker Control CheckBox ChoiceBox Label TextField Tab TabPane]
           [javafx.scene.layout AnchorPane GridPane]
           [javafx.scene.input KeyCode KeyEvent]
           [javafx.stage Stage Modality DirectoryChooser FileChooser]
           [javafx.util StringConverter]))

(set! *warn-on-reflection* true)

(defmulti create-control! (fn [prefs grid desc] (:type desc)))

(defn- create-generic [^Class class prefs grid desc]
  (let [control (.newInstance class)
        commit (fn [] (prefs/set-prefs prefs (:key desc) (ui/value control)))]
    (ui/value! control (prefs/get-prefs prefs (:key desc) (:default desc)))
    (ui/on-focus! control (fn [focus] (when-not focus (commit))))
    (ui/on-action! control (fn [e] (commit)))
    control))

(defmethod create-control! :boolean [prefs grid desc]
  (create-generic CheckBox prefs grid desc))

(defmethod create-control! :color [prefs grid desc]
  (create-generic ColorPicker prefs grid desc))

(defmethod create-control! :string [prefs grid desc]
  (create-generic TextField prefs grid desc))

(defmethod create-control! :long [prefs grid desc]
  (create-generic LongField prefs grid desc))

(defmethod create-control! :choicebox [prefs grid desc]
  (let [control (ChoiceBox.)
        options (:options desc)
        options-map (apply hash-map (flatten options))
        inv-options-map (clojure.set/map-invert options-map)]
    (.setConverter control
      (proxy [StringConverter] []
        (toString [value]
          (get options-map value))
        (fromString [s]
          (inv-options-map s))))
    (.addAll (.getItems control) ^java.util.Collection (map first options))
    (.select (.getSelectionModel control) (prefs/get-prefs prefs (:key desc) (:default desc)))

    (ui/observe (.valueProperty control) (fn [observable old-val new-val]
                                           (prefs/set-prefs prefs (:key desc) new-val)))
    control))

(defn- create-prefs-row! [prefs ^GridPane grid row desc]
  (let [label (Label. (str (:label desc) ":"))
        ^Parent control (create-control! prefs grid desc)]
    (GridPane/setConstraints label 1 row)
    (GridPane/setConstraints control 2 row)

    (.add (.getChildren grid) label)
    (.add (.getChildren grid) control)))

(defn- add-page! [prefs ^TabPane pane page-desc]
  (let [tab (Tab. (:name page-desc))
        grid (GridPane.)]
    (.setHgap grid 4)
    (.setVgap grid 6)
    (.setContent tab grid)
    (doall (map-indexed (fn [i desc] (create-prefs-row! prefs grid i desc))
                        (:prefs page-desc)))
    (ui/add-style! grid "prefs")
    (.add (.getTabs pane) tab)))

(defn- pref-pages
  []
  (cond-> [{:name  "General"
            :prefs [{:label "Enable Texture Compression" :type :boolean :key "general-enable-texture-compression" :default false}
                    {:label "Escape Quits Game" :type :boolean :key "general-quit-on-esc" :default false}
                    {:label "Track Active Tab in Asset Browser" :type :boolean :key "asset-browser-track-active-tab?" :default false}]}
           {:name  "Code"
            :prefs [{:label "Custom Editor" :type :string :key "code-custom-editor" :default ""}
                    {:label "Open File" :type :string :key "code-open-file" :default "{file}"}
                    {:label "Open File at Line" :type :string :key "code-open-file-at-line" :default "{file}:{line}"}]}
           {:name  "Extensions"
            :prefs [{:label "Build Server" :type :string :key "extensions-server" :default native-extensions/defold-build-server-url}]}]
    
    (system/defold-dev?)
    (conj {:name "Dev"
           :prefs [{:label "Custom Engine" :type :string :key engine/custom-engine-pref-key :default ""}]})))

(defn open-prefs [preferences]
  (let [root ^Parent (ui/load-fxml "prefs.fxml")
        stage (ui/make-dialog-stage (ui/main-stage))
        scene (Scene. root)]

    (ui/with-controls root [^TabPane prefs]
      (doseq [p (pref-pages)]
        (add-page! preferences prefs p)))

    (ui/title! stage "Preferences")
    (.setScene stage scene)

    (.addEventFilter scene KeyEvent/KEY_PRESSED
                     (ui/event-handler event
                                       (let [code (.getCode ^KeyEvent event)]
                                         (when (= code KeyCode/ESCAPE)
                                           (.close stage)))))

    (ui/show-and-wait! stage)))
