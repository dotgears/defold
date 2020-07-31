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

(ns editor.bundle-dialog
  (:require [clojure.java.io :as io]
            [clojure.set :as set]
            [editor.bundle :as bundle]
            [editor.dialogs :as dialogs]
            [editor.fs :as fs]
            [editor.fxui :as fxui]
            [editor.handler :as handler]
            [editor.prefs :as prefs]
            [editor.system :as system]
            [editor.ui :as ui])
  (:import [java.io File]
           [javafx.scene Scene]
           [javafx.scene.control Button CheckBox ChoiceBox Label TextField]
           [javafx.scene.input KeyCode]
           [javafx.scene.layout ColumnConstraints GridPane HBox Priority VBox]
           [javafx.stage DirectoryChooser Window]
           [javafx.util StringConverter]))

(set! *warn-on-reflection* true)

(defonce ^:private os-32-bit? (= (system/os-arch) "x86"))

(defn- query-directory!
  ^File [title ^File initial-directory ^Window owner-window]
  (let [chooser (DirectoryChooser.)]
    (when (some-> initial-directory .isDirectory)
      (.setInitialDirectory chooser initial-directory))
    (.setTitle chooser title)
    (.showDialog chooser owner-window)))

(defn- query-overwrite!
  [^File existing-entry ^Window owner-window]
  (let [writable? (try
                    (and (.isDirectory existing-entry)
                         (.canWrite existing-entry))
                    (catch SecurityException _
                      false))]
    (if-not writable?
      (do (dialogs/make-info-dialog
            {:title "Cannot Overwrite"
             :icon :icon/triangle-error
             :header "Cannot create a directory"
             :content {:text (str "Cannot create directory at \"" (.getAbsolutePath existing-entry) "\". You might not have permission to write to that directory, or there might be a file with the same name as the directory we're trying to create.")
                       :wrap-text true}})
          false)
      (dialogs/make-confirmation-dialog
        {:title "Overwrite Existing Directory?"
         :owner owner-window
         :icon :icon/circle-question
         :header {:fx/type :v-box
                  :children [{:fx/type fxui/label
                              :variant :header
                              :text "A directory already exists"}
                             {:fx/type fxui/label
                              :text (format "Overwrite \"%s\"?" (.getAbsolutePath existing-entry))}]}
         :buttons [{:text "Cancel"
                    :cancel-button true
                    :result false}
                   {:text "Overwrite"
                    :default-button true
                    :result true}]}))))

(defn- get-file
  ^File [^TextField text-field]
  (when-let [path (not-empty (.getText text-field))]
    (io/file path)))

(defn- set-file! [^TextField text-field ^File file]
  (let [new-text (if (some? file) (.getAbsolutePath file) "")]
    (when (not= new-text (.getText text-field))
      (.setText text-field new-text))))

(defn- set-choice! [^ChoiceBox choice-box entries selected-entry]
  ;; Selecting nil won't update the ChoiceBox since it thinks it is
  ;; already nil to begin with. Select the first entry here as a workaround.
  (.setItems choice-box (ui/observable-list entries))
  (.selectFirst (.getSelectionModel choice-box))
  (ui/value! choice-box selected-entry))

(defn- existing-file-of-type? [^String ext ^File file]
  (and (fs/existing-file? file)
       (.endsWith (.toLowerCase (.getPath file))
                  (str "." ext))))

(defn- labeled!
  ^HBox [label-text control]
  (assert (string? (not-empty label-text)))
  (HBox/setHgrow control Priority/ALWAYS)
  (let [label (doto (Label. label-text) (ui/add-style! "field-label"))]
    (doto (HBox.)
      (ui/children! [label control]))))

(defn- set-style-class! [stylable key style-classes-by-key]
  (assert (map? style-classes-by-key))
  (assert (or (nil? key) (contains? style-classes-by-key key)))
  (ui/remove-styles! stylable (vals style-classes-by-key))
  (when-let [style-class (style-classes-by-key key)]
    (ui/add-style! stylable style-class)))

(defn- set-label-status! [label [severity message]]
  (ui/text! label message)
  (set-style-class! label severity {:fatal "error"
                                    :warning "warning"
                                    :info "info"}))

(defn- set-field-status! [field [severity message]]
  (ui/tooltip! field (not-empty message))
  (set-style-class! field severity {:fatal "error"
                                    :warning "warning"
                                    :info "info"}))

(defn- make-file-field
  ^GridPane [refresh! owner-window text-field-css-id title-text filter-descs]
  (assert (fn? refresh!))
  (assert (some? owner-window))
  (assert (string? (not-empty title-text)))
  (assert (seq filter-descs))
  (assert (every? seq filter-descs))
  (let [text-field (doto (TextField.)
                     (.setId text-field-css-id)
                     (GridPane/setFillWidth true)
                     (GridPane/setConstraints 0 0)
                     (ui/prevent-auto-focus!)
                     (ui/on-action! refresh!)
                     (ui/auto-commit! refresh!))
        browse-button (doto (Button. "\u2026") ; "..." (HORIZONTAL ELLIPSIS)
                        (.setFocusTraversable false)
                        (GridPane/setConstraints 1 0)
                        (ui/add-style! "button-small")
                        (ui/on-action! (fn [event]
                                         (when-let [file (dialogs/make-file-dialog title-text filter-descs (get-file text-field) owner-window)]
                                           (set-file! text-field file)
                                           (refresh! event)))))
        container (doto (GridPane.)
                    (ui/add-style! "composite-property-control-container")
                    (ui/children! [text-field browse-button]))]
    (doto (.getColumnConstraints container)
      (.add (doto (ColumnConstraints.)
              (.setHgrow Priority/ALWAYS)))
      (.add (doto (ColumnConstraints.)
              (.setMinWidth ColumnConstraints/CONSTRAIN_TO_PREF)
              (.setHgrow Priority/NEVER))))
    container))

(defprotocol BundleOptionsPresenter
  (make-views [this owner-window])
  (load-prefs! [this prefs])
  (save-prefs! [this prefs])
  (get-options [this])
  (set-options! [this options]))

(defn- refresh-presenter! [presenter]
  (set-options! presenter (get-options presenter)))

(defn- make-presenter-refresher [presenter]
  (assert (satisfies? BundleOptionsPresenter presenter))
  (let [refresh-in-progress (volatile! false)]
    (fn refresh! [_]
      (when-not @refresh-in-progress
        (vreset! refresh-in-progress true)
        (try
          (refresh-presenter! presenter)
          (finally
            (vreset! refresh-in-progress false)))))))

(defn- get-string-pref
  ^String [prefs key]
  (some-> (prefs/get-prefs prefs key nil) not-empty))

(defn- set-string-pref!
  [prefs key ^String string]
  (prefs/set-prefs prefs key (not-empty string)))

(defn- get-file-pref
  ^File [prefs key]
  (io/file (get-string-pref prefs key)))

(defn- set-file-pref!
  [prefs key ^File file]
  (set-string-pref! prefs key (when file (.getPath file))))

;; -----------------------------------------------------------------------------
;; Generic
;; -----------------------------------------------------------------------------

(def ^:private no-issues-header-info-text "Proceed to select output folder.")

(defn- make-generic-headers
  ^VBox [header]
  (doto (VBox.)
    (ui/add-style! "headers")
    (ui/children! [(doto (Label. header) (ui/add-style! "header-label"))
                   (doto (Label. no-issues-header-info-text) (.setId "header-info-label"))])))

(defn- get-header-status [issues-by-key key-order]
  (assert (map? issues-by-key))
  (assert (sequential? key-order))
  (or (:general issues-by-key)
      (some issues-by-key key-order)
      [:info no-issues-header-info-text]))

(defn- set-generic-headers! [view issues-by-key key-order]
  (ui/with-controls view [header-info-label]
    (set-label-status! header-info-label (get-header-status issues-by-key key-order))))

(defn- make-choice-box
  ^ChoiceBox [refresh! label-value-pairs]
  (assert (fn? refresh!))
  (assert (sequential? (not-empty label-value-pairs)))
  (assert (every? (fn [[k v]] (and (string? (not-empty k)) (string? (not-empty v)))) label-value-pairs))
  (let [values-by-label (into {} label-value-pairs)
        labels-by-value (set/map-invert values-by-label)]
    (doto (ChoiceBox. (ui/observable-list (map second label-value-pairs)))
      (ui/on-action! refresh!)
      (.setConverter (proxy [StringConverter] []
                       (toString [value]
                         (labels-by-value value))
                       (fromString [label]
                         (values-by-label label)))))))

(defn- make-generic-controls [refresh! variant-choices compression-choices]
  (assert (fn? refresh!))
  [(doto (VBox.)
     (ui/add-style! "settings")
     (ui/add-style! "generic")
     (ui/children! [(labeled! "Variant"
                              (doto (make-choice-box refresh! variant-choices)
                                (.setId "variant-choice-box")))
                    (labeled! "Texture Compression"
                              (doto (make-choice-box refresh! compression-choices)
                                (.setId "compression-choice-box")))]))
   (doto (VBox.)
     (ui/add-style! "settings")
     (ui/add-style! "toggles")
     (ui/children! [(doto (CheckBox. "Generate debug symbols") (.setId "generate-debug-symbols-check-box") (.setFocusTraversable false) (ui/on-action! refresh!))
                    (doto (CheckBox. "Generate build report") (.setId "generate-build-report-check-box") (.setFocusTraversable false) (ui/on-action! refresh!))
                    (doto (CheckBox. "Publish Live Update content") (.setId "publish-live-update-content-check-box") (.setFocusTraversable false) (ui/on-action! refresh!))]))])

(defn- load-generic-prefs! [prefs view]
  (ui/with-controls view [variant-choice-box compression-choice-box generate-debug-symbols-check-box generate-build-report-check-box publish-live-update-content-check-box]
    (ui/value! variant-choice-box (prefs/get-prefs prefs "bundle-variant" "debug"))
    (ui/value! compression-choice-box (prefs/get-prefs prefs "bundle-texture-compression" "enabled"))
    (ui/value! generate-debug-symbols-check-box (prefs/get-prefs prefs "bundle-generate-debug-symbols?" true))
    (ui/value! generate-build-report-check-box (prefs/get-prefs prefs "bundle-generate-build-report?" false))
    (ui/value! publish-live-update-content-check-box (prefs/get-prefs prefs "bundle-publish-live-update-content?" false))))

(defn- save-generic-prefs! [prefs view]
  (ui/with-controls view [variant-choice-box compression-choice-box generate-debug-symbols-check-box generate-build-report-check-box publish-live-update-content-check-box]
    (prefs/set-prefs prefs "bundle-variant" (ui/value variant-choice-box))
    (prefs/set-prefs prefs "bundle-texture-compression" (ui/value compression-choice-box))
    (prefs/set-prefs prefs "bundle-generate-debug-symbols?" (ui/value generate-debug-symbols-check-box))
    (prefs/set-prefs prefs "bundle-generate-build-report?" (ui/value generate-build-report-check-box))
    (prefs/set-prefs prefs "bundle-publish-live-update-content?" (ui/value publish-live-update-content-check-box))))

(defn- get-generic-options [view]
  (ui/with-controls view [variant-choice-box compression-choice-box generate-debug-symbols-check-box generate-build-report-check-box publish-live-update-content-check-box]
    {:variant (ui/value variant-choice-box)
     :texture-compression (ui/value compression-choice-box)
     :generate-debug-symbols? (ui/value generate-debug-symbols-check-box)
     :generate-build-report? (ui/value generate-build-report-check-box)
     :publish-live-update-content? (and (ui/value publish-live-update-content-check-box)
                                        (ui/editable publish-live-update-content-check-box))}))

(defn- set-generic-options! [view options workspace]
  (ui/with-controls view [variant-choice-box compression-choice-box generate-debug-symbols-check-box generate-build-report-check-box publish-live-update-content-check-box]
    (ui/value! variant-choice-box (:variant options))
    (ui/value! compression-choice-box (:texture-compression options))
    (ui/value! generate-debug-symbols-check-box (:generate-debug-symbols? options))
    (ui/value! generate-build-report-check-box (:generate-build-report? options))
    (doto publish-live-update-content-check-box
      (ui/value! (:publish-live-update-content? options)))))

(deftype GenericBundleOptionsPresenter [workspace view title platform variant-choices compression-choices]
  BundleOptionsPresenter
  (make-views [this _owner-window]
    (assert (string? (not-empty platform)))
    (let [refresh! (make-presenter-refresher this)]
      (into [(make-generic-headers title)]
            (make-generic-controls refresh! variant-choices compression-choices))))
  (load-prefs! [_this prefs]
    (load-generic-prefs! prefs view))
  (save-prefs! [_this prefs]
    (save-generic-prefs! prefs view))
  (get-options [_this]
    (merge {:platform platform}
           (get-generic-options view)))
  (set-options! [_this options]
    (set-generic-options! view options workspace)))

(defn- make-labeled-check-box
  ^CheckBox [^String label ^String id ^Boolean default-value refresh!]
  (doto (if (some? label)
          (CheckBox. label)
          (CheckBox.))
    (ui/add-style! "labeled-check-box")
    (.setMnemonicParsing false)
    (.setId id)
    (.setFocusTraversable false)
    (ui/on-action! refresh!)
    (ui/value! default-value)))

;; -----------------------------------------------------------------------------
;; Selectable platform
;; -----------------------------------------------------------------------------

(defn- make-platform-controls [refresh! bob-platform-choices]
  (doto (VBox.)
    (ui/add-style! "settings")
    (ui/add-style! "platform")
    (ui/children! [(labeled! "Architecture"
                             (doto (make-choice-box refresh! bob-platform-choices)
                               (.setId "platform-choice-box")))])))

(declare bundle-options-presenter)

(defn- get-bundle-platform-prefs-key [platform]
  (assert (keyword? platform))
  (assert (not= :default platform))
  (assert (some? (get-method bundle-options-presenter platform)))
  (str "bundle-" (name platform) "-platform"))

(defn- load-platform-prefs! [prefs view platform default]
  (ui/with-controls view [platform-choice-box]
    (let [prefs-key (get-bundle-platform-prefs-key platform)]
      (ui/value! platform-choice-box (prefs/get-prefs prefs prefs-key default)))))

(defn- save-platform-prefs! [prefs view platform]
  (ui/with-controls view [platform-choice-box]
    (let [prefs-key (get-bundle-platform-prefs-key platform)]
      (prefs/set-prefs prefs prefs-key (ui/value platform-choice-box)))))

(defn- get-platform-options [view]
  (ui/with-controls view [platform-choice-box]
    {:platform (ui/value platform-choice-box)}))

(defn- set-platform-options! [view options]
  (ui/with-controls view [platform-choice-box]
    (ui/value! platform-choice-box (:platform options))))

(deftype SelectablePlatformBundleOptionsPresenter [workspace view title platform bob-platform-choices bob-platform-default variant-choices compression-choices]
  BundleOptionsPresenter
  (make-views [this _owner-window]
    (let [refresh! (make-presenter-refresher this)]
      (into [(make-generic-headers title)
             (make-platform-controls refresh! bob-platform-choices)]
            (make-generic-controls refresh! variant-choices compression-choices))))
  (load-prefs! [_this prefs]
    (load-generic-prefs! prefs view)
    (load-platform-prefs! prefs view platform bob-platform-default))
  (save-prefs! [_this prefs]
    (save-generic-prefs! prefs view)
    (save-platform-prefs! prefs view platform))
  (get-options [_this]
    (merge (get-generic-options view)
           (get-platform-options view)))
  (set-options! [_this options]
    (set-generic-options! view options workspace)
    (set-platform-options! view options)))

;; -----------------------------------------------------------------------------
;; Android
;; -----------------------------------------------------------------------------

(defn- make-android-controls [refresh! owner-window]
  (assert (fn? refresh!))
  (let [make-file-field (partial make-file-field refresh! owner-window)
        certificate-file-field (make-file-field "certificate-text-field" "Choose Certificate" [["Certificates (*.pem)" "*.pem"]])
        private-key-file-field (make-file-field "private-key-text-field" "Choose Private Key" [["Private Keys (*.pk8)" "*.pk8"]])
        architecture-controls (doto (VBox.)
                                    (ui/children! [(make-labeled-check-box "32-bit (armv7)" "architecture-32bit-check-box" true refresh!)
                                                   (make-labeled-check-box "64-bit (arm64)" "architecture-64bit-check-box" true refresh!)]))]
    (doto (VBox.)
      (ui/add-style! "settings")
      (ui/add-style! "android")
      (ui/children! [(labeled! "Certificate" certificate-file-field)
                     (labeled! "Private key" private-key-file-field)
                     (labeled! "Architectures" architecture-controls)
                     (labeled! "Bundle Format" (doto (make-choice-box refresh! [["APK" "apk"] ["AAB" "aab"]])
                                                (.setId "bundle-format-choice-box")))]))))

(defn- load-android-prefs! [prefs view]
  (ui/with-controls view [certificate-text-field private-key-text-field architecture-32bit-check-box architecture-64bit-check-box bundle-format-choice-box]
    (ui/value! certificate-text-field (get-string-pref prefs "bundle-android-certificate"))
    (ui/value! private-key-text-field (get-string-pref prefs "bundle-android-private-key"))
    (ui/value! architecture-32bit-check-box (prefs/get-prefs prefs "bundle-android-architecture-32bit?" true))
    (ui/value! architecture-64bit-check-box (prefs/get-prefs prefs "bundle-android-architecture-64bit?" false))
    (ui/value! bundle-format-choice-box (prefs/get-prefs prefs "bundle-android-bundle-format" "apk"))))

(defn- save-android-prefs! [prefs view]
  (ui/with-controls view [certificate-text-field private-key-text-field architecture-32bit-check-box architecture-64bit-check-box bundle-format-choice-box]
    (set-string-pref! prefs "bundle-android-certificate" (ui/value certificate-text-field))
    (set-string-pref! prefs "bundle-android-private-key" (ui/value private-key-text-field))
    (prefs/set-prefs prefs "bundle-android-architecture-32bit?" (ui/value architecture-32bit-check-box))
    (prefs/set-prefs prefs "bundle-android-architecture-64bit?" (ui/value architecture-64bit-check-box))
    (set-string-pref! prefs "bundle-android-bundle-format" (ui/value bundle-format-choice-box))))

(defn- get-android-options [view]
  (ui/with-controls view [architecture-32bit-check-box architecture-64bit-check-box certificate-text-field private-key-text-field bundle-format-choice-box]
    {:architecture-32bit? (ui/value architecture-32bit-check-box)
     :architecture-64bit? (ui/value architecture-64bit-check-box)
     :certificate (get-file certificate-text-field)
     :private-key (get-file private-key-text-field)
     :bundle-format (ui/value bundle-format-choice-box)}))

(defn- set-android-options! [view {:keys [architecture-32bit? architecture-64bit? certificate private-key bundle-format] :as _options} issues]
  (ui/with-controls view [architecture-32bit-check-box architecture-64bit-check-box certificate-text-field private-key-text-field bundle-format-choice-box ok-button]
    (doto certificate-text-field
      (set-file! certificate)
      (set-field-status! (:certificate issues)))
    (doto private-key-text-field
      (set-file! private-key)
      (set-field-status! (:private-key issues)))
    (doto architecture-32bit-check-box
      (ui/value! architecture-32bit?)
      (set-field-status! (:architecture issues)))
    (doto architecture-64bit-check-box
      (ui/value! architecture-64bit?)
      (set-field-status! (:architecture issues)))
    (doto bundle-format-choice-box
      (ui/value! bundle-format)
      (set-field-status! (:bundle-format issues)))
    (ui/enable! ok-button (and (nil? (:architecture issues))
                               (or (and (nil? certificate)
                                        (nil? private-key))
                                   (and (existing-file-of-type? "pem" certificate)
                                        (existing-file-of-type? "pk8" private-key)))))))

(defn- get-android-issues [{:keys [certificate private-key architecture-32bit? architecture-64bit? bundle-format] :as _options}]
  {:general (when (and (nil? certificate) (nil? private-key))
              [:info "Set certificate and private key, or leave blank to sign APK with an auto-generated debug certificate."])
   :certificate (cond
                  (and (some? certificate) (not (fs/existing-file? certificate)))
                  [:fatal "Certificate file not found."]

                  (and (some? certificate) (not (existing-file-of-type? "pem" certificate)))
                  [:fatal "Invalid certificate."]

                  (and (nil? certificate) (some? private-key))
                  [:fatal "Certificate must be set if private key is specified."])
   :private-key (cond
                  (and (some? private-key) (not (fs/existing-file? private-key)))
                  [:fatal "Private key file not found."]

                  (and (some? private-key) (not (existing-file-of-type? "pk8" private-key)))
                  [:fatal "Invalid private key."]

                  (and (some? certificate) (nil? private-key))
                  [:fatal "Private key must be set if certificate is specified."])
   :architecture (when-not (or architecture-32bit? architecture-64bit?)
                   [:fatal "At least one architecture must be selected."])
   :bundle-format (when-not bundle-format
                   [:fatal "No bundle format selected."])})

(deftype AndroidBundleOptionsPresenter [workspace view variant-choices compression-choices]
  BundleOptionsPresenter
  (make-views [this owner-window]
    (let [refresh! (make-presenter-refresher this)]
      (into [(make-generic-headers "Bundle Android Application")
             (make-android-controls refresh! owner-window)]
            (make-generic-controls refresh! variant-choices compression-choices))))
  (load-prefs! [_this prefs]
    (load-generic-prefs! prefs view)
    (load-android-prefs! prefs view))
  (save-prefs! [_this prefs]
    (save-generic-prefs! prefs view)
    (save-android-prefs! prefs view))
  (get-options [_this]
    (merge {:platform "armv7-android"}
           (get-generic-options view)
           (get-android-options view)))
  (set-options! [_this options]
    (let [issues (get-android-issues options)]
      (set-generic-options! view options workspace)
      (set-android-options! view options issues)
      (set-generic-headers! view issues [:architecture :certificate :private-key :bundle-format]))))

;; -----------------------------------------------------------------------------
;; iOS
;; -----------------------------------------------------------------------------

(defn- get-code-signing-identity-names []
  (mapv second (bundle/find-identities)))

(defn- make-ios-controls [refresh! owner-window]
  (assert (fn? refresh!))
  (let [sign-app-check-box (make-labeled-check-box nil "sign-app-check-box" true refresh!)
        provisioning-profile-file-field (make-file-field refresh! owner-window "provisioning-profile-text-field" "Choose Provisioning Profile" [["Provisioning Profiles (*.mobileprovision)" "*.mobileprovision"]])
        no-identity-label "None"
        code-signing-identity-choice-box (doto (ChoiceBox.)
                                           (.setId "code-signing-identity-choice-box")
                                           (.setFocusTraversable false)
                                           (.setMaxWidth Double/MAX_VALUE) ; Required to fill available space.
                                           (.setConverter (proxy [StringConverter] []
                                                            (toString [value]
                                                              (if (some? value) value no-identity-label))
                                                            (fromString [label]
                                                              (if (= no-identity-label label) nil label)))))
        architecture-controls (doto (VBox.)
                                (ui/children! [(make-labeled-check-box "32-bit (armv7)" "architecture-32bit-check-box" true refresh!)
                                               (make-labeled-check-box "64-bit (arm64)" "architecture-64bit-check-box" true refresh!)
                                               (make-labeled-check-box "Simulator (x86_64)" "architecture-simulator-check-box" false refresh!)]))]
    (ui/on-action! code-signing-identity-choice-box refresh!)
    (doto (VBox.)
      (ui/add-style! "settings")
      (ui/add-style! "ios")
      (ui/children! [(labeled! "Sign application" sign-app-check-box)
                     (labeled! "Code signing identity" code-signing-identity-choice-box)
                     (labeled! "Provisioning profile" provisioning-profile-file-field)
                     (labeled! "Architectures" architecture-controls)]))))


(defn- load-ios-prefs! [prefs view code-signing-identity-names]
  ;; This falls back on settings from the Sign iOS Application dialog if available,
  ;; but we will write to our own keys in the preference for these.
  (ui/with-controls view [sign-app-check-box code-signing-identity-choice-box provisioning-profile-text-field architecture-32bit-check-box architecture-64bit-check-box architecture-simulator-check-box]
    (ui/value! sign-app-check-box (prefs/get-prefs prefs "bundle-ios-sign-app?" true))
    (ui/value! code-signing-identity-choice-box (or ((set code-signing-identity-names)
                                                     (or (get-string-pref prefs "bundle-ios-code-signing-identity")
                                                         (second (prefs/get-prefs prefs "last-identity" [nil nil]))))
                                                    (first code-signing-identity-names)))
    (ui/value! provisioning-profile-text-field (or (get-string-pref prefs "bundle-ios-provisioning-profile")
                                                   (get-string-pref prefs "last-provisioning-profile")))
    (ui/value! architecture-32bit-check-box (prefs/get-prefs prefs "bundle-ios-architecture-32bit?" true))
    (ui/value! architecture-64bit-check-box (prefs/get-prefs prefs "bundle-ios-architecture-64bit?" true))
    (ui/value! architecture-simulator-check-box (prefs/get-prefs prefs "bundle-ios-architecture-simulator?" false))))

(defn- save-ios-prefs! [prefs view]
  (ui/with-controls view [sign-app-check-box code-signing-identity-choice-box provisioning-profile-text-field architecture-32bit-check-box architecture-64bit-check-box architecture-simulator-check-box]
    (prefs/set-prefs prefs "bundle-ios-sign-app?" (ui/value sign-app-check-box))
    (set-string-pref! prefs "bundle-ios-code-signing-identity" (ui/value code-signing-identity-choice-box))
    (set-string-pref! prefs "bundle-ios-provisioning-profile" (ui/value provisioning-profile-text-field))
    (prefs/set-prefs prefs "bundle-ios-architecture-32bit?" (ui/value architecture-32bit-check-box))
    (prefs/set-prefs prefs "bundle-ios-architecture-64bit?" (ui/value architecture-64bit-check-box))
    (prefs/set-prefs prefs "bundle-ios-architecture-simulator?" (ui/value architecture-simulator-check-box))))

(defn- get-ios-options [view]
  (ui/with-controls view [sign-app-check-box code-signing-identity-choice-box provisioning-profile-text-field architecture-32bit-check-box architecture-64bit-check-box architecture-simulator-check-box]
    {:architecture-32bit? (ui/value architecture-32bit-check-box)
     :architecture-64bit? (ui/value architecture-64bit-check-box)
     :architecture-simulator? (ui/value architecture-simulator-check-box)
     :code-signing-identity (ui/value code-signing-identity-choice-box)
     :provisioning-profile (get-file provisioning-profile-text-field)
     :sign-app? (ui/value sign-app-check-box)}))

(defn- set-ios-options! [view {:keys [architecture-32bit? architecture-64bit? architecture-simulator? code-signing-identity provisioning-profile sign-app?] :as _options} issues code-signing-identity-names]
  (ui/with-controls view [architecture-32bit-check-box architecture-64bit-check-box architecture-simulator-check-box code-signing-identity-choice-box ok-button provisioning-profile-text-field sign-app-check-box]
    (ui/value! sign-app-check-box sign-app?)
    (doto code-signing-identity-choice-box
      (set-choice! (into [nil] code-signing-identity-names) code-signing-identity)
      (set-field-status! (:code-signing-identity issues))
      (ui/disable! (or (not sign-app?) (empty? code-signing-identity-names))))
    (doto provisioning-profile-text-field
      (set-file! provisioning-profile)
      (set-field-status! (:provisioning-profile issues))
      (ui/disable! (not sign-app?)))
    (doto architecture-32bit-check-box
      (ui/value! architecture-32bit?)
      (set-field-status! (:architecture issues)))
    (doto architecture-64bit-check-box
      (ui/value! architecture-64bit?)
      (set-field-status! (:architecture issues)))
    (doto architecture-simulator-check-box
      (ui/value! architecture-simulator?)
      (set-field-status! (:architecture issues)))
    (ui/enable! ok-button (and (nil? (:architecture issues))
                               (or (not sign-app?)
                                   (and (some? code-signing-identity)
                                        (fs/existing-file? provisioning-profile)))))))

(defn- get-ios-issues [{:keys [architecture-32bit? architecture-64bit? architecture-simulator? code-signing-identity provisioning-profile sign-app?] :as _options} code-signing-identity-names]
  {:general (when (and sign-app? (empty? code-signing-identity-names))
              [:fatal "No code signing identities found on this computer."])
   :code-signing-identity (when (and sign-app? (nil? code-signing-identity))
                            [:fatal "Code signing identity must be set to sign the application."])
   :provisioning-profile (when sign-app?
                           (cond
                             (nil? provisioning-profile)
                             [:fatal "Provisioning profile must be set to sign the application."]

                             (not (fs/existing-file? provisioning-profile))
                             [:fatal "Provisioning profile file not found."]

                             (not (existing-file-of-type? "mobileprovision" provisioning-profile))
                             [:fatal "Invalid provisioning profile."]))
   :architecture (when-not (or architecture-32bit? architecture-64bit? architecture-simulator?)
                   [:fatal "At least one architecture must be selected."])})

(deftype IOSBundleOptionsPresenter [workspace view variant-choices compression-choices]
  BundleOptionsPresenter
  (make-views [this owner-window]
    (let [refresh! (make-presenter-refresher this)]
      (into [(make-generic-headers "Bundle iOS Application")
             (make-ios-controls refresh! owner-window)]
            (make-generic-controls refresh! variant-choices compression-choices))))
  (load-prefs! [_this prefs]
    (load-generic-prefs! prefs view)
    (load-ios-prefs! prefs view (get-code-signing-identity-names)))
  (save-prefs! [_this prefs]
    (save-generic-prefs! prefs view)
    (save-ios-prefs! prefs view))
  (get-options [_this]
    (merge {:platform "armv7-darwin"}
           (get-generic-options view)
           (get-ios-options view)))
  (set-options! [_this options]
    (let [code-signing-identity-names (get-code-signing-identity-names)
          issues (get-ios-issues options code-signing-identity-names)]
      (set-generic-options! view options workspace)
      (set-ios-options! view options issues code-signing-identity-names)
      (set-generic-headers! view issues [:architecture :code-signing-identity :provisioning-profile]))))

;; -----------------------------------------------------------------------------

(def ^:private common-variants [["Debug" "debug"]
                                ["Release" "release"]])

(def ^:private desktop-variants (conj common-variants ["Headless" "headless"]))

(def ^:private common-compressions [["Enabled" "enabled"]
                                    ["Disabled" "disabled"]
                                    ["Use Editor Preference" "editor"]])

(defmulti bundle-options-presenter (fn [_workspace _view platform] platform))
(defmethod bundle-options-presenter :default [_workspace _view platform] (throw (IllegalArgumentException. (str "Unsupported platform: " platform))))
(defmethod bundle-options-presenter :android [workspace view _platform] (AndroidBundleOptionsPresenter. workspace view common-variants common-compressions))
(defmethod bundle-options-presenter :html5   [workspace view _platform] (GenericBundleOptionsPresenter. workspace view "Bundle HTML5 Application" "js-web" common-variants common-compressions))
(defmethod bundle-options-presenter :ios     [workspace view _platform] (IOSBundleOptionsPresenter. workspace view common-variants common-compressions))
(defmethod bundle-options-presenter :linux   [workspace view _platform] (GenericBundleOptionsPresenter. workspace view "Bundle Linux Application" "x86_64-linux" desktop-variants common-compressions))
(defmethod bundle-options-presenter :macos   [workspace view _platform] (GenericBundleOptionsPresenter. workspace view "Bundle macOS Application" "x86_64-darwin" desktop-variants common-compressions))
(defmethod bundle-options-presenter :windows [workspace view _platform] (SelectablePlatformBundleOptionsPresenter. workspace view "Bundle Windows Application" :windows [["32-bit" "x86-win32"] ["64-bit" "x86_64-win32"]] (if os-32-bit? "x86-win32" "x86_64-win32") desktop-variants common-compressions))

(handler/defhandler ::close :bundle-dialog
  (run [stage]
    (ui/close! stage)))

(handler/defhandler ::query-output-directory :bundle-dialog
  (run [bundle! prefs presenter stage]
    (save-prefs! presenter prefs)
    (let [bundle-options (get-options presenter)
          initial-directory (get-file-pref prefs "bundle-output-directory")]
      (assert (string? (not-empty (:platform bundle-options))))
      (when-let [output-directory (query-directory! "Output Directory" initial-directory stage)]
        (set-file-pref! prefs "bundle-output-directory" output-directory)
        (let [platform-bundle-output-directory (io/file output-directory (:platform bundle-options))
              platform-bundle-output-directory-exists? (.exists platform-bundle-output-directory)]
          (when (or (not platform-bundle-output-directory-exists?)
                    (query-overwrite! platform-bundle-output-directory stage))
            (let [bundle-options (assoc bundle-options :output-directory platform-bundle-output-directory)]
              (ui/close! stage)
              (bundle! bundle-options))))))))

(defn show-bundle-dialog! [workspace platform prefs owner-window bundle!]
  (let [stage (doto (ui/make-dialog-stage owner-window)
                (ui/title! "Bundle Application"))
        root (doto (VBox.)
               (ui/add-style! "bundle-dialog")
               (ui/add-style! (name platform))
               (ui/apply-css!))
        presenter (bundle-options-presenter workspace root platform)]

    ;; Dialog context.
    (ui/context! root :bundle-dialog {:bundle! bundle!
                                      :prefs prefs
                                      :presenter presenter
                                      :stage stage} nil)
    ;; Key bindings.
    (ui/ensure-receives-key-events! stage)
    (ui/bind-keys! root {KeyCode/ESCAPE ::close})

    ;; Presenter views.
    (doseq [view (make-views presenter stage)]
      (ui/add-child! root view))

    ;; Button bar.
    (let [ok-button (doto (Button. "Create Bundle...") (.setId "ok-button") (.setFocusTraversable false))
          buttons (doto (HBox.) (ui/add-style! "buttons"))]
      (ui/add-child! buttons ok-button)
      (ui/add-child! root buttons)
      (ui/bind-action! ok-button ::query-output-directory)
      (.setDefaultButton ok-button true))

    ;; Load preferences and refresh the view.
    ;; We also refresh whenever our application becomes active.
    (load-prefs! presenter prefs)
    (refresh-presenter! presenter)
    (ui/add-application-focused-callback! :bundle-dialog refresh-presenter! presenter)
    (ui/on-closed! stage (fn [_event] (ui/remove-application-focused-callback! :bundle-dialog)))

    ;; Show dialog.
    (let [scene (Scene. root)]
      (.setScene stage scene)
      (ui/show! stage))))
