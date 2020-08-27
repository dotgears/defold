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

(ns editor.workspace
  "Define the concept of a project, and its Project node type. This namespace bridges between Eclipse's workbench and
ordinary paths."
  (:require [clojure.java.io :as io]
            [clojure.set :as set]
            [clojure.string :as string]
            [clojure.tools.reader.edn :as edn]
            [dynamo.graph :as g]
            [editor.fs :as fs]
            [editor.library :as library]
            [editor.prefs :as prefs]
            [editor.progress :as progress]
            [editor.resource :as resource]
            [editor.resource-watch :as resource-watch]
            [editor.url :as url]
            [service.log :as log])
  (:import [java.io File PushbackReader]
           [java.net URI]
           [editor.resource FileResource]
           [org.apache.commons.io FilenameUtils]))

(set! *warn-on-reflection* true)

(def build-dir "/build/default/")

(defn project-path
  (^File [workspace]
   (g/with-auto-evaluation-context evaluation-context
     (project-path workspace evaluation-context)))
  (^File [workspace evaluation-context]
   (io/as-file (g/node-value workspace :root evaluation-context))))

(defn build-path [workspace]
  (io/file (project-path workspace) "build/default/"))

(defrecord BuildResource [resource prefix]
  resource/Resource
  (children [this] nil)
  (ext [this] (:build-ext (resource/resource-type this) "unknown"))
  (resource-type [this] (resource/resource-type resource))
  (source-type [this] (resource/source-type resource))
  (read-only? [this] false)
  (path [this] (let [ext (resource/ext this)
                     ext (if (not-empty ext) (str "." ext) "")
                     suffix (format "%x" (resource/resource-hash this))]
                 (if-let [path (resource/path resource)]
                   (str (FilenameUtils/removeExtension path) ext)
                   (str prefix "_generated_" suffix ext))))
  (abs-path [this] (.getAbsolutePath (io/file (build-path (resource/workspace this)) (resource/path this))))
  (proj-path [this] (str "/" (resource/path this)))
  (resource-name [this] (resource/resource-name resource))
  (workspace [this] (resource/workspace resource))
  (resource-hash [this] (resource/resource-hash resource))
  (openable? [this] false)

  io/IOFactory
  (io/make-input-stream  [this opts] (io/make-input-stream (File. (resource/abs-path this)) opts))
  (io/make-reader        [this opts] (io/make-reader (io/make-input-stream this opts) opts))
  (io/make-output-stream [this opts] (let [file (File. (resource/abs-path this))] (io/make-output-stream file opts)))
  (io/make-writer        [this opts] (io/make-writer (io/make-output-stream this opts) opts))

  io/Coercions
  (io/as-file [this] (File. (resource/abs-path this))))

(def build-resource? (partial instance? BuildResource))

(defn make-build-resource
  ([resource]
   (make-build-resource resource nil))
  ([resource prefix]
   (assert (resource/resource? resource))
   (BuildResource. resource prefix)))

(defn sort-resource-tree [{:keys [children] :as tree}]
  (let [sorted-children (sort-by (fn [r]
                                   [(resource/file-resource? r)
                                    ({:folder 0 :file 1} (resource/source-type r))
                                    (when-let [node-name (resource/resource-name r)]
                                      (string/lower-case node-name))])
                                 (map sort-resource-tree children))]
    (assoc tree :children (vec sorted-children))))

(g/defnk produce-resource-tree [_node-id root resource-snapshot]
  (sort-resource-tree
    (resource/make-file-resource _node-id root (io/as-file root) (:resources resource-snapshot))))

(g/defnk produce-resource-list [resource-tree]
  (resource/resource-seq resource-tree))

(g/defnk produce-resource-map [resource-list]
  (into {} (map #(do [(resource/proj-path %) %]) resource-list)))

(defn get-view-type [workspace id]
  (get (g/node-value workspace :view-types) id))

(defn- editable-view-type? [view-type]
  (case view-type
    (:default :text) false
    true))

(defn register-resource-type [workspace & {:keys [textual? ext build-ext node-type load-fn dependencies-fn read-fn write-fn icon view-types view-opts tags tag-opts template label stateless? auto-connect-save-data?]}]
  (let [resource-type {:textual? (true? textual?)
                       :editable? (some? (some editable-view-type? view-types))
                       :build-ext (if (nil? build-ext) (str ext "c") build-ext)
                       :node-type node-type
                       :load-fn load-fn
                       :dependencies-fn dependencies-fn
                       :write-fn write-fn
                       :read-fn read-fn
                       :icon icon
                       :view-types (map (partial get-view-type workspace) view-types)
                       :view-opts view-opts
                       :tags tags
                       :tag-opts tag-opts
                       :template template
                       :label label
                       :stateless? (if (nil? stateless?) (nil? load-fn) stateless?)
                       :auto-connect-save-data? (and (some? write-fn)
                                                     (not (false? auto-connect-save-data?)))}
        resource-types (if (string? ext)
                         [(assoc resource-type :ext (string/lower-case ext))]
                         (map (fn [ext] (assoc resource-type :ext (string/lower-case ext))) ext))]
    (for [resource-type resource-types]
      (g/update-property workspace :resource-types assoc (:ext resource-type) resource-type))))

(defn get-resource-type [workspace ext]
  (get (g/node-value workspace :resource-types) ext))

(defn get-resource-type-map [workspace]
  (g/node-value workspace :resource-types))

(defn get-resource-types
  ([workspace]
   (map second (g/node-value workspace :resource-types)))
  ([workspace tag]
   (filter #(contains? (:tags %) tag) (map second (g/node-value workspace :resource-types)))))

(defn- template-path [resource-type]
  (or (:template resource-type)
      (some->> resource-type :ext (str "templates/template."))))

(defn has-template? [resource-type]
  (some? (some-> resource-type template-path io/resource)))

(defn template [resource-type]
  (when-let [template-path (template-path resource-type)]
    (when-let [resource (io/resource template-path)]
      (with-open [f (io/reader resource)]
        (slurp f)))))

(defn resource-icon [resource]
  (when resource
    (if (and (resource/read-only? resource)
             (= (resource/path resource) (resource/resource-name resource)))
      "icons/32/Icons_03-Builtins.png"
      (condp = (resource/source-type resource)
        :file
        (or (:icon (resource/resource-type resource)) "icons/32/Icons_29-AT-Unknown.png")
        :folder
        "icons/32/Icons_01-Folder-closed.png"))))

(defn file-resource
  ([workspace path-or-file]
   (g/with-auto-evaluation-context evaluation-context
     (file-resource workspace path-or-file evaluation-context)))
  ([workspace path-or-file evaluation-context]
   (let [root (g/node-value workspace :root evaluation-context)
         f (if (instance? File path-or-file)
             path-or-file
             (File. (str root path-or-file)))]
     (resource/make-file-resource workspace root f []))))

(defn find-resource
  ([workspace proj-path]
   (g/with-auto-evaluation-context evaluation-context
     (find-resource workspace proj-path evaluation-context)))
  ([workspace proj-path evaluation-context]
   (get (g/node-value workspace :resource-map evaluation-context) proj-path)))

(defn resolve-workspace-resource
  ([workspace path]
   (g/with-auto-evaluation-context evaluation-context
     (resolve-workspace-resource workspace path evaluation-context)))
  ([workspace path evaluation-context]
   (when (and path (not-empty path))
     (or
       (find-resource workspace path evaluation-context)
       (file-resource workspace path evaluation-context)))))

(defn- absolute-path [^String path]
  (.startsWith path "/"))

(defn to-absolute-path
  ([rel-path] (to-absolute-path "" rel-path))
  ([base rel-path]
   (if (absolute-path rel-path)
     rel-path
     (str base "/" rel-path))))

(defn resolve-resource [base-resource path]
  (when-not (empty? path)
    (let [path (if (absolute-path path)
                 path
                 (to-absolute-path (str (.getParent (File. (resource/proj-path base-resource)))) path))]
      (when-let [workspace (:workspace base-resource)]
        (resolve-workspace-resource workspace path)))))

(defn set-project-dependencies! [workspace library-uris]
  (g/set-property! workspace :dependencies library-uris)
  library-uris)

(defn dependencies [workspace]
  (g/node-value workspace :dependencies))

(defn dependencies-reachable? [dependencies]
  (let [hosts (into #{} (map url/strip-path) dependencies)]
    (every? url/reachable? hosts)))

(defn missing-dependencies [workspace]
  (let [project-directory (project-path workspace)
        dependencies (g/node-value workspace :dependencies)]
    (into #{}
          (comp (remove :file)
                (map :uri))
          (library/current-library-state project-directory dependencies))))

(defn make-snapshot-info [workspace project-path dependencies snapshot-cache]
  (let [snapshot-info (resource-watch/make-snapshot-info workspace project-path dependencies snapshot-cache)]
    (assoc snapshot-info :map (resource-watch/make-resource-map (:snapshot snapshot-info)))))

(defn update-snapshot-cache! [workspace snapshot-cache]
  (g/set-property! workspace :snapshot-cache snapshot-cache))

(defn snapshot-cache [workspace]
  (g/node-value workspace :snapshot-cache))

(defn resource-sync!
  ([workspace]
   (resource-sync! workspace []))
  ([workspace moved-files]
   (resource-sync! workspace moved-files progress/null-render-progress!))
  ([workspace moved-files render-progress!]
   (let [snapshot-info (make-snapshot-info workspace (project-path workspace) (dependencies workspace) (snapshot-cache workspace))
         {new-snapshot :snapshot new-map :map new-snapshot-cache :snapshot-cache} snapshot-info]
     (update-snapshot-cache! workspace new-snapshot-cache)
     (resource-sync! workspace moved-files render-progress! new-snapshot new-map)))
  ([workspace moved-files render-progress! new-snapshot new-map]
   (let [project-path (project-path workspace)
         moved-proj-paths (keep (fn [[src tgt]]
                                  (let [src-path (resource/file->proj-path project-path src)
                                        tgt-path (resource/file->proj-path project-path tgt)]
                                    (assert (some? src-path) (str "project does not contain source " (pr-str src)))
                                    (assert (some? tgt-path) (str "project does not contain target " (pr-str tgt)))
                                    (when (not= src-path tgt-path)
                                      [src-path tgt-path])))
                                moved-files)
         old-snapshot (g/node-value workspace :resource-snapshot)
         old-map      (resource-watch/make-resource-map old-snapshot)
         changes      (resource-watch/diff old-snapshot new-snapshot)]
     (when (or (not (resource-watch/empty-diff? changes)) (seq moved-proj-paths))
       (g/set-property! workspace :resource-snapshot new-snapshot)
       (let [changes (into {} (map (fn [[type resources]] [type (filter #(= :file (resource/source-type %)) resources)]) changes))
             move-source-paths (map first moved-proj-paths)
             move-target-paths (map second moved-proj-paths)
             chain-moved-paths (set/intersection (set move-source-paths) (set move-target-paths))
             merged-target-paths (set (map first (filter (fn [[k v]] (> v 1)) (frequencies move-target-paths))))
             moved (keep (fn [[source-path target-path]]
                           (when-not (or
                                       ;; resource sync currently can't handle chained moves, so refactoring is
                                       ;; temporarily disabled for those cases (no move pair)
                                       (chain-moved-paths source-path) (chain-moved-paths target-path)
                                       ;; also can't handle merged targets, multiple files with same name moved to same dir
                                       (merged-target-paths target-path))
                             (let [src-resource (old-map source-path)
                                   tgt-resource (new-map target-path)]
                               ;; We used to (assert (some? src-resource)), but this could fail for instance if
                               ;; * source-path refers to a .dotfile (like .DS_Store) that we ignore in resource-watch
                               ;; * Some external process has created a file in a to-be-moved directory and we haven't run a resource-sync! before the move
                               ;; We handle these cases by ignoring the move. Any .dotfiles will stay ignored, and any new files will pop up as :added
                               ;;
                               ;; We also used to (assert (some? tgt-resource)) but an arguably very unlikely case is that the target of the move is
                               ;; deleted from disk after the move but before the snapshot.
                               ;; We handle that by ignoring the move and effectively treating target as just :removed.
                               ;; The source will be :removed or :changed (if a library snuck in).
                               (cond
                                 (nil? src-resource)
                                 (do (log/warn :msg (str "can't find source of move " source-path)) nil)

                                 (nil? tgt-resource)
                                 (do (log/warn :msg (str "can't find target of move " target-path)) nil)

                                 (and (= :file (resource/source-type src-resource))
                                      (= :file (resource/source-type tgt-resource))) ; paranoia
                                 [src-resource tgt-resource]))))
                         moved-proj-paths)
             changes-with-moved (assoc changes :moved moved)]
         (assert (= (count (distinct (map (comp resource/proj-path first) moved)))
                    (count (distinct (map (comp resource/proj-path second) moved)))
                    (count moved))) ; no overlapping sources, dito targets
         (assert (= (count (distinct (concat (map (comp resource/proj-path first) moved)
                                             (map (comp resource/proj-path second) moved))))
                    (* 2 (count moved)))) ; no chained moves src->tgt->tgt2...
         (assert (empty? (set/intersection (set (map (comp resource/proj-path first) moved))
                                           (set (map resource/proj-path (:added changes)))))) ; no move-source is in :added
         (try
           (let [listeners @(g/node-value workspace :resource-listeners)
                 total-progress-size (transduce (map first) + 0 listeners)]
             (loop [listeners listeners
                    parent-progress (progress/make "" total-progress-size)]
               (when-some [[progress-span listener] (first listeners)]
                 (resource/handle-changes listener changes-with-moved
                                          (progress/nest-render-progress render-progress! parent-progress progress-span))
                 (recur (next listeners)
                        (progress/advance parent-progress progress-span)))))
           (finally
             (render-progress! progress/done)))))
     changes)))

(defn fetch-and-validate-libraries [workspace library-uris render-fn]
  (->> (library/current-library-state (project-path workspace) library-uris)
       (library/fetch-library-updates library/default-http-resolver render-fn)
       (library/validate-updated-libraries)))

(defn install-validated-libraries! [workspace library-uris lib-states]
  (set-project-dependencies! workspace library-uris)
  (library/install-validated-libraries! (project-path workspace) lib-states))

(defn add-resource-listener! [workspace progress-span listener]
  (swap! (g/node-value workspace :resource-listeners) conj [progress-span listener]))


(g/deftype UriVec [URI])

(g/defnode Workspace
  (property root g/Str)
  (property dependencies UriVec)
  (property opened-files g/Any (default (atom #{})))
  (property resource-snapshot g/Any)
  (property resource-listeners g/Any (default (atom [])))
  (property view-types g/Any)
  (property resource-types g/Any)
  (property snapshot-cache g/Any (default {}))
  (property build-settings g/Any)

  (output resource-tree FileResource :cached produce-resource-tree)
  (output resource-list g/Any :cached produce-resource-list)
  (output resource-map g/Any :cached produce-resource-map))

(defn make-build-settings
  [prefs]
  {:compress-textures? (prefs/get-prefs prefs "general-enable-texture-compression" false)})

(defn update-build-settings!
  [workspace prefs]
  (g/set-property! workspace :build-settings (make-build-settings prefs)))

(defn artifact-map [workspace]
  (g/user-data workspace ::artifact-map))

(defn artifact-map! [workspace artifact-map]
  (g/user-data! workspace ::artifact-map artifact-map))

(defn etags [workspace]
  (g/user-data workspace ::etags))

(defn etag [workspace proj-path]
  (get (etags workspace) proj-path))

(defn etags! [workspace etags]
  (g/user-data! workspace ::etags etags))

(defn- artifact-map-file
  ^File [workspace]
  (io/file (build-path workspace) ".artifact-map"))

(defn- try-read-artifact-map [^File file]
  (when (.exists file)
    (try
      (with-open [reader (PushbackReader. (io/reader file))]
        (edn/read reader))
      (catch Exception error
        (log/warn :msg "Failed to read artifact map. Build cache invalidated." :exception error)
        nil))))

(defn artifact-map->etags [artifact-map]
  (when (seq artifact-map)
    (into {}
          (map (juxt key (comp :etag val)))
          artifact-map)))

(defn load-build-cache! [workspace]
  (let [file (artifact-map-file workspace)
        artifact-map (try-read-artifact-map file)
        etags (artifact-map->etags artifact-map)]
    (artifact-map! workspace artifact-map)
    (etags! workspace etags)
    nil))

(defn save-build-cache! [workspace]
  (let [file (artifact-map-file workspace)
        artifact-map (artifact-map workspace)]
    (if (empty? artifact-map)
      (fs/delete-file! file)
      (let [saved-artifact-map (into (sorted-map) artifact-map)]
        (fs/create-file! file (pr-str saved-artifact-map))))
    nil))

(defn clear-build-cache! [workspace]
  (let [file (artifact-map-file workspace)]
    (artifact-map! workspace nil)
    (etags! workspace nil)
    (fs/delete-file! file)
    nil))

(defn make-workspace [graph project-path build-settings]
  (g/make-node! graph Workspace
                :root project-path
                :resource-snapshot (resource-watch/empty-snapshot)
                :view-types {:default {:id :default}}
                :resource-listeners (atom [])
                :build-settings build-settings))

(defn register-view-type [workspace & {:keys [id label make-view-fn make-preview-fn dispose-preview-fn focus-fn text-selection-fn]}]
  (let [view-type (merge {:id    id
                          :label label}
                         (when make-view-fn
                           {:make-view-fn make-view-fn})
                         (when make-preview-fn
                           {:make-preview-fn make-preview-fn})
                         (when dispose-preview-fn
                           {:dispose-preview-fn dispose-preview-fn})
                         (when focus-fn
                           {:focus-fn focus-fn})
                         (when text-selection-fn
                           {:text-selection-fn text-selection-fn}))]
     (g/update-property workspace :view-types assoc (:id view-type) view-type)))
