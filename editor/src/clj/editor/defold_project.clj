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

(ns editor.defold-project
  "Define the concept of a project, and its Project node type. This namespace bridges between Eclipse's workbench and
  ordinary paths."
  (:require [clojure.java.io :as io]
            [clojure.set :as set]
            [dynamo.graph :as g]
            [editor.code.script-intelligence :as si]
            [editor.collision-groups :as collision-groups]
            [editor.core :as core]
            [editor.error-reporting :as error-reporting]
            [editor.gl :as gl]
            [editor.handler :as handler]
            [editor.ui :as ui]
            [editor.library :as library]
            [editor.progress :as progress]
            [editor.resource :as resource]
            [editor.resource-io :as resource-io]
            [editor.resource-node :as resource-node]
            [editor.resource-update :as resource-update]
            [editor.workspace :as workspace]
            [editor.game-project-core :as gpc]
            [editor.settings-core :as settings-core]
            [editor.pipeline :as pipeline]
            [editor.placeholder-resource :as placeholder-resource]
            [editor.util :as util]
            [service.log :as log]
            [editor.graph-util :as gu]
            [util.text-util :as text-util]
            [schema.core :as s]
            [util.thread-util :as thread-util])
  (:import (java.util.concurrent.atomic AtomicLong)))

(set! *warn-on-reflection* true)

(def ^:dynamic *load-cache* nil)

(def ^:private TBreakpoint
  {:resource s/Any
   :row Long})

(g/deftype Breakpoints [TBreakpoint])

(defn graph [project]
  (g/node-id->graph-id project))

(defn- load-registered-resource-node [load-fn project node-id resource]
  (concat
    (load-fn project node-id resource)
    (when (and (resource/file-resource? resource)
               (:auto-connect-save-data? (resource/resource-type resource)))
      (g/connect node-id :save-data project :save-data))))

(defn load-node [project node-id node-type resource]
  ;; Note that node-id here may be temporary (a make-node not
  ;; g/transact'ed) here, so we can't use (node-value node-id :...)
  ;; to inspect it. That's why we pass in node-type, resource.
  (try
    (let [resource-type (some-> resource resource/resource-type)
          loaded? (and *load-cache* (contains? @*load-cache* node-id))
          load-fn (:load-fn resource-type)]
      (when-not loaded?
        (if (or (= :folder (resource/source-type resource))
                (not (resource/exists? resource)))
          (g/mark-defective node-id node-type (resource-io/file-not-found-error node-id nil :fatal resource))
          (try
            (when *load-cache*
              (swap! *load-cache* conj node-id))
            (if (nil? load-fn)
              (placeholder-resource/load-node project node-id resource)
              (load-registered-resource-node load-fn project node-id resource))
            (catch Exception e
              (log/warn :msg (format "Unable to load resource '%s'" (resource/proj-path resource)) :exception e)
              (g/mark-defective node-id node-type (resource-io/invalid-content-error node-id nil :fatal resource)))))))
    (catch Throwable t
      (throw (ex-info (format "Error when loading resource '%s'" (resource/resource->proj-path resource))
                      {:node-type node-type
                       :resource-path (resource/resource->proj-path resource)}
                      t)))))

(defn- node-load-dependencies
  "Returns node-ids for the immediate dependencies of node-id.

  `loaded-nodes` is the complete set of nodes being loaded.

  `loaded-nodes-by-resource-path` is a map from project path to node id for the nodes being
  reloaded.

  `nodes-by-resource-path` is a map from project path to:
    * new node-id if the path is being reloaded
    * old node-id if the path is not being reloaded

  `resource-node-dependencies` is a function from node id to the project
  paths which are the in-memory/current dependencies for nodes not
  being reloaded."

  [node-id loaded-nodes nodes-by-resource-path resource-node-dependencies evaluation-context]
  (let [dependency-paths (if (contains? loaded-nodes node-id)
                           (try
                             (resource-node/resource-node-dependencies node-id evaluation-context)
                             (catch Exception e
                               (log/warn :msg (format "Unable to determine dependencies for resource '%s', assuming none."
                                                      (resource/proj-path (g/node-value node-id :resource evaluation-context)))
                                         :exception e)
                               nil))
                           (resource-node-dependencies node-id))
        dependency-nodes (keep nodes-by-resource-path dependency-paths)]
    dependency-nodes))

(defn sort-nodes-for-loading
  ([node-ids load-deps]
   (first (sort-nodes-for-loading node-ids #{} [] #{} (set node-ids) load-deps)))
  ([node-ids in-progress queue queued batch load-deps]
   (if-not (seq node-ids)
     [queue queued]
     (let [node-id (first node-ids)]
       ;; TODO: Handle recursive dependencies properly. Here we treat
       ;; a recurring node-id as "already loaded", which might not be
       ;; correct. Maybe log? Keep information about circular
       ;; dependency to be used in get-resource-node etc?
       (if (or (contains? queued node-id) (contains? in-progress node-id))
         (recur (rest node-ids) in-progress queue queued batch load-deps)
         (let [deps (load-deps node-id)
               [dep-queue dep-queued] (sort-nodes-for-loading deps (conj in-progress node-id) queue queued batch load-deps)]
           (recur (rest node-ids)
                  in-progress
                  (if (contains? batch node-id) (conj dep-queue node-id) dep-queue)
                  (conj dep-queued node-id)
                  batch
                  load-deps)))))))

(defn load-resource-nodes [project node-ids render-progress! resource-node-dependencies]
  (let [evaluation-context (g/make-evaluation-context)
        node-id->resource (into {}
                                (map (fn [node-id]
                                       [node-id (g/node-value node-id :resource evaluation-context)]))
                                node-ids)
        old-nodes-by-resource-path (g/node-value project :nodes-by-resource-path evaluation-context)
        loaded-nodes-by-resource-path (into {} (map (fn [[node-id resource]]
                                                      [(resource/proj-path resource) node-id]))
                                            node-id->resource)
        nodes-by-resource-path (merge old-nodes-by-resource-path loaded-nodes-by-resource-path)
        loaded-nodes (set node-ids)
        load-deps (fn [node-id] (node-load-dependencies node-id loaded-nodes nodes-by-resource-path resource-node-dependencies evaluation-context))
        node-ids (sort-nodes-for-loading loaded-nodes load-deps)
        basis (:basis evaluation-context)
        render-loading-progress! (progress/nest-render-progress render-progress! (progress/make "" 5 0) 4)
        render-processing-progress! (progress/nest-render-progress render-progress! (progress/make "" 5 4))
        load-txs (doall
                   (for [[node-index node-id] (map-indexed #(clojure.lang.MapEntry/create (inc %1) %2) node-ids)]
                     (let [resource-path (resource/resource->proj-path (node-id->resource node-id))]
                       (do
                         (render-loading-progress! (progress/make (str "Loading " resource-path)
                                                                  (count node-ids)
                                                                  node-index))
                         [(g/callback render-processing-progress! (progress/make (str "Processing " resource-path)
                                                                                 (count node-ids)
                                                                                 node-index))
                          (load-node project node-id (g/node-type* basis node-id) (node-id->resource node-id))]))))]
    (g/update-cache-from-evaluation-context! evaluation-context)
    load-txs))

(defn- load-nodes! [project node-ids render-progress! resource-node-dependencies]
  (g/transact (load-resource-nodes project node-ids render-progress! resource-node-dependencies))
  (render-progress! progress/done))

(defn connect-if-output [src-type src tgt connections]
  (let [outputs (g/output-labels src-type)]
    (for [[src-label tgt-label] connections
          :when (contains? outputs src-label)]
      (g/connect src src-label tgt tgt-label))))

(defn- resource-type->node-type [resource-type]
  (or (:node-type resource-type)
      placeholder-resource/PlaceholderResourceNode))

(def resource-node-type (comp resource-type->node-type resource/resource-type))

(defn- make-nodes! [project resources]
  (let [project-graph (graph project)]
    (g/tx-nodes-added
      (g/transact
        (for [[resource-type resources] (group-by resource/resource-type resources)
              :let [node-type (resource-type->node-type resource-type)]
              resource resources
              :when (not= :folder (resource/source-type resource))]
          (g/make-nodes project-graph [node [node-type :resource resource]]
                        (g/connect node :_node-id project :nodes)
                        (g/connect node :node-id+resource project :node-id+resources)))))))

(defn get-resource-node
  ([project path-or-resource]
   (g/with-auto-evaluation-context ec
     (get-resource-node project path-or-resource ec)))
  ([project path-or-resource evaluation-context]
   (when-let [resource (cond
                         (string? path-or-resource) (workspace/find-resource (g/node-value project :workspace evaluation-context) path-or-resource evaluation-context)
                         (satisfies? resource/Resource path-or-resource) path-or-resource
                         :else (assert false (str (type path-or-resource) " is neither a path nor a resource: " (pr-str path-or-resource))))]
     (let [nodes-by-resource-path (g/node-value project :nodes-by-resource-path evaluation-context)]
       (get nodes-by-resource-path (resource/proj-path resource))))))

(defn script-intelligence
  ([project]
   (g/with-auto-evaluation-context evaluation-context
     (script-intelligence project evaluation-context)))
  ([project evaluation-context]
   (g/node-value project :script-intelligence evaluation-context)))

(defn load-project
  ([project]
   (load-project project (g/node-value project :resources)))
  ([project resources]
   (load-project project resources progress/null-render-progress!))
  ([project resources render-progress!]
   (assert (not (seq (g/node-value project :nodes))) "load-project should only be used when loading an empty project")
   (with-bindings {#'*load-cache* (atom (into #{} (g/node-value project :nodes)))}
     (let [nodes (make-nodes! project resources)
           script-intel (script-intelligence project)]
       (load-nodes! project nodes render-progress! {})
       (when-let [game-project (get-resource-node project "/game.project")]
         (g/transact
           (concat
             (g/connect script-intel :build-errors game-project :build-errors)
             (g/connect game-project :display-profiles-data project :display-profiles)
             (g/connect game-project :texture-profiles-data project :texture-profiles)
             (g/connect game-project :settings-map project :settings))))
       project))))

(defn make-embedded-resource [project type data]
  (when-let [resource-type (get (g/node-value project :resource-types) type)]
    (resource/make-memory-resource (g/node-value project :workspace) resource-type data)))

(defn all-save-data [project]
  (g/node-value project :save-data))

(defn dirty-save-data
  ([project]
   (g/node-value project :dirty-save-data))
  ([project evaluation-context]
   (g/node-value project :dirty-save-data evaluation-context)))

(declare make-count-progress-steps-tracer make-progress-tracer)

(defn dirty-save-data-with-progress [project evaluation-context render-progress!]
  (ui/with-progress [render-progress! render-progress!]
    (let [step-count (AtomicLong.)
          step-count-tracer (make-count-progress-steps-tracer :save-data step-count)
          progress-message-fn (constantly "Saving...")]
      (render-progress! (progress/make "Saving..."))
      (dirty-save-data project (assoc evaluation-context :dry-run true :tracer step-count-tracer))
      (let [progress-tracer (make-progress-tracer :save-data (.get step-count) progress-message-fn render-progress!)]
        (dirty-save-data project (assoc evaluation-context :tracer progress-tracer))))))

(defn textual-resource-type? [resource-type]
  ;; Unregistered resources that are connected to the project
  ;; save-data input are assumed to produce text data.
  (or (nil? resource-type)
      (:textual? resource-type)))

(defn write-save-data-to-disk! [save-data {:keys [render-progress!]
                                           :or {render-progress! progress/null-render-progress!}
                                           :as opts}]
  (render-progress! (progress/make "Writing files..."))
  (if (g/error? save-data)
    (throw (Exception. ^String (g/error-message save-data)))
    (do
      (progress/progress-mapv
        (fn [{:keys [resource content value node-id]} _]
          (when-not (resource/read-only? resource)
            ;; If the file is non-binary, convert line endings to the
            ;; type used by the existing file.
            (if (and (textual-resource-type? (resource/resource-type resource))
                     (resource/exists? resource)
                     (= :crlf (text-util/guess-line-endings (io/make-reader resource nil))))
              (spit resource (text-util/lf->crlf content))
              (spit resource content))))
        save-data
        render-progress!
        (fn [{:keys [resource]}] (and resource (str "Writing " (resource/resource->proj-path resource))))))))

(defn invalidate-save-data-source-values! [save-data]
  (g/invalidate-outputs! (mapv (fn [sd] [(:node-id sd) :source-value]) save-data)))

(defn workspace
  ([project]
   (g/with-auto-evaluation-context evaluation-context
     (workspace project evaluation-context)))
  ([project evaluation-context]
   (g/node-value project :workspace evaluation-context)))

(defn make-count-progress-steps-tracer [watched-label ^AtomicLong step-count]
  (fn [state node output-type label]
    (when (and (= label watched-label) (= state :begin) (= output-type :output))
      (.getAndIncrement step-count))))

(defn make-progress-tracer [watched-label step-count progress-message-fn render-progress!]
  (let [steps-done (atom #{})
        initial-progress-message (progress-message-fn nil)
        progress (atom (progress/make initial-progress-message step-count))]
    (fn [state node output-type label]
      (when (and (= label watched-label) (= output-type :output))
        (case state
          :begin
          (let [progress-message (progress-message-fn node)]
            (render-progress! (swap! progress
                                     #(progress/with-message % (or progress-message
                                                                   (progress/message %)
                                                                   "")))))

          :end
          (let [already-done (loop []
                               (let [old @steps-done
                                     new (conj old node)]
                                 (if (compare-and-set! steps-done old new)
                                   (get old node)
                                   (recur))))]
            (when-not already-done
              (render-progress! (swap! progress progress/advance 1))))

          :fail
          nil)))))

(handler/defhandler :undo :global
  (enabled? [project-graph] (g/has-undo? project-graph))
  (run [project-graph] (g/undo! project-graph)))

(handler/defhandler :redo :global
  (enabled? [project-graph] (g/has-redo? project-graph))
  (run [project-graph] (g/redo! project-graph)))

(def ^:private bundle-targets
  (into []
        (concat (when (util/is-mac-os?) [[:ios "iOS Application..."]]) ; macOS is required to sign iOS ipa.
                [[:android "Android Application..."]
                 [:macos   "macOS Application..."]
                 [:windows "Windows Application..."]
                 [:linux   "Linux Application..."]
                 [:html5   "HTML5 Application..."]])))

(handler/register-menu! ::menubar :editor.app-view/view
  [{:label "Project"
    :id ::project
    :children [{:label "Build"
                :command :build}
               {:label "Rebuild"
                :command :rebuild}
               {:label "Build HTML5"
                :command :build-html5}
               {:label "Bundle"
                :children (mapv (fn [[platform label]]
                                  {:label label
                                   :command :bundle
                                   :user-data {:platform platform}})
                                bundle-targets)}
               {:label "Rebundle"
                :command :rebundle}
               {:label "Fetch Libraries"
                :command :fetch-libraries}
               {:label "Reload Editor Scripts"
                :command :reload-extensions}
               {:label "Live Update Settings"
                :command :live-update-settings}
               {:label "Sign iOS App..."
                :command :sign-ios-app}
               {:label :separator
                :id ::project-end}]}])

(defn- update-selection [s open-resource-nodes active-resource-node selection-value]
  (->> (assoc s active-resource-node selection-value)
    (filter (comp (set open-resource-nodes) first))
    (into {})))

(defn- perform-selection [project all-selections]
  (let [all-node-ids (->> all-selections
                       vals
                       (reduce into [])
                       distinct
                       vec)
        old-all-selections (g/node-value project :all-selections)]
    (when-not (= old-all-selections all-selections)
      (concat
        (g/set-property project :all-selections all-selections)
        (for [[node-id label] (g/sources-of project :all-selected-node-ids)]
          (g/disconnect node-id label project :all-selected-node-ids))
        (for [[node-id label] (g/sources-of project :all-selected-node-properties)]
          (g/disconnect node-id label project :all-selected-node-properties))
        (for [node-id all-node-ids]
          (concat
            (g/connect node-id :_node-id    project :all-selected-node-ids)
            (g/connect node-id :_properties project :all-selected-node-properties)))))))

(defn select
  ([project resource-node node-ids open-resource-nodes]
   (assert (every? some? node-ids) "Attempting to select nil values")
   (let [node-ids (if (seq node-ids)
                    (-> node-ids distinct vec)
                    [resource-node])
         all-selections (-> (g/node-value project :all-selections)
                            (update-selection open-resource-nodes resource-node node-ids))]
     (perform-selection project all-selections))))

(defn- perform-sub-selection
  ([project all-sub-selections]
   (g/set-property project :all-sub-selections all-sub-selections)))

(defn sub-select
  ([project resource-node sub-selection open-resource-nodes]
   (g/update-property project :all-sub-selections update-selection open-resource-nodes resource-node sub-selection)))

(defn- remap-selection [m key-m val-fn]
  (reduce (fn [m [old new]]
            (if-let [v (get m old)]
              (-> m
                (dissoc old)
                (assoc new (val-fn [new v])))
              m))
    m key-m))

(def ^:private make-resource-nodes-by-path-map
  (partial into {} (map (juxt (comp resource/proj-path second) first))))

(defn- perform-resource-change-plan [plan project render-progress!]
  (binding [*load-cache* (atom (into #{} (g/node-value project :nodes)))]
    (let [collected-properties-by-resource
          (g/with-auto-evaluation-context evaluation-context
            (into {}
                  (map (fn [[resource old-node-id]]
                         [resource
                          (g/collect-overridden-properties old-node-id evaluation-context)]))
                  (:transfer-overrides plan)))

          old-nodes-by-path (g/node-value project :nodes-by-resource-path)
          rn-dependencies-evaluation-context (g/make-evaluation-context)
          old-resource-node-dependencies (memoize
                                           (fn [node-id]
                                             (let [deps (g/node-value node-id :reload-dependencies rn-dependencies-evaluation-context)]
                                               (when-not (g/error? deps)
                                                 deps))))
          resource->old-node (comp old-nodes-by-path resource/proj-path)
          new-nodes (make-nodes! project (:new plan))
          resource-path->new-node (into {} (map (fn [resource-node]
                                                  (let [resource (g/node-value resource-node :resource)]
                                                    [(resource/proj-path resource) resource-node]))
                                                new-nodes))
          resource->new-node (comp resource-path->new-node resource/proj-path)
          ;; when transferring overrides and arcs, the target is either a newly created or already (still!)
          ;; existing node.
          resource->node (fn [resource]
                           (or (resource->new-node resource)
                               (resource->old-node resource)))]
      ;; Transfer of overrides must happen before we delete the original nodes below.
      ;; The new target nodes do not need to be loaded. When loading the new targets,
      ;; corresponding override-nodes for the incoming connections will be created in the
      ;; overrides.
      (let [transfers (into {}
                            (map (juxt second (comp resource->node first)))
                            (:transfer-overrides plan))]
        (g/transact
          (g/transfer-overrides transfers)))

      ;; must delete old versions of resource nodes before loading to avoid
      ;; load functions finding these when doing lookups of dependencies...
      (g/transact
        (for [node (:delete plan)]
          (g/delete-node node)))

      (load-nodes! project new-nodes render-progress! old-resource-node-dependencies)

      (g/update-cache-from-evaluation-context! rn-dependencies-evaluation-context)

      (g/transact
        (for [[source-resource output-arcs] (:transfer-outgoing-arcs plan)]
          (let [source-node (resource->node source-resource)
                existing-arcs (set (gu/explicit-outputs source-node))]
            (for [[source-label [target-node target-label]] (remove existing-arcs output-arcs)]
              ;; if (g/node-by-id target-node), the target of the outgoing arc
              ;; has not been deleted above - implying it has not been replaced by a
              ;; new version.
              ;; Otherwise, the target-node has probably been replaced by another version
              ;; (reloaded) and that should have reestablished any incoming arcs to it already
              (if (g/node-by-id target-node)
                (g/connect source-node source-label target-node target-label)
                [])))))

      (g/transact
        (for [[resource-node new-resource] (:redirect plan)]
          (g/set-property resource-node :resource new-resource)))

      (g/transact
        (for [node (:mark-deleted plan)]
          (let [flaw (resource-io/file-not-found-error node nil :fatal (g/node-value node :resource))]
            (g/mark-defective node flaw))))

      (let [all-outputs (mapcat (fn [node]
                                  (map (fn [[output _]] [node output]) (gu/explicit-outputs node)))
                                (:invalidate-outputs plan))]
        (g/invalidate-outputs! all-outputs))

      ;; restore overridden properties.
      (let [restore-properties-tx-data
            (g/with-auto-evaluation-context evaluation-context
              (into []
                    (mapcat (fn [[resource collected-properties]]
                              (when-some [new-node-id (resource->new-node resource)]
                                (g/restore-overridden-properties new-node-id collected-properties evaluation-context))))
                    collected-properties-by-resource))]
        (when (seq restore-properties-tx-data)
          (g/transact
            restore-properties-tx-data)))

      (let [old->new (into {} (map (fn [[p n]] [(old-nodes-by-path p) n]) resource-path->new-node))
            dissoc-deleted (fn [x] (apply dissoc x (:mark-deleted plan)))]
        (g/transact
          (concat
            (let [all-selections (-> (g/node-value project :all-selections)
                                     (dissoc-deleted)
                                     (remap-selection old->new (comp vector first)))]
              (perform-selection project all-selections))
            (let [all-sub-selections (-> (g/node-value project :all-sub-selections)
                                         (dissoc-deleted)
                                         (remap-selection old->new (constantly [])))]
              (perform-sub-selection project all-sub-selections)))))

      ;; invalidating outputs is the only change that does not reset the undo history
      (when (some seq (vals (dissoc plan :invalidate-outputs)))
        (g/reset-undo! (graph project))))))

(defn- handle-resource-changes [project changes render-progress!]
  (-> (resource-update/resource-change-plan (g/node-value project :nodes-by-resource-path) changes)
      ;; for debugging resource loading/reloading issues: (resource-update/print-plan)
      (perform-resource-change-plan project render-progress!)))

(g/defnk produce-collision-groups-data
  [collision-group-nodes]
  (collision-groups/make-collision-groups-data collision-group-nodes))

(defn parse-filter-param
  [_node-id ^String s]
  (cond
    (.equalsIgnoreCase "nearest" s) gl/nearest
    (.equalsIgnoreCase "linear" s) gl/linear
    :else (g/error-fatal (format "Invalid value for filter param: '%s'" s))))

(g/defnk produce-default-tex-params
  [_node-id settings]
  (let [min (parse-filter-param _node-id (get settings ["graphics" "default_texture_min_filter"]))
        mag (parse-filter-param _node-id (get settings ["graphics" "default_texture_mag_filter"]))
        errors (filter g/error? [min mag])]
    (if (seq errors)
      (g/error-aggregate errors)
      {:min-filter min
       :mag-filter mag})))

(g/defnode Project
  (inherits core/Scope)

  (property workspace g/Any)

  (property all-selections g/Any)
  (property all-sub-selections g/Any)

  (input script-intelligence g/NodeID :cascade-delete)
  (input editor-extensions g/NodeID :cascade-delete)
  (input all-selected-node-ids g/Any :array)
  (input all-selected-node-properties g/Any :array)
  (input resources g/Any)
  (input resource-map g/Any)
  (input resource-types g/Any)
  (input save-data g/Any :array :substitute gu/array-subst-remove-errors)
  (input node-id+resources g/Any :array)
  (input settings g/Any :substitute (constantly (gpc/default-settings)))
  (input display-profiles g/Any)
  (input texture-profiles g/Any)
  (input collision-group-nodes g/Any :array :substitute gu/array-subst-remove-errors)
  (input build-settings g/Any)
  (input breakpoints Breakpoints :array :substitute gu/array-subst-remove-errors)

  (output selected-node-ids-by-resource-node g/Any :cached (g/fnk [all-selected-node-ids all-selections]
                                                             (let [selected-node-id-set (set all-selected-node-ids)]
                                                               (->> all-selections
                                                                 (map (fn [[key vals]] [key (filterv selected-node-id-set vals)]))
                                                                 (into {})))))
  (output selected-node-properties-by-resource-node g/Any :cached (g/fnk [all-selected-node-properties all-selections]
                                                                    (let [props (->> all-selected-node-properties
                                                                                  (map (fn [p] [(:node-id p) p]))
                                                                                  (into {}))]
                                                                      (->> all-selections
                                                                        (map (fn [[key vals]] [key (vec (keep props vals))]))
                                                                        (into {})))))
  (output sub-selections-by-resource-node g/Any :cached (g/fnk [all-selected-node-ids all-sub-selections]
                                                               (let [selected-node-id-set (set all-selected-node-ids)]
                                                                 (->> all-sub-selections
                                                                   (map (fn [[key vals]] [key (filterv (comp selected-node-id-set first) vals)]))
                                                                   (into {})))))
  (output resource-map g/Any (gu/passthrough resource-map))
  (output nodes-by-resource-path g/Any :cached (g/fnk [node-id+resources] (make-resource-nodes-by-path-map node-id+resources)))
  (output save-data g/Any :cached (g/fnk [save-data] (filterv #(and % (:content %)) save-data)))
  (output dirty-save-data g/Any :cached (g/fnk [save-data] (filterv #(and (:dirty? %)
                                                                       (when-let [r (:resource %)]
                                                                         (not (resource/read-only? r)))) save-data)))
  (output settings g/Any :cached (gu/passthrough settings))
  (output display-profiles g/Any :cached (gu/passthrough display-profiles))
  (output texture-profiles g/Any :cached (gu/passthrough texture-profiles))
  (output nil-resource resource/Resource (g/constantly nil))
  (output collision-groups-data g/Any :cached produce-collision-groups-data)
  (output default-tex-params g/Any :cached produce-default-tex-params)
  (output build-settings g/Any (gu/passthrough build-settings))
  (output breakpoints Breakpoints :cached (g/fnk [breakpoints] (into [] cat breakpoints))))

(defn get-resource-type [resource-node]
  (when resource-node (resource/resource-type (g/node-value resource-node :resource))))

(defn get-project
  ([node]
   (get-project (g/now) node))
  ([basis node]
   (g/graph-value basis (g/node-id->graph-id node) :project-id)))

(defn find-resources [project query]
  (let [resource-path-to-node (g/node-value project :nodes-by-resource-path)
        resources        (resource/filter-resources (g/node-value project :resources) query)]
    (map (fn [r] [r (get resource-path-to-node (resource/proj-path r))]) resources)))

(defn settings [project]
  (g/node-value project :settings))

(defn project-dependencies [project]
  (when-let [settings (settings project)]
    (settings ["project" "dependencies"])))

(defn shared-script-state? [project]
  (some-> (settings project) (get ["script" "shared_state"])))

(defn project-title [project]
  (some-> project
    (settings)
    (get ["project" "title"])))

(defn- disconnect-from-inputs [basis src tgt connections]
  (let [outputs (set (g/output-labels (g/node-type* basis src)))
        inputs (set (g/input-labels (g/node-type* basis tgt)))]
    (for [[src-label tgt-label] connections
          :when (and (outputs src-label) (inputs tgt-label))]
      (g/disconnect src src-label tgt tgt-label))))

(defn resolve-path-or-resource [project path-or-resource evaluation-context]
  (if (string? path-or-resource)
    (workspace/resolve-workspace-resource (workspace project evaluation-context) path-or-resource evaluation-context)
    path-or-resource))

(defn disconnect-resource-node [evaluation-context project path-or-resource consumer-node connections]
  (let [basis (:basis evaluation-context)
        resource (resolve-path-or-resource project path-or-resource evaluation-context)
        node (get-resource-node project resource evaluation-context)]
    (disconnect-from-inputs basis node consumer-node connections)))

(defn- ensure-resource-node-created [tx-data-context project resource]
  (assert (satisfies? resource/Resource resource))
  (if-some [[_ pending-resource-node-id] (find (:created-resource-nodes tx-data-context) resource)]
    [tx-data-context pending-resource-node-id nil]
    (let [graph-id (g/node-id->graph-id project)
          node-type (resource-node-type resource)
          creation-tx-data (g/make-nodes graph-id [resource-node-id [node-type :resource resource]]
                                         (g/connect resource-node-id :_node-id project :nodes)
                                         (g/connect resource-node-id :node-id+resource project :node-id+resources))
          created-resource-node-id (first (g/tx-data-nodes-added creation-tx-data))
          tx-data-context' (assoc-in tx-data-context [:created-resource-nodes resource] created-resource-node-id)]
      [tx-data-context' created-resource-node-id creation-tx-data])))

(defn connect-resource-node
  "Creates transaction steps for creating `connections` between the
  corresponding node for `path-or-resource` and `consumer-node`. If
  there is no corresponding node for `path-or-resource`, transactions
  for creating and loading the node will be included. Returns map with
  transactions in :tx-data and node-id corresponding to
  `path-or-resource` in :node-id"
  [evaluation-context project path-or-resource consumer-node connections]
  ;; TODO: This is typically run from a property setter, where currently the
  ;; evaluation-context does not contain a cache. This makes resource lookups
  ;; very costly as they need to produce the lookup maps every time.
  ;; In large projects, this has a huge impact on load time. To work around
  ;; this, we use the default, cached evaluation-context to resolve resources.
  ;; This has been reported as DEFEDIT-1411.
  (g/with-auto-evaluation-context default-evaluation-context
    (when-some [resource (resolve-path-or-resource project path-or-resource default-evaluation-context)]
      (let [[node-id creation-tx-data] (if-some [existing-resource-node-id (get-resource-node project resource default-evaluation-context)]
                                         [existing-resource-node-id nil]
                                         (thread-util/swap-rest! (:tx-data-context evaluation-context) ensure-resource-node-created project resource))
            node-type (resource-node-type resource)]
        {:node-id node-id
         :tx-data (concat
                    creation-tx-data
                    (when (or creation-tx-data *load-cache*)
                      (load-node project node-id node-type resource))
                    (connect-if-output node-type node-id consumer-node connections))}))))

(deftype ProjectResourceListener [project-id]
  resource/ResourceListener
  (handle-changes [this changes render-progress!]
    (handle-resource-changes project-id changes render-progress!)))

(defn make-project [graph workspace-id extensions]
  (let [project-id
        (second
          (g/tx-nodes-added
            (g/transact
              (g/make-nodes graph
                  [script-intelligence si/ScriptIntelligenceNode
                   project [Project :workspace workspace-id]]
                (g/connect extensions :_node-id project :editor-extensions)
                (g/connect script-intelligence :_node-id project :script-intelligence)
                (g/connect workspace-id :build-settings project :build-settings)
                (g/connect workspace-id :resource-list project :resources)
                (g/connect workspace-id :resource-map project :resource-map)
                (g/connect workspace-id :resource-types project :resource-types)
                (g/set-graph-value graph :project-id project)))))]
    (workspace/add-resource-listener! workspace-id 1 (ProjectResourceListener. project-id))
    project-id))

(defn- read-dependencies [game-project-resource]
  (with-open [game-project-reader (io/reader game-project-resource)]
    (-> (settings-core/parse-settings game-project-reader)
        (settings-core/get-setting ["project" "dependencies"])
        (library/parse-library-uris))))

(def ^:private embedded-resource? (comp nil? resource/proj-path))

(defn project-resource-node? [node-id evaluation-context]
  (let [basis (:basis evaluation-context)]
    (and (g/node-instance? basis resource-node/ResourceNode node-id)
         (not (embedded-resource? (g/node-value node-id :resource evaluation-context))))))

(defn- cached-save-data-output? [node-id label evaluation-context]
  (case label
    (:save-data :source-value) (project-resource-node? node-id evaluation-context)
    false))

(defn update-system-cache-save-data! [evaluation-context]
  ;; To avoid cache churn, we only transfer the most important entries to the system cache.
  (let [pruned-evaluation-context (g/pruned-evaluation-context evaluation-context cached-save-data-output?)]
    (g/update-cache-from-evaluation-context! pruned-evaluation-context)))

(defn- cache-save-data! [project]
  ;; Save data is required for the Search in Files feature so we pull
  ;; it in the background here to cache it.
  (let [evaluation-context (g/make-evaluation-context)]
    (future
      (error-reporting/catch-all!
        ;; TODO: Progress reporting.
        (g/node-value project :save-data evaluation-context)
        (ui/run-later
          (update-system-cache-save-data! evaluation-context))))))

(defn open-project! [graph extensions workspace-id game-project-resource render-progress!]
  (let [dependencies (read-dependencies game-project-resource)
        progress (atom (progress/make "Updating dependencies..." 13 0))]
    (render-progress! @progress)

    ;; Fetch+install libs if we have network, otherwise fallback to disk state
    (if (workspace/dependencies-reachable? dependencies)
      (->> (workspace/fetch-and-validate-libraries workspace-id dependencies (progress/nest-render-progress render-progress! @progress 4))
           (workspace/install-validated-libraries! workspace-id dependencies))
      (workspace/set-project-dependencies! workspace-id dependencies))

    (render-progress! (swap! progress progress/advance 4 "Syncing resources..."))
    (workspace/resource-sync! workspace-id [] (progress/nest-render-progress render-progress! @progress))
    (render-progress! (swap! progress progress/advance 1 "Loading project..."))
    (let [project (make-project graph workspace-id extensions)
          populated-project (load-project project (g/node-value project :resources) (progress/nest-render-progress render-progress! @progress 8))]
      ;; Prime the auto completion cache
      (g/node-value (script-intelligence project) :lua-completions)
      (cache-save-data! populated-project)
      populated-project)))

(defn resource-setter [evaluation-context self old-value new-value & connections]
  (let [project (get-project (:basis evaluation-context) self)]
    (concat
      (when old-value (disconnect-resource-node evaluation-context project old-value self connections))
      (when new-value (:tx-data (connect-resource-node evaluation-context project new-value self connections))))))
