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

(ns integration.test-util
  (:require [clojure.java.io :as io]
            [clojure.string :as string]
            [clojure.test :refer [is testing]]
            [dynamo.graph :as g]
            [editor.graph-util :as gu]
            [editor.app-view :as app-view]
            [editor.ui :as ui]
            [editor.fs :as fs]
            [editor.game-object :as game-object]
            [editor.defold-project :as project]
            [editor.editor-extensions :as extensions]
            [editor.material :as material]
            [editor.prefs :as prefs]
            [editor.resource :as resource]
            [editor.resource-types :as resource-types]
            [editor.scene :as scene]
            [editor.scene-selection :as scene-selection]
            [editor.workspace :as workspace]
            [editor.handler :as handler]
            [editor.view :as view]
            [internal.system :as is]
            [support.test-support :as test-support]
            [util.http-server :as http-server]
            [util.thread-util :as thread-util])
  (:import [java.io File FilenameFilter FileInputStream ByteArrayOutputStream]
           [java.util UUID]
           [java.util.concurrent LinkedBlockingQueue]
           [java.util.zip ZipEntry ZipOutputStream]
           [javafx.scene Scene]
           [javafx.scene.layout VBox]
           [javax.imageio ImageIO]
           [org.apache.commons.io FilenameUtils IOUtils]))

(def project-path "test/resources/test_project")

(defn make-dir! ^File [^File dir]
  (fs/create-directory! dir))

(defn make-temp-dir! ^File []
  (fs/create-temp-directory! "foo"))

(defmacro with-temp-dir!
  "Creates a temporary directory and binds it to name as a File. Executes body
  in a try block and finally deletes the temporary directory."
  [name & body]
  `(let [~name (make-temp-dir!)]
     (try
       ~@body
       (finally
         (fs/delete! ~name {:fail :silently})))))

(defn- file-tree-helper [^File entry]
  (if (.isDirectory entry)
    {(.getName entry)
     (mapv file-tree-helper (sort-by #(.getName %) (filter #(not (.isHidden %)) (.listFiles entry))))}
    (.getName entry)))

(defn file-tree
  "Returns a vector of file tree entries below dir. A file entry is represented
  as a String file name. A directory entry is represented as a single-entry map
  where the key is the directory name and the value is a vector of tree entries."
  [^File dir]
  (second (first (file-tree-helper dir))))

(defn make-file-tree!
  "Takes a vector of file tree entries and creates a mock directory structure
  below the specified directory entry. A random UUID string will be written to
  each file."
  [^File entry file-tree]
  (doseq [decl file-tree]
    (if (map? decl)
      (let [[^String dir-name children] (first decl)
            dir-entry (make-dir! (io/file entry dir-name))]
        (make-file-tree! dir-entry children))
      (let [^String file-name decl]
        (spit (io/file entry file-name) (.toString (UUID/randomUUID)))))))

(defn make-test-prefs []
  (prefs/load-prefs "test/resources/test_prefs.json"))

(declare prop prop!)

(defn code-editor-source [script-id]
  (string/join "\n" (prop script-id :modified-lines)))

(defn code-editor-source! [script-id source]
  (prop! script-id :modified-lines (string/split source #"\r?\n" -1)))

(defn setup-workspace!
  ([graph]
   (setup-workspace! graph project-path))
  ([graph project-path]
   (let [workspace (workspace/make-workspace graph
                                             (.getAbsolutePath (io/file project-path))
                                             {})]
     (g/transact
       (concat
         (scene/register-view-types workspace)))
     (resource-types/register-resource-types! workspace)
     (workspace/resource-sync! workspace)
     workspace)))

(defn make-temp-project-copy! [project-path]
  (let [temp-project-path (-> (fs/create-temp-directory! "test")
                              (.getAbsolutePath))]
    (fs/copy-directory! (io/file project-path) (io/file temp-project-path))
    temp-project-path))

(defn setup-scratch-workspace!
  ([graph]
   (setup-scratch-workspace! graph project-path))
  ([graph project-path]
   (let [temp-project-path (make-temp-project-copy! project-path)]
     (setup-workspace! graph temp-project-path))))

(defn setup-project!
  ([workspace]
   (let [proj-graph (g/make-graph! :history true :volatility 1)
         extensions (extensions/make proj-graph)
         project (project/make-project proj-graph workspace extensions)
         project (project/load-project project)]
     (g/reset-undo! proj-graph)
     project))
  ([workspace resources]
   (let [proj-graph (g/make-graph! :history true :volatility 1)
         extensions (extensions/make proj-graph)
         project (project/make-project proj-graph workspace extensions)
         project (project/load-project project resources)]
     (g/reset-undo! proj-graph)
     project)))

(defn project-node-resources [project]
  (g/with-auto-evaluation-context evaluation-context
    (sort-by resource/proj-path
             (map (comp #(g/node-value % :resource evaluation-context) first)
                  (g/sources-of project :node-id+resources)))))

(defrecord FakeFileResource [workspace root ^File file children exists? source-type read-only? content]
  resource/Resource
  (children [this] children)
  (ext [this] (FilenameUtils/getExtension (.getPath file)))
  (resource-type [this] (get (g/node-value workspace :resource-types) (resource/ext this)))
  (source-type [this] source-type)
  (exists? [this] exists?)
  (read-only? [this] read-only?)
  (path [this] (if (= "" (.getName file)) "" (resource/relative-path (io/file ^String root) file)))
  (abs-path [this] (.getAbsolutePath  file))
  (proj-path [this] (if (= "" (.getName file)) "" (str "/" (resource/path this))))
  (resource-name [this] (.getName file))
  (workspace [this] workspace)
  (resource-hash [this] (hash (resource/proj-path this)))
  (openable? [this] (= :file source-type))

  io/IOFactory
  (io/make-input-stream  [this opts] (io/make-input-stream content opts))
  (io/make-reader        [this opts] (io/make-reader (io/make-input-stream this opts) opts))
  (io/make-output-stream [this opts] (assert false "writing to not supported"))
  (io/make-writer        [this opts] (assert false "writing to not supported")))

(defn make-fake-file-resource
  ([workspace root file content]
   (make-fake-file-resource workspace root file content nil))
  ([workspace root file content {:keys [children exists? source-type read-only?]
                                 :or {children nil
                                      exists? true
                                      source-type :file
                                      read-only? false}
                                 :as opts}]
   (FakeFileResource. workspace root file children exists? source-type read-only? content)))

(defn resource-node [project path]
  (project/get-resource-node project path))

(defn empty-selection? [app-view]
  (let [sel (g/node-value app-view :selected-node-ids)]
    (empty? sel)))

(defn selected? [app-view tgt-node-id]
  (let [sel (g/node-value app-view :selected-node-ids)]
    (not (nil? (some #{tgt-node-id} sel)))))

(g/defnode MockView
  (inherits view/WorkbenchView))

(g/defnode MockAppView
  (inherits app-view/AppView)
  (property active-view g/NodeID)
  (output active-view g/NodeID (gu/passthrough active-view)))

(defn make-view-graph! []
  (g/make-graph! :history false :volatility 2))

(defn setup-app-view! [project]
  (let [view-graph (make-view-graph!)]
    (-> (g/make-nodes view-graph [app-view [MockAppView
                                            :active-tool :move
                                            :manip-space :world
                                            :scene (Scene. (VBox.))]]
          (g/connect project :_node-id app-view :project-id)
          (for [label [:selected-node-ids-by-resource-node :selected-node-properties-by-resource-node :sub-selections-by-resource-node]]
            (g/connect project label app-view label)))
      g/transact
      g/tx-nodes-added
      first)))

(defn- make-tab! [project app-view path make-view-fn!]
  (let [node-id (project/get-resource-node project path)
        views-by-node-id (let [views (g/node-value app-view :open-views)]
                           (zipmap (map :resource-node (vals views)) (keys views)))
        view (get views-by-node-id node-id)]
    (if view
      (do
        (g/set-property! app-view :active-view view)
        [node-id view])
      (let [view-graph (g/make-graph! :history false :volatility 2)
            view (make-view-fn! view-graph node-id)]
        (g/transact
          (concat
            (g/connect node-id :_node-id view :resource-node)
            (g/connect node-id :valid-node-id+resource view :node-id+resource)
            (g/connect view :view-data app-view :open-views)
            (g/set-property app-view :active-view view)))
        (app-view/select! app-view [node-id])
        [node-id view]))))

(defn open-tab! [project app-view path]
  (first
    (make-tab! project app-view path (fn [view-graph resource-node]
                                      (->> (g/make-node view-graph MockView)
                                        g/transact
                                        g/tx-nodes-added
                                        first)))))

(defn open-scene-view! [project app-view path width height]
  (make-tab! project app-view path (fn [view-graph resource-node]
                                     (scene/make-preview view-graph resource-node {:prefs (make-test-prefs) :app-view app-view :project project :select-fn (partial app-view/select app-view)} width height))))

(defn close-tab! [project app-view path]
  (let [node-id (project/get-resource-node project path)
        view (some (fn [[view-id {:keys [resource-node]}]]
                     (when (= resource-node node-id) view-id)) (g/node-value app-view :open-views))]
    (when view
      (g/delete-graph! (g/node-id->graph-id view)))))

(defn setup!
  ([graph]
   (setup! graph project-path))
  ([graph project-path]
   (let [workspace (setup-workspace! graph project-path)
         project (setup-project! workspace)
         app-view (setup-app-view! project)]
     [workspace project app-view])))

(defn- load-system-and-project-raw [path]
  (test-support/with-clean-system
    (let [workspace (setup-workspace! world path)
          project (setup-project! workspace)]
      [@g/*the-system* workspace project])))

(def load-system-and-project (memoize load-system-and-project-raw))

(defmacro with-loaded-project
  [& forms]
  (let [custom-path?  (or (string? (first forms)) (symbol? (first forms)))
        project-path  (if custom-path? (first forms) project-path)
        forms         (if custom-path? (next forms) forms)]
    `(let [[system# ~'workspace ~'project] (load-system-and-project ~project-path)
           system-clone# (is/clone-system system#)
           ~'cache  (:cache system-clone#)
           ~'world  (g/node-id->graph-id ~'workspace)]
       (binding [g/*the-system* (atom system-clone#)]
         (let [~'app-view (setup-app-view! ~'project)]
           ~@forms)))))

(defmacro with-ui-run-later-rebound
  [& forms]
  `(let [laters# (atom [])]
     (with-redefs [ui/do-run-later (fn [f#] (swap! laters# conj f#))]
       (let [result# (do ~@forms)]
         (doseq [f# @laters#] (f#))
         result#))))

(defn run-event-loop!
  "Starts a simulated event loop and enqueues the supplied function on it.
  The function is invoked with a single argument, which is a function that must
  be called to exit the event loop. Blocks until the event loop is terminated,
  or an exception is thrown from an enqueued action. While the event loop is
  running, ui/run-now and ui/run-later are rebound to enqueue actions on the
  simulated event loop, and ui/on-ui-thread? will return true only if called
  from inside the event loop."
  [f]
  (let [ui-thread (Thread/currentThread)
        action-queue (LinkedBlockingQueue.)
        enqueue-action! (fn [action!]
                          (.add action-queue action!)
                          nil)
        exit-event-loop! #(enqueue-action! ::exit-event-loop)]
    (with-redefs [ui/on-ui-thread? #(= ui-thread (Thread/currentThread))
                  ui/do-run-later enqueue-action!]
      (enqueue-action! (fn [] (f exit-event-loop!)))
      (loop []
        (let [action! (.take action-queue)]
          (when (not= ::exit-event-loop action!)
            (action!)
            (recur)))))))

(defn set-active-tool! [app-view tool]
  (g/transact (g/set-property app-view :active-tool tool)))

(defn- fake-input!
  ([view type x y]
   (fake-input! view type x y []))
  ([view type x y modifiers]
   (fake-input! view type x y modifiers 0))
  ([view type x y modifiers click-count]
   (let [pos [x y 0.0]]
     (g/transact (g/set-property view :tool-picking-rect (scene-selection/calc-picking-rect pos pos))))
   (let [handlers (g/sources-of view :input-handlers)
         user-data (g/node-value view :selected-tool-renderables)
         action (reduce #(assoc %1 %2 true)
                        {:type type :x x :y y :click-count click-count}
                        modifiers)
         action (scene/augment-action view action)]
     (scene/dispatch-input handlers action user-data))))

(defn mouse-press!
  ([view x y]
   (fake-input! view :mouse-pressed x y))
  ([view x y modifiers]
   (fake-input! view :mouse-pressed x y modifiers 0))
  ([view x y modifiers click-count]
   (fake-input! view :mouse-pressed x y modifiers click-count)))

(defn mouse-move! [view x y]
  (fake-input! view :mouse-moved x y))

(defn mouse-release! [view x y]
  (fake-input! view :mouse-released x y))

(defn mouse-click!
  ([view x y]
   (mouse-click! view x y []))
  ([view x y modifiers]
   (mouse-click! view x y modifiers 0))
  ([view x y modifiers click-count]
   (mouse-press! view x y modifiers (inc click-count))
   (mouse-release! view x y)))

(defn mouse-dbl-click!
  ([view x y]
   (mouse-dbl-click! view x y []))
  ([view x y modifiers]
   (mouse-click! view x y modifiers)
   (mouse-click! view x y modifiers 1)))

(defn mouse-drag! [view x0 y0 x1 y1]
  (mouse-press! view x0 y0)
  (mouse-move! view x1 y1)
  (mouse-release! view x1 y1))

(defn dump-frame! [view path]
  (let [image (g/node-value view :frame)]
    (let [file (io/file path)]
      (ImageIO/write image "png" file))))

(defn outline [root path]
  (get-in (g/node-value root :node-outline) (interleave (repeat :children) path)))

(defn- outline->str
  ([outline]
   (outline->str outline "" true))
  ([outline prefix recurse?]
   (if outline
     (format "%s%s [%d] [%s]%s%s"
             (if recurse? (str prefix "* ") "")
             (:label outline "<no-label>")
             (:node-id outline -1)
             (some-> (g/node-type* (:node-id outline -1))
                     deref
                     :name)
             (if (:alt-outline outline) (format " (ALT: %s)" (outline->str (:alt-outline outline) prefix false)) "")
             (if recurse?
               (string/join (map #(str "\n" (outline->str % (str prefix "  ") true)) (:children outline)))
               ""))
     "")))

(defn dump-outline [root path]
  (-> (outline root path)
    outline->str
    println))

(defn resolve-prop [node-id label]
  (let [prop (get-in (g/node-value node-id :_properties) [:properties label])
        resolved-node-id (:node-id prop)
        resolved-label (get prop :prop-kw label)]
    [resolved-node-id resolved-label]))

(defn prop [node-id label]
  (get-in (g/node-value node-id :_properties) [:properties label :value]))

(defn prop-error [node-id label]
  (get-in (g/node-value node-id :_properties) [:properties label :error]))

(defn prop! [node-id label val]
  (let [[node-id label] (resolve-prop node-id label)]
    (g/set-property! node-id label val)))

(defn prop-clear! [node-id label]
  (let [[node-id label] (resolve-prop node-id label)]
    (g/clear-property! node-id label)))

(defn prop-read-only? [node-id label]
  (get-in (g/node-value node-id :_properties) [:properties label :read-only?]))

(defn prop-overridden? [node-id label]
  (contains? (get-in (g/node-value node-id :_properties) [:properties label]) :original-value))

(defn resource [workspace path]
  (workspace/file-resource workspace path))

(defn file ^File [workspace path]
  (File. (workspace/project-path workspace) path))

(defn selection [app-view]
  (-> app-view
    app-view/->selection-provider
    handler/selection))

;; Extension library server

(defn ->lib-server []
  (doto (http-server/->server 0 {"/lib" (fn [request]
                                          (let [lib (subs (:url request) 5)
                                                path-offset (count (format "test/resources/%s/" lib))
                                                ignored #{".internal" "build"}
                                                file-filter (reify FilenameFilter
                                                              (accept [this file name] (not (contains? ignored name))))
                                                files (->> (tree-seq (fn [^File f] (.isDirectory f)) (fn [^File f] (.listFiles f file-filter)) (io/file (format "test/resources/%s" lib)))
                                                        (filter (fn [^File f] (not (.isDirectory f)))))]
                                            (with-open [byte-stream (ByteArrayOutputStream.)
                                                        out (ZipOutputStream. byte-stream)]
                                              (doseq [f files]
                                                (with-open [in (FileInputStream. f)]
                                                  (let [entry (doto (ZipEntry. (subs (.getPath f) path-offset))
                                                                (.setSize (.length f)))]
                                                    (.putNextEntry out entry)
                                                    (IOUtils/copy in out)
                                                    (.closeEntry out))))
                                              (.finish out)
                                              (let [bytes (.toByteArray byte-stream)]
                                                {:headers {"ETag" "tag"}
                                                 :body bytes}))))})
    (http-server/start!)))

(defn kill-lib-server [server]
  (http-server/stop! server))

(defn lib-server-uri [server lib]
  (format "%s/lib/%s" (http-server/local-url server) lib))

(defn handler-run [command command-contexts user-data]
  (let [command-contexts (handler/eval-contexts command-contexts true)]
    (-> (handler/active command command-contexts user-data)
      handler/run)))

(defn handler-options [command command-contexts user-data]
  (let [command-contexts (handler/eval-contexts command-contexts true)]
    (-> (handler/active command command-contexts user-data)
      handler/options)))

(defn handler-state [command command-contexts user-data]
  (let [command-contexts (handler/eval-contexts command-contexts true)]
    (-> (handler/active command command-contexts user-data)
      handler/state)))

(defmacro with-prop [binding & forms]
  (let [[node-id# property# value#] binding]
    `(let [old-value# (g/node-value ~node-id# ~property#)]
       (g/set-property! ~node-id# ~property# ~value#)
       ~@forms
       (g/set-property! ~node-id# ~property# old-value#))))

(defn make-call-logger
  "Returns a function that keeps track of its invocations. Every
  time it is called, the call and its arguments are stored in the
  metadata associated with the returned function. If fn f is
  supplied, it will be invoked after the call is logged."
  ([]
   (make-call-logger (constantly nil)))
  ([f]
   (let [calls (atom [])]
     (with-meta (fn [& args]
                  (swap! calls conj args)
                  (apply f args))
                {::calls calls}))))

(defn call-logger-calls
  "Given a function obtained from make-call-logger, returns a
  vector of sequences containing the arguments for every time it
  was called."
  [call-logger]
  (-> call-logger meta ::calls deref))

(defmacro with-logged-calls
  "Temporarily redefines the specified functions into call-loggers
  while executing the body. Returns a map of functions to the
  result of (call-logger-calls fn). Non-invoked functions will not
  be included in the returned map.

  Example:
  (with-logged-calls [print println]
    (println :a)
    (println :a :b))
  => {#object[clojure.core$println] [(:a)
                                     (:a :b)]}"
  [var-symbols & body]
  `(let [binding-map# ~(into {}
                             (map (fn [var-symbol]
                                    `[(var ~var-symbol) (make-call-logger)]))
                             var-symbols)]
     (with-redefs-fn binding-map# (fn [] ~@body))
     (into {}
           (keep (fn [[var# call-logger#]]
                   (let [calls# (call-logger-calls call-logger#)]
                     (when (seq calls#)
                       [(deref var#) calls#]))))
           binding-map#)))

(def temp-directory-path
  (memoize
    (fn temp-directory-path []
      (let [temp-file (fs/create-temp-file!)
            parent-directory-path (.getAbsolutePath (.getParentFile temp-file))]
        (fs/delete! temp-file {:fail :silently})
        parent-directory-path))))

(defn make-directory-deleter
  "Returns an AutoCloseable that deletes the directory at the specified
  path when closed. Suitable for use with the (with-open) macro. The
  directory path must be a temp directory."
  [directory-path]
  (let [directory (io/file directory-path)]
    (assert (string/starts-with? (.getAbsolutePath directory)
                                 (temp-directory-path))
            (str "directory-path `" (.getAbsolutePath directory) "` is not a temp directory"))
    (reify java.lang.AutoCloseable
      (close [_]
        (fs/delete-directory! directory {:fail :silently})))))

(defn make-graph-reverter
  "Returns an AutoCloseable that reverts the specified graph to
  the state it was at construction time when its close method
  is invoked. Suitable for use with the (with-open) macro."
  [graph-id]
  (let [initial-undo-stack-count (g/undo-stack-count graph-id)]
    (reify java.lang.AutoCloseable
      (close [_]
        (loop [undo-stack-count (g/undo-stack-count graph-id)]
          (when (< initial-undo-stack-count undo-stack-count)
            (g/undo! graph-id)
            (recur (g/undo-stack-count graph-id))))))))

(defn add-embedded-component!
  "Adds a new instance of an embedded component to the specified
  game object node inside a transaction and makes it the current
  selection. Returns the id of the added EmbeddedComponent node."
  [app-view select-fn resource-type go-id]
  (game-object/add-embedded-component-handler {:_node-id go-id :resource-type resource-type} select-fn)
  (first (selection app-view)))

(defonce ^:private png-image-bytes
  ;; Bytes representing a single-color 256 by 256 pixel PNG file.
  ;; Source: https://www.mjt.me.uk/posts/smallest-png/
  (byte-array [0x89 0x50 0x4E 0x47  0x0D 0x0A 0x1A 0x0A  0x00 0x00 0x00 0x0D  0x49 0x48 0x44 0x52
               0x00 0x00 0x01 0x00  0x00 0x00 0x01 0x00  0x01 0x03 0x00 0x00  0x00 0x66 0xBC 0x3A
               0x25 0x00 0x00 0x00  0x03 0x50 0x4C 0x54  0x45 0xB5 0xD0 0xD0  0x63 0x04 0x16 0xEA
               0x00 0x00 0x00 0x1F  0x49 0x44 0x41 0x54  0x68 0x81 0xED 0xC1  0x01 0x0D 0x00 0x00
               0x00 0xC2 0xA0 0xF7  0x4F 0x6D 0x0E 0x37  0xA0 0x00 0x00 0x00  0x00 0x00 0x00 0x00
               0x00 0xBE 0x0D 0x21  0x00 0x00 0x01 0x9A  0x60 0xE1 0xD5 0x00  0x00 0x00 0x00 0x49
               0x45 0x4E 0x44 0xAE  0x42 0x60 0x82]))

(defn make-png-resource!
  "Adds a PNG image file to the workspace. Returns the created FileResource."
  [workspace proj-path]
  (assert (integer? workspace))
  (assert (.startsWith proj-path "/"))
  (let [resource (resource workspace proj-path)]
    (with-open [out (io/output-stream resource)]
      (.write out png-image-bytes))
    resource))

(defn make-resource!
  "Adds a new file to the workspace. Returns the created FileResource."
  [workspace proj-path]
  (assert (integer? workspace))
  (assert (.startsWith proj-path "/"))
  (let [resource (resource workspace proj-path)
        resource-type (resource/resource-type resource)
        template (workspace/template resource-type)]
    (spit resource template)
    resource))

(defn make-resource-node!
  "Adds a new file to the project. Returns the node-id of the created resource."
  [project proj-path]
  (assert (integer? project))
  (let [workspace (project/workspace project)
        resource (make-resource! workspace proj-path)]
    (workspace/resource-sync! workspace)
    (project/get-resource-node project resource)))

(defn move-file!
  "Moves or renames a file in the workspace, then performs a resource sync."
  [workspace ^String from-proj-path ^String to-proj-path]
  (let [from-file (file workspace from-proj-path)
        to-file (file workspace to-proj-path)]
    (fs/move-file! from-file to-file)
    (workspace/resource-sync! workspace [[from-file to-file]])))

(defn block-until
  "Blocks the calling thread until the supplied predicate is satisfied for the
  return value of the specified polling function or the timeout expires. Returns
  nil if the timeout expires, or the last returned value of poll-fn otherwise."
  [done? timeout-ms poll-fn! & args]
  (let [deadline (+ (System/nanoTime) (long (* timeout-ms 1000000)))]
    (loop []
      (thread-util/throw-if-interrupted!)
      (let [result (apply poll-fn! args)]
        (if (done? result)
          result
          (if (< (System/nanoTime) deadline)
            (recur)))))))

(defn test-uses-assigned-material
  [workspace project node-id material-prop shader-path gpu-texture-path]
  (let [material-node (project/get-resource-node project "/materials/test_samplers.material")]
    (testing "uses shader and texture params from assigned material "
      (with-prop [node-id material-prop (workspace/resolve-workspace-resource workspace "/materials/test_samplers.material")]
        (let [scene-data (g/node-value node-id :scene)]
          ;; Additional uniforms might be introduced during rendering.
          (is (= (dissoc (get-in scene-data shader-path) :uniforms)
                 (dissoc (g/node-value material-node :shader) :uniforms)))
          (is (= (dissoc (get-in scene-data (conj gpu-texture-path :params)) :default-tex-params)
                 (dissoc (material/sampler->tex-params (first (g/node-value material-node :samplers))) :default-tex-params))))))))

