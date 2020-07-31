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

(ns editor.build
  (:require [clojure.set :as set]
            [dynamo.graph :as g]
            [editor.defold-project :as project]
            [editor.editor-extensions :as extensions]
            [editor.engine :as engine]
            [editor.pipeline :as pipeline]
            [editor.progress :as progress]
            [editor.workspace :as workspace]))

(defn- batched-pmap [f batches]
  (->> batches
       (pmap (fn [batch] (doall (map f batch))))
       (reduce concat)
       doall))

(defn- available-processors []
  (.availableProcessors (Runtime/getRuntime)))

(defn- compiling-progress-message [node-id->resource-path node-id]
  (if (nil? node-id)
    "Compiling..."
    (when-some [resource-path (node-id->resource-path node-id)]
      (str "Compiling " resource-path))))

(defn make-collect-progress-steps-tracer [watched-label steps-atom]
  (fn [state node output-type label]
    (when (and (= label watched-label) (= state :begin) (= output-type :output))
      (swap! steps-atom conj node))))

(defn build-project!
  [project node evaluation-context extra-build-targets old-artifact-map render-progress!]
  (if-let [extension-error (extensions/execute-hook! project :on-build-started {:exception-policy :as-error
                                                                                :opts {:platform (engine/current-platform)}})]
    (do
      (extensions/execute-hook! project :on-build-finished {:exception-policy :ignore
                                                            :opts {:success false
                                                                   :platform (engine/current-platform)}})
      {:error extension-error})
    (let [steps (atom [])
          collect-tracer (make-collect-progress-steps-tracer :build-targets steps)
          _ (g/node-value node :build-targets (assoc evaluation-context :dry-run true :tracer collect-tracer))
          progress-message-fn (partial compiling-progress-message (set/map-invert (g/node-value project :nodes-by-resource-path evaluation-context)))
          step-count (count @steps)
          progress-tracer (project/make-progress-tracer :build-targets step-count progress-message-fn (progress/nest-render-progress render-progress! (progress/make "" 10) 5))
          evaluation-context-with-progress-trace (assoc evaluation-context :tracer progress-tracer)
          prewarm-partitions (partition-all (max (quot step-count (+ (available-processors) 2)) 1000) (rseq @steps))
          _ (batched-pmap (fn [node-id] (g/node-value node-id :build-targets evaluation-context-with-progress-trace)) prewarm-partitions)
          node-build-targets (g/node-value node :build-targets evaluation-context)
          build-targets (cond-> node-build-targets
                                (seq extra-build-targets)
                                (into extra-build-targets))
          build-dir (workspace/build-path (project/workspace project))
          ret (if (g/error? build-targets)
                {:error build-targets}
                (pipeline/build! build-targets build-dir old-artifact-map (progress/nest-render-progress render-progress! (progress/make "" 10 5) 5)))]
      (extensions/execute-hook! project
                                :on-build-finished
                                {:exception-policy :ignore
                                 :opts {:success (not (:error ret))
                                        :platform (engine/current-platform)}})
      ret)))
