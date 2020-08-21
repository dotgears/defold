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

(ns editor.gviz
  (:require [dynamo.graph :as g]
            [clojure.java.io :as io]
            [clojure.string :as string]
            [editor.ui :as ui]
            [editor.fs :as fs])
  (:import [java.io File BufferedWriter StringWriter IOException]))

(set! *warn-on-reflection* true)

(defn installed? []
  (try
    (let [p (.exec (Runtime/getRuntime) "dot -V")]
      (.waitFor p)
      (= 0 (.exitValue p)))
    (catch IOException e
      false)))

(defn- source [[source _ _ _]] source)
(defn- target [[_ _ target _]] target)

(defn- gen-file [ext] (fs/create-temp-file! "graph" ext))

(defonce ^:private ^:dynamic ^File *dot-file* (gen-file ".dot"))
(defonce ^:private ^:dynamic ^File *png-file* (gen-file ".png"))

(defn- writer ^BufferedWriter []
  (BufferedWriter. (try
                     (io/writer *dot-file*)
                     (catch Throwable t
                       (alter-var-root #'*dot-file* (fn [_] (gen-file ".png")))
                       (io/writer *dot-file*)))))

(defn- write [^BufferedWriter w ^String s]
  (.write w s)
  (.newLine w))

(defn- node-type-name [basis node-id]
  (when-let [t (g/node-type* basis node-id)]
    (:name @t)))

(defn- flatten-arcs [arcs]
  (mapcat (fn [[nid label-arc]] (mapcat second label-arc)) arcs))

(defn- nodes->arcs [basis nodes]
  (let [nodes (set nodes)]
    (seq (into #{} (mapcat (fn [[_ graph]]
                          (->>
                            (concat (filter (fn [a] (or (nodes (:source-id a)) (nodes (:target-id a)))) (flatten-arcs (:sarcs graph)))
                                    (filter #(or (nodes (:source-id %)) (nodes (:target-id %))) (flatten-arcs (:tarcs graph))))
                            (map (fn [a] [(:source-id a) (:source-label a) (:target-id a) (:target-label a)]))))
                           (:graphs basis))))))

(defn escape-field-label [label]
  (-> label
      (string/replace " " "\\ ")
      (string/replace "<" "\\<")
      (string/replace ">" "\\>")
      (string/replace "|" "\\|")))

(defn write-node [w basis node-id color inputs outputs]
  (let [inputs (map name (set (inputs node-id)))
        outputs (map name (set (outputs node-id)))
        node-label (node-type-name basis node-id)
        color (if node-label color "red")
        node-label (or node-label "Unknown")
        label (let [input-fields (string/join "|" (map (fn [x] (format "<_in-%s>%s" (escape-field-label x) (escape-field-label x))) inputs))
                    input-fields (when-not (string/blank? input-fields)
                                   (str "{" input-fields "}"))
                    output-fields (string/join "|" (map (fn [x] (format "<_out-%s>%s" (escape-field-label x) (escape-field-label x))) outputs))
                    output-fields (when-not (string/blank? output-fields)
                                    (str "{" output-fields "}"))
                    fields (string/join "|" (remove string/blank? [input-fields output-fields]))
                    fields (when-not (string/blank? fields)
                             (str "{" fields "}"))]
                (str node-label "|<_nid>" node-id (when-not (string/blank? fields) "|") fields))]
    (write w (format "%s [shape=record, label=\"%s\", color=\"%s\", fontcolor=\"%s\"];" node-id label color color))))

(defn- include-overrides [basis nodes]
  (let [child-fn (partial g/overrides basis)]
    (mapcat #(tree-seq (constantly true) child-fn %) nodes)))

(defn- extract-nodes [basis opts]
  (->> (if (:root-id opts)
         (let [root-id (:root-id opts)
               input-fn (:input-fn opts (constantly false))
               output-fn (:output-fn opts (constantly false))]
           (mapcat
             (fn [[f arcs-fn key]]
               (g/pre-traverse basis [root-id] (fn [basis node-id]
                                                 (map key (filter f (arcs-fn basis node-id))))))
             [[input-fn g/inputs first]
              [output-fn g/outputs (comp last butlast)]]))
         (mapcat (comp keys :nodes second) (:graphs basis)))
    (include-overrides basis)))

(defn subgraph->dot ^String [basis & {:keys [root-id input-fn output-fn] :or {root-id nil} :as opts}]
  (let [nodes (extract-nodes basis opts)
        arcs (nodes->arcs basis nodes) inputs (reduce (fn [inputs [s sl t tl]] (update inputs t conj tl)) {} arcs)
        outputs (reduce (fn [outputs [s sl t tl]] (update outputs s conj sl)) {} arcs)
        sw (StringWriter.)
        node-set (set nodes)
        all-nodes (set (mapcat (comp keys :nodes second) (:graphs basis)))
        referred-nodes (mapcat (fn [[s _ t _]] [s t]) arcs)
        excluded-nodes (set (filter (complement node-set) referred-nodes))]
    (with-open [w (BufferedWriter. sw)]
      (write w "digraph G {")
      (write w "rankdir=LR")

      (doseq [[gid node-ids] (group-by g/node-id->graph-id (concat nodes excluded-nodes))]
        (write w (format "subgraph %d {" gid))
        (doseq [node-id node-ids
                :let [color (if (excluded-nodes node-id) "grey" "black")]]
          (write-node w basis node-id color inputs outputs))
        (write w "}"))

      (doseq [[source source-label target target-label] arcs]
        (let [color (if (or (not (all-nodes source)) (not (all-nodes target)))
                      "red"
                      (if (or (excluded-nodes source) (excluded-nodes target))
                        "grey"
                        "black"))]
          (write w (format "%s:\"_out-%s\" -> %s:\"_in-%s\" [color=\"%s\"];" source (name source-label) target (name target-label) color))))
      (doseq [node-id nodes]
        (when-let [original (g/override-original basis node-id)]
          (write w (format "%s:\"_nid\" -> %s:\"_nid\" [style=dashed];" node-id original))))
      (write w "}"))
    (.toString sw)))

(defn dot->image ^File [^String dot]
  (with-open [w (writer)]
    (.write w dot))
  (let [p (.exec (Runtime/getRuntime) (format "dot %s -Tpng -o%s" *dot-file* *png-file*))]
    (.waitFor p)
    (when (= 0 (.exitValue p))
      *png-file*)))

(defn show [basis & {:keys [root-id input-fn output-fn] :or {root-id nil} :as opts}]
  (let [f (-> (apply subgraph->dot basis (mapcat identity opts))
            (dot->image))]
    (when f
      (ui/open-file f))))
