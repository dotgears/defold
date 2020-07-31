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

(ns internal.util
  "Helpful utility functions for graph"
  (:require [camel-snake-kebab :refer [->Camel_Snake_Case_String]]
            [clojure.set :as set]
            [clojure.string :as str]
            [schema.core :as s])
  (:import [clojure.lang IEditableCollection]))

(set! *warn-on-reflection* true)

(defn predicate? [x] (instance? schema.core.Predicate x))
(defn schema?    [x] (satisfies? s/Schema x))
(defn protocol?  [x] (and (map? x) (contains? x :on-interface)))

(defn var-get-recursive [var-or-value]
  (if (var? var-or-value)
    (recur (var-get var-or-value))
    var-or-value))

(defn apply-if-fn [f & args]
  (if (fn? f)
    (apply f args)
    f))

(defn removev [pred coll]
  (filterv (complement pred) coll))

(defn conjv [coll x]
  (conj (or coll []) x))

(defn distinct-by
  "Returns a lazy sequence of the elements of coll with duplicates removed.
  Elements are considered equal if they return equivalent values for keyfn.
  If coll does contain duplicate elements, only the first equivalent is kept.
  Returns a stateful transducer when no collection is provided."
  ([keyfn]
   (fn [rf]
     (let [seen (volatile! #{})]
       (fn
         ([] (rf))
         ([result] (rf result))
         ([result input]
          (let [key (keyfn input)]
            (if (contains? @seen key)
              result
              (do (vswap! seen conj key)
                  (rf result input)))))))))
  ([keyfn coll]
   (let [step (fn step [xs seen]
                (lazy-seq
                  ((fn [[f :as xs] seen]
                     (when-let [s (seq xs)]
                       (let [key (keyfn f)]
                         (if (contains? seen key)
                           (recur (rest s) seen)
                           (cons f (step (rest s) (conj seen key)))))))
                    xs seen)))]
     (step coll #{}))))

(defn group-into
  "Like core.group-by, but you can specify the associative container to group
  into, as well as the empty collection to use for the groups. If the optional
  value-fn is supplied, it will be used to transform each element before adding
  them to the groups."
  ([groups-container empty-group key-fn coll]
   (group-into groups-container empty-group key-fn nil coll))
  ([groups-container empty-group key-fn value-fn coll]
   (assert (associative? groups-container))
   (assert (coll? empty-group))
   (let [group-transient? (instance? IEditableCollection empty-group)
         group-prepare (if group-transient? transient identity)
         group-conj (if group-transient? conj! conj)
         group-finish (if-not group-transient?
                        identity
                        (fn [transient-group]
                          (with-meta (persistent! transient-group)
                                     (meta empty-group))))
         groups-container-transient? (instance? IEditableCollection groups-container)
         groups-container-prepare (if groups-container-transient? transient identity)
         groups-container-assoc (if groups-container-transient? assoc! assoc)
         groups-container-finish (cond
                                   (and groups-container-transient? group-transient?)
                                   (fn [transient-groups-container]
                                     (with-meta (persistent!
                                                  (reduce-kv (fn [transient-groups-container key value]
                                                               (assoc! transient-groups-container key (group-finish value)))
                                                             (transient (empty groups-container))
                                                             (persistent! transient-groups-container)))
                                                (meta groups-container)))

                                   groups-container-transient?
                                   (fn [transient-groups-container]
                                     (with-meta (persistent! transient-groups-container)
                                                (meta groups-container)))

                                   group-transient?
                                   (fn [groups-container]
                                     (reduce-kv (fn [groups-container key value]
                                                  (assoc groups-container key (group-finish value)))
                                                (empty groups-container)
                                                groups-container))

                                   :else
                                   identity)]
     (groups-container-finish
       (reduce (fn [groups-container elem]
                 (let [key (key-fn elem)
                       value (if value-fn (value-fn elem) elem)
                       group (or (get groups-container key)
                                 (group-prepare empty-group))]
                   (groups-container-assoc groups-container key (group-conj group value))))
               (groups-container-prepare groups-container)
               coll)))))

(defn filterm
  "like filter but applies the predicate to each key value pair of the map"
  [pred m]
  (into {} (filter pred m)))

(defn key-set
  [m]
  (set (keys m)))

(defn map-keys
  [f m]
  (reduce-kv (fn [m k v] (assoc m (f k) v)) (empty m) m))

(defn map-vals
  [f m]
  (reduce-kv (fn [m k v] (assoc m k (f v))) m m))

(defn map-first
  [f pairs]
  (mapv (fn [[a b]] [(f a) b]) pairs))

(defn map-diff
  [m1 m2 sub-fn]
  (reduce-kv
    (fn [result k v]
      (if (contains? result k)
        (let [ov (get result k)]
          (let [nv (sub-fn ov v)]
            (if (empty? nv)
              (dissoc result k)
              (assoc result k nv))))
        result))
    m1
    m2))

(defn parse-number
  "Reads a number from a string. Returns nil if not a number."
  [s]
  (when (re-find #"^-?\d+\.?\d*$" s)
    (read-string (str/replace s #"^(-?)0*(\d+\.?\d*)$" "$1$2"))))

(defn parse-int
  "Reads an integer from a string. Returns nil if not an integer."
  [s]
  (when (re-find #"^-?\d+\.?0*$" s)
    (read-string (str/replace s #"^(-?)0*(\d+)\.?0*$" "$1$2"))))

(defn replace-top
  [stack value]
  (conj (pop stack) value))

(defn apply-deltas
  [old new removed-f added-f]
  (let [removed (set/difference old new)
        added   (set/difference new old)]
    [(removed-f removed) (added-f added)]))

(defn keyword->label
  "Turn a keyword into a human readable string"
  [k]
  (-> k
    name
    ->Camel_Snake_Case_String
    (str/replace "_" " ")))

(def safe-inc (fnil inc 0))

(defn collify
  [val-or-coll]
  (if (and (coll? val-or-coll) (not (map? val-or-coll))) val-or-coll [val-or-coll]))

(defn stackify [x]
  (if (coll? x)
    (if (vector? x)
      x
      (vec x))
    [x]))

(defn project
  "Like clojure.set/project, but returns a vector of tuples instead of
  a set of maps."
  [xrel ks]
  (with-meta (vec (map #(vals (select-keys % ks)) xrel)) (meta xrel)))

(defn filter-vals [p m] (select-keys m (for [[k v] m :when (p v)] k)))

(defn guard [f g]
  (fn [& args]
    (when (apply f args)
      (apply g args))))

(defn var?! [v] (when (var? v) v))

(defn vgr [s] (some-> s resolve var?! var-get))

(defn protocol-symbol?
  [x]
  (and (symbol? x)
       (let [x' (vgr x)]
         (and (map? x') (contains? x' :on-interface)))))

(defn class-symbol?
  [x]
  (and (symbol? x)
       (class? (resolve x))))

(defn assert-form-kind [place required-kind-label required-kind-pred label form]
  (assert (not (nil? form))
          (str place " " label " requires a " required-kind-label " but got nil"))
  (assert (required-kind-pred form)
          (str place " " label " requires a " required-kind-label " not '" form "' of type " (type form))))

(defn fnk-arguments [f] (set (:arguments (meta f))))

(defn pfnk?
  "True if the function a valid production function"
  [f]
  (and (fn? f) (contains? (meta f) :arguments)))

(defn pfnksymbol? [x] (or (pfnk? x) (and (symbol? x) (pfnk? (vgr x)))))
(defn pfnkvar?    [x] (or (pfnk? x) (and (var? x) (fn? (var-get-recursive x)))))

(defn- quoted-var? [x] (and (seq? x) (= 'var (first x))))
(defn- pfnk-form?  [x] (and (seq? x) (symbol? (first x)) (= "fnk" (name (first x)))))

(defn- maybe-expand-macros
  [[call & _ :as body]]
  (if (or (= "fn" (name call)) (= "fnk" (name call)))
    body
    (macroexpand-1 body)))

(defn- parse-fnk-arguments
  [f]
  (let [f (maybe-expand-macros f)]
    (loop [args  #{}
           forms (seq (second f))]
      (if-let [arg (first forms)]
        (if (or (= :- (first forms))
                (= :as (first forms)))
          (recur args (drop 2 forms))
          (recur (conj args (keyword (first forms))) (next forms)))
        args))))

(defn inputs-needed [x]
  (cond
    (pfnk? x)       (fnk-arguments x)
    (symbol? x)     (inputs-needed (resolve x))
    (var? x)        (inputs-needed (var-get x))
    (quoted-var? x) (inputs-needed (var-get (resolve (second x))))
    (pfnk-form? x)  (parse-fnk-arguments x)
    :else           #{}))

(defn- wrap-protocol
  [tp tp-form]
  (if (protocol? tp)
    (list `s/protocol tp-form)
    tp-form))

(defn- wrap-enum
  [tp tp-form]
  (if (set? tp)
    (do (println tp-form)
      (list `s/enum tp-form))
    tp-form))

(defn update-paths
  [m paths xf]
  (reduce (fn [m [p v]] (assoc-in m p (xf p v (get-in m p)))) m paths))

(defn seq-starts-with?
  [coll subcoll]
  (if-not (seq subcoll)
    true
    (if-not (seq coll)
      false
      (and (= (first coll) (first subcoll))
           (recur (next coll) (next subcoll))))))

(defn count-where
  "Count the number of elements in coll where pred returns true. If max-counted
  is specified, will stop and return max-counted once the specified number of
  elements have passed the predicate."
  (^long [pred coll]
   (count-where Long/MAX_VALUE pred coll))
  (^long [^long max-counted pred coll]
   (assert (not (neg? max-counted)))
   (reduce (fn [^long num-counted elem]
             (if (= max-counted num-counted)
               (reduced max-counted)
               (if (pred elem)
                 (inc num-counted)
                 num-counted)))
           0
           coll)))

(defn first-where
  "Returns the first element in coll where pred returns true, or nil if there was
  no matching element. If coll is a map, key-value pairs are passed to pred."
  [pred coll]
  (loop [elems coll]
    (when (seq elems)
      (let [elem (first elems)]
        (if (pred elem)
          elem
          (recur (next elems)))))))

(defn first-index-where
  "Returns the index of the first element in coll where pred returns true,
  or nil if there was no matching element. If coll is a map, key-value
  pairs are passed to pred."
  [pred coll]
  (loop [index 0
         elems coll]
    (cond
      (empty? elems) nil
      (pred (first elems)) index
      :else (recur (inc index) (next elems)))))

(defn only
  "Returns the only element in coll, or nil if there are more than one element."
  [coll]
  (when (nil? (next coll))
    (first coll)))

(defn map->sort-key
  "Given any map, returns a vector that can be used to order it relative to any other map in a deterministic order."
  [m]
  (into []
        (mapcat (juxt identity (partial get m)))
        (sort (keys m))))

(declare select-keys-deep)

(defn- select-keys-deep-value-helper [kept-keys value]
  (cond
    (map? value)
    (select-keys-deep value kept-keys)

    (coll? value)
    (into (empty value)
          (map (partial select-keys-deep-value-helper kept-keys))
          value)

    :else
    value))

(defn select-keys-deep
  "Like select-keys, but applies the filter recursively to nested data structures."
  [m kept-keys]
  (assert (or (nil? m) (map? m)))
  (with-meta (into (if (or (nil? m) (record? m))
                     {}
                     (empty m))
                   (keep (fn [key]
                           (when-some [[_ value] (find m key)]
                             [key (select-keys-deep-value-helper kept-keys value)])))
                   kept-keys)
             (meta m)))
