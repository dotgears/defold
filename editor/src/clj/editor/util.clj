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

(ns editor.util
  (:require [clojure.string :as string])
  (:import [java.util Locale]
           [com.defold.editor Platform]))

(set! *warn-on-reflection* true)

(defmacro spy
  [& body]
  `(let [ret# (try ~@body (catch Throwable t# (prn t#) (throw t#)))]
     (prn ret#)
     ret#))


;; See http://mattryall.net/blog/2009/02/the-infamous-turkish-locale-bug

(defn lower-case*
  "Like clojure.string/lower-case but using root locale."
  [^CharSequence s]
  (.. s toString (toLowerCase Locale/ROOT)))

(defn upper-case*
  "Like clojure.string/upper-case but using root locale."
  [^CharSequence s]
  (.. s toString (toUpperCase Locale/ROOT)))

(defn capitalize*
  "Like clojure.string/capitalize but using root locale."
  ^String [^CharSequence s]
  (let [s (.toString s)]
    (if (< (count s) 2)
      (.toUpperCase s Locale/ROOT)
      (str (.toUpperCase (subs s 0 1) Locale/ROOT)
           (.toLowerCase (subs s 1) Locale/ROOT)))))

(defn format*
  "Like clojure.core/format but using root locale."
  ^String [fmt & args]
  (String/format Locale/ROOT fmt (to-array args)))


;; non-lazy implementation of variant of the Alphanum Algorithm:
;; http://www.davekoelle.com/alphanum.html

(defn- alphanum-chunks
  "Returns a vector of groups of consecutive digit or non-digit substrings in
  string. The strings are converted to lowercase."
  [s]
  (letfn [(make-sb [c]
            (doto (StringBuilder.) (.append c)))
          (digit? [^Character c]
            (when c (Character/isDigit c)))
          (complete-chunk [state ^StringBuilder sb]
            (case state
              :digit (bigint (.toString sb))
              :other (string/lower-case (.toString sb))))]
    (loop [[c & cs]          (seq s)
           state             (if (digit? c) :digit :other)
           ^StringBuilder sb (StringBuilder.)
           ret []]
      (if c
        (case state
          :digit (if (digit? c)
                   (recur cs :digit (.append sb c) ret)
                   (recur cs :other (make-sb c) (conj ret (complete-chunk state sb))))
          :other (if-not (digit? c)
                   (recur cs :other (.append sb c) ret)
                   (recur cs :digit (make-sb c) (conj ret (complete-chunk state sb)))))
        (cond-> ret (pos? (.length sb)) (conj (complete-chunk state sb)))))))

(defn- start-with-string [v]
  (let [v0 (first v)]
    (if (not (string? v0))
      (assoc v 0 (str v0))
      v)))

(defn- pad-nil [v l]
  (let [pad (- l (count v))]
    (into v (repeat pad nil))))

(def natural-order (reify java.util.Comparator
                     (compare [_ a b]
                       (if (and a b)
                         (let [ancs (map (comp start-with-string alphanum-chunks) [a b])
                               l (reduce max (map count ancs))
                               [a' b'] (map #(pad-nil % l) ancs)]
                           (compare a' b'))
                         (compare a b)))))

(def natural-order-key alphanum-chunks)

(defn os-raw
  "Returns :win32, :darwin or :linux"
  []
  (keyword (.. Platform getHostPlatform getOs)))

(def os (memoize os-raw))

(defn is-mac-os? []
  (= (os) :darwin))

(defn is-linux? []
  (= (os) :linux))

(defn positions [pred coll]
  (keep-indexed (fn [idx x]
                  (when (pred x)
                    idx))
                coll))

(defn join-words
  "Returns a string joining the elements in coll together in such a way that
  they form valid english syntax. Examples:
  (join-words ', ' ' or ' []) => ''
  (join-words ', ' ' or ' ['a']) => 'a'
  (join-words ', ' ' or ' ['a' 'b']) => 'a or b'
  (join-words ', ' ' or ' ['a' 'b' 'c']) => 'a, b or c'"
  ^String [separator final-separator coll]
  (string/join final-separator
               (concat
                 (some->> coll
                          butlast
                          not-empty
                          (string/join separator)
                          list)
                 (some->> coll
                          last
                          list))))

(defn dissoc-in
  "Dissociates an entry from a nested associative structure returning a new
  nested structure. k & ks is a sequence of keys. Any empty maps that result
  will not be present in the new structure."
  [m [k & ks]]
  (if ks
    (if-let [child (get m k)]
      (let [new-child (dissoc-in child ks)]
        (if (zero? (count new-child))
          (dissoc m k)
          (assoc m k new-child)))
      m)
    (dissoc m k)))
