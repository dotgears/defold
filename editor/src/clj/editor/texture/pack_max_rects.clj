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

(ns editor.texture.pack-max-rects
  (:require [dynamo.graph :as g]
            [editor.types :as types :refer [rect width height]]
            [editor.geom :refer [area split-rect]]
            [editor.texture.math :refer [binary-search-start binary-search-current binary-search-next]]
            [schema.core :as s])
  (:import [editor.types Rect Image TexturePacking]))

(set! *warn-on-reflection* true)

;; ---------------------------------------------------------------------------
;; Configuration parameters
;; ---------------------------------------------------------------------------
(def texture-min-width    64)
(def texture-max-width  4096)
(def texture-min-height   64)
(def texture-max-height 4096)

;; ---------------------------------------------------------------------------
;; Packing algorithm
;; ---------------------------------------------------------------------------

(defn- short-side-fit
  [^Rect r1 ^Rect r2]
  (min (Math/abs ^int (- (width r1)  (width r2)))
       (Math/abs ^int (- (height r1) (height r2)))))

(s/defn ^:private area-fit :- s/Int
  [r1 :- Rect r2 :- Rect]
  (let [a1 (area r1)
        a2 (area r2)]
    (when (> a1 a2)
      (/ a2 a1))))

(s/defn score-rect :- [(s/one s/Int "score") (s/one Rect "free-rect")]
  [free-rect :- Rect rect :- Rect]
  (if (and  (>= (:width free-rect) (:width rect))
            (>= (:height free-rect) (:height rect)))
    [(area-fit free-rect rect) free-rect]
    [nil free-rect]))

; find the best free-rect into which to place rect
(s/defn score-rects :- [[(s/one s/Int "score") (s/one Rect "free-rect")]]
  "Sort the free-rects according to how well rect fits into them.
   A 'good fit' means the least amount of leftover area."
  [free-rects :- [Rect] {:keys [width height] :as rect} :- Rect]
  (reverse (sort-by first (map score-rect free-rects (repeat rect)))))

(s/defn place-in :- Rect
  [container :- Rect content :- Rect]
  (assoc content
         :x (.x container)
         :y (.y container)))

(def max-width 2048)

(s/defn with-top-right-margin :- Rect
  [margin :- s/Int r :- Rect]
  (assoc r
         :width (+ margin (:width r))
         :height (+ margin (:height r))))

(def ^:dynamic *debug-packing* true)
(defmacro debug [& forms] (when *debug-packing* `(do ~@forms)))
(def trace (atom []))

(s/defn pack-at-size :- TexturePacking
  [margin :- s/Int sources :- [Rect] space-available :- Rect]
  (loop [free-rects   [space-available]
         remaining    (reverse (sort-by area sources))
         placed       []]
    (debug
      (swap! trace conj {:free-rects free-rects :remaining remaining :placed placed}))
    (if-let [unplaced (first remaining)]
      (let [scored-rects          (score-rects free-rects (with-top-right-margin margin unplaced))
            [best-score best-fit] (first scored-rects)
            remaining-free        (map second (rest scored-rects))]
        (if (nil? best-score)
          nil
          (let [newly-placed (place-in best-fit unplaced)]
            (recur (reverse (sort-by area (concat remaining-free (split-rect best-fit (with-top-right-margin margin newly-placed)))))
                   (rest remaining)
                   (conj placed newly-placed)))))
      (types/->TexturePacking space-available nil placed sources []))))

; ---------------------------------------------------------------------------
; Binary search of sizes
; ---------------------------------------------------------------------------
(defn- occupancy
  [{:keys [aabb sources]}]
  (if-not (nil? aabb)
    (/ (reduce + (map area sources)) (area aabb))
    0))

(defn- keep-best
  [p1 p2]
  (cond
    (nil? p1)                         p2
    (nil? p2)                         p1
    (> (occupancy p1) (occupancy p2)) p1
    :else                             p2))

(defn- pack-width-search
  [height min-width max-width margin sources]
  (loop [width-search  (binary-search-start min-width texture-max-width)
         best-so-far   nil]
    (if-let [current-width (binary-search-current width-search)]
      (let [packing (pack-at-size margin sources (Rect. "" 0 0 current-width height))]
        (recur
          (binary-search-next width-search (nil? packing))
          (keep-best best-so-far packing)))
      best-so-far)))

(defn- pack-height-search
  [min-height max-height min-width max-width margin sources]
  (loop [height-search (binary-search-start min-height max-height)
         best-so-far   nil]
    (if-let [current-height (binary-search-current height-search)]
      (let [packing-at-height (pack-width-search current-height min-width max-width margin sources)]
        (recur
          (binary-search-next height-search (nil? packing-at-height))
          (keep-best best-so-far packing-at-height)))
      best-so-far)))

(defn pack-sources
  [margin sources]
  (debug
    (reset! trace []))
  (let [min-width     (max texture-min-width (reduce min (map :width sources)))
        min-height    (max texture-min-height (reduce min (map :height sources)))]
    (pack-height-search min-height texture-max-height min-width texture-max-width margin sources)))

; ---------------------------------------------------------------------------
; External API
; ---------------------------------------------------------------------------
(s/defn max-rects-packing :- TexturePacking
  ([sources :- [Rect]]
    (max-rects-packing 0 sources))
  ([margin :- s/Int sources :- [Rect]]
    (case (count sources)
      0   :packing-failed
      1   (types/->TexturePacking (first sources) nil sources sources [])
      (pack-sources margin sources))))
