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

(ns editor.console
  (:require [clojure.string :as string]
            [dynamo.graph :as g]
            [editor.code.data :as data]
            [editor.code.resource :as r]
            [editor.code.util :refer [re-match-result-seq split-lines]]
            [editor.code.view :as view]
            [editor.graph-util :as gu]
            [editor.handler :as handler]
            [editor.resource :as resource]
            [editor.ui :as ui]
            [editor.workspace :as workspace])
  (:import [clojure.lang PersistentQueue]
           [editor.code.data Cursor CursorRange LayoutInfo Rect]
           [java.util.regex MatchResult]
           [javafx.beans.property SimpleStringProperty]
           [javafx.scene Parent Scene]
           [javafx.scene.canvas Canvas GraphicsContext]
           [javafx.scene.control Button Tab TabPane TextField]
           [javafx.scene.input Clipboard KeyCode KeyEvent MouseEvent ScrollEvent]
           [javafx.scene.layout GridPane Pane]
           [javafx.scene.paint Color]
           [javafx.scene.text Font FontSmoothingType TextAlignment]))

(set! *warn-on-reflection* true)
(set! *unchecked-math* :warn-on-boxed)

(def ^:private pending-atom
  (atom {:clear? false :entries PersistentQueue/EMPTY}))

(def ^:private gutter-bubble-font
  (Font. "Source Sans Pro", 10.5))

(def ^:private gutter-bubble-glyph-metrics
  (view/make-glyph-metrics gutter-bubble-font 1.0))

(defn append-console-entry!
  "Append to the console. Callable from a background thread. If type is
  non-nil, a region of the specified type will encompass the line."
  [type line]
  (assert (or (nil? type) (keyword? type)))
  (assert (string? line))
  (swap! pending-atom update :entries conj [type line]))

(def append-console-line! (partial append-console-entry! nil))

(defn clear-console!
  "Clear the console. Callable from a background thread."
  []
  (reset! pending-atom {:clear? true :entries PersistentQueue/EMPTY}))

(defn- pop-n [coll ^long n]
  (let [max-n (min n (count coll))]
    (loop [i 0
           xs coll]
      (if (= max-n i)
        xs
        (recur (inc i) (pop xs))))))

(defn- dequeue-pending! [n]
  (let [batch (first (swap-vals!
                       pending-atom
                       (fn [{:keys [entries]}]
                         {:clear? false
                          :entries (pop-n entries n)})))]
    (update batch :entries #(take n %))))

;; -----------------------------------------------------------------------------
;; Tool Bar
;; -----------------------------------------------------------------------------

(defonce ^SimpleStringProperty find-term-property (doto (SimpleStringProperty.) (.setValue "")))

(defn- setup-tool-bar!
  ^Parent [^Parent tool-bar view-node]
  (ui/with-controls tool-bar [^TextField search-console ^Button prev-console ^Button next-console ^Button clear-console]
    (ui/context! tool-bar :console-tool-bar {:term-field search-console :view-node view-node} nil)
    (.bindBidirectional (.textProperty search-console) find-term-property)
    (ui/bind-keys! search-console {KeyCode/ENTER :find-next})
    (ui/bind-action! prev-console :find-prev)
    (ui/bind-action! next-console :find-next)
    (ui/bind-action! clear-console :clear-console))
  tool-bar)

(defn- dispose-tool-bar! [^Parent tool-bar]
  (ui/with-controls tool-bar [^TextField search-console]
    (.unbindBidirectional (.textProperty search-console) find-term-property)))

(defn- focus-term-field! [^Parent bar]
  (ui/with-controls bar [^TextField search-console]
    (.requestFocus search-console)
    (.selectAll search-console)))

(defn- set-find-term! [^String term-text]
  (.setValue find-term-property (or term-text "")))

(defn- find-next! [view-node]
  (view/set-properties! view-node :selection
                        (data/find-next (view/get-property view-node :lines)
                                        (view/get-property view-node :cursor-ranges)
                                        (view/get-property view-node :layout)
                                        (split-lines (.getValue find-term-property))
                                        false
                                        false
                                        true)))

(defn- find-prev! [view-node]
  (view/set-properties! view-node :selection
                        (data/find-prev (view/get-property view-node :lines)
                                        (view/get-property view-node :cursor-ranges)
                                        (view/get-property view-node :layout)
                                        (split-lines (.getValue find-term-property))
                                        false
                                        false
                                        true)))

(handler/defhandler :find-text :console-view
  (run [term-field view-node]
       (when-some [selected-text (view/non-empty-single-selection-text view-node)]
         (set-find-term! selected-text))
       (focus-term-field! term-field)))

(handler/defhandler :find-next :console-view
  (run [view-node] (find-next! view-node)))

(handler/defhandler :find-next :console-tool-bar
  (run [view-node] (find-next! view-node)))

(handler/defhandler :find-prev :console-view
  (run [view-node] (find-prev! view-node)))

(handler/defhandler :find-prev :console-tool-bar
  (run [view-node] (find-prev! view-node)))

;; -----------------------------------------------------------------------------
;; Read-only code view action handlers
;; -----------------------------------------------------------------------------

(handler/defhandler :select-up :console-view
  (run [view-node] (view/move! view-node :selection :up)))

(handler/defhandler :select-down :console-view
  (run [view-node] (view/move! view-node :selection :down)))

(handler/defhandler :select-left :console-view
  (run [view-node] (view/move! view-node :selection :left)))

(handler/defhandler :select-right :console-view
  (run [view-node] (view/move! view-node :selection :right)))

(handler/defhandler :prev-word :console-view
  (run [view-node] (view/move! view-node :navigation :prev-word)))

(handler/defhandler :select-prev-word :console-view
  (run [view-node] (view/move! view-node :selection :prev-word)))

(handler/defhandler :next-word :console-view
  (run [view-node] (view/move! view-node :navigation :next-word)))

(handler/defhandler :select-next-word :console-view
  (run [view-node] (view/move! view-node :selection :next-word)))

(handler/defhandler :beginning-of-line :console-view
  (run [view-node] (view/move! view-node :navigation :line-start)))

(handler/defhandler :select-beginning-of-line :console-view
  (run [view-node] (view/move! view-node :selection :line-start)))

(handler/defhandler :beginning-of-line-text :console-view
  (run [view-node] (view/move! view-node :navigation :home)))

(handler/defhandler :select-beginning-of-line-text :console-view
  (run [view-node] (view/move! view-node :selection :home)))

(handler/defhandler :end-of-line :console-view
  (run [view-node] (view/move! view-node :navigation :end)))

(handler/defhandler :select-end-of-line :console-view
  (run [view-node] (view/move! view-node :selection :end)))

(handler/defhandler :page-up :console-view
  (run [view-node] (view/page-up! view-node :navigation)))

(handler/defhandler :select-page-up :console-view
  (run [view-node] (view/page-up! view-node :selection)))

(handler/defhandler :page-down :console-view
  (run [view-node] (view/page-down! view-node :navigation)))

(handler/defhandler :select-page-down :console-view
  (run [view-node] (view/page-down! view-node :selection)))

(handler/defhandler :beginning-of-file :console-view
  (run [view-node] (view/move! view-node :navigation :file-start)))

(handler/defhandler :select-beginning-of-file :console-view
  (run [view-node] (view/move! view-node :selection :file-start)))

(handler/defhandler :end-of-file :console-view
  (run [view-node] (view/move! view-node :navigation :file-end)))

(handler/defhandler :select-end-of-file :console-view
  (run [view-node] (view/move! view-node :selection :file-end)))

(handler/defhandler :copy :console-view
  (enabled? [view-node evaluation-context] (view/has-selection? view-node evaluation-context))
  (run [view-node clipboard] (view/copy! view-node clipboard)))

(handler/defhandler :select-all :console-view
  (run [view-node] (view/select-all! view-node)))

(handler/defhandler :select-next-occurrence :console-view
  (run [view-node] (view/select-next-occurrence! view-node)))

(handler/defhandler :select-next-occurrence :console-tool-bar
  (run [view-node] (view/select-next-occurrence! view-node)))

(handler/defhandler :split-selection-into-lines :console-view
  (run [view-node] (view/split-selection-into-lines! view-node)))

;; -----------------------------------------------------------------------------
;; Console view action handlers
;; -----------------------------------------------------------------------------

(handler/defhandler :clear-console :console-tool-bar
  (run [view-node] (clear-console!)))

;; -----------------------------------------------------------------------------
;; Setup
;; -----------------------------------------------------------------------------

(g/defnode ConsoleNode
  (property indent-type r/IndentType (default :two-spaces))
  (property cursor-ranges r/CursorRanges (default [data/document-start-cursor-range]) (dynamic visible (g/constantly false)))
  (property invalidated-rows r/InvalidatedRows (default []) (dynamic visible (g/constantly false)))
  (property modified-lines r/Lines (default [""]) (dynamic visible (g/constantly false)))
  (property regions r/Regions (default []) (dynamic visible (g/constantly false)))
  (output lines r/Lines (gu/passthrough modified-lines)))

(defn- gutter-metrics []
  [44.0 0.0])

(defn- draw-gutter! [^GraphicsContext gc ^Rect gutter-rect ^LayoutInfo layout ^Font font color-scheme lines regions]
  (let [glyph-metrics (.glyph layout)
        ^double line-height (data/line-height glyph-metrics)
        ^double ascent (data/ascent glyph-metrics)
        ^double gutter-bubble-ascent (data/ascent gutter-bubble-glyph-metrics)
        visible-regions (data/visible-cursor-ranges lines layout regions)
        text-right (Math/floor (- (+ (.x gutter-rect) (.w gutter-rect)) (/ line-height 2.0) 3.0))
        bubble-background-color (Color/valueOf "rgba(255, 255, 255, 0.1)")
        ^Color gutter-foreground-color (view/color-lookup color-scheme "editor.gutter.foreground")
        gutter-background-color (view/color-lookup color-scheme "editor.gutter.background")
        gutter-shadow-color (view/color-lookup color-scheme "editor.gutter.shadow")
        gutter-eval-expression-color (view/color-lookup color-scheme "editor.gutter.eval.expression")
        gutter-eval-error-color (view/color-lookup color-scheme "editor.gutter.eval.error")
        gutter-eval-result-color (view/color-lookup color-scheme "editor.gutter.eval.result")]

    ;; Draw gutter background and shadow when scrolled horizontally.
    (when (neg? (.scroll-x layout))
      (.setFill gc gutter-background-color)
      (.fillRect gc (.x gutter-rect) (.y gutter-rect) (.w gutter-rect) (.h gutter-rect))
      (.setFill gc gutter-shadow-color)
      (.fillRect gc (+ (.x gutter-rect) (.w gutter-rect)) 0.0 8.0 (.h gutter-rect)))

    ;; Draw gutter annotations.
    (.setFontSmoothingType gc FontSmoothingType/LCD)
    (.setTextAlign gc TextAlignment/RIGHT)
    (doseq [^CursorRange region visible-regions]
      (let [line-y (data/row->y layout (.row ^Cursor (.from region)))]
        (case (:type region)
          :repeat
          (let [^long repeat-count (:count region)
                overflow? (<= 1000 repeat-count)
                repeat-text (if overflow?
                              (format "+9%02d" (mod repeat-count 100))
                              (str repeat-count))
                text-top (+ 2.0 line-y)
                text-bottom (+ gutter-bubble-ascent text-top)
                text-width (Math/ceil (data/text-width gutter-bubble-glyph-metrics repeat-text))
                ^double text-height (data/line-height gutter-bubble-glyph-metrics)
                text-left (- text-right text-width)
                ^Rect bubble-rect (data/expand-rect (data/->Rect text-left text-top text-width text-height) 3.0 0.0)]
            (.setFill gc bubble-background-color)
            (.fillRoundRect gc (.x bubble-rect) (.y bubble-rect) (.w bubble-rect) (.h bubble-rect) 5.0 5.0)
            (.setFont gc gutter-bubble-font)
            (.setFill gc gutter-foreground-color)
            (.fillText gc repeat-text text-right text-bottom))

          :eval-expression
          (let [text-y (+ ascent line-y)]
            (.setFont gc font)
            (.setFill gc gutter-eval-expression-color)
            (.fillText gc ">" text-right text-y))

          :eval-result
          (let [text-y (+ ascent line-y)]
            (.setFont gc font)
            (.setFill gc gutter-eval-result-color)
            (.fillText gc "=" text-right text-y))

          :eval-error
          (let [text-y (+ ascent line-y)]
            (.setFont gc font)
            (.setFill gc gutter-eval-error-color)
            (.fillText gc "!" text-right text-y))

          :extension-out
          (let [text-y (+ ascent line-y)]
            (.setFont gc font)
            (.setFill gc gutter-eval-expression-color)
            (.fillText gc "⚙" text-right text-y))

          :extension-err
          (let [text-y (+ ascent line-y)]
            (.setFont gc font)
            (.setFill gc gutter-eval-error-color)
            (.fillText gc "⚙" text-right text-y))
          nil)))))

(deftype ConsoleGutterView []
  view/GutterView

  (gutter-metrics [_this _lines _regions _glyph-metrics]
    (gutter-metrics))

  (draw-gutter! [_this gc gutter-rect layout font color-scheme lines regions _visible-cursors]
    (draw-gutter! gc gutter-rect layout font color-scheme lines regions)))

(defn- setup-view! [console-node view-node]
  (g/transact
    (concat
      (g/connect console-node :_node-id view-node :resource-node)
      (g/connect console-node :indent-type view-node :indent-type)
      (g/connect console-node :cursor-ranges view-node :cursor-ranges)
      (g/connect console-node :invalidated-rows view-node :invalidated-rows)
      (g/connect console-node :lines view-node :lines)
      (g/connect console-node :regions view-node :regions)))
  view-node)

(def ^:const line-sub-regions-pattern #"(?<=^|\s|[<\"'`])(\/[^\s>\"'`:]+)(?::?)(\d+)?")
(def ^:private ^:const line-sub-regions-pattern-partial #"([^\s<>:]+):(\d+)")

(defn- make-resource-reference-region
  ([row start-col end-col resource-proj-path on-click!]
   (assert (string? (not-empty resource-proj-path)))
   (assert (ifn? on-click!))
   (assoc (data/->CursorRange (data/->Cursor row start-col)
                              (data/->Cursor row end-col))
     :type :resource-reference
     :proj-path resource-proj-path
     :on-click! on-click!))
  ([row start-col end-col resource-proj-path resource-row on-click!]
   (assert (integer? resource-row))
   (assert (not (neg? ^long resource-row)))
   (assoc (make-resource-reference-region row start-col end-col resource-proj-path on-click!)
     :row resource-row)))

(defn- find-project-resource-from-potential-match
  [resource-map partial-path]
  (if (contains? resource-map partial-path)
    partial-path ;; Already a valid path
    (let [partial-matches (filter #(.endsWith ^String % partial-path) (keys resource-map))]
      (when (= 1 (bounded-count 2 partial-matches))
        (first partial-matches)))))

(defn- make-line-sub-regions [resource-map on-region-click! row line]
  (into []
        (comp
          (mapcat #(re-match-result-seq % line))
          (keep (fn [^MatchResult result]
                  (when-let [resource-proj-path (find-project-resource-from-potential-match resource-map (.group result 1))]
                    (let [resource-row (some-> (.group result 2) Long/parseUnsignedLong)
                          start-col (.start result)
                          end-col (if (string/ends-with? (.group result) ":")
                                    (dec (.end result))
                                    (.end result))]
                      (if (nil? resource-row)
                        (make-resource-reference-region row start-col end-col resource-proj-path on-region-click!)
                        (make-resource-reference-region row start-col end-col resource-proj-path (dec (long resource-row)) on-region-click!))))))
          (distinct))
        [line-sub-regions-pattern line-sub-regions-pattern-partial]))

(defn- make-whole-line-region [type ^long row line]
  (assert (keyword? type))
  (assert (not (neg? row)))
  (assert (string? line))
  (assoc (data/->CursorRange (data/->Cursor row 0)
                             (data/->Cursor row (count line)))
    :type type))

(defn- make-line-regions [resource-map on-region-click! ^long row [type line]]
  (assert (keyword? type))
  (assert (string? line))
  (cons (make-whole-line-region type row line)
        (make-line-sub-regions resource-map on-region-click! row line)))

(defn- append-distinct-lines [{:keys [lines regions] :as props} entries resource-map on-region-click!]
  (merge props
         (data/append-distinct-lines lines regions
                                     (mapv second entries)
                                     (partial make-line-sub-regions resource-map on-region-click!))))

(defn- append-regioned-lines [{:keys [lines regions] :as props} entries resource-map on-region-click!]
  (assert (vector? lines))
  (assert (vector? regions))
  (let [clean-lines (if (= [""] lines) [] lines)
        lines' (into clean-lines (map second) entries)
        lines' (if (empty? lines') [""] lines')
        regions' (into regions
                       (mapcat (partial make-line-regions resource-map on-region-click!)
                               (iterate inc (count clean-lines))
                               entries))]
    (cond-> (assoc props
              :lines lines'
              :regions regions')

            (empty? clean-lines)
            (assoc :invalidated-row 0))))

(defn- append-entries [props entries resource-map on-region-click!]
  (assert (map? props))
  (assert (vector? (not-empty (:lines props))))
  (assert (vector? (:regions props)))
  (reduce (fn [props entries]
            (if (nil? (ffirst entries))
              (append-distinct-lines props entries resource-map on-region-click!)
              (append-regioned-lines props entries resource-map on-region-click!)))
          props
          (partition-by #(nil? (first %)) entries)))

(defn- repaint-console-view! [view-node workspace on-region-click! elapsed-time]
  (let [{:keys [clear? entries]} (dequeue-pending! 1000)]
    (when (or clear? (seq entries))
      (let [resource-map (g/node-value workspace :resource-map)
            ^LayoutInfo prev-layout (g/node-value view-node :layout)
            prev-lines (g/node-value view-node :lines)
            prev-regions (g/node-value view-node :regions)
            prev-document-width (if clear? 0.0 (.document-width prev-layout))
            appended-width (data/max-line-width (.glyph prev-layout) (.tab-stops prev-layout) (mapv second entries))
            document-width (max prev-document-width ^double appended-width)
            was-scrolled-to-bottom? (data/scrolled-to-bottom? prev-layout (count prev-lines))
            props (append-entries {:lines (if clear? [""] prev-lines)
                                   :regions (if clear? [] prev-regions)}
                                  entries resource-map on-region-click!)]
        (view/set-properties! view-node nil
                              (cond-> (assoc props :document-width document-width)
                                      was-scrolled-to-bottom? (assoc :scroll-y (data/scroll-to-bottom prev-layout (count (:lines props))))
                                      clear? (assoc :cursor-ranges [data/document-start-cursor-range])
                                      clear? (assoc :invalidated-row 0)
                                      clear? (data/frame-cursor prev-layout))))))
  (view/repaint-view! view-node elapsed-time {:cursor-visible? false}))

(def ^:private console-grammar
  {:name "Console"
   :scope-name "source.console"
   :patterns [{:match #"^INFO:RESOURCE: (.+?) was successfully reloaded\."
               :name "console.reload.successful"}
              {:match #"^ERROR:.+?:"
               :name "console.error"}
              {:match #"^WARNING:.+?:"
               :name "console.warning"}
              {:match #"^INFO:.+?:"
               :name "console.info"}
              {:match #"^DEBUG:.+?:"
               :name "console.debug"}]})

(def ^:private console-color-scheme
  (let [^Color background-color (Color/valueOf "#27292D")
        ^Color selection-background-color (Color/valueOf "#264A8B")]
    (view/make-color-scheme
      [["console.error" (Color/valueOf "#FF6161")]
       ["console.warning" (Color/valueOf "#FF9A34")]
       ["console.info" (Color/valueOf "#CCCFD3")]
       ["console.debug" (Color/valueOf "#3B8CF8")]
       ["console.reload.successful" (Color/valueOf "#33CC33")]
       ["editor.foreground" (Color/valueOf "#A2B0BE")]
       ["editor.background" background-color]
       ["editor.cursor" Color/TRANSPARENT]
       ["editor.gutter.eval.expression" (Color/valueOf "#DDDDDD")]
       ["editor.gutter.eval.error" (Color/valueOf "#FF6161")]
       ["editor.gutter.eval.result" (Color/valueOf "#52575C")]
       ["editor.selection.background" selection-background-color]
       ["editor.selection.background.inactive" (.interpolate selection-background-color background-color 0.25)]
       ["editor.selection.occurrence.outline" (Color/valueOf "#A2B0BE")]])))

(defn make-console! [graph workspace ^Tab console-tab ^GridPane console-grid-pane open-resource-fn]
  (let [^Pane canvas-pane (.lookup console-grid-pane "#console-canvas-pane")
        canvas (Canvas. (.getWidth canvas-pane) (.getHeight canvas-pane))
        view-node (setup-view! (g/make-node! graph ConsoleNode)
                               (g/make-node! graph view/CodeEditorView
                                             :canvas canvas
                                             :canvas-width (.getWidth canvas)
                                             :canvas-height (.getHeight canvas)
                                             :color-scheme console-color-scheme
                                             :grammar console-grammar
                                             :gutter-view (ConsoleGutterView.)
                                             :highlighted-find-term (.getValue find-term-property)
                                             :line-height-factor 1.2
                                             :resize-reference :bottom))
        tool-bar (setup-tool-bar! (.lookup console-grid-pane "#console-tool-bar") view-node)
        on-region-click! (fn on-region-click! [region]
                           (when (= :resource-reference (:type region))
                             (let [resource (workspace/find-resource workspace (:proj-path region))]
                               (when (and (resource/openable-resource? resource)
                                          (resource/exists? resource))
                                 (let [opts (when-some [row (:row region)]
                                              {:cursor-range (data/Cursor->CursorRange (data/->Cursor row 0))})]
                                   (open-resource-fn resource opts))))))
        repainter (ui/->timer "repaint-console-view" (fn [_ elapsed-time]
                                                       (when (and (.isSelected console-tab) (not (ui/ui-disabled?)))
                                                         (repaint-console-view! view-node workspace on-region-click! elapsed-time))))
        context-env {:clipboard (Clipboard/getSystemClipboard)
                     :term-field (.lookup tool-bar "#search-console")
                     :view-node view-node}]

    ;; Canvas stretches to fit view, and updates properties in view node.
    (ui/add-child! canvas-pane canvas)
    (.bind (.widthProperty canvas) (.widthProperty canvas-pane))
    (.bind (.heightProperty canvas) (.heightProperty canvas-pane))
    (ui/observe (.widthProperty canvas) (fn [_ _ width] (g/set-property! view-node :canvas-width width)))
    (ui/observe (.heightProperty canvas) (fn [_ _ height] (g/set-property! view-node :canvas-height height)))

    ;; Configure canvas.
    (doto canvas
      (.setFocusTraversable true)
      (.addEventFilter KeyEvent/KEY_PRESSED (ui/event-handler event (view/handle-key-pressed! view-node event)))
      (.addEventHandler MouseEvent/MOUSE_MOVED (ui/event-handler event (view/handle-mouse-moved! view-node event)))
      (.addEventHandler MouseEvent/MOUSE_PRESSED (ui/event-handler event (view/handle-mouse-pressed! view-node event)))
      (.addEventHandler MouseEvent/MOUSE_DRAGGED (ui/event-handler event (view/handle-mouse-moved! view-node event)))
      (.addEventHandler MouseEvent/MOUSE_RELEASED (ui/event-handler event (view/handle-mouse-released! view-node event)))
      (.addEventHandler MouseEvent/MOUSE_EXITED (ui/event-handler event (view/handle-mouse-exited! view-node event)))
      (.addEventHandler ScrollEvent/SCROLL (ui/event-handler event (view/handle-scroll! view-node event))))

    ;; Configure contexts.
    (ui/context! console-grid-pane :console-grid-pane context-env nil)
    (ui/context! canvas :console-view context-env nil)

    ;; Highlight occurrences of search term.
    (let [find-term-setter (view/make-property-change-setter view-node :highlighted-find-term)]
      (.addListener find-term-property find-term-setter)

      ;; Ensure the focus-state property reflects the current input focus state.
      (let [^Scene scene (.getScene console-grid-pane)
            focus-owner-property (.focusOwnerProperty scene)
            focus-change-listener (view/make-focus-change-listener view-node console-grid-pane canvas)]
        (.addListener focus-owner-property focus-change-listener)

        ;; Remove callbacks if the console tab is closed.
        (ui/on-closed! console-tab (fn [_]
                                     (ui/timer-stop! repainter)
                                     (dispose-tool-bar! tool-bar)
                                     (.removeListener find-term-property find-term-setter)
                                     (.removeListener focus-owner-property focus-change-listener)))))

    ;; Start repaint timer.
    (ui/timer-start! repainter)
    view-node))
