(ns internal.ui.property
  (:require [clojure.string :as str]
            [schema.core :as s]
            [plumbing.core :refer [defnk]]
            [dynamo.node :as n]
            [dynamo.system :as ds]
            [dynamo.types :as t]
            [dynamo.ui :as ui]
            [dynamo.util :refer :all]
            [internal.node :as in]
            [internal.query :as iq]
            [camel-snake-kebab :refer :all])
  (:import [org.eclipse.core.runtime IStatus Status]
           [org.eclipse.swt SWT]
           [org.eclipse.swt.custom StackLayout]
           [org.eclipse.swt.layout FillLayout GridData GridLayout]
           [org.eclipse.swt.widgets Composite]
           [org.eclipse.ui ISelectionListener]
           [org.eclipse.ui.forms.widgets FormToolkit]
           [com.dynamo.cr.properties Messages]))

(set! *warn-on-reflection* true)

(defnk aggregate-properties
  [properties]
  (apply merge {} properties))

(defn- niceify-label
  [k]
  (-> k
    name
    ->Camel_Snake_Case_String
    (str/replace "_" " ")))

(defprotocol Presenter
  (control-for-property [this])
  (settings-for-control [this value])
  (on-event [this path event value]))

(defn no-change [] nil)
(defn intermediate-value [v] {:update-type :intermediate :value v})
(defn final-value [v]        {:update-type :final :value v})
(defn reset-default []       {:update-type :reset})

(defn- is-enter-key? [event]
  (#{\return \newline} (:character event)))

(defrecord Vec3Presenter []
  Presenter
  (control-for-property [_]
    {:type :composite
     :layout {:type :grid :num-columns 3 :margin-width 0}
     :children [[:x {:type :text :listen #{:key-down :focus-out}}]
                [:y {:type :text :listen #{:key-down :focus-out}}]
                [:z {:type :text :listen #{:key-down :focus-out}}]]})
  (settings-for-control [_ value]
    {:children [[:x {:text (str (nth value 0))}]
                [:y {:text (str (nth value 1))}]
                [:z {:text (str (nth value 2))}]]})
  (on-event [_ path event value]
    (when-let [index (get {:x 0 :y 1 :z 2} (first path))]
      (let [current-value (assoc value index (parse-number (ui/get-text (:widget event))))]
        (case (:type event)
          :key-down (if (is-enter-key? event)
                      (final-value current-value)
                      (no-change))
          :focus-out (final-value current-value)
          (no-change))))))

(defrecord StringPresenter []
  Presenter
  (control-for-property [this]
    {:type :text :listen #{:key-down :focus-out}})
  (settings-for-control [_ value]
    {:text (str value)})

  (on-event [_ _ event value]
    (let [current-value (ui/get-text (:widget event))]
      (case (:type event)
        :key-down (if (is-enter-key? event)
                    (final-value current-value)
                    (no-change))
        :focus-out (final-value current-value)
        (no-change)))))

(defrecord DefaultPresenter []
  Presenter
  (control-for-property [_]
    {:type :label})
  (settings-for-control [_ value]
    {:text (str value)}))

(defmulti presenter-for-property (comp :value-type :type))
(defmethod presenter-for-property s/Str [_] (->StringPresenter))
(defmethod presenter-for-property t/Vec3 [_] (->Vec3Presenter))
(defmethod presenter-for-property :default [_] (->DefaultPresenter))

(defn- attach-user-data
  [spec prop-name presenter path]
  (assoc spec
    :user-data {:presenter presenter :prop-name prop-name :path path}
    :children  (mapv (fn [[child-name child-spec]] [child-name (attach-user-data child-spec prop-name presenter (conj path child-name))]) (:children spec))))

(defn- attach-listeners
  [spec ui-event-listener]
  (assoc spec
    :listen   (zipmap (:listen spec) (repeat ui-event-listener))
    :children (mapv (fn [[child-name child-spec]] [child-name (attach-listeners child-spec ui-event-listener)]) (:children spec))))

(def ^:private right-column-layout {:layout-data {:type :grid :grab-excess-horizontal-space true :horizontal-alignment SWT/FILL :width-hint 50}})

(defn- control-spec
  [ui-event-listener prop-name presenter]
  (-> right-column-layout
    (merge (control-for-property presenter))
    (attach-user-data prop-name presenter [])
    (attach-listeners ui-event-listener)))

(defn- property-control-strip
  [ui-event-listener [prop-name {:keys [presenter]}]]
  (let [label-text (niceify-label prop-name)]
    [[:label-composite {:type :composite
                        :layout {:type :stack}
                        :children [[:label      {:type :label :text label-text}]
                                   [:label-link {:type :hyperlink :text label-text :underlined true :on-click (fn [_] (prn "RESET " prop-name)) :foreground [0 0 255] :tooltip-text Messages/FormPropertySheetViewer_RESET_VALUE}]]}]
     [prop-name (control-spec ui-event-listener prop-name presenter)]
     [:dummy           {:type :label :layout-data {:type :grid :exclude true}}]
     [:status-label    {:type :status-label :style :border :status Status/OK_STATUS :layout-data {:type :grid :grab-excess-horizontal-space true :horizontal-fill SWT/HORIZONTAL :width-hint 50 :exclude true}}]]))

(defn- property-page
  [control-strips]
  [:page-content
   {:type :composite
    :layout {:type :grid :num-columns 2 :margin-width 0}
    :children control-strips}])

(defn- make-property-page
  [toolkit ui-event-listener properties-form properties]
  (ui/make-control toolkit (ui/widget properties-form [:form :composite])
    (property-page (mapcat #(property-control-strip ui-event-listener %) properties))))

(def empty-property-page
  [:page-content
   {:type :composite
    :layout {:type :grid}
    :children [[:no-selection-label {:type :label :text Messages/FormPropertySheetViewer_NO_PROPERTIES}]]}])

(defn- make-empty-property-page
  [toolkit properties-form]
  (ui/make-control toolkit (ui/widget properties-form [:form :composite]) empty-property-page))

(defn- settings-for-page
  [properties]
  {:children
   (for [[prop-name {:keys [presenter value]}] properties]
     [prop-name (settings-for-control presenter value)])})

(defn- cache-key
  [properties]
  (map-vals :type properties))

(defn- lookup-or-create [cache key f & args]
  (-> cache
      (swap! (fn [cache]
                (if (contains? cache key)
                  cache
                  (assoc cache key (apply f args)))))
      (get key)))

(defn- attach-presenters
  [node content]
  (map-vals #(assoc % :presenter (presenter-for-property %)) content))

(defn- refresh-property-page
  [{:keys [sheet-cache toolkit properties-form ui-event-listener] :as node}]
  (let [content (attach-presenters node (in/get-node-value node :content))
        key     (cache-key content)
        page    (lookup-or-create sheet-cache key make-property-page toolkit ui-event-listener properties-form content)]
    (ui/update-ui!      (get-in page [:page-content]) (settings-for-page content))
    (ui/bring-to-front! (get-in page [:page-content]))
    (ui/scroll-to-top!  (get-in properties-form [:form]))))

(defn- refresh-after-a-while
  [graph this transaction]
  (when (and (ds/is-modified? transaction this :content) (:debouncer this))
    (t/signal (:debouncer this))))

(def gui
  [:form {:type   :form
          :text   "Properties"
          :layout {:type :grid}
          :children [[:composite
                      {:type :composite
                       :layout-data {:type :grid
                                     :horizontal-alignment SWT/FILL
                                     :vertical-alignment SWT/FILL
                                     :grab-excess-vertical-space true
                                     :grab-excess-horizontal-space true}
                       :layout {:type :stack}}]]}])

(n/defnode PropertyView
  (input properties [t/Properties])

  (property triggers t/Triggers (default [#'refresh-after-a-while]))

  (output content s/Any aggregate-properties)

  (on :create
    (let [toolkit           (FormToolkit. (.getDisplay ^Composite (:parent event)))
          properties-form   (ui/make-control toolkit (:parent event) gui)
          sheet-cache       (atom {})
          ui-event-listener (ui/make-listener #(n/dispatch-message self :ui-event :ui-event %) [])]
      (lookup-or-create sheet-cache (cache-key {}) make-empty-property-page toolkit properties-form)
      (ds/set-property self
        :sheet-cache       sheet-cache
        :toolkit           toolkit
        :properties-form   properties-form
        :ui-event-listener ui-event-listener
        :debouncer         (ui/display-debouncer 100 #(refresh-property-page (ds/refresh self))))))

  (on :ui-event
    (let [ui-event (:ui-event event)
          {:keys [presenter prop-name path]} (ui/get-user-data (:widget ui-event))
          content (in/get-node-value self :content)
          prop (get content prop-name)
          result (on-event presenter path ui-event (:value prop))]
      (prn :ui-event (:type ui-event) :prop-name prop-name :path path :result result)
      (when-let [new-value (:value result)]
        (ds/set-property {:_id (:node-id prop)} prop-name new-value))))

  ISelectionListener
  (selectionChanged [this part selection]
    (ds/transactional
      (doseq [[source-node source-label] (iq/sources-of this :properties)]
        (ds/disconnect source-node source-label this :properties))
      (doseq [n @selection]
        (ds/connect {:_id n} :properties this :properties)))))

(defn implementation-for
  [scope]
  (ds/transactional
    (ds/in scope
      (ds/add (make-property-view)))))

(defn get-control
  "This is called by the Java shim GenericPropertySheetPage. Not for other use."
  [property-view-node]
  (-> property-view-node
      ds/refresh
      :properties-form
      (ui/widget [:form])))
