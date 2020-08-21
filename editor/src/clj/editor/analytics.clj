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

(ns editor.analytics
  (:require [clojure.data.json :as json]
            [clojure.java.io :as io]
            [clojure.string :as string]
            [editor.connection-properties :refer [connection-properties]]
            [editor.system :as sys]
            [editor.url :as url]
            [service.log :as log])
  (:import (clojure.lang PersistentQueue)
           (com.defold.editor Editor)
           (java.io File)
           (java.net HttpURLConnection MalformedURLException URL)
           (java.nio.charset StandardCharsets)
           (java.util UUID)
           (java.util.concurrent CancellationException)))

(set! *warn-on-reflection* true)

;; Set this to true to see the events that get sent when testing.
(defonce ^:private log-events? false)

(defonce ^:private batch-size 16)
(defonce ^:private cid-regex #"[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}")
(defonce ^:private config-atom (atom nil))
(defonce ^:private event-queue-atom (atom PersistentQueue/EMPTY))
(defonce ^:private max-failed-send-attempts 5)
(defonce ^:private payload-content-type "application/x-www-form-urlencoded; charset=UTF-8")
(defonce ^:private shutdown-timeout 1000)
(defonce ^:private uid-regex #"[0-9A-F]{16}")
(defonce ^:private worker-atom (atom nil))

;; -----------------------------------------------------------------------------
;; Validation
;; -----------------------------------------------------------------------------

(defn- valid-analytics-url? [value]
  (and (string? value)
       (try
         (let [url (URL. value)]
           (and (nil? (.getQuery url))
                (nil? (.getRef url))
                (let [protocol (.getProtocol url)]
                  (or (= "http" protocol)
                      (= "https" protocol)))))
         (catch MalformedURLException _
           false))))

(defn- valid-cid? [value]
  (and (string? value)
       (.matches (re-matcher cid-regex value))))

(defn- valid-uid? [value]
  (and (string? value)
       (.matches (re-matcher uid-regex value))))

(defn- valid-config? [config]
  (and (map? config)
       (= #{:cid :uid} (set (keys config)))
       (let [{:keys [cid uid]} config]
         (and (valid-cid? cid)
              (or (nil? uid)
                  (valid-uid? uid))))))

(def ^:private valid-event?
  (every-pred map?
              (comp nil? :cd1) ; Custom Dimension 1 is reserved for the uid.
              (comp (every-pred string? not-empty) :t)
              (partial every?
                       (every-pred (comp keyword? key)
                                   (comp string? val)
                                   (comp not-empty val)))))

;; -----------------------------------------------------------------------------
;; Configuration
;; -----------------------------------------------------------------------------

(defn- make-config []
  {:post [(valid-config? %)]}
  {:cid (str (UUID/randomUUID))
   :uid nil})

(defn- config-file
  ^File []
  (.toFile (.resolve (Editor/getSupportPath) ".defold-analytics")))

(defn- write-config-to-file! [{:keys [cid uid]} ^File config-file]
  {:pre [(valid-cid? cid)
         (or (nil? uid) (valid-uid? uid))]}
  (let [json (if (some? uid)
               {"cid" cid "uid" uid}
               {"cid" cid})]
    (with-open [writer (io/writer config-file)]
      (json/write json writer))))

(defn- write-config! [config]
  (let [config-file (config-file)]
    (try
      (write-config-to-file! config config-file)
      (catch Throwable error
        (log/warn :msg (str "Failed to write analytics config: " config-file) :exception error)))))

(defn- read-config-from-file! [^File config-file]
  (with-open [reader (io/reader config-file)]
    (let [{cid "cid" uid "uid"} (json/read reader)]
      {:cid cid :uid uid})))

(defn- read-config! [invalidate-uid?]
  {:post [(valid-config? %)]}
  (let [config-file (config-file)
        config-from-file (try
                           (when (.exists config-file)
                             (read-config-from-file! config-file))
                           (catch Throwable error
                             (log/warn :msg (str "Failed to read analytics config: " config-file) :exception error)
                             nil))
        config (if-not (valid-config? config-from-file)
                 (make-config)
                 (cond-> config-from-file
                         (and invalidate-uid?
                              (contains? config-from-file :uid))
                         (assoc :uid nil)))]
    (when (not= config-from-file config)
      (write-config! config))
    config))

;; -----------------------------------------------------------------------------
;; Internals
;; -----------------------------------------------------------------------------

;; Entries sent to Google Analytics must be both UTF-8 and URL Encoded.
;; See the "URL Encoding Values" section here for details:
;; https://developers.google.com/analytics/devguides/collection/protocol/v1/reference
(def encode-string (partial url/encode url/x-www-form-urlencoded-safe-character? false StandardCharsets/UTF_8))

(defn- encode-key-value-pair
  ^String [[k v]]
  (str (encode-string (name k)) "=" (encode-string v)))

(defn- event->line
  ^String [event {:keys [cid uid] :as _config}]
  {:pre [(valid-event? event)
         (valid-cid? cid)
         (or (nil? uid) (valid-uid? uid))]}
  (let [tid (str "tid=" (get-in connection-properties [:google-analytics :tid]))
        common-pairs (if (some? uid) ; NOTE: The uid is also supplied as Custom Dimension 1.
                       ["v=1" tid (str "cid=" cid) (str "uid=" uid) (str "cd1=" uid)]
                       ["v=1" tid (str "cid=" cid)])
        pairs (into common-pairs
                    (map encode-key-value-pair)
                    event)]
    (string/join "&" pairs)))

(defn- batch->payload
  ^bytes [batch]
  {:pre [(vector? batch)
         (not-empty batch)]}
  (let [text (string/join "\n" batch)]
    (.getBytes text StandardCharsets/UTF_8)))

(defn- get-response-code!
  "Wrapper rebound to verify response from dev server in tests."
  [^HttpURLConnection connection]
  (.getResponseCode connection))

(defn- post!
  ^HttpURLConnection [^String url-string ^String content-type ^bytes payload]
  {:pre [(not-empty payload)]}
  (let [^HttpURLConnection connection (.openConnection (URL. url-string))]
    (doto connection
      (.setRequestMethod "POST")
      (.setDoOutput true)
      (.setFixedLengthStreamingMode (count payload))
      (.setRequestProperty "Content-Type" content-type)
      (.connect))
    (with-open [output-stream (.getOutputStream connection)]
      (.write output-stream payload))
    connection))

(defn- ok-response-code? [^long response-code]
  (<= 200 response-code 299))

(defn- send-payload! [analytics-url ^bytes payload]
  (try
    (let [response-code (get-response-code! (post! analytics-url payload-content-type payload))]
      (if (ok-response-code? response-code)
        true
        (do
          (log/warn :msg (str "Analytics server sent non-OK response code " response-code))
          false)))
    (catch Exception error
      (log/warn :msg "An exception was thrown when sending analytics data" :exception error)
      false)))

(defn- pop-count [queue ^long count]
  (nth (iterate pop queue) count))

(defn- send-one-batch!
  "Sends one batch of events from the queue. Returns false if there were events
  on the queue that could not be sent. Otherwise removes the successfully sent
  events from the queue and returns true."
  [analytics-url]
  (let [event-queue @event-queue-atom
        batch (into [] (take batch-size) event-queue)]
    (if (empty? batch)
      true
      (if-not (send-payload! analytics-url (batch->payload batch))
        false
        (do
          (swap! event-queue-atom pop-count (count batch))
          true)))))

(defn- send-remaining-batches! [analytics-url]
  (let [event-queue @event-queue-atom]
    (loop [event-queue event-queue]
      (when-some [batch (not-empty (into [] (take batch-size) event-queue))]
        (send-payload! analytics-url (batch->payload batch))
        (recur (pop-count event-queue (count batch)))))
    (swap! event-queue-atom pop-count (count event-queue))
    nil))

(declare shutdown!)

(defn- start-worker! [analytics-url ^long send-interval]
  (let [stopped-atom (atom false)
        thread (future
                 (try
                   (loop [failed-send-attempts 0]
                     (cond
                       (>= failed-send-attempts max-failed-send-attempts)
                       (do
                         (log/warn :msg (str "Analytics shut down after " max-failed-send-attempts " failed send attempts"))
                         (shutdown! 0)
                         (reset! event-queue-atom PersistentQueue/EMPTY)
                         nil)

                       @stopped-atom
                       (send-remaining-batches! analytics-url)

                       :else
                       (do
                         (Thread/sleep send-interval)
                         (if (send-one-batch! analytics-url)
                           (recur 0)
                           (recur (inc failed-send-attempts))))))
                   (catch CancellationException _
                     nil)
                   (catch InterruptedException _
                     nil)
                   (catch Throwable error
                     (log/warn :msg "Abnormal worker thread termination" :exception error))))]
    {:stopped-atom stopped-atom
     :thread thread}))

(defn- shutdown-worker! [{:keys [stopped-atom thread]} timeout-ms]
  (if-not (pos? timeout-ms)
    (future-cancel thread)
    (do
      ;; Try to shut down the worker thread gracefully, otherwise cancel the thread.
      (reset! stopped-atom true)
      (when (= ::timeout (deref thread timeout-ms ::timeout))
        (future-cancel thread))))
  nil)

(declare enabled?)

(defn- append-event! [event]
  (when-some [config @config-atom]
    (let [line (event->line event config)]
      (when (enabled?)
        (swap! event-queue-atom conj line))
      (when log-events?
        (log/info :msg line)))))

;; -----------------------------------------------------------------------------
;; Public interface
;; -----------------------------------------------------------------------------

(defn start! [^String analytics-url send-interval invalidate-uid?]
  {:pre [(valid-analytics-url? analytics-url)]}
  (reset! config-atom (read-config! invalidate-uid?))
  (when (some? (sys/defold-version))
    (swap! worker-atom
           (fn [started-worker]
             (when (some? started-worker)
               (shutdown-worker! started-worker 0))
             (start-worker! analytics-url send-interval)))))

(defn shutdown!
  ([]
   (shutdown! shutdown-timeout))
  ([timeout-ms]
   (swap! worker-atom
          (fn [started-worker]
            (when (some? started-worker)
              (shutdown-worker! started-worker timeout-ms))))))

(defn enabled? []
  (some? @worker-atom))

(defn set-uid! [^String uid]
  {:pre [(or (nil? uid) (valid-uid? uid))]}
  (swap! config-atom
         (fn [config]
           (let [config (or config (read-config! false))
                 updated-config (assoc config :uid uid)]
             (when (not= config updated-config)
               (write-config! updated-config))
             updated-config))))

(defn track-event!
  ([^String category ^String action]
   (append-event! {:t "event"
                   :ec category
                   :ea action}))
  ([^String category ^String action ^String label]
   (append-event! {:t "event"
                   :ec category
                   :ea action
                   :el label})))

(defn track-exception! [^Throwable exception]
  (append-event! {:t "exception"
                  :exd (.getSimpleName (class exception))}))

(defn track-screen! [^String screen-name]
  (if-some [version (sys/defold-version)]
    (append-event! {:t "screenview"
                    :an "defold"
                    :av version
                    :cd screen-name})
    (append-event! {:t "screenview"
                    :an "defold"
                    :cd screen-name})))
