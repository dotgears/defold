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

(ns util.digest
  (:import [java.io InputStream OutputStream]
           [java.security DigestOutputStream MessageDigest]
           [org.apache.commons.codec.digest DigestUtils]
           [org.apache.commons.codec.binary Hex]))

(set! *warn-on-reflection* true)

(defn bytes->hex [^bytes data]
  (Hex/encodeHexString data))

(defn sha1 [^bytes data]
  (DigestUtils/sha1 data))

(defn sha1-hex [^bytes data]
  (DigestUtils/sha1Hex data))

(defn string->sha1 [^String s]
  (DigestUtils/sha1 s))

(defn string->sha1-hex [^String s]
  (DigestUtils/sha1Hex s))

(defn stream->sha1 [^InputStream stream]
  (DigestUtils/sha1 stream))

(defn stream->sha1-hex [^InputStream stream]
  (DigestUtils/sha1Hex stream))

(def ^:private ^OutputStream sink-output-stream
  (proxy [OutputStream] []
    (write
      ([byte-or-bytes])
      ([^bytes b, ^long off, ^long len]))))

(defn make-digest-output-stream
  ^DigestOutputStream [^String algorithm]
  (DigestOutputStream. sink-output-stream (MessageDigest/getInstance algorithm)))

(defn digest-output-stream->hex
  ^String [^DigestOutputStream digest-output-stream]
  (-> digest-output-stream
      .getMessageDigest
      .digest
      bytes->hex))
