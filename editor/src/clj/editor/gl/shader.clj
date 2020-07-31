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

(ns editor.gl.shader
"# Building Shaders

To construct a shader object from .vp and .fp files on disk, use `load-shaders`.

Example:

(load-shaders gl (make-project-path project \"/builtins/tools/atlas/pos_uv\"))

This will look for pos_uv.vp (a vertex shader) and pos_uv.fp (a fragment shader). It will
load both shaders and link them into a program.

To make a shader object from GLSL strings (either just as literal strings, or created
via defshader), use `make-shader`

Example
(defshader frag ,,,)
(defshader vert ,,,)
(make-shader [vert] [frag])

This will use all the strings in the first collection as sources for the vertex shader
and all the strings in the second collection as sources for the fragment shader. If you
only have one string, you can pass that instead of a collection.

# GLSL Translator

The GLSL translator is derived from Roger Allen's Shadertone project (https://github.com/overtone/shadertone).
See licenses/shadertone.txt

This is only a single-pass \"lisp\" to GLSL translator.  Very basic.
If this is useful, then we can work to improve it.

# Basic Forms

Here are the essential forms.
  * define functions
    (defn <return-type> <function-name> <function-args-vector> <body-stmt1> ... )
  * function calls
    (<name> <arg1> <arg2> ... )
  * return value
    (return <statement>)
  * variable creation/assignment
    (uniform <type> <name>)
    (setq <type> <name> <statement>)
    (setq <name> <statement>)
  * looping
    (forloop [ <init-stmt> <test-stmt> <step-stmt> ] <body-stmt1> ... )
    (while <test-stmt> <body-stmt1> ... )
    (break)
    (continue)
  * conditionals
    (if <test> <stmt>)
    (if <test> (do <body-stmt1> ...))
    (if <test> <stmt> <else-stmt>)
    (if <test> (do <body-stmt1> ...) (do <else-stmt1> ...))
  * switch
    (switch <test> <case-int-1> <case-stmt-1> ...)
    cases can only be integers or the keyword :default

# Types

Variable types are exactly the GLSL types.

# Examples

The very simplest case, a constant fragment color.

(defshader test-shader
   (defn void main []
      (setq gl_FragColor (vec4 1.0 0.5 0.5 1.0))))

Note that the \"defn\" inside of defshader resembles clojure.core/defn, but
here it specifically means to create a shader function. Note also the return
type in the declaration.

Here is an example that uses a uniform variable to be set by the application.

(defshader test-shader
  (uniform vec3 iResolution)
  (defn void main []
    (setq vec2 uv (/ gl_FragCoord.xy iResolution.xy))
    (setq gl_FragColor (vec4 uv.x uv.y 0.0 1.0))))

There are some examples in the testcases in dynamo.shader.translate-test."
(:require [clojure.string :as string]
          [clojure.walk :as walk]
          [editor.buffers :refer [bbuf->string]]
          [editor.geom :as geom]
          [editor.gl :as gl]
          [editor.gl.protocols :refer [GlBind]]
          [editor.types :as types]
          [editor.resource :as resource]
          [editor.scene-cache :as scene-cache])
(:import [java.nio IntBuffer ByteBuffer]
         [com.jogamp.opengl GL GL2]
         [javax.vecmath Matrix4d Vector4f Vector4d Point3d]))

(set! *warn-on-reflection* true)

;; ======================================================================
;; shader translation comes from https://github.com/overtone/shadertone.
;; See licenses/shadertone.txt

;; ======================================================================
;; translation functions for a dialect of clojure-like s-expressions
(declare shader-walk)

(defn- shader-typed-assign-str [z]
  (let [[type name value] z
        _ (assert (= 3 (count z)))
        asn-str (format "%s %s = %s;\n"
                          type name
                          (shader-walk (list value)))]
    asn-str))

(defn- shader-assign-str [z]
  (let [[name value] z
        _ (assert (= 2 (count z)))
        asn-str (format "%s = %s;\n"
                        name
                        (shader-walk (list value)))]
    asn-str))

(defn- shader-walk-assign [x]
  (case (count (rest x))
    2 (shader-assign-str (rest x))
    3 (shader-typed-assign-str (rest x))
    :else (assert false "incorrect number of args for setq statement")))

(defn- shader-walk-defn-args [x]
  (assert (vector? x))
  (if (empty? x)
    "void"
    (string/join \, (map #(apply (partial format "%s %s") %) (partition 2 x)))))

(defn- shader-walk-defn [x]
  (let [fn-str (format "%s %s(%s) {\n%s}\n"
                       (nth x 1)
                       (nth x 2)
                       (shader-walk-defn-args (nth x 3))
                       (string/join (shader-walk (drop 4 x))))]  ;; FIXME add indentation level?
   fn-str))

(defn- shader-walk-fn [x]
  (let [pre-fn (if (= (first (str (first x))) \.) "" (str (first x)))
        post-fn (if (= (first (str (first x))) \.) (str (first x)) "")
        fn-str (format "%s(%s)%s"
                       pre-fn
                       (string/join
                        \,
                        (map #(shader-walk (list %)) (rest x)))
                       post-fn)]
    fn-str))

(defn- shader-walk-infix [x]
  (let [fn-str (format "(%s)"
                       (string/join
                        (format " %s " (str (first x)))
                        (map #(shader-walk (list %)) (rest x))))]
    fn-str))

(defn- shader-stmt [x]
  (format "%s;\n" (string/join \space x)))

;; (forloop [ init-stmt test-stmt step-stmt ] body )
(defn- shader-walk-forloop [x]
  (let [[init-stmt test-stmt step-stmt] (nth x 1)
        fl-str (format "for( %s %s; %s ) {\n%s}\n"
                       (shader-walk (list init-stmt))
                       (shader-walk (list test-stmt))
                       (shader-walk (list step-stmt))
                       (string/join (shader-walk (drop 2 x))))]
    fl-str))

;; (whileloop test-stmt body )
(defn- shader-walk-while [x]
  (let [w-str (format "while%s {\n%s}\n"
                      (shader-walk (list (nth x 1)))
                      (string/join (shader-walk (drop 2 x))))]
    w-str))

(defn- shader-walk-do [x]
  (let [w-str (format "{\n%s}\n" (string/join (shader-walk (drop 1 x))))]
    w-str))

(defn- shader-walk-if [x]
  (case (count (rest x))
    2  (let [w-str (format "if%s\n%s" ;; if() {}
                           (shader-walk (list (nth x 1)))
                           (shader-walk (list (nth x 2))))]
         w-str)
    3  (let [w-str (format "if%s\n%selse\n%s" ;; if() {} else {}
                           (shader-walk (list (nth x 1)))
                           (shader-walk (list (nth x 2)))
                           (shader-walk (list (nth x 3))))]
         w-str)
    :else (assert false "incorrect number of args for if statement")))

(defn- shader-walk-case [x]
  (let [[v s] x
        _ (assert (= 2 (count x)))
        c-str (if (number? v)
                (format "case %d:" v)
                (if (= v :default)
                  "default:"
                  (assert false (format "expected integer or default:, got: %s" v))))
        w-str (format "%s\n%s"
                      c-str
                      (shader-walk (list s)))]
    w-str))

(defn- shader-walk-switch [x]
  (let [v     (nth x 1)
        v-str (if (list? v)
                (shader-walk (list v))
                (format "(%s)" (shader-walk (list v))))
        w-str (format "switch%s {\n%s}\n"
                      v-str
                      (string/join (map shader-walk-case (partition 2 (drop 2 x)))))]
    w-str))

(defn- shader-walk-return [x]
  (format "%s;\n" (shader-walk-fn x)))

(defn- shader-walk-index [x]
  (format "%s[%d]" (second x) (nth x 2)))

(defn- inner-walk
  [x]
  (cond
   (list? x)    (let [sfx (str (first x))]
                  (case sfx
                    "defn"        (shader-walk-defn x)
                    "setq"        (shader-walk-assign x)
                    "forloop"     (shader-walk-forloop x)
                    "while"       (shader-walk-while x)
                    "if"          (shader-walk-if x)
                    "do"          (shader-walk-do x)
                    "switch"      (shader-walk-switch x)
                    "break"       (shader-stmt x)
                    "continue"    (shader-stmt x)
                    "discard"     (shader-stmt x)
                    "uniform"     (shader-stmt x)
                    "varying"     (shader-stmt x)
                    "attribute"   (shader-stmt x)
                    "return"      (shader-walk-return x)
                    "nth"         (shader-walk-index x)
                    ("+" "-" "*" "/" "=" "<" ">" "<=" ">=" "==" "!=" ">>" "<<") (shader-walk-infix x)
                    (shader-walk-fn x)))
   (symbol? x)  (identity x)
   (float? x)   (identity x)
   (integer? x) (identity x)
   :else        (shader-walk x)))

(defn- outer-walk [x]
  (cond
   (list? x)     (string/join x)
   :else         (identity x)))

(defn- shader-walk [form]
  (walk/walk inner-walk outer-walk form))

;; ======================================================================
;; Public API
(defn create-shader
  "Returns a string in GLSL suitable for compilation. Takes a list of forms.
These forms should be quoted, as if they came from a macro."
  [params]
  (apply str "#version 120" \newline (shader-walk params)))

(defmacro defshader
  "Macro to define the fragment shader program. Defines a new var whose contents will
be the return value of `create-shader`.

This must be submitted to the driver for compilation before you can use it. See
`make-shader`"
  [name & body]
  `(def ~name ~(create-shader body)))

(defprotocol ShaderVariables
  (get-attrib-location [this gl name])
  (set-uniform [this gl name val]))

(defmulti set-uniform-at-index (fn [_ _ _ val] (class val)))

(defmethod set-uniform-at-index Matrix4d
  [^GL2 gl progn loc val]
  (.glUniformMatrix4fv gl loc 1 false (float-array (geom/as-array val)) 0))

(defmethod set-uniform-at-index Vector4f
  [^GL2 gl progn loc ^Vector4f val]
  (.glUniform4f gl loc (.x val) (.y val) (.z val) (.w val)))

(defmethod set-uniform-at-index Vector4d
  [^GL2 gl progn loc ^Vector4d val]
  (.glUniform4f gl loc (.x val) (.y val) (.z val) (.w val)))

(defmethod set-uniform-at-index Point3d
  [^GL2 gl progn loc ^Point3d val]
  (.glUniform3f gl loc (float (.x val)) (float (.y val)) (float (.z val))))

(defmethod set-uniform-at-index (class (float-array []))
  [^GL2 gl progn loc ^floats val]
  (case (count val)
    3 (.glUniform4f gl loc (aget val 0) (aget val 1) (aget val 2) 1)
    4 (.glUniform4f gl loc (aget val 0) (aget val 1) (aget val 2) (aget val 3))))

(defmethod set-uniform-at-index Integer
  [^GL2 gl progn loc val]
  (.glUniform1i gl loc val))

(defmethod set-uniform-at-index Long
  [^GL2 gl progn loc val]
  (.glUniform1i gl loc (int val)))

(defmethod set-uniform-at-index nil
  [^GL2 gl progn loc val]
  ;; No-Op. This is for sampler uniforms. They are just a name. Contains no
  ;; value to set.
  )

(defn program-link-errors
  [^GL2 gl progn]
  (let [msg-len (IntBuffer/allocate 1)]
    (.glGetProgramiv gl progn GL2/GL_INFO_LOG_LENGTH msg-len)
    (let [msg (ByteBuffer/allocate (.get msg-len 0))]
      (.glGetProgramInfoLog gl progn (.capacity msg) nil msg)
      (bbuf->string msg))))

(defn- delete-program [^GL2 gl program]
  (when-not (zero? program)
    (.glDeleteProgram gl program)))

(defn- attach-shaders [^GL2 gl program shaders]
  (doseq [shader shaders]
    (.glAttachShader gl program shader)))

(defn make-program
  [^GL2 gl & shaders]
  (let [program (.glCreateProgram gl)]
    (attach-shaders gl program shaders)
    (.glLinkProgram gl program)
    (let [status (IntBuffer/allocate 1)]
      (.glGetProgramiv gl program GL2/GL_LINK_STATUS status)
      (if (= GL/GL_TRUE (.get status 0))
        program
        (try
          (throw (Exception. (str "Program link failure.\n" (program-link-errors gl program))))
          (finally
            (delete-program gl program)))))))

(defn shader-compile-errors
  [^GL2 gl shader-name]
  (let [msg-len (IntBuffer/allocate 1)]
    (.glGetShaderiv gl shader-name GL2/GL_INFO_LOG_LENGTH msg-len)
    (let [msg (ByteBuffer/allocate (.get msg-len 0))]
      (.glGetShaderInfoLog gl shader-name (.capacity msg) nil msg)
      (bbuf->string msg))))

(defn delete-shader
  [^GL2 gl shader]
  (when-not (zero? shader)
    (.glDeleteShader gl shader)))

(defn make-shader*
  [type ^GL2 gl source]
  ;; Shader source can be either a string or a collection of strings.
  ;; However, it is not intended to be a collection of lines. The
  ;; shader compiler will simply read from each string in turn as if
  ;; they were concatenated. Thus, you need to have newline characters
  ;; at the end of each line.
  (assert (or (string? source) (coll? source)))
  (let [shader-name (.glCreateShader gl type)
        source-strings (into-array String
                                   (if (coll? source)
                                     source
                                     [source]))]
    (.glShaderSource gl shader-name (count source-strings) source-strings nil)
    (.glCompileShader gl shader-name)
    (let [status (IntBuffer/allocate 1)]
      (.glGetShaderiv gl shader-name GL2/GL_COMPILE_STATUS status)
      (if (= GL/GL_TRUE (.get status 0))
        shader-name
        (try
          (throw (Exception. (str "Shader compilation failure.\n" (shader-compile-errors gl shader-name))))
          (finally
            (delete-shader gl shader-name)))))))

(def make-fragment-shader (partial make-shader* GL2/GL_FRAGMENT_SHADER))
(def make-vertex-shader (partial make-shader* GL2/GL_VERTEX_SHADER))

(defrecord ShaderLifecycle [request-id verts frags uniforms]
  GlBind
  (bind [_this gl render-args]
    (let [[program uniform-locs] (scene-cache/request-object! ::shader request-id gl [verts frags (into #{} (map first) uniforms)])]
      (.glUseProgram ^GL2 gl program)
      (when-not (zero? program)
        (doseq [[name val] uniforms
                :let [val (if (keyword? val)
                            (get render-args val)
                            val)
                      loc (uniform-locs name (.glGetUniformLocation ^GL2 gl program name))]]
          (try
            (set-uniform-at-index gl program loc val)
            (catch IllegalArgumentException e
              (throw (IllegalArgumentException. (format "Failed setting uniform '%s'" name) e))))))))

  (unbind [_this gl render-args]
    (.glUseProgram ^GL2 gl 0))

  ShaderVariables
  (get-attrib-location [this gl name]
    (assert (string? (not-empty name)))
    (when-let [[program _] (scene-cache/request-object! ::shader request-id gl [verts frags (into #{} (map first) uniforms)])]
      (if (zero? program)
        -1
        (gl/gl-get-attrib-location ^GL2 gl program name))))

  (set-uniform [this gl name val]
    (assert (string? (not-empty name)))
    (when-let [[program uniform-locs] (scene-cache/request-object! ::shader request-id gl [verts frags (into #{} (map first) uniforms)])]
      (when (and (not (zero? program)) (= program (gl/gl-current-program gl)))
        (let [loc (uniform-locs name (.glGetUniformLocation ^GL2 gl program name))]
          (try
            (set-uniform-at-index gl program loc val)
            (catch IllegalArgumentException e
              (throw (IllegalArgumentException. (format "Failed setting uniform '%s'" name) e)))))))))

(defn make-shader
  "Ready a shader program for use by compiling and linking it. Takes a collection
of GLSL strings and returns an object that satisfies GlBind and GlEnable."
  ([request-id verts frags]
    (make-shader request-id verts frags {}))
  ([request-id verts frags uniforms]
    (->ShaderLifecycle request-id verts frags uniforms)))

(def compat-directives {"vp" [""
                              "#ifndef GL_ES"
                              "#define lowp"
                              "#define mediump"
                              "#define highp"
                              "#endif"
                              ""]
                        "fp" [""
                              "#ifdef GL_ES"
                              "precision mediump float;"
                              "#endif"
                              "#ifndef GL_ES"
                              "#define lowp"
                              "#define mediump"
                              "#define highp"
                              "#endif"
                              ""]})

(def ^:private directive-line-re #"^\s*(#|//).*")

(defn- directive-line? [line]
  (or (string/blank? line)
      (some? (re-matches directive-line-re line))))

(defn insert-directives
  [code-lines inserted-directive-lines]
  ;; Our directives should be inserted after any directives in the shader.
  ;; This makes it possible to use directives such as #extension in the shader.
  (let [[code-directive-lines code-non-directive-lines] (split-with directive-line? code-lines)]
    (into []
          (concat code-directive-lines
                  inserted-directive-lines
                  [(str "#line " (count code-directive-lines))]
                  code-non-directive-lines))))

(defn- make-shader-program [^GL2 gl [verts frags uniform-names]]
  (let [vs (make-vertex-shader gl verts)]
    (try
      (let [fs (make-fragment-shader gl frags)]
        (try
          (let [program (make-program gl vs fs)
                uniform-locs (into {}
                                   (map (fn [uniform-name] [uniform-name (.glGetUniformLocation ^GL2 gl program uniform-name)]))
                                   uniform-names)]
            [program uniform-locs])
          (finally
            (delete-shader gl fs)))) ; flag shaders for deletion: they will be deleted immediately, or when we delete the program to which they are attached
      (finally
        (delete-shader gl vs)))))

(defn- update-shader-program [^GL2 gl [program uniform-locs] data]
  (delete-program gl program)
  (try
    (make-shader-program gl data)
    (catch Exception _
      [0 {}])))

(defn- destroy-shader-programs [^GL2 gl programs _]
  (doseq [[program _] programs]
    (delete-program gl program)))

(scene-cache/register-object-cache! ::shader make-shader-program update-shader-program destroy-shader-programs)
