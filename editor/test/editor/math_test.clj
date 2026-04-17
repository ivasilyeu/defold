;; Copyright 2020-2026 The Defold Foundation
;; Copyright 2014-2020 King
;; Copyright 2009-2014 Ragnar Svensson, Christian Murray
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

(ns editor.math-test
 (:require [clojure.test :refer :all]
           [editor.math :as math])
 (:import [java.lang Math]
          [javax.vecmath Matrix3d Matrix4d Point3d Tuple4d Vector3d Quat4d]))

(set! *warn-on-reflection* true)
(set! *unchecked-math* :warn-on-boxed)

(defn- near? [^double v1 ^double v2]
  (<= (Math/abs (- v1 v2)) math/epsilon))

(defn- eq? [^Tuple4d q1 ^Tuple4d q2]
  (boolean (and (near? (.getX q1) (.getX q2))
                (near? (.getY q1) (.getY q2))
                (near? (.getZ q1) (.getZ q2))
                (near? (.getW q1) (.getW q2)))))

(defn- rot-eq? [^Quat4d q1 ^Quat4d q2]
  ;; Rotation applied by a negated quaternion is equivalent.
  (or (eq? q1 q2)
      (eq? q1 (doto (Quat4d. q2) (.negate)))))

(defn- mat-eq? [^Matrix4d m1 ^Matrix4d m2]
  (boolean
    (every?
      true?
      (for [row (range 4)
            col (range 4)]
        (near? (.getElement m1 row col)
               (.getElement m2 row col))))))

(defn- ->vec3
  ^Vector3d [v]
  (Vector3d. (double-array v)))

(defn- ->point3
  ^Point3d [v]
  (Point3d. (double-array v)))

(defn- ->quat
  ^Quat4d [v]
  (Quat4d. (double-array v)))

(defn- axis-angle->quat
  ^Quat4d [axis angle-deg]
  (let [half-angle (* 0.5 (math/deg->rad angle-deg))
        sin (Math/sin half-angle)
        cos (Math/cos half-angle)]
    (case axis
      :x (Quat4d. sin 0.0 0.0 cos)
      :y (Quat4d. 0.0 sin 0.0 cos)
      :z (Quat4d. 0.0 0.0 sin cos))))

(defn- mul-quats [^Quat4d q1 ^Quat4d q2]
  (doto (Quat4d. q1)
    (.mul q2)))

(deftest quat->euler->quat []
  (let [test-vals [[1 0 0 0]
                   [0 1 0 0]
                   [0 0 -1 0]
                   [0 0 0 1]
                   [1 1 1 1]]
        quats (map ->quat test-vals)]
    (doseq [result (map rot-eq? quats (map (comp math/euler->quat math/quat->euler) quats))]
      (is result))))

(def ^:private checked-xyz-euler-angles
  [[-35.0 120.0 -15.0]
   [-90.0 180.0 0.0]
   [0.0 0.0 90.0]
   [10.0 20.0 30.0]
   [30.0 0.0 20.0]
   [44.0 15.0 89.0]
   [45.0 15.0 -90.0]
   [45.0 15.0 89.0]
   [45.0 15.0 90.0]
   [90.0 45.0 0.0]])


(comment
  (mapv (fn [^Quat4d quat]
          (math/->mat4-non-uniform (Vector3d.) quat (Vector3d. 1.0 1.0 1.0))
          #_
          (doto (javax.vecmath.Matrix3d.)
            (.set quat)))
        [(math/euler->quat [179.0 0.01 0.0])
         (math/euler->quat [180.0 0.01 0.0])]))

(deftest euler->quat-matches-yzx-composition
  (doseq [[x y z :as euler] checked-xyz-euler-angles]
    (let [expected (-> (axis-angle->quat :y y)
                       (mul-quats (axis-angle->quat :z z))
                       (mul-quats (axis-angle->quat :x x)))
          actual (math/euler->quat euler)]
      (is (rot-eq? expected actual) euler))))

(deftest quat->euler-yzx-roundtrip
  (doseq [euler checked-xyz-euler-angles]
    (let [quat (math/euler->quat euler)
            roundtrip (math/euler->quat (math/quat->euler quat))]
      (is (rot-eq? quat roundtrip) euler))))

(deftest split-mat4-roundtrip
  (doseq [[position euler scale]
          [[[1.0 2.0 3.0] [10.0 20.0 30.0] [2.0 3.0 4.0]]
           [[1.0 2.0 3.0] [10.0 20.0 30.0] [-2.0 3.0 4.0]]
           [[1.0 2.0 3.0] [10.0 20.0 30.0] [2.0 -3.0 4.0]]
           [[1.0 2.0 3.0] [10.0 20.0 30.0] [2.0 3.0 -4.0]]
           [[1.0 2.0 3.0] [0.0 90.0 0.0] [-2.0 3.0 4.0]]]]
    (let [original (math/clj->mat4 position
                                   (math/vecmath->clj (math/euler->quat euler))
                                   scale)
          split-position (Vector3d.)
          split-rotation (Quat4d.)
          split-scale (Vector3d.)
          _ (math/split-mat4 original split-position split-rotation split-scale)
          roundtrip (math/->mat4-non-uniform split-position split-rotation split-scale)]
      (is (mat-eq? original roundtrip)
          {:position position
           :euler euler
           :scale scale
           :split-scale [(.x split-scale) (.y split-scale) (.z split-scale)]}))))

(defn- vec-eq? [^Vector3d lhs ^Vector3d rhs]
  (let [diff (doto (Vector3d. rhs) (.sub lhs))
        sq (.dot diff diff)
        ep-sq (* math/epsilon math/epsilon)]
    (< sq ep-sq)))

(deftest from-to->quat->vec
  (let [test-vals [[1 0 0]
                   [0 1 0]
                   [0 0 1]
                   [-1 0 0]
                   [0 -1 0]
                   [0 0 -1]]
        test-vecs (map ->vec3 test-vals)]
    (doall
      (for [^Vector3d from test-vecs
            ^Vector3d to test-vecs
            :let [q (math/from-to->quat from to)]]
        (or
          (vec-eq? from (doto (Vector3d. to) (.negate)))
          (is (vec-eq? to (math/rotate q from))))))))

(deftest line-circle []
  (testing "Line circle projections"
           ; Line orthogonal to circle axis
           (let [x-vals [-1.5 -1 0 1 1.5]
                 line-positions (map #(do [% 1 2]) x-vals)
                 line-dir [0 0 -1]
                 circle-pos [0 0 0]
                 circle-axis [0 1 0]
                 circle-radius 1
                 expected-vals [[-1 0 0]
                                [-1 0 0]
                                [0 0 1]
                                [1 0 0]
                                [1 0 0]]]
             (doseq [[^Point3d line-pos ^Vector3d expected] (partition 2 (interleave (map ->point3 line-positions) (map ->vec3 expected-vals)))
                     :let [line-dir (->vec3 line-dir)
                           circle-pos (->point3 circle-pos)
                           circle-axis (->vec3 circle-axis)
                           result (math/project-line-circle line-pos line-dir circle-pos circle-axis circle-radius)]]
               (is (.epsilonEquals expected result math/epsilon))))))
