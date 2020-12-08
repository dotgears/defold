// Copyright 2020 The Defold Foundation
// Licensed under the Defold License version 1.0 (the "License"); you may not use
// this file except in compliance with the License.
//
// You may obtain a copy of the License, together with FAQs at
// https://www.defold.com/license
//
// Unless required by applicable law or agreed to in writing, software distributed
// under the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
// CONDITIONS OF ANY KIND, either express or implied. See the License for the
// specific language governing permissions and limitations under the License.

#ifndef DMSDK_VMATH_H
#define DMSDK_VMATH_H

#include <dmsdk/vectormath/cpp/vectormath_aos.h>

/*# SDK Vector Math API documentation
 * Vector Math functions.
 *
 * @document
 * @name Vector Math
 * @namespace dmVMath
 * @path engine/dlib/src/dmsdk/dlib/vmath.h
 */

namespace dmVMath
{
    typedef Vectormath::Aos::Point3     Point3;
    typedef Vectormath::Aos::Vector3    Vector3;
    typedef Vectormath::Aos::Vector4    Vector4;
    typedef Vectormath::Aos::Quat       Quat;
    typedef Vectormath::Aos::Matrix4    Matrix4;

} // dmVMath

#endif // DMSDK_VMATH_H
