/*
 * Copyright (C) 2025 Alberto Irurueta Carro (alberto@irurueta.com)
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *         http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

package com.irurueta.android.navigation.inertial.collectors.converters

import com.irurueta.algebra.Matrix
import com.irurueta.geometry.Quaternion
import com.irurueta.geometry.Rotation3D
import kotlin.math.sqrt

/**
 * Converts quaternions between different coordinate systems, specifically between
 * ENU (East-North-Up) and NED (North-East-Down).
 *
 * The conversion is based on the fact that a rotation in one coordinate system can be
 * transformed to the other by applying a specific conversion rotation before and after
 * the original rotation.
 *
 * The conversion rotation used is:
 * C = 	[0  1   0 ]
 *      [1  0   0 ]
 *      [0  0   -1]
 *
 * This means that to convert a quaternion representing a rotation from ENU to NED or vice versa,
 * we use the formula:
 * R_enu = C * R_ned * C
 * R_ned = C * R_enu * C
 *
 * Where R_enu and R_ned are the quaternions in ENU and NED coordinate systems respectively.
 */
object QuaternionCoordinateSystemConverter {
    /**
     * Minus half of square root of 2.0.
     */
    private val VALUE = -1.0 / sqrt(2.0)

    /**
     * Rotation representing the conversion between ENU to NED and NED to ENU.
     * This rotation is equivalent to rotation matrix below:
     * [0   1   0]
     * [1   0   0]
     * [0   0  -1]
     * Which is equivalent to roll = 0 rad, pitch = -[Math.PI] rad, yaw = -[Math.PI] / 2.0 rad,
     * which is also equivalent to roll = -[Math.PI] rad, pitch = 0 rad, yaw = [Math.PI] / 2.0 rad.
     */
    private val CONVERSION_ROTATION = Quaternion(0.0, VALUE, VALUE, 0.0)

    /**
     * Converts a rotation from ENU to NED or from NED to ENU, taking into account that:
     * Renu = CONVERSION_ROTATION * Rned * CONVERSION_ROTATION
     * And also:
     * Rned = CONVERSION_ROTATION * Renu * CONVERSION_ROTATION
     *
     * @param input Input quaternion to be converted.
     * @param output Instance where result will be stored.
     */
    fun convert(input: Quaternion, output: Quaternion) {
        // Xenu =  [0   1   0 ] * Xned
        //         [1   0   0 ]
        //	       [0   0   -1]

        // Xned =  [0   1   0 ] * Xenu
        //         [1   0   0 ]
        //         [0   0   -1]

        // C = 	[0  1   0 ]
        //      [1  0   0 ]
        //      [0  0   -1]

        // Notice that	conversion = C, and C * C = I -> C = C^-1

        // Xrotated_ned = Rned * Xned = Rned * C * Xenu
        // Xrotated_enu = Renu * Xenu = Renu * C * Xned

        // Xrotated_enu = C * Xrotated_ned = C * Rned * C * Xenu = Renu * Xenu
        // Xrotated_ned = C * Xrotated_enu = C * Renu * C * Xned = Rned * Xned

        // Hence:
        // Renu = C * Rned * C
        // Rned = C * Renu * C

        // Because CONVERSION_ROTATION = Cq quaternion is [0, -1/sqrt(2), -1/sqrt(2), 0]
        // And quaternion q is [a b c d]
        // Then the product: q' = Cq * q * Cq is:
        // q'' = q * Cq =   [a*0 + b/sqrt(2) + c/sqrt(2) - d*0 ] =  [b/sqrt(2) + c/sqrt(2) ]
        //                  [-a/sqrt(2) + b*0 + c*0 + d/sqrt(2)]    [-a/sqrt(2) + d/sqrt(2)]
        //                  [-a/sqrt(2) - b*0 + c*0 - d/sqrt(2)]    [-a/sqrt(2) - d/sqrt(2)]
        //                  [a*0 - b/sqrt(2) + c/sqrt(2)  d*0  ]    [-b/sqrt(2) + c/sqrt(2)]

        // q'' = 1/sqrt(2) * [b + c ]
        //                   [-a + d]
        //                   [-a - d]
        //                   [-b + c]

        // Finally:
        // q' = Cq * q'' = 1/sqrt(2) * [0*(b + c) + (-a + d)/sqrt(2) + (-a - d)/sqrt(2) - 0*(-b + c)]
        //                             [0*(-a + d) -(b + c)/sqrt(2) - (-b + c)/sqrt(2) - 0 *(-a -d) ]
        //                             [0*(-a -d) + (-b + c)/sqrt(2) - (b + c)/sqrt(2) + 0*(-a + d) ]
        //                             [0*(-b + c) - (-a - d)/sqrt(2) + (-a + d)/sqrt(2) + 0*(b + c)]

        // q' = 1/sqrt(2) * [(-a + d -a - d)/sqrt(2) ] = 1/2 * [-2a] = [-a]
        //                  [(-b - c + b - c)/sqrt(2)]         [-2c]   [-c]
        //                  [(-b + c - b - c)/sqrt(2)]         [-2b]   [-b]
        //                  [(a + d - a + d)/sqrt(2) ]         [2d ]   [ d]

        // Conversion could be computed as:
        /// Quaternion.product(CONVERSION_ROTATION, input, output)
        // Quaternion.product(output, CONVERSION_ROTATION, output)
        // output.normalize()

        // However, since we know the actual expression, it is more efficient to compute:

        val a = input.a
        val b = input.b
        val c = input.c
        val d = input.d

        output.a = -a
        output.b = -c
        output.c = -b
        output.d = d
    }

    /**
     * Converts a rotation from ENU to NED or from NED to ENU, taking into account that:
     * Renu = CONVERSION_ROTATION * Rned * CONVERSION_ROTATION
     * And also:
     * Rned = CONVERSION_ROTATION * Renu * CONVERSION_ROTATION
     *
     * @param value Input quaternion to be converted.
     * @return new converted quaternion.
     */
    fun convertAndReturnNew(value: Quaternion): Quaternion {
        val result = Quaternion()
        convert(value, result)
        return result
    }

    /**
     * Obtains a rotation to convert from ENU to NED and from NED to ENU coordinate systems.
     *
     * @param result instance where result will be stored.
     */
    fun conversionRotation(result: Rotation3D) {
        result.fromRotation(CONVERSION_ROTATION)
    }

    /**
     * Obtains a rotation to convert from ENU to NED and from NED to ENU coordinate systems.
     */
    val conversionRotation: Quaternion
        get() = Quaternion(CONVERSION_ROTATION)

    /**
     * Obtains a matrix to convert from ENU to NED and from NED to ENU coordinate systems.
     *
     * @param result instance where result will be stored. If matrix is not 3x3, it will be resized.
     */
    fun conversionRotationMatrix(result: Matrix) {
        if (result.rows != Rotation3D.INHOM_COORDS || result.columns != Rotation3D.INHOM_COORDS) {
            result.resize(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)
        }
        result.setElementAtIndex(0, 0.0)
        result.setElementAtIndex(1, 1.0)
        result.setElementAtIndex(2, 0.0)

        result.setElementAtIndex(3, 1.0)
        result.setElementAtIndex(4, 0.0)
        result.setElementAtIndex(5, 0.0)

        result.setElementAtIndex(6, 0.0)
        result.setElementAtIndex(7, 0.0)
        result.setElementAtIndex(8, -1.0)
    }

    /**
     * Obtains a matrix to convert from ENU to NED and from NED to ENU coordinate systems.
     */
    val conversionRotationMatrix: Matrix
        get() {
            val result = Matrix(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)
            conversionRotationMatrix(result)
            return result
        }

    /**
     * Computes jacobian resulting from ENU to NED quaternion conversion.
     *
     * @param result instance where jacobian will be stored. If matrix is not 4x4, it will be
     * resized.
     */
    fun quaternionConversionJacobian(result: Matrix) {
        if (result.rows != Quaternion.N_PARAMS || result.columns != Quaternion.N_PARAMS) {
            result.resize(Quaternion.N_PARAMS, Quaternion.N_PARAMS)
        }

        // Because CONVERSION_ROTATION = Cq quaternion is [0, -1/sqrt(2), -1/sqrt(2), 0]
        // And quaternion q is [a b c d]
        // Then the product: q' = Cq * q * Cq is:

        // q' = [-a]
        //      [-c]
        //      [-b]
        //      [ d]

        // The jacobian of q' = [a' b' c' d'] respect q = [a b c d] is:
        // J =  [d(a')/d(a)  d(a')/d(b)  d(a')/d(c)  d(a')/d(d)]
        //      [d(b')/d(a)  d(b')/d(b)  d(b')/d(c)  d(b')/d(d)]
        //      [d(c')/d(a)  d(c')/d(b)  d(c')/d(c)  d(c')/d(d)]
        //      [d(d')/d(a)  d(d')/d(b)  d(d')/d(c)  d(d')/d(d)]

        // J =  [-1  0   0   0]
        //      [ 0  0  -1   0]
        //      [ 0 -1   0   0]
        //      [ 0  0   0   1]

        result.setElementAtIndex(0, -1.0)
        result.setElementAtIndex(1, 0.0)
        result.setElementAtIndex(2, 0.0)
        result.setElementAtIndex(3, 0.0)

        result.setElementAtIndex(4, 0.0)
        result.setElementAtIndex(5, 0.0)
        result.setElementAtIndex(6, -1.0)
        result.setElementAtIndex(7, 0.0)

        result.setElementAtIndex(8, 0.0)
        result.setElementAtIndex(9, -1.0)
        result.setElementAtIndex(10, 0.0)
        result.setElementAtIndex(11, 0.0)

        result.setElementAtIndex(12, 0.0)
        result.setElementAtIndex(13, 0.0)
        result.setElementAtIndex(14, 0.0)
        result.setElementAtIndex(15, 1.0)
    }

    /**
     * Obtains jacobian resulting from ENU to NED quaternion conversion.
     */
    val quaternionConversionJacobian: Matrix
        get() {
            val result = Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS)
            quaternionConversionJacobian(result)
            return  result
        }
}