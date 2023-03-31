/*
 * Copyright (C) 2022 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.android.navigation.inertial

import com.irurueta.algebra.Matrix
import com.irurueta.geometry.Quaternion
import kotlin.math.pow

/**
 * Helper related to quaternion operations.
 */
object QuaternionHelper {

    /**
     * Computes dot product of two quaternions.
     *
     * @param q1 1st quaternion.
     * @param q2 2nd quaternion.
     * @return dot product.
     */
    fun dotProduct(q1: Quaternion, q2: Quaternion): Double {
        q1.normalize()
        q2.normalize()
        return q1.a * q2.a +
                q1.b * q2.b +
                q1.c * q2.c +
                q1.d * q2.d
    }

    /**
     * Normalizes a quaternion and (if provided) computes the jacobian with respect provided
     * quaternion resulting from normalization.
     *
     * @param q quaternion to be normalized.
     * @param jacobian matrix where jacobian will be stored (if provided). Must be 4x4.
     * @throws IllegalArgumentException if [jacobian] matrix is not null and not 4x4.
     */
    @Throws(IllegalArgumentException::class)
    fun normalize(q: Quaternion, jacobian: Matrix? = null) {
        // Given a quaternion q = [a b c d]
        // Its norm is defined by: (a^2 + b^2 + c^2 + d^2)^2
        // And the normalized quaternion is q' = 1/(a^2 + b^2 + c^2 + d^2)^0.5 * [a b c d] = [a' b' c' d']
        // Where both q and q' are considered to represent an equivalent rotation, since quaternion
        // rotations are independent of their norm.

        // Hence:
        // a' = a / (a^2 + b^2 + c^2 + d^2)^0.5
        // b' = b / (a^2 + b^2 + c^2 + d^2)^0.5
        // c' = c / (a^2 + b^2 + c^2 + d^2)^0.5
        // d' = d / (a^2 + b^2 + c^2 + d^2)^0.5

        // The jacobian of q' = [a' b' c' d'] respect q = [a b c d] is:
        // J =  [d(a')/d(a)  d(a')/d(b)  d(a')/d(c)  d(a')/d(d)]
        //      [d(b')/d(a)  d(b')/d(b)  d(b')/d(c)  d(b')/d(d)]
        //      [d(c')/d(a)  d(c')/d(b)  d(c')/d(c)  d(c')/d(d)]
        //      [d(d')/d(a)  d(d')/d(b)  d(d')/d(c)  d(d')/d(d)]

        // d(a')/d(a) = (b^2 + c^2 + d^2) / (a^2 + b^2 + c^2 + d^2)^1.5
        // d(a')/d(b) = - a*b / (a^2 + b^2 + c^2 + d^2)^1.5
        // d(a')/d(c) = - a*c / (a^2 + b^2 + c^2 + d^2)^1.5
        // d(a')/d(d) = - a*d / (a^2 + b^2 + c^2 + d^2)^1.5

        // d(b')/d(a) = - a*b / (a^2 + b^2 + c^2 + d^2)^1.5
        // d(b')/d(b) = (a^2 + c^2 + d^2) / (a^2 + b^2 + c^2 + d^2)^1.5
        // d(b')/d(c) = - b*c / (a^2 + b^2 + c^2 + d^2)^1.5
        // d(b')/d(d) = - b*d / (a^2 + b^2 + c^2 + d^2)^1.5

        // d(c')/d(a) = - a*c / (a^2 + b^2 + c^2 + d^2)^1.5
        // d(c')/d(b) = - b*c / (a^2 + b^2 + c^2 + d^2)^1.5
        // d(c')/d(c) = (a^2 + b^2 + d^2) / (a^2 + b^2 + c^2 + d^2)^1.5
        // d(c')/d(d) = - c*d / (a^2 + b^2 + c^2 + d^2)^1.5

        // d(d')/d(a) = - a*d / (a^2 + b^2 + c^2 + d^2)^1.5
        // d(d')/d(b) = - b*d / (a^2 + b^2 + c^2 + d^2)^1.5
        // d(d')/d(c) = - c*d / (a^2 + b^2 + c^2 + d^2)^1.5
        // d(d')/d(d) = (a^2 + b^2 + c^2) / (a^2 + b^2 + c^2 + d^2)^1.5

        if (jacobian != null) {
            require(jacobian.rows == Quaternion.N_PARAMS)
            require(jacobian.columns == Quaternion.N_PARAMS)

            val a = q.a
            val b = q.b
            val c = q.c
            val d = q.d

            val a2 = a * a
            val b2 = b * b
            val c2 = c * c
            val d2 = d * d

            val sqrNorm = a2 + b2 + c2 + d2
            val norm3 = sqrNorm.pow(1.5)

            val minusAb = -a * b
            val minusAc = -a * c
            val minusAd = -a * d

            val minusBc = -b * c
            val minusBd = -b * d

            val minusCd = -c * d

            val b2plusc2 = b2 + c2
            val a2plusd2 = a2 + d2

            val dada = (b2plusc2 + d2) / norm3
            val dadb = minusAb / norm3
            val dadc = minusAc / norm3
            val dadd = minusAd / norm3

            val dbdb = (a2plusd2 + c2) / norm3
            val dbdc = minusBc / norm3
            val dbdd = minusBd / norm3

            val dcdc = (a2plusd2 + b2) / norm3
            val dcdd = minusCd / norm3

            val dddd = (b2plusc2 + a2) / norm3

            jacobian.setElementAt(0, 0, dada)
            jacobian.setElementAt(0, 1, dadb)
            jacobian.setElementAt(0, 2, dadc)
            jacobian.setElementAt(0, 3, dadd)

            jacobian.setElementAt(1, 0, dadb)
            jacobian.setElementAt(1, 1, dbdb)
            jacobian.setElementAt(1, 2, dbdc)
            jacobian.setElementAt(1, 3, dbdd)

            jacobian.setElementAt(2, 0, dadc)
            jacobian.setElementAt(2, 1, dbdc)
            jacobian.setElementAt(2, 2, dcdc)
            jacobian.setElementAt(2, 3, dcdd)

            jacobian.setElementAt(3, 0, dadd)
            jacobian.setElementAt(3, 1, dbdd)
            jacobian.setElementAt(3, 2, dcdd)
            jacobian.setElementAt(3, 3, dddd)
        }

        q.normalize()
    }
}