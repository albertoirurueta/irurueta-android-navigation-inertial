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

import com.irurueta.geometry.Quaternion

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
}