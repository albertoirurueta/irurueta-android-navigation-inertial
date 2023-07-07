/*
 * Copyright (C) 2023 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.android.navigation.inertial.processors.attitude

import com.irurueta.algebra.Matrix

/**
 * Approximately computes matrix that relates previous and predicted Kalman filter state by assuming
 * short time intervals and approximating the exponential of process equation matrix by the first
 * term of its Taylor expansion series.
 *
 * @property a Process equation matrix relating previous and predicted state in a time continuous
 * space.
 */
class ApproximatedPhiMatrixEstimator(a: Matrix? = null) : PhiMatrixEstimator(a) {

    /**
     * Contains scaled copy of [a].
     */
    private val at: Matrix = Matrix(N_ROWS, N_COLUMNS)

    /**
     * Contains squared [a].
     */
    private val a2: Matrix = Matrix(N_ROWS, N_COLUMNS)

    /**
     * 9x9 identity matrix.
     */
    private val identity9: Matrix = Matrix.identity(N_ROWS, N_COLUMNS)

    /**
     * Gets method to estimate phi matrix.
     */
    override val method: PhiMatrixMethod
        get() = PhiMatrixMethod.APPROXIMATED

    /**
     * Estimates matrix that relates previous and predicted Kalman filter state.
     *
     * @param timeIntervalSeconds time interval between current and last sample expressed in
     * seconds.
     * @param result instance where estimated phi matrix will be stored.
     * @throws IllegalArgumentException if provided time interval is negative or result matrix is
     * not 9x9.
     * @throws IllegalStateException if [a] matrix was not provided.
     */
    override fun estimate(timeIntervalSeconds: Double, result: Matrix) {
        val a = this.a
        check(a != null)
        require(result.rows == N_ROWS)
        require(result.columns == N_COLUMNS)
        require(timeIntervalSeconds >= 0.0)

        // Approximate Phi = exp(a * dt) by the first two terms of its Taylor expansion series.
        // Phi = eye(9) + A * dt + 1/2 * A^2 * dt^2;
        a.multiply(a, a2)

        at.copyFrom(a)
        at.multiplyByScalar(timeIntervalSeconds)

        a2.multiplyByScalar(0.5 * timeIntervalSeconds * timeIntervalSeconds)

        identity9.add(at, result)
        result.add(a2)
    }
}