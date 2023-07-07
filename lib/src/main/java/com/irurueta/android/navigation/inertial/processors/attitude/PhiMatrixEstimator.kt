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
 * Computes matrix that relates previous and predicted Kalman filter state.
 *
 * @property a Process equation matrix relating previous and predicted state in a time continuous
 * space.
 */
abstract class PhiMatrixEstimator(a: Matrix?) {

    /**
     * Gets or sets process equation matrix relating previous and predicted state in a time
     * continuous space.
     */
    var a: Matrix? = a
        set(value) {
            if (value != null) {
                require(value.rows == ProcessNoiseCovarianceIntegrator.N_ROWS)
                require(value.columns == ProcessNoiseCovarianceIntegrator.N_COLUMNS)
            }

            field = value
        }

    /**
     * Gets method to estimate phi matrix.
     */
    abstract val method: PhiMatrixMethod

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
    abstract fun estimate(timeIntervalSeconds: Double, result: Matrix)

    init {
        this.a = a
    }

    companion object {

        /**
         * Number of rows of A (Process equation matrix)
         */
        const val N_ROWS = 9

        /**
         * Number of columns of A (Process equation matrix)
         */
        const val N_COLUMNS = 9

        /**
         * Default method used to instantiate estimators.
         */
        val DEFAULT_METHOD = PhiMatrixMethod.APPROXIMATED

        /**
         * Creates a new instance of [PhiMatrixEstimator] using provided method.
         *
         * @param a Process equation matrix relating previous and predicted state in a time
         * continuous space.
         * @param method method to be used (either approximated or precise).
         * @return created instance
         */
        fun create(a: Matrix? = null, method: PhiMatrixMethod = DEFAULT_METHOD): PhiMatrixEstimator {
            return when(method) {
                PhiMatrixMethod.APPROXIMATED -> ApproximatedPhiMatrixEstimator(a)
                PhiMatrixMethod.PRECISE -> PrecisePhiMatrixEstimator(a)
            }
        }
    }
}