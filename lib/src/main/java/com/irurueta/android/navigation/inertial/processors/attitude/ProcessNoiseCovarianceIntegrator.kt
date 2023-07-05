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
 * Integrates continuous time process noise covariance matrix.
 *
 * @property q Continuous time process noise covariance matrix.
 * @property a Process equation matrix relating previous and predicted state in a time continuous
 * space.
 * @throws IllegalArgumentException if provided q and a matrices are not 9x9.
 */
abstract class ProcessNoiseCovarianceIntegrator(
    val q: Matrix,
    val a: Matrix
) {

    /**
     * Gets method use to estimate process noise covariance.
     */
    abstract val method: ProcessNoiseCovarianceMethod

    /**
     * Computes process noise covariance as the integration of [a] and [q] during provided time
     * interval.
     *
     * @param lowerBoundTimestamp time interval lower bound expressed in seconds.
     * @param upperBoundTimestamp time interval upper bound expressed in seconds.
     * @param result instance where result of integration will be stored.
     */
    abstract fun integrate(lowerBoundTimestamp: Double, upperBoundTimestamp: Double, result: Matrix)

    init {
        require(a.rows == N_ROWS)
        require(a.columns == N_COLUMNS)
        require(q.rows == N_ROWS)
        require(q.columns == N_COLUMNS)
    }

    companion object {
        /**
         * Number of rows of A (Process equation matrix) and Q (Continuous time process noise
         * covariance matrix) matrices to be integrated.
         */
        const val N_ROWS = 9

        /**
         * Number of columns of A (Process equation matrix) and Q (Continuous time process noise
         * covariance matrix) matrices to be integrated.
         */
        const val N_COLUMNS = 9

        /**
         * Default method used by instantiate integrators.
         */
        val DEFAULT_METHOD = ProcessNoiseCovarianceMethod.APPROXIMATED

        /**
         * Creates a new instance of [ProcessNoiseCovarianceIntegrator] using provided method.
         *
         * @param q Continuous time process noise covariance matrix.
         * @param a Process equation matrix relating previous and predicted state in a time
         * continuous space.
         * @param method method to be used (either approximated or precise).
         * @return created instance.
         */
        fun create(
            q: Matrix,
            a: Matrix,
            method: ProcessNoiseCovarianceMethod
        ): ProcessNoiseCovarianceIntegrator {
            return when (method) {
                ProcessNoiseCovarianceMethod.APPROXIMATED -> ApproximatedProcessNoiseCovarianceIntegrator(
                    q,
                    a
                )

                ProcessNoiseCovarianceMethod.PRECISE -> PreciseProcessNoiseCovarianceIntegrator(
                    q,
                    a
                )
            }
        }

        /**
         * Creates a new instance of [ProcessNoiseCovarianceIntegrator] using default method.
         *
         * @param q Continuous time process noise covariance matrix.
         * @param a Process equation matrix relating previous and predicted state in a time
         * continuous space.
         * @return created instance.
         */
        fun create(q: Matrix, a: Matrix): ProcessNoiseCovarianceIntegrator {
            return create(q, a, DEFAULT_METHOD)
        }
    }
}