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
package com.irurueta.android.navigation.inertial.old.processors.attitude

import com.irurueta.algebra.Matrix
import com.irurueta.numerical.ExponentialMatrixEstimator
import com.irurueta.numerical.integration.IntegrationException
import com.irurueta.numerical.integration.IntegratorType
import com.irurueta.numerical.integration.MatrixIntegrator
import com.irurueta.numerical.integration.MatrixSingleDimensionFunctionEvaluatorListener
import com.irurueta.numerical.integration.QuadratureType

/**
 * Integrates continuous time process noise covariance matrix.
 *
 * @property q Continuous time process noise covariance matrix.
 * @property a Process equation matrix relating previous and predicted state in a time continuous
 * space.
 * @throws IllegalArgumentException if provided q and a matrices are not 9x9.
 */
class PreciseProcessNoiseCovarianceIntegrator(
    q: Matrix? = null,
    a: Matrix? = null,
    val integratorType: IntegratorType = MatrixIntegrator.DEFAULT_INTEGRATOR_TYPE,
    val quadratureType: QuadratureType = MatrixIntegrator.DEFAULT_QUADRATURE_TYPE
) : ProcessNoiseCovarianceIntegrator(q, a) {
    /**
     * Computes matrix exponential.
     */
    private val exponentialMatrixEstimator: ExponentialMatrixEstimator =
        ExponentialMatrixEstimator()

    /**
     * Contains product of [a] with a time value, being reused for efficiency purposes.
     */
    private val at: Matrix = Matrix(N_ROWS, N_COLUMNS)

    /**
     * Exponential of a * t matrix being reused for efficiency purposes.
     */
    private val expAt: Matrix = Matrix(N_ROWS, N_COLUMNS)

    /**
     * Transposed of [expAt] being reused for efficiency purposes.
     */
    private val transposedExpAt: Matrix = Matrix(N_ROWS, N_COLUMNS)

    /**
     * Matrix integrand. This is reused for efficiency purposes.
     */
    private val integrand: Matrix = Matrix(N_ROWS, N_COLUMNS)

    /**
     * Listener to evaluate function to be integrated.
     */
    private val listener = object : MatrixSingleDimensionFunctionEvaluatorListener {
        override fun evaluate(t: Double, result: Matrix) {
            // compute integral of: e^(A .* t) * Q * (e^(A .* t))';
            at.copyFrom(this@PreciseProcessNoiseCovarianceIntegrator.a)
            at.multiplyByScalar(t)
            exponentialMatrixEstimator.exponential(at, expAt)
            expAt.transpose(transposedExpAt)

            integrand.copyFrom(expAt)
            integrand.multiply(this@PreciseProcessNoiseCovarianceIntegrator.q)
            integrand.multiply(transposedExpAt)
            result.copyFrom(integrand)
        }

        override fun getRows(): Int {
            return N_ROWS
        }

        override fun getColumns(): Int {
            return N_COLUMNS
        }
    }

    /**
     * Gets method use to estimate process noise covariance.
     */
    override val method: ProcessNoiseCovarianceMethod
        get() = ProcessNoiseCovarianceMethod.PRECISE

    /**
     * Computes process noise covariance as the integration of [a] and [q] during provided time
     * interval.
     *
     * @param timeIntervalSeconds time interval between current and last sample expressed in
     * seconds.
     * @param result instance where result of integration will be stored.
     * @throws IntegrationException if integration fails
     */
    @Throws(IntegrationException::class)
    override fun integrate(timeIntervalSeconds: Double, result: Matrix) {
        check(a != null)
        check(q != null)
        require(result.rows == N_ROWS)
        require(result.columns == N_COLUMNS)
        require(timeIntervalSeconds >= 0.0)

        val integrator = MatrixIntegrator.create(0.0, timeIntervalSeconds, listener)
        integrator.integrate(result)
    }
}