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
 * Approximately integrates continuous time process noise covariance matrix.
 *
 * @property q Continuous time process noise covariance matrix.
 * @property a Process equation matrix relating previous and predicted state in a time continuous
 * space.
 * @throws IllegalArgumentException if provided q and a matrices are not 9x9.
 */
class ApproximatedProcessNoiseCovarianceIntegrator(
    q: Matrix,
    a: Matrix,
) : ProcessNoiseCovarianceIntegrator(q, a) {

    private val qd: Matrix = Matrix(N_ROWS, N_COLUMNS)

    private val aq: Matrix = Matrix(N_ROWS, N_COLUMNS)

    private val aTransposed: Matrix = Matrix(N_ROWS, N_COLUMNS)

    private val qaTransposed: Matrix = Matrix(N_ROWS, N_COLUMNS)

    /**
     * Gets method use to estimate process noise covariance.
     */
    override val method: ProcessNoiseCovarianceMethod
        get() = ProcessNoiseCovarianceMethod.APPROXIMATED

    /**
     * Computes process noise covariance as the integration of [a] and [q] during provided time
     * interval.
     *
     * @param lowerBoundTimestamp time interval lower bound expressed in seconds.
     * @param upperBoundTimestamp time interval upper bound expressed in seconds.
     * @param result instance where result of integration will be stored.
     */
    override fun integrate(
        lowerBoundTimestamp: Double,
        upperBoundTimestamp: Double,
        result: Matrix
    ) {
        require(result.rows == N_ROWS)
        require(result.columns == N_COLUMNS)

        // whe have process equation matrix that relates previous and predicted state in a time
        // continuous space A(t), which is evaluated at periodic intervals k*dt, where dt is the
        // time interval between samples: A(0), A(k*dt), A(2*k*dt)...

        // we want to integrate expression:
        // f(t) = e^(A(t)) * Q * (e^(A(t)))'
        // for short intervals t exponential can be approximated by the first term of its Taylor
        // series expansion as:
        // e^(A(t)) ~ I + A(t)

        // Hence:
        // f(t) ~ (I + A(t)) * Q * (I + A(t))' = (Q + A(t)*Q) * (I + A(t)') =
        // f(t) ~ Q + A(t)*Q + Q*A(t)' + A(t)*Q*A(t)'

        // Since time interval t is assumed to be small, the second degree term (last term) is much
        // smaller than the first one, and consequently it can be dropped:
        // f(t) ~ Q + A(t)*Q + Q*A(t)'

        // The first term is constant, and the second term is second degree

        // The integrand is a first degree matrix polynomial, and its integral is:
        // F(t) = Q*t + 1/2*(A(t)*Q + Q*A(t)')

        // the integral between lower and upper bounds during a single k-th interval is:
        // upper bound: u = (k + 1)*dt
        // lower bound = l = k*dt
        // where the time interval dt is dt = u - l

        // F(u) - F(l) = F(k,dt) = Q*dt + 1/2*(A(k*dt)*Q + Q*A(k*dt)')

        // Expression above is continuous in time, its discrete version for a single interval is
        // (k = 1):
        // F = Q*dt + 1/2*(A*Q + Q*A')
        // F = Q*dt + 1/2*A*Q + 1/2*Q*A'

        val dt = upperBoundTimestamp - lowerBoundTimestamp
        qd.copyFrom(q)
        qd.multiplyByScalar(dt)

        a.multiply(q, aq)
        aq.multiplyByScalar(0.5)
        qd.add(aq)

        a.transpose(aTransposed)
        q.multiply(aTransposed, qaTransposed)
        qaTransposed.multiplyByScalar(0.5)
        qd.add(qaTransposed)

        result.copyFrom(qd)
    }
}