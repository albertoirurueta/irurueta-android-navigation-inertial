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
 * Better approximation to integrate continuous time process noise covariance matrix.
 *
 * @property q Continuous time process noise covariance matrix.
 * @property a Process equation matrix relating previous and predicted state in a time continuous
 * space.
 * @throws IllegalArgumentException if provided q and a matrices are not 9x9.
 */
class BetterProcessNoiseCovarianceIntegrator(
    q: Matrix? = null,
    a: Matrix? = null,
) : ProcessNoiseCovarianceIntegrator(q, a) {

    /**
     * Contains approximated integration.
     */
    private val qd: Matrix = Matrix(N_ROWS, N_COLUMNS)

    /**
     * Contains product of [a] and [q]
     */
    private val aq: Matrix = Matrix(N_ROWS, N_COLUMNS)

    /**
     * Contains transpose of [a].
     */
    private val aTransposed: Matrix = Matrix(N_ROWS, N_COLUMNS)

    /**
     * Contains [q] * [aTransposed].
     */
    private val qaTransposed: Matrix = Matrix(N_ROWS, N_COLUMNS)

    /**
     * Gets method use to estimate process noise covariance.
     */
    override val method: ProcessNoiseCovarianceMethod
        get() = ProcessNoiseCovarianceMethod.BETTER

    /**
     * Computes process noise covariance as the integration of [a] and [q] during provided time
     * interval.
     *
     * @param timeIntervalSeconds time interval between current and last sample expressed in
     * seconds.
     * @param result instance where result of integration will be stored.
     */
    override fun integrate(
        timeIntervalSeconds: Double,
        result: Matrix
    ) {
        val a = this.a
        val q = this.q
        check(a != null)
        check(q != null)
        require(result.rows == N_ROWS)
        require(result.columns == N_COLUMNS)
        require(timeIntervalSeconds >= 0.0)

        // whe have process equation matrix that relates previous and predicted state in a time
        // continuous space A(t), which is evaluated at periodic intervals k*dt, where dt is the
        // time interval between samples: A(0) = 0, A(k*dt) = k*dt*A, A(2*k*dt) = 2*k*dt*A...

        // we want to integrate expression:
        // f(t) = e^(A(t)) * Q * (e^(A(t)))' = e^(A*t) * Q * (e^(A*t))'
        // for short intervals t exponential can be approximated by the first term of its Taylor
        // series expansion as:
        // e^(A(t)) ~ I + A(t) = I + A*t

        // Hence:
        // f(t) ~ (I + A*t) * Q * (I + A*t)' = (Q + A*t*Q) * (I + A(t)') =
        // f(t) ~ Q + A*t*Q + Q*(A*t)' + A*t*Q*(A*t)'
        // f(t) ~ Q + A*Q*t + Q*t'*A' + A*t*Q*t'*A'
        // f(t) ~ Q + A*Q*t + Q*A'*t + A*Q*A'*t^2

        // Since time interval t is assumed to be small, the second degree term (last term) is much
        // smaller than the other ones, and consequently it can be dropped:
        // f(t) ~ Q + A*Q*t + Q*A'*t
        // f(t) ~ Q + (A*Q + Q*A')*t

        // The first term is constant, and the second term is first degree

        // The integrand is a first degree matrix polynomial, and its integral is:
        // F(t) = Q*t + 1/2*(A*Q + Q*A')*t^2

        // the integral between lower and upper bounds during a single k-th interval is:
        // upper bound: u = (k + 1)*dt
        // lower bound = l = k*dt
        // where the time interval dt is dt = u - l

        // F(u) - F(l) =  Q*(u - l) + 1/2*(A*Q + Q*A')*(u^2 - l^2)

        // where:
        // u - l = (k + 1)*dt - k*dt = dt
        // u^2 - l^2 = (k + 1)^2*dt^2 - k^2*dt^2 = (k^2 + 2*k + 1)*dt^2 - k^2*dt^2 =
        // = k^2*dt^2 + 2*k*dt^2 + dt^2 - k^2*dt^2 = 2*k*dt^2 + dt^2 = (2*k + 1)*dt^2

        // then:
        // F(u) - F(l) = F(k,dt) = Q*dt + 1/2*(A*Q + Q*A')*(2*k + 1)*dt^2

        // assuming first sample since last measurement (k = 0):
        // F(dt) = Q*dt + 1/2*(A*Q + Q*A')*dt^2

        val dt2 = timeIntervalSeconds * timeIntervalSeconds

        qd.copyFrom(q)
        qd.multiplyByScalar(timeIntervalSeconds)

        a.multiply(q, aq)
        aq.multiplyByScalar(0.5 * dt2)
        qd.add(aq)

        a.transpose(aTransposed)
        q.multiply(aTransposed, qaTransposed)
        qaTransposed.multiplyByScalar(0.5 * dt2)
        qd.add(qaTransposed)

        result.copyFrom(qd)
    }
}