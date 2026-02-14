package com.irurueta.android.navigation.inertial.old.processors.attitude

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
        get() = ProcessNoiseCovarianceMethod.APPROXIMATED

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

        // The first term is constant, and the second term is first degree

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

        qd.copyFrom(q)
        qd.multiplyByScalar(timeIntervalSeconds)

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