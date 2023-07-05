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
import com.irurueta.algebra.Utils
import com.irurueta.android.navigation.inertial.KalmanFilter
import com.irurueta.android.navigation.inertial.numerical.ExponentialMatrixEstimator
import com.irurueta.android.navigation.inertial.numerical.SuhQuaternionStepIntegrator
import com.irurueta.android.navigation.inertial.numerical.TrawnyQuaternionStepIntegrator
import com.irurueta.android.navigation.inertial.numerical.YuanQuaternionStepIntegrator
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegrator

/**
 * Estimates absolute attitude using a Kalman filter.
 * This is based on:
 * https://github.com/liviobisogni/quaternion-kalman-filter
 * And:
 * Young Soo Suh. Orientation estimation using a quaternion-based indirect Kalman filter with
 * adaptive estimation of external acceleration.
 */
class KalmanAbsoluteAttitudeProcessor4(
    val gyroscopeNoiseStandardDeviation: Double, // rad/s
    val accelerometerNoiseStandardDeviation: Double, // m /s ^2)
    val magnetometerNoiseStanardDeviation: Double, // T
    val phiMatrixMethod: PhiMatrixMethod = PhiMatrixMethod.APPROXIMATED,
    quaternionStepIntegrator: QuaternionStepIntegratorType
) {

    /**
     * Indicates whether processor has been initialized or not.
     * A processor is considered to be initialized if enough measurements are available
     * and time interval can be determined.
     */
    private var initialized = false

    /**
     * Timestamp of previous sample expressed in nanoseconds.
     */
    private var previousTimestamp: Long = -1L

    /**
     * Time interval between measurements expressed in seconds (s).
     */
    var timeIntervalSeconds: Double = 0.0
        private set

    /**
     * Contains accelerometer data expressed in NED coordinates.
     * This is reused for performance reasons.
     */
    private val nedAccelerationTriad: AccelerationTriad = AccelerationTriad()

    /**
     * Contains gyroscope data expressed in NED coordinates.
     */
    private val nedAngularSpeedTriad: AngularSpeedTriad = AngularSpeedTriad()

    /**
     * Contains previous gyroscope data expressed in NED coordinates.
     * This is reused for performance reasons.
     */
    private val previousNedAngularSpeedTriad: AngularSpeedTriad = AngularSpeedTriad()

    /**
     * Contains magnetometer data expressed in NED coordinates.
     * This is reused for performance reasons.
     */
    private val nedMagneticFluxDensityTriad: MagneticFluxDensityTriad = MagneticFluxDensityTriad()

    /**
     * NED angular speed represented as a matrix.
     */
    private val angularSpeedMatrix: Matrix = Matrix(AngularSpeedTriad.COMPONENTS, 1)

    /**
     * Computes matrix exponentials.
     */
    private val exponentialMatrixEstimator: ExponentialMatrixEstimator =
        ExponentialMatrixEstimator()

    /**
     * Skew matrix of angular speed.
     */
    private val angularSpeedSkewMatrix: Matrix =
        Matrix(AngularSpeedTriad.COMPONENTS, AngularSpeedTriad.COMPONENTS)

    /**
     * Process equation matrix relating previous and predicted state in a time continuous space.
     */
    private val a: Matrix =
        Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    /**
     * Contains squared [a].
     */
    private val a2: Matrix =
        Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    /**
     * Contains transposed [a].
     */
    private val aTransposed: Matrix =
        Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    /**
     * 3x3 identity matrix.
     */
    private val identity3: Matrix =
        Matrix.identity(AngularSpeedTriad.COMPONENTS, AngularSpeedTriad.COMPONENTS)

    /**
     * 9x9 identity matrix.
     */
    private val identity9: Matrix =
        Matrix.identity(KALMAN_FILTER_DYNAMIC_PARAMETERS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    /**
     * Matrix relating previous and predicted state when time sampling.
     */
    private val phi: Matrix =
        Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    /**
     * Continuous time process noise covariance matrix.
     */
    private val q: Matrix =
        Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    /**
     * Discrete-time process Noise covariance matrix.
     */
    private val qd: Matrix =
        Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    /**
     * Contains [a] * [q] matrix.
     */
    private val aq: Matrix =
        Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    /**
     * Contains [a] * [aTransposed] matrix.
     */
    private val qaTransposed: Matrix =
        Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    /**
     * Contains previous step NED attitude.
     */
    private val previousNedQ: Quaternion = Quaternion()

    /**
     * Contains predicted NED attitude for next step.
     */
    private val predictedNedQ: Quaternion = Quaternion()

    /**
     * Integrates angular speed measurements to compute predicted quaternion variation.
     */
    private val quaternionStepIntegrator: QuaternionStepIntegrator by lazy {
        createQuaternionStepIntegratorType(
            quaternionStepIntegrator
        )
    }

    /**
     * Kalman filter.
     */
    private val kalmanFilter =
        KalmanFilter(KALMAN_FILTER_DYNAMIC_PARAMETERS, KALMAN_FILTER_MEASUREMENT_PARAMETERS)

    private fun predict() {
        // Propagate quaternion using a step integrator
        val previousWx = previousNedAngularSpeedTriad.valueX
        val previousWy = previousNedAngularSpeedTriad.valueY
        val previousWz = previousNedAngularSpeedTriad.valueZ
        val currentWx = nedAngularSpeedTriad.valueX
        val currentWy = nedAngularSpeedTriad.valueY
        val currentWz = nedAngularSpeedTriad.valueZ
        quaternionStepIntegrator.integrate(
            previousNedQ,
            previousWx,
            previousWy,
            previousWz,
            currentWx,
            currentWy,
            currentWz,
            timeIntervalSeconds,
            predictedNedQ
        )

        // Compute the state transition matrix Φ and the discrete-time process noise covariance
        computePhyAndNoiseCovarianceMatrices()

        val transitionMatrix = kalmanFilter.transitionMatrix
        transitionMatrix.copyFrom(phi)

        val processNoiseCov = kalmanFilter.processNoiseCov
        processNoiseCov.copyFrom(q)

// TODO:        kalmanFilter.statePost.copyFrom()

        kalmanFilter.predict()
    }

    private fun computePhyAndNoiseCovarianceMatrices() {
        computeProcessEquationMatrix()
        computePhiMatrix()
        computeProcessNoiseCovarianceMatrix()
    }

    /**
     * Computes process equation matrix defining the differential equation system:
     * d(x(f))/dt = A*x(t) + w
     */
    private fun computeProcessEquationMatrix() {
        nedAngularSpeedTriad.getValuesAsMatrix(angularSpeedMatrix)
        Utils.skewMatrix(angularSpeedMatrix, angularSpeedSkewMatrix)
        angularSpeedSkewMatrix.multiplyByScalar(-1.0)

        a.initialize(0.0)
        a.setSubmatrix(0, 0, 2, 2, angularSpeedSkewMatrix)

        Matrix.identity(identity3)
        identity3.multiplyByScalar(-0.5)
        a.setSubmatrix(0, 3, 2, 5, identity3)
    }

    /**
     * Computes matrix that relates previous and predicted Kalman filter state.
     * Kalman filter state contains:
     * - quaternion respect to Earth (in local navigation system expressed in NED coordinates).
     * 4x1 vector
     * - bias of gyroscope respect to body expressed in NED coordinates. 3x1 vector
     * - bias of accelerometer respect to body expressed in NED coordinates. 3x1 vector.
     */
    private fun computePhiMatrix() {

        when (phiMatrixMethod) {
            PhiMatrixMethod.PRECISE -> {
                // Phi = exp(A * dt);
                a.multiplyByScalar(timeIntervalSeconds)
                exponentialMatrixEstimator.exponential(a, phi)
            }

            PhiMatrixMethod.APPROXIMATED -> {
                // Phi = eye(9) + A * dt + 1/2 * A^2 * dt^2;
                a.multiply(a, a2)

                a.multiplyByScalar(timeIntervalSeconds)
                a2.multiplyByScalar(0.5 * timeIntervalSeconds * timeIntervalSeconds)

                identity9.add(a, phi)
                phi.add(a2)
            }
        }
    }

    private fun computeProcessNoiseCovarianceMatrix() {
        q.initialize(0.0)
        q.setSubmatrix(0, 0, 2, 2, rg)
        q.multiplyByScalar(0.25)
        q.setSubmatrix(3, 3, 5, 5, qbg)
        q.setSubmatrix(6, 6, 8, 8, qba)

        // Q_d = Q * dt + 1/2 * A * Q + 1/2 * Q * A';
        qd.copyFrom(q)
        qd.multiplyByScalar(timeIntervalSeconds)

        a.multiply(q, aq)
        aq.multiplyByScalar(0.5)
        qd.add(aq)

        a.transpose(aTransposed)
        q.multiply(aTransposed, qaTransposed)
        qaTransposed.multiplyByScalar(0.5)
        qd.add(qaTransposed)

        // TODO: precise -> ProcessNoiseCovarianceIntegrator
    }

    /**
     * Covariance measurement matrix of the gyroscope
     */
    private val rg: Matrix =
        Matrix.identity(AngularSpeedTriad.COMPONENTS, AngularSpeedTriad.COMPONENTS)

    /**
     * Covariance noise matrix of the gyroscope bias
     */
    private val qbg: Matrix =
        Matrix.identity(AngularSpeedTriad.COMPONENTS, AngularSpeedTriad.COMPONENTS)

    /**
     * Covariance noise matrix of the accelerometer bias
     */
    private val qba: Matrix =
        Matrix.identity(AccelerationTriad.COMPONENTS, AccelerationTriad.COMPONENTS)

    init {
        Matrix.identity(rg)
        rg.multiplyByScalar(gyroscopeNoiseStandardDeviation * gyroscopeNoiseStandardDeviation)
    }

    /**
     * Supported methods to estimate phi matrix relating previous and predicted Kalman filter state.
     */
    enum class PhiMatrixMethod {
        PRECISE,
        APPROXIMATED
    }

    /**
     * Inicates type of quaternion integrator step. Different types exist with different levels of
     * accuracy and computation accuracy.
     */
    enum class QuaternionStepIntegratorType {
        /**
         * Performs quaternion integration using Suh's method, which has a medium accuracy and
         * computational complexity.
         */
        SUH,

        /**
         * Performs quaternion integration using Trawny's method, which has a medium accuracy and
         * computational complexity.
         */
        TRAWNY,

        /**
         * Performs quaternion integration using Yuan's meth0d, which has a medium accuracy and
         * computational complexity.
         */
        YUAN,

        /**
         * Performs quaternion integration using Euler's method, which is the item accurate and has
         * the smallest computational complexity.
         */
        EULER_METHOD,

        /**
         * Performs quaternion integration using mid-point algorithm, which offers a medium accuracy
         * an computational complexity.
         */
        MID_POINT,

        /**
         * Performs quaternion integration using Runge-Kutta of 4th order (aka RK4) algorithm, which
         * offers high accuracy at the expense of higher computational complexity.
         */
        RUNGE_KUTTA
    }

    companion object {
        /**
         * Kalman filter's number of dynamic parameters.
         * Filter state contains:
         * - quaternion respect to Earth (in local navigation system expressed in NED coordinates).
         * 4x1 vector
         * - bias of gyroscope respect to body expressed in NED coordinates. 3x1 vector
         * - bias of accelerometer respect to body expressed in NED coordinates. 3x1 vector.
         */
        private const val KALMAN_FILTER_DYNAMIC_PARAMETERS = 9

        /**
         * Kalman filter's number of measurement parameters.
         * Filter uses measurements of:
         * - Accelerometer respect to body expressed in NED coordinates. 3x1 vector.
         * - Gyroscope respect to body expressed in NED coordinates. 3x1 vector.
         * - Magnetometer respect to body expressed in NED coordinates. 3x1 vector.
         */
        private const val KALMAN_FILTER_MEASUREMENT_PARAMETERS = 9

        private fun createQuaternionStepIntegratorType(type: QuaternionStepIntegratorType):
                QuaternionStepIntegrator {

            return when (type) {
                QuaternionStepIntegratorType.SUH -> SuhQuaternionStepIntegrator()

                QuaternionStepIntegratorType.TRAWNY -> TrawnyQuaternionStepIntegrator()

                QuaternionStepIntegratorType.YUAN -> YuanQuaternionStepIntegrator()

                QuaternionStepIntegratorType.EULER_METHOD -> QuaternionStepIntegrator.create(
                    com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegratorType.EULER_METHOD
                )

                QuaternionStepIntegratorType.MID_POINT -> QuaternionStepIntegrator.create(
                    com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegratorType.MID_POINT
                )

                QuaternionStepIntegratorType.RUNGE_KUTTA -> QuaternionStepIntegrator.create(
                    com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegratorType.RUNGE_KUTTA
                )
            }
        }
    }
}