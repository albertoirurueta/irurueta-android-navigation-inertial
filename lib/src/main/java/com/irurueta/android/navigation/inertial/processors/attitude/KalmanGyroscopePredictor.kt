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
import com.irurueta.android.navigation.inertial.numerical.SuhQuaternionStepIntegrator
import com.irurueta.android.navigation.inertial.numerical.TrawnyQuaternionStepIntegrator
import com.irurueta.android.navigation.inertial.numerical.YuanQuaternionStepIntegrator
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegrator

/**
 * Predicts next Kalman filter state based on current gyroscope measurement.
 *
 * @param phiMatrixMethod method to estimate phi matrix relating previous and current filter state.
 * @param processNoiseCovarianceMethod method to estimate process noise covariance matrix.
 * @param quaternionStepIntegratorType method to integrate quaternions on each measurement.
 * @property gyroscopeNoisePsd gyroscope noise PSD.
 */
class KalmanGyroscopePredictor(
    phiMatrixMethod: PhiMatrixMethod = PhiMatrixMethod.APPROXIMATED,
    processNoiseCovarianceMethod: ProcessNoiseCovarianceMethod = ProcessNoiseCovarianceMethod.BETTER,
    quaternionStepIntegratorType: QuaternionStepIntegratorType = QuaternionStepIntegratorType.RUNGE_KUTTA,
    val gyroscopeNoisePsd: Double = DEFAULT_GYROSCOPE_NOISE_PSD,
) {
    /**
     * Contains corrected leveled local NED attitude respect to Earth from previous iteration.
     */
    /*var nedQ: Quaternion = Quaternion()
        set(value) {
            field.fromQuaternion(value)
        }*/

    /**
     * Contains gyroscope data expressed in NED coordinates respect to body.
     */
    var nedBodyAngularSpeedTriad: AngularSpeedTriad = AngularSpeedTriad()
        set(value) {
            field.copyFrom(value)
        }

    /**
     * Time interval between measurements expressed in seconds (s).
     *
     * @throws IllegalArgumentException if provided value is negative.
     */
    var timeIntervalSeconds: Double = 0.0
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value >= 0.0)
            field = value
        }

    /**
     * Previous step attitude respect to body.
     * Because rotation is defined respect to body, rotation converts from body coordinates
     * to leveled local NED coordinates respect to Earth:
     * pn = previousBodyQ * pb
     */
    var previousBodyQ : Quaternion = Quaternion()
        set(value) {
            field.fromQuaternion(value)
        }

    /**
     * Contains predicted attitude respect to body for next step.
     * Because rotation is defined respect to body, rotation converts from body coordinates
     * to leveled local NED coordinates respect to Earth:
     * pn = previousBodyQ * pb
     */
    val predictedBodyQ: Quaternion = Quaternion()

    /**
     * Contains predicted leveled local NED attitude respect to Earth for next step.
     * Converts from leveled NED local coordinates to body NED coordinates:
     * pb = predictedNedQ * pn
     */
//    val predictedNedQ: Quaternion = Quaternion()

    /**
     * Contains predicted attitude respect to body for next step.
     * This is the inverse of [predictedNedQ] and converts from body coordinates to leveled NED
     * ones: pn = predictedBodyQ * pb
     */
//    val predictedBodyQ: Quaternion = Quaternion()

    /**
     * Covariance measurement matrix of the gyroscope measurements.
     */
    val rg: Matrix =
        Matrix.identity(AngularSpeedTriad.COMPONENTS, AngularSpeedTriad.COMPONENTS)

    /**
     * Continuous time process noise covariance matrix.
     */
    val q: Matrix =
        Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    /**
     * Covariance noise matrix of the gyroscope bias
     */
    val qbg: Matrix =
        Matrix.identity(AngularSpeedTriad.COMPONENTS, AngularSpeedTriad.COMPONENTS)

    /**
     * Covariance noise matrix of the accelerometer bias
     */
    val qba: Matrix =
        Matrix.identity(AccelerationTriad.COMPONENTS, AccelerationTriad.COMPONENTS)

    // x_hat__prev_prev
    /**
     * Kalman filter state before prediction.
     *
     * @throws IllegalArgumentException if provided matrix is not 9x1.
     */
    var kalmanStatePrePredicted = Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, 1)
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value.rows == KALMAN_FILTER_DYNAMIC_PARAMETERS)
            require(value.columns == 1)
            field.copyFrom(value)
        }

    // x_hat__next_prev
    /**
     * Kalman filter state after prediction.
     */
    val kalmanStatePostPredicted = Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, 1)

    /**
     * Kalman filter error covariance matrix before prediction.
     *
     * @throws IllegalArgumentException if provided matrix is not 9x9.
     */
    var kalmanErrorCovPrePredicted =
        Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, KALMAN_FILTER_DYNAMIC_PARAMETERS)
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value.rows == KALMAN_FILTER_DYNAMIC_PARAMETERS)
            require(value.columns == KALMAN_FILTER_DYNAMIC_PARAMETERS)
            field.copyFrom(value)
        }

    /**
     * Kalman filter error covariance matrix after prediction.
     */
    val kalmanErrorCovPostPredicted =
        Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    /**
     * Contains previous step leveled local NED attitude respect to Earth.
     * Rotation is expressed respect to Earth, which means that a point in Earth leveled local NED
     * coordinates can be converted to body coordinates using this quaternion:
     * pb = previousNedQ * pn
     */
//    private val previousNedQ: Quaternion = Quaternion()

    /**
     * Contains previous step attitude respect to body.
     * This is the inverse of [previousNedQ] and converts points from body coordinates to Earth
     * leveled local ned coordinates: pn = previousBodyQ * pb
     */
//    private val previousBodyQ: Quaternion = Quaternion()

    /**
     * Contains previous gyroscope data expressed in NED coordinates respect to body.
     */
    private val previousNedBoyAngularSpeedTriad: AngularSpeedTriad = AngularSpeedTriad()

    /**
     * Integrates angular speed measurements to compute predicted quaternion variation.
     */
    private val quaternionStepIntegrator: QuaternionStepIntegrator by lazy {
        createQuaternionStepIntegratorType(quaternionStepIntegratorType)
    }

    /**
     * NED angular speed represented as a matrix.
     */
    private val angularSpeedMatrix: Matrix = Matrix(AngularSpeedTriad.COMPONENTS, 1)

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
     * 3x3 identity matrix.
     */
    private val identity3: Matrix =
        Matrix.identity(AngularSpeedTriad.COMPONENTS, AngularSpeedTriad.COMPONENTS)

    /**
     * Computes matrix that relates previous and predicted Kalman filter state.
     */
    private val phiMatrixEstimator: PhiMatrixEstimator =
        PhiMatrixEstimator.create(method = phiMatrixMethod)

    /**
     * Matrix relating previous and predicted state when time sampling.
     */
    private val phi: Matrix =
        Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    /**
     * Integrates continuous time process noise covariance matrix.
     */
    private val processNoiseCovarianceIntegrator: ProcessNoiseCovarianceIntegrator =
        ProcessNoiseCovarianceIntegrator.create(method = processNoiseCovarianceMethod)

    /**
     * Discrete-time process Noise covariance matrix.
     */
    private val qd: Matrix =
        Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    /**
     * Transposed of [phi].
     */
    private val phiTransposed =
        Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    /**
     * Resets internal parameter.
     */
    fun reset() {
        timeIntervalSeconds = 0.0

        Matrix.identity(rg)
        q.initialize(0.0)
        qbg.initialize(0.0)
        qba.initialize(0.0)

        // set small values for bias matrices in continuous time process noise covariance matrix
        // to avoid bias components being stuck at a given value
        Matrix.identity(qbg)
        qbg.multiplyByScalar(BG)
        Matrix.identity(qba)
        qba.multiplyByScalar(BG)


        // reset Kalman filter matrices
        kalmanStatePrePredicted.initialize(0.0)
        kalmanStatePostPredicted.initialize(0.0)

        kalmanErrorCovPrePredicted.initialize(0.0)
        kalmanErrorCovPostPredicted.initialize(0.0)

        // reset previous attitude estimation
        previousBodyQ.a = 1.0
        previousBodyQ.b = 0.0
        previousBodyQ.c = 0.0
        previousBodyQ.d = 0.0

        predictedBodyQ.fromQuaternion(previousBodyQ)

/*        previousNedQ.a = 1.0
        previousNedQ.b = 0.0
        previousNedQ.c = 0.0
        previousNedQ.d = 0.0

        previousBodyQ.fromQuaternion(previousNedQ)*/

        // reset previous gyroscope measurement
        previousNedBoyAngularSpeedTriad.setValueCoordinates(0.0, 0.0, 0.0)
    }

    /**
     * Predicts expected orientation, orientation error, gyroscope and accelerometer biases using
     * current gyroscope measurements and current orientation estimation.
     */
    fun predict() {
//        previousNedQ.fromQuaternion(nedQ)
//        previousNedQ.inverse(previousBodyQ)

        // Propagate quaternion using a step integrator
        val previousWx = previousNedBoyAngularSpeedTriad.valueX
        val previousWy = previousNedBoyAngularSpeedTriad.valueY
        val previousWz = previousNedBoyAngularSpeedTriad.valueZ
        val currentWx = nedBodyAngularSpeedTriad.valueX
        val currentWy = nedBodyAngularSpeedTriad.valueY
        val currentWz = nedBodyAngularSpeedTriad.valueZ
        quaternionStepIntegrator.integrate(
            previousBodyQ,
            previousWx,
            previousWy,
            previousWz,
            currentWx,
            currentWy,
            currentWz,
            timeIntervalSeconds,
            predictedBodyQ
        )

//        predictedBodyQ.inverse(predictedNedQ)

        // Compute the state transition matrix Φ and the discrete-time process noise covariance
        computePhiAndNoiseCovarianceMatrices()

        // project the state ahead
        phi.multiply(kalmanStatePrePredicted, kalmanStatePostPredicted)

        // Project the state covariance matrix ahead (P__next_prev)
        // kalmanTemp1 = Phi * P(k)
        phi.multiply(kalmanErrorCovPrePredicted, kalmanErrorCovPostPredicted)
        // P'(k) =  kalmanTemp1 * Phi' + Qd
        phi.transpose(phiTransposed)
        kalmanErrorCovPostPredicted.multiply(phiTransposed)
        kalmanErrorCovPostPredicted.add(qd)

        previousNedBoyAngularSpeedTriad.copyFrom(nedBodyAngularSpeedTriad)
        predictedBodyQ.fromQuaternion(predictedBodyQ)
//        previousNedQ.fromQuaternion(predictedNedQ)
    }

    /**
     * Compute the state transition matrix Φ and the discrete-time process noise covariance.
     */
    private fun computePhiAndNoiseCovarianceMatrices() {
        computeProcessEquationMatrix()
        computePhiMatrix()
        computeProcessNoiseCovarianceMatrix()
    }

    /**
     * Computes process equation matrix defining the differential equation system:
     * d(x(f))/dt = A*x(t) + w
     */
    private fun computeProcessEquationMatrix() {
        nedBodyAngularSpeedTriad.getValuesAsMatrix(angularSpeedMatrix)
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
        phiMatrixEstimator.a = a
        phiMatrixEstimator.estimate(timeIntervalSeconds, phi)
    }

    /**
     * Computes process noise covariance matrix.
     */
    private fun computeProcessNoiseCovarianceMatrix() {
        updateCovarianceMeasurementMatrix()

        q.initialize(0.0)
        q.setSubmatrix(0, 0, 2, 2, rg)
        q.multiplyByScalar(0.25)
        q.setSubmatrix(3, 3, 5, 5, qbg)
        q.setSubmatrix(6, 6, 8, 8, qba)

        processNoiseCovarianceIntegrator.a = a
        processNoiseCovarianceIntegrator.q = q
        processNoiseCovarianceIntegrator.integrate(timeIntervalSeconds, qd)
    }

    private fun updateCovarianceMeasurementMatrix() {
        Matrix.identity(rg)
        val gyroVariance = gyroscopeNoisePsd / timeIntervalSeconds
        rg.multiplyByScalar(gyroVariance)
    }

    /**
     * Creates a quaternion step integrator to integrate angular speed measurements to predict
     * a new orientation when new gyroscope measurements arrive.
     *
     * @param type type of step integrator to create.
     * @return created quaternion step integrator.
     */
    private fun createQuaternionStepIntegratorType(type: QuaternionStepIntegratorType): QuaternionStepIntegrator {
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

    init {
        qbg.multiplyByScalar(BG)
        qba.multiplyByScalar(BG)
    }

    companion object {
        /**
         * Kalman filter's number of dynamic parameters.
         * Filter state contains:
         * - error of orientation quaternion respect to Earth (in local navigation system expressed
         * in NED coordinates). Because error is small, real value is assumed to be 1, and only 3
         * axis components are store.
         * - bias of gyroscope respect to body expressed in NED coordinates. 3x1 vector
         * - bias of accelerometer respect to body expressed in NED coordinates. 3x1 vector.
         */
        private const val KALMAN_FILTER_DYNAMIC_PARAMETERS = 9

        /**
         * Default variance of the gyroscope output per Hz (or variance at 1 Hz).
         * This is equivalent to the gyroscope PSD (Power Spectral Density) that can be
         * obtained during calibration or with noise estimators.
         * This is a typical value that can be used for non-calibrated devices.
         * This is expressed as (rad/s)^2 / Hz or rad^2/s.
         */
        const val DEFAULT_GYROSCOPE_NOISE_PSD = 1e-7

        /**
         * Small value to avoid process noise bias components being stuck.
         */
        private const val BG = 1e-6
    }
}