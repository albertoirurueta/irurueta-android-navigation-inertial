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

import android.hardware.SensorManager
import com.irurueta.algebra.ArrayUtils
import com.irurueta.algebra.Matrix
import com.irurueta.algebra.Utils
import com.irurueta.android.navigation.inertial.ENUtoNEDConverter
import com.irurueta.android.navigation.inertial.KalmanFilter
import com.irurueta.android.navigation.inertial.collectors.AccelerometerAndGyroscopeSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorMeasurement
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegrator
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegratorType
import com.irurueta.units.TimeConverter
import kotlin.math.abs
import kotlin.math.exp
import kotlin.math.sqrt

/**
 * Estimates relative attitude using a Kalman filter.
 * This is based on Android native implementation:
 * https://android.googlesource.com/platform/frameworks/native/+/refs/heads/master/services/sensorservice/Fusion.cpp
 *
 * This implementation uses a Kalman filter where attitude is estimated based on UP unitary
 * direction vector obtained from accelerometer measurements for filter corrections, and current
 * system state based on angular speed for attitude predictions.
 *
 * @property processCovariance Indicates whether attitude covariance must be processed or not.
 * @property processEulerAngles Indicates whether Euler angles must be estimated from NED attitude
 * or not. This is only taken into account if [processCovariance] is true.
 * @property processEulerAnglesCovariance Indicates whether propagated covariance for Euler angles
 * expressed in NED coordinates is estimated or not. This is only taken into account if
 * [processCovariance] and [processEulerAngles] are true.
 * @property gyroscopeVariance Variance of the gyroscope output per Hz (or variance at 1Hz). This is
 * equivalent to the gyroscope PSD (Power Spectral Density) that can be obtained during calibration
 * or with noise estimators. If not provided [DEFAULT_GYROSCOPE_VARIANCE] will be used.
 * @property gyroscopeBiasVariance Variance of gyroscope bias expressed as (rad/s^2) / s. If not
 * provided [DEFAULT_GYROSCOPE_BIAS_VARIANCE] will be used.
 * @property accelerometerStandardDeviation Accelerometer standard deviation expressed in m/s^2. If
 * not provided [DEFAULT_ACCELEROMETER_STANDARD_DEVIATION] will be used.
 * @property freeFallThreshold Threshold to consider that device is in free fall. When device is in
 * free fall, accelerometer measurements are considered unreliable and are ignored (only gyroscope
 * predictions are made).
 * @property quaternionStepIntegratorType type of quaternion step integrator to be used for
 * gyroscope integration.
 * @property processorListener listener to notify new attitudes and their uncertainties.
 * @property errorCovarianceResetListener listener to notify when error covariance needs to be reset
 * because numerical instabilities have been found.
 */
class KalmanRelativeAttitudeProcessor(
    val processCovariance: Boolean = true,
    val processEulerAngles: Boolean = true,
    val processEulerAnglesCovariance: Boolean = true,
    val gyroscopeVariance: Double = DEFAULT_GYROSCOPE_VARIANCE,
    val gyroscopeBiasVariance: Double = DEFAULT_GYROSCOPE_BIAS_VARIANCE,
    val accelerometerStandardDeviation: Double = DEFAULT_ACCELEROMETER_STANDARD_DEVIATION,
    val freeFallThreshold: Double = DEFAULT_FREE_FALL_THRESHOLD,
    val quaternionStepIntegratorType: QuaternionStepIntegratorType =
        QuaternionStepIntegrator.DEFAULT_TYPE,
    var processorListener: OnProcessedListener? = null,
    var errorCovarianceResetListener: OnErrorCovarianceResetListener? = null
) {

    /**
     * Estimated attitude expressed in NED coordinates.
     */
    val nedAttitude = Quaternion()

    /**
     * Gyroscope interval between measurements expressed in seconds (s).
     */
    var gyroscopeIntervalSeconds: Double = 0.0
        private set

    /**
     * Error covariance of estimated quaternion attitude expressed in NED coordinates.
     * This is only available if [processCovariance] is true.
     */
    val nedQuaternionCovariance: Matrix?
        get() = if (processCovariance) {
            internalNedQuaternionCovariance
        } else {
            null
        }

    /**
     * Euler angles associated to estimated NED attitude.
     * Array contains roll, pitch and yaw angles expressed in radians (rad) and following order
     * indicated here.
     * This is only available if [processCovariance] and [processEulerAngles] are true.
     */
    val eulerAngles: DoubleArray?
        get() = if (processCovariance && processEulerAngles) {
            internalEulerAngles
        } else {
            null
        }

    /**
     * Error covariance of estimated Euler angles for NED attitude.
     * This is only available if [processCovariance] and [processEulerAngles] and
     * [processEulerAnglesCovariance] are true.
     */
    val eulerAnglesCovariance: Matrix?
        get() = if (processCovariance && processEulerAngles && processEulerAnglesCovariance) {
            internalEulerAnglesCovariance
        } else {
            null
        }

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
     * Contains accelerometer data expressed in NED coordinates.
     * This is reused for performance reasons.
     */
    private val accelerationTriad: AccelerationTriad = AccelerationTriad()

    /**
     * Contains gyroscope data expressed in NED coordinates.
     * This is reused for performance reasons.
     */
    private val angularSpeedTriad: AngularSpeedTriad = AngularSpeedTriad()

    /**
     * Contains previous gyroscope data expressed in NED coordinates.
     * This is reused for performance reasons.
     */
    private val previousAngularSpeedTriad: AngularSpeedTriad = AngularSpeedTriad()

    /**
     * Array containing unit vector pointing upwards. This is used during initialization to obtain
     * an initial attitude.
     * This is reused for performance reasons.
     */
    private val down: DoubleArray = DoubleArray(COMPONENTS)

    /**
     * Array containing unit vector pointing east. This is used during initialization to obtain
     * an initial attitude.
     * This is reused for performance reasons.
     */
    private val east: DoubleArray = DoubleArray(COMPONENTS)

    /**
     * Array containing unit vector pointing north. This is used during initialization to obtain
     * an initial attitude.
     * This is reused for performance reasons.
     */
    private val north: DoubleArray = DoubleArray(COMPONENTS)

    /**
     * Contains new measurements provided to Kalman filter, which will consist of normalized
     * acceleration (which ideally should point upwards). Accelerometer data conveniently low-pass
     * filtered senses gravity component of measured specific force. Since Kalman filter predicts
     * new up unit vector based on gyroscope measurements, new accelerometer measurements can be
     * conveniently filtered out to obtain up direction (opposite of gravity) estimations.
     * This is reused for performance reasons.
     */
    private val m = Matrix(COMPONENTS, 1)

    /**
     * Rotation matrix containing current attitude expressed in ENU coordinates.
     * This is reused for performance reasons.
     */
    private val r = Matrix(COMPONENTS, COMPONENTS)

    /**
     * Quaternion containing current attitude expressed in ENU coordinates.
     * This is reused for performance reasons.
     */
    private val q = Quaternion()

    /**
     * Quaternion containing expected attitude expressed in ENU coordinates after integration.
     * This is reused for performance reasons.
     */
    private val expectedQ = Quaternion()

    /**
     * Inverse of quaternion [q].
     * This is reused for performance reasons.
     */
    private val invQ = Quaternion()

    /**
     * Variation of attitude between [q] and [expectedQ].
     * This is reused for performance reasons.
     */
    private val deltaQ = Quaternion()

    /**
     * Kalman filter using normalized accelerometer vector as measurements.
     * This Kalman filter estimates attitude based on UP unitary
     * direction vector obtained from accelerometer measurements for filter corrections, and current
     * system state based on angular speed for attitude predictions.
     * With such predictions the filter can average accelerometer measurements to find
     * an accurate attitude estimation.
     */
    private val kalmanFilter = KalmanFilter(Quaternion.N_PARAMS, COMPONENTS)

    /**
     * Error covariance of normalized quaternion attitude expressed in ENU coordinates.
     * This is used internally and reused for performance reasons.
     */
    private val normalizedCovariance =
        Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS)

    /**
     * Error covariance of quaternion attitude expressed in NED coordinates.
     * This is used internally and reused for performance reasons.
     */
    private val internalNedQuaternionCovariance = Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS)

    /**
     * Euler angles of estimated NED attitude.
     * This is used internally and reused for performance reasons.
     */
    private val internalEulerAngles = DoubleArray(Quaternion.N_ANGLES)

    /**
     * Jacobian of quaternion normalization.
     */
    private val normalizationJacobian = Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS)

    /**
     * Transposed of [normalizationJacobian].
     * This is reused for performance reasons.
     */
    private val transposedNormalizationJacobian = Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS)

    /**
     * Jacobian of conversion NED quaternion attitude into Euler angles.
     * This is used for propagation of error covariance uncertainty into euler angles covariance
     * uncertainty.
     */
    private val eulerJacobian = Matrix(Quaternion.N_ANGLES, Quaternion.N_PARAMS)

    /**
     * Transposed of [eulerJacobian].
     * This is reused for performance reasons.
     */
    private val transposedEulerJacobian = Matrix(Quaternion.N_ANGLES, Quaternion.N_PARAMS)

    /**
     * Error covariance of estimated Euler angles for NED attitude.
     * This is reused for performance reasons.
     */
    private val internalEulerAnglesCovariance = Matrix(Quaternion.N_ANGLES, Quaternion.N_ANGLES)

    /**
     * Integrates angular speed measurements to compute predicted quaternion variation.
     */
    private val quaternionStepIntegrator =
        QuaternionStepIntegrator.create(quaternionStepIntegratorType)

    /**
     * Processes provided synced accelerometer and gyroscope measurement to obtain a relative
     * leveled attitude.
     *
     * @param syncedMeasurement measurement to be processed.
     * @return true if a new attitude is estimated, false otherwise.
     */
    fun process(syncedMeasurement: AccelerometerAndGyroscopeSyncedSensorMeasurement): Boolean {
        val accelerometerMeasurement = syncedMeasurement.accelerometerMeasurement
        val gyroscopeMeasurement = syncedMeasurement.gyroscopeMeasurement
        return if (accelerometerMeasurement != null && gyroscopeMeasurement != null) {
            process(accelerometerMeasurement, gyroscopeMeasurement, syncedMeasurement.timestamp)
        } else {
            false
        }
    }

    /**
     * Processes provided accelerometer and gyroscope measurements at provided timestamp to obtain a
     * relative leveled attitude.
     *
     * @param accelerometerMeasurement accelerometer measurement to be processed.
     * @param gyroscopeMeasurement gyroscope measurement to be processed.
     * @param timestamp timestamp when both measurements are assumed to occur. If not provided,
     * this is assumed to be gyroscope timestamp.
     * @return true if a new attitude is estimated, false otherwise.
     */
    fun process(
        accelerometerMeasurement: AccelerometerSensorMeasurement,
        gyroscopeMeasurement: GyroscopeSensorMeasurement,
        timestamp: Long = gyroscopeMeasurement.timestamp
    ): Boolean {
        val isFirst = updateTimeInterval(timestamp)
        val result = if (isFirst) {
            processMeasurement(gyroscopeMeasurement, angularSpeedTriad)
            false
        } else {
            processMeasurement(accelerometerMeasurement, accelerationTriad)
            // we will always work with normalized accelerometer data to obtain up direction
            val accelerometerNorm = normalize(accelerationTriad)

            processMeasurement(gyroscopeMeasurement, angularSpeedTriad)

            if (!initialized) {
                estimateInitialAttitude()

                initKalmanFilter()

                initialized = true
            } else {
                // set transition matrix for current state
                updateTransitionMatrix()

                // noise associated to accelerometer measurements, the closer the device is at free
                // fall (accelerometer data closer to zero), the less reliable measurements are
                // considered
                updateMeasurementNoiseCovariance(accelerometerNorm)

                // predict using current transition matrix
                val statePre = kalmanFilter.predict()

                val isFreeFall = accelerometerNorm < freeFallThreshold
                val state = if (isFreeFall) {
                    // device is in free fall
                    kalmanFilter.statePost.copyFrom(statePre)
                    statePre
                } else {
                    // after prediction, update measurement matrix for next correction
                    updateMeasurementMatrix()

                    // correct using current accelerometer data
                    accelerationTriad.getValuesAsMatrix(m)
                    kalmanFilter.correct(m)
                }

                // represent state as quaternion
                copyColumnMatrixToQuaternion(state, q)

                normalizeAndUpdateState(state)

                // copy internal attitude
                nedAttitude.fromQuaternion(q)

                processCovariance(isFreeFall)

                processorListener?.onProcessed(
                    this,
                    nedAttitude,
                    eulerAngles,
                    nedQuaternionCovariance,
                    eulerAnglesCovariance
                )
            }

            true
        }

        angularSpeedTriad.copyTo(previousAngularSpeedTriad)
        return result
    }

    /**
     * Resets internal parameters.
     */
    fun reset() {
        previousTimestamp = -1L
        gyroscopeIntervalSeconds = 0.0
        initialized = false
        initKalmanFilter()
    }

    /**
     * Updates current time interval estimation between gyroscope measurements.
     *
     * @param timestamp timestamp when measurements being processed are assumed to occur.
     * @return true if it is the 1st measurement by the time interval estimator, false otherwise.
     */
    private fun updateTimeInterval(timestamp: Long): Boolean {
        val isFirst = previousTimestamp <= 0
        if (!isFirst) {
            val diff = timestamp - previousTimestamp
            gyroscopeIntervalSeconds = TimeConverter.nanosecondToSecond(diff.toDouble())
        }
        previousTimestamp = timestamp

        return isFirst
    }

    /**
     * Estimates initial attitude based on available accelerometer data.
     */
    private fun estimateInitialAttitude() {
        // set initial attitude in ENU coordinates
        accelerationTriad.getValuesAsArray(down)
        estimateOrthogonal(down, east)
        crossProduct(down, east, north)

        r.setSubmatrix(0, 0, 2, 0, north)
        r.setSubmatrix(0, 1, 2, 1, east)
        r.setSubmatrix(0, 2, 2, 2, down)
        q.fromInhomogeneousMatrix(r)
        q.normalize()
    }

    /**
     * Initializes Kalman filter by assuming that state is equal to initial attitude,
     * process noise covariance is set depending on known gyroscope calibration parameters, error
     * covariance are initialized to zero, and measurement matrix is set to make proper conversion
     * between quaternion attitudes and unitary up direction (based on accelerometer measurements).
     */
    private fun initKalmanFilter() {
        q.normalize()

        // set state
        val statePre = kalmanFilter.statePre
        copyQuaternionToColumnMatrix(q, statePre)

        val statePost = kalmanFilter.statePost
        copyQuaternionToColumnMatrix(q, statePost)

        val dt = gyroscopeIntervalSeconds
        val dt2 = dt * dt
        val dt3 = dt2 * dt

        // variance of integrated output at 1/dT Hz (random drift)
        val q00 = gyroscopeVariance * dt + gyroscopeBiasVariance * dt3 / 3.0

        // variance of drift rate ramp
        val q11 = gyroscopeBiasVariance * dt
        val q10 = 0.5 * gyroscopeBiasVariance * dt2

        // noise associated to quaternion variance
        val processNoiseCov = kalmanFilter.processNoiseCov
        processNoiseCov.initialize(0.0)
        processNoiseCov.setElementAt(0, 0, q00)
        processNoiseCov.setElementAt(1, 1, q11)
        processNoiseCov.setElementAt(2, 2, q11)
        processNoiseCov.setElementAt(3, 3, q11)
        processNoiseCov.setElementAt(0, 1, q10)
        processNoiseCov.setElementAt(0, 2, q10)
        processNoiseCov.setElementAt(0, 3, q10)
        processNoiseCov.setElementAt(1, 0, q10)
        processNoiseCov.setElementAt(2, 0, q10)
        processNoiseCov.setElementAt(3, 0, q10)

        // initialize error covariance a priori and posteriori to zero
        initErrorCovariances()
    }

    /**
     * Initializes Kalman filter predicted and corrected error covariances to zero.
     */
    private fun initErrorCovariances() {
        kalmanFilter.errorCovPre.initialize(0.0)
        kalmanFilter.errorCovPost.initialize(0.0)
    }

    /**
     * Updates measurement noise covariance based on the fact that accelerometer measurements close
     * to zero (when device is in free fall) are unreliable because norm cannot be reliably
     * estimated. Consequently, the closer device is to free fall, measurements are considered less
     * reliable.
     */
    private fun updateMeasurementNoiseCovariance(accelerometerNorm: Double) {
        val d = sqrt(abs(accelerometerNorm - EARTH_GRAVITY))
        val std = accelerometerStandardDeviation * exp(d) / accelerometerNorm

        val measurementNoiseCov = kalmanFilter.measurementNoiseCov
        val accVar = std * std
        for (i in 0 until COMPONENTS) {
            measurementNoiseCov.setElementAt(i, i, accVar)
        }
    }

    /**
     * Normalizes attitude quaternion, updates filter state with normalized quaternion
     * and updates uncertainties to reflect effects of normalization.
     *
     * @param state matrix containing filter state.
     */
    private fun normalizeAndUpdateState(state: Matrix) {
        // estimate Jacobian to propagate uncertainty after normalization
        q.normalize(normalizationJacobian)
        normalizationJacobian.transpose(transposedNormalizationJacobian)

        // update state with normalized quaternion for greater accuracy
        copyQuaternionToColumnMatrix(q, state)

        // update covariances by propagating effects of normalization
        propagateNormalization(kalmanFilter.errorCovPre)
        propagateNormalization(kalmanFilter.errorCovPost)
    }

    /**
     * Propagates effects of normalization to provided covariance matrix using current estimated
     * Jacobian of normalization of attitude quaternion.
     *
     * @param covariance Matrix containing covariance to be updated by propagating effects of
     * normalization.
     */
    private fun propagateNormalization(covariance: Matrix) {
        // propagate uncertainty taking into account that: Cov`= J * Cov * J^T
        normalizationJacobian.multiply(covariance, normalizedCovariance)
        normalizedCovariance.multiply(transposedNormalizationJacobian, covariance)
    }

    /**
     * Updates filter transition matrix assuming Euler integration between quaternions and angular
     * speeds.
     * @see com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegrator to get
     * more information on quaternion integration.
     */
    private fun updateTransitionMatrix() {
        quaternionStepIntegrator.integrate(
            q,
            previousAngularSpeedTriad.valueX,
            previousAngularSpeedTriad.valueY,
            previousAngularSpeedTriad.valueZ,
            angularSpeedTriad.valueX,
            angularSpeedTriad.valueY,
            angularSpeedTriad.valueZ,
            gyroscopeIntervalSeconds,
            expectedQ
        )


        // compute delta attitude between current and expected quaternions:
        // q1 = deltaQ1 * q0
        // deltaQ1 = q1 * invQ0

        q.inverse(invQ)
        invQ.normalize()
        expectedQ.multiply(invQ, deltaQ)
        deltaQ.normalize()

        // q0 = [a0 b0 c0 d0]
        // deltaQ1 = [da db dc dd]
        // q1 = [a1 b1 c1 d1]

        // q1 = deltaQ1 * q0
        // [a1] = [da * a0 - db * b0 - dc * c0 - dd * d0] = [da     -db     -dc     -dd][a0]
        // [b1]   [da * b0 + db * a0 + dc * d0 - dd * c0]   [db      da     -dd      dc][b0]
        // [c1]   [da * c0 - db * d0 + dc * a0 + dd * b0]   [dc      dd      da     -db][c0]
        // [d1]   [da * d0 + db * c0 - dc * b0 + dd * a0]   [dd     -dc      db      da][d0]

        // matrix above corresponds to method [Quaternion#quaternionMatrix(Matrix)]

        val transitionMatrix = kalmanFilter.transitionMatrix
        deltaQ.quaternionMatrix(transitionMatrix)
    }

    private fun updateMeasurementMatrix() {
        // A point (or unit vector pointing up) up = [0 0 1]^T
        // Can be rotated with rotation matrix R as: up' = R * up

        // The rotation matrix R can be obtained from current attitude state
        // x = [a b c d]^T as:
        //
        // R = [a^2 + b^2 - c^2 - d^2       2*b*c - 2*a*d               2*b*d + 2*a*c        ]
        //     [2*b*c + 2*a*d               a^2 - b^2 + c^2 - d^2       2*c*d - 2*a*b        ]
        //     [2*b*d - 2*a*c               2*c*d + 2*a*b               a^2 - b^2 - c^2 + d^2]
        //
        // where x is the state of the Kalman filter containing
        // current attitude

        // Hence:
        // z = H * x = R * [0 0 1]^T = [2*(b*d + a*c)        ]
        //                             [2*(c*d - a*b)        ]
        //                             [a^2 - b^2 - c^2 + d^2]

        // Consequently:
        // z = H * x = [2*(b*d + a*c)/a         0                   0                           0][a]
        //             [0                       2*(c*d - a*b)/b     0                           0][b]
        //             [0                       0                   (a^2 - b^2 - c^2 + d^2)/c   0][c]
        //                                                                                        [d]

        val statePre = kalmanFilter.statePre
        val a = statePre.getElementAtIndex(0)
        val b = statePre.getElementAtIndex(1)
        val c = statePre.getElementAtIndex(2)
        val d = statePre.getElementAtIndex(3)

        val measurementMatrix = kalmanFilter.measurementMatrix
        measurementMatrix.setElementAt(0, 0, 2.0 * (b * d + a * c) / a)
        measurementMatrix.setElementAt(1, 1, 2.0 * (c * d - a * b) / b)
        measurementMatrix.setElementAt(2, 2, (a * a - b * b - c * c - d * d) / c)
    }

    /**
     * Processes current Kalman filter error covariance to obtain uncertainty estimations for
     * quaternion attitudes (both in ENU and NED coordinates), estimate Euler angles corresponding
     * to NED attitude, and also their corresponding uncertainties.
     * Estimation of covariances and Euler angles will depend of properties [processCovariance],
     * [processEulerAngles] and [processEulerAnglesCovariance].
     *
     * @param isFreeFall true indicates that device is in free fall, and consequently no measurement
     * corrections are made. Hence, in such cases predicted error covariance is used instead of
     * corrected error covariance.
     */
    private fun processCovariance(isFreeFall: Boolean) {
        if (processCovariance) {
            // process covariances
            val errorCov = if (isFreeFall) {
                kalmanFilter.errorCovPre
            } else {
                kalmanFilter.errorCovPost
            }

            // check covariance state
            if (!isPositiveSemidefinite(errorCov)) {
                // when covariance is not positive semi-definite we can assume that Kalman filter
                // has become numerically unstable, and we need to reset error covariance value
                initErrorCovariances()

                errorCovarianceResetListener?.onErrorCovarianceReset(this)
            }

            // take only the part corresponding to ENU quaternion covariance
            internalNedQuaternionCovariance.copyFrom(errorCov)

            if (processEulerAngles) {
                // As defined in: https://en.wikipedia.org/wiki/Propagation_of_uncertainty
                // covariance propagation for a function converting quaternion to euler angles is:
                // Cov`= J * Cov * J^T
                if (processEulerAnglesCovariance) {
                    nedAttitude.toEulerAngles(internalEulerAngles, eulerJacobian)
                    eulerJacobian.transpose(transposedEulerJacobian)

                    eulerJacobian.multiply(
                        internalNedQuaternionCovariance,
                        internalEulerAnglesCovariance
                    )
                    internalEulerAnglesCovariance.multiply(transposedEulerJacobian)
                } else {
                    nedAttitude.toEulerAngles(internalEulerAngles)
                }
            }
        }
    }

    companion object {
        /**
         * Default variance of the gyroscope output per Hz (or variance at 1Hz).
         * This is equivalent to the gyroscope PSD (Power Spectral Density) that can be
         * obtained during calibration or with noise estimators.
         * This is a typical value that can be used for non-calibrated devices.
         * This is expressed as (rad/s)^2 / Hz or rad^2/s.
         */
        const val DEFAULT_GYROSCOPE_VARIANCE = 1e-7

        /**
         * Default variance of gyroscope bias expressed as (rad/s^2) / s.
         */
        const val DEFAULT_GYROSCOPE_BIAS_VARIANCE = 1e-12

        /**
         * Default standard deviation for accelerometer expressed in m/s^2.
         */
        const val DEFAULT_ACCELEROMETER_STANDARD_DEVIATION = 0.015

        /**
         * Average gravity at Earth's sea level expressed in m/s^2.
         */
        private const val EARTH_GRAVITY = SensorManager.GRAVITY_EARTH

        /**
         * Default threshold to consider that device is in free fall.
         * When device is in free fall, accelerometer measurements are considered
         * unreliable and are ignored (only gyroscope predictions are made).
         */
        const val DEFAULT_FREE_FALL_THRESHOLD = 0.1 * EARTH_GRAVITY

        /**
         * Amount of components of measurements and arrays.
         */
        private const val COMPONENTS = AccelerationTriad.COMPONENTS

        /**
         * Threshold to determine whether covariance matrices are positive definite or not.
         */
        private const val SYMMETRIC_THRESHOLD = 1e-10

        /**
         * Processes an accelerometer measurement, taking into account their biases (if available).
         * Measurement will contain accelerometer data expressed in NED coordinates.
         *
         * @param measurement measurement to be processed.
         * @param result triad where measurement data will be stored in NED coordinates.
         * @throws IllegalArgumentException if provided result array does not have length 3.
         */
        @Throws(IllegalArgumentException::class)
        private fun processMeasurement(
            measurement: AccelerometerSensorMeasurement,
            result: AccelerationTriad
        ) {
            val ax = measurement.ax.toDouble()
            val ay = measurement.ay.toDouble()
            val az = measurement.az.toDouble()
            val bx = measurement.bx?.toDouble()
            val by = measurement.by?.toDouble()
            val bz = measurement.bz?.toDouble()

            val valueX = if (bx != null) ax - bx else ax
            val valueY = if (by != null) ay - by else ay
            val valueZ = if (bz != null) az - bz else az

            // convert to NED coordinates
            ENUtoNEDConverter.convert(valueX, valueY, valueZ, result)
        }

        /**
         * Processes a gyroscope measurement, taking into account their biases (if available).
         * Measurement will contain gyroscope data expressed in BED coordinates.
         *
         * @param measurement measurement to be processed.
         * @param result triad where measurement data will be stored in NED coordinates.
         * @throws IllegalArgumentException if provided result array does not have length 3.
         */
        @Throws(IllegalArgumentException::class)
        private fun processMeasurement(
            measurement: GyroscopeSensorMeasurement,
            result: AngularSpeedTriad
        ) {
            val wx = measurement.wx.toDouble()
            val wy = measurement.wy.toDouble()
            val wz = measurement.wz.toDouble()
            val bx = measurement.bx?.toDouble()
            val by = measurement.by?.toDouble()
            val bz = measurement.bz?.toDouble()

            val valueX = if (bx != null) wx - bx else wx
            val valueY = if (by != null) wy - by else wy
            val valueZ = if (bz != null) wz - bz else wz

            // convert to NED coordinates
            ENUtoNEDConverter.convert(valueX, valueY, valueZ, result)
        }

        /**
         * Gets Frobenious norm of provided array.
         *
         * @param data array to be evaluated.
         * @return estimated Frobenious norm.
         */
        private fun getNorm(data: DoubleArray): Double {
            return Utils.normF(data)
        }

        /**
         * Estimates an orthogonal vector to provided input one.
         *
         * @param input vector to obtain orthogonal one from.
         * @param output instance where estimated orthogonal vector will be stored.
         * @throws IllegalArgumentException if provided array do not have length 3.
         */
        @Throws(IllegalArgumentException::class)
        private fun estimateOrthogonal(input: DoubleArray, output: DoubleArray) {
            val absInput0 = abs(input[0])
            val absInput1 = abs(input[1])
            val absInput2 = abs(input[2])
            if (absInput0 <= absInput1 && absInput0 <= absInput2) {
                output[0] = 0.0
                output[1] = input[2]
                output[2] = -input[1]
            } else if (absInput1 <= absInput2) {
                output[0] = input[2]
                output[1] = 0.0
                output[2] = -input[0]
            } else {
                output[0] = input[1]
                output[1] = -input[0]
                output[2] = 0.0
            }

            normalize(output)
        }

        /**
         * Normalizes provided array by its Frobenious norm.
         *
         * @param array array to be normalized.
         * @return norm that array had before normalization.
         */
        private fun normalize(array: DoubleArray): Double {
            val norm = getNorm(array)
            ArrayUtils.multiplyByScalar(array, 1.0 / norm, array)
            return norm
        }

        /**
         * Normalizes provided acceleration triad by its Frobenious norm.
         *
         * @param triad triad to be normalized.
         * @return norm that the tria had before normalization.
         */
        private fun normalize(triad: AccelerationTriad): Double {
            val norm = triad.norm
            triad.valueX /= norm
            triad.valueY /= norm
            triad.valueZ /= norm
            return norm
        }

        /**
         * Computes cross product between 2 arrays.
         *
         * @param u First array.
         * @param v Second array.
         * @param result array where result of cross product will be stored.
         * @throws IllegalArgumentException if any of the arrays do not have length 3.
         */
        @Throws(IllegalArgumentException::class)
        private fun crossProduct(u: DoubleArray, v: DoubleArray, result: DoubleArray) {
            result[0] = u[1] * v[2] - u[2] * v[1]
            result[1] = u[2] * v[0] - u[0] * v[2]
            result[2] = u[0] * v[1] - u[1] * v[0]
        }

        /**
         * Indicates whether provided matrix is positive semidefinite or not upt to provided
         * threshold.
         * Covariance matrices must always be positive semidefinite, otherwise we can assume that
         * there are numerical instabilities in Kalman filter.
         *
         * @param m matrix to be evaluated.
         * @throws threshold threshold to determine whether matrix is positive semidefinite. This
         * is a value close to zero due to machine precision inaccuracies.
         * @return true if matrix is positive semidefinite, false otherwise.
         */
        private fun isPositiveSemidefinite(
            m: Matrix,
            threshold: Double = SYMMETRIC_THRESHOLD
        ): Boolean {
            val rows = m.rows
            for (i in 0 until rows) {
                if (m.getElementAt(i, i) < 0.0) {
                    return false
                }
            }

            return Utils.isSymmetric(m, threshold)
        }

        /**
         * Copies quaternion values to provided vector matrix.
         *
         * @param q quaternion to copy data from.
         * @param result matrix instance where data will be stored.
         * @throws IllegalArgumentException if provided result matrix is not 4x1.
         */
        @Throws(IllegalArgumentException::class)
        private fun copyQuaternionToColumnMatrix(q: Quaternion, result: Matrix) {
            result.setElementAtIndex(0, q.a)
            result.setElementAtIndex(1, q.b)
            result.setElementAtIndex(2, q.c)
            result.setElementAtIndex(3, q.d)
        }

        /**
         * Copies vector matrix data into quaternion.
         *
         * @param m matrix to copy data from.
         * @param result quaternion where data will be stored.
         * @throws IllegalArgumentException if provided matrix is not 4x1.
         */
        @Throws(IllegalArgumentException::class)
        private fun copyColumnMatrixToQuaternion(m: Matrix, result: Quaternion) {
            result.a = m.getElementAtIndex(0)
            result.b = m.getElementAtIndex(1)
            result.c = m.getElementAtIndex(2)
            result.d = m.getElementAtIndex(3)
        }
    }

    /**
     * Interface to notify when a new attitude has been processed.
     */
    fun interface OnProcessedListener {
        /**
         * Called when a new attitude is processed.
         *
         * @param processor processor that raised this event.
         * @param nedAttitude estimated attitude expressed in NED coordinates.
         * @param eulerAngles estimated Euler angles associated to estimated NED attitude.
         * @param nedQuaternionCovariance error covariance of quaternion attitude expressed in NED
         * coordinates.
         * @param eulerAnglesCovariance error covariance of Euler angles for NED attitude.
         */
        fun onProcessed(
            processor: KalmanRelativeAttitudeProcessor,
            nedAttitude: Quaternion,
            eulerAngles: DoubleArray?,
            nedQuaternionCovariance: Matrix?,
            eulerAnglesCovariance: Matrix?
        )
    }

    /**
     * Interface to notify when error covariances have been reset because numerical instabilities
     * have been detected in Kalman filter.
     */
    fun interface OnErrorCovarianceResetListener {
        /**
         * Called when error covariances are reset to zero.
         *
         * @param processor processor that raised this event.
         */
        fun onErrorCovarianceReset(processor: KalmanRelativeAttitudeProcessor)
    }
}