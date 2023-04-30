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
import com.irurueta.android.navigation.inertial.processors.attitude.KalmanRelativeAttitudeProcessor.Companion.DEFAULT_ACCELEROMETER_NOISE_STANDARD_DEVIATION
import com.irurueta.android.navigation.inertial.processors.attitude.KalmanRelativeAttitudeProcessor.Companion.DEFAULT_GYROSCOPE_NOISE_PSD
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegrator
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegratorType
import com.irurueta.units.AccelerationUnit
import com.irurueta.units.TimeConverter
import kotlin.math.abs
import kotlin.math.pow

/**
 * Estimates relative attitude using a Kalman filter.
 * This is based on Android native implementation:
 * https://android.googlesource.com/platform/frameworks/native/+/refs/heads/master/services/sensorservice/Fusion.cpp
 * And also based on:
 * https://www.mdpi.com/1424-8220/20/23/6731
 *
 * This implementation uses a Kalman filter where attitude is estimated based on direction vector
 * obtained from accelerometer measurements for filter corrections, and current system state based
 * on angular speed for attitude predictions.
 *
 * @property computeEulerAngles Indicates whether Euler angles must be estimated from NED attitude
 * or not.
 * @property computeCovariances Indicates whether attitude covariances must be processed or not.
 * @property computeQuaternionCovariance Indicates whether attitude quaternion covariance must be
 * processed or not. This is only taken into account if [computeCovariances] is true.
 * @property computeEulerAnglesCovariance Indicates whether propagated covariance for Euler angles
 * expressed in NED coordinates is estimated or not. This is only taken into account if
 * [computeCovariances] is true.
 * @property computeAccelerometerCovariance Indicates whether accelerometer covariance is estimated
 * or not. This is only taken into account if [computeCovariances] is true.
 * @property computeRotationAxisCovariance Indicates whether rotation axis covariance is estimated
 * or not. This is only taken into account if [computeCovariances] is true.
 * @property gyroscopeNoisePsd Variance of the gyroscope output per Hz (or variance at 1Hz). This is
 * equivalent to the gyroscope PSD (Power Spectral Density) that can be obtained during calibration
 * or with noise estimators. If not provided [DEFAULT_GYROSCOPE_NOISE_PSD] will be used.
 * @property accelerometerNoiseStandardDeviation Accelerometer noise standard deviation expressed in
 * m/s^2. If not provided [DEFAULT_ACCELEROMETER_NOISE_STANDARD_DEVIATION] will be used.
 * @property quaternionStepIntegratorType type of quaternion step integrator to be used for
 * gyroscope integration.
 * @property gravityNorm Gravity norm at current device location expressed in meters per squared
 * second (m/s^2).
 * @property processorListener listener to notify new attitudes and their uncertainties.
 * @property numericalInstabilitiesListener listener to notify when error covariances become
 * numerically unstable and have been reset.
 */
class KalmanRelativeAttitudeProcessor(
    val computeEulerAngles: Boolean = true,
    val computeCovariances: Boolean = true,
    val computeQuaternionCovariance: Boolean = true,
    val computeEulerAnglesCovariance: Boolean = true,
    val computeAccelerometerCovariance: Boolean = true,
    val computeRotationAxisCovariance: Boolean = true,
    val gyroscopeNoisePsd: Double = DEFAULT_GYROSCOPE_NOISE_PSD,
    val accelerometerNoiseStandardDeviation: Double = DEFAULT_ACCELEROMETER_NOISE_STANDARD_DEVIATION,
    val quaternionStepIntegratorType: QuaternionStepIntegratorType =
        QuaternionStepIntegrator.DEFAULT_TYPE,
    val gravityNorm: Double = EARTH_GRAVITY,
    val symmetricThreshold: Double = SYMMETRIC_THRESHOLD,
    var processorListener: OnProcessedListener? = null,
    var numericalInstabilitiesListener: OnNumericalInstabilitiesDetectedListener? = null
) {
    /**
     * Estimated attitude expressed in NED coordinates.
     */
    val nedAttitude = Quaternion()

    /**
     * Contains estimated accelerometer state as measurements expressed in NED coordinates.
     */
    val accelerationState = AccelerationTriad()

    /**
     * Contains estimated rotation axis state.
     */
    val rotationAxisState = DoubleArray(COMPONENTS)

    /**
     * Gyroscope interval between measurements expressed in seconds (s).
     */
    var gyroscopeIntervalSeconds: Double = 0.0
        private set

    /**
     * Euler angles associated to estimated NED attitude.
     * Array contains roll, pitch and yaw angles expressed in radians (rad) and following order
     * indicated here.
     * This is only available if [computeCovariances] and [computeEulerAngles] are true.
     */
    val eulerAngles: DoubleArray?
        get() = if (computeCovariances && computeEulerAngles) {
            internalEulerAngles
        } else {
            null
        }

    /**
     * Error covariance of estimated quaternion attitude expressed in NED coordinates.
     * This is only available if [computeCovariances] and [computeQuaternionCovariance] are true.
     */
    val nedQuaternionCovariance: Matrix?
        get() = if (computeCovariances && computeQuaternionCovariance) {
            internalNedQuaternionCovariance
        } else {
            null
        }

    /**
     * Error covariance of estimated Euler angles for NED attitude.
     * This is only available if [computeCovariances] and [computeEulerAnglesCovariance] are true.
     */
    val eulerAnglesCovariance: Matrix?
        get() = if (computeCovariances && computeEulerAnglesCovariance) {
            internalEulerAnglesCovariance
        } else {
            null
        }

    /**
     * Error covariance of estimated accelerometer state.
     * This is only available if [computeCovariances] and [computeAccelerometerCovariance] are true.
     */
    val accelerometerCovariance: Matrix?
        get() = if (computeCovariances && computeAccelerometerCovariance) {
            internalAccelerometerCovariance
        } else {
            null
        }

    /**
     * Error covariance of estimated rotation axis state.
     * This is only available if [computeCovariances] and [computeRotationAxisCovariance] are true.
     */
    val rotationAxisCovariance: Matrix?
        get() = if (computeCovariances && computeRotationAxisCovariance) {
            internalRotationAxisCovariance
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
     * Contains [deltaQ] expressed in quaternion matrix form to be used for matrix multiplication.
     * This is reused for performance reasons.
     */
    private val deltaQMatrix = Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS)

    /**
     * Sub-matrix for measurement matrix relating acceleration respect to Earth and respect to body.
     * This is reused for performance reasons.
     */
    private val h11 = Matrix(COMPONENTS, COMPONENTS)

    /**
     * Sub-matrix relating quaternion respect to Earth and acceleration respect to body.
     * This is reused for performance reasons.
     */
    private val h12 = Matrix(COMPONENTS, Quaternion.N_PARAMS)

    /**
     * Sub-matrix relating body rotation axis and body angular speed.
     * This is reused for performance reasons.
     */
    private val h33 = Matrix(COMPONENTS, COMPONENTS)

    /**
     * Kalman filter using normalized accelerometer vector as measurements.
     * This Kalman filter estimates attitude based on DOWN unitary
     * direction vector obtained from accelerometer measurements for filter corrections, and current
     * system state based on angular speed for attitude predictions.
     * With such predictions the filter can average accelerometer measurements to find
     * an accurate attitude estimation.
     */
    private val kalmanFilter =
        KalmanFilter(KALMAN_FILTER_DYNAMIC_PARAMETERS, KALMAN_FILTER_MEASUREMENT_PARAMETERS)

    /**
     * Integrates angular speed measurements to compute predicted quaternion variation.
     */
    private val quaternionStepIntegrator =
        QuaternionStepIntegrator.create(quaternionStepIntegratorType)

    /**
     * Contains Kalman filter measurements data.
     * This is reused for performance reasons.
     */
    private val m = Matrix(KALMAN_FILTER_MEASUREMENT_PARAMETERS, 1)

    /**
     * Contains gyroscope measurements covariance in matrix form.
     * This is reused for performance reasons when process noise covariance must be updated.
     */
    private val gyroCov = Matrix(COMPONENTS, COMPONENTS)

    /**
     * Contains quaternion in matrix form to compute quaternion product with angular speed.
     * This is reused for performance reasons when quaternion process noise covariance needs to be
     * updated.
     */
    private val prodQ = Matrix(Quaternion.N_PARAMS, COMPONENTS)

    /**
     * Transposed of prodQ.
     * This is reused for performance reasons.
     */
    private val transposedProdQ = Matrix(COMPONENTS, Quaternion.N_PARAMS)

    /**
     * Sub-matrix containing process noise covariance associated to quaternion.
     * This is reused for performance reasons when process noise covariance needs to be updated.
     */
    private val qq = Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS)

    /**
     * Error covariance of quaternion attitude expressed in NED coordinates.
     * This is used internally and reused for performance reasons.
     */
    private val internalNedQuaternionCovariance = Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS)

    /**
     * Error covariance of estimated Euler angles for NED attitude.
     * This is used internally and reused for performance reasons.
     */
    private val internalEulerAnglesCovariance = Matrix(Quaternion.N_ANGLES, Quaternion.N_ANGLES)

    /**
     * Error covariance of Kalman filter state accelerometer measurement values.
     * This is used internally and reused for performance reasons.
     */
    private val internalAccelerometerCovariance = Matrix(COMPONENTS, COMPONENTS)

    /**
     * Error covariance of Kalman filter state rotation axis.
     * This is used internally and reused for performance reasons.
     */
    private val internalRotationAxisCovariance = Matrix(COMPONENTS, COMPONENTS)

    /**
     * Euler angles of estimated NED attitude.
     * This is used internally and reused for performance reasons.
     */
    private val internalEulerAngles = DoubleArray(Quaternion.N_ANGLES)

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
    private val transposedEulerJacobian = Matrix(Quaternion.N_PARAMS, Quaternion.N_ANGLES)

    /**
     * Temporary value of Euler error covariance.
     */
    private val tmpEulerCov = Matrix(Quaternion.N_ANGLES, Quaternion.N_ANGLES)

    /**
     * Processed provided synced accelerometer and gyroscope measurement to obtain a relative
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
        processMeasurement(gyroscopeMeasurement, angularSpeedTriad)
        val result = if (isFirst) {
            false
        } else {
            processMeasurement(accelerometerMeasurement, accelerationTriad)

            if (!initialized) {
                estimateInitialAttitude()

                initKalmanFilter()

                initialized = true
            } else {
                // set transition matrix for current state
                updateTransitionMatrix()
                updateMeasurementMatrix()
                updateProcessNoiseCovariance()

                // predict using current transition matrix
                kalmanFilter.predict()

                // update Kalman filter measurement
                updateMeasurement()

                val statePost = kalmanFilter.correct(m)

                // copy state to quaternion
                copyFilterStateToQuaternion(statePost, q)

                // copy internal state to public properties
                nedAttitude.fromQuaternion(q)
                copyFilterStateToAcceleration(statePost, accelerationState)
                copyFilterStateToRotationAxis(statePost, rotationAxisState)

                processCovariance()

                processorListener?.onProcessed(
                    this,
                    nedAttitude,
                    eulerAngles,
                    nedQuaternionCovariance,
                    eulerAnglesCovariance,
                    accelerometerCovariance,
                    rotationAxisCovariance
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
        accelerationTriad.getValuesAsArray(down)
        normalize(down)
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

        // update state with initial attitude
        val statePre = kalmanFilter.statePre
        copyQuaternionToFilterState(q, statePre)

        val statePost = kalmanFilter.statePost
        copyQuaternionToFilterState(q, statePost)

        // initialize error covariance a priori and posteriori to zero
        initErrorCovariances()
    }

    /**
     * Updates measurement matrix with accelerometer and gyroscope measurements to be used during
     * Kalman filter correction phase.
     */
    private fun updateMeasurement() {
        m.setElementAtIndex(0, accelerationTriad.valueX)
        m.setElementAtIndex(1, accelerationTriad.valueY)
        m.setElementAtIndex(2, accelerationTriad.valueZ)
        m.setElementAtIndex(3, angularSpeedTriad.valueX)
        m.setElementAtIndex(4, angularSpeedTriad.valueY)
        m.setElementAtIndex(5, angularSpeedTriad.valueZ)
    }

    /**
     * Updates process noise covariance on a block by block basis taking into account time interval
     * between measurements, accelerometer standard deviation, current estimated quaternion state.
     */
    private fun updateProcessNoiseCovariance() {
        val processNoiseCov = kalmanFilter.processNoiseCov

        // process noise covariance is a block diagonal matrix following expression:
        // Q = diag(Qa, Qq, Qr)

        // where:
        // - Qa corresponds to covariance of acceleration respect Earth. 3x3 matrix
        // - Qq corresponds to covariance of quaternion respect Earth. 4x4 matrix.
        // - Qr corresponds to covariance of unitary body rotation axis. 3x3 matrix.

        // set Qa
        updateAccelerationProcessNoiseCovariance(processNoiseCov)

        val sqrHalfInterval = (gyroscopeIntervalSeconds / 2.0).pow(2.0)

        // set Qq
        val gyroVariance = gyroscopeNoisePsd / gyroscopeIntervalSeconds
        updateQuaternionProcessNoiseCovariance(gyroVariance, sqrHalfInterval, processNoiseCov)

        // setQr
        updateRotationVectorProcessNoiseCovariance(gyroVariance, sqrHalfInterval, processNoiseCov)
    }

    /**
     * Updates sub-matrix block of process noise covariance associated to acceleration.
     *
     * @param processNoiseCov process noise covariance where updated sub-matrix will be stored.
     */
    private fun updateAccelerationProcessNoiseCovariance(processNoiseCov: Matrix) {
        val accelerometerVariance =
            accelerometerNoiseStandardDeviation * accelerometerNoiseStandardDeviation
        for (i in 0 until COMPONENTS) {
            processNoiseCov.setElementAt(i, i, accelerometerVariance)
        }
    }

    /**
     * Updates sub-matrix block of process noise covariance associated to attitude quaternion.
     *
     * @param gyroVariance variance of gyroscope measurements expressed as (rad/s)^2.
     * @param sqrHalfInterval contains squared half interval that is being reused multiple times.
     * @param processNoiseCov process noise covariance where updated sub-matrix will be stored.
     */
    private fun updateQuaternionProcessNoiseCovariance(
        gyroVariance: Double,
        sqrHalfInterval: Double,
        processNoiseCov: Matrix
    ) {
        // Qq = T^2/4 * prodQ * gyroCov * prodQ^T

        for (i in 0 until COMPONENTS) {
            gyroCov.setElementAt(i, i, gyroVariance)
        }

        val q0 = q.a
        val q1 = q.b
        val q2 = q.c
        val q3 = q.d

        prodQ.setElementAtIndex(0, -q1)
        prodQ.setElementAtIndex(1, q0)
        prodQ.setElementAtIndex(2, q3)
        prodQ.setElementAtIndex(3, -q2)

        prodQ.setElementAtIndex(4, -q2)
        prodQ.setElementAtIndex(5, -q3)
        prodQ.setElementAtIndex(6, q0)
        prodQ.setElementAtIndex(7, q1)

        prodQ.setElementAtIndex(8, -q3)
        prodQ.setElementAtIndex(9, q2)
        prodQ.setElementAtIndex(10, -q1)
        prodQ.setElementAtIndex(11, q0)

        prodQ.transpose(transposedProdQ)

        prodQ.multiply(gyroCov, qq)
        qq.multiply(transposedProdQ)
        qq.multiplyByScalar(sqrHalfInterval)

        processNoiseCov.setSubmatrix(
            COMPONENTS,
            COMPONENTS,
            END_QQ,
            END_QQ,
            qq
        )
    }

    /**
     * Updates sub-matrix block of process noise covariance associated to rotation axis vector.
     *
     * @param gyroVariance variance of gyroscope measurements expressed as (rad/s)^2.
     * @param sqrHalfInterval contains squared half interval that is being reused multiple times.
     * @param processNoiseCov process noise covariance where updated sub-matrix will be stored.
     */
    private fun updateRotationVectorProcessNoiseCovariance(
        gyroVariance: Double,
        sqrHalfInterval: Double,
        processNoiseCov: Matrix
    ) {
        // set Qr as the identity multiplied with variance of rotation vector.
        // we know from [updateMeasurementMatrix] that:
        // w_b = 2 / deltaT * v_b
        // Consequently the variation of rotation vector can be expressed in terms of angular
        // speed as:
        // v_b = deltaT / 2 * w_b
        // and consequently, variance of rotation vector is:
        // var(v_b) = (deltaT / 2)^2 * var(w_b)

        val varianceRotationVector = sqrHalfInterval * gyroVariance
        for (i in ROTATION_AXIS_OFFSET until KALMAN_FILTER_DYNAMIC_PARAMETERS) {
            processNoiseCov.setElementAt(i, i, varianceRotationVector)
        }
    }

    /**
     * Initializes Kalman filter predicted and corrected error covariances to zero.
     */
    private fun initErrorCovariances() {
        kalmanFilter.errorCovPre.initialize(0.0)
        kalmanFilter.errorCovPost.initialize(0.0)
    }

    /**
     * Updates filter transition matrix.
     * Transition matrix is made of three submatrices.
     * Acceleration 3x3 submatrix will always be the identity assuming that no external forces are
     * applies and consequently acceleration remains constant during a single interval. Since
     * Kalman Filter constructor already initializes the transition matrix to the identity, there is
     * no need to update this block.
     * The second 4x4 block corresponds to the quaternion contained within Kalman filter state.
     * This matrix should express expected quaternion variation during a time interval.
     * There are several ways to integrate quaternions with angular speeds during a time interval.
     * The simplest one is Euler integration, but the least accurate.
     * In order to be able to obtain a more accurate estimation, a QuaternionStepIntegrator is used
     * to obtain expected quaternion after integrating current one with angular speed and time
     * interval, so that quaternion variation is estimated and expressed in matrix form.
     * The third 3x3 block corresponds to the rotation vector. Since the angular velocity can be
     * assumed to be constant on a single time interval, it is assumed to be the identity, and there
     * is no ned to update this block.
     * @see com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegrator to get
     * more information on quaternion integration.
     */
    private fun updateTransitionMatrix() {
        // accurately integrates current attitude with current and previous angular speeds
        // to estimate variation (Delta) of quaternion attitude.
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

        // copy quaternion variation deltaQ into corresponding 4x4 submatrix within transition
        // matrix
        deltaQ.quaternionMatrix(deltaQMatrix)
        val transitionMatrix = kalmanFilter.transitionMatrix
        transitionMatrix.setSubmatrix(3, 3, 6, 6, deltaQMatrix)
    }

    /**
     * Updates Kalman filter measurement matrix which relates filter state and measurement matrices.
     */
    private fun updateMeasurementMatrix() {
        // Kalman filter state is made of:
        // - acceleration respect to Earth (a_e)
        // - quaternion attitude respect to Earth (q_e)
        // - rotation vector respect to body (v_b)

        // Kalman filter measurements contain:
        // - acceleration (specific force) respect to body. (a_b)
        // - angular speed respect to body. (w_b)

        // Consequently, measurement matrix m can be obtained from Kalman filter state x through
        // measurement matrix H as:
        // m = H * x

        // where H is a 6x10 matrix made of the following blocks:
        // H = [H11  H12  0  ]
        //     [0    0    H33]

        // H11 is a 3x3 matrix relating acceleration respect to Earth and respect to body
        // H12 is a 3x4 matrix relating quaternion respect to Earth and acceleration respect to body
        // So that:
        // a_b = H11 * a_e + H12 * q_e

        // where q_e = [q0, q1, q2, q3]

        // H11 = [1 - 2 * (q2^2 + q3^2)     2 * (q0 * q3 + q1 * q2)     2 * (q1 * q3 - q0 * q2)]
        //       [2 * (q1 * q2 - q0 * q3)   1 - 2 * (q1^2 + q3^2)       2 * (q0 * q1 + q2 * q3)]
        //       [2 * (q0 * q2 + q1 * q3)   2 * (q2 * q3 - q0 * q1)     1 - 2 * (q1^2 + q2^2)  ]

        // H12 = -g * [-q2      q3      -q0     q1]
        //            [ q1      q0       q3     q2]
        //            [ q0     -q1      -q2     q3]

        // where g is gravity norm


        // Matrix H33 is a 3x3 matrix performs a prediction of gyroscope measurements both in body
        // coordinates (consequently quaternion is not needed).
        // w_b = H33 * v_b

        // Since a unit quaternion (representing a rotation) is made of the following components):
        // deltaQ = [cos(omega/2)  sin(omega/2)*vx  sin(omega/2)*vy  sin(omega/2)*vz)

        // where rotation angle omega can be assumed to be:
        // omega = w * deltaT
        // where w is angular speed (assumed constant in a single interval) and deltaT is the time
        // interval expressed in seconds.

        // For small enough time intervals (deltaT -> 0), the quaternion expression can be
        // approximated as:
        // deltaQ = [1  omega/2*vx  omega/2*vy  omega/2*vz] = [1  w*deltaT/2*vx  w*deltaT/2*vy w*deltaT/2*vz]

        // consequently the variation of rotation axis v = [vx vy vz] is related to angular speed
        // as follows:
        // v' = w*deltaT/2*v
        // where v' is the new rotation axis, and v is the old one. And both must have unitary norm
        // so that quaternion remains unitary: |v'| = 1, |v| = 1.
        // Consequently, variation in norm is:
        // |v'| = |w*deltaT/2|*|v|
        // |w * deltaT / 2 | = 1
        // And finally we obtain the approximate relation betweeb angular speed and rotation axis
        // as:
        // w_b = 2 / deltaT * v_b

        // And consequently matrix H33 becomes:
        // H33 = 2 / deltaT * [1  0  0]
        //                    [0  1  0]
        //                    [0  0  1]

        val q0 = q.a
        val q1 = q.b
        val q2 = q.c
        val q3 = q.d

        val sqrQ1 = q1 * q1
        val sqrQ2 = q2 * q2
        val sqrQ3 = q3 * q3

        val q1q2 = q1 * q2
        val q0q3 = q0 * q3
        val q0q2 = q0 * q2
        val q1q3 = q1 * q3
        val q2q3 = q2 * q3
        val q0q1 = q0 * q1

        h11.setElementAtIndex(0, 1.0 - 2.0 * (sqrQ2 + sqrQ3))
        h11.setElementAtIndex(1, 2.0 * (q1q2 - q0q3))
        h11.setElementAtIndex(2, 2.0 * (q0q2 + q1q3))

        h11.setElementAtIndex(3, 2.0 * (q0q3 + q1q2))
        h11.setElementAtIndex(4, 1.0 - 2.0 * (sqrQ1 + sqrQ3))
        h11.setElementAtIndex(5, 2.0 * (q2q3 - q0q1))

        h11.setElementAtIndex(6, 2.0 * (q1q3 - q0q2))
        h11.setElementAtIndex(7, 2.0 * (q0q1 + q2q3))
        h11.setElementAtIndex(8, 1.0 - 2.0 * (sqrQ1 + sqrQ2))


        h12.setElementAtIndex(0, gravityNorm * q2)
        h12.setElementAtIndex(1, -gravityNorm * q1)
        h12.setElementAtIndex(2, gravityNorm * q0)

        h12.setElementAtIndex(3, -gravityNorm * q3)
        h12.setElementAtIndex(4, -gravityNorm * q0)
        h12.setElementAtIndex(5, gravityNorm * q1)

        h12.setElementAtIndex(6, gravityNorm * q0)
        h12.setElementAtIndex(7, -gravityNorm * q3)
        h12.setElementAtIndex(8, gravityNorm * q2)

        h12.setElementAtIndex(9, -gravityNorm * q1)
        h12.setElementAtIndex(10, -gravityNorm * q2)
        h12.setElementAtIndex(11, -gravityNorm * q3)

        val value = 2.0 / gyroscopeIntervalSeconds
        for (i in 0 until COMPONENTS) {
            h33.setElementAt(i, i, value)
        }
    }

    /**
     * Propagates uncertainty of attitude covariance respect to Euler angles.
     *
     * @param covariance quaternion attitude covariance matrix.
     * @param result instance where propagated covariance will be stored representing covariance of
     * attitude Euler angles.
     */
    private fun propagateEulerAnglesUncertainty(covariance: Matrix, result: Matrix) {
        propagateUncertainty(
            covariance,
            eulerJacobian,
            tmpEulerCov,
            transposedEulerJacobian,
            result
        )
    }

    /**
     * Processes Kalman filter state error covariance to obtain attitude, acceleration and rotation
     * axis covariance.
     */
    private fun processCovariance() {
        if (computeCovariances || computeEulerAngles || computeEulerAnglesCovariance) {
            // process covariances
            val errorCov = kalmanFilter.errorCovPost

            // check covariance state
            if (!isPositiveSemidefinite(errorCov, symmetricThreshold)) {
                // when covariance is not positive semi-definite, we can assume that Kalman filter
                // has become numerically unstable, and we need to reset error covariance value
                initErrorCovariances()

                numericalInstabilitiesListener?.onNumericalInstabilityDetected(this)
            }

            // take only the part corresponding to ENU quaternion covariance
            errorCov.getSubmatrix(3, 3, 6, 6, internalNedQuaternionCovariance)

            if (computeEulerAngles) {
                // As defined in: https://en.wikipedia.org/wiki/Propagation_of_uncertainty
                // covariance propagation for a function converting quaternion to euler angles is:
                // Cov`= J * Cov * J^T
                if (computeEulerAnglesCovariance) {
                    nedAttitude.toEulerAngles(internalEulerAngles, eulerJacobian)
                    eulerJacobian.transpose(transposedEulerJacobian)

                    propagateEulerAnglesUncertainty(
                        internalNedQuaternionCovariance,
                        internalEulerAnglesCovariance
                    )
                } else {
                    nedAttitude.toEulerAngles(internalEulerAngles)
                }
            }

            if (computeAccelerometerCovariance) {
                errorCov.getSubmatrix(0, 0, 2, 2, internalAccelerometerCovariance)
            }

            if (computeRotationAxisCovariance) {
                errorCov.getSubmatrix(7, 7, 9, 9, internalRotationAxisCovariance)
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
        const val DEFAULT_GYROSCOPE_NOISE_PSD = 1e-7

        /**
         * Default standard deviation for accelerometer noise expressed in m/s^2.
         */
        const val DEFAULT_ACCELEROMETER_NOISE_STANDARD_DEVIATION = 0.015

        /**
         * Average gravity at Earth's sea level expressed in m/s^2.
         */
        const val EARTH_GRAVITY = SensorManager.GRAVITY_EARTH.toDouble()

        /**
         * Amount of components of measurements and arrays.
         */
        private const val COMPONENTS = AccelerationTriad.COMPONENTS

        /**
         * Threshold to determine whether covariance matrices are positive definite or not.
         */
        private const val SYMMETRIC_THRESHOLD = 1e-10

        /**
         * Kalman filter's number of dynamic parameters.
         */
        private const val KALMAN_FILTER_DYNAMIC_PARAMETERS = 10

        /**
         * Kalman filter's number of measurement parameters.
         */
        private const val KALMAN_FILTER_MEASUREMENT_PARAMETERS = 6

        /**
         * Offset of rotation axis vector within Kalman filter state.
         */
        private const val ROTATION_AXIS_OFFSET = 7

        /**
         * Position where internal [qq] matrix ends within process noise covariance matrix.
         */
        private const val END_QQ = 6

        /**
         * Processes an accelerometer measurement, taking into account their biases (if available).
         * Measurement will contain accelerometer data expressed in NED coordinates.
         *
         * @param measurement measurement to be processed.
         * @param result triad where measurement data will be stored in NED coordinates.
         * @throws IllegalArgumentException if provided result array does not have length 3.
         */
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
         * Computes Frobenious norm for provided array.
         *
         * @param data array to be evaluated.
         * @return estimated Frobenious norm.
         */
        private fun norm(data: DoubleArray): Double {
            return Utils.normF(data)
        }

        /**
         * Estimates an orthogonal vector to provided input one.
         *
         * @param input vector to obtain orthogonal one from.
         * @param output instance where estimated orthogonal vector will be stored.
         * @throws IllegalArgumentException if provided array does not have length 3.
         */
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
            val norm = norm(array)
            ArrayUtils.multiplyByScalar(array, 1.0 / norm, array)
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
            threshold: Double
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
         * Copies quaternion to provided Kalman filter state matrix.
         *
         * @param q quaternion to copy data from.
         * @param result matrix instance where data will be stored.
         */
        private fun copyQuaternionToFilterState(q: Quaternion, result: Matrix) {
            result.setElementAtIndex(3, q.a)
            result.setElementAtIndex(4, q.b)
            result.setElementAtIndex(5, q.c)
            result.setElementAtIndex(6, q.d)
        }

        /**
         * Copies Kalman filter state data associated to acceleration into provided acceleration
         * triad.
         *
         * @param state Kalman filter state matrix.
         * @param result acceleration triad where data will be stored.
         */
        private fun copyFilterStateToAcceleration(state: Matrix, result: AccelerationTriad) {
            result.setValueCoordinatesAndUnit(
                state.getElementAtIndex(0),
                state.getElementAtIndex(1),
                state.getElementAtIndex(2),
                AccelerationUnit.METERS_PER_SQUARED_SECOND
            )
        }

        /**
         * Copies Kalman filter state data associated to quaternion attitude into provided
         * quaternion.
         *
         * @param state Kalman filter state matrix.
         * @param result quaternion where data will be stored.
         */
        private fun copyFilterStateToQuaternion(state: Matrix, result: Quaternion) {
            result.a = state.getElementAtIndex(3)
            result.b = state.getElementAtIndex(4)
            result.c = state.getElementAtIndex(5)
            result.d = state.getElementAtIndex(6)
        }

        /**
         * Copies Kalman filter state data associated to unitary rotation axis into provided array.
         *
         * @param state Kalman filter state matrix.
         * @param result array where data will be stored.
         */
        private fun copyFilterStateToRotationAxis(state: Matrix, result: DoubleArray) {
            var j = ROTATION_AXIS_OFFSET
            for (i in 0 until COMPONENTS) {
                result[i] = state.getElementAtIndex(j++)
            }
        }

        /**
         * Propagates effects of uncertainty to provided covariance matrix using provided jacobian,
         * and transposed jacobian.
         * Uncertainty is propagated as: Cov' = J * Cov * J^T
         *
         * @param covariance input covariance whose uncertainty is being propagated
         * @param jacobian jacobian to propagate uncertainty of uncovariance.
         */
        private fun propagateUncertainty(
            covariance: Matrix,
            jacobian: Matrix,
            tmp: Matrix,
            transposedJacobian: Matrix,
            result: Matrix
        ) {
            // As defined in: https://en.wikipedia.org/wiki/Propagation_of_uncertainty
            // covariance propagation for a function converting quaternion to euler angles is:
            // Cov`= J * Cov * J^T

            jacobian.multiply(covariance, tmp)
            tmp.multiply(transposedJacobian, result)
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
         * @param attitude estimated attitude expressed in NED coordinates.
         * @param eulerAngles estimated Euler angles associated to estimated NED attitude.
         * @param quaternionCovariance error covariance of quaternion attitude expressed in NED
         * coordinates.
         * @param eulerAnglesCovariance error covariance of Euler angles for NED attitude. Variance
         * is expressed in radians^2
         * @param accelerometerCovariance error covariance of estimated accelerometer state
         * expressed in NED coordinates. Variance is expressed in (m/s^2)^2
         * @param rotationAxisCovariance error covariance of estimated unitary rotation axis
         * expressed in NED coordinates. Variance is unit-less.
         */
        fun onProcessed(
            processor: KalmanRelativeAttitudeProcessor,
            attitude: Quaternion,
            eulerAngles: DoubleArray?,
            quaternionCovariance: Matrix?,
            eulerAnglesCovariance: Matrix?,
            accelerometerCovariance: Matrix?,
            rotationAxisCovariance: Matrix?
        )
    }

    /**
     * Interface to notify when error covariances are reset because numerical instabilities have
     * been detected in Kalman filter.
     */
    fun interface OnNumericalInstabilitiesDetectedListener {

        /**
         * Called when numerical instability is detected.
         *
         * @param processor processor that raised this event.
         */
        fun onNumericalInstabilityDetected(processor: KalmanRelativeAttitudeProcessor)
    }
}