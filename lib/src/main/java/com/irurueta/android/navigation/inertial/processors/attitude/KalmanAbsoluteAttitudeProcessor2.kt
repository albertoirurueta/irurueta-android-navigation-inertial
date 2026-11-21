/*
 * Copyright (C) 2026 Alberto Irurueta Carro (alberto@irurueta.com)
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
import android.location.Location
import com.irurueta.algebra.Matrix
import com.irurueta.algebra.Utils
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorMeasurement
import com.irurueta.android.navigation.inertial.processors.filters.AveragingFilter
import com.irurueta.android.navigation.inertial.processors.filters.LowPassAveragingFilter
import com.irurueta.geometry.AxisRotation3D
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad
import com.irurueta.navigation.inertial.calibration.Triad
import com.irurueta.numerical.signal.processing.KalmanFilter
import com.irurueta.units.Acceleration
import com.irurueta.units.AccelerationUnit
import com.irurueta.units.TimeConverter
import kotlin.math.acos
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Estimates absolute attitude using a Kalman filter.
 * This is based on:
 * https://www.mdpi.com/1424-8220/20/23/6731
 *
 * This implementation uses a Kalman filter where attitude is estimated based on accelerometer,
 * magnetometer and gyroscope measurements.
 *
 * @property averagingFilter averaging or low-pass-band filter to be used for accelerometer
 * measurements to estimate gravity.
 * @property location current device location. This is used to estimate expected gravity norm.
 * @property adjustGravityNorm when enabled, estimated gravity norm is adjusted to expected gravity
 * when location is provided (otherwise Earth's default gravity at sea-level is used).
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
 * @property freeFallThreshold Threshold to consider that device is in free fall. When device is in
 * free fall, accelerometer measurements are considered unreliable and are ignored (only gyroscope
 * predictions are made).
 * @property symmetricThreshold Threshold to determine whether covariance matrices are positive
 * definite or not.
 * @property processorListener listener to notify new attitudes and their uncertainties.
 * @property numericalInstabilitiesListener listener to notify when error covariances become
 * numerically unstable and have been reset.
 */
class KalmanAbsoluteAttitudeProcessor2(
    val averagingFilter: AveragingFilter<AccelerationUnit, Acceleration, AccelerationTriad> = LowPassAveragingFilter(),
    location: Location? = null,
    adjustGravityNorm: Boolean = true,
    val computeEulerAngles: Boolean = true,
    val computeCovariances: Boolean = true,
    val computeQuaternionCovariance: Boolean = true,
    val computeEulerAnglesCovariance: Boolean = true,
    val computeAccelerometerCovariance: Boolean = true,
    val computeRotationAxisCovariance: Boolean = true,
    val gyroscopeNoisePsd: Double = DEFAULT_GYROSCOPE_NOISE_PSD,
    val accelerometerNoiseStandardDeviation: Double = DEFAULT_ACCELEROMETER_NOISE_STANDARD_DEVIATION,
    val magnetometerNoiseStandardDeviation: Double = DEFAULT_MAGNETOMETER_NOISE_STANDARD_DEVIATION,
    val freeFallThreshold: Double = DEFAULT_FREE_FALL_THRESHOLD,
    val symmetricThreshold: Double = DEFAULT_SYMMETRIC_THRESHOLD,
    var processorListener: OnProcessedListener? = null,
    var numericalInstabilitiesListener: OnNumericalInstabilitiesDetectedListener? = null
) {
    /**
     * Processes accelerometer measurements to obtain an estimation of gravity.
     */
    private val gravityProcessor = AccelerometerGravityProcessor(averagingFilter)

    /**
     * Kalman filter using accelerometer, magnetometer and gyroscope measurements.
     */
    private val kalmanFilter = KalmanFilter(
        KALMAN_FILTER_DYNAMIC_PARAMETERS,
        KALMAN_FILTER_MEASUREMENT_PARAMETERS
    )

    /**
     * Timestamp of previous sample expressed in nanoseconds.
     */
    private var previousTimestamp: Long = -1L

    /**
     * Indicates whether processor has been initialized or not.
     * A processor is considered to be initialized if enough measurements are available
     * and time interval can be determined.
     */
    private var initialized = false

    /**
     * Accelerometer measurement expressed in NED body coordinates. This instance is reused for
     * performance reasons.
     */
    private val nedAccelerometerMeasurement = AccelerometerSensorMeasurement()

    /**
     * Acceleration triad expressed in NED coordinates. This instance is reused for performance
     * reasons.
     */
    private val accelerationTriad = AccelerationTriad()

    /**
     * Gyroscope measurement expressed in NED body coordinates. This instance is reused for
     * performance reasons.
     */
    private val nedGyrometerMeasurement = GyroscopeSensorMeasurement()

    /**
     * Angular speed triad expressed in NED coordinates. This instance is reused for performance
     * reasons.
     */
    private val angularSpeedTriad = AngularSpeedTriad()

    /**
     * Magnetometer measurement expressed in NED body coordinates. This instance is reused for
     * performance reasons.
     */
    private val nedMagnetometerMeasurement = MagnetometerSensorMeasurement()

    /**
     * Magnetic flux density triad expressed in NED coordinates. This instance is reused for
     * performance reasons.
     */
    private val magneticFluxDensityTriad = MagneticFluxDensityTriad()

    /**
     * Contains Kalman filter measurements data.
     * This is reused for performance reasons.
     */
    private val z = Matrix(KALMAN_FILTER_MEASUREMENT_PARAMETERS, 1)

    /**
     * Contains rotation axis or body orientation change.
     * This is reused for performance reasons.
     */
    private val axis = DoubleArray(AxisRotation3D.AXIS_PARAMS)

    /**
     * Contains matrix to estimate transition matrix corresponding to second block.
     * This is reused for performance reasons.
     */
    private val w0I44 = Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS)

    /**
     * Contains estimated orientation change expressed in matrix form, so that quaternion product
     * can be represented as a matrix product.
     * This is reused for performance reasons.
     *
     * @see Quaternion.quaternionMatrix
     */
    private val omega = Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS)

    /**
     * Contains second block of estimated transition matrix.
     * This is reused for performance reasons.
     */
    private val phi22 = Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS)

    /**
     * H11 sub-matrix of Kalman filter measurement matrix.
     */
    private val h11 = Matrix(Triad.COMPONENTS, Triad.COMPONENTS)

    /**
     * H12 sub-matrix of Kalman filter measurement matrix.
     */
    private val h12 = Matrix(Triad.COMPONENTS, Quaternion.N_PARAMS)

    /**
     * H22 sub-matrix of Kalman filter measurement matrix.
     */
    private val h22 = Matrix(Triad.COMPONENTS, Quaternion.N_PARAMS)

    /**
     * H33 sub-matrix of Kalman filter measurement matrix.
     */
    private val h33 = Matrix(Triad.COMPONENTS, Triad.COMPONENTS)

    /**
     * Accelerometer related sub-matrix of process noise covariance matrix.
     */
    private val qA = Matrix(Triad.COMPONENTS, Triad.COMPONENTS)

    /**
     * Quaternion related sub-matrix of process noise covariance matrix.
     */
    private val qQ = Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS)

    /**
     * Rotation vector related sub-matrix of process noise covariance matrix.
     */
    private val qR = Matrix(Triad.COMPONENTS, Triad.COMPONENTS)

    /**
     * Contains gyroscope measurements covariance in 3x3 matrix form.
     * This is reused for performance reasons when process noise covariance must be updated.
     */
    private val gyroCov = Matrix(Triad.COMPONENTS, Triad.COMPONENTS)

    /**
     * Contains quaternion in 4x3 matrix form to compute quaternion product with angular speed.
     * This is reused for performance reasons when quaternion process noise covariance needs to be
     * updated.
     */
    private val prodQ = Matrix(Quaternion.N_PARAMS, Triad.COMPONENTS)

    /**
     * 3x4 Transposed of prodQ.
     * This is reused for performance reasons.
     */
    private val transposedProdQ = Matrix(Triad.COMPONENTS, Quaternion.N_PARAMS)

    /**
     * Temporary matrix to compute [qQ].
     */
    private val tmpQ = Matrix(Quaternion.N_PARAMS, Triad.COMPONENTS)

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
    private val transposedEulerJacobian = Matrix(
        Quaternion.N_PARAMS,
        Quaternion.N_ANGLES
    )

    /**
     * Error covariance of quaternion attitude expressed in NED coordinates.
     * This is used internally and reused for performance reasons.
     */
    private val internalNedQuaternionCovariance = Matrix(
        Quaternion.N_PARAMS,
        Quaternion.N_PARAMS
    )

    /**
     * Error covariance of estimated Euler angles for NED attitude.
     * This is used internally and reused for performance reasons.
     */
    private val internalEulerAnglesCovariance = Matrix(
        Quaternion.N_ANGLES,
        Quaternion.N_ANGLES
    )

    /**
     * Error covariance of Kalman filter state accelerometer measurement values.
     * This is used internally and reused for performance reasons.
     */
    private val internalAccelerometerCovariance = Matrix(
        Triad.COMPONENTS, Triad.COMPONENTS
    )

    /**
     * Error covariance of Kalman filter state rotation axis.
     * This is used internally and reused for performance reasons.
     */
    private val internalRotationAxisCovariance = Matrix(
        Triad.COMPONENTS, Triad.COMPONENTS
    )

    /**
     * Temporary value of Euler error covariance.
     */
    private val tmpEulerCov = Matrix(Quaternion.N_ANGLES, Quaternion.N_ANGLES)

    /**
     * Gets or sets device location
     */
    var location: Location? = null
        set(value) {
            field = value
            gravityProcessor.location = value
        }

    /**
     * Indicates whether gravity norm must be adjusted to either Earth
     * standard norm, or norm at provided location. If no location is provided, this should only be
     * enabled when device is close to sea level.
     */
    var adjustGravityNorm: Boolean
        get() = gravityProcessor.adjustGravityNorm
        set(value) {
            gravityProcessor.adjustGravityNorm = value
        }

    /**
     * Contains estimated acceleration expressed in Earth coordinates.
     */
    val estimatedAcceleration = AccelerationTriad()

    /**
     * Estimated body orientation respect to Earth.
     */
    val estimatedAttitude = Quaternion()

    /**
     * Estimated Earth orientation respect to body.
     */
    val estimatedBodyAttitude = Quaternion()

    /**
     * Estimated orientation change in body coordinates.
     */
    val estimatedOrientationChange = Quaternion()

    /**
     * Current estimated time interval between measurements expressed in seconds (s).
     */
    var estimatedTimeIntervalSeconds: Double = 0.0
        private set

    /**
     * Current estimated gravity norm expressed in m/s^2.
     */
    var estimatedGravityNorm: Double = 0.0
        private set

    /**
     * Euler angles associated to estimated attitude.
     * Array contains roll, pitch and yaw angles expressed in radians (rad) and following order
     * indicated here.
     * This is only available if [computeCovariances] and [computeEulerAngles] are true.
     */
    val estimatedEulerAngles: DoubleArray?
        get() = if (computeCovariances && computeEulerAngles) {
            internalEulerAngles
        } else {
            null
        }

    /**
     * Error covariance of estimated quaternion attitude expressed in NED coordinates.
     * This is only available if [computeCovariances] and [computeQuaternionCovariance] are true.
     */
    val estimatedAttitudeCovariance: Matrix?
        get() = if (computeCovariances && computeQuaternionCovariance) {
            internalNedQuaternionCovariance
        } else {
            null
        }

    /**
     * Error covariance of estimated Euler angles for NED attitude.
     * This is only available if [computeCovariances] and [computeEulerAnglesCovariance] are true.
     */
    val estimatedEulerAnglesCovariance: Matrix?
        get() = if (computeCovariances && computeEulerAnglesCovariance) {
            internalEulerAnglesCovariance
        } else {
            null
        }

    /**
     * Error covariance of estimated accelerometer state.
     * This is only available if [computeCovariances] and [computeAccelerometerCovariance] are true.
     */
    val estimatedAccelerometerCovariance: Matrix?
        get() = if (computeCovariances && computeAccelerometerCovariance) {
            internalAccelerometerCovariance
        } else {
            null
        }

    /**
     * Error covariance of estimated rotation axis state.
     * This is only available if [computeCovariances] and [computeRotationAxisCovariance] are true.
     */
    val estimatedRotationAxisCovariance: Matrix?
        get() = if (computeCovariances && computeRotationAxisCovariance) {
            internalRotationAxisCovariance
        } else {
            null
        }

    /**
     * Processes provided synced accelerometer, gyroscope and magnetometer measurement to obtain an
     * absolute attitude.
     *
     * @param syncedMeasurement measurement to be processed.
     * @return true if a new attitude is estimated, false otherwise.
     */
    fun process(syncedMeasurement: AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement): Boolean {
        val accelerometerMeasurement = syncedMeasurement.accelerometerMeasurement
        val gyroscopeMeasurement = syncedMeasurement.gyroscopeMeasurement
        val magnetometerMeasurement = syncedMeasurement.magnetometerMeasurement
        val timestamp = syncedMeasurement.timestamp
        return if (accelerometerMeasurement != null && gyroscopeMeasurement != null && magnetometerMeasurement != null) {
            process(
                accelerometerMeasurement,
                gyroscopeMeasurement,
                magnetometerMeasurement,
                timestamp
            )
        } else {
            false
        }
    }

    /**
     * Processes provided accelerometer, gyroscope and magnetometer measurements to obtain absolute
     * attitude.
     *
     * @param accelerometerMeasurement accelerometer measurement to be processed.
     * @param gyroscopeMeasurement gyroscope measurement to be processed.
     * @param magnetometerMeasurement magnetometer measurement to be processed.
     * @param timestamp timestamp when measurements being processed are assumed to occur.
     * @return true if a new attitude is estimated, false otherwise.
     */
    fun process(
        accelerometerMeasurement: AccelerometerSensorMeasurement,
        gyroscopeMeasurement: GyroscopeSensorMeasurement,
        magnetometerMeasurement: MagnetometerSensorMeasurement,
        timestamp: Long = gyroscopeMeasurement.timestamp
    ): Boolean {
        val isFirst = updateTimeInterval(timestamp)
        if (isFirst) {
            return false
        } else {
            processAccelerometerMeasurement(accelerometerMeasurement)
            processGyroscopeMeasurement(gyroscopeMeasurement)
            processMagnetometerMeasurement(magnetometerMeasurement)

            if (!gravityProcessor.process(accelerometerMeasurement, timestamp)) {
                return false
            } else {
                estimatedGravityNorm = gravityProcessor.gravityNorm

                if (!initialized) {
                    initKalmanFilter()
                    initialized = true
                } else {
                    updateTransitionMatrix()
                    updateMeasurementMatrix()
                    updateProcessNoiseCovariance()

                    // predict using current transition matrix
                    val statePre = kalmanFilter.predict()
                    retrieveParametersFromState(statePre)

                    // fixes state to ensure that quaternion rotation axis remains unitary
                    fixState(statePre)

                    val isFreeFall = estimatedGravityNorm < freeFallThreshold
                    if (isFreeFall) {
                        // device is in free fall
                        kalmanFilter.statePost.copyFrom(statePre)
                    } else {

                        // update Kalman filter measurement
                        updateMeasurement()
                        updateMeasurementNoiseCovariance()

                        val statePost = kalmanFilter.correct(z)

                        retrieveParametersFromState(statePost)

                        // fixes state to ensure that quaternion rotation axis remains unitary
                        fixState(statePost)
                    }

                    processCovariance(isFreeFall)

                    processorListener?.onProcessed(
                        this,
                        estimatedAttitude,
                        estimatedEulerAngles,
                        estimatedAttitudeCovariance,
                        estimatedEulerAnglesCovariance,
                        estimatedAccelerometerCovariance,
                        estimatedRotationAxisCovariance
                    )
                }

            }
            return true
        }
    }

    /**
     * Resets internal parameters.
     */
    fun reset() {
        gravityProcessor.reset()
        previousTimestamp = -1L
        estimatedTimeIntervalSeconds = 0.0
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
            estimatedTimeIntervalSeconds = TimeConverter.nanosecondToSecond(diff.toDouble())
        }
        previousTimestamp = timestamp

        return isFirst
    }

    /**
     * Processes provided accelerometer measurement to convert into NED body coordinates and
     * then into a triad taking into account any existing bias data in the measurement.
     *
     * @param accelerometerMeasurement accelerometer measurement to be processed.
     */
    private fun processAccelerometerMeasurement(
        accelerometerMeasurement: AccelerometerSensorMeasurement
    ) {
        accelerometerMeasurement.toNed(nedAccelerometerMeasurement)
        nedAccelerometerMeasurement.toTriad(accelerationTriad)
    }

    /**
     * Processes provided gyroscope measurement to convert into NED body coordinates and
     * then into a triad taking into account any existing bias data in the measurement.
     *
     * @param gyroscopeMeasurement gyroscope measurement to be processed.
     */
    private fun processGyroscopeMeasurement(gyroscopeMeasurement: GyroscopeSensorMeasurement) {
        gyroscopeMeasurement.toNed(nedGyrometerMeasurement)
        nedGyrometerMeasurement.toTriad(angularSpeedTriad)
    }

    /**
     * Processes provided magnetometer measurement to convert into NED body coordinates and
     * then into a triad taking into account any existing bias data in the measurement.
     *
     * @param magnetometerMeasurement magnetometer measurement to be processed.
     */
    private fun processMagnetometerMeasurement(magnetometerMeasurement: MagnetometerSensorMeasurement) {
        magnetometerMeasurement.toNed(nedMagnetometerMeasurement)
        nedMagnetometerMeasurement.toTriad(magneticFluxDensityTriad)
    }

    /**
     * Initializes Kalman filter.
     */
    private fun initKalmanFilter() {
        // initialize state to zero acceleration, no-rotation, and axis pointing downwards.
        val statePre = kalmanFilter.statePre
        statePre.initialize(0.0)
        statePre.setElementAtIndex(3, 1.0)
        statePre.setElementAtIndex(9, 1.0)

        val statePost = kalmanFilter.statePost
        statePost.copyFrom(statePre)

        // initialize error covariance a priori and posteriori to zero
        kalmanFilter.errorCovPre.initialize(0.0)
        kalmanFilter.errorCovPost.initialize(0.0)
    }

    /**
     * Updates filter transition matrix.
     * Transition matrix is made of three sub-matrices.
     * Acceleration 3x3 sub-matrix will always be the identity assuming that no external forces are
     * applied and consequently acceleration remains constant during a single interval. Since
     * Kalman Filter constructor already initializes the transition matrix to the identity, there is
     * no need to update this block.
     * Quaternion 4x4 sub-matrix corresponds to the quaternion contained within the Kalman filter
     * state. This matrix should express expected change of body orientation respect to Earth during
     * a time interval.
     * The third 3x3 block corresponds to the rotation vector. Since the angular velocity can be
     * assumed to be constant on a single time interval, it is assumed to be the identity, and there
     * is no need to update this block.
     */
    private fun updateTransitionMatrix() {
        val transitionMatrix = kalmanFilter.transitionMatrix

        // transition matrix is build as block diagonal
        transitionMatrix.initialize(0.0)

        // the first block is a 3x3 identity matrix
        transitionMatrix.setElementAt(0, 0, 1.0)
        transitionMatrix.setElementAt(1, 1, 1.0)
        transitionMatrix.setElementAt(2, 2, 1.0)

        // the second block is a 4x4 matrix having the following expression
        // phi_22 = w_0 * I_44 + omega

        // w_o is the angle corresponding to the estimated orientation change

        val w0 = estimatedOrientationChange.toAxisAndRotationAngle(axis)

        // w0I44 = w_0 * I_44
        w0I44.initialize(0.0)
        w0I44.setElementAt(0, 0, w0)
        w0I44.setElementAt(1, 1, w0)
        w0I44.setElementAt(2, 2, w0)
        w0I44.setElementAt(3, 3, w0)

        // q0 = [a0 b0 c0 d0]
        // deltaQ1 = [da db dc dd]
        // q1 = [a1 b1 c1 d1]

        // q1 = deltaQ1 * q0
        // [a1] = [da * a0 - db * b0 - dc * c0 - dd * d0] = [da     -db     -dc     -dd][a0]
        // [b1]   [da * b0 + db * a0 + dc * d0 - dd * c0]   [db      da     -dd      dc][b0]
        // [c1]   [da * c0 - db * d0 + dc * a0 + dd * b0]   [dc      dd      da     -db][c0]
        // [d1]   [da * d0 + db * c0 - dc * b0 + dd * a0]   [dd     -dc      db      da][d0]
        estimatedOrientationChange.quaternionMatrix(omega)

        // phi_22 = w_0 * I_44 + omega
        phi22.copyFrom(w0I44)
        phi22.add(omega)

        transitionMatrix.setSubmatrix(
            3,
            3,
            6,
            6,
            phi22
        )

        // the third block is also a 3x3 identity matrix
        transitionMatrix.setElementAt(7, 7, 1.0)
        transitionMatrix.setElementAt(8, 8, 1.0)
        transitionMatrix.setElementAt(9, 9, 1.0)
    }

    /**
     * Updates Kalman filter measurement matrix which relates filter state and measurement matrices.
     */
    private fun updateMeasurementMatrix() {
        // Kalman filter state (10x1 matrix) is made of:
        // - acceleration respect to Earth (a_e) - 3 components
        // - quaternion attitude respect to Earth (q_e) - 4 components
        // - rotation vector respect to body (w_b) - 3 components

        // Kalman filter measurements (9x1 matrix) is made of:
        // - acceleration (specific force) respect to body (a_b) - 3 components
        // - magnetic flux density (magnetic field) respect to body (m_b) - 3 components
        // - angular speed (angular velocity) respect to body (w_b) - 3 components

        // Consequently, measurement matrix m can be obtained from Kalman filter state x through
        // measurement matrix H as:
        // z = H * x

        // where H is a 9x10 matrix made of the following blocks:
        // H = [H11  H12  0  ]
        //     [0    H22  0  ]
        //     [0    0    H33]

        // where:
        // H11 is a 3x3 matrix relating acceleration respect to Earth and acceleration respect to
        // body
        // H12 is a 3x4 matrix relating quaternion respect to Earth and acceleration respect to body
        // So that:
        // a_b = H11 * a_3 + H12 * q_e

        // where q_e = [q0, q1, q2, q3]

        // H11 = [1 - 2 * (q2^2 + q3^2)     2 * (q0 * q3 + q1 * q2)     2 * (q1 * q3 - q0 * q2)]
        //       [2 * (q1 * q2 - q0 * q3)   1 - 2 * (q1^2 + q3^2)       2 * (q0 * q1 + q2 * q3)]
        //       [2 * (q0 * q2 + q1 * q3)   2 * (q2 * q3 - q0 * q1)     1 - 2 * (q1^2 + q2^2)  ]

        // H12 = -g * [-q2      q3      -q0     q1]
        //            [ q1      q0       q3     q2]
        //            [ q0     -q1      -q2     q3]

        // where g is gravity norm

        // Matrix H22 is a 3x4 matrix relating quaternion respect to Earth and measured body
        // magnetic flux density

        // H22 = h * sin(alpha) * [q0   q1  -q2  -q3] + h * cos(alpha) * [-q2   q3  -q0  q1]
        //                        [-q3  q2   q1  -q0]                    [ q1   q0   q3  q2]
        //                        [q2   q3   q0   q1]                    [ q0  -q1  -q2  q3]

        // where q_e = [q0, q1, q2, q3], alpha is the angle between the local magnetic and gravity
        // fields, and h is magnetic field modulus

        // Matrix H33 is a 3x3 matrix performs a prediction of gyroscope measurements in body
        // coordinates using only body rotation axis
        // w_b = H33 * v_b

        // Since a unit quaternion (representing a rotation) is made of the following components):
        // deltaQ = [cos(omega/2)  sin(omega/2)*vx  sin(omega/2)*vy  sin(omega/2)*vz)

        // where rotation angle omega can be assumed to be:
        // omega = w * deltaT
        // where w is angular speed (assumed constant in a single interval) and deltaT is the time
        // interval expressed in seconds.

        // For small enough time intervals (deltaT -> 0), the quaternion expression can be
        // approximated as:
        // deltaQ = [1  omega/2*vx  omega/2*vy  omega/2*vz] =
        //   [1  w*deltaT/2*vx  w*deltaT/2*vy w*deltaT/2*vz]

        // consequently the variation of rotation axis v = [vx vy vz] is related to angular speed
        // as follows:
        // v' = w*deltaT/2*v
        // where v' is the new rotation axis, and v is the old one. And both must have unitary norm
        // so that quaternion remains unitary: |v'| = 1, |v| = 1.
        // Consequently, variation in norm is:
        // |v'| = |w*deltaT/2|*|v|
        // |w * deltaT / 2 | = 1
        // And finally we obtain the approximate relation between angular speed and rotation axis
        // as:
        // w_b = 2 / deltaT * v_b

        // And consequently matrix H33 becomes:
        // H33 = 2 / deltaT * [1  0  0]
        //                    [0  1  0]
        //                    [0  0  1]

        estimatedAttitude.normalize()
        val q0 = estimatedAttitude.a
        val q1 = estimatedAttitude.b
        val q2 = estimatedAttitude.c
        val q3 = estimatedAttitude.d

        val sqrQ1 = q1 * q1
        val sqrQ2 = q2 * q2
        val sqrQ3 = q3 * q3

        val q1q2 = q1 * q2
        val q0q3 = q0 * q3
        val q0q2 = q0 * q2
        val q1q3 = q1 * q3
        val q2q3 = q2 * q3
        val q0q1 = q0 * q1

        // compute H11
        h11.setElementAtIndex(0, 1.0 - 2.0 * (sqrQ2 + sqrQ3))
        h11.setElementAtIndex(1, 2.0 * (q1q2 - q0q3))
        h11.setElementAtIndex(2, 2.0 * (q0q2 + q1q3))

        h11.setElementAtIndex(3, 2.0 * (q0q3 + q1q2))
        h11.setElementAtIndex(4, 1.0 - 2.0 * (sqrQ1 + sqrQ3))
        h11.setElementAtIndex(5, 2.0 * (q2q3 - q0q1))

        h11.setElementAtIndex(6, 2.0 * (q1q3 - q0q2))
        h11.setElementAtIndex(7, 2.0 * (q0q1 + q2q3))
        h11.setElementAtIndex(8, 1.0 - 2.0 * (sqrQ1 + sqrQ2))

        // compute H12
        val g = estimatedGravityNorm
        h12.setElementAtIndex(0, g * q2)
        h12.setElementAtIndex(1, -g * q1)
        h12.setElementAtIndex(2, -g * q0)

        h12.setElementAtIndex(3, -g * q3)
        h12.setElementAtIndex(4, -g * q0)
        h12.setElementAtIndex(5, g * q1)

        h12.setElementAtIndex(6, g * q0)
        h12.setElementAtIndex(7, -g * q3)
        h12.setElementAtIndex(8, g * q2)

        h12.setElementAtIndex(9, -g * q1)
        h12.setElementAtIndex(10, -g * q2)
        h12.setElementAtIndex(11, -g * q3)

        // compute H22
        val gx = gravityProcessor.gx
        val gy = gravityProcessor.gy
        val gz = gravityProcessor.gz

        val h = magneticFluxDensityTriad.norm
        val bx = magneticFluxDensityTriad.valueX
        val by = magneticFluxDensityTriad.valueY
        val bz = magneticFluxDensityTriad.valueZ

        // cos(akpha) = g * b^T / (|g| * |b|) = (gx * bx + gy * by + gz * bz) / (h * g)

        val cosAlpha = (gx * bx + gy * by + gz * bz) / (h * g)
        val alpha = acos(cosAlpha)
        val sinAlpha = sin(alpha)

        val hSinAlpha = h * sinAlpha
        val hCosAlpha = h * cosAlpha

        val hSinAlphaQ0 = hSinAlpha * q0
        val hSinAlphaQ1 = hSinAlpha * q1
        val hSinAlphaQ2 = hSinAlpha * q2
        val hSinAlphaQ3 = hSinAlpha * q3

        val hCosAlphaQ0 = hCosAlpha * q0
        val hCosAlphaQ1 = hCosAlpha * q1
        val hCosAlphaQ2 = hCosAlpha * q2
        val hCosAlphaQ3 = hCosAlpha * q3

        h22.setElementAtIndex(0, hSinAlphaQ0 - hCosAlphaQ2)
        h22.setElementAtIndex(1, -hSinAlphaQ3 + hCosAlphaQ1)
        h22.setElementAtIndex(2, hSinAlphaQ2 + hCosAlphaQ0)

        h22.setElementAtIndex(3, hSinAlphaQ1 + hCosAlphaQ3)
        h22.setElementAtIndex(4, hSinAlphaQ2 + hCosAlphaQ0)
        h22.setElementAtIndex(5, hSinAlphaQ3 - hCosAlphaQ1)

        h22.setElementAtIndex(6, -hSinAlphaQ2 - hCosAlphaQ0)
        h22.setElementAtIndex(7, hSinAlphaQ1 + hCosAlphaQ3)
        h22.setElementAtIndex(8, hSinAlphaQ0 - hCosAlphaQ2)

        h22.setElementAtIndex(9, -hSinAlphaQ3 + hCosAlphaQ1)
        h22.setElementAtIndex(10, -hSinAlphaQ0 + hCosAlphaQ2)
        h22.setElementAtIndex(11, hSinAlphaQ1 + hCosAlphaQ3)

        // compute H33
        val value = 2.0 / estimatedTimeIntervalSeconds
        h33.initialize(0.0)
        for (i in 0 until Triad.COMPONENTS) {
            h33.setElementAt(i, i, value)
        }

        // update measurement matrix

        // H = [H11  H12  0  ]
        //     [0    H22  0  ]
        //     [0    0    H33]
        val measurementMatrix = kalmanFilter.measurementMatrix
        measurementMatrix.initialize(0.0)
        measurementMatrix.setSubmatrix(
            0,
            0,
            2,
            2,
            h11
        )
        measurementMatrix.setSubmatrix(
            0,
            3,
            2,
            6,
            h12
        )
        measurementMatrix.setSubmatrix(
            3,
            3,
            5,
            6,
            h22
        )
        measurementMatrix.setSubmatrix(
            6,
            7,
            8,
            9,
            h33
        )
    }

    /**
     * Updates process noise covariance on a block by block basis taking into account time interval
     * between measurements, accelerometer standard deviation, current estimated quaternion state.
     */
    private fun updateProcessNoiseCovariance() {
        // process noise covariance is a block diagonal matrix following expression:
        // Q = diag(Qa, Qq, Qr)

        // where:
        // - Qa corresponds to covariance of acceleration respect Earth. 3x3 matrix
        // - Qq corresponds to covariance of quaternion respect Earth. 4x4 matrix.
        // - Qr corresponds to covariance of unitary body rotation axis. 3x3 matrix.

        // Qa = sigma_a^2 * I_33
        val accelerometerVariance =
            accelerometerNoiseStandardDeviation * accelerometerNoiseStandardDeviation
        qA.initialize(0.0)
        for (i in 0 until Triad.COMPONENTS) {
            qA.setElementAt(i, i, accelerometerVariance)
        }

        // Qq
        val gyroVariance = gyroscopeNoisePsd / estimatedTimeIntervalSeconds
        val sqrHalfInterval = (estimatedTimeIntervalSeconds / 2.0).pow(2.0)

        // Qq = T^2/4 * prodQ * gyroCov * prodQ^T

        gyroCov.initialize(0.0)
        for (i in 0 until Triad.COMPONENTS) {
            gyroCov.setElementAt(i, i, gyroVariance)
        }

        estimatedAttitude.normalize()
        val q0 = estimatedAttitude.a
        val q1 = estimatedAttitude.b
        val q2 = estimatedAttitude.c
        val q3 = estimatedAttitude.d

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

        prodQ.multiply(gyroCov, tmpQ)
        tmpQ.multiply(transposedProdQ, qQ)
        qQ.multiplyByScalar(sqrHalfInterval)

        // Qr
        // We know from [updateMeasurementMatrix] that:
        // Variation of rotation axis v = [vx vy vz] is related to angular speed
        // as follows:
        // v' = w_b * deltaT / 2 * v --> d(v') / d(v) = w_b * deltaT / 2
        // We know that propagation of variance if  var(y(x)) = (dy/dx)^2 * var(x)
        // Taking both expressions above,
        // var(v) = (deltaT / 2)^2 * var(w_b)

        val varianceRotationVector = sqrHalfInterval * gyroVariance
        qR.initialize(0.0)
        for (i in 0 until Triad.COMPONENTS) {
            qR.setElementAt(i, i, varianceRotationVector)
        }

        // update process noise covariance matrix
        val processNoiseCov = kalmanFilter.processNoiseCov
        processNoiseCov.setSubmatrix(
            0,
            0,
            2,
            2,
            qA
        )
        processNoiseCov.setSubmatrix(
            3,
            3,
            6,
            6,
            qQ
        )
        processNoiseCov.setSubmatrix(
            7,
            7,
            9,
            9,
            qR
        )
    }

    /**
     * Updates measurement matrix with accelerometer, magnetometer and gyroscope measurements to be
     * used during Kalman filter correction phase
     */
    private fun updateMeasurement() {
        z.setElementAtIndex(0, accelerationTriad.valueX)
        z.setElementAtIndex(1, accelerationTriad.valueY)
        z.setElementAtIndex(2, accelerationTriad.valueZ)

        z.setElementAtIndex(3, magneticFluxDensityTriad.valueX)
        z.setElementAtIndex(4, magneticFluxDensityTriad.valueY)
        z.setElementAtIndex(5, magneticFluxDensityTriad.valueZ)

        z.setElementAtIndex(6, angularSpeedTriad.valueX)
        z.setElementAtIndex(7, angularSpeedTriad.valueY)
        z.setElementAtIndex(8, angularSpeedTriad.valueZ)
    }

    /**
     * Updates measurement noise covariance based on accelerometer and gyroscope variances.
     */
    private fun updateMeasurementNoiseCovariance() {
        val measurementNoiseCov = kalmanFilter.measurementNoiseCov

        // accelerometer measurement covariance
        val accelerometerVariance =
            accelerometerNoiseStandardDeviation * accelerometerNoiseStandardDeviation
        for (i in 0 until 3) {
            measurementNoiseCov.setElementAt(i, i, accelerometerVariance)
        }

        // magnetometer covariance
        val magnetometerVariance =
            magnetometerNoiseStandardDeviation * magnetometerNoiseStandardDeviation
        for (i in 3 until 6) {
            measurementNoiseCov.setElementAt(i, i, magnetometerVariance)
        }

        // gyroscope measurement covariance
        val gyroVariance = gyroscopeNoisePsd / estimatedTimeIntervalSeconds
        for (i in 6 until 9) {
            measurementNoiseCov.setElementAt(i, i, gyroVariance)
        }
    }

    /**
     * Retrieves parameters from Kalman state vector.
     */
    private fun retrieveParametersFromState(state: Matrix) {
        estimatedAcceleration.valueX = state.getElementAtIndex(0)
        estimatedAcceleration.valueY = state.getElementAtIndex(1)
        estimatedAcceleration.valueZ = state.getElementAtIndex(2)
        estimatedAcceleration.unit = AccelerationUnit.METERS_PER_SQUARED_SECOND

        estimatedAttitude.a = state.getElementAtIndex(3)
        estimatedAttitude.b = state.getElementAtIndex(4)
        estimatedAttitude.c = state.getElementAtIndex(5)
        estimatedAttitude.d = state.getElementAtIndex(6)
        estimatedAttitude.normalize()

        estimatedAttitude.inverse(estimatedBodyAttitude)

        // 3 last parameters correspond to the last 3 parameters of a unitary quaternion, hence
        val b = state.getElementAtIndex(7)
        val c = state.getElementAtIndex(8)
        val d = state.getElementAtIndex(9)

        // 1 = sqrt(a^2 + b^2 + c^2 + d^2)
        // 1 = a^2 + b^2 + c^2 + d^2
        // a^2 = 1 - b^2 - c^2 - d^2
        // a = sqrt(1 - b^2 - c^2 - d^2)
        val a = sqrt(1.0 - b * b - c * c - d * d)

        estimatedOrientationChange.a = a
        estimatedOrientationChange.b = b
        estimatedOrientationChange.c = c
        estimatedOrientationChange.d = d
        estimatedOrientationChange.normalize()
    }

    /**
     * Fixes state to ensure quaternion part remains normalized and rotation axis remains unitary.
     *
     * @param state Kalman filter state.
     */
    private fun fixState(state: Matrix) {
        estimatedAttitude.normalize()
        state.setElementAtIndex(3, estimatedAttitude.a)
        state.setElementAtIndex(4, estimatedAttitude.b)
        state.setElementAtIndex(5, estimatedAttitude.c)
        state.setElementAtIndex(6, estimatedAttitude.d)

        estimatedAttitude.toAxisAndRotationAngle(axis)
        state.setElementAtIndex(7, axis[0])
        state.setElementAtIndex(8, axis[1])
        state.setElementAtIndex(9, axis[2])
    }

    /**
     * Processes Kalman filter state error covariance to obtain uncertainty estimations for
     * attitude, acceleration and rotation axis (expressed in NED coordinates). If needed, Euler
     * angles for current attitude are also estimated along with their uncertainty.
     *
     * @param isFreeFall true indicates that device is in free fall, and consequently no measurement
     * corrections are made. Hence, in such cases predicted error covariance is used instead of
     * corrected error covariance.
     */
    private fun processCovariance(isFreeFall: Boolean) {
        if (computeCovariances || computeEulerAngles || computeEulerAnglesCovariance) {
            // process covariances
            val errorCov = if (isFreeFall) {
                kalmanFilter.errorCovPre
            } else {
                kalmanFilter.errorCovPost
            }

            // check covariance state
            if (!isPositiveSemiDefinite(errorCov, symmetricThreshold)) {
                // when covariance is not positive semi-definite, we can assume that Kalman filter
                // has become numerically unstable, and we need to reset error covariance value
                initErrorCovariances()

                numericalInstabilitiesListener?.onNumericalInstabilityDetected(this)
            }

            // take only the part corresponding to ENU quaternion covariance
            errorCov.getSubmatrix(
                3,
                3,
                6,
                6,
                internalNedQuaternionCovariance
            )

            if (computeEulerAngles) {
                // As defined in: https://en.wikipedia.org/wiki/Propagation_of_uncertainty
                // covariance propagation for a function converting quaternion to euler angles is:
                // Cov`= J * Cov * J^T
                if (computeEulerAnglesCovariance) {
                    estimatedAttitude.toEulerAngles(internalEulerAngles, eulerJacobian)
                    eulerJacobian.transpose(transposedEulerJacobian)

                    propagateEulerAnglesUncertainty(
                        internalNedQuaternionCovariance,
                        internalEulerAnglesCovariance
                    )
                } else {
                    estimatedAttitude.toEulerAngles(internalEulerAngles)
                }
            }

            if (computeAccelerometerCovariance) {
                errorCov.getSubmatrix(
                    0,
                    0,
                    2,
                    2,
                    internalAccelerometerCovariance
                )
            }

            if (computeRotationAxisCovariance) {
                errorCov.getSubmatrix(
                    7,
                    7,
                    9,
                    9,
                    internalRotationAxisCovariance
                )
            }
        }
    }

    /**
     * Indicates whether provided matrix is positive semi-definite or not upt to provided
     * threshold.
     * Covariance matrices must always be positive semi-definite, otherwise we can assume that
     * there are numerical instabilities in Kalman filter.
     *
     * @param m matrix to be evaluated.
     * @throws threshold threshold to determine whether matrix is positive semi-definite. This
     * is a value close to zero due to machine precision inaccuracies.
     * @return true if matrix is positive semi-definite, false otherwise.
     */
    private fun isPositiveSemiDefinite(
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
     * Initializes Kalman filter predicted and corrected error covariances to zero.
     */
    private fun initErrorCovariances() {
        kalmanFilter.errorCovPre.initialize(0.0)
        kalmanFilter.errorCovPost.initialize(0.0)
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
     * Propagates effects of uncertainty to provided covariance matrix using provided jacobian,
     * and transposed jacobian.
     * Uncertainty is propagated as: Cov' = J * Cov * J^T
     *
     * @param covariance input covariance whose uncertainty is being propagated
     * @param jacobian jacobian to propagate uncertainty of covariance.
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

    init {
        this.location = location
        this.adjustGravityNorm = adjustGravityNorm
    }

    companion object {
        /**
         * Kalman filter's number of dynamic parameters.
         * 3 parameters for estimated acceleration respect to Earth, 4 parameters for estimated
         * quaternion respect to Earth (to convert from Earth to Body system of coordinates taking
         * into account device orientation), and 3 parameters containing vectorial part of body
         * rotation.
         */
        private const val KALMAN_FILTER_DYNAMIC_PARAMETERS = 10

        /**
         * Kalman filter's number of measurement parameters.
         * 3 parameters for measured body acceleration, 3 parameters for measured body magnetic
         * field, 3 parameters for measured body angular speed.
         */
        private const val KALMAN_FILTER_MEASUREMENT_PARAMETERS = 9

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
         * Default standard deviation for magnetometer noise expressed in Teslas.
         * Typical value is 0.5 µT
         */
        const val DEFAULT_MAGNETOMETER_NOISE_STANDARD_DEVIATION = 0.5e-6

        /**
         * Average gravity at Earth's sea level expressed in m/s^2.
         */
        const val EARTH_GRAVITY = SensorManager.GRAVITY_EARTH.toDouble()

        /**
         * Default threshold to consider that device is in free fall.
         * When device is in free fall, accelerometer measurements are considered
         * unreliable and are ignored (only gyroscope predictions are made).
         */
        const val DEFAULT_FREE_FALL_THRESHOLD = 0.1 * EARTH_GRAVITY

        /**
         * Threshold to determine whether covariance matrices are positive definite or not.
         */
        const val DEFAULT_SYMMETRIC_THRESHOLD = 1e-10
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
            processor: KalmanAbsoluteAttitudeProcessor2,
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
        fun onNumericalInstabilityDetected(processor: KalmanAbsoluteAttitudeProcessor2)
    }
}