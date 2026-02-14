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

import android.hardware.SensorManager
import android.location.Location
import com.irurueta.algebra.Matrix
import com.irurueta.android.navigation.inertial.ENUtoNEDConverter
import com.irurueta.android.navigation.inertial.old.collectors.AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorMeasurement
import com.irurueta.android.navigation.inertial.toNEDPosition
import com.irurueta.geometry.Quaternion
import com.irurueta.geometry.Rotation3D
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.NEDPosition
import com.irurueta.navigation.inertial.NEDGravity
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad
import com.irurueta.navigation.inertial.estimators.NEDGravityEstimator
import com.irurueta.navigation.inertial.wmm.NEDMagneticFluxDensity
import com.irurueta.navigation.inertial.wmm.WMMEarthMagneticFluxDensityEstimator
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel
import com.irurueta.units.MagneticFluxDensityConverter
import com.irurueta.units.TimeConverter
import java.util.Date
import java.util.GregorianCalendar

/**
 * Estimates absolute attitude using a Kalman filter.
 * This is based on:
 * https://github.com/liviobisogni/quaternion-kalman-filter
 * And:
 * Young Soo Suh. Orientation estimation using a quaternion-based indirect Kalman filter with
 * adaptive estimation of external acceleration.
 *
 * @param location current device location or null if location is not known. When location and
 * [currentDate] are provided, magnetic field dip and declination angles are estimated along with
 * current gravity norm.
 * @property currentDate current date. This is used to estimate magnetic field dip and declination
 * angles and provided location. If null, current date is assumed.
 * @property computeExternalAcceleration true to compute external acceleration, false otherwise.
 * @property computeEulerAngles true to compute Euler angles of estimated attitude, false otherwise.
 * @property computeCovariances true to compute covariances, false otherwise.
 * @property computeEulerAnglesCovariance true to compute Euler angles covariance, false otherwise.
 * @param phiMatrixMethod method to compute the matrix exponential to integrate orientation.
 * @param processNoiseCovarianceMethod method to compute process noise covariance matrix.
 * @param quaternionStepIntegrator type of integrator to use to compute quaternions.
 * @property gyroscopeNoisePsd gyroscope noise PSD (Power Spectral Density) expressed in (rad^2/s)/Hz.
 * @property accelerometerNoiseStandardDeviation accelerometer noise standard deviation expressed in
 * meters per squared second (m/s^2).
 * @property magnetometerNoiseStandardDeviation magnetometer noise standard deviation expressed in
 * Teslas (T).
 * @πroperty externalAccelerationCovarianceMatrixEstimationMethod method to estimate external
 * acceleration.
 * @property processorListener listener to handle events raised by this processor.
 */
class KalmanAbsoluteAttitudeProcessor5(
    location: Location? = null,
    var currentDate: Date? = null,
    var computeExternalAcceleration: Boolean = true,
    var computeEulerAngles: Boolean = true,
    var computeCovariances: Boolean = true,
    var computeEulerAnglesCovariance: Boolean = true,
    phiMatrixMethod: PhiMatrixMethod = PhiMatrixMethod.APPROXIMATED, // could also be PRECISE
    processNoiseCovarianceMethod: ProcessNoiseCovarianceMethod = ProcessNoiseCovarianceMethod.BETTER,
    quaternionStepIntegrator: QuaternionStepIntegratorType = QuaternionStepIntegratorType.SUH,
    val gyroscopeNoisePsd: Double = DEFAULT_GYROSCOPE_NOISE_PSD,
    val accelerometerNoiseStandardDeviation: Double = DEFAULT_ACCELEROMETER_NOISE_STANDARD_DEVIATION,
    val magnetometerNoiseStandardDeviation: Double = DEFAULT_MAGNETOMETER_NOISE_STANDARD_DEVIATION,
    externalAccelerationCovarianceMatrixEstimationMethod: ExternalAccelerationCovarianceMatrixEstimationMethod =
        DEFAULT_EXTERNAL_ACCELERATION_COVARIANCE_MATRIX_ESTIMATION_METHOD,
    val processorListener: OnProcessedListener? = null,
) {

    /**
     * Gets or sets current device location.
     * When location and [currentDate] are provided, magnetic field dip and declination angles are
     * estimated along with current gravity norm.
     */
    var location: Location? = location
        set(value) {
            field = value
            estimatePropertiesDependingOnLocation()
        }

    /**
     * Earth's magnetic model. If null, the default model is used if [useWorldMagneticModel] is
     * true. If [useWorldMagneticModel] is false, this is ignored.
     */
    var worldMagneticModel: WorldMagneticModel? = null
        set(value) {
            field = value
            buildWMMEstimator()
        }

    /**
     * Indicates whether world magnetic model is taken into account magnetic field dip and
     * declination angles, so that orientation pointing to the true north is estimated.
     */
    var useWorldMagneticModel: Boolean = true
        set(value) {
            field = value
            buildWMMEstimator()
        }

    /**
     * Obtains magnetic field declination of local NED reference system expressed in radians if
     * World Magnetic Model is used, otherwise zero is returned.
     * Magnetic field declination indicates how much deviation the heading angle has between
     * the magnetic north pole and the true geographic north pole.
     */
    var magneticDeclination: Double = 0.0
        private set

    /**
     * Returns magnetic field dip angle of local NED reference system expressed in radians if World
     * Magnetic Model is used, otherwise zero is returned.
     * Magnetic field dip angle indicates how much the magnetic field points towards Earth's center.
     * At magnetic equator dip angle is zero, at Norm magnetic pole is +90º, and at South magnetic
     * pole is -90º.
     * At the north hemisphere dip angle is positive (magnetic field points downwards), indicating
     * the vertical deviation of magnetic field respect local Earth surface at a given position
     * (latitude, longitude and height).
     * At the south hemisphere dip angle is negative (magnetic field points upwards), indicating
     * the vertical deviation of magnetic field respect local Earth surface at a given position
     * (latitude, longitude and height).
     */
    var magneticDip: Double = 0.0
        private set

    /**
     * Returns magnetic flux density at current location expressed in local NED reference system.
     * This is only available if World Magnetic Model is used, otherwise a zero triad is returned.
     */
    var magneticFluxDensity: MagneticFluxDensityTriad = MagneticFluxDensityTriad()
        private set

    /**
     * Gets expected gravity norm at current location.
     * If no location is available, average gravity at sea level is returned instead.
     */
    var gravityNorm: Double = EARTH_GRAVITY
        private set

    /**
     * Time interval between measurements expressed in seconds (s).
     */
    var timeIntervalSeconds: Double = 0.0
        private set

    /**
     * Estimated external acceleration applied on the device expressed in leveled local NED
     * coordinates respect to Earth.
     */
    val nedExternalAcceleration: AccelerationTriad?
        get() = if (computeExternalAcceleration) {
            nedA
        } else {
            null
        }

    /**
     * Gets estimated external acceleration applied on the device and expressed in leveled local NED
     * coordinates respect to Earth.
     *
     * @param result instance where result will be stored.
     * @return true if result was stored, false otherwise.
     */
    fun getNedExternalAcceleration(result: AccelerationTriad): Boolean {
        return if (computeExternalAcceleration) {
            result.copyFrom(nedA)
            true
        } else {
            false
        }
    }

    /**
     * Estimated external acceleration applied on the device expressed in ECEF coordinates respect
     * to Earth.
     */
    val ecefExternalAcceleration: AccelerationTriad?
        get() = if (computeExternalAcceleration) {
            ecefA
        } else {
            null
        }

    /**
     * Gets estimated external acceleration applied on the device end expressed in ECEF coordinates
     * respect to Earth.
     *
     * @param result instance where result will be stored.
     * @return true if result was stored, false otherwise.
     */
    fun getEcefExternalAcceleration(result: AccelerationTriad): Boolean {
        return if (computeExternalAcceleration) {
            result.copyFrom(ecefA)
            true
        } else {
            false
        }
    }

    /**
     * Estimated external acceleration applied on the device expressed in NED coordinates respect
     * to device body.
     */
    val bodyExternalAcceleration: AccelerationTriad?
        get() = if (computeExternalAcceleration) {
            bodyA
        } else {
            null
        }

    /**
     * Gets estimated external acceleration applied on the device expressed in NED coordinates
     * respect to device body.
     *
     * @param result instance where result will be stored.
     * @return true if result was stored, false otherwise.
     */
    fun getBodyExternalAcceleration(result: AccelerationTriad): Boolean {
        return if (computeExternalAcceleration) {
            result.copyFrom(bodyA)
            true
        } else {
            false
        }
    }

    /**
     * Estimated device attitude expressed in ECEF coordinates respect to Earth.
     */
    val ecefAttitude = Quaternion()

    /**
     * Estimated device attitude expressed in leveled local NED coordinates respect to Earth.
     */
    val nedAttitude = Quaternion()

    /**
     * Estimated device attitude expressed in body coordinates.
     */
    val bodyAttitude = Quaternion()

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
     * Gets Euler angles associated to estimated NED attitude.
     * Array contains roll, pitch and yaw angles expressed in radians (rad) and following order
     * indicated here.
     * This is only available if [computeCovariances] and [computeEulerAngles] are true.
     *
     * @param result instance where result will be stored.
     * @return true if result was stored, false otherwise.
     */
    fun getEulerAngles(result: DoubleArray): Boolean {
        return if (computeCovariances && computeEulerAngles) {
            internalEulerAngles.copyInto(result)
            true
        } else {
            false
        }
    }

    /**
     * Error covariance of estimated quaternion attitude expressed in NED coordinates.
     * This is only available if [computeCovariances] is true.
     */
    val nedAttitudeCovariance: Matrix?
        get() = if (computeCovariances) {
            internalNedQCovariance
        } else {
            null
        }

    /**
     * Gets error covariance of estimated quaternion attitude expressed in NED coordinates.
     * This is only available if [computeCovariances] is true.
     *
     * @param result instance where result will be stored.
     * @return true if result was stored, false otherwise.
     */
    fun getAttitudeCovariance(result: Matrix): Boolean {
        return if (computeCovariances) {
            result.copyFrom(internalNedQCovariance)
            true
        } else {
            false
        }
    }

    /**
     * Error covariance of estimated Euler angles for NED attitude.
     * This is only available if [computeCovariances] and [computeEulerAnglesCovariance] are true.
     */
    val eulerAnglesCovariance: Matrix?
        get() = if (computeCovariances && computeEulerAnglesCovariance) {
            internalEulerCovariance
        } else {
            null
        }

    /**
     * Gets error covariance of estimated Euler angles for NED attitude.
     * This is only available if [computeCovariances] and [computeEulerAnglesCovariance] are true.
     *
     * @param result instance where result will be stored.
     * @return true if result was stored, false otherwise.
     */
    fun getEulerAnglesCovariance(result: Matrix): Boolean {
        return if (computeCovariances && computeEulerAnglesCovariance) {
            result.copyFrom(internalEulerCovariance)
            true
        } else {
            false
        }
    }

    /**
     * Estimated gyroscope bias.
     */
    val bg = Matrix(AngularSpeedTriad.COMPONENTS, 1)

    /**
     * Estimated accelerometer bias.
     */
    val ba = Matrix(AccelerationTriad.COMPONENTS, 1)

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

    /**
     * Contains magnetic flux density at current location expressed in local NED reference system
     * used internally.
     * This is only available if World Magnetic Model is used, otherwise a zero triad is returned.
     */
    private val nedMagneticFluxDensity = NEDMagneticFluxDensity()

    /**
     * Column vector representing gravity in local leveled NED coordinates.
     */
    private val nedGravityMatrix: Matrix = Matrix(AccelerationTriad.COMPONENTS, 1)

    /**
     * Instance to be reused to compute year with fractional part when World Magnetic Model is
     * taken into account for magnetic dip and declination estimation purposes.
     */
    private val calendar = GregorianCalendar()

    /**
     * Estimator of magnetic dip an declination angles if world magnetic model is taken into
     * account.
     */
    private var wmmEstimator: WMMEarthMagneticFluxDensityEstimator? = null

    /**
     * Position expressed in NED coordinates being reused.
     */
    private val nedPosition = NEDPosition()

    /**
     * Gravity expressed in NED coordinates.
     */
    private val nedGravity = NEDGravity()

    /**
     * Rotation matrix to convert local NED coordinates to ECEF ones.
     */
    private val nedToEcefMatrix = Matrix(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)

    /**
     * Internal estimated external acceleration applied on the device expressed in NED coordinates
     * respect to device body.
     */
    private val bodyA = AccelerationTriad()

    /**
     * Internal estimated external acceleration applied on the device expressed in leveled local NED
     * coordinates respect to Earth.
     */
    private val nedA = AccelerationTriad()

    /**
     * Internal estimated external acceleration applied on device expressed in ECEF coordinates
     * respect to Earth.
     */
    private val ecefA = AccelerationTriad()

    /**
     * [bodyA] expressed in matrix form.
     */
    private val bodyAMatrix = Matrix(AccelerationTriad.COMPONENTS, 1)

    /**
     * [nedA] expressed in matrix form.
     */
    private val nedAMatrix = Matrix(AccelerationTriad.COMPONENTS, 1)

    /**
     * [ecefA] expressed in matrix form.
     */
    private val ecefAMatrix = Matrix(AccelerationTriad.COMPONENTS, 1)

    /**
     * Estimated covariance of leveled local NED attitude.
     */
    private val internalNedQCovariance: Matrix = Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS)

    /**
     * Euler angles of estimated NED attitude.
     * This is used internally and reused for performance reasons.
     */
    private val internalEulerAngles = DoubleArray(Quaternion.N_ANGLES)

    /**
     * Covariance of estimated Euler angles.
     */
    private val internalEulerCovariance: Matrix = Matrix(Quaternion.N_ANGLES, Quaternion.N_ANGLES)

    /**
     * Contains gyroscope data expressed in NED coordinates respect to body.
     */
    private val nedBodyAngularSpeedTriad: AngularSpeedTriad = AngularSpeedTriad()

    /**
     * Contains accelerometer data expressed in NED coordinates respect to body.
     * This is reused for performance reasons.
     */
    private val nedBodyAccelerationTriad: AccelerationTriad = AccelerationTriad()

    /**
     * Contains magnetometer data expressed in NED coordinates respect to body.
     */
    private val nedBodyMagneticFluxDensityTriad: MagneticFluxDensityTriad =
        MagneticFluxDensityTriad()

    /**
     * Timestamp of previous sample expressed in nanoseconds.
     */
    private var previousTimestamp: Long = -1L

    /**
     * Contains predicted attitude respect to body for next step.
     * Because rotation is defined respect to body, rotation converts from body coordinates
     * to leveled local NED coordinates respect to Earth:
     * pn = previousBodyQ * pb
     */
    private val predictedBodyQ: Quaternion = Quaternion()

    /**
     * Contains predicted leveled local NED attitude respect to Earth for next step.
     * Converts from leveled NED local coordinates to body NED coordinates.
     */
//    private val predictedNedQ: Quaternion = Quaternion()

    /**
     * Contains corrected attitude respect to body.
     * Because rotation is defined respect to body, rotation converts from body coordinates
     * to leveled local NED coordinates respect to Earth:
     * pn = previousBodyQ * pb
     */
    private val bodyQ: Quaternion = Quaternion()

    /**
     * Contains corrected leveled local NED attitude respect to Earth.
     */
    private val nedQ: Quaternion = Quaternion()

    /**
     * Contains corrected ECEF attitude respect to Earth.
     */
    private val ecefQ: Quaternion = Quaternion()

    /**
     * Continuous time process noise covariance matrix.
     */
    private val q: Matrix =
        Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    /**
     * Covariance measurement matrix of the gyroscope measurements.
     */
    private val rg: Matrix =
        Matrix.identity(AngularSpeedTriad.COMPONENTS, AngularSpeedTriad.COMPONENTS)

    /**
     * Predicts Kalman filter state using gyroscope measurements.
     */
    private val gyroscopePredictor = KalmanGyroscopePredictor(
        phiMatrixMethod,
        processNoiseCovarianceMethod,
        quaternionStepIntegrator,
        gyroscopeNoisePsd
    )

    /**
     * Corrects Kalman filter state using accelerometer measurements.
     */
    private val accelerometerCorrector = KalmanAccelerometerCorrector(
        externalAccelerationCovarianceMatrixEstimationMethod
    )

    /**
     * Corrects Kalman filter state using magnetometer measurements.
     */
    private val magnetometerCorrector = KalmanMagnetometerCorrector()

    // x_hat__prev_prev
    /**
     * Kalman state before prediction step.
     */
    private val kalmanStatePrePredicted = Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, 1)

    /**
     * Kalman filter state error covariance matrix before prediction step.
     */
    private val kalmanErrorCovPrePredicted =
        Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    // x_hat__next_prev
    /**
     * Kalman state after prediction step.
     */
    private val kalmanStatePostPredicted = Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, 1)

    /**
     * Kalman filter state error covariance matrix after prediction step.
     */
    private val kalmanErrorCovPostPredicted =
        Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    // x_hat_a__next_next
    /**
     * Corrected Kalman filter state after processing accelerometer measurement.
     */
    private val kalmanStateAccelerometerCorrected = Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, 1)

    /**
     * Kalman filter state error covariance matrix after accelerometer correction.
     */
    private val kalmanErrorCovAccelerometerCorrected =
        Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    // x_hat__next_next
    /**
     * Corrected Kalman filter state after processing magnetometer measurement.
     */
    private val kalmanStateCorrected = Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, 1)

    /**
     * Kalman filter state error covariance matrix after magnetometer correction.
     */
    private val kalmanErrorCovCorrected =
        Matrix(KALMAN_FILTER_DYNAMIC_PARAMETERS, KALMAN_FILTER_DYNAMIC_PARAMETERS)

    /**
     * Contains corrected NED attitude.
     * Converts from leveled NED local coordinates to body NED coordinates.
     */
    private val nedRotationMatrix: Matrix = Matrix(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)

    /**
     * Contains corrected inverse NED attitude.
     * Converts from body NED coordinates to leveled NED local coordinates.
     */
    private val inverseNedRotationMatrix: Matrix =
        Matrix(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)

    /**
     * Covariance of estimated quaternion axis coordinates error.
     */
    private val qeAxisCovariance: Matrix = Matrix(Quaternion.N_ANGLES, Quaternion.N_ANGLES)

    /**
     * Jacobian of estimated quaternion axis coordinates error.
     */
    private val qeJacobian: Matrix = Matrix(Quaternion.N_PARAMS, Quaternion.N_ANGLES)

    /**
     * Transposed of [qeJacobian].
     */
    private val transposedQeJacobian: Matrix = Matrix(Quaternion.N_ANGLES, Quaternion.N_PARAMS)

    /**
     * Temporal matrix to estimate error quaternion covariance.
     */
    private val tmp1: Matrix = Matrix(Quaternion.N_PARAMS, Quaternion.N_ANGLES)

    /**
     * Estimated error quaternion covariance.
     */
    private val qeCovariance: Matrix = Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS)

    /**
     * Jacobian of [predictedNedQ] respect to estimated error quaternion.
     */
    private val predictedNedQJacobian: Matrix = Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS)

    /**
     * Transposed of [predictedNedQJacobian].
     */
    private val transposedPredictedNedQJacobian: Matrix =
        Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS)

    /**
     * Temporal matrix to estimate covariance of leveled local NED attitude.
     */
    private val tmp2: Matrix = Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS)

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
     * Temporal matrix to estimate covariance of euler angles of leveled local NED attitude.
     */
    private val tmp3: Matrix = Matrix(Quaternion.N_ANGLES, Quaternion.N_PARAMS)

    /**
     * Rotation quaternion to convert local NED coordinates to ECEF coordinates.
     */
    private val nedToEcefQ = Quaternion()

    /**
     * Rotation quaternion to convert ECEF coordinates to local NED coordinates.
     */
    private val ecefToNedQ = Quaternion()

    /**
     * Temporary quaternion used internally.
     */
    private val tmpQ = Quaternion()

    /**
     * Processes provided synced accelerometer, gyroscope and magnetometer measurement to obtain  a
     * relative leveled attitude.
     *
     * @param syncedMeasurement measurement to be processed.
     * @return true if a new attitude is estimated, false otherwise.
     */
    fun process(syncedMeasurement: AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement):
            Boolean {
        val accelerometerMeasurement = syncedMeasurement.accelerometerMeasurement
        val gyroscopeMeasurement = syncedMeasurement.gyroscopeMeasurement
        val magnetometerMeasurement = syncedMeasurement.magnetometerMeasurement
        return if (accelerometerMeasurement != null
            && gyroscopeMeasurement != null
            && magnetometerMeasurement != null
        ) {
            process(
                accelerometerMeasurement,
                gyroscopeMeasurement,
                magnetometerMeasurement,
                syncedMeasurement.timestamp
            )
        } else {
            false
        }
    }

    /**
     * Processes provided accelerometer, gyroscope and magnetometer measurements at provided
     * timestamp to obtain a relative leveled attitude.
     *
     * @param accelerometerMeasurement accelerometer measurement to be processed.
     * @param gyroscopeMeasurement gyroscope measurement to be processed.
     * @param magnetometerMeasurement magnetometer measurement to be processed.
     * @param timestamp timestamp when both measurements are assumed to occur. If not provided,
     * this is assumed to be gyroscope timestamp.
     * @return true if a new attitude is estimated, false otherwise.
     */
    fun process(
        accelerometerMeasurement: AccelerometerSensorMeasurement,
        gyroscopeMeasurement: GyroscopeSensorMeasurement,
        magnetometerMeasurement: MagnetometerSensorMeasurement,
        timestamp: Long = gyroscopeMeasurement.timestamp
    ): Boolean {
        val isFirst = updateTimeInterval(timestamp)
        processMeasurement(gyroscopeMeasurement, nedBodyAngularSpeedTriad)
        return if (isFirst) {
            initKalmanErrorCov()
            false
        } else {
            processMeasurement(accelerometerMeasurement, nedBodyAccelerationTriad)
            processMeasurement(magnetometerMeasurement, nedBodyMagneticFluxDensityTriad)

            predict()
            correctAccelerometer()
            //correctMagnetometer()

            //processCovariance()

            true
        }
    }

    /**
     * Resets internal parameter.
     */
    fun reset() {
        previousTimestamp = -1L
        timeIntervalSeconds = 0.0

        // reset Kalman filter matrices
        kalmanStatePrePredicted.initialize(0.0)
        kalmanStatePostPredicted.initialize(0.0)

        kalmanErrorCovPrePredicted.initialize(0.0)
        kalmanErrorCovPostPredicted.initialize(0.0)

        kalmanStateCorrected.initialize(0.0)
        kalmanErrorCovCorrected.initialize(0.0)

        initKalmanErrorCov()

        accelerometerCorrector.reset()
    }

    fun initKalmanErrorCov() {
        val qErrorVar = 0.2
        Matrix.identity(kalmanErrorCovCorrected)
        for (i in 0 until 3) {
            kalmanErrorCovCorrected.setElementAt(i, i, qErrorVar)
        }

        val bgVar = 0.0005
        for (i in 3 until 6) {
            kalmanErrorCovCorrected.setElementAt(i, i, bgVar)
        }

        val baVar = 0.05
        for (i in 6 until 9) {
            kalmanErrorCovCorrected.setElementAt(i, i, baVar)
        }

        kalmanErrorCovPrePredicted.copyFrom(kalmanErrorCovCorrected)
        kalmanErrorCovPostPredicted.copyFrom(kalmanErrorCovCorrected)
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
            timeIntervalSeconds = TimeConverter.nanosecondToSecond(diff.toDouble())
        }
        previousTimestamp = timestamp

        return isFirst
    }

    /**
     * Predicts expected orientation, orientation error, gyroscope and accelerometer biases using
     * current gyroscope measurements and current orientation estimation.
     */
    private fun predict() {
        kalmanStatePrePredicted.copyFrom(kalmanStateCorrected)
        kalmanErrorCovPrePredicted.copyFrom(kalmanErrorCovCorrected)

        gyroscopePredictor.kalmanStatePrePredicted = kalmanStatePrePredicted
        gyroscopePredictor.kalmanErrorCovPrePredicted = kalmanErrorCovPrePredicted
        gyroscopePredictor.previousBodyQ = bodyQ
//        gyroscopePredictor.nedQ = nedQ
        gyroscopePredictor.nedBodyAngularSpeedTriad = nedBodyAngularSpeedTriad
        gyroscopePredictor.timeIntervalSeconds = timeIntervalSeconds

        gyroscopePredictor.predict()

        predictedBodyQ.fromQuaternion(gyroscopePredictor.predictedBodyQ)
//        predictedNedQ.fromQuaternion(gyroscopePredictor.predictedNedQ)
        rg.copyFrom(gyroscopePredictor.rg)
        q.copyFrom(gyroscopePredictor.q)
        qbg.copyFrom(gyroscopePredictor.qbg)
        qba.copyFrom(gyroscopePredictor.qba)
        kalmanStatePostPredicted.copyFrom(gyroscopePredictor.kalmanStatePostPredicted)
        kalmanErrorCovPostPredicted.copyFrom(gyroscopePredictor.kalmanErrorCovPostPredicted)

        kalmanStateCorrected.copyFrom(kalmanStatePostPredicted)
        kalmanErrorCovCorrected.copyFrom(kalmanErrorCovPostPredicted)
        bodyQ.fromQuaternion(predictedBodyQ)
        bodyQ.inverse(nedQ)
        nedQ.normalize()
//        nedQ.fromQuaternion(predictedNedQ)
        // TODO: we need to notify body attitude and thus the inverse?
//        nedQ.inverse(nedAttitude)
        bodyAttitude.fromQuaternion(bodyQ)
        nedAttitude.fromQuaternion(nedQ)
//        convertNedToEcef(nedQ, ecefQ)
//        ecefAttitude.fromQuaternion(ecefQ)
    }

    /**
     * Corrects Kalman filter state using accelerometer measurements.
     */
    private fun correctAccelerometer() {
        accelerometerCorrector.accelerometerNoiseStandardDeviation =
            accelerometerNoiseStandardDeviation
        accelerometerCorrector.gravityNorm = gravityNorm
        // TODO: use nedQ before prediction instead of predictedNedQ?
        accelerometerCorrector.predictedBodyQ = bodyQ
//        accelerometerCorrector.predictedNedQ = nedQ //predictedNedQ
        accelerometerCorrector.nedBodyAccelerationTriad = nedBodyAccelerationTriad
        accelerometerCorrector.kalmanErrorCovPostPredicted = kalmanErrorCovCorrected //kalmanErrorCovPostPredicted
        accelerometerCorrector.kalmanStatePostPredicted = kalmanStateCorrected //kalmanStatePostPredicted

        accelerometerCorrector.correct()

        // get results

        // Kalman filter state error covariance
        kalmanErrorCovAccelerometerCorrected.copyFrom(
            accelerometerCorrector.kalmanErrorCovAccelerometerCorrected
        )

        // Kalman filter state
        kalmanStateAccelerometerCorrected.copyFrom(
            accelerometerCorrector.kalmanStateAccelerometerCorrected
        )

        kalmanStateCorrected.copyFrom(kalmanStateAccelerometerCorrected)
        kalmanErrorCovCorrected.copyFrom(kalmanErrorCovAccelerometerCorrected)

        // attitude
        bodyQ.fromQuaternion(accelerometerCorrector.bodyQ)
        bodyQ.inverse(nedQ)
        nedQ.normalize()
//        nedQ.fromQuaternion(accelerometerCorrector.nedQ)
        bodyAttitude.fromQuaternion(bodyQ)
        nedAttitude.fromQuaternion(nedQ)
//        convertNedToEcef(nedQ, ecefQ)
//        ecefAttitude.fromQuaternion(ecefQ)

        // accelerometer bias
        ba.copyFrom(accelerometerCorrector.ba)

        // gyroscope bias
        bg.copyFrom(accelerometerCorrector.bg)

        if (computeExternalAcceleration) {
            // external acceleration in body coordinates
            /*bodyA.copyFrom(accelerometerCorrector.externalAcceleration)

            // convert boy external acceleration to leveled local NED coordinates respect to Earth
            nedA.getValuesAsMatrix(bodyAMatrix)

            nedQ.toMatrixRotation(nedRotationMatrix)
            nedRotationMatrix.transpose(inverseNedRotationMatrix)
            inverseNedRotationMatrix.multiply(bodyAMatrix, nedAMatrix)
            nedA.setValueCoordinates(nedAMatrix)

            // convert leveled local NED coordinates to ECEF coordinates respect to Earth
            nedToEcefMatrix.multiply(nedAMatrix, ecefAMatrix)
            ecefA.setValueCoordinates(ecefAMatrix)*/
        }

        if (computeCovariances || computeEulerAngles || computeEulerAnglesCovariance) {
            //predictedNedQJacobian.copyFrom(accelerometerCorrector.predictedNedQJacobian)
        }
    }

    /**
     * Corrects Kalman filter state using magnetometer measurements.
     */
    private fun correctMagnetometer() {
        magnetometerCorrector.magnetometerNoiseStandardDeviation =
            magnetometerNoiseStandardDeviation
        magnetometerCorrector.correctedBodyQ = bodyQ
//        magnetometerCorrector.correctedNedQ = nedQ
        magnetometerCorrector.nedBodyMagneticFluxDensityTriad = nedBodyMagneticFluxDensityTriad
        magnetometerCorrector.magneticFluxDensity = magneticFluxDensity
        magnetometerCorrector.kalmanErrorCovAccelerometerCorrected =
            kalmanErrorCovAccelerometerCorrected
        magnetometerCorrector.kalmanStateAccelerometerCorrected = kalmanStateAccelerometerCorrected

        magnetometerCorrector.correct()

        // get results
        kalmanStateCorrected.copyFrom(magnetometerCorrector.kalmanStateMagnetometerCorrected)
        kalmanErrorCovCorrected.copyFrom(magnetometerCorrector.kalmanErrorCovMagnetometerCorrected)
    }

    /**
     * Processes covariances of estimated results.
     */
    private fun processCovariance() {
        if (computeCovariances || computeEulerAngles || computeEulerAnglesCovariance) {
            // process covariances

            // TODO: isPositiveSemiDefinite covariance?

            // take only the part corresponding to ECEF quaternion covariance
            kalmanErrorCovAccelerometerCorrected.getSubmatrix(0, 0, 2, 2, qeAxisCovariance)

            propagateUncertainty(
                qeAxisCovariance,
                qeJacobian,
                tmp1,
                transposedQeJacobian,
                qeCovariance
            )

            predictedNedQJacobian.transpose(transposedPredictedNedQJacobian)

            propagateUncertainty(
                qeCovariance,
                predictedNedQJacobian,
                tmp2,
                transposedPredictedNedQJacobian,
                internalNedQCovariance
            )

            if (computeEulerAngles) {
                if (computeEulerAnglesCovariance) {
                    nedQ.toEulerAngles(internalEulerAngles, eulerJacobian)
                    eulerJacobian.transpose(transposedEulerJacobian)

                    propagateUncertainty(
                        internalNedQCovariance,
                        eulerJacobian,
                        tmp3,
                        transposedEulerJacobian,
                        internalEulerCovariance
                    )
                } else {
                    nedQ.toEulerAngles(internalEulerAngles)
                }
            }
        }
    }

    /**
     * Estimates properties that depend on current location, such as magnetic field dip angle,
     * gravity norm, or ECEF-NED conversion matrix.
     */
    private fun estimatePropertiesDependingOnLocation() {
        val wmmEstimator = this.wmmEstimator
        val location = this.location

        if (location != null) {
            location.toNEDPosition(nedPosition)
            if (wmmEstimator != null) {
                magneticDip = wmmEstimator.getDip(
                    nedPosition, year(calendar)
                )
                magneticDeclination = wmmEstimator.getDeclination(
                    nedPosition, year(calendar)
                )

                wmmEstimator.estimate(nedPosition, year(calendar), nedMagneticFluxDensity)
                nedMagneticFluxDensity.getCoordinatesAsTriad(magneticFluxDensity)
            }

            NEDGravityEstimator.estimateGravity(nedPosition, nedGravity)
            gravityNorm = nedGravity.norm

            CoordinateTransformation.nedToEcefMatrix(
                nedPosition.latitude,
                nedPosition.longitude,
                nedToEcefMatrix
            )
        } else {
            gravityNorm = EARTH_GRAVITY

            Matrix.identity(nedToEcefMatrix)
        }

        nedToEcefQ.fromInhomogeneousMatrix(nedToEcefMatrix)
        nedToEcefQ.normalize()
        nedToEcefQ.conjugate(ecefToNedQ)

        if (wmmEstimator == null) {
            magneticDip = 0.0
            magneticDeclination = 0.0
        }

        nedGravityMatrix.initialize(0.0)
        nedGravityMatrix.setElementAtIndex(2, gravityNorm)
    }

    /**
     * Converts quaternion from NED to ECEF coordinates.
     */
    private fun convertNedToEcef(nedQuaternion: Quaternion, result: Quaternion) {
        nedToEcefQ.multiply(nedQuaternion, tmpQ)
        tmpQ.multiply(ecefToNedQ, result)
    }

    /**
     * Builds World Magnetic Model if needed.
     */
    private fun buildWMMEstimator() {
        wmmEstimator = if (useWorldMagneticModel) {
            val model = worldMagneticModel
            if (model != null) {
                WMMEarthMagneticFluxDensityEstimator(model)
            } else {
                WMMEarthMagneticFluxDensityEstimator()
            }
        } else {
            null
        }
    }

    init {
        // set small values for bias matrices in continuous time process noise covariance matrix
        // to avoid bias components being stuck at a given value
        qbg.multiplyByScalar(BG)
        qba.multiplyByScalar(BG)

        buildWMMEstimator()
        this.location = location

        for (i in 0 until Quaternion.N_ANGLES) {
            qeJacobian.setElementAt(i + 1, i, 1.0)
        }
        qeJacobian.transpose(transposedQeJacobian)
    }

    companion object {
        /**
         * Default variance of the gyroscope output per Hz (or variance at 1 Hz).
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
         * Default standard deviation for magnetometer noise expressed in Teslas (T).
         */
        const val DEFAULT_MAGNETOMETER_NOISE_STANDARD_DEVIATION = 0.5e-6

        /**
         * Average gravity at Earth's sea level expressed in m/s^2.
         */
        const val EARTH_GRAVITY = SensorManager.GRAVITY_EARTH.toDouble()

        /**
         * Default external acceleration covariance matrix estimation method.
         */
        val DEFAULT_EXTERNAL_ACCELERATION_COVARIANCE_MATRIX_ESTIMATION_METHOD =
            ExternalAccelerationCovarianceMatrixEstimationMethod.SUH //.ZEROES //.SABATINI //.SUH

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
         * Small value to avoid process noise bias components being stuck.
         */
        private const val BG = 1e-6

        /**
         * Processes an accelerometer measurement, taking into account their biases (if available).
         * Results will contain accelerometer data expressed in NED body coordinates.
         *
         * @param measurement measurement to be processed.
         * @param nedBodyResult triad where measurement data will be stored in NED coordinates.
         */
        private fun processMeasurement(
            measurement: AccelerometerSensorMeasurement,
            nedBodyResult: AccelerationTriad
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
            ENUtoNEDConverter.convert(valueX, valueY, valueZ, nedBodyResult)
        }

        /**
         * Processes a gyroscope measurement, taking into account their biases (if available).
         * Result will contain gyroscope data expressed in NED body coordinates.
         *
         * @param measurement measurement to be processed.
         * @param nedBodyResult triad where measurement data will be stored in NED coordinates.
         */
        private fun processMeasurement(
            measurement: GyroscopeSensorMeasurement,
            nedBodyResult: AngularSpeedTriad
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
            ENUtoNEDConverter.convert(valueX, valueY, valueZ, nedBodyResult)
        }

        /**
         * Processes a magnetometer measurement, taking into account their hard iron (if available).
         * Measurement will contain magnetometer data expressed in NED body coordinates.
         *
         * @param measurement measurement to be processed.
         * @param nedBodyResult triad where measurement data will be stored in NED coordinates.
         */
        private fun processMeasurement(
            measurement: MagnetometerSensorMeasurement,
            nedBodyResult: MagneticFluxDensityTriad
        ) {
            val bx = measurement.bx.toDouble()
            val by = measurement.by.toDouble()
            val bz = measurement.bz.toDouble()
            val hardIronX = measurement.hardIronX?.toDouble()
            val hardIronY = measurement.hardIronY?.toDouble()
            val hardIronZ = measurement.hardIronZ?.toDouble()

            val valueX = MagneticFluxDensityConverter.microTeslaToTesla(
                if (hardIronX != null) bx - hardIronX else bx
            )
            val valueY = MagneticFluxDensityConverter.microTeslaToTesla(
                if (hardIronY != null) by - hardIronY else by
            )
            val valueZ = MagneticFluxDensityConverter.microTeslaToTesla(
                if (hardIronZ != null) bz - hardIronZ else bz
            )

            // convert to NED coordinates
            ENUtoNEDConverter.convert(valueX, valueY, valueZ, nedBodyResult)
        }

        /**
         * Gets year value with fractional part to be used for estimation using World Magnetic
         * Model.
         *
         * @param calendar containing current timestamp.
         * @param currentDate timestamp when World Magnetic Model will be evaluated. If not defined
         * current timestamp will be used.
         * @return year with fractional part.
         */
        private fun year(calendar: GregorianCalendar, currentDate: Date? = null): Double {
            calendar.timeInMillis = currentDate?.time ?: System.currentTimeMillis()
            return WMMEarthMagneticFluxDensityEstimator.convertTime(calendar)
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
         */
        fun onProcessed(
            processor: KalmanAbsoluteAttitudeProcessor5,
            attitude: Quaternion,
            eulerAngles: DoubleArray?,
            quaternionCovariance: Matrix?,
            eulerAnglesCovariance: Matrix?
        )
    }
}