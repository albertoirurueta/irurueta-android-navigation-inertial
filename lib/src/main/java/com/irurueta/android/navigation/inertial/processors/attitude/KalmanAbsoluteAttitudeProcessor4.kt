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
import android.location.Location
import com.irurueta.algebra.Matrix
import com.irurueta.algebra.Utils
import com.irurueta.android.navigation.inertial.ENUtoNEDConverter
import com.irurueta.android.navigation.inertial.old.KalmanFilter
import com.irurueta.android.navigation.inertial.old.collectors.AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorMeasurement
import com.irurueta.navigation.inertial.calibration.gyroscope.SuhQuaternionStepIntegrator
import com.irurueta.navigation.inertial.calibration.gyroscope.TrawnyQuaternionStepIntegrator
import com.irurueta.navigation.inertial.calibration.gyroscope.YuanQuaternionStepIntegrator
import com.irurueta.android.navigation.inertial.toNEDPosition
import com.irurueta.geometry.Quaternion
import com.irurueta.geometry.Rotation3D
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.NEDPosition
import com.irurueta.navigation.inertial.NEDGravity
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegrator
import com.irurueta.navigation.inertial.estimators.NEDGravityEstimator
import com.irurueta.navigation.inertial.wmm.EarthMagneticFluxDensityEstimator
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
 */
class KalmanAbsoluteAttitudeProcessor4(
    location: Location? = null,
    var currentDate: Date? = null,
    var computeExternalAcceleration: Boolean = true,
    var computeEulerAngles: Boolean = true,
    val computeCovariances: Boolean = true,
    val computeEulerAnglesCovariance: Boolean = true,
    val fixMagnetometerMeasurements: Boolean = true,
    phiMatrixMethod: PhiMatrixMethod = PhiMatrixMethod.APPROXIMATED,
    processNoiseCovarianceMethod: ProcessNoiseCovarianceMethod = ProcessNoiseCovarianceMethod.BETTER,
    quaternionStepIntegrator: QuaternionStepIntegratorType = QuaternionStepIntegratorType.RUNGE_KUTTA,
    val gyroscopeNoisePsd: Double = DEFAULT_GYROSCOPE_NOISE_PSD,
    val accelerometerNoiseStandardDeviation: Double = DEFAULT_ACCELEROMETER_NOISE_STANDARD_DEVIATION,
    val magnetometerNoiseStandardDeviation: Double = DEFAULT_MAGNETOMETER_NOISE_STANDARD_DEVIATION,
    val processorListener: OnProcessedListener? = null
) {

    /**
     * Gets or sets current device location.
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
     * Estimated device attitude expressed in ECEF coordinates respect to Earth.
     */
    val ecefAttitude = Quaternion()

    /**
     * Estimated device attitude expressed in leveled local NED coordinates respect to Earth.
     */
    val nedAttitude = Quaternion()

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
     * This is only available if [computeCovariances] is true.
     */
    val nedAttitudeCovariance: Matrix?
        get() = if (computeCovariances) {
            internalNedQCovariance
        } else {
            null
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
     * Estimated gyroscope bias.
     */
    val bg = Matrix(AngularSpeedTriad.COMPONENTS, 1)

    /**
     * Estimated accelerometer bias.
     */
    val ba = Matrix(AccelerationTriad.COMPONENTS, 1)

    /**
     * Column vector representing gravity in local leveled NED coordinates.
     */
    private val nedGravityMatrix: Matrix = Matrix(AccelerationTriad.COMPONENTS, 1)

    /**
     * Column vector representing gravity in body coordinates taking into account current
     * orientation.
     */
    private val bodyGravityMatrix: Matrix = Matrix(AccelerationTriad.COMPONENTS, 1)

    /**
     * Skew matrix representation of gravity expressed in body coordinates.
     */
    private val skewBodyGravityMatrix: Matrix =
        Matrix(AccelerationTriad.COMPONENTS, AccelerationTriad.COMPONENTS)

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
     * Contains accelerometer data expressed in NED coordinates respect to body.
     * This is reused for performance reasons.
     */
    private val nedBodyAccelerationTriad: AccelerationTriad = AccelerationTriad()

    /**
     * Contains gyroscope data expressed in NED coordinates respect to body.
     */
    private val nedBodyAngularSpeedTriad: AngularSpeedTriad = AngularSpeedTriad()

    /**
     * Contains previous gyroscope data expressed in NED coordinates respect to body.
     */
    private val previousNedBoyAngularSpeedTriad: AngularSpeedTriad = AngularSpeedTriad()

    /**
     * Contains magnetometer data expressed in NED coordinates respect to body.
     */
    private val nedBodyMagneticFluxDensityTriad: MagneticFluxDensityTriad =
        MagneticFluxDensityTriad()

    /**
     * Measured NED acceleration respect to body represented as a column matrix.
     */
    private val measuredAccelerometerMatrix: Matrix = Matrix(AccelerationTriad.COMPONENTS, 1)

    /**
     * Column matrix containing measured accelerometer error.
     */
    private val accelerometerError: Matrix = Matrix(AccelerationTriad.COMPONENTS, 1)

    /**
     * Measured NED magnetic field respect to body represented as a column matrix.
     */
    private val measuredMagneticFluxDensityMatrix: Matrix =
        Matrix(MagneticFluxDensityTriad.COMPONENTS, 1)

    /**
     * Corrected measured NED magnetic field respect to body so that only components defining yaw
     * are considered.
     */
    private val fixedMagneticFluxDensityMatrix: Matrix =
        Matrix(MagneticFluxDensityTriad.COMPONENTS, 1)

    /**
     * Column matrix containing measured magnetometer error.
     */
    private val magnetometerError: Matrix = Matrix(MagneticFluxDensityTriad.COMPONENTS, 1)

    /**
     * NED angular speed represented as a matrix.
     */
    private val angularSpeedMatrix: Matrix = Matrix(AngularSpeedTriad.COMPONENTS, 1)

    /**
     * Computes matrix that relates previous and predicted Kalman filter state.
     */
    private val phiMatrixEstimator: PhiMatrixEstimator =
        PhiMatrixEstimator.create(method = phiMatrixMethod)

    /**
     * Integrates continuous time process noise covariance matrix.
     */
    private val processNoiseCovarianceIntegrator: ProcessNoiseCovarianceIntegrator =
        ProcessNoiseCovarianceIntegrator.create(method = processNoiseCovarianceMethod)

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
     * Contains previous step leveled local NED attitude respect to Earth.
     */
    private val previousNedQ: Quaternion = Quaternion()

    /**
     * Contains predicted leveled local NED attitude respect to Earth for next step.
     * Converts from leveled NED local coordinates to body NED coordinates.
     */
    private val predictedNedQ: Quaternion = Quaternion()

    /**
     * Quaternion containing error component of estimation.
     */
    private val errorQ: Quaternion = Quaternion()

    /**
     * Contains corrected leveled local NED attitude respect to Earth.
     */
    private val nedQ: Quaternion = Quaternion()

    /**
     * Contains corrected ECEF attitude respect to Earth.
     */
    private val ecefQ: Quaternion = Quaternion()

    /**
     * Contains predicted NED attitude for next step in matrix form.
     * Converts from leveled NED local coordinates to body NED coordinates.
     */
    private val predictedNedRotationMatrix: Matrix =
        Matrix(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)

    /**
     * Contains predicted inverse NED attitude.
     * Converts from body NED coordinates to leveled NED local coordinates.
     */
    private val inversePredictedNedRotationMatrix: Matrix =
        Matrix(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)

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
     * Column matrix containing measurement provided to internal Kalman filter during correction
     * stage.
     */
    private val kalmanMeasurement: Matrix = Matrix(KALMAN_FILTER_MEASUREMENT_PARAMETERS, 1)

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

    /**
     * Contains expected magnetic flux density expressed in leveled local NED coordinates respect
     * to Earth at current device location.
     */
    private val nedMagneticFluxDensity: NEDMagneticFluxDensity = NEDMagneticFluxDensity()

    /**
     * Contains [nedMagneticFluxDensity] expressed as a column matrix.
     */
    private val nedMagneticFluxDensityMatrix: Matrix =
        Matrix(MagneticFluxDensityTriad.COMPONENTS, 1)

    /**
     * Columns vector containing magnetic flux density in body coordinates taking into account
     * current orientation.
     */
    private val bodyMagneticFluxDensityMatrix: Matrix =
        Matrix(MagneticFluxDensityTriad.COMPONENTS, 1)

    /**
     * Skew matrix representation of magnetic flux density expressed in body coordinates.
     */
    private val skewBodyMagneticFluxDensityMatrix: Matrix =
        Matrix(MagneticFluxDensityTriad.COMPONENTS, MagneticFluxDensityTriad.COMPONENTS)

    /**
     * Timestamp of previous sample expressed in nanoseconds.
     */
    private var previousTimestamp: Long = -1L

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
     * Internal estimated external acceleration applied on the device expressed in NED coordinates
     * respect to device body.
     */
    private val bodyA = AccelerationTriad()

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
     * Jacobian of [predictedNedQ] respect to estimated [errorQ].
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
     * Estimated covariance of leveled local NED attitude.
     */
    private val internalNedQCovariance: Matrix = Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS)

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
     * Temporal matrix to estimate covariance of euler angles of leveled local NED attitude.
     */
    private val tmp3: Matrix = Matrix(Quaternion.N_ANGLES, Quaternion.N_PARAMS)

    /**
     * Covariance of estimated Euler angles.
     */
    private val internalEulerCovariance: Matrix = Matrix(Quaternion.N_ANGLES, Quaternion.N_ANGLES)

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
            false
        } else {
            processMeasurement(accelerometerMeasurement, nedBodyAccelerationTriad)
            processMeasurement(magnetometerMeasurement, nedBodyMagneticFluxDensityTriad)

            predict()
            correct()

            processCovariance()

            if (computeExternalAcceleration) {
                // accelerometer measurements follow expression below:
                // ya = C(q) * g + ba + na + ab
                // where:
                // - ya: is the sensed accelerometer measurement
                // - C(q): is the estimated quaternion in matrix form
                // - q: is the column vector representing vertical gravity acceleration respect to
                // Earth as [0 0 gnorm]'
                // - ba: is the estimated accelerometer bias (assumed nearly constant)
                // - na: is the accelerometer noise (assumed to be Gaussian since Kalman filter
                // finds optimal solutions for Gaussian noises)
                // - ab: is the external acceleration applied to the device body.

                // Then, external acceleration (+ noise that cannot be isolated) is:
                // ab + na = ya - C(q)*q - ba

                nedQ.toMatrixRotation(nedRotationMatrix)
                nedRotationMatrix.transpose(inverseNedRotationMatrix)

                nedRotationMatrix.multiply(nedGravityMatrix, bodyGravityMatrix)

                measuredAccelerometerMatrix.subtract(bodyGravityMatrix, bodyAMatrix)
                bodyAMatrix.subtract(ba)

                bodyA.setValueCoordinates(ba)

                // convert body NED coordinates to leveled local NED coordinates respect to Earth
                inverseNedRotationMatrix.multiply(ba, nedAMatrix)
                nedA.setValueCoordinates(nedAMatrix)

                // convert leveled local NED coordinates to ECEF coordinates respect to Earth
                nedToEcefMatrix.multiply(nedAMatrix, ecefAMatrix)
                ecefA.setValueCoordinates(ecefAMatrix)
            }

            nedAttitude.fromQuaternion(nedQ)
            ecefAttitude.fromQuaternion(ecefQ)

            processorListener?.onProcessed(
                this,
                nedAttitude,
                eulerAngles,
                nedAttitudeCovariance,
                eulerAnglesCovariance
            )
            true
        }
    }

    /**
     * Resets internal parameter.
     */
    fun reset() {
        previousTimestamp = -1L
        timeIntervalSeconds = 0.0

        // reset attitude estimation
        nedQ.a = 1.0
        nedQ.b = 0.0
        nedQ.c = 0.0
        nedQ.d = 0.0
        previousNedQ.fromQuaternion(nedQ)

        // reset internal Kalman filter
        kalmanFilter.statePre.initialize(0.0)
        kalmanFilter.statePost.initialize(0.0)

        kalmanFilter.errorCovPre.initialize(0.0)
        kalmanFilter.errorCovPost.initialize(0.0)
    }

    /**
     * Gets estimated gyroscope bias.
     *
     * @param result instance where result will be stored.
     */
    fun getGyroscopeBias(result: AngularSpeedTriad) {
        result.setValueCoordinates(bg)
    }

    /**
     * Gets estimated accelerometer bias.
     *
     * @param result instance where result will be stored.
     */
    fun getAccelerometerBias(result: AccelerationTriad) {
        result.setValueCoordinates(ba)
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
     * Processes covariances of estimated results.
     */
    private fun processCovariance() {
        if (computeCovariances || computeEulerAngles || computeEulerAnglesCovariance) {
            // process covariances
            val errorCov = kalmanFilter.errorCovPost

            // TODO: isPositiveSemiDefinite covariance?

            // take only the part corresponding to ECEF quaternion covariance
            errorCov.getSubmatrix(0, 0, 2, 2, qeAxisCovariance)

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
     * Predicts expected orientation, orientation error, gyroscope and accelerometer biases using
     * current gyroscope measurements and current orientation estimation.
     */
    private fun predict() {
        previousNedQ.fromQuaternion(nedQ)

        // Propagate quaternion using a step integrator
        val previousWx = previousNedBoyAngularSpeedTriad.valueX
        val previousWy = previousNedBoyAngularSpeedTriad.valueY
        val previousWz = previousNedBoyAngularSpeedTriad.valueZ
        val currentWx = nedBodyAngularSpeedTriad.valueX
        val currentWy = nedBodyAngularSpeedTriad.valueY
        val currentWz = nedBodyAngularSpeedTriad.valueZ
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
        computePhiAndNoiseCovarianceMatrices()

        val transitionMatrix = kalmanFilter.transitionMatrix
        transitionMatrix.copyFrom(phi)

        val processNoiseCov = kalmanFilter.processNoiseCov
        processNoiseCov.copyFrom(q)

        kalmanFilter.predict()
    }

    /**
     * Corrects orientation estimation an updates local leveled NED orientation respect to Earth
     * by taking into account accelerometer and magnetometer measurements.
     */
    private fun correct() {
        setupMeasurementMatrix()

        // accelerometer error
        nedBodyAccelerationTriad.getValuesAsMatrix(measuredAccelerometerMatrix)
        measuredAccelerometerMatrix.subtract(bodyGravityMatrix, accelerometerError)

        // magnetometer error
        nedBodyMagneticFluxDensityTriad.getValuesAsMatrix(measuredMagneticFluxDensityMatrix)

        if (fixMagnetometerMeasurements) {
            // convert measured magnetic field from body to leveled NED coordinate
            inversePredictedNedRotationMatrix.multiply(
                measuredMagneticFluxDensityMatrix,
                fixedMagneticFluxDensityMatrix
            )

            // set z component to zero (only horizontal plane components are taken into account to
            // indicate yaw direction)
            fixedMagneticFluxDensityMatrix.setElementAtIndex(2, 0.0)

            // convert back to body coordinates
            predictedNedRotationMatrix.multiply(
                fixedMagneticFluxDensityMatrix,
                measuredMagneticFluxDensityMatrix
            )
        }

        measuredMagneticFluxDensityMatrix.subtract(bodyMagneticFluxDensityMatrix, magnetometerError)

        kalmanMeasurement.setSubmatrix(0, 0, 2, 0, accelerometerError)
        kalmanMeasurement.setSubmatrix(3, 0, 5, 0, magnetometerError)
        val statePost = kalmanFilter.correct(kalmanMeasurement)

        errorQ.a = 1.0
        errorQ.b = statePost.getElementAtIndex(0)
        errorQ.c = statePost.getElementAtIndex(1)
        errorQ.d = statePost.getElementAtIndex(2)

        // qk+1 = qk * qe
        Quaternion.product(predictedNedQ, errorQ, nedQ, predictedNedQJacobian, null)

        // normalize quaternion
        nedQ.normalize()

        // reset error components of state vector
        for (i in 0 until AngularSpeedTriad.COMPONENTS) {
            statePost.setElementAtIndex(i, 0.0)
        }

        // copy estimated biases
        bg.setSubmatrix(0, 0, 2, 0, statePost, 3, 0, 5, 0)
        ba.setSubmatrix(0, 0, 2, 0, statePost, 6, 0, 8, 0)
    }

    /**
     * Builds measurement matrix for internal Kalman filter taking into account
     * current orientation, gravity and Earth magnetic field at current device location.
     */
    private fun setupMeasurementMatrix() {
        // setup measurement matrix
        val measurementMatrix = kalmanFilter.measurementMatrix
        measurementMatrix.initialize(0.0)

        // accelerometer

        predictedNedQ.normalize()
        predictedNedQ.toMatrixRotation(predictedNedRotationMatrix)
        predictedNedRotationMatrix.transpose(inversePredictedNedRotationMatrix)

        predictedNedRotationMatrix.multiply(nedGravityMatrix, bodyGravityMatrix)
        Utils.skewMatrix(bodyGravityMatrix, skewBodyGravityMatrix)
        skewBodyGravityMatrix.multiplyByScalar(2.0)

        measurementMatrix.setSubmatrix(0, 0, 2, 2, skewBodyGravityMatrix)
        Matrix.identity(identity3)
        measurementMatrix.setSubmatrix(0, 6, 2, 8, identity3)

        // magnetometer

        // obtain expected magnetic field in leveled NED coordinates respect to Earth
        // for current measured magnitude
        val magneticFluxDensityMagnitude = nedBodyMagneticFluxDensityTriad.norm
        EarthMagneticFluxDensityEstimator.estimate(
            magneticFluxDensityMagnitude,
            magneticDeclination,
            magneticDip,
            nedMagneticFluxDensity
        )
        nedMagneticFluxDensity.asMatrix(nedMagneticFluxDensityMatrix)

        predictedNedRotationMatrix.multiply(
            nedMagneticFluxDensityMatrix,
            bodyMagneticFluxDensityMatrix
        )
        Utils.skewMatrix(bodyMagneticFluxDensityMatrix, skewBodyMagneticFluxDensityMatrix)
        skewBodyMagneticFluxDensityMatrix.multiplyByScalar(2.0)

        measurementMatrix.setSubmatrix(3, 0, 5, 2, skewBodyMagneticFluxDensityMatrix)
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

        if (wmmEstimator == null) {
            magneticDip = 0.0
            magneticDeclination = 0.0
        }

        nedGravityMatrix.initialize(0.0)
        nedGravityMatrix.setElementAtIndex(2, gravityNorm)
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

    private fun updateCovarianceMeasurementMatrix() {
        Matrix.identity(rg)
        val gyroVariance = gyroscopeNoisePsd / timeIntervalSeconds
        rg.multiplyByScalar(gyroVariance)
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

    /**
     * Indicates type of quaternion integrator step. Different types exist with different levels of
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
         * Kalman filter's number of measurement parameters.
         * Filter uses measurements of:
         * - Accelerometer respect to body expressed in NED coordinates. 3x1 vector.
         * - Magnetometer respect to body expressed in NED coordinates. 3x1 vector.
         */
        private const val KALMAN_FILTER_MEASUREMENT_PARAMETERS = 6

        /**
         * Average gravity at Earth's sea level expressed in m/s^2.
         */
        const val EARTH_GRAVITY = SensorManager.GRAVITY_EARTH.toDouble()

        /**
         * Small value to avoid process noise bias components being stuck.
         */
        const val BG = 1e-6

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
         * Creates a quaternion step integrator to integrate angular speed measurements to predict
         * a new orientation when new gyroscope measurements arrive.
         *
         * @param type type of step integrator to create.
         * @return created quaternion step integrator.
         */
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
            processor: KalmanAbsoluteAttitudeProcessor4,
            attitude: Quaternion,
            eulerAngles: DoubleArray?,
            quaternionCovariance: Matrix?,
            eulerAnglesCovariance: Matrix?
        )
    }
}