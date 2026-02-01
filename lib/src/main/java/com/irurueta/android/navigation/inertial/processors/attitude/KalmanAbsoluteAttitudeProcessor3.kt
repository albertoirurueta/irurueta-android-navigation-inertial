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
import com.irurueta.algebra.ArrayUtils
import com.irurueta.algebra.Matrix
import com.irurueta.algebra.Utils
import com.irurueta.android.navigation.inertial.ENUtoNEDConverter
import com.irurueta.android.navigation.inertial.old.KalmanFilter
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
import com.irurueta.navigation.inertial.calibration.Triad
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegrator
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegratorType
import com.irurueta.navigation.inertial.estimators.NEDGravityEstimator
import com.irurueta.navigation.inertial.wmm.WMMEarthMagneticFluxDensityEstimator
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel
import com.irurueta.units.AccelerationUnit
import com.irurueta.units.AngularSpeedUnit
import com.irurueta.units.MagneticFluxDensityConverter
import com.irurueta.units.MagneticFluxDensityUnit
import com.irurueta.units.TimeConverter
import java.util.Date
import java.util.GregorianCalendar
import kotlin.math.cos
import kotlin.math.pow
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Class.
 *
 * @param location Gets or sets current device location. If null, it is assumed that device is
 * located at magnetic field equator (with zero dip and declination angles). This is only taken into
 * account if [useWorldMagneticModel] is true.
 * @param currentDate Timestamp being used when World Magnetic Model is evaluated to obtain current
 * magnetic dip and declination angles. This is only taken into account if [useWorldMagneticModel]
 * is true. If not defined, current date is assumed.
 */
class KalmanAbsoluteAttitudeProcessor3(
    location: Location? = null,
    var currentDate: Date? = null,
    var currentTemperature: Double = DEFAULT_TEMPERATURE_CELSIUS,
    val computeEulerAngles: Boolean = true,
    val computeCovariances: Boolean = true,
    val computeEcefAttitudeCovariance: Boolean = true,
    val computeEulerAnglesCovariance: Boolean = true,
    val computeEcefAccelerationCovariance: Boolean = true,
    val computeNedRotationAxisCovariance: Boolean = true,
    val gyroscopeNoisePsd: Double = DEFAULT_GYROSCOPE_NOISE_PSD,
    val accelerometerNoiseStandardDeviation: Double = DEFAULT_ACCELEROMETER_NOISE_STANDARD_DEVIATION,
    magnetometerNoiseStandardDeviation: Double = DEFAULT_MAGNETOMETER_NOISE_STANDARD_DEVIATION,
    temperatureStandardDeviation: Double = DEFAULT_TEMPERATURE_NOISE_STANDARD_DEVIATION,
    quaternionStepIntegratorType: QuaternionStepIntegratorType =
        QuaternionStepIntegrator.DEFAULT_TYPE,
    val freeFallThreshold: Double = DEFAULT_FREE_FALL_THRESHOLD,
    val symmetricThreshold: Double = SYMMETRIC_THRESHOLD,
    val processorListener: OnProcessedListener? = null,
    val numericalInstabilitiesListener: OnNumericalInstabilitiesDetectedListener? = null
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
     * At magnetic equator dip angle is zero, at North magnetic pole is +90º, and at South magnetic
     * pose is -90º.
     * At the north hemisphere dip angle is positive (magnetic field points downwards), indicating
     * the vertical deviation of magnetic field respect local Earth surface at a given position
     * (latitude, longitude and height)
     * At the south hemisphere dip angle is negative (magnetic field points upwards), indicating
     * the vertical deviation of magnetic field respect local Earth surface at a given position
     * (latitude, longitude and height)
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
     * Contains estimated linear acceleration applied to the device (which is equal to the sensed
     * specific force minus the gravity component) expressed in ECEF coordinates.
     * By integrating linear acceleration speed can be estimated, and by double
     * integration, ECEF position can be estimated as well.
     */
    val ecefAcceleration = AccelerationTriad()

    /**
     * Estimated ECEF attitude.
     */
    val ecefAttitude = Quaternion()

    /**
     * Gets current local body attitude expressed in NED coordinate system.
     */
    val nedAttitude = Quaternion()

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
     * Error covariance of estimated quaternion attitude expressed in ECEF coordinates.
     * This is only available if [computeCovariances] and [computeEcefAttitudeCovariance] are true.
     */
    val ecefAttitudeCovariance: Matrix?
        get() = if (computeCovariances && computeEcefAttitudeCovariance) {
            internalEcefQuaternionCovariance
        } else {
            null
        }

    /**
     * Error covariance of estimate Euler angles for local NED attitude.
     * This is only available if [computeCovariances] and [computeEulerAnglesCovariance] are true.
     */
    val eulerAnglesCovariance: Matrix?
        get() = if (computeCovariances && computeEulerAnglesCovariance) {
            internalEulerAnglesCovariance
        } else {
            null
        }

    /**
     * Error covariance of estimated ECEF acceleration state.
     * This is only available if [computeCovariances and [computeEcefAccelerationCovariance]] are
     * true.
     */
    val ecefAccelerationCovariance: Matrix?
        get() = if (computeCovariances && computeEcefAccelerationCovariance) {
            internalAccelerationCovariance
        } else {
            null
        }

    /**
     * Error covariance of estimated local body rotation axis expressed in NED coordinates.
     * This is only available if [computeCovariances] and [computeNedRotationAxisCovariance] are true.
     */
    val nedRotationAxisCovariance: Matrix?
        get() = if (computeCovariances && computeNedRotationAxisCovariance) {
            internalNedRotationAxisCovariance
        } else {
            null
        }

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
     * Contains local body accelerometer data expressed in ENU coordinates.
     * This is reused for performance reasons.
     */
    private val enuAcceleration: AccelerationTriad = AccelerationTriad()

    /**
     * Contains local body accelerometer data expressed in NED coordinates.
     * This is reused for performance reasons.
     */
    private val nedAcceleration: AccelerationTriad = AccelerationTriad()

    /**
     * Contains local body magnetometer data expressed in ENU coordinates.
     * This is reused for performance reasons.
     */
    private val enuMagneticFluxDensity: MagneticFluxDensityTriad = MagneticFluxDensityTriad()

    /**
     * Contains local body magnetometer data expressed in NED coordinates.
     * This is reused for performance reasons.
     */
    private val nedMagneticFluxDensity: MagneticFluxDensityTriad = MagneticFluxDensityTriad()

    /**
     * Contains local body gyroscope data expressed in ENU coordinates.
     * This is reused for performance reasons.
     */
    private val enuAngularSpeed: AngularSpeedTriad = AngularSpeedTriad()

    /**
     * Contains local body gyroscope data expressed in NED coordinates.
     * This is reused for performance reasons.
     */
    private val nedAngularSpeed: AngularSpeedTriad = AngularSpeedTriad()

    /**
     * Contains previous local body gyroscope data expressed in NED coordinates.
     * This is reused for performance reasons.
     */
    private val previousNedAngularSpeed: AngularSpeedTriad = AngularSpeedTriad()

    /**
     * Array containing unit vector pointing east. East direction can be obtained from the cross
     * product of north and up vectors.
     * This is reused for performance reasons.
     */
    private val east: DoubleArray = DoubleArray(COMPONENTS)

    /**
     * Array containing unit vector pointing north. When there are no magnetic interferences, ENU
     * magnetometer data points towards geomagnetic north pole with a certain declination and dip
     * angles (known by the World Magnetic Model).
     * This is reused for performance reasons.
     */
    private val north: DoubleArray = DoubleArray(COMPONENTS)

    /**
     * Array containing unit vector pointing upwards. When device remains static, ENU accelerometer
     * data points upwards.
     * This is reused for performance reasons.
     */
    private val up: DoubleArray = DoubleArray(COMPONENTS)

    /**
     * Rotation matrix containing current attitude expressed in ENU coordinates.
     * This is reused for performance reasons.
     */
    private val r = Matrix(COMPONENTS, COMPONENTS)

    /**
     * Internal estimated linear acceleration.
     * This is reused for performance reasons
     */
    private val ecefA = AccelerationTriad()

    /**
     * Quaternion expressed in ECEF coordinates respect Earth.
     * This is reused for performance reasons.
     */
    private val ecefQ = Quaternion()

    /**
     * Quaternion expressed in local NED coordinates respect the body.
     * This is reused for performance reasons.
     */
    private val nedQ = Quaternion()

    /**
     * Estimated local ENU reference attitude obtained from accelerometer and magnetometer.
     */
    private val refEnuQ = Quaternion()

    /**
     * Estimated local NED reference attitude obtained from accelerometer and magnetometer.
     */
    private val refNedQ = Quaternion()

    /**
     * Estimated ECEF reference attitude obtained from accelerometer and magnetometer.
     */
    private val refEcefQ = Quaternion()

    /**
     * Quaternion containing expected attitude after integration.
     * This is reused for performance reasons.
     */
    private val expectedNedQ = Quaternion()

    /**
     * Quaternion containing expected attitude after integration.
     * This is reused for performance reasons.
     */
    private val expectedEcefQ = Quaternion()

    /**
     * Inverse of quaternion [nedQ].
     * This is reused for performance reasons.
     */
    private val invQ = Quaternion()

    /**
     * Variation of attitude between [ecefQ] and [expectedEcefQ].
     * This is reused for performance reasons.
     */
    private val ecefDeltaQ = Quaternion()

    /**
     * Variation of attitude between [nedQ] and [expectedNedQ].
     * This is reused for performance reasons.
     */
    private val nedDeltaQ = Quaternion()

    /**
     * Contains deltaQ expressed in quaternion matrix form to be used for matrix multiplication.
     * This is reused for performance reasons.
     */
    private val deltaQMatrix = Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS)

    /**
     * Rotation matrix to convert ECEF coordinates to local NED ones.
     * This is reused for performance reasons.
     */
    private val ecefToNedMatrix = Matrix(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)

    /**
     * Rotation quaternion to convert local NED coordinates to ECF ones.
     * This is reused for performance reasons.
     */
    private val nedToEcefQ = Quaternion()

    /**
     * Rotation quaternion to convert ECEF coordinates to local NED ones.
     * This is reused for performance reasons.
     */
    private val ecefToNedQ = Quaternion()

    /**
     * Temporary quaternion used internally.
     */
    private val tmpQ = Quaternion()

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
     * Sub-matrix relating current temperature and acceleration respect to body.
     * This is reused for performance reasons.
     */
    private val h14 = Matrix(COMPONENTS, 1)

    /**
     * Sub-matrix relating quaternion respect to Earth and magnetic field density respect to body.
     * This is reused for performance reasons.
     */
    private val h22 = Matrix(COMPONENTS, Quaternion.N_PARAMS)

    /**
     * Sub-matrix relating current temperature and magnetic field density respect to body.
     * This is reused for performance reasons.
     */
    private val h24 = Matrix(COMPONENTS, 1)

    /**
     * Sub-matrix relating body nod attitude rotation axis and body angular speed.
     * This is reused for performance reasons.
     */
    private val h33 = Matrix(COMPONENTS, COMPONENTS)

    /**
     * Sub-matrix relating current temperature and boy ned attitude rotation axis.
     * This is reused for performance reasons.
     */
    private val h34 = Matrix(COMPONENTS, 1)

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
     * Error covariance of quaternion attitude expressed in ECEF coordinates.
     * This is used internally and reused for performance reasons.
     */
    private val internalEcefQuaternionCovariance = Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS)

    /**
     * Error covariance of estimated Euler angles for local NED attitude.
     * This is used internally and reused for performance reasons.
     */
    private val internalEulerAnglesCovariance = Matrix(Quaternion.N_ANGLES, Quaternion.N_ANGLES)

    /**
     * Error covariance of Kalman filter state accelerometer measurement values.
     * This is used internally and reused for performance reasons.
     */
    private val internalAccelerationCovariance = Matrix(COMPONENTS, COMPONENTS)

    /**
     * Error covariance of Kalman filter state rotation axis expressed in local NED coordinates.
     * This is used internally and reused for performance reasons.
     */
    private val internalNedRotationAxisCovariance = Matrix(COMPONENTS, COMPONENTS)

    /**
     * Euler angles of estimated NED attitude.
     * This is used internally and reused for performance reasons.
     */
    private val internalEulerAngles = DoubleArray(Quaternion.N_ANGLES)

    /**
     * Jacobian of conversion of NED quaternion attitude into Euler angles.
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
     * Contains gyroscope variance obtained on each measurement
     */
    private var gyroVariance = 0.0

    /**
     * Contains accelerometer variance expressed in (m/s^2)^2.
     */
    private val accelerometerVariance =
        accelerometerNoiseStandardDeviation * accelerometerNoiseStandardDeviation

    /**
     * Contains temperature variance expressed in squared Celsius (C^2).
     */
    private val temperatureVariance = temperatureStandardDeviation * temperatureStandardDeviation

    /**
     * Contains magnetometer variance expressed in T^2.
     */
    private val magnetometerVariance =
        magnetometerNoiseStandardDeviation * magnetometerNoiseStandardDeviation

    /**
     * Processes provided synced accelerometer, gyroscope and magnetometer measurement to obtain a
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
        processMeasurement(gyroscopeMeasurement, enuAngularSpeed, nedAngularSpeed)
        val result = if (isFirst) {
            false
        } else {
            processMeasurement(accelerometerMeasurement, enuAcceleration, nedAcceleration)
            processMeasurement(
                magnetometerMeasurement,
                enuMagneticFluxDensity,
                nedMagneticFluxDensity
            )

            return if (!initialized) {
                estimateReferenceAttitude()
                //copy reference into attitude
                nedQ.fromQuaternion(refNedQ)
                ecefQ.fromQuaternion(refEcefQ)

                initKalmanFilter()

                initialized = true
                false
            } else {
                // set transition matrix for current state
                updateTransitionMatrix()
                updateMeasurementMatrix()
                updateProcessNoiseCovariance()

                // predict using current transition matrix
                val statePre = kalmanFilter.predict()

                val accelerometerNorm = nedAcceleration.norm
                val isFreeFall = accelerometerNorm < freeFallThreshold

                val state = if (isFreeFall) {
                    // device is in free fall
                    kalmanFilter.statePost.copyFrom(statePre)
                    statePre
                } else {
                    // update Kalman filter measurement
                    updateMeasurement()
                    updateMeasurementNoiseCovariance()

                    kalmanFilter.correct(m)
                }

                // copy from state
                copyFilterStateToEcefAcceleration(state, ecefA)
                fixStateAndCopyToEcefQuaternion(state, ecefQ)
//                copyFilterStateToDeltaQuaternion(state, nedDeltaQ)

                // convert ecefQ to NED
                convertEcefToNed(ecefQ, nedQ)
                //fixStateAndCopyToNedQuaternion(state, nedQ)
                currentTemperature = getTemperatureFromFilterState(state)

                processCovariance(isFreeFall)

                ecefAcceleration.copyFrom(ecefA)
                nedAttitude.fromQuaternion(nedQ)
                ecefAttitude.fromQuaternion(ecefQ)

                processorListener?.onProcessed(
                    this,
                    nedAttitude,
                    eulerAngles,
                    null, //ecefAttitudeCovariance,
                    null, //eulerAnglesCovariance,
                    ecefAccelerationCovariance,
                    nedRotationAxisCovariance
                )

                true
            }
        }

        nedAngularSpeed.copyTo(previousNedAngularSpeed)
        return false
    }

    /**
     * Resets internal parameters.
     */
    fun reset() {
        previousTimestamp = -1L
        gyroscopeIntervalSeconds = 0.0
        ecefA.setValueCoordinates(0.0, 0.0, 0.0)
        nedDeltaQ.a = 1.0
        nedDeltaQ.b = 0.0
        nedDeltaQ.c = 0.0
        nedDeltaQ.d = 0.0
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
     * Estimate local NED reference attitude based on available accelerometer and magnetometer data,
     * and stores result in [refNedQ] and [refEcefQ].
     */
    private fun estimateReferenceAttitude() {
        enuAcceleration.getValuesAsArray(up)
        normalize(up)
        enuMagneticFluxDensity.getValuesAsArray(north)
        normalize(north)

        crossProduct(north, up, east)
        normalize(east)

        crossProduct(up, east, north)

        r.setSubmatrix(0, 0, 2, 0, east)
        r.setSubmatrix(0, 1, 2, 1, north)
        r.setSubmatrix(0, 2, 2, 2, up)
        refEnuQ.fromInhomogeneousMatrix(r)
        refEnuQ.normalize()

        // convert attitude to NED coordinates
        ENUtoNEDConverter.convert(refEnuQ, refNedQ)
        refNedQ.normalize()

        // convert refNedQ to ECEF coordinates
        convertNedToEcef(refNedQ, refEcefQ)
    }

    /**
     * Initializes Kalman filter by assuming that state is equal to initial attitude,
     * process noise covariance is set depending on known gyroscope calibration parameters, error
     * covariance are initialized to zero, and measurement matrix is set to make proper conversion
     * between quaternion attitudes and unitary up direction (based on accelerometer measurements).
     */
    private fun initKalmanFilter() {
        ecefQ.normalize()
        nedQ.normalize()
        nedDeltaQ.normalize()

        // update state with initial attitude
        val statePre = kalmanFilter.statePre
        copyEcefAccelerationToFilterState(ecefA, statePre)
        copyEcefQuaternionToFilterState(ecefQ, statePre)
        copyDeltaQuaternionToFilterState(nedDeltaQ, statePre)

        val statePost = kalmanFilter.statePost
        copyEcefAccelerationToFilterState(ecefA, statePost)
        copyEcefQuaternionToFilterState(ecefQ, statePost)
        copyDeltaQuaternionToFilterState(nedDeltaQ, statePost)

        // initialize error covariance a priori and posteriori to zero
        initErrorCovariances()
    }

    /**
     * Updates measurement noise covariance based on accelerometer and gyroscope variances.
     */
    private fun updateMeasurementNoiseCovariance() {
        val measurementNoiseCov = kalmanFilter.measurementNoiseCov
        for (i in 0 until 3) {
            measurementNoiseCov.setElementAt(i, i, accelerometerVariance)
        }
        for (i in 3 until 6) {
            measurementNoiseCov.setElementAt(i, i, magnetometerVariance)
        }
        for (i in 7 until 9) {
            measurementNoiseCov.setElementAt(i, i, gyroVariance)
        }
        measurementNoiseCov.setElementAt(9, 9, temperatureVariance)
    }

    /**
     * Updates measurement matrix with accelerometer and gyroscope measurements to be used during
     * Kalman filter correction phase.
     */
    private fun updateMeasurement() {
        m.setElementAtIndex(0, nedAcceleration.valueX)
        m.setElementAtIndex(1, nedAcceleration.valueY)
        m.setElementAtIndex(2, nedAcceleration.valueZ)

        m.setElementAtIndex(3, nedMagneticFluxDensity.valueX)
        m.setElementAtIndex(4, nedMagneticFluxDensity.valueY)
        m.setElementAtIndex(5, nedMagneticFluxDensity.valueZ)

        m.setElementAtIndex(6, nedAngularSpeed.valueX)
        m.setElementAtIndex(7, nedAngularSpeed.valueY)
        m.setElementAtIndex(8, nedAngularSpeed.valueZ)

        m.setElementAtIndex(9, currentTemperature)
    }

    /**
     * Updates process noise covariance on a block by block basis taking into account time interval
     * between measurements, accelerometer standard deviation, current estimated quaternion state.
     */
    private fun updateProcessNoiseCovariance() {
        val processNoiseCov = kalmanFilter.processNoiseCov

        // process noise covariance is a block diagonal matrix following expression:
        // Q = diag(Qa, Qq, Qr, Qt)

        // where:
        // - Qa corresponds to covariance of acceleration respect Earth. 3x3 matrix
        // - Qq corresponds to covariance of quaternion respect Earth. 4x4 matrix.
        // - Qr corresponds to covariance of unitary body rotation axis. 3x3 matrix.
        // - Qt corresponds to variance of temperature.

        // set Qa
        updateAccelerationProcessNoiseCovariance(processNoiseCov)

        val sqrHalfInterval = (gyroscopeIntervalSeconds / 2.0).pow(2.0)

        // set Qq
        val gyroVariance = gyroscopeNoisePsd / gyroscopeIntervalSeconds
        updateQuaternionProcessNoiseCovariance(gyroVariance, sqrHalfInterval, processNoiseCov)

        // setQr
        updateRotationVectorProcessNoiseCovariance(gyroVariance, sqrHalfInterval, processNoiseCov)

        // setQt
        processNoiseCov.setElementAt(TEMPERATURE_OFFSET, TEMPERATURE_OFFSET, temperatureVariance)
    }

    /**
     * Updates sub-matrix block of process noise covariance associated to acceleration.
     *
     * @param processNoiseCov process noise covariance where updated sub-matrix will be stored.
     */
    private fun updateAccelerationProcessNoiseCovariance(processNoiseCov: Matrix) {
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

        val q0 = ecefQ.a
        val q1 = ecefQ.b
        val q2 = ecefQ.c
        val q3 = ecefQ.d

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
        for (i in ROTATION_AXIS_OFFSET_START..ROTATION_AXIS_OFFSET_END) {
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
     * Transition matrix is made of four sub-matrices.
     * Acceleration 3x3 sub-matrix will always be the identity assuming that no external forces are
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
        /*copyFilterStateToDeltaQuaternion(kalmanFilter.statePost, nedDeltaQ)

        deltaQMatrix.setElementAtIndex(0, nedDeltaQ.a)
        deltaQMatrix.setElementAtIndex(1, nedDeltaQ.b)
        deltaQMatrix.setElementAtIndex(2, nedDeltaQ.c)
        deltaQMatrix.setElementAtIndex(3, nedDeltaQ.d)

        deltaQMatrix.setElementAtIndex(4, -nedDeltaQ.b)
        deltaQMatrix.setElementAtIndex(5, nedDeltaQ.a)
        deltaQMatrix.setElementAtIndex(6, -nedDeltaQ.d)
        deltaQMatrix.setElementAtIndex(7, nedDeltaQ.c)

        deltaQMatrix.setElementAtIndex(8, -nedDeltaQ.c)
        deltaQMatrix.setElementAtIndex(9, nedDeltaQ.d)
        deltaQMatrix.setElementAtIndex(10, nedDeltaQ.a)
        deltaQMatrix.setElementAtIndex(11, -nedDeltaQ.b)

        deltaQMatrix.setElementAtIndex(12, -nedDeltaQ.d)
        deltaQMatrix.setElementAtIndex(13, -nedDeltaQ.c)
        deltaQMatrix.setElementAtIndex(14, nedDeltaQ.b)
        deltaQMatrix.setElementAtIndex(15, nedDeltaQ.a)

        val transitionMatrix = kalmanFilter.transitionMatrix
        transitionMatrix.setSubmatrix(3, 3, 6, 6, deltaQMatrix)*/

        estimateDeltaQ()

        // q0 = [a0 b0 c0 d0]
        // deltaQ1 = [da db dc dd]
        // q1 = [a1 b1 c1 d1]

        // q1 = deltaQ1 * q0
        // [a1] = [da * a0 - db * b0 - dc * c0 - dd * d0] = [da     -db     -dc     -dd][a0]
        // [b1]   [da * b0 + db * a0 + dc * d0 - dd * c0]   [db      da     -dd      dc][b0]
        // [c1]   [da * c0 - db * d0 + dc * a0 + dd * b0]   [dc      dd      da     -db][c0]
        // [d1]   [da * d0 + db * c0 - dc * b0 + dd * a0]   [dd     -dc      db      da][d0]

        // matrix above corresponds to method [Quaternion#quaternionMatrix(Matrix)]

        // copy quaternion variation deltaQ into corresponding 4x4 sub-matrix within transition
        // matrix

        val transitionMatrix = kalmanFilter.transitionMatrix
        ecefDeltaQ.quaternionMatrix(deltaQMatrix)
        transitionMatrix.setSubmatrix(3, 3, 6, 6, deltaQMatrix)


// TODO:        nedDeltaQ.quaternionMatrix(deltaQMatrix)
// TODO:        transitionMatrix.setSubmatrix(7, 7, 9, 9, deltaQMatrix, 1, 1, 3, 3)

        // other parts of transition matrix don't need to be updated as they are the identity
    }

    private fun estimateDeltaQ() {
        // accurately integrates current attitude with current and previous angular speeds
        // to estimate variation (Delta) of quaternion attitude.
        quaternionStepIntegrator.integrate(
            nedQ,
            previousNedAngularSpeed.valueX,
            previousNedAngularSpeed.valueY,
            previousNedAngularSpeed.valueZ,
            nedAngularSpeed.valueX,
            nedAngularSpeed.valueY,
            nedAngularSpeed.valueZ,
            gyroscopeIntervalSeconds,
            expectedNedQ
        )

        // convert ned to ECEF coordinates
        convertNedToEcef(nedQ, ecefQ)
        convertNedToEcef(expectedNedQ, expectedEcefQ)

        // compute delta attitude between current and expected quaternions:
        // q1 = deltaQ1 * q0
        // deltaQ1 = q1 * invQ0

        ecefQ.inverse(invQ)
        invQ.normalize()
        expectedEcefQ.multiply(invQ, ecefDeltaQ)
        ecefDeltaQ.normalize()

        nedQ.inverse(invQ)
        invQ.normalize()
        expectedNedQ.multiply(invQ, nedDeltaQ)
        nedDeltaQ.normalize()

        //copyDeltaQuaternionToFilterState(nedDeltaQ, kalmanFilter.statePost)
    }

    /**
     * Updates Kalman filter measurement matrix which relates filter state and measurement matrices.
     */
    private fun updateMeasurementMatrix() {
        // Kalman filter state is made of:
        // - ECEF acceleration respect to Earth (a_e)
        // - ECEF quaternion attitude respect to Earth (q_e)
        // - NED rotation vector respect to body (v_b)
        // - temperature

        // Kalman filter measurements contain:
        // - NED acceleration (specific force) respect to body. (a_b)
        // - NED magnetic flux density respect to body (m_b)
        // - NED angular speed respect to body. (w_b)
        // - temperature

        // Consequently, measurement matrix m can be obtained from Kalman filter state x through
        // measurement matrix H as:
        // m = H * x

        // where H is a 9x10 matrix made of the following blocks:
        // H = [H11  H12  0    H14]
        //     [0    H22  0    H24]
        //     [0    0    H33  H34]
        //     [0    0    0    H44]

        // H11 is a 3x3 matrix relating acceleration respect to Earth and respect to body
        // H12 is a 3x4 matrix relating quaternion respect to Earth and acceleration respect to body
        // So that:
        // a_b = H11 * a_e + H12 * q_e

        // where q_e = [q0, q1, q2, q3]

        // H11 = [1 - 2 * (q2^2 + q3^2)     2 * (q0 * q3 + q1 * q2)     2 * (q1 * q3 - q0 * q2)]
        //       [2 * (q1 * q2 - q0 * q3)   1 - 2 * (q1^2 + q3^2)       2 * (q0 * q1 + q2 * q3)]
        //       [2 * (q0 * q2 + q1 * q3)   2 * (q2 * q3 - q0 * q1)     1 - 2 * (q1^2 + q2^2)  ]

        // H12 = g * [-q2      q3      -q0     q1]
        //           [ q1      q0       q3     q2]
        //           [ q0     -q1      -q2     q3]

        // where g is gravity norm

        // H22 is a 3x4 matrix relating quaternion respect to Earth and respect to body

        // H22 = h * sin(alpha) * [ q0      q1      -q2     -q3] +
        //                        [-q3      q2       q1     -q0]
        //                        [ q2      q3       q0      q1]
        //
        //       h * cos(apha) * [-q2     q3      -q0     q1]
        //                       [ q1     q0       q3     q2]
        //                       [ q0    -q1      -q2     q3]

        // where alpha is the angle between the local magnetic and gravity fields and h is the
        // magnetic field modulus.

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
        // And finally we obtain the approximate relation between angular speed and rotation axis
        // as:
        // w_b = 2 / deltaT * v_b

        // And consequently matrix H33 becomes:
        // H33 = 2 / deltaT * [1  0  0]
        //                    [0  1  0]
        //                    [0  0  1]

        // H14 is 3x1
        // H24 is 3x1
        // H34 is 3x1
        // H44 is 1x1

        val q0 = ecefQ.a
        val q1 = ecefQ.b
        val q2 = ecefQ.c
        val q3 = ecefQ.d

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
        h12.setElementAtIndex(0, gravityNorm * q2)
        h12.setElementAtIndex(1, -gravityNorm * q1)
        h12.setElementAtIndex(2, -gravityNorm * q0)

        h12.setElementAtIndex(3, -gravityNorm * q3)
        h12.setElementAtIndex(4, -gravityNorm * q0)
        h12.setElementAtIndex(5, gravityNorm * q1)

        h12.setElementAtIndex(6, gravityNorm * q0)
        h12.setElementAtIndex(7, -gravityNorm * q3)
        h12.setElementAtIndex(8, gravityNorm * q2)

        h12.setElementAtIndex(9, -gravityNorm * q1)
        h12.setElementAtIndex(10, -gravityNorm * q2)
        h12.setElementAtIndex(11, -gravityNorm * q3)


        // compute H22
        val h = nedMagneticFluxDensity.norm

        val alpha = Math.PI / 2.0 - magneticDip
        val cosAlpha = cos(alpha)
        val sinAlpha = sin(alpha)

        val hsinAlpha = h * sinAlpha
        val hcosAlpha = h * cosAlpha

        val hsinAlphaQ0 = hsinAlpha * q0
        val hsinAlphaQ1 = hsinAlpha * q1
        val hsinAlphaQ2 = hsinAlpha * q2
        val hsinAlphaQ3 = hsinAlpha * q3

        val hcosAlphaQ0 = hcosAlpha * q0
        val hcosAlphaQ1 = hcosAlpha * q1
        val hcosAlphaQ2 = hcosAlpha * q2
        val hcosAlphaQ3 = hcosAlpha * q3

        h22.setElementAtIndex(0, hsinAlphaQ0 - hcosAlphaQ2)
        h22.setElementAtIndex(1, -hsinAlphaQ3 + hcosAlphaQ1)
        h22.setElementAtIndex(2, hsinAlphaQ2 + hcosAlphaQ0)

        h22.setElementAtIndex(3, hsinAlphaQ1 + hcosAlphaQ3)
        h22.setElementAtIndex(4, hsinAlphaQ2 + hcosAlphaQ0)
        h22.setElementAtIndex(5, hsinAlphaQ3 - hcosAlphaQ1)

        h22.setElementAtIndex(6, -hsinAlphaQ2 - hcosAlphaQ0)
        h22.setElementAtIndex(7, hsinAlphaQ1 + hcosAlphaQ3)
        h22.setElementAtIndex(8, hsinAlphaQ0 - hcosAlphaQ2)

        h22.setElementAtIndex(9, -hsinAlphaQ3 + hcosAlphaQ1)
        h22.setElementAtIndex(10, -hsinAlphaQ0 + hcosAlphaQ2)
        h22.setElementAtIndex(11, hsinAlphaQ1 + hcosAlphaQ3)


        // compute H33
        val value = 2.0 / gyroscopeIntervalSeconds
        for (i in 0 until COMPONENTS) {
            h33.setElementAt(i, i, value)
        }

        // update measurement matrix
        val measurementMatrix = kalmanFilter.measurementMatrix
        measurementMatrix.initialize(0.0)
        measurementMatrix.setSubmatrix(0, 0, 2, 2, h11)
        measurementMatrix.setSubmatrix(0, 3, 2, 6, h12)
        measurementMatrix.setSubmatrix(3, 3, 5, 6, h22)
        measurementMatrix.setSubmatrix(6, 7, 8, 9, h33)
//TODO: H14, H24 and H34 are assumed to be zero for now
        measurementMatrix.setElementAt(9, 10, 1.0)
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
            if (!isPositiveSemiDefinite(
                    errorCov,
                    symmetricThreshold
                )
            ) {
                // when covariance is not positive semi-definite, we can assume that Kalman filter
                // has become numerically unstable, and we need to reset error covariance value
                initErrorCovariances()

                numericalInstabilitiesListener?.onNumericalInstabilityDetected(this)
            }

            // take only the part corresponding to ECEF quaternion covariance
            errorCov.getSubmatrix(3, 3, 6, 6, internalEcefQuaternionCovariance)

            if (computeEulerAngles) {
                // As defined in: https://en.wikipedia.org/wiki/Propagation_of_uncertainty
                // covariance propagation for a function converting quaternion to euler angles is:
                // Cov`= J * Cov * J^T
                if (computeEulerAnglesCovariance) {
                    nedAttitude.toEulerAngles(internalEulerAngles, eulerJacobian)
                    eulerJacobian.transpose(transposedEulerJacobian)

//TODO: fix
                    /*propagateEulerAnglesUncertainty(
                        internalNedQuaternionCovariance,
                        internalEulerAnglesCovariance
                    )*/
                } else {
                    nedAttitude.toEulerAngles(internalEulerAngles)
                }
            }

            if (computeEcefAccelerationCovariance) {
                errorCov.getSubmatrix(0, 0, 2, 2, internalAccelerationCovariance)
            }

            if (computeNedRotationAxisCovariance) {
                errorCov.getSubmatrix(7, 7, 9, 9, internalNedRotationAxisCovariance)
            }
        }
    }

    private fun convertNedToEcef(nedQuaternion: Quaternion, result: Quaternion) {
        /*ecefToNedQ.multiply(nedQuaternion, tmpQ)
        tmpQ.multiply(nedToEcefQ, result)*/

        nedToEcefQ.multiply(nedQuaternion, tmpQ)
        tmpQ.multiply(ecefToNedQ, result)
    }

    private fun convertEcefToNed(ecefQuaternion: Quaternion, result: Quaternion) {
        /*nedToEcefQ.multiply(ecefQuaternion, tmpQ)
        tmpQ.multiply(ecefToNedQ, result)*/

        ecefToNedQ.multiply(ecefQuaternion, tmpQ)
        tmpQ.multiply(nedToEcefQ, result)
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
                magneticDip = wmmEstimator.getDip(nedPosition, year(calendar))
                magneticDeclination = wmmEstimator.getDeclination(nedPosition, year(calendar))
            }

            NEDGravityEstimator.estimateGravity(nedPosition, nedGravity)
            gravityNorm = nedGravity.norm

            CoordinateTransformation.ecefToNedMatrix(
                nedPosition.latitude,
                nedPosition.longitude,
                ecefToNedMatrix
            )
        } else {
            gravityNorm = EARTH_GRAVITY

            Matrix.identity(ecefToNedMatrix)
        }
        ecefToNedQ.fromInhomogeneousMatrix(ecefToNedMatrix)
        ecefToNedQ.normalize()
        ecefToNedQ.conjugate(nedToEcefQ)

        if (wmmEstimator == null) {
            magneticDip = 0.0
            magneticDeclination = 0.0
        }
    }

    init {
        buildWMMEstimator()
        this.location = location
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
         * Default standard deviation for temperature noise expressed in Celsius (C).
         */
        const val DEFAULT_TEMPERATURE_NOISE_STANDARD_DEVIATION = 1.0

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
         * Default temperature expressed in Celsius (20 ºC).
         */
        const val DEFAULT_TEMPERATURE_CELSIUS = 20.0

        /**
         * Amount of components of triad measurements and arrays.
         */
        private const val COMPONENTS = Triad.COMPONENTS

        /**
         * Threshold to determine whether covariance matrices are positive definite or not.
         */
        private const val SYMMETRIC_THRESHOLD = 1e-10

        /**
         * Kalman filter's number of dynamic parameters.
         */
        private const val KALMAN_FILTER_DYNAMIC_PARAMETERS = 11

        /**
         * Kalman filter's number of measurement parameters.
         */
        private const val KALMAN_FILTER_MEASUREMENT_PARAMETERS = 10

        /**
         * Start offset of rotation axis vector within Kalman filter state.
         */
        private const val ROTATION_AXIS_OFFSET_START = 7

        /**
         * End offset of rotation axis vector within Kalman filter state.
         */
        private const val ROTATION_AXIS_OFFSET_END = 9

        /**
         * Position where temperature offset is stored in process noise covariance matrix.
         */
        private const val TEMPERATURE_OFFSET = 10

        /**
         * Position where internal [qq] matrix ends within process noise covariance matrix.
         */
        private const val END_QQ = 6

        /**
         * Processes an accelerometer measurement, taking into account their biases (if available).
         * Results will contain accelerometer data expressed in both ENU and NED coordinates.
         *
         * @param measurement measurement to be processed.
         * @param enuResult triad where measurement data will be stored in ENU coordinates.
         * @param nedResult triad where measurement data will be stored in NED coordinates.
         */
        private fun processMeasurement(
            measurement: AccelerometerSensorMeasurement,
            enuResult: AccelerationTriad,
            nedResult: AccelerationTriad
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

            enuResult.setValueCoordinatesAndUnit(
                valueX,
                valueY,
                valueZ,
                AccelerationUnit.METERS_PER_SQUARED_SECOND
            )

            // convert to NED coordinates
            ENUtoNEDConverter.convert(enuResult, nedResult)
        }

        /**
         * Processes a gyroscope measurement, taking into account their biases (if available).
         * Results will contain gyroscope data expressed in both ENU and NED coordinates.
         *
         * @param measurement measurement to be processed.
         * @param enuResult triad where measurement data will be stored in ENU coordinates.
         * @param nedResult triad where measurement data will be stored in NED coordinates.
         */
        private fun processMeasurement(
            measurement: GyroscopeSensorMeasurement,
            enuResult: AngularSpeedTriad,
            nedResult: AngularSpeedTriad
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

            enuResult.setValueCoordinatesAndUnit(
                valueX,
                valueY,
                valueZ,
                AngularSpeedUnit.RADIANS_PER_SECOND
            )

            // convert to NED coordinates
            ENUtoNEDConverter.convert(enuResult, nedResult)
        }

        /**
         * Processes a magnetometer measurement, taking into account their hard iron (if available).
         * Measurement will contain magnetometer data expressed in NED coordinates.
         *
         * @param measurement measurement to be processed.
         * @param enuResult triad where measurement data will be stored in ENU coordinates.
         * @param nedResult triad where measurement data will be stored in NED coordinates.
         */
        private fun processMeasurement(
            measurement: MagnetometerSensorMeasurement,
            enuResult: MagneticFluxDensityTriad,
            nedResult: MagneticFluxDensityTriad
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

            enuResult.setValueCoordinatesAndUnit(
                valueX,
                valueY,
                valueZ,
                MagneticFluxDensityUnit.TESLA
            )

            // convert to NED coordinates
            ENUtoNEDConverter.convert(enuResult, nedResult)
        }

        /**
         * Computes Frobenius norm for provided array.
         *
         * @param data array to be evaluated.
         * @return estimated Frobenius norm.
         */
        private fun norm(data: DoubleArray): Double {
            return Utils.normF(data)
        }

        /**
         * Normalizes provided array by its Frobenius norm.
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
         * Copy ECEF acceleration triad into Kalman filter state.
         *
         * @param ecefAcceleration ECEF acceleration triad.
         * @param result matrix instance where data will be stored.
         */
        private fun copyEcefAccelerationToFilterState(
            ecefAcceleration: AccelerationTriad,
            result: Matrix
        ) {
            result.setElementAtIndex(0, ecefAcceleration.valueX)
            result.setElementAtIndex(1, ecefAcceleration.valueY)
            result.setElementAtIndex(2, ecefAcceleration.valueZ)
        }

        /**
         * Copies Kalman filter state data associated to ECEF acceleration into provided
         * acceleration triad.
         *
         * @param state Kalman filter state matrix.
         * @param result acceleration triad where data will be stored.
         */
        private fun copyFilterStateToEcefAcceleration(
            state: Matrix,
            result: AccelerationTriad
        ) {
            result.setValueCoordinatesAndUnit(
                state.getElementAtIndex(0),
                state.getElementAtIndex(1),
                state.getElementAtIndex(2),
                AccelerationUnit.METERS_PER_SQUARED_SECOND
            )
        }

        /**
         * Copies ECEF quaternion to provided Kalman filter state matrix.
         *
         * @param ecefQuaternion ECEF quaternion to copy data from.
         * @param result matrix instance where data will be stored.
         */
        private fun copyEcefQuaternionToFilterState(ecefQuaternion: Quaternion, result: Matrix) {
            ecefQuaternion.normalize()
            result.setElementAtIndex(3, ecefQuaternion.a)
            result.setElementAtIndex(4, ecefQuaternion.b)
            result.setElementAtIndex(5, ecefQuaternion.c)
            result.setElementAtIndex(6, ecefQuaternion.d)
        }

        /**
         * Fixes and copies Kalman filter state data associated to provided ECEF quaternion.
         *
         * @param state Kalman filter state.
         * @param result quaternion where data will be stored.
         */
        private fun fixStateAndCopyToEcefQuaternion(state: Matrix, result: Quaternion) {
            result.a = state.getElementAtIndex(3)
            result.b = state.getElementAtIndex(4)
            result.c = state.getElementAtIndex(5)
            result.d = state.getElementAtIndex(6)

            copyEcefQuaternionToFilterState(result, state)
        }

        /**
         * Copies NED quaternion to provided Kalman filter state matrix.
         *
         * @param deltaQ NED quaternion to copy data from.
         * @param result matrix instance where data will be stored.
         */
        private fun copyDeltaQuaternionToFilterState(deltaQ: Quaternion, result: Matrix) {
            deltaQ.normalize()
            result.setElementAtIndex(7, deltaQ.b)
            result.setElementAtIndex(8, deltaQ.c)
            result.setElementAtIndex(9, deltaQ.d)
        }

        /**
         * Fixes and copies Kalman filter state data associated to provided NED quaternion.
         *
         * @param state Kalman filter state.
         * @param result quaternion where data will be stored.
         */
        private fun copyFilterStateToDeltaQuaternion(state: Matrix, result: Quaternion) {
            val vx = state.getElementAtIndex(7)
            val vy = state.getElementAtIndex(8)
            val vz = state.getElementAtIndex(9)

            getAttitudeFromAxis(vx, vy, vz, result)
        }

        /**
         * Copies provided temperature to Kalman filter state.
         *
         * @param temperature temperature expressed in Celsius (C).
         * @return matrix instance where data will be stored.
         */
        private fun copyTemperatureToFilterState(temperature: Double, result: Matrix) {
            result.setElementAtIndex(10, temperature)
        }

        /**
         * Gets temperature store in Kalman filter state.
         *
         * @return temperature expressed in Celsius (C)
         */
        private fun getTemperatureFromFilterState(state: Matrix): Double {
            return state.getElementAtIndex(10)
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
         * Gets associated quaternion attitude from the vector part of a unitary quaternion.
         *
         * @param vx x-coordinate of vector part of a quaternion.
         * @param vy y-coordinate of vector part of a quaternion.
         * @param vz z-coordinate of vector part of a quaternion.
         * @param result instance where resulting quaternion will be stored.
         */
        private fun getAttitudeFromAxis(vx: Double, vy: Double, vz: Double, result: Quaternion) {
            val sqrNorm = vx * vx + vy * vy + vz * vz
            result.a = sqrt(1.0 - sqrNorm)
            result.b = vx
            result.c = vy
            result.d = vz
            result.normalize()
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
            processor: KalmanAbsoluteAttitudeProcessor3,
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
        fun onNumericalInstabilityDetected(processor: KalmanAbsoluteAttitudeProcessor3)
    }
}