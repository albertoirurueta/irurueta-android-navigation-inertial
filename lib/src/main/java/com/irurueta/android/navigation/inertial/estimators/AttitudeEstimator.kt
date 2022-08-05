/*
 * Copyright (C) 2022 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.android.navigation.inertial.estimators

import android.content.Context
import android.hardware.Sensor
import android.location.Location
import android.os.Build
import com.irurueta.algebra.ArrayUtils
import com.irurueta.algebra.Matrix
import com.irurueta.algebra.Utils
import com.irurueta.android.navigation.inertial.DisplayOrientationHelper
import com.irurueta.android.navigation.inertial.collectors.*
import com.irurueta.android.navigation.inertial.toNEDPosition
import com.irurueta.geometry.InvalidRotationMatrixException
import com.irurueta.geometry.Quaternion
import com.irurueta.geometry.Rotation3D
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType
import com.irurueta.navigation.frames.NEDPosition
import com.irurueta.navigation.inertial.estimators.AttitudeEstimator
import com.irurueta.navigation.inertial.estimators.LevelingEstimator
import com.irurueta.navigation.inertial.estimators.LevelingEstimator2
import com.irurueta.navigation.inertial.estimators.NEDGravityEstimator
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel
import com.irurueta.units.MagneticFluxDensityConverter
import java.util.*

/**
 * Estimates attitude based on accelerometer and magnetometer samples when using a given World
 * Magnetic Model at a specific timestamp, or based on accelerometer and gyroscope samples if
 * gyroscope is precise enough.
 * IMU attitude estimation based on accelerometer and gyroscope should only be used when gyroscope
 * is accurate enough, otherwise unreliable results will be obtained.
 * Notice that when gyroscope is accurate, IMU attitude estimation can be used for purposes
 * where device is subject to electromagnetic interference, since accelerometer and gyroscope is
 * not interfered. However, gyroscope suffers from drift that increases with time.
 * On the other hand, geomagnetic attitude estimation does not suffer from drift but is less precise
 * and can be subject to electromagnetic interference.
 *
 * @property context Android context.
 * @property accelerometerSensorType One of the supported accelerometer sensor types.
 * @property gyroscopeSensorType One of the supported gyroscope sensor types.
 * @property magnetometerSensorType One of the supported magnetometer sensor types.
 * @property accelerometerSensorDelay Delay of accelerometer sensor between samples.
 * @property gyroscopeSensorDelay Delay of gyroscope sensor between samples.
 * @property magnetometerSensorDelay Delay of magnetometer sensor between samples.
 * @property gravitySensorDelay Delay of gravity sensor between samples.
 * @property timestamp Timestamp when World Magnetic Model will be evaluated to obtain current
 * magnetic declination.
 * @property attitudeAvailableListener listener to notify when a new attitude has been estimated.
 */
class AttitudeEstimator private constructor(
    val context: Context,
    val accelerometerSensorType: AccelerometerSensorCollector.SensorType,
    val gyroscopeSensorType: GyroscopeSensorCollector.SensorType,
    val magnetometerSensorType: MagnetometerSensorCollector.SensorType,
    val accelerometerSensorDelay: SensorDelay,
    val gyroscopeSensorDelay: SensorDelay,
    val magnetometerSensorDelay: SensorDelay,
    val gravitySensorDelay: SensorDelay,
    var timestamp: Date,
    var attitudeAvailableListener: OnAttitudeAvailableListener?
) {
    /**
     * Constructor.
     *
     * @param context Android context.
     * @param accelerometerSensorType One of the supported accelerometer sensor types.
     * @param gyroscopeSensorType One of the supported gyroscope sensor types.
     * @param magnetometerSensorType One of the supported magnetometer sensor types.
     * @param accelerometerSensorDelay Delay of accelerometer sensor between samples.
     * @param gyroscopeSensorDelay Delay of gyroscope sensor between samples.
     * @param magnetometerSensorDelay Delay of magnetometer sensor between samples.
     * @param location Current device location. This is needed to obtain geomagnetic attitude by
     * using accelerometer and magnetometer measurements and evaluating World Magnetic Model at
     * provided location and timestamp. If no location is set, then attitude will be estimated by
     * means of leveling using accelerometer and gyroscope measurements.
     * @param estimateImuLeveling true to use accelerometer + gyroscope for attitude estimation
     * even if location is provided, false to use accelerometer + magnetometer when location is
     * provided. IMU leveling should only be used when gyroscope is accurate enough.
     * @param worldMagneticModel Earth's magnetic model. Null to use default model.
     * @param timestamp Timestamp when World Magnetic Model will be evaluated to obtain current
     * magnetic declination. By default this is the current timestamp.
     * @param attitudeAvailableListener listener to notify when a new attitude has been estimated.
     */
    constructor(
        context: Context,
        accelerometerSensorType: AccelerometerSensorCollector.SensorType = AccelerometerSensorCollector.SensorType.ACCELEROMETER,
        gyroscopeSensorType: GyroscopeSensorCollector.SensorType = GyroscopeSensorCollector.SensorType.GYROSCOPE,
        magnetometerSensorType: MagnetometerSensorCollector.SensorType = MagnetometerSensorCollector.SensorType.MAGNETOMETER,
        accelerometerSensorDelay: SensorDelay = SensorDelay.UI,
        gyroscopeSensorDelay: SensorDelay = SensorDelay.UI,
        magnetometerSensorDelay: SensorDelay = SensorDelay.UI,
        gravitySensorDelay: SensorDelay = SensorDelay.UI,
        location: Location? = null,
        estimateImuLeveling: Boolean = false,
        gravitySensorEnabled: Boolean = true,
        worldMagneticModel: WorldMagneticModel? = null,
        timestamp: Date = Date(),
        attitudeAvailableListener: OnAttitudeAvailableListener? = null
    ) : this(
        context,
        accelerometerSensorType,
        gyroscopeSensorType,
        magnetometerSensorType,
        accelerometerSensorDelay,
        gyroscopeSensorDelay,
        magnetometerSensorDelay,
        gravitySensorDelay,
        timestamp,
        attitudeAvailableListener
    ) {
        this.location = location
        this.estimateImuLeveling = estimateImuLeveling
        this.gravitySensorEnabled = gravitySensorEnabled
        this.worldMagneticModel = worldMagneticModel
    }

    /**
     * Instance to be reused containing coordinate transformation in NED coordinates.
     */
    private val coordinateTransformation =
        CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME)

    /**
     * Instance to be reused containing estimated attitude in NED coordinates.
     */
    private val attitude = Quaternion()

    private val displayOrientation = Quaternion()

    /**
     * Estimator to obtain geomagnetic attitude using accelerometer + magnetometer measurements.
     */
    private var attitudeEstimator: AttitudeEstimator? = null

    /**
     * Indicates that a new accelerometer sample is available since last processing.
     */
    private var accelerometerSampleAvailable = false

    /**
     * Last accelerometer x-coordinate measurement expressed in meters per squared second (m/s^2).
     */
    private var sensorAx = 0.0f

    /**
     * Last accelerometer y-coordinate measurement expressed in meters per squared second (m/s^2).
     */
    private var sensorAy = 0.0f

    /**
     * Last accelerometer z-coordinate measurement expressed in meters per squared second (m/s^2).
     */
    private var sensorAz = 0.0f

    /**
     * Internal accelerometer sensor collector.
     */
    private val accelerometerSensorCollector = AccelerometerSensorCollector(
        context, accelerometerSensorType, accelerometerSensorDelay,
        { ax, ay, az, _, _, _, _, _ ->
            this@AttitudeEstimator.sensorAx = ax
            this@AttitudeEstimator.sensorAy = ay
            this@AttitudeEstimator.sensorAz = az
            accelerometerSampleAvailable = true
            processSamples()
        })

    /**
     * Indicates that a new gyroscope sample is available since last processing.
     */
    private var gyroscopeSampleAvailable = false

    /**
     * Last gyroscope x-coordinate measurement expressed in radians per second (rad/s).
     */
    private var sensorWx = 0.0f

    /**
     * Last gyroscope y-coordinate measurement expressed in radians per second (rad/s).
     */
    private var sensorWy = 0.0f

    /**
     * Last gyroscope z-coordinate measurement expressed in radians per second (rad/s).
     */
    private var sensorWz = 0.0f

    /**
     * Internal gyroscope sensor collector.
     */
    private val gyroscopeSensorCollector = GyroscopeSensorCollector(
        context,
        gyroscopeSensorType,
        gyroscopeSensorDelay,
        { wx, wy, wz, _, _, _, _, _ ->
            this@AttitudeEstimator.sensorWx = wx
            this@AttitudeEstimator.sensorWy = wy
            this@AttitudeEstimator.sensorWz = wz
            gyroscopeSampleAvailable = true
            processSamples()
        })

    /**
     * Indicates that a new magnetometer sample is available since last processing.
     */
    private var magnetometerSampleAvailable = false

    /**
     * Last magnetometer x-coordinate measurement expressed in micro Teslas (µT).
     */
    private var sensorBx = 0.0f

    /**
     * Last magnetometer y-coordinate measurement expressed in micro Teslas (µT).
     */
    private var sensorBy = 0.0f

    /**
     * Last magnetometer z-coordinate measurement expressed in micro Teslas (µT).
     */
    private var sensorBz = 0.0f

    /**
     * Internal magnetometer sensor collector.
     */
    private val magnetometerSensorCollector = MagnetometerSensorCollector(
        context,
        magnetometerSensorType,
        magnetometerSensorDelay,
        { bx, by, bz, _, _, _, _, _ ->
            this@AttitudeEstimator.sensorBx = bx
            this@AttitudeEstimator.sensorBy = by
            this@AttitudeEstimator.sensorBz = bz
            magnetometerSampleAvailable = true
            processSamples()
        })

    /**
     * Indicates that a new gravity sample is available since last processing.
     */
    private var gravitySampleAvailable = false

    /**
     * Last gravity x-coordinate measurement expressed in meters per squared seconds (m/s^2).
     */
    private var sensorGx = 0.0f

    /**
     * Last gravity y-coordinate measurement expressed in meters per squared seconds (m/s^2).
     */
    private var sensorGy = 0.0f

    /**
     * Last gravity z-coordinate measurement expressed in meters per squared seconds (m/s^2).
     */
    private var sensorGz = 0.0f

    /**
     * Internal gravity sensor collector.
     */
    private val gravitySensorCollector = GravitySensorCollector(
        context,
        gravitySensorDelay,
        measurementListener = { gx, gy, gz, _, _, _ ->
            this@AttitudeEstimator.sensorGx = gx
            this@AttitudeEstimator.sensorGy = gy
            this@AttitudeEstimator.sensorGz = gz
            gravitySampleAvailable = true
            processSamples()
        })

    /**
     * Indicates whether estimator is currently running or not.
     */
    var running = false
        private set

    /**
     * Gets or sets current device location. This is needed to obtain geomagnetic attitude by
     * using accelerometer and magnetometer measurements and evaluating World Magnetic Model at
     * provided location and timestamp. If no location is set, then attitude will be estimated by
     * means of leveling using accelerometer and gyroscope measurements.
     *
     * @throws IllegalStateException if estimator is running and location is set to null.
     */
    var location: Location? = null
        @Throws(IllegalStateException::class)
        set(value) {
            if (running && field != null) {
                check(value != null)
            }

            field = value
            nedPosition = value?.toNEDPosition()
            positionHorizontalAccuracyMeters = value?.accuracy
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
                positionVerticalAccuracyMeters = value?.verticalAccuracyMeters
            }
            computeType()
        }

    /**
     * Current device location expressed in NED coordinates.
     */
    var nedPosition: NEDPosition? = null
        private set

    /**
     * Horizontal accuracy of current device location expressed in meters.
     */
    var positionHorizontalAccuracyMeters: Float? = null
        private set

    /**
     * Vertical accuracy of current device location expressed in meters.
     */
    var positionVerticalAccuracyMeters: Float? = null
        private set

    /**
     * Earth's magnetic model. If null, the default model is used.
     *
     * @throws IllegalStateException if estimator is running.
     */
    var worldMagneticModel: WorldMagneticModel? = null
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)

            field = value
            attitudeEstimator = if (value != null) {
                AttitudeEstimator(value)
            } else {
                AttitudeEstimator()
            }
        }

    /**
     * Indicates whether IMU (accelerometer + gyroscope) leveling is enforced or not.
     * When true accelerometer + gyroscope is used for attitude estimation even if location is
     * provided.
     * When false accelerometer + magnetometer is used when location is provided.
     *
     * @throws IllegalStateException if estimator is running
     */
    var estimateImuLeveling: Boolean = false
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)

            field = value
            computeType()
        }

    /**
     * Indicates whether gravity sensor is used instead of accelerometer to obtain gravity readings.
     * If accelerometer is used, readings are low-pass filtered to estimate gravity
     *
     * @throws IllegalStateException if estimator is running.
     */
    var gravitySensorEnabled: Boolean = true
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)

            field = value
            computeType()
        }

    /**
     * Gets accelerometer sensor being used to obtain measurements or null if not available.
     * This can be used to obtain additional information about the sensor.
     */
    val accelerometerSensor: Sensor?
        get() = accelerometerSensorCollector.sensor

    /**
     * Gets gyroscope sensor being used to obtain measurements or null if not available.
     * This can be used to obtain additional information about the sensor.
     */
    val gyroscopeSensor: Sensor?
        get() = gyroscopeSensorCollector.sensor

    /**
     * Gets magnetometer sensor being used to obtain measurements or null if not available.
     * This can be used to obtain additional information about the sensor.
     */
    val magnetometerSensor: Sensor?
        get() = magnetometerSensorCollector.sensor

    /**
     * Gets gravity sensor being used to obtain measurements or null if not available.
     * This can be used to obtain additional information about the sensor.
     */
    val gravitySensor: Sensor?
        get() = gravitySensorCollector.sensor

    /**
     * Indicates whether requested accelerometer sensor is available or not.
     */
    val accelerometerSensorAvailable: Boolean
        get() = accelerometerSensorCollector.sensorAvailable

    /**
     * Indicates whether requested gyroscope sensor is available or not.
     */
    val gyroscopeSensorAvailable: Boolean
        get() = gyroscopeSensorCollector.sensorAvailable

    /**
     * Indicates whether requested magnetometer sensor is available or not.
     */
    val magnetometerSensorAvailable: Boolean
        get() = magnetometerSensorCollector.sensorAvailable

    /**
     * Indicates whether requested gravity sensor is available or not.
     */
    val gravitySensorAvailable: Boolean
        get() = gravitySensorCollector.sensorAvailable

    /**
     * Indicates whether this estimator will use IMU leveling using accelerometer + gyroscope
     * measurements to estimate attitude.
     */
    val isLevelingEnabled
        get() = estimateImuLeveling && (accelerometerSensorAvailable || gravitySensorAvailable)
                && gyroscopeSensorAvailable

    /**
     * Indicates whether this estimator will use improved IMU leveling using accelerometer +
     * gyroscope measurements and current device location to estimate attitude.
     */
    val isImprovedLevelingEnabled
        get() = location != null && isLevelingEnabled

    /**
     * Indicates whether this estimator will use geomagnetic attitude estimation using accelerometer
     * + magnetometer measurements along with current device location, timestamp and World Magnetic
     * Model.
     */
    val isGeomagneticAttitudeEnabled
        get() = location != null && !estimateImuLeveling
                && (accelerometerSensorAvailable || gravitySensorAvailable)
                && magnetometerSensorAvailable

    /**
     * Gets type of attitude estimator being used.
     * If estimator is not ready, null is returned.
     */
    var type: AttitudeEstimatorType? = null
        private set

    /**
     * Indicates whether this estimator is ready to start.
     */
    val isReady: Boolean
        get() = isGeomagneticAttitudeEnabled || isLevelingEnabled || isImprovedLevelingEnabled

    /**
     * Starts collecting sensor (accelerometer + magnetometer or accelerometer + gyroscope)
     * measurements to estimate attitude.
     *
     * @throws IllegalStateException if estimator is not ready.
     */
    @Throws(IllegalStateException::class)
    fun start(): Boolean {
        check(isReady)
        check(!running)

        return if (isGeomagneticAttitudeEnabled) {
            if (gravitySensorEnabled) {
                if (!gravitySensorCollector.start() || !magnetometerSensorCollector.start()) {
                    gravitySensorCollector.stop()
                    magnetometerSensorCollector.stop()
                    false
                } else {
                    gyroscopeSampleAvailable = true
                    accelerometerSampleAvailable = true
                    running = true
                    true
                }
            } else {
                if (!accelerometerSensorCollector.start() || !magnetometerSensorCollector.start()) {
                    accelerometerSensorCollector.stop()
                    magnetometerSensorCollector.stop()
                    false
                } else {
                    gyroscopeSampleAvailable = true
                    gravitySampleAvailable = true
                    running = true
                    true
                }
            }
        } else {
            if (gravitySensorEnabled) {
                if (!gravitySensorCollector.start() || !gyroscopeSensorCollector.start()) {
                    gravitySensorCollector.stop()
                    gyroscopeSensorCollector.stop()
                    false
                } else {
                    magnetometerSampleAvailable = true
                    accelerometerSampleAvailable = true
                    running = true
                    true
                }
            } else {
                if (!accelerometerSensorCollector.start() || !gyroscopeSensorCollector.start()) {
                    accelerometerSensorCollector.stop()
                    gyroscopeSensorCollector.stop()
                    false
                } else {
                    magnetometerSampleAvailable = true
                    gravitySampleAvailable = true
                    running = true
                    true
                }
            }
        }
    }

    /**
     * Stops collection of sensor measurements.
     */
    fun stop() {
        magnetometerSensorCollector.stop()
        accelerometerSensorCollector.stop()
        gyroscopeSensorCollector.stop()

        accelerometerSampleAvailable = false
        gyroscopeSampleAvailable = false
        magnetometerSampleAvailable = false

        running = false
    }

    /**
     * Obtains attitude estimator type being used.
     */
    private fun computeType() {
        type = when {
            isGeomagneticAttitudeEnabled -> AttitudeEstimatorType.GEOMAGNETIC
            isImprovedLevelingEnabled -> AttitudeEstimatorType.IMPROVED_LEVELING
            isLevelingEnabled -> AttitudeEstimatorType.LEVELING
            else -> null
        }
    }

    /**
     * Processes current samples when all required ones are available.
     */
    private fun processSamples() {
        if ((accelerometerSampleAvailable || gravitySampleAvailable) && gyroscopeSampleAvailable
            && magnetometerSampleAvailable
        ) {
            // In navigation, body frame is defined as follows:
            // x is the forward axis, pointing in the usual direction of travel;
            // z is the down axis, pointing in the usual direction of gravity;
            // and y is the right axis, completing the orthogonal set.
            // For angular motion, the body-frame axes are also known as roll, pitch, and yaw.
            // Roll motion is about the x-axis,
            // pitch motion is about the y-axis,
            // and yaw motion is about the z-axis
            // These are the NED coordinates system.

            // On the other hand, values returned by Android sensors are as follows:
            // The X axis is horizontal and points to the right,
            // the Y axis is vertical and points up and the Z axis points towards the outside
            // of the front face of the screen. In this system, coordinates behind the screen
            // have negative Z values

            // Consequently, to convert to NED coordinates, we have to negate z-axis, and
            // exchange x and y axes
            // [x2] = [0    1   0 ][x1] = [y1 ]
            // [y2]   [1    0   0 ][y1]   [x1 ]
            // [z2]   [0    0   -1][z1]   [-z1]

            // Additionally, if screen is rotated an angle of theta, then sensor coordinates
            // need to be rotated as well
            //[x3] = [cosTheta -sinTheta   0][x2] = [x2*cosTheta - y2*sinTheta]
            //[y3]   [sinTheta cosTheta    0][y2]   [x2*sinTheta + y2*cosTheta]
            //[z3]   [0        0           1][z2]   [z2]

            val fx: Double
            val fy: Double
            val fz: Double
            if (gravitySensorEnabled) {

                // gravity sensor obtains amount of gravity on each coordinate, but sensed specific
                // force is the negated gravity vector f = -g
                fx = -sensorGx.toDouble()
                fy = -sensorGy.toDouble()
                fz = -sensorGz.toDouble()

            } else {
                fx = sensorAx.toDouble()
                fy = sensorAy.toDouble()
                fz = sensorAz.toDouble()
            }

            if (isGeomagneticAttitudeEnabled) {
                processGeomagneticAttitude(fx, fy, fz)
                magnetometerSampleAvailable = false
            } else if (isImprovedLevelingEnabled) {
                processImprovedLeveling(fx, fy, fz)
                gyroscopeSampleAvailable = false
            } else {
                processLeveling(fx, fy, fz)
                gyroscopeSampleAvailable = false
            }
            if (gravitySensorEnabled) {
                gravitySampleAvailable = false
            } else {
                accelerometerSampleAvailable = false
            }

            // notify
            val type = this.type ?: return
            attitudeAvailableListener?.onAttitudeAvailable(attitude, coordinateTransformation, type)
        }
    }

    /**
     * Process accelerometer and magnetometer measurements along with current device location and
     * timestamp to estimate attitude.
     *
     * @param fx x-coordinate of measured body specific force expressed in meters per squared
     * second (m/s^2).
     * @param fy y-coordinate of measured body specific force expressed in meters per squared
     * second (m/s^2).
     * @param fz z-coordinate of measured body specific force expressed in meters per squared
     * second (m/s^2).
     */
    private fun processGeomagneticAttitude(fx: Double, fy: Double, fz: Double) {
        val nedPosition = this.nedPosition ?: return
        val attitudeEstimator = this.attitudeEstimator ?: return
        val displayRotationRadians = DisplayOrientationHelper.getDisplayRotationRadians(context)
        displayOrientation.setFromEulerAngles(0.0, 0.0, displayRotationRadians)

        val bx = MagneticFluxDensityConverter.microTeslaToTesla(sensorBx.toDouble())
        val by = MagneticFluxDensityConverter.microTeslaToTesla(sensorBy.toDouble())
        val bz = MagneticFluxDensityConverter.microTeslaToTesla(sensorBz.toDouble())

        val roll = LevelingEstimator.getRoll(fy, fz)
        val pitch = LevelingEstimator.getPitch(fx, fy, fz)
        val yaw = AttitudeEstimator.getYaw(bx, by, bz, 0.0, roll, pitch)

        attitude.setFromEulerAngles(roll, pitch, yaw)
        attitude.combine(displayOrientation)
        attitude.inverse()
        attitude.normalize()
        coordinateTransformation.fromRotation(attitude)

        /*val normF = doubleArrayOf(fx, fy, fz)
        ArrayUtils.normalize(normF)

        val nedGravity =
            NEDGravityEstimator.estimateGravityAndReturnNew(
                nedPosition.latitude,
                nedPosition.height
            )

        val normG = nedGravity.asArray()

        ArrayUtils.multiplyByScalar(normG, -1.0, normG)

        ArrayUtils.normalize(normG)

        val cosAlpha = ArrayUtils.dotProduct(normF, normG)

        val skew = Utils.skewMatrix(normG)
        val tmp1 = Matrix.newFromArray(normF)
        val tmp2 = skew.multiplyAndReturnNew(tmp1)

        val sinAlpha = Utils.normF(tmp2)

        val axis = tmp2.toArray()
        ArrayUtils.normalize(axis)

        val alpha = Math.atan2(sinAlpha, cosAlpha)

        attitude.setFromAxisAndRotation(axis, alpha)


        val roll = LevelingEstimator.getRoll(fy, fz)
        val pitch = LevelingEstimator.getPitch(fx, fy, fz)
        val yaw = AttitudeEstimator.getYaw(bx, by, bz, 0.0, roll, pitch)

        attitude.setFromEulerAngles(roll, pitch, yaw)
        attitude.inverse()
        attitude.normalize()
        attitude.asInhomogeneousMatrix(rotationMatrix)
        try {
            coordinateTransformation.matrix = rotationMatrix
        } catch (ignore: InvalidRotationMatrixException) {

        }

        attitude.inverse()
        val eulerAngles = attitude.toEulerAngles()
        val roll = eulerAngles[0]
        val pitch = eulerAngles[1]
        val yaw = AttitudeEstimator.getYaw(bx, by, bz, 0.0, roll, pitch)
        attitude.setFromEulerAngles(roll, pitch, yaw)
        attitude.inverse()*/

        /*attitudeEstimator.getAttitude(
            nedPosition.latitude,
            nedPosition.longitude,
            nedPosition.height,
            timestamp,
            fx,
            fy,
            fz,
            bx,
            by,
            bz,
            coordinateTransformation
        )
        coordinateTransformation.inverse()
        coordinateTransformation.asRotation(attitude)*/
    }

    /**
     * Processes accelerometer and gyroscope measurements along with current device location to
     * estimate attitude by means of improved leveling.
     *
     * @param fx x-coordinate of measured body specific force expressed in meters per squared
     * second (m/s^2).
     * @param fy y-coordinate of measured body specific force expressed in meters per squared
     * second (m/s^2).
     * @param fz z-coordinate of measured body specific force expressed in meters per squared
     * second (m/s^2).
     */
    private fun processImprovedLeveling(fx: Double, fy: Double, fz: Double) {
        val wx = sensorWy.toDouble()
        val wy = sensorWx.toDouble()
        val wz = -sensorWz.toDouble()
        LevelingEstimator2.getAttitude(
            nedPosition,
            fx,
            fy,
            fz,
            wx,
            wy,
            wz,
            coordinateTransformation
        )
        coordinateTransformation.inverse()
        coordinateTransformation.asRotation(attitude)
    }

    /**
     * Processes accelerometer and gyroscope measurements to estimate attitude by means of leveling.
     *
     * @param fx x-coordinate of measured body specific force expressed in meters per squared
     * second (m/s^2).
     * @param fy y-coordinate of measured body specific force expressed in meters per squared
     * second (m/s^2).
     * @param fz z-coordinate of measured body specific force expressed in meters per squared
     * second (m/s^2).
     */
    private fun processLeveling(fx: Double, fy: Double, fz: Double) {
        val wx = sensorWy.toDouble()
        val wy = sensorWx.toDouble()
        val wz = -sensorWz.toDouble()
        LevelingEstimator.getAttitude(
            fx,
            fy,
            fz,
            wx,
            wy,
            wz,
            coordinateTransformation
        )
        coordinateTransformation.inverse()
        coordinateTransformation.asRotation(attitude)
    }

    /**
     * Identifies the type of attitude estimation being used by [AttitudeEstimator].
     */
    enum class AttitudeEstimatorType {
        /**
         * Only accelerometer + gyroscope measurements are used.
         * It is robust against electromagnetic interference.
         * Gyroscope must be accurate to obtain accurate results.
         * Has a temporal drift caused by gyroscope.
         */
        LEVELING,

        /**
         * Accelerometer + gyroscope and location are used to obtain an improved leveling.
         * It is robust against electromagnetic interference.
         * Gyroscope must be accurate to obtain accurate results.
         * Has a temporal drift caused by gyroscope.
         */
        IMPROVED_LEVELING,

        /**
         * Accelerometer + magnetometer measurements are used along with current location and
         * timestamp.
         * It is not robust against electromagnetic interference.
         * Magnetometer usually is not very accurate.
         * Does not have temporal drift.
         */
        GEOMAGNETIC
    }

    /**
     * Interface to notify when a new attitude measurement is available.
     */
    fun interface OnAttitudeAvailableListener {
        /**
         * Called when a new attitude measurement is available.
         *
         * @param attitude measured attitude expressed in NED coordinates.
         * @param coordinateTransformation coordinate transformation containing measured attitude.
         * @param type type of attitude estimation being used.
         */
        fun onAttitudeAvailable(
            attitude: Quaternion,
            coordinateTransformation: CoordinateTransformation,
            type: AttitudeEstimatorType
        )
    }
}