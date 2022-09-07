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
import android.location.Location
import com.irurueta.algebra.ArrayUtils
import com.irurueta.android.navigation.inertial.ENUtoNEDTriadConverter
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorCollector
import com.irurueta.android.navigation.inertial.collectors.MagnetometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.estimators.filter.AveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.toNEDPosition
import com.irurueta.geometry.EuclideanTransformation3D
import com.irurueta.geometry.Point3D
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.ECEFFrame
import com.irurueta.navigation.frames.NEDFrame
import com.irurueta.navigation.frames.NEDVelocity
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter
import com.irurueta.navigation.inertial.BodyKinematics
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.navigation.inertial.estimators.ECEFKinematicsEstimator
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel
import java.util.*

/**
 * Estimates pose with 6DOF (Degrees of Freedom) containing absolute attitude and position.
 *
 * @property context Android context.
 * @property sensorDelay Delay of sensors between samples.
 * @property accelerometerSensorType One of the supported accelerometer sensor types. It is
 * suggested to avoid using non-calibrated accelerometers.
 * @property magnetometerSensorType One of the supported magnetometer sensor types.
 * @property accelerometerAveragingFilter an averaging filter for accelerometer samples to obtain
 * sensed gravity component of specific force.
 * @property gyroscopeSensorType One of the supported gyroscope sensor types. It is suggested to
 * avoid using non-calibrated gyroscopes.
 * @property estimateInitialTransformation true to estimate 3D metric transformation between
 * current frame and initial frame, false to skip transformation computation.
 * @property estimatePreviousTransformation true to estimate 3D metric transformation between
 * current and previous frames, false to skip transformation computation.
 * @property poseAvailableListener notifies when a new estimated pose is available.
 * @property accelerometerMeasurementListener listener to notify new accelerometer measurements.
 * @property gyroscopeMeasurementListener listener to notify new gyroscope measurements.
 * @property magnetometerMeasurementListener listener to notify new magnetometer measurements.
 * @property gravityEstimationListener listener to notify when a new gravity estimation is
 * available.
 * @property initialVelocity initial velocity of device expressed in NED coordinates.
 */
class PoseEstimator private constructor(
    val context: Context,
    val sensorDelay: SensorDelay,
    val accelerometerSensorType: AccelerometerSensorCollector.SensorType,
    val magnetometerSensorType: MagnetometerSensorCollector.SensorType,
    val accelerometerAveragingFilter: AveragingFilter,
    val gyroscopeSensorType: GyroscopeSensorCollector.SensorType,
    val estimateInitialTransformation: Boolean,
    val estimatePreviousTransformation: Boolean,
    var poseAvailableListener: OnPoseAvailableListener?,
    var accelerometerMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener?,
    var gyroscopeMeasurementListener: GyroscopeSensorCollector.OnMeasurementListener?,
    var gravityEstimationListener: GravityEstimator.OnEstimationListener?,
    val initialVelocity: NEDVelocity,
    _location: Location
) {

    /**
     * Constructor.
     *
     * @param context Android context.
     * @param location current device location.
     * @param initialVelocity initial velocity of device expressed in NED coordinates.
     * @param sensorDelay Delay of sensors between samples.
     * @param accelerometerSensorType One of the supported accelerometer sensor types. It is
     * suggested to avoid using non-calibrated accelerometers.
     * @param magnetometerSensorType One of the supported magnetometer sensor types.
     * @param accelerometerAveragingFilter an averaging filter for accelerometer samples to obtain
     * sensed gravity component of specific force.
     * @param gyroscopeSensorType One of the supported gyroscope sensor types. It is suggested to
     * avoid using non-calibrated gyroscopes.
     * @param worldMagneticModel Earth's magnetic model. Null to use default model
     * when [useWorldMagneticModel] is true. If [useWorldMagneticModel] is false, this is ignored.
     * @param timestamp Timestamp when World Magnetic Model will be evaluated to obtain current.
     * Only taken into account if [useWorldMagneticModel] is tue.
     * @param useWorldMagneticModel true so that world magnetic model is taken into account to
     * adjust attitude yaw angle by current magnetic declination based on current World Magnetic
     * Model, location and timestamp, false to ignore declination.
     * @param useAccurateLevelingEstimator true to use accurate leveling, false to use a normal one.
     * @param useAccurateRelativeGyroscopeAttitudeEstimator true to use accurate non-leveled
     * relative attitude estimator, false to use non-accurate non-leveled relative attitude
     * estimator.
     * @param estimateInitialTransformation true to estimate 3D metric transformation between
     * current frame and initial frame, false to skip transformation computation.
     * @param estimatePreviousTransformation true to estimate 3D metric transformation between
     * current and previous frames, false to skip transformation computation.
     * @param poseAvailableListener notifies when a new estimated pose is available.
     * @param accelerometerMeasurementListener listener to notify new accelerometer measurements.
     * @param gyroscopeMeasurementListener listener to notify new gyroscope measurements.
     * @param magnetometerMeasurementListener listener to notify new magnetometer measurements.
     * @param gravityEstimationListener listener to notify when a new gravity estimation is
     * available.
     */
    constructor(
        context: Context,
        location: Location,
        initialVelocity: NEDVelocity = NEDVelocity(),
        sensorDelay: SensorDelay = SensorDelay.GAME,
        accelerometerSensorType: AccelerometerSensorCollector.SensorType =
            AccelerometerSensorCollector.SensorType.ACCELEROMETER,
        magnetometerSensorType: MagnetometerSensorCollector.SensorType =
            MagnetometerSensorCollector.SensorType.MAGNETOMETER,
        accelerometerAveragingFilter: AveragingFilter = LowPassAveragingFilter(),
        gyroscopeSensorType: GyroscopeSensorCollector.SensorType =
            GyroscopeSensorCollector.SensorType.GYROSCOPE,
        worldMagneticModel: WorldMagneticModel? = null,
        timestamp: Date = Date(),
        useWorldMagneticModel: Boolean = false,
        useAccurateLevelingEstimator: Boolean = true,
        useAccurateRelativeGyroscopeAttitudeEstimator: Boolean = true,
        estimateInitialTransformation: Boolean = false,
        estimatePreviousTransformation: Boolean = false,
        poseAvailableListener: OnPoseAvailableListener? = null,
        accelerometerMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener? = null,
        gyroscopeMeasurementListener: GyroscopeSensorCollector.OnMeasurementListener? = null,
        magnetometerMeasurementListener: MagnetometerSensorCollector.OnMeasurementListener? = null,
        gravityEstimationListener: GravityEstimator.OnEstimationListener? = null
    ) : this(
        context,
        sensorDelay,
        accelerometerSensorType,
        magnetometerSensorType,
        accelerometerAveragingFilter,
        gyroscopeSensorType,
        estimateInitialTransformation,
        estimatePreviousTransformation,
        poseAvailableListener,
        accelerometerMeasurementListener,
        gyroscopeMeasurementListener,
        gravityEstimationListener,
        initialVelocity,
        location
    ) {
        this.magnetometerMeasurementListener = magnetometerMeasurementListener
        this.location = location
        this.worldMagneticModel = worldMagneticModel
        this.timestamp = timestamp
        this.useWorldMagneticModel = useWorldMagneticModel
        this.useAccurateLevelingEstimator = useAccurateLevelingEstimator
        this.useAccurateRelativeGyroscopeAttitudeEstimator =
            useAccurateRelativeGyroscopeAttitudeEstimator
    }

    /**
     * Indicates whether Earth rotation is available or not.
     */
    private var earthRotationAvailable = false

    /**
     * Body kinematics containing las accelerometer and gyroscope measurements.
     * This is reused for performance reasons.
     */
    private val bodyKinematics = BodyKinematics()

    /**
     * Acceleration measured by accelerometer sensor.
     */
    private val acceleration = AccelerationTriad()

    /**
     * Angular speed measured by gyroscope sensor.
     */
    private val angularSpeed = AngularSpeedTriad()

    /**
     * Current estimated specific force related to gravity at current location.
     */
    private val gravity = AccelerationTriad()

    /**
     * True indicates that estimator has been initialized with an initial frame,
     * containing estimated absolute attitude and current device location (if available).
     */
    private var initializedFrame = false

    /**
     * Device frame expressed in ECEF coordinates when estimator starts.
     * This is reused for performance reasons.
     */
    private val initialEcefFrame = ECEFFrame()

    /**
     * Device frame expressed in NED coordinates when estimator starts.
     * This is reused for performance reasons.
     */
    private val initialNedFrame = NEDFrame()

    /**
     * Device frame expressed in ECEF coordinates for previous set of measurements.
     * This is reused for performance reasons.
     */
    private val previousEcefFrame = ECEFFrame()

    /**
     * Current device frame expressed in ECEF coordinates.
     * This is reused for performance reasons.
     */
    private val currentEcefFrame = ECEFFrame()

    /**
     * 3D metric transformation relating initial ECEF frame and current ECEF frame.
     * This is reused for performance reasons.
     */
    private val initialTransformation = EuclideanTransformation3D()

    /**
     * 3D metric transformation relating previous and current ECEF frames.
     * This is reused for performance reasons.
     */
    private val previousTransformation = EuclideanTransformation3D()

    /**
     * Rotation being reused to compute a 3D metric transformation.
     */
    private val startRotation = Quaternion()

    /**
     * Inverse rotation being reused to compute a 3D metric transformation.
     */
    private val inverseStartRotation = Quaternion()

    /**
     * Rotation being reused to compute a 3D metric transformation.
     */
    private val endRotation = Quaternion()

    /**
     * Rotation being reused to compute a 3D metric transformation.
     */
    private val deltaRotation = Quaternion()

    /**
     * Point to be reused.
     */
    private val point = Point3D.create()

    /**
     * Listener to notify new magnetometer measurements.
     */
    var magnetometerMeasurementListener: MagnetometerSensorCollector.OnMeasurementListener? = null
        set(value) {
            field = value
            absoluteAttitudeEstimator.magnetometerMeasurementListener = value
        }

    /**
     * Gets or sets device location
     *
     * @throws IllegalStateException if estimator is running and a null value is set.
     */
    var location: Location = _location
        set(value) {
            field = value
            absoluteAttitudeEstimator.location = value
        }

    /**
     * Earth's magnetic model. If null, the default model is used if [useWorldMagneticModel] is
     * true. If [useWorldMagneticModel] is false, this is ignored.
     *
     * @throws IllegalStateException if estimator is running.
     */
    var worldMagneticModel: WorldMagneticModel? = null
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
            absoluteAttitudeEstimator.worldMagneticModel = value
        }

    /**
     * Timestamp when World Magnetic Model will be evaluated to obtain current.
     * Only taken into account if [useWorldMagneticModel] is tue.
     */
    var timestamp: Date = Date()
        set(value) {
            field = value
            absoluteAttitudeEstimator.timestamp = value
        }

    /**
     * Indicates whether world magnetic model is taken into account to adjust attitude yaw angle by
     * current magnetic declination based on current Wolrd MAgnetic Model, location and timestamp.
     */
    var useWorldMagneticModel: Boolean = false
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
            absoluteAttitudeEstimator.useWorldMagneticModel = value
        }

    /**
     * Indicates whether accurate leveling must be used or not.
     *
     * @throws IllegalStateException if estimator is running.
     */
    var useAccurateLevelingEstimator: Boolean = false
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)

            field = value
            absoluteAttitudeEstimator.useAccurateLevelingEstimator = value
        }

    /**
     * Indicates whether accurate non-leveled relative attitude estimator must be used or not.
     *
     * @throws IllegalStateException if estimator is running.
     */
    var useAccurateRelativeGyroscopeAttitudeEstimator: Boolean = false
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)

            field = value
            absoluteAttitudeEstimator.useAccurateRelativeGyroscopeAttitudeEstimator = value
        }

    private val ecefStaticKinematics = BodyKinematics()

    private val previousAttitude = Quaternion()

    private val inversePreviousAttitude = Quaternion()

    private val deltaAttitude = Quaternion()

    private val inverseDeltaAttitude = Quaternion()

    private val inverseEulerAngles = DoubleArray(Quaternion.N_ANGLES)

    private val initialAttitude = Quaternion()

    /**
     * Estimates device absolute attitude using accelerometer, gyroscope and magnetometer sensors.
     */
    private val absoluteAttitudeEstimator = FusedGeomagneticAttitudeEstimator(
        context,
        location,
        sensorDelay,
        true,
        accelerometerSensorType,
        magnetometerSensorType,
        accelerometerAveragingFilter,
        gyroscopeSensorType,
        worldMagneticModel,
        timestamp,
        useWorldMagneticModel,
        useAccurateLevelingEstimator,
        useAccurateRelativeGyroscopeAttitudeEstimator,
        estimateCoordinateTransformation = true,
        estimateEulerAngles = false,
        attitudeAvailableListener = { estimator, attitude, timestamp, _, _, _, coordinateTransformation ->

            val timeInterval = estimator.gyroscopeAverageTimeInterval

            if (!initialize(attitude, coordinateTransformation)) {
                return@FusedGeomagneticAttitudeEstimator
            }

            previousAttitude.inverse(inversePreviousAttitude)
            Quaternion.product(attitude, inversePreviousAttitude, deltaAttitude)
            deltaAttitude.inverse(inverseDeltaAttitude)

            inverseDeltaAttitude.toEulerAngles(inverseEulerAngles)

            val invTimeInterval = 1.0 / timeInterval
            ArrayUtils.multiplyByScalar(inverseEulerAngles, invTimeInterval, inverseEulerAngles)

            angularSpeed.setValueCoordinates(
                inverseEulerAngles[0],
                inverseEulerAngles[1],
                inverseEulerAngles[2]
            )

            ECEFKinematicsEstimator.estimateKinematics(
                timeInterval,
                previousEcefFrame,
                previousEcefFrame,
                ecefStaticKinematics
            )

            // compensate measured gravity respect theoretical gravity
            bodyKinematics.fx = acceleration.valueX - gravity.valueX + ecefStaticKinematics.fx
            bodyKinematics.fy = acceleration.valueY - gravity.valueY + ecefStaticKinematics.fy
            bodyKinematics.fz = acceleration.valueZ - gravity.valueZ + ecefStaticKinematics.fz

            // compensate measured angular speed respect theoretical Earth rotation
            bodyKinematics.angularRateX = angularSpeed.valueX
            bodyKinematics.angularRateY = angularSpeed.valueY
            bodyKinematics.angularRateZ = angularSpeed.valueZ

            ECEFInertialNavigator.navigateECEF(
                timeInterval,
                previousEcefFrame,
                bodyKinematics,
                currentEcefFrame
            )

            // compute transformation between initial and current frame and previous and current frame
            if (estimateInitialTransformation) {
                computeTransformation(initialEcefFrame, currentEcefFrame, initialTransformation)
            }
            if (estimatePreviousTransformation) {
                computeTransformation(previousEcefFrame, currentEcefFrame, previousTransformation)
            }

            // notify
            val initialTransformation =
                if (estimateInitialTransformation) initialTransformation else null
            val previousTransformation =
                if (estimatePreviousTransformation) previousTransformation else null

            poseAvailableListener?.onPoseAvailable(
                this,
                currentEcefFrame,
                previousEcefFrame,
                initialEcefFrame,
                attitude,
                previousAttitude,
                initialAttitude,
                timestamp,
                initialTransformation,
                previousTransformation
            )

            // update previous frame
            previousEcefFrame.copyFrom(currentEcefFrame)
            previousAttitude.fromQuaternion(attitude)
        },
        accelerometerMeasurementListener = { ax, ay, az, bx, by, bz, timestamp, accuracy ->

            val currentAx = if (bx != null)
                ax.toDouble() - bx.toDouble()
            else
                ax.toDouble()
            val currentAy = if (by != null)
                ay.toDouble() - by.toDouble()
            else
                ay.toDouble()
            val currentAz = if (bz != null)
                az.toDouble() - bz.toDouble()
            else
                az.toDouble()

            ENUtoNEDTriadConverter.convert(currentAx, currentAy, currentAz, acceleration)

            /*val fx = -(ax - (bx ?: 0.0f)).toDouble()
            val fy = -(ay - (by ?: 0.0f)).toDouble()
            val fz = -(az - (bz ?: 0.0f)).toDouble()
            acceleration.setValueCoordinates(fx, fy, fz)*/

            accelerometerMeasurementListener?.onMeasurement(
                ax,
                ay,
                az,
                bx,
                by,
                bz,
                timestamp,
                accuracy
            )
        },
        gyroscopeMeasurementListener = { wx, wy, wz, bx, by, bz, timestamp, accuracy ->
            val currentWx = if (bx != null)
                wx.toDouble() - bx.toDouble()
            else
                wx.toDouble()

            val currentWy = if (by != null)
                wy.toDouble() - by.toDouble()
            else
                wy.toDouble()

            val currentWz = if (bz != null)
                wz.toDouble() - bz.toDouble()
            else
                wz.toDouble()

            ENUtoNEDTriadConverter.convert(currentWx, currentWy, currentWz, angularSpeed)

            /*val angularRateX = -(wx - (bx ?: 0.0f)).toDouble()
            val angularRateY = -(wy - (by ?: 0.0f)).toDouble()
            val angularRateZ = -(wz - (bz ?: 0.0f)).toDouble()
            angularSpeed.setValueCoordinates(angularRateX, angularRateY, angularRateZ)*/

            gyroscopeMeasurementListener?.onMeasurement(
                wx,
                wy,
                wz,
                bx,
                by,
                bz,
                timestamp,
                accuracy
            )
        },
        gravityEstimationListener = { estimator, fx, fy, fz, timestamp ->
            gravity.setValueCoordinates(fx, fy, fz)
            // TODO: keep estimated gravity
            gravityEstimationListener?.onEstimation(estimator, fx, fy, fz, timestamp)
        },
        magnetometerMeasurementListener = magnetometerMeasurementListener
    )

    /**
     * Indicates whether this estimator is running or not.
     */
    var running: Boolean = false
        private set

    /**
     * Gets average time interval between gyroscope samples expressed in seconds.
     */
    val averageTimeInterval
        get() = absoluteAttitudeEstimator.gyroscopeAverageTimeInterval

    /**
     * Starts this estimator.
     *
     * @return true if estimator successfully started, false otherwise.
     * @throws IllegalStateException if estimator is already running.
     */
    @Throws(IllegalStateException::class)
    fun start(): Boolean {
        check(!running)

        reset()
        running = absoluteAttitudeEstimator.start()
        if (!running) {
            stop()
        }

        return running
    }

    /**
     * Stops this estimator.
     */
    fun stop() {
        absoluteAttitudeEstimator.stop()
        running = false
    }

    /**
     * Resets internal parameters.
     */
    private fun reset() {
        initializedFrame = false
        earthRotationAvailable = false
    }

    /**
     * Estimates initial ECEF frame when the first absolute attitude is obtained after starting
     * this estimator.
     *
     * @param attitude first obtained attitude.
     * @param coordinateTransformation first obtained coordinate transformation containing initial
     * absolute attitude.
     * @return true if this estimator was already initialized, false if it wasn't.
     */
    private fun initialize(
        attitude: Quaternion,
        coordinateTransformation: CoordinateTransformation?
    ): Boolean {
        val result = initializedFrame
        if (!initializedFrame && coordinateTransformation != null) {

            initialNedFrame.coordinateTransformation = coordinateTransformation
            initialNedFrame.position = location.toNEDPosition()
            initialNedFrame.velocity = initialVelocity

            NEDtoECEFFrameConverter.convertNEDtoECEF(initialNedFrame, initialEcefFrame)

            initialEcefFrame.copyTo(previousEcefFrame)

            initialAttitude.fromQuaternion(attitude)
            previousAttitude.fromQuaternion(attitude)

            initializedFrame = true
        }
        return result
    }

    /**
     * Computes 3D metric transformation between procided ECEF frames.
     *
     * @param startFrame starting frame.
     * @param endFrame end frame.
     * @param result instance where result will be stored.
     */
    private fun computeTransformation(
        startFrame: ECEFFrame,
        endFrame: ECEFFrame,
        result: EuclideanTransformation3D,
    ) {
        // deltaRotation * startRotation = endRotation
        // deltaRotation = endRotation * invStartRotation

        // transformedPoint = deltaRotation * (point - startPoint) + endPoint
        // transformedPoint = [x' y' z' 1]^T
        // point = [x y z 1] ^T
        // [x'] = deltaR * [x] - deltaR * startPoint + endPoint
        // [y']            [y]
        // [z']            [z]
        // [1 ]

        // transformedPoint = [deltaR      -deltaR * startPoint + endPoint] * point
        //                    [0^T          1                             ]

        // T = [deltaR      -deltaR * startPoint + endPoint]
        //     [0^T          1                             ]

        startFrame.coordinateTransformation.asRotation(startRotation)
        startRotation.normalize()
        endFrame.coordinateTransformation.asRotation(endRotation)
        endRotation.normalize()
        startRotation.inverse(inverseStartRotation)
        inverseStartRotation.normalize()
        Quaternion.product(endRotation, inverseStartRotation, deltaRotation)
        deltaRotation.normalize()
        result.rotation.fromRotation(deltaRotation)

        val startX = startFrame.x
        val startY = startFrame.y
        val startZ = startFrame.z
        val endX = endFrame.x
        val endY = endFrame.y
        val endZ = endFrame.z

        point.setInhomogeneousCoordinates(startX, startY, startZ)
        deltaRotation.rotate(point, point)
        val rotX = point.inhomX
        val rotY = point.inhomY
        val rotZ = point.inhomZ

        result.translationX = -rotX + endX
        result.translationY = -rotY + endY
        result.translationZ = -rotZ + endZ
    }

    /**
     * Interface t notify when a new pose is available.
     */
    fun interface OnPoseAvailableListener {

        /**
         * Called when a new pose is available.
         *
         * @param estimator pose estimator that raised this event.
         * @param currentFrame current ECEF frame containing device position, velocity and attitude.
         * @param previousFrame ECEF frame of previous measurement.
         * @param initialFrame initial ECEF frame when estimator was started.
         * @param timestamp time in nanoseconds at which the measurement was made. Each measurement
         * wil be monotonically increasing using the same time base as
         * [android.os.SystemClock.elapsedRealtimeNanos].
         * @param initialTransformation 3D metric transformation relating initial and current
         * ECEF frames.
         * @param previousTransformation 3D metric transformation relating previous and current
         * ECEF frames.
         */
        fun onPoseAvailable(
            estimator: PoseEstimator,
            currentFrame: ECEFFrame,
            previousFrame: ECEFFrame,
            initialFrame: ECEFFrame,
            currentAttitude: Quaternion,
            previousAttitude: Quaternion,
            initialAttitude: Quaternion,
            timestamp: Long,
            initialTransformation: EuclideanTransformation3D?,
            previousTransformation: EuclideanTransformation3D?
        )
    }
}