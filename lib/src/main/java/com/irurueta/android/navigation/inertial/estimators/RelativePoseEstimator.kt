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
import com.irurueta.algebra.Matrix
import com.irurueta.android.navigation.inertial.ENUtoNEDTriadConverter
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.estimators.filter.AveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.LowPassAveragingFilter
import com.irurueta.geometry.EuclideanTransformation3D
import com.irurueta.geometry.InhomogeneousPoint3D
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad

/**
 * Estimates pose with 6DOF (Degrees of Freedom) containing relative attitude from an unknown
 * location.
 *
 * @property context Android context.
 * @property sensorDelay Delay of sensors between samples.
 * @property accelerometerSensorType One of the supported accelerometer sensor types. It is
 * suggested to avoid using non-calibrated accelerometers.
 * @property gyroscopeSensorType One of the supported gyroscope sensor types. It is suggested to
 * avoid using non-calibrated gyroscopes.
 * @property accelerometerAveragingFilter an averaging filter for accelerometer samples to obtain
 * sensed gravity component of specific force.
 * @property poseAvailableListener notifies when a new estimated pose is available.
 * @property accelerometerMeasurementListener listener to notify new accelerometer measurements.
 * @property gyroscopeMeasurementListener listener to notify new gyroscope measurements.
 * @property gravityEstimationListener Listener to notify when a new gravity estimation is
 * available.
 */
class RelativePoseEstimator private constructor(
    override val context: Context,
    override val sensorDelay: SensorDelay,
    override val accelerometerSensorType: AccelerometerSensorCollector.SensorType,
    override val gyroscopeSensorType: GyroscopeSensorCollector.SensorType,
    override val accelerometerAveragingFilter: AveragingFilter,
    var poseAvailableListener: OnPoseAvailableListener?,
    override var accelerometerMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener?,
    override var gyroscopeMeasurementListener: GyroscopeSensorCollector.OnMeasurementListener?,
    override var gravityEstimationListener: GravityEstimator.OnEstimationListener?
) : PoseEstimator {

    /**
     * Constructor.
     *
     * @param context Android context.
     * @param sensorDelay Delay of sensors between samples.
     * @param accelerometerSensorType One of the supported accelerometer sensor types. It is
     * suggested to avoid using non-calibrated accelerometers.
     * @param gyroscopeSensorType One of the supported gyroscope sensor types. It is suggested to
     * avoid using non-calibrated gyroscopes.
     * @param accelerometerAveragingFilter an averaging filter for accelerometer samples to obtain
     * sensed gravity component of specific force.
     * @param useAccurateLevelingEstimator true to use accurate leveling, false to use a normal one.
     * @param useAccurateRelativeGyroscopeAttitudeEstimator true to use accurate relative attitude,
     * false to use a normal one.
     * @param poseAvailableListener notifies when a new estimated pose is available.
     * @param accelerometerMeasurementListener listener to notify new accelerometer measurements.
     * @param gyroscopeMeasurementListener listener to notify new gyroscope measurements.
     * @param gravityEstimationListener listener to notify when a new gravity estimation is
     * available.
     * @param initialSpeed initial device speed expressed in NED system of coordinates. If not
     * provided it is assumed that initial speed is zero.
     * @param initialLocation initial device location when estimator starts. Only when an initial
     * location is provided [useAccurateLevelingEstimator] is actually taken into account,
     * otherwise it is ignored.
     */
    constructor(
        context: Context,
        sensorDelay: SensorDelay = SensorDelay.GAME,
        accelerometerSensorType: AccelerometerSensorCollector.SensorType =
            AccelerometerSensorCollector.SensorType.ACCELEROMETER,
        gyroscopeSensorType: GyroscopeSensorCollector.SensorType =
            GyroscopeSensorCollector.SensorType.GYROSCOPE,
        accelerometerAveragingFilter: AveragingFilter = LowPassAveragingFilter(),
        useAccurateLevelingEstimator: Boolean = false,
        useAccurateRelativeGyroscopeAttitudeEstimator: Boolean = true,
        poseAvailableListener: OnPoseAvailableListener? = null,
        accelerometerMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener? = null,
        gyroscopeMeasurementListener: GyroscopeSensorCollector.OnMeasurementListener? = null,
        gravityEstimationListener: GravityEstimator.OnEstimationListener? = null,
        initialSpeed: SpeedTriad = SpeedTriad(),
        initialLocation: Location? = null
    ) : this(
        context,
        sensorDelay,
        accelerometerSensorType,
        gyroscopeSensorType,
        accelerometerAveragingFilter,
        poseAvailableListener,
        accelerometerMeasurementListener,
        gyroscopeMeasurementListener,
        gravityEstimationListener
    ) {
        buildAttitudeEstimator()
        this.initialSpeed = initialSpeed
        this.initialLocation = initialLocation
        this.useAccurateLevelingEstimator = useAccurateLevelingEstimator
        this.useAccurateRelativeGyroscopeAttitudeEstimator =
            useAccurateRelativeGyroscopeAttitudeEstimator
    }

    /**
     * Current estimated specific force related to gravity at current location.
     */
    private val gravity = AccelerationTriad()

    /**
     * Acceleration measured by accelerometer sensor.
     */
    private val acceleration = AccelerationTriad()

    /**
     * Angular speed measured by gyroscope sensor.
     */
    private val angularSpeed = AngularSpeedTriad()

    /**
     * Indicates whether estimator has been initialized.
     */
    private var initialized = false

    /**
     * Previous attitude in last sample.
     */
    private val previousAttitude = Quaternion()

    /**
     * Current attitude.
     */
    private val currentAttitude = Quaternion()

    /**
     * Attitude when estimator starts.
     */
    private val initialAttitude = Quaternion()

    /**
     * Average attitude between prvious and current attitudes.
     */
    private val averageAttitude = Quaternion()

    /**
     * Contains specific force in NED body coordinates. This is reused for performance reasons.
     */
    private val fibb = Matrix(AccelerationTriad.COMPONENTS, 1)

    /**
     * Contains average attitude expressed in matrix form. This is reused for performance reasons.
     */
    private val avgAttitudeMatrix = Matrix(Quaternion.N_ANGLES, Quaternion.N_ANGLES)

    /**
     * Contains specific force in NED coordinates. This is reused for performance reasons.
     */
    private val fibn = Matrix(AccelerationTriad.COMPONENTS, 1)

    /**
     * Previous speed in last sample.
     */
    private val previousSpeed = SpeedTriad()

    /**
     * Current speed.
     */
    private val currentSpeed = SpeedTriad()

    /**
     * Previous position in last sample.
     */
    private val previousPosition = InhomogeneousPoint3D()

    /**
     * Current position.
     */
    private val currentPosition = InhomogeneousPoint3D()

    /**
     * Contains current pose respect to the start of this estimator.
     */
    private val poseTransformation = EuclideanTransformation3D()

    /**
     * Contains rotation in pose transformation.
     */
    private val transformationRotation = Quaternion()

    /**
     * Contains translation in pose transformation.
     */
    private val transformationPosition = InhomogeneousPoint3D()

    /**
     * Rotation to convert from NED to ENU coordinates and viceversa.
     */
    private val conversionRotation = ENUtoNEDTriadConverter.conversionRotation

    /**
     * Internal leveled relative attitude estimator.
     */
    private var attitudeEstimator: LeveledRelativeAttitudeEstimator? = null

    /**
     * Gets or sets initial device speed expressed in ENU body coordinates.
     *
     * @throws IllegalStateException if estimator is running.
     */
    var initialSpeed: SpeedTriad = SpeedTriad()
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
        }

    /**
     * Gets or sets device initial location when this estimator starts.
     * Only when an initial location is provided [useAccurateLevelingEstimator] is actually taken
     * into account, otherwise it is ignored.
     *
     * @throws IllegalStateException if estimator is running.
     */
    var initialLocation: Location? = null
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)

            attitudeEstimator?.location = value
            field = value
        }

    /**
     * Indicates whether accurate leveling must be used or not.
     *
     * @throws IllegalStateException if estimator is running.
     */
    override var useAccurateLevelingEstimator: Boolean = false
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            if (value) {
                checkNotNull(initialLocation)
            }

            attitudeEstimator?.useAccurateLevelingEstimator = value
            field = value
        }

    /**
     * Indicates whether accurate non-leveled relative attitude estimator must be used or not.
     *
     * @throws IllegalStateException if estimator is running.
     */
    override var useAccurateRelativeGyroscopeAttitudeEstimator: Boolean = true
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)

            attitudeEstimator?.useAccurateRelativeGyroscopeAttitudeEstimator = value
            field = value
        }

    /**
     * Indicates whether fusion between leveling and relative attitudes occurs based
     * on changing interpolation value that depends on actual relative attitude rotation
     * velocity.
     */
    override var useIndirectAttitudeInterpolation: Boolean = true
        set(value) {
            attitudeEstimator?.useIndirectInterpolation = value
            field = value
        }

    /**
     * Interpolation value to be used to combine both leveling and relative attitudes.
     * Must be between 0.0 and 1.0 (both included).
     * The closer to 0.0 this value is, the more resemblance the result will have to a pure
     * leveling (which feels more jerky when using accelerometer). On the contrary, the closer
     * to 1.0 this value is, the more resemblance the result will have to a pure non-leveled
     * relative attitude (which feels softer but might have arbitrary roll and pitch Euler angles).
     *
     * @throws IllegalArgumentException if value is not between 0.0 and 1.0 (both included).
     */
    override var attitudeInterpolationValue: Double =
        LeveledRelativeAttitudeEstimator.DEFAULT_INTERPOLATION_VALUE
        @Throws(IllegalArgumentException::class)
        set(value) {
            attitudeEstimator?.interpolationValue = value
            field = value
        }

    /**
     * Factor to take into account when interpolation value is computed and
     * [useIndirectAttitudeInterpolation] is enabled to determine actual interpolation value based
     * on current relative attitude rotation velocity.
     *
     * @throws IllegalArgumentException if value is zero or negative.
     */
    override var attitudeIndirectInterpolationWeight: Double =
        LeveledRelativeAttitudeEstimator.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT
        @Throws(IllegalArgumentException::class)
        set(value) {
            attitudeEstimator?.indirectInterpolationWeight = value
            field = value
        }

    /**
     * Threshold to determine that current leveling appears to be an outlier respect
     * to estimated fused attitude.
     * When leveling and fused attitudes diverge, fusion is not performed, and instead
     * only gyroscope relative attitude is used for fusion estimation.
     *
     * @throws IllegalStateException if estimator is already running.
     * @throws IllegalArgumentException if value is not between 0.0 and 1.0 (both included).
     */
    override var attitudeOutlierThreshold: Double =
        LeveledRelativeAttitudeEstimator.DEFAULT_OUTLIER_THRESHOLD
        @Throws(IllegalStateException::class, IllegalArgumentException::class)
        set(value) {
            attitudeEstimator?.outlierThreshold = value
            field = value
        }

    /**
     * Threshold to determine that leveling has largely diverged and if situation is not
     * reverted soon, attitude will be reset to leveling
     *
     * @throws IllegalStateException if estimator is already running.
     * @throws IllegalArgumentException if value is not between 0.0 and 1.0 (both included).
     */
    override var attitudeOutlierPanicThreshold: Double =
        LeveledRelativeAttitudeEstimator.DEFAULT_OUTLIER_PANIC_THRESHOLD
        @Throws(IllegalStateException::class, IllegalArgumentException::class)
        set(value) {
            attitudeEstimator?.outlierPanicThreshold = value
            field = value
        }

    /**
     * Threshold to determine when fused attitude has largely diverged for a given
     * number of samples and must be reset.
     *
     * @throws IllegalStateException if estimator is already running.
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    override var attitudePanicCounterThreshold: Int =
        LeveledRelativeAttitudeEstimator.DEFAULT_PANIC_COUNTER_THRESHOLD
        @Throws(IllegalStateException::class, IllegalArgumentException::class)
        set(value) {
            attitudeEstimator?.panicCounterThreshold = value
            field = value
        }

    /**
     * Indicates whether this estimator is running or not.
     */
    override var running: Boolean = false
        private set

    /**
     * Gets average time interval between gyroscope samples expressed in seconds.
     */
    override val averageTimeInterval
        get() = attitudeEstimator?.gyroscopeAverageTimeInterval ?: 0.0

    /**
     * Starts this estimator.
     *
     * @return true if estimator successfully started, false otherwise.
     * @throws IllegalStateException if estimator is already running.
     */
    @Throws(IllegalStateException::class)
    override fun start(): Boolean {
        check(!running)

        reset()
        running = attitudeEstimator?.start() ?: false
        if (!running) {
            stop()
        }

        return running
    }

    /**
     * Stops this estimator.
     */
    override fun stop() {
        attitudeEstimator?.stop()
        running = false
    }

    /**
     * Resets internal parameters.
     */
    private fun reset() {
        initialized = false
    }

    /**
     * Builds internal attitude estimator.
     */
    private fun buildAttitudeEstimator() {
        attitudeEstimator = LeveledRelativeAttitudeEstimator(
            context,
            initialLocation,
            sensorDelay,
            useAccelerometer = true,
            accelerometerSensorType,
            accelerometerAveragingFilter,
            gyroscopeSensorType,
            useAccurateLevelingEstimator,
            useAccurateRelativeGyroscopeAttitudeEstimator,
            estimateCoordinateTransformation = false,
            estimateEulerAngles = false,
            attitudeAvailableListener = { estimator, attitude, timestamp, _, _, _, _ ->
                if (!initialize(attitude)) {
                    return@LeveledRelativeAttitudeEstimator
                }

                val timeInterval = estimator.gyroscopeAverageTimeInterval

                // current attitude is expressed in NED coordinates system
                attitude.copyTo(currentAttitude)
                // obtain average attitude between current and previous attitude
                Quaternion.slerp(previousAttitude, currentAttitude, 0.5, averageAttitude)

                // transform specific force to NED frame
                acceleration.getValuesAsMatrix(fibb)
                averageAttitude.asInhomogeneousMatrix(avgAttitudeMatrix)
                avgAttitudeMatrix.multiply(fibb, fibn)

                val ax = fibn.getElementAtIndex(0)
                val ay = fibn.getElementAtIndex(1)
                val az = fibn.getElementAtIndex(2)

                val gx = gravity.valueX
                val gy = gravity.valueY
                val gz = gravity.valueZ

                val fx = ax - gx
                val fy = ay - gy
                val fz = az - gz

                // Update velocity
                val oldVx = previousSpeed.valueX
                val oldVy = previousSpeed.valueY
                val oldVz = previousSpeed.valueZ

                val newVx = oldVx + fx * timeInterval
                val newVy = oldVy + fy * timeInterval
                val newVz = oldVz + fz * timeInterval
                currentSpeed.setValueCoordinates(newVx, newVy, newVz)

                // Update position
                val oldX = previousPosition.inhomX
                val oldY = previousPosition.inhomY
                val oldZ = previousPosition.inhomZ

                val newX = oldX + 0.5 * (oldVx + newVx) * timeInterval
                val newY = oldY + 0.5 * (oldVy + newVy) * timeInterval
                val newZ = oldZ + 0.5 * (oldVz + newVz) * timeInterval
                currentPosition.setCoordinates(newX, newY, newZ)

                // compute transformation
                computeTransformation()

                // notify
                poseAvailableListener?.onPoseAvailable(this, timestamp, poseTransformation)

                // update previous values
                currentAttitude.copyTo(previousAttitude)
                currentSpeed.copyTo(previousSpeed)
                previousPosition.setCoordinates(
                    currentPosition.inhomX,
                    currentPosition.inhomY,
                    currentPosition.inhomY
                )
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

                gravityEstimationListener?.onEstimation(estimator, fx, fy, fz, timestamp)
            }
        )
    }

    /**
     * Computes pose transformation using current attitude and position.
     */
    private fun computeTransformation() {
        // convert attitude to ENU coordinates
        Quaternion.product(currentAttitude, conversionRotation, transformationRotation)
        // convert position to ENU coordinates
        transformationPosition.setCoordinates(
            currentPosition.inhomX,
            currentPosition.inhomY,
            currentPosition.inhomZ
        )
        // (initial position is at zero, consequently we can directly rotate it)
        conversionRotation.rotate(transformationPosition, transformationPosition)

        poseTransformation.rotation.fromRotation(transformationRotation)
        poseTransformation.setTranslation(transformationPosition)
    }

    /**
     * Keeps initial attitude.
     *
     * @param attitude first obtained attitude.
     * @return true if this estimator was already initialized, false if it wasn't.
     */
    private fun initialize(attitude: Quaternion): Boolean {
        val result = initialized
        if (!initialized) {
            attitude.copyTo(initialAttitude)
            attitude.copyTo(previousAttitude)

            initialSpeed.copyTo(previousSpeed)

            // initial position is assumed to be at origin (zero).
            previousPosition.setCoordinates(0.0, 0.0, 0.0)

            initialized = true
        }
        return result
    }

    /**
     * Interface to notify when a new pose is available.
     */
    fun interface OnPoseAvailableListener {

        /**
         * Called when a new pose is available.
         *
         * @param estimator pose estimator that raised this event.
         * @param timestamp time in nanoseconds at which the measurement was made. Each measurement
         * wil be monotonically increasing using the same time base as
         * [android.os.SystemClock.elapsedRealtimeNanos].
         * @param pose 3D euclidean transformation containing attitude and translation respect to
         * the start timestamp of the estimator.
         */
        fun onPoseAvailable(
            estimator: RelativePoseEstimator,
            timestamp: Long,
            pose: EuclideanTransformation3D
        )
    }
}