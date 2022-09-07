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
import android.util.Log
import com.irurueta.android.navigation.inertial.QuaternionHelper
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.GravitySensorCollector
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.estimators.filter.AveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.LowPassAveragingFilter
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType
import kotlin.math.abs
import kotlin.math.min

/**
 * Estimates a leveled relative attitude, where only yaw Euler angle is relative to the start
 * instant of this estimator.
 * Roll and pitch Euler angles are leveled using accelerometer or gravity sensors.
 *
 * @property context Android context.
 * @property sensorDelay Delay of sensors between samples.
 * @property useAccelerometer true to use accelerometer sensor, false to use system gravity sensor
 * for leveling purposes.
 * @property accelerometerSensorType One of the supported accelerometer sensor types.
 * (Only used if [useAccelerometer] is true).
 * @property accelerometerAveragingFilter an averaging filter for accelerometer samples to obtain
 * sensed gravity component of specific force. (Only used if [useAccelerometer] is true).
 * @property gyroscopeSensorType One of the supported gyroscope sensor types.
 * @property estimateCoordinateTransformation true to estimate coordinate transformation, false
 * otherwise. If not needed, it can be disabled to improve performance and decrease cpu load.
 * @property estimateEulerAngles true to estimate euler angles, false otherwise. If not
 * needed, it can be disabled to improve performance and decrease cpu load.
 * @property attitudeAvailableListener listener to notify when a new attitude measurement is
 * available.
 */
class LeveledRelativeAttitudeEstimator private constructor(
    val context: Context,
    val sensorDelay: SensorDelay,
    val useAccelerometer: Boolean,
    val accelerometerSensorType: AccelerometerSensorCollector.SensorType,
    val accelerometerAveragingFilter: AveragingFilter,
    val gyroscopeSensorType: GyroscopeSensorCollector.SensorType,
    val estimateCoordinateTransformation: Boolean,
    val estimateEulerAngles: Boolean,
    var attitudeAvailableListener: OnAttitudeAvailableListener?,
) {

    /**
     * Constructor.
     *
     * @param context Android context.
     * @param location Device location.
     * @param sensorDelay Delay of sensors between samples.
     * @param useAccelerometer true to use accelerometer sensor, false to use system gravity sensor
     * for leveling purposes.
     * @param accelerometerSensorType One of the supported accelerometer sensor types.
     * (Only used if [useAccelerometer] is true).
     * @param accelerometerAveragingFilter an averaging filter for accelerometer samples to obtain
     * sensed gravity component of specific force. (Only used if [useAccelerometer] is true).
     * @param gyroscopeSensorType One of the supported gyroscope sensor types.
     * @param useAccurateLevelingEstimator true to use accurate leveling, false to use a normal one.
     * @param useAccurateRelativeGyroscopeAttitudeEstimator true to use accurate relative attitude,
     * false to use a normal one.
     * @param estimateCoordinateTransformation true to estimate coordinate transformation, false
     * otherwise. If not needed, it can be disabled to improve performance and decrease cpu load.
     * @param estimateEulerAngles true to estimate euler angles, false otherwise. If not
     * needed, it can be disabled to improve performance and decrease cpu load.
     * @param attitudeAvailableListener listener to notify when a new attitude measurement is
     * available.
     * @param accelerometerMeasurementListener listener to notify new accelerometer measurements.
     * (Only used if [useAccelerometer] is true).
     * @param gravityMeasurementListener listener to notify new gravity measurements.
     * (Only used if [useAccelerometer] is false).
     * @param gyroscopeMeasurementListener listener to notify new gyroscope measurements.
     * @param gravityEstimationListener listener to notify when a new gravity estimation is
     * available.
     */
    constructor(
        context: Context,
        location: Location? = null,
        sensorDelay: SensorDelay = SensorDelay.GAME,
        useAccelerometer: Boolean = false,
        accelerometerSensorType: AccelerometerSensorCollector.SensorType =
            AccelerometerSensorCollector.SensorType.ACCELEROMETER,
        accelerometerAveragingFilter: AveragingFilter = LowPassAveragingFilter(),
        gyroscopeSensorType: GyroscopeSensorCollector.SensorType =
            GyroscopeSensorCollector.SensorType.GYROSCOPE,
        useAccurateLevelingEstimator: Boolean = false,
        useAccurateRelativeGyroscopeAttitudeEstimator: Boolean = false,
        estimateCoordinateTransformation: Boolean = false,
        estimateEulerAngles: Boolean = true,
        attitudeAvailableListener: OnAttitudeAvailableListener? = null,
        accelerometerMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener? = null,
        gravityMeasurementListener: GravitySensorCollector.OnMeasurementListener? = null,
        gyroscopeMeasurementListener: GyroscopeSensorCollector.OnMeasurementListener? = null,
        gravityEstimationListener: GravityEstimator.OnEstimationListener? = null
    ) : this(
        context,
        sensorDelay,
        useAccelerometer,
        accelerometerSensorType,
        accelerometerAveragingFilter,
        gyroscopeSensorType,
        estimateCoordinateTransformation,
        estimateEulerAngles,
        attitudeAvailableListener
    ) {
        this.location = location
        this.useAccurateLevelingEstimator = useAccurateLevelingEstimator
        this.useAccurateRelativeGyroscopeAttitudeEstimator =
            useAccurateRelativeGyroscopeAttitudeEstimator
        this.accelerometerMeasurementListener = accelerometerMeasurementListener
        this.gravityMeasurementListener = gravityMeasurementListener
        this.gyroscopeMeasurementListener = gyroscopeMeasurementListener
        this.gravityEstimationListener = gravityEstimationListener
    }

    /**
     * Internal leveling estimator.
     */
    private lateinit var levelingEstimator: BaseLevelingEstimator<*, *>

    /**
     * Internal relative attitude estimator.
     */
    private lateinit var attitudeEstimator: BaseRelativeGyroscopeAttitudeEstimator<*, *>

    /**
     * Timestamp of last attitude estimation.
     */
    private var timestamp = 0L

    /**
     * Instance to be reused which contains attitude of internal leveling estimator.
     */
    private val levelingAttitude = Quaternion()

    /**
     * Instance to be reused which contains attitude of internal relative attitude estimator.
     */
    private val relativeAttitude = Quaternion()

    /**
     * Relative attitude of previous sample.
     */
    private var previousRelativeAttitude: Quaternion? = null

    /**
     * Inverse previous relative attitude to be reused internally for performance reasons.
     */
    private var inversePreviousRelativeAttitude = Quaternion()

    /**
     * Delta attitude of relative attitude estimator, which only uses gyroscope sensor.
     */
    private val deltaRelativeAttitude = Quaternion()

    /**
     * Instance to be reused which contains merged attitudes of both the internal leveling
     * estimator and the relative attitude estimator taking into account display orientation.
     */
    private val fusedAttitude = Quaternion()

    /**
     * Array to be reused containing euler angles of leveling attitude.
     */
    private val eulerAngles = DoubleArray(Quaternion.N_ANGLES)

    /**
     * Instance to be reused containing coordinate transformation in NED coordinates.
     */
    private val coordinateTransformation =
        CoordinateTransformation(FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME)

    /**
     * Indicates whether a relative attitude has been received.
     */
    private var hasRelativeAttitude = false

    /**
     * Indicates whether a delta relative attitude has been computed.
     */
    private var hasDeltaRelativeAttitude = false

    /**
     * Indicates whether fused attitude must be reset to leveling one.
     */
    private val resetToLeveling
        get() = panicCounter >= panicCounterThreshold

    /**
     * Gets or sets device location
     *
     * @throws IllegalStateException if estimator is running and a null value is set.
     */
    var location: Location? = null
        @Throws(IllegalStateException::class)
        set(value) {
            check(value != null || !running)
            field = value
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
            if (value) {
                checkNotNull(location)
            }

            field = value
            buildLevelingEstimator()
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
            buildAttitudeEstimator()
        }

    /**
     * Indicates whether fusion between leveling and relative attitudes occurs based
     * on changing interpolation value that depends on actual relative attitude rotation
     * velocity.
     */
    var useIndirectInterpolation = true

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
    var interpolationValue = DEFAULT_INTERPOLATION_VALUE
        set(value) {
            require(value in 0.0..1.0)
            field = value
        }

    /**
     * Factor to take into account when interpolation value is computed and
     * [useIndirectInterpolation] is enabled to determine actual interpolation value based
     * on current relative attitude rotation velocity.
     *
     * @throws IllegalArgumentException if value is zero or negative.
     */
    var indirectInterpolationWeight = DEFAULT_INDIRECT_INTERPOLATION_WEIGHT
        set(value) {
            require(value > 0.0)
            field = value
        }

    /**
     * Gets average time interval between gyroscope samples expressed in seconds.
     */
    val gyroscopeAverageTimeInterval
        get() = attitudeEstimator.averageTimeInterval

    /**
     * Listener to notify new accelerometer measurements.
     * (Only used if [useAccelerometer] is true).
     */
    var accelerometerMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener? = null
        set(value) {
            field = value
            levelingEstimator.accelerometerMeasurementListener = value
        }

    /**
     * listener to notify new gravity measurements.
     * (Only used if [useAccelerometer] is false).
     */
    var gravityMeasurementListener: GravitySensorCollector.OnMeasurementListener? = null
        set(value) {
            field = value
            levelingEstimator.gravityMeasurementListener = value
        }

    /**
     * Listener to notify new gyroscope measurements.
     */
    var gyroscopeMeasurementListener: GyroscopeSensorCollector.OnMeasurementListener? = null
        set(value) {
            field = value
            attitudeEstimator.gyroscopeMeasurementListener = value
        }

    /**
     * Listener to notify when a new gravity estimation is
     * available.
     */
    var gravityEstimationListener: GravityEstimator.OnEstimationListener? = null
        set(value) {
            field = value
            levelingEstimator.gravityEstimationListener = value
        }

    /**
     * Indicates whether this estimator is running or not.
     */
    var running: Boolean = false
        private set

    /**
     * Threshold to determine that current leveling appears to be an outlier respect
     * to estimated fused attitude.
     * When leveling and fused attitudes diverge, fusion is not performed, and instead
     * only gyroscope relative attitude is used for fusion estimation.
     *
     * @throws IllegalStateException if estimator is already running.
     * @throws IllegalArgumentException if value is not between 0.0 and 1.0 (both included).
     */
    var outlierThreshold = DEFAULT_OUTLIER_THRESHOLD
        set(value) {
            check(!running)
            require(value in 0.0..1.0)

            field = value
        }

    /**
     * Threshold to determine that leveling has largely diverged and if situation is not
     * reverted soon, attitude will be reset to leveling
     *
     * @throws IllegalStateException if estimator is already running.
     * @throws IllegalArgumentException if value is not between 0.0 and 1.0 (both included).
     */
    var outlierPanicThreshold = DEFAULT_OUTLIER_PANIC_THRESHOLD
        set(value) {
            check(!running)
            require(value in 0.0..1.0)

            field = value
        }

    /**
     * Threshold to determine when fused attitude has largely diverged for a given
     * number of samples and must be reset.
     *
     * @throws IllegalStateException if estimator is already running.
     */
    var panicCounterThreshold = DEFAULT_PANIC_COUNTER_THRESHOLD
        set(value) {
            check(!running)
            require(value > 0)

            field = value
        }

    /**
     * Counter indicating the number of fused attitudes that have largely diverged
     * from leveling attitudes.
     */
    private var panicCounter = panicCounterThreshold

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
        running = levelingEstimator.start() && attitudeEstimator.start()
        if (!running) {
            stop()
        }
        return running
    }

    /**
     * Stops this estimator.
     */
    fun stop() {
        levelingEstimator.stop()
        attitudeEstimator.stop()
        running = false
    }

    /**
     * Resets internal parameters.
     */
    private fun reset() {
        hasRelativeAttitude = false
        hasDeltaRelativeAttitude = false
        panicCounter = panicCounterThreshold
    }

    /**
     * Builds the internal leveling estimator.
     */
    private fun buildLevelingEstimator() {
        levelingEstimator = if (useAccurateLevelingEstimator) {

            val location = this.location
            checkNotNull(location)
            AccurateLevelingEstimator(
                context,
                location,
                sensorDelay,
                useAccelerometer,
                accelerometerSensorType,
                accelerometerAveragingFilter,
                estimateCoordinateTransformation = false,
                estimateEulerAngles = false,
                levelingAvailableListener = { _, attitude, _, _, _, _ ->
                    processLeveling(attitude)
                },
                gravityEstimationListener = gravityEstimationListener,
                accelerometerMeasurementListener = accelerometerMeasurementListener,
                gravityMeasurementListener = gravityMeasurementListener
            )
        } else {
            LevelingEstimator(
                context,
                sensorDelay,
                useAccelerometer,
                accelerometerSensorType,
                accelerometerAveragingFilter,
                estimateCoordinateTransformation = false,
                estimateEulerAngles = false,
                levelingAvailableListener = { _, attitude, _, _, _, _ ->
                    processLeveling(attitude)
                },
                gravityEstimationListener = gravityEstimationListener,
                accelerometerMeasurementListener = accelerometerMeasurementListener,
                gravityMeasurementListener = gravityMeasurementListener
            )
        }
    }

    /**
     * Builds the internal attitude estimator.
     */
    private fun buildAttitudeEstimator() {
        attitudeEstimator = if (useAccurateRelativeGyroscopeAttitudeEstimator) {
            AccurateRelativeGyroscopeAttitudeEstimator(
                context,
                gyroscopeSensorType,
                sensorDelay,
                estimateCoordinateTransformation = false,
                estimateEulerAngles = false,
                attitudeAvailableListener = { _, attitude, timestamp, _, _, _, _ ->
                    processRelativeAttitude(attitude, timestamp)
                },
                gyroscopeMeasurementListener
            )
        } else {
            RelativeGyroscopeAttitudeEstimator(
                context,
                gyroscopeSensorType,
                sensorDelay,
                estimateCoordinateTransformation = false,
                estimateDisplayEulerAngles = false,
                attitudeAvailableListener = { _, attitude, timestamp, _, _, _, _ ->
                    processRelativeAttitude(attitude, timestamp)
                },
                gyroscopeMeasurementListener
            )
        }
    }

    /**
     * Computes variation of relative attitude between samples.
     * Relative attitude only depends on gyroscope and is usually smoother and has
     * less interferences than leveling attitude.
     *
     * @return true if estimator already has a fully initialized delta relative attitude, false
     * otherwise.
     */
    private fun computeDeltaRelativeAttitude(): Boolean {
        val previousRelativeAttitude = this.previousRelativeAttitude
        return if (previousRelativeAttitude != null) {
            // compute delta attitude between relative samples
            // Q1 = deltaQ1 * Q0
            // deltaQ1 = Q1 * invQ0
            previousRelativeAttitude.inverse(inversePreviousRelativeAttitude)
            Quaternion.product(
                relativeAttitude,
                inversePreviousRelativeAttitude,
                deltaRelativeAttitude
            )
            true
        } else {
            this.previousRelativeAttitude = Quaternion()
            relativeAttitude.copyTo(this.previousRelativeAttitude)
            false
        }
    }

    /**
     * Processes last received leveling attitude to estimate a fused attitude containing
     * leveled relative attitude, where only yaw is kept relative to the start of this estimator.
     */
    private fun processLeveling(attitude: Quaternion) {
        if (!hasRelativeAttitude) {
            return
        }

        attitude.copyTo(levelingAttitude)

        if (hasDeltaRelativeAttitude) {
            // set yaw angle into leveled attitude
            levelingAttitude.toEulerAngles(eulerAngles)
            val levelingRoll = eulerAngles[0]
            val levelingPitch = eulerAngles[1]

            relativeAttitude.toEulerAngles(eulerAngles)
            val yaw = eulerAngles[2]
            levelingAttitude.setFromEulerAngles(levelingRoll, levelingPitch, yaw)

            if (resetToLeveling) {
                levelingAttitude.copyTo(fusedAttitude)
                panicCounter = 0
                Log.d(
                    LeveledRelativeAttitudeEstimator::class.simpleName,
                    "Attitude reset to leveling"
                )
                return
            }

            // change attitude by the delta obtained from relative attitude (gyroscope)
            Quaternion.product(deltaRelativeAttitude, fusedAttitude, fusedAttitude)

            val absDot = abs(QuaternionHelper.dotProduct(fusedAttitude, levelingAttitude))

            // check if fused attitude and leveling attitude have diverged
            if (absDot < outlierThreshold) {
                Log.i(
                    LeveledRelativeAttitudeEstimator::class.simpleName,
                    "Threshold exceeded: $absDot"
                )
                // increase panic counter
                if (absDot < outlierPanicThreshold) {
                    panicCounter++
                    Log.i(
                        LeveledRelativeAttitudeEstimator::class.simpleName,
                        "Panic counter increased: $panicCounter"
                    )
                }

                // directly use attitude which has combined only delta gyroscope data
            } else {
                // both are nearly the same. Perform normal fusion
                Quaternion.slerp(
                    fusedAttitude,
                    levelingAttitude,
                    getSlerpFactor(),
                    fusedAttitude
                )
                panicCounter = 0
            }

            relativeAttitude.copyTo(previousRelativeAttitude)
            hasRelativeAttitude = false

            val c: CoordinateTransformation? =
                if (estimateCoordinateTransformation) {
                    coordinateTransformation.fromRotation(fusedAttitude)
                    coordinateTransformation
                } else {
                    null
                }

            val displayRoll: Double?
            val displayPitch: Double?
            val displayYaw: Double?
            if (estimateEulerAngles) {
                fusedAttitude.toEulerAngles(eulerAngles)
                displayRoll = eulerAngles[0]
                displayPitch = eulerAngles[1]
                displayYaw = eulerAngles[2]
            } else {
                displayRoll = null
                displayPitch = null
                displayYaw = null
            }

            attitudeAvailableListener?.onAttitudeAvailable(
                this,
                fusedAttitude,
                timestamp,
                displayRoll,
                displayPitch,
                displayYaw,
                c
            )
        }
    }

    /**
     * Processes last received internal relative attitude to estimate attitude variation
     * between samples.
     *
     * @param attitude attitude expressed in NED coordinates.
     * @param timestamp time in nanoseconds at which the measurement was made. Each measurement
     * wil be monotonically increasing using the same time base as
     * [android.os.SystemClock.elapsedRealtimeNanos].
     */
    private fun processRelativeAttitude(attitude: Quaternion, timestamp: Long) {
        attitude.copyTo(relativeAttitude)
        hasRelativeAttitude = true
        hasDeltaRelativeAttitude = computeDeltaRelativeAttitude()

        this.timestamp = timestamp
    }

    /**
     * Computes factor to be used to compute fusion between relative and leveling attitudes.
     */
    private fun getSlerpFactor(): Double {
        return if (useIndirectInterpolation) {
            val rotationVelocity =
                deltaRelativeAttitude.rotationAngle / gyroscopeAverageTimeInterval
            min(interpolationValue + indirectInterpolationWeight * abs(rotationVelocity), 1.0)
        } else {
            interpolationValue
        }
    }

    companion object {
        /**
         * Default value to be used to combine both leveling and relative attitudes.
         */
        const val DEFAULT_INTERPOLATION_VALUE = 0.005

        /**
         * Default value to obtain a fusion interpolation value based on current relative attitude
         * rotation velocity.
         */
        const val DEFAULT_INDIRECT_INTERPOLATION_WEIGHT = 0.01

        /**
         * Default threshold that indicates that there is an outlier in leveling.
         * If the dot-product between the attitude using gyroscope data and leveling
         * fails below this threshold, the system falls back to the gyroscope (relative attitude)
         * and ignores the leveling attitude
         */
        const val DEFAULT_OUTLIER_THRESHOLD = 0.85

        /**
         * Default threshold that indicates a massive discrepancy between the gyroscope (relative
         * attitude) and leveling attitude
         */
        const val DEFAULT_OUTLIER_PANIC_THRESHOLD = 0.65

        /**
         * Default threshold to determine that the attitude has completely diverged from leveling
         * value and must be completely reset.
         */
        const val DEFAULT_PANIC_COUNTER_THRESHOLD = 60
    }

    /**
     * Interface to notify when a new attitude measurement is available.
     */
    fun interface OnAttitudeAvailableListener {

        /**
         * Called when a new attitude measurement is available.
         *
         * @param estimator attitude estimator that raised this event.
         * @param attitude attitude expressed in NED coordinates.
         * @param timestamp time in nanoseconds at which the measurement was made. Each measurement
         * wil be monotonically increasing using the same time base as
         * @param roll roll angle expressed in radians respect to NED coordinate system. Only
         * available if [estimateEulerAngles] is true.
         * @param pitch pitch angle expressed in radians respect to NED coordinate system. Only
         * available if [estimateEulerAngles] is true.
         * @param yaw yaw angle expressed in radians respect to NED coordinate system. Only
         * available if [estimateEulerAngles] is true.
         * @param coordinateTransformation coordinate transformation containing measured leveling
         * attitude. Only available if [estimateCoordinateTransformation] is true.
         */
        fun onAttitudeAvailable(
            estimator: LeveledRelativeAttitudeEstimator,
            attitude: Quaternion,
            timestamp: Long,
            roll: Double?,
            pitch: Double?,
            yaw: Double?,
            coordinateTransformation: CoordinateTransformation?
        )
    }
}