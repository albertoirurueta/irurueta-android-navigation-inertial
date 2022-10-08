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
package com.irurueta.android.navigation.inertial.estimators.attitude

import android.content.Context
import android.location.Location
import android.util.Log
import com.irurueta.android.navigation.inertial.QuaternionHelper
import com.irurueta.android.navigation.inertial.collectors.*
import com.irurueta.android.navigation.inertial.estimators.filter.AveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.LowPassAveragingFilter
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel
import java.util.*
import kotlin.math.abs
import kotlin.math.min

/**
 * Estimates a fused absolute attitude, where accelerometer + magnetometer measurements are fused
 * with relative attitude obtained from gyroscope ones.
 * This is equivalent to [FusedGeomagneticAttitudeEstimator] with an additional filtering layer,
 * which requires additional cpu usage.
 *
 * @property context Android context.
 * @property sensorDelay Delay of sensors between samples.
 * @property useAccelerometer true to use accelerometer sensor, false to use system gravity sensor
 * for leveling purposes.
 * @property accelerometerSensorType One of the supported accelerometer sensor types.
 * (Only used if [useAccelerometer] is true).
 * @property magnetometerSensorType One of the supported magnetometer sensor types.
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
class FusedGeomagneticAttitudeEstimator2 private constructor(
    override val context: Context,
    override val sensorDelay: SensorDelay,
    override val useAccelerometer: Boolean,
    override val accelerometerSensorType: AccelerometerSensorCollector.SensorType,
    override val magnetometerSensorType: MagnetometerSensorCollector.SensorType,
    override val accelerometerAveragingFilter: AveragingFilter,
    override val gyroscopeSensorType: GyroscopeSensorCollector.SensorType,
    override val estimateCoordinateTransformation: Boolean,
    override val estimateEulerAngles: Boolean,
    override var attitudeAvailableListener: OnAttitudeAvailableListener?
) : AbsoluteAttitudeEstimator<FusedGeomagneticAttitudeEstimator2, FusedGeomagneticAttitudeEstimator2.OnAttitudeAvailableListener> {

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
     * @param magnetometerSensorType One of the supported magnetometer sensor types.
     * @param accelerometerAveragingFilter an averaging filter for accelerometer samples to obtain
     * sensed gravity component of specific force. (Only used if [useAccelerometer] is true).
     * @param gyroscopeSensorType One of the supported gyroscope sensor types.
     * @param worldMagneticModel Earth's magnetic model. Null to use default model
     * when [useWorldMagneticModel] is true. If [useWorldMagneticModel] is false, this is ignored.
     * @param timestamp Timestamp when World Magnetic Model will be evaluated to obtain current.
     * Only taken into account if [useWorldMagneticModel] is tue.
     * @param useWorldMagneticModel true so that world magnetic model is taken into account to
     * adjust attitude yaw angle by current magnetic declination based on current World Magnetic
     * Model, location and timestamp, false to ignore declination.
     * @param useAccurateLevelingEstimator true to use accurate leveling, false to use a normal one.
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
     * @param magnetometerMeasurementListener listener to notify new magnetometer measurements.
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
        magnetometerSensorType: MagnetometerSensorCollector.SensorType =
            MagnetometerSensorCollector.SensorType.MAGNETOMETER,
        accelerometerAveragingFilter: AveragingFilter = LowPassAveragingFilter(),
        gyroscopeSensorType: GyroscopeSensorCollector.SensorType =
            GyroscopeSensorCollector.SensorType.GYROSCOPE,
        worldMagneticModel: WorldMagneticModel? = null,
        timestamp: Date = Date(),
        useWorldMagneticModel: Boolean = false,
        useAccurateLevelingEstimator: Boolean = false,
        useAccurateRelativeGyroscopeAttitudeEstimator: Boolean = false,
        estimateCoordinateTransformation: Boolean = false,
        estimateEulerAngles: Boolean = true,
        attitudeAvailableListener: OnAttitudeAvailableListener? = null,
        accelerometerMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener? = null,
        gravityMeasurementListener: GravitySensorCollector.OnMeasurementListener? = null,
        gyroscopeMeasurementListener: GyroscopeSensorCollector.OnMeasurementListener? = null,
        magnetometerMeasurementListener: MagnetometerSensorCollector.OnMeasurementListener? = null,
        gravityEstimationListener: GravityEstimator.OnEstimationListener? = null
    ) : this(
        context,
        sensorDelay,
        useAccelerometer,
        accelerometerSensorType,
        magnetometerSensorType,
        accelerometerAveragingFilter,
        gyroscopeSensorType,
        estimateCoordinateTransformation,
        estimateEulerAngles,
        attitudeAvailableListener
    ) {
        buildGeomagneticAttitudeEstimator()
        this.location = location
        this.worldMagneticModel = worldMagneticModel
        this.timestamp = timestamp
        this.useWorldMagneticModel = useWorldMagneticModel
        this.useAccurateLevelingEstimator = useAccurateLevelingEstimator
        this.useAccurateRelativeGyroscopeAttitudeEstimator =
            useAccurateRelativeGyroscopeAttitudeEstimator
        this.accelerometerMeasurementListener = accelerometerMeasurementListener
        this.gravityMeasurementListener = gravityMeasurementListener
        this.gyroscopeMeasurementListener = gyroscopeMeasurementListener
        this.magnetometerMeasurementListener = magnetometerMeasurementListener
        this.gravityEstimationListener = gravityEstimationListener
    }

    /**
     * Internal geomagnetic absolute attitude estimator.
     */
    private lateinit var geomagneticAttitudeEstimator: GeomagneticAttitudeEstimator

    /**
     * Internal relative attitude estimator.
     */
    private lateinit var relativeAttitudeEstimator: LeveledRelativeAttitudeEstimator

    /**
     * Timestamp of last attitude estimation.
     */
    private var relativeAttitudeTimestamp = 0L

    /**
     * Instance to be reused which contains geomagnetic absolute attitude.
     */
    private val geomagneticAttitude = Quaternion()

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
    private val internalFusedAttitude = Quaternion()

    /**
     * Instance being reused to externally notify attitudes so that it does not have additional
     * effects if it is modified.
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
     * Indicates whether fused attitude must be reset to geomagnetic one.
     */
    private val resetToGeomagnetic
        get() = panicCounter >= panicCounterThreshold

    /**
     * Gets or sets device location
     *
     * @throws IllegalStateException if estimator is running and a null value is set.
     */
    override var location: Location? = null
        @Throws(IllegalStateException::class)
        set(value) {
            check(value != null || !running)
            field = value
            geomagneticAttitudeEstimator.location = value
        }

    /**
     * Earth's magnetic model. If null, the default model is used if [useWorldMagneticModel] is
     * true. If [useWorldMagneticModel] is false, this is ignored.
     *
     * @throws IllegalStateException if estimator is running.
     */
    override var worldMagneticModel: WorldMagneticModel? = null
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
            geomagneticAttitudeEstimator.worldMagneticModel = value
        }

    /**
     * Timestamp when World Magnetic Model will be evaluated to obtain current.
     * Only taken into account if [useWorldMagneticModel] is tue.
     */
    override var timestamp: Date = Date()
        set(value) {
            field = value
            geomagneticAttitudeEstimator.timestamp = value
        }

    /**
     * Indicates whether world magnetic model is taken into account to adjust attitude yaw angle by
     * current magnetic declination based on current Wolrd MAgnetic Model, location and timestamp.
     */
    override var useWorldMagneticModel: Boolean = false
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            field = value
            geomagneticAttitudeEstimator.useWorldMagneticModel = value
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
                checkNotNull(location)
            }

            field = value
            geomagneticAttitudeEstimator.useAccurateLevelingEstimator = value
            buildRelativeAttitudeEstimator()
        }

    /**
     * Indicates whether accurate non-leveled relative attitude estimator must be used or not.
     *
     * @throws IllegalStateException if estimator is running.
     */
    override var useAccurateRelativeGyroscopeAttitudeEstimator: Boolean = false
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)

            field = value
            buildRelativeAttitudeEstimator()
        }

    /**
     * Indicates whether fusion between leveling and relative attitudes occurs based
     * on changing interpolation value that depends on actual relative attitude rotation
     * velocity.
     */
    override var useIndirectInterpolation = true

    /**
     * Interpolation value to be used to combine both geomagnetic and relative attitudes.
     * Must be between 0.0 and 1.0 (both included).
     * The closer to 0.0 this value is, the more resemblance the result will have to a pure
     * geomagnetic (which feels more jerky when using accelerometer). On the contrary, the closer
     * to 1.0 this value is, the more resemblance the result will have to a pure non-leveled
     * relative attitude (which feels softer but might have arbitrary roll pitch and yaw Euler
     * angles).
     *
     * @throws IllegalArgumentException if value is not between 0.0 and 1.0 (both included).
     */
    override var interpolationValue = DEFAULT_INTERPOLATION_VALUE
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
    override var indirectInterpolationWeight = DEFAULT_INDIRECT_INTERPOLATION_WEIGHT
        set(value) {
            require(value > 0.0)
            field = value
        }

    /**
     * Gets average time interval between gyroscope samples expressed in seconds.
     */
    override val gyroscopeAverageTimeInterval
        get() = relativeAttitudeEstimator.gyroscopeAverageTimeInterval

    /**
     * Listener to notify new accelerometer measurements.
     * (Only used if [useAccelerometer] is true).
     */
    override var accelerometerMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener? = null
        set(value) {
            field = value
            relativeAttitudeEstimator.accelerometerMeasurementListener = value
        }

    /**
     * listener to notify new gravity measurements.
     * (Only used if [useAccelerometer] is false).
     */
    override var gravityMeasurementListener: GravitySensorCollector.OnMeasurementListener? = null
        set(value) {
            field = value
            relativeAttitudeEstimator.gravityMeasurementListener = value
        }

    /**
     * Listener to notify new gyroscope measurements.
     */
    override var gyroscopeMeasurementListener: GyroscopeSensorCollector.OnMeasurementListener? =
        null
        set(value) {
            field = value
            relativeAttitudeEstimator.gyroscopeMeasurementListener = value
        }

    /**
     * Listener to notify new magnetometer measurements.
     */
    override var magnetometerMeasurementListener: MagnetometerSensorCollector.OnMeasurementListener? =
        null
        set(value) {
            field = value
            geomagneticAttitudeEstimator.magnetometerMeasurementListener = value
        }

    /**
     * Listener to notify when a new gravity estimation is
     * available.
     */
    override var gravityEstimationListener: GravityEstimator.OnEstimationListener? = null
        set(value) {
            field = value
            relativeAttitudeEstimator.gravityEstimationListener = value
        }

    /**
     * Indicates whether this estimator is running or not.
     */
    override var running: Boolean = false
        private set

    /**
     * Threshold to determine that current geomagnetic attitude appears to be an outlier respect
     * to estimated fused attitude.
     * When geomagnetic attitude and fused attitudes diverge, fusion is not performed, and instead
     * only gyroscope relative attitude is used for fusion estimation.
     *
     * @throws IllegalStateException if estimator is already running.
     * @throws IllegalArgumentException if value is not between 0.0 and 1.0 (both included).
     */
    override var outlierThreshold = DEFAULT_OUTLIER_THRESHOLD
        set(value) {
            check(!running)
            require(value in 0.0..1.0)

            field = value
        }

    /**
     * Threshold to determine that geomagnetic attitude has largely diverged and if situation is not
     * reverted soon, attitude will be reset to geomagnetic one.
     *
     * @throws IllegalStateException if estimator is already running.
     * @throws IllegalArgumentException if value is not between 0.0 and 1.0 (both included).
     */
    override var outlierPanicThreshold = DEFAULT_OUTLIER_PANIC_THRESHOLD
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
    override var panicCounterThreshold = DEFAULT_PANIC_COUNTER_THRESHOLD
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
    override fun start(): Boolean {
        check(!running)

        reset()
        running = geomagneticAttitudeEstimator.start() && relativeAttitudeEstimator.start()
        if (!running) {
            stop()
        }
        return running
    }

    /**
     * Stops this estimator.
     */
    override fun stop() {
        geomagneticAttitudeEstimator.stop()
        relativeAttitudeEstimator.stop()
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
     * Builds the internal relative attitude estimator.
     */
    private fun buildRelativeAttitudeEstimator() {
        relativeAttitudeEstimator = LeveledRelativeAttitudeEstimator(
            context,
            location,
            sensorDelay,
            useAccelerometer,
            accelerometerSensorType,
            accelerometerAveragingFilter,
            gyroscopeSensorType,
            useAccurateLevelingEstimator,
            useAccurateRelativeGyroscopeAttitudeEstimator,
            estimateCoordinateTransformation = false,
            estimateEulerAngles = false,
            attitudeAvailableListener = { _, attitude, timestamp, _, _, _, _ ->
                processRelativeAttitude(attitude, timestamp)
            },
            accelerometerMeasurementListener = accelerometerMeasurementListener,
            gravityMeasurementListener = gravityMeasurementListener,
            gyroscopeMeasurementListener = gyroscopeMeasurementListener,
            gravityEstimationListener = gravityEstimationListener
        )
    }

    /**
     * Builds internal geomagnetic attitude estimator.
     */
    private fun buildGeomagneticAttitudeEstimator() {
        geomagneticAttitudeEstimator = GeomagneticAttitudeEstimator(
            context,
            location,
            sensorDelay,
            useAccelerometer,
            accelerometerSensorType,
            magnetometerSensorType,
            accelerometerAveragingFilter,
            worldMagneticModel,
            timestamp,
            useWorldMagneticModel,
            useAccurateLevelingEstimator,
            estimateCoordinateTransformation = false,
            estimateEulerAngles = false,
            attitudeAvailableListener = { _, attitude, _, _, _, _, _ ->
                processGeomagneticAttitude(attitude)
            },
            magnetometerMeasurementListener = magnetometerMeasurementListener
        )
    }

    /**
     * Computes variation of relative attitude between samples.
     * Relative attitude only depends on gyroscope and is usually smoother and has
     * less interferences than geomagnetic attitude.
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
     * Processes last received internal relative attitude to estimate attitude variation
     * between samples.
     *
     * @param attitude estimated absolute attitude respect NED coordinate system.
     * @param timestamp time in nanoseconds at which the measurement was made. Each measurement
     * wil be monotonically increasing using the same time base as
     * [android.os.SystemClock.elapsedRealtimeNanos].
     */
    private fun processRelativeAttitude(attitude: Quaternion, timestamp: Long) {
        attitude.copyTo(relativeAttitude)
        hasRelativeAttitude = true
        hasDeltaRelativeAttitude = computeDeltaRelativeAttitude()

        this.relativeAttitudeTimestamp = timestamp
    }

    /**
     * Processes last received geomagnetic attitude to estimate a fused attitude containing
     * absolute attitude.
     *
     * @param attitude estimated absolute attitude respect NED coordinate system.
     */
    private fun processGeomagneticAttitude(attitude: Quaternion) {
        if (!hasRelativeAttitude) {
            return
        }

        attitude.copyTo(geomagneticAttitude)

        if (hasDeltaRelativeAttitude) {
            if (resetToGeomagnetic) {
                geomagneticAttitude.copyTo(internalFusedAttitude)
                panicCounter = 0
                Log.d(
                    FusedGeomagneticAttitudeEstimator::class.simpleName,
                    "Attitude reset to geomagnetic one"
                )
                return
            }

            // change attitude by the delta obtained from relative attitude (gyroscope)
            Quaternion.product(deltaRelativeAttitude, internalFusedAttitude, internalFusedAttitude)

            val absDot = abs(QuaternionHelper.dotProduct(internalFusedAttitude, geomagneticAttitude))

            // check if fused attitude and leveling attitude have diverged
            if (absDot < outlierThreshold) {
                Log.i(
                    FusedGeomagneticAttitudeEstimator::class.simpleName,
                    "Threshold exceeded: $absDot"
                )
                // increase panic counter
                if (absDot < outlierPanicThreshold) {
                    panicCounter++
                    Log.i(
                        FusedGeomagneticAttitudeEstimator::class.simpleName,
                        "Panic counter increased: $panicCounter"
                    )
                }

                // directly use attitude which has combined only delta gyroscope data
            } else {
                // both are nearly the same. Perform normal fusion
                Quaternion.slerp(
                    internalFusedAttitude,
                    geomagneticAttitude,
                    getSlerpFactor(),
                    internalFusedAttitude
                )
                panicCounter = 0
            }

            relativeAttitude.copyTo(previousRelativeAttitude)

            val c: CoordinateTransformation? =
                if (estimateCoordinateTransformation) {
                    coordinateTransformation.fromRotation(internalFusedAttitude)
                    coordinateTransformation
                } else {
                    null
                }

            val displayRoll: Double?
            val displayPitch: Double?
            val displayYaw: Double?
            if (estimateEulerAngles) {
                internalFusedAttitude.toEulerAngles(eulerAngles)
                displayRoll = eulerAngles[0]
                displayPitch = eulerAngles[1]
                displayYaw = eulerAngles[2]
            } else {
                displayRoll = null
                displayPitch = null
                displayYaw = null
            }

            internalFusedAttitude.copyTo(fusedAttitude)
            attitudeAvailableListener?.onAttitudeAvailable(
                this@FusedGeomagneticAttitudeEstimator2,
                fusedAttitude,
                relativeAttitudeTimestamp,
                displayRoll,
                displayPitch,
                displayYaw,
                c
            )
        }
    }

    /**
     * Computes factor to be used to compute fusion between relative and geomagnetic attitudes.
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
    fun interface OnAttitudeAvailableListener :
        AbsoluteAttitudeEstimator.OnAttitudeAvailableListener<FusedGeomagneticAttitudeEstimator2>
}