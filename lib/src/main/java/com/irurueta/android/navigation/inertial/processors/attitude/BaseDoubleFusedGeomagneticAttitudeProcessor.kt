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

import android.location.Location
import android.util.Log
import com.irurueta.android.navigation.inertial.QuaternionHelper
import com.irurueta.android.navigation.inertial.collectors.*
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel
import java.util.*
import kotlin.math.abs
import kotlin.math.min

/**
 * Base class to estimate absolute attitude by fusing absolute leveled geomagnetic attitude and
 * leveled relative attitude.
 *
 * @property processorListener listener to notify new fused absolute attitude.
 */
abstract class BaseDoubleFusedGeomagneticAttitudeProcessor<M : SensorMeasurement<M>, S : SyncedSensorMeasurement>(
    var processorListener: OnProcessedListener<M, S>?
) {

    /**
     * Internal processor to estimate leveled absolute attitude.
     */
    protected abstract val geomagneticProcessor: BaseGeomagneticAttitudeProcessor<M, *>

    /**
     * Internal processor to estimate leveled relative attitude.
     */
    protected abstract val relativeGyroscopeProcessor: BaseLeveledRelativeAttitudeProcessor<M, *>

    /**
     * Instance to be reused which contains geomagnetic absolute attitude.
     */
    private val geomagneticAttitude = Quaternion()

    /**
     * Instance to be reused which contains attitude of internal leveled relative attitude
     * estimator.
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
     * Indicates whether fused attitude must be reset to geomagnetic one.
     */
    private val resetToGeomagnetic
        get() = panicCounter >= panicCounterThreshold

    /**
     * Indicates whether a delta relative attitude has been computed.
     */
    private var hasDeltaRelativeAttitude = false

    /**
     * Instance being reused to externally notify attitudes so that it does not have additional
     * effects if it is modified.
     */
    val fusedAttitude = Quaternion()

    /**
     * X-coordinates of last sensed gravity component of specific force expressed in NED coordinates
     * and in meters per squared second (m/s^2).
     */
    val gx: Double
        get() = geomagneticProcessor.gx

    /**
     * Y-coordinate of last sensed gravity component of specific force expressed in NED coordinates
     * and in meters per squared second (m/s^2).
     */
    val gy: Double
        get() = geomagneticProcessor.gy

    /**
     * Z-coordinate of last sensed gravity component of specific force expressed in NED coordinates
     * and in meters per squared second (m/s^2).
     */
    val gz: Double
        get() = geomagneticProcessor.gz

    /**
     * Gets a new triad containing gravity component of specific force expressed in NED coordinates
     * and in meters per squared second (m/s^2).
     */
    val gravity: AccelerationTriad
        get() = geomagneticProcessor.gravity

    /**
     * Updates provided triad to contain gravity component of specific force expressed in NED
     * coordinates and in meters per squared second (m/s^2).
     */
    fun getGravity(result: AccelerationTriad) {
        geomagneticProcessor.getGravity(result)
    }

    /**
     * Gets or sets device location
     */
    var location: Location?
        get() = geomagneticProcessor.location
        set(value) {
            geomagneticProcessor.location = value
            relativeGyroscopeProcessor.location = value
        }

    /**
     * Indicates whether gravity norm must be adjusted to either Earth
     * standard norm, or norm at provided location. If no location is provided, this should only be
     * enabled when device is close to sea level.
     */
    var adjustGravityNorm: Boolean
        get() = geomagneticProcessor.adjustGravityNorm
        set(value) {
            geomagneticProcessor.adjustGravityNorm = value
            relativeGyroscopeProcessor.adjustGravityNorm = value
        }

    /**
     * Timestamp being used when World Magnetic Model is evaluated to obtain current magnetic
     * declination. This is only taken into account if [useWorldMagneticModel] is true.
     * If not defined, current date is assumed.
     */
    var currentDate: Date?
        get() = geomagneticProcessor.currentDate
        set(value) {
            geomagneticProcessor.currentDate = value
        }

    /**
     * Indicates whether accurate leveling must be used or not.
     *
     * @throws IllegalStateException if set to true and no location is available.
     */
    var useAccurateLevelingProcessor: Boolean
        get() = geomagneticProcessor.useAccurateLevelingProcessor
        @Throws(IllegalStateException::class)
        set(value) {
            geomagneticProcessor.useAccurateLevelingProcessor = value
        }

    /**
     * Earth's magnetic model. If null, the default model is used if [useWorldMagneticModel] is
     * true. If [useWorldMagneticModel] is false, this is ignored.
     */
    var worldMagneticModel: WorldMagneticModel?
        get() = geomagneticProcessor.worldMagneticModel
        set(value) {
            geomagneticProcessor.worldMagneticModel = value
        }

    /**
     * Indicates whether world magnetic model is taken into account to adjust attitude yaw angle by
     * current magnetic declination based on current World Magnetic Model, location and timestamp.
     */
    var useWorldMagneticModel: Boolean
        get() = geomagneticProcessor.useWorldMagneticModel
        set(value) {
            geomagneticProcessor.useWorldMagneticModel = value
        }

    /**
     * Indicates whether accurate non-leveled relative attitude processor must be used or not.
     */
    var useAccurateRelativeGyroscopeAttitudeProcessor: Boolean
        get() = relativeGyroscopeProcessor.useAccurateRelativeGyroscopeAttitudeProcessor
        set(value) {
            relativeGyroscopeProcessor.useAccurateRelativeGyroscopeAttitudeProcessor = value
        }

    /**
     * Indicates whether fusion between leveling and relative attitudes occurs based
     * on changing interpolation value that depends on actual relative attitude rotation
     * velocity.
     */
    var useIndirectInterpolation = true

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
     * Time interval expressed in seconds between consecutive gyroscope measurements
     */
    val gyroscopeTimeIntervalSeconds
        get() = relativeGyroscopeProcessor.timeIntervalSeconds

    /**
     * Threshold to determine that current geomagnetic attitude appears to be an outlier respect
     * to estimated fused attitude.
     * When geomagnetic attitude and fused attitudes diverge, fusion is not performed, and instead
     * only gyroscope relative attitude is used for fusion estimation.
     *
     * @throws IllegalArgumentException if value is not between 0.0 and 1.0 (both included).
     */
    var outlierThreshold = DEFAULT_OUTLIER_THRESHOLD
        set(value) {
            require(value in 0.0..1.0)

            field = value
        }

    /**
     * Threshold to determine that geomagnetic attitude has largely diverged and if situation is not
     * reverted soon, attitude will be reset to geomagnetic one.
     *
     * @throws IllegalArgumentException if value is not between 0.0 and 1.0 (both included).
     */
    var outlierPanicThreshold = DEFAULT_OUTLIER_PANIC_THRESHOLD
        set(value) {
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
            require(value > 0)

            field = value
        }

    /**
     * Counter indicating the number of fused attitudes that have largely diverged
     * from leveling attitudes.
     */
    private var panicCounter = panicCounterThreshold

    /**
     * Resets internal parameters.
     */
    fun reset() {
        geomagneticProcessor.reset()
        relativeGyroscopeProcessor.reset()

        previousRelativeAttitude = null
        hasDeltaRelativeAttitude = false
        panicCounter = panicCounterThreshold
    }

    /**
     * Processes provided synced measurement to estimate current fused absolute attitude.
     *
     * @param syncedMeasurement synced measurement to be processed.
     * @return true if a new fused absolute attitude has been estimated, false otherwise.
     */
    abstract fun process(syncedMeasurement: S): Boolean

    /**
     * Processes provided measurements to estimate current fused absolute attitude.
     *
     * @param accelerometerOrGravityMeasurement accelerometer or gravity measurement.
     * @param gyroscopeMeasurement gyroscope measurement.
     * @param magnetometerMeasurement magnetometer measurement.
     * @param timestamp timestamp when all measurements are assumed to occur.
     * @return true if new fused absolute attitude is processed, false otherwise.
     */
    internal fun process(
        accelerometerOrGravityMeasurement: M,
        gyroscopeMeasurement: GyroscopeSensorMeasurement,
        magnetometerMeasurement: MagnetometerSensorMeasurement,
        timestamp: Long = gyroscopeMeasurement.timestamp
    ): Boolean {
        if (geomagneticProcessor.process(
                accelerometerOrGravityMeasurement,
                magnetometerMeasurement
            )
        ) {
            geomagneticProcessor.fusedAttitude.copyTo(geomagneticAttitude)

            if (relativeGyroscopeProcessor.process(
                    accelerometerOrGravityMeasurement,
                    gyroscopeMeasurement,
                    timestamp
                )
            ) {
                return processRelativeAttitude(relativeGyroscopeProcessor.fusedAttitude)
            }
        }

        return false
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
            deltaRelativeAttitude.normalize()
            true
        } else {
            this.previousRelativeAttitude = Quaternion()
            relativeAttitude.copyTo(this.previousRelativeAttitude)
            false
        }
    }

    /**
     * Processes last received internal relative attitude to estimate attitude variation
     * between samples and fuse it with geomagnetic absolute attitude.
     *
     * @param attitude estimated absolute attitude respect NED coordinate system.
     * @return true if relative attitude was processed, false otherwise.
     */
    private fun processRelativeAttitude(attitude: Quaternion): Boolean {
        attitude.copyTo(relativeAttitude)
        hasDeltaRelativeAttitude = computeDeltaRelativeAttitude()

        if (!hasDeltaRelativeAttitude) {
            return false
        }

        if (resetToGeomagnetic) {
            geomagneticAttitude.copyTo(internalFusedAttitude)
            internalFusedAttitude.copyTo(fusedAttitude)
            panicCounter = 0
            Log.d(
                BaseDoubleFusedGeomagneticAttitudeProcessor::class.simpleName,
                "Attitude reset to geomagnetic one"
            )
            return true
        }

        // change attitude by the delta obtained from relative attitude (gyroscope)
        Quaternion.product(deltaRelativeAttitude, internalFusedAttitude, internalFusedAttitude)
        internalFusedAttitude.normalize()

        val absDot =
            abs(QuaternionHelper.dotProduct(internalFusedAttitude, geomagneticAttitude))

        // check if fused attitude and leveling attitude have diverged
        if (absDot < outlierThreshold) {
            Log.i(
                BaseDoubleFusedGeomagneticAttitudeProcessor::class.simpleName,
                "Threshold exceeded: $absDot"
            )
            // increase panic counter
            if (absDot < outlierPanicThreshold) {
                panicCounter++
                Log.i(
                    BaseDoubleFusedGeomagneticAttitudeProcessor::class.simpleName,
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
            internalFusedAttitude.normalize()
            panicCounter = 0
        }

        relativeAttitude.copyTo(previousRelativeAttitude)

        internalFusedAttitude.copyTo(fusedAttitude)

        return true
    }

    /**
     * Computes factor to be used to compute fusion between relative and geomagnetic attitudes.
     */
    private fun getSlerpFactor(): Double {
        return if (useIndirectInterpolation) {
            val rotationVelocity =
                deltaRelativeAttitude.rotationAngle / gyroscopeTimeIntervalSeconds
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
     * Interface to notify when a new relative attitude has been processed.
     */
    fun interface OnProcessedListener<M : SensorMeasurement<M>, S : SyncedSensorMeasurement> {

        /**
         * Called when a new fused attitude is processed.
         *
         * @param processor processor that raised this event.
         * @param fusedAttitude resulting fused attitude expressed in NED coordinates.
         * @param accelerometerOrGravityAccuracy accuracy of accelerometer or gravity measurement.
         * @param gyroscopeAccuracy accuracy of gyroscope measurement.
         * @param magnetometerAccuracy accuracy of magnetometer measurement.
         */
        fun onProcessed(
            processor: BaseFusedGeomagneticAttitudeProcessor<M, S>,
            fusedAttitude: Quaternion,
            accelerometerOrGravityAccuracy: SensorAccuracy?,
            gyroscopeAccuracy: SensorAccuracy?,
            magnetometerAccuracy: SensorAccuracy?
        )
    }

}