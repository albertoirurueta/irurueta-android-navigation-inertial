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
package com.irurueta.android.navigation.inertial.processors

import android.location.Location
import android.util.Log
import com.irurueta.android.navigation.inertial.QuaternionHelper
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.SensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.SyncedSensorMeasurement
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import kotlin.math.abs
import kotlin.math.min

/**
 * Base class to estimate leveled relative attitude by fusing leveling attitude obtained
 * from accelerometer or gravity sensors, and relative attitude obtained from gyroscope sensor.
 *
 * @property processorListener listener to notify new leveled relative attitudes.
 *
 * @param M type of sensor measurement.
 * @param S type of synced sensor measurement.
 */
abstract class BaseLeveledRelativeAttitudeProcessor<M : SensorMeasurement<M>,
        S : SyncedSensorMeasurement>(
    var processorListener: OnProcessedListener<M, S>?
) {
    /**
     * Internal processor to estimate gravity from accelerometer or gravity sensor measurements.
     */
    protected abstract val gravityProcessor: BaseGravityProcessor<M>

    /**
     * Internal processor to estimate leveling attitude from estimated gravity.
     */
    private lateinit var levelingProcessor: BaseLevelingProcessor

    /**
     * Internal processor to estimate relative attitude from an arbitrary initial attitude using
     * gyroscope sensor measurements.
     */
    private lateinit var relativeAttitudeProcessor: BaseRelativeGyroscopeAttitudeProcessor

    /**
     * Timestamp of last attitude estimation.
     */
    private var timestamp = 0L

    /**
     * Instance to be reused which contains attitude of internal leveling processor.
     */
    private val levelingAttitude = Quaternion()

    /**
     * Instance to be reused which contains attitude of internal relative attitude processor.
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
     * Delta attitude of relative attitude, which only uses gyroscope sensor.
     */
    private val deltaRelativeAttitude = Quaternion()

    /**
     * Array to be reused containing euler angles of leveling attitude.
     */
    private val eulerAngles = DoubleArray(Quaternion.N_ANGLES)

    /**
     * Indicates whether fused attitude must be reset to leveling one.
     */
    private val resetToLeveling
        get() = panicCounter >= panicCounterThreshold

    /**
     * Instance to be reused which contains merged attitudes of both the internal leveling
     * estimator and the relative attitude estimator taking into account display orientation.
     */
    private val internalFusedAttitude = Quaternion()

    /**
     * Instance to be reused which contains merged attitudes of both the internal leveling
     * processor and the relative attitude processor.
     */
    val fusedAttitude = Quaternion()

    /**
     * X-coordinates of last sensed gravity component of specific force expressed in NED coordinates
     * and in meters per squared second (m/s^2).
     */
    val gx: Double
        get() = gravityProcessor.gx

    /**
     * Y-coordinate of last sensed gravity component of specific force expressed in NED coordinates
     * and in meters per squared second (m/s^2).
     */
    val gy: Double
        get() = gravityProcessor.gy

    /**
     * Z-coordinate of last sensed gravity component of specific force expressed in NED coordinates
     * and in meters per squared second (m/s^2).
     */
    val gz: Double
        get() = gravityProcessor.gz

    /**
     * Gets a new triad containing gravity component of specific force expressed in NED coordinates
     * and in meters per squared second (m/s^2).
     */
    val gravity: AccelerationTriad
        get() = gravityProcessor.gravity

    /**
     * Updates provided triad to contain gravity component of specific force expressed in NED
     * coordinates and in meters per squared second (m/s^2).
     */
    fun getGravity(result: AccelerationTriad) {
        gravityProcessor.getGravity(result)
    }

    /**
     * Gets or sets device location
     */
    var location: Location? = null

    /**
     * Indicates whether accurate leveling must be used or not.
     *
     * @throws IllegalStateException if set to true and no location is available.
     */
    var useAccurateLevelingProcessor: Boolean = false
        @Throws(IllegalStateException::class)
        set(value) {
            if (value) {
                checkNotNull(location)
            }

            field = value
            buildLevelingProcessor()
        }

    /**
     * Indicates whether accurate non-leveled relative attitude must be used or not.
     */
    var useAccurateRelativeGyroscopeAttitudeProcessor: Boolean = true
        set(value) {
            field = value
            buildRelativeAttitudeProcessor()
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
        @Throws(IllegalArgumentException::class)
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
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value > 0.0)
            field = value
        }

    /**
     * Gets average time interval between gyroscope samples expressed in seconds.
     */
    val gyroscopeAverageTimeInterval
        get() = relativeAttitudeProcessor.averageTimeInterval

    /**
     * Threshold to determine that current leveling appears to be an outlier respect
     * to estimated fused attitude.
     * When leveling and fused attitudes diverge, fusion is not performed, and instead
     * only gyroscope relative attitude is used for fusion estimation.
     *
     * @throws IllegalArgumentException if value is not between 0.0 and 1.0 (both included).
     */
    var outlierThreshold = DEFAULT_OUTLIER_THRESHOLD
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value in 0.0..1.0)

            field = value
        }

    /**
     * Threshold to determine that leveling has largely diverged and if situation is not
     * reverted soon, attitude will be reset to leveling
     *
     * @throws IllegalArgumentException if value is not between 0.0 and 1.0 (both included).
     */
    var outlierPanicThreshold = DEFAULT_OUTLIER_PANIC_THRESHOLD
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value in 0.0..1.0)

            field = value
        }

    /**
     * Threshold to determine when fused attitude has largely diverged for a given
     * number of samples and must be reset.
     *
     * @throws IllegalArgumentException if provided is zero or negative.
     */
    var panicCounterThreshold = DEFAULT_PANIC_COUNTER_THRESHOLD
        @Throws(IllegalArgumentException::class)
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
     * Processes provided synced measurement to estimate current leveled relative attitude.
     *
     * @param syncedMeasurement synced measurement to be processed.
     * @return true if a new leveled relative attitude has been estimated, false otherwise.
     */
    abstract fun process(syncedMeasurement: S): Boolean

    /**
     * Resets internal parameters.
     */
    fun reset() {
        panicCounter = panicCounterThreshold

        gravityProcessor.reset()
        levelingProcessor.reset()
        relativeAttitudeProcessor.reset()
    }

    /**
     * Processes provided measurements to estimate fused leveled relative attitude.
     *
     * @param accelerometerOrGravityMeasurement accelerometer or gravity measurement.
     * @param gyroscopeMeasurement gyroscope measurement.
     * @param timestamp timestamp when both measurements are assumed to occur.
     */
    protected fun process(
        accelerometerOrGravityMeasurement: M,
        gyroscopeMeasurement: GyroscopeSensorMeasurement,
        timestamp: Long
    ): Boolean {
        if (relativeAttitudeProcessor.process(gyroscopeMeasurement, timestamp)) {
            if (processRelativeAttitude(relativeAttitudeProcessor.attitude, timestamp)
                && gravityProcessor.process(accelerometerOrGravityMeasurement, timestamp)
            ) {
                val gx = gravityProcessor.gx
                val gy = gravityProcessor.gy
                val gz = gravityProcessor.gz
                levelingProcessor.process(gx, gy, gz)

                processLeveling(levelingProcessor.attitude)

                // notify
                processorListener?.onProcessed(
                    this,
                    fusedAttitude,
                    accelerometerOrGravityMeasurement.accuracy,
                    gyroscopeMeasurement.accuracy
                )

                return true
            }
        }

        return false
    }

    /**
     * Builds the internal leveling processor.
     */
    private fun buildLevelingProcessor() {
        levelingProcessor = if (useAccurateLevelingProcessor) {
            val location = this.location
            checkNotNull(location)

            AccurateLevelingProcessor(location)
        } else {
            LevelingProcessor()
        }
    }

    /**
     * Builds the internal relative attitude processor.
     */
    private fun buildRelativeAttitudeProcessor() {
        relativeAttitudeProcessor = if (useAccurateRelativeGyroscopeAttitudeProcessor) {
            AccurateRelativeGyroscopeAttitudeProcessor()
        } else {
            RelativeGyroscopeAttitudeProcessor()
        }
    }

    /**
     * Computes variation of relative attitude between samples.
     * Relative attitude only depends on gyroscope and is usually smoother and has
     * less interferences than leveling attitude.
     *
     * @return true if processor already has a fully initialized delta relative attitude, false
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
     * leveled relative attitude, where only yaw is kept relative to the start of this processor.
     */
    private fun processLeveling(attitude: Quaternion) {
        attitude.copyTo(levelingAttitude)

        // set yaw angle into leveled attitude
        levelingAttitude.toEulerAngles(eulerAngles)
        val levelingRoll = eulerAngles[0]
        val levelingPitch = eulerAngles[1]

        relativeAttitude.toEulerAngles(eulerAngles)
        val yaw = eulerAngles[2]
        levelingAttitude.setFromEulerAngles(levelingRoll, levelingPitch, yaw)

        if (resetToLeveling) {
            levelingAttitude.copyTo(internalFusedAttitude)
            relativeAttitude.copyTo(previousRelativeAttitude)
            internalFusedAttitude.copyTo(fusedAttitude)
            panicCounter = 0
            Log.d(
                BaseLeveledRelativeAttitudeProcessor::class.simpleName,
                "Attitude reset to leveling"
            )
            return
        }

        // change attitude by the delta obtained from relative attitude (gyroscope)
        Quaternion.product(deltaRelativeAttitude, internalFusedAttitude, internalFusedAttitude)

        val absDot = abs(QuaternionHelper.dotProduct(internalFusedAttitude, levelingAttitude))

        // check if fused attitude and leveling attitude have diverged
        if (absDot < outlierThreshold) {
            Log.i(
                BaseLeveledRelativeAttitudeProcessor::class.simpleName,
                "Threshold exceeded: $absDot"
            )
            // increase panic counter
            if (absDot < outlierPanicThreshold) {
                panicCounter++
                Log.i(
                    BaseLeveledRelativeAttitudeProcessor::class.simpleName,
                    "Panic counter increased: $panicCounter"
                )
            }

            // directly use attitude which has combined only delta gyroscope data
        } else {
            // both are nearly the same. Perform normal fusion
            Quaternion.slerp(
                internalFusedAttitude,
                levelingAttitude,
                getSlerpFactor(),
                internalFusedAttitude
            )
            panicCounter = 0
        }

        relativeAttitude.copyTo(previousRelativeAttitude)
        internalFusedAttitude.copyTo(fusedAttitude)
    }

    /**
     * Processes last received internal relative attitude to estimate attitude variation
     * between samples.
     *
     * @param attitude attitude expressed in NED coordinates.
     * @param timestamp time in nanoseconds at which the measurement was made. Each measurement
     * wil be monotonically increasing using the same time base as
     * [android.os.SystemClock.elapsedRealtimeNanos].
     * @return true if delta relative attitude is fully estimated when two or more gyroscope
     * measurements have been processed, false otherwise.
     */
    private fun processRelativeAttitude(attitude: Quaternion, timestamp: Long): Boolean {
        attitude.copyTo(relativeAttitude)
        this.timestamp = timestamp
        return computeDeltaRelativeAttitude()
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

    init {
        buildLevelingProcessor()
        buildRelativeAttitudeProcessor()
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
     * Interface to notify when a new leveled relative attitude has been processed.
     *
     * @param M type of sensor measurement.
     * @param S type of synced sensor measurement.
     */
    fun interface OnProcessedListener<M : SensorMeasurement<M>, S : SyncedSensorMeasurement> {
        /**
         * Called when a new leveled relative attitude is processed.
         *
         * @param processor processor that raised this event.
         * @param fusedAttitude estimated fused attitude.
         * @param accelerometerOrGravityAccuracy accuracy of accelerometer or gravity sensor used
         * for leveling.
         * @param gyroscopeAccuracy accuracy of gyroscope sensor used for relative attitude.
         */
        fun onProcessed(
            processor: BaseLeveledRelativeAttitudeProcessor<M, S>,
            fusedAttitude: Quaternion,
            accelerometerOrGravityAccuracy: SensorAccuracy?,
            gyroscopeAccuracy: SensorAccuracy?
        )
    }
}