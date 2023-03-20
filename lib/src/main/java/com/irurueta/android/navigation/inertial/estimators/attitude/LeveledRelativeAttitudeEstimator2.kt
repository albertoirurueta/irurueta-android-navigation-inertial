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
package com.irurueta.android.navigation.inertial.estimators.attitude

import android.content.Context
import android.location.Location
import android.os.SystemClock
import com.irurueta.android.navigation.inertial.collectors.*
import com.irurueta.android.navigation.inertial.estimators.filter.AveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.processors.attitude.AccelerometerLeveledRelativeAttitudeProcessor
import com.irurueta.android.navigation.inertial.processors.attitude.LeveledRelativeAttitudeProcessor
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType

/**
 * Estimates a leveled relative attitude, where only yaw Euler angle is relative to the start
 * instant of this estimator.
 * Roll and pitch Euler angles are leveled using accelerometer or gravity sensors.
 *
 * @property context Android context.
 * @param location Device location.
 * @property sensorDelay Delay of sensors between samples.
 * @property useAccelerometer true to use accelerometer sensor, false to use system gravity sensor
 * for leveling purposes.
 * @property startOffsetEnabled indicates whether start offsets will be computed when first
 * measurement is received or not. True indicates that offset is computed, false assumes that offset
 * is null.
 * @property accelerometerSensorType One of the supported accelerometer sensor types.
 * (Only used if [useAccelerometer] is true).
 * @property accelerometerAveragingFilter an averaging filter for accelerometer samples to obtain
 * sensed gravity component of specific force. (Only used if [useAccelerometer] is true).
 * @property gyroscopeSensorType One of the supported gyroscope sensor types.
 * @param useAccurateLevelingEstimator true to use accurate leveling, false to use a normal one.
 * @param useAccurateRelativeGyroscopeAttitudeEstimator true to use accurate relative attitude,
 * false to use a normal one.
 * @property estimateCoordinateTransformation true to estimate coordinate transformation, false
 * otherwise. If not needed, it can be disabled to improve performance and decrease cpu load.
 * @property estimateEulerAngles true to estimate euler angles, false otherwise. If not
 * needed, it can be disabled to improve performance and decrease cpu load.
 * @property attitudeAvailableListener listener to notify when a new attitude measurement is
 * available.
 * @property accuracyChangedListener listener to notify changes in accuracy.
 * @property bufferFilledListener listener to notify that some buffer has been filled. This usually
 * happens when consumer of measurements cannot keep up with the rate at which measurements are
 * generated.
 */
class LeveledRelativeAttitudeEstimator2(
    val context: Context,
    location: Location? = null,
    val sensorDelay: SensorDelay = SensorDelay.GAME,
    val useAccelerometer: Boolean = true,
    val startOffsetEnabled: Boolean = true,
    val accelerometerSensorType: AccelerometerSensorType =
        AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
    val accelerometerAveragingFilter: AveragingFilter = LowPassAveragingFilter(),
    val gyroscopeSensorType: GyroscopeSensorType = GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
    useAccurateLevelingEstimator: Boolean = false,
    useAccurateRelativeGyroscopeAttitudeEstimator: Boolean = true,
    val estimateCoordinateTransformation: Boolean = false,
    val estimateEulerAngles: Boolean = true,
    var attitudeAvailableListener: OnAttitudeAvailableListener? = null,
    var accuracyChangedListener: OnAccuracyChangedListener? = null,
    var bufferFilledListener: OnBufferFilledListener? = null,
    adjustGravityNorm: Boolean = true
) {
    /**
     * Processes accelerometer + gyroscope measurements.
     */
    private val accelerometerProcessor = AccelerometerLeveledRelativeAttitudeProcessor()

    /**
     * Processes gravity + gyroscope measurements.
     */
    private val gravityProcessor = LeveledRelativeAttitudeProcessor()

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
     * Internal syncer to collect and sync gravity and gyroscope measurements.
     */
    private val gravityAndGyroscopeSyncer = GravityAndGyroscopeSensorMeasurementSyncer(
        context,
        gyroscopeSensorType,
        sensorDelay,
        sensorDelay,
        gravityStartOffsetEnabled = startOffsetEnabled,
        gyroscopeStartOffsetEnabled = startOffsetEnabled,
        stopWhenFilledBuffer = false,
        staleDetectionEnabled = true,
        accuracyChangedListener = { _, sensorType, accuracy ->
            notifyAccuracyChanged(sensorType, accuracy)
        },
        bufferFilledListener = { _, sensorType ->
            notifyBufferFilled(sensorType)
        },
        syncedMeasurementListener = { _, measurement ->
            if (gravityProcessor.process(measurement)) {
                gravityProcessor.fusedAttitude.copyTo(fusedAttitude)

                val timestamp = measurement.timestamp
                postProcessAttitudeAndNotify(timestamp)
            }
        }
    )

    /**
     * Internal syncer to collect and sync accelerometer and magnetometer measurements.
     */
    private val accelerometerAndGyroscopeSyncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(
        context,
        accelerometerSensorType,
        gyroscopeSensorType,
        sensorDelay,
        sensorDelay,
        accelerometerStartOffsetEnabled = startOffsetEnabled,
        gyroscopeStartOffsetEnabled = startOffsetEnabled,
        stopWhenFilledBuffer = false,
        staleDetectionEnabled = true,
        accuracyChangedListener = { _, sensorType, accuracy ->
            notifyAccuracyChanged(sensorType, accuracy)
        },
        bufferFilledListener = { _, sensorType ->
            notifyBufferFilled(sensorType)
        },
        syncedMeasurementListener = { _, measurement ->
            if (accelerometerProcessor.process(measurement)) {
                accelerometerProcessor.fusedAttitude.copyTo(fusedAttitude)

                val timestamp = measurement.timestamp
                postProcessAttitudeAndNotify(timestamp)
            }
        }
    )

    /**
     * Gets or sets device location
     *
     * @throws IllegalStateException if estimator is running and a null value is set.
     */
    var location: Location? = location
        @Throws(IllegalStateException::class)
        set(value) {
            check(value != null || !running)

            accelerometerProcessor.location = value
            gravityProcessor.location = value
            field = value
        }

    /**
     * Indicates whether gravity norm must be adjusted to either Earth standard norm, or norm at
     * provided location. If no location is provided, this should only be enabled when device is
     * close to sea level.
     */
    var adjustGravityNorm: Boolean = adjustGravityNorm
        set(value) {
            field = value
            gravityProcessor.adjustGravityNorm = value
            accelerometerProcessor.adjustGravityNorm = value
        }

    /**
     * Indicates whether accurate leveling must be used or not.
     *
     * @throws IllegalStateException if estimator is running.
     */
    var useAccurateLevelingEstimator: Boolean = useAccurateLevelingEstimator
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            if (value) {
                checkNotNull(location)
            }

            accelerometerProcessor.useAccurateLevelingProcessor = value
            gravityProcessor.useAccurateLevelingProcessor = value
            field = value
        }

    /**
     * Indicates whether accurate non-leveled relative attitude estimator must be used or not.
     *
     * @throws IllegalStateException if estimator is running.
     */
    var useAccurateRelativeGyroscopeAttitudeEstimator: Boolean =
        useAccurateRelativeGyroscopeAttitudeEstimator
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)

            accelerometerProcessor.useAccurateRelativeGyroscopeAttitudeProcessor = value
            gravityProcessor.useAccurateRelativeGyroscopeAttitudeProcessor = value
            field = value
        }

    /**
     * Indicates whether fusion between leveling and relative attitudes occurs based
     * on changing interpolation value that depends on actual relative attitude rotation
     * velocity.
     */
    var useIndirectInterpolation: Boolean
        get() = accelerometerProcessor.useIndirectInterpolation
        set(value) {
            accelerometerProcessor.useIndirectInterpolation = value
            gravityProcessor.useIndirectInterpolation = value
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
    var interpolationValue: Double
        get() = accelerometerProcessor.interpolationValue
        @Throws(IllegalArgumentException::class)
        set(value) {
            accelerometerProcessor.interpolationValue = value
            gravityProcessor.interpolationValue = value
        }

    /**
     * Factor to take into account when interpolation value is computed and
     * [useIndirectInterpolation] is enabled to determine actual interpolation value based
     * on current relative attitude rotation velocity.
     *
     * @throws IllegalArgumentException if value is zero or negative.
     */
    var indirectInterpolationWeight: Double
        get() = accelerometerProcessor.indirectInterpolationWeight
        @Throws(IllegalArgumentException::class)
        set(value) {
            accelerometerProcessor.indirectInterpolationWeight = value
            gravityProcessor.indirectInterpolationWeight = value
        }

    /**
     * Gets time interval between gyroscope samples expressed in seconds.
     */
    val gyroscopeTimeIntervalSeconds
        get() = if (useAccelerometer) {
            accelerometerProcessor.timeIntervalSeconds
        } else {
            gravityProcessor.timeIntervalSeconds
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
    var outlierThreshold: Double
        get() = accelerometerProcessor.outlierThreshold
        @Throws(IllegalStateException::class, IllegalArgumentException::class)
        set(value) {
            check(!running)
            accelerometerProcessor.outlierThreshold = value
            gravityProcessor.outlierThreshold = value
        }

    /**
     * Threshold to determine that leveling has largely diverged and if situation is not
     * reverted soon, attitude will be reset to leveling
     *
     * @throws IllegalStateException if estimator is already running.
     * @throws IllegalArgumentException if value is not between 0.0 and 1.0 (both included).
     */
    var outlierPanicThreshold: Double
        get() = accelerometerProcessor.outlierPanicThreshold
        @Throws(IllegalStateException::class, IllegalArgumentException::class)
        set(value) {
            check(!running)
            accelerometerProcessor.outlierPanicThreshold = value
            gravityProcessor.outlierPanicThreshold = value
        }

    /**
     * Threshold to determine when fused attitude has largely diverged for a given
     * number of samples and must be reset.
     *
     * @throws IllegalStateException if estimator is already running.
     * @throws IllegalArgumentException if provided is zero or negative.
     */
    var panicCounterThreshold: Int
        get() = accelerometerProcessor.panicCounterThreshold
        @Throws(IllegalStateException::class, IllegalArgumentException::class)
        set(value) {
            check(!running)
            accelerometerProcessor.panicCounterThreshold = value
            gravityProcessor.panicCounterThreshold = value
        }

    /**
     * Starts this estimator.
     *
     * @param startTimestamp monotonically increasing timestamp when collector starts. If not
     * provided, system clock is used by default, otherwise, the value can be provided to sync
     * multiple sensor collector instances.
     * @return true if estimator successfully started, false otherwise.
     * @throws IllegalStateException if estimator is already running.
     */
    @Throws(IllegalStateException::class)
    fun start(startTimestamp: Long = SystemClock.elapsedRealtimeNanos()): Boolean {
        check(!running)

        running = if (useAccelerometer) {
            accelerometerProcessor.reset()
            accelerometerAndGyroscopeSyncer.start(startTimestamp)
        } else {
            gravityProcessor.reset()
            gravityAndGyroscopeSyncer.start(startTimestamp)
        }

        return running
    }

    /**
     * Stops this estimator.
     */
    fun stop() {
        accelerometerAndGyroscopeSyncer.stop()
        gravityAndGyroscopeSyncer.stop()
        running = false
    }

    /**
     * Notifies changes in sensor accuracy.
     */
    private fun notifyAccuracyChanged(
        sensorType: SensorType,
        accuracy: SensorAccuracy?
    ) {
        accuracyChangedListener?.onAccuracyChanged(this, sensorType, accuracy)
    }

    /**
     * Notifies when a sensor buffer becomes full.
     */
    private fun notifyBufferFilled(sensorType: SensorType) {
        bufferFilledListener?.onBufferFilled(this, sensorType)
    }

    /**
     * Processes current attitude and computes (if needed) a coordinate transformation or display
     * Euler angles.
     *
     * @param timestamp time in nanoseconds at which the measurement was made. Each measurement
     * wil be monotonically increasing using the same time base as
     * [android.os.SystemClock.elapsedRealtimeNanos].
     */
    private fun postProcessAttitudeAndNotify(timestamp: Long) {
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

        // notify
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

    init {
        this.location = location
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
            estimator: LeveledRelativeAttitudeEstimator2,
            attitude: Quaternion,
            timestamp: Long,
            roll: Double?,
            pitch: Double?,
            yaw: Double?,
            coordinateTransformation: CoordinateTransformation?
        )
    }

    /**
     * Interface to notify when sensor (either accelerometer, gravity or gyroscope) accuracy
     * changes.
     */
    fun interface OnAccuracyChangedListener {
        /**
         * Called when accuracy changes.
         *
         * @param estimator leveled relative attitude estimator that raised this event.
         * @param sensorType sensor that has changed its accuracy
         * @param accuracy new accuracy.
         */
        fun onAccuracyChanged(
            estimator: LeveledRelativeAttitudeEstimator2,
            sensorType: SensorType,
            accuracy: SensorAccuracy?
        )
    }

    /**
     * Interface to notify when a buffer gets completely filled.
     * When buffers get filled, internal collectors will continue collection at the expense of
     * loosing old data. Consumers of this listener should device what to do at this point (which
     * might require stopping this estimator)..
     */
    fun interface OnBufferFilledListener {
        /**
         * Called when any buffer gets completely filled.
         *
         * @param estimator leveled relative attitude estimator that raised this event.
         * @param sensorType sensor that got its buffer filled.
         */
        fun onBufferFilled(
            estimator: LeveledRelativeAttitudeEstimator2,
            sensorType: SensorType
        )
    }
}