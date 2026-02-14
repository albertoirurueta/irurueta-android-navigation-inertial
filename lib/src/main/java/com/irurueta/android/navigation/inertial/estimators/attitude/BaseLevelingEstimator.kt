/*
 * Copyright (C) 2026 Alberto Irurueta Carro (alberto@irurueta.com)
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
import android.os.SystemClock
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.GravitySensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorType
import com.irurueta.android.navigation.inertial.processors.attitude.AccelerometerGravityProcessor
import com.irurueta.android.navigation.inertial.processors.attitude.BaseLevelingProcessor
import com.irurueta.android.navigation.inertial.processors.attitude.GravityProcessor
import com.irurueta.android.navigation.inertial.processors.filters.AveragingFilter
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.units.Acceleration
import com.irurueta.units.AccelerationUnit

/**
 * Base class for estimators of device leveling (roll and pitch angles ignoring yaw angle) by
 * estimating gravity vector using accelerometer measurements only, or by using Android's gravity
 * sensor.
 * Implementations of this estimator does not estimate attitude yaw angle, as either a magnetometer
 * or very accurate gyroscope would be needed.
 *
 * @property context Android context.
 * @property sensorDelay Delay of accelerometer or gravity sensor between samples.
 * @property useAccelerometer true to use accelerometer sensor, false to use system gravity sensor.
 * @property accelerometerSensorType One of the supported accelerometer sensor types.
 * (Only used if [useAccelerometer] is true).
 * @property accelerometerAveragingFilter an averaging filter for accelerometer samples to obtain
 * sensed gravity component of specific force. (Only used if [useAccelerometer] is true).
 * @property estimateCoordinateTransformation true to estimate coordinate transformation, false
 * otherwise. If not needed, it can be disabled to improve performance and decrease cpu load.
 * @property estimateEulerAngles true to estimate euler angles, false otherwise. If not
 * needed, it can be disabled to improve performance and decrease cpu load.
 * @property levelingAvailableListener listener to notify when a new leveling measurement is
 * available.
 * @property accuracyChangedListener listener to notify changes in accuracy.
 * @property adjustGravityNorm indicates whether gravity norm must be adjusted to either Earth
 * standard norm, or norm at provided location. If no location is provided, this should only be
 * enabled when device is close to sea level.
 */
abstract class BaseLevelingEstimator<T : BaseLevelingEstimator<T>>(
    val context: Context,
    val sensorDelay: SensorDelay,
    val useAccelerometer: Boolean,
    val accelerometerSensorType: AccelerometerSensorType,
    val accelerometerAveragingFilter: AveragingFilter<AccelerationUnit, Acceleration, AccelerationTriad>,
    val estimateCoordinateTransformation: Boolean,
    val estimateEulerAngles: Boolean,
    var levelingAvailableListener: OnLevelingAvailableListener<T>?,
    var accuracyChangedListener: OnAccuracyChangedListener<T>?,
    adjustGravityNorm: Boolean
) {
    /**
     * Internal processor to estimate leveled attitude from accelerometer or gravity measurements.
     */
    protected abstract val levelingProcessor: BaseLevelingProcessor

    /**
     * Processor to estimate gravity expressed in NED coordinates from Android gravity sensor
     * measurements.
     */
    protected val gravityProcessor = GravityProcessor()

    /**
     * Processor to estimate gravity expressed in NED coordinates from Android accelerometer sensor
     * measurements.
     */
    protected val accelerometerGravityProcessor =
        AccelerometerGravityProcessor(accelerometerAveragingFilter)

    /**
     * Instance to be reused containing estimated leveling attitude (roll and pitch angles) in NED
     * coordinates.
     */
    private val attitude = Quaternion()

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
     * Internal gravity sensor collector.
     */
    private val gravitySensorCollector = GravitySensorCollector(
        context,
        sensorDelay,
        accuracyChangedListener = { _, accuracy ->
            notifyAccuracyChanged(accuracy)
        },
        measurementListener = { _, measurement ->
            if (gravityProcessor.process(measurement)) {
                val gx = gravityProcessor.gx
                val gy = gravityProcessor.gy
                val gz = gravityProcessor.gz
                levelingProcessor.process(gx, gy, gz)

                attitude.fromQuaternion(levelingProcessor.attitude)
                val timestamp = measurement.timestamp
                postProcessAttitudeAndNotify(timestamp)
            }
        }
    )

    /**
     * Internal accelerometer sensor collector
     */
    private val accelerometerSensorCollector = AccelerometerSensorCollector(
        context,
        accelerometerSensorType,
        sensorDelay,
        accuracyChangedListener = { _, accuracy ->
            notifyAccuracyChanged(accuracy)
        },
        measurementListener = { _, measurement ->
            if (accelerometerGravityProcessor.process(measurement)) {
                val gx = accelerometerGravityProcessor.gx
                val gy = accelerometerGravityProcessor.gy
                val gz = accelerometerGravityProcessor.gz
                levelingProcessor.process(gx, gy, gz)

                attitude.fromQuaternion(levelingProcessor.attitude)
                val timestamp = measurement.timestamp
                postProcessAttitudeAndNotify(timestamp)
            }
        }
    )

    /**
     * Indicates whether gravity norm must be adjusted to either Earth standard norm, or norm at
     * provided location. If no location is provided, this should only be enabled when device is
     * close to sea level.
     */
    var adjustGravityNorm: Boolean = adjustGravityNorm
        set(value) {
            field = value
            gravityProcessor.adjustGravityNorm = value
            accelerometerGravityProcessor.adjustGravityNorm = value
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

        return if (useAccelerometer) {
            accelerometerSensorCollector.start(startTimestamp)
        } else {
            gravitySensorCollector.start(startTimestamp)
        }
    }

    /**
     * Stops this estimator.
     */
    fun stop() {
        accelerometerSensorCollector.stop()
        gravitySensorCollector.stop()
    }

    /**
     * Indicates whether this estimator is running or not.
     */
    val running
        get() = accelerometerSensorCollector.running || gravitySensorCollector.running

    /**
     * Processes current attitude and computes (if needed) a coordinate transformation or display
     * Euler angles.
     *
     * @param timestamp time in nanoseconds at which the measurement was made. Each measurement
     * wil be monotonically increasing using the same time base as
     * [SystemClock.elapsedRealtimeNanos].
     */
    private fun postProcessAttitudeAndNotify(timestamp: Long) {
        val c: CoordinateTransformation? =
            if (estimateCoordinateTransformation) {
                coordinateTransformation.fromRotation(attitude)
                coordinateTransformation
            } else {
                null
            }

        val roll: Double?
        val pitch: Double?
        if (estimateEulerAngles) {
            attitude.toEulerAngles(eulerAngles)
            roll = eulerAngles[0]
            pitch = eulerAngles[1]
        } else {
            roll = null
            pitch = null
        }

        // notify
        @Suppress("UNCHECKED_CAST")
        levelingAvailableListener?.onLevelingAvailable(
            this as T,
            attitude,
            timestamp,
            roll,
            pitch,
            c
        )
    }

    /**
     * Notifies accuracy change.
     */
    private fun notifyAccuracyChanged(accuracy: SensorAccuracy?) {
        val sensorType = if (useAccelerometer) {
            SensorType.from(accelerometerSensorType.value)
        } else {
            SensorType.GRAVITY
        }

        @Suppress("UNCHECKED_CAST")
        accuracyChangedListener?.onAccuracyChanged(this as T, sensorType, accuracy)
    }

    fun interface OnLevelingAvailableListener<T : BaseLevelingEstimator<T>> {
        /**
         * Called when a new leveling measurement is available.
         *
         * @param estimator leveling estimator that raised this event.
         * @param attitude leveling attitude (roll and pitch angles) expressed in NED coordinates.
         * @param timestamp time in nanoseconds at which the measurement was made. Each measurement
         * wil be monotonically increasing using the same time base as
         * [SystemClock.elapsedRealtimeNanos].
         * @param roll roll angle expressed in radians respect to NED coordinate system. Only
         * available if [estimateEulerAngles].
         * @param pitch pitch angle expressed in radians respect to NED coordinate system. Only
         * available if [estimateEulerAngles].
         * @param coordinateTransformation coordinate transformation containing measured leveling
         * attitude. Only available if [estimateCoordinateTransformation].
         */
        fun onLevelingAvailable(
            estimator: T,
            attitude: Quaternion,
            timestamp: Long,
            roll: Double?,
            pitch: Double?,
            coordinateTransformation: CoordinateTransformation?
        )
    }

    /**
     * Interface to notify when sensor (either accelerometer or gravity) accuracy changes.
     *
     * @property T type of estimator.
     */
    fun interface OnAccuracyChangedListener<T : BaseLevelingEstimator<T>> {
        /**
         * Called when accuracy changes.
         *
         * @param estimator leveling estimator that raised this event.
         * @param sensorType sensor that has changed its accuracy
         * @param accuracy new accuracy.
         */
        fun onAccuracyChanged(
            estimator: T,
            sensorType: SensorType?,
            accuracy: SensorAccuracy?
        )
    }
}