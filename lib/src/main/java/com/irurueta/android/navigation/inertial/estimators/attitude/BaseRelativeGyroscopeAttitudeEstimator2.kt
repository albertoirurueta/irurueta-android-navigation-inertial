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
import android.os.SystemClock
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorCollector2
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.processors.BaseRelativeGyroscopeAttitudeProcessor
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType

/**
 * Base class for estimators of device relative attitude respect to start attitude by integrating
 * gyroscope sensor data without using any additional sensors.
 *
 * @property context Android context.
 * @property sensorType One of the supported gyroscope sensor types.
 * @property sensorDelay Delay of gyroscope between samples.
 * @property startOffsetEnabled indicates whether [startOffset] will be computed when first
 * measurement is received or not. True indicates that offset is computed, false assumes that offset
 * is null.
 * @property estimateCoordinateTransformation true to estimate coordinate transformation, false
 * otherwise. If not needed, it can be disabled to improve performance and decrease cpu load.
 * @property estimateEulerAngles true to estimate euler angles, false otherwise. If not
 * needed, it can be disabled to improve performance and decrease cpu load.
 * @property attitudeAvailableListener listener to notify when a new attitude measurement is
 * available.
 * @property accuracyChangedListener listener to notify changes in accuracy.
 */
abstract class BaseRelativeGyroscopeAttitudeEstimator2<T : BaseRelativeGyroscopeAttitudeEstimator2<T, L1, L2>,
        L1 : BaseRelativeGyroscopeAttitudeEstimator2.OnAttitudeAvailableListener<T, L1, L2>,
        L2 : BaseRelativeGyroscopeAttitudeEstimator2.OnAccuracyChangedListener<T, L1, L2>>(
    val context: Context,
    val sensorType: GyroscopeSensorType,
    val sensorDelay: SensorDelay,
    val startOffsetEnabled: Boolean,
    val estimateCoordinateTransformation: Boolean,
    val estimateEulerAngles: Boolean,
    var attitudeAvailableListener: L1?,
    var accuracyChangedListener: L2?
) {
    /**
     * Instance to be reused containing estimated relative attitude in NED coordinates.
     */
    private val attitude = Quaternion()

    /**
     * Internal gyroscope sensor collector.
     */
    private val gyroscopeSensorCollector = GyroscopeSensorCollector2(
        context,
        sensorType,
        sensorDelay,
        startOffsetEnabled,
        accuracyChangedListener = { _, accuracy ->
            @Suppress("UNCHECKED_CAST")
            accuracyChangedListener?.onAccuracyChanged(this as T, accuracy)
        },
        measurementListener = { _, measurement ->
            if (processor.process(measurement)) {
                attitude.fromQuaternion(processor.attitude)
                val timestamp = measurement.timestamp
                postProcessAttitudeAndNotify(timestamp)
            }
        }
    )

    /**
     * Internal processor in charge of estimating attitude based on gyroscope measurements.
     */
    protected abstract val processor: BaseRelativeGyroscopeAttitudeProcessor

    /**
     * Array to be reused containing euler angles of leveling attitude.
     */
    protected val eulerAngles = DoubleArray(Quaternion.N_ANGLES)

    /**
     * Instance to be reused containing coordinate transformation in NED coordinates.
     */
    protected val coordinateTransformation =
        CoordinateTransformation(FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME)

    /**
     * Initial offset expressed in nano seconds between first received measurement timestamp and
     * start time expressed in the monotonically increasing system clock obtained by
     * [SystemClock.elapsedRealtimeNanos].
     */
    val startOffset: Long?
        get() = gyroscopeSensorCollector.startOffset

    /**
     * Indicates whether this estimator is running or not.
     */
    val running
        get() = gyroscopeSensorCollector.running

    /**
     * Time interval expressed in seconds between consecutive gyroscope measurements
     */
    val timeIntervalSeconds
        get() = processor.timeIntervalSeconds

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

        processor.reset()
        return gyroscopeSensorCollector.start(startTimestamp)
    }

    /**
     * Stops this estimator.
     */
    fun stop() {
        gyroscopeSensorCollector.stop()
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
                coordinateTransformation.fromRotation(attitude)
                coordinateTransformation
            } else {
                null
            }

        val displayRoll: Double?
        val displayPitch: Double?
        val displayYaw: Double?
        if (estimateEulerAngles) {
            attitude.toEulerAngles(eulerAngles)
            displayRoll = eulerAngles[0]
            displayPitch = eulerAngles[1]
            displayYaw = eulerAngles[2]
        } else {
            displayRoll = null
            displayPitch = null
            displayYaw = null
        }

        // notify
        @Suppress("UNCHECKED_CAST")
        attitudeAvailableListener?.onAttitudeAvailable(
            this as T,
            attitude,
            timestamp,
            displayRoll,
            displayPitch,
            displayYaw,
            c
        )
    }

    /**
     * Interface to notify when a new relative attitude measurement is available.
     */
    fun interface OnAttitudeAvailableListener<T : BaseRelativeGyroscopeAttitudeEstimator2<T, L1, L2>,
            L1 : OnAttitudeAvailableListener<T, L1, L2>,
            L2 : OnAccuracyChangedListener<T, L1, L2>> {

        /**
         * Called when a new attitude measurement is available.
         *
         * @param estimator attitude estimator that raised this event.
         * @param attitude attitude expressed in NED coordinates.
         * @param timestamp time in nanoseconds at which the measurement was made. Each measurement
         * wil be monotonically increasing using the same time base as
         * [android.os.SystemClock.elapsedRealtimeNanos].
         * @param roll roll angle expressed in radians respect to NED coordinate system. Only
         * available if [estimateEulerAngles].
         * @param pitch pitch angle expressed in radians respect to NED coordinate system. Only
         * available if [estimateEulerAngles].
         * @param yaw yaw angle expressed in radians respect to NED coordinate system. Only
         * available if [estimateEulerAngles].
         * @param coordinateTransformation coordinate transformation containing measured leveling
         * attitude. Only available if [estimateCoordinateTransformation].
         */
        fun onAttitudeAvailable(
            estimator: T,
            attitude: Quaternion,
            timestamp: Long,
            roll: Double?,
            pitch: Double?,
            yaw: Double?,
            coordinateTransformation: CoordinateTransformation?
        )
    }

    /**
     * Interface to notify when sensor accuracy changes.
     */
    fun interface OnAccuracyChangedListener<T : BaseRelativeGyroscopeAttitudeEstimator2<T, L1, L2>,
            L1 : OnAttitudeAvailableListener<T, L1, L2>,
            L2 : OnAccuracyChangedListener<T, L1, L2>> {

        /**
         * Called when gyroscope accuracy changes.
         *
         * @param estimator leveling estimator that raised this event.
         * @param accuracy new accuracy.
         */
        fun onAccuracyChanged(estimator: T, accuracy: SensorAccuracy?)
    }
}