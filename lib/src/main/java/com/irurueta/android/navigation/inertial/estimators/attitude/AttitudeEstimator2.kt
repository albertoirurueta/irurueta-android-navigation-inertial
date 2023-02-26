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
import com.irurueta.android.navigation.inertial.collectors.*
import com.irurueta.android.navigation.inertial.processors.attitude.AttitudeProcessor
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType

/**
 * Estimates absolute or relative attitude using Android rotation vector sensors.
 * When using an [AttitudeSensorType.ABSOLUTE_ATTITUDE], android combines accelerometer and
 * magnetometer measurements to obtain a leveled absolute attitude respect to Earth.
 * When using an [AttitudeSensorType.RELATIVE_ATTITUDE], android does not use a magnetometer
 * and a leveled attitude is obtained with an arbitrary yaw angle respect to Earth.
 *
 * @property context Android context.
 * @property sensorDelay Delay of sensors between samples.
 * @property attitudeSensorType One of the supported attitude sensor types.
 * @property startOffsetEnabled indicates whether start offsets will be computed when first
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
class AttitudeEstimator2(
    val context: Context,
    val sensorDelay: SensorDelay = SensorDelay.GAME,
    val attitudeSensorType: AttitudeSensorType = AttitudeSensorType.ABSOLUTE_ATTITUDE,
    val startOffsetEnabled: Boolean = true,
    val estimateCoordinateTransformation: Boolean = false,
    val estimateEulerAngles: Boolean = true,
    var attitudeAvailableListener: OnAttitudeAvailableListener? = null,
    var accuracyChangedListener: OnAccuracyChangedListener? = null
) {
    /**
     * Processor to convert from Android ENU coordinates system to NED coordinates system.
     */
    private val attitudeProcessor = AttitudeProcessor()

    /**
     * Instance to be reused containing estimated attitude in NED coordinates.
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
     * Internal sensor type being reused.
     */
    private val sensorType = SensorType.from(attitudeSensorType)

    /**
     * Internal Android attitude sensor collector.
     */
    private val attitudeSensorCollector = AttitudeSensorCollector2(
        context,
        attitudeSensorType,
        sensorDelay,
        startOffsetEnabled,
        accuracyChangedListener = { _, accuracy ->
            accuracyChangedListener?.onAccuracyChanged(
                this@AttitudeEstimator2,
                sensorType,
                accuracy
            )
        },
        measurementListener = { _, measurement ->
            attitudeProcessor.process(measurement)
            attitude.fromQuaternion(attitudeProcessor.nedAttitude)
            val timestamp = measurement.timestamp
            val headingAccuracy = measurement.headingAccuracy
            postProcessAttitudeAndNotify(timestamp, headingAccuracy)
        }
    )

    /**
     * Indicates whether this estimator is running or not.
     */
    var running: Boolean = false
        private set

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

        running = attitudeSensorCollector.start(startTimestamp)
        return running
    }

    /**
     * Stops this estimator.
     */
    fun stop() {
        attitudeSensorCollector.stop()
        running = false
    }

    /**
     * Processes current attitude and computes (if needed) a coordinate transformation or display
     * Euler angles.
     *
     * @param timestamp time in nanoseconds at which the measurement was made. Each measurement
     * wil be monotonically increasing using the same time base as
     * [android.os.SystemClock.elapsedRealtimeNanos].
     * @param headingAccuracy heading accuracy expressed in radians or null if not available.
     */
    private fun postProcessAttitudeAndNotify(timestamp: Long, headingAccuracy: Float?) {
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
        attitudeAvailableListener?.onAttitudeAvailable(
            this@AttitudeEstimator2,
            attitude,
            timestamp,
            headingAccuracy,
            displayRoll,
            displayPitch,
            displayYaw,
            c
        )
    }

    /**
     * Interface to notify when a new absolute or relative attitude measurement is available.
     */
    fun interface OnAttitudeAvailableListener {
        /**
         * Called when a new attitude measurement is available.
         *
         * @param estimator attitude estimator that raised this event.
         * @param attitude attitude expressed in NED coordinates.
         * @param timestamp time in nanoseconds at which the measurement was made. Each measurement
         * wil be monotonically increasing using the same time base as
         * [android.os.SystemClock.elapsedRealtimeNanos].
         * @param roll roll angle expressed in radians respect to NED coordinate system. Only
         * available if [estimateEulerAngles] is true.
         * @param pitch pitch angle expressed in radians respect to NED coordinate system. Only
         * available if [estimateEulerAngles] is true.
         * @param yaw yaw angle expressed in radians respect to NED coordinate system. Only
         * available if [estimateEulerAngles] is true.
         * @param coordinateTransformation coordinate transformation containing measured leveled
         * geomagnetic attitude. Only available if [estimateCoordinateTransformation] is true.
         */
        fun onAttitudeAvailable(
            estimator: AttitudeEstimator2,
            attitude: Quaternion,
            timestamp: Long,
            headingAccuracy: Float?,
            roll: Double?,
            pitch: Double?,
            yaw: Double?,
            coordinateTransformation: CoordinateTransformation?
        )
    }

    /**
     * Interface to notify when sensor (either accelerometer, gravity or magnetometer) accuracy
     * changes.
     */
    fun interface OnAccuracyChangedListener {
        /**
         * Called when accuracy changes.
         *
         * @param estimator leveled absolute attitude estimator that raised this event.
         * @param sensorType sensor that has changed its accuracy.
         * @param accuracy new accuracy.
         */
        fun onAccuracyChanged(
            estimator: AttitudeEstimator2,
            sensorType: SensorType,
            accuracy: SensorAccuracy?
        )
    }
}