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
import com.irurueta.algebra.Matrix
import com.irurueta.android.navigation.inertial.collectors.AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.collectors.MagnetometerSensorType
import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.collectors.SensorType
import com.irurueta.android.navigation.inertial.collectors.interpolators.AccelerometerDirectSensorMeasurementInterpolator
import com.irurueta.android.navigation.inertial.collectors.interpolators.GyroscopeDirectSensorMeasurementInterpolator
import com.irurueta.android.navigation.inertial.collectors.interpolators.MagnetometerDirectSensorMeasurementInterpolator
import com.irurueta.android.navigation.inertial.processors.attitude.KalmanAbsoluteAttitudeProcessor5
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType
import com.irurueta.units.TimeConverter
import kotlin.math.abs

/**
 * Estimates absolute attitude.
 *
 * @property context Android context.
 * @property sensorDelay Delay of sensors between samples.
 * @property startOffsetEnabled indicates whether start offsets will be computed when first
 * measurement is received or not. True indicates that offset is computed, false assumes that offset
 * is null.
 * @property accelerometerSensorType One of the supported accelerometer sensor types.
 * @property gyroscopeSensorType One of the supported gyroscope sensor types.
 * @property estimateCoordinateTransformation true to estimate coordinate transformation, false
 * otherwise. If not needed, it can be disabled to improve performance and decrease cpu load.
 * @property estimateEulerAngles true to estimate euler angles, false otherwise. If not
 * needed, it can be disabled to improve performance and decrease cpu load.
 * @property gyroscopeNoisePsd Variance of the gyroscope output per Hz (or variance at 1Hz). This is
 * equivalent to the gyroscope PSD (Power Spectral Density) that can be obtained during calibration
 * or with noise estimators. If not provided
 * [KalmanAbsoluteAttitudeProcessor5.DEFAULT_GYROSCOPE_NOISE_PSD] will be used.
 * @property accelerometerNoiseStandardDeviation Accelerometer standard deviation expressed in
 * m/s^2. If not provided
 * [KalmanAbsoluteAttitudeProcessor5.DEFAULT_ACCELEROMETER_NOISE_STANDARD_DEVIATION] will be used.
 * @property minimumUnreliableDurationSeconds Minimum duration to keep unreliable accuracy when
 * Kalman filter numerical instability is detected. This value is expressed in seconds (s).
 * @property attitudeAvailableListener listener to notify when a new attitude measurement is
 * available.
 * @property accuracyChangedListener listener to notify changes in accuracy.
 * @property bufferFilledListener listener to notify that some buffer has been filled. This usually
 * happens when consumer of measurements cannot keep up with the rate at which measurements are
 * generated.
 */
class KalmanAbsoluteAttitudeEstimator(
    val context: Context,
    location: Location? = null,
    val sensorDelay: SensorDelay = SensorDelay.GAME,
    val startOffsetEnabled: Boolean = false,
    val accelerometerSensorType: AccelerometerSensorType =
        AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
    val gyroscopeSensorType: GyroscopeSensorType = GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
    val magnetometerSensorType: MagnetometerSensorType =
        MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
    val estimateCoordinateTransformation: Boolean = false,
    val estimateEulerAngles: Boolean = true,
    val estimateCovariances: Boolean = true,
    val gyroscopeNoisePsd: Double = KalmanAbsoluteAttitudeProcessor5.DEFAULT_GYROSCOPE_NOISE_PSD,
    val accelerometerNoiseStandardDeviation: Double =
        KalmanAbsoluteAttitudeProcessor5.DEFAULT_ACCELEROMETER_NOISE_STANDARD_DEVIATION,
    val magnetometerNoiseStandardDeviation: Double =
        KalmanAbsoluteAttitudeProcessor5.DEFAULT_MAGNETOMETER_NOISE_STANDARD_DEVIATION,
    val minimumUnreliableDurationSeconds: Double = DEFAULT_MINIMUM_UNRELIABLE_DURATION_SECONDS,
    var attitudeAvailableListener: OnAttitudeAvailableListener? = null,
    var accuracyChangedListener: OnAccuracyChangedListener? = null,
    var bufferFilledListener: OnBufferFilledListener? = null
) {
    /**
     * Gets flag indicating whether covariances of estimated attitude must be processed or not.
     */
    private val processCovariance: Boolean
        get() = estimateEulerAngles || estimateCovariances

    /**
     * Gets flag indicating whether covariance of Euler angles must be processed or not.
     */
    private val processEulerAnglesCovariance: Boolean
        get() = estimateEulerAngles && estimateCovariances

    /**
     * Indicates whether sensor estimation is unreliable or not.
     */
    private var unreliable = false

    /**
     * Timestamp of last processed measurement.
     */
    private var lastTimestamp: Long = -1

    /**
     * Timestamp when an unreliable measurement was found.
     */
    private var unreliableTimestamp: Long = -1

    /**
     * Processor to estimate absolute attitude using a Kalman filter.
     */
    private val processor = KalmanAbsoluteAttitudeProcessor5(
        location,
        gyroscopeNoisePsd = gyroscopeNoisePsd,
        accelerometerNoiseStandardDeviation = accelerometerNoiseStandardDeviation,
        magnetometerNoiseStandardDeviation = magnetometerNoiseStandardDeviation,
    )

    /**
     * Instance being reused to externally notify attitudes so that it does not have additional
     * effects if it is modified.
     */
    private val attitude = Quaternion()

    /**
     * Instance to be reused containing coordinate transformation in NED coordinates.
     */
    private val coordinateTransformation =
        CoordinateTransformation(FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME)

    /**
     * Copy of quaternion covariance to be reused.
     */
    private val quaternionCovariance = Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS)

    /**
     * Copy of Euler angles covariance to be reused.
     */
    private val eulerAnglesCovariance = Matrix(Quaternion.N_ANGLES, Quaternion.N_ANGLES)

    /**
     * Current sensor accuracy.
     */
    private var sensorAccuracy: SensorAccuracy? = null

    /**
     * Internal syncer to collect and sync accelerometer, gyroscope and magnetometer measurements.
     */
    private val syncer = AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer(
        context,
        accelerometerSensorType,
        gyroscopeSensorType,
        magnetometerSensorType,
        sensorDelay,
        sensorDelay,
        sensorDelay,
        accelerometerStartOffsetEnabled = startOffsetEnabled,
        gyroscopeStartOffsetEnabled = startOffsetEnabled,
        magnetometerStartOffsetEnabled = startOffsetEnabled,
        stopWhenFilledBuffer = false,
        staleDetectionEnabled = true,
        accelerometerInterpolator = AccelerometerDirectSensorMeasurementInterpolator(),
        gyroscopeInterpolator = GyroscopeDirectSensorMeasurementInterpolator(),
        magnetometerInterpolator = MagnetometerDirectSensorMeasurementInterpolator(),
        accuracyChangedListener = { _, sensorType, accuracy ->
            sensorAccuracy = accuracy
            accuracyChangedListener?.onAccuracyChanged(
                this@KalmanAbsoluteAttitudeEstimator,
                sensorType,
                accuracy
            )
        },
        bufferFilledListener = { _, sensorType ->
            bufferFilledListener?.onBufferFilled(this@KalmanAbsoluteAttitudeEstimator, sensorType)
        },
        syncedMeasurementListener = { _, measurement ->
            if (processor.process(measurement)) {
                processor.bodyAttitude.copyTo(attitude)
                postProcessAttitudeAndNotify(measurement.timestamp)
            }
        }
    )

    /**
     * Indicates whether this estimator is running or not.
     */
    var running: Boolean = false
        private set

    /**
     * Gets or sets current device location.
     */
    var location: Location? = location
        set(value) {
            field = value
            processor.location = value
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

        running = syncer.start(startTimestamp)
        return running
    }

    /**
     * Stops this estimator.
     */
    fun stop() {
        syncer.stop()
        reset()
        running = false
    }

    /**
     * Resets estimator state.
     */
    private fun reset() {
        lastTimestamp = -1L
        unreliableTimestamp = -1L
        processor.reset()
    }

    /**
     * Processes current attitude and computes (if needed) a coordinate transformation, Euler
     * angles or covariance.
     *
     * @param timestamp time in nanoseconds at which the measurement was made. Each measurement
     * wil be monotonically increasing using the same time base as
     * [android.os.SystemClock.elapsedRealtimeNanos].
     */
    private fun postProcessAttitudeAndNotify(timestamp: Long) {
        lastTimestamp = timestamp

        val c: CoordinateTransformation? =
            if (estimateCoordinateTransformation) {
                coordinateTransformation.fromRotation(attitude)
                coordinateTransformation
            } else {
                null
            }

        val roll: Double?
        val pitch: Double?
        val yaw: Double?
        val eulerAngles = processor.eulerAngles
        if (eulerAngles != null) {
            roll = eulerAngles[0]
            pitch = eulerAngles[1]
            yaw = eulerAngles[2]
        } else {
            roll = null
            pitch = null
            yaw = null
        }

        val nedQuaternionCov = processor.nedAttitudeCovariance
        // TODO: fix val nedQuaternionCov = null
        if (nedQuaternionCov != null) {
            quaternionCovariance.copyFrom(nedQuaternionCov)
        }

        val eulerAnglesCov = processor.eulerAnglesCovariance
        if (eulerAnglesCov != null) {
            eulerAnglesCovariance.copyFrom(eulerAnglesCov)
        }

        //notify
        attitudeAvailableListener?.onAttitudeAvailable(
            this,
            attitude,
            timestamp,
            roll,
            pitch,
            yaw,
            c,
            if (nedQuaternionCov != null) quaternionCovariance else null,
            if (eulerAnglesCov != null) eulerAnglesCovariance else null
        )

        if (unreliable) {
            val durationSeconds =
                TimeConverter.nanosecondToSecond(abs(timestamp - unreliableTimestamp).toDouble())
            if (durationSeconds >= minimumUnreliableDurationSeconds) {
                accuracyChangedListener?.onAccuracyChanged(
                    this,
                    SensorType.RELATIVE_ATTITUDE,
                    sensorAccuracy
                )
                unreliable = false
            }
        }
    }

    companion object {
        /**
         * Default minimum duration to keep unreliable accuracy when Kalman filter numerical
         * instability is detected. This values is expressed in seconds.
         */
        const val DEFAULT_MINIMUM_UNRELIABLE_DURATION_SECONDS = 1.0
    }

    /**
     * Interface to notify when a new attitude measurement is available.
     */
    fun interface OnAttitudeAvailableListener {

        /**
         * Called when a new attitude measurement is available.
         *
         * @param estimator estimator that raised this event.
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
         * @param coordinateTransformation coordinate transformation containing measured leveling
         * attitude. Only available if [estimateCoordinateTransformation] is true.
         * @param quaternionCovariance covariance of estimated quaternion attitude expressed as a
         * 4x4 matrix. Only available if [estimateCovariances] is true.
         * @param eulerAnglesCovariance covariance of estimated Euler angles corresponding to
         * current attitude in NED coordinates, as a 3x3 matrix. Only available if
         * [estimateEulerAngles] and [estimateCovariances] are true.
         */
        fun onAttitudeAvailable(
            estimator: KalmanAbsoluteAttitudeEstimator,
            attitude: Quaternion,
            timestamp: Long,
            roll: Double?,
            pitch: Double?,
            yaw: Double?,
            coordinateTransformation: CoordinateTransformation?,
            quaternionCovariance: Matrix?,
            eulerAnglesCovariance: Matrix?
        )
    }

    /**
     * Interface to notify when sensor (either accelerometer or gyroscope) accuracy changes.
     */
    fun interface OnAccuracyChangedListener {
        /**
         * Called when accuracy changes.
         *
         * @param estimator estimator that raised this event.
         * @param sensorType sensor that has changed its accuracy.
         * @param accuracy new accuracy.
         */
        fun onAccuracyChanged(
            estimator: KalmanAbsoluteAttitudeEstimator,
            sensorType: SensorType,
            accuracy: SensorAccuracy?
        )
    }

    /**
     * Interface to notify when a buffer gets completely filled.
     * When buffers get filled, internal collectors will continue collection at the expense of
     * loosing old data. Consumers of this listener should decide what to do at this point (which
     * might require stopping this estimator).
     */
    fun interface OnBufferFilledListener {
        /**
         * Called when any buffer gets completely filled.
         *
         * @param estimator estimator that raised this event.
         * @param sensorType sensor that got its buffer filled.
         */
        fun onBufferFilled(estimator: KalmanAbsoluteAttitudeEstimator, sensorType: SensorType)
    }
}