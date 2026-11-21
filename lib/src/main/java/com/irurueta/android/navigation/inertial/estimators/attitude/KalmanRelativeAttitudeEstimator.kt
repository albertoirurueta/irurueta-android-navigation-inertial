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
import android.location.Location
import android.os.SystemClock
import com.irurueta.algebra.Matrix
import com.irurueta.android.navigation.inertial.collectors.AccelerometerAndGyroscopeSyncedSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorType
import com.irurueta.android.navigation.inertial.processors.attitude.KalmanRelativeAttitudeProcessor
import com.irurueta.android.navigation.inertial.processors.filters.AveragingFilter
import com.irurueta.android.navigation.inertial.processors.filters.LowPassAveragingFilter
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegrator
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegratorType
import com.irurueta.units.Acceleration
import com.irurueta.units.AccelerationUnit
import com.irurueta.units.TimeConverter
import kotlin.math.abs

/**
 * Estimates a leveled relative attitude, where only yaw Euler angle is relative to the start
 * instant of this estimator.
 * Roll and pitch Euler angles are leveled using accelerometer sensor.
 *
 * @property context Android context.
 * @property sensorDelay Delay of sensors between samples.
 * @property accelerometerSensorType One of the supported accelerometer sensor types.
 * @property gyroscopeSensorType One of the supported gyroscope sensor types.
 * @property averagingFilter averaging or low-pass-band filter to be used for accelerometer
 * measurements to estimate gravity.
 * @property location current device location. This is used to estimate expected gravity norm.
 * @property adjustGravityNorm when enabled, estimated gravity norm is adjusted to expected gravity
 * when location is provided (otherwise Earth's default gravity at sea-level is used).
 * @property estimateCoordinateTransformation true to estimate coordinate transformation, false
 * otherwise. If not needed, it can be disabled to improve performance and decrease cpu load.
 * @property estimateEulerAngles true to estimate euler angles, false otherwise. If not
 * needed, it can be disabled to improve performance and decrease cpu load.
 * @property estimateCovariances Indicates whether attitude covariances must be processed or not.
 * @property gyroscopeNoisePsd Variance of the gyroscope output per Hz (or variance at 1Hz). This is
 * equivalent to the gyroscope PSD (Power Spectral Density) that can be obtained during calibration
 * or with noise estimators. If not provided
 * [KalmanRelativeAttitudeProcessor.DEFAULT_GYROSCOPE_NOISE_PSD] will be used.
 * @property accelerometerNoiseStandardDeviation Accelerometer standard deviation expressed in
 * m/s^2. If not provided
 * [KalmanRelativeAttitudeProcessor.DEFAULT_ACCELEROMETER_NOISE_STANDARD_DEVIATION] will be used.
 * @property quaternionStepIntegratorType type of quaternion step integrator to be used for
 * gyroscope integration.
 * @property freeFallThreshold Threshold to consider that device is in free fall. When device is in
 * free fall, accelerometer measurements are considered unreliable and are ignored (only gyroscope
 * predictions are made).
 * @property symmetricThreshold Threshold to determine whether covariance matrices are positive
 * definite or not.
 * @property minimumUnreliableDurationSeconds Minimum duration to keep unreliable accuracy when
 * Kalman filter numerical instability is detected. This value is expressed in seconds (s).
 * @property attitudeAvailableListener listener to notify when a new attitude measurement is
 * available.
 * @property accuracyChangedListener listener to notify changes in accuracy.
 */
class KalmanRelativeAttitudeEstimator(
    val context: Context,
    val sensorDelay: SensorDelay = SensorDelay.FASTEST,
    val accelerometerSensorType: AccelerometerSensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
    val gyroscopeSensorType: GyroscopeSensorType = GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
    val averagingFilter: AveragingFilter<AccelerationUnit, Acceleration, AccelerationTriad> = LowPassAveragingFilter(),
    location: Location? = null,
    adjustGravityNorm: Boolean = true,
    val estimateCoordinateTransformation: Boolean = false,
    val estimateEulerAngles: Boolean = true,
    val estimateCovariances: Boolean = true,
    val gyroscopeNoisePsd: Double = KalmanRelativeAttitudeProcessor.DEFAULT_GYROSCOPE_NOISE_PSD,
    val accelerometerNoiseStandardDeviation: Double = KalmanRelativeAttitudeProcessor.DEFAULT_ACCELEROMETER_NOISE_STANDARD_DEVIATION,
    val quaternionStepIntegratorType: QuaternionStepIntegratorType = QuaternionStepIntegrator.DEFAULT_TYPE,
    val freeFallThreshold: Double = KalmanRelativeAttitudeProcessor.DEFAULT_FREE_FALL_THRESHOLD,
    val symmetricThreshold: Double = KalmanRelativeAttitudeProcessor.DEFAULT_SYMMETRIC_THRESHOLD,
    val minimumUnreliableDurationSeconds: Double = DEFAULT_MINIMUM_UNRELIABLE_DURATION_SECONDS,
    var attitudeAvailableListener: OnAttitudeAvailableListener? = null,
    var accuracyChangedListener: OnAccuracyChangedListener? = null,
) {
    /**
     * Gets or sets device location. This is used to estimate expected gravity norm.
     */
    var location: Location?
        get() = processor.location
        set(value) {
            processor.location = value
        }

    /**
     * Gets or sets flag indicating whether estimated gravity norm must be adjusted to expected
     * gravity when location is provided (otherwise Earth's default gravity at sea-level is used).
     */
    var adjustGravityNorm: Boolean
        get() = processor.adjustGravityNorm
        set(value) {
            processor.adjustGravityNorm = value
        }

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
     * Processor to estimate relative attitude using a Kalman filter.
     */
    private val processor = KalmanRelativeAttitudeProcessor(
        averagingFilter = averagingFilter,
        location = location,
        adjustGravityNorm = adjustGravityNorm,
        computeEulerAngles = estimateEulerAngles,
        computeCovariances = processCovariance,
        computeQuaternionCovariance = true,
        computeEulerAnglesCovariance = processEulerAnglesCovariance,
        computeAccelerometerCovariance = false,
        computeRotationAxisCovariance = false,
        gyroscopeNoisePsd = gyroscopeNoisePsd,
        accelerometerNoiseStandardDeviation = accelerometerNoiseStandardDeviation,
        quaternionStepIntegratorType = quaternionStepIntegratorType,
        freeFallThreshold = freeFallThreshold,
        symmetricThreshold = symmetricThreshold,
        numericalInstabilitiesListener = {
            unreliable = true
            unreliableTimestamp = lastTimestamp
            accuracyChangedListener?.onAccuracyChanged(
                this@KalmanRelativeAttitudeEstimator,
                SensorType.RELATIVE_ATTITUDE,
                SensorAccuracy.UNRELIABLE
            )
        }
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

    private val collector = AccelerometerAndGyroscopeSyncedSensorCollector(
        context,
        accelerometerSensorType = accelerometerSensorType,
        gyroscopeSensorType = gyroscopeSensorType,
        accelerometerSensorDelay = sensorDelay,
        gyroscopeSensorDelay = sensorDelay,
        measurementListener = { _, measurement ->
            if (processor.process(measurement)) {
                processor.nedAttitude.copyTo(attitude)
                postProcessAttitudeAndNotify(measurement.timestamp)
            }
        },
        accuracyChangedListener = { _, sensorType, accuracy ->
            sensorAccuracy = accuracy
            accuracyChangedListener?.onAccuracyChanged(
                this@KalmanRelativeAttitudeEstimator,
                sensorType,
                accuracy
            )
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

        reset()
        running = collector.start(startTimestamp)
        return running
    }

    /**
     * Stops this estimator.
     */
    fun stop() {
        collector.stop()
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
     * [SystemClock.elapsedRealtimeNanos].
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
                unreliable = false
                accuracyChangedListener?.onAccuracyChanged(
                    this,
                    SensorType.RELATIVE_ATTITUDE,
                    sensorAccuracy
                )
            }
        }
    }

    // Initialization
    init {
        this.location = location
        this.adjustGravityNorm = adjustGravityNorm
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
         * [SystemClock.elapsedRealtimeNanos].
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
            estimator: KalmanRelativeAttitudeEstimator,
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
            estimator: KalmanRelativeAttitudeEstimator,
            sensorType: SensorType,
            accuracy: SensorAccuracy?
        )
    }
}