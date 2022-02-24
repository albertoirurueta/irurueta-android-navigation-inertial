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
package com.irurueta.android.navigation.inertial.calibration.intervals.measurements

import android.content.Context
import com.irurueta.android.navigation.inertial.calibration.intervals.ErrorReason
import com.irurueta.android.navigation.inertial.calibration.intervals.Status
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.SensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.navigation.inertial.calibration.TimeIntervalEstimator
import com.irurueta.navigation.inertial.calibration.TimedBodyKinematics
import com.irurueta.navigation.inertial.calibration.intervals.TriadStaticIntervalDetector
import com.irurueta.units.Acceleration
import com.irurueta.units.Time
import com.irurueta.units.TimeConverter

/**
 * @param I type of input data to be processed.
 */
abstract class CalibrationMeasurementGenerator<I>(
    val context: Context,
    val accelerometerSensorType: AccelerometerSensorCollector.SensorType =
        AccelerometerSensorCollector.SensorType.ACCELEROMETER,
    val accelerometerSensorDelay: SensorDelay = SensorDelay.FASTEST,
    var accuracyChangedListener: SensorCollector.OnAccuracyChangedListener? = null
) {

    /**
     * Internal time estimator for accelerometer samples used for static/dynamic interval detection.
     * This is used to estimate statistics about time intervals of accelerometer measurements.
     */
    protected val accelerometerTimeIntervalEstimator = TimeIntervalEstimator()

    /**
     * Indicates whether generator successfully completed initialization.
     */
    protected var initialized: Boolean = false

    /**
     * Timestamp when accelerometer started.
     */
    protected var initialAccelerometerTimestamp: Long = 0L

    /**
     * Indicates whether a sensor has become unreliable, and thus
     * the interval detector is considered to be in [Status.FAILED].
     */
    protected var unreliable = false


    /**
     * Gets or sets minimum number of samples required in a static interval to be taken into
     * account. Smaller static intervals will be discarded.
     *
     * @throws IllegalArgumentException if provided value is less than 2.
     * @throws IllegalStateException if generator is currently running.
     */
    abstract var minStaticSamples: Int

    /**
     * Gets or sets maximum number of samples allowed in dynamic intervals.
     * Dynamic intervals exceeding this value are discarded.
     *
     * @throws IllegalArgumentException if provided value is less than 2.
     * @throws IllegalStateException if generator is currently running.
     */
    abstract var maxDynamicSamples: Int

    /**
     * Gets or sets length of number of samples to keep within the window being processed to
     * determine instantaneous sensor noise level. Window size must always be larger than
     * allowed minimum value, which is 2 and must have an odd value.
     *
     * @throws IllegalArgumentException if provided value is not valid.
     * @throws IllegalStateException if generator is currently running.
     */
    abstract var windowSize: Int

    /**
     * Gets or sets number of samples to be processed initially while keeping the sensor static in
     * order to find the base noise level when device is static.
     *
     * @throws IllegalArgumentException if provided value is less than
     * [TriadStaticIntervalDetector.MINIMUM_INITIAL_STATIC_SAMPLES].
     * @throws IllegalStateException if generator is currently running.
     */
    abstract var initialStaticSamples: Int

    /**
     * Gets or sets factor to be applied to detected base noise level in order to determine
     * threshold for static/dynamic period changes. This factor is unit-less.
     *
     * @throws IllegalArgumentException if provided value is zero or negative
     * @throws IllegalStateException if generator is currently running.
     */
    abstract var thresholdFactor: Double

    /**
     * Gets or sets factor to determine that a sudden movement has occurred during initialization if
     * instantaneous noise level exceeds accumulated noise level by this factor amount. This factor
     * is unit-less.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if generator is currently running
     */
    abstract var instantaneousNoiseLevelFactor: Double

    /**
     * Gets or sets overall absolute threshold to determine whether there has been excessive motion
     * during the whole initialization phase. Failure will be detected if estimated base noise level
     * exceeds this threshold when initialization completes. This threshold is expressed in meters
     * per squared second (m/s^2).
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if detector is currently running.
     */
    abstract var baseNoiseLevelAbsoluteThreshold: Double

    /**
     * Gets or sets overall absolute threshold to determine whether there has been excessive motion
     * during the whole initialization phase. Failure will be detected if estimated base noise level
     * exceeds this threshold when initialization completes.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     * @throws IllegalStateException if detector is currently running.
     */
    abstract var baseNoiseLevelAbsoluteThresholdAsMeasurement: Acceleration

    /**
     * Gets overall absolute threshold to determine whether there has been excessive motion during
     * the whole initialization phase. Failure will be detected if estimated base noise level
     * exceeds this threshold when initialization completes.
     *
     * @param result instance where result will be stored.
     */
    abstract fun getBaseNoiseLevelAbsoluteThresholdAsMeasurement(result: Acceleration)

    /**
     * Gets accelerometer measurement base noise level that has been detected during initialization
     * expressed in meters per squared second (m/s^2).
     * This is only available once detector completes initialization.
     */
    abstract val accelerometerBaseNoiseLevel: Double?

    /**
     * Gets sensor measurement base noise level that has been detected during initialization.
     * This is only available once detector completes initialization.
     */
    abstract val accelerometerBaseNoiseLevelAsMeasurement: Acceleration?

    /**
     * Gets sensor measurement base noise level that has been detected during initialization.
     * This is only available once detector completes initialization.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    abstract fun getAccelerometerBaseNoiseLevelAsMeasurement(result: Acceleration): Boolean

    /**
     * Gets measurement base noise level PSD (Power Spectral Density) expressed in (m^2 * s^-3).
     */
    abstract val accelerometerBaseNoiseLevelPsd: Double?

    /**
     * Gets measurement base noise level root PSD (Power Spectral Density) expressed
     * in (m * s^-1.5).
     * This is only available once detector completes initialization.
     */
    abstract val accelerometerBaseNoiseLevelRootPsd: Double?

    /**
     * Gets estimated threshold to determine static/dynamic period changes expressed in meters per
     * squared second (m/s^2).
     * This is only available once detector completes initialization.
     */
    abstract val threshold: Double?

    /**
     * Gets estimated threshold to determine static/dynamic period changes.
     * This is only available once detector completes initialization.
     */
    abstract val thresholdAsMeasurement: Acceleration?

    /**
     * Gets estimated threshold to determine static/dynamic period changes.
     * This is only available once detector completes initialization.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    abstract fun getThresholdAsMeasurement(result: Acceleration): Boolean

    /**
     * Gets number of samples that have been processed in a static period so far.
     */
    abstract val processedStaticSamples: Int

    /**
     * Gets number of samples that have been processed in a dynamic period so far.
     */
    abstract val processedDynamicSamples: Int

    /**
     * Indicates whether last static interval must be skipped.
     */
    abstract val isStaticIntervalSkipped: Boolean

    /**
     * Indicates whether last dynamic interval must be skipped.
     */
    abstract val isDynamicIntervalSkipped: Boolean

    /**
     * Gets average time interval between accelerometer samples expressed in seconds (s).
     * This is only available once this generator completes initialization.
     */
    val accelerometerAverageTimeInterval
        get() = if (initialized) {
            accelerometerTimeIntervalEstimator.averageTimeInterval
        } else {
            null
        }

    /**
     * Gets average time interval between accelerometer samples.
     * This is only available once this generator completes initialization.
     */
    val accelerometerAverageTimeIntervalAsTime
        get() = if (initialized) {
            accelerometerTimeIntervalEstimator.averageTimeIntervalAsTime
        } else {
            null
        }

    /**
     * Gets average time interval between accelerometer measurements.
     * This is only available once detector completes initialization.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getAccelerometerAverageTimeIntervalAsTime(result: Time): Boolean {
        return if (initialized) {
            accelerometerTimeIntervalEstimator.getAverageTimeIntervalAsTime(result)
            true
        } else {
            false
        }
    }

    /**
     * Gets estimated variance of time interval between accelerometer measurements expressed in
     * squared seconds (s^2).
     * This is only available once detector completes initialization.
     */
    val accelerometerTimeIntervalVariance
        get() = if (initialized) {
            accelerometerTimeIntervalEstimator.timeIntervalVariance
        } else {
            null
        }

    /**
     * Gets estimated standard deviation of time interval between accelerometer measurements
     * expressed in seconds (s).
     * This is only available once detector completes initialization.
     */
    val accelerometerTimeIntervalStandardDeviation
        get() = if (initialized) {
            accelerometerTimeIntervalEstimator.timeIntervalStandardDeviation
        } else {
            null
        }

    /**
     * Gets estimated standard deviation of time interval between accelerometer measurements.
     * This is only available once detector completes initialization.
     */
    val accelerometerTimeIntervalStandardDeviationAsTime
        get() = if (initialized) {
            accelerometerTimeIntervalEstimator.timeIntervalStandardDeviationAsTime
        } else {
            null
        }

    /**
     * Gets estimated standard deviation of time interval between accelerometer measurements.
     * This is only available once detector completes initialization.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getTimeIntervalStandardDeviationAsTime(result: Time): Boolean {
        return if (initialized) {
            accelerometerTimeIntervalEstimator.getTimeIntervalStandardDeviationAsTime(result)
            true
        } else {
            false
        }
    }

    /**
     * Number of accelerometer measurements that have been processed.
     */
    var numberOfProcessedAccelerometerMeasurements: Int = 0
        protected set

    /**
     * Indicates whether this generator is already running.
     */
    var running = false
        protected set

    /**
     * Gets status of measurement generator.
     * Initially the generator will be idle.
     * Once it starts, it will start the initialization phase, and once
     * initialization is complete, it will switch between static or dynamic interval
     * until generator is stopped or an error occurs.
     */
    abstract val status: Status

    /**
     * Starts collection of sensor measurements.
     *
     * @throws IllegalStateException if generator is already running or sensor is not available.
     */
    @Throws(IllegalStateException::class)
    abstract fun start()

    /**
     * Stops collection of sensor measurements.
     */
    abstract fun stop()

    /**
     * Maps error reason to an [ErrorReason]
     *
     * @param reason reason to map from.
     * @return mapped reason.
     */
    protected fun mapErrorReason(reason: TriadStaticIntervalDetector.ErrorReason): ErrorReason {
        return ErrorReason.mapErrorReason(reason, unreliable)
    }

    /**
     * Listener to detect when accuracy of sensor changes.
     * When sensor becomes unreliable, an error is notified.
     */
    @Suppress("UNCHECKED_CAST")
    protected val collectorAccuracyChangedListener =
        SensorCollector.OnAccuracyChangedListener { accuracy ->
            if (accuracy == SensorAccuracy.UNRELIABLE) {
                stop()
                unreliable = true
                notifyUnreliableSensor()
            }

            accuracyChangedListener?.onAccuracyChanged(accuracy)
        }

    /**
     * Notifies that sensor has become unreliable.
     */
    protected abstract fun notifyUnreliableSensor()

    /**
     * Sample used by internal measurement generator which combines information from accelerometer
     * and possibly other sensors.
     */
    protected abstract val sample: I

    /**
     * Internal listener for accelerometer sensor collector.
     * Handles measurements collected by the accelerometer sensor so that they are processed by
     * the internal measurement generator.
     */
    private val accelerometerCollectorMeasurementListener =
        AccelerometerSensorCollector.OnMeasurementListener { ax, ay, az, bx, by, bz, timestamp, accuracy ->
            val status = status
            var diffSeconds = 0.0
            if (status == Status.INITIALIZING) {
                // during initialization phase, also estimate time interval duration.
                if (numberOfProcessedAccelerometerMeasurements > 0) {
                    val diff = timestamp - initialAccelerometerTimestamp
                    diffSeconds = TimeConverter.nanosecondToSecond(diff.toDouble())
                    accelerometerTimeIntervalEstimator.addTimestamp(diffSeconds)
                } else {
                    initialAccelerometerTimestamp = timestamp
                }
            }

            processSample(ax, ay, az, diffSeconds, sample)
            processSampleInInternalGenerator()
            numberOfProcessedAccelerometerMeasurements++

            if (status == Status.INITIALIZATION_COMPLETED) {
                // once initialized, set time interval into internal detector
                updateTimeIntervalOfInternalGenerator()
                initialized = true
            }

            notifyAccelerometerMeasurement(ax, ay, az, bx, by, bz, timestamp, accuracy)
        }

    /**
     * Processes sample in internal measurements generator.
     */
    protected abstract fun processSampleInInternalGenerator()

    /**
     * Updates time interval of internal generator once the interval detector has been initialized.
     */
    protected abstract fun updateTimeIntervalOfInternalGenerator()

    /**
     * Notifies that an accelerometer measurement has been received.
     *
     * @param ax acceleration on device x-axis expressed in meters per squared second (m/s^2).
     * @param ay acceleration on device y-axis expressed in meters per squared second (m/s^2).
     * @param az acceleration on device z-axis expressed in meters per squared second (m/s^2).
     * @param bx bias on device x-axis expressed in meters per squared second (m/s^2). Only
     * available when using [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED].
     * If available, this value remains constant with calibrated bias value.
     * @param by bias on device y-axis expressed in meters per squared second (m/s^2). Only
     * available when using [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED].
     * If available, this value remains constant with calibrated bias value.
     * @param bz bias on device z-axis expressed in meters per squared second (m/s^2). Only
     * available when using [AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED].
     * If available, this value remains constant with calibrated bias value.
     * @param timestamp time in nanoseconds at which the measurement was made. Each measurement
     * will be monotonically increasing using the same time base as
     * [android.os.SystemClock.elapsedRealtimeNanos].
     * @param accuracy accelerometer sensor accuracy.
     */
    protected abstract fun notifyAccelerometerMeasurement(
        ax: Float,
        ay: Float,
        az: Float,
        bx: Float?,
        by: Float?,
        bz: Float?,
        timestamp: Long,
        accuracy: SensorAccuracy?
    )

    /**
     * Accelerometer sensor collector.
     * Collects accelerometer measurements.
     */
    protected val accelerometerCollector = AccelerometerSensorCollector(
        context,
        accelerometerSensorType,
        accelerometerSensorDelay,
        accelerometerCollectorMeasurementListener,
        collectorAccuracyChangedListener
    )

    /**
     * Gets accelerometer sensor being used to obtain measurements and detect static/dynamic
     * intervals, or null if not available.
     * This can be used to obtain additional information about the sensor.
     */
    val accelerometerSensor
        get() = accelerometerCollector.sensor

    /**
     * Processes an accelerometer measurement to generate an instance of type [TimedBodyKinematics]
     * to be used by the internal measurement generator.
     *
     * @param ax acceleration on device x-axis expressed in meters per squared second (m/s^2).
     * @param ay acceleration on device y-axis expressed in meters per squared second (m/s^2).
     * @param az acceleration on device z-axis expressed in meters per squared second (m/s^2).
     * @param diffSeconds elapsed seconds since accelerometer started.
     * @param result instance where processed sample result will be stored.
     */
    protected abstract fun processSample(
        ax: Float,
        ay: Float,
        az: Float,
        diffSeconds: Double,
        result: I
    )
}