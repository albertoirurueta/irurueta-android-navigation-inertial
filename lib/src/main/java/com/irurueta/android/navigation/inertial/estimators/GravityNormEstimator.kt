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
package com.irurueta.android.navigation.inertial.estimators

import android.content.Context
import com.irurueta.android.navigation.inertial.collectors.GravitySensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.SensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.navigation.inertial.calibration.TimeIntervalEstimator
import com.irurueta.navigation.inertial.calibration.noise.AccumulatedAccelerationMeasurementNoiseEstimator
import com.irurueta.units.Acceleration
import com.irurueta.units.Time
import com.irurueta.units.TimeConverter
import com.irurueta.units.TimeUnit

/**
 * Estimates gravity norm.
 * This estimator takes a given number of measurements during a given duration of time
 * to estimate gravity norm average, standard deviation and variance, as well as average time
 * interval between measurements.
 *
 * @property context Android context
 * @property sensorDelay Delay of sensor between samples.
 * @property stopMode Determines when this estimator will consider its estimation completed.
 * @property completedListener Listener to notify when estimation is complete.
 * @property unreliableListener Listener to notify when sensor becomes unreliable, and thus,
 * estimation must be discarded.
 */
class GravityNormEstimator private constructor(
    val context: Context,
    val sensorDelay: SensorDelay = SensorDelay.FASTEST,
    val stopMode: StopMode = StopMode.MAX_SAMPLES_OR_DURATION,
    var completedListener: OnEstimationCompletedListener? = null,
    var unreliableListener: OnUnreliableListener? = null
) {

    /**
     * Constructor.
     *
     * @param context Android context
     * @param sensorDelay Delay of sensor between samples.
     * @param maxSamples Maximum number of samples to take into account before completion. This is
     * only taken into account if using either [StopMode.MAX_SAMPLES_ONLY] or
     * [StopMode.MAX_SAMPLES_OR_DURATION].
     * @param maxDurationMillis Maximum duration expressed in milliseconds to take into account
     * before completion. This is only taken into account if using either
     * [StopMode.MAX_DURATION_ONLY] or [StopMode.MAX_SAMPLES_OR_DURATION].
     * @param stopMode Determines when this estimator will consider its estimation completed.
     * @property completedListener Listener to notify when estimation is complete.
     * @property unreliableListener Listener to notify when sensor becomes unreliable, and thus,
     * estimation must be discarded.
     */
    constructor(
        context: Context,
        sensorDelay: SensorDelay = SensorDelay.FASTEST,
        maxSamples: Int = DEFAULT_MAX_SAMPLES,
        maxDurationMillis: Long = DEFAULT_MAX_DURATION_MILLIS,
        stopMode: StopMode = StopMode.MAX_SAMPLES_OR_DURATION,
        completedListener: OnEstimationCompletedListener? = null,
        unreliableListener: OnUnreliableListener? = null
    ) : this(context, sensorDelay, stopMode, completedListener, unreliableListener) {

        require(maxSamples >= 0)
        require(maxDurationMillis >= 0)

        this.maxSamples = maxSamples
        this.maxDurationMillis = maxDurationMillis
    }

    /**
     * Gets maximum number of samples to take into account before completion. This is
     * only taken into account if using either [StopMode.MAX_SAMPLES_ONLY] or
     * [StopMode.MAX_SAMPLES_OR_DURATION].
     */
    var maxSamples: Int = DEFAULT_MAX_SAMPLES
        private set

    /**
     * Gets maximum duration expressed in milliseconds to take into account
     * before completion. This is only taken into account if using either
     * [StopMode.MAX_DURATION_ONLY] or [StopMode.MAX_SAMPLES_OR_DURATION].
     */
    var maxDurationMillis: Long = DEFAULT_MAX_DURATION_MILLIS
        private set

    /**
     * Internal noise estimator of a acceleration magnitude measurements.
     * This can be used to estimate statistics about gravity measurements.
     */
    private val noiseEstimator = AccumulatedAccelerationMeasurementNoiseEstimator()

    /**
     * Internal time estimator.
     * This can be used to estimate statistics about time intervals of gravity
     * measurements.
     */
    private val timeIntervalEstimator = TimeIntervalEstimator()

    /**
     * Listener to handle gravity measurements.
     */
    private val gravityMeasurementListener = object : GravitySensorCollector.OnMeasurementListener {
        override fun onMeasurement(
            gx: Float,
            gy: Float,
            gz: Float,
            g: Double,
            timestamp: Long,
            accuracy: SensorAccuracy?
        ) {
            if (accuracy == SensorAccuracy.UNRELIABLE) {
                resultUnreliable = true
            }

            if (numberOfProcessedMeasurements == 0) {
                initialTimestampNanos = timestamp
            }

            noiseEstimator.addMeasurement(g)

            val diff = timestamp - initialTimestampNanos
            val diffSeconds = TimeConverter.nanosecondToSecond(diff.toDouble())
            timeIntervalEstimator.addTimestamp(diffSeconds)

            endTimestampNanos = timestamp
            numberOfProcessedMeasurements++

            if (isComplete()) {
                // once time interval has been estimated, it it set into noise estimator so that
                // PSD values can be correctly estimated.
                noiseEstimator.timeInterval = timeIntervalEstimator.averageTimeInterval
                resultAvailable = true

                stop()

                completedListener?.onEstimatedCompleted(this@GravityNormEstimator)
            }
        }
    }

    /**
     * Listener to handle changes of accuracy in gravity sensor.
     */
    private val gravityAccuracyChangedListener =
        object : SensorCollector.OnAccuracyChangedListener {
            override fun onAccuracyChanged(accuracy: SensorAccuracy?) {
                if (accuracy == SensorAccuracy.UNRELIABLE) {
                    resultUnreliable = true
                    unreliableListener?.onUnreliable(this@GravityNormEstimator)
                }
            }
        }

    /**
     * Collector for gravity measurements.
     */
    private val gravityCollector = GravitySensorCollector(
        context,
        sensorDelay,
        gravityMeasurementListener,
        gravityAccuracyChangedListener
    )

    /**
     * Contains timestamp when estimator started.
     * This is based on [android.os.SystemClock.elapsedRealtimeNanos].
     */
    private var initialTimestampNanos: Long = 0

    /**
     * Contains timestamp of last measurement.
     * This is based on [android.os.SystemClock.elapsedRealtimeNanos].
     */
    private var endTimestampNanos: Long = 0

    /**
     * Indicates whether this estimator is already running.
     */
    var running = false
        private set

    /**
     * Number of gravity measurements that have been processed.
     */
    var numberOfProcessedMeasurements: Int = 0
        private set

    /**
     * Indicates whether estimated gravity norm and time interval between measurements are available
     * or not.
     */
    var resultAvailable: Boolean = false
        private set

    /**
     * Indicates whether estimated result is unreliable or not.
     */
    var resultUnreliable: Boolean = false

    /**
     * Gets estimated average gravity norm expressed in meters per squared second (m/s^2).
     * This is only available when estimation completes successfully and [resultAvailable] is
     * true.
     */
    val averageGravityNorm
        get() = if (resultAvailable) {
            noiseEstimator.avg
        } else {
            null
        }

    /**
     * Gets estimated average gravity norm as an acceleration measurement.
     * This is only available when estimation completes successfully and [resultAvailable] is
     * true.
     */
    val averageGravityNormAsAcceleration
        get() = if (resultAvailable) {
            noiseEstimator.avgAsMeasurement
        } else {
            null
        }

    /**
     * Gets estimated average gravity norm as an acceleration measurement.
     * This is only available when estimation completes successfully and [resultAvailable] is
     * true.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getAverageGravityNormAsAcceleration(result: Acceleration): Boolean {
        return if (resultAvailable) {
            noiseEstimator.getAvgAsMeasurement(result)
            true
        } else {
            false
        }
    }

    /**
     * Gets estimated variance of gravity norm expressed in (m^2/s^4).
     */
    val gravityNormVariance
        get() = if (resultAvailable) {
            noiseEstimator.variance
        } else {
            null
        }

    /**
     * Gets estimated standard deviation of gravity norm expressed in meters per squared second
     * (m/s^2).
     */
    val gravityNormStandardDeviation
        get() = if (resultAvailable) {
            noiseEstimator.standardDeviation
        } else {
            null
        }

    /**
     * Gets estimated standard deviation of gravity norm as an acceleration measurement.
     */
    val gravityNormStandardDeviationAsAcceleration
        get() = if (resultAvailable) {
            noiseEstimator.standardDeviationAsMeasurement
        } else {
            null
        }

    /**
     * Gets estimated standard deviation of gravity norm as an acceleration measurement.
     * This is only available when estimation completes successfully and [resultAvailable] is
     * true.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getGravityNormStandardDeviationAsAcceleration(result: Acceleration): Boolean {
        return if (resultAvailable) {
            noiseEstimator.getStandardDeviationAsMeasurement(result)
            true
        } else {
            false
        }
    }

    /**
     * Gets PSD (Power Spectral Density) of gravity norm noise expressed in (m^2 * s^-3).
     * This is only available when estimation completes successfully and average time interval
     * between measurements is reliably estimated.
     */
    val gravityPsd
        get() = if (resultAvailable) {
            noiseEstimator.psd
        } else {
            null
        }

    /**
     * Gets root PSD (Power Spectral Density) of gravity norm noise expressed in (m * s^-1.5).
     * This is only available when estimation completes successfully and average time interval
     * between measurements is reliably estimated.
     */
    val gravityRootPsd
        get() = if (resultAvailable) {
            noiseEstimator.rootPsd
        } else {
            null
        }

    /**
     * Gets average time interval between gravity measurements expressed in seconds (s).
     * This is only available when estimation completes successfully and [resultAvailable] is
     * true.
     */
    val averageTimeInterval
        get() = if (resultAvailable) {
            timeIntervalEstimator.averageTimeInterval
        } else {
            null
        }

    /**
     * Gets average time interval between gravity measurements.
     * This is only available when estimation completes successfully and [resultAvailable] is
     * true.
     */
    val averageTimeIntervalAsTime
        get() = if (resultAvailable) {
            timeIntervalEstimator.averageTimeIntervalAsTime
        } else {
            null
        }

    /**
     * Gets average time interval between gravity measurements.
     * This is only available when estimation completes successfully and [resultAvailable] is
     * true.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getAverageTimeIntervalAsTime(result: Time): Boolean {
        return if (resultAvailable) {
            timeIntervalEstimator.getAverageTimeIntervalAsTime(result)
            true
        } else {
            false
        }
    }

    /**
     * Gets estimated variance of time interval between gravity measurements expressed in squared
     * seconds (s^2).
     * This is only available when estimation completes successfully and [resultAvailable] is
     * true.
     */
    val timeIntervalVariance
        get() = if (resultAvailable) {
            timeIntervalEstimator.timeIntervalVariance
        } else {
            null
        }

    /**
     * Gets estimated standard deviation of time interval between gravity measurements expressed in
     * seconds (s).
     * This is only available when estimation completes successfully and [resultAvailable] is
     * true.
     */
    val timeIntervalStandardDeviation
        get() = if (resultAvailable) {
            timeIntervalEstimator.timeIntervalStandardDeviation
        } else {
            null
        }

    /**
     * Gets estimated standard deviation of time interval between gravity measurements.
     * This is only available when estimation completes successfully and [resultAvailable] is
     * true.
     */
    val timeIntervalStandardDeviationAsTime
        get() = if (resultAvailable) {
            timeIntervalEstimator.timeIntervalStandardDeviationAsTime
        } else {
            null
        }

    /**
     * Gets estimated standard deviation of time interval between gravity measurements.
     * This is only available when estimation completes successfully and [resultAvailable] is
     * true.
     */
    fun getTimeIntervalStandardDeviationAsTime(result: Time): Boolean {
        return if (resultAvailable) {
            timeIntervalEstimator.getTimeIntervalStandardDeviationAsTime(result)
            true
        } else {
            false
        }
    }

    /**
     * Gets amount of elapsed time to compute gravity norm estimation expressed in nanosconds (ns),
     * either if computation succeeds or not.
     */
    val elapsedTimeNanos
        get() = endTimestampNanos - initialTimestampNanos

    /**
     * Gets amount of elapsed time to compute gravity norm, either if computation succeeds or not.
     */
    val elapsedTime
        get() = Time(elapsedTimeNanos.toDouble(), TimeUnit.NANOSECOND)

    /**
     * Gets amount of time elapsed, either if computation succeeds or not.
     *
     * @param result instance where result will be stored.
     */
    fun getElapsedTime(result: Time) {
        result.unit = TimeUnit.NANOSECOND
        result.value = elapsedTimeNanos.toDouble()
    }

    /**
     * Starts collection of gravity norm measurements.
     *
     * @throws IllegalStateException if estimator is already running.
     */
    @Throws(IllegalStateException::class)
    fun start() {
        check(!running)

        reset()

        running = true
        gravityCollector.start()
    }

    /**
     * Stops collection of gravity norm measurements.
     */
    fun stop() {
        gravityCollector.stop()
        running = false
    }

    /**
     * Resets internal estimators.
     */
    private fun reset() {
        noiseEstimator.reset()
        noiseEstimator.timeInterval = 0.0

        timeIntervalEstimator.reset()
        if (stopMode == StopMode.MAX_DURATION_ONLY) {
            timeIntervalEstimator.totalSamples = Integer.MAX_VALUE
        } else {
            timeIntervalEstimator.totalSamples = maxSamples
        }

        initialTimestampNanos = 0
        endTimestampNanos = 0
        numberOfProcessedMeasurements = 0
        resultAvailable = false
    }

    /**
     * Indicates whether estimation is complete.
     *
     * @return true if estimation is complete, false otherwise.
     */
    private fun isComplete(): Boolean {
        return when (stopMode) {
            StopMode.MAX_DURATION_ONLY -> {
                val elapsedMillis = (elapsedTimeNanos / NANOS_TO_MILLIS)
                elapsedMillis >= maxDurationMillis
            }
            StopMode.MAX_SAMPLES_ONLY -> {
                numberOfProcessedMeasurements >= maxSamples
            }
            StopMode.MAX_SAMPLES_OR_DURATION -> {
                val elapsedMillis = (elapsedTimeNanos / NANOS_TO_MILLIS)
                (elapsedMillis >= maxDurationMillis
                        || numberOfProcessedMeasurements >= maxSamples)
            }
        }
    }

    /**
     * Mode to use to stop the estimator.
     */
    enum class StopMode {
        /**
         * Takes into account maximum number of samples to be processed only.
         */
        MAX_SAMPLES_ONLY,

        /**
         * Takes into account maximum duration to take measurements only.
         */
        MAX_DURATION_ONLY,

        /**
         * Takes into account maximum number of samples to be processed or maximum duration,
         * whichever comes first.
         */
        MAX_SAMPLES_OR_DURATION
    }

    companion object {
        /**
         * Typically sensors at maximum speed work at 100 Hz, so using this value should require
         * approximately 10 seconds to complete.
         * This is hardware dependant, since different sensors might achieve different maximum
         * speeds when sampling.
         */
        const val DEFAULT_MAX_SAMPLES = 1000

        /**
         * Maximum duration to keep sampling measurements until measurement is finished.
         */
        const val DEFAULT_MAX_DURATION_MILLIS = 20000L

        /**
         * Constant to convert nanoseconds to milliseconds.
         */
        private const val NANOS_TO_MILLIS = 1000000L
    }

    /**
     * Interface to notify when estimation completes.
     */
    interface OnEstimationCompletedListener {
        /**
         * Called when estimation completes.
         *
         * @param estimator estimator that generated the event.
         */
        fun onEstimatedCompleted(estimator: GravityNormEstimator)
    }

    /**
     * Interface to notify when gravity measurements become unreliable.
     */
    interface OnUnreliableListener {
        /**
         * Called when gravity measurements become unreliable.
         *
         * @param estimator estimator that generated the event.
         */
        fun onUnreliable(estimator: GravityNormEstimator)
    }
}