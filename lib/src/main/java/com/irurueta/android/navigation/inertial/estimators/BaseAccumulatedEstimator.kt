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

import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.SensorCollector
import com.irurueta.navigation.inertial.calibration.TimeIntervalEstimator
import com.irurueta.units.Time
import com.irurueta.units.TimeConverter
import com.irurueta.units.TimeUnit

/**
 * Base implementation containing common logic for all estimators.
 *
 * @property stopMode Determines when this estimator will consider its estimation completed.
 */
abstract class BaseAccumulatedEstimator private constructor(
    val stopMode: StopMode = StopMode.MAX_SAMPLES_OR_DURATION
) {

    /**
     * Constructor.
     *
     * @param maxSamples Maximum number of samples to take into account before completion. This is
     * only taken into account if using either [StopMode.MAX_SAMPLES_ONLY] or
     * [StopMode.MAX_SAMPLES_OR_DURATION].
     * @param maxDurationMillis Maximum duration expressed in milliseconds to take into account
     * before completion. This is only taken into account if using either
     * [StopMode.MAX_DURATION_ONLY] or [StopMode.MAX_SAMPLES_OR_DURATION].
     * @param stopMode Determines when this estimator will consider its estimation completed.
     */
    constructor(
        maxSamples: Int = DEFAULT_MAX_SAMPLES,
        maxDurationMillis: Long = DEFAULT_MAX_DURATION_MILLIS,
        stopMode: StopMode = StopMode.MAX_SAMPLES_OR_DURATION,
    ) : this(stopMode) {

        require(maxSamples >= 0)
        require(maxDurationMillis >= 0)

        this.maxSamples = maxSamples
        this.maxDurationMillis = maxDurationMillis
    }

    /**
     * Internal time estimator.
     * This can be used to estimate statistics about time intervals of measurements.
     */
    protected val timeIntervalEstimator = TimeIntervalEstimator()

    /**
     * Contains timestamp when estimator started.
     * This is based on [android.os.SystemClock.elapsedRealtimeNanos].
     */
    protected var initialTimestampNanos: Long = 0

    /**
     * Contains timestamp of last measurement.
     * This is based on [android.os.SystemClock.elapsedRealtimeNanos].
     */
    private var endTimestampNanos: Long = 0

    /**
     * Listener to handle changes of accuracy in gravity sensor.
     */
    protected val accuracyChangedListener =
        object : SensorCollector.OnAccuracyChangedListener {
            override fun onAccuracyChanged(accuracy: SensorAccuracy?) {
                if (accuracy == SensorAccuracy.UNRELIABLE) {
                    resultUnreliable = true
                    notifyUnreliableListener()
                }
            }
        }

    /**
     * Number of measurements that have been processed.
     */
    var numberOfProcessedMeasurements: Int = 0
        protected set

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
     * Indicates whether estimated average and time interval between measurements are available
     * or not.
     */
    var resultAvailable: Boolean = false
        protected set

    /**
     * Indicates whether estimated result is unreliable or not.
     */
    var resultUnreliable: Boolean = false
        protected set

    /**
     * Gets average time interval between measurements expressed in seconds (s).
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     */
    val averageTimeInterval
        get() = if (resultAvailable) {
            timeIntervalEstimator.averageTimeInterval
        } else {
            null
        }

    /**
     * Gets average time interval between measurements.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     */
    val averageTimeIntervalAsTime
        get() = if (resultAvailable) {
            timeIntervalEstimator.averageTimeIntervalAsTime
        } else {
            null
        }

    /**
     * Gets average time interval between measurements.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
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
     * Gets estimated variance of time interval between measurements expressed in squared
     * seconds (s^2).
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     */
    val timeIntervalVariance
        get() = if (resultAvailable) {
            timeIntervalEstimator.timeIntervalVariance
        } else {
            null
        }

    /**
     * Gets estimated standard deviation of time interval between measurements expressed in
     * seconds (s).
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     */
    val timeIntervalStandardDeviation
        get() = if (resultAvailable) {
            timeIntervalEstimator.timeIntervalStandardDeviation
        } else {
            null
        }

    /**
     * Gets estimated standard deviation of time interval between measurements.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     */
    val timeIntervalStandardDeviationAsTime
        get() = if (resultAvailable) {
            timeIntervalEstimator.timeIntervalStandardDeviationAsTime
        } else {
            null
        }

    /**
     * Gets estimated standard deviation of time interval between measurements.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
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
     * Gets amount of elapsed time to compute sensor noise estimation expressed in
     * nanoseconds (ns), either if computation succeeds or not.
     */
    val elapsedTimeNanos
        get() = endTimestampNanos - initialTimestampNanos

    /**
     * Gets amount of elapsed time to compute sensor noise, either if computation succeeds
     * or not.
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
     * Handles new timestamp.
     */
    protected fun handleTimestamp(timestamp: Long) {
        if (numberOfProcessedMeasurements > 0) {
            val diff = timestamp - initialTimestampNanos
            val diffSeconds = TimeConverter.nanosecondToSecond(diff.toDouble())
            timeIntervalEstimator.addTimestamp(diffSeconds)
        }

        endTimestampNanos = timestamp
    }

    /**
     * Indicates whether estimation is complete.
     *
     * @return true if estimation is complete, false otherwise.
     */
    protected fun isComplete(): Boolean {
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
     * Resets internal estimator.
     */
    protected open fun reset() {
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
        resultUnreliable = false
    }

    /**
     * Notifies unreliable listener.
     */
    protected abstract fun notifyUnreliableListener()

    companion object {
        /**
         * Most sensors at maximum speed work at 50 Hz or 100 Hz, so using this value should require
         * approximately between 10 to 20 seconds to complete.
         * This is hardware dependant, since different sensors might achieve different maximum
         * sampling rates.
         */
        const val DEFAULT_MAX_SAMPLES = 1000

        /**
         * Maximum duration to keep sampling measurements until estimation is completed.
         */
        const val DEFAULT_MAX_DURATION_MILLIS = 20000L

        /**
         * Constant to convert nanoseconds to milliseconds.
         */
        private const val NANOS_TO_MILLIS = 1000000L
    }
}