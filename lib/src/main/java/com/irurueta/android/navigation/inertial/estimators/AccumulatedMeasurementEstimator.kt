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
import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.SensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.navigation.inertial.calibration.noise.AccumulatedMeasurementNoiseEstimator
import com.irurueta.units.Measurement

/**
 * Estimates the accumulated mean value of a measurement.
 * This estimator takes a given number of measurement magnitudes (acceleration norm, gravity norm,
 * angular speed norm, magnetic field norm, etc) during a given duration of time to estimate
 * measurement norm average, standard deviation and variance, as well as average time interval
 * between measurements.
 * For best accuracy of estimated results, device should remain static while data is being
 * collected.
 *
 * @property context Android context.
 * @property sensorDelay Delay of sensor between samples.
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
abstract class AccumulatedMeasurementEstimator<A : AccumulatedMeasurementEstimator<A, N, C, U, M>,
        N : AccumulatedMeasurementNoiseEstimator<U, M, *, *>, C : SensorCollector,
        U : Enum<*>, M : Measurement<U>>(
    val context: Context,
    val sensorDelay: SensorDelay = SensorDelay.FASTEST,
    maxSamples: Int = DEFAULT_MAX_SAMPLES,
    maxDurationMillis: Long = DEFAULT_MAX_DURATION_MILLIS,
    stopMode: StopMode = StopMode.MAX_SAMPLES_OR_DURATION,
    var completedListener: OnEstimationCompletedListener<A>? = null,
    var unreliableListener: OnUnreliableListener<A>? = null
) : BaseAccumulatedEstimator(maxSamples, maxDurationMillis, stopMode) {

    /**
     * Internal noise estimator of magnitude measurements.
     * This can be used to estimate statistics about a given measurement magnitude.
     */
    protected abstract val noiseEstimator: N

    /**
     * Collector for magnitude measurements.
     */
    protected abstract val collector: C

    /**
     * Indicates whether this estimator is already running.
     */
    var running = false
        private set

    /**
     * Gets estimated average of measurement norm expressed in measurement unit (m/s^2 for
     * acceleration, rad/s for angular speed or T for magnetic flux density).
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     */
    val averageNorm
        get() = if (resultAvailable) {
            noiseEstimator.avg
        } else {
            null
        }

    /**
     * Gets estimated average measurement norm as a measurement.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     */
    val averageNormAsMeasurement
        get() = if (resultAvailable) {
            noiseEstimator.avgAsMeasurement
        } else {
            null
        }

    /**
     * Gets estimated average measurement norm as a measurement instance.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getAverageNormAsMeasurement(result: M): Boolean {
        return if (resultAvailable) {
            noiseEstimator.getAvgAsMeasurement(result)
            true
        } else {
            false
        }
    }

    /**
     * Gets estimated variance of measurement norm expressed in the default
     * squared unit of measurement (m^2/s^4 for acceleration, rad^2/s^2 for angular speed or T^2 for
     * magnetic flux density).
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     */
    val normVariance
        get() = if (resultAvailable) {
            noiseEstimator.variance
        } else {
            null
        }

    /**
     * Gets estimated standard deviation of norm expressed in the default unit of
     * measurement (m/s^2 for acceleration, rad/s for angular speed or T for magnetic flux density).
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     */
    val normStandardDeviation
        get() = if (resultAvailable) {
            noiseEstimator.standardDeviation
        } else {
            null
        }

    /**
     * Gets estimated standard deviation of norm as a measurement.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     */
    val normStandardDeviationAsMeasurement
        get() = if (resultAvailable) {
            noiseEstimator.standardDeviationAsMeasurement
        } else {
            null
        }

    /**
     * Gets estimated standard deviation of norm as a measurement.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getNormStandardDeviationAsMeasurement(result: M): Boolean {
        return if (resultAvailable) {
            noiseEstimator.getStandardDeviationAsMeasurement(result)
            true
        } else {
            false
        }
    }

    /**
     * Gets PSD (Power Spectral Density) of norm noise expressed in (m^2 * s^-3) for accelerometer,
     * (rad^2/s) for gyroscope or (T^2 * s) for magnetometer.
     * This is only available when estimation completes successfully and average time interval
     * between measurements is reliably estimated.
     */
    val psd
        get() = if (resultAvailable) {
            noiseEstimator.psd
        } else {
            null
        }

    /**
     * Gets root PSD (Power Spectral Density) of norm noise expressed in (m * s^-1.5) for
     * accelerometer, (rad * s^-0.5) for gyroscope or (T * s^0.5) for magnetometer.
     * This is only available when estimation completes successfully and average time interval
     * between measurements is reliably estimated.
     */
    val rootPsd
        get() = if (resultAvailable) {
            noiseEstimator.rootPsd
        } else {
            null
        }

    /**
     * Starts collection of sensor norm measurements.
     *
     * @throws IllegalStateException if estimator is already running.
     */
    @Throws(IllegalStateException::class)
    fun start() {
        check(!running)

        reset()

        running = true
        collector.start()
    }

    /**
     * Stops collection of sensor norm measurements.
     */
    fun stop() {
        collector.stop()
        running = false
    }

    /**
     * Notifies unreliable listener.
     */
    override fun notifyUnreliableListener() {
        @Suppress("UNCHECKED_CAST")
        unreliableListener?.onUnreliable(this as A)
    }

    /**
     * Handles a magnitude measurement
     *
     * @param value value of measurement expressed in sensor unit.
     * @param timestamp time in nanoseconds at which the measurement was made. Each measurement
     * will be monotonically increasing using the same time base as
     * [android.os.SystemClock.elapsedRealtimeNanos].
     * @param accuracy sensor accuracy.
     */
    protected fun handleMeasurement(value: Double, timestamp: Long, accuracy: SensorAccuracy?) {
        if (accuracy == SensorAccuracy.UNRELIABLE) {
            resultUnreliable = true
        }

        if (numberOfProcessedMeasurements == 0) {
            initialTimestampNanos = timestamp
        }

        noiseEstimator.addMeasurement(value)

        handleTimestamp(timestamp)
        numberOfProcessedMeasurements++

        if (isComplete()) {
            // once time interval has been estimated, it is set into noise estimator so that
            // PSD values can be correctly estimated.
            noiseEstimator.timeInterval = timeIntervalEstimator.averageTimeInterval
            resultAvailable = true

            stop()

            @Suppress("UNCHECKED_CAST")
            completedListener?.onEstimationCompleted(this as A)
        }
    }

    /**
     * Resets internal estimators.
     */
    override fun reset() {
        noiseEstimator.reset()
        noiseEstimator.timeInterval = 0.0

        super.reset()
    }

    /**
     * Interface to notify when estimation completes.
     */
    interface OnEstimationCompletedListener<A : AccumulatedMeasurementEstimator<*, *, *, *, *>> {
        /**
         * Called when estimation completes.
         *
         * @param estimator estimator that generated the event.
         */
        fun onEstimationCompleted(estimator: A)
    }

    /**
     * Interface to notify when measurements become unreliable.
     */
    interface OnUnreliableListener<A : AccumulatedMeasurementEstimator<*, *, *, *, *>> {
        /**
         * Called when measurements become unreliable.
         *
         * @param estimator estimator that generated the event.
         */
        fun onUnreliable(estimator: A)
    }
}