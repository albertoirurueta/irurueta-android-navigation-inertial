/*
 * Copyright (C) 2025 Alberto Irurueta Carro (alberto@irurueta.com)
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

package com.irurueta.android.navigation.inertial.calibration.noise

import android.content.Context
import android.hardware.Sensor
import com.irurueta.android.navigation.inertial.collectors.SensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorMeasurement
import com.irurueta.units.Measurement
import com.irurueta.units.Time

/**
 * Estimates the accumulated mean value of a measurement.
 * This estimator takes a given number of measurement magnitudes (acceleration norm, gravity norm,
 * angular speed norm, magnetic field norm, etc) during a given duration of time to estimate
 * measurement norm average, standard deviation and variance, as well as average time interval
 * between measurements.
 * For best accuracy of estimated results, device should remain static while data is being
 * processed.
 *
 * @param E Type of estimator.
 * @param P Type of processor.
 * @param C Sensor collector.
 * @param M Type of measurement.
 * @param SM Type of sensor measurement.
 * @property context Android context.
 * @property sensorDelay Delay of sensor between samples.
 * @property completedListener listener to notify when estimation completes.
 * @property unreliableListener listener to notify when measurements become unreliable.
 */
abstract class AccumulatedMeasurementEstimator<E : AccumulatedMeasurementEstimator<E, P, C, M, SM>,
        P : AccumulatedMeasurementProcessor<*, *, *, M, SM>, C: SensorCollector<SM, C>,
        M : Measurement<*>, SM : SensorMeasurement<SM>>(
    val context: Context,
    val sensorDelay: SensorDelay = SensorDelay.FASTEST,
    var completedListener: OnEstimationCompletedListener<E>? = null,
    var unreliableListener: OnUnreliableListener<E>? = null
) {

    /**
     * Internal processor that processes measurements.
     */
    protected abstract val processor: P

    /**
     * Internal collector that collects measurements.
     */
    protected abstract val collector: C

    /**
     * Listener to handle changes of accuracy in accelerometer sensor.
     */
    protected val accuracyChangedListener =
        SensorCollector.OnAccuracyChangedListener<SM, C> { _, accuracy ->
            if (accuracy == SensorAccuracy.UNRELIABLE) {
                resultUnreliable = true
                notifyUnreliableListener()
            }
        }

    /**
     * Listener to handle accelerometer measurements.
     */
    protected val measurementListener =
        SensorCollector.OnMeasurementListener<SM, C> { _, measurement ->
            handleMeasurement(measurement)
        }

    /**
     * Gets sensor being used to obtain measurements or null if not available.
     *This can be used to obtain additional information about the sensor.
     */
    val sensor: Sensor?
        get() = collector.sensor

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
    val averageNorm: Double?
        get() = processor.averageNorm

    /**
     * Gets estimated average measurement norm as a measurement.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     */
    val averageNormAsMeasurement: M?
        get() = processor.averageNormAsMeasurement

    /**
     * Gets estimated average measurement norm as a measurement instance.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     *
     * @param result instance where result will be stored.
     * @return true if result was set, false otherwise.
     */
    fun getAverageNormAsMeasurement(result: M): Boolean {
        return processor.getAverageNormAsMeasurement(result)
    }

    /**
     * Gets estimated variance of measurement norm expressed in the default
     * squared unit of measurement (m^2/s^4 for acceleration, rad^2/s^2 for angular speed or T^2 for
     * magnetic flux density).
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     */
    val normVariance: Double?
        get() = processor.normVariance

    /**
     * Gets estimated standard deviation of norm expressed in the default unit of
     * measurement (m/s^2 for acceleration, rad/s for angular speed or T for magnetic flux density).
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     */
    val normStandardDeviation: Double?
        get() = processor.normStandardDeviation

    /**
     * Gets estimated standard deviation of norm as a measurement.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     */
    val normStandardDeviationAsMeasurement: M?
        get() = processor.normStandardDeviationAsMeasurement

    /**
     * Gets estimated standard deviation of norm as a measurement.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getNormStandardDeviationAsMeasurement(result: M): Boolean {
        return processor.getNormStandardDeviationAsMeasurement(result)
    }

    /**
     * Gets PSD (Power Spectral Density) of norm noise expressed in (m^2 * s^-3) for accelerometer,
     * (rad^2/s) for gyroscope or (T^2 * s) for magnetometer.
     * This is only available when estimation completes successfully and average time interval
     * between measurements is reliably estimated.
     */
    val psd : Double?
        get() = processor.psd

    /**
     * Gets root PSD (Power Spectral Density) of norm expressed in (m * s^-1.5) for
     * accelerometer, (rad * s^-0.5) for gyroscope or (T * s^0.5) for magnetometer.
     * This is only available when estimation completes successfully and average time interval
     * between measurements is reliably estimated.
     */
    val rootPsd: Double?
        get() = processor.rootPsd

    /**
     * Contains timestamp when the first measurement was processed expressed in nanoseconds
     * using [android.os.SystemClock.elapsedRealtimeNanos] time base.
     */
    val initialTimestampNanos: Long
        get () = processor.initialTimestampNanos

    /**
     * Contains timestamp when the last measurement was processed expressed in nanoseconds
     * using [android.os.SystemClock.elapsedRealtimeNanos] time base.
     */
    val endTimestampNanos: Long
        get() = processor.endTimestampNanos

    /**
     * Number of measurements that have been processed.
     */
    val numberOfProcessedMeasurements: Long
        get() = processor.numberOfProcessedMeasurements

    /**
     * Gets maximum number of samples to take into account before completion. This is only taken
     * into account if using either [StopMode.MAX_SAMPLES_ONLY] or
     * [StopMode.MAX_SAMPLES_OR_DURATION].
     */
    val maxSamples: Int
        get() = processor.maxSamples

    /**
     * Gets maximum duration expressed in milliseconds to take into account before completion. This
     * is only taken into account if using either [StopMode.MAX_DURATION_ONLY] or
     * [StopMode.MAX_SAMPLES_OR_DURATION].
     */
    val maxDurationMillis: Long
        get() = processor.maxDurationMillis

    /**
     * Determines when this processor will consider its estimation completed.
     */
    val stopMode: StopMode
        get() = processor.stopMode

    /**
     * Indicates whether estimated average and time interval between measurements are available
     * or not.
     */
    val resultAvailable: Boolean
        get() = processor.resultAvailable

    /**
     * Indicates whether estimated result is unreliable or not.
     */
    var resultUnreliable: Boolean = false
        protected set

    /**
     * Gets average time interval between measurements expressed in seconds (s). This is only
     * available when estimation completes successfully and [resultAvailable] is true.
     */
    val averageTimeInterval: Double?
        get() = processor.averageTimeInterval

    /**
     * Gets average time interval between measurements. This is only available when estimation
     * completes successfully and [resultAvailable] is true.
     */
    val averageTimeIntervalAsTime: Time?
        get() = processor.averageTimeIntervalAsTime

    /**
     * Gets average time interval between measurements.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     *
     * @param result instance where average time interval will be stored.
     * @return true if result was set, false otherwise.
     */
    fun getAverageTimeIntervalAsTime(result: Time): Boolean {
        return processor.getAverageTimeIntervalAsTime(result)
    }

    /**
     * Gets estimated variance of time interval between measurements expressed in seconds squared
     * (s^2). This is only available when estimation completes successfully and [resultAvailable]
     * is true.
     */
    val timeIntervalVariance: Double?
        get() = processor.timeIntervalVariance

    /**
     * Gets estimated standard deviation of time interval between measurements expressed in
     * seconds (s). This is only available when estimation completes successfully and
     * [resultAvailable] is true.
     */
    val timeIntervalStandardDeviation: Double?
        get() = processor.timeIntervalStandardDeviation

    /**
     * Gets estimated standard deviation of time interval between measurements. This is only
     * available when estimation completes successfully and [resultAvailable] is true.
     */
    val timeIntervalStandardDeviationAsTime: Time?
        get() = processor.timeIntervalStandardDeviationAsTime

    /**
     * Gets estimated standard deviation of time interval between measurements. This is only
     * available when estimation completes successfully and [resultAvailable] is true.
     *
     * @param result instance where result will be stored.
     * @return true if result was set, false otherwise.
     */
    fun getTimeIntervalStandardDeviationAsTime(result: Time): Boolean {
        return processor.getTimeIntervalStandardDeviationAsTime(result)
    }

    /**
     * Gets elapsed time since first measurement was processed expressed in nanoseconds (ns).
     */
    val elapsedTimeNanos: Long
        get() = processor.elapsedTimeNanos


    /**
     * Gets elapsed time since first measurement was processed.
     */
    val elapsedTime: Time
        get() = processor.elapsedTime

    /**
     * Gets elapsed time since first measurement was processed.
     *
     * @param result instance where result will be stored.
     */
    fun getElapsedTime(result: Time) {
        processor.getElapsedTime(result)
    }

    /**
     * Starts collection of sensor norm measurements.
     *
     * @throws IllegalStateException if estimator is already running or sensor is not available.
     */
    @Throws(IllegalStateException::class)
    fun start() {
        check(!running)

        reset()

        running = true
        if (!collector.start()) {
            throw IllegalStateException("Unavailable sensor")
        }
    }

    /**
     * Stops collection of sensor norm measurements.
     *
     * @throws IllegalStateException if estimator is NOT already running.
     */
    @Throws(IllegalStateException::class)
    fun stop() {
        check(running)

        collector.stop()
        running = false
    }

    /**
     * Notifies unreliable listener.
     */
    protected fun notifyUnreliableListener() {
        @Suppress("UNCHECKED_CAST")
        unreliableListener?.onUnreliable(this as E)
    }

    /**
     * Handles a new measurement.
     *
     * @param measurement new measurement.
     */
    protected fun handleMeasurement(measurement: SM) {
        if (measurement.accuracy == SensorAccuracy.UNRELIABLE) {
            resultUnreliable = true
        }

        if (processor.process(measurement)) {
            stop()

            @Suppress("UNCHECKED_CAST")
            completedListener?.onEstimationCompleted(this as E)
        }
    }

    /**
     * Resets this estimator to its initial state.
     */
    fun reset() {
        processor.reset()
    }

    /**
     * Notifies when estimation completes.
     */
    fun interface OnEstimationCompletedListener<E : AccumulatedMeasurementEstimator<E, *, *, *, *>> {

        /**
         * Called when estimation completes.
         *
         * @param estimator instance that raised the event.
         */
        fun onEstimationCompleted(estimator: E)
    }

    /**
     * Notifies when measurements become unreliable.
     */
    fun interface OnUnreliableListener<E : AccumulatedMeasurementEstimator<E, *, *, *, *>> {

        /**
         * Called when measurements become unreliable.
         *
         * @param estimator instance that raised the event.
         */
        fun onUnreliable(estimator: E)
    }
}