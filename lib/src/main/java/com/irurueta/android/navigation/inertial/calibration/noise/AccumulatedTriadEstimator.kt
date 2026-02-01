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
import com.irurueta.android.navigation.inertial.collectors.measurements.TriadConvertible
import com.irurueta.navigation.inertial.calibration.Triad
import com.irurueta.units.Measurement
import com.irurueta.units.Time
import kotlin.Enum

/**
 * Estimates the accumulated mean values of a triad of measurements.
 * This estimator takes a given number of measurement triads (acceleration, angular speed,
 * magnetic field, etc) during a given duration of time to estimate measurement averages, standard
 * deviations, variances, as well as average time interval between measurements.
 * For best accuracy of estimated results, device should remain static while data is being
 * collected.
 * This estimator converts sensor measurements from device ENU coordinates to local plane NED
 * coordinates. Thus, all values referring to a given x-y-z coordinates refers to local plane
 * NED system of coordinates.
 *
 * @property context Android context.
 * @property sensorDelay Delay of sensor between samples.
 * @property completedListener listener to notify when estimation completes.
 * @property unreliableListener listener to notify when measurements become unreliable.
 * @param E Type of estimator.
 * @param P Type of processor.
 * @param C Sensor collector.
 * @param U Type of measurement unit.
 * @param M Type of measurement.
 * @param T Type of triad.
 * @param SM Type of sensor measurement.
 */
abstract class AccumulatedTriadEstimator<E, P, C, U, M, T, SM>(
    val context: Context,
    val sensorDelay: SensorDelay = SensorDelay.FASTEST,
    var completedListener: OnEstimationCompletedListener<E>? = null,
    var unreliableListener: OnUnreliableListener<E>? = null
) where E : AccumulatedTriadEstimator<E, P, C, U, M, T, SM>,
        P : AccumulatedTriadProcessor<P, *, U, M, T, SM>,
        C : SensorCollector<SM, C>,
        U : Enum<*>,
        M : Measurement<U>,
        T : Triad<U, M>,
        SM : SensorMeasurement<SM>,
        SM : TriadConvertible<T> {

    /**
     * Internal processor that processes measurements.
     */
    protected abstract val processor: P

    /**
     * Internal sensor collector that collects measurements.
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
     * Gets estimated average value of sensor x-axis measurements expressed in sensor default unit
     * (m/s^2 for acceleration, rad/s for angular speed or T for magnetic flux density).
     * This is only available when estimation completes successfully and [resultAvailable] is
     * true.
     * If device remained completely static during estimation, this should be equal to the x-axis
     * component of gravity acceleration (for accelerometer sensor), x-axis component of Earth
     * rotation rate (for gyroscope sensor), or x-axis component of magnetic field intensity (for
     * magnetometer).
     */
    val averageX
        get() = processor.averageX

    /**
     * Gets estimated average value of sensor x-axis measurements.
     * This is only available when estimation completes successfully and [resultAvailable] is
     * true.
     * If device remained completely static during estimation, this should be equal to the x-axis
     * component of gravity acceleration (for accelerometer sensor), x-axis component of Earth
     * rotation rate (for gyroscope sensor), or x-axis component of magnetic field intensity (for
     * magnetometer).
     */
    val averageXAsMeasurement
        get() = processor.averageXAsMeasurement

    /**
     * Gets estimated average value of sensor x-axis measurements.
     * This is only available when estimation completes successfully and [resultAvailable] is
     * true.
     * If device remained completely static during estimation, this should be equal to the x-axis
     * component of gravity acceleration (for accelerometer sensor), x-axis component of Earth
     * rotation rate (for gyroscope sensor), or x-axis component of magnetic field intensity (for
     * magnetometer).
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getAverageXAsMeasurement(result: M): Boolean {
        return processor.getAverageXAsMeasurement(result)
    }

    /**
     * Gets estimated average value of accelerometer y-axis measurements expressed in sensor default unit
     * (m/s^2 for acceleration, rad/s for angular speed or T for magnetic flux density).
     * This is only available when estimation completes successfully and [resultAvailable] is
     * true.
     * If device remained completely static during estimation, this should be equal to the x-axis
     * component of gravity acceleration (for accelerometer sensor), x-axis component of Earth
     * rotation rate (for gyroscope sensor), or x-axis component of magnetic field intensity (for
     * magnetometer).
     */
    val averageY
        get() = processor.averageY

    /**
     * Gets estimated average value of accelerometer y-axis measurements.
     * This is only available when estimation completes successfully and [resultAvailable] is
     * true.
     * If device remained completely static during estimation, this should be equal to the x-axis
     * component of gravity acceleration (for accelerometer sensor), x-axis component of Earth
     * rotation rate (for gyroscope sensor), or x-axis component of magnetic field intensity (for
     * magnetometer).
     */
    val averageYAsMeasurement
        get() = processor.averageYAsMeasurement

    /**
     * Gets estimated average value of accelerometer y-axis measurements.
     * This is only available when estimation completes successfully and [resultAvailable] is
     * true.
     * If device remained completely static during estimation, this should be equal to the x-axis
     * component of gravity acceleration (for accelerometer sensor), x-axis component of Earth
     * rotation rate (for gyroscope sensor), or x-axis component of magnetic field intensity (for
     * magnetometer).
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getAverageYAsMeasurement(result: M): Boolean {
        return processor.getAverageYAsMeasurement(result)
    }

    /**
     * Gets estimated average value of accelerometer z-axis measurements expressed in sensor default unit
     * (m/s^2 for acceleration, rad/s for angular speed or T for magnetic flux density).
     * This is only available when estimation completes successfully and [resultAvailable] is
     * true.
     * If device remained completely static during estimation, this should be equal to the x-axis
     * component of gravity acceleration (for accelerometer sensor), x-axis component of Earth
     * rotation rate (for gyroscope sensor), or x-axis component of magnetic field intensity (for
     * magnetometer).
     */
    val averageZ
        get() = processor.averageZ

    /**
     * Gets estimated average value of sensor z-axis measurements.
     * This is only available when estimation completes successfully and [resultAvailable] is
     * true.
     * If device remained completely static during estimation, this should be equal to the x-axis
     * component of gravity acceleration (for accelerometer sensor), x-axis component of Earth
     * rotation rate (for gyroscope sensor), or x-axis component of magnetic field intensity (for
     * magnetometer).
     */
    val averageZAsMeasurement
        get() = processor.averageZAsMeasurement

    /**
     * Gets estimated average value of sensor z-axis measurements.
     * This is only available when estimation completes successfully and [resultAvailable] is
     * true.
     * If device remained completely static during estimation, this should be equal to the x-axis
     * component of gravity acceleration (for accelerometer sensor), x-axis component of Earth
     * rotation rate (for gyroscope sensor), or x-axis component of magnetic field intensity (for
     * magnetometer).
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getAverageZAsMeasurement(result: M): Boolean {
        return processor.getAverageZAsMeasurement(result)
    }

    /**
     * Gets estimated average values of sensor.
     * This is only available when estimation completes successfully and [resultAvailable] is
     * true.
     * If device remained completely static during estimation, this should be equal to the x-axis
     * component of gravity acceleration (for accelerometer sensor), x-axis component of Earth
     * rotation rate (for gyroscope sensor), or x-axis component of magnetic field intensity (for
     * magnetometer).
     */
    val averageTriad
        get() = processor.averageTriad

    /**
     * Gets estimated average values of sensor.
     * This is only available when estimation completes successfully and [resultAvailable] is
     * true.
     * If device remained completely static during estimation, this should be equal to the x-axis
     * component of gravity acceleration (for accelerometer sensor), x-axis component of Earth
     * rotation rate (for gyroscope sensor), or x-axis component of magnetic field intensity (for
     * magnetometer).
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getAverageTriad(result: T): Boolean {
        return processor.getAverageTriad(result)
    }

    /**
     * Gets estimated average norm of sensor measurements expressed in sensor default unit
     * (m/s^2 for acceleration, rad/s for angular speed or T for magnetic flux density).
     * This is only available when estimation completes successfully and [resultAvailable] is
     * true.
     * If device remained completely static during estimation, this should be equal to gravity norm
     * (for accelerometer sensor), Earth rotation rate (for gyroscope sensor), or magnetic field
     * intensity (for magnetometer).
     */
    val averageNorm
        get() = processor.averageNorm

    /**
     * Gets estimated average norm of sensor measurements expressed in sensor default unit
     * (m/s^2 for acceleration, rad/s for angular speed or T for magnetic flux density).
     * This is only available when estimation completes successfully and [resultAvailable] is
     * true.
     * If device remained completely static during estimation, this should be equal to gravity norm
     * (for accelerometer sensor), Earth rotation rate (for gyroscope sensor), or magnetic field
     * intensity (for magnetometer).
     */
    val averageNormAsMeasurement
        get() = processor.averageNormAsMeasurement

    /**
     * Gets estimated average norm of sensor measurements expressed in sensor default unit
     * (m/s^2 for acceleration, rad/s for angular speed or T for magnetic flux density).
     * This is only available when estimation completes successfully and [resultAvailable] is
     * true.
     * If device remained completely static during estimation, this should be equal to gravity norm
     * (for accelerometer sensor), Earth rotation rate (for gyroscope sensor), or magnetic field
     * intensity (for magnetometer).
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getAverageNormAsMeasurement(result: M): Boolean {
        return processor.getAverageNormAsMeasurement(result)
    }

    /**
     * Gets estimated variance value of sensor x-axis measurements expressed in its default squared
     * unit (m^2/s^4 for acceleration, rad^2/s^2 for angular speed or T^2 for magnetic flux
     * density).
     * This is only available when estimation completes successfully and [resultAvailable] is
     * true.
     * If device remained completely static during estimation, this variance is an indication of
     * sensor noise.
     */
    val varianceX
        get() = processor.varianceX

    /**
     * Gets estimated variance value of sensor y-axis measurements expressed in its default squared
     * unit (m^2/s^4 for acceleration, rad^2/s^2 for angular speed or T^2 for magnetic flux
     * density).
     * This is only available when estimation completes successfully and [resultAvailable] is
     * true.
     * If device remained completely static during estimation, this variance is an indication of
     * sensor noise.
     */
    val varianceY
        get() = processor.varianceY

    /**
     * Gets estimated variance value of sensor z-axis measurements expressed in its default squared
     * unit (m^2/s^4 for acceleration, rad^2/s^2 for angular speed or T^2 for magnetic flux
     * density).
     * This is only available when estimation completes successfully and [resultAvailable] is
     * true.
     * If device remained completely static during estimation, this variance is an indication of
     * sensor noise.
     */
    val varianceZ
        get() = processor.varianceZ

    /**
     * Gets estimated standard deviation value of sensor x-axis measurements expressed in its
     * default unit (m/s^2 for acceleration, rad/s for angular speed or T for magnetic flux
     * density).
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this standard deviation is an
     * indication of sensor noise.
     */
    val standardDeviationX
        get() = processor.standardDeviationX

    /**
     * Gets estimated standard deviation value of sensor x-axis measurements.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this standard deviation is an
     * indication of sensor noise.
     */
    val standardDeviationXAsMeasurement
        get() = processor.standardDeviationXAsMeasurement

    /**
     * Gets estimated standard deviation value of sensor x-axis measurements.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this standard deviation is an
     * indication of sensor noise.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getStandardDeviationXAsMeasurement(result: M): Boolean {
        return processor.getStandardDeviationXAsMeasurement(result)
    }

    /**
     * Gets estimated standard deviation value of sensor y-axis measurements expressed in its
     * default unit (m/s^2 for acceleration, rad/s for angular speed or T for magnetic flux
     * density).
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this standard deviation is an
     * indication of sensor noise.
     */
    val standardDeviationY
        get() = processor.standardDeviationY

    /**
     * Gets estimated standard deviation value of sensor y-axis measurements.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this standard deviation is an
     * indication of sensor noise.
     */
    val standardDeviationYAsMeasurement
        get() = processor.standardDeviationYAsMeasurement

    /**
     * Gets estimated standard deviation value of sensor y-axis measurements.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this standard deviation is an
     * indication of sensor noise.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getStandardDeviationYAsMeasurement(result: M): Boolean {
        return processor.getStandardDeviationYAsMeasurement(result)
    }

    /**
     * Gets estimated standard deviation value of sensor z-axis measurements expressed in its
     * default unit (m/s^2 for acceleration, rad/s for angular speed or T for magnetic flux
     * density).
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this standard deviation is an
     * indication of sensor noise.
     */
    val standardDeviationZ
        get() = processor.standardDeviationZ

    /**
     * Gets estimated standard deviation value of sensor z-axis measurements.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this standard deviation is an
     * indication of sensor noise.
     */
    val standardDeviationZAsMeasurement
        get() = processor.standardDeviationZAsMeasurement

    /**
     * Gets estimated standard deviation value of sensor z-axis measurements.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this standard deviation is an
     * indication of sensor noise.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getStandardDeviationZAsMeasurement(result: M): Boolean {
        return processor.getStandardDeviationZAsMeasurement(result)
    }

    /**
     * Gets estimated standard deviation values of sensor.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this is an indication of
     * sensor noise.
     */
    val standardDeviationTriad
        get() = processor.standardDeviationTriad

    /**
     * Gets estimated standard deviation values of sensor.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this is an indication of
     * sensor noise.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getStandardDeviationTriad(result: T): Boolean {
        return processor.getStandardDeviationTriad(result)
    }

    /**
     * Gets norm of estimated standard deviations of sensor measurements expressed in its default
     * unit (m/s^2 for acceleration, rad/s for angular speed or T for magnetic flux density).
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this standard deviation is an
     * indication of sensor noise.
     */
    val standardDeviationNorm
        get() = processor.standardDeviationNorm

    /**
     * Gets norm of estimated standard deviations of sensor measurements.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this standard deviation is an
     * indication of sensor noise.
     */
    val standardDeviationNormAsMeasurement
        get() = processor.standardDeviationNormAsMeasurement

    /**
     * Gets norm of estimated standard deviations of sensor measurements.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this standard deviation is an
     * indication of sensor noise.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getStandardDeviationNormAsMeasurement(result: M): Boolean {
        return processor.getStandardDeviationNormAsMeasurement(result)
    }

    /**
     * Gets average of estimated standard deviations of sensor measurements expressed in its default
     * unit (m/s^2 for acceleration, rad/s for angular speed or T for magnetic flux density).
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this standard deviation is an
     * indication of sensor noise.
     */
    val averageStandardDeviation
        get() = processor.averageStandardDeviation

    /**
     * Gets average of estimated standard deviations of sensor measurements.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this standard deviation is an
     * indication of sensor noise.
     */
    val averageStandardDeviationAsMeasurement
        get() = processor.averageStandardDeviationAsMeasurement

    /**
     * Gets average of estimated standard deviations of sensor measurements.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this standard deviation is an
     * indication of sensor noise.
     *
     * @param result instance where result will be stored.
     * @return true if result is available, false otherwise.
     */
    fun getAverageStandardDeviationAsMeasurement(result: M): Boolean {
        return processor.getAverageStandardDeviationAsMeasurement(result)
    }

    /**
     * Gets sensor noise PSD (Power Spectral Density) on x axis expressed in  (m^2 * s^-3) for
     * accelerometer, (rad^2/s) for gyroscope or (T^2 * s) for magnetometer.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this is an indication of
     * sensor noise.
     */
    val psdX
        get() = processor.psdX

    /**
     * Gets sensor noise PSD (Power Spectral Density) on y axis expressed in  (m^2 * s^-3) for
     * accelerometer, (rad^2/s) for gyroscope or (T^2 * s) for magnetometer.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this is an indication of
     * sensor noise.
     */
    val psdY
        get() = processor.psdY

    /**
     * Gets sensor noise PSD (Power Spectral Density) on z axis expressed in  (m^2 * s^-3) for
     * accelerometer, (rad^2/s) for gyroscope or (T^2 * s) for magnetometer.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this is an indication of
     * sensor noise.
     */
    val psdZ
        get() = processor.psdZ

    /**
     * Gets sensor noise root PSD (Power Spectral Density) on x axis expressed in
     * (m * s^-1.5) for accelerometer, (rad * s^-0.5) for gyroscope or (T * s^0.5) for magnetometer.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this is an indication of
     * sensor noise.
     */
    val rootPsdX
        get() = processor.rootPsdX

    /**
     * Gets sensor noise root PSD (Power Spectral Density) on y axis expressed in
     * (m * s^-1.5) for accelerometer, (rad * s^-0.5) for gyroscope or (T * s^0.5) for magnetometer.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this is an indication of
     * sensor noise.
     */
    val rootPsdY
        get() = processor.rootPsdY

    /**
     * Gets sensor noise root PSD (Power Spectral Density) on z axis expressed in
     * (m * s^-1.5) for accelerometer, (rad * s^-0.5) for gyroscope or (T * s^0.5) for magnetometer.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this is an indication of
     * sensor noise.
     */
    val rootPsdZ
        get() = processor.rootPsdZ

    /**
     * Gets average sensor measurement noise PSD (Power Spectral Density) expressed in
     * (m^2 * s^-3) for accelerometer, (rad^2/s) for gyroscope or (T^2 * s) for magnetometer.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this is an indication of
     * sensor noise.
     */
    val averageNoisePsd
        get() = processor.averageNoisePsd

    /**
     * Gets norm of sensor noise root PSD (Power Spectral Density) expressed in (m * s^-1.5) for
     * accelerometer, (rad * s^-0.5) for gyroscope or (T * s^0.5) for magnetometer.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this is an indication of
     * sensor noise.
     */
    val noiseRootPsdNorm
        get() = processor.noiseRootPsdNorm

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
    fun interface OnEstimationCompletedListener<E : AccumulatedTriadEstimator<E, *, *, *, *, *, *>> {

        /**
         * Called when estimation is completed.
         *
         * @param estimator estimator that completed its estimation.
         */
        fun onEstimationCompleted(estimator: E)
    }

    /**
     * Notifies when measurements become unreliable.
     */
    fun interface OnUnreliableListener<E : AccumulatedTriadEstimator<E, *, *, *, *, *, *>> {

        /**
         * Called when measurements become unreliable.
         *
         * @param estimator estimator whose measurements have become unreliable.
         */
        fun onUnreliable(estimator: E)
    }
}