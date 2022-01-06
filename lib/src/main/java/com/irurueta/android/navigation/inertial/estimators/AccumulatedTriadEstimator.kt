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
import com.irurueta.navigation.inertial.calibration.TimeIntervalEstimator
import com.irurueta.navigation.inertial.calibration.Triad
import com.irurueta.navigation.inertial.calibration.noise.AccumulatedTriadNoiseEstimator
import com.irurueta.units.Measurement
import com.irurueta.units.Time
import com.irurueta.units.TimeConverter
import com.irurueta.units.TimeUnit

/**
 * Estimates the accumulated mean values of a triad of measurements.
 * This estimator takes a given number of measurement triads (acceleration, angular speed,
 * magnetic field, etc) during a given duration of time to estimate measurement averages, standard
 * deviations, variances, as well as average time interval between measurements.
 * For best accuracy of estimated results, device should remain static while data is being
 * collected.
 *
 * @property context Android context.
 * @property sensorDelay Delay of sensor between samples.
 * @property stopMode Determines when this estimator will consider its estimation completed.
 * @property completedListener Listener to notify when estimation is complete.
 * @property unreliableListener Listener to notify when sensor becomes unreliable, and thus,
 * estimation must be discarded.
 */
abstract class AccumulatedTriadEstimator<A : AccumulatedTriadEstimator<A, N, C, U, M, T>,
        N : AccumulatedTriadNoiseEstimator<U, M, T, *, *>, C : SensorCollector,
        U : Enum<*>, M : Measurement<U>, T : Triad<U, M>> private constructor(
    val context: Context,
    val sensorDelay: SensorDelay = SensorDelay.FASTEST,
    val stopMode: StopMode = StopMode.MAX_SAMPLES_OR_DURATION,
    var completedListener: OnEstimationCompletedListener<A>? = null,
    var unreliableListener: OnUnreliableListener<A>? = null
) {

    /**
     * Constructor.
     *
     * @param context Android context.
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
     * @throws IllegalArgumentException when either [maxSamples] or [maxDurationMillis] is negative.
     */
    @Throws(IllegalArgumentException::class)
    constructor(
        context: Context,
        sensorDelay: SensorDelay = SensorDelay.FASTEST,
        maxSamples: Int = DEFAULT_MAX_SAMPLES,
        maxDurationMillis: Long = DEFAULT_MAX_DURATION_MILLIS,
        stopMode: StopMode = StopMode.MAX_SAMPLES_OR_DURATION,
        completedListener: OnEstimationCompletedListener<A>? = null,
        unreliableListener: OnUnreliableListener<A>? = null
    ) : this(context, sensorDelay, stopMode, completedListener, unreliableListener) {

        require(maxSamples >= 0)
        require(maxDurationMillis >= 0)

        this.maxSamples = maxSamples
        this.maxDurationMillis = maxDurationMillis
    }

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
     * Internal time estimator.
     * This can be used to estimate statistics about time intervals of measurements.
     */
    private val timeIntervalEstimator = TimeIntervalEstimator()

    /**
     * Listener to handle changes of accuracy in gravity sensor.
     */
    protected val accuracyChangedListener =
        object : SensorCollector.OnAccuracyChangedListener {
            override fun onAccuracyChanged(accuracy: SensorAccuracy?) {
                if (accuracy == SensorAccuracy.UNRELIABLE) {
                    resultUnreliable = true
                    @Suppress("UNCHECKED_CAST")
                    unreliableListener?.onUnreliable(this@AccumulatedTriadEstimator as A)
                }
            }
        }

    /**
     * Internal noise estimator of magnitude measurements.
     * This can be used to estimate statistics about a given measurement magnitude.
     */
    protected abstract val noiseEstimator: N

    /**
     * Collector for measurements.
     */
    protected abstract val collector: C

    /**
     * Gets maximum number of samples to take into account before completion. This is
     * only taken into account if using either [StopMode.MAX_SAMPLES_ONLY] or
     * [StopMode.MAX_SAMPLES_OR_DURATION].
     */
    var maxSamples: Int = AccumulatedMeasurementEstimator.DEFAULT_MAX_SAMPLES
        private set

    /**
     * Gets maximum duration expressed in milliseconds to take into account
     * before completion. This is only taken into account if using either
     * [StopMode.MAX_DURATION_ONLY] or [StopMode.MAX_SAMPLES_OR_DURATION].
     */
    var maxDurationMillis: Long = AccumulatedMeasurementEstimator.DEFAULT_MAX_DURATION_MILLIS
        private set

    /**
     * Indicates whether this estimator is already running.
     */
    var running = false
        private set

    /**
     * Number of measurements that have been processed.
     */
    var numberOfProcessedMeasurements: Int = 0
        private set

    /**
     * Indicates whether estimated averages and time interval between measurements are available
     * or not.
     */
    var resultAvailable: Boolean = false
        private set

    /**
     * Indicates whether estimated result is unreliable or not.
     */
    var resultUnreliable: Boolean = false
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
        get() = if (resultAvailable) {
            noiseEstimator.avgX
        } else {
            null
        }

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
        get() = if (resultAvailable) {
            noiseEstimator.avgXAsMeasurement
        } else {
            null
        }

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
        return if (resultAvailable) {
            noiseEstimator.getAvgXAsMeasurement(result)
            true
        } else {
            false
        }
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
        get() = if (resultAvailable) {
            noiseEstimator.avgY
        } else {
            null
        }

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
        get() = if (resultAvailable) {
            noiseEstimator.avgYAsMeasurement
        } else {
            null
        }

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
        return if (resultAvailable) {
            noiseEstimator.getAvgYAsMeasurement(result)
            true
        } else {
            false
        }
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
        get() = if (resultAvailable) {
            noiseEstimator.avgZ
        } else {
            null
        }

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
        get() = if (resultAvailable) {
            noiseEstimator.avgZAsMeasurement
        } else {
            null
        }

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
        return if (resultAvailable) {
            noiseEstimator.getAvgZAsMeasurement(result)
            true
        } else {
            false
        }
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
        get() = if (resultAvailable) {
            noiseEstimator.avgTriad
        } else {
            null
        }

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
        return if (resultAvailable) {
            noiseEstimator.getAvgTriad(result)
            true
        } else {
            false
        }
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
        get() = if (resultAvailable) {
            noiseEstimator.avgNorm
        } else {
            null
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
    val averageNormAsMeasurement
        get() = if (resultAvailable) {
            noiseEstimator.avgNormAsMeasurement
        } else {
            null
        }

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
        return if (resultAvailable) {
            noiseEstimator.getAvgNormAsMeasurement(result)
            true
        } else {
            false
        }
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
        get() = if (resultAvailable) {
            noiseEstimator.varianceX
        } else {
            null
        }

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
        get() = if (resultAvailable) {
            noiseEstimator.varianceY
        } else {
            null
        }

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
        get() = if (resultAvailable) {
            noiseEstimator.varianceZ
        } else {
            null
        }

    /**
     * Gets estimated standard deviation value of sensor x-axis measurements expressed in its
     * default unit (m/s^2 for acceleration, rad/s for angular speed or T for magnetic flux
     * density).
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this standard deviation is an
     * indication of sensor noise.
     */
    val standardDeviationX
        get() = if (resultAvailable) {
            noiseEstimator.standardDeviationX
        } else {
            null
        }

    /**
     * Gets estimated standard deviation value of sensor x-axis measurements.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this standard deviation is an
     * indication of sensor noise.
     */
    val standardDeviationXAsMeasurement
        get() = if (resultAvailable) {
            noiseEstimator.standardDeviationXAsMeasurement
        } else {
            null
        }

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
        return if (resultAvailable) {
            noiseEstimator.getStandardDeviationXAsMeasurement(result)
            true
        } else {
            false
        }
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
        get() = if (resultAvailable) {
            noiseEstimator.standardDeviationY
        } else {
            null
        }

    /**
     * Gets estimated standard deviation value of sensor y-axis measurements.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this standard deviation is an
     * indication of sensor noise.
     */
    val standardDeviationYAsMeasurement
        get() = if (resultAvailable) {
            noiseEstimator.standardDeviationYAsMeasurement
        } else {
            null
        }

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
        return if (resultAvailable) {
            noiseEstimator.getStandardDeviationYAsMeasurement(result)
            true
        } else {
            false
        }
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
        get() = if (resultAvailable) {
            noiseEstimator.standardDeviationZ
        } else {
            null
        }

    /**
     * Gets estimated standard deviation value of sensor z-axis measurements.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this standard deviation is an
     * indication of sensor noise.
     */
    val standardDeviationZAsMeasurement
        get() = if (resultAvailable) {
            noiseEstimator.standardDeviationZAsMeasurement
        } else {
            null
        }

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
        return if (resultAvailable) {
            noiseEstimator.getStandardDeviationZAsMeasurement(result)
            true
        } else {
            false
        }
    }

    /**
     * Gets estimated standard deviation values of sensor.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this is an indication of
     * sensor noise.
     */
    val standardDeviationTriad
        get() = if (resultAvailable) {
            noiseEstimator.standardDeviationTriad
        } else {
            null
        }

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
        return if (resultAvailable) {
            noiseEstimator.getStandardDeviationTriad(result)
            true
        } else {
            false
        }
    }

    /**
     * Gets norm of estimated standard deviations of sensor measurements expressed in its default
     * unit (m/s^2 for acceleration, rad/s for angular speed or T for magnetic flux density).
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this standard deviation is an
     * indication of sensor noise.
     */
    val standardDeviationNorm
        get() = if (resultAvailable) {
            noiseEstimator.standardDeviationNorm
        } else {
            null
        }

    /**
     * Gets norm of estimated standard deviations of sensor measurements.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this standard deviation is an
     * indication of sensor noise.
     */
    val standardDeviationNormAsMeasurement
        get() = if (resultAvailable) {
            noiseEstimator.standardDeviationNormAsMeasurement
        } else {
            null
        }

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
        return if (resultAvailable) {
            noiseEstimator.getStandardDeviationNormAsMeasurement(result)
            true
        } else {
            false
        }
    }

    /**
     * Gets average of estimated standard deviations of sensor measurements expressed in its default
     * unit (m/s^2 for acceleration, rad/s for angular speed or T for magnetic flux density).
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this standard deviation is an
     * indication of sensor noise.
     */
    val averageStandardDeviation
        get() = if (resultAvailable) {
            noiseEstimator.averageStandardDeviation
        } else {
            null
        }

    /**
     * Gets average of estimated standard deviations of sensor measurements.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this standard deviation is an
     * indication of sensor noise.
     */
    val averageStandardDeviationAsMeasurement
        get() = if (resultAvailable) {
            noiseEstimator.averageStandardDeviationAsMeasurement
        } else {
            null
        }

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
        return if (resultAvailable) {
            noiseEstimator.getAverageStandardDeviationAsMeasurement(result)
            true
        } else {
            false
        }
    }

    /**
     * Gets sensor noise PSD (Power Spectral Density) on x axis expressed in  (m^2 * s^-3) for
     * accelerometer, (rad^2/s) for gyroscope or (T^2 * s) for magnetometer.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this is an indication of
     * sensor noise.
     */
    val psdX
        get() = if (resultAvailable) {
            noiseEstimator.psdX
        } else {
            null
        }

    /**
     * Gets sensor noise PSD (Power Spectral Density) on y axis expressed in  (m^2 * s^-3) for
     * accelerometer, (rad^2/s) for gyroscope or (T^2 * s) for magnetometer.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this is an indication of
     * sensor noise.
     */
    val psdY
        get() = if (resultAvailable) {
            noiseEstimator.psdY
        } else {
            null
        }

    /**
     * Gets sensor noise PSD (Power Spectral Density) on z axis expressed in  (m^2 * s^-3) for
     * accelerometer, (rad^2/s) for gyroscope or (T^2 * s) for magnetometer.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this is an indication of
     * sensor noise.
     */
    val psdZ
        get() = if (resultAvailable) {
            noiseEstimator.psdZ
        } else {
            null
        }

    /**
     * Gets sensor noise root PSD (Power Spectral Density) on x axis expressed in
     * (m * s^-1.5) for accelerometer, (rad * s^-0.5) for gyroscope or (T * s^0.5) for magnetometer.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this is an indication of
     * sensor noise.
     */
    val rootPsdX
        get() = if (resultAvailable) {
            noiseEstimator.rootPsdX
        } else {
            null
        }

    /**
     * Gets sensor noise root PSD (Power Spectral Density) on y axis expressed in
     * (m * s^-1.5) for accelerometer, (rad * s^-0.5) for gyroscope or (T * s^0.5) for magnetometer.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this is an indication of
     * sensor noise.
     */
    val rootPsdY
        get() = if (resultAvailable) {
            noiseEstimator.rootPsdY
        } else {
            null
        }

    /**
     * Gets sensor noise root PSD (Power Spectral Density) on z axis expressed in
     * (m * s^-1.5) for accelerometer, (rad * s^-0.5) for gyroscope or (T * s^0.5) for magnetometer.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this is an indication of
     * sensor noise.
     */
    val rootPsdZ
        get() = if (resultAvailable) {
            noiseEstimator.rootPsdZ
        } else {
            null
        }

    /**
     * Gets average sensor measurement noise PSD (Power Spectral Density) expressed in
     * (m^2 * s^-3) for accelerometer, (rad^2/s) for gyroscope or (T^2 * s) for magnetometer.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this is an indication of
     * sensor noise.
     */
    val averageNoisePsd
        get() = if (resultAvailable) {
            noiseEstimator.avgNoisePsd
        } else {
            null
        }

    /**
     * Gets norm of sensor noise root PSD (Power Spectral Density) expressed in (m * s^-1.5) for
     * accelerometer, (rad * s^-0.5) for gyroscope or (T * s^0.5) for magnetometer.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     * If device remained completely static during estimation, this is an indication of
     * sensor noise.
     */
    val noiseRootPsdNorm
        get() = if (resultAvailable) {
            noiseEstimator.noiseRootPsdNorm
        } else {
            null
        }

    /**
     * Gets average time interval between sensor measurements expressed in seconds (s).
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     */
    val averageTimeInterval
        get() = if (resultAvailable) {
            timeIntervalEstimator.averageTimeInterval
        } else {
            null
        }

    /**
     * Gets average time interval between sensor measurements.
     * This is only available when estimation completes successfully and [resultAvailable] is true.
     */
    val averageTimeIntervalAsTime
        get() = if (resultAvailable) {
            timeIntervalEstimator.averageTimeIntervalAsTime
        } else {
            null
        }

    /**
     * Gets average time interval between sensor measurements.
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
     * Gets estimated variance of time interval between sensor measurements expressed in
     * squared seconds (s^2).
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
     * Gets estimated standard deviation of time interval between sensor measurements
     * expressed in seconds (s).
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
     * Gets estimated standard deviation of time interval between sensor measurements.
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
     * Gets estimated standard deviation of time interval between sensor measurements.
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
     * Starts collection of sensor measurements.
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
     * Stops collection of sensor measurements.
     */
    fun stop() {
        collector.stop()
        running = false
    }

    /**
     * Handles a measurement triad.
     *
     * @param valueX value of x-axis measurement expressed in sensor unit.
     * @param valueY value of y-axis measurement expressed in sensor unit.
     * @param valueZ value of z-axis measurement expressed in sensor unit.
     * @param timestamp time in nanoseconds at which the measurement was made. Each measurement
     * will be monotonically increasing using the same time base as
     * [android.os.SystemClock.elapsedRealtimeNanos].
     * @param accuracy sensor accuracy.
     */
    protected fun handleMeasurement(
        valueX: Double,
        valueY: Double,
        valueZ: Double,
        timestamp: Long,
        accuracy: SensorAccuracy?
    ) {
        if (accuracy == SensorAccuracy.UNRELIABLE) {
            resultUnreliable = true
        }

        if (numberOfProcessedMeasurements == 0) {
            initialTimestampNanos = timestamp
        }

        noiseEstimator.addTriad(valueX, valueY, valueZ)

        val diff = timestamp - initialTimestampNanos
        val diffSeconds = TimeConverter.nanosecondToSecond(diff.toDouble())
        timeIntervalEstimator.addTimestamp(diffSeconds)

        endTimestampNanos = timestamp
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
        resultUnreliable = false
    }

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

    /**
     * Interface to notify when estimation completes.
     */
    interface OnEstimationCompletedListener<A : AccumulatedTriadEstimator<*, *, *, *, *, *>> {
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
    interface OnUnreliableListener<A : AccumulatedTriadEstimator<*, *, *, *, *, *>> {
        /**
         * Called when measurements become unreliable.
         *
         * @param estimator estimator that generated the event.
         */
        fun onUnreliable(estimator: A)
    }
}