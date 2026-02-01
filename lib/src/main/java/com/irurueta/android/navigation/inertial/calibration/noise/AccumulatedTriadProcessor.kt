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

import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.TriadConvertible
import com.irurueta.navigation.inertial.calibration.Triad
import com.irurueta.navigation.inertial.calibration.noise.AccumulatedTriadNoiseEstimator
import com.irurueta.units.Measurement

/**
 * Processes the accumulated mean values of a triad of measurements.
 * This processor takes a given number of measurement triads (acceleration, angular speed,
 * magnetic field, etc) during a given duration of time to estimate measurement averages, standard
 * deviations, variances, as well as average time interval between measurements.
 * For best accuracy of estimated results, device should remain static while data is being
 * collected.
 * This processor converts sensor measurements from device ENU coordinates to local plane NED
 * coordinates. Thus, all values referring to a given x-y-z coordinates refers to local plane
 * NED system of coordinates.
 *
 * @param maxSamples Maximum number of samples to take into account before completion. This is
 * only taken into account if using either [StopMode.MAX_SAMPLES_ONLY] or
 * [StopMode.MAX_SAMPLES_OR_DURATION].
 * @param maxDurationMillis Maximum duration expressed in milliseconds to take into account
 * before completion. This is only taken into account if using either [StopMode.MAX_DURATION_ONLY] or
 * [StopMode.MAX_SAMPLES_OR_DURATION].
 * @param stopMode Determines when this processor will consider its estimation completed.
 * @param P Type of processor.
 * @param N Type of noise estimator.
 * @param U Type of measurement unit.
 * @param M Type of measurement.
 * @param T Type of triad.
 * @param SM Type of sensor measurement.
 */
abstract class AccumulatedTriadProcessor<P, N, U, M, T, SM>(
    maxSamples: Int = DEFAULT_MAX_SAMPLES,
    maxDurationMillis: Long = DEFAULT_MAX_DURATION_MILLIS,
    stopMode: StopMode = StopMode.MAX_SAMPLES_OR_DURATION
) : BaseAccumulatedProcessor(maxSamples, maxDurationMillis, stopMode)
        where P : AccumulatedTriadProcessor<P, N, U, M, T, SM>,
              N : AccumulatedTriadNoiseEstimator<U, M, T, *, *>,
              U : Enum<*>,
              M : Measurement<U>,
              T : Triad<U, M>,
              SM : SensorMeasurement<SM>,
              SM : TriadConvertible<T> {

    /**
     * Measurement converted to NED system coordinates.
     */
    protected abstract val nedMeasurement: SM

    /**
     * Triad containing samples converted from device ENU coordinates to local plane NED
     * coordinates.
     * This is reused for performance reasons.
     */
    protected abstract val triad: T

    /**
     * Internal noise estimator of magnitude measurements.
     * This can be used to estimate statistics about a given measurement magnitude.
     */
    protected abstract val noiseEstimator: N

    /**
     * Indicates whether estimated result is unreliable or not.
     */
    var resultUnreliable: Boolean = false
        protected set

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
     * Processes a new measurement.
     *
     * @param measurement measurement to be processed.
     * @return true if estimation completed, false otherwise.
     */
    fun process(measurement: SM): Boolean {
        if (measurement.accuracy == SensorAccuracy.UNRELIABLE) {
            resultUnreliable = true
        }

        if (numberOfProcessedMeasurements == 0L) {
            initialTimestampNanos = measurement.timestamp
        }

        // sensors in Android are provided in device ENU coordinates, so we need to convert
        // them to local plane NED coordinates
        measurement.toNed(nedMeasurement)
        nedMeasurement.toTriad(triad)

        // when triad is processed it is converted to default unit when needed
        noiseEstimator.addTriad(triad)

        handleTimestamp(measurement.timestamp)
        numberOfProcessedMeasurements++

        if (isComplete()) {
            // once time interval has been estimated, it is set into noise estimator so that
            // PSD values can be correctly estimated.
            noiseEstimator.timeInterval = timeIntervalEstimator.averageTimeInterval
            resultAvailable = true
            return true
        }

        return false
    }

    /**
     * Resets internal estimators.
     */
    override fun reset() {
        noiseEstimator.reset()
        noiseEstimator.timeInterval = 0.0

        super.reset()
    }
}