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

package com.irurueta.android.navigation.inertial.processors.pose.zupt

import android.location.Location
import com.irurueta.android.navigation.inertial.collectors.measurements.SyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.WithAccelerometerSensorMeasurement
import kotlin.math.abs

/**
 * ZUPT processor based on accelerometer and gyroscope magnitude and variance thresholds.
 * A ZUPT processor determines whether Velocity should be updated to zero depending on
 * certain conditions on measurements (related to accelerometer magnitude and accelerometer and
 * gyroscope variance)
 * When accelerometer magnitude is close to gravity and accelerometer and/or gyroscope variance is
 * low, a ZUPT is performed.
 * Notice that this processor takes into account gyroscope variance only if available
 *
 * @param location current device location. If provided, precise expected gravity is estimated,
 * otherwise default Earth gravity is used.
 * @param gravityThreshold threshold to determine whether sensed accelerometer specific force is
 * close to gravity in magnitude. Typical values range from 0.1 to 0.3 m/s^2. Default value is
 * 0.2 m/se2.
 * @param windowNanoseconds amount of time to take measurements into account to compute variance
 * and average of accelerometer and gyroscope measurements. Value is expressed in nanoseconds.
 */
abstract class ThresholdZuptProcessor<T>(
    location: Location? = null,
    gravityThreshold: Double = DEFAULT_GRAVITY_THRESHOLD,
    windowNanoseconds: Long = DEFAULT_WINDOW_NANOSECONDS
) : ZuptProcessor<T>(
    location,
    windowNanoseconds
) where T : SyncedSensorMeasurement<T>, T : WithAccelerometerSensorMeasurement {

    /**
     * Threshold to determine whether sensed accelerometer specific force is close to gravity in
     * magnitude.
     *
     * @throws IllegalArgumentException if provided value is negative.
     */
    var gravityThreshold: Double = gravityThreshold
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value >= 0.0)
            field = value
        }

    /**
     * Gets accelerometer variance threshold.
     *
     * @return accelerometer variance threshold.
     */
    abstract val accelerometerVarianceThreshold: Double

    /**
     * Gets gyroscope variance threshold.
     *
     * @return gyroscope variance threshold.
     */
    abstract val gyroscopeVarianceThreshold: Double

    /**
     * Evaluates accelerometer average magnitude and variance to determine whether ZUPT (Zero
     * Velocity Update) conditions apply or not.
     *
     * @return 1.0 if ZUPT conditions apply, 0.0 otherwise.
     *
     */
    override fun evaluateAccelerometer() : Double {
        return if (isAccelerometerMagnitudeCloseToGravity() &&
            isAccelerometerVarianceCloseToZero()
        ) {
            1.0
        } else {
            0.0
        }
    }

    /**
     * Evaluates accelerometer average magnitude and variance, and gyroscope variance to
     * determine whether ZUPT (Zero Velocity Update) conditions apply or not.
     *
     * @return 1.0 if ZUPT conditions apply, 0.0 otherwise.
     */
    override fun evaluateAccelerometerAndGyroscope() : Double {
        return if (isAccelerometerMagnitudeCloseToGravity() &&
            isAccelerometerVarianceCloseToZero() &&
            isGyroscopeVarianceCloseToZero()
        ) {
            1.0
        } else {
            0.0
        }
    }

    /**
     * Checks whether accelerometer specific force is close to gravity in magnitude.
     *
     * @return true if close, false otherwise.
     */
    private fun isAccelerometerMagnitudeCloseToGravity(): Boolean {
        val gravityNorm = expectedGravityNorm
        val avg = accelerationAverage ?: return false
        val diff = abs(avg - gravityNorm)
        return (diff < gravityThreshold)
    }

    /**
     * Checks whether sensed accelerometer variance is close to zero.
     *
     * @return true if close, false otherwise.
     */
    private fun isAccelerometerVarianceCloseToZero(): Boolean {
        return ((accelerationVariance ?: Double.MAX_VALUE) < accelerometerVarianceThreshold)
    }

    /**
     * Checks whether sensed gyroscope variance is close to zero.
     *
     * @return true if close, false otherwise.
     */
    private fun isGyroscopeVarianceCloseToZero(): Boolean {
        return ((gyroscopeVariance ?: Double.MAX_VALUE) < gyroscopeVarianceThreshold)
    }

    // Initialization
    init {
        this.gravityThreshold = gravityThreshold
    }

    companion object {
        /**
         * Absolute threshold to determine when sensed accelerometer specific force is close to
         * gravity force in magnitude.
         * Threshold is expressed in meters per squared second (m/s^2).
         * Whenever acceleration is close to gravity magnitude by this amount, a ZUPT is returned
         * if acceleration and gyroscope variance is also low.
         * Typical values range from 0.1 to 0.3 m/s2
         */
        const val DEFAULT_GRAVITY_THRESHOLD = 0.2
    }
}