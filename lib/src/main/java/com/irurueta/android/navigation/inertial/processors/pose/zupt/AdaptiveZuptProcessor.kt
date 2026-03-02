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

/**
 * Adaptive ZUPT processor.
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
 * @param accelerometerNoiseVariance accelerometer sensor noise variance expressed in (m/s^2)^2.
 * Typical values range from 4e-4 to 6e-3 (m/s^2)^2. Default value is 1.6e-3.
 * @param gyroscopeNoiseVariance gyroscope sensor noise variance expressed in (rad/s)^2.
 * Typical values range from 4e-6 to 1e-4 (rad/s)^2. Default value is 2.5e-5.
 * @param accelerometerFactor accelerometer factor. Typical values range from 2 to 4.
 * Default value is 3.0.
 * @param gyroscopeFactor gyroscope factor. Typical values range from 2 to 6.
 * Default value is 4.0.
 * @param windowNanoseconds amount of time to take measurements into account to compute variance
 * and average of accelerometer and gyroscope measurements. Value is expressed in nanoseconds.
 */
class AdaptiveZuptProcessor<T>(
    location: Location? = null,
    gravityThreshold: Double = DEFAULT_GRAVITY_THRESHOLD,
    accelerometerNoiseVariance: Double = DEFAULT_ACCELEROMETER_NOISE_VARIANCE,
    gyroscopeNoiseVariance: Double = DEFAULT_GYROSCOPE_NOISE_VARIANCE,
    accelerometerFactor: Double = DEFAULT_ACCELEROMETER_FACTOR,
    gyroscopeFactor: Double = DEFAULT_GYROSCOPE_FACTOR,
    windowNanoseconds: Long = DEFAULT_WINDOW_NANOSECONDS
) : ThresholdZuptProcessor<T>(
    location,
    gravityThreshold,
    windowNanoseconds
) where T : SyncedSensorMeasurement<T>, T : WithAccelerometerSensorMeasurement {

    /**
     * Accelerometer sensor noise variance expressed in (m/s^2)^2.
     * Default value is 1.6e-3. Typical values range from 4e-4 to 6e-3 (m/s^2)^2.
     *
     * @throws IllegalArgumentException if provided value is negative.
     */
    var accelerometerNoiseVariance: Double = accelerometerNoiseVariance
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value >= 0.0)
            field = value
        }

    /**
     * Gyroscope sensor noise variance expressed in (rad/s)^2.
     * Default value is 2.5e-5. Typical values range from 4e-6 to 1e-4 (rad/s)^2.
     *
     * @throws IllegalArgumentException if provided value is negative.
     */
    var gyroscopeNoiseVariance: Double = gyroscopeNoiseVariance
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value >= 0.0)
            field = value
        }

    /**
     * Accelerometer factor.
     * Typical values range from 2 to 4.
     *
     * @throws IllegalArgumentException if provided value is negative.
     */
    var accelerometerFactor: Double = DEFAULT_ACCELEROMETER_FACTOR
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value >= 0.0)
            field = value
        }

    /**
     * Gyroscope factor.
     * Typical values range from 2 to 6.
     *
     * @throws IllegalArgumentException if provided value is negative.
     */
    var gyroscopeFactor: Double = DEFAULT_GYROSCOPE_FACTOR
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
    override val accelerometerVarianceThreshold: Double
        get() = accelerometerNoiseVariance * accelerometerFactor

    /**
     * Gets gyroscope variance threshold.
     *
     * @return gyroscope variance threshold.
     */
    override val gyroscopeVarianceThreshold: Double
        get() = gyroscopeNoiseVariance * gyroscopeFactor

    // Initialization
    init {
        this.accelerometerNoiseVariance = accelerometerNoiseVariance
        this.gyroscopeNoiseVariance = gyroscopeNoiseVariance
        this.accelerometerFactor = accelerometerFactor
        this.gyroscopeFactor = gyroscopeFactor
    }

    companion object {
        /**
         * Accelerometer sensor noise variance expressed in (m/s^2)^2.
         * Typical values range from 4e-4 to 6e-3 (m/s^2)^2.
         */
        const val DEFAULT_ACCELEROMETER_NOISE_VARIANCE = 1.6e-3

        /**
         * Gyroscope sensor noise variance expressed in (rad/s)^2.
         * Typical values range from 4e-6 to 1e-4 (rad/s)^2.
         */
        const val DEFAULT_GYROSCOPE_NOISE_VARIANCE = 2.5e-5

        /**
         * Accelerometer factor.
         * Typical values range from 2 to 4.
         */
        const val DEFAULT_ACCELEROMETER_FACTOR = 3.0

        /**
         * Gyroscope factor.
         * Typical values range from 2 to 6.
         */
        const val DEFAULT_GYROSCOPE_FACTOR = 4.0
    }
}