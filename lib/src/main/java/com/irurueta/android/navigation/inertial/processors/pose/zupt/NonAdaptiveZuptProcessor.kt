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
 * Non-adaptive ZUPT processor.
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
 * @param accelerometerVarianceThreshold threshold to determine whether sensed accelerometer
 * variance is close to zero. Typical values range from 0.01 to 0.05 (m/s^2)^2. Default value is
 * 0.02 (m/s^2)^2.
 * @param gyroscopeVarianceThreshold threshold to determine whether sensed gyroscope variance is
 * close to zero. Typical values range from 1e-4 to 5e-4 (rad/s)^2. Default value is 2e-4 (rad/s)^
 * @param windowNanoseconds amount of time to take measurements into account to compute variance
 * and average of accelerometer and gyroscope measurements. Value is expressed in nanoseconds.
 */
class NonAdaptiveZuptProcessor<T>(
    location: Location? = null,
    gravityThreshold: Double = DEFAULT_GRAVITY_THRESHOLD,
    accelerometerVarianceThreshold: Double = DEFAULT_ACCELEROMETER_VARIANCE_THRESHOLD,
    gyroscopeVarianceThreshold: Double = DEFAULT_GYROSCOPE_VARIANCE_THRESHOLD,
    windowNanoseconds: Long = DEFAULT_WINDOW_NANOSECONDS
) : ThresholdZuptProcessor<T>(
    location,
    gravityThreshold,
    windowNanoseconds
) where T : SyncedSensorMeasurement<T>, T : WithAccelerometerSensorMeasurement {

    /**
     * Threshold to determine whether sensed accelerometer variance is close to zero.
     *
     * @throws IllegalArgumentException if provided value is negative.
     */
    override var accelerometerVarianceThreshold: Double = accelerometerVarianceThreshold
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value >= 0.0)
            field = value
        }

    /**
     * Threshold to determine whether sensed gyroscope variance is close to zero.
     *
     * @throws IllegalArgumentException if provided value is negative.
     */
    override var gyroscopeVarianceThreshold: Double = gyroscopeVarianceThreshold
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value >= 0.0)
            field = value
        }

    // Initialization
    init {
        this.accelerometerVarianceThreshold = accelerometerVarianceThreshold
        this.gyroscopeVarianceThreshold = gyroscopeVarianceThreshold
    }

    companion object {
        /**
         * Absolute threshold to determine when sensed accelerometer variance is close to zero.
         * Threshold is expressed in square meters per squared second (m/s^2)^2.
         * Typical values range from 0.01 to 0.05 (m/s^2)^2.
         */
        const val DEFAULT_ACCELEROMETER_VARIANCE_THRESHOLD = 0.02

        /**
         * Absolute threshold to determine when sensed gyroscope variance is close to zero.
         * Threshold is expressed in squared radians per second (rad/s)^2
         * Typical values range from 1e-4 to 5e-4 (rad/s)^2.
         */
        const val DEFAULT_GYROSCOPE_VARIANCE_THRESHOLD = 2e-4
    }
}