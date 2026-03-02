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
 * Soft ZUPT processor.
 * A ZUPT processor determines whether Velocity should be updated to zero depending on
 * certain conditions on measurements (related to accelerometer magnitude and accelerometer
 * and gyroscope variance).
 * Soft ZUPT scores range between 0.0 and 1.0. When accelerometer magnitude is close to gravity and
 * accelerometer and/or gyroscope variance is low, the closer ZUPT score is to 1.0.
 * Notice that this processor takes into account gyroscope variance only if available.
 *
 * @param location current device location. If provided, precise expected gravity is estimated,
 * otherwise default Earth gravity is used.
 * @param gravityWeight weight related to gravity to compute soft ZUPT score.
 * Gravity, accelerometer and gyroscope scores are weighted according to this value.
 * @param accelerometerWeight weight related to gyroscope to compute soft ZUPT score.
 * Gravity, accelerometer and gyroscope scores are weighted according to this value.
 * @param gyroscopeWeight weight related to gyroscope to compute soft ZUPT score.
 * Gravity, accelerometer and gyroscope scores are weighted according to this value.
 * @param gravityNormalizationFactor gravity normalization factor.
 * @param accelerometerNormalizationFactor accelerometer normalization factor.
 * @param gyroscopeNormalizationFactor gyroscope normalization factor.
 * @param windowNanoseconds amount of time to take measurements into account to compute variance
 * and average of accelerometer and gyroscope measurements. Value is expressed in nanoseconds.
 */
class SoftZuptProcessor<T>(
    location: Location? = null,
    gravityWeight: Double = DEFAULT_GRAVITY_WEIGHT,
    accelerometerWeight: Double = DEFAULT_ACCELEROMETER_WEIGHT,
    gyroscopeWeight: Double = DEFAULT_GYROSCOPE_WEIGHT,
    gravityNormalizationFactor: Double = DEFAULT_GRAVITY_NORMALIZATION_FACTOR,
    accelerometerNormalizationFactor: Double = DEFAULT_ACCELEROMETER_VARIANCE_NORMALIZATION_FACTOR,
    gyroscopeNormalizationFactor: Double = DEFAULT_GYROSCOPE_VARIANCE_NORMALIZATION_FACTOR,
    windowNanoseconds: Long = DEFAULT_WINDOW_NANOSECONDS
) : ZuptProcessor<T>(
    location,
    windowNanoseconds
) where T : SyncedSensorMeasurement<T>, T : WithAccelerometerSensorMeasurement {

    /**
     * Weight related to gravity to compute soft ZUPT score.
     * Gravity, accelerometer and gyroscope scores are weighted according to this value.
     * Weight value must be between 0.0 and 1.0 (both included)
     *
     * @throws IllegalArgumentException if provided value is not within 0.0 and 1.0
     */
    var gravityWeight: Double = gravityWeight
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value in 0.0..1.0)
            field = value
        }

    /**
     * Weight related to gyroscope to compute soft ZUPT score.
     * Gravity, accelerometer and gyroscope scores are weighted according to this value.
     * Weight value must be between 0.0 and 1.0 (both included)
     *
     * @throws IllegalArgumentException if provided value is not within 0.0 and 1.0
     */
    var accelerometerWeight: Double = accelerometerWeight
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value in 0.0..1.0)
            field = value
        }

    /**
     * Weight related to gyroscope to compute soft ZUPT score.
     * Gravity, accelerometer and gyroscope scores are weighted according to this value.
     * Weight value must be between 0.0 and 1.0 (both included)
     *
     * @throws IllegalArgumentException if provided value is not within 0.0 and 1.0
     */
    var gyroscopeWeight: Double = gyroscopeWeight
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value in 0.0..1.0)
            field = value
        }

    /**
     * Gravity normalization factor.
     *
     * @throws IllegalArgumentException if provided value is negative.
     */
    var gravityNormalizationFactor: Double = gravityNormalizationFactor
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value >= 0.0)
            field = value
        }

    /**
     * Accelerometer normalization factor.
     *
     * @throws IllegalArgumentException if provided value is negative.
     */
    var accelerometerNormalizationFactor: Double = accelerometerNormalizationFactor
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value >= 0.0)
            field = value
        }

    /**
     * Gyroscope normalization factor.
     *
     * @throws IllegalArgumentException if provided value is negative.
     */
    var gyroscopeNormalizationFactor: Double = gyroscopeNormalizationFactor
        @Throws(IllegalArgumentException::class)
        set(value) {
            require(value >= 0.0)
            field = value
        }

    /**
     * Evaluates accelerometer average magnitude and variance to determine whether ZUPT (Zero
     * Velocity Update) conditions apply or not.
     *
     * @return A value between 0.0 and 1.0. The closer to 1.0 the more likely to apply ZUPT.
     *
     */
    override fun evaluateAccelerometer(): Double {
        return (gravityWeight * gravityScore() +
                accelerometerWeight * accelerometerVarianceScore()) /
                (gravityWeight + accelerometerWeight)
    }

    /**
     * Evaluates accelerometer average magnitude and variance, and gyroscope variance to
     * determine whether ZUPT (Zero Velocity Update) conditions apply or not.
     *
     * @return A value between 0.0 and 1.0. The closer to 1.0 the more likely to apply ZUPT.
     */
    override fun evaluateAccelerometerAndGyroscope(): Double {
        return (gravityWeight * gravityScore() +
                accelerometerWeight * accelerometerVarianceScore() +
                gyroscopeWeight * gyroscopeVarianceScore()) /
                (gravityWeight + accelerometerWeight + gyroscopeWeight)
    }

    /**
     * Computes score related to accelerometer magnitude and gravity.,
     * The closer that accelerometer magnitude is to gravity, the higher the score will be.
     *
     * @return score as a value between 0.0 and 1.0.
     */
    fun gravityScore(): Double {
        val gravityNorm = expectedGravityNorm
        val avg = accelerationAverage ?: return 0.0
        val diff = abs(avg - gravityNorm)
        return clamp(1.0 - (diff / gravityNormalizationFactor))
    }

    /**
     * Computes score related to accelerometer variance.
     * The lower that accelerometer variance is, the higher the score will be.
     *
     * @return score as a value between 0.0 and 1.0.
     */
    fun accelerometerVarianceScore(): Double {
        val variance = accelerationVariance ?: return 0.0
        return clamp(1.0 - (variance / accelerometerNormalizationFactor))
    }

    /**
     * Computes score related to gyroscope variance.
     * The lower that gyroscope variance is, the higher the score will be.
     *
     * @return score as a value between 0.0 and 1.0.
     */
    fun gyroscopeVarianceScore(): Double {
        val variance = gyroscopeVariance ?: return 0.0
        return clamp(1.0 - (variance / gyroscopeNormalizationFactor))
    }

    /**
     * Clamps a value between 0.0 and 1.0.
     *
     * @param x value to clamp.
     * @return clamped value.
     */
    private fun clamp(x: Double): Double {
        if (x < 0.0) {
            return 0.0
        }
        if (x > 1.0) {
            return 1.0
        }
        return x
    }

    // Initialization
    init {
        this.gravityWeight = gravityWeight
        this.accelerometerWeight = accelerometerWeight
        this.gyroscopeWeight = gyroscopeWeight
        this.gravityNormalizationFactor = gravityNormalizationFactor
        this.accelerometerNormalizationFactor = accelerometerNormalizationFactor
        this.gyroscopeNormalizationFactor = gyroscopeNormalizationFactor
    }

    companion object {
        /**
         * Weight related to gravity to compute soft ZUPT score.
         */
        const val DEFAULT_GRAVITY_WEIGHT = 0.3

        const val DEFAULT_ACCELEROMETER_WEIGHT = 0.4

        const val DEFAULT_GYROSCOPE_WEIGHT = 0.3

        // Typical values 0.2 to 0.4 m/s^2
        const val DEFAULT_GRAVITY_NORMALIZATION_FACTOR = 0.3

        // Typical values 0.02 to 0.05 (m/s^2)^2
        const val DEFAULT_ACCELEROMETER_VARIANCE_NORMALIZATION_FACTOR = 0.03

        // Typical values 2e-4 to 5e-4 (rad/s)^2
        const val DEFAULT_GYROSCOPE_VARIANCE_NORMALIZATION_FACTOR = 3e-4
    }
}