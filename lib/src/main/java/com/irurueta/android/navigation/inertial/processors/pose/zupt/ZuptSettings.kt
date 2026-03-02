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

/**
 * Contains settings for ZUPT processor creation.
 *
 * @param location current device location. If provided, precise expected gravity is estimated,
 * otherwise default Earth gravity is used.
 * @param windowNanoseconds amount of time to take measurements into account to compute variance
 * and average of accelerometer and gyroscope measurements. Value is expressed in nanoseconds.
 * @param gravityThreshold threshold to determine whether sensed accelerometer specific force is
 * close to gravity in magnitude. Typical values range from 0.1 to 0.3 m/s^2. Default value is
 * 0.2 m/se2.
 * @param accelerometerVarianceThreshold threshold to determine whether sensed accelerometer
 * variance is close to zero. Typical values range from 0.01 to 0.05 (m/s^2)^2. Default value is
 * 0.02 (m/s^2)^2.
 * @param gyroscopeVarianceThreshold threshold to determine whether sensed gyroscope variance is
 * close to zero. Typical values range from 1e-4 to 5e-4 (rad/s)^2. Default value is 2e-4 (rad/s)^
 * @param accelerometerNoiseVariance accelerometer sensor noise variance expressed in (m/s^2)^2.
 * Typical values range from 4e-4 to 6e-3 (m/s^2)^2. Default value is 1.6e-3.
 * @param gyroscopeNoiseVariance gyroscope sensor noise variance expressed in (rad/s)^2.
 * Typical values range from 4e-6 to 1e-4 (rad/s)^2. Default value is 2.5e-5.
 * @param accelerometerFactor accelerometer factor. Typical values range from 2 to 4.
 * Default value is 3.0.
 * @param gyroscopeFactor gyroscope factor. Typical values range from 2 to 6.
 * Default value is 4.0.
 * @param gravityWeight weight related to gravity to compute soft ZUPT score.
 * Gravity, accelerometer and gyroscope scores are weighted according to this value.
 * @param accelerometerWeight weight related to gyroscope to compute soft ZUPT score.
 * Gravity, accelerometer and gyroscope scores are weighted according to this value.
 * @param gyroscopeWeight weight related to gyroscope to compute soft ZUPT score.
 * Gravity, accelerometer and gyroscope scores are weighted according to this value.
 * @param gravityNormalizationFactor gravity normalization factor.
 * @param accelerometerNormalizationFactor accelerometer normalization factor.
 * @param gyroscopeNormalizationFactor gyroscope normalization factor.
 */
data class ZuptSettings(
    val location: Location? = null,
    val windowNanoseconds: Long = ZuptProcessor.DEFAULT_WINDOW_NANOSECONDS,
    val gravityThreshold: Double = ThresholdZuptProcessor.DEFAULT_GRAVITY_THRESHOLD,
    val accelerometerVarianceThreshold: Double = NonAdaptiveZuptProcessor.DEFAULT_ACCELEROMETER_VARIANCE_THRESHOLD,
    val gyroscopeVarianceThreshold: Double = NonAdaptiveZuptProcessor.DEFAULT_GYROSCOPE_VARIANCE_THRESHOLD,
    val accelerometerNoiseVariance: Double = AdaptiveZuptProcessor.DEFAULT_ACCELEROMETER_NOISE_VARIANCE,
    val gyroscopeNoiseVariance: Double = AdaptiveZuptProcessor.DEFAULT_GYROSCOPE_NOISE_VARIANCE,
    val accelerometerFactor: Double = AdaptiveZuptProcessor.DEFAULT_ACCELEROMETER_FACTOR,
    val gyroscopeFactor: Double = AdaptiveZuptProcessor.DEFAULT_GYROSCOPE_FACTOR,
    val gravityWeight: Double = SoftZuptProcessor.DEFAULT_GRAVITY_WEIGHT,
    val accelerometerWeight: Double = SoftZuptProcessor.DEFAULT_ACCELEROMETER_WEIGHT,
    val gyroscopeWeight: Double = SoftZuptProcessor.DEFAULT_GYROSCOPE_WEIGHT,
    val gravityNormalizationFactor: Double = SoftZuptProcessor.DEFAULT_GRAVITY_NORMALIZATION_FACTOR,
    val accelerometerNormalizationFactor: Double = SoftZuptProcessor.DEFAULT_ACCELEROMETER_VARIANCE_NORMALIZATION_FACTOR,
    val gyroscopeNormalizationFactor: Double = SoftZuptProcessor.DEFAULT_GYROSCOPE_VARIANCE_NORMALIZATION_FACTOR,
    val processorType: ZuptProcessorType = ZuptProcessorType.NONE
) {
    // Check requirements during initialization
    init {
        require(windowNanoseconds >= 0)
        require(gravityThreshold >= 0.0)
        require(accelerometerVarianceThreshold >= 0.0)
        require(gyroscopeVarianceThreshold >= 0.0)
        require(accelerometerNoiseVariance >= 0.0)
        require(gyroscopeNoiseVariance >= 0.0)
        require(accelerometerFactor >= 0.0)
        require(gyroscopeFactor >= 0.0)
        require(gravityWeight in 0.0..1.0)
        require(accelerometerWeight in 0.0..1.0)
        require(gyroscopeWeight in 0.0..1.0)
        require(gravityNormalizationFactor >= 0.0)
        require(accelerometerNormalizationFactor >= 0.0)
        require(gyroscopeNormalizationFactor >= 0.0)
    }
}