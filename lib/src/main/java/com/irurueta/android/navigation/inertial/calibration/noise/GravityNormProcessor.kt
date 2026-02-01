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

import com.irurueta.android.navigation.inertial.collectors.measurements.GravitySensorMeasurement
import com.irurueta.navigation.inertial.calibration.noise.AccumulatedAccelerationMeasurementNoiseEstimator
import com.irurueta.units.Acceleration
import com.irurueta.units.AccelerationUnit

/**
 * Estimates gravity norm.
 * This processor takes a given number of measurements during a given duration of time
 * to estimate gravity norm average, standard deviation and variance, as well as average time
 * interval between measurements.
 * For best accuracy of estimated results, device should remain static while data is being
 * processed.
 *
 * @param maxSamples Maximum number of samples to take into account before completion. This is
 * only taken into account if using either [StopMode.MAX_SAMPLES_ONLY] or
 * [StopMode.MAX_SAMPLES_OR_DURATION].
 * @param maxDurationMillis Maximum duration expressed in milliseconds to take into account
 * before completion. This is only taken into account if using either [StopMode.MAX_DURATION_ONLY] or
 * [StopMode.MAX_SAMPLES_OR_DURATION].
 * @param stopMode Determines when this processor will consider its estimation completed.
 */
class GravityNormProcessor(
    maxSamples: Int = DEFAULT_MAX_SAMPLES,
    maxDurationMillis: Long = DEFAULT_MAX_DURATION_MILLIS,
    stopMode: StopMode = StopMode.MAX_SAMPLES_OR_DURATION
) : AccumulatedMeasurementProcessor<GravityNormProcessor,
        AccumulatedAccelerationMeasurementNoiseEstimator,
        AccelerationUnit, Acceleration, GravitySensorMeasurement>(
    maxSamples, maxDurationMillis, stopMode
) {
    /**
     * Internal noise estimator of acceleration magnitude measurements.
     * This can be used to estimate statistics about a given measurement magnitude.
     */
    override val noiseEstimator = AccumulatedAccelerationMeasurementNoiseEstimator()

    /**
     * Processes a new measurement.
     *
     * @param measurement new measurement to be processed.
     * @return true if processed measurement was taken into account, false otherwise.
     */
    override fun process(measurement: GravitySensorMeasurement): Boolean {
        return handleMeasurement(measurement.toNorm(), measurement.timestamp)
    }
}