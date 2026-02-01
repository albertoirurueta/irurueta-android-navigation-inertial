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

import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.noise.AccumulatedAccelerationTriadNoiseEstimator
import com.irurueta.units.Acceleration
import com.irurueta.units.AccelerationUnit

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
 */
class AccelerometerNoiseProcessor(
    maxSamples: Int = DEFAULT_MAX_SAMPLES,
    maxDurationMillis: Long = DEFAULT_MAX_DURATION_MILLIS,
    stopMode: StopMode = StopMode.MAX_SAMPLES_OR_DURATION
) : AccumulatedTriadProcessor<AccelerometerNoiseProcessor,
        AccumulatedAccelerationTriadNoiseEstimator, AccelerationUnit, Acceleration,
        AccelerationTriad, AccelerometerSensorMeasurement>(
    maxSamples, maxDurationMillis, stopMode
) {
    /**
     * Measurement converted to NED system coordinates.
     */
    override val nedMeasurement = AccelerometerSensorMeasurement()

    /**
     * Triad containing samples converted from device ENU coordinates to local plane NED
     * coordinates.
     * This is reused for performance reasons.
     */
    override val triad = AccelerationTriad()

    /**
     * Internal noise estimator of magnitude measurements.
     * This can be used to estimate statistics about a given measurement magnitude.
     */
    override val noiseEstimator = AccumulatedAccelerationTriadNoiseEstimator()
}