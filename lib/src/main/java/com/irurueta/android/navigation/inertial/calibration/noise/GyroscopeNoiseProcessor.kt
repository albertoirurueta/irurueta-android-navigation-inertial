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

import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorMeasurement
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.navigation.inertial.calibration.noise.AccumulatedAngularSpeedTriadNoiseEstimator
import com.irurueta.units.AngularSpeed
import com.irurueta.units.AngularSpeedUnit

/**
 * Processes gyroscope noise.
 * This processor takes a given number of gyroscope measurements during a given duration of time
 * to estimate gyroscope measurements average, standard deviation and variance, as well as average
 * time interval between measurements.
 * To be able to measure gyroscope noise, device should remain static so that average gyroscope
 * measurements are constant and their standard deviations reflect actual sensor noise, otherwise
 * only norm values will be reliable.
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
class GyroscopeNoiseProcessor(
    maxSamples: Int = DEFAULT_MAX_SAMPLES,
    maxDurationMillis: Long = DEFAULT_MAX_DURATION_MILLIS,
    stopMode: StopMode = StopMode.MAX_SAMPLES_OR_DURATION
) : AccumulatedTriadProcessor<GyroscopeNoiseProcessor, AccumulatedAngularSpeedTriadNoiseEstimator,
        AngularSpeedUnit, AngularSpeed, AngularSpeedTriad, GyroscopeSensorMeasurement>(
    maxSamples, maxDurationMillis, stopMode
) {
    /**
     * Measurement converted to NED system coordinates.
     */
    override val nedMeasurement = GyroscopeSensorMeasurement()

    /**
     * Triad containing samples converted from device ENU coordinates to local plane NED
     * coordinates.
     * This is reused for performance reasons.
     */
    override val triad = AngularSpeedTriad()

    /**
     * Internal noise estimator of magnitude measurements.
     * This can be used to estimate statistics about a given measurement magnitude.
     */
    override val noiseEstimator = AccumulatedAngularSpeedTriadNoiseEstimator()
}