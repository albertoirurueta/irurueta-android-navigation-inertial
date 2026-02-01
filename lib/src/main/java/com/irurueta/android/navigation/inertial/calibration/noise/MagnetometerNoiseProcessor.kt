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

import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorMeasurement
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad
import com.irurueta.navigation.inertial.calibration.noise.AccumulatedMagneticFluxDensityTriadNoiseEstimator
import com.irurueta.units.MagneticFluxDensity
import com.irurueta.units.MagneticFluxDensityUnit

class MagnetometerNoiseProcessor(
    maxSamples: Int = DEFAULT_MAX_SAMPLES,
    maxDurationMillis: Long = DEFAULT_MAX_DURATION_MILLIS,
    stopMode: StopMode = StopMode.MAX_SAMPLES_OR_DURATION
) : AccumulatedTriadProcessor<MagnetometerNoiseProcessor,
        AccumulatedMagneticFluxDensityTriadNoiseEstimator, MagneticFluxDensityUnit,
        MagneticFluxDensity, MagneticFluxDensityTriad, MagnetometerSensorMeasurement>(
    maxSamples, maxDurationMillis, stopMode
) {
    /**
     * Measurement converted to NED system coordinates.
     */
    override val nedMeasurement = MagnetometerSensorMeasurement()

    /**
     * Triad containing samples converted from device ENU coordinates to local plane NED
     * coordinates.
     * This is reused for performance reasons.
     */
    override val triad = MagneticFluxDensityTriad()

    /**
     * Internal noise estimator of magnitude measurements.
     * This can be used to estimate statistics about a given measurement magnitude.
     */
    override val noiseEstimator = AccumulatedMagneticFluxDensityTriadNoiseEstimator()
}