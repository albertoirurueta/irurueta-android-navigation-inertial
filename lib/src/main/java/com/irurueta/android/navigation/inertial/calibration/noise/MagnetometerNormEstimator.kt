/*
 * Copyright (C) 2022 Alberto Irurueta Carro (alberto@irurueta.com)
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

import android.content.Context
import com.irurueta.android.navigation.inertial.collectors.MagnetometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.MagnetometerSensorType
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.navigation.inertial.calibration.noise.AccumulatedMagneticFluxDensityMeasurementNoiseEstimator
import com.irurueta.units.MagneticFluxDensity
import com.irurueta.units.MagneticFluxDensityConverter
import com.irurueta.units.MagneticFluxDensityUnit
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * Estimates magnetometer measurement norm.
 * This estimator takes a given number of magnetometer measurements during a given duration of time
 * to estimate magnetometer norm average, standard deviation and variance, as well as average time
 * interval between measurements.
 * For best accuracy of estimated results, device should remain static while data is being
 * collected. In such case, average magnetometer norm should match expected Earth magnetic flux
 * density magnitude at current location and timestamp.
 *
 * @param context Android context.
 * @property sensorType One of the supported magnetometer sensor types.
 * @param sensorDelay Delay of sensor between samples.
 * @param maxSamples Maximum number of samples to take into account before completion. This is
 * only taken into account if using either [StopMode.MAX_SAMPLES_ONLY] or
 * [StopMode.MAX_SAMPLES_OR_DURATION].
 * @param maxDurationMillis Maximum duration expressed in milliseconds to take into account
 * before completion. This is only taken into account if using either
 * [StopMode.MAX_DURATION_ONLY] or [StopMode.MAX_SAMPLES_OR_DURATION].
 * @param stopMode Determines when this estimator will consider its estimation completed.
 * @param completedListener Listener to notify when estimation is complete.
 * @param unreliableListener Listener to notify when sensor becomes unreliable, and thus,
 * estimation must be discarded.
 * @property measurementListener Listener to notify collected sensor measurement.
 * @throws IllegalArgumentException when either [maxSamples] or [maxDurationMillis] is negative.
 */
class MagnetometerNormEstimator(
    context: Context,
    val sensorType: MagnetometerSensorType = MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
    sensorDelay: SensorDelay = SensorDelay.FASTEST,
    maxSamples: Int = DEFAULT_MAX_SAMPLES,
    maxDurationMillis: Long = DEFAULT_MAX_DURATION_MILLIS,
    stopMode: StopMode = StopMode.MAX_SAMPLES_OR_DURATION,
    completedListener: OnEstimationCompletedListener<MagnetometerNormEstimator>? = null,
    unreliableListener: OnUnreliableListener<MagnetometerNormEstimator>? = null,
    var measurementListener: MagnetometerSensorCollector.OnMeasurementListener? = null
) : AccumulatedMeasurementEstimator<MagnetometerNormEstimator,
        AccumulatedMagneticFluxDensityMeasurementNoiseEstimator, MagnetometerSensorCollector,
        MagneticFluxDensityUnit, MagneticFluxDensity>(
    context,
    sensorDelay,
    maxSamples,
    maxDurationMillis,
    stopMode,
    completedListener,
    unreliableListener
) {
    /**
     * Listener to handle magnetometer measurements.
     */
    private val magnetometerMeasurementListener =
        MagnetometerSensorCollector.OnMeasurementListener { bx, by, bz, hardIronX, hardIronY, hardIronZ, timestamp, accuracy ->
            val norm =
                sqrt(bx.toDouble().pow(2.0) + by.toDouble().pow(2.0) + bz.toDouble().pow(2.0))
            val normT = MagneticFluxDensityConverter.microTeslaToTesla(norm)
            handleMeasurement(normT, timestamp, accuracy)
            measurementListener?.onMeasurement(
                bx,
                by,
                bz,
                hardIronX,
                hardIronY,
                hardIronZ,
                timestamp,
                accuracy
            )
        }

    /**
     * Internal noise estimator of magnetometer magnitude measurements.
     * This can be used to estimate statistics about a given measurement magnitude.
     */
    override val noiseEstimator = AccumulatedMagneticFluxDensityMeasurementNoiseEstimator()

    /**
     * Collector for magnetometer measurements.
     */
    override val collector = MagnetometerSensorCollector(
        context,
        sensorType,
        sensorDelay,
        magnetometerMeasurementListener,
        accuracyChangedListener
    )
}