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
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad
import com.irurueta.navigation.inertial.calibration.noise.AccumulatedMagneticFluxDensityTriadNoiseEstimator
import com.irurueta.units.MagneticFluxDensity
import com.irurueta.units.MagneticFluxDensityConverter
import com.irurueta.units.MagneticFluxDensityUnit

/**
 * Estimates magnetometer noise.
 * This estimator takes a given number of magnetometer measurements during a given duration of time
 * to estimate magnetometer measurements average, standard deviation and variance, as well as
 * average time interval between measurements.
 * To be able to measure magnetometer noise, device should remain static so that average
 * magnetometer measurements are constant and their standard deviation reflect actual sensor noise,
 * otherwise only norm values will be reliable.
 * This estimator converts sensor measurements from device ENU coordinates to local plane NED
 * coordinates. Thus, all values referring to a given x-y-z coordinates refers to local plane
 * NED system of coordinates.
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
 * @property measurementListener Listener to notify collected sensor measurements.
 * @throws IllegalArgumentException when either [maxSamples] or [maxDurationMillis] is negative.
 */
class MagnetometerNoiseEstimator(
    context: Context,
    val sensorType: MagnetometerSensorCollector.SensorType =
        MagnetometerSensorCollector.SensorType.MAGNETOMETER,
    sensorDelay: SensorDelay = SensorDelay.FASTEST,
    maxSamples: Int = DEFAULT_MAX_SAMPLES,
    maxDurationMillis: Long = DEFAULT_MAX_DURATION_MILLIS,
    stopMode: StopMode = StopMode.MAX_SAMPLES_OR_DURATION,
    completedListener: OnEstimationCompletedListener<MagnetometerNoiseEstimator>? = null,
    unreliableListener: OnUnreliableListener<MagnetometerNoiseEstimator>? = null,
    var measurementListener: MagnetometerSensorCollector.OnMeasurementListener? = null
) : AccumulatedTriadEstimator<MagnetometerNoiseEstimator,
        AccumulatedMagneticFluxDensityTriadNoiseEstimator, MagnetometerSensorCollector,
        MagneticFluxDensityUnit, MagneticFluxDensity, MagneticFluxDensityTriad>(
    context,
    sensorDelay,
    maxSamples,
    maxDurationMillis,
    stopMode,
    completedListener,
    unreliableListener
) {
    /**
     * Triad containing samples converted from device ENU coordinates to local plane NED
     * coordinates.
     * This is reused for performance reasons.
     */
    override val triad = MagneticFluxDensityTriad()

    /**
     * Internal noise estimator of magnetometer measurements.
     * This can be used to estimate statistics about magnetometer noise measurements.
     */
    override val noiseEstimator = AccumulatedMagneticFluxDensityTriadNoiseEstimator()

    /**
     * Listener to handle magnetometer measurements.
     */
    private val magnetometerMeasurementListener =
        MagnetometerSensorCollector.OnMeasurementListener { bx, by, bz, hardIronX, hardIronY, hardIronZ, timestamp, accuracy ->
            val bxT = MagneticFluxDensityConverter.microTeslaToTesla(bx.toDouble())
            val byT = MagneticFluxDensityConverter.microTeslaToTesla(by.toDouble())
            val bzT = MagneticFluxDensityConverter.microTeslaToTesla(bz.toDouble())

            handleMeasurement(bxT, byT, bzT, timestamp, accuracy)
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