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
import com.irurueta.android.navigation.inertial.collectors.GravitySensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.navigation.inertial.calibration.noise.AccumulatedAccelerationMeasurementNoiseEstimator
import com.irurueta.units.Acceleration
import com.irurueta.units.AccelerationUnit

/**
 * Estimates gravity norm.
 * This estimator takes a given number of measurements during a given duration of time
 * to estimate gravity norm average, standard deviation and variance, as well as average time
 * interval between measurements.
 * For best accuracy of estimated results, device should remain static while data is being
 * collected.
 *
 * @property context Android context
 * @property sensorDelay Delay of sensor between samples.
 * @param maxSamples Maximum number of samples to take into account before completion. This is
 * only taken into account if using either [StopMode.MAX_SAMPLES_ONLY] or
 * [StopMode.MAX_SAMPLES_OR_DURATION].
 * @param maxDurationMillis Maximum duration expressed in milliseconds to take into account
 * before completion. This is only taken into account if using either
 * [StopMode.MAX_DURATION_ONLY] or [StopMode.MAX_SAMPLES_OR_DURATION].
 * @property stopMode Determines when this estimator will consider its estimation completed.
 * @property completedListener Listener to notify when estimation is complete.
 * @property unreliableListener Listener to notify when sensor becomes unreliable, and thus,
 * estimation must be discarded.
 * @property measurementListener Listener to notify collected sensor measurements.
 * @throws IllegalArgumentException when either [maxSamples] or [maxDurationMillis] is negative.
 */
class GravityNormEstimator(
    context: Context,
    sensorDelay: SensorDelay = SensorDelay.FASTEST,
    maxSamples: Int = DEFAULT_MAX_SAMPLES,
    maxDurationMillis: Long = DEFAULT_MAX_DURATION_MILLIS,
    stopMode: StopMode = StopMode.MAX_SAMPLES_OR_DURATION,
    completedListener: OnEstimationCompletedListener<GravityNormEstimator>? = null,
    unreliableListener: OnUnreliableListener<GravityNormEstimator>? = null,
    var measurementListener: GravitySensorCollector.OnMeasurementListener? = null
) : AccumulatedMeasurementEstimator<GravityNormEstimator,
        AccumulatedAccelerationMeasurementNoiseEstimator, GravitySensorCollector, AccelerationUnit,
        Acceleration>(
    context,
    sensorDelay,
    maxSamples,
    maxDurationMillis,
    stopMode,
    completedListener,
    unreliableListener
) {
    /**
     * Listener to handle gravity measurements.
     */
    private val gravityMeasurementListener =
        GravitySensorCollector.OnMeasurementListener { gx, gy, gz, g, timestamp, accuracy ->
            handleMeasurement(g, timestamp, accuracy)
            measurementListener?.onMeasurement(gx, gy, gz, g, timestamp, accuracy)
        }

    /**
     * Internal noise estimator of acceleration magnitude measurements.
     * This can be used to estimate statistics about a given measurement magnitude.
     */
    override val noiseEstimator = AccumulatedAccelerationMeasurementNoiseEstimator()

    /**
     * Collector for gravity measurements.
     */
    override val collector = GravitySensorCollector(
        context,
        sensorDelay,
        gravityMeasurementListener,
        accuracyChangedListener
    )
}