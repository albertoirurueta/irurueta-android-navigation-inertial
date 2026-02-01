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

import android.content.Context
import com.irurueta.android.navigation.inertial.collectors.MagnetometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorType
import com.irurueta.units.MagneticFluxDensity

/**
 * Estimates magnetometer norm.
 * This estimator takes a given number of measurements during a given duration of time
 * to estimate magnetometer norm average, standard deviation and variance, as well as average time
 * interval between measurements.
 * For best accuracy of estimated results, device should remain static while data is being
 * collected. In such case (when there are no electromagnetic interferences), average magnetometer
 * norm should match Earth magnetic field norm at current location.
 *
 * @param context Android context.
 * @param sensorDelay Delay of sensor between samples.
 * @param maxSamples Maximum number of samples to take into account before completion. This is
 * only taken into account if using either [StopMode.MAX_SAMPLES_ONLY] or
 * [StopMode.MAX_SAMPLES_OR_DURATION].
 * @param maxDurationMillis Maximum duration expressed in milliseconds to take into account
 * before completion. This is only taken into account if using either [StopMode.MAX_DURATION_ONLY]
 * or [StopMode.MAX_SAMPLES_OR_DURATION].
 * @param stopMode Determines when this estimator will consider its estimation completed.
 * @param completedListener listener to notify when estimation completes.
 * @param unreliableListener listener to notify when measurements become unreliable.
 */
class MagnetometerNormEstimator(
    context: Context,
    val sensorType: MagnetometerSensorType = MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
    sensorDelay: SensorDelay = SensorDelay.FASTEST,
    maxSamples: Int = BaseAccumulatedProcessor.DEFAULT_MAX_SAMPLES,
    maxDurationMillis: Long = BaseAccumulatedProcessor.DEFAULT_MAX_DURATION_MILLIS,
    stopMode: StopMode = StopMode.MAX_SAMPLES_OR_DURATION,
    completedListener: OnEstimationCompletedListener<MagnetometerNormEstimator>? = null,
    unreliableListener: OnUnreliableListener<MagnetometerNormEstimator>? = null
) : AccumulatedMeasurementEstimator<MagnetometerNormEstimator, MagnetometerNormProcessor,
        MagnetometerSensorCollector, MagneticFluxDensity, MagnetometerSensorMeasurement>(
    context,
    sensorDelay,
    completedListener,
    unreliableListener
) {
    /**
     * Internal processor that processes measurements.
     */
    override val processor = MagnetometerNormProcessor(
        maxSamples,
        maxDurationMillis,
        stopMode
    )

    /**
     * Internal collector that collects measurements.
     */
    override val collector = MagnetometerSensorCollector(
        context,
        sensorType,
        sensorDelay,
        accuracyChangedListener,
        measurementListener,
    )
}