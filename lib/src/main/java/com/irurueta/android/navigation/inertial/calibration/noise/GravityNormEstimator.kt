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
import com.irurueta.android.navigation.inertial.collectors.GravitySensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.collectors.measurements.GravitySensorMeasurement
import com.irurueta.units.Acceleration

/**
 * Estimates gravity norm.
 * This estimator takes a given number of measurements during a given duration of time
 * to estimate gravity norm average, standard deviation and variance, as well as average time
 * interval between measurements.
 * For best accuracy of estimated results, device should remain static while data is being
 * collected.
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
class GravityNormEstimator(
    context: Context,
    sensorDelay: SensorDelay = SensorDelay.FASTEST,
    maxSamples: Int = BaseAccumulatedProcessor.DEFAULT_MAX_SAMPLES,
    maxDurationMillis: Long = BaseAccumulatedProcessor.DEFAULT_MAX_DURATION_MILLIS,
    stopMode: StopMode = StopMode.MAX_SAMPLES_OR_DURATION,
    completedListener: OnEstimationCompletedListener<GravityNormEstimator>? = null,
    unreliableListener: OnUnreliableListener<GravityNormEstimator>? = null
) : AccumulatedMeasurementEstimator<GravityNormEstimator, GravityNormProcessor,
        GravitySensorCollector, Acceleration, GravitySensorMeasurement>(
    context,
    sensorDelay,
    completedListener,
    unreliableListener
) {
    /**
     * Internal processor that processes measurements.
     */
    override val processor = GravityNormProcessor(
        maxSamples,
        maxDurationMillis,
        stopMode
    )

    /**
     * Internal collector that collects measurements.
     */
    override val collector = GravitySensorCollector(
        context,
        sensorDelay,
        accuracyChangedListener,
        measurementListener,
    )
}