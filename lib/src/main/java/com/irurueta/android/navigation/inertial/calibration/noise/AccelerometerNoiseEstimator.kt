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
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.noise.AccumulatedAccelerationTriadNoiseEstimator
import com.irurueta.units.*

/**
 * Estimates accelerometer noise.
 * This estimator takes a given number of accelerometer measurements during a given duration of time
 * to estimate accelerometer measurements average, standard deviation and variance, as well as
 * average time interval between measurements.
 * To be able to measure accelerometer noise, device should remain static so that average
 * accelerometer measurements are constant and their standard deviations reflect actual sensor
 * noise.
 *
 * @param context Android context.
 * @property sensorType One of the supported accelerometer sensor types.
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
 * @throws IllegalArgumentException when either [maxSamples] or [maxDurationMillis] is negative.
 */
class AccelerometerNoiseEstimator(
    context: Context,
    val sensorType: AccelerometerSensorCollector.SensorType =
        AccelerometerSensorCollector.SensorType.ACCELEROMETER,
    sensorDelay: SensorDelay = SensorDelay.FASTEST,
    maxSamples: Int = DEFAULT_MAX_SAMPLES,
    maxDurationMillis: Long = DEFAULT_MAX_DURATION_MILLIS,
    stopMode: StopMode = StopMode.MAX_SAMPLES_OR_DURATION,
    completedListener: OnEstimationCompletedListener<AccelerometerNoiseEstimator>? = null,
    unreliableListener: OnUnreliableListener<AccelerometerNoiseEstimator>? = null
) : AccumulatedTriadEstimator<AccelerometerNoiseEstimator,
        AccumulatedAccelerationTriadNoiseEstimator, AccelerometerSensorCollector, AccelerationUnit,
        Acceleration, AccelerationTriad>(
    context, sensorDelay, maxSamples, maxDurationMillis,
    stopMode, completedListener, unreliableListener
) {
    /**
     * Internal noise estimator of accelerometer measurements.
     * This can be used to estimate statistics about accelerometer noise measurements.
     */
    override val noiseEstimator = AccumulatedAccelerationTriadNoiseEstimator()

    /**
     * Listener to handle accelerometer measurements.
     */
    private val measurementListener =
        object : AccelerometerSensorCollector.OnMeasurementListener {
            override fun onMeasurement(
                ax: Float,
                ay: Float,
                az: Float,
                bx: Float?,
                by: Float?,
                bz: Float?,
                timestamp: Long,
                accuracy: SensorAccuracy?
            ) {
                handleMeasurement(ax.toDouble(), ay.toDouble(), az.toDouble(), timestamp, accuracy)
            }
        }

    /**
     * Collector for accelerometer measurements.
     */
    override val collector = AccelerometerSensorCollector(
        context,
        sensorType,
        sensorDelay,
        measurementListener,
        accuracyChangedListener
    )
}