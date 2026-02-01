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
package com.irurueta.android.navigation.inertial.old.calibration.noise

import android.content.Context
import com.irurueta.android.navigation.inertial.calibration.noise.StopMode
import com.irurueta.android.navigation.inertial.old.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.navigation.inertial.calibration.noise.AccumulatedAccelerationMeasurementNoiseEstimator
import com.irurueta.units.Acceleration
import com.irurueta.units.AccelerationUnit
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * Estimates accelerometer norm.
 * This estimator takes a given number of measurements during a given duration of time
 * to estimate accelerometer norm average, standard deviation and variance, as well as average time
 * interval between measurements.
 * For best accuracy of estimated results, device should remain static while data is being
 * collected. In such case, average accelerometer norm should match gravity norm at current
 * location.
 *
 * @param context Android context
 * @property sensorType One of the supported accelerometer sensor types.
 * @param sensorDelay Delay of sensor between samples.
 * @param maxSamples Maximum number of samples to take into account before completion. This is
 * only taken into account if using either [com.irurueta.android.navigation.inertial.calibration.noise.StopMode.MAX_SAMPLES_ONLY] or
 * [com.irurueta.android.navigation.inertial.calibration.noise.StopMode.MAX_SAMPLES_OR_DURATION].
 * @param maxDurationMillis Maximum duration expressed in milliseconds to take into account
 * before completion. This is only taken into account if using either
 * [com.irurueta.android.navigation.inertial.calibration.noise.StopMode.MAX_DURATION_ONLY] or [com.irurueta.android.navigation.inertial.calibration.noise.StopMode.MAX_SAMPLES_OR_DURATION].
 * @param stopMode Determines when this estimator will consider its estimation completed.
 * @param completedListener Listener to notify when estimation is complete.
 * @param unreliableListener Listener to notify when sensor becomes unreliable, and thus,
 * estimation must be discarded.
 * @param measurementListener Listener to notify collected sensor measurements.
 * @throws IllegalArgumentException when either [maxSamples] or [maxDurationMillis] is negative.
 */
class AccelerometerNormEstimator(
    context: Context,
    val sensorType: AccelerometerSensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
    sensorDelay: SensorDelay = SensorDelay.FASTEST,
    maxSamples: Int = DEFAULT_MAX_SAMPLES,
    maxDurationMillis: Long = DEFAULT_MAX_DURATION_MILLIS,
    stopMode: StopMode = StopMode.MAX_SAMPLES_OR_DURATION,
    completedListener: OnEstimationCompletedListener<AccelerometerNormEstimator>? = null,
    unreliableListener: OnUnreliableListener<AccelerometerNormEstimator>? = null,
    var measurementListener: AccelerometerSensorCollector.OnMeasurementListener? = null
) : AccumulatedMeasurementEstimator<AccelerometerNormEstimator,
        AccumulatedAccelerationMeasurementNoiseEstimator, AccelerometerSensorCollector,
        AccelerationUnit, Acceleration>(
    context,
    sensorDelay,
    maxSamples,
    maxDurationMillis,
    stopMode,
    completedListener,
    unreliableListener
) {

    /**
     * Listener to handle accelerometer measurements.
     */
    private val accelerometerMeasurementListener =
        AccelerometerSensorCollector.OnMeasurementListener { ax, ay, az, bx, by, bz, timestamp, accuracy ->
            val a = sqrt(ax.toDouble().pow(2.0) + ay.toDouble().pow(2.0) + az.toDouble().pow(2.0))
            handleMeasurement(a, timestamp, accuracy)
            measurementListener?.onMeasurement(ax, ay, az, bx, by, bz, timestamp, accuracy)
        }

    /**
     * Internal noise estimator of acceleration magnitude measurements.
     * This can be used to estimate statistics about a given measurement magnitude.
     */
    override val noiseEstimator = AccumulatedAccelerationMeasurementNoiseEstimator()

    /**
     * Collector for accelerometer measurements.
     */
    override val collector = AccelerometerSensorCollector(
        context,
        sensorType,
        sensorDelay,
        accelerometerMeasurementListener,
        accuracyChangedListener
    )
}