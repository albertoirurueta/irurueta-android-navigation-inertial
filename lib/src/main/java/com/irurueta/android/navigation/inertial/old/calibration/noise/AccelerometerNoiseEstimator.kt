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
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.noise.AccumulatedAccelerationTriadNoiseEstimator
import com.irurueta.units.Acceleration
import com.irurueta.units.AccelerationUnit

/**
 * Estimates accelerometer noise.
 * This estimator takes a given number of accelerometer measurements during a given duration of time
 * to estimate accelerometer measurements average, standard deviation and variance, as well as
 * average time interval between measurements.
 * To be able to measure accelerometer noise, device should remain static so that average
 * accelerometer measurements are constant and their standard deviations reflect actual sensor
 * noise.
 * This estimator converts sensor measurements from device ENU coordinates to local plane NED
 * coordinates. Thus, all values referring to a given x-y-z coordinates refers to local plane
 * NED system of coordinates.
 *
 * @param context Android context.
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
 * @property measurementListener Listener to notify collected sensor measurements.
 * @throws IllegalArgumentException when either [maxSamples] or [maxDurationMillis] is negative.
 */
class AccelerometerNoiseEstimator(
    context: Context,
    val sensorType: AccelerometerSensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
    sensorDelay: SensorDelay = SensorDelay.FASTEST,
    maxSamples: Int = DEFAULT_MAX_SAMPLES,
    maxDurationMillis: Long = DEFAULT_MAX_DURATION_MILLIS,
    stopMode: StopMode = StopMode.MAX_SAMPLES_OR_DURATION,
    completedListener: OnEstimationCompletedListener<AccelerometerNoiseEstimator>? = null,
    unreliableListener: OnUnreliableListener<AccelerometerNoiseEstimator>? = null,
    var measurementListener: AccelerometerSensorCollector.OnMeasurementListener? = null
) : AccumulatedTriadEstimator<AccelerometerNoiseEstimator,
        AccumulatedAccelerationTriadNoiseEstimator, AccelerometerSensorCollector, AccelerationUnit,
        Acceleration, AccelerationTriad>(
    context, sensorDelay, maxSamples, maxDurationMillis,
    stopMode, completedListener, unreliableListener
) {
    /**
     * Triad containing samples converted from device ENU coordinates to local plane NED
     * coordinates.
     * This is reused for performance reasons.
     */
    override val triad = AccelerationTriad()

    /**
     * Internal noise estimator of accelerometer measurements.
     * This can be used to estimate statistics about accelerometer noise measurements.
     */
    override val noiseEstimator = AccumulatedAccelerationTriadNoiseEstimator()

    /**
     * Listener to handle accelerometer measurements.
     */
    private val accelerometerMeasurementListener =
        AccelerometerSensorCollector.OnMeasurementListener { ax, ay, az, bx, by, bz, timestamp, accuracy ->
            handleMeasurement(ax.toDouble(), ay.toDouble(), az.toDouble(), timestamp, accuracy)
            measurementListener?.onMeasurement(ax, ay, az, bx, by, bz, timestamp, accuracy)
        }

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