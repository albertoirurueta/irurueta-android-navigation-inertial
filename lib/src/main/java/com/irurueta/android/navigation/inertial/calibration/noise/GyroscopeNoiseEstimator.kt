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
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.navigation.inertial.calibration.noise.AccumulatedAngularSpeedTriadNoiseEstimator
import com.irurueta.units.AngularSpeed
import com.irurueta.units.AngularSpeedUnit

/**
 * Estimates gyroscope noise.
 * This estimator takes a given number of gyroscope measurements during a given duration of time
 * to estimate gyroscope measurements average, standard deviation and variance, as well as average
 * time interval between measurements.
 * To be able to measure gyroscope noise, device should remain static so that average gyroscope
 * measurements are constant and their standard deviations reflect actual sensor noise, otherwise
 * only norm values will be reliable.
 *
 * @param context Android context.
 * @property sensorType One of the supported gyroscope sensor types.
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
class GyroscopeNoiseEstimator(
    context: Context,
    val sensorType: GyroscopeSensorCollector.SensorType =
        GyroscopeSensorCollector.SensorType.GYROSCOPE,
    sensorDelay: SensorDelay = SensorDelay.FASTEST,
    maxSamples: Int = DEFAULT_MAX_SAMPLES,
    maxDurationMillis: Long = DEFAULT_MAX_DURATION_MILLIS,
    stopMode: StopMode = StopMode.MAX_SAMPLES_OR_DURATION,
    completedListener: OnEstimationCompletedListener<GyroscopeNoiseEstimator>? = null,
    unreliableListener: OnUnreliableListener<GyroscopeNoiseEstimator>? = null,
    var measurementListener: GyroscopeSensorCollector.OnMeasurementListener? = null
) : AccumulatedTriadEstimator<GyroscopeNoiseEstimator, AccumulatedAngularSpeedTriadNoiseEstimator,
        GyroscopeSensorCollector, AngularSpeedUnit, AngularSpeed, AngularSpeedTriad>(
    context,
    sensorDelay,
    maxSamples,
    maxDurationMillis,
    stopMode,
    completedListener,
    unreliableListener
) {
    /**
     * Internal noise estimator of gyroscope measurements.
     * This can be used to estimate statistics about gyroscope noise measurements.
     */
    override val noiseEstimator = AccumulatedAngularSpeedTriadNoiseEstimator()

    /**
     * Listener to handle gyroscope measurements.
     */
    private val gyroscopeMeasurementListener =
        GyroscopeSensorCollector.OnMeasurementListener { wx, wy, wz, bx, by, bz, timestamp, accuracy ->
            handleMeasurement(wx.toDouble(), wy.toDouble(), wz.toDouble(), timestamp, accuracy)
            measurementListener?.onMeasurement(wx, wy, wz, bx, by, bz, timestamp, accuracy)
        }

    /**
     * Collector for gyroscope measurements.
     */
    override val collector = GyroscopeSensorCollector(
        context,
        sensorType,
        sensorDelay,
        gyroscopeMeasurementListener,
        accuracyChangedListener
    )
}