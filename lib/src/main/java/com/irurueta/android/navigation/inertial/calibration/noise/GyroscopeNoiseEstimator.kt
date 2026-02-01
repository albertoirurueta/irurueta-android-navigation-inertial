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
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorType
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
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
 * This estimator converts sensor measurements from device ENU coordinates to local plane NED
 * coordinates. Thus, all values referring to a given x-y-z coordinates refers to local plane
 * NED system of coordinates.
 *
 * @param context Android context.
 * @property sensorType One of the supported gyroscope sensor types.
 * @param sensorDelay Delay of sensor between samples.
 * @param maxSamples Maximum number of samples to take into account before completion. This is
 * only taken into account if using either [StopMode.MAX_SAMPLES_ONLY] or
 * [StopMode.MAX_SAMPLES_OR_DURATION].
 * @param maxDurationMillis Maximum duration expressed in milliseconds to take into account
 * before completion. This is only taken into account if using either [StopMode.MAX_DURATION_ONLY] or
 * [StopMode.MAX_SAMPLES_OR_DURATION].
 * @param stopMode Determines when this estimator will consider its estimation completed.
 * @param completedListener Listener to notify when estimation is complete.
 * @param unreliableListener Listener to notify when sensor becomes unreliable, and thus,
 * estimation must be discarded.
 * @throws IllegalArgumentException when either [maxSamples] or [maxDurationMillis] is negative.
 */
class GyroscopeNoiseEstimator(
    context: Context,
    val sensorType: GyroscopeSensorType = GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
    sensorDelay: SensorDelay = SensorDelay.FASTEST,
    maxSamples: Int = BaseAccumulatedProcessor.DEFAULT_MAX_SAMPLES,
    maxDurationMillis: Long = BaseAccumulatedProcessor.DEFAULT_MAX_DURATION_MILLIS,
    stopMode: StopMode = StopMode.MAX_SAMPLES_OR_DURATION,
    completedListener: OnEstimationCompletedListener<GyroscopeNoiseEstimator>? = null,
    unreliableListener: OnUnreliableListener<GyroscopeNoiseEstimator>? = null
) : AccumulatedTriadEstimator<GyroscopeNoiseEstimator, GyroscopeNoiseProcessor,
        GyroscopeSensorCollector, AngularSpeedUnit, AngularSpeed, AngularSpeedTriad,
        GyroscopeSensorMeasurement>(
    context,
    sensorDelay,
    completedListener,
    unreliableListener
) {
    /**
     * Internal processor that processes measurements.
     */
    override val processor = GyroscopeNoiseProcessor(
        maxSamples,
        maxDurationMillis,
        stopMode
    )

    /**
     * Internal collector that collects measurements.
     */
    override val collector = GyroscopeSensorCollector(
        context,
        sensorType,
        sensorDelay,
        accuracyChangedListener,
        measurementListener
    )
}