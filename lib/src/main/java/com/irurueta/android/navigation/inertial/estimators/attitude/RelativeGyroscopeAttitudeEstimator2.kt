/*
 * Copyright (C) 2023 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.android.navigation.inertial.estimators.attitude

import android.content.Context
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.processors.attitude.RelativeGyroscopeAttitudeProcessor

/**
 * Estimates relative attitude respect to start attitude by integrating gyroscope sensor data
 * without using any additional sensors.
 *
 * @property context Android context.
 * @property sensorType One of the supported gyroscope sensor types.
 * @property sensorDelay Delay of gyroscope between samples.
 * @property startOffsetEnabled indicates whether [startOffset] will be computed when first
 * measurement is received or not. True indicates that offset is computed, false assumes that offset
 * is null.
 * @property estimateCoordinateTransformation true to estimate coordinate transformation, false
 * otherwise. If not needed, it can be disabled to improve performance and decrease cpu load.
 * @property estimateEulerAngles true to estimate euler angles, false otherwise. If not
 * needed, it can be disabled to improve performance and decrease cpu load.
 * @property attitudeAvailableListener listener to notify when a new attitude measurement is
 * available.
 * @property accuracyChangedListener listener to notify changes in accuracy.
 */
class RelativeGyroscopeAttitudeEstimator2(
    context: Context,
    sensorType: GyroscopeSensorType = GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
    sensorDelay: SensorDelay = SensorDelay.GAME,
    startOffsetEnabled: Boolean = false,
    estimateCoordinateTransformation: Boolean = false,
    estimateEulerAngles: Boolean = true,
    attitudeAvailableListener: OnAttitudeAvailableListener? = null,
    accuracyChangedListener: OnAccuracyChangedListener? = null
) : BaseRelativeGyroscopeAttitudeEstimator2<RelativeGyroscopeAttitudeEstimator2,
        RelativeGyroscopeAttitudeEstimator2.OnAttitudeAvailableListener,
        RelativeGyroscopeAttitudeEstimator2.OnAccuracyChangedListener>(
    context,
    sensorType,
    sensorDelay,
    startOffsetEnabled,
    estimateCoordinateTransformation,
    estimateEulerAngles,
    attitudeAvailableListener,
    accuracyChangedListener
) {
    /**
     * Internal processor in charge of estimating attitude based on gyroscope measurements.
     */
    override val processor = RelativeGyroscopeAttitudeProcessor()

    /**
     * Interface to notify when a new relative attitude measurement is available.
     */
    fun interface OnAttitudeAvailableListener :
        BaseRelativeGyroscopeAttitudeEstimator2.OnAttitudeAvailableListener<
                RelativeGyroscopeAttitudeEstimator2, OnAttitudeAvailableListener,
                OnAccuracyChangedListener>

    /**
     * Interface to notify when sensor accuracy changes.
     */
    fun interface OnAccuracyChangedListener :
        BaseRelativeGyroscopeAttitudeEstimator2.OnAccuracyChangedListener<
                RelativeGyroscopeAttitudeEstimator2, OnAttitudeAvailableListener,
                OnAccuracyChangedListener>
}