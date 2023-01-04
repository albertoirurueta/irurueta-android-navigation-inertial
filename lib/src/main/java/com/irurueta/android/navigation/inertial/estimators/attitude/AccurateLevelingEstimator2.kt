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
import android.location.Location
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.estimators.filter.AveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.processors.AccurateLevelingProcessor

/**
 * Estimates leveling of device (roll and pitch angle) by estimating gravity vector using
 * accelerometer measurements only.
 * This estimator does not estimate attitude yaw angle, as either a magnetometer or gyroscope would
 * be needed.
 * This estimator is more accurate than [LevelingEstimator2] since it takes into account device
 * location (which requires location permission), and at the expense of higher CPU load.
 *
 * @property context Android context.
 * @property location Device location.
 * @property sensorDelay Delay of accelerometer or gravity sensor between samples.
 * @property useAccelerometer true to use accelerometer sensor, false to use system gravity sensor.
 * @property startOffsetEnabled indicates whether [startOffset] will be computed when first
 * measurement is received or not. True indicates that offset is computed, false assumes that offset
 * is null.
 * @property accelerometerSensorType One of the supported accelerometer sensor types.
 * @property accelerometerAveragingFilter an averaging filter for accelerometer samples to obtain
 * sensed gravity component of specific force.
 * @property estimateCoordinateTransformation true to estimate coordinate transformation, false
 * otherwise. If not needed, it can be disabled to improve performance and decrease cpu load.
 * @property estimateEulerAngles true to estimate euler angles, false otherwise. If not
 * needed, it can be disabled to improve performance and decrease cpu load.
 * @property levelingAvailableListener listener to notify when a new leveling measurement is
 * available.
 * @property accuracyChangedListener listener to notify changes in accuracy.
 */
class AccurateLevelingEstimator2(
    context: Context,
    location: Location,
    sensorDelay: SensorDelay = SensorDelay.GAME,
    useAccelerometer: Boolean = true,
    startOffsetEnabled: Boolean = true,
    accelerometerSensorType: AccelerometerSensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
    accelerometerAveragingFilter: AveragingFilter = LowPassAveragingFilter(),
    estimateCoordinateTransformation: Boolean = false,
    estimateEulerAngles: Boolean = true,
    levelingAvailableListener: OnLevelingAvailableListener? = null,
    accuracyChangedListener: OnAccuracyChangedListener? = null
) : BaseLevelingEstimator2<AccurateLevelingEstimator2, AccurateLevelingEstimator2.OnLevelingAvailableListener, AccurateLevelingEstimator2.OnAccuracyChangedListener>(
    context,
    sensorDelay,
    useAccelerometer,
    startOffsetEnabled,
    accelerometerSensorType,
    accelerometerAveragingFilter,
    estimateCoordinateTransformation,
    estimateEulerAngles,
    levelingAvailableListener,
    accuracyChangedListener
) {
    /**
     * Internal processor to estimate leveled attitude from accelerometer or gravity measurements.
     */
    override val levelingProcessor = AccurateLevelingProcessor(location)

    /**
     * Gets or sets device location.
     * Location can be updated while this estimator is running to obtain more accurate attitude
     * estimations.
     */
    var location: Location = location
        set(value) {
            field = value
            levelingProcessor.location = location
        }

    /**
     * Interface to notify when a new leveling measurement is available.
     */
    fun interface OnLevelingAvailableListener :
        BaseLevelingEstimator2.OnLevelingAvailableListener<AccurateLevelingEstimator2,
                OnLevelingAvailableListener, OnAccuracyChangedListener>

    /**
     * Interface to notify when sensor accuracy changes.
     */
    fun interface OnAccuracyChangedListener :
        BaseLevelingEstimator2.OnAccuracyChangedListener<AccurateLevelingEstimator2,
                OnLevelingAvailableListener, OnAccuracyChangedListener>
}