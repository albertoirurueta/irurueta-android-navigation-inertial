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
package com.irurueta.android.navigation.inertial.old.estimators.attitude

import android.content.Context
import android.location.Location
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.old.estimators.filter.AveragingFilter
import com.irurueta.android.navigation.inertial.old.estimators.filter.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.old.processors.attitude.LevelingProcessor

/**
 * Estimates leveling of device (roll and pitch angle) by estimating gravity vector using
 * accelerometer measurements only.
 * This estimator does not estimate attitude yaw angle, as either a magnetometer or gyroscope would
 * be needed.
 * When device is placed vertically (pitch values close to 90 degrees), this estimator might become
 * unreliable. If device is expected to be in this orientation, avoid using this estimator and use
 * [AccurateLevelingEstimator2] instead.
 *
 * @property context Android context.
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
 * @property location Device location.
 * @property adjustGravityNorm indicates whether gravity norm must be adjusted to either Earth
 * standard norm, or norm at provided location. If no location is provided, this should only be
 * enabled when device is close to sea level.
 */
class LevelingEstimator2(
    context: Context,
    sensorDelay: SensorDelay = SensorDelay.GAME,
    useAccelerometer: Boolean = true,
    startOffsetEnabled: Boolean = true,
    accelerometerSensorType: AccelerometerSensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
    accelerometerAveragingFilter: AveragingFilter = LowPassAveragingFilter(),
    estimateCoordinateTransformation: Boolean = false,
    estimateEulerAngles: Boolean = true,
    levelingAvailableListener: OnLevelingAvailableListener? = null,
    accuracyChangedListener: OnAccuracyChangedListener? = null,
    location: Location? = null,
    adjustGravityNorm: Boolean = true
) : BaseLevelingEstimator2<LevelingEstimator2, LevelingEstimator2.OnLevelingAvailableListener, LevelingEstimator2.OnAccuracyChangedListener>(
    context,
    sensorDelay,
    useAccelerometer,
    startOffsetEnabled,
    accelerometerSensorType,
    accelerometerAveragingFilter,
    estimateCoordinateTransformation,
    estimateEulerAngles,
    levelingAvailableListener,
    accuracyChangedListener,
    adjustGravityNorm
) {

    /**
     * Internal processor to estimate leveled attitude from accelerometer or gravity measurements.
     */
    override val levelingProcessor = LevelingProcessor()

    /**
     * Gets or sets device location.
     * Location can be updated while this estimator is running to obtain more accurate attitude
     * estimations.
     */
    var location: Location? = location
        set(value) {
            field = value
            gravityProcessor.location = value
            accelerometerGravityProcessor.location = value
        }

    // initializes gravity processor
    init {
        gravityProcessor.location = location
        accelerometerGravityProcessor.location = location

        gravityProcessor.adjustGravityNorm = adjustGravityNorm
        accelerometerGravityProcessor.adjustGravityNorm = adjustGravityNorm
    }

    /**
     * Interface to notify when a new leveling measurement is available.
     */
    fun interface OnLevelingAvailableListener :
        BaseLevelingEstimator2.OnLevelingAvailableListener<LevelingEstimator2,
                OnLevelingAvailableListener, OnAccuracyChangedListener>

    /**
     * Interface to notify when sensor accuracy changes.
     */
    fun interface OnAccuracyChangedListener :
        BaseLevelingEstimator2.OnAccuracyChangedListener<LevelingEstimator2,
                OnLevelingAvailableListener, OnAccuracyChangedListener>
}