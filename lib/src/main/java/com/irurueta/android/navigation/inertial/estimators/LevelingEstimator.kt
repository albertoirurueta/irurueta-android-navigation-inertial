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
package com.irurueta.android.navigation.inertial.estimators

import android.content.Context
import com.irurueta.android.navigation.inertial.DisplayOrientationHelper
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.estimators.filter.AveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.LowPassAveragingFilter

/**
 * Estimates leveling of device (roll and pitch angle) by estimating gravity vector using
 * accelerometer measurements only.
 * This estimator does not estimate attitude yaw angle, as either a magnetometer or gyroscope would
 * be needed.
 *
 * @property context Android context.
 * @property sensorDelay Delay of accelerometer or gravity sensor between samples.
 * @property useAccelerometer true to use accelerometer sensor, false to use system gravity sensor.
 * @property accelerometerSensorType One of the supported accelerometer sensor types.
 * @property accelerometerAveragingFilter an averaging filter for accelerometer samples to obtain
 * sensed gravity component of specific force.
 * @property estimateCoordinateTransformation true to estimate coordinate transformation, false
 * otherwise. If not needed, it can be disabled to improve performance and decrease cpu load.
 * @property estimateDisplayEulerAngles true to estimate euler angles, false otherwise. If not
 * needed, it can be disabled to improve performance and decrease cpu load.
 * @property levelingAvailableListener listener to notify when a new leveling measurement is
 * available.
 */
class LevelingEstimator(
    context: Context,
    sensorDelay: SensorDelay = SensorDelay.GAME,
    useAccelerometer: Boolean = false,
    accelerometerSensorType: AccelerometerSensorCollector.SensorType =
        AccelerometerSensorCollector.SensorType.ACCELEROMETER,
    accelerometerAveragingFilter: AveragingFilter = LowPassAveragingFilter(),
    estimateCoordinateTransformation: Boolean = false,
    estimateDisplayEulerAngles: Boolean = true,
    levelingAvailableListener: OnLevelingAvailableListener? = null
) : BaseLevelingEstimator<LevelingEstimator, LevelingEstimator.OnLevelingAvailableListener>(
    context,
    sensorDelay,
    useAccelerometer,
    accelerometerSensorType,
    accelerometerAveragingFilter,
    estimateCoordinateTransformation,
    estimateDisplayEulerAngles,
    levelingAvailableListener
) {
    /**
     * Internal gravity estimator sensed as a component of specific force.
     */
    override val gravityEstimator = GravityEstimator(
        context,
        sensorDelay,
        useAccelerometer,
        accelerometerSensorType,
        { _, fx, fy, fz, _ ->
            val displayRotationRadians =
                DisplayOrientationHelper.getDisplayRotationRadians(context)
            displayOrientation.setFromEulerAngles(0.0, 0.0, -displayRotationRadians)

            val roll = com.irurueta.navigation.inertial.estimators.LevelingEstimator.getRoll(fy, fz)
            val pitch =
                com.irurueta.navigation.inertial.estimators.LevelingEstimator.getPitch(fx, fy, fz)

            attitude.setFromEulerAngles(roll, pitch, 0.0)

            postProcessAttitudeAndNotify()
        },
        accelerometerAveragingFilter
    )

    /**
     * Interface to notify when a new leveling measurement is available.
     */
    fun interface OnLevelingAvailableListener :
        BaseLevelingEstimator.OnLevelingAvailableListener<LevelingEstimator,
                OnLevelingAvailableListener>
}