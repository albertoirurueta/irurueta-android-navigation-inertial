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
package com.irurueta.android.navigation.inertial.estimators.attitude

import android.content.Context
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.collectors.GravitySensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.estimators.filter.AveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.LowPassAveragingFilter

/**
 * Estimates leveling of device (roll and pitch angle) by estimating gravity vector using
 * accelerometer measurements only.
 * This estimator does not estimate attitude yaw angle, as either a magnetometer or gyroscope would
 * be needed.
 * When device is placed vertically (pitch values close to 90 degrees), this estimator might become
 * unreliable. If device is expected to be in this orientation, avoid using this estimator and use
 * [AccurateLevelingEstimator] instead.
 *
 * @property context Android context.
 * @property sensorDelay Delay of accelerometer or gravity sensor between samples.
 * @property useAccelerometer true to use accelerometer sensor, false to use system gravity sensor.
 * @property accelerometerSensorType One of the supported accelerometer sensor types.
 * @property accelerometerAveragingFilter an averaging filter for accelerometer samples to obtain
 * sensed gravity component of specific force.
 * @property estimateCoordinateTransformation true to estimate coordinate transformation, false
 * otherwise. If not needed, it can be disabled to improve performance and decrease cpu load.
 * @property estimateEulerAngles true to estimate euler angles, false otherwise. If not
 * needed, it can be disabled to improve performance and decrease cpu load.
 * @property levelingAvailableListener listener to notify when a new leveling measurement is
 * available.
 * @property gravityEstimationListener listener to notify when a new gravity estimation is
 * available.
 */
class LevelingEstimator private constructor(
    context: Context,
    sensorDelay: SensorDelay,
    useAccelerometer: Boolean,
    accelerometerSensorType: AccelerometerSensorType,
    accelerometerAveragingFilter: AveragingFilter,
    estimateCoordinateTransformation: Boolean,
    estimateEulerAngles: Boolean,
    levelingAvailableListener: OnLevelingAvailableListener?,
    gravityEstimationListener: GravityEstimator.OnEstimationListener?
) : BaseLevelingEstimator<LevelingEstimator, LevelingEstimator.OnLevelingAvailableListener>(
    context,
    sensorDelay,
    useAccelerometer,
    accelerometerSensorType,
    accelerometerAveragingFilter,
    estimateCoordinateTransformation,
    estimateEulerAngles,
    levelingAvailableListener,
    gravityEstimationListener
) {

    /**
     * Constructor.
     *
     * @param context Android context.
     * @param sensorDelay Delay of accelerometer or gravity sensor between samples.
     * @param useAccelerometer true to use accelerometer sensor, false to use system gravity sensor.
     * @param accelerometerSensorType One of the supported accelerometer sensor types.
     * @param accelerometerAveragingFilter an averaging filter for accelerometer samples to obtain
     * sensed gravity component of specific force.
     * @param estimateCoordinateTransformation true to estimate coordinate transformation, false
     * otherwise. If not needed, it can be disabled to improve performance and decrease cpu load.
     * @param estimateEulerAngles true to estimate euler angles, false otherwise. If not
     * needed, it can be disabled to improve performance and decrease cpu load.
     * @param levelingAvailableListener listener to notify when a new leveling measurement is
     * available.
     * @param gravityEstimationListener listener to notify when a new gravity estimation is
     * available.
     * @param accelerometerMeasurementListener listener to notify new accelerometer measurements.
     * (Only used if [useAccelerometer] is true).
     * @param gravityMeasurementListener listener to notify new gravity measurements.
     * (Only used if [useAccelerometer] is false).
     */
    constructor(
        context: Context,
        sensorDelay: SensorDelay = SensorDelay.GAME,
        useAccelerometer: Boolean = true,
        accelerometerSensorType: AccelerometerSensorType =
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
        accelerometerAveragingFilter: AveragingFilter = LowPassAveragingFilter(),
        estimateCoordinateTransformation: Boolean = false,
        estimateEulerAngles: Boolean = true,
        levelingAvailableListener: OnLevelingAvailableListener? = null,
        gravityEstimationListener: GravityEstimator.OnEstimationListener? = null,
        accelerometerMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener? = null,
        gravityMeasurementListener: GravitySensorCollector.OnMeasurementListener? = null
    ) : this(
        context,
        sensorDelay,
        useAccelerometer,
        accelerometerSensorType,
        accelerometerAveragingFilter,
        estimateCoordinateTransformation,
        estimateEulerAngles,
        levelingAvailableListener,
        gravityEstimationListener
    ) {
        gravityEstimator = GravityEstimator(
            context,
            sensorDelay,
            useAccelerometer,
            accelerometerSensorType,
            estimationListener = { estimator, fx, fy, fz, timestamp ->
                this.gravityEstimationListener?.onEstimation(estimator, fx, fy, fz, timestamp)

                val roll =
                    com.irurueta.navigation.inertial.estimators.LevelingEstimator.getRoll(fy, fz)
                val pitch =
                    com.irurueta.navigation.inertial.estimators.LevelingEstimator.getPitch(
                        fx,
                        fy,
                        fz
                    )

                attitude.setFromEulerAngles(roll, pitch, 0.0)

                postProcessAttitudeAndNotify(timestamp)
            },
            accelerometerAveragingFilter,
            accelerometerMeasurementListener,
            gravityMeasurementListener
        )
    }

    /**
     * Internal gravity estimator sensed as a component of specific force.
     */
    override lateinit var gravityEstimator: GravityEstimator

    /**
     * Listener to notify new accelerometer measurements.
     * (Only used if [useAccelerometer] is true).
     */
    override var accelerometerMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener?
        get() = gravityEstimator.accelerometerMeasurementListener
        set(value) {
            gravityEstimator.accelerometerMeasurementListener = value
        }

    /**
     * listener to notify new gravity measurements.
     * (Only used if [useAccelerometer] is false).
     */
    override var gravityMeasurementListener: GravitySensorCollector.OnMeasurementListener?
        get() = gravityEstimator.gravityMeasurementListener
        set(value) {
            gravityEstimator.gravityMeasurementListener = value
        }

    /**
     * Interface to notify when a new leveling measurement is available.
     */
    fun interface OnLevelingAvailableListener :
        BaseLevelingEstimator.OnLevelingAvailableListener<LevelingEstimator,
                OnLevelingAvailableListener>
}