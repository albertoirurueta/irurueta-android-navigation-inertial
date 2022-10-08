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
package com.irurueta.android.navigation.inertial.estimators.pose

import android.content.Context
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.estimators.attitude.GravityEstimator
import com.irurueta.android.navigation.inertial.estimators.filter.AveragingFilter

/**
 * Base interface for pose estimators.
 */
interface PoseEstimator {

    /**
     * Android context.
     */
    val context: Context

    /**
     * Delay of sensors between samples.
     */
    val sensorDelay: SensorDelay

    /**
     * One of the supported accelerometer sensor types.
     */
    val accelerometerSensorType: AccelerometerSensorCollector.SensorType

    /**
     * One of the supported gyroscope sensor types.
     */
    val gyroscopeSensorType: GyroscopeSensorCollector.SensorType

    /**
     * An averaging filter for accelerometer samples to obtain sensed gravity component of
     * specific force.
     */
    val accelerometerAveragingFilter: AveragingFilter

    /**
     * Listener to notify new accelerometer measurements.
     */
    var accelerometerMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener?

    /**
     * Listener to notify new gyroscope measurements.
     */
    var gyroscopeMeasurementListener: GyroscopeSensorCollector.OnMeasurementListener?

    /**
     * Listener to notify when a new gravity estimation is available.
     */
    var gravityEstimationListener: GravityEstimator.OnEstimationListener?

    /**
     * Indicates whether accurate leveling must be used or not.
     */
    var useAccurateLevelingEstimator: Boolean

    /**
     * Indicates whether accurate non-leveled relative attitude estimator must be used or not.
     */
    var useAccurateRelativeGyroscopeAttitudeEstimator: Boolean

    /**
     * Indicates whether fusion between leveling and relative attitudes occurs based
     * on changing interpolation value that depends on actual relative attitude rotation
     * velocity.
     */
    var useIndirectAttitudeInterpolation: Boolean

    /**
     * Interpolation value to be used to combine both leveling and relative attitudes.
     * Must be between 0.0 and 1.0 (both included).
     * The closer to 0.0 this value is, the more resemblance the result will have to a pure
     * leveling (which feels more jerky). On the contrary, the closer to 1.0 this value is,
     * the more resemblance the result will have to a pure non-leveled relative attitude (which
     * feels softer but might have arbitrary roll and pitch Euler angles).
     */
    var attitudeInterpolationValue: Double

    /**
     * Factor to take into account when interpolation value is computed and
     * [useIndirectAttitudeInterpolation] is enabled to determine actual interpolation value based
     * on current relative attitude rotation velocity.
     */
    var attitudeIndirectInterpolationWeight: Double

    /**
     * Threshold to determine that current leveling appears to be an outlier respect
     * to estimated fused attitude.
     * When leveling and fused attitudes diverge, fusion is not performed, and instead
     * only gyroscope relative attitude is used for fusion estimation.
     */
    var attitudeOutlierThreshold: Double

    /**
     * Threshold to determine that leveling has largely diverged and if situation is not
     * reverted soon, attitude will be reset to leveling.
     */
    var attitudeOutlierPanicThreshold: Double

    /**
     * Threshold to determine when fused attitude has largely diverged for a given
     * number of samples and must be reset.
     */
    var attitudePanicCounterThreshold: Int

    /**
     * Gets average time interval between gyroscope samples expressed in seconds.
     */
    val averageTimeInterval: Double

    /**
     * Indicates whether this estimator is running or not.
     */
    val running: Boolean

    /**
     * Starts this estimator.
     *
     * @return true if estimator successfully started, false otherwise.
     * @throws IllegalStateException if estimator is already running.
     */
    @Throws(IllegalStateException::class)
    fun start(): Boolean

    /**
     * Stops this estimator.
     */
    fun stop()
}