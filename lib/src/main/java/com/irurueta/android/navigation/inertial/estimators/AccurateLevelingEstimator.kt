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
import android.location.Location
import com.irurueta.algebra.ArrayUtils
import com.irurueta.algebra.Matrix
import com.irurueta.algebra.Utils
import com.irurueta.android.navigation.inertial.DisplayOrientationHelper
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.estimators.filter.AveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.LowPassAveragingFilter
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.inertial.estimators.NEDGravityEstimator
import kotlin.math.atan2

/**
 * Estimates leveling of device (roll and pitch angle) by estimating gravity vector using
 * accelerometer measurements only.
 * This estimator does not estimate attitude yaw angle, as either a magnetometer or gyroscope would
 * be needed.
 * This estimator is more accurate than [LevelingEstimator] since it takes into account device
 * location (which requires location permission), and at the expense of higher CPU load.
 *
 * @property context Android context.
 * @property location Device location.
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
class AccurateLevelingEstimator(
    context: Context,
    var location: Location,
    sensorDelay: SensorDelay = SensorDelay.GAME,
    useAccelerometer: Boolean = false,
    accelerometerSensorType: AccelerometerSensorCollector.SensorType =
        AccelerometerSensorCollector.SensorType.ACCELEROMETER,
    accelerometerAveragingFilter: AveragingFilter = LowPassAveragingFilter(),
    estimateCoordinateTransformation: Boolean = false,
    estimateDisplayEulerAngles: Boolean = true,
    levelingAvailableListener: OnLevelingAvailableListener? = null
) : BaseLevelingEstimator<AccurateLevelingEstimator, AccurateLevelingEstimator.OnLevelingAvailableListener>(
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
            displayOrientation.setFromEulerAngles(0.0, 0.0, displayRotationRadians)

            computeLevelingAttitude(
                Math.toRadians(location.latitude),
                location.altitude,
                fx,
                fy,
                fz,
                attitude
            )

            postProcessAttitudeAndNotify()
        },
        accelerometerAveragingFilter
    )

    companion object {
        /**
         * Computes an attitude containing device leveling (roll and pitch) based on current
         * location (latitude and height) and sensed specific force containing gravity components.
         *
         * @param latitude current device latitude expressed in radians.
         * @param height current device height above mean sea level expressed in meters (m).
         * @param fx x-coordinate of sensed specific force containing gravity component.
         * @param fy y-coordinate of sensed specific force containing gravity component.
         * @param fz z-coordinate of sensed specific force containing gravity component.
         * @param result quaternion where attitude containing estimated leveling (roll and pitch
         * angles) will be stored.
         */
        fun computeLevelingAttitude(
            latitude: Double,
            height: Double,
            fx: Double,
            fy: Double,
            fz: Double,
            result: Quaternion
        ) {
            // get normalized vector from measured specific force, which
            // mainly contains sensed gravity in the local navigation frame
            // when device is static (Coriolis force is neglected in this
            // implementation).

            // obtain normalized specific force in local navigation coordinates
            val normF = doubleArrayOf(fx, fy, fz)
            ArrayUtils.normalize(normF)

            // obtain gravity in NED coordinates (locally equivalent to
            // the one in local navigation frame).
            // Because Earth is not fully spherical, normalized vector won't
            // be (0, 0, 1), because there will always be a small north
            // gravity component.
            val nedGravity = NEDGravityEstimator.estimateGravityAndReturnNew(latitude, height)

            val normG = nedGravity.asArray()

            // ensure that down coordinate points towards Earth center, just
            // like sensed specific force

            // ensure that down coordinate points towards Earth center, just
            // like sensed specific force
            ArrayUtils.multiplyByScalar(normG, -1.0, normG)

            ArrayUtils.normalize(normG)

            // compute angle between both normalized vectors using dot product
            // cos(alpha) = normF' * normG
            val cosAlpha = ArrayUtils.dotProduct(normF, normG)

            // compute vector perpendicular to both normF and normG which will
            // be the rotation axis
            val skew = Utils.skewMatrix(normG)
            val tmp1 = Matrix.newFromArray(normF)
            val tmp2 = skew.multiplyAndReturnNew(tmp1)

            val sinAlpha = Utils.normF(tmp2)

            val axis = tmp2.toArray()
            ArrayUtils.normalize(axis)

            val alpha = atan2(sinAlpha, cosAlpha)

            result.setFromAxisAndRotation(axis, -alpha)
        }
    }

    /**
     * Interface to notify when a new leveling measurement is available.
     */
    fun interface OnLevelingAvailableListener :
        BaseLevelingEstimator.OnLevelingAvailableListener<AccurateLevelingEstimator, OnLevelingAvailableListener>
}