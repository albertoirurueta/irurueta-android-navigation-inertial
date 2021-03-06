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
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.geometry.InvalidRotationMatrixException
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.units.TimeConverter

/**
 * Estimates relative attitude respect to start attitude by integrating gyroscope sensor data
 * using Runge-Kutta integration for greater accuracy, and without using any additional sensors.
 *
 * @property context Android context.
 * @property sensorType One of the supported gyroscope sensor types.
 * @property sensorDelay Delay of gyroscope between samples.
 * @property estimateCoordinateTransformation true to estimate coordinate transformation, false
 * otherwise. If not needed, it can be disabled to improve performance and decrease cpu load.
 * @property estimateDisplayEulerAngles true to estimate euler angles, false otherwise. If not
 * needed, it can be disabled to improve performance and decrease cpu load.
 * @property attitudeAvailableListener listener to notify when a new attitude measurement is
 * available.
 */
class AccurateRelativeGyroscopeAttitudeEstimator(
    context: Context,
    sensorType: GyroscopeSensorCollector.SensorType =
        GyroscopeSensorCollector.SensorType.GYROSCOPE,
    sensorDelay: SensorDelay = SensorDelay.GAME,
    estimateCoordinateTransformation: Boolean = false,
    estimateDisplayEulerAngles: Boolean = true,
    attitudeAvailableListener: OnAttitudeAvailableListener? = null
) : BaseRelativeGyroscopeAttitudeEstimator<AccurateRelativeGyroscopeAttitudeEstimator,
        AccurateRelativeGyroscopeAttitudeEstimator.OnAttitudeAvailableListener>(
    context,
    sensorType,
    sensorDelay,
    estimateCoordinateTransformation,
    estimateDisplayEulerAngles,
    attitudeAvailableListener
) {

    /**
     * Previous x-coordinate angular speed expressed in radians per second (rad/s).
     */
    private var previousWx = 0.0

    /**
     * Previous y-coordinate angular speed expressed in radians per second (rad/s).
     */
    private var previousWy = 0.0

    /**
     * Previous z-coordinate angular speed expressed in radians per second (rad/s).
     */
    private var previousWz = 0.0

    /**
     * Internal gyroscope sensor collector.
     */
    override val gyroscopeSensorCollector = GyroscopeSensorCollector(
        context,
        sensorType,
        sensorDelay,
        { wx, wy, wz, bx, by, bz, timestamp, _ ->
            val isFirst = timeIntervalEstimator.numberOfProcessedSamples == 0
            if (isFirst) {
                initialTimestamp = timestamp
            }
            val diff = timestamp - initialTimestamp
            val diffSeconds = TimeConverter.nanosecondToSecond(diff.toDouble())
            timeIntervalEstimator.addTimestamp(diffSeconds)

            val currentWx = if (bx != null)
                wx.toDouble() - bx.toDouble()
            else
                wx.toDouble()

            val currentWy = if (by != null)
                wy.toDouble() - by.toDouble()
            else
                wy.toDouble()

            val currentWz = if (bz != null)
                wz.toDouble() - bz.toDouble()
            else
                wz.toDouble()

            if (!isFirst) {
                val dt = timeIntervalEstimator.averageTimeInterval

                val displayRotationRadians =
                    DisplayOrientationHelper.getDisplayRotationRadians(context)
                displayOrientation.setFromEulerAngles(0.0, 0.0, -displayRotationRadians)

                resetQuaternion(deltaAttitude)
                AttitudeIntegrator.integrationStep(
                    deltaAttitude,
                    previousWx,
                    previousWy,
                    previousWz,
                    currentWy,
                    currentWy,
                    currentWz,
                    dt,
                    deltaAttitude
                )
                deltaAttitude.normalize()
                internalAttitude.combine(deltaAttitude)
                internalAttitude.normalize()

                internalAttitude.copyTo(attitude)
                attitude.combine(displayOrientation)
                attitude.inverse()
                attitude.normalize()

                val c: CoordinateTransformation? =
                    if (estimateCoordinateTransformation) {
                        attitude.asInhomogeneousMatrix(rotationMatrix)
                        try {
                            coordinateTransformation.matrix = rotationMatrix
                            coordinateTransformation
                        } catch (ignore: InvalidRotationMatrixException) {
                            null
                        }
                    } else {
                        null
                    }

                val displayRoll: Double?
                val displayPitch: Double?
                val displayYaw: Double?
                if (estimateDisplayEulerAngles) {
                    attitude.toEulerAngles(displayEulerAngles)
                    displayRoll = displayEulerAngles[0]
                    displayPitch = displayEulerAngles[1]
                    displayYaw = displayEulerAngles[2]
                } else {
                    displayRoll = null
                    displayPitch = null
                    displayYaw = null
                }

                // notify
                attitudeAvailableListener?.onAttitudeAvailable(
                    this@AccurateRelativeGyroscopeAttitudeEstimator,
                    attitude,
                    displayRoll,
                    displayPitch,
                    displayYaw,
                    c
                )
            }

            previousWx = currentWx
            previousWy = currentWy
            previousWz = currentWz
        }
    )

    /**
     * Resets this estimator to its initial state.
     */
    override fun reset() {
        previousWx = 0.0
        previousWy = 0.0
        previousWz = 0.0
        super.reset()
    }

    /**
     * Interface to notify when a new attitude measurement is available.
     */
    fun interface OnAttitudeAvailableListener :
        BaseRelativeGyroscopeAttitudeEstimator.OnAttitudeAvailableListener<AccurateRelativeGyroscopeAttitudeEstimator,
                OnAttitudeAvailableListener>
}