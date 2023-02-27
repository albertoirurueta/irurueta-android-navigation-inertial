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
import com.irurueta.android.navigation.inertial.ENUtoNEDConverter
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorCollector
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.units.TimeConverter

/**
 * Estimates relative attitude respect to start attitude by integrating gyroscope sensor data
 * without using any additional sensors.
 *
 * @property context Android context.
 * @property sensorType One of the supported gyroscope sensor types.
 * @property sensorDelay Delay of gyroscope between samples.
 * @property estimateCoordinateTransformation true to estimate coordinate transformation, false
 * otherwise. If not needed, it can be disabled to improve performance and decrease cpu load.
 * @property estimateEulerAngles true to estimate euler angles, false otherwise. If not
 * needed, it can be disabled to improve performance and decrease cpu load.
 * @property attitudeAvailableListener listener to notify when a new attitude measurement is
 * available.
 * @property gyroscopeMeasurementListener listener to notify new gyroscope measurements.
 */
class RelativeGyroscopeAttitudeEstimator(
    context: Context,
    sensorType: GyroscopeSensorType = GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
    sensorDelay: SensorDelay = SensorDelay.GAME,
    estimateCoordinateTransformation: Boolean = false,
    estimateDisplayEulerAngles: Boolean = true,
    attitudeAvailableListener: OnAttitudeAvailableListener? = null,
    gyroscopeMeasurementListener: GyroscopeSensorCollector.OnMeasurementListener? = null
) : BaseRelativeGyroscopeAttitudeEstimator<RelativeGyroscopeAttitudeEstimator,
        RelativeGyroscopeAttitudeEstimator.OnAttitudeAvailableListener>(
    context,
    sensorType,
    sensorDelay,
    estimateCoordinateTransformation,
    estimateDisplayEulerAngles,
    attitudeAvailableListener,
    gyroscopeMeasurementListener
) {
    /**
     * Instance to be reused which contains variation of attitude between gyroscope samples.
     */
    private val deltaAttitude = Quaternion()

    /**
     * Internal gyroscope sensor collector.
     */
    override val gyroscopeSensorCollector = GyroscopeSensorCollector(
        context,
        sensorType,
        sensorDelay,
        { wx, wy, wz, bx, by, bz, timestamp, accuracy ->
            gyroscopeMeasurementListener?.onMeasurement(wx, wy, wz, bx, by, bz, timestamp, accuracy)

            if (timeIntervalEstimator.numberOfProcessedSamples == 0) {
                initialTimestamp = timestamp
            }
            val diff = timestamp - initialTimestamp
            val diffSeconds = TimeConverter.nanosecondToSecond(diff.toDouble())
            timeIntervalEstimator.addTimestamp(diffSeconds)

            val dt = timeIntervalEstimator.averageTimeInterval

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

            ENUtoNEDConverter.convert(currentWx, currentWy, currentWz, triad)

            val roll = triad.valueX * dt
            val pitch = triad.valueY * dt
            val yaw = triad.valueZ * dt
            deltaAttitude.setFromEulerAngles(roll, pitch, yaw)

            internalAttitude.combine(deltaAttitude)
            internalAttitude.normalize()

            internalAttitude.copyTo(attitude)

            val c: CoordinateTransformation? =
                if (estimateCoordinateTransformation) {
                    coordinateTransformation.fromRotation(attitude)
                    coordinateTransformation
                } else {
                    null
                }

            val displayRoll: Double?
            val displayPitch: Double?
            val displayYaw: Double?
            if (estimateDisplayEulerAngles) {
                attitude.toEulerAngles(eulerAngles)
                displayRoll = eulerAngles[0]
                displayPitch = eulerAngles[1]
                displayYaw = eulerAngles[2]
            } else {
                displayRoll = null
                displayPitch = null
                displayYaw = null
            }

            // notify
            attitudeAvailableListener?.onAttitudeAvailable(
                this@RelativeGyroscopeAttitudeEstimator,
                attitude,
                timestamp,
                displayRoll,
                displayPitch,
                displayYaw,
                c
            )
        })

    /**
     * Interface to notify when a new attitude measurement is available.
     */
    fun interface OnAttitudeAvailableListener :
        BaseRelativeGyroscopeAttitudeEstimator.OnAttitudeAvailableListener<RelativeGyroscopeAttitudeEstimator,
                OnAttitudeAvailableListener>
}