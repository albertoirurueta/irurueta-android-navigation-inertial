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
package com.irurueta.android.navigation.inertial.old.estimators.attitude

import android.content.Context
import com.irurueta.android.navigation.inertial.ENUtoNEDConverter
import com.irurueta.android.navigation.inertial.old.collectors.GyroscopeSensorCollector
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegrator
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegratorType
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
 * @property estimateEulerAngles true to estimate euler angles, false otherwise. If not
 * needed, it can be disabled to improve performance and decrease cpu load.
 * @property attitudeAvailableListener listener to notify when a new attitude measurement is
 * available.
 * @property gyroscopeMeasurementListener listener to notify new gyroscope measurements.
 */
class AccurateRelativeGyroscopeAttitudeEstimator(
    context: Context,
    sensorType: GyroscopeSensorType = GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
    sensorDelay: SensorDelay = SensorDelay.GAME,
    estimateCoordinateTransformation: Boolean = false,
    estimateEulerAngles: Boolean = true,
    attitudeAvailableListener: OnAttitudeAvailableListener? = null,
    gyroscopeMeasurementListener: GyroscopeSensorCollector.OnMeasurementListener? = null
) : BaseRelativeGyroscopeAttitudeEstimator<AccurateRelativeGyroscopeAttitudeEstimator,
        AccurateRelativeGyroscopeAttitudeEstimator.OnAttitudeAvailableListener>(
    context,
    sensorType,
    sensorDelay,
    estimateCoordinateTransformation,
    estimateEulerAngles,
    attitudeAvailableListener,
    gyroscopeMeasurementListener
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
     * Integrates new gyroscope measurements into existing attitude.
     */
    private val quaternionStepIntegrator = QuaternionStepIntegrator.create(
        QuaternionStepIntegratorType.RUNGE_KUTTA
    )

    /**
     * Internal gyroscope sensor collector.
     */
    override val gyroscopeSensorCollector = GyroscopeSensorCollector(
        context,
        sensorType,
        sensorDelay,
        { wx, wy, wz, bx, by, bz, timestamp, accuracy ->
            gyroscopeMeasurementListener?.onMeasurement(wx, wy, wz, bx, by, bz, timestamp, accuracy)

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

            ENUtoNEDConverter.convert(currentWx, currentWy, currentWz, triad)

            if (!isFirst) {
                val dt = timeIntervalEstimator.averageTimeInterval

                quaternionStepIntegrator.integrate(
                    internalAttitude,
                    previousWx,
                    previousWy,
                    previousWz,
                    triad.valueX,
                    triad.valueY,
                    triad.valueZ,
                    dt,
                    internalAttitude
                )

                internalAttitude.copyTo(attitude)

                val c: CoordinateTransformation? =
                    if (estimateCoordinateTransformation) {
                        coordinateTransformation.fromRotation(attitude)
                        coordinateTransformation
                    } else {
                        null
                    }

                val roll: Double?
                val pitch: Double?
                val yaw: Double?
                if (estimateEulerAngles) {
                    attitude.toEulerAngles(eulerAngles)
                    roll = eulerAngles[0]
                    pitch = eulerAngles[1]
                    yaw = eulerAngles[2]
                } else {
                    roll = null
                    pitch = null
                    yaw = null
                }

                // notify
                attitudeAvailableListener?.onAttitudeAvailable(
                    this@AccurateRelativeGyroscopeAttitudeEstimator,
                    attitude,
                    timestamp,
                    roll,
                    pitch,
                    yaw,
                    c
                )
            }

            previousWx = triad.valueX
            previousWy = triad.valueY
            previousWz = triad.valueZ
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