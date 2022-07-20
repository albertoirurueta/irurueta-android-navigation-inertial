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
import com.irurueta.algebra.Matrix
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.geometry.Quaternion
import com.irurueta.geometry.Rotation3D
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType
import com.irurueta.navigation.inertial.calibration.TimeIntervalEstimator

/**
 * Base class for estimators of device relative attitude respect to start attitude by integrating
 * gyroscope sensor data without using any additional sensors.
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
abstract class BaseRelativeGyroscopeAttitudeEstimator<T : BaseRelativeGyroscopeAttitudeEstimator<T, L>,
        L : BaseRelativeGyroscopeAttitudeEstimator.OnAttitudeAvailableListener<T, L>>(
    val context: Context,
    val sensorType: GyroscopeSensorCollector.SensorType =
        GyroscopeSensorCollector.SensorType.GYROSCOPE,
    val sensorDelay: SensorDelay = SensorDelay.GAME,
    val estimateCoordinateTransformation: Boolean = false,
    val estimateDisplayEulerAngles: Boolean = true,
    var attitudeAvailableListener: L? = null
) {
    /**
     * Instance to be reused which contains integrated attitude of all gyroscope samples taking
     * into account display orientation.
     */
    protected val attitude = Quaternion()

    /**
     * Instance to be reused which contains integrated attitude of all gyroscope samples without
     * taking into account display orientation.
     */
    protected val internalAttitude = Quaternion()

    /**
     * Instance to be reused which contains variation of attitude between gyroscope samples.
     */
    protected val deltaAttitude = Quaternion()

    /**
     * Instance to be reused containing display rotation as a yaw angle.
     */
    protected val displayOrientation = Quaternion()

    /**
     * Instance to be reused containing rotation matrix of coordinate transformation.
     */
    protected val rotationMatrix = Matrix(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)

    /**
     * Array to be reused containing euler angles of leveling attitude.
     */
    protected val displayEulerAngles = DoubleArray(Quaternion.N_ANGLES)

    /**
     * Instance to be reused containing coordinate transformation in NED coordinates.
     */
    protected val coordinateTransformation =
        CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME)

    /**
     * Estimates average time interval between gyroscope measurements.
     */
    protected val timeIntervalEstimator = TimeIntervalEstimator(Integer.MAX_VALUE)

    /**
     * Timestamp of first sample expressed in nanoseconds.
     */
    protected var initialTimestamp: Long = 0L

    /**
     * Internal gyroscope sensor collector.
     */
    protected abstract val gyroscopeSensorCollector: GyroscopeSensorCollector

    /**
     * Indicates whether this estimator is running or not.
     */
    var running: Boolean = false
        private set

    /**
     * Gets average time interval between gyroscope samples expressed in seconds.
     */
    val averageTimeInterval
        get() = timeIntervalEstimator.averageTimeInterval

    /**
     * Starts this estimator.
     *
     * @return true if estimator successfully started, false otherwise.
     * @throws IllegalStateException if estimator is already running.
     */
    @Throws(IllegalStateException::class)
    fun start(): Boolean {
        check(!running)

        reset()
        running = gyroscopeSensorCollector.start()
        return running
    }

    /**
     * Stops this estimator.
     */
    fun stop() {
        gyroscopeSensorCollector.stop()
        running = false
    }

    /**
     * Resets this estimator to its initial state.
     */
    protected open fun reset() {
        timeIntervalEstimator.reset()

        resetQuaternion(attitude)
        resetQuaternion(internalAttitude)
        resetQuaternion(deltaAttitude)
    }

    internal companion object {
        /**
         * Resets provided quaternion to the identity
         */
        internal fun resetQuaternion(q: Quaternion) {
            q.a = 1.0
            q.b = 0.0
            q.c = 0.0
            q.d = 0.0
            q.normalize()
        }
    }

    /**
     * Interface to notify when a new attitude measurement is available.
     */
    fun interface OnAttitudeAvailableListener<T : BaseRelativeGyroscopeAttitudeEstimator<T, L>,
            L : OnAttitudeAvailableListener<T, L>> {

        /**
         * Called when a new attitude measurement is available.
         *
         * @param estimator attitude estimator that raised this event.
         * @param attitude attitude expressed in NED coordinates.
         * @param roll roll angle expressed in radians. Only available if
         * [estimateDisplayEulerAngles].
         * @param pitch pitch angle expressed in radians. Only available if
         * [estimateDisplayEulerAngles].
         * @param yaw yaw angle expressed in radians. Only available if
         * [estimateDisplayEulerAngles].
         * @param coordinateTransformation coordinate transformation containing measured leveling
         * attitude. Only available if [estimateCoordinateTransformation].
         */
        fun onAttitudeAvailable(
            estimator: T,
            attitude: Quaternion,
            roll: Double?,
            pitch: Double?,
            yaw: Double?,
            coordinateTransformation: CoordinateTransformation?
        )
    }
}