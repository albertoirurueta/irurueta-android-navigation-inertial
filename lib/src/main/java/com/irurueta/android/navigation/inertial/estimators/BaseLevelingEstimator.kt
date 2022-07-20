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
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.estimators.filter.AveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.LowPassAveragingFilter
import com.irurueta.geometry.InvalidRotationMatrixException
import com.irurueta.geometry.Quaternion
import com.irurueta.geometry.Rotation3D
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType

/**
 * Base class for estimators of device leveling (roll and pitch angles) by estimating gravity vector
 * using accelerometer measurements only.
 * Implementations of this estimator does not estimate attitude yaw angle, as either a magnetometer
 * or gyroscope would be needed.
 *
 * @property context Android context.
 * @property sensorDelay Delay of accelerometer or gravity sensor between samples.
 * @property useAccelerometer true to use accelerometer sensor, false to use system gravity sensor.
 * @property accelerometerSensorType One of the supported accelerometer sensor types.
 * (Only used if [useAccelerometer] is true).
 * @property accelerometerAveragingFilter an averaging filter for accelerometer samples to obtain
 * sensed gravity component of specific force. (Only used if [useAccelerometer] is true).
 * @property estimateCoordinateTransformation true to estimate coordinate transformation, false
 * otherwise. If not needed, it can be disabled to improve performance and decrease cpu load.
 * @property estimateDisplayEulerAngles true to estimate euler angles, false otherwise. If not
 * needed, it can be disabled to improve performance and decrease cpu load.
 * @property levelingAvailableListener listener to notify when a new leveling measurement is
 * available.
 */
abstract class BaseLevelingEstimator<T : BaseLevelingEstimator<T, L>,
        L : BaseLevelingEstimator.OnLevelingAvailableListener<T, L>>(
    val context: Context,
    val sensorDelay: SensorDelay = SensorDelay.GAME,
    val useAccelerometer: Boolean = false,
    val accelerometerSensorType: AccelerometerSensorCollector.SensorType =
        AccelerometerSensorCollector.SensorType.ACCELEROMETER,
    val accelerometerAveragingFilter: AveragingFilter = LowPassAveragingFilter(),
    val estimateCoordinateTransformation: Boolean = false,
    val estimateDisplayEulerAngles: Boolean = true,
    var levelingAvailableListener: L? = null
) {

    /**
     * Instance to be reused containing estimated leveling attitude (roll and pitch angles) in NED
     * coordinates.
     */
    protected val attitude = Quaternion()

    /**
     * Instance to be reused containing display rotation as a yaw angle.
     */
    protected val displayOrientation = Quaternion()

    /**
     * Instance to be reused containing rotation matrix of coordinate transformation.
     */
    private val rotationMatrix = Matrix(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)

    /**
     * Array to be reused containing euler angles of leveling attitude.
     */
    private val displayEulerAngles = DoubleArray(Quaternion.N_ANGLES)

    /**
     * Instance to be reused containing coordinate transformation in NED coordinates.
     */
    protected val coordinateTransformation =
        CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME)

    /**
     * Internal gravity estimator sensed as a component of specific force.
     */
    protected abstract val gravityEstimator: GravityEstimator

    /**
     * Indicates whether this estimator is running or not.
     */
    val running
        get() = gravityEstimator.running

    /**
     * Starts this estimator.
     *
     * @return true if estimator successfully started, false otherwise.
     * @throws IllegalStateException if estimator is already running.
     */
    @Throws(IllegalStateException::class)
    fun start(): Boolean {
        return gravityEstimator.start()
    }

    /**
     * Stops this estimator.
     */
    fun stop() {
        gravityEstimator.stop()
    }

    /**
     * Processes current attitude to take into account display orientation and compute
     * (if needed) a coordinate transformation or display Euler angles.
     */
    protected fun postProcessAttitudeAndNotify() {
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
        if (estimateDisplayEulerAngles) {
            attitude.toEulerAngles(displayEulerAngles)
            displayRoll = displayEulerAngles[0]
            displayPitch = displayEulerAngles[1]
        } else {
            displayRoll = null
            displayPitch = null
        }

        // notify
        @Suppress("UNCHECKED_CAST")
        levelingAvailableListener?.onLevelingAvailable(
            this as T,
            attitude,
            displayRoll,
            displayPitch,
            c
        )
    }

    /**
     * Interface to notify when a new leveling measurement is available.
     */
    fun interface OnLevelingAvailableListener<T : BaseLevelingEstimator<T, L>,
            L : OnLevelingAvailableListener<T, L>> {
        /**
         * Called when a new leveling measurement is available.
         *
         * @param estimator leveling estimator that raised this event.
         * @param attitude leveling attitude (roll and pitch angles) expressed in NED coordinates.
         * @param roll roll angle expressed in radians. Only available if
         * [estimateDisplayEulerAngles].
         * @param pitch pitch angle expressed in radians. Only available if
         * [estimateDisplayEulerAngles].
         * @param coordinateTransformation coordinate transformation containing measured leveling
         * attitude. Only available if [estimateCoordinateTransformation].
         */
        fun onLevelingAvailable(
            estimator: T,
            attitude: Quaternion,
            roll: Double?,
            pitch: Double?,
            coordinateTransformation: CoordinateTransformation?
        )
    }
}