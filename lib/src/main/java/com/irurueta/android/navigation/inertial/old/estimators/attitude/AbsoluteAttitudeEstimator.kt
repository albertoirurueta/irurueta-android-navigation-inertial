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
import android.location.Location
import com.irurueta.android.navigation.inertial.old.estimators.filter.AveragingFilter
import com.irurueta.android.navigation.inertial.old.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.old.collectors.GravitySensorCollector
import com.irurueta.android.navigation.inertial.old.collectors.GyroscopeSensorCollector
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.old.collectors.MagnetometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorType
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel
import java.util.*

/**
 * Interface for absolute attitude estimators.
 */
interface AbsoluteAttitudeEstimator<T : AbsoluteAttitudeEstimator<T, L>,
        L : AbsoluteAttitudeEstimator.OnAttitudeAvailableListener<T>> {

    /**
     * Gets Android Context.
     */
    val context: Context

    /**
     * Gets delay of sensors between samples.
     */
    val sensorDelay: SensorDelay

    /**
     * Indicates whether accelerometer or gravity sensor is used.
     * True to use accelerometer sensor, false to use system gravity sensor
     * for leveling purposes.
     */
    val useAccelerometer: Boolean

    /**
     * Gets one of the supported accelerometer sensor types.
     * (Only used if [useAccelerometer] is true).
     */
    val accelerometerSensorType: AccelerometerSensorType

    /**
     * Gets one of the supported magnetometer sensor types.
     */
    val magnetometerSensorType: MagnetometerSensorType

    /**
     * Gets an averaging filter for accelerometer samples to obtain
     * sensed gravity component of specific force. (Only used if [useAccelerometer] is true).
     */
    val accelerometerAveragingFilter: AveragingFilter

    /**
     * Gets one of the supported gyroscope sensor types.
     */
    val gyroscopeSensorType: GyroscopeSensorType

    /**
     * Indicates whether coordinate transformation must be estimated or not.
     * True to estimate coordinate transformation, false otherwise. If not needed, it can be
     * disabled to improve performance and decrease cpu load.
     */
    val estimateCoordinateTransformation: Boolean

    /**
     * Indicates whether Euler angles must be estimated or not.
     * True to estimate euler angles, false otherwise. If not needed, it can be disabled to
     * improve performance and decrease cpu load.
     */
    val estimateEulerAngles: Boolean

    /**
     * Gets or sets listener to notify when a new attitude measurement is available.
     */
    var attitudeAvailableListener: L?

    /**
     * Listener to notify new accelerometer measurements.
     * (Only used if [useAccelerometer] is true).
     */
    var accelerometerMeasurementListener: AccelerometerSensorCollector.OnMeasurementListener?

    /**
     * Listener to notify new gravity measurements.
     * (Only used if [useAccelerometer] is false).
     */
    var gravityMeasurementListener: GravitySensorCollector.OnMeasurementListener?

    /**
     * Listener to notify new gyroscope measurements.
     */
    var gyroscopeMeasurementListener: GyroscopeSensorCollector.OnMeasurementListener?

    /**
     * Listener to notify new magnetometer measurements.
     */
    var magnetometerMeasurementListener: MagnetometerSensorCollector.OnMeasurementListener?

    /**
     * Listener to notify when a new gravity estimation is
     * available.
     */
    var gravityEstimationListener: GravityEstimator.OnEstimationListener?

    /**
     * Gets or sets device location
     *
     * @throws IllegalStateException if estimator is running and a null value is set.
     */
    var location: Location?

    /**
     * Earth's magnetic model. If null, the default model is used if [useWorldMagneticModel] is
     * true. If [useWorldMagneticModel] is false, this is ignored.
     *
     * @throws IllegalStateException if estimator is running.
     */
    var worldMagneticModel: WorldMagneticModel?

    /**
     * Timestamp when World Magnetic Model will be evaluated to obtain current.
     * Only taken into account if [useWorldMagneticModel] is tue.
     */
    var timestamp: Date

    /**
     * Indicates whether world magnetic model is taken into account to adjust attitude yaw angle by
     * current magnetic declination based on current World Magnetic Model, location and timestamp.
     */
    var useWorldMagneticModel: Boolean

    /**
     * Indicates whether accurate leveling must be used or not.
     *
     * @throws IllegalStateException if estimator is running.
     */
    var useAccurateLevelingEstimator: Boolean

    /**
     * Indicates whether accurate non-leveled relative attitude estimator must be used or not.
     *
     * @throws IllegalStateException if estimator is running.
     */
    var useAccurateRelativeGyroscopeAttitudeEstimator: Boolean

    /**
     * Indicates whether fusion between leveling and relative attitudes occurs based
     * on changing interpolation value that depends on actual relative attitude rotation
     * velocity.
     */
    var useIndirectInterpolation: Boolean

    /**
     * Interpolation value to be used to combine both geomagnetic and relative attitudes.
     * Must be between 0.0 and 1.0 (both included).
     * The closer to 0.0 this value is, the more resemblance the result will have to a pure
     * geomagnetic (which feels more jerky when using accelerometer). On the contrary, the closer
     * to 1.0 this value is, the more resemblance the result will have to a pure non-leveled
     * relative attitude (which feels softer but might have arbitrary roll pitch and yaw Euler
     * angles).
     *
     * @throws IllegalArgumentException if value is not between 0.0 and 1.0 (both included).
     */
    var interpolationValue: Double

    /**
     * Factor to take into account when interpolation value is computed and
     * [useIndirectInterpolation] is enabled to determine actual interpolation value based
     * on current relative attitude rotation velocity.
     *
     * @throws IllegalArgumentException if value is zero or negative.
     */
    var indirectInterpolationWeight: Double

    /**
     * Gets average time interval between gyroscope samples expressed in seconds.
     */
    val gyroscopeAverageTimeInterval: Double

    /**
     * Indicates whether this estimator is running or not.
     */
    val running: Boolean

    /**
     * Threshold to determine that current geomagnetic attitude appears to be an outlier respect
     * to estimated fused attitude.
     * When geomagnetic attitude and fused attitudes diverge, fusion is not performed, and instead
     * only gyroscope relative attitude is used for fusion estimation.
     *
     * @throws IllegalStateException if estimator is already running.
     * @throws IllegalArgumentException if value is not between 0.0 and 1.0 (both included).
     */
    var outlierThreshold: Double

    /**
     * Threshold to determine that geomagnetic attitude has largely diverged and if situation is not
     * reverted soon, attitude will be reset to geomagnetic one.
     *
     * @throws IllegalStateException if estimator is already running.
     * @throws IllegalArgumentException if value is not between 0.0 and 1.0 (both included).
     */
    var outlierPanicThreshold: Double

    /**
     * Threshold to determine when fused attitude has largely diverged for a given
     * number of samples and must be reset.
     *
     * @throws IllegalStateException if estimator is already running.
     */
    var panicCounterThreshold: Int

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

    /**
     * Interface to notify when a new attitude measurement is available.
     */
    fun interface OnAttitudeAvailableListener<T : AbsoluteAttitudeEstimator<T, *>> {

        /**
         * Called when a new attitude measurement is available.
         *
         * @param estimator attitude estimator that raised this event.
         * @param attitude attitude expressed in NED coordinates.
         * @param timestamp time in nanoseconds at which the measurement was made. Each measurement
         * wil be monotonically increasing using the same time base as
         * [android.os.SystemClock.elapsedRealtimeNanos].
         * @param roll roll angle expressed in radians respect to NED coordinate system. Only
         * available if [estimateEulerAngles] is true.
         * @param pitch pitch angle expressed in radians respect to NED coordinate system. Only
         * available if [estimateEulerAngles] is true.
         * @param yaw yaw angle expressed in radians respect to NED coordinate system. Only
         * available if [estimateEulerAngles] is true.
         * @param coordinateTransformation coordinate transformation containing measured leveled
         * geomagnetic attitude. Only available if [estimateCoordinateTransformation] is true.
         */
        fun onAttitudeAvailable(
            estimator: T,
            attitude: Quaternion,
            timestamp: Long,
            roll: Double?,
            pitch: Double?,
            yaw: Double?,
            coordinateTransformation: CoordinateTransformation?
        )
    }
}