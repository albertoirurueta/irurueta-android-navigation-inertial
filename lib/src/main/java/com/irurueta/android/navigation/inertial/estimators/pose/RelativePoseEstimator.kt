/*
 * Copyright (C) 2026 Alberto Irurueta Carro (alberto@irurueta.com)
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
import android.location.Location
import android.os.SystemClock
import com.irurueta.android.navigation.inertial.collectors.AccelerometerAndGyroscopeSyncedSensorCollector
import com.irurueta.android.navigation.inertial.collectors.AccelerometerGravityAndGyroscopeSyncedSensorCollector
import com.irurueta.android.navigation.inertial.collectors.AttitudeAndAccelerometerSyncedSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.AttitudeSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorType
import com.irurueta.android.navigation.inertial.processors.filters.AveragingFilter
import com.irurueta.android.navigation.inertial.processors.filters.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.processors.pose.AccelerometerFusedRelativePoseProcessor
import com.irurueta.android.navigation.inertial.processors.pose.AttitudeRelativePoseProcessor
import com.irurueta.android.navigation.inertial.processors.pose.FusedRelativePoseProcessor
import com.irurueta.geometry.EuclideanTransformation3D
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.SpeedTriad
import com.irurueta.units.Acceleration
import com.irurueta.units.AccelerationUnit

/**
 * Estimates relative pose with 6DOF (Degrees of Freedom) containing relative attitude and position.
 *
 * @property context Android context.
 * @param initialSpeed initial velocity of device expressed in NED coordinates.
 * @property sensorDelay Delay of sensors between samples.
 * @property useAttitudeSensor true to use Android system attitude sensor, false to fuse
 * accelerometer/gravity + gyroscope + magnetometer sensors to estimate attitude.
 * @property useAccelerometerForAttitudeEstimation true to use accelerometer sensor for attitude
 * estimation, false to use system gravity sensor for leveling purposes. Regardless of this
 * values, pose estimator always uses the accelerometer sensor, however, it can be used both for
 * leveling and device motion, or only for device motion. This is ignored if [useAttitudeSensor] is
 * true.
 * @property accelerometerSensorType One of the supported accelerometer sensor types. It is
 * suggested to avoid using non-calibrated accelerometers.
 * @property accelerometerAveragingFilter an averaging filter for accelerometer samples to obtain
 * sensed gravity component of specific force. This is ignored if [useAttitudeSensor] is
 * true.
 * @property gyroscopeSensorType One of the supported gyroscope sensor types. It is suggested to
 * avoid using non-calibrated gyroscopes.
 * @property poseAvailableListener notifies when a new estimated pose is available.
 * @property accuracyChangedListener listener to notify changes in accuracy.
 * @property adjustGravityNorm indicates whether gravity norm must be adjusted to either Earth
 * standard norm, or norm at provided location. If no location is provided, this should only be
 * enabled when device is close to sea level.
 */
class RelativePoseEstimator(
    val context: Context,
    initialSpeed: SpeedTriad = SpeedTriad(),
    val sensorDelay: SensorDelay = SensorDelay.FASTEST,
    val useAttitudeSensor: Boolean = true,
    val useAccelerometerForAttitudeEstimation: Boolean = false,
    val accelerometerSensorType: AccelerometerSensorType =
        AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
    val accelerometerAveragingFilter: AveragingFilter<AccelerationUnit, Acceleration, AccelerationTriad> = LowPassAveragingFilter(),
    val gyroscopeSensorType: GyroscopeSensorType = GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
    useAccurateRelativeGyroscopeAttitudeProcessor: Boolean = true,
    var poseAvailableListener: OnPoseAvailableListener? = null,
    var accuracyChangedListener: OnAccuracyChangedListener? = null,
    location: Location? = null,
    adjustGravityNorm: Boolean = true
) {
    /**
     * Internal processor using fused attitude estimation with gravity + pose estimation.
     */
    private val fusedProcessor = FusedRelativePoseProcessor(initialSpeed)

    /**
     * Internal processor using fused attitude estimation with accelerometer + pose estimation.
     */
    private val accelerometerFusedProcessor = AccelerometerFusedRelativePoseProcessor(initialSpeed)

    /**
     * Internal processor using attitude sensor + pose estimation.
     */
    private val attitudeProcessor =
        AttitudeRelativePoseProcessor(initialSpeed, accelerometerAveragingFilter)

    /**
     * Measurement syncer for internal [fusedProcessor].
     */
    private val fusedCollector = AccelerometerGravityAndGyroscopeSyncedSensorCollector(
        context,
        accelerometerSensorType = accelerometerSensorType,
        gyroscopeSensorType = gyroscopeSensorType,
        accelerometerSensorDelay = sensorDelay,
        gravitySensorDelay = sensorDelay,
        gyroscopeSensorDelay = sensorDelay,
        accuracyChangedListener = { _, sensorType, accuracy ->
            notifyAccuracyChanged(sensorType, accuracy)
        },
        measurementListener = { _, measurement ->

            if (fusedProcessor.process(measurement)) {
                val poseTransformation = fusedProcessor.poseTransformation
                notifyPose(
                    measurement.timestamp,
                    poseTransformation
                )
            }
        }
    )

    /**
     * Measurement syncer for internal [accelerometerFusedProcessor].
     */
    private val accelerometerFusedCollector = AccelerometerAndGyroscopeSyncedSensorCollector(
            context,
            accelerometerSensorType = accelerometerSensorType,
            gyroscopeSensorType = gyroscopeSensorType,
            accelerometerSensorDelay = sensorDelay,
            gyroscopeSensorDelay = sensorDelay,
            accuracyChangedListener = { _, sensorType, accuracy ->
                notifyAccuracyChanged(sensorType, accuracy)
            },
            measurementListener = { _, measurement ->

                if (accelerometerFusedProcessor.process(measurement)) {
                    val poseTransformation = accelerometerFusedProcessor.poseTransformation
                    notifyPose(
                        measurement.timestamp,
                        poseTransformation
                    )
                }
            }
        )

    /**
     * Measurement syncer for internal [attitudeProcessor].
     */
    private val attitudeCollector = AttitudeAndAccelerometerSyncedSensorCollector(
        context,
        attitudeSensorType = AttitudeSensorType.ABSOLUTE_ATTITUDE,
        accelerometerSensorType = accelerometerSensorType,
        attitudeSensorDelay = sensorDelay,
        accelerometerSensorDelay = sensorDelay,
        accuracyChangedListener = { _, sensorType, accuracy ->
            notifyAccuracyChanged(sensorType, accuracy)
        },
        measurementListener = { _, measurement ->
            if (attitudeProcessor.process(measurement)) {
                val poseTransformation = attitudeProcessor.poseTransformation
                notifyPose(
                    measurement.timestamp,
                    poseTransformation
                )
            }
        }
    )

    /**
     * Gets initial device velocity.
     */
    val initialSpeed: SpeedTriad
        get() = if (useAttitudeSensor) {
            attitudeProcessor.initialSpeed
        } else {
            if (useAccelerometerForAttitudeEstimation) {
                accelerometerFusedProcessor.initialSpeed
            } else {
                fusedProcessor.initialSpeed
            }
        }

    /**
     * Indicates whether accurate non-leveled relative attitude processor must be used or not.
     */
    var useAccurateRelativeGyroscopeAttitudeProcessor: Boolean =
        useAccurateRelativeGyroscopeAttitudeProcessor
        private set(value) {
            fusedProcessor.useAccurateRelativeGyroscopeAttitudeProcessor = value
            accelerometerFusedProcessor.useAccurateRelativeGyroscopeAttitudeProcessor = value
            field = value
        }

    /**
     * Indicates whether fusion between leveling and relative attitudes occurs based
     * on changing interpolation value that depends on actual relative attitude rotation
     * velocity.
     */
    var useIndirectAttitudeInterpolation: Boolean
        get() = fusedProcessor.useIndirectAttitudeInterpolation
        set(value) {
            fusedProcessor.useIndirectAttitudeInterpolation = value
            accelerometerFusedProcessor.useIndirectAttitudeInterpolation = value
        }

    /**
     * Interpolation value to be used to combine both leveling and relative attitudes.
     * Must be between 0.0 and 1.0 (both included).
     * The closer to 0.0 this value is, the more resemblance the result will have to a pure
     * leveling (which feels more jerky). On the contrary, the closer to 1.0 this value is,
     * the more resemblance the result will have to a pure non-leveled relative attitude (which
     * feels softer but might have arbitrary roll and pitch Euler angles).
     *
     * @throws IllegalArgumentException if value is not between 0.0 and 1.0 (both included).
     */
    var attitudeInterpolationValue: Double
        get() = fusedProcessor.attitudeInterpolationValue
        @Throws(IllegalArgumentException::class)
        set(value) {
            fusedProcessor.attitudeInterpolationValue = value
            accelerometerFusedProcessor.attitudeInterpolationValue = value
        }

    /**
     * Factor to take into account when interpolation value is computed and
     * [useIndirectAttitudeInterpolation] is enabled to determine actual interpolation value based
     * on current relative attitude rotation velocity.
     *
     * @throws IllegalArgumentException if value is zero or negative.
     */
    var attitudeIndirectInterpolationWeight: Double
        get() = fusedProcessor.attitudeIndirectInterpolationWeight
        @Throws(IllegalArgumentException::class)
        set(value) {
            fusedProcessor.attitudeIndirectInterpolationWeight = value
            accelerometerFusedProcessor.attitudeIndirectInterpolationWeight = value
        }

    /**
     * Threshold to determine that current geomagnetic attitude appears to be an outlier respect
     * to estimated fused attitude.
     * When geomagnetic attitude and fused attitudes diverge, fusion is not performed, and instead
     * only gyroscope relative attitude is used for fusion estimation.
     *
     * @throws IllegalArgumentException if value is not between 0.0 and 1.0 (both included).
     */
    var attitudeOutlierThreshold: Double
        get() = fusedProcessor.attitudeOutlierThreshold
        @Throws(IllegalArgumentException::class)
        set(value) {
            fusedProcessor.attitudeOutlierThreshold = value
            accelerometerFusedProcessor.attitudeOutlierThreshold = value
        }

    /**
     * Threshold to determine that geomagnetic attitude has largely diverged and if situation is not
     * reverted soon, attitude will be reset to geomagnetic one.
     *
     * @throws IllegalArgumentException if value is not between 0.0 and 1.0 (both included).
     */
    var attitudeOutlierPanicThreshold: Double
        get() = fusedProcessor.attitudeOutlierPanicThreshold
        @Throws(IllegalArgumentException::class)
        set(value) {
            fusedProcessor.attitudeOutlierPanicThreshold = value
            accelerometerFusedProcessor.attitudeOutlierPanicThreshold = value
        }

    /**
     * Threshold to determine when fused attitude has largely diverged for a given
     * number of samples and must be reset.
     *
     * @throws IllegalArgumentException if provided value is zero or negative.
     */
    var attitudePanicCounterThreshold: Int
        get() = fusedProcessor.attitudePanicCounterThreshold
        @Throws(IllegalArgumentException::class)
        set(value) {
            fusedProcessor.attitudePanicCounterThreshold = value
            accelerometerFusedProcessor.attitudePanicCounterThreshold = value
        }

    /**
     * Time interval expressed in seconds between consecutive measurements
     */
    val timeIntervalSeconds: Double?
        get() = if (useAttitudeSensor) {
            null
        } else {
            if (useAccelerometerForAttitudeEstimation) {
                accelerometerFusedProcessor.timeIntervalSeconds
            } else {
                fusedProcessor.timeIntervalSeconds
            }
        }

    /**
     * Gets or sets device location
     */
    var location: Location? = location
        set(value) {
            fusedProcessor.location = value
            accelerometerFusedProcessor.location = value
            attitudeProcessor.location = value
            field = value
        }

    /**
     * Indicates whether gravity norm must be adjusted to either Earth
     * standard norm, or norm at provided location. If no location is provided, this should only be
     * enabled when device is close to sea level.
     */
    var adjustGravityNorm: Boolean = adjustGravityNorm
        set(value) {
            check(!running)

            fusedProcessor.adjustGravityNorm = value
            accelerometerFusedProcessor.adjustGravityNorm = value
            attitudeProcessor.adjustGravityNorm = value
            field = value
        }

    /**
     * Indicates whether this estimator is running or not.
     */
    var running: Boolean = false
        private set

    /**
     * Starts this estimator.
     *
     * @param startTimestamp monotonically increasing timestamp when collector starts. If not
     * provided, system clock is used by default, otherwise, the value can be provided to sync
     * multiple sensor collector instances.
     * @return true if estimator successfully started, false otherwise.
     * @throws IllegalStateException if estimator is already running.
     */
    @Throws(IllegalStateException::class)
    fun start(startTimestamp: Long = SystemClock.elapsedRealtimeNanos()): Boolean {
        check(!running)

        running = if (useAttitudeSensor) {
            attitudeProcessor.reset()
            attitudeCollector.start(startTimestamp)
        } else {
            if (useAccelerometerForAttitudeEstimation) {
                accelerometerFusedProcessor.reset()
                accelerometerFusedCollector.start(startTimestamp)
            } else {
                fusedProcessor.reset()
                fusedCollector.start(startTimestamp)
            }
        }

        return running
    }

    /**
     * Stops this estimator.
     */
    fun stop() {
        fusedCollector.stop()
        accelerometerFusedCollector.stop()
        attitudeCollector.stop()
    }

    /**
     * Notifies changes in sensor accuracy.
     */
    private fun notifyAccuracyChanged(
        sensorType: SensorType,
        accuracy: SensorAccuracy?
    ) {
        accuracyChangedListener?.onAccuracyChanged(this, sensorType, accuracy)
    }

    /**
     * Notifies pose.
     */
    private fun notifyPose(
        timestamp: Long,
        poseTransformation: EuclideanTransformation3D
    ) {
        poseAvailableListener?.onPoseAvailable(
            this,
            timestamp,
            poseTransformation
        )
    }

    init {
        this.useAccurateRelativeGyroscopeAttitudeProcessor =
            useAccurateRelativeGyroscopeAttitudeProcessor
    }

    /**
     * Interface to notify when a new pose is available.
     */
    fun interface OnPoseAvailableListener {
        /**
         * Called when a new pose is available.
         *
         * @param estimator pose estimator that raised this event.
         * @param timestamp time in nanoseconds at which the measurement was made. Each measurement
         * wil be monotonically increasing using the same time base as
         * [SystemClock.elapsedRealtimeNanos].
         * @param poseTransformation 3D metric transformation containing leveled attitude and
         * translation variation since this estimator started expressed in ENU system of
         * coordinates.
         */
        fun onPoseAvailable(
            estimator: RelativePoseEstimator,
            timestamp: Long,
            poseTransformation: EuclideanTransformation3D
        )
    }

    /**
     * Interface to notify when sensor (either accelerometer, gravity or magnetometer) accuracy
     * changes.
     */
    fun interface OnAccuracyChangedListener {
        /**
         * Called when accuracy changes.
         *
         * @param estimator pose estimator that raised this event.
         * @param sensorType sensor that has changed its accuracy.
         * @param accuracy new accuracy.
         */
        fun onAccuracyChanged(
            estimator: RelativePoseEstimator,
            sensorType: SensorType,
            accuracy: SensorAccuracy?
        )
    }

}