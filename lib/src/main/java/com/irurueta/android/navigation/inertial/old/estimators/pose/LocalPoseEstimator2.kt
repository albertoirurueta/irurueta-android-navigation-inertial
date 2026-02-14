/*
 * Copyright (C) 2023 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.android.navigation.inertial.old.estimators.pose

import android.content.Context
import android.location.Location
import android.os.SystemClock
import com.irurueta.android.navigation.inertial.old.estimators.filter.AveragingFilter
import com.irurueta.android.navigation.inertial.old.estimators.filter.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.old.collectors.AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer
import com.irurueta.android.navigation.inertial.old.collectors.AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.old.collectors.AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncer
import com.irurueta.android.navigation.inertial.collectors.measurements.AttitudeSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.old.collectors.SensorType
import com.irurueta.android.navigation.inertial.old.processors.pose.AccelerometerDoubleFusedLocalPoseProcessor
import com.irurueta.android.navigation.inertial.old.processors.pose.AccelerometerFusedLocalPoseProcessor
import com.irurueta.android.navigation.inertial.old.processors.pose.AttitudeLocalPoseProcessor
import com.irurueta.android.navigation.inertial.old.processors.pose.DoubleFusedLocalPoseProcessor
import com.irurueta.android.navigation.inertial.old.processors.pose.FusedLocalPoseProcessor
import com.irurueta.geometry.EuclideanTransformation3D
import com.irurueta.navigation.frames.ECEFFrame
import com.irurueta.navigation.frames.NEDVelocity
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel
import java.util.*

/**
 * Estimates pose with 6DOF (Degrees of Freedom) containing absolute attitude and position using
 * local plane navigation.
 *
 * @property context Android context.
 * @param initialLocation initial device location.
 * @param initialVelocity initial velocity of device expressed in NED coordinates.
 * @property sensorDelay Delay of sensors between samples.
 * @property useAttitudeSensor true to use Android system attitude sensor, false to fuse
 * accelerometer/gravity + gyroscope + magnetometer sensors to estimate attitude.
 * @property useAccelerometerForAttitudeEstimation true to use accelerometer sensor for attitude
 * estimation, false to use system gravity sensor for leveling purposes. Regardless of this
 * values, pose estimator always uses the accelerometer sensor, however, it can be used both for
 * leveling and device motion, or only for device motion. This is ignored if [useAttitudeSensor] is
 * true.
 * @property startOffsetEnabled indicates whether start offsets will be computed when first
 * measurement is received or not. True indicates that offset is computed, false assumes that offset
 * is null.
 * @property accelerometerSensorType One of the supported accelerometer sensor types. It is
 * suggested to avoid using non-calibrated accelerometers.
 * @property magnetometerSensorType One of the supported magnetometer sensor types. This is ignored
 * if [useAttitudeSensor] is true.
 * @property accelerometerAveragingFilter an averaging filter for accelerometer samples to obtain
 * sensed gravity component of specific force. This is ignored if [useAttitudeSensor] is
 * true.
 * @property gyroscopeSensorType One of the supported gyroscope sensor types. It is suggested to
 * avoid using non-calibrated gyroscopes.
 * @param worldMagneticModel Earth's magnetic model. Null to use default model
 * when [useWorldMagneticModel] is true. If [useWorldMagneticModel] is false, this is ignored. This
 * is ignored if [useAttitudeSensor] is true.
 * @param timestamp Timestamp when World Magnetic Model will be evaluated to obtain current.
 * Only taken into account if [useWorldMagneticModel] is tue. This is ignored if
 * [useAttitudeSensor] is true.
 * @param useWorldMagneticModel true so that world magnetic model is taken into account to
 * adjust attitude yaw angle by current magnetic declination based on current World Magnetic
 * Model, location and timestamp, false to ignore declination. This is ignored if
 * [useAttitudeSensor] is true.
 * @param useAccurateLevelingProcessor true to use accurate leveling, false to use a normal one.
 * This is ignored if [useAttitudeSensor] is true.
 * @property useDoubleFusedAttitudeProcessor true to use double fused absolute attitude estimator,
 * false otherwise. This is ignored if [useAttitudeSensor] is true.
 * @param estimatePoseTransformation true to estimate 3D metric pose transformation, false to
 * skip transformation computation.
 * @property poseAvailableListener notifies when a new estimated pose is available.
 * @property accuracyChangedListener listener to notify changes in accuracy.
 * @property bufferFilledListener listener to notify that some buffer has been filled. This usually
 * happens when consumer of measurements cannot keep up with the rate at which measurements are
 * generated.
 * @property adjustGravityNorm indicates whether gravity norm must be adjusted to either Earth
 * standard norm, or norm at provided location. If no location is provided, this should only be
 * enabled when device is close to sea level.
 */
class LocalPoseEstimator2(
    val context: Context,
    initialLocation: Location,
    initialVelocity: NEDVelocity = NEDVelocity(),
    val sensorDelay: SensorDelay = SensorDelay.FASTEST,
    val useAttitudeSensor: Boolean = true,
    val useAccelerometerForAttitudeEstimation: Boolean = false,
    val startOffsetEnabled: Boolean = true,
    val accelerometerSensorType: AccelerometerSensorType =
        AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
    val magnetometerSensorType: MagnetometerSensorType =
        MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
    val accelerometerAveragingFilter: AveragingFilter = LowPassAveragingFilter(),
    val gyroscopeSensorType: GyroscopeSensorType = GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
    worldMagneticModel: WorldMagneticModel? = null,
    timestamp: Date? = null,
    useWorldMagneticModel: Boolean = false,
    useAccurateLevelingProcessor: Boolean = true,
    useAccurateRelativeGyroscopeAttitudeProcessor: Boolean = true,
    val useDoubleFusedAttitudeProcessor: Boolean = true,
    estimatePoseTransformation: Boolean = false,
    var poseAvailableListener: OnPoseAvailableListener? = null,
    var accuracyChangedListener: OnAccuracyChangedListener? = null,
    var bufferFilledListener: OnBufferFilledListener? = null,
    adjustGravityNorm: Boolean = true
) {
    /**
     * Internal processor using fused attitude estimation with gravity + pose estimation.
     */
    private val fusedProcessor =
        FusedLocalPoseProcessor(initialLocation, initialVelocity, estimatePoseTransformation)

    /**
     * Internal processor using fused attitude estimation with accelerometer + pose estimation.
     */
    private val accelerometerFusedProcessor = AccelerometerFusedLocalPoseProcessor(
        initialLocation,
        initialVelocity,
        estimatePoseTransformation
    )

    /**
     * Internal processor using double fused attitude estimation with gravity + pose estimation.
     */
    private val doubleFusedProcessor =
        DoubleFusedLocalPoseProcessor(initialLocation, initialVelocity, estimatePoseTransformation)

    /**
     * Internal processor using double fused attitude estimation with accelerometer + pose
     * estimation.
     */
    private val accelerometerDoubleFusedProcessor = AccelerometerDoubleFusedLocalPoseProcessor(
        initialLocation,
        initialVelocity,
        estimatePoseTransformation
    )

    /**
     * Internal processor using attitude sensor + pose estimation.
     */
    private val attitudeProcessor =
        AttitudeLocalPoseProcessor(initialLocation, initialVelocity, estimatePoseTransformation)

    /**
     * Measurement syncer for internal [fusedProcessor].
     */
    private val fusedSyncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
        context,
        accelerometerSensorType,
        gyroscopeSensorType,
        magnetometerSensorType,
        sensorDelay,
        sensorDelay,
        sensorDelay,
        accelerometerStartOffsetEnabled = startOffsetEnabled,
        gravityStartOffsetEnabled = startOffsetEnabled,
        gyroscopeStartOffsetEnabled = startOffsetEnabled,
        magnetometerStartOffsetEnabled = startOffsetEnabled,
        stopWhenFilledBuffer = false,
        staleDetectionEnabled = true,
        accuracyChangedListener = { _, sensorType, accuracy ->
            notifyAccuracyChanged(sensorType, accuracy)
        },
        bufferFilledListener = { _, sensorType ->
            notifyBufferFilled(sensorType)
        },
        syncedMeasurementListener = { _, measurement ->
            if (useDoubleFusedAttitudeProcessor) {
                if (doubleFusedProcessor.process(measurement)) {
                    val poseTransformation =
                        if (estimatePoseTransformation) doubleFusedProcessor.poseTransformation else null
                    notifyPose(
                        doubleFusedProcessor.currentEcefFrame,
                        doubleFusedProcessor.previousEcefFrame,
                        doubleFusedProcessor.initialEcefFrame,
                        measurement.timestamp,
                        poseTransformation
                    )
                }
            } else {
                if (fusedProcessor.process(measurement)) {
                    val poseTransformation =
                        if (estimatePoseTransformation) fusedProcessor.poseTransformation else null
                    notifyPose(
                        fusedProcessor.currentEcefFrame,
                        fusedProcessor.previousEcefFrame,
                        fusedProcessor.initialEcefFrame,
                        measurement.timestamp,
                        poseTransformation
                    )
                }
            }
        }
    )

    /**
     * Measurement syncer for internal [accelerometerFusedProcessor].
     */
    private val accelerometerFusedSyncer =
        AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer(
            context,
            accelerometerSensorType,
            gyroscopeSensorType,
            magnetometerSensorType,
            sensorDelay,
            sensorDelay,
            sensorDelay,
            accelerometerStartOffsetEnabled = startOffsetEnabled,
            gyroscopeStartOffsetEnabled = startOffsetEnabled,
            magnetometerStartOffsetEnabled = startOffsetEnabled,
            stopWhenFilledBuffer = false,
            staleDetectionEnabled = true,
            accuracyChangedListener = { _, sensorType, accuracy ->
                notifyAccuracyChanged(sensorType, accuracy)
            },
            bufferFilledListener = { _, sensorType ->
                notifyBufferFilled(sensorType)
            },
            syncedMeasurementListener = { _, measurement ->
                if (useDoubleFusedAttitudeProcessor) {
                    if (accelerometerDoubleFusedProcessor.process(measurement)) {
                        val poseTransformation =
                            if (estimatePoseTransformation)
                                accelerometerDoubleFusedProcessor.poseTransformation
                            else
                                null
                        notifyPose(
                            accelerometerDoubleFusedProcessor.currentEcefFrame,
                            accelerometerDoubleFusedProcessor.previousEcefFrame,
                            accelerometerDoubleFusedProcessor.initialEcefFrame,
                            measurement.timestamp,
                            poseTransformation
                        )
                    }
                } else {
                    if (accelerometerFusedProcessor.process(measurement)) {
                        val poseTransformation =
                            if (estimatePoseTransformation)
                                accelerometerFusedProcessor.poseTransformation
                            else
                                null
                        notifyPose(
                            accelerometerFusedProcessor.currentEcefFrame,
                            accelerometerFusedProcessor.previousEcefFrame,
                            accelerometerFusedProcessor.initialEcefFrame,
                            measurement.timestamp,
                            poseTransformation
                        )
                    }
                }
            }
        )

    /**
     * Measurement syncer for internal [attitudeProcessor].
     */
    private val attitudeSyncer = AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncer(
        context,
        attitudeSensorType = AttitudeSensorType.ABSOLUTE_ATTITUDE,
        accelerometerSensorType,
        gyroscopeSensorType,
        sensorDelay,
        sensorDelay,
        sensorDelay,
        attitudeStartOffsetEnabled = startOffsetEnabled,
        accelerometerStartOffsetEnabled = startOffsetEnabled,
        gyroscopeStartOffsetEnabled = startOffsetEnabled,
        stopWhenFilledBuffer = false,
        staleDetectionEnabled = true,
        accuracyChangedListener = { _, sensorType, accuracy ->
            notifyAccuracyChanged(sensorType, accuracy)
        },
        bufferFilledListener = { _, sensorType ->
            notifyBufferFilled(sensorType)
        },
        syncedMeasurementListener = { _, measurement ->
            if (attitudeProcessor.process(measurement)) {
                val poseTransformation =
                    if (estimatePoseTransformation) attitudeProcessor.poseTransformation else null
                notifyPose(
                    attitudeProcessor.currentEcefFrame,
                    attitudeProcessor.previousEcefFrame,
                    attitudeProcessor.initialEcefFrame,
                    measurement.timestamp,
                    poseTransformation
                )
            }
        }
    )

    /**
     * Gets initial device location.
     */
    val initialLocation: Location
        get() = if (useAttitudeSensor) {
            attitudeProcessor.initialLocation
        } else {
            if (useDoubleFusedAttitudeProcessor) {
                if (useAccelerometerForAttitudeEstimation) {
                    accelerometerDoubleFusedProcessor.initialLocation
                } else {
                    doubleFusedProcessor.initialLocation
                }
            } else {
                if (useAccelerometerForAttitudeEstimation) {
                    accelerometerFusedProcessor.initialLocation
                } else {
                    fusedProcessor.initialLocation
                }
            }
        }

    /**
     * Gets initial device velocity.
     */
    val initialVelocity: NEDVelocity
        get() = if (useAttitudeSensor) {
            attitudeProcessor.initialVelocity
        } else {
            if (useDoubleFusedAttitudeProcessor) {
                if (useAccelerometerForAttitudeEstimation) {
                    accelerometerDoubleFusedProcessor.initialVelocity
                } else {
                    doubleFusedProcessor.initialVelocity
                }
            } else {
                if (useAccelerometerForAttitudeEstimation) {
                    accelerometerFusedProcessor.initialVelocity
                } else {
                    fusedProcessor.initialVelocity
                }
            }
        }

    /**
     * True to estimate 3D metric pose transformation, false otherwise.
     */
    val estimatePoseTransformation: Boolean
        get() = if (useAttitudeSensor) {
            attitudeProcessor.estimatePoseTransformation
        } else {
            if (useDoubleFusedAttitudeProcessor) {
                if (useAccelerometerForAttitudeEstimation) {
                    accelerometerDoubleFusedProcessor.estimatePoseTransformation
                } else {
                    doubleFusedProcessor.estimatePoseTransformation
                }
            } else {
                if (useAccelerometerForAttitudeEstimation) {
                    accelerometerFusedProcessor.estimatePoseTransformation
                } else {
                    fusedProcessor.estimatePoseTransformation
                }
            }
        }

    /**
     * Indicates whether accurate leveling must be used or not.
     */
    var useAccurateLevelingProcessor: Boolean = useAccurateLevelingProcessor
        private set(value) {
            fusedProcessor.useAccurateLevelingProcessor = value
            accelerometerFusedProcessor.useAccurateLevelingProcessor = value
            doubleFusedProcessor.useAccurateLevelingProcessor = value
            accelerometerDoubleFusedProcessor.useAccurateLevelingProcessor = value
            field = value
        }

    /**
     * Indicates whether world magnetic model is taken into account to adjust attitude yaw angle by
     * current magnetic declination based on current World Magnetic Model, location and timestamp.
     * If null, the default model is used if [useWorldMagneticModel] is
     * true. If [useWorldMagneticModel] is false, this is ignored.
     */
    var worldMagneticModel: WorldMagneticModel? = worldMagneticModel
        private set(value) {
            fusedProcessor.worldMagneticModel = value
            accelerometerFusedProcessor.worldMagneticModel = value
            doubleFusedProcessor.worldMagneticModel = value
            accelerometerDoubleFusedProcessor.worldMagneticModel = value
            field = value
        }

    /**
     * Indicates whether world magnetic model is taken into account to adjust attitude yaw angle by
     * current magnetic declination based on current World Magnetic Model, location and timestamp.
     * If null, the default model is used if [useWorldMagneticModel] is
     * true. If [useWorldMagneticModel] is false, this is ignored.
     */
    var useWorldMagneticModel: Boolean = useWorldMagneticModel
        private set(value) {
            fusedProcessor.useWorldMagneticModel = value
            accelerometerFusedProcessor.useWorldMagneticModel = value
            doubleFusedProcessor.useWorldMagneticModel = value
            accelerometerDoubleFusedProcessor.useWorldMagneticModel = value
            field = value
        }

    /**
     * Timestamp being used when World Magnetic Model is evaluated to obtain current magnetic
     * declination. This is only taken into account if [useWorldMagneticModel] is true.
     * If not defined, current date is assumed.
     */
    var timestamp: Date? = timestamp
        set(value) {
            fusedProcessor.currentDate = value
            accelerometerFusedProcessor.currentDate = value
            doubleFusedProcessor.currentDate = value
            accelerometerDoubleFusedProcessor.currentDate = value
            field = value
        }

    /**
     * Indicates whether accurate non-leveled relative attitude processor must be used or not.
     */
    var useAccurateRelativeGyroscopeAttitudeProcessor: Boolean =
        useAccurateRelativeGyroscopeAttitudeProcessor
        private set(value) {
            fusedProcessor.useAccurateRelativeGyroscopeAttitudeProcessor = value
            accelerometerFusedProcessor.useAccurateRelativeGyroscopeAttitudeProcessor = value
            doubleFusedProcessor.useAccurateRelativeGyroscopeAttitudeProcessor = value
            accelerometerDoubleFusedProcessor.useAccurateRelativeGyroscopeAttitudeProcessor = value
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
            doubleFusedProcessor.useIndirectAttitudeInterpolation = value
            accelerometerDoubleFusedProcessor.useIndirectAttitudeInterpolation = value
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
            doubleFusedProcessor.attitudeInterpolationValue = value
            accelerometerDoubleFusedProcessor.attitudeInterpolationValue = value
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
            doubleFusedProcessor.attitudeIndirectInterpolationWeight = value
            accelerometerDoubleFusedProcessor.attitudeIndirectInterpolationWeight = value
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
            doubleFusedProcessor.attitudeOutlierThreshold = value
            accelerometerDoubleFusedProcessor.attitudeOutlierThreshold = value
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
            doubleFusedProcessor.attitudeOutlierPanicThreshold = value
            accelerometerDoubleFusedProcessor.attitudeOutlierPanicThreshold = value
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
            doubleFusedProcessor.attitudePanicCounterThreshold = value
            accelerometerDoubleFusedProcessor.attitudePanicCounterThreshold = value
        }

    /**
     * Time interval expressed in seconds between consecutive measurements
     */
    val timeIntervalSeconds: Double?
        get() = if (useAttitudeSensor) {
            null
        } else {
            if (useDoubleFusedAttitudeProcessor) {
                if (useAccelerometerForAttitudeEstimation) {
                    accelerometerDoubleFusedProcessor.gyroscopeTimeIntervalSeconds
                } else {
                    doubleFusedProcessor.gyroscopeTimeIntervalSeconds
                }
            } else {
                if (useAccelerometerForAttitudeEstimation) {
                    accelerometerFusedProcessor.gyroscopeTimeIntervalSeconds
                } else {
                    fusedProcessor.gyroscopeTimeIntervalSeconds
                }
            }
        }

    /**
     * True indicates that attitude is leveled and expressed respect to estimator start.
     * False indicates that attitude is absolute.
     *
     * @throws IllegalStateException if estimator is running
     */
    var useLeveledRelativeAttitudeRespectStart = true
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)

            fusedProcessor.useLeveledRelativeAttitudeRespectStart = value
            accelerometerFusedProcessor.useLeveledRelativeAttitudeRespectStart = value
            doubleFusedProcessor.useLeveledRelativeAttitudeRespectStart = value
            accelerometerDoubleFusedProcessor.useLeveledRelativeAttitudeRespectStart = value
            attitudeProcessor.useLeveledRelativeAttitudeRespectStart = value
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
            doubleFusedProcessor.adjustGravityNorm = value
            accelerometerDoubleFusedProcessor.adjustGravityNorm = value
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
            attitudeSyncer.start(startTimestamp)
        } else {
            if (useAccelerometerForAttitudeEstimation) {
                if (useDoubleFusedAttitudeProcessor) {
                    accelerometerDoubleFusedProcessor.reset()
                } else {
                    accelerometerFusedProcessor.reset()
                }
                accelerometerFusedSyncer.start(startTimestamp)
            } else {
                if (useDoubleFusedAttitudeProcessor) {
                    doubleFusedProcessor.reset()
                } else {
                    fusedProcessor.reset()
                }
                fusedSyncer.start(startTimestamp)
            }
        }

        return running
    }

    /**
     * Stops this estimator.
     */
    fun stop() {
        fusedSyncer.stop()
        accelerometerFusedSyncer.stop()
        attitudeSyncer.stop()
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
     * Notifies when a sensor buffer becomes full.
     */
    private fun notifyBufferFilled(sensorType: SensorType) {
        bufferFilledListener?.onBufferFilled(this, sensorType)
    }

    /**
     * Notifies pose.
     */
    private fun notifyPose(
        currentEcefFrame: ECEFFrame,
        previousEcefFrame: ECEFFrame,
        initialEcefFrame: ECEFFrame,
        timestamp: Long,
        poseTransformation: EuclideanTransformation3D?
    ) {
        poseAvailableListener?.onPoseAvailable(
            this,
            currentEcefFrame,
            previousEcefFrame,
            initialEcefFrame,
            timestamp,
            poseTransformation
        )
    }

    init {
        this.useAccurateLevelingProcessor = useAccurateLevelingProcessor
        this.worldMagneticModel = worldMagneticModel
        this.useWorldMagneticModel = useWorldMagneticModel
        this.timestamp = timestamp
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
         * @param currentFrame current ECEF frame containing device position, velocity and attitude.
         * @param previousFrame ECEF frame of previous measurement.
         * @param initialFrame initial ECEF frame when estimator was started.
         * @param timestamp time in nanoseconds at which the measurement was made. Each measurement
         * wil be monotonically increasing using the same time base as
         * [SystemClock.elapsedRealtimeNanos].
         * @param poseTransformation 3D metric transformation containing leveled attitude and
         * translation variation since this estimator started expressed in ENU system of
         * coordinates.
         */
        fun onPoseAvailable(
            estimator: LocalPoseEstimator2,
            currentFrame: ECEFFrame,
            previousFrame: ECEFFrame,
            initialFrame: ECEFFrame,
            timestamp: Long,
            poseTransformation: EuclideanTransformation3D?
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
            estimator: LocalPoseEstimator2,
            sensorType: SensorType,
            accuracy: SensorAccuracy?
        )
    }

    /**
     * Interface to notify when a buffer gets completely filled.
     * When buffers get filled, internal collectors will continue collection at the expense of
     * loosing old data. Consumers of this listener should device what to do at this point (which
     * might require stopping this estimator)..
     */
    fun interface OnBufferFilledListener {
        /**
         * Called when any buffer gets completely filled.
         *
         * @param estimator pose estimator that raised this event.
         * @param sensorType sensor that got its buffer filled.
         */
        fun onBufferFilled(
            estimator: LocalPoseEstimator2,
            sensorType: SensorType
        )
    }
}