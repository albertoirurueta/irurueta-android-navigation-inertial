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
package com.irurueta.android.navigation.inertial.estimators.attitude

import android.content.Context
import android.location.Location
import android.os.SystemClock
import com.irurueta.android.navigation.inertial.collectors.*
import com.irurueta.android.navigation.inertial.estimators.filter.AveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.processors.attitude.AccelerometerGeomagneticAttitudeProcessor
import com.irurueta.android.navigation.inertial.processors.attitude.GeomagneticAttitudeProcessor
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel
import java.util.*

/**
 * Estimates a leveled absolute attitude using accelerometer (or gravity) and magnetometer sensors.
 * Gyroscope is not used in this estimator.
 * Roll and pitch Euler angles are leveled using accelerometer or gravity sensors.
 * Yaw angle is obtained from magnetometer once the leveling is estimated.
 *
 * @property context Android context.
 * @param location Device location.
 * @property sensorDelay Delay of sensors between samples.
 * @property useAccelerometer true to use accelerometer sensor, false to use system gravity sensor
 * for leveling purposes.
 * @property startOffsetEnabled indicates whether start offsets will be computed when first
 * measurement is received or not. True indicates that offset is computed, false assumes that offset
 * is null.
 * @property accelerometerSensorType One of the supported accelerometer sensor types.
 * (Only used if [useAccelerometer] is true).
 * @property magnetometerSensorType One of the supported magnetometer sensor types.
 * @property accelerometerAveragingFilter an averaging filter for accelerometer samples to obtain
 * sensed gravity component of specific force. (Only used if [useAccelerometer] is true).
 * @param worldMagneticModel Earth's magnetic model. Null to use default model
 * when [useWorldMagneticModel] is true. If [useWorldMagneticModel] is false, this is ignored.
 * @param timestamp Timestamp when World Magnetic Model will be evaluated to obtain current magnetic
 * declination. Only taken into account if [useWorldMagneticModel] is true.
 * @param useWorldMagneticModel true so that world magnetic model is taken into account to
 * adjust attitude yaw angle by current magnetic declination based on current World Magnetic
 * Model, location and timestamp, false to ignore declination.
 * @param useAccurateLevelingEstimator true to use accurate leveling, false to use a normal one.
 * @property estimateCoordinateTransformation true to estimate coordinate transformation, false
 * otherwise. If not needed, it can be disabled to improve performance and decrease cpu load.
 * @property estimateEulerAngles true to estimate euler angles, false otherwise. If not
 * needed, it can be disabled to improve performance and decrease cpu load.
 * @property attitudeAvailableListener listener to notify when a new attitude measurement is
 * available.
 * @property accuracyChangedListener listener to notify changes in accuracy.
 * @property bufferFilledListener listener to notify that some buffer has been filled. This usually
 * happens when consumer of measurements cannot keep up with the rate at which measurements are
 * generated.
 * @property adjustGravityNorm indicates whether gravity norm must be adjusted to either Earth
 * standard norm, or norm at provided location. If no location is provided, this should only be
 * enabled when device is close to sea level.
 */
class GeomagneticAttitudeEstimator2(
    val context: Context,
    location: Location? = null,
    val sensorDelay: SensorDelay = SensorDelay.GAME,
    val useAccelerometer: Boolean = true,
    val startOffsetEnabled: Boolean = true,
    val accelerometerSensorType: AccelerometerSensorType =
        AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
    val magnetometerSensorType: MagnetometerSensorType =
        MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
    val accelerometerAveragingFilter: AveragingFilter = LowPassAveragingFilter(),
    worldMagneticModel: WorldMagneticModel? = null,
    timestamp: Date? = null,
    useWorldMagneticModel: Boolean = false,
    useAccurateLevelingEstimator: Boolean = false,
    val estimateCoordinateTransformation: Boolean = false,
    val estimateEulerAngles: Boolean = true,
    var attitudeAvailableListener: OnAttitudeAvailableListener? = null,
    var accuracyChangedListener: OnAccuracyChangedListener? = null,
    var bufferFilledListener: OnBufferFilledListener? = null,
    adjustGravityNorm: Boolean = true
) {
    /**
     * Processes accelerometer + magnetometer measurements.
     */
    private val accelerometerProcessor = AccelerometerGeomagneticAttitudeProcessor()

    /**
     * Processes gravity + magnetometer measurements.
     */
    private val gravityProcessor = GeomagneticAttitudeProcessor()

    /**
     * Instance being reused to externally notify attitudes so that it does not have additional
     * effects if it is modified.
     */
    private val fusedAttitude = Quaternion()

    /**
     * Array to be reused containing euler angles of leveling attitude.
     */
    private val eulerAngles = DoubleArray(Quaternion.N_ANGLES)

    /**
     * Instance to be reused containing coordinate transformation in NED coordinates.
     */
    private val coordinateTransformation =
        CoordinateTransformation(FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME)

    /**
     * Internal syncer to collect and sync gravity and magnetometer measurements.
     */
    private val gravityAndMagnetometerSyncer = GravityAndMagnetometerSensorMeasurementSyncer(
        context,
        magnetometerSensorType,
        sensorDelay,
        sensorDelay,
        gravityStartOffsetEnabled = startOffsetEnabled,
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
            if (gravityProcessor.process(measurement)) {
                gravityProcessor.fusedAttitude.copyTo(fusedAttitude)
                postProcessAttitudeAndNotify(measurement.timestamp)
            }
        }
    )

    /**
     * Internal syncer to collect and sync accelerometer and magnetometer measurements.
     */
    private val accelerometerAndMagnetometerSyncer =
        AccelerometerAndMagnetometerSensorMeasurementSyncer(
            context,
            accelerometerSensorType,
            magnetometerSensorType,
            sensorDelay,
            sensorDelay,
            accelerometerStartOffsetEnabled = startOffsetEnabled,
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
                if (accelerometerProcessor.process(measurement)) {
                    accelerometerProcessor.fusedAttitude.copyTo(fusedAttitude)
                    postProcessAttitudeAndNotify(measurement.timestamp)
                }
            }
        )

    /**
     * Gets or sets device location
     *
     * @throws IllegalStateException if estimator is running and a null value is set.
     */
    var location: Location? = location
        @Throws(IllegalStateException::class)
        set(value) {
            check(value != null || !running)

            accelerometerProcessor.location = value
            gravityProcessor.location = value
            field = value
        }

    /**
     * Indicates whether gravity norm must be adjusted to either Earth standard norm, or norm at
     * provided location. If no location is provided, this should only be enabled when device is
     * close to sea level.
     */
    var adjustGravityNorm: Boolean = adjustGravityNorm
        set(value) {
            field = value
            gravityProcessor.adjustGravityNorm = value
            accelerometerProcessor.adjustGravityNorm = value
        }

    /**
     * Indicates whether accurate leveling must be used or not.
     *
     * @throws IllegalStateException if estimator is running.
     */
    var useAccurateLevelingEstimator: Boolean = useAccurateLevelingEstimator
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)
            if (value) {
                checkNotNull(location)
            }

            accelerometerProcessor.useAccurateLevelingProcessor = value
            gravityProcessor.useAccurateLevelingProcessor = value
            field = value
        }

    /**
     * Earth's magnetic model. If null, the default model is used if [useWorldMagneticModel] is
     * true. If [useWorldMagneticModel] is false, this is ignored.
     *
     * @throws IllegalStateException if estimator is running.
     */
    var worldMagneticModel: WorldMagneticModel? = worldMagneticModel
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)

            accelerometerProcessor.worldMagneticModel = value
            gravityProcessor.worldMagneticModel = value
            field = value
        }

    /**
     * Gets or sets timestamp when World Magnetic Model will be evaluated to obtain current
     * magnetic declination. Only taken into account if [useWorldMagneticModel] is true.
     */
    var timestamp: Date? = timestamp
        set(value) {
            accelerometerProcessor.currentDate = value
            gravityProcessor.currentDate = value
            field = value
        }

    /**
     * Indicates whether world magnetic model is taken into account to adjust attitude yaw angle by
     * current magnetic declination based on current World Magnetic Model, location and timestamp.
     *
     * @throws IllegalStateException if estimator is running.
     */
    var useWorldMagneticModel: Boolean = useWorldMagneticModel
        @Throws(IllegalStateException::class)
        set(value) {
            check(!running)

            accelerometerProcessor.useWorldMagneticModel = value
            gravityProcessor.useWorldMagneticModel = value
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

        running = if (useAccelerometer) {
            accelerometerProcessor.reset()
            accelerometerAndMagnetometerSyncer.start(startTimestamp)
        } else {
            gravityProcessor.reset()
            gravityAndMagnetometerSyncer.start(startTimestamp)
        }

        return running
    }

    /**
     * Stops this estimator.
     */
    fun stop() {
        accelerometerAndMagnetometerSyncer.stop()
        gravityAndMagnetometerSyncer.stop()
        running = false
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
     * Processes current attitude and computes (if needed) a coordinate transformation or display
     * Euler angles.
     *
     * @param timestamp time in nanoseconds at which the measurement was made. Each measurement
     * wil be monotonically increasing using the same time base as
     * [android.os.SystemClock.elapsedRealtimeNanos].
     */
    private fun postProcessAttitudeAndNotify(timestamp: Long) {
        val c: CoordinateTransformation? =
            if (estimateCoordinateTransformation) {
                coordinateTransformation.fromRotation(fusedAttitude)
                coordinateTransformation
            } else {
                null
            }

        val displayRoll: Double?
        val displayPitch: Double?
        val displayYaw: Double?
        if (estimateEulerAngles) {
            fusedAttitude.toEulerAngles(eulerAngles)
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
            this,
            fusedAttitude,
            timestamp,
            displayRoll,
            displayPitch,
            displayYaw,
            c
        )
    }

    init {
        this.location = location
        this.worldMagneticModel = worldMagneticModel
        this.timestamp = timestamp
        this.useWorldMagneticModel = useWorldMagneticModel
        this.useAccurateLevelingEstimator = useAccurateLevelingEstimator

        accelerometerProcessor.adjustGravityNorm = adjustGravityNorm
        gravityProcessor.adjustGravityNorm = adjustGravityNorm
    }

    /**
     * Interface to notify when a new attitude measurement is available.
     */
    fun interface OnAttitudeAvailableListener {
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
            estimator: GeomagneticAttitudeEstimator2,
            attitude: Quaternion,
            timestamp: Long,
            roll: Double?,
            pitch: Double?,
            yaw: Double?,
            coordinateTransformation: CoordinateTransformation?
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
         * @param estimator leveled absolute attitude estimator that raised this event.
         * @param sensorType sensor that has changed its accuracy.
         * @param accuracy new accuracy.
         */
        fun onAccuracyChanged(
            estimator: GeomagneticAttitudeEstimator2,
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
         * @param estimator leveled absolute attitude estimator that raised this event.
         * @param sensorType sensor that got its buffer filled.
         */
        fun onBufferFilled(
            estimator: GeomagneticAttitudeEstimator2,
            sensorType: SensorType
        )
    }
}