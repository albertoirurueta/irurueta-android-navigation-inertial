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
package com.irurueta.android.navigation.inertial.collectors

import android.content.Context
import android.hardware.Sensor
import android.os.Build
import android.os.SystemClock

/**
 * Syncs measurements collected from multiple sensors.
 * Collected measurements are buffered to ensure they can be synced, if any measurement arrives out
 * of order, it is notified by this class.
 *
 * @property context Android context.
 * @property stopWhenFilledBuffer true to stop syncer when any buffer completely fills, false to
 * continue processing measurements at the expense of loosing old data. This will be notified using
 * [bufferFilledListener].
 * @property outOfOrderDetectionEnabled true to enable out of order detection of measurements, false
 * otherwise.
 * @property stopWhenOutOfOrder true to stop syncer if any out of order measurement is detected,
 * folse to continue processing measurements at the expense of receiving unordered data. This will
 * be notified using [outOfOrderMeasurementListener].
 * @property accuracyChangedListener listener to notify changes in accuracy.
 * @property bufferFilledListener listener to notify that some buffer has been filled. This usually
 * happens when consumer of measurements cannot keep up with the rate at which measurements are
 * generated.
 * @property outOfOrderMeasurementListener listener to notify that an out of order measurement has
 * been received. This usually happens when capacity of buffers are too small.
 * @property syncedMeasurementListener listener to notify the generation of a new synced
 * measurement.
 */
abstract class SensorMeasurementSyncer<M : SyncedSensorMeasurement, S : SensorMeasurementSyncer<M, S>>(
    val context: Context,
    val stopWhenFilledBuffer: Boolean,
    val outOfOrderDetectionEnabled: Boolean,
    val stopWhenOutOfOrder: Boolean,
    var accuracyChangedListener: OnAccuracyChangedListener<M, S>?,
    var bufferFilledListener: OnBufferFilledListener<M, S>?,
    var outOfOrderMeasurementListener: OnOutOfOrderMeasurementListener<M, S>?,
    var syncedMeasurementListener: OnSyncedMeasurementsListener<M, S>?
) {
    /**
     * Synced measurement to be reused for efficiency purposes.
     */
    protected abstract val syncedMeasurement: M

    /**
     * Timestamp when collector started expressed as a monotonically increasing timestamp in
     * nanoseconds as indicated by [SystemClock.elapsedRealtimeNanos].
     */
    var startTimestamp = 0L
        protected set

    /**
     * Indicates whether syncer is running and processing measurements.
     */
    var running = false
        protected set

    /**
     * Gets number of processed measurements since this syncer started.
     */
    var numberOfProcessedMeasurements: Int = 0
        protected set

    /**
     * Gets most recent timestamp of all measurements processed so far.
     * Timestamp is expressed in nanoseconds by a monotonic clock based on
     * [SystemClock.elapsedRealtimeNanos].
     */
    var mostRecentTimestamp: Long? = null
        protected set

    /**
     * Gets oldest timestamp in the buffer. If a new measurement arrives having
     * a timestamp older than this value, [OnOutOfOrderMeasurementListener] will be notified.
     */
    var oldestTimestamp: Long? = null
        protected set

    /**
     * Starts collecting and syncing sensor measurements.
     *
     * @param startTimestamp monotonically increasing timestamp when syncing starts. If not
     * provided, system clock is used by default, otherwise, the value can be provided to sync
     * multiple sensor collector instances.
     * @return true if all sensors are available and were successfully enabled.
     * @throws IllegalStateException if syncer is already running.
     */
    @Throws(IllegalStateException::class)
    abstract fun start(startTimestamp: Long = SystemClock.elapsedRealtimeNanos()): Boolean

    /**
     * Stops processing and syncing sensor measurements.
     */
    abstract fun stop()

    /**
     * Interface to notify when sensor accuracy changes.
     *
     * @param M type of synced measurement.
     * @param S type of syncer.
     */
    fun interface OnAccuracyChangedListener<M : SyncedSensorMeasurement, S : SensorMeasurementSyncer<M, S>> {
        /**
         * Called when accuracy changes.
         *
         * @param syncer syncer that raised this event.
         * @param sensorType sensor type whose accuracy has changed.
         * @param accuracy new accuracy.
         */
        fun onAccuracyChanged(syncer: S, sensorType: SensorType, accuracy: SensorAccuracy?)
    }

    /**
     * Interface to notify when a buffer gets completely filled.
     * When a buffer completely fills, internal collectors will stop if [stopWhenFilledBuffer] is
     * true.
     * If [stopWhenFilledBuffer] is false, internal collectors will continue collection at the
     * expense of loosing old data. Consumers of this listener should decide what to do at this
     * point.
     *
     * @param M type of synced measurement.
     * @param S type of syncer.
     */
    fun interface OnBufferFilledListener<M : SyncedSensorMeasurement, S : SensorMeasurementSyncer<M, S>> {
        /**
         * Called when any buffer gets completely filled.
         *
         * @param syncer syncer that raised this event.
         * @param sensorType sensor type whose buffer has filled.
         */
        fun onBufferFilled(syncer: S, sensorType: SensorType)
    }

    /**
     * Interface to notify when an out of order measurement is detected.
     * This usually happens if capacity of internal buffers is too small.
     * If [stopWhenOutOfOrder] is true, syncer stops if any out of order measurement is detected.
     * If [stopWhenOutOfOrder] is false, processing of measurements continues at the expense of
     * possibly receiving some unordered data. Consumers of this listener should decide what to do
     * at this point.
     *
     * @param M type of synced measurement.
     * @param S type of syncer.
     */
    fun interface OnOutOfOrderMeasurementListener<M : SyncedSensorMeasurement, S : SensorMeasurementSyncer<M, S>> {

        /**
         * Called when an out of order measurement is detected.
         *
         * @param syncer syncer that raised this event.
         * @param sensorType sensor type of detected out of order measurement.
         * @param measurement detected out of order measurement.
         */
        fun onOutOfOrderMeasurement(
            syncer: S,
            sensorType: SensorType,
            measurement: SensorMeasurement<*>
        )
    }

    /**
     * Interface to notify when a nw synced measurement is available.
     * Measurements notified by this listener are guaranteed to be ordered and synced.
     * Notice that notified measurement i reused for memory efficiency.
     *
     * @param M type of synced measurement.
     * @param S type of syncer.
     */
    fun interface OnSyncedMeasurementsListener<M : SyncedSensorMeasurement, S : SensorMeasurementSyncer<M, S>> {
        /**
         * Called when a new synced sensor measurement is available.
         *
         * @param syncer syncer that raised this event.
         * @param measurement a synced measurement containing measurements for each sensor being synced.
         */
        fun onSyncedMeasurements(syncer: S, measurement: M)
    }

    /**
     * Indicates the sensor types supported by [SensorMeasurementSyncer].
     *
     * @property value numerical value representing sensor type.
     */
    enum class SensorType(val value: Int) {
        /**
         * Accelerometer sensor.
         * Returns acceleration including gravity.
         */
        ACCELEROMETER(Sensor.TYPE_ACCELEROMETER),

        /**
         * Uncalibrated accelerometer sensor.
         * Returns acceleration including gravity but without bias correction.
         * This accelerometer is only available for SDK 26 or later.
         */
        ACCELEROMETER_UNCALIBRATED(Constants.TYPE_ACCELEROMETER_UNCALIBRATED),

        /**
         * Gyroscope.
         * Returns angular speed measurements.
         */
        GYROSCOPE(Sensor.TYPE_GYROSCOPE),

        /**
         * Uncalibrated gyroscope.
         * Returns angular speed measurements without bias correction.
         */
        GYROSCOPE_UNCALIBRATED(Sensor.TYPE_GYROSCOPE_UNCALIBRATED),

        /**
         * Magnetometer.
         * Returns magnetic field measurements.
         */
        MAGNETOMETER(Sensor.TYPE_MAGNETIC_FIELD),

        /**
         * Uncalibrated magnetometer.
         * Returns magnetic field measurements without hard-iron bias correction.
         */
        MAGNETOMETER_UNCALIBRATED(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED),

        /**
         * Gravity.
         */
        GRAVITY(Sensor.TYPE_GRAVITY);

        companion object {
            /**
             * Gets sensor type based on provided numerical value.
             *
             * @param value code used for sensor types.
             * @return code expressed as an enum or null if code has no match.
             */
            fun from(value: Int): SensorType? {
                if (Build.VERSION.SDK_INT < Build.VERSION_CODES.O
                    && value == Constants.TYPE_ACCELEROMETER_UNCALIBRATED
                ) {
                    return null
                }
                return values().find { it.value == value }
            }

            /**
             * Gets sensor type based on provided [AccelerometerSensorType].
             *
             * @param accelerometerSensorType accelerometer sensor type.
             * @return conversion to [SensorType] or null if no match is found.
             */
            fun from(accelerometerSensorType: AccelerometerSensorType): SensorType {
                return when (accelerometerSensorType) {
                    AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED -> ACCELEROMETER_UNCALIBRATED
                    else -> ACCELEROMETER
                }
            }

            /**
             * Gets sensor type based on provided [GyroscopeSensorType].
             *
             * @param gyroscopeSensorType gyroscope sensor type.
             * @return conversion to [SensorType] or null if no match is found.
             */
            fun from(gyroscopeSensorType: GyroscopeSensorType): SensorType {
                return when (gyroscopeSensorType) {
                    GyroscopeSensorType.GYROSCOPE_UNCALIBRATED -> GYROSCOPE_UNCALIBRATED
                    else -> GYROSCOPE
                }
            }

            /**
             * Gets sensor type based on provided [MagnetometerSensorType].
             *
             * @param magnetometerSensorType magnetometer sensor type.
             * @return conversion to [SensorType] or null if no match is found.
             */
            fun from(magnetometerSensorType: MagnetometerSensorType): SensorType {
                return when (magnetometerSensorType) {
                    MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED -> MAGNETOMETER_UNCALIBRATED
                    else -> MAGNETOMETER
                }
            }
        }
    }
}