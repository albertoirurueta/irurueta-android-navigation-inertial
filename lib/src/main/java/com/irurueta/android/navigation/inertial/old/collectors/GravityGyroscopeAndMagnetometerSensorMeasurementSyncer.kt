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
package com.irurueta.android.navigation.inertial.old.collectors

import android.content.Context
import android.hardware.Sensor
import android.os.SystemClock
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.collectors.measurements.GravitySensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorType
import com.irurueta.android.navigation.inertial.old.collectors.interpolators.GravityQuadraticSensorMeasurementInterpolator
import com.irurueta.android.navigation.inertial.old.collectors.interpolators.GravitySensorMeasurementInterpolator
import com.irurueta.android.navigation.inertial.old.collectors.interpolators.GyroscopeQuadraticSensorMeasurementInterpolator
import com.irurueta.android.navigation.inertial.old.collectors.interpolators.GyroscopeSensorMeasurementInterpolator
import com.irurueta.android.navigation.inertial.old.collectors.interpolators.MagnetometerQuadraticSensorMeasurementInterpolator
import com.irurueta.android.navigation.inertial.old.collectors.interpolators.MagnetometerSensorMeasurementInterpolator
import java.util.*

/**
 * Syncs gravity, gyroscope and magnetometer sensor measurements in case they arrive with certain
 * delay.
 *
 * Typically when synchronization is needed is for correct pose estimation, for attitude estimation,
 * not synced sensor measurements can also be used to achieve a reasonable estimation.
 *
 * @property context Android context.
 * @property gyroscopeSensorType One of the supported gyroscope sensor types.
 * @property magnetometerSensorType One of the supported magnetometer sensor types.
 * @property gravitySensorDelay Delay of gravity sensor between samples.
 * @property gyroscopeSensorDelay Delay of gyroscope sensor between samples.
 * @property magnetometerSensorDelay Delay of magnetometer sensor between samples.
 * @property gravityCapacity capacity of gravity buffer.
 * @property gyroscopeCapacity capacity of gyroscope buffer.
 * @property magnetometerCapacity capacity of magnetometer buffer.
 * @property gravityStartOffsetEnabled indicates whether gravity start offset will be
 * computed when first measurement is received. True indicates that offset is computed, false
 * assumes that offset is null.
 * @property gyroscopeStartOffsetEnabled indicates whether gyroscope start offset will be computed
 * when first measurement is received. True indicates that offset is computed, false assumes that
 * offset is null.
 * @property magnetometerStartOffsetEnabled indicates whether gyroscope start offset will be
 * computed when first measurement is received. True indicates that offset is computed, false
 * assumes that offset is null.
 * @property stopWhenFilledBuffer true to stop syncer when any buffer completely fills, false to
 * continue processing measurements at the expense of loosing old data. This will be notified using
 * [bufferFilledListener].
 * @property staleOffsetNanos offset respect most recent received timestamp of a measurement to
 * consider the measurement as stale so that it is skipped from synced measurement processing and
 * returned back from buffer to cache of measurements.
 * @property staleDetectionEnabled true to enable stale measurement detection, false otherwise.
 * @property skipWhenProcessing true to skip new measurements when processing a measurement.
 * @property accuracyChangedListener listener to notify changes in accuracy.
 * @property bufferFilledListener listener to notify that some buffer has been filled. This usually
 * happens when consumer of measurements cannot keep up with the rate at which measurements are
 * generated.
 * @property syncedMeasurementListener listener to notify the generation of a new synced
 * measurement.
 * @property staleDetectedMeasurementsListener listener to notify when stale measurements are found.
 * This might indicate that buffers are too small and data is not being properly synced.
 * @property gravityInterpolator interpolator for gravity measurements.
 * @property gyroscopeInterpolator interpolator for gyroscope measurements.
 * @property magnetometerInterpolator interpolator for magnetometer measurements.
 */
class GravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
    context: Context,
    val gyroscopeSensorType: GyroscopeSensorType = GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
    val magnetometerSensorType: MagnetometerSensorType = MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
    val gravitySensorDelay: SensorDelay = SensorDelay.FASTEST,
    val gyroscopeSensorDelay: SensorDelay = SensorDelay.FASTEST,
    val magnetometerSensorDelay: SensorDelay = SensorDelay.FASTEST,
    val gravityCapacity: Int = DEFAULT_GRAVITY_CAPACITY,
    val gyroscopeCapacity: Int = DEFAULT_GYROSCOPE_CAPACITY,
    val magnetometerCapacity: Int = DEFAULT_MAGNETOMETER_CAPACITY,
    val gravityStartOffsetEnabled: Boolean = false,
    val gyroscopeStartOffsetEnabled: Boolean = false,
    val magnetometerStartOffsetEnabled: Boolean = false,
    stopWhenFilledBuffer: Boolean = true,
    staleOffsetNanos: Long = DEFAULT_STALE_OFFSET_NANOS,
    staleDetectionEnabled: Boolean = true,
    skipWhenProcessing: Boolean = true,
    accuracyChangedListener: OnAccuracyChangedListener<GravityGyroscopeAndMagnetometerSyncedSensorMeasurement, GravityGyroscopeAndMagnetometerSensorMeasurementSyncer>? = null,
    bufferFilledListener: OnBufferFilledListener<GravityGyroscopeAndMagnetometerSyncedSensorMeasurement, GravityGyroscopeAndMagnetometerSensorMeasurementSyncer>? = null,
    syncedMeasurementListener: OnSyncedMeasurementsListener<GravityGyroscopeAndMagnetometerSyncedSensorMeasurement, GravityGyroscopeAndMagnetometerSensorMeasurementSyncer>? = null,
    staleDetectedMeasurementsListener: OnStaleDetectedMeasurementsListener<GravityGyroscopeAndMagnetometerSyncedSensorMeasurement, GravityGyroscopeAndMagnetometerSensorMeasurementSyncer>? = null,
    val gravityInterpolator: GravitySensorMeasurementInterpolator = GravityQuadraticSensorMeasurementInterpolator(),
    val gyroscopeInterpolator: GyroscopeSensorMeasurementInterpolator = GyroscopeQuadraticSensorMeasurementInterpolator(),
    val magnetometerInterpolator: MagnetometerSensorMeasurementInterpolator = MagnetometerQuadraticSensorMeasurementInterpolator()
) : SensorMeasurementSyncer<GravityGyroscopeAndMagnetometerSyncedSensorMeasurement, GravityGyroscopeAndMagnetometerSensorMeasurementSyncer>(
    context,
    stopWhenFilledBuffer,
    staleOffsetNanos,
    staleDetectionEnabled,
    skipWhenProcessing,
    accuracyChangedListener,
    bufferFilledListener,
    syncedMeasurementListener,
    staleDetectedMeasurementsListener
) {
    /**
     * Gravity measurements to be processed in next batch.
     */
    private val gravityMeasurements = ArrayDeque<GravitySensorMeasurement>(gravityCapacity)

    /**
     * Gyroscope measurements to be processed in next batch.
     */
    private val gyroscopeMeasurements = ArrayDeque<GyroscopeSensorMeasurement>(gyroscopeCapacity)

    /**
     * Magnetometer measurements to be processed in next batch.
     */
    private val magnetometerMeasurements =
        ArrayDeque<MagnetometerSensorMeasurement>(magnetometerCapacity)

    /**
     * List of gravity measurements that have already been processed and can be returned back
     * to the cache of available measurements.
     */
    private val alreadyProcessedGravityMeasurements =
        ArrayDeque<GravitySensorMeasurement>(gravityCapacity)

    /**
     * List of gyroscope measurements that have already been processed and can be returned back to
     * the cache af available measurements.
     */
    private val alreadyProcessedGyroscopeMeasurements =
        ArrayDeque<GyroscopeSensorMeasurement>(gyroscopeCapacity)

    /**
     * List of magnetometer measurements that have already been processed and can be returned back
     * to the cache of available measurements.
     */
    private val alreadyProcessedMagnetometerMeasurements =
        ArrayDeque<MagnetometerSensorMeasurement>(magnetometerCapacity)

    /**
     * List of found gyroscope measurements.
     */
    private val foundGyroscopeMeasurements =
        ArrayDeque<GyroscopeSensorMeasurement>(gyroscopeCapacity)

    /**
     * List of found magnetometer measurements.
     */
    private val foundMagnetometerMeasurements =
        ArrayDeque<MagnetometerSensorMeasurement>(magnetometerCapacity)

    /**
     * Previous gravity measurement. This instance is reused for efficiency reasons.
     */
    private val previousGravityMeasurement = GravitySensorMeasurement()

    /**
     * Previous gyroscope measurement. This instance is reused for efficiency reasons.
     */
    private val previousGyroscopeMeasurement = GyroscopeSensorMeasurement()

    /**
     * Previous magnetometer measurement. This instance is reused for efficiency reasons.
     */
    private val previousMagnetometerMeasurement = MagnetometerSensorMeasurement()

    /**
     * Interpolated gravity measurement. This instance is reused for efficiency reasons.
     */
    private val interpolatedGravityMeasurement = GravitySensorMeasurement()

    /**
     * Interpolated gyroscope measurement. This instance is reused for efficiency reasons.
     */
    private val interpolatedGyroscopeMeasurement = GyroscopeSensorMeasurement()

    /**
     * Interpolated magnetometer measurement. This instance is reused for efficiency reasons.
     */
    private val interpolatedMagnetometerMeasurement = MagnetometerSensorMeasurement()

    /**
     * Flag indicating whether a previous gravity measurement has been processed.
     */
    private var hasPreviousGravityMeasurement = false

    /**
     * Flag indicating whether a previous gyroscope measurement has been processed.
     */
    private var hasPreviousGyroscopeMeasurement = false

    /**
     * Flag indicating whether a previous magnetometer measurement has been processed.
     */
    private var hasPreviousMagnetometerMeasurement = false

    /**
     * Timestamp of last notified synced measurement. This is used to ensure that synced
     * measurements have monotonically increasing timestamp.
     */
    private var lastNotifiedTimestamp = 0L

    /**
     * Timestamp of last gravity measurement that was processed and notified.
     */
    private var lastNotifiedGravityTimestamp = 0L

    /**
     * Timestamp of last gyroscope measurement that was processed and notified.
     */
    private var lastNotifiedGyroscopeTimestamp = 0L

    /**
     * Timestamp of last magnetometer measurement that was processed and notified.
     */
    private var lastNotifiedMagnetometerTimestamp = 0L

    /**
     * Internal buffered gravity sensor collector.
     * Collects and buffers gravity data.
     */
    private val gravitySensorCollector = BufferedGravitySensorCollector(
        context,
        gravitySensorDelay,
        gravityCapacity,
        gravityStartOffsetEnabled,
        stopWhenFilledBuffer,
        accuracyChangedListener = { _, accuracy ->
            accuracyChangedListener?.onAccuracyChanged(
                this@GravityGyroscopeAndMagnetometerSensorMeasurementSyncer,
                SensorType.GRAVITY,
                accuracy
            )
        },
        bufferFilledListener = {
            bufferFilledListener?.onBufferFilled(
                this@GravityGyroscopeAndMagnetometerSensorMeasurementSyncer,
                SensorType.GRAVITY
            )
            if (stopWhenFilledBuffer) {
                stop()
            }
        },
        measurementListener = { collector, _, bufferPosition ->
            if (this.skipWhenProcessing && processing) {
                return@BufferedGravitySensorCollector
            }

            synchronized(this@GravityGyroscopeAndMagnetometerSensorMeasurementSyncer) {
                processing = true

                val measurementsBeforePosition =
                    collector.getMeasurementsBeforePosition(bufferPosition)
                val lastTimestamp = measurementsBeforePosition.lastOrNull()?.timestamp
                if (lastTimestamp != null) {
                    mostRecentTimestamp = lastTimestamp

                    // copy measurements
                    copyToGravityMeasurements(measurementsBeforePosition)
                    processMeasurements()
                }

                processing = false
            }
        }
    )

    /**
     * Internal buffered gyroscope sensor collector.
     * Collects and buffers gyroscope data.
     */
    private val gyroscopeSensorCollector = BufferedGyroscopeSensorCollector(
        context,
        gyroscopeSensorType,
        gyroscopeSensorDelay,
        gyroscopeCapacity,
        gyroscopeStartOffsetEnabled,
        stopWhenFilledBuffer,
        accuracyChangedListener = { _, accuracy ->
            accuracyChangedListener?.onAccuracyChanged(
                this@GravityGyroscopeAndMagnetometerSensorMeasurementSyncer,
                SensorType.from(gyroscopeSensorType),
                accuracy
            )
        },
        bufferFilledListener = {
            bufferFilledListener?.onBufferFilled(
                this@GravityGyroscopeAndMagnetometerSensorMeasurementSyncer,
                SensorType.from(gyroscopeSensorType)
            )
            if (stopWhenFilledBuffer) {
                stop()
            }
        },
        measurementListener = { collector, _, _ ->
            if (this.skipWhenProcessing && processing) {
                return@BufferedGyroscopeSensorCollector
            }

            synchronized(this@GravityGyroscopeAndMagnetometerSensorMeasurementSyncer) {
                processing = true

                val mostRecentTimestamp =
                    this@GravityGyroscopeAndMagnetometerSensorMeasurementSyncer.mostRecentTimestamp
                if (mostRecentTimestamp != null) {
                    val measurementsBeforeTimestamp =
                        collector.getMeasurementsBeforeTimestamp(mostRecentTimestamp)
                    if (measurementsBeforeTimestamp.isNotEmpty()) {
                        // copy measurements
                        copyToGyroscopeMeasurements(measurementsBeforeTimestamp)
                    }
                }

                processing = false
            }
        }
    )

    /**
     * Internal buffered magnetometer sensor collector.
     * Collects and buffers magnetometer data.
     */
    private val magnetometerSensorCollector = BufferedMagnetometerSensorCollector(
        context,
        magnetometerSensorType,
        magnetometerSensorDelay,
        magnetometerCapacity,
        magnetometerStartOffsetEnabled,
        stopWhenFilledBuffer,
        accuracyChangedListener = { _, accuracy ->
            accuracyChangedListener?.onAccuracyChanged(
                this@GravityGyroscopeAndMagnetometerSensorMeasurementSyncer,
                SensorType.from(magnetometerSensorType),
                accuracy
            )
        },
        bufferFilledListener = {
            bufferFilledListener?.onBufferFilled(
                this@GravityGyroscopeAndMagnetometerSensorMeasurementSyncer,
                SensorType.from(magnetometerSensorType)
            )
            if (stopWhenFilledBuffer) {
                stop()
            }
        },
        measurementListener = { collector, _, _ ->
            if (this.skipWhenProcessing && processing) {
                return@BufferedMagnetometerSensorCollector
            }

            synchronized(this@GravityGyroscopeAndMagnetometerSensorMeasurementSyncer) {
                processing = true

                val mostRecentTimestamp =
                    this@GravityGyroscopeAndMagnetometerSensorMeasurementSyncer.mostRecentTimestamp
                if (mostRecentTimestamp != null) {
                    val measurementsBeforeTimestamp =
                        collector.getMeasurementsBeforeTimestamp(mostRecentTimestamp)
                    if (measurementsBeforeTimestamp.isNotEmpty()) {
                        // copy measurements
                        copyToMagnetometerMeasurements(measurementsBeforeTimestamp)
                    }
                }

                processing = false
            }
        }
    )

    /**
     * Synced measurement to be reused for efficiency purposes.
     */
    override val syncedMeasurement = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
        GravitySensorMeasurement(),
        GyroscopeSensorMeasurement(),
        MagnetometerSensorMeasurement(),
        0L
    )

    /**
     * Gets gravity sensor being used to obtain measurements or null if not available.
     * This can be used to obtain additional information about the sensor.
     * @see gravitySensorAvailable
     */
    val gravitySensor: Sensor?
        get() = gravitySensorCollector.sensor

    /**
     * Gets gyroscope sensor being used to obtain measurements or null if not available.
     * This can be used to obtain additional information about the sensor.
     * @see gyroscopeSensorAvailable
     */
    val gyroscopeSensor: Sensor?
        get() = gyroscopeSensorCollector.sensor

    /**
     * Gets magnetometer sensor being used to obtain measurements or null if not available.
     * This can be used to obtain additional information about the sensor.
     * @see magnetometerSensorAvailable
     */
    val magnetometerSensor: Sensor?
        get() = magnetometerSensorCollector.sensor

    /**
     * Indicates whether requested gravity sensor is available or not.
     */
    val gravitySensorAvailable: Boolean
        get() = gravitySensorCollector.sensorAvailable

    /**
     * Indicates whether requested gyroscope sensor is available or not.
     */
    val gyroscopeSensorAvailable: Boolean
        get() = gyroscopeSensorCollector.sensorAvailable

    /**
     * Indicates whether requested magnetometer sensor is available or not.
     */
    val magnetometerSensorAvailable: Boolean
        get() = magnetometerSensorCollector.sensorAvailable

    /**
     * Gets initial gravity offset expressed in nano seconds between first received
     * measurement timestamp and start time expressed in the monotonically increasing system clock
     * obtained by [SystemClock.elapsedRealtimeNanos].
     */
    val gravityStartOffset: Long?
        get() = gravitySensorCollector.startOffset

    /**
     * Gets initial gyroscope offset expressed in nano seconds between first received
     * measurement timestamp and start time expressed in the monotonically increasing system clock
     * obtained by [SystemClock.elapsedRealtimeNanos].
     */
    val gyroscopeStartOffset: Long?
        get() = gyroscopeSensorCollector.startOffset

    /**
     * Gets initial magnetometer offset expressed in nano seconds between first received
     * measurement timestamp and start time expressed in the monotonically increasing system clock
     * obtained by [SystemClock.elapsedRealtimeNanos].
     */
    val magnetometerStartOffset: Long?
        get() = magnetometerSensorCollector.startOffset

    /**
     * Gets gravity collector current usage as a value between 0.0 and 1.0.
     * 0.0 indicates that buffer is empty and no measurement has yet been received.
     * 1.0 indicates that buffer is full.
     */
    val gravityCollectorUsage: Float
        get() = gravitySensorCollector.usage

    /**
     * Gets gyroscope collector current usage as a value between 0.0 and 1.0.
     * 0.0 indicates that buffer is empty and no measurement has yet been received.
     * 1.0 indicates that buffer is full.
     */
    val gyroscopeCollectorUsage: Float
        get() = gyroscopeSensorCollector.usage

    /**
     * Gets magnetometer collector current usage as a value between 0.0 and 1.0.
     * 0.0 indicates that buffer is empty and no measurement has yet been received.
     * 1.0 indicates that buffer is full.
     */
    val magnetometerCollectorUsage: Float
        get() = magnetometerSensorCollector.usage

    /**
     * Gets gravity buffer current usage as a value between 0.0 and 1.0.
     * 0.0 indicates that buffer is empty and no measurement has yet ben received.
     * 1.0 indicates that buffer is full.
     */
    val gravityUsage: Float
        get() = gravityMeasurements.size.toFloat() / gravityCapacity.toFloat()

    /**
     * Gets gyroscope buffer current usage as a value between 0.0 and 1.0.
     * 0.0 indicates that buffer is empty and no measurement has yet been received.
     * 1.0 indicates that buffer is full.
     */
    val gyroscopeUsage: Float
        get() = gyroscopeMeasurements.size.toFloat() / gyroscopeCapacity.toFloat()

    /**
     * Gets magnetometer buffer current usage as a value between 0.0 and 1.0.
     * 0.0 indicates that buffer is empty and no measurement has yet been received.
     * 1.0 indicates that buffer is full.
     */
    val magnetometerUsage: Float
        get() = magnetometerMeasurements.size.toFloat() / magnetometerCapacity.toFloat()

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
    override fun start(startTimestamp: Long): Boolean {
        check(!running)

        stopping = false
        clearCollectionsAndReset()
        this.startTimestamp = startTimestamp
        running = if (gravitySensorCollector.start(startTimestamp)
            && gyroscopeSensorCollector.start(startTimestamp)
            && magnetometerSensorCollector.start(startTimestamp)
        ) {
            true
        } else {
            stop()
            false
        }

        return running
    }

    /**
     * Stops processing and syncing sensor measurements.
     */
    @Synchronized
    override fun stop() {
        stopping = true
        processing = false
        gravitySensorCollector.stop()
        gyroscopeSensorCollector.stop()
        magnetometerSensorCollector.stop()

        reset()
    }

    /**
     * Copies provided measurements into an internal list to be processed when next batch is
     * available.
     * Measurements are moved from a collection of cached available measurements to a new list
     * to avoid inadvertently reusing measurements by internal collector.
     */
    private fun copyToGravityMeasurements(measurements: Collection<GravitySensorMeasurement>) {
        for (measurement in measurements) {
            gravityMeasurements.add(GravitySensorMeasurement(measurement))
        }
    }

    /**
     * Copies provided measurements into an internal list to be processed when next batch is
     * available.
     * Measurements are moved from a collection of cached available measurements to a new list
     * to avoid inadvertently reusing measurements by internal collector.
     */
    private fun copyToGyroscopeMeasurements(measurements: Collection<GyroscopeSensorMeasurement>) {
        for (measurement in measurements) {
            gyroscopeMeasurements.add(GyroscopeSensorMeasurement(measurement))
        }
    }

    /**
     * Copies provided measurements into an internal list to be processed when next batch is
     * available.
     * Measurements are moved from a collection of cached available measurements to a new list
     * to avoid inadvertently reusing measurements by internal collector.
     */
    private fun copyToMagnetometerMeasurements(measurements: Collection<MagnetometerSensorMeasurement>) {
        for (measurement in measurements) {
            magnetometerMeasurements.add(MagnetometerSensorMeasurement(measurement))
        }
    }

    /**
     * Processes all measurements collected in the last batch.
     */
    private fun processMeasurements() {
        val oldestTimestamp = gravityMeasurements.firstOrNull()?.timestamp
        this.oldestTimestamp = oldestTimestamp

        alreadyProcessedGravityMeasurements.clear()
        for (gravityMeasurement in gravityMeasurements) {
            val previousGravityTimestamp = if (hasPreviousGravityMeasurement) {
                previousGravityMeasurement.timestamp
            } else {
                0L
            }
            val gravityTimestamp = gravityMeasurement.timestamp

            findGyroscopeMeasurementsBetween(
                previousGravityTimestamp,
                gravityTimestamp
            )

            var processedAccelerometer = false
            if (foundGyroscopeMeasurements.isEmpty()) {
                if (hasPreviousGyroscopeMeasurement && hasPreviousMagnetometerMeasurement
                    && gravityTimestamp > lastNotifiedTimestamp
                    && gravityTimestamp >= lastNotifiedGravityTimestamp
                    && previousGyroscopeMeasurement.timestamp > lastNotifiedGyroscopeTimestamp
                    && previousMagnetometerMeasurement.timestamp > lastNotifiedMagnetometerTimestamp
                    && gyroscopeInterpolator.interpolate(
                        previousGyroscopeMeasurement,
                        gravityTimestamp,
                        interpolatedGyroscopeMeasurement
                    )
                    && magnetometerInterpolator.interpolate(
                        previousMagnetometerMeasurement,
                        gravityTimestamp,
                        interpolatedMagnetometerMeasurement
                    )
                ) {
                    // generate synchronized measurement when rate of gravity is greater
                    // than gyroscope one
                    syncedMeasurement.timestamp = gravityTimestamp
                    syncedMeasurement.gravityMeasurement?.copyFrom(
                        gravityMeasurement
                    )
                    gravityInterpolator.push(gravityMeasurement)
                    syncedMeasurement.gyroscopeMeasurement?.copyFrom(
                        interpolatedGyroscopeMeasurement
                    )
                    syncedMeasurement.magnetometerMeasurement?.copyFrom(
                        interpolatedMagnetometerMeasurement
                    )

                    numberOfProcessedMeasurements++

                    syncedMeasurementListener?.onSyncedMeasurements(
                        this@GravityGyroscopeAndMagnetometerSensorMeasurementSyncer,
                        syncedMeasurement
                    )
                    lastNotifiedTimestamp = gravityTimestamp
                    lastNotifiedGravityTimestamp = gravityTimestamp
                    lastNotifiedGyroscopeTimestamp = previousGyroscopeMeasurement.timestamp
                    lastNotifiedMagnetometerTimestamp = previousMagnetometerMeasurement.timestamp

                    processedAccelerometer = true
                }
            } else {
                for (gyroscopeMeasurement in foundGyroscopeMeasurements) {
                    val previousGyroscopeTimestamp = if (hasPreviousGyroscopeMeasurement) {
                        previousGyroscopeMeasurement.timestamp
                    } else {
                        0L
                    }
                    val gyroscopeTimestamp = gyroscopeMeasurement.timestamp

                    findMagnetometerMeasurementsBetween(
                        previousGyroscopeTimestamp,
                        gyroscopeTimestamp
                    )

                    var processedGyroscope = false
                    if (foundMagnetometerMeasurements.isEmpty()) {
                        if (hasPreviousMagnetometerMeasurement
                            && gyroscopeTimestamp > lastNotifiedTimestamp
                            && gravityTimestamp >= lastNotifiedGravityTimestamp
                            && gyroscopeTimestamp >= lastNotifiedGyroscopeTimestamp
                            && previousMagnetometerMeasurement.timestamp > lastNotifiedMagnetometerTimestamp
                            && gravityInterpolator.interpolate(
                                gravityMeasurement,
                                gyroscopeTimestamp,
                                interpolatedGravityMeasurement
                            )
                            && magnetometerInterpolator.interpolate(
                                previousMagnetometerMeasurement,
                                gyroscopeTimestamp,
                                interpolatedMagnetometerMeasurement
                            )
                        ) {
                            // generate synchronized measurement when rate of magnetometer is greater
                            // than gyroscope one
                            syncedMeasurement.timestamp = gyroscopeTimestamp
                            syncedMeasurement.gravityMeasurement?.copyFrom(
                                interpolatedGravityMeasurement
                            )
                            syncedMeasurement.gyroscopeMeasurement?.copyFrom(gyroscopeMeasurement)
                            gyroscopeInterpolator.push(gyroscopeMeasurement)
                            syncedMeasurement.magnetometerMeasurement?.copyFrom(
                                interpolatedMagnetometerMeasurement
                            )

                            numberOfProcessedMeasurements++

                            syncedMeasurementListener?.onSyncedMeasurements(
                                this@GravityGyroscopeAndMagnetometerSensorMeasurementSyncer,
                                syncedMeasurement
                            )
                            lastNotifiedTimestamp = gyroscopeTimestamp
                            lastNotifiedGravityTimestamp = gravityTimestamp
                            lastNotifiedGyroscopeTimestamp = gyroscopeTimestamp
                            lastNotifiedMagnetometerTimestamp =
                                previousMagnetometerMeasurement.timestamp

                            processedGyroscope = true
                        }
                    } else {
                        for (magnetometerMeasurement in foundMagnetometerMeasurements) {
                            val magnetometerTimestamp = magnetometerMeasurement.timestamp
                            if (magnetometerTimestamp > lastNotifiedTimestamp
                                && gravityTimestamp >= lastNotifiedGravityTimestamp
                                && magnetometerTimestamp >= lastNotifiedMagnetometerTimestamp
                                && gravityInterpolator.interpolate(
                                    gravityMeasurement,
                                    magnetometerTimestamp,
                                    interpolatedGravityMeasurement
                                )
                                && gyroscopeInterpolator.interpolate(
                                    gyroscopeMeasurement,
                                    magnetometerTimestamp,
                                    interpolatedGyroscopeMeasurement
                                )
                            ) {
                                // generate synchronized measurement when rate of magnetometer is
                                // greater than gyroscope and accelerometer one
                                syncedMeasurement.timestamp = magnetometerTimestamp
                                syncedMeasurement.gravityMeasurement?.copyFrom(
                                    interpolatedGravityMeasurement
                                )
                                syncedMeasurement.gyroscopeMeasurement?.copyFrom(
                                    interpolatedGyroscopeMeasurement
                                )
                                syncedMeasurement.magnetometerMeasurement?.copyFrom(
                                    magnetometerMeasurement
                                )
                                magnetometerInterpolator.push(magnetometerMeasurement)

                                numberOfProcessedMeasurements++

                                syncedMeasurementListener?.onSyncedMeasurements(
                                    this@GravityGyroscopeAndMagnetometerSensorMeasurementSyncer,
                                    syncedMeasurement
                                )
                                lastNotifiedTimestamp = gyroscopeTimestamp
                                lastNotifiedGravityTimestamp = gravityTimestamp
                                lastNotifiedGyroscopeTimestamp = gyroscopeTimestamp
                                lastNotifiedMagnetometerTimestamp = magnetometerTimestamp
                            }
                        }
                        processedGyroscope = true

                        previousMagnetometerMeasurement.copyFrom(foundMagnetometerMeasurements.last())
                        hasPreviousMagnetometerMeasurement = true
                    }

                    if (processedGyroscope) {
                        alreadyProcessedGyroscopeMeasurements.add(gyroscopeMeasurement)

                        // remove processed magnetometer measurements
                        magnetometerMeasurements.removeAll(foundMagnetometerMeasurements)
                    }
                }

                processedAccelerometer = true

                previousGyroscopeMeasurement.copyFrom(foundGyroscopeMeasurements.last())
                hasPreviousGyroscopeMeasurement = true
            }

            if (processedAccelerometer) {
                alreadyProcessedGravityMeasurements.add(gravityMeasurement)

                // remove processed gyroscope measurements
                gyroscopeMeasurements.removeAll(alreadyProcessedGyroscopeMeasurements)
            }

            previousGravityMeasurement.copyFrom(gravityMeasurement)
            hasPreviousGravityMeasurement = true
        }

        if (alreadyProcessedGravityMeasurements.isNotEmpty()) {
            // return processed gravity measurements
            gravityMeasurements.removeAll(alreadyProcessedGravityMeasurements)
        }

        foundGyroscopeMeasurements.clear()
        foundMagnetometerMeasurements.clear()

        cleanupStaleMeasurements()

        if (stopping) {
            // collections are reset at this point to prevent concurrent modifications
            clearCollectionsAndReset()
            stopping = false
            processing = false
        }
    }

    /**
     * Resets syncer status.
     */
    private fun reset() {
        numberOfProcessedMeasurements = 0
        mostRecentTimestamp = null
        oldestTimestamp = null
        running = false
        processing = false

        hasPreviousGravityMeasurement = false
        hasPreviousGyroscopeMeasurement = false
        hasPreviousMagnetometerMeasurement = false
        lastNotifiedTimestamp = 0L
        lastNotifiedGravityTimestamp = 0L
        lastNotifiedGyroscopeTimestamp = 0L
        lastNotifiedMagnetometerTimestamp = 0L

        gravityInterpolator.reset()
        gyroscopeInterpolator.reset()
        magnetometerInterpolator.reset()
    }

    /**
     * Clears internal collections and resets
     */
    private fun clearCollectionsAndReset() {
        gravityMeasurements.clear()
        gyroscopeMeasurements.clear()
        magnetometerMeasurements.clear()
        alreadyProcessedGravityMeasurements.clear()
        alreadyProcessedGyroscopeMeasurements.clear()
        alreadyProcessedMagnetometerMeasurements.clear()
        foundGyroscopeMeasurements.clear()
        foundMagnetometerMeasurements.clear()

        reset()
    }

    /**
     * Finds gyroscope measurements in the buffer within provided minimum and maximum timestamp.
     *
     * @param minTimestamp minimum timestamp.
     * @param maxTimestamp maximum timestamp.
     * @return found gyroscope measurements or empty.
     */
    private fun findGyroscopeMeasurementsBetween(
        minTimestamp: Long,
        maxTimestamp: Long
    ) {
        findMeasurementsBetween(
            minTimestamp,
            maxTimestamp,
            gyroscopeMeasurements,
            foundGyroscopeMeasurements
        )
    }

    /**
     * Finds magnetometer measurements in the buffer within provided minimum and maximum timestamp.
     *
     * @param minTimestamp minimum timestamp.
     * @param maxTimestamp maximum timestamp.
     * @return found magnetometer measurements or empty.
     */
    private fun findMagnetometerMeasurementsBetween(
        minTimestamp: Long,
        maxTimestamp: Long
    ) {
        findMeasurementsBetween(
            minTimestamp,
            maxTimestamp,
            magnetometerMeasurements,
            foundMagnetometerMeasurements
        )
    }

    /**
     * Removes stale measurements from buffer and returns them to cache if [staleDetectionEnabled]
     * is true.
     */
    private fun cleanupStaleMeasurements() {
        if (!staleDetectionEnabled) {
            return
        }
        val mostRecentTimestamp = this.mostRecentTimestamp ?: return

        val staleTimestamp = mostRecentTimestamp - staleOffsetNanos

        cleanupStaleMeasurements(
            staleTimestamp,
            alreadyProcessedGravityMeasurements,
            gravityMeasurements,
            SensorType.GRAVITY,
            this,
            staleDetectedMeasurementsListener
        )

        cleanupStaleMeasurements(
            staleTimestamp,
            alreadyProcessedGyroscopeMeasurements,
            gyroscopeMeasurements,
            SensorType.from(gyroscopeSensorType),
            this,
            staleDetectedMeasurementsListener
        )

        cleanupStaleMeasurements(
            staleTimestamp,
            alreadyProcessedMagnetometerMeasurements,
            magnetometerMeasurements,
            SensorType.from(magnetometerSensorType),
            this,
            staleDetectedMeasurementsListener
        )
    }

    /**
     * Initializes this syncer.
     */
    init {
        // check that capacities are larger than zero.
        require(gravityCapacity > 0)
        require(gyroscopeCapacity > 0)
        require(magnetometerCapacity > 0)
    }

    companion object {
        /**
         * Default capacity for gravity measurement cache.
         */
        const val DEFAULT_GRAVITY_CAPACITY = 100

        /**
         * Default capacity for gyroscope measurement cache.
         */
        const val DEFAULT_GYROSCOPE_CAPACITY = 200

        /**
         * Default capacity for magnetometer measurement cache.
         */
        const val DEFAULT_MAGNETOMETER_CAPACITY = 100

        /**
         * Default offset to consider a measurement as stale to be removed from cache.
         * By default the value is equal to 5 seconds expressed in nanoseconds.
         */
        const val DEFAULT_STALE_OFFSET_NANOS = 5000_000_000L
    }
}