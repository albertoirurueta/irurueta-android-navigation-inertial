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
import android.os.SystemClock
import com.irurueta.android.navigation.inertial.collectors.interpolators.*
import java.util.*

/**
 * Syncs accelerometer, gravity and gyroscope sensor measurements in case they arrive with certain
 * delay.
 *
 * Typically when synchronization is needed is for correct pose estimation, for attitude estimation,
 * not synced sensor measurements can also be used to achieve a reasonable estimation.
 *
 * @property context Android context.
 * @property accelerometerSensorType One of the supported accelerometer sensor types.
 * @property gyroscopeSensorType One of the supported gyroscope sensor types.
 * @property accelerometerSensorDelay Delay of accelerometer sensor between samples.
 * @property gravitySensorDelay Delay of gravity sensor between samples.
 * @property gyroscopeSensorDelay Delay of gyroscope sensor between samples.
 * @property accelerometerCapacity capacity of accelerometer buffer.
 * @property gravityCapacity capacity of gravity buffer.
 * @property gyroscopeCapacity capacity of gyroscope buffer.
 * @property accelerometerStartOffsetEnabled indicates whether accelerometer start offset will be
 * computed when first measurement is received. True indicates that offset is computed, false
 * assumes that offset is null.
 * @property gravityStartOffsetEnabled indicates whether gravity start offset will be computed when
 * first measurement is received. True indicates that offset is computed, false assumes that offset
 * is null.
 * @property gyroscopeStartOffsetEnabled indicates whether gyroscope start offset will be computed
 * when first measurement is received. True indicates that offset is computed, false assumes that
 * offset is null.
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
 * @property accelerometerInterpolator interpolator for accelerometer measurements.
 * @property gravityInterpolator interpolator for gravity measurements.
 * @property gyroscopeInterpolator interpolator for gyroscope measurements.
 */
class AccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
    context: Context,
    val accelerometerSensorType: AccelerometerSensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
    val gyroscopeSensorType: GyroscopeSensorType = GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
    val accelerometerSensorDelay: SensorDelay = SensorDelay.FASTEST,
    val gravitySensorDelay: SensorDelay = SensorDelay.FASTEST,
    val gyroscopeSensorDelay: SensorDelay = SensorDelay.FASTEST,
    val accelerometerCapacity: Int = DEFAULT_ACCELEROMETER_CAPACITY,
    val gravityCapacity: Int = DEFAULT_GRAVITY_CAPACITY,
    val gyroscopeCapacity: Int = DEFAULT_GYROSCOPE_CAPACITY,
    val accelerometerStartOffsetEnabled: Boolean = false,
    val gravityStartOffsetEnabled: Boolean = false,
    val gyroscopeStartOffsetEnabled: Boolean = false,
    stopWhenFilledBuffer: Boolean = true,
    staleOffsetNanos: Long = DEFAULT_STALE_OFFSET_NANOS,
    staleDetectionEnabled: Boolean = true,
    skipWhenProcessing: Boolean = true,
    accuracyChangedListener: OnAccuracyChangedListener<AccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AccelerometerGravityAndGyroscopeSensorMeasurementSyncer>? = null,
    bufferFilledListener: OnBufferFilledListener<AccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AccelerometerGravityAndGyroscopeSensorMeasurementSyncer>? = null,
    syncedMeasurementListener: OnSyncedMeasurementsListener<AccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AccelerometerGravityAndGyroscopeSensorMeasurementSyncer>? = null,
    staleDetectedMeasurementsListener: OnStaleDetectedMeasurementsListener<AccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AccelerometerGravityAndGyroscopeSensorMeasurementSyncer>? = null,
    val accelerometerInterpolator: AccelerometerSensorMeasurementInterpolator = AccelerometerQuadraticSensorMeasurementInterpolator(),
    val gravityInterpolator: GravitySensorMeasurementInterpolator = GravityQuadraticSensorMeasurementInterpolator(),
    val gyroscopeInterpolator: GyroscopeSensorMeasurementInterpolator = GyroscopeQuadraticSensorMeasurementInterpolator()
) : SensorMeasurementSyncer<AccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AccelerometerGravityAndGyroscopeSensorMeasurementSyncer>(
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
     * Accelerometer measurements to be processed in next batch.
     */
    private val accelerometerMeasurements =
        ArrayDeque<AccelerometerSensorMeasurement>(accelerometerCapacity)

    /**
     * Gravity measurements to be processed in next batch.
     */
    private val gravityMeasurements = ArrayDeque<GravitySensorMeasurement>(gravityCapacity)

    /**
     * Gyroscope measurements to be processed in next batch.
     */
    private val gyroscopeMeasurements = ArrayDeque<GyroscopeSensorMeasurement>(gyroscopeCapacity)

    /**
     * List of accelerometer measurements that have already been processed and can be returned back
     * to the cache of available measurements.
     */
    private val alreadyProcessedAccelerometerMeasurements =
        ArrayDeque<AccelerometerSensorMeasurement>(accelerometerCapacity)

    /**
     * List of gravity measurements that have already been processed and can be returned back to the
     * cache of available measurements.
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
     * List of found gravity measurements.
     */
    private val foundGravityMeasurements =
        ArrayDeque<GravitySensorMeasurement>(gravityCapacity)

    /**
     * List of found gyroscope measurements.
     */
    private val foundGyroscopeMeasurements =
        ArrayDeque<GyroscopeSensorMeasurement>(gyroscopeCapacity)

    /**
     * Previous accelerometer measurement. This instance is reused for efficiency reasons.
     */
    private val previousAccelerometerMeasurement = AccelerometerSensorMeasurement()

    /**
     * Previous gravity measurement. This instance is reused for efficiency reasons.
     */
    private val previousGravityMeasurement = GravitySensorMeasurement()

    /**
     * Previous gyroscope measurement. This instance is reused for efficiency reasons.
     */
    private val previousGyroscopeMeasurement = GyroscopeSensorMeasurement()

    /**
     * Interpolated accelerometer measurement. This instance is reused for efficiency reasons.
     */
    private val interpolatedAccelerometerMeasurement = AccelerometerSensorMeasurement()

    /**
     * Interpolated gravity measurement. This instance is reused for efficiency reasons.
     */
    private val interpolatedGravityMeasurement = GravitySensorMeasurement()

    /**
     * Interpolated gyroscope measurement. This instance is reused for efficiency reasons.
     */
    private val interpolatedGyroscopeMeasurement = GyroscopeSensorMeasurement()

    /**
     * Flag indicating whether a previous accelerometer measurement has been processed.
     */
    private var hasPreviousAccelerometerMeasurement = false

    /**
     * Flag indicating whether a previous gravity measurement has been processed.
     */
    private var hasPreviousGravityMeasurement = false

    /**
     * Flag indicating whether a previous gyroscope measurement has been processed.
     */
    private var hasPreviousGyroscopeMeasurement = false

    /**
     * Timestamp of last notified synced measurement. This is used to ensure that synced
     * measurements have monotonically increasing timestamp.
     */
    private var lastNotifiedTimestamp = 0L

    /**
     * Timestamp of last accelerometer measurement that was processed and notified.
     */
    private var lastNotifiedAccelerometerTimestamp = 0L

    /**
     * Timestamp of last gravity measurement that was processed and notified.
     */
    private var lastNotifiedGravityTimestamp = 0L

    /**
     * Timestamp of last gyroscope measurement that was processed and notified.
     */
    private var lastNotifiedGyroscopeTimestamp = 0L

    /**
     * Internal buffered accelerometer sensor collector.
     * Collects and buffers accelerometer data.
     */
    private val accelerometerSensorCollector = BufferedAccelerometerSensorCollector(
        context,
        accelerometerSensorType,
        accelerometerSensorDelay,
        accelerometerCapacity,
        accelerometerStartOffsetEnabled,
        stopWhenFilledBuffer,
        accuracyChangedListener = { _, accuracy ->
            accuracyChangedListener?.onAccuracyChanged(
                this@AccelerometerGravityAndGyroscopeSensorMeasurementSyncer,
                SensorType.from(accelerometerSensorType),
                accuracy
            )
        },
        bufferFilledListener = {
            bufferFilledListener?.onBufferFilled(
                this@AccelerometerGravityAndGyroscopeSensorMeasurementSyncer,
                SensorType.from(accelerometerSensorType)
            )
            if (stopWhenFilledBuffer) {
                stop()
            }
        },
        measurementListener = { collector, _, bufferPosition ->
            if (this.skipWhenProcessing && processing) {
                return@BufferedAccelerometerSensorCollector
            }

            synchronized(this@AccelerometerGravityAndGyroscopeSensorMeasurementSyncer) {
                processing = true

                val measurementsBeforePosition =
                    collector.getMeasurementsBeforePosition(bufferPosition)
                val lastTimestamp = measurementsBeforePosition.lastOrNull()?.timestamp
                if (lastTimestamp != null) {
                    mostRecentTimestamp = lastTimestamp

                    // copy measurements
                    copyToAccelerometerMeasurements(measurementsBeforePosition)
                    processMeasurements()
                }

                processing = false
            }
        }
    )

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
                this@AccelerometerGravityAndGyroscopeSensorMeasurementSyncer,
                SensorType.GRAVITY,
                accuracy
            )
        },
        bufferFilledListener = {
            bufferFilledListener?.onBufferFilled(
                this@AccelerometerGravityAndGyroscopeSensorMeasurementSyncer,
                SensorType.GRAVITY
            )
            if (stopWhenFilledBuffer) {
                stop()
            }
        },
        measurementListener = { collector, _, _ ->
            synchronized(this@AccelerometerGravityAndGyroscopeSensorMeasurementSyncer) {
                val mostRecentTimestamp =
                    this@AccelerometerGravityAndGyroscopeSensorMeasurementSyncer.mostRecentTimestamp
                if (mostRecentTimestamp != null) {
                    val measurementsBeforeTimestamp =
                        collector.getMeasurementsBeforeTimestamp(mostRecentTimestamp)
                    if (measurementsBeforeTimestamp.isNotEmpty()) {
                        // copy measurements
                        copyToGravityMeasurements(measurementsBeforeTimestamp)
                    }
                }
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
                this@AccelerometerGravityAndGyroscopeSensorMeasurementSyncer,
                SensorType.from(gyroscopeSensorType),
                accuracy
            )
        },
        bufferFilledListener = {
            bufferFilledListener?.onBufferFilled(
                this@AccelerometerGravityAndGyroscopeSensorMeasurementSyncer,
                SensorType.from(gyroscopeSensorType)
            )
            if (stopWhenFilledBuffer) {
                stop()
            }
        },
        measurementListener = { collector, _, _ ->
            synchronized(this@AccelerometerGravityAndGyroscopeSensorMeasurementSyncer) {
                val mostRecentTimestamp =
                    this@AccelerometerGravityAndGyroscopeSensorMeasurementSyncer.mostRecentTimestamp
                if (mostRecentTimestamp != null) {
                    val measurementsBeforeTimestamp =
                        collector.getMeasurementsBeforeTimestamp(mostRecentTimestamp)
                    if (measurementsBeforeTimestamp.isNotEmpty()) {
                        // copy measurements
                        copyToGyroscopeMeasurements(measurementsBeforeTimestamp)
                    }
                }
            }
        }
    )

    /**
     * Synced measurement to be reused for efficiency purposes.
     */
    override val syncedMeasurement = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
        AccelerometerSensorMeasurement(),
        GravitySensorMeasurement(),
        GyroscopeSensorMeasurement(),
        0L
    )

    /**
     * Gets accelerometer sensor being used to obtain measurements or null if not available.
     * This can be used to obtain additional information about the sensor.
     * @see accelerometerSensorAvailable
     */
    val accelerometerSensor: Sensor?
        get() = accelerometerSensorCollector.sensor

    /**
     * Gets gravity sensor being used to obtain measurements or null if not available.
     * This can be used to obtain additional information about the sensor
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
     * Indicates whether requested accelerometer sensor is available or not.
     */
    val accelerometerSensorAvailable: Boolean
        get() = accelerometerSensorCollector.sensorAvailable

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
     * Gets initial accelerometer offset expressed in nano seconds between first received
     * measurement timestamp and start time expressed in the monotonically increasing system clock
     * obtained by [SystemClock.elapsedRealtimeNanos].
     */
    val accelerometerStartOffset: Long?
        get() = accelerometerSensorCollector.startOffset

    /**
     * Gets initial gravity offset expressed in nano seconds between first received
     * measurement timestamp and start time expressed in the monotonically increasing system clock
     * obtained by [SystemClock.elapsedRealtimeNanos]
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
     * Gets accelerometer collector current usage as a value between 0.0 and 1.0.
     * 0.0 indicates that buffer is empty and no measurement has yet been received.
     * 1.0 indicates that buffer is full.
     */
    val accelerometerCollectorUsage: Float
        get() = accelerometerSensorCollector.usage

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
     * Gets accelerometer buffer current usage as a value between 0.0 and 1.0.
     * 0.0 indicates that buffer is empty and no measurement has yet ben received.
     * 1.0 indicates that buffer is full.
     */
    val accelerometerUsage: Float
        get() = accelerometerMeasurements.size.toFloat() / accelerometerCapacity.toFloat()

    /**
     * Gets gravity buffer current usage as a value between 0.0 and 1.0.
     * 0.0 indicates that buffer is empty and no measurement has yet been received.
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
        running = if (accelerometerSensorCollector.start(startTimestamp)
            && gravitySensorCollector.start(startTimestamp)
            && gyroscopeSensorCollector.start(startTimestamp)
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
        accelerometerSensorCollector.stop()
        gravitySensorCollector.stop()
        gyroscopeSensorCollector.stop()

        reset()
    }

    /**
     * Copies provided measurements into an internal list to be processed when next batch is
     * available.
     * Measurements are moved from a collection of cached available measurements to a new list
     * to avoid inadvertently reusing measurements by internal collector.
     */
    private fun copyToAccelerometerMeasurements(measurements: Collection<AccelerometerSensorMeasurement>) {
        for (measurement in measurements) {
            accelerometerMeasurements.add(AccelerometerSensorMeasurement(measurement))
        }
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
     * Processes all measurements collected in the last batch.
     */
    private fun processMeasurements() {
        val oldestTimestamp = accelerometerMeasurements.firstOrNull()?.timestamp
        this.oldestTimestamp = oldestTimestamp

        alreadyProcessedAccelerometerMeasurements.clear()
        for (accelerometerMeasurement in accelerometerMeasurements) {
            val previousAccelerometerTimestamp = if (hasPreviousAccelerometerMeasurement) {
                previousAccelerometerMeasurement.timestamp
            } else {
                0L
            }
            val accelerometerTimestamp = accelerometerMeasurement.timestamp

            findGravityMeasurementsBetween(
                previousAccelerometerTimestamp,
                accelerometerTimestamp
            )

            var processedAccelerometer = false
            if (foundGravityMeasurements.isEmpty()) {
                if (hasPreviousGravityMeasurement && hasPreviousGyroscopeMeasurement
                    && accelerometerTimestamp > lastNotifiedTimestamp
                    && accelerometerTimestamp >= lastNotifiedAccelerometerTimestamp
                    && previousGravityMeasurement.timestamp > lastNotifiedGravityTimestamp
                    && previousGyroscopeMeasurement.timestamp > lastNotifiedGyroscopeTimestamp
                    && gravityInterpolator.interpolate(
                        previousGravityMeasurement,
                        accelerometerTimestamp,
                        interpolatedGravityMeasurement
                    )
                    && gyroscopeInterpolator.interpolate(
                        previousGyroscopeMeasurement,
                        accelerometerTimestamp,
                        interpolatedGyroscopeMeasurement
                    )
                ) {
                    // generate synchronized measurement when rate of accelerometer is greater
                    // than gravity one
                    syncedMeasurement.timestamp = accelerometerTimestamp
                    syncedMeasurement.accelerometerMeasurement?.copyFrom(
                        accelerometerMeasurement
                    )
                    accelerometerInterpolator.push(accelerometerMeasurement)
                    syncedMeasurement.gravityMeasurement?.copyFrom(interpolatedGravityMeasurement)
                    syncedMeasurement.gyroscopeMeasurement?.copyFrom(
                        interpolatedGyroscopeMeasurement
                    )

                    numberOfProcessedMeasurements++

                    syncedMeasurementListener?.onSyncedMeasurements(
                        this@AccelerometerGravityAndGyroscopeSensorMeasurementSyncer,
                        syncedMeasurement
                    )
                    lastNotifiedTimestamp = accelerometerTimestamp
                    lastNotifiedAccelerometerTimestamp = accelerometerTimestamp
                    lastNotifiedGravityTimestamp = previousGravityMeasurement.timestamp
                    lastNotifiedGyroscopeTimestamp = previousGyroscopeMeasurement.timestamp

                    processedAccelerometer = true
                }
            } else {
                for (gravityMeasurement in foundGravityMeasurements) {
                    val previousGravityTimestamp = if (hasPreviousGravityMeasurement) {
                        previousGravityMeasurement.timestamp
                    } else {
                        0L
                    }
                    val gravityTimestamp = gravityMeasurement.timestamp

                    findGyroscopeMeasurementsBetween(previousGravityTimestamp, gravityTimestamp)

                    var processedGravity = false
                    if (foundGyroscopeMeasurements.isEmpty()) {
                        if (hasPreviousGyroscopeMeasurement
                            && gravityTimestamp > lastNotifiedTimestamp
                            && accelerometerTimestamp >= lastNotifiedAccelerometerTimestamp
                            && gravityTimestamp >= lastNotifiedGravityTimestamp
                            && previousGyroscopeMeasurement.timestamp > lastNotifiedGyroscopeTimestamp
                            && accelerometerInterpolator.interpolate(
                                accelerometerMeasurement,
                                gravityTimestamp,
                                interpolatedAccelerometerMeasurement
                            )
                            && gyroscopeInterpolator.interpolate(
                                previousGyroscopeMeasurement,
                                gravityTimestamp,
                                interpolatedGyroscopeMeasurement
                            )
                        ) {
                            // generate synchronized measurement when rate of gravity is greater
                            // than gyroscope one
                            syncedMeasurement.timestamp = gravityTimestamp
                            syncedMeasurement.accelerometerMeasurement?.copyFrom(
                                interpolatedAccelerometerMeasurement
                            )
                            syncedMeasurement.gravityMeasurement?.copyFrom(gravityMeasurement)
                            gravityInterpolator.push(gravityMeasurement)
                            syncedMeasurement.gyroscopeMeasurement?.copyFrom(
                                interpolatedGyroscopeMeasurement
                            )

                            numberOfProcessedMeasurements++

                            syncedMeasurementListener?.onSyncedMeasurements(
                                this@AccelerometerGravityAndGyroscopeSensorMeasurementSyncer,
                                syncedMeasurement
                            )
                            lastNotifiedTimestamp = gravityTimestamp
                            lastNotifiedAccelerometerTimestamp = accelerometerTimestamp
                            lastNotifiedGravityTimestamp = gravityTimestamp
                            lastNotifiedGyroscopeTimestamp =
                                previousGyroscopeMeasurement.timestamp

                            processedGravity = true
                        }
                    } else {
                        for (gyroscopeMeasurement in foundGyroscopeMeasurements) {
                            val gyroscopeTimestamp = gyroscopeMeasurement.timestamp
                            if (gyroscopeTimestamp > lastNotifiedTimestamp
                                && accelerometerTimestamp >= lastNotifiedAccelerometerTimestamp
                                && gravityTimestamp >= lastNotifiedGravityTimestamp
                                && gyroscopeTimestamp >= lastNotifiedGyroscopeTimestamp
                                && accelerometerInterpolator.interpolate(
                                    accelerometerMeasurement,
                                    gyroscopeTimestamp,
                                    interpolatedAccelerometerMeasurement
                                )
                                && gravityInterpolator.interpolate(
                                    gravityMeasurement,
                                    gyroscopeTimestamp,
                                    interpolatedGravityMeasurement
                                )
                            ) {
                                // generate synchronized measurement when rate of gyroscope is
                                // greater than gravity and accelerometer one
                                syncedMeasurement.timestamp = gyroscopeTimestamp
                                syncedMeasurement.accelerometerMeasurement?.copyFrom(
                                    interpolatedAccelerometerMeasurement
                                )
                                syncedMeasurement.gravityMeasurement?.copyFrom(
                                    interpolatedGravityMeasurement
                                )
                                syncedMeasurement.gyroscopeMeasurement?.copyFrom(
                                    gyroscopeMeasurement
                                )
                                gyroscopeInterpolator.push(gyroscopeMeasurement)

                                numberOfProcessedMeasurements++

                                syncedMeasurementListener?.onSyncedMeasurements(
                                    this@AccelerometerGravityAndGyroscopeSensorMeasurementSyncer,
                                    syncedMeasurement
                                )
                                lastNotifiedTimestamp = gyroscopeTimestamp
                                lastNotifiedAccelerometerTimestamp = accelerometerTimestamp
                                lastNotifiedGravityTimestamp = gravityTimestamp
                                lastNotifiedGyroscopeTimestamp = gyroscopeTimestamp
                            }
                        }
                        processedGravity = true

                        previousGyroscopeMeasurement.copyFrom(foundGyroscopeMeasurements.last())
                        hasPreviousGyroscopeMeasurement = true
                    }

                    if (processedGravity) {
                        alreadyProcessedGravityMeasurements.add(gravityMeasurement)

                        // remove processed gyroscope measurements
                        gyroscopeMeasurements.removeAll(foundGyroscopeMeasurements)
                    }
                }

                processedAccelerometer = true

                previousGravityMeasurement.copyFrom(foundGravityMeasurements.last())
                hasPreviousGravityMeasurement = true
            }

            if (processedAccelerometer) {
                alreadyProcessedAccelerometerMeasurements.add(accelerometerMeasurement)

                // remove processed gravity measurements
                gravityMeasurements.removeAll(alreadyProcessedGravityMeasurements)
            }

            previousAccelerometerMeasurement.copyFrom(accelerometerMeasurement)
            hasPreviousAccelerometerMeasurement = true
        }

        if (alreadyProcessedAccelerometerMeasurements.isNotEmpty()) {
            // remove processed accelerometer measurements
            accelerometerMeasurements.removeAll(alreadyProcessedAccelerometerMeasurements)
        }

        foundGravityMeasurements.clear()
        foundGyroscopeMeasurements.clear()

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

        hasPreviousAccelerometerMeasurement = false
        hasPreviousGravityMeasurement = false
        hasPreviousGyroscopeMeasurement = false
        lastNotifiedTimestamp = 0L
        lastNotifiedAccelerometerTimestamp = 0L
        lastNotifiedGravityTimestamp = 0L
        lastNotifiedGyroscopeTimestamp = 0L

        accelerometerInterpolator.reset()
        gravityInterpolator.reset()
        gyroscopeInterpolator.reset()
    }

    /**
     * Clears internal collections and resets
     */
    private fun clearCollectionsAndReset() {
        accelerometerMeasurements.clear()
        gravityMeasurements.clear()
        gyroscopeMeasurements.clear()
        alreadyProcessedAccelerometerMeasurements.clear()
        alreadyProcessedGravityMeasurements.clear()
        alreadyProcessedGyroscopeMeasurements.clear()
        foundGravityMeasurements.clear()
        foundGyroscopeMeasurements.clear()

        reset()
    }

    /**
     * Finds gravity measurements in the buffer within provided minimum and maximum timestamp.
     *
     * @param minTimestamp minimum timestamp.
     * @param maxTimestamp maximum timestamp.
     * @return found gravity measurements or empty.
     */
    private fun findGravityMeasurementsBetween(
        minTimestamp: Long,
        maxTimestamp: Long
    ) {
        findMeasurementsBetween(
            minTimestamp,
            maxTimestamp,
            gravityMeasurements,
            foundGravityMeasurements
        )
    }

    /**
     * Finds gyroscope measurements in the buffer within provided minimum and maximum timestamp.
     *
     * @param minTimestamp minimum timestamp.
     * @param maxTimestamp maximum timestamp.
     * @return found gyroscope measurements or empty.
     */
    @Synchronized
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
            alreadyProcessedAccelerometerMeasurements,
            accelerometerMeasurements,
            SensorType.from(accelerometerSensorType),
            this,
            staleDetectedMeasurementsListener
        )

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
    }

    /**
     * Initializes this syncer.
     */
    init {
        // check that capacities are larger than zero.
        require(accelerometerCapacity > 0)
        require(gravityCapacity > 0)
        require(gyroscopeCapacity > 0)
    }

    companion object {
        /**
         * Default capacity for accelerometer measurement cache.
         */
        const val DEFAULT_ACCELEROMETER_CAPACITY = 100

        /**
         * Default capacity for gravity measurement cache.
         */
        const val DEFAULT_GRAVITY_CAPACITY = 10

        /**
         * Default capacity for gyroscope measurement cache.
         */
        const val DEFAULT_GYROSCOPE_CAPACITY = 10

        /**
         * Default offset to consider a measurement as stale to be removed from cache.
         * By default the value is equal to 5 seconds expressed in nanoseconds.
         */
        const val DEFAULT_STALE_OFFSET_NANOS = 5000_000_000L
    }
}