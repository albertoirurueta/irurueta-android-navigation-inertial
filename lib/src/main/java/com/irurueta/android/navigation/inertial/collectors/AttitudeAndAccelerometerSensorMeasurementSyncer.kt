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
import java.util.ArrayDeque

/**
 * Syncs attitude and accelerometer sensor measurements in case they arrive with certain delay.
 *
 * Typically when synchronization is needed is for correct pose estimation, for attitude estimation,
 * not synced sensor measurements can also be used to achieve a reasonable estimation.
 *
 * @property context Android context.
 * @property attitudeSensorType One of the supported attitude sensor types.
 * @property accelerometerSensorType One of the supported accelerometer sensor types.
 * @property attitudeSensorDelay Delay of attitude sensor between samples.
 * @property accelerometerSensorDelay Delay of accelerometer sensor between samples.
 * @property attitudeCapacity capacity of attitude buffer.
 * @property accelerometerCapacity capacity of accelerometer buffer.
 * @property attitudeStartOffsetEnabled indicates whether attitude start offset will be computed
 * when first measurement is received. True indicates that offset is computed, false assumes that
 * offset is null.
 * @property accelerometerStartOffsetEnabled indicates whether accelerometer start offset will be
 * computed when first measurement is received. True indicates that offset is computed, false
 * assumes that offset is null.
 * @property stopWhenFilledBuffer true to stop syncer when any buffer completely fills, false to
 * continue processing measurements at the expense of loosing old data. This will be notified using
 * [bufferFilledListener].
 * @property staleOffsetNanos offset respect most recent received timestamp of a measurement to
 * consider the measurement as stale so that t is skipped from synced measurement processing and
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
 * @property attitudeInterpolator interpolator for attitude measurements.
 * @property accelerometerInterpolator interpolator for accelerometer measurements.
 */
class AttitudeAndAccelerometerSensorMeasurementSyncer(
    context: Context,
    val attitudeSensorType: AttitudeSensorType = AttitudeSensorType.ABSOLUTE_ATTITUDE,
    val accelerometerSensorType: AccelerometerSensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
    val attitudeSensorDelay: SensorDelay = SensorDelay.FASTEST,
    val accelerometerSensorDelay: SensorDelay = SensorDelay.FASTEST,
    val attitudeCapacity: Int = DEFAULT_ATTITUDE_CAPACITY,
    val accelerometerCapacity: Int = DEFAULT_ACCELEROMETER_CAPACITY,
    val attitudeStartOffsetEnabled: Boolean = false,
    val accelerometerStartOffsetEnabled: Boolean = false,
    stopWhenFilledBuffer: Boolean = true,
    staleOffsetNanos: Long = AttitudeAccelerometerAndGyroscopeSensorMeasurementSyncer.DEFAULT_STALE_OFFSET_NANOS,
    staleDetectionEnabled: Boolean = true,
    skipWhenProcessing: Boolean = true,
    accuracyChangedListener: OnAccuracyChangedListener<AttitudeAndAccelerometerSyncedSensorMeasurement, AttitudeAndAccelerometerSensorMeasurementSyncer>? = null,
    bufferFilledListener: OnBufferFilledListener<AttitudeAndAccelerometerSyncedSensorMeasurement, AttitudeAndAccelerometerSensorMeasurementSyncer>? = null,
    syncedMeasurementListener: OnSyncedMeasurementsListener<AttitudeAndAccelerometerSyncedSensorMeasurement, AttitudeAndAccelerometerSensorMeasurementSyncer>? = null,
    staleDetectedMeasurementsListener: OnStaleDetectedMeasurementsListener<AttitudeAndAccelerometerSyncedSensorMeasurement, AttitudeAndAccelerometerSensorMeasurementSyncer>? = null,
    val attitudeInterpolator: AttitudeSensorMeasurementInterpolator = AttitudeLinearSensorMeasurementInterpolator(),
    val accelerometerInterpolator: AccelerometerSensorMeasurementInterpolator = AccelerometerQuadraticSensorMeasurementInterpolator(),
) : SensorMeasurementSyncer<AttitudeAndAccelerometerSyncedSensorMeasurement, AttitudeAndAccelerometerSensorMeasurementSyncer>(
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
     * Attitude measurements to be processed in next batch.
     */
    private val attitudeMeasurements = ArrayDeque<AttitudeSensorMeasurement>(attitudeCapacity)

    /**
     * Accelerometer measurements to be processed in next batch.
     */
    private val accelerometerMeasurements =
        ArrayDeque<AccelerometerSensorMeasurement>(accelerometerCapacity)

    /**
     * List of attitude measurements that have already been processed and can be returned back
     * to the cache of available measurements.
     */
    private val alreadyProcessedAttitudeMeasurements =
        ArrayDeque<AttitudeSensorMeasurement>(attitudeCapacity)

    /**
     * List of accelerometer measurements that have already been processed and can be returned back
     * to the cache of available measurements.
     */
    private val alreadyProcessedAccelerometerMeasurements =
        ArrayDeque<AccelerometerSensorMeasurement>(accelerometerCapacity)

    /**
     * List of found accelerometer measurements.
     */
    private val foundAccelerometerMeasurements =
        ArrayDeque<AccelerometerSensorMeasurement>(accelerometerCapacity)

    /**
     * Previous attitude measurement. This instance is reused for efficiency reasons.
     */
    private val previousAttitudeMeasurement = AttitudeSensorMeasurement()

    /**
     * Previous accelerometer measurement. This instance is reused for efficiency reasons.
     */
    private val previousAccelerometerMeasurement = AccelerometerSensorMeasurement()

    /**
     * Interpolated attitude measurement. This instance is reused for efficiency reasons.
     */
    private val interpolatedAttitudeMeasurement = AttitudeSensorMeasurement()

    /**
     * Interpolated accelerometer measurement. This instance is reused for efficiency reasons.
     */
    private val interpolatedAccelerometerMeasurement = AccelerometerSensorMeasurement()

    /**
     * Flag indicating whether a previous attitude measurement has been processed.
     */
    private var hasPreviousAttitudeMeasurement = false

    /**
     * Flag indicating whether a previous accelerometer measurement has been processed.
     */
    private var hasPreviousAccelerometerMeasurement = false

    /**
     * Timestamp of last notified synced measurement. This is used to ensure that synced
     * measurements have monotonically increasing timestamp.
     */
    private var lastNotifiedTimestamp = 0L

    /**
     * Timestamp of last attitude measurement that was processed and notified.
     */
    private var lastNotifiedAttitudeTimestamp = 0L

    /**
     * Timestamp of last accelerometer measurement that was processed and notified.
     */
    private var lastNotifiedAccelerometerTimestamp = 0L

    /**
     * Internal buffered attitude sensor collector.
     * Collects and buffers attitude data.
     */
    private val attitudeSensorCollector = BufferedAttitudeSensorCollector(
        context,
        attitudeSensorType,
        attitudeSensorDelay,
        attitudeCapacity,
        attitudeStartOffsetEnabled,
        stopWhenFilledBuffer,
        accuracyChangedListener = { _, accuracy ->
            accuracyChangedListener?.onAccuracyChanged(
                this@AttitudeAndAccelerometerSensorMeasurementSyncer,
                SensorType.from(attitudeSensorType),
                accuracy
            )
        },
        bufferFilledListener = {
            bufferFilledListener?.onBufferFilled(
                this@AttitudeAndAccelerometerSensorMeasurementSyncer,
                SensorType.from(attitudeSensorType)
            )
            if (stopWhenFilledBuffer) {
                stop()
            }
        },
        measurementListener = { collector, _, bufferPosition ->
            if (this.skipWhenProcessing && processing) {
                return@BufferedAttitudeSensorCollector
            }

            synchronized(this@AttitudeAndAccelerometerSensorMeasurementSyncer) {
                processing = true

                val measurementsBeforePosition =
                    collector.getMeasurementsBeforePosition(bufferPosition)
                val lastTimestamp = measurementsBeforePosition.lastOrNull()?.timestamp
                if (lastTimestamp != null) {
                    mostRecentTimestamp = lastTimestamp

                    // copy measurements
                    copyToAttitudeMeasurements(measurementsBeforePosition)
                    processMeasurements()
                }

                processing = false
            }
        }
    )

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
                this@AttitudeAndAccelerometerSensorMeasurementSyncer,
                SensorType.from(accelerometerSensorType),
                accuracy
            )
        },
        bufferFilledListener = {
            bufferFilledListener?.onBufferFilled(
                this@AttitudeAndAccelerometerSensorMeasurementSyncer,
                SensorType.from(accelerometerSensorType)
            )
            if (stopWhenFilledBuffer) {
                stop()
            }
        },
        measurementListener = { collector, _, _ ->
            if (this.skipWhenProcessing && processing) {
                return@BufferedAccelerometerSensorCollector
            }

            synchronized(this@AttitudeAndAccelerometerSensorMeasurementSyncer) {
                processing = true

                val mostRecentTimestamp =
                    this@AttitudeAndAccelerometerSensorMeasurementSyncer.mostRecentTimestamp
                if (mostRecentTimestamp != null) {
                    val measurementsBeforeTimestamp =
                        collector.getMeasurementsBeforeTimestamp(mostRecentTimestamp)
                    if (measurementsBeforeTimestamp.isNotEmpty()) {
                        // copy measurements
                        copyToAccelerometerMeasurements(measurementsBeforeTimestamp)
                    }
                }

                processing = false
            }
        }
    )

    /**
     * Synced measurement to be reused for efficiency purposes.
     */
    override val syncedMeasurement = AttitudeAndAccelerometerSyncedSensorMeasurement(
        AttitudeSensorMeasurement(),
        AccelerometerSensorMeasurement(),
        0L
    )

    /**
     * Gets attitude sensor being used to obtain measurements or null if not available.
     * This can be used to obtain additional information about the sensor.
     * @see attitudeSensorAvailable
     */
    val attitudeSensor: Sensor?
        get() = attitudeSensorCollector.sensor

    /**
     * Gets accelerometer sensor being used to obtain measurements or null if not available.
     * This can be used to obtain additional information about the sensor.
     * @see accelerometerSensorAvailable
     */
    val accelerometerSensor: Sensor?
        get() = accelerometerSensorCollector.sensor

    /**
     * Indicates whether requested attitude sensor is available or not.
     */
    val attitudeSensorAvailable: Boolean
        get() = attitudeSensorCollector.sensorAvailable

    /**
     * Indicates whether requested accelerometer sensor is available or not.
     */
    val accelerometerSensorAvailable: Boolean
        get() = accelerometerSensorCollector.sensorAvailable

    /**
     * Gets initial attitude offset expressed in nano seconds between first received measurement
     * timestamp and start time expressed in the monotonically increasing system clock obtained by
     * [SystemClock.elapsedRealtimeNanos].
     */
    val attitudeStartOffset: Long?
        get() = attitudeSensorCollector.startOffset

    /**
     * Gets initial accelerometer offset expressed in nano seconds between first received
     * measurement timestamp and start time expressed in the monotonically increasing system clock
     * obtained by [SystemClock.elapsedRealtimeNanos].
     */
    val accelerometerStartOffset: Long?
        get() = accelerometerSensorCollector.startOffset

    /**
     * Gets attitude collector current usage as a value between 0.0 and 1.0.
     * 0.0 indicates that buffer is empty and no measurement has yet been received.
     * 1.0 indicates that buffer is full.
     */
    val attitudeCollectorUsage: Float
        get() = attitudeSensorCollector.usage

    /**
     * Gets accelerometer collector current usage as a value between 0.0 and 1.0.
     * 0.0 indicates that buffer is empty and no measurement has yet been received.
     * 1.0 indicates that buffer is full.
     */
    val accelerometerCollectorUsage: Float
        get() = accelerometerSensorCollector.usage

    /**
     * Gets attitude buffer current usage as a value between 0.0 and 1.0.
     * 0.0 indicates that buffer is empty and no measurement has yet ben received.
     * 1.0 indicates that buffer is full.
     */
    val attitudeUsage: Float
        get() = attitudeMeasurements.size.toFloat() / attitudeCapacity.toFloat()

    /**
     * Gets accelerometer buffer current usage as a value between 0.0 and 1.0.
     * 0.0 indicates that buffer is empty and no measurement has yet ben received.
     * 1.0 indicates that buffer is full.
     */
    val accelerometerUsage: Float
        get() = accelerometerMeasurements.size.toFloat() / accelerometerCapacity.toFloat()

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
        running = if (attitudeSensorCollector.start(startTimestamp)
            && accelerometerSensorCollector.start(startTimestamp)
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
        attitudeSensorCollector.stop()
        accelerometerSensorCollector.stop()

        reset()
    }

    /**
     * Copies provided measurements into an internal list to be processed when next batch is
     * available.
     * Measurements are moved from a collection of cached available measurements to a new list
     * to avoid inadvertently reusing measurements by internal collector.
     */
    private fun copyToAttitudeMeasurements(measurements: Collection<AttitudeSensorMeasurement>) {
        for (measurement in measurements) {
            attitudeMeasurements.add(AttitudeSensorMeasurement(measurement))
        }
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
     * Processes all measurements collected in the last batch.
     */
    private fun processMeasurements() {
        val oldestTimestamp = attitudeMeasurements.firstOrNull()?.timestamp
        this.oldestTimestamp = oldestTimestamp

        alreadyProcessedAttitudeMeasurements.clear()
        for (attitudeMeasurement in attitudeMeasurements) {
            val previousAttitudeTimestamp = if (hasPreviousAttitudeMeasurement) {
                previousAttitudeMeasurement.timestamp
            } else {
                0L
            }
            val attitudeTimestamp = attitudeMeasurement.timestamp

            findAccelerometerMeasurementsBetween(previousAttitudeTimestamp, attitudeTimestamp)

            var processedAttitude = false
            if (foundAccelerometerMeasurements.isEmpty()) {
                if (hasPreviousAccelerometerMeasurement
                    && attitudeTimestamp > lastNotifiedTimestamp
                    && attitudeTimestamp >= lastNotifiedAttitudeTimestamp
                    && previousAccelerometerMeasurement.timestamp > lastNotifiedAccelerometerTimestamp
                    && accelerometerInterpolator.interpolate(
                        previousAccelerometerMeasurement,
                        attitudeTimestamp,
                        interpolatedAccelerometerMeasurement
                    )
                ) {
                    // generate synchronized measurement when rate of attitude is greater than
                    // accelerometer one
                    syncedMeasurement.timestamp = attitudeTimestamp
                    syncedMeasurement.attitudeMeasurement?.copyFrom(attitudeMeasurement)
                    attitudeInterpolator.push(attitudeMeasurement)
                    syncedMeasurement.accelerometerMeasurement?.copyFrom(
                        interpolatedAccelerometerMeasurement
                    )

                    numberOfProcessedMeasurements++

                    syncedMeasurementListener?.onSyncedMeasurements(
                        this@AttitudeAndAccelerometerSensorMeasurementSyncer,
                        syncedMeasurement
                    )
                    lastNotifiedTimestamp = attitudeTimestamp
                    lastNotifiedAttitudeTimestamp = attitudeTimestamp
                    lastNotifiedAccelerometerTimestamp = previousAccelerometerMeasurement.timestamp

                    processedAttitude = true
                }
            } else {
                for (accelerometerMeasurement in foundAccelerometerMeasurements) {
                    val accelerometerTimestamp = accelerometerMeasurement.timestamp
                    if (accelerometerTimestamp > lastNotifiedTimestamp
                        && attitudeTimestamp >= lastNotifiedAttitudeTimestamp
                        && accelerometerTimestamp >= lastNotifiedAccelerometerTimestamp
                        && attitudeInterpolator.interpolate(
                            attitudeMeasurement,
                            accelerometerTimestamp,
                            interpolatedAttitudeMeasurement
                        )
                    ) {
                        // generate synchronized measurement when rate of accelerometer is greater
                        // than attitude one
                        syncedMeasurement.timestamp = accelerometerTimestamp
                        syncedMeasurement.attitudeMeasurement?.copyFrom(
                            interpolatedAttitudeMeasurement
                        )
                        syncedMeasurement.accelerometerMeasurement?.copyFrom(
                            accelerometerMeasurement
                        )
                        accelerometerInterpolator.push(accelerometerMeasurement)

                        numberOfProcessedMeasurements++

                        syncedMeasurementListener?.onSyncedMeasurements(
                            this@AttitudeAndAccelerometerSensorMeasurementSyncer,
                            syncedMeasurement
                        )
                        lastNotifiedTimestamp = accelerometerTimestamp
                        lastNotifiedAttitudeTimestamp = attitudeTimestamp
                        lastNotifiedAccelerometerTimestamp = accelerometerTimestamp
                    }
                }
                processedAttitude = true

                previousAccelerometerMeasurement.copyFrom(foundAccelerometerMeasurements.last())
                hasPreviousAccelerometerMeasurement = true
            }

            if (processedAttitude) {
                alreadyProcessedAttitudeMeasurements.add(attitudeMeasurement)

                // remove processed accelerometer measurements
                accelerometerMeasurements.removeAll(foundAccelerometerMeasurements)
            }

            previousAttitudeMeasurement.copyFrom(attitudeMeasurement)
            hasPreviousAttitudeMeasurement = true
        }

        if (alreadyProcessedAttitudeMeasurements.size > 0) {
            // return processed attitude measurements
            attitudeMeasurements.removeAll(alreadyProcessedAttitudeMeasurements)
        }

        foundAccelerometerMeasurements.clear()

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

        hasPreviousAttitudeMeasurement = false
        hasPreviousAccelerometerMeasurement = false
        lastNotifiedTimestamp = 0L
        lastNotifiedAttitudeTimestamp = 0L
        lastNotifiedAccelerometerTimestamp = 0L

        attitudeInterpolator.reset()
        accelerometerInterpolator.reset()
    }

    /**
     * Clears internal collections and resets
     */
    private fun clearCollectionsAndReset() {
        attitudeMeasurements.clear()
        accelerometerMeasurements.clear()
        alreadyProcessedAttitudeMeasurements.clear()
        alreadyProcessedAccelerometerMeasurements.clear()
        foundAccelerometerMeasurements.clear()

        reset()
    }

    /**
     * Finds accelerometer measurements in the buffer within provided minimum and maximum timestamp.
     *
     * @param minTimestamp minimum timestamp.
     * @param maxTimestamp maximum timestamp.
     * @return found accelerometer measurements or empty.
     */
    private fun findAccelerometerMeasurementsBetween(
        minTimestamp: Long,
        maxTimestamp: Long
    ) {
        findMeasurementsBetween(
            minTimestamp,
            maxTimestamp,
            accelerometerMeasurements,
            foundAccelerometerMeasurements
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
            alreadyProcessedAttitudeMeasurements,
            attitudeMeasurements,
            SensorType.from(attitudeSensorType),
            this,
            staleDetectedMeasurementsListener
        )

        cleanupStaleMeasurements(
            staleTimestamp,
            alreadyProcessedAccelerometerMeasurements,
            accelerometerMeasurements,
            SensorType.from(accelerometerSensorType),
            this,
            staleDetectedMeasurementsListener
        )
    }

    /**
     * Initializes this syncer.
     */
    init {
        // check that capacities are larger than zero.
        require(attitudeCapacity > 0)
        require(accelerometerCapacity > 0)
    }

    companion object {
        /**
         * Default capacity for attitude measurement cache.
         */
        const val DEFAULT_ATTITUDE_CAPACITY = 100

        /**
         * Default capacity for accelerometer measurement cache.
         */
        const val DEFAULT_ACCELEROMETER_CAPACITY = 100

        /**
         * Default offset to consider a measurement as stale to be removed from cache.
         * By default the value is equal to 5 seconds expressed in nanoseconds.
         */
        const val DEFAULT_STALE_OFFSET_NANOS = 5000_000_000L
    }
}