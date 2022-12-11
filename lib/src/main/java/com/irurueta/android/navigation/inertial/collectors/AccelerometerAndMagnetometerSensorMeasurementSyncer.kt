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
import java.util.*

/**
 * Syncs accelerometer and magnetometer sensor measurements in case they arrive with certain delay.
 *
 * Typically when synchronization is needed is for correct pose estimation, for attitude estimation,
 * not synced sensor measurements can also be used to achieve a reasonable estimation.
 *
 * @property context Android context.
 * @property accelerometerSensorType One of the supported accelerometer sensor types.
 * @property magnetometerSensorType One of the supported magnetometer sensor types.
 * @property accelerometerSensorDelay Delay of accelerometer sensor between samples.
 * @property magnetometerSensorDelay Delay of magnetometer sensor between samples.
 * @property accelerometerCapacity capacity of accelerometer buffer.
 * @property magnetometerCapacity capacity of magnetometer buffer.
 * @property accelerometerStartOffsetEnabled indicates whether accelerometer start offset will be
 * computed when first measurement is received. True indicates that offset is computed, false
 * assumes that offset is null.
 * @property magnetometerStartOffsetEnabled indicates whether magnetometer start offset will be
 * computed when first measurement is received. True indicates that offset is computed, false
 * assumes that offset is null.
 * @property stopWhenFilledBuffer true to stop syncer when any buffer completely fills, false to
 * continue processing measurements at the expense of loosing old data. This will be notified using
 * [bufferFilledListener].
 * @property outOfOrderDetectionEnabled true to enable out of order detection of measurements, false
 * otherwise.
 * @property stopWhenOutOfOrder true to stop syncer if any out of order measurement is detected,
 * false to continue processing measurements at the expense of receiving unordered data. This will
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
class AccelerometerAndMagnetometerSensorMeasurementSyncer(
    context: Context,
    val accelerometerSensorType: AccelerometerSensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
    val magnetometerSensorType: MagnetometerSensorType = MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
    val accelerometerSensorDelay: SensorDelay = SensorDelay.FASTEST,
    val magnetometerSensorDelay: SensorDelay = SensorDelay.FASTEST,
    val accelerometerCapacity: Int = BufferedSensorCollector.DEFAULT_CAPACITY,
    val magnetometerCapacity: Int = BufferedSensorCollector.DEFAULT_CAPACITY,
    val accelerometerStartOffsetEnabled: Boolean = true,
    val magnetometerStartOffsetEnabled: Boolean = true,
    stopWhenFilledBuffer: Boolean = true,
    outOfOrderDetectionEnabled: Boolean = false,
    stopWhenOutOfOrder: Boolean = true,
    accuracyChangedListener: OnAccuracyChangedListener<AccelerometerAndMagnetometerSyncedSensorMeasurement, AccelerometerAndMagnetometerSensorMeasurementSyncer>? = null,
    bufferFilledListener: OnBufferFilledListener<AccelerometerAndMagnetometerSyncedSensorMeasurement, AccelerometerAndMagnetometerSensorMeasurementSyncer>? = null,
    outOfOrderMeasurementListener: OnOutOfOrderMeasurementListener<AccelerometerAndMagnetometerSyncedSensorMeasurement, AccelerometerAndMagnetometerSensorMeasurementSyncer>? = null,
    syncedMeasurementListener: OnSyncedMeasurementsListener<AccelerometerAndMagnetometerSyncedSensorMeasurement, AccelerometerAndMagnetometerSensorMeasurementSyncer>? = null
) : SensorMeasurementSyncer<AccelerometerAndMagnetometerSyncedSensorMeasurement, AccelerometerAndMagnetometerSensorMeasurementSyncer>(
    context,
    stopWhenFilledBuffer,
    outOfOrderDetectionEnabled,
    stopWhenOutOfOrder,
    accuracyChangedListener,
    bufferFilledListener,
    outOfOrderMeasurementListener,
    syncedMeasurementListener
) {
    /**
     * Cached accelerometer measurements ready to be used.
     */
    private val availableAccelerometerMeasurements = LinkedList<AccelerometerSensorMeasurement>()

    /**
     * Cached magnetometer measurements ready to be used.
     */
    private val availableMagnetometerMeasurements = LinkedList<MagnetometerSensorMeasurement>()

    /**
     * Accelerometer measurements to be processed in next batch.
     */
    private val accelerometerMeasurements = LinkedList<AccelerometerSensorMeasurement>()

    /**
     * Magnetometer measurements to be processed in next batch.
     */
    private val magnetometerMeasurements = LinkedList<MagnetometerSensorMeasurement>()

    /**
     * List of accelerometer measurements that have already been processed and can be returned back
     * to the cache of available measurements.
     */
    private val alreadyProcessedAccelerometerMeasurements =
        LinkedList<AccelerometerSensorMeasurement>()

    /**
     * List of magnetometer measurements that have already been processed and can be returned back
     * to the cache of available measurements.
     */
    private val alreadyProcessedMagnetometerMeasurements =
        LinkedList<MagnetometerSensorMeasurement>()

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
                this@AccelerometerAndMagnetometerSensorMeasurementSyncer,
                SensorType.from(accelerometerSensorType),
                accuracy
            )
        },
        bufferFilledListener = {
            bufferFilledListener?.onBufferFilled(
                this@AccelerometerAndMagnetometerSensorMeasurementSyncer,
                SensorType.from(accelerometerSensorType)
            )
            if (stopWhenFilledBuffer) {
                stop()
            }
        },
        measurementListener = { collector, _, bufferPosition ->
            val measurementsBeforePosition =
                collector.getMeasurementsBeforePosition(bufferPosition)
            val lastTimestamp = measurementsBeforePosition.lastOrNull()?.timestamp
            if (lastTimestamp != null) {
                mostRecentTimestamp = lastTimestamp

                // copy measurements
                copyToAccelerometerMeasurements(measurementsBeforePosition)
                processMeasurements()
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
                this@AccelerometerAndMagnetometerSensorMeasurementSyncer,
                SensorType.from(magnetometerSensorType),
                accuracy
            )
        },
        bufferFilledListener = {
            bufferFilledListener?.onBufferFilled(
                this@AccelerometerAndMagnetometerSensorMeasurementSyncer,
                SensorType.from(magnetometerSensorType)
            )
            if (stopWhenFilledBuffer) {
                stop()
            }
        },
        measurementListener = { collector, _, _ ->
            val mostRecentTimestamp =
                this@AccelerometerAndMagnetometerSensorMeasurementSyncer.mostRecentTimestamp
            if (mostRecentTimestamp != null) {
                val measurementsBeforeTimestamp =
                    collector.getMeasurementsBeforeTimestamp(mostRecentTimestamp)
                if (measurementsBeforeTimestamp.isNotEmpty()) {
                    // copy measurements
                    copyToMagnetometerMeasurements(measurementsBeforeTimestamp)
                    processMeasurements()
                }
            }
        }
    )

    /**
     * Synced measurement to be reused for efficiency purposes.
     */
    override val syncedMeasurement = AccelerometerAndMagnetometerSyncedSensorMeasurement()

    /**
     * Gets accelerometer sensor being used to obtain measurements or null if not available.
     * This can be used to obtain additional information about the sensor.
     * @see accelerometerSensorAvailable
     */
    val accelerometerSensor: Sensor?
        get() = accelerometerSensorCollector.sensor

    /**
     * Gets magnetometer sensor being used to obtain measurements or null if not available.
     * This can be used to obtain additional information about the sensor.
     * @see magnetometerSensorAvailable
     */
    val magnetometerSensor: Sensor?
        get() = magnetometerSensorCollector.sensor

    /**
     * Indicates whether requested accelerometer sensor is available or not.
     */
    val accelerometerSensorAvailable: Boolean
        get() = accelerometerSensorCollector.sensorAvailable

    /**
     * Indicates whether requested magnetometer sensor is available or not.
     */
    val magnetometerSensorAvailable: Boolean
        get() = magnetometerSensorCollector.sensorAvailable

    /**
     * Gets initial accelerometer offset expressed in nano seconds between first received
     * measurement timestamp and start time expressed in the monotonically increasing system clock
     * obtained by [SystemClock.elapsedRealtimeNanos].
     */
    val accelerometerStartOffset: Long?
        get() = accelerometerSensorCollector.startOffset

    /**
     * Gets initial magnetometer offset expressed in nano seconds between first received
     * measurement timestamp and start time expressed in the monotonically increasing system clock
     * obtained by [SystemClock.elapsedRealtimeNanos]
     */
    val magnetometerStartOffset: Long?
        get() = magnetometerSensorCollector.startOffset

    /**
     * Gets accelerometer collector current usage as a value between 0.0 and 1.0.
     * 0.0 indicates that buffer is empty and no measurement has yet been received.
     * 1.0 indicates that buffer is full.
     */
    val accelerometerUsage: Float
        get() = accelerometerSensorCollector.usage

    /**
     * Gets magnetometer collector current usage as a value between 0.0 and 1.0.
     * 0.0 indicates that buffer is empty and no measurement has yet been received.
     * 1.0 indicates that buffer is full.
     */
    val magnetometerUsage: Float
        get() = magnetometerSensorCollector.usage

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

        this.startTimestamp = startTimestamp
        running = if (accelerometerSensorCollector.start(startTimestamp)
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
    override fun stop() {
        accelerometerSensorCollector.stop()
        magnetometerSensorCollector.stop()
        initializeMeasurements()

        numberOfProcessedMeasurements = 0
        mostRecentTimestamp = null
        oldestTimestamp = null
        running = false
    }

    /**
     * Copies provided measurements into an internal list to be processed when next batch is
     * available.
     * Measurements are moved from a collection of cached available measurements to a new list
     * to avoid inadvertently reusing measurements by internal collector.
     */
    private fun copyToAccelerometerMeasurements(measurements: Collection<AccelerometerSensorMeasurement>) {
        for (measurement in measurements) {
            // move an available measurement to the list of measurements
            // if for any reason there are no more available cached measurements, build new ones
            val availableMeasurement = availableAccelerometerMeasurements.removeFirstOrNull()
                ?: AccelerometerSensorMeasurement()
            availableMeasurement.copyFrom(measurement)
            accelerometerMeasurements.add(availableMeasurement)
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
            // move an available measurement to the list of measurements
            // if for any reason there are no more available cached measurements, build new ones
            val availableMeasurement = availableMagnetometerMeasurements.removeFirstOrNull()
                ?: MagnetometerSensorMeasurement()
            availableMeasurement.copyFrom(measurement)
            magnetometerMeasurements.add(availableMeasurement)
        }
    }

    /**
     * Processes all measurements collected in the last batch.
     */
    private fun processMeasurements() {
        val oldestTimestamp = accelerometerMeasurements.firstOrNull()?.timestamp
        this.oldestTimestamp = oldestTimestamp

        var stopped = false
        val initialNumberOfProcessedMeasurements = numberOfProcessedMeasurements
        var previousAccelerometerMeasurement: AccelerometerSensorMeasurement? = null
        alreadyProcessedAccelerometerMeasurements.clear()
        for (accelerometerMeasurement in accelerometerMeasurements) {
            val previousAccelerometerTimestamp = previousAccelerometerMeasurement?.timestamp
            val accelerometerTimestamp = accelerometerMeasurement.timestamp

            alreadyProcessedMagnetometerMeasurements.clear()
            for (magnetometerMeasurement in magnetometerMeasurements) {
                val magnetometerTimestamp = magnetometerMeasurement.timestamp
                if (outOfOrderDetectionEnabled && oldestTimestamp != null && magnetometerTimestamp < oldestTimestamp) {
                    // out of order measurement detected
                    outOfOrderMeasurementListener?.onOutOfOrderMeasurement(
                        this,
                        SensorType.from(magnetometerSensorType),
                        magnetometerMeasurement
                    )

                    if (stopWhenOutOfOrder) {
                        stopped = true
                        break
                    }
                }

                if ((previousAccelerometerTimestamp == null && magnetometerTimestamp < accelerometerTimestamp)
                    || (previousAccelerometerTimestamp != null && magnetometerTimestamp >= previousAccelerometerTimestamp && magnetometerTimestamp < accelerometerTimestamp)
                ) {
                    // generate synchronized measurement
                    syncedMeasurement.timestamp = magnetometerTimestamp
                    syncedMeasurement.accelerometerMeasurement = accelerometerMeasurement
                    syncedMeasurement.magnetometerMeasurement = magnetometerMeasurement

                    alreadyProcessedMagnetometerMeasurements.add(magnetometerMeasurement)
                    numberOfProcessedMeasurements++

                    syncedMeasurementListener?.onSyncedMeasurements(
                        this@AccelerometerAndMagnetometerSensorMeasurementSyncer,
                        syncedMeasurement
                    )
                } else if (magnetometerTimestamp > accelerometerTimestamp) {
                    // no need to keep processing gyroscope measurements until next accelerometer
                    // measurement is processed
                    break
                }
            }

            if (alreadyProcessedMagnetometerMeasurements.size > 0) {
                alreadyProcessedAccelerometerMeasurements.add(accelerometerMeasurement)

                // return processed magnetometer measurements to the cache of available ones
                magnetometerMeasurements.removeAll(alreadyProcessedMagnetometerMeasurements)
                availableMagnetometerMeasurements.addAll(alreadyProcessedMagnetometerMeasurements)
            }

            previousAccelerometerMeasurement = accelerometerMeasurement

            if (stopped) {
                break
            }
        }

        if (alreadyProcessedAccelerometerMeasurements.size > 0) {
            // return accelerometer measurements to the cache of available ones
            accelerometerMeasurements.removeAll(alreadyProcessedAccelerometerMeasurements)
            availableAccelerometerMeasurements.addAll(alreadyProcessedAccelerometerMeasurements)
        }

        if (initialNumberOfProcessedMeasurements < numberOfProcessedMeasurements) {
            // only resize if synced measurements have been processed
            resizeAvailableMeasurementsIfNeeded()
        }

        if (stopped) {
            stop()
        }
    }

    /**
     * Resizes caches of available measurements if for any reason they increase beyond collectors
     * capacities.
     */
    private fun resizeAvailableMeasurementsIfNeeded() {
        // if for any reason, new measurements had to be created in the cache, remove them now
        if (availableAccelerometerMeasurements.size > accelerometerCapacity) {
            val diff = availableAccelerometerMeasurements.size - accelerometerCapacity
            if (diff > 0) {
                for (i in 0 until diff) {
                    availableAccelerometerMeasurements.removeFirst()
                }
            }
        }
        if (availableMagnetometerMeasurements.size > magnetometerCapacity) {
            val diff = availableMagnetometerMeasurements.size - magnetometerCapacity
            if (diff > 0) {
                for (i in 0 until diff) {
                    availableMagnetometerMeasurements.removeFirst()
                }
            }
        }
    }

    /**
     * Initializes collections of measurements available to be reused.
     * A fixed number of [accelerometerCapacity] and [magnetometerCapacity] measurements are
     * instantiated once, and then are reused.
     */
    private fun initializeMeasurements() {
        availableAccelerometerMeasurements.clear()
        for (i in 1..accelerometerCapacity) {
            availableAccelerometerMeasurements.add(AccelerometerSensorMeasurement())
        }

        availableMagnetometerMeasurements.clear()
        for (i in 1..magnetometerCapacity) {
            availableMagnetometerMeasurements.add(MagnetometerSensorMeasurement())
        }

        accelerometerMeasurements.clear()
        magnetometerMeasurements.clear()
    }

    /**
     * Initializes this syncer.
     */
    init {
        // check that capacities are larger than zero.
        require(accelerometerCapacity > 0)
        require(magnetometerCapacity > 0)

        // initializes collections of measurements to be reused.
        initializeMeasurements()
    }
}