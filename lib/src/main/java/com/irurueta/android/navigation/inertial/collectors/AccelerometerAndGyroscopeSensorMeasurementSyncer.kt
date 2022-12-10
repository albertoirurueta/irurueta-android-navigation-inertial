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
 * Typically when synchronization is needed is for correct pose estimation, in which first
 * attitude is estimated (which starts by doing some sort of leveling or gravity estimation) plus
 * changes based on gyroscope data, and once attitude is estimated, then accelerometer data
 * (discounting gravity components based on current attitude) is taken into account to modify
 * speed and position.
 * Consequently, this syncer will assume accelerometer sensor as the main sensor that needs to reach
 * a certain buffer level before any syncing processing can occur.
 */
class AccelerometerAndGyroscopeSensorMeasurementSyncer(
    context: Context,
    val accelerometerSensorType: AccelerometerSensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
    val gyroscopeSensorType: GyroscopeSensorType = GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
    val accelerometerSensorDelay: SensorDelay = SensorDelay.FASTEST,
    val gyroscopeSensorDelay: SensorDelay = SensorDelay.FASTEST,
    val accelerometerCapacity: Int = BufferedSensorCollector.DEFAULT_CAPACITY,
    val gyroscopeCapacity: Int = BufferedSensorCollector.DEFAULT_CAPACITY,
    val accelerometerStartOffsetEnabled: Boolean = true,
    val gyroscopeStartOffsetEnabled: Boolean = true,
    stopWhenFilledBuffer: Boolean = true,
    outOfOrderDetectionEnabled: Boolean = false,
    stopWhenOutOfOrder: Boolean = true,
    accuracyChangedListener: OnAccuracyChangedListener<AccelerometerAndGyroscopeSyncedSensorMeasurement, AccelerometerAndGyroscopeSensorMeasurementSyncer>? = null,
    bufferFilledListener: OnBufferFilledListener<AccelerometerAndGyroscopeSyncedSensorMeasurement, AccelerometerAndGyroscopeSensorMeasurementSyncer>? = null,
    outOfOrderMeasurementListener: OnOutOfOrderMeasurementListener<AccelerometerAndGyroscopeSyncedSensorMeasurement, AccelerometerAndGyroscopeSensorMeasurementSyncer>? = null,
    syncedMeasurementListener: OnSyncedMeasurementsListener<AccelerometerAndGyroscopeSyncedSensorMeasurement, AccelerometerAndGyroscopeSensorMeasurementSyncer>? = null
) : SensorMeasurementSyncer<AccelerometerAndGyroscopeSyncedSensorMeasurement, AccelerometerAndGyroscopeSensorMeasurementSyncer>(
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
     * Cached gyroscope measurements ready to be used.
     */
    private val availableGyroscopeMeasurements = LinkedList<GyroscopeSensorMeasurement>()

    /**
     * Accelerometer measurements to be processed in next batch.
     */
    private val accelerometerMeasurements = LinkedList<AccelerometerSensorMeasurement>()

    /**
     * Gyroscope measurements to be processed in next batch.
     */
    private val gyroscopeMeasurements = LinkedList<GyroscopeSensorMeasurement>()

    /**
     * List of accelerometer measurements that have already been processed and can be returned back
     * to the cache of available measurements.
     */
    private val alreadyProcessedAccelerometerMeasurements =
        LinkedList<AccelerometerSensorMeasurement>()

    /**
     * List of gyroscope measurements that have already been processed and can be returned back to
     * the cache af available measurements.
     */
    private val alreadyProcessedGyroscopeMeasurements = LinkedList<GyroscopeSensorMeasurement>()

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
                this@AccelerometerAndGyroscopeSensorMeasurementSyncer,
                SensorType.from(accelerometerSensorType),
                accuracy
            )
        },
        bufferFilledListener = {
            bufferFilledListener?.onBufferFilled(
                this@AccelerometerAndGyroscopeSensorMeasurementSyncer,
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
                this@AccelerometerAndGyroscopeSensorMeasurementSyncer,
                SensorType.from(gyroscopeSensorType),
                accuracy
            )
        },
        bufferFilledListener = {
            bufferFilledListener?.onBufferFilled(
                this@AccelerometerAndGyroscopeSensorMeasurementSyncer,
                SensorType.from(gyroscopeSensorType)
            )
            if (stopWhenFilledBuffer) {
                stop()
            }
        },
        measurementListener = { collector, _, _ ->
            val mostRecentTimestamp =
                this@AccelerometerAndGyroscopeSensorMeasurementSyncer.mostRecentTimestamp
            if (mostRecentTimestamp != null) {
                val measurementsBeforeTimestamp =
                    collector.getMeasurementsBeforeTimestamp(mostRecentTimestamp)
                if (measurementsBeforeTimestamp.isNotEmpty()) {
                    // copy measurements
                    copyToGyroscopeMeasurements(measurementsBeforeTimestamp)
                    processMeasurements()
                }
            }
        }
    )

    /**
     * Synced measurement to be reused for efficiency purposes.
     */
    override val syncedMeasurement = AccelerometerAndGyroscopeSyncedSensorMeasurement()

    /**
     * Gets accelerometer sensor being used to obtain measurements or null if not available.
     * This can be used to obtain additional information about the sensor.
     * @see accelerometerSensorAvailable
     */
    val accelerometerSensor: Sensor?
        get() = accelerometerSensorCollector.sensor

    /**
     * Gets gyroscope sensor being used to obtain measurements or null if not available.
     * This can b used to obtain additional information about the sensor.
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
    val accelerometerUsage: Float
        get() = accelerometerSensorCollector.usage

    /**
     * Gets gyroscope collector current usage as a value between 0.0 and 1.0.
     * 0.0 indicates that buffer is empty and no measurement has yet been received.
     * 1.0 indicates that buffer is full.
     */
    val gyroscopeUsage: Float
        get() = gyroscopeSensorCollector.usage

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
    override fun stop() {
        accelerometerSensorCollector.stop()
        gyroscopeSensorCollector.stop()
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
    private fun copyToGyroscopeMeasurements(measurements: Collection<GyroscopeSensorMeasurement>) {
        for (measurement in measurements) {
            // move an available measurement to the list of measurements
            // if for any reason there are no more available cached measurements, build new ones
            val availableMeasurement =
                availableGyroscopeMeasurements.removeFirstOrNull() ?: GyroscopeSensorMeasurement()
            availableMeasurement.copyFrom(measurement)
            gyroscopeMeasurements.add(availableMeasurement)
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

            alreadyProcessedGyroscopeMeasurements.clear()
            for (gyroscopeMeasurement in gyroscopeMeasurements) {
                val gyroscopeTimestamp = gyroscopeMeasurement.timestamp
                if (outOfOrderDetectionEnabled && oldestTimestamp != null && gyroscopeTimestamp < oldestTimestamp) {
                    // out of order measurement detected
                    outOfOrderMeasurementListener?.onOutOfOrderMeasurement(
                        this,
                        SensorType.from(gyroscopeSensorType),
                        gyroscopeMeasurement
                    )

                    if (stopWhenOutOfOrder) {
                        stopped = true
                        break
                    }
                }

                if ((previousAccelerometerTimestamp == null && gyroscopeTimestamp < accelerometerTimestamp)
                    || (previousAccelerometerTimestamp != null && gyroscopeTimestamp >= previousAccelerometerTimestamp && gyroscopeTimestamp < accelerometerTimestamp)
                ) {
                    // generate synchronized measurement
                    syncedMeasurement.timestamp = gyroscopeTimestamp
                    syncedMeasurement.accelerometerMeasurement = accelerometerMeasurement
                    syncedMeasurement.gyroscopeMeasurement = gyroscopeMeasurement

                    alreadyProcessedGyroscopeMeasurements.add(gyroscopeMeasurement)
                    numberOfProcessedMeasurements++

                    syncedMeasurementListener?.onSyncedMeasurements(
                        this@AccelerometerAndGyroscopeSensorMeasurementSyncer,
                        syncedMeasurement
                    )

                } else if (gyroscopeTimestamp > accelerometerTimestamp) {
                    // no need to keep processing gyroscope measurements until next accelerometer
                    // measurement is processed
                    break
                }
            }

            if (alreadyProcessedGyroscopeMeasurements.size > 0) {
                alreadyProcessedAccelerometerMeasurements.add(accelerometerMeasurement)

                // return processed gyroscope measurements to the cache of available ones
                gyroscopeMeasurements.removeAll(alreadyProcessedGyroscopeMeasurements)
                availableGyroscopeMeasurements.addAll(alreadyProcessedGyroscopeMeasurements)
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
        if (availableGyroscopeMeasurements.size > gyroscopeCapacity) {
            val diff = availableGyroscopeMeasurements.size - gyroscopeCapacity
            if (diff > 0) {
                for (i in 0 until diff) {
                    availableGyroscopeMeasurements.removeFirst()
                }
            }
        }
    }

    /**
     * Initializes collections of measurements available to be reused.
     * A fixed number of [accelerometerCapacity] and [gyroscopeCapacity] measurements are
     * instantiated once, and then are reused.
     */
    private fun initializeMeasurements() {
        availableAccelerometerMeasurements.clear()
        for (i in 1..accelerometerCapacity) {
            availableAccelerometerMeasurements.add(AccelerometerSensorMeasurement())
        }

        availableGyroscopeMeasurements.clear()
        for (i in 1..gyroscopeCapacity) {
            availableGyroscopeMeasurements.add(GyroscopeSensorMeasurement())
        }

        accelerometerMeasurements.clear()
        gyroscopeMeasurements.clear()
    }

    /**
     * Initializes thi syncer.
     */
    init {
        // check that capacities are larger than zero.
        require(accelerometerCapacity > 0)
        require(gyroscopeCapacity > 0)

        // initializes collections of measurements to be reused.
        initializeMeasurements()
    }
}