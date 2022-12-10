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
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.SystemClock
import java.util.*
import kotlin.math.min

/**
 * Base class for buffered sensor collectors.
 * A buffered collector allows proper synchronization of events from multiple collectors
 * by colling [getMeasurementsBeforeTimestamp] or [getMeasurementsBeforePosition] periodically to
 * obtain measurements in the buffer prior to a certain timestamp or buffer position.
 *
 * @property context Android context.
 * @property sensorDelay Delay of sensor between samples.
 * @property capacity capacity of buffer.
 * @property startOffsetEnabled indicates whether [startOffset] will be computed when first
 * measurement is received or not. True indicates that offset is computed, false assumes that offset
 * is null.
 * @property stopWhenFilledBuffer true to stop collector when buffer completely fills, false to
 * continue collection at the expense of loosing old data. This will be notified using
 * [bufferFilledListener].
 * @property accuracyChangedListener listener to notify changes in accuracy.
 * @property bufferFilledListener listener to notify that buffer has been filled. This usually
 * happens when consumer of measurements cannot keep up with the rate at which measurements are
 * generated.
 * @property measurementListener listener to notify new measurements. It must be noticed that
 * measurements notification might be delayed and measurements might arrive out of order.
 * @throws IllegalArgumentException if provided capacity is zero or negative.
 * @param M type of measurement.
 * @param C type of collector.
 */
abstract class BufferedSensorCollector<M : SensorMeasurement<M>, C : BufferedSensorCollector<M, C>>(
    val context: Context,
    val sensorDelay: SensorDelay,
    val capacity: Int,
    val startOffsetEnabled: Boolean,
    val stopWhenFilledBuffer: Boolean,
    var accuracyChangedListener: OnAccuracyChangedListener<M, C>?,
    var bufferFilledListener: OnBufferFilledListener<M, C>?,
    var measurementListener: OnMeasurementListener<M, C>?
) {
    /**
     * Internal listener to handle sensor events.
     */
    private val sensorEventListener = object : SensorEventListener {
        /**
         * Called when new measurement is received.
         *
         * @param event contains measurement data.
         */
        override fun onSensorChanged(event: SensorEvent?) {
            if (event == null) {
                return
            }

            // when first measurement arrives, compute offset (if enabled). This can be used to
            // synchronize multiple sensors.
            val eventTimestamp = event.timestamp
            if (startOffsetEnabled && startOffset == null) {
                startOffset = eventTimestamp - startTimestamp
            }

            // Pick available measurement, update its values and put it into buffer
            val measurement: M
            val bufferPosition: Int
            synchronized(this@BufferedSensorCollector) {
                // pick available measurement and add it into buffer
                measurement = if (availableMeasurements.isEmpty()) {
                    // notify buffer is full and no more measurements are available
                    @Suppress("UNCHECKED_CAST")
                    bufferFilledListener?.onBufferFilled(this@BufferedSensorCollector as C)

                    if (stopWhenFilledBuffer) {
                        // stop collector
                        stop()
                        return
                    }

                    // happens when buffer is filled, no more available measurements are available
                    // and stopWhenFilledBuffer is disabled
                    buffer.removeFirst()
                } else {
                    availableMeasurements.removeFirst()
                }
                if (!updateMeasurementWithSensorEvent(measurement, event)) {
                    // if measurement conversion fails, return it to available measurements collection
                    availableMeasurements.addFirst(measurement)
                    return
                }
                buffer.add(measurement)

                bufferPosition = buffer.size - 1
            }

            mostRecentTimestamp = measurement.timestamp
            numberOfProcessedMeasurements++

            // notify measurement
            val measurementListener = this@BufferedSensorCollector.measurementListener
            if (measurementListener != null) {
                // copy measurement into reusable instance to prevent changes introduced
                // from listener call
                this@BufferedSensorCollector.measurement.copyFrom(measurement)
                @Suppress("UNCHECKED_CAST")
                measurementListener.onMeasurement(
                    this@BufferedSensorCollector as C,
                    measurement,
                    bufferPosition
                )
            }
        }

        /**
         * Called when sensor accuracy changes.
         *
         * @param sensor sensor whose accuracy has changed.
         * @param accuracy new accuracy.
         */
        override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {
            processAccuracyChanged(sensor, accuracy)
        }
    }

    /**
     * Buffer containing ordered measurements by ascending timestamp.
     */
    private val buffer = ArrayDeque<M>(capacity)

    /**
     * Queue of measurements that ar no longer buffered and ar available to be reused when a new
     * measurement arrives.
     */
    private val availableMeasurements = ArrayDeque<M>(capacity)

    /**
     * Internal list being reused containing retrieved measurements using
     * [getMeasurementsBeforeTimestamp].
     * This is used for efficiency purposes.
     */
    private val measurementsBeforeTimestamp = ArrayDeque<M>(capacity)

    /**
     * Internal list being reused containing retrieved measurements using
     * [getMeasurementsBeforePosition].
     * This is used for efficiency purposes.
     */
    private val measurementsBeforePosition = ArrayDeque<M>(capacity)

    /**
     * Internal list containing instance of [measurementsBeforeTimestamp] that can be reused for
     * performance reasons. Instances in this list are reused every time that
     * [getMeasurementsBeforeTimestamp] is called.
     */
    private val availableMeasurementsBeforeTimestamp = ArrayList<M>(capacity)

    /**
     * Internal list containing instance of [measurementsBeforePosition] that can be reused for
     * performance reasons. Instances in this list are reused every time that
     * [getMeasurementsBeforePosition] is called.
     */
    private val availableMeasurementsBeforePosition = ArrayList<M>(capacity)

    /**
     * System sensor manager.
     */
    protected val sensorManager: SensorManager? by lazy {
        context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
    }

    /**
     * Measurement being reused internally to copy data and avoid exposing
     * internal instances through listeners for security purposes.
     */
    protected abstract val measurement: M

    /**
     * Sensor being used to obtain measurements or null if not available.
     * This can be used to obtain additional information about the sensor.
     * @see sensorAvailable
     */
    abstract val sensor: Sensor?

    /**
     * Indicates whether requested sensor is available or not.
     */
    open val sensorAvailable: Boolean by lazy {
        sensor != null
    }

    /**
     * Timestamp when collector started expressed as a monotonically increasing timestamp in
     * nanoseconds as indicated by [SystemClock.elapsedRealtimeNanos].
     */
    var startTimestamp = 0L
        private set

    /**
     * Initial offset expressed in nano seconds between first received measurement timestamp and
     * start time expressed in the monotonically increasing system clock obtained by
     * [SystemClock.elapsedRealtimeNanos].
     */
    var startOffset: Long? = null
        protected set

    /**
     * Gets a copy of the buffer containing ordered measurements by ascending timestamp.
     *
     * For performance reasons, because this method is synchronized and makes object copies, it
     * should only be used sporadically or for debugging purposes.
     */
    val bufferedMeasurements: List<M>
        @Synchronized
        get() = buffer.map { measurement -> measurement.copy() }.toList()

    /**
     * Gets current usage of this collector as a value between 0.0 and 1.0.
     * 0.0 indicates that buffer is empty and no measurement has yet been received.
     * 1.0 indicates that buffer is full and [getMeasurementsBeforeTimestamp] or
     * [getMeasurementsBeforePosition] must be called to release some measurements from buffer or
     * otherwise collector will stop or old measurements will be lost.
     */
    val usage: Float
        get() = buffer.size.toFloat() / capacity.toFloat()

    /**
     * Indicates whether collector is running and collecting measurements.
     */
    var running = false
        private set

    /**
     * Gets number of processed measurements since this collector started.
     */
    var numberOfProcessedMeasurements: Int = 0
        private set

    /**
     * Gets most recent timestamp of all measurements processed so far.
     * Timestamp is expressed in nanoseconds by a monotonic clock based on
     * [SystemClock.elapsedRealtimeNanos].
     */
    var mostRecentTimestamp: Long = 0L
        private set

    /**
     * Gets oldest timestamp within buffered measurements, if any is available.
     * Timestamp is expressed in nanoseconds by a monotonic clock based on
     * [SystemClock.elapsedRealtimeNanos].
     */
    val oldestTimestampInBuffer: Long?
        get() = buffer.firstOrNull()?.timestamp

    /**
     * Method to be called periodically to remove from buffer measurements matching requested
     * timestamp.
     * Failing to call this method or [getMeasurementsBeforePosition] with required frequency,
     * will result in a full buffer, which will be notified and stop this sensor collector or
     * lead to loss of oldest measurements.
     * Notice that returned list and measurements will be reused for memory efficiency purposes.
     *
     * @param timestamp timestamp to limit measurements to be retrieved from buffer.
     */
    @Synchronized
    fun getMeasurementsBeforeTimestamp(timestamp: Long): Deque<M> {
        measurementsBeforeTimestamp.clear()
        do {
            val measurement = buffer.peek()
            if (measurement != null && measurement.timestamp <= timestamp) {
                // remove from buffer and add to available measurements to be reused
                buffer.poll()
                availableMeasurements.add(measurement)

                // add copy to result of this method
                val pos = measurementsBeforeTimestamp.size
                val measurementCopy = availableMeasurementsBeforeTimestamp[pos]
                measurementCopy.copyFrom(measurement)
                measurementsBeforeTimestamp.add(measurementCopy)
            } else {
                // buffer is sorted and there are no mor measurements to be retrieved
                break
            }
        } while (buffer.isNotEmpty())

        return measurementsBeforeTimestamp
    }

    /**
     * Method to be called periodically to remove from buffer measurements matching requested
     * buffer position.
     * Failing to call this method or [getMeasurementsBeforeTimestamp] with required frequency,
     * will result in a full buffer, which will be notified and stop this sensor collector or
     * lead to loss of oldest measurements.
     * Notice that returned list and measurements will be reused for memory efficiency purposes.
     *
     * @param position position to limit measurements to be retrieved from buffer
     */
    @Synchronized
    fun getMeasurementsBeforePosition(position: Int): Deque<M> {
        measurementsBeforePosition.clear()
        val size = buffer.size
        for (i in 0 until min(size, position + 1)) {
            // remove from buffer and add to available measurements to be reused
            val measurement = buffer.removeFirst()
            availableMeasurements.add(measurement)

            // add copy to result of this method
            val pos = measurementsBeforePosition.size
            val measurementCopy = availableMeasurementsBeforePosition[pos]
            measurementCopy.copyFrom(measurement)
            measurementsBeforePosition.add(measurementCopy)
        }

        return measurementsBeforePosition
    }

    /**
     * Starts collecting sensor measurements.
     *
     * @param startTimestamp monotonically increasing timestamp when collector starts. If not
     * provided, system clock is used by default, otherwise, the value can be provided to sync
     * multiple sensor collector instances.
     * @return true if sensor is available and was successfully enabled.
     * @throws IllegalStateException if collector is already running.
     */
    @Throws(IllegalStateException::class)
    fun start(startTimestamp: Long = SystemClock.elapsedRealtimeNanos()): Boolean {
        check(!running)

        val sensor = this.sensor ?: return false
        this.startTimestamp = startTimestamp
        running = sensorManager?.registerListener(sensorEventListener, sensor, sensorDelay.value)
            ?: false
        return running
    }

    /**
     * Stops collecting sensor measurements.
     */
    fun stop() {
        sensorManager?.unregisterListener(sensorEventListener, sensor)

        // clear buffer and initialize available measurements for next execution
        buffer.clear()
        initializeMeasurements()
        startOffset = null

        numberOfProcessedMeasurements = 0
        mostRecentTimestamp = 0L
        running = false
    }

    /**
     * Updates measurement values with provided [SensorEvent].
     *
     * @param measurement measurement to be updated.
     * @param event event containing data to update measurement.
     * @return true if measurement was successfully updated, false otherwise.
     */
    protected abstract fun updateMeasurementWithSensorEvent(
        measurement: M,
        event: SensorEvent?
    ): Boolean

    /**
     * Processes accuracy changed event for proper notification.
     *
     * @param sensor sensor whose accuracy has changed.
     * @param accuracy new accuracy.
     */
    protected abstract fun processAccuracyChanged(sensor: Sensor?, accuracy: Int)

    /**
     * Creates a new instance of a measurement of type [M].
     */
    protected abstract fun createEmptyMeasurement(): M

    /**
     * Initializes collections of measurements available to be reused.
     * A fixed number of [capacity] measurements are instantiated once, and then
     * are reused, being placed into an internal [buffer] or returned to [availableMeasurements]
     * collection when they are freed from the buffer.
     */
    private fun initializeMeasurements() {
        // fill available measurements up to provided capacity
        availableMeasurements.clear()
        availableMeasurementsBeforeTimestamp.clear()
        availableMeasurementsBeforePosition.clear()
        for (i in 1..capacity) {
            availableMeasurements.add(createEmptyMeasurement())
            availableMeasurementsBeforeTimestamp.add(createEmptyMeasurement())
            availableMeasurementsBeforePosition.add(createEmptyMeasurement())
        }
    }

    /**
     * Initializes this collector.
     */
    init {
        // check that capacity is larger than zero
        require(capacity > 0)

        // initializes collections of measurements to be reused
        initializeMeasurements()
    }

    companion object {
        /**
         * Default initial capacity of buffer.
         */
        const val DEFAULT_CAPACITY = 100
    }

    /**
     * Interface to notify when sensor accuracy changes.
     *
     * @param M type of measurement.
     * @param C type of collector.
     */
    fun interface OnAccuracyChangedListener<M : SensorMeasurement<M>, C : BufferedSensorCollector<M, C>> {
        /**
         * Called when accuracy changes.
         *
         * @param collector collector that raised this event.
         * @param accuracy new accuracy.
         */
        fun onAccuracyChanged(collector: C, accuracy: SensorAccuracy?)
    }

    /**
     * Interface to notify when buffer gets completely filled.
     * When buffer completely fills, collector will stop if [stopWhenFilledBuffer] is true.
     * If [stopWhenFilledBuffer] is false, collector will continue collection at the expense of
     * loosing old data. Consumers of this listener should decide what to do at this point.
     *
     * @param M type of measurement.
     * @param C type of collector.
     */
    fun interface OnBufferFilledListener<M : SensorMeasurement<M>, C : BufferedSensorCollector<M, C>> {
        /**
         * Called when buffer gets completely filled.
         *
         * @param collector collector that raised this event.
         */
        fun onBufferFilled(collector: C)
    }

    /**
     * Interface to notify when a new measurement is available.
     * Notice that notified measurements from multiple collectors of different sensors, might be
     * out of order.
     * Calling [getMeasurementsBeforeTimestamp] or [getMeasurementsBeforePosition] periodically will
     * help to synchronize measurements among collectors.
     * Notice that notified measurement is reused for memory efficiency and security purposes.
     *
     * @param M type of measurement.
     * @param C type of collector.
     */
    fun interface OnMeasurementListener<M : SensorMeasurement<M>, C : BufferedSensorCollector<M, C>> {
        /**
         * Called when a new sensor measurement is available.
         *
         * @param collector collector that raised this event.
         * @param measurement a measurement.
         * @param bufferPosition position where this measurement is located within the buffer.
         */
        fun onMeasurement(collector: C, measurement: M, bufferPosition: Int)
    }
}