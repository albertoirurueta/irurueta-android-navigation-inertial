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
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.SystemClock
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorMeasurement

/**
 * Base class for sensor collectors.
 * This collector does not have an internal buffer.
 *
 * @property context Android context.
 * @property sensorDelay Delay of sensor between samples.
 * @property startOffsetEnabled indicates whether [startOffset] will be computed when first
 * measurement is received or not. True indicates that offset is computed, false assumes that offset
 * is null.
 * @property accuracyChangedListener listener to notify changes in accuracy.
 * @property measurementListener listener to notify new measurements. It must be noticed that
 * measurements notification might be delayed.
 *
 * @param M type of measurement.
 * @param C type of collector.
 */

abstract class SensorCollector2<M : SensorMeasurement<M>, C : SensorCollector2<M, C>>(
    val context: Context,
    val sensorDelay: SensorDelay,
    val startOffsetEnabled: Boolean,
    var accuracyChangedListener: OnAccuracyChangedListener<M, C>?,
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

            if (updateMeasurementWithSensorEvent(measurement, event)) {

                mostRecentTimestamp = measurement.timestamp
                numberOfProcessedMeasurements++

                @Suppress("UNCHECKED_CAST")
                measurementListener?.onMeasurement(this@SensorCollector2 as C, measurement)
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
     * System sensor manager.
     */
    protected val sensorManager: SensorManager? by lazy {
        context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
    }

    /**
     * Instance of measurement being reused and notified after conversion of sensor events.
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
     * Interface to notify when sensor accuracy changes.
     *
     * @param M type of measurement.
     * @param C type of collector.
     */
    fun interface OnAccuracyChangedListener<M : SensorMeasurement<M>, C : SensorCollector2<M, C>> {
        /**
         * Called when accuracy changes.
         *
         * @param collector collector that raised this event.
         * @param accuracy new accuracy.
         */
        fun onAccuracyChanged(collector: C, accuracy: SensorAccuracy?)
    }

    /**
     * Interface to notify when a new measurement is available.
     * Notice that notified measurements from multiple collectors of different sensors, might be
     * out of order.
     * Notice that notified measurement is reused for memory efficiency.
     *
     * @param M type of measurement.
     * @param C type of collector.
     */
    fun interface OnMeasurementListener<M : SensorMeasurement<M>, C : SensorCollector2<M, C>> {
        /**
         * Called when a new sensor measurement is available.
         *
         * @param collector collector that raised this event.
         * @param measurement a measurement. Notice that this instance is reused between
         * consecutive calls.
         */
        fun onMeasurement(collector: C, measurement: M)
    }

}