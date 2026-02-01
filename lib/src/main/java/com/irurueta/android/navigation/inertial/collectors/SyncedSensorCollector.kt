/*
 * Copyright (C) 2026 Alberto Irurueta Carro (alberto@irurueta.com)
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
import android.os.SystemClock.elapsedRealtimeNanos
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.SyncedSensorMeasurement
import java.util.LinkedList
import java.util.Queue

/**
 * Base class for synced sensor collectors.
 * This collectors temporarily buffers measurements from multiple sensors in order to perform
 * synchronization.
 * Synchronized measurement values are by default interpolated
 *
 * @property context Android context.
 * @property windowNanoseconds amount of time to keep measurements buffered expressed in nanoseconds.
 * @property interpolationEnabled indicates whether measurements interpolation is enabled or not.
 * @property measurementListener listener to notify new measurements.
 *
 * @param M type of synchronized measurement.
 * @param C type of synchronized collector.
 */
abstract class SyncedSensorCollector<M : SyncedSensorMeasurement<M>, C : SyncedSensorCollector<M, C>>(
    val context: Context,
    val windowNanoseconds: Long = DEFAULT_WINDOW_NANOSECONDS,
    val interpolationEnabled: Boolean = true,
    var measurementListener: OnMeasurementListener<M, C>? = null
) {
    /**
     * Buffer of measurements being kept for a specified window of time to allow measurement synchronization
     */
    protected val buffer: MutableMap<Int, Queue<SensorMeasurement<*>>> = HashMap()

    /**
     * Internal listener to handle sensor events.
     */
    private val sensorEventListener = object : SensorEventListener {

        override fun onSensorChanged(event: SensorEvent?) {
            if (event == null) {
                return
            }

            val sensorType = event.sensor.type
            val sensorMeasurement = createSensorMeasurement(event)
            // is no sensor measurement is created, it means that event is unsupported and we
            // should stop here
            val timestamp = sensorMeasurement?.timestamp ?: return

            // only store measurements from allowed sensor types
            val buffer = buffer
            buffer.putIfAbsent(sensorType, LinkedList())
            val queue = buffer[sensorType]
            queue?.add(sensorMeasurement)

            // keep buffers short (e.g. 200ms of data)
            trimBuffer(timestamp)

            // Try to synchronize when primary sensor sample arrives (e.g. Gyroscope)
            if (sensorType == primarySensorType && processSyncedSample(timestamp)) {
                @Suppress("UNCHECKED_CAST")
                measurementListener?.onMeasurement(this@SyncedSensorCollector as C, measurement)

            }
        }

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
     * Indicates the sensor type of the primary sensor being used for measurements synchronization.
     */
    protected abstract val primarySensorType: Int

    /**
     * Instance of measurement being reused and notified after conversion of sensor events.
     */
    protected abstract val measurement: M

    /**
     * Sensors being used to obtain measurements or null if not available.
     * This can be used to obtain additional information about the sensors.
     * @see sensorsAvailable
     */
    abstract val sensors: Collection<Sensor>?

    /**
     * Indicates whether requested sensors are available or not.
     */
    abstract val sensorsAvailable: Boolean

    /**
     * Timestamp when collector started expressed as a monotonically increasing timestamp in
     * nanoseconds as indicated by [SystemClock.elapsedRealtimeNanos].
     */
    var startTimestamp = 0L
        private set

    /**
     * Indicates whether collector is running and collecting measurements.
     */
    var running = false
        private set

    /**
     * Gets number of processed measurements since this collector started.
     */
    var numberOfProcessedMeasurements: Long = 0
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
     * @return true if sensor is available and was successfully enabled, false if sensor could not
     * be started or is already running.
     */
    fun start(startTimestamp: Long = elapsedRealtimeNanos()): Boolean {
        if (running) return false

        val sensors = this.sensors ?: return false
        this.startTimestamp = startTimestamp
        var r = true
        for (sensor in sensors) {
            val sensorRunning = sensorManager?.registerListener(
                sensorEventListener,
                sensor,
                getSensorDelayFor(sensor)
            ) ?: false
            r = r && sensorRunning
        }
        running = r
        return running
    }

    /**
     * Stops collecting sensor measurements.
     */
    fun stop() {
        val sensors = this.sensors ?: return
        for (sensor in sensors) {
            sensorManager?.unregisterListener(sensorEventListener, sensor)
        }

        numberOfProcessedMeasurements = 0L
        mostRecentTimestamp = 0L
        running = false
    }

    /**
     * Gets sensor delay for provided sensor.
     *
     * @param sensor sensor to get delay for.
     * @return sensor delay as an integer value.
     */
    protected abstract fun getSensorDelayFor(sensor: Sensor): Int

    /**
     * Creates a new instance of [SensorMeasurement] from provided [SensorEvent].
     *
     * @param event event to convert from.
     * @return new instance of [SensorMeasurement] or null if conversion failed.
     */
    protected abstract fun createSensorMeasurement(event: SensorEvent?): SensorMeasurement<*>?

    /**
     * Performs interpolation of buffered measurements at a given timestamp when measurement from
     * primary sensor arrives.
     *
     * @param nowNanoSeconds timestamp expressed in nanoseconds from last received measurement in
     * primary sensor.
     */
    protected abstract fun processSyncedSample(nowNanoSeconds: Long): Boolean

    /**
     * Processes accuracy changed event for proper notification.
     *
     * @param sensor sensor whose accuracy has changed.
     * @param accuracy new accuracy.
     */
    protected abstract fun processAccuracyChanged(sensor: Sensor?, accuracy: Int)

    /**
     * Removes measurements older than configured window from buffer.
     *
     * @param nowNanoSeconds timestamp expressed in nanoseconds from last received measurement.
     */
    private fun trimBuffer(nowNanoSeconds: Long) {
        buffer.forEach { (_, queue) ->
            // trim each queue in the buffer
            while (!queue.isEmpty() && queue.peek().timestamp < nowNanoSeconds - windowNanoseconds) {
                queue.poll()
            }
        }
    }

    companion object {
        /**
         * Default amount of time to keep measurements buffered expressed in nanoseconds. By default
         * this is 200 ms.
         */
        const val DEFAULT_WINDOW_NANOSECONDS = 200_000_000L
    }

    /**
     * Interface to notify when a new measurement is available.
     * Notice that notified measurement is reused for memory efficiency.
     *
     * @param M type of measurement.
     * @param C type of collector.
     */
    fun interface OnMeasurementListener<M : SyncedSensorMeasurement<M>, C : SyncedSensorCollector<M, C>> {

        /**
         * Called when a new measurement is available.
         *
         * @param collector collector that raised this event.
         * @param measurement a measurement. Notice that this instance might be reused between
         * consecutive calls.
         */
        fun onMeasurement(
            collector: C,
            measurement: M
        )
    }
}