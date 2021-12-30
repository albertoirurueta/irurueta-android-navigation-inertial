/*
 * Copyright (C) 2021 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.android.navigation.inertial

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * Manages and collects gravity sensor measurements.
 *
 * @property context Android context.
 * @property sensorDelay Delay of sensor between samples.
 * @property measurementListener listener to notify new gravity measurements.
 * @property accuracyChangedListener listener to notify changes in gravity sensor accuracy.
 */
class GravitySensorCollector(
    val context: Context,
    val sensorDelay: SensorDelay = SensorDelay.FASTEST,
    var measurementListener: OnMeasurementListener? = null,
    var accuracyChangedListener: OnAccuracyChangedListener? = null
) {
    /**
     * System sensor manager.
     */
    private val sensorManager: SensorManager? by lazy {
        context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
    }

    /**
     * Internal listener to handle sensor events.
     */
    private val sensorEventListener = object : SensorEventListener {
        override fun onSensorChanged(event: SensorEvent?) {
            if (event == null) {
                return
            }
            if (event.sensor.type != Sensor.TYPE_GRAVITY) {
                return
            }

            val sensorAccuracy = SensorAccuracy.from(event.accuracy)
            val timestamp = event.timestamp

            val gx = event.values[0]
            val gy = event.values[1]
            val gz = event.values[2]

            val g = sqrt(gx.toDouble().pow(2.0) + gy.toDouble().pow(2.0) + gz.toDouble().pow(2.0))

            measurementListener?.onMeasurement(
                gx,
                gy,
                gz,
                g,
                timestamp,
                sensorAccuracy
            )
        }

        override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {
            if (sensor == null) {
                return
            }
            if (sensor.type != Sensor.TYPE_GRAVITY) {
                return
            }

            val sensorAccuracy = SensorAccuracy.from(accuracy)
            accuracyChangedListener?.onAccuracyChanged(sensorAccuracy)
        }

    }

    /**
     * Sensor being used to obtain measurements or null if not available.
     * This can be used to obtain additional information about the sensor.
     * @see sensorAvailable
     */
    val sensor: Sensor? by lazy { sensorManager?.getDefaultSensor(Sensor.TYPE_GRAVITY) }

    /**
     * Indicates whether requested sensor is available or not.
     */
    val sensorAvailable: Boolean by lazy {
        val availabilityService = SensorAvailabilityService(context)
        availabilityService.hasSensor(SensorAvailabilityService.SensorType.GRAVITY)
    }

    /**
     * Starts collecting gravity measurements.
     *
     * @return true if sensor is available and was successfully enabled.
     */
    fun start(): Boolean {
        val sensor = this.sensor ?: return false
        return sensorManager?.registerListener(sensorEventListener, sensor, sensorDelay.value)
            ?: false
    }

    /**
     * Stops collecting gravity measurements.
     */
    fun stop() {
        sensorManager?.unregisterListener(sensorEventListener, sensor)
    }

    /**
     * Interface to notify when a new gravity measurement is available.
     */
    interface OnMeasurementListener {

        /**
         * Called when a new gravity measurement is available.
         *
         * @param gx gravity acceleration on device x-axis expressed in meters per squared second
         * (m/s^2).
         * @param gy gravity acceleration on device y-axis expressed in meters per squared second
         * (m/s^2).
         * @param gz gravity acceleration on device z-axis expressed in meters per squared second
         * (m/s^2).
         * @param g magnitude of gravity acceleration expressed in meters per squared second
         * (m/s^2).
         * @param timestamp time in nanoseconds at which the measurement was made. Each measurement
         * will be monotonically increasing using the same time base as
         * [android.os.SystemClock.elapsedRealtimeNanos].
         * @param accuracy gravity sensor accuracy.
         */
        fun onMeasurement(
            gx: Float,
            gy: Float,
            gz: Float,
            g: Double,
            timestamp: Long,
            accuracy: SensorAccuracy?
        )
    }

    /**
     * Interface to notify when gravity sensor accuracy changes.
     */
    interface OnAccuracyChangedListener {
        /**
         * Called when gravity sensor accuracy changes.
         *
         * @param accuracy new gravity sensor accuracy.
         */
        fun onAccuracyChanged(accuracy: SensorAccuracy?)
    }
}