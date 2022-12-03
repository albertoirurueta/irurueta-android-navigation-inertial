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
package com.irurueta.android.navigation.inertial.collectors

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import kotlin.math.pow
import kotlin.math.sqrt

/**
 * Manages and collects gravity sensor measurements.
 * This collector does not have an internal buffer, and consequently out of order measurements can
 * be notified.
 *
 * @property context Android context.
 * @property sensorDelay Delay of sensor between samples.
 * @property measurementListener listener to notify new gravity measurements.
 * @property accuracyChangedListener listener to notify changes in gravity sensor accuracy.
 */
class GravitySensorCollector(
    context: Context,
    sensorDelay: SensorDelay = SensorDelay.FASTEST,
    var measurementListener: OnMeasurementListener? = null,
    accuracyChangedListener: OnAccuracyChangedListener? = null
) : SensorCollector(context, sensorDelay, accuracyChangedListener) {

    /**
     * Internal listener to handle sensor events.
     */
    override val sensorEventListener = object : SensorEventListener {
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
    override val sensor: Sensor? by lazy { sensorManager?.getDefaultSensor(Sensor.TYPE_GRAVITY) }

    /**
     * Interface to notify when a new gravity measurement is available.
     */
    fun interface OnMeasurementListener {

        /**
         * Called when a new gravity measurement is available.
         *
         * @param gx gravity acceleration on device x-axis expressed in meters per squared second
         * (m/s^2) and in ENU coordinates system.
         * @param gy gravity acceleration on device y-axis expressed in meters per squared second
         * (m/s^2) and in ENU coordinates system.
         * @param gz gravity acceleration on device z-axis expressed in meters per squared second
         * (m/s^2) and in ENU coordinates system.
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
}