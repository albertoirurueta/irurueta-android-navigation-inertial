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
import android.hardware.SensorEventListener
import android.hardware.SensorManager

/**
 * Base class for sensor collectors.
 * This collector does not have an internal buffer, and consequently out of order measurements can
 * be notified.
 *
 * @property context Android context.
 * @property sensorDelay Delay of sensor between samples.
 * @property accuracyChangedListener listener to notify changes in accuracy.
 */
abstract class SensorCollector(
    val context: Context,
    val sensorDelay: SensorDelay = SensorDelay.FASTEST,
    var accuracyChangedListener: OnAccuracyChangedListener? = null
) {
    /**
     * System sensor manager.
     */
    protected val sensorManager: SensorManager? by lazy {
        context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
    }

    /**
     * Internal listener to handle sensor events.
     */
    protected abstract val sensorEventListener: SensorEventListener

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
     * Starts collecting sensor measurements.
     *
     * @return true if sensor is available and was successfully enabled.
     */
    fun start(): Boolean {
        val sensor = this.sensor ?: return false
        return sensorManager?.registerListener(sensorEventListener, sensor, sensorDelay.value)
            ?: false
    }

    /**
     * Stops collecting sensor measurements.
     */
    fun stop() {
        sensorManager?.unregisterListener(sensorEventListener, sensor)
    }

    /**
     * Interface to notify when sensor accuracy changes.
     */
    fun interface OnAccuracyChangedListener {
        /**
         * Called when accuracy changes.
         *
         * @param accuracy new accuracy.
         */
        fun onAccuracyChanged(accuracy: SensorAccuracy?)
    }
}