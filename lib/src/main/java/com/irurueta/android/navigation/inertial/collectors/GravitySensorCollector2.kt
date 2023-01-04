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
package com.irurueta.android.navigation.inertial.collectors

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent

/**
 * Manages and collects gravity sensor measurements.
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
 */
class GravitySensorCollector2(
    context: Context,
    sensorDelay: SensorDelay = SensorDelay.FASTEST,
    startOffsetEnabled: Boolean = true,
    accuracyChangedListener: OnAccuracyChangedListener<GravitySensorMeasurement, GravitySensorCollector2>? = null,
    measurementListener: OnMeasurementListener<GravitySensorMeasurement, GravitySensorCollector2>? = null
) : SensorCollector2<GravitySensorMeasurement, GravitySensorCollector2>(
    context,
    sensorDelay,
    startOffsetEnabled,
    accuracyChangedListener,
    measurementListener
) {
    /**
     * Instance of measurement being reused and notified after conversion of sensor events.
     */
    override val measurement = GravitySensorMeasurement()

    /**
     * Sensor being used to obtain measurements or null if not available.
     * This can be used to obtain additional information about the sensor.
     * @see sensorAvailable
     */
    override val sensor: Sensor? by lazy { sensorManager?.getDefaultSensor(Sensor.TYPE_GRAVITY) }

    /**
     * Updates measurement values with provided [SensorEvent].
     *
     * @param measurement measurement to be updated.
     * @param event event containing data to update measurement.
     * @return true if measurement was successfully updated, false otherwise.
     */
    override fun updateMeasurementWithSensorEvent(
        measurement: GravitySensorMeasurement,
        event: SensorEvent?
    ): Boolean {
        return GravitySensorMeasurementConverter.convert(event, measurement, startOffset)
    }

    /**
     * Processes accuracy changed event for proper notification.
     *
     * @param sensor sensor whose accuracy has changed.
     * @param accuracy new accuracy.
     */
    override fun processAccuracyChanged(sensor: Sensor?, accuracy: Int) {
        if (sensor == null) {
            return
        }
        if (sensor.type != Sensor.TYPE_GRAVITY) {
            return
        }

        val sensorAccuracy = SensorAccuracy.from(accuracy)
        accuracyChangedListener?.onAccuracyChanged(this, sensorAccuracy)
    }
}