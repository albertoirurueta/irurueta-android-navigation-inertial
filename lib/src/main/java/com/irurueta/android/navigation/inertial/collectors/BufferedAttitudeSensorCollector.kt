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
 * Manages and collects attitude sensor measurements using a buffer.
 * A buffered collector allows proper synchronization of events from multiple collectors
 * by calling [getMeasurementsBeforeTimestamp] or [getMeasurementsBeforePosition] periodically to
 * obtain measurements in th buffer prior to a certain timestamp or buffer position.
 *
 * @property context Android context.
 * @property sensorType One of the supported attitude sensor types.
 * @property sensorDelay Delay of sensor between samples.
 * @property capacity capacity of buffer.
 * @property startOffsetEnabled indicates whether [startOffset] will be computed when first
 * measurement is received or not. Tru indicates that offset is computed, false assumes that offset
 * is null.
 * @property stopWhenFilledBuffer true to stop collector when buffer completely fills, false to
 * continue collection at the expense of loosing old data. This will be notified using
 * [bufferFilledListener].
 * @property skipWhenProcessing true to skip new measurements when processing a measurement.
 * @property accuracyChangedListener listener to notify changes in accuracy.
 * @property bufferFilledListener listener to notify that buffer has been filled. This usually
 * happens when consumer of measurements cannot keep up with the rate at which measurements are
 * generated.
 * @property measurementListener listener to notify new measurements. It must be noticed that
 * measurements notification might be delayed.
 * @throws IllegalArgumentException if provided capacity is zero or negative.
 */
class BufferedAttitudeSensorCollector(
    context: Context,
    val sensorType: AttitudeSensorType = AttitudeSensorType.ABSOLUTE_ATTITUDE,
    sensorDelay: SensorDelay = SensorDelay.FASTEST,
    capacity: Int = DEFAULT_CAPACITY,
    startOffsetEnabled: Boolean = true,
    stopWhenFilledBuffer: Boolean = true,
    skipWhenProcessing: Boolean = true,
    accuracyChangedListener: OnAccuracyChangedListener<AttitudeSensorMeasurement, BufferedAttitudeSensorCollector>? = null,
    bufferFilledListener: OnBufferFilledListener<AttitudeSensorMeasurement, BufferedAttitudeSensorCollector>? = null,
    measurementListener: OnMeasurementListener<AttitudeSensorMeasurement, BufferedAttitudeSensorCollector>? = null
) : BufferedSensorCollector<AttitudeSensorMeasurement, BufferedAttitudeSensorCollector>(
    context,
    sensorDelay,
    capacity,
    startOffsetEnabled,
    stopWhenFilledBuffer,
    skipWhenProcessing,
    accuracyChangedListener,
    bufferFilledListener,
    measurementListener
) {
    /**
     * Sensor being used to obtain measurements or null if not available.
     * This can be used to obtain additional information about the sensor.
     * @see sensorAvailable
     */
    override val sensor: Sensor? by lazy { sensorManager?.getDefaultSensor(sensorType.value) }

    /**
     * Indicates whether requested attitude sensor is available or not.
     */
    override val sensorAvailable: Boolean by lazy {
        AttitudeSensorType.from(sensorType.value) != null && super.sensorAvailable
    }

    /**
     * Updates measurement values with provided [SensorEvent].
     *
     * @param measurement measurement to be updated.
     * @param event event containing data to update measurement.
     * @return true if measurement was successfully updated, false otherwise.
     */
    override fun updateMeasurementWithSensorEvent(
        measurement: AttitudeSensorMeasurement,
        event: SensorEvent?
    ): Boolean {
        return AttitudeSensorMeasurementConverter.convert(event, measurement, startOffset)
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
        if (AttitudeSensorType.from(sensor.type) == null) {
            return
        }

        val sensorAccuracy = SensorAccuracy.from(accuracy)
        accuracyChangedListener?.onAccuracyChanged(this, sensorAccuracy)
    }

    /**
     * Creates a new instance of an [AttitudeSensorMeasurement] measurement.
     */
    override fun createEmptyMeasurement(): AttitudeSensorMeasurement {
        return AttitudeSensorMeasurement()
    }
}