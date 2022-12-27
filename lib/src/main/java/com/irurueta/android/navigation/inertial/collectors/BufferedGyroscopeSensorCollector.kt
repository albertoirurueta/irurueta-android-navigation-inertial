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

/**
 * Manages and collects gyroscope sensor measurements using a buffer.
 * A buffered collector allows proper synchronization of events from multiple collectors
 * by colling [getMeasurementsBeforeTimestamp] periodically to obtain measurements in the buffer prior to
 * a certain timestamp.
 *
 * @property context Android context.
 * @property sensorType One of the supported gyroscope sensor types.
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
 */
class BufferedGyroscopeSensorCollector(
    context: Context,
    val sensorType: GyroscopeSensorType = GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
    sensorDelay: SensorDelay = SensorDelay.FASTEST,
    capacity: Int = DEFAULT_CAPACITY,
    startOffsetEnabled: Boolean = true,
    stopWhenFilledBuffer: Boolean = true,
    accuracyChangedListener: OnAccuracyChangedListener<GyroscopeSensorMeasurement, BufferedGyroscopeSensorCollector>? = null,
    bufferFilledListener: OnBufferFilledListener<GyroscopeSensorMeasurement, BufferedGyroscopeSensorCollector>? = null,
    measurementListener: OnMeasurementListener<GyroscopeSensorMeasurement, BufferedGyroscopeSensorCollector>? = null
) : BufferedSensorCollector<GyroscopeSensorMeasurement, BufferedGyroscopeSensorCollector>(
    context,
    sensorDelay,
    capacity,
    startOffsetEnabled,
    stopWhenFilledBuffer,
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
     * Updates measurement values with provided [SensorEvent].
     *
     * @param measurement measurement to be updated.
     * @param event event containing data to update measurement.
     * @return true if measurement was successfully updated, false otherwise.
     */
    override fun updateMeasurementWithSensorEvent(
        measurement: GyroscopeSensorMeasurement,
        event: SensorEvent?
    ): Boolean {
        return GyroscopeSensorMeasurementConverter.convert(event, measurement, startOffset)
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
        if (GyroscopeSensorType.from(sensor.type) == null) {
            return
        }

        val sensorAccuracy = SensorAccuracy.from(accuracy)
        accuracyChangedListener?.onAccuracyChanged(this, sensorAccuracy)
    }

    /**
     * Creates a new instance of a [GyroscopeSensorMeasurement] measurement.
     */
    override fun createEmptyMeasurement(): GyroscopeSensorMeasurement {
        return GyroscopeSensorMeasurement()
    }
}