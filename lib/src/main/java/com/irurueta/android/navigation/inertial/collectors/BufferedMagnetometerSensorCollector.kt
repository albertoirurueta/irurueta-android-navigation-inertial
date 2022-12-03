package com.irurueta.android.navigation.inertial.collectors

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent

/**
 * Manages and collects magnetometer sensor measurements using a buffer.
 * A buffered collector allows proper synchronization of events from multiple collectors
 * by colling [getMeasurementsBeforeTimestamp] periodically to obtain measurements in the buffer prior to
 * a certain timestamp.
 *
 * @property context Android context.
 * @property sensorType One of the supported magnetometer sensor types.
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
class BufferedMagnetometerSensorCollector(
    context: Context,
    val sensorType: MagnetometerSensorType = MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
    sensorDelay: SensorDelay = SensorDelay.FASTEST,
    capacity: Int = DEFAULT_CAPACITY,
    startOffsetEnabled: Boolean = true,
    stopWhenFilledBuffer: Boolean = true,
    accuracyChangedListener: OnAccuracyChangedListener<MagnetometerSensorMeasurement, BufferedMagnetometerSensorCollector>? = null,
    bufferFilledListener: OnBufferFilledListener<MagnetometerSensorMeasurement, BufferedMagnetometerSensorCollector>? = null,
    measurementListener: OnMeasurementListener<MagnetometerSensorMeasurement, BufferedMagnetometerSensorCollector>? = null
) : BufferedSensorCollector<MagnetometerSensorMeasurement, BufferedMagnetometerSensorCollector>(
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
     * Measurement being reused internally to copy data and avoid exposing
     * internal instances through listeners for security purposes.
     */
    override val measurement = createEmptyMeasurement()

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
        measurement: MagnetometerSensorMeasurement,
        event: SensorEvent?
    ): Boolean {
        return MagnetometerSensorMeasurementConverter.convert(event, measurement, startOffset)
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
        if (MagnetometerSensorType.from(sensor.type) == null) {
            return
        }

        val sensorAccuracy = SensorAccuracy.from(accuracy)
        accuracyChangedListener?.onAccuracyChanged(this, sensorAccuracy)
    }

    /**
     * Creates a new instance of a [MagnetometerSensorMeasurement] measurement.
     */
    override fun createEmptyMeasurement(): MagnetometerSensorMeasurement {
        return MagnetometerSensorMeasurement()
    }
}