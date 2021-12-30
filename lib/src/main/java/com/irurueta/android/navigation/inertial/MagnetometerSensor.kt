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

/**
 * Manages magnetometer sensor.
 *
 * @property context Android context.
 * @property sensorType One of the supported magnetometer sensor types.
 * @property sensorDelay Delay of sensor between samples.
 * @property magnetometerMeasurementListener listener to notify new magnetometer measurements.
 * @property magnetometerAccuracyChangedListener listener to notify changes in magnetometer
 * accuracy.
 */
class MagnetometerSensor(
    val context: Context,
    val sensorType: MagnetometerSensorType = MagnetometerSensorType.MAGNETOMETER,
    val sensorDelay: SensorDelay = SensorDelay.FASTEST,
    var magnetometerMeasurementListener: OnMagnetometerMeasurementListener? = null,
    var magnetometerAccuracyChangedListener: OnMagnetometerAccuracyChangedListener? = null
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
            val sensorType = MagnetometerSensorType.from(event.sensor.type) ?: return

            val sensorAccuracy = SensorAccuracy.from(event.accuracy)
            val timestamp = event.timestamp

            val bx = event.values[0]
            val by = event.values[1]
            val bz = event.values[2]
            var hardIronX: Float? = null
            var hardIronY: Float? = null
            var hardIronZ: Float? = null
            if (sensorType == MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED) {
                hardIronX = event.values[3]
                hardIronY = event.values[4]
                hardIronZ = event.values[5]
            }

            magnetometerMeasurementListener?.onMagnetometerMeasurement(
                bx,
                by,
                bz,
                hardIronX,
                hardIronY,
                hardIronZ,
                timestamp,
                sensorAccuracy
            )
        }

        override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {
            if (sensor == null) {
                return
            }
            if (MagnetometerSensorType.from(sensor.type) == null) {
                return
            }

            val sensorAccuracy = SensorAccuracy.from(accuracy)
            magnetometerAccuracyChangedListener?.onMagnetometerAccuracyChanged(sensorAccuracy)
        }
    }

    /**
     * Sensor being used to obtain measurements or null if not available.
     * This can be used to obtain additional information about the sensor.
     * @see sensorAvailable
     */
    val sensor: Sensor? by lazy { sensorManager?.getDefaultSensor(sensorType.value) }

    /**
     * Indicates whether requested magnetometer sensor is available or not.
     */
    val sensorAvailable: Boolean by lazy {
        val type = SensorAvailabilityService.SensorType.from(sensorType.value)
        if (type != null) {
            val availabilityService = SensorAvailabilityService(context)
            availabilityService.hasSensor(type)
        } else {
            false
        }
    }

    /**
     * Starts collecting magnetometer measurements.
     *
     * @return true if sensor is available and was successfully enabled.
     */
    fun start(): Boolean {
        val sensor = this.sensor ?: return false
        return sensorManager?.registerListener(sensorEventListener, sensor, sensorDelay.value)
            ?: false
    }

    /**
     * Stops collecting magnetometer measurements.
     */
    fun stop() {
        sensorManager?.unregisterListener(sensorEventListener, sensor)
    }

    /**
     * Indicates the magnetometer types supported by this magnetometer sensor.
     *
     * @property value numerical value representing magnetometer sensor type.
     */

    enum class MagnetometerSensorType(val value: Int) {
        /**
         * Magnetometer.
         * Returns magnetic field measurements.
         */
        MAGNETOMETER(Sensor.TYPE_MAGNETIC_FIELD),

        /**
         * Uncalibrated magnetometer.
         * Returns magnetic field measurements without hard-iron bias correction.
         */
        MAGNETOMETER_UNCALIBRATED(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED);

        companion object {
            fun from(value: Int): MagnetometerSensorType? {
                return values().find { it.value == value }
            }
        }
    }

    /**
     * Interface to notify when a new magnetometer measurement is available.
     */
    interface OnMagnetometerMeasurementListener {
        /**
         * Called when a new magnetometer measurement is available.
         *
         * @param bx magnetic field on device x-axis expressed in micro-Teslas (µT).
         * @param by magnetic field on device y-axis expressed in micro-Teslas (µT).
         * @param bz magnetic field on device z-axis expressed in micro-Teslas (µT).
         * @param hardIronX hard iron on device x-axis expressed in micro-Teslas (µT). Only
         * available when using [MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED].
         * @param hardIronY hard iron on device y-axis expressed in micro-Teslas (µT). Only
         * available when using [MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED].
         * @param hardIronZ hard iron on device y-axis expressed in micro-Teslas (µT). Only
         * available when using [MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED].
         * @param timestamp time in nanoseconds at which the measurement was made. Each measurement
         * will be monotonically increasing using the same time base as
         * [android.os.SystemClock.elapsedRealtimeNanos].
         * @param accuracy accelerometer sensor accuracy.
         */
        fun onMagnetometerMeasurement(
            bx: Float,
            by: Float,
            bz: Float,
            hardIronX: Float?,
            hardIronY: Float?,
            hardIronZ: Float?,
            timestamp: Long,
            accuracy: SensorAccuracy?
        )
    }

    /**
     * Interface to notify when magnetometer sensor accuracy changes.
     */
    interface OnMagnetometerAccuracyChangedListener {
        /**
         * Called when magnetometer accuracy changes.
         *
         * @param accuracy new magnetometer accuracy.
         */
        fun onMagnetometerAccuracyChanged(accuracy: SensorAccuracy?)
    }
}