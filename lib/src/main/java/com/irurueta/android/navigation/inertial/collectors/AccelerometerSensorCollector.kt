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
import android.os.Build

/**
 * Manages and collects accelerometer sensor measurements.
 *
 * @property context Android context.
 * @property sensorType One of the supported accelerometer sensor types.
 * @property sensorDelay Delay of sensor between samples.
 * @property measurementListener listener to notify new accelerometer measurements.
 * @property accuracyChangedListener listener to notify changes in accelerometer
 * accuracy.
 */
class AccelerometerSensorCollector(
    context: Context,
    val sensorType: SensorType = SensorType.ACCELEROMETER,
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
            val sensorType = SensorType.from(event.sensor.type) ?: return

            val sensorAccuracy = SensorAccuracy.from(event.accuracy)
            val timestamp = event.timestamp

            val ax = event.values[0]
            val ay = event.values[1]
            val az = event.values[2]
            var bx: Float? = null
            var by: Float? = null
            var bz: Float? = null
            if (sensorType == SensorType.ACCELEROMETER_UNCALIBRATED) {
                bx = event.values[3]
                by = event.values[4]
                bz = event.values[5]
            }

            measurementListener?.onMeasurement(
                ax,
                ay,
                az,
                bx,
                by,
                bz,
                timestamp,
                sensorAccuracy
            )
        }

        override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {
            if (sensor == null) {
                return
            }
            if (SensorType.from(sensor.type) == null) {
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
    override val sensor: Sensor? by lazy { sensorManager?.getDefaultSensor(sensorType.value) }

    /**
     * Indicates whether requested accelerometer sensor is available or not.
     */
    override val sensorAvailable: Boolean by lazy {
        SensorType.from(sensorType.value) != null && super.sensorAvailable
    }

    private companion object {
        /**
         * Constant defining uncalibrated accelerometer type.
         */
        val TYPE_ACCELEROMETER_UNCALIBRATED =
            if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O)
                Sensor.TYPE_ACCELEROMETER_UNCALIBRATED
            else 35
    }

    /**
     * Indicates the accelerometer types supported by this accelerometer sensor.
     *
     * @property value numerical value representing accelerometer sensor type.
     */
    enum class SensorType(val value: Int) {
        /**
         * Accelerometer sensor.
         * Returns acceleration including gravity.
         */
        ACCELEROMETER(Sensor.TYPE_ACCELEROMETER),

        /**
         * Uncalibrated accelerometer sensor.
         * Returns acceleration including gravity but without bias correction.
         * This accelerometer is only available for SDK 26 or later.
         */
        ACCELEROMETER_UNCALIBRATED(TYPE_ACCELEROMETER_UNCALIBRATED);

        companion object {
            /**
             * Gets accelerometer sensor type based on provided numerical value.
             *
             * @param value numerical value representing accelerometer sensor type.
             * @return accelerometer sensor type as an enum or null if value has no match.
             */
            fun from(value: Int): SensorType? {
                if (Build.VERSION.SDK_INT < Build.VERSION_CODES.O
                    && value == TYPE_ACCELEROMETER_UNCALIBRATED
                ) {
                    return null
                }
                return values().find { it.value == value }
            }
        }
    }

    /**
     * Interface to notify when a new accelerometer measurement is available.
     */
    fun interface OnMeasurementListener {
        /**
         * Called when a new accelerometer measurement is available.
         *
         * @param ax acceleration on device x-axis expressed in meters per squared second (m/s^2).
         * @param ay acceleration on device y-axis expressed in meters per squared second (m/s^2).
         * @param az acceleration on device z-axis expressed in meters per squared second (m/s^2).
         * @param bx bias on device x-axis expressed in meters per squared second (m/s^2). Only
         * available when using [SensorType.ACCELEROMETER_UNCALIBRATED]. If available, this value
         * remains constant with calibrated bias value.
         * @param by bias on device y-axis expressed in meters per squared second (m/s^2). Only
         * available when using [SensorType.ACCELEROMETER_UNCALIBRATED]. If available, this value
         * remains constant with calibrated bias value.
         * @param bz bias on device z-axis expressed in meters per squared second (m/s^2). Only
         * available when using [SensorType.ACCELEROMETER_UNCALIBRATED]. If available, this value
         * remains constant with calibrated bias value.
         * @param timestamp time in nanoseconds at which the measurement was made. Each measurement
         * will be monotonically increasing using the same time base as
         * [android.os.SystemClock.elapsedRealtimeNanos].
         * @param accuracy accelerometer sensor accuracy.
         */
        fun onMeasurement(
            ax: Float,
            ay: Float,
            az: Float,
            bx: Float?,
            by: Float?,
            bz: Float?,
            timestamp: Long,
            accuracy: SensorAccuracy?
        )
    }
}