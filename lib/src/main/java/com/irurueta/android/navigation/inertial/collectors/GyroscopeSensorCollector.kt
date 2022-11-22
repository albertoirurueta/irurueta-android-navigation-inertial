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

/**
 * Manages and collects gyroscope sensor measurements.
 *
 * @property context Android context.
 * @property sensorType One of the supported gyroscope sensor types.
 * @property sensorDelay Delay of sensor between samples.
 * @property measurementListener listener to notify new gyroscope measurements.
 * @property accuracyChangedListener listener to notify changes in gyroscope accuracy.
 */
class GyroscopeSensorCollector(
    context: Context,
    val sensorType: SensorType = SensorType.GYROSCOPE_UNCALIBRATED,
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

            val wx = event.values[0]
            val wy = event.values[1]
            val wz = event.values[2]
            var bx: Float? = null
            var by: Float? = null
            var bz: Float? = null
            if (sensorType == SensorType.GYROSCOPE_UNCALIBRATED) {
                bx = event.values[3]
                by = event.values[4]
                bz = event.values[5]
            }

            measurementListener?.onMeasurement(
                wx,
                wy,
                wz,
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
     * Indicates the gyroscope types supported by this gyroscope sensor.
     */
    enum class SensorType(val value: Int) {
        /**
         * Gyroscope sensor.
         * Returns angular speed measurements.
         */
        GYROSCOPE(Sensor.TYPE_GYROSCOPE),

        /**
         * Uncalibrated gyroscope.
         * Returns angular speed measurements without bias correction.
         */
        GYROSCOPE_UNCALIBRATED(Sensor.TYPE_GYROSCOPE_UNCALIBRATED);

        companion object {
            /**
             * Gets gyroscope sensor type based on provided numerical value.
             *
             * @param value numerical value representing gyroscope sensor type.
             * @return gyroscope sensor type as an enum or null if value has no match.
             */
            fun from(value: Int): SensorType? {
                return values().find { it.value == value }
            }
        }
    }

    /**
     * Interface to notify when a new gyroscope measurement is available.
     */
    fun interface OnMeasurementListener {
        /**
         * Called when a new gyroscope measurement is available.
         *
         * @param wx angular speed around device x-axis expressed in radians per second (rad/s) and
         * in ENU coordinates system.
         * @param wy angular speed around device y-axis expressed in radians per second (rad/s) and
         * in ENU coordinates system.
         * @param wz angular speed around device z-axis expressed in radians per second (rad/s) and
         * in ENU coordinates system.
         * @param bx estimated drift around device x-axis expressed in radians per second (rad/s)
         * and in ENU coordinates system.
         * Only available when using [SensorType.GYROSCOPE_UNCALIBRATED]. If available, this value
         * remains constant with calibrated bias value.
         * @param by estimated drift around device y-axis expressed in radians per second (rad/s)
         * and in ENU coordinates system.
         * Only available when using [SensorType.GYROSCOPE_UNCALIBRATED]. If available, this value
         * remains constant with calibrated bias value.
         * @param bz estimated drift around device z-axis expressed in radians per second (rad/s)
         * and in ENU coordinates system.
         * Only available when using [SensorType.GYROSCOPE_UNCALIBRATED]. If available, this value
         * remains constant with calibrated bias value.
         * @param timestamp time in nanoseconds at which the measurement was made. Each measurement
         * wil be monotonically increasing using the same time base as
         * [android.os.SystemClock.elapsedRealtimeNanos].
         * @param accuracy gyroscope sensor accuracy.
         */
        fun onMeasurement(
            wx: Float,
            wy: Float,
            wz: Float,
            bx: Float?,
            by: Float?,
            bz: Float?,
            timestamp: Long,
            accuracy: SensorAccuracy?
        )
    }
}