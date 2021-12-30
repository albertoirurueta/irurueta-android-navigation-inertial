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
 * Manages gyroscope sensor.
 *
 * @property context Android context.
 * @property sensorType One of the supported gyroscope sensor types.
 * @property sensorDelay Delay of sensor between samples.
 * @property gyroscopeMeasurementListener listener to notify new gyroscope measurements.
 * @property gyroscopeAccuracyChangedListener listener to notify changes in gyroscope accuracy.
 */
class GyroscopeSensor(
    val context: Context,
    val sensorType: GyroscopeSensorType = GyroscopeSensorType.GYROSCOPE,
    val sensorDelay: SensorDelay = SensorDelay.FASTEST,
    var gyroscopeMeasurementListener: OnGyroscopeMeasurementListener? = null,
    var gyroscopeAccuracyChangedListener: OnGyroscopeAccuracyChangedListener? = null
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
            val sensorType = GyroscopeSensorType.from(event.sensor.type) ?: return

            val sensorAccuracy = SensorAccuracy.from(event.accuracy)
            val timestamp = event.timestamp

            val wx = event.values[0]
            val wy = event.values[1]
            val wz = event.values[2]
            var bx: Float? = null
            var by: Float? = null
            var bz: Float? = null
            if (sensorType == GyroscopeSensorType.GYROSCOPE_UNCALIBRATED) {
                bx = event.values[3]
                by = event.values[4]
                bz = event.values[5]
            }

            gyroscopeMeasurementListener?.onGyroscopeMeasurement(
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
            if (GyroscopeSensorType.from(sensor.type) == null) {
                return
            }

            val sensorAccuracy = SensorAccuracy.from(accuracy)
            gyroscopeAccuracyChangedListener?.onGyroscopeAccuracyChanged(sensorAccuracy)
        }
    }

    /**
     * Sensor being used to obtain measurements or null if not available.
     * This can be used to obtain additional information about the sensor.
     * @see sensorAvailable
     */
    val sensor: Sensor? by lazy { sensorManager?.getDefaultSensor(sensorType.value) }

    /**
     * Indicates whether requested gyroscope sensor is available or not.
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
     * Starts collecting gyroscope measurements.
     *
     * @return true if sensor is available and was successfully enabled.
     */
    fun start() : Boolean {
        val sensor = this.sensor ?: return false
        return sensorManager?.registerListener(sensorEventListener, sensor, sensorDelay.value)
            ?: false
    }

    /**
     * Stops collecting gyroscope measurements.
     */
    fun stop() {
        sensorManager?.unregisterListener(sensorEventListener, sensor)
    }

    /**
     * Indicates the gyroscope types supported by this gyroscope sensor.
     */
    enum class GyroscopeSensorType(val value: Int) {
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
            fun from(value: Int): GyroscopeSensorType? {
                return values().find { it.value == value }
            }
        }
    }

    /**
     * Interface to notify when a new gyroscope measurement is available.
     */
    interface OnGyroscopeMeasurementListener {
        /**
         * Called when a new gyroscope measurement is available.
         *
         * @param wx angular speed around device x-axis expressed in radians per second (rad/s).
         * @param wy angular speed around device y-axis expressed in radians per second (rad/s).
         * @param wz angular speed around device z-axis expressed in radians per second (rad/s).
         * @param bx estimated drift around device x-axis expressed in radians per second (rad/s).
         * Only available when using [GyroscopeSensorType.GYROSCOPE_UNCALIBRATED].
         * @param by estimated drift around device y-axis expressed in radians per second (rad/s).
         * Only available when using [GyroscopeSensorType.GYROSCOPE_UNCALIBRATED].
         * @param bz estimated drift around device z-axis expressed in radians per second (rad/s).
         * Only available when using [GyroscopeSensorType.GYROSCOPE_UNCALIBRATED].
         * @param timestamp time in nanoseconds at which the measurement was made. Each measurement
         * wil be monotonically increasing using the same time base as
         * [android.os.SystemClock.elapsedRealtimeNanos].
         * @param accuracy gyroscope sensor accuracy.
         */
        fun onGyroscopeMeasurement(
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

    /**
     * Interface to notify when gyroscope sensor accuracy changes.
     */
    interface OnGyroscopeAccuracyChangedListener {
        /**
         * Called when gyroscope accuracy changes.
         *
         * @param accuracy new gyroscope accuracy.
         */
        fun onGyroscopeAccuracyChanged(accuracy: SensorAccuracy?)
    }
}