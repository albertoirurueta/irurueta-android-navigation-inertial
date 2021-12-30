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
import com.irurueta.algebra.Matrix
import com.irurueta.geometry.Quaternion
import com.irurueta.geometry.Rotation3D
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType

/**
 * Handles attitude sensor to obtain device orientation measurements.
 *
 * @property context Android context.
 * @property sensorType One of the supported attitude sensor types.
 * @property sensorDelay Delay of sensor between samples.
 * @property attitudeMeasurementListener listener to notify new attitude measurements.
 * @property attitudeAccuracyChangedListener listener to notify changes in sensor accuracy.
 */
class AttitudeSensor(
    val context: Context,
    val sensorType: AttitudeSensorType = AttitudeSensorType.ABSOLUTE_ATTITUDE,
    val sensorDelay: SensorDelay = SensorDelay.FASTEST,
    var attitudeMeasurementListener: OnAttitudeMeasurementListener? = null,
    var attitudeAccuracyChangedListener: OnAttitudeAccuracyChangedListener? = null
) {
    /**
     * Internal quaternion reused for efficiency reasons containing measured orientation.
     */
    private val quaternion = Quaternion()

    /**
     * Internal matrix reused for efficiency reasons containing measured orientation.
     */
    private val rotationMatrix: Matrix =
        Matrix.identity(CoordinateTransformation.ROWS, CoordinateTransformation.COLS)

    /**
     * Coordinate transformation reused for efficiency reasons containing measured orientation.
     */
    private val coordinateTransformation =
        CoordinateTransformation(FrameType.LOCAL_NAVIGATION_FRAME, FrameType.BODY_FRAME)

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
            if (AttitudeSensorType.from(event.sensor.type) == null) {
                return
            }

            val sensorAccuracy = SensorAccuracy.from(event.accuracy)
            val timestamp = event.timestamp

            val headingAccuracyAvailable = event.values[4] != -1.0f
            val headingAccuracyRadians = if (headingAccuracyAvailable) {
                event.values[4]
            } else {
                null
            }

            val a = event.values[3]
            val b = event.values[0]
            val c = event.values[1]
            val d = event.values[2]
            quaternion.a = a.toDouble()
            quaternion.b = b.toDouble()
            quaternion.c = c.toDouble()
            quaternion.d = d.toDouble()
            quaternion.normalize()

            quaternion.asInhomogeneousMatrix(rotationMatrix)

            coordinateTransformation.matrix = rotationMatrix

            attitudeMeasurementListener?.onAttitudeMeasurement(
                quaternion,
                coordinateTransformation,
                headingAccuracyRadians,
                timestamp,
                sensorAccuracy
            )
        }

        override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) {
            if (sensor == null) {
                return
            }
            if (AttitudeSensorType.from(sensor.type) == null) {
                return
            }

            val sensorAccuracy = SensorAccuracy.from(accuracy)
            attitudeAccuracyChangedListener?.onAttitudeAccuracyChanged(sensorAccuracy)
        }
    }

    /**
     * Sensor being used to obtain orientation measurements or null if not available.
     * This can be used to obtain additional information about the sensor.
     * @see sensorAvailable
     */
    val sensor: Sensor? by lazy { sensorManager?.getDefaultSensor(sensorType.value) }

    /**
     * Indicates whether requested orientation sensor is available or not.
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
     * Starts collecting orientation measurements.
     *
     * @return true if sensor is available and was successfully enabled.
     */
    fun start() : Boolean {
        val sensor = this.sensor ?: return false
        return sensorManager?.registerListener(sensorEventListener, sensor, sensorDelay.value)
            ?: false
    }

    /**
     * Stops collecting orientation measurements.
     */
    fun stop() {
        sensorManager?.unregisterListener(sensorEventListener, sensor)
    }

    /**
     * Indicates the orientation types supported by this sensor.
     *
     * @property value numerical value representing orientation sensor type.
     */
    enum class AttitudeSensorType(val value: Int) {
        /**
         * Absolute attitude.
         * This sensor requires a magnetometer and returns absolute device orientation respect to
         * Earth.
         */
        ABSOLUTE_ATTITUDE(Sensor.TYPE_ROTATION_VECTOR),

        /**
         * Relative attitude.
         * This sensor does not require a magnetometer and returns device orientation respect to
         * an arbitrary initial orientation that might drift over time.
         */
        RELATIVE_ATTITUDE(Sensor.TYPE_GAME_ROTATION_VECTOR);

        companion object {
            /**
             * Gets attitude sensor type based on provided numerical value.
             *
             * @param value numerical value representing attitude sensor type.
             * @return attitude sensor type as an enum or null if value has no match.
             */
            fun from(value: Int): AttitudeSensorType? {
                return values().find { it.value == value }
            }
        }
    }

    /**
     * Interface to notify when a new attitude measurement is available.
     */
    interface OnAttitudeMeasurementListener {
        /**
         * Called when a new attitude measurement is available.
         *
         * @param rotation contains measured device orientation (either absolute or relative).
         * @param coordinationTransformation contains measured device orientation as a coordinate
         * transformation.
         * @param headingAccuracyRadians accuracy of heading measurement expressed in radians (rad),
         * if available.
         * @param timestamp time in nanoseconds at which the measurement was made. Each measurement
         * will be monotonically increasing using the same time base as
         * [android.os.SystemClock.elapsedRealtimeNanos].
         * @param accuracy attitude sensor accuracy.
         */
        fun onAttitudeMeasurement(
            rotation: Rotation3D,
            coordinationTransformation: CoordinateTransformation,
            headingAccuracyRadians: Float?,
            timestamp: Long,
            accuracy: SensorAccuracy?
        )
    }

    /**
     * Interface to notify when attitude sensor accuracy changes.
     */
    interface OnAttitudeAccuracyChangedListener {
        /**
         * Called when attitude accuracy changes.
         *
         * @param accuracy new attitude accuracy.
         */
        fun onAttitudeAccuracyChanged(accuracy: SensorAccuracy?)
    }
}