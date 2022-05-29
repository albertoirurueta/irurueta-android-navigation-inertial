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
import android.hardware.SensorEventListener
import com.irurueta.algebra.Matrix
import com.irurueta.android.navigation.inertial.AttitudeHelper
import com.irurueta.geometry.MatrixRotation3D
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType

/**
 * Manages and collects attitude sensor measurements.
 *
 * @property context Android context.
 * @property sensorType One of the supported attitude sensor types.
 * @property sensorDelay Delay of sensor between samples.
 * @property estimateCoordinateTransformation true to estimate coordinate transformation along with
 * attitude on each measurement, false to only obtain attitude.
 * @property measurementListener listener to notify new attitude measurements.
 * @property accuracyChangedListener listener to notify changes in attitude accuracy.
 */
class AttitudeSensorCollector(
    context: Context,
    val sensorType: SensorType = SensorType.ABSOLUTE_ATTITUDE,
    sensorDelay: SensorDelay = SensorDelay.FASTEST,
    var estimateCoordinateTransformation: Boolean = false,
    var measurementListener: OnMeasurementListener? = null,
    accuracyChangedListener: OnAccuracyChangedListener? = null
) : SensorCollector(context, sensorDelay, accuracyChangedListener) {

    /**
     * Instance being reused for performance reasons and containing device attitude expressed in
     * NED coordinate system.
     */
    private val attitude = Quaternion()

    /**
     * Instance being reused for performance reasons and containing device attitude expressed in
     * NED coordinate system.
     */
    private val coordinateTransformation =
        CoordinateTransformation(FrameType.BODY_FRAME, FrameType.LOCAL_NAVIGATION_FRAME)

    /**
     * Instance being reused for performance reasons and containing device attitude expressed in
     * NED coordinate system.
     */
    private val matrix = Matrix(
        MatrixRotation3D.ROTATION3D_INHOM_MATRIX_ROWS,
        MatrixRotation3D.ROTATION3D_INHOM_MATRIX_COLS
    )

    /**
     * Internal listener to handle sensor events.
     */
    override val sensorEventListener = object : SensorEventListener {
        override fun onSensorChanged(event: SensorEvent?) {
            if (event == null) {
                return
            }

            if (SensorType.from(event.sensor.type) == null) {
                return
            }

            val sensorAccuracy = SensorAccuracy.from(event.accuracy)
            val timestamp = event.timestamp
            val headingAccuracy = if (estimateCoordinateTransformation) {
                 AttitudeHelper.convertToNED(
                    event.values,
                    coordinateTransformation,
                    attitude,
                    matrix
                )
            } else {
                AttitudeHelper.convertToNED(event.values, attitude)
            }

            measurementListener?.onMeasurement(
                attitude,
                if (estimateCoordinateTransformation) coordinateTransformation else null,
                headingAccuracy,
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
     *
     * @see sensorAvailable
     */
    override val sensor: Sensor? by lazy { sensorManager?.getDefaultSensor(sensorType.value) }

    /**
     * Indicates the supported attitude types.
     *
     * @property value numerical value representing attitude sensor type.
     */
    enum class SensorType(val value: Int) {
        /**
         * Returns relative device attitude respect to an arbitrary system of coordinates.
         * The measured attitude may drift after some time.
         */
        RELATIVE_ATTITUDE(Sensor.TYPE_GAME_ROTATION_VECTOR),

        /**
         * Returns absolute device attitude respect to Earth.
         * Uses a fusion of gyroscope and magnetometer measurements to avoid attitude drift.
         * Attitude is expressed in NED coordinates.
         */
        ABSOLUTE_ATTITUDE(Sensor.TYPE_ROTATION_VECTOR),

        /**
         * Returns absolute device attitude respect to Earth.
         * Only uses magnetometer measurements, which might be noisier than using gyroscope
         * and accelerometer measurements alone but uses less power and has no drift.
         * Attitude is expressed in NED coordinates.
         */
        GEOMAGNETIC_ABSOLUTE_ATTITUDE(Sensor.TYPE_GEOMAGNETIC_ROTATION_VECTOR);

        companion object {
            /**
             * Gets attitude sensor type based on provided numerical value.
             *
             * @param value numerical value representing attitude sensor type.
             * @return corresponding sensor type as an enum or null if value has no match.
             */
            fun from(value: Int): SensorType? {
                return values().find { it.value == value }
            }
        }
    }

    /**
     * Interface to notify when a new attitude measurement is available.
     */
    fun interface OnMeasurementListener {
        /**
         * Called when a new attitude measurement is available.
         *
         * @param coordinateTransformation coordinate transformation contained measured attitude.
         * @param attitude measured attitude.
         * @param headingAccuracy accuracy of heading angle expressed in radians.
         * @param timestamp time in nanoseconds at which the measurement was made. Each measurement
         * will be monotonically increasing using the same time base as
         * [android.os.SystemClock.elapsedRealtimeNanos].
         * @param accuracy sensor accuracy.
         */
        fun onMeasurement(
            attitude: Quaternion,
            coordinateTransformation: CoordinateTransformation?,
            headingAccuracy: Double?,
            timestamp: Long,
            accuracy: SensorAccuracy?
        )
    }
}