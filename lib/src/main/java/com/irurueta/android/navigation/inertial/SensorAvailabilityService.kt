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
import android.hardware.SensorManager
import android.os.Build

/**
 * Indicates whether a given sensor is available or not.
 */
class SensorAvailabilityService(val context: Context) {

    /**
     * Indicates whether device has an accelerometer.
     * Accelerometer returns acceleration including gravity in x, y and z axes.
     * In this sensor gravity is always influencing the measured acceleration, for this reason, when
     * the device is sitting on a table, the accelerometer reads a magnitude equal to the gravity.
     * Likewise, when the device is in free-fall accelerating towards the ground, the accelerometer
     * reads a magnitude of zero.
     * This type of accelerometer applies bias corrections to returned measures.
     * If no bias correction is required, use [uncalibratedAccelerometerAvailable].
     * @see uncalibratedAccelerometerAvailable
     */
    val accelerometerAvailable: Boolean by lazy { hasSensor(Sensor.TYPE_ACCELEROMETER) }

    /**
     * Indicates whether device has a linear accelerometer.
     * Linear accelerometer returns acceleration NOT including gravity in x, y and z axes.
     * In this sensor, when the device is sitting on a table, the accelerometer reads a magnitude of
     * zero.
     * Likewise, when the device is in free-fall accelerating towards the ground, the accelerometer
     * reads a magnitude equal to the gravity.
     * This type of accelerometer applies bias corrections to returned measures.
     */
    val linearAccelerometerAvailable: Boolean by lazy { hasSensor(Sensor.TYPE_LINEAR_ACCELERATION) }

    /**
     * Indicates whether device has an uncalibrated accelerometer.
     * This is like the normal accelerometer without applying any bias correction (factory bias
     * compensation and temperature compensation are still applied).
     * This accelerometer is only available for SDK 26 or later.
     * @see accelerometerAvailable
     */
    val uncalibratedAccelerometerAvailable: Boolean by lazy {
        if (Build.VERSION.SDK_INT >= Build.VERSION_CODES.O) {
            hasSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED)
        } else {
            false
        }
    }

    /**
     * Indicates whether device has a gyroscope.
     * Gyroscope returns measurements of the rate of rotation expressed in radians/second around the
     * device x, y and z axes.
     * Rotation is positive in counter-clockwise directions.
     * This type of gyroscope applies gyro-drift compensation.
     * If no gyro-drift compensation is required, use [uncalibratedGyroscopeAvailable].
     * @see uncalibratedGyroscopeAvailable
     */
    val gyroscopeAvailable: Boolean by lazy { hasSensor(Sensor.TYPE_GYROSCOPE) }

    /**
     * Indicates whether device has an uncalibrated gyroscope.
     * This is like the normal gyroscope without applying any gyro-drift compensation (factory
     * calibration and temperature compensation are still applied).
     * @see gyroscopeAvailable
     */
    val uncalibratedGyroscopeAvailable: Boolean by lazy {
        hasSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
    }

    /**
     * Indicates whether device has a magnetometer.
     * Magnetometer measures the ambient magnetic field expressed in micro-Tesla in the device x, y
     * and z axes.
     * If no hard-iron bias correction is required, use [uncalibratedMagnetometerAvailable].
     * @see uncalibratedMagnetometerAvailable
     */
    val magnetometerAvailable: Boolean by lazy { hasSensor(Sensor.TYPE_MAGNETIC_FIELD) }

    /**
     * Indicates whether device has an uncalibrated magnetometer.
     * This is like the normal magnetometer without applying hard-iron compensation (factory
     * calibration and temperature compensation are still applied).
     * @see magnetometerAvailable
     */
    val uncalibratedMagnetometerAvailable: Boolean by lazy {
        hasSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED)
    }

    /**
     * Indicates whether device can return measurements containing the direction and magnitude of
     * gravity, expressed in meters per squared second around the device x, y and z aes.
     */
    val gravityAvailable: Boolean by lazy { hasSensor(Sensor.TYPE_GRAVITY) }

    /**
     * Indicates whether device can return measurements containing absolute device attitude respect
     * to Earth.
     */
    val absoluteAttitudeAvailable: Boolean by lazy { hasSensor(Sensor.TYPE_ROTATION_VECTOR) }

    /**
     * Indicates whether device can return measurements containing relative device attitude rotation
     * respect to Earth.
     */
    val relativeAttitudeAvailable: Boolean by lazy { hasSensor(Sensor.TYPE_GAME_ROTATION_VECTOR) }

    /**
     * Indicates whether device has provided sensor type.
     *
     * @param sensorType a sensor type.
     * @return true if sensor exists, false otherwise.
     */
    fun hasSensor(sensorType: SensorType): Boolean {
        return hasSensor(sensorType.value)
    }

    /**
     * Returns the device sensor manager, if any is available.
     * Sensor manager can measure acceleration, angular speed, magnetic fields, gravity and
     * attitude.
     */
    private val sensorManager: SensorManager? by lazy {
        context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
    }

    /**
     * Indicates whether device has provided sensor type.
     *
     * @param sensorType a constant indicating a sensor type.
     * @return true if sensor exists, false otherwise.
     */
    private fun hasSensor(sensorType: Int): Boolean {
        return sensorManager?.getDefaultSensor(sensorType) != null
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
     * Indicates the sensor types supported for inertial navigation.
     *
     * @property value numerical value representing sensor type.
     */
    enum class SensorType(val value: Int) {
        /**
         * Accelerometer sensor.
         * Returns acceleration including gravity.
         */
        ACCELEROMETER(Sensor.TYPE_ACCELEROMETER),

        /**
         * Linear acceleration.
         * Returns accelerometer not including gravity.
         */
        LINEAR_ACCELERATION(Sensor.TYPE_LINEAR_ACCELERATION),

        /**
         * Uncalibrated accelerometer sensor.
         * Returns acceleration including gravity but without bias correction.
         * This accelerometer is only available for SDK 26 or later.
         */
        ACCELEROMETER_UNCALIBRATED(TYPE_ACCELEROMETER_UNCALIBRATED),

        /**
         * Gyroscope.
         * Returns angular speed measurements.
         */
        GYROSCOPE(Sensor.TYPE_GYROSCOPE),

        /**
         * Uncalibrated gyroscope.
         * Returns angular speed measurements without bias correction.
         */
        GYROSCOPE_UNCALIBRATED(Sensor.TYPE_GYROSCOPE_UNCALIBRATED),

        /**
         * Magnetometer.
         * Returns magnetic field measurements.
         */
        MAGNETOMETER(Sensor.TYPE_MAGNETIC_FIELD),

        /**
         * Uncalibrated magnetometer.
         * Returns magnetic field measurements without hard-iron bias correction.
         */
        MAGNETOMETER_UNCALIBRATED(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED),

        /**
         * Gravity.
         */
        GRAVITY(Sensor.TYPE_GRAVITY),

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
             * Gets sensor type based on provided numerical value.
             *
             * @param value code used for sensor types.
             * @return code expressed as an enum or null if code has no match.
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
}