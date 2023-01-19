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
import com.irurueta.android.navigation.inertial.collectors.SensorType

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
}