/*
 * Copyright (C) 2023 Alberto Irurueta Carro (alberto@irurueta.com)
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

import android.hardware.Sensor
import android.os.Build

/**
 * Indicates a sensor type.
 *
 * @property value numerical value representing a sensor type.
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
    ACCELEROMETER_UNCALIBRATED(Constants.TYPE_ACCELEROMETER_UNCALIBRATED),

    /**
     * Linear acceleration.
     * Returns accelerometer not including gravity.
     */
    LINEAR_ACCELERATION(Sensor.TYPE_LINEAR_ACCELERATION),

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
                && value == Constants.TYPE_ACCELEROMETER_UNCALIBRATED
            ) {
                return null
            }
            return values().find { it.value == value }
        }

        /**
         * Gets sensor type based on provided [AccelerometerSensorType].
         *
         * @param accelerometerSensorType accelerometer sensor type.
         * @return conversion to [SensorType] or null if no match is found.
         */
        fun from(accelerometerSensorType: AccelerometerSensorType): SensorType {
            return when (accelerometerSensorType) {
                AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED -> ACCELEROMETER_UNCALIBRATED
                else -> ACCELEROMETER
            }
        }

        /**
         * Gets sensor type based on provided [GyroscopeSensorType].
         *
         * @param gyroscopeSensorType gyroscope sensor type.
         * @return conversion to [SensorType] or null if no match is found.
         */
        fun from(gyroscopeSensorType: GyroscopeSensorType): SensorType {
            return when (gyroscopeSensorType) {
                GyroscopeSensorType.GYROSCOPE_UNCALIBRATED -> GYROSCOPE_UNCALIBRATED
                else -> GYROSCOPE
            }
        }

        /**
         * Gets sensor type based on provided [MagnetometerSensorType].
         *
         * @param magnetometerSensorType magnetometer sensor type.
         * @return conversion to [SensorType] or null if no match is found.
         */
        fun from(magnetometerSensorType: MagnetometerSensorType): SensorType {
            return when (magnetometerSensorType) {
                MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED -> MAGNETOMETER_UNCALIBRATED
                else -> MAGNETOMETER
            }
        }

        /**
         * Gets sensor type based on provided [AttitudeSensorType].
         *
         * @param attitudeSensorType attitude sensor type.
         * @return conversion to [SensorType] or null if no match is found.
         */
        fun from(attitudeSensorType: AttitudeSensorType): SensorType {
            return when(attitudeSensorType) {
                AttitudeSensorType.ABSOLUTE_ATTITUDE -> ABSOLUTE_ATTITUDE
                else -> RELATIVE_ATTITUDE
            }
        }
    }
}