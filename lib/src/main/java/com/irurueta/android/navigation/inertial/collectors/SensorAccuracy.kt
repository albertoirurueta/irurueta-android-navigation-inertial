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

import android.hardware.SensorManager

/**
 * Indicates the accuracy of a sensor.
 *
 * @property value numerical value representing sensor accuracy.
 */
enum class SensorAccuracy(val value: Int) {
    /**
     * The values returned by this sensor cannot be trusted, calibration is needed or the
     * environment doesn't allow readings.
     */
    UNRELIABLE(SensorManager.SENSOR_STATUS_UNRELIABLE),

    /**
     * Sensor is reporting data with low accuracy, calibration with the environment is needed.
     */
    LOW(SensorManager.SENSOR_STATUS_ACCURACY_LOW),

    /**
     * Sensor is reporting data with an average level of accuracy, calibration with the environment
     * may improve the readings.
     */
    MEDIUM(SensorManager.SENSOR_STATUS_ACCURACY_MEDIUM),

    /**
     * Sensor is reporting data with maximum accuracy.
     */
    HIGH(SensorManager.SENSOR_STATUS_ACCURACY_HIGH);

    companion object {
        /**
         * Gets sensor accuracy based on provided numerical value.
         *
         * @param value numerical value representing sensor accuracy.
         * @return sensor accuracy as an enum or null if value has no match.
         */
        fun from(value: Int): SensorAccuracy? {
            return values().find { it.value == value }
        }
    }
}