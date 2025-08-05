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

import android.hardware.Sensor

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
        /**
         * Gets magnetometer sensor type based on provided numerical value.
         *
         * @param value numerical value representing magnetometer sensor type.
         * @return corresponding sensor type as an enum or null if value has no match.
         */
        fun from(value: Int): MagnetometerSensorType? {
            return entries.find { it.value == value }
        }
    }
}