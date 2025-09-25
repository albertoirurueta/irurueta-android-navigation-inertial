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
            return entries.find { it.value == value }
        }
    }
}