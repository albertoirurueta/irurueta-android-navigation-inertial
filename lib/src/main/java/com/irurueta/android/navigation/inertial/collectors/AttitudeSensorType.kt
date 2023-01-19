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

/**
 * Indicates the attitude type supported by a rotation sensor.
 *
 * @property value numerical value representing attitude sensor type.
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