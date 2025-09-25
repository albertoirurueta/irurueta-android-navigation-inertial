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
 * Defines the delay of a sensor between samples.
 *
 * @property value numerical value representing sensor delay.
 */
enum class SensorDelay(val value: Int) {

    /**
     * Gets sensor data as fast as possible.
     */
    FASTEST(SensorManager.SENSOR_DELAY_FASTEST),

    /**
     * Gets sensor date at a rate suitable for games.
     */
    GAME(SensorManager.SENSOR_DELAY_GAME),

    /**
     * Gets sensor data at a rate suitable for the user interface.
     */
    UI(SensorManager.SENSOR_DELAY_UI),

    /**
     * Gets sensor data at default rate suitable for screen orientation changes.
     */
    NORMAL(SensorManager.SENSOR_DELAY_NORMAL);

    companion object {
        /**
         * Gets sensor delay based on provided numerical value.
         *
         * @param value numerical value representing sensor delay.
         * @return sensor delay as an enum or null if value has no match.
         */
        fun from(value: Int): SensorDelay? {
            return entries.find { it.value == value }
        }
    }
}