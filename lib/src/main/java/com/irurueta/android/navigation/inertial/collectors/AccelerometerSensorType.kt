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
import android.os.Build


/**
 * Indicates the accelerometer types supported by this accelerometer sensor.
 *
 * @property value numerical value representing accelerometer sensor type.
 */
enum class AccelerometerSensorType(val value: Int) {
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
    ACCELEROMETER_UNCALIBRATED(Constants.TYPE_ACCELEROMETER_UNCALIBRATED);

    companion object {
        /**
         * Gets accelerometer sensor type based on provided numerical value.
         *
         * @param value numerical value representing accelerometer sensor type.
         * @return accelerometer sensor type as an enum or null if value has no match.
         */
        fun from(value: Int): AccelerometerSensorType? {
            if (Build.VERSION.SDK_INT < Build.VERSION_CODES.O
                && value == Constants.TYPE_ACCELEROMETER_UNCALIBRATED
            ) {
                return null
            }
            return values().find { it.value == value }
        }
    }
}