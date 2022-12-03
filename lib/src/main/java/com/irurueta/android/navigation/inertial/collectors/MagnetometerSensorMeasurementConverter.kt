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

import android.hardware.SensorEvent

/**
 * Converts [SensorEvent] data into an [MagnetometerSensorMeasurement].
 */
object MagnetometerSensorMeasurementConverter {

    /**
     * Converts [SensorEvent] data into an [MagnetometerSensorMeasurement] by setting proper values
     * into provided [result] parameter.
     *
     * @param event event to convert from.
     * @param result instance where result of conversion will be stored.
     * @param startOffset optional parameter to offset timestamps of measurements. If not provided,
     * no offset is applied, otherwise provided [startOffset] is added to [SensorEvent.timestamp].
     * @return true if result measurement has been updated, false if no event is provided or belongs
     * to a non-supported sensor type.
     */
    fun convert(
        event: SensorEvent?,
        result: MagnetometerSensorMeasurement,
        startOffset: Long? = null
    ): Boolean {
        if (event == null) {
            return false
        }
        val sensorType = MagnetometerSensorType.from(event.sensor.type) ?: return false

        val sensorAccuracy = SensorAccuracy.from(event.accuracy)
        val timestamp = event.timestamp

        val bx = event.values[0]
        val by = event.values[1]
        val bz = event.values[2]
        var hardIronX: Float? = null
        var hardIronY: Float? = null
        var hardIronZ: Float? = null
        if (sensorType == MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED) {
            hardIronX = event.values[3]
            hardIronY = event.values[4]
            hardIronZ = event.values[5]
        }

        // update result measurement values
        result.bx = bx
        result.by = by
        result.bz = bz
        result.hardIronX = hardIronX
        result.hardIronY = hardIronY
        result.hardIronZ = hardIronZ
        result.timestamp = timestamp + (startOffset ?: 0L)
        result.accuracy = sensorAccuracy
        return true
    }
}