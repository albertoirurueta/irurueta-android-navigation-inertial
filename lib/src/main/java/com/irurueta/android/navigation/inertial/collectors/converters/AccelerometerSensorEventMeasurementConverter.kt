/*
 * Copyright (C) 2025 Alberto Irurueta Carro (alberto@irurueta.com)
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

package com.irurueta.android.navigation.inertial.collectors.converters

import android.hardware.SensorEvent
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorCoordinateSystem

/**
 * Converts [android.hardware.SensorEvent] data into an
 * [com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement].
 */
object AccelerometerSensorEventMeasurementConverter {

    /**
     * Converts [android.hardware.SensorEvent] data into an
     * [com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement]
     * by setting proper values into provided [result] parameter.
     *
     * @param event event to convert from.
     * @param result instance where result of conversion will be stored.
     * @return true if result measurement has been updated, false if no event is provided or belongs
     * to a non-supported sensor type.
     */
    fun convert(
        event: SensorEvent?,
        result: AccelerometerSensorMeasurement
    ): Boolean {
        if (event == null) {
            return false
        }

        val sensorType = AccelerometerSensorType.from(event.sensor.type) ?: return false

        val sensorAccuracy = SensorAccuracy.from(event.accuracy)
        val timestamp = event.timestamp

        val ax = event.values[0]
        val ay = event.values[1]
        val az = event.values[2]
        var bx: Float? = null
        var by: Float? = null
        var bz: Float? = null
        if (sensorType == AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED) {
            bx = event.values[3]
            by = event.values[4]
            bz = event.values[5]
        }

        // update result measurement values
        result.ax = ax
        result.ay = ay
        result.az = az
        result.bx = bx
        result.by = by
        result.bz = bz
        result.timestamp = timestamp
        result.accuracy = sensorAccuracy
        result.sensorType = sensorType
        // Android sensors use ENU coordinate system
        result.sensorCoordinateSystem = SensorCoordinateSystem.ENU
        return true
    }

    /**
     * Converts [SensorEvent] data into a new instance of
     * [AccelerometerSensorMeasurement]
     *
     * @param event event to convert from
     * @return converted measurement or null if no sensor event or invalid one is provided.
     */
    fun convert(event: SensorEvent?): AccelerometerSensorMeasurement? {
        if (event == null) {
            return null
        }

        val measurement = AccelerometerSensorMeasurement()
        return if (convert(event, measurement)) {
            measurement
        } else {
            null
        }
    }
}