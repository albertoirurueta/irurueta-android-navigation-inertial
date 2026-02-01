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

import android.hardware.Sensor
import android.hardware.SensorEvent
import com.irurueta.android.navigation.inertial.collectors.measurements.GravitySensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorCoordinateSystem

/**
 * Converts [android.hardware.SensorEvent] data into an [com.irurueta.android.navigation.inertial.collectors.measurements.GravitySensorMeasurement].
 */
object GravitySensorEventMeasurementConverter {
    /**
     * Converts [android.hardware.SensorEvent] data into a [com.irurueta.android.navigation.inertial.collectors.measurements.GravitySensorMeasurement] by setting proper values
     * into provided [result] parameter.
     *
     * @param event event to convert from.
     * @param result instance where result of conversion will be stored.
     * @return true if result measurement has been updated, false if no event is provided or belongs
     * to a non-supported sensor type.
     */
    fun convert(
        event: SensorEvent?,
        result: GravitySensorMeasurement
    ): Boolean {
        if (event == null) {
            return false
        }
        if (event.sensor.type != Sensor.TYPE_GRAVITY) {
            return false
        }

        val sensorAccuracy = SensorAccuracy.from(event.accuracy)
        val timestamp = event.timestamp

        val gx = event.values[0]
        val gy = event.values[1]
        val gz = event.values[2]

        // update result measurement values
        result.gx = gx
        result.gy = gy
        result.gz = gz
        result.timestamp = timestamp
        result.accuracy = sensorAccuracy
        // Android sensors use ENU coordinate system
        result.sensorCoordinateSystem = SensorCoordinateSystem.ENU
        return true
    }

    /**
     * Converts [SensorEvent] data into a new instance of
     * [GravitySensorMeasurement]
     *
     * @param event event to convert from
     * @return converted measurement or null if no sensor event or invalid one is provided.
     */
    fun convert(event: SensorEvent?): GravitySensorMeasurement? {
        if (event == null) {
            return null
        }

        val measurement = GravitySensorMeasurement()
        return if (convert(event, measurement)) {
            measurement
        } else {
            null
        }
    }
}