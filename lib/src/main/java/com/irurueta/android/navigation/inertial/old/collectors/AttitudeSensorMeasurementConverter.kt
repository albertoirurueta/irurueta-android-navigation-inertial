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
package com.irurueta.android.navigation.inertial.old.collectors

import android.hardware.SensorEvent
import com.irurueta.android.navigation.inertial.collectors.measurements.AttitudeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AttitudeSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy

/**
 * Converts [SensorEvent] data into an [AttitudeSensorMeasurementConverter].
 */
object AttitudeSensorMeasurementConverter {

    /**
     * Converts [SensorEvent] data into an [com.irurueta.android.navigation.inertial.collectors.measurements.AttitudeSensorMeasurement] by setting proper values
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
        result: AttitudeSensorMeasurement,
        startOffset: Long? = null
    ): Boolean {
        if (event == null) {
            return false
        }

        val sensorType = AttitudeSensorType.from(event.sensor.type) ?: return false

        val sensorAccuracy = SensorAccuracy.from(event.accuracy)
        val timestamp = event.timestamp

        result.attitude.b = event.values[0].toDouble()
        result.attitude.c = event.values[1].toDouble()
        result.attitude.d = event.values[2].toDouble()
        result.attitude.a = event.values[3].toDouble()
        result.attitude.normalize()

        if (event.values.size > 4) {
            result.headingAccuracy = event.values[4]
        }

        result.timestamp = timestamp + (startOffset ?: 0L)
        result.accuracy = sensorAccuracy
        result.sensorType = sensorType
        return true
    }
}