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

package com.irurueta.android.navigation.inertial.collectors.measurements

import org.junit.Assert.*
import org.junit.Test

class SensorMeasurementCoordinateSystemConverterRequirementsTest {

    @Test
    fun requireENUSensorMeasurement_whenMeasurementIsENU_doesNotThrow() {
        val measurement =
            AccelerometerSensorMeasurement(sensorCoordinateSystem = SensorCoordinateSystem.ENU)
        SensorMeasurementCoordinateSystemConverterRequirements.requireENUSensorMeasurement(
            measurement
        )
    }

    @Test
    fun requireENUSensorMeasurement_whenMeasurementIsNotENU_throwsIllegalArgumentException() {
        val measurement =
            AccelerometerSensorMeasurement(sensorCoordinateSystem = SensorCoordinateSystem.NED)
        assertThrows(IllegalArgumentException::class.java) {
            SensorMeasurementCoordinateSystemConverterRequirements.requireENUSensorMeasurement(
                measurement
            )
        }
    }

    @Test
    fun requireNEDSensorMeasurement_whenMeasurementIsNED_doesNotThrow() {
        val measurement =
            AccelerometerSensorMeasurement(sensorCoordinateSystem = SensorCoordinateSystem.NED)
        SensorMeasurementCoordinateSystemConverterRequirements.requireNEDSensorMeasurement(
            measurement
        )
    }

    @Test
    fun requireNEDSensorMeasurement_whenMeasurementIsNotNED_throwsIllegalArgumentException() {
        val measurement =
            AccelerometerSensorMeasurement(sensorCoordinateSystem = SensorCoordinateSystem.ENU)
        assertThrows(IllegalArgumentException::class.java) {
            SensorMeasurementCoordinateSystemConverterRequirements.requireNEDSensorMeasurement(
                measurement
            )
        }
    }
}