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
package com.irurueta.android.navigation.inertial.old.collectors.interpolators

import com.irurueta.android.navigation.inertial.collectors.measurements.GravitySensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.statistics.UniformRandomizer
import org.junit.Assert.*
import org.junit.Test

class GravityDirectSensorMeasurementInterpolatorTest {

    @Test
    fun constructor_createsInstance() {
        assertNotNull(GravityDirectSensorMeasurementInterpolator())
    }

    @Test
    fun push_makesNoAction() {
        val interpolator = GravityDirectSensorMeasurementInterpolator()

        val measurement = generateMeasurement()
        interpolator.push(measurement)
    }

    @Test
    fun interpolate_computesExpectedValue() {
        val interpolator = GravityDirectSensorMeasurementInterpolator()

        val currentMeasurement = generateMeasurement()
        val result = GravitySensorMeasurement()
        val timestamp = System.nanoTime()

        assertTrue(interpolator.interpolate(currentMeasurement, timestamp, result))

        assertNotSame(currentMeasurement, result)
        assertEquals(currentMeasurement.gx, result.gx, 0.0f)
        assertEquals(currentMeasurement.gy, result.gy, 0.0f)
        assertEquals(currentMeasurement.gz, result.gz, 0.0f)
        assertEquals(currentMeasurement.timestamp, result.timestamp)
        assertEquals(currentMeasurement.accuracy, result.accuracy)
    }

    @Test
    fun reset_makesNoAction() {
        val interpolator = GravityDirectSensorMeasurementInterpolator()

        interpolator.reset()
    }

    private companion object {

        fun generateMeasurement(): GravitySensorMeasurement {
            val randomizer = UniformRandomizer()
            val gx = randomizer.nextFloat()
            val gy = randomizer.nextFloat()
            val gz = randomizer.nextFloat()
            val timestamp = System.nanoTime()
            return GravitySensorMeasurement(gx, gy, gz, timestamp, SensorAccuracy.MEDIUM)
        }
    }
}