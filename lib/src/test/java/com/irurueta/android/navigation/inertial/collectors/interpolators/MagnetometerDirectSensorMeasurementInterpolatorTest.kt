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
package com.irurueta.android.navigation.inertial.collectors.interpolators

import com.irurueta.android.navigation.inertial.collectors.MagnetometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.statistics.UniformRandomizer
import io.mockk.clearAllMocks
import io.mockk.unmockkAll
import org.junit.After
import org.junit.Assert.*
import org.junit.Test

class MagnetometerDirectSensorMeasurementInterpolatorTest {

    @After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
    }

    @Test
    fun constructor_createsInstance() {
        assertNotNull(MagnetometerDirectSensorMeasurementInterpolator())
    }

    @Test
    fun push_makesNoAction() {
        val interpolator = MagnetometerDirectSensorMeasurementInterpolator()

        val measurement = generateMeasurement()
        interpolator.push(measurement)
    }

    @Test
    fun interpolate_computesExpectedValue() {
        val interpolator = MagnetometerDirectSensorMeasurementInterpolator()

        val currentMeasurement = generateMeasurement()
        val result = MagnetometerSensorMeasurement()
        val timestamp = System.nanoTime()

        assertTrue(interpolator.interpolate(currentMeasurement, timestamp, result))

        assertNotSame(currentMeasurement, result)
        assertEquals(currentMeasurement.bx, result.bx, 0.0f)
        assertEquals(currentMeasurement.by, result.by, 0.0f)
        assertEquals(currentMeasurement.bz, result.bz, 0.0f)
        assertEquals(currentMeasurement.hardIronX, result.hardIronX)
        assertEquals(currentMeasurement.hardIronY, result.hardIronY)
        assertEquals(currentMeasurement.hardIronZ, result.hardIronZ)
        assertEquals(currentMeasurement.timestamp, result.timestamp)
        assertEquals(currentMeasurement.accuracy, result.accuracy)
    }

    @Test
    fun reset_makesNoAction() {
        val interpolator = MagnetometerDirectSensorMeasurementInterpolator()

        interpolator.reset()
    }

    private companion object {

        fun generateMeasurement(): MagnetometerSensorMeasurement {
            val randomizer = UniformRandomizer()
            val bx = randomizer.nextFloat()
            val by = randomizer.nextFloat()
            val bz = randomizer.nextFloat()
            val hardIronX = randomizer.nextFloat()
            val hardIronY = randomizer.nextFloat()
            val hardIronZ = randomizer.nextFloat()
            val timestamp = System.nanoTime()
            return MagnetometerSensorMeasurement(
                bx,
                by,
                bz,
                hardIronX,
                hardIronY,
                hardIronZ,
                timestamp,
                SensorAccuracy.LOW
            )
        }
    }
}