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

import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.statistics.UniformRandomizer
import io.mockk.clearAllMocks
import io.mockk.unmockkAll
import org.junit.After
import org.junit.Assert.*
import org.junit.Test

class GyroscopeDirectSensorMeasurementInterpolatorTest {

    @After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
    }

    @Test
    fun push_makesNoAction() {
        val interpolator = GyroscopeDirectSensorMeasurementInterpolator()

        val measurement = generateMeasurement()
        interpolator.push(measurement)
    }

    @Test
    fun interpolate_computesExpectedValue() {
        val interpolator = GyroscopeDirectSensorMeasurementInterpolator()

        val currentMeasurement = generateMeasurement()
        val result = GyroscopeSensorMeasurement()
        val timestamp = System.nanoTime()

        assertTrue(interpolator.interpolate(currentMeasurement, timestamp, result))

        assertNotSame(currentMeasurement, result)
        assertEquals(currentMeasurement.wx, result.wx, 0.0f)
        assertEquals(currentMeasurement.wy, result.wy, 0.0f)
        assertEquals(currentMeasurement.wz, result.wz, 0.0f)
        assertEquals(currentMeasurement.bx, result.bx)
        assertEquals(currentMeasurement.by, result.by)
        assertEquals(currentMeasurement.bz, result.bz)
        assertEquals(currentMeasurement.timestamp, result.timestamp)
        assertEquals(currentMeasurement.accuracy, result.accuracy)
    }

    @Test
    fun reset_makesNoAction() {
        val interpolator = GyroscopeDirectSensorMeasurementInterpolator()

        interpolator.reset()
    }

    private companion object {

        fun generateMeasurement(): GyroscopeSensorMeasurement {
            val randomizer = UniformRandomizer()
            val wx = randomizer.nextFloat()
            val wy = randomizer.nextFloat()
            val wz = randomizer.nextFloat()
            val bx = randomizer.nextFloat()
            val by = randomizer.nextFloat()
            val bz = randomizer.nextFloat()
            val timestamp = System.nanoTime()
            return GyroscopeSensorMeasurement(
                wx,
                wy,
                wz,
                bx,
                by,
                bz,
                timestamp,
                SensorAccuracy.HIGH
            )
        }
    }
}