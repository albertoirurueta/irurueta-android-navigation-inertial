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

import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.statistics.UniformRandomizer
import org.junit.Assert.*
import org.junit.Test

class AccelerometerDirectSensorMeasurementInterpolatorTest {

    @Test
    fun constructor_createsInstance() {
        assertNotNull(AccelerometerDirectSensorMeasurementInterpolator())
    }

    @Test
    fun push_makesNoAction() {
        val interpolator = AccelerometerDirectSensorMeasurementInterpolator()

        val measurement = generateMeasurement()
        interpolator.push(measurement)
    }

    @Test
    fun interpolate_computesExpectedValue() {
        val interpolator = AccelerometerDirectSensorMeasurementInterpolator()

        val currentMeasurement = generateMeasurement()
        val result = AccelerometerSensorMeasurement()
        val timestamp = System.nanoTime()

        assertTrue(interpolator.interpolate(currentMeasurement, timestamp, result))

        assertNotSame(currentMeasurement, result)
        assertEquals(currentMeasurement.ax, result.ax, 0.0f)
        assertEquals(currentMeasurement.ay, result.ay, 0.0f)
        assertEquals(currentMeasurement.az, result.az, 0.0f)
        assertEquals(currentMeasurement.bx, result.bx)
        assertEquals(currentMeasurement.by, result.by)
        assertEquals(currentMeasurement.bz, result.bz)
        assertEquals(currentMeasurement.timestamp, result.timestamp)
        assertEquals(currentMeasurement.accuracy, result.accuracy)
    }

    @Test
    fun reset_makesNoAction() {
        val interpolator = AccelerometerDirectSensorMeasurementInterpolator()

        interpolator.reset()
    }

    private companion object {

        fun generateMeasurement(): AccelerometerSensorMeasurement {
            val randomizer = UniformRandomizer()
            val ax = randomizer.nextFloat()
            val ay = randomizer.nextFloat()
            val az = randomizer.nextFloat()
            val bx = randomizer.nextFloat()
            val by = randomizer.nextFloat()
            val bz = randomizer.nextFloat()
            val timestamp = System.nanoTime()
            return AccelerometerSensorMeasurement(
                ax,
                ay,
                az,
                bx,
                by,
                bz,
                timestamp,
                SensorAccuracy.HIGH
            )
        }
    }
}