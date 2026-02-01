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

import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement
import com.irurueta.statistics.UniformRandomizer
import org.junit.Assert.assertEquals
import org.junit.Test
import kotlin.math.sqrt

class AccelerometerSensorMeasurementNormConverterTest {

    @Test
    fun convert_whenNoBias_returnsExpectedNorm() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val measurement = AccelerometerSensorMeasurement(ax, ay, az)

        val norm = AccelerometerSensorMeasurementNormConverter.convert(measurement)

        val ax2 = ax.toDouble() * ax.toDouble()
        val ay2 = ay.toDouble() * ay.toDouble()
        val az2 = az.toDouble() * az.toDouble()
        val expectedNorm = sqrt(ax2 + ay2 + az2)
        assertEquals(expectedNorm, norm, 0.0)
    }

    @Test
    fun convert_whenBias_returnsExpectedNorm() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val measurement = AccelerometerSensorMeasurement(ax, ay, az, bx, by, bz)

        val norm = AccelerometerSensorMeasurementNormConverter.convert(measurement)

        val x = ax.toDouble() + bx.toDouble()
        val y = ay.toDouble() + by.toDouble()
        val z = az.toDouble() + bz.toDouble()
        val expectedNorm = sqrt(x * x + y * y + z * z)
        assertEquals(expectedNorm, norm, 0.0)
    }
}