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

import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorMeasurement
import com.irurueta.statistics.UniformRandomizer
import org.junit.Assert.assertEquals
import org.junit.Test
import kotlin.math.sqrt

class MagnetometerSensorMeasurementNormConverterTest {

    @Test
    fun convert_whenNoBias_returnsExpectedNorm() {
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val measurement = MagnetometerSensorMeasurement(bx, by, bz)

        val norm = MagnetometerSensorMeasurementNormConverter.convert(measurement)

        val bx2 = bx.toDouble() * bx.toDouble()
        val by2 = by.toDouble() * by.toDouble()
        val bz2 = bz.toDouble() * bz.toDouble()
        val expectedNorm = sqrt(bx2 + by2 + bz2)
        assertEquals(expectedNorm, norm, 0.0)
    }

    @Test
    fun convert_whenBias_returnsExpectedNorm() {
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val measurement = MagnetometerSensorMeasurement(bx, by, bz, hardIronX, hardIronY, hardIronZ)

        val norm = MagnetometerSensorMeasurementNormConverter.convert(measurement)

        val x = bx.toDouble() + hardIronX.toDouble()
        val y = by.toDouble() + hardIronY.toDouble()
        val z = bz.toDouble() + hardIronZ.toDouble()
        val expectedNorm = sqrt(x * x + y * y + z * z)
        assertEquals(expectedNorm, norm, 0.0)
    }
}