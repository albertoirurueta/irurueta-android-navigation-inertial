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

import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorMeasurement
import com.irurueta.statistics.UniformRandomizer
import org.junit.Assert.assertEquals
import org.junit.Test
import kotlin.math.sqrt

class GyroscopeSensorMeasurementNormConverterTest {

    @Test
    fun convert_whenNoBias_returnsExpectedNorm() {
        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val measurement = GyroscopeSensorMeasurement(wx, wy, wz)

        val norm = GyroscopeSensorMeasurementNormConverter.convert(measurement)

        val wx2 = wx.toDouble() * wx.toDouble()
        val wy2 = wy.toDouble() * wy.toDouble()
        val wz2 = wz.toDouble() * wz.toDouble()
        val expectedNorm = sqrt(wx2 + wy2 + wz2)
        assertEquals(expectedNorm, norm, 0.0)
    }

    @Test
    fun convert_whenBias_returnsExpectedNorm() {
        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val measurement = GyroscopeSensorMeasurement(wx, wy, wz, bx, by, bz)

        val norm = GyroscopeSensorMeasurementNormConverter.convert(measurement)

        val x = wx.toDouble() + bx.toDouble()
        val y = wy.toDouble() + by.toDouble()
        val z = wz.toDouble() + bz.toDouble()
        val expectedNorm = sqrt(x * x + y * y + z * z)
        assertEquals(expectedNorm, norm, 0.0)
    }
}