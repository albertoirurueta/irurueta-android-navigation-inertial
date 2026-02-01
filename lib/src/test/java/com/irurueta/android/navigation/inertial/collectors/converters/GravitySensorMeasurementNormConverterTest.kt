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

import com.irurueta.android.navigation.inertial.collectors.measurements.GravitySensorMeasurement
import com.irurueta.statistics.UniformRandomizer
import org.junit.Assert.assertEquals
import org.junit.Test
import kotlin.math.sqrt

class GravitySensorMeasurementNormConverterTest {

    @Test
    fun convert_returnsExpectedNorm() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val measurement = GravitySensorMeasurement(gx, gy, gz)

        val norm = GravitySensorMeasurementNormConverter.convert(measurement)

        val expectedNorm = sqrt(
            gx.toDouble() * gx.toDouble()
                    + gy.toDouble() * gy.toDouble()
                    + gz.toDouble() * gz.toDouble()
        )
        assertEquals(expectedNorm, norm, 0.0)
    }
}