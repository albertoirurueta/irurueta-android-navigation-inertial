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
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.AccelerationUnit
import org.junit.Assert.assertEquals
import org.junit.Test

class GravitySensorMeasurementTriadConverterTest {

    @Test
    fun convert_returnsExpectedTriad() {
        val randomize = UniformRandomizer()
        val gx = randomize.nextFloat()
        val gy = randomize.nextFloat()
        val gz = randomize.nextFloat()
        val measurement = GravitySensorMeasurement(gx, gy, gz)
        val triad1 = AccelerationTriad()
        GravitySensorMeasurementTriadConverter.convert(measurement, triad1)
        val triad2 = GravitySensorMeasurementTriadConverter.convert(measurement)

        assertEquals(gx.toDouble(), triad1.valueX, 0.0)
        assertEquals(gy.toDouble(), triad1.valueY, 0.0)
        assertEquals(gz.toDouble(), triad1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.unit)
        assertEquals(triad1, triad2)
    }
}