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
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.AngularSpeedUnit
import org.junit.Assert.assertEquals
import org.junit.Test

class GyroscopeSensorMeasurementTriadConverterTest {

    @Test
    fun convert_whenNoBias_returnsExpectedTriad() {
        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val measurement = GyroscopeSensorMeasurement(wx, wy, wz)
        val triad1 = GyroscopeSensorMeasurementTriadConverter.convert(measurement)
        val triad2 = AngularSpeedTriad()
        GyroscopeSensorMeasurementTriadConverter.convert(measurement, triad2)

        assertEquals(wx.toDouble(), triad1.valueX, 0.0)
        assertEquals(wy.toDouble(), triad1.valueY, 0.0)
        assertEquals(wz.toDouble(), triad1.valueZ, 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad1.unit)
        assertEquals(triad1, triad2)
    }

    @Test
    fun convert_whenBias_returnsExpectedTriad() {
        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val measurement = GyroscopeSensorMeasurement(wx, wy, wz, bx, by, bz)
        val triad1 = GyroscopeSensorMeasurementTriadConverter.convert(measurement)
        val triad2 = AngularSpeedTriad()
        GyroscopeSensorMeasurementTriadConverter.convert(measurement, triad2)

        assertEquals(wx.toDouble() + bx.toDouble(), triad1.valueX, 0.0)
        assertEquals(wy.toDouble() + by.toDouble(), triad1.valueY, 0.0)
        assertEquals(wz.toDouble() + bz.toDouble(), triad1.valueZ, 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad1.unit)
        assertEquals(triad1, triad2)
    }
}