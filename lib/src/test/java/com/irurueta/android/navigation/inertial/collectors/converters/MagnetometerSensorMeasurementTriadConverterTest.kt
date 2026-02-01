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
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.MagneticFluxDensityUnit
import org.junit.Assert.assertEquals
import org.junit.Test

class MagnetometerSensorMeasurementTriadConverterTest {

    @Test
    fun convert_whenNoBias_returnsExpectedTriad() {
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val measurement = MagnetometerSensorMeasurement(bx, by, bz)
        val triad1 = MagneticFluxDensityTriad()
        MagnetometerSensorMeasurementTriadConverter.convert(measurement, triad1)
        val triad2 = MagnetometerSensorMeasurementTriadConverter.convert(measurement)

        assertEquals(bx.toDouble(), triad1.valueX, 0.0)
        assertEquals(by.toDouble(), triad1.valueY, 0.0)
        assertEquals(bz.toDouble(), triad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.MICROTESLA, triad1.unit)
        assertEquals(triad1, triad2)
    }

    @Test
    fun convert_whenBias_returnsExpectedTriad() {
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val measurement = MagnetometerSensorMeasurement(bx, by, bz, hardIronX, hardIronY, hardIronZ)
        val triad1 = MagneticFluxDensityTriad()
        MagnetometerSensorMeasurementTriadConverter.convert(measurement, triad1)
        val triad2 = MagnetometerSensorMeasurementTriadConverter.convert(measurement)

        assertEquals(
            bx.toDouble() + hardIronX.toDouble(), triad1.valueX,
            0.0
        )
        assertEquals(
            by.toDouble() + hardIronY.toDouble(), triad1.valueY,
            0.0
        )
        assertEquals(
            bz.toDouble() + hardIronZ.toDouble(), triad1.valueZ,
            0.0
        )
        assertEquals(MagneticFluxDensityUnit.MICROTESLA, triad1.unit)
        assertEquals(triad1, triad2)
    }
}