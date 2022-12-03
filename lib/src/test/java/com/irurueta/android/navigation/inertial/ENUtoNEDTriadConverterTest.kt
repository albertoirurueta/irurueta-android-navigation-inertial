/*
 * Copyright (C) 2022 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.android.navigation.inertial

import com.irurueta.algebra.Matrix
import com.irurueta.geometry.Quaternion
import com.irurueta.geometry.Rotation3D
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad
import com.irurueta.statistics.UniformRandomizer
import org.junit.Assert.*
import org.junit.Test
import kotlin.math.sqrt

class ENUtoNEDTriadConverterTest {

    @Test
    fun convert_whenAccelerationTriad_returnsExpectedResult() {
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()

        // convert from ENU to NED
        val enuTriad1 = AccelerationTriad(valueX, valueY, valueZ)
        val nedTriad = AccelerationTriad()
        ENUtoNEDTriadConverter.convert(enuTriad1, nedTriad)

        // check
        assertEquals(nedTriad.valueX, enuTriad1.valueY, 0.0)
        assertEquals(nedTriad.valueY, enuTriad1.valueX, 0.0)
        assertEquals(nedTriad.valueZ, -enuTriad1.valueZ, 0.0)

        // convert back from NED to ENU
        val enuTriad2 = AccelerationTriad()
        ENUtoNEDTriadConverter.convert(nedTriad, enuTriad2)

        // check
        assertEquals(enuTriad1, enuTriad2)

        // convert using rotation matrix
        val values = enuTriad1.valuesAsMatrix
        val rotation = ENUtoNEDTriadConverter.conversionRotationMatrix
        val expected = rotation.multiplyAndReturnNew(values)
        assertTrue(expected.equals(nedTriad.valuesAsMatrix, ABSOLUTE_ERROR))

        // convert back
        val values2 = rotation.multiplyAndReturnNew(expected)
        assertTrue(values2.equals(values, ABSOLUTE_ERROR))
    }

    @Test
    fun convert_whenAngularSpeedTriad_returnsExpectedResult() {
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()

        // convert from ENU to NED
        val enuTriad1 = AngularSpeedTriad(valueX, valueY, valueZ)
        val nedTriad = AngularSpeedTriad()
        ENUtoNEDTriadConverter.convert(enuTriad1, nedTriad)

        // check
        assertEquals(nedTriad.valueX, enuTriad1.valueY, 0.0)
        assertEquals(nedTriad.valueY, enuTriad1.valueX, 0.0)
        assertEquals(nedTriad.valueZ, -enuTriad1.valueZ, 0.0)

        // convert back from NED to ENU
        val enuTriad2 = AngularSpeedTriad()
        ENUtoNEDTriadConverter.convert(nedTriad, enuTriad2)

        // check
        assertEquals(enuTriad1, enuTriad2)

        // convert using rotation matrix
        val values = enuTriad1.valuesAsMatrix
        val rotation = ENUtoNEDTriadConverter.conversionRotationMatrix
        val expected = rotation.multiplyAndReturnNew(values)
        assertTrue(expected.equals(nedTriad.valuesAsMatrix, ABSOLUTE_ERROR))

        // convert back
        val values2 = rotation.multiplyAndReturnNew(expected)
        assertTrue(values2.equals(values, ABSOLUTE_ERROR))
    }

    @Test
    fun convert_whenMagneticFluxDensityTriad_returnsExpectedResult() {
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()

        // convert from ENU to NED
        val enuTriad1 = MagneticFluxDensityTriad(valueX, valueY, valueZ)
        val nedTriad = MagneticFluxDensityTriad()
        ENUtoNEDTriadConverter.convert(enuTriad1, nedTriad)

        // check
        assertEquals(nedTriad.valueX, enuTriad1.valueY, 0.0)
        assertEquals(nedTriad.valueY, enuTriad1.valueX, 0.0)
        assertEquals(nedTriad.valueZ, -enuTriad1.valueZ, 0.0)

        // convert back from NED to ENU
        val enuTriad2 = MagneticFluxDensityTriad()
        ENUtoNEDTriadConverter.convert(nedTriad, enuTriad2)

        // check
        assertEquals(enuTriad1, enuTriad2)

        // convert using rotation matrix
        val values = enuTriad1.valuesAsMatrix
        val rotation = ENUtoNEDTriadConverter.conversionRotationMatrix
        val expected = rotation.multiplyAndReturnNew(values)
        assertTrue(expected.equals(nedTriad.valuesAsMatrix, ABSOLUTE_ERROR))

        // convert back
        val values2 = rotation.multiplyAndReturnNew(expected)
        assertTrue(values2.equals(values, ABSOLUTE_ERROR))
    }

    @Test
    fun convert_whenAccelerationTriadCoordinates_returnsExpectedResult() {
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()

        // convert from ENU to NED
        val enuTriad1 = AccelerationTriad(valueX, valueY, valueZ)
        val nedTriad = AccelerationTriad()
        ENUtoNEDTriadConverter.convert(valueX, valueY, valueZ, nedTriad)

        // check
        assertEquals(nedTriad.valueX, enuTriad1.valueY, 0.0)
        assertEquals(nedTriad.valueY, enuTriad1.valueX, 0.0)
        assertEquals(nedTriad.valueZ, -enuTriad1.valueZ, 0.0)

        // convert back from NED to ENU
        val enuTriad2 = AccelerationTriad()
        ENUtoNEDTriadConverter.convert(nedTriad, enuTriad2)

        // check
        assertEquals(enuTriad1, enuTriad2)

        // convert using rotation matrix
        val values = enuTriad1.valuesAsMatrix
        val rotation = ENUtoNEDTriadConverter.conversionRotationMatrix
        val expected = rotation.multiplyAndReturnNew(values)
        assertTrue(expected.equals(nedTriad.valuesAsMatrix, ABSOLUTE_ERROR))

        // convert back
        val values2 = rotation.multiplyAndReturnNew(expected)
        assertTrue(values2.equals(values, ABSOLUTE_ERROR))
    }

    @Test
    fun convert_whenAngularSpeedTriadCoordinates_returnsExpectedResult() {
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()

        // convert from ENU to NED
        val enuTriad1 = AngularSpeedTriad(valueX, valueY, valueZ)
        val nedTriad = AngularSpeedTriad()
        ENUtoNEDTriadConverter.convert(valueX, valueY, valueZ, nedTriad)

        // check
        assertEquals(nedTriad.valueX, enuTriad1.valueY, 0.0)
        assertEquals(nedTriad.valueY, enuTriad1.valueX, 0.0)
        assertEquals(nedTriad.valueZ, -enuTriad1.valueZ, 0.0)

        // convert back from NED to ENU
        val enuTriad2 = AngularSpeedTriad()
        ENUtoNEDTriadConverter.convert(nedTriad, enuTriad2)

        // check
        assertEquals(enuTriad1, enuTriad2)

        // convert using rotation matrix
        val values = enuTriad1.valuesAsMatrix
        val rotation = ENUtoNEDTriadConverter.conversionRotationMatrix
        val expected = rotation.multiplyAndReturnNew(values)
        assertTrue(expected.equals(nedTriad.valuesAsMatrix, ABSOLUTE_ERROR))

        // convert back
        val values2 = rotation.multiplyAndReturnNew(expected)
        assertTrue(values2.equals(values, ABSOLUTE_ERROR))
    }

    @Test
    fun convert_whenMagneticFluxDensityTriadCoordinates_returnsExpectedResult() {
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()

        // convert from ENU to NED
        val enuTriad1 = MagneticFluxDensityTriad(valueX, valueY, valueZ)
        val nedTriad = MagneticFluxDensityTriad()
        ENUtoNEDTriadConverter.convert(valueX, valueY, valueZ, nedTriad)

        // check
        assertEquals(nedTriad.valueX, enuTriad1.valueY, 0.0)
        assertEquals(nedTriad.valueY, enuTriad1.valueX, 0.0)
        assertEquals(nedTriad.valueZ, -enuTriad1.valueZ, 0.0)

        // convert back from NED to ENU
        val enuTriad2 = MagneticFluxDensityTriad()
        ENUtoNEDTriadConverter.convert(nedTriad, enuTriad2)

        // check
        assertEquals(enuTriad1, enuTriad2)

        // convert using rotation matrix
        val values = enuTriad1.valuesAsMatrix
        val rotation = ENUtoNEDTriadConverter.conversionRotationMatrix
        val expected = rotation.multiplyAndReturnNew(values)
        assertTrue(expected.equals(nedTriad.valuesAsMatrix, ABSOLUTE_ERROR))

        // convert back
        val values2 = rotation.multiplyAndReturnNew(expected)
        assertTrue(values2.equals(values, ABSOLUTE_ERROR))
    }

    @Test
    fun conversionRotation_returnsExpectedResult() {
        val rotation1 = Quaternion()
        ENUtoNEDTriadConverter.conversionRotation(rotation1)
        val rotation2 = ENUtoNEDTriadConverter.conversionRotation

        assertEquals(rotation1, rotation2)

        val matrix1 = Matrix(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)
        matrix1.setElementAt(0, 1, 1.0)
        matrix1.setElementAt(1, 0, 1.0)
        matrix1.setElementAt(2, 2, -1.0)

        val matrix2 = rotation1.asInhomogeneousMatrix()
        assertTrue(matrix1.equals(matrix2, ABSOLUTE_ERROR))

        assertEquals(0.0, rotation1.a, 0.0)
        assertEquals(-1.0 / sqrt(2.0), rotation1.b, 0.0)
        assertEquals(-1.0 / sqrt(2.0), rotation1.c, 0.0)
        assertEquals(0.0, rotation1.d, 0.0)

        val angles = rotation1.toEulerAngles()
        assertEquals(-Math.PI, angles[0], 0.0)
        assertEquals(0.0, angles[1], 0.0)
        assertEquals(Math.PI / 2.0, angles[2], 0.0)
    }

    @Test
    fun conversionRotationMatrix_whenValidSize_returnsExpectedResult() {
        val m1 = Matrix(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)
        ENUtoNEDTriadConverter.conversionRotationMatrix(m1)
        val m2 = ENUtoNEDTriadConverter.conversionRotationMatrix

        assertEquals(m1, m2)

        val m3 = Matrix(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)
        m3.setElementAt(0, 1, 1.0)
        m3.setElementAt(1, 0, 1.0)
        m3.setElementAt(2, 2, -1.0)

        assertEquals(m1, m3)

        val m4 = ENUtoNEDTriadConverter.conversionRotation.asInhomogeneousMatrix()
        assertTrue(m1.equals(m4, ABSOLUTE_ERROR))
    }

    @Test
    fun conversionRotationMatrix_whenInvalidRows_returnsExpectedResult() {
        val m1 = Matrix(1, Rotation3D.INHOM_COORDS)
        ENUtoNEDTriadConverter.conversionRotationMatrix(m1)
        val m2 = ENUtoNEDTriadConverter.conversionRotationMatrix

        assertEquals(m1, m2)

        val m3 = Matrix(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)
        m3.setElementAt(0, 1, 1.0)
        m3.setElementAt(1, 0, 1.0)
        m3.setElementAt(2, 2, -1.0)

        assertEquals(m1, m3)

        val m4 = ENUtoNEDTriadConverter.conversionRotation.asInhomogeneousMatrix()
        assertTrue(m1.equals(m4, ABSOLUTE_ERROR))
    }

    @Test
    fun conversionRotationMatrix_whenInvalidColumns_returnsExpectedResult() {
        val m1 = Matrix(Rotation3D.INHOM_COORDS, 1)
        ENUtoNEDTriadConverter.conversionRotationMatrix(m1)
        val m2 = ENUtoNEDTriadConverter.conversionRotationMatrix

        assertEquals(m1, m2)

        val m3 = Matrix(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)
        m3.setElementAt(0, 1, 1.0)
        m3.setElementAt(1, 0, 1.0)
        m3.setElementAt(2, 2, -1.0)

        assertEquals(m1, m3)

        val m4 = ENUtoNEDTriadConverter.conversionRotation.asInhomogeneousMatrix()
        assertTrue(m1.equals(m4, ABSOLUTE_ERROR))
    }

    private companion object {
        const val ABSOLUTE_ERROR = 1e-11
    }
}