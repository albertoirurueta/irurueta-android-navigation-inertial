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
import com.irurueta.geometry.EuclideanTransformation3D
import com.irurueta.geometry.InhomogeneousPoint3D
import com.irurueta.geometry.Point3D
import com.irurueta.geometry.Quaternion
import com.irurueta.geometry.Rotation3D
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegrator
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegratorType
import com.irurueta.numerical.JacobianEstimator
import com.irurueta.numerical.MultiVariateFunctionEvaluatorListener
import com.irurueta.statistics.UniformRandomizer
import io.mockk.clearAllMocks
import io.mockk.unmockkAll
import org.junit.After
import org.junit.Assert.*
import org.junit.Test
import kotlin.math.sqrt

class ENUtoNEDConverterTest {

    @After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
    }

    @Test
    fun convert_whenAccelerationTriad_returnsExpectedResult() {
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()

        // convert from ENU to NED
        val enuTriad1 = AccelerationTriad(valueX, valueY, valueZ)
        val nedTriad = AccelerationTriad()
        ENUtoNEDConverter.convert(enuTriad1, nedTriad)

        // check
        assertEquals(nedTriad.valueX, enuTriad1.valueY, 0.0)
        assertEquals(nedTriad.valueY, enuTriad1.valueX, 0.0)
        assertEquals(nedTriad.valueZ, -enuTriad1.valueZ, 0.0)
        assertEquals(nedTriad.unit, enuTriad1.unit)

        // convert back from NED to ENU
        val enuTriad2 = AccelerationTriad()
        ENUtoNEDConverter.convert(nedTriad, enuTriad2)

        // check
        assertEquals(enuTriad1, enuTriad2)

        // convert using rotation matrix
        val values = enuTriad1.valuesAsMatrix
        val rotation = ENUtoNEDConverter.conversionRotationMatrix
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
        val nedTriad1 = AngularSpeedTriad()
        ENUtoNEDConverter.convert(enuTriad1, nedTriad1)

        // check
        assertEquals(nedTriad1.valueX, enuTriad1.valueY, 0.0)
        assertEquals(nedTriad1.valueY, enuTriad1.valueX, 0.0)
        assertEquals(nedTriad1.valueZ, -enuTriad1.valueZ, 0.0)
        assertEquals(nedTriad1.unit, enuTriad1.unit)

        // convert back from NED to ENU
        val enuTriad2 = AngularSpeedTriad()
        ENUtoNEDConverter.convert(nedTriad1, enuTriad2)

        // check
        assertEquals(enuTriad1, enuTriad2)

        // convert using rotation matrix
        val values = enuTriad1.valuesAsMatrix
        val rotation = ENUtoNEDConverter.conversionRotationMatrix
        val expected = rotation.multiplyAndReturnNew(values)
        assertTrue(expected.equals(nedTriad1.valuesAsMatrix, ABSOLUTE_ERROR))
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
        ENUtoNEDConverter.convert(enuTriad1, nedTriad)

        // check
        assertEquals(nedTriad.valueX, enuTriad1.valueY, 0.0)
        assertEquals(nedTriad.valueY, enuTriad1.valueX, 0.0)
        assertEquals(nedTriad.valueZ, -enuTriad1.valueZ, 0.0)
        assertEquals(nedTriad.unit, enuTriad1.unit)

        // convert back from NED to ENU
        val enuTriad2 = MagneticFluxDensityTriad()
        ENUtoNEDConverter.convert(nedTriad, enuTriad2)

        // check
        assertEquals(enuTriad1, enuTriad2)

        // convert using rotation matrix
        val values = enuTriad1.valuesAsMatrix
        val rotation = ENUtoNEDConverter.conversionRotationMatrix
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
        ENUtoNEDConverter.convert(valueX, valueY, valueZ, nedTriad)

        // check
        assertEquals(nedTriad.valueX, enuTriad1.valueY, 0.0)
        assertEquals(nedTriad.valueY, enuTriad1.valueX, 0.0)
        assertEquals(nedTriad.valueZ, -enuTriad1.valueZ, 0.0)

        // convert back from NED to ENU
        val enuTriad2 = AccelerationTriad()
        ENUtoNEDConverter.convert(nedTriad, enuTriad2)

        // check
        assertEquals(enuTriad1, enuTriad2)

        // convert using rotation matrix
        val values = enuTriad1.valuesAsMatrix
        val rotation = ENUtoNEDConverter.conversionRotationMatrix
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
        val nedTriad1 = AngularSpeedTriad()
        ENUtoNEDConverter.convert(valueX, valueY, valueZ, nedTriad1)

        // check
        assertEquals(nedTriad1.valueX, enuTriad1.valueY, 0.0)
        assertEquals(nedTriad1.valueY, enuTriad1.valueX, 0.0)
        assertEquals(nedTriad1.valueZ, -enuTriad1.valueZ, 0.0)

        // convert back from NED to ENU
        val enuTriad2 = AngularSpeedTriad()
        ENUtoNEDConverter.convert(nedTriad1, enuTriad2)

        // check
        assertEquals(enuTriad1, enuTriad2)

        // convert using rotation matrix
        val values = enuTriad1.valuesAsMatrix
        val rotation = ENUtoNEDConverter.conversionRotationMatrix
        val expected = rotation.multiplyAndReturnNew(values)
        assertTrue(expected.equals(nedTriad1.valuesAsMatrix, ABSOLUTE_ERROR))
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
        ENUtoNEDConverter.convert(valueX, valueY, valueZ, nedTriad)

        // check
        assertEquals(nedTriad.valueX, enuTriad1.valueY, 0.0)
        assertEquals(nedTriad.valueY, enuTriad1.valueX, 0.0)
        assertEquals(nedTriad.valueZ, -enuTriad1.valueZ, 0.0)

        // convert back from NED to ENU
        val enuTriad2 = MagneticFluxDensityTriad()
        ENUtoNEDConverter.convert(nedTriad, enuTriad2)

        // check
        assertEquals(enuTriad1, enuTriad2)

        // convert using rotation matrix
        val values = enuTriad1.valuesAsMatrix
        val rotation = ENUtoNEDConverter.conversionRotationMatrix
        val expected = rotation.multiplyAndReturnNew(values)
        assertTrue(expected.equals(nedTriad.valuesAsMatrix, ABSOLUTE_ERROR))

        // convert back
        val values2 = rotation.multiplyAndReturnNew(expected)
        assertTrue(values2.equals(values, ABSOLUTE_ERROR))
    }

    @Test
    fun convertAndReturn_whenAccelerationTriad_returnsExpectedResult() {
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()

        // convert from ENU to NED
        val enuTriad1 = AccelerationTriad(valueX, valueY, valueZ)
        val nedTriad = ENUtoNEDConverter.convertAndReturnNew(enuTriad1)

        // check
        assertEquals(nedTriad.valueX, enuTriad1.valueY, 0.0)
        assertEquals(nedTriad.valueY, enuTriad1.valueX, 0.0)
        assertEquals(nedTriad.valueZ, -enuTriad1.valueZ, 0.0)

        // convert back from NED to ENU
        val enuTriad2 = ENUtoNEDConverter.convertAndReturnNew(nedTriad)

        // check
        assertEquals(enuTriad1, enuTriad2)

        // convert using rotation matrix
        val values = enuTriad1.valuesAsMatrix
        val rotation = ENUtoNEDConverter.conversionRotationMatrix
        val expected = rotation.multiplyAndReturnNew(values)
        assertTrue(expected.equals(nedTriad.valuesAsMatrix, ABSOLUTE_ERROR))

        // convert back
        val values2 = rotation.multiplyAndReturnNew(expected)
        assertTrue(values2.equals(values, ABSOLUTE_ERROR))
    }

    @Test
    fun convertAndReturn_whenAngularSpeedTriad_returnsExpectedResult() {
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()

        // convert from ENU to NED
        val enuTriad1 = AngularSpeedTriad(valueX, valueY, valueZ)
        val nedTriad1 = ENUtoNEDConverter.convertAndReturnNew(enuTriad1)

        // check
        assertEquals(nedTriad1.valueX, enuTriad1.valueY, 0.0)
        assertEquals(nedTriad1.valueY, enuTriad1.valueX, 0.0)
        assertEquals(nedTriad1.valueZ, -enuTriad1.valueZ, 0.0)

        // convert back from NED to ENU
        val enuTriad2 = ENUtoNEDConverter.convertAndReturnNew(nedTriad1)

        // check
        assertEquals(enuTriad1, enuTriad2)

        // convert using rotation matrix
        val values = enuTriad1.valuesAsMatrix
        val rotation = ENUtoNEDConverter.conversionRotationMatrix
        val expected = rotation.multiplyAndReturnNew(values)
        assertTrue(expected.equals(nedTriad1.valuesAsMatrix, ABSOLUTE_ERROR))
    }

    @Test
    fun convertAndReturn_whenMagneticFluxDensityTriad_returnsExpectedResult() {
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()

        // convert from ENU to NED
        val enuTriad1 = MagneticFluxDensityTriad(valueX, valueY, valueZ)
        val nedTriad = ENUtoNEDConverter.convertAndReturnNew(enuTriad1)

        // check
        assertEquals(nedTriad.valueX, enuTriad1.valueY, 0.0)
        assertEquals(nedTriad.valueY, enuTriad1.valueX, 0.0)
        assertEquals(nedTriad.valueZ, -enuTriad1.valueZ, 0.0)

        // convert back from NED to ENU
        val enuTriad2 = ENUtoNEDConverter.convertAndReturnNew(nedTriad)

        // check
        assertEquals(enuTriad1, enuTriad2)

        // convert using rotation matrix
        val values = enuTriad1.valuesAsMatrix
        val rotation = ENUtoNEDConverter.conversionRotationMatrix
        val expected = rotation.multiplyAndReturnNew(values)
        assertTrue(expected.equals(nedTriad.valuesAsMatrix, ABSOLUTE_ERROR))

        // convert back
        val values2 = rotation.multiplyAndReturnNew(expected)
        assertTrue(values2.equals(values, ABSOLUTE_ERROR))
    }

    @Test
    fun conversionRotation_returnsExpectedResult() {
        val rotation1 = Quaternion()
        ENUtoNEDConverter.conversionRotation(rotation1)
        val rotation2 = ENUtoNEDConverter.conversionRotation

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
        ENUtoNEDConverter.conversionRotationMatrix(m1)
        val m2 = ENUtoNEDConverter.conversionRotationMatrix

        assertEquals(m1, m2)

        val m3 = Matrix(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)
        m3.setElementAt(0, 1, 1.0)
        m3.setElementAt(1, 0, 1.0)
        m3.setElementAt(2, 2, -1.0)

        assertEquals(m1, m3)

        val m4 = ENUtoNEDConverter.conversionRotation.asInhomogeneousMatrix()
        assertTrue(m1.equals(m4, ABSOLUTE_ERROR))
    }

    @Test
    fun conversionRotationMatrix_whenInvalidRows_returnsExpectedResult() {
        val m1 = Matrix(1, Rotation3D.INHOM_COORDS)
        ENUtoNEDConverter.conversionRotationMatrix(m1)
        val m2 = ENUtoNEDConverter.conversionRotationMatrix

        assertEquals(m1, m2)

        val m3 = Matrix(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)
        m3.setElementAt(0, 1, 1.0)
        m3.setElementAt(1, 0, 1.0)
        m3.setElementAt(2, 2, -1.0)

        assertEquals(m1, m3)

        val m4 = ENUtoNEDConverter.conversionRotation.asInhomogeneousMatrix()
        assertTrue(m1.equals(m4, ABSOLUTE_ERROR))
    }

    @Test
    fun conversionRotationMatrix_whenInvalidColumns_returnsExpectedResult() {
        val m1 = Matrix(Rotation3D.INHOM_COORDS, 1)
        ENUtoNEDConverter.conversionRotationMatrix(m1)
        val m2 = ENUtoNEDConverter.conversionRotationMatrix

        assertEquals(m1, m2)

        val m3 = Matrix(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)
        m3.setElementAt(0, 1, 1.0)
        m3.setElementAt(1, 0, 1.0)
        m3.setElementAt(2, 2, -1.0)

        assertEquals(m1, m3)

        val m4 = ENUtoNEDConverter.conversionRotation.asInhomogeneousMatrix()
        assertTrue(m1.equals(m4, ABSOLUTE_ERROR))
    }

    @Test
    fun convert_whenRotationWithoutJacobian_returnsExpectedResult() {
        // Xenu =  [0   1   0 ] * Xned
        //         [1   0   0 ]
        //	       [0   0   -1]

        // Xned =  [0   1   0 ] * Xenu
        //         [1   0   0 ]
        //         [0   0   -1]

        // C = 	[0  1   0 ]
        //      [1  0   0 ]
        //      [0  0   -1]

        // Notice that	conversion = C, and C * C = I -> C = C^-1

        // Xrotated_ned = Rned * Xned = Rned * C * Xenu
        // Xrotated_enu = Renu * Xenu = Renu * C * Xned

        // Xrotated_enu = C * Xrotated_ned = C * Rned * C * Xenu = Renu * Xenu
        // Xrotated_ned = C * Xrotated_enu = C * Renu * C * Xned = Rned * Xned

        // Hence:
        // Renu = C * Rned * C
        // Rned = C * Renu * C
        // Consequently, angular speed triads should be directly converted, instead only rotations
        // should be converted
        val conversionQ = ENUtoNEDConverter.conversionRotation

        val enuInitQ = getQuaternion()
        val nedInitQ1 = conversionQ.multiplyAndReturnNew(enuInitQ).multiplyAndReturnNew(conversionQ)
        nedInitQ1.normalize()

        val nedInitQ2 = Quaternion()
        ENUtoNEDConverter.convert(enuInitQ, nedInitQ2)

        assertEquals(nedInitQ1, nedInitQ2)

        val randomizer = UniformRandomizer()
        // randomly define angular speed and time interval
        val wx = randomizer.nextDouble()
        val wy = randomizer.nextDouble()
        val wz = randomizer.nextDouble()
        val dt = randomizer.nextDouble()

        val enuRoll = wx * dt
        val enuPitch = wy * dt
        val enuYaw = wz * dt
        val enuDeltaQ = Quaternion(enuRoll, enuPitch, enuYaw)

        // This is wrong. Conversion cannot be done like this
        val nedDeltaQ1 = Quaternion(enuPitch, enuRoll, -enuYaw)

        val nedDeltaQ2 =
            conversionQ.multiplyAndReturnNew(enuDeltaQ).multiplyAndReturnNew(conversionQ)
        nedDeltaQ2.normalize()

        val enuFinalQ1 = enuDeltaQ.multiplyAndReturnNew(enuInitQ)
        enuFinalQ1.normalize()

        val nedFinalQ1 = nedDeltaQ1.multiplyAndReturnNew(nedInitQ1)
        nedFinalQ1.normalize()

        val nedFinalQ2 =
            conversionQ.multiplyAndReturnNew(enuFinalQ1).multiplyAndReturnNew(conversionQ)
        nedFinalQ2.normalize()

        val nedFinalQ3 = nedDeltaQ2.multiplyAndReturnNew(nedInitQ1)
        nedFinalQ3.normalize()

        val nedFinalQ4 = Quaternion()
        ENUtoNEDConverter.convert(enuFinalQ1, nedFinalQ4)

        assertFalse(nedFinalQ1.equals(nedFinalQ2, ABSOLUTE_ERROR))
        assertFalse(nedFinalQ1.equals(nedFinalQ3, ABSOLUTE_ERROR))
        assertTrue(nedFinalQ2.equals(nedFinalQ3, ABSOLUTE_ERROR))

        assertTrue(nedFinalQ2.equals(nedFinalQ4, ABSOLUTE_ERROR))
        assertTrue(nedFinalQ3.equals(nedFinalQ4, ABSOLUTE_ERROR))

        // convert back
        val enuFinalQ2 = Quaternion()
        ENUtoNEDConverter.convert(nedFinalQ4, enuFinalQ2)

        assertTrue(enuFinalQ1.equals(enuFinalQ2, ABSOLUTE_ERROR))
    }

    @Test
    fun convertAndReturnNew_returnsExpectedResult() {
        // Xenu =  [0   1   0 ] * Xned
        //         [1   0   0 ]
        //	       [0   0   -1]

        // Xned =  [0   1   0 ] * Xenu
        //         [1   0   0 ]
        //         [0   0   -1]

        // C = 	[0  1   0 ]
        //      [1  0   0 ]
        //      [0  0   -1]

        // Notice that	conversion = C, and C * C = I -> C = C^-1

        // Xrotated_ned = Rned * Xned = Rned * C * Xenu
        // Xrotated_enu = Renu * Xenu = Renu * C * Xned

        // Xrotated_enu = C * Xrotated_ned = C * Rned * C * Xenu = Renu * Xenu
        // Xrotated_ned = C * Xrotated_enu = C * Renu * C * Xned = Rned * Xned

        // Hence:
        // Renu = C * Rned * C
        // Rned = C * Renu * C
        // Consequently, angular speed triads should be directly converted, instead only rotations
        // should be converted
        val conversionQ = ENUtoNEDConverter.conversionRotation

        val enuInitQ = getQuaternion()
        val nedInitQ1 = conversionQ.multiplyAndReturnNew(enuInitQ).multiplyAndReturnNew(conversionQ)
        nedInitQ1.normalize()

        val nedInitQ2 = ENUtoNEDConverter.convertAndReturnNew(enuInitQ)

        assertEquals(nedInitQ1, nedInitQ2)

        val randomizer = UniformRandomizer()
        // randomly define angular speed and time interval
        val wx = randomizer.nextDouble()
        val wy = randomizer.nextDouble()
        val wz = randomizer.nextDouble()
        val dt = randomizer.nextDouble()

        val enuRoll = wx * dt
        val enuPitch = wy * dt
        val enuYaw = wz * dt
        val enuDeltaQ = Quaternion(enuRoll, enuPitch, enuYaw)

        // This is wrong. Conversion cannot be done like this
        val nedDeltaQ1 = Quaternion(enuPitch, enuRoll, -enuYaw)

        val nedDeltaQ2 =
            conversionQ.multiplyAndReturnNew(enuDeltaQ).multiplyAndReturnNew(conversionQ)
        nedDeltaQ2.normalize()

        val enuFinalQ1 = enuDeltaQ.multiplyAndReturnNew(enuInitQ)
        enuFinalQ1.normalize()

        val nedFinalQ1 = nedDeltaQ1.multiplyAndReturnNew(nedInitQ1)
        nedFinalQ1.normalize()

        val nedFinalQ2 =
            conversionQ.multiplyAndReturnNew(enuFinalQ1).multiplyAndReturnNew(conversionQ)
        nedFinalQ2.normalize()

        val nedFinalQ3 = nedDeltaQ2.multiplyAndReturnNew(nedInitQ1)
        nedFinalQ3.normalize()

        val nedFinalQ4 = ENUtoNEDConverter.convertAndReturnNew(enuFinalQ1)

        assertFalse(nedFinalQ1.equals(nedFinalQ2, ABSOLUTE_ERROR))
        assertFalse(nedFinalQ1.equals(nedFinalQ3, ABSOLUTE_ERROR))
        assertTrue(nedFinalQ2.equals(nedFinalQ3, ABSOLUTE_ERROR))

        assertTrue(nedFinalQ2.equals(nedFinalQ4, ABSOLUTE_ERROR))
        assertTrue(nedFinalQ3.equals(nedFinalQ4, ABSOLUTE_ERROR))

        // convert back
        val enuFinalQ2 = ENUtoNEDConverter.convertAndReturnNew(nedFinalQ4)

        assertTrue(enuFinalQ1.equals(enuFinalQ2, ABSOLUTE_ERROR))
    }

    @Test
    fun convert_whenRotationUsingStepIntegrator_returnsExpectedValue() {
        val integrator = QuaternionStepIntegrator.create(QuaternionStepIntegratorType.EULER_METHOD)

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextDouble()
        val wy = randomizer.nextDouble()
        val wz = randomizer.nextDouble()
        val dt = randomizer.nextDouble()
        val enuW = AngularSpeedTriad(wx, wy, wz)
        val enuInitialQ = getQuaternion()

        // integrate in ENU coordinates
        val enuEndQ = Quaternion()
        integrator.integrate(
            enuInitialQ,
            enuW.valueX,
            enuW.valueY,
            enuW.valueZ,
            enuW.valueX,
            enuW.valueY,
            enuW.valueZ,
            dt,
            enuEndQ
        )

        // and then convert to NED coordinates
        val nedEndQ1 = ENUtoNEDConverter.convertAndReturnNew(enuEndQ)

        //Or convert first to NED coordinates
        val nedW = ENUtoNEDConverter.convertAndReturnNew(enuW)
        val nedInitialQ = ENUtoNEDConverter.convertAndReturnNew(enuInitialQ)

        // and then integrate in NED coordinates
        val nedEndQ2 = Quaternion()
        integrator.integrate(
            nedInitialQ,
            nedW.valueX,
            nedW.valueY,
            nedW.valueZ,
            nedW.valueX,
            nedW.valueY,
            nedW.valueZ,
            dt,
            nedEndQ2
        )

        nedEndQ1.normalize()
        nedEndQ2.normalize()
        assertTrue(nedEndQ1.equals(nedEndQ2, ABSOLUTE_ERROR))
    }

    @Test
    fun convertPoint_whenPointCoordinates_returnsExpectedResult() {
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()

        val result = DoubleArray(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH)
        ENUtoNEDConverter.convertPoint(valueX, valueY, valueZ, result)

        // check
        assertEquals(valueY, result[0], 0.0)
        assertEquals(valueX, result[1], 0.0)
        assertEquals(-valueZ, result[2], 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun convertPoint_whenPointCoordinatesAndInvalidArrayLength_throwsIllegalArgumentException() {
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()

        val result = DoubleArray(1)
        ENUtoNEDConverter.convertPoint(valueX, valueY, valueZ, result)
    }

    @Test
    fun convertPoint_whenArray_returnsExpectedValue() {
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()

        val input = doubleArrayOf(valueX, valueY, valueZ)

        val result1 = DoubleArray(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH)
        ENUtoNEDConverter.convertPoint(input, result1)

        // check
        assertEquals(valueY, result1[0], 0.0)
        assertEquals(valueX, result1[1], 0.0)
        assertEquals(-valueZ, result1[2], 0.0)

        // convert back
        val result2 = DoubleArray(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH)
        ENUtoNEDConverter.convertPoint(result1, result2)

        assertArrayEquals(input, result2, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun convertPoint_whenArrayAndInvalidInputLength_throwsIllegalArgumentException() {
        val input = DoubleArray(1)

        val result = DoubleArray(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH)
        ENUtoNEDConverter.convertPoint(input, result)
    }

    @Test(expected = IllegalArgumentException::class)
    fun convertPoint_whenArrayAndInvalidOutputLength_throwsIllegalArgumentException() {
        val input = DoubleArray(Point3D.POINT3D_INHOMOGENEOUS_COORDINATES_LENGTH)

        val result = DoubleArray(1)
        ENUtoNEDConverter.convertPoint(input, result)
    }

    @Test
    fun convertPoint_whenPoint_returnsExpectedValue() {
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()

        val input = InhomogeneousPoint3D(valueX, valueY, valueZ)

        val result1 = InhomogeneousPoint3D()
        ENUtoNEDConverter.convertPoint(input, result1)

        // check
        assertEquals(valueY, result1.inhomX, 0.0)
        assertEquals(valueX, result1.inhomY, 0.0)
        assertEquals(-valueZ, result1.inhomZ, 0.0)

        // convert back
        val result2 = InhomogeneousPoint3D()
        ENUtoNEDConverter.convertPoint(result1, result2)

        assertEquals(input, result2)
    }

    @Test
    fun convertPointAndReturnNew_returnsExpectedValue() {
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()

        val input = doubleArrayOf(valueX, valueY, valueZ)

        val result1 = ENUtoNEDConverter.convertPointAndReturnNew(input)

        // check
        assertEquals(valueY, result1[0], 0.0)
        assertEquals(valueX, result1[1], 0.0)
        assertEquals(-valueZ, result1[2], 0.0)

        // convert back
        val result2 = ENUtoNEDConverter.convertPointAndReturnNew(result1)

        assertArrayEquals(input, result2, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun convertPointAndReturnNew_whenInvalidLength_throwsIllegalArgumentException() {
        val input = DoubleArray(1)
        ENUtoNEDConverter.convertPointAndReturnNew(input)
    }

    @Test
    fun convertPointAndReturnNew_whenPoint_returnsExpectedValue() {
        val randomizer = UniformRandomizer()
        val valueX = randomizer.nextDouble()
        val valueY = randomizer.nextDouble()
        val valueZ = randomizer.nextDouble()

        val input = InhomogeneousPoint3D(valueX, valueY, valueZ)

        val result1 = ENUtoNEDConverter.convertPointAndReturnNew(input)

        // check
        assertEquals(valueY, result1.inhomX, 0.0)
        assertEquals(valueX, result1.inhomY, 0.0)
        assertEquals(-valueZ, result1.inhomZ, 0.0)

        // convert back
        val result2 = ENUtoNEDConverter.convertPointAndReturnNew(result1)

        assertEquals(input, result2)
    }

    @Test
    fun convert_whenEuclideanTransformation_returnsExpectedValue() {
        val enuRotation = getQuaternion()
        val enuTranslation = getTranslation()
        val enuT1 = EuclideanTransformation3D(enuRotation, enuTranslation)

        // convert
        val nedT1 = EuclideanTransformation3D()
        ENUtoNEDConverter.convert(enuT1, nedT1)

        // check
        val nedRotation = ENUtoNEDConverter.convertAndReturnNew(enuRotation)
        val nedTranslation = ENUtoNEDConverter.convertPointAndReturnNew(enuTranslation)
        val nedT2 = EuclideanTransformation3D(nedRotation, nedTranslation)

        assertEquals(nedT1.asMatrix(), nedT2.asMatrix())

        // compute as C * T * C
        val c = Matrix(EuclideanTransformation3D.HOM_COORDS, EuclideanTransformation3D.HOM_COORDS)
        c.setSubmatrix(0, 0, 2, 2, ENUtoNEDConverter.conversionRotationMatrix)
        c.setElementAt(3, 3, 1.0)

        val nedT3 = c.multiplyAndReturnNew(enuT1.asMatrix()).multiplyAndReturnNew(c)

        val m = nedT1.asMatrix()
        assertTrue(nedT3.equals(m, ABSOLUTE_ERROR))

        // convert back
        val enuT2 = EuclideanTransformation3D()
        ENUtoNEDConverter.convert(nedT1, enuT2)

        assertTrue(enuT1.asMatrix().equals(enuT2.asMatrix(), ABSOLUTE_ERROR))
    }

    @Test
    fun convertAndReturnNew_whenEuclideanTransformation_returnsExpectedValue() {
        val enuRotation = getQuaternion()
        val enuTranslation = getTranslation()
        val enuT1 = EuclideanTransformation3D(enuRotation, enuTranslation)

        // convert
        val nedT1 = ENUtoNEDConverter.convertAndReturnNew(enuT1)

        // check
        val nedRotation = ENUtoNEDConverter.convertAndReturnNew(enuRotation)
        val nedTranslation = ENUtoNEDConverter.convertPointAndReturnNew(enuTranslation)
        val nedT2 = EuclideanTransformation3D(nedRotation, nedTranslation)

        assertEquals(nedT1.asMatrix(), nedT2.asMatrix())

        // compute as C * T * C
        val c = Matrix(EuclideanTransformation3D.HOM_COORDS, EuclideanTransformation3D.HOM_COORDS)
        c.setSubmatrix(0, 0, 2, 2, ENUtoNEDConverter.conversionRotationMatrix)
        c.setElementAt(3, 3, 1.0)

        val nedT3 = c.multiplyAndReturnNew(enuT1.asMatrix()).multiplyAndReturnNew(c)

        val m = nedT1.asMatrix()
        assertTrue(nedT3.equals(m, ABSOLUTE_ERROR))

        // convert back
        val enuT2 = ENUtoNEDConverter.convertAndReturnNew(nedT1)

        assertTrue(enuT1.asMatrix().equals(enuT2.asMatrix(), ABSOLUTE_ERROR))
    }

    @Test
    fun quaternionConversionJacobian_whenValidSize_returnsExpectedResult() {
        val j1 = Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS)
        ENUtoNEDConverter.quaternionConversionJacobian(j1)
        val j2 = ENUtoNEDConverter.quaternionConversionJacobian

        assertEquals(j1, j2)

        val j3 = Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS)
        j3.setElementAt(0, 0, -1.0)
        j3.setElementAt(2, 1, -1.0)
        j3.setElementAt(1, 2, -1.0)
        j3.setElementAt(3, 3, 1.0)

        assertEquals(j1, j3)

        val randomizer = UniformRandomizer()
        val a1 = randomizer.nextDouble()
        val b1 = randomizer.nextDouble()
        val c1 = randomizer.nextDouble()
        val d1 = randomizer.nextDouble()

        val q1 = Quaternion(a1, b1, c1, d1)

        val jacobianEstimator = JacobianEstimator(object: MultiVariateFunctionEvaluatorListener {
            override fun evaluate(point: DoubleArray, result: DoubleArray) {
                val q = Quaternion(point)
                val convertedQ = ENUtoNEDConverter.convertAndReturnNew(q)
                convertedQ.values(result)
            }

            override fun getNumberOfVariables(): Int {
                return Quaternion.N_PARAMS
            }
        })

        val j4 = jacobianEstimator.jacobian(q1.values)

        assertTrue(j4.equals(j1, ABSOLUTE_ERROR))
    }

    @Test
    fun quaternionConversionJacobian_whenInvalidRows_returnsExpectedResult() {
        val j1 = Matrix(1, Quaternion.N_PARAMS)
        ENUtoNEDConverter.quaternionConversionJacobian(j1)
        val j2 = ENUtoNEDConverter.quaternionConversionJacobian

        assertEquals(j1, j2)

        val j3 = Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS)
        j3.setElementAt(0, 0, -1.0)
        j3.setElementAt(2, 1, -1.0)
        j3.setElementAt(1, 2, -1.0)
        j3.setElementAt(3, 3, 1.0)

        assertEquals(j1, j3)
    }

    @Test
    fun quaternionConversionJacobian_whenInvalidColumns_returnsExpectedResult() {
        val j1 = Matrix(Quaternion.N_PARAMS, 1)
        ENUtoNEDConverter.conversionRotationMatrix(j1)
        ENUtoNEDConverter.quaternionConversionJacobian(j1)
        val j2 = ENUtoNEDConverter.quaternionConversionJacobian

        assertEquals(j1, j2)

        val j3 = Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS)
        j3.setElementAt(0, 0, -1.0)
        j3.setElementAt(2, 1, -1.0)
        j3.setElementAt(1, 2, -1.0)
        j3.setElementAt(3, 3, 1.0)

        assertEquals(j1, j3)
    }

    private companion object {
        const val ABSOLUTE_ERROR = 1e-11

        const val MIN_DEGREES = -45.0
        const val MAX_DEGREES = 45.0

        const val MIN_TRANSLATION = -100.0
        const val MAX_TRANSLATION = 100.0

        fun getQuaternion(): Quaternion {
            val randomizer = UniformRandomizer()
            val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
            val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
            val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
            return Quaternion(roll, pitch, yaw)
        }

        fun getTranslation(): DoubleArray {
            val randomizer = UniformRandomizer()
            val valueX = randomizer.nextDouble(MIN_TRANSLATION, MAX_TRANSLATION)
            val valueY = randomizer.nextDouble(MIN_TRANSLATION, MAX_TRANSLATION)
            val valueZ = randomizer.nextDouble(MIN_TRANSLATION, MAX_TRANSLATION)
            return doubleArrayOf(valueX, valueY, valueZ)
        }
    }
}