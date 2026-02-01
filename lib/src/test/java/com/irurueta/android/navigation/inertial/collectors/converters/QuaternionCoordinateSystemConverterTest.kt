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

import com.irurueta.algebra.Matrix
import com.irurueta.geometry.Quaternion
import com.irurueta.geometry.Rotation3D
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegrator
import com.irurueta.navigation.inertial.calibration.gyroscope.QuaternionStepIntegratorType
import com.irurueta.numerical.JacobianEstimator
import com.irurueta.numerical.MultiVariateFunctionEvaluatorListener
import com.irurueta.statistics.UniformRandomizer
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.sqrt

class QuaternionCoordinateSystemConverterTest {

    @Test
    fun conversionRotation_returnsExpectedResult() {
        val rotation1 = Quaternion()
        QuaternionCoordinateSystemConverter.conversionRotation(rotation1)
        val rotation2 = QuaternionCoordinateSystemConverter.conversionRotation

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
        QuaternionCoordinateSystemConverter.conversionRotationMatrix(m1)
        val m2 = QuaternionCoordinateSystemConverter.conversionRotationMatrix

        assertEquals(m1, m2)

        val m3 = Matrix(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)
        m3.setElementAt(0, 1, 1.0)
        m3.setElementAt(1, 0, 1.0)
        m3.setElementAt(2, 2, -1.0)

        assertEquals(m1, m3)

        val m4 = QuaternionCoordinateSystemConverter.conversionRotation.asInhomogeneousMatrix()
        assertTrue(m1.equals(m4, ABSOLUTE_ERROR))
    }

    @Test
    fun conversionRotationMatrix_whenInvalidRows_returnsExpectedResult() {
        val m1 = Matrix(1, Rotation3D.INHOM_COORDS)
        QuaternionCoordinateSystemConverter.conversionRotationMatrix(m1)
        val m2 = QuaternionCoordinateSystemConverter.conversionRotationMatrix

        assertEquals(m1, m2)

        val m3 = Matrix(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)
        m3.setElementAt(0, 1, 1.0)
        m3.setElementAt(1, 0, 1.0)
        m3.setElementAt(2, 2, -1.0)

        assertEquals(m1, m3)

        val m4 = QuaternionCoordinateSystemConverter.conversionRotation.asInhomogeneousMatrix()
        assertTrue(m1.equals(m4, ABSOLUTE_ERROR))
    }

    @Test
    fun conversionRotationMatrix_whenInvalidColumns_returnsExpectedResult() {
        val m1 = Matrix(Rotation3D.INHOM_COORDS, 1)
        QuaternionCoordinateSystemConverter.conversionRotationMatrix(m1)
        val m2 = QuaternionCoordinateSystemConverter.conversionRotationMatrix

        assertEquals(m1, m2)

        val m3 = Matrix(Rotation3D.INHOM_COORDS, Rotation3D.INHOM_COORDS)
        m3.setElementAt(0, 1, 1.0)
        m3.setElementAt(1, 0, 1.0)
        m3.setElementAt(2, 2, -1.0)

        assertEquals(m1, m3)

        val m4 = QuaternionCoordinateSystemConverter.conversionRotation.asInhomogeneousMatrix()
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
        val conversionQ = QuaternionCoordinateSystemConverter.conversionRotation

        val enuInitQ = getQuaternion()
        val nedInitQ1 = conversionQ.multiplyAndReturnNew(enuInitQ).multiplyAndReturnNew(conversionQ)
        nedInitQ1.normalize()

        val nedInitQ2 = Quaternion()
        QuaternionCoordinateSystemConverter.convert(enuInitQ, nedInitQ2)

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
        QuaternionCoordinateSystemConverter.convert(enuFinalQ1, nedFinalQ4)

        assertFalse(nedFinalQ1.equals(nedFinalQ2, ABSOLUTE_ERROR))
        assertFalse(nedFinalQ1.equals(nedFinalQ3, ABSOLUTE_ERROR))
        assertTrue(nedFinalQ2.equals(nedFinalQ3, ABSOLUTE_ERROR))

        assertTrue(nedFinalQ2.equals(nedFinalQ4, ABSOLUTE_ERROR))
        assertTrue(nedFinalQ3.equals(nedFinalQ4, ABSOLUTE_ERROR))

        // convert back
        val enuFinalQ2 = Quaternion()
        QuaternionCoordinateSystemConverter.convert(nedFinalQ4, enuFinalQ2)

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
        val conversionQ = QuaternionCoordinateSystemConverter.conversionRotation

        val enuInitQ = getQuaternion()
        val nedInitQ1 = conversionQ.multiplyAndReturnNew(enuInitQ).multiplyAndReturnNew(conversionQ)
        nedInitQ1.normalize()

        val nedInitQ2 = QuaternionCoordinateSystemConverter.convertAndReturnNew(enuInitQ)

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

        val nedFinalQ4 = QuaternionCoordinateSystemConverter.convertAndReturnNew(enuFinalQ1)

        assertFalse(nedFinalQ1.equals(nedFinalQ2, ABSOLUTE_ERROR))
        assertFalse(nedFinalQ1.equals(nedFinalQ3, ABSOLUTE_ERROR))
        assertTrue(nedFinalQ2.equals(nedFinalQ3, ABSOLUTE_ERROR))

        assertTrue(nedFinalQ2.equals(nedFinalQ4, ABSOLUTE_ERROR))
        assertTrue(nedFinalQ3.equals(nedFinalQ4, ABSOLUTE_ERROR))

        // convert back
        val enuFinalQ2 = QuaternionCoordinateSystemConverter.convertAndReturnNew(nedFinalQ4)

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
        val nedEndQ1 = QuaternionCoordinateSystemConverter.convertAndReturnNew(enuEndQ)

        //Or convert first to NED coordinates
        val nedW = AngularSpeedTriad(wy, wx, -wz)
        val nedInitialQ = QuaternionCoordinateSystemConverter.convertAndReturnNew(enuInitialQ)

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
    fun quaternionConversionJacobian_whenValidSize_returnsExpectedResult() {
        val j1 = Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS)
        QuaternionCoordinateSystemConverter.quaternionConversionJacobian(j1)
        val j2 = QuaternionCoordinateSystemConverter.quaternionConversionJacobian

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

        val jacobianEstimator = JacobianEstimator(object : MultiVariateFunctionEvaluatorListener {
            override fun evaluate(point: DoubleArray, result: DoubleArray) {
                val q = Quaternion(point)
                val convertedQ = QuaternionCoordinateSystemConverter.convertAndReturnNew(q)
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
        QuaternionCoordinateSystemConverter.quaternionConversionJacobian(j1)
        val j2 = QuaternionCoordinateSystemConverter.quaternionConversionJacobian

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
        QuaternionCoordinateSystemConverter.conversionRotationMatrix(j1)
        QuaternionCoordinateSystemConverter.quaternionConversionJacobian(j1)
        val j2 = QuaternionCoordinateSystemConverter.quaternionConversionJacobian

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

        fun getQuaternion(): Quaternion {
            val randomizer = UniformRandomizer()
            val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
            val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
            val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
            return Quaternion(roll, pitch, yaw)
        }
    }
}