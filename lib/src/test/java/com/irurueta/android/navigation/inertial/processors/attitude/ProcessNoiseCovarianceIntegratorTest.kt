/*
 * Copyright (C) 2023 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.android.navigation.inertial.processors.attitude

import com.irurueta.algebra.Matrix
import com.irurueta.algebra.Utils
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.AngularSpeedUnit
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertSame
import org.junit.Assert.assertTrue
import org.junit.Test

class ProcessNoiseCovarianceIntegratorTest {

    @Test
    fun create_whenApproximatedMethod_returnsExpectedInstance() {
        val q = generateContinuousTimeProcessNoiseCovarianceMatrix()
        val a = generateProcessEquationMatrix()

        val integrator =
            ProcessNoiseCovarianceIntegrator.create(q, a, ProcessNoiseCovarianceMethod.APPROXIMATED)
        assertTrue(integrator is ApproximatedProcessNoiseCovarianceIntegrator)
        assertEquals(ProcessNoiseCovarianceMethod.APPROXIMATED, integrator.method)
        assertSame(q, integrator.q)
        assertSame(a, integrator.a)
    }

    @Test
    fun create_whenBetterMethod_returnsExpectedInstance() {
        val q = generateContinuousTimeProcessNoiseCovarianceMatrix()
        val a = generateProcessEquationMatrix()

        val integrator =
            ProcessNoiseCovarianceIntegrator.create(q, a, ProcessNoiseCovarianceMethod.BETTER)
        assertTrue(integrator is BetterProcessNoiseCovarianceIntegrator)
        assertEquals(ProcessNoiseCovarianceMethod.BETTER, integrator.method)
        assertSame(q, integrator.q)
        assertSame(a, integrator.a)
    }

    @Test
    fun create_whenPrecisedMethod_returnsExpectedInstance() {
        val q = generateContinuousTimeProcessNoiseCovarianceMatrix()
        val a = generateProcessEquationMatrix()

        val integrator =
            ProcessNoiseCovarianceIntegrator.create(q, a, ProcessNoiseCovarianceMethod.PRECISE)
        assertTrue(integrator is PreciseProcessNoiseCovarianceIntegrator)
        assertEquals(ProcessNoiseCovarianceMethod.PRECISE, integrator.method)
        assertSame(q, integrator.q)
        assertSame(a, integrator.a)
    }

    @Test
    fun create_whenDefaultMethod_returnsExpectedInstance() {
        val q = generateContinuousTimeProcessNoiseCovarianceMatrix()
        val a = generateProcessEquationMatrix()

        val integrator =
            ProcessNoiseCovarianceIntegrator.create(q, a)
        assertTrue(integrator is BetterProcessNoiseCovarianceIntegrator)
        assertEquals(ProcessNoiseCovarianceMethod.BETTER, integrator.method)
        assertSame(q, integrator.q)
        assertSame(a, integrator.a)
    }

    @Test
    fun create_whenNoParameters_returnsExpectedInstance() {
        val integrator =
            ProcessNoiseCovarianceIntegrator.create()
        assertTrue(integrator is BetterProcessNoiseCovarianceIntegrator)
        assertEquals(ProcessNoiseCovarianceMethod.BETTER, integrator.method)
        assertNull(integrator.q)
        assertNull(integrator.a)
    }

    private companion object {
        const val ROWS = 9

        const val COLUMNS = 9

        const val MIN_ANGULAR_SPEED = -Math.PI

        const val MAX_ANGULAR_SPEED = Math.PI

        const val BG = 1e-6

        const val GYROSCOPE_STANDARD_DEVIATION = 0.006

        fun generateProcessEquationMatrix(): Matrix {
            val randomizer = UniformRandomizer()
            val wx = randomizer.nextDouble(MIN_ANGULAR_SPEED, MAX_ANGULAR_SPEED)
            val wy = randomizer.nextDouble(MIN_ANGULAR_SPEED, MAX_ANGULAR_SPEED)
            val wz = randomizer.nextDouble(MIN_ANGULAR_SPEED, MAX_ANGULAR_SPEED)
            val angularSpeedTriad =
                AngularSpeedTriad(AngularSpeedUnit.RADIANS_PER_SECOND, wx, wy, wz)
            val angularSpeedMatrix = angularSpeedTriad.valuesAsMatrix
            val angularSpeedSkewMatrix = Utils.skewMatrix(angularSpeedMatrix)
            angularSpeedSkewMatrix.multiplyByScalar(-1.0)

            val a = Matrix(ROWS, COLUMNS)
            a.setSubmatrix(0, 0, 2, 2, angularSpeedSkewMatrix)

            val identity3 = Matrix.identity(3, 3)
            identity3.multiplyByScalar(-0.5)
            a.setSubmatrix(0, 3, 2, 5, identity3)

            return a
        }

        fun generateContinuousTimeProcessNoiseCovarianceMatrix(): Matrix {

            val rg: Matrix =
                Matrix.identity(AngularSpeedTriad.COMPONENTS, AngularSpeedTriad.COMPONENTS)
            rg.multiplyByScalar(GYROSCOPE_STANDARD_DEVIATION * GYROSCOPE_STANDARD_DEVIATION)

            val qbg: Matrix =
                Matrix.identity(AngularSpeedTriad.COMPONENTS, AngularSpeedTriad.COMPONENTS)
            qbg.multiplyByScalar(BG)

            val qba: Matrix =
                Matrix.identity(AccelerationTriad.COMPONENTS, AccelerationTriad.COMPONENTS)
            qba.multiplyByScalar(BG)

            val q = Matrix(ROWS, COLUMNS)
            q.setSubmatrix(0, 0, 2, 2, rg)
            q.multiplyByScalar(0.25)
            q.setSubmatrix(3, 3, 5, 5, qbg)
            q.setSubmatrix(6, 6, 8, 8, qba)

            return q
        }
    }
}