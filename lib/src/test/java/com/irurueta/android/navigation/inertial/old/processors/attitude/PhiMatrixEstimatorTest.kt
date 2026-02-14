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
package com.irurueta.android.navigation.inertial.old.processors.attitude

import com.irurueta.algebra.Matrix
import com.irurueta.algebra.Utils
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.AngularSpeedUnit
import org.junit.Assert.*
import org.junit.Test

class PhiMatrixEstimatorTest {

    @Test
    fun create_whenApproximatedMethod_returnsExpectedInstance() {
        val a = generateProcessEquationMatrix()
        val estimator = PhiMatrixEstimator.create(a, PhiMatrixMethod.APPROXIMATED)

        assertTrue(estimator is ApproximatedPhiMatrixEstimator)
        assertEquals(PhiMatrixMethod.APPROXIMATED, estimator.method)
        assertSame(a, estimator.a)
    }

    @Test
    fun create_whenPreciseMethod_returnExpectedInstance() {
        val a = generateProcessEquationMatrix()
        val estimator = PhiMatrixEstimator.create(a, PhiMatrixMethod.PRECISE)

        assertTrue(estimator is PrecisePhiMatrixEstimator)
        assertEquals(PhiMatrixMethod.PRECISE, estimator.method)
        assertSame(a, estimator.a)
    }

    @Test
    fun create_whenDefaultMethod_returnsExpectedInstance() {
        val a = generateProcessEquationMatrix()
        val estimator = PhiMatrixEstimator.create(a)

        assertTrue(estimator is ApproximatedPhiMatrixEstimator)
        assertEquals(PhiMatrixMethod.APPROXIMATED, estimator.method)
        assertSame(a, estimator.a)
    }

    @Test
    fun create_whenNoParameters_returnsExpectedInstance() {
        val estimator = PhiMatrixEstimator.create()

        assertTrue(estimator is ApproximatedPhiMatrixEstimator)
        assertEquals(PhiMatrixMethod.APPROXIMATED, estimator.method)
        assertNull(estimator.a)
    }

    private companion object {
        const val ROWS = 9

        const val COLUMNS = 9

        const val MIN_ANGULAR_SPEED = -Math.PI

        const val MAX_ANGULAR_SPEED = Math.PI

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
    }
}