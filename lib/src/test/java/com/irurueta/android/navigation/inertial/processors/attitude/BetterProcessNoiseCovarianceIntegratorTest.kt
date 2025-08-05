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
import org.junit.Assert.*
import org.junit.Test
import java.lang.IllegalStateException

class BetterProcessNoiseCovarianceIntegratorTest {

    @Test
    fun constructor_whenNoParameters_returnsExpectedValues() {
        val integrator = BetterProcessNoiseCovarianceIntegrator()

        assertNull(integrator.a)
        assertNull(integrator.q)
        assertEquals(ProcessNoiseCovarianceMethod.BETTER, integrator.method)
    }

    @Test
    fun constructor_whenValidParameters_returnsExpectedValues() {
        val a = Matrix(ROWS, COLUMNS)
        val q = Matrix(ROWS, COLUMNS)
        val integrator = BetterProcessNoiseCovarianceIntegrator(q, a)

        assertSame(a, integrator.a)
        assertSame(q, integrator.q)
        assertEquals(ProcessNoiseCovarianceMethod.BETTER, integrator.method)
    }

    @Test
    fun constructor_whenInvalidQRows_throwsIllegalArgumentException() {
        val a = Matrix(ROWS, COLUMNS)
        val q = Matrix(1, COLUMNS)
        assertThrows(IllegalArgumentException::class.java) {
            BetterProcessNoiseCovarianceIntegrator(
                q,
                a
            )
        }
    }

    @Test
    fun constructor_whenInvalidQColumns_throwsIllegalArgumentException() {
        val a = Matrix(ROWS, COLUMNS)
        val q = Matrix(ROWS, 1)
        assertThrows(IllegalArgumentException::class.java) {
            BetterProcessNoiseCovarianceIntegrator(
                q,
                a
            )
        }
    }

    @Test
    fun constructor_whenInvalidARows_throwsIllegalArgumentException() {
        val a = Matrix(1, COLUMNS)
        val q = Matrix(ROWS, COLUMNS)
        assertThrows(IllegalArgumentException::class.java) {
            BetterProcessNoiseCovarianceIntegrator(
                q,
                a
            )
        }
    }

    @Test
    fun constructor_whenInvalidAColumns_throwsIllegalArgumentException() {
        val a = Matrix(ROWS, 1)
        val q = Matrix(ROWS, COLUMNS)
        assertThrows(IllegalArgumentException::class.java) {
            BetterProcessNoiseCovarianceIntegrator(
                q,
                a
            )
        }
    }

    @Test
    fun a_whenInvalidRows_throwsIllegalArgumentException() {
        val integrator = BetterProcessNoiseCovarianceIntegrator()

        // check initial value
        assertNull(integrator.a)

        val a = Matrix(1, COLUMNS)
        assertThrows(IllegalArgumentException::class.java) {
            integrator.a = a
        }
    }

    @Test
    fun a_whenInvalidColumns_throwsIllegalArgumentException() {
        val integrator = BetterProcessNoiseCovarianceIntegrator()

        // check initial value
        assertNull(integrator.a)

        val a = Matrix(ROWS, 1)
        assertThrows(IllegalArgumentException::class.java) {
            integrator.a = a
        }
    }

    @Test
    fun a_whenValid_setsExpectedValue() {
        val integrator = BetterProcessNoiseCovarianceIntegrator()

        // check initial value
        assertNull(integrator.a)

        // set new value
        val a = Matrix(ROWS, COLUMNS)
        integrator.a = a

        // check
        assertSame(a, integrator.a)
    }

    @Test
    fun q_whenInvalidRows_throwsIllegalArgumentException() {
        val integrator = BetterProcessNoiseCovarianceIntegrator()

        // check initial value
        assertNull(integrator.q)

        val q = Matrix(1, COLUMNS)
        assertThrows(IllegalArgumentException::class.java) {
            integrator.q = q
        }
    }

    @Test
    fun q_whenInvalidColumns_throwsIllegalArgumentException() {
        val integrator = BetterProcessNoiseCovarianceIntegrator()

        // check initial value
        assertNull(integrator.q)

        val q = Matrix(ROWS, 1)
        assertThrows(IllegalArgumentException::class.java) {
            integrator.q = q
        }
    }

    @Test
    fun q_whnValid_setsExpectedValue() {
        val integrator = BetterProcessNoiseCovarianceIntegrator()

        // check initial value
        assertNull(integrator.q)

        // set new value
        val q = Matrix(ROWS, COLUMNS)
        integrator.q = q

        // check
        assertSame(q, integrator.q)
    }

    @Test
    fun method_returnsExpectedValue() {
        val a = Matrix(ROWS, COLUMNS)
        val q = Matrix(ROWS, COLUMNS)
        val integrator = BetterProcessNoiseCovarianceIntegrator(q, a)

        assertEquals(ProcessNoiseCovarianceMethod.BETTER, integrator.method)
    }

    @Test
    fun integrate_returnsExpectedResult() {
        val a = generateProcessEquationMatrix()
        val q = generateContinuousTimeProcessNoiseCovarianceMatrix()

        val integrator = BetterProcessNoiseCovarianceIntegrator(q, a)

        val result = Matrix(ROWS, COLUMNS)
        integrator.integrate(TIME_INTERVAL, result)

        val expected = computeBetterApproximateIntegration(q, a)
        assertEquals(expected, result)

        val preciseIntegrator = PreciseProcessNoiseCovarianceIntegrator(q, a)
        val preciseResult = Matrix(ROWS, COLUMNS)
        preciseIntegrator.integrate(TIME_INTERVAL, preciseResult)

        val error = Utils.normF(result.subtractAndReturnNew(preciseResult))

        val result2 = computeApproximateIntegration(q, a)
        val error2 = Utils.normF(result2.subtractAndReturnNew(preciseResult))

        assertTrue(error2 >= error)
    }

    @Test
    fun integrate_whenNoAMatrix_throwsIllegalStateException() {
        val q = generateContinuousTimeProcessNoiseCovarianceMatrix()
        val integrator = BetterProcessNoiseCovarianceIntegrator(q)

        val result = Matrix(ROWS, COLUMNS)
        assertThrows(IllegalStateException::class.java) {
            integrator.integrate(TIME_INTERVAL, result)
        }
    }

    @Test
    fun integrate_whenNoQMatrix_throwsIllegalStateException() {
        val a = generateProcessEquationMatrix()
        val integrator = BetterProcessNoiseCovarianceIntegrator(a = a)

        val result = Matrix(ROWS, COLUMNS)
        assertThrows(IllegalStateException::class.java) {
            integrator.integrate(TIME_INTERVAL, result)
        }
    }

    @Test
    fun integrate_whenInvalidResultRows_throwsIllegalArgumentException() {
        val a = generateProcessEquationMatrix()
        val q = generateContinuousTimeProcessNoiseCovarianceMatrix()

        val integrator = ApproximatedProcessNoiseCovarianceIntegrator(q, a)

        val result = Matrix(1, COLUMNS)
        assertThrows(IllegalArgumentException::class.java) {
            integrator.integrate(TIME_INTERVAL, result)
        }
    }

    @Test
    fun integrate_whenInvalidResultColumns_throwsIllegalArgumentException() {
        val a = generateProcessEquationMatrix()
        val q = generateContinuousTimeProcessNoiseCovarianceMatrix()

        val integrator = ApproximatedProcessNoiseCovarianceIntegrator(q, a)

        val result = Matrix(ROWS, 1)
        assertThrows(IllegalArgumentException::class.java) {
            integrator.integrate(TIME_INTERVAL, result)
        }
    }

    @Test
    fun integrate_whenNegativeTimeInterval_throwsIllegalArgumentException() {
        val a = generateProcessEquationMatrix()
        val q = generateContinuousTimeProcessNoiseCovarianceMatrix()

        val integrator = ApproximatedProcessNoiseCovarianceIntegrator(q, a)
        val result = Matrix(ROWS, COLUMNS)
        assertThrows(IllegalArgumentException::class.java) {
            integrator.integrate(-TIME_INTERVAL, result)
        }
    }

    private companion object {
        const val ROWS = 9

        const val COLUMNS = 9

        const val MIN_ANGULAR_SPEED = -Math.PI

        const val MAX_ANGULAR_SPEED = Math.PI

        const val TIME_INTERVAL = 0.02

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

        fun computeApproximateIntegration(q: Matrix, a: Matrix): Matrix {
            val qd = Matrix(q)
            qd.multiplyByScalar(TIME_INTERVAL)

            val aq = a.multiplyAndReturnNew(q)
            aq.multiplyByScalar(0.5)
            qd.add(aq)

            val aTransposed = a.transposeAndReturnNew()
            val qaTransposed = q.multiplyAndReturnNew(aTransposed)
            qaTransposed.multiplyByScalar(0.5)
            qd.add(qaTransposed)

            return qd
        }

        fun computeBetterApproximateIntegration(q: Matrix, a: Matrix): Matrix {
            val qd = Matrix(q)
            qd.multiplyByScalar(TIME_INTERVAL)

            val aq = a.multiplyAndReturnNew(q)
            aq.multiplyByScalar(0.5 * TIME_INTERVAL * TIME_INTERVAL)
            qd.add(aq)

            val aTransposed = a.transposeAndReturnNew()
            val qaTransposed = q.multiplyAndReturnNew(aTransposed)
            qaTransposed.multiplyByScalar(0.5 * TIME_INTERVAL * TIME_INTERVAL)
            qd.add(qaTransposed)

            return qd
        }
    }

}