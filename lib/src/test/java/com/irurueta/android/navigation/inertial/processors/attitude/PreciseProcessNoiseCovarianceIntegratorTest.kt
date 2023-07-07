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
import com.irurueta.android.navigation.inertial.numerical.ExponentialMatrixEstimator
import com.irurueta.android.navigation.inertial.numerical.integration.IntegratorType
import com.irurueta.android.navigation.inertial.numerical.integration.MatrixIntegrator
import com.irurueta.android.navigation.inertial.numerical.integration.MatrixSingleDimensionFunctionEvaluatorListener
import com.irurueta.android.navigation.inertial.numerical.integration.QuadratureType
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.AngularSpeedUnit
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertSame
import org.junit.Assert.assertThrows
import org.junit.Assert.assertTrue
import org.junit.Test
import java.lang.IllegalStateException
import java.util.logging.Level
import java.util.logging.Logger

class PreciseProcessNoiseCovarianceIntegratorTest {

    @Test
    fun constructor_whenNoParameters_returnsExpectedValues() {
        val integrator = PreciseProcessNoiseCovarianceIntegrator()

        assertNull(integrator.a)
        assertNull(integrator.q)
        assertEquals(ProcessNoiseCovarianceMethod.PRECISE, integrator.method)
    }

    @Test
    fun constructor_whenValidParameters_returnsExpectedValues() {
        val a = Matrix(ROWS, COLUMNS)
        val q = Matrix(ROWS, COLUMNS)
        val integrator = PreciseProcessNoiseCovarianceIntegrator(q, a)

        assertSame(a, integrator.a)
        assertSame(q, integrator.q)
        assertEquals(ProcessNoiseCovarianceMethod.PRECISE, integrator.method)
    }

    @Test
    fun constructor_whenInvalidQRows_throwsIllegalArgumentException() {
        val a = Matrix(ROWS, COLUMNS)
        val q = Matrix(1, COLUMNS)
        assertThrows(IllegalArgumentException::class.java) {
            PreciseProcessNoiseCovarianceIntegrator(
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
            PreciseProcessNoiseCovarianceIntegrator(
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
            PreciseProcessNoiseCovarianceIntegrator(
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
            PreciseProcessNoiseCovarianceIntegrator(
                q,
                a
            )
        }
    }

    @Test
    fun method_returnsExpectedValue() {
        val a = Matrix(ROWS, COLUMNS)
        val q = Matrix(ROWS, COLUMNS)
        val integrator = PreciseProcessNoiseCovarianceIntegrator(q, a)

        assertEquals(ProcessNoiseCovarianceMethod.PRECISE, integrator.method)
    }

    @Test
    fun integrate_returnsExpectedResult() {
        val a = generateProcessEquationMatrix()
        val q = generateContinuousTimeProcessNoiseCovarianceMatrix()

        val integrator = PreciseProcessNoiseCovarianceIntegrator(q, a)

        val result = Matrix(ROWS, COLUMNS)
        integrator.integrate(TIME_INTERVAL, result)

        // check
        val exponentialMatrixEstimator = ExponentialMatrixEstimator()
        val integratorType = integrator.integratorType
        val quadratureType = integrator.quadratureType
        val listener = object : MatrixSingleDimensionFunctionEvaluatorListener {
            override fun evaluate(t: Double, result: Matrix) {
                // compute integral of: e^(A .* t) * Q * (e^(A .* t))';
                val at = Matrix(a)
                at.multiplyByScalar(t)
                val expAt = Matrix(ROWS, COLUMNS)
                exponentialMatrixEstimator.exponential(at, expAt)
                val transposedExpAt = expAt.transposeAndReturnNew()

                val integrand = Matrix(expAt)
                integrand.multiply(q)
                integrand.multiply(transposedExpAt)
                result.copyFrom(integrand)
            }

            override fun getRows(): Int {
                return ProcessNoiseCovarianceIntegrator.N_ROWS
            }

            override fun getColumns(): Int {
                return ProcessNoiseCovarianceIntegrator.N_COLUMNS
            }
        }

        val internalIntegrator =
            MatrixIntegrator.create(0.0, TIME_INTERVAL, listener, integratorType, quadratureType)
        val expected = Matrix(ROWS, COLUMNS)
        internalIntegrator.integrate(expected)

        assertEquals(result, expected)

        val approximatedResult = computeApproximateIntegration(q, a)
        assertTrue(result.equals(approximatedResult, ABSOLUTE_ERROR))
        val error = Utils.normF(result.subtractAndReturnNew(approximatedResult))

        val approximatedResult2 = computeBetterApproximateIntegration(q, a)
        val error2 = Utils.normF(result.subtractAndReturnNew(approximatedResult2))

        assertTrue(error2 < error)
    }

    @Test
    fun integrate_whenNoAMatrix_throwsIllegalStateException() {
        val q = Matrix(ROWS, COLUMNS)
        val integrator = PreciseProcessNoiseCovarianceIntegrator(q)

        val result = Matrix(ROWS, COLUMNS)
        assertThrows(IllegalStateException::class.java) {
            integrator.integrate(TIME_INTERVAL, result)
        }
    }

    @Test
    fun integrate_whenNoQMatrix_throwsIllegalStateException() {
        val a = generateProcessEquationMatrix()
        val integrator = PreciseProcessNoiseCovarianceIntegrator(a = a)

        val result = Matrix(ROWS, COLUMNS)
        assertThrows(IllegalStateException::class.java) {
            integrator.integrate(TIME_INTERVAL, result)
        }
    }

    @Test
    fun integrate_whenInvalidResultRows_throwsIllegalArgumentException() {
        val a = generateProcessEquationMatrix()
        val q = generateContinuousTimeProcessNoiseCovarianceMatrix()

        val integrator = PreciseProcessNoiseCovarianceIntegrator(q, a)

        val result = Matrix(1, COLUMNS)
        assertThrows(IllegalArgumentException::class.java) {
            integrator.integrate(TIME_INTERVAL, result)
        }
    }

    @Test
    fun integrate_whenInvalidResultColumns_throwsIllegalArgumentException() {
        val a = generateProcessEquationMatrix()
        val q = generateContinuousTimeProcessNoiseCovarianceMatrix()

        val integrator = PreciseProcessNoiseCovarianceIntegrator(q, a)

        val result = Matrix(ROWS, 1)
        assertThrows(IllegalArgumentException::class.java) {
            integrator.integrate(TIME_INTERVAL, result)
        }
    }

    @Test
    fun integrate_whenNegativeTimeInterval_throwsIllegalArgumentException() {
        val a = generateProcessEquationMatrix()
        val q = generateContinuousTimeProcessNoiseCovarianceMatrix()

        val integrator = PreciseProcessNoiseCovarianceIntegrator(q, a)

        val result = Matrix(ROWS, COLUMNS)
        assertThrows(IllegalArgumentException::class.java) {
            integrator.integrate(-TIME_INTERVAL, result)
        }
    }

    @Test
    fun comparePerformance() {
        val a = generateProcessEquationMatrix()
        val q = generateContinuousTimeProcessNoiseCovarianceMatrix()

        var bestDuration = Long.MAX_VALUE
        var bestIntegratorType: IntegratorType? = null
        var bestQuadratureType: QuadratureType? = null
        for (integratorType in IntegratorType.values()) {
            for (quadratureType in QuadratureType.values()) {
                val integrator =
                    PreciseProcessNoiseCovarianceIntegrator(q, a, integratorType, quadratureType)

                val start = System.nanoTime()
                val result = Matrix(ROWS, COLUMNS)
                integrator.integrate(TIME_INTERVAL, result)
                val end = System.nanoTime()

                val duration = end - start
                Logger.getGlobal().log(
                    Level.INFO,
                    "Integrator type: $integratorType, quadrature type: $quadratureType, " +
                            "duration: $duration ns"
                )

                if (duration < bestDuration) {
                    bestDuration = duration
                    bestIntegratorType = integratorType
                    bestQuadratureType = quadratureType
                }
            }
        }

        Logger.getGlobal().log(
            Level.INFO,
            "Best integrator type: $bestIntegratorType, quadrature type: $bestQuadratureType, " +
                    "duration: $bestDuration ns"
        )
    }

    private companion object {
        const val ROWS = 9

        const val COLUMNS = 9

        const val MIN_ANGULAR_SPEED = -Math.PI

        const val MAX_ANGULAR_SPEED = Math.PI

        const val TIME_INTERVAL = 0.02

        const val BG = 1e-6

        const val GYROSCOPE_STANDDARD_DEVIATION = 0.006

        const val ABSOLUTE_ERROR = 1e-6

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
            rg.multiplyByScalar(GYROSCOPE_STANDDARD_DEVIATION * GYROSCOPE_STANDDARD_DEVIATION)

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