package com.irurueta.android.navigation.inertial

import com.irurueta.algebra.Matrix
import com.irurueta.geometry.Quaternion
import com.irurueta.numerical.JacobianEstimator
import com.irurueta.numerical.MultiVariateFunctionEvaluatorListener
import com.irurueta.statistics.UniformRandomizer
import io.mockk.clearAllMocks
import io.mockk.unmockkAll
import org.junit.After
import org.junit.Assert.*
import org.junit.Test
import kotlin.math.sqrt

class QuaternionHelperTest {

    @After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
    }

    @Test
    fun dotProduct_normalizesAndComputesDotProduct() {
        val randomizer = UniformRandomizer()
        val a1 = randomizer.nextDouble()
        val b1 = randomizer.nextDouble()
        val c1 = randomizer.nextDouble()
        val d1 = randomizer.nextDouble()

        val a2 = randomizer.nextDouble()
        val b2 = randomizer.nextDouble()
        val c2 = randomizer.nextDouble()
        val d2 = randomizer.nextDouble()

        val q1 = Quaternion(a1, b1, c1, d1)
        val q2 = Quaternion(a2, b2, c2, d2)

        // create normalized copies
        val q1b = Quaternion(q1)
        val q2b = Quaternion(q2)
        q1b.normalize()
        q2b.normalize()

        assertFalse(q1.isNormalized)
        assertFalse(q2.isNormalized)
        assertTrue(q1b.isNormalized)
        assertTrue(q2b.isNormalized)

        val expected = q1b.a * q2b.a + q1b.b * q2b.b + q1b.c * q2b.c + q1b.d * q2b.d
        val dot = QuaternionHelper.dotProduct(q1, q2)

        assertEquals(expected, dot, 0.0)
    }

    @Test
    fun normalize_whenNoJacobian_executesNormalization() {
        val randomizer = UniformRandomizer()
        val a = randomizer.nextDouble()
        val b = randomizer.nextDouble()
        val c = randomizer.nextDouble()
        val d = randomizer.nextDouble()

        val q = Quaternion(a, b, c, d)

        assertFalse(q.isNormalized)

        // normalize
        QuaternionHelper.normalize(q)

        // check
        assertTrue(q.isNormalized)
        val norm = sqrt(q.a * q.a + q.b * q.b + q.c * q.c + q.d * q.d)
        assertEquals(1.0, norm, ABSOLUTE_ERROR)
    }

    @Test
    fun normalize_whenInvalidJacobianSize_throwsException() {
        val randomizer = UniformRandomizer()
        val a = randomizer.nextDouble()
        val b = randomizer.nextDouble()
        val c = randomizer.nextDouble()
        val d = randomizer.nextDouble()

        val q = Quaternion(a, b, c, d)

        assertThrows(IllegalArgumentException::class.java) {
            QuaternionHelper.normalize(q, Matrix(1, Quaternion.N_PARAMS))
        }
        assertThrows(IllegalArgumentException::class.java) {
            QuaternionHelper.normalize(q, Matrix(Quaternion.N_PARAMS, 1))
        }
    }

    @Test
    fun normalize_whenValidJacobianSize_normalizesAndComputesExpectedJacobian() {
        val randomizer = UniformRandomizer()
        val a = randomizer.nextDouble()
        val b = randomizer.nextDouble()
        val c = randomizer.nextDouble()
        val d = randomizer.nextDouble()

        val q = Quaternion(a, b, c, d)

        assertFalse(q.isNormalized)

        // normalize
        val jacobian = Matrix(Quaternion.N_PARAMS, Quaternion.N_PARAMS)
        QuaternionHelper.normalize(q, jacobian)

        // check
        assertTrue(q.isNormalized)
        val norm = sqrt(q.a * q.a + q.b * q.b + q.c * q.c + q.d * q.d)
        assertEquals(1.0, norm, ABSOLUTE_ERROR)

        val jacobianEstimator = JacobianEstimator(object : MultiVariateFunctionEvaluatorListener {
            override fun evaluate(point: DoubleArray, result: DoubleArray) {
                val a0 = point[0]
                val b0 = point[1]
                val c0 = point[2]
                val d0 = point[3]

                val norm0 = sqrt(a0 * a0 + b0 * b0 + c0 * c0 + d0 * d0)

                result[0] = a0 / norm0
                result[1] = b0 / norm0
                result[2] = c0 / norm0
                result[3] = d0 / norm0
            }

            override fun getNumberOfVariables(): Int {
                return Quaternion.N_PARAMS
            }
        })

        val jacobian2 = jacobianEstimator.jacobian(doubleArrayOf(a, b, c, d))

        assertTrue(jacobian.equals(jacobian2, LARGE_ABSOLUTE_ERROR))
    }

    private companion object {
        const val ABSOLUTE_ERROR = 1e-12

        const val LARGE_ABSOLUTE_ERROR = 1e-6
    }
}