package com.irurueta.android.navigation.inertial

import com.irurueta.geometry.Quaternion
import com.irurueta.statistics.UniformRandomizer
import org.junit.Assert.*
import org.junit.Test

class QuaternionHelperTest {

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
}