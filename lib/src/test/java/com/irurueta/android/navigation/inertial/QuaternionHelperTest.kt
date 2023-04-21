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

import com.irurueta.geometry.Quaternion
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
    fun sqrNorm_returnsExpectedValue() {
        val randomizer = UniformRandomizer()
        val a = randomizer.nextDouble()
        val b = randomizer.nextDouble()
        val c = randomizer.nextDouble()
        val d = randomizer.nextDouble()

        val q = Quaternion(a, b, c, d)

        assertEquals(a * a + b * b + c * c + d * d, QuaternionHelper.sqrNorm(q), 0.0)
    }

    @Test
    fun norm_returnsExpectedValue() {
        val randomizer = UniformRandomizer()
        val a = randomizer.nextDouble()
        val b = randomizer.nextDouble()
        val c = randomizer.nextDouble()
        val d = randomizer.nextDouble()

        val q = Quaternion(a, b, c, d)

        assertEquals(sqrt(QuaternionHelper.sqrNorm(q)), QuaternionHelper.norm(q), 0.0)
    }
}