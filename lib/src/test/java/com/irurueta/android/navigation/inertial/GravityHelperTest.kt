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

import android.location.Location
import com.irurueta.navigation.frames.ECEFPosition
import com.irurueta.navigation.frames.ECEFVelocity
import com.irurueta.navigation.frames.NEDPosition
import com.irurueta.navigation.frames.NEDVelocity
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter
import com.irurueta.navigation.inertial.estimators.ECEFGravityEstimator
import com.irurueta.statistics.UniformRandomizer
import io.mockk.clearAllMocks
import io.mockk.every
import io.mockk.mockk
import io.mockk.unmockkAll
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNotSame
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class GravityHelperTest {

    @After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
    }

    @Test
    fun getGravityForLocation_returnsExpectedValue() {
        val location = getLocation()

        val nedPosition = NEDPosition(
            Math.toRadians(location.latitude),
            Math.toRadians(location.longitude),
            location.altitude
        )
        val nedVelocity = NEDVelocity()
        val ecefPosition = ECEFPosition()
        val ecefVelocity = ECEFVelocity()
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
            nedPosition,
            nedVelocity,
            ecefPosition,
            ecefVelocity
        )
        val expectedGravity = ECEFGravityEstimator.estimateGravityAndReturnNew(
            ecefPosition.x,
            ecefPosition.y,
            ecefPosition.z
        )

        val result = GravityHelper.getGravityForLocation(location)

        assertNotSame(expectedGravity, result)
        assertEquals(expectedGravity, result)
    }

    @Test
    fun getGravityForPosition_returnsExpectedValue() {
        val location = getLocation()

        val nedPosition = NEDPosition(
            Math.toRadians(location.latitude),
            Math.toRadians(location.longitude),
            location.altitude
        )
        val expectedGravity = GravityHelper.getGravityForLocation(location)

        val result = GravityHelper.getGravityForPosition(nedPosition)

        assertNotSame(expectedGravity, result)
        assertEquals(expectedGravity, result)
    }

    @Test
    fun getGravityNormForLocation_returnsExpectedValue() {
        val location = getLocation()
        val gravity = GravityHelper.getGravityForLocation(location)
        val expectedNorm = gravity.norm

        val result = GravityHelper.getGravityNormForLocation(location)

        assertEquals(expectedNorm, result, 0.0)
    }

    private companion object {
        const val MIN_ANGLE_DEGREES = -90.0
        const val MAX_ANGLE_DEGREES = 90.0

        const val MIN_HEIGHT = -100.0
        const val MAX_HEIGHT = 3000.0

        private fun getLocation(): Location {
            val randomizer = UniformRandomizer()
            val latitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
            val longitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
            val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

            val location = mockk<Location>()
            every { location.latitude }.returns(latitudeDegrees)
            every { location.longitude }.returns(longitudeDegrees)
            every { location.altitude }.returns(height)

            return location
        }
    }
}