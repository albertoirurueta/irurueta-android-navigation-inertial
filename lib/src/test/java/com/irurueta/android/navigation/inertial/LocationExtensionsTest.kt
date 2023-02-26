/*
 * Copyright (C) 2021 Alberto Irurueta Carro (alberto@irurueta.com)
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
import com.irurueta.navigation.frames.NEDPosition
import com.irurueta.statistics.UniformRandomizer
import io.mockk.clearAllMocks
import io.mockk.every
import io.mockk.mockk
import io.mockk.unmockkAll
import org.junit.After
import org.junit.Assert.assertEquals
import org.junit.Test

class LocationExtensionsTest {

    @After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
    }

    @Test
    fun toNEDPosition_returnsExpectedValue() {
        val randomizer = UniformRandomizer()
        val latitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val longitudeDegrees = randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES)
        val latitudeRadians = Math.toRadians(latitudeDegrees)
        val longitudeRadians = Math.toRadians(longitudeDegrees)

        val height = randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT)

        val location = mockk<Location>()
        every { location.latitude }.returns(latitudeDegrees)
        every { location.longitude }.returns(longitudeDegrees)
        every { location.altitude }.returns(height)

        val nedPosition1 = NEDPosition()
        location.toNEDPosition(nedPosition1)
        val nedPosition2 = location.toNEDPosition()

        assertEquals(nedPosition1, nedPosition2)
        assertEquals(latitudeRadians, nedPosition1.latitude, 0.0)
        assertEquals(longitudeRadians, nedPosition1.longitude, 0.0)
        assertEquals(height, nedPosition1.height, 0.0)
    }

    private companion object {
        const val MIN_ANGLE_DEGREES = -90.0
        const val MAX_ANGLE_DEGREES = 90.0

        const val MIN_HEIGHT = -100.0
        const val MAX_HEIGHT = 3000.0
    }
}