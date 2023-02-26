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

import com.irurueta.android.navigation.inertial.ENUtoNEDTriadConverter
import com.irurueta.android.navigation.inertial.collectors.AttitudeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.geometry.Quaternion
import com.irurueta.statistics.UniformRandomizer
import io.mockk.clearAllMocks
import io.mockk.mockk
import io.mockk.unmockkAll
import io.mockk.verify
import org.junit.After
import org.junit.Assert.*
import org.junit.Test

class AttitudeProcessorTest {

    @After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
    }

    @Test
    fun constructor_whenNoParameters_returnsExpectedValues() {
        val processor = AttitudeProcessor()

        assertNull(processor.processorListener)
        assertNotNull(processor.nedAttitude)
        assertEquals(Quaternion(), processor.nedAttitude)
    }

    @Test
    fun constructor_whenListener_returnsExpectedValues() {
        val listener = mockk<AttitudeProcessor.OnProcessedListener>()
        val processor = AttitudeProcessor(listener)

        assertSame(listener, processor.processorListener)
        assertNotNull(processor.nedAttitude)
        assertEquals(Quaternion(), processor.nedAttitude)
    }

    @Test
    fun process_whenNoListener_returnsExpectedValue() {
        val processor = AttitudeProcessor()

        val enuAttitude = getAttitude()
        val randomizer = UniformRandomizer()
        val headingAccuracy =
            Math.toRadians(randomizer.nextDouble(0.0, MAX_ANGLE_DEGREES)).toFloat()
        val timestamp = System.nanoTime()
        val measurement =
            AttitudeSensorMeasurement(enuAttitude, headingAccuracy, timestamp, SensorAccuracy.HIGH)

        val nedAttitude = processor.process(measurement)

        // check
        val expected = Quaternion()
        ENUtoNEDTriadConverter.convert(enuAttitude, expected)
        assertEquals(expected, nedAttitude)
        assertSame(nedAttitude, processor.nedAttitude)
    }

    @Test
    fun process_whenListener_returnsExpectedValue() {
        val listener = mockk<AttitudeProcessor.OnProcessedListener>(relaxUnitFun = true)
        val processor = AttitudeProcessor(listener)

        val enuAttitude = getAttitude()
        val randomizer = UniformRandomizer()
        val headingAccuracy =
            Math.toRadians(randomizer.nextDouble(0.0, MAX_ANGLE_DEGREES)).toFloat()
        val timestamp = System.nanoTime()
        val measurement =
            AttitudeSensorMeasurement(enuAttitude, headingAccuracy, timestamp, SensorAccuracy.HIGH)

        val nedAttitude = processor.process(measurement)

        // check
        val expected = Quaternion()
        ENUtoNEDTriadConverter.convert(enuAttitude, expected)
        assertEquals(expected, nedAttitude)
        assertSame(nedAttitude, processor.nedAttitude)

        verify(exactly = 1) {
            listener.onProcessed(
                processor,
                nedAttitude,
                timestamp,
                SensorAccuracy.HIGH
            )
        }
    }

    private companion object {
        const val MIN_ANGLE_DEGREES = -45.0
        const val MAX_ANGLE_DEGREES = 45.0

        fun getAttitude(): Quaternion {
            val randomizer = UniformRandomizer()
            val roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))

            return Quaternion(roll, pitch, yaw)
        }
    }
}