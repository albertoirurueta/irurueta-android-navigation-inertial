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
package com.irurueta.android.navigation.inertial.collectors.interpolators

import com.irurueta.android.navigation.inertial.collectors.MagnetometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.android.navigation.inertial.getPrivateProperty
import com.irurueta.statistics.UniformRandomizer
import io.mockk.clearAllMocks
import io.mockk.unmockkAll
import org.junit.After
import org.junit.Assert.*
import org.junit.Test

class MagnetometerQuadraticSensorMeasurementInterpolatorTest {

    @After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
    }

    @Test
    fun constructor_whenEmpty_setsExpectedProperties() {
        val interpolator = MagnetometerQuadraticSensorMeasurementInterpolator()

        assertTrue(interpolator.copyIfNotInitialized)
    }

    @Test
    fun constructor_whenNotEmpty_setsExpectedProperties() {
        val interpolator = MagnetometerQuadraticSensorMeasurementInterpolator(false)

        assertFalse(interpolator.copyIfNotInitialized)
    }

    @Test
    fun push_whenOnce_setsExpectedValues() {
        val interpolator = MagnetometerQuadraticSensorMeasurementInterpolator()

        // check initial values
        val hasMeasurement2a: Boolean? = getPrivateProperty(
            QuadraticSensorMeasurementInterpolator::class,
            interpolator,
            "hasMeasurement2"
        )
        requireNotNull(hasMeasurement2a)
        assertFalse(hasMeasurement2a)

        val hasMeasurement1a: Boolean? = getPrivateProperty(
            QuadraticSensorMeasurementInterpolator::class,
            interpolator,
            "hasMeasurement1"
        )
        requireNotNull(hasMeasurement1a)
        assertFalse(hasMeasurement1a)

        val hasMeasurement0a: Boolean? = getPrivateProperty(
            QuadraticSensorMeasurementInterpolator::class,
            interpolator,
            "hasMeasurement0"
        )
        requireNotNull(hasMeasurement0a)
        assertFalse(hasMeasurement0a)

        val measurement2a = generateMeasurement()

        interpolator.push(measurement2a)

        // check
        val measurement2b: MagnetometerSensorMeasurement? = getPrivateProperty(
            QuadraticSensorMeasurementInterpolator::class,
            interpolator,
            "measurement2"
        )
        requireNotNull(measurement2b)
        assertMeasurement(measurement2a, measurement2b)

        val hasMeasurement2b: Boolean? = getPrivateProperty(
            QuadraticSensorMeasurementInterpolator::class,
            interpolator,
            "hasMeasurement2"
        )
        requireNotNull(hasMeasurement2b)
        assertTrue(hasMeasurement2b)

        val hasMeasurement0b: Boolean? = getPrivateProperty(
            QuadraticSensorMeasurementInterpolator::class,
            interpolator,
            "hasMeasurement0"
        )
        requireNotNull(hasMeasurement0b)
        assertFalse(hasMeasurement0b)

        // reset
        interpolator.reset()

        // check
        val hasMeasurement2c: Boolean? = getPrivateProperty(
            QuadraticSensorMeasurementInterpolator::class,
            interpolator,
            "hasMeasurement2"
        )
        requireNotNull(hasMeasurement2c)
        assertFalse(hasMeasurement2c)

        val hasMeasurement1c: Boolean? = getPrivateProperty(
            QuadraticSensorMeasurementInterpolator::class,
            interpolator,
            "hasMeasurement1"
        )
        requireNotNull(hasMeasurement1c)
        assertFalse(hasMeasurement1c)

        val hasMeasurement0c: Boolean? = getPrivateProperty(
            QuadraticSensorMeasurementInterpolator::class,
            interpolator,
            "hasMeasurement0"
        )
        requireNotNull(hasMeasurement0c)
        assertFalse(hasMeasurement0c)
    }

    @Test
    fun push_whenTwice_setsExpectedValues() {
        val interpolator = MagnetometerQuadraticSensorMeasurementInterpolator()

        // check initial values
        val hasMeasurement2a: Boolean? = getPrivateProperty(
            QuadraticSensorMeasurementInterpolator::class,
            interpolator,
            "hasMeasurement2"
        )
        requireNotNull(hasMeasurement2a)
        assertFalse(hasMeasurement2a)

        val hasMeasurement1a: Boolean? = getPrivateProperty(
            QuadraticSensorMeasurementInterpolator::class,
            interpolator,
            "hasMeasurement1"
        )
        requireNotNull(hasMeasurement1a)
        assertFalse(hasMeasurement1a)

        val hasMeasurement0a: Boolean? = getPrivateProperty(
            QuadraticSensorMeasurementInterpolator::class,
            interpolator,
            "hasMeasurement0"
        )
        requireNotNull(hasMeasurement0a)
        assertFalse(hasMeasurement0a)

        val measurement1a = generateMeasurement()
        val measurement2a = generateMeasurement()

        interpolator.push(measurement1a)
        interpolator.push(measurement2a)

        // check
        val measurement2b: MagnetometerSensorMeasurement? = getPrivateProperty(
            QuadraticSensorMeasurementInterpolator::class,
            interpolator,
            "measurement2"
        )
        requireNotNull(measurement2b)
        assertMeasurement(measurement2a, measurement2b)

        val hasMeasurement2b: Boolean? = getPrivateProperty(
            QuadraticSensorMeasurementInterpolator::class,
            interpolator,
            "hasMeasurement2"
        )
        requireNotNull(hasMeasurement2b)
        assertTrue(hasMeasurement2b)

        val measurement1b: MagnetometerSensorMeasurement? = getPrivateProperty(
            QuadraticSensorMeasurementInterpolator::class,
            interpolator,
            "measurement1"
        )
        requireNotNull(measurement1b)
        assertMeasurement(measurement1a, measurement1b)

        val hasMeasurement1b: Boolean? = getPrivateProperty(
            QuadraticSensorMeasurementInterpolator::class,
            interpolator,
            "hasMeasurement1"
        )
        requireNotNull(hasMeasurement1b)
        assertTrue(hasMeasurement1b)

        // reset
        interpolator.reset()

        // check
        val hasMeasurement2c: Boolean? = getPrivateProperty(
            QuadraticSensorMeasurementInterpolator::class,
            interpolator,
            "hasMeasurement2"
        )
        requireNotNull(hasMeasurement2c)
        assertFalse(hasMeasurement2c)

        val hasMeasurement1c: Boolean? = getPrivateProperty(
            QuadraticSensorMeasurementInterpolator::class,
            interpolator,
            "hasMeasurement1"
        )
        requireNotNull(hasMeasurement1c)
        assertFalse(hasMeasurement1c)

        val hasMeasurement0c: Boolean? = getPrivateProperty(
            QuadraticSensorMeasurementInterpolator::class,
            interpolator,
            "hasMeasurement0"
        )
        requireNotNull(hasMeasurement0c)
        assertFalse(hasMeasurement0c)
    }

    @Test
    fun push_whenThreeTimes_setsExpectedValues() {
        val interpolator = MagnetometerQuadraticSensorMeasurementInterpolator()

        // check initial values
        val hasMeasurement2a: Boolean? = getPrivateProperty(
            QuadraticSensorMeasurementInterpolator::class,
            interpolator,
            "hasMeasurement2"
        )
        requireNotNull(hasMeasurement2a)
        assertFalse(hasMeasurement2a)

        val hasMeasurement1a: Boolean? = getPrivateProperty(
            QuadraticSensorMeasurementInterpolator::class,
            interpolator,
            "hasMeasurement1"
        )
        requireNotNull(hasMeasurement1a)
        assertFalse(hasMeasurement1a)

        val hasMeasurement0a: Boolean? = getPrivateProperty(
            QuadraticSensorMeasurementInterpolator::class,
            interpolator,
            "hasMeasurement0"
        )
        requireNotNull(hasMeasurement0a)
        assertFalse(hasMeasurement0a)

        val measurement0a = generateMeasurement()
        val measurement1a = generateMeasurement()
        val measurement2a = generateMeasurement()

        interpolator.push(measurement0a)
        interpolator.push(measurement1a)
        interpolator.push(measurement2a)

        // check
        val measurement2b: MagnetometerSensorMeasurement? = getPrivateProperty(
            QuadraticSensorMeasurementInterpolator::class,
            interpolator,
            "measurement2"
        )
        requireNotNull(measurement2b)
        assertMeasurement(measurement2a, measurement2b)

        val hasMeasurement2b: Boolean? = getPrivateProperty(
            QuadraticSensorMeasurementInterpolator::class,
            interpolator,
            "hasMeasurement2"
        )
        requireNotNull(hasMeasurement2b)
        assertTrue(hasMeasurement2b)

        val measurement1b: MagnetometerSensorMeasurement? = getPrivateProperty(
            QuadraticSensorMeasurementInterpolator::class,
            interpolator,
            "measurement1"
        )
        requireNotNull(measurement1b)
        assertMeasurement(measurement1a, measurement1b)

        val hasMeasurement1b: Boolean? = getPrivateProperty(
            QuadraticSensorMeasurementInterpolator::class,
            interpolator,
            "hasMeasurement1"
        )
        requireNotNull(hasMeasurement1b)
        assertTrue(hasMeasurement1b)

        val measurement0b: MagnetometerSensorMeasurement? = getPrivateProperty(
            QuadraticSensorMeasurementInterpolator::class,
            interpolator,
            "measurement0"
        )
        requireNotNull(measurement0b)
        assertMeasurement(measurement0a, measurement0b)

        val hasMeasurement0b: Boolean? = getPrivateProperty(
            QuadraticSensorMeasurementInterpolator::class,
            interpolator,
            "hasMeasurement0"
        )
        requireNotNull(hasMeasurement0b)
        assertTrue(hasMeasurement0b)

        // reset
        interpolator.reset()

        // check
        val hasMeasurement2c: Boolean? = getPrivateProperty(
            QuadraticSensorMeasurementInterpolator::class,
            interpolator,
            "hasMeasurement2"
        )
        requireNotNull(hasMeasurement2c)
        assertFalse(hasMeasurement2c)

        val hasMeasurement1c: Boolean? = getPrivateProperty(
            QuadraticSensorMeasurementInterpolator::class,
            interpolator,
            "hasMeasurement1"
        )
        requireNotNull(hasMeasurement1c)
        assertFalse(hasMeasurement1c)

        val hasMeasurement0c: Boolean? = getPrivateProperty(
            QuadraticSensorMeasurementInterpolator::class,
            interpolator,
            "hasMeasurement0"
        )
        requireNotNull(hasMeasurement0c)
        assertFalse(hasMeasurement0c)
    }

    @Test
    fun interpolate_whenNotInitializedAndCopyIfNotInitializedDisabled_returnsFalse() {
        val interpolator =
            MagnetometerQuadraticSensorMeasurementInterpolator(copyIfNotInitialized = false)

        val currentMeasurement = generateMeasurement()
        val timestamp = System.nanoTime()
        val result = MagnetometerSensorMeasurement()
        assertFalse(interpolator.interpolate(currentMeasurement, timestamp, result))
    }

    @Test
    fun interpolate_whenNotInitializedAndCopyIfNotInitializedEnabled_returnsFalse() {
        val interpolator =
            MagnetometerQuadraticSensorMeasurementInterpolator(copyIfNotInitialized = true)

        val currentMeasurement = generateMeasurement()
        val timestamp = currentMeasurement.timestamp
        val result = MagnetometerSensorMeasurement()
        assertTrue(interpolator.interpolate(currentMeasurement, timestamp, result))
        assertMeasurement(currentMeasurement, result)
    }

    @Test
    fun interpolate_whenInitialized_returnsTrue() {
        val interpolator = MagnetometerQuadraticSensorMeasurementInterpolator()

        val measurement0 = generateMeasurement()
        val measurement1 = generateMeasurement()
        val measurement2 = generateMeasurement()

        interpolator.push(measurement0)
        interpolator.push(measurement1)
        interpolator.push(measurement2)

        // check with measurement
        val currentMeasurement = generateMeasurement()
        val timestamp0 = measurement0.timestamp
        val result = MagnetometerSensorMeasurement()

        assertTrue(interpolator.interpolate(currentMeasurement, timestamp0, result))
        assertMeasurement(measurement0, result)

        val timestamp1 = measurement1.timestamp
        assertTrue(interpolator.interpolate(currentMeasurement, timestamp1, result))
        assertMeasurement(measurement1, result)

        val timestamp2 = measurement2.timestamp
        assertTrue(interpolator.interpolate(currentMeasurement, timestamp2, result))
        assertMeasurement(measurement2, result)
    }

    @Test
    fun interpolate_whenFutureTimestamp_returnsExpectedResult() {
        val interpolator = MagnetometerQuadraticSensorMeasurementInterpolator()

        val randomizer = UniformRandomizer()
        val a = randomizer.nextDoubles(SIZE)
        val b = randomizer.nextDoubles(SIZE)
        val c = randomizer.nextDoubles(SIZE)
        val measurement0 = generateMeasurement(a, b, c, 0.0)
        val measurement1 = generateMeasurement(a, b, c, 1.0)
        val timestamp2 = System.nanoTime()
        val x =
            (timestamp2 - measurement0.timestamp).toDouble() / (measurement1.timestamp - measurement0.timestamp).toDouble()
        val measurement2 = generateMeasurement(a, b, c, x)
        measurement2.timestamp = timestamp2

        interpolator.push(measurement0)
        interpolator.push(measurement1)
        interpolator.push(measurement2)

        // check with previous measurements
        val currentMeasurement = generateMeasurement()
        val timestamp0 = measurement0.timestamp
        val result = MagnetometerSensorMeasurement()

        assertTrue(interpolator.interpolate(currentMeasurement, timestamp0, result))
        assertMeasurement(measurement0, result)

        val timestamp1 = measurement1.timestamp
        assertTrue(interpolator.interpolate(currentMeasurement, timestamp1, result))
        assertMeasurement(measurement1, result)

        assertTrue(interpolator.interpolate(currentMeasurement, timestamp2, result))
        assertMeasurement(measurement2, result)

        // check with future measurement
        val timestamp = System.nanoTime()
        val xB = (timestamp - timestamp0).toDouble() / (timestamp1 - timestamp0).toDouble()
        val expectedMeasurement = generateMeasurement(a, b, c, xB)
        expectedMeasurement.timestamp = timestamp

        assertTrue(interpolator.interpolate(currentMeasurement, timestamp, result))
        assertMeasurement(expectedMeasurement, result)
    }

    @Test
    fun interpolate_whenNoBias_returnsTrue() {
        val interpolator = MagnetometerQuadraticSensorMeasurementInterpolator()

        val measurement0 = generateMeasurement(false)
        val measurement1 = generateMeasurement(false)
        val measurement2 = generateMeasurement(false)

        interpolator.push(measurement0)
        interpolator.push(measurement1)
        interpolator.push(measurement2)

        // check with measurement
        val currentMeasurement = generateMeasurement()
        val timestamp0 = measurement0.timestamp
        val result = MagnetometerSensorMeasurement()

        assertTrue(interpolator.interpolate(currentMeasurement, timestamp0, result))
        assertMeasurement(measurement0, result)

        val timestamp1 = measurement1.timestamp
        assertTrue(interpolator.interpolate(currentMeasurement, timestamp1, result))
        assertMeasurement(measurement1, result)

        val timestamp2 = measurement2.timestamp
        assertTrue(interpolator.interpolate(currentMeasurement, timestamp2, result))
        assertMeasurement(measurement2, result)
    }

    private companion object {

        const val SIZE = 6

        const val ABSOLUTE_ERROR = 1e-3f

        fun assertMeasurement(
            measurement1: MagnetometerSensorMeasurement,
            measurement2: MagnetometerSensorMeasurement
        ) {
            assertNotSame(measurement1, measurement2)
            assertEquals(measurement1.bx, measurement2.bx, ABSOLUTE_ERROR)
            assertEquals(measurement1.by, measurement2.by, ABSOLUTE_ERROR)
            assertEquals(measurement1.bz, measurement2.bz, ABSOLUTE_ERROR)
            assertEquals(
                measurement1.hardIronX ?: 0.0f,
                measurement2.hardIronX ?: 0.0f,
                ABSOLUTE_ERROR
            )
            assertEquals(
                measurement1.hardIronY ?: 0.0f,
                measurement2.hardIronY ?: 0.0f,
                ABSOLUTE_ERROR
            )
            assertEquals(
                measurement1.hardIronZ ?: 0.0f,
                measurement2.hardIronZ ?: 0.0f,
                ABSOLUTE_ERROR
            )
            assertEquals(measurement1.timestamp, measurement2.timestamp)
            assertEquals(measurement1.accuracy, measurement2.accuracy)
        }

        fun generateMeasurement(
            a: DoubleArray,
            b: DoubleArray,
            c: DoubleArray,
            x: Double,
            withHardIron: Boolean = true
        ): MagnetometerSensorMeasurement {
            val x2 = x * x
            val bx = (a[0] * x2 + b[0] * x + c[0]).toFloat()
            val by = (a[1] * x2 + b[1] * x + c[1]).toFloat()
            val bz = (a[2] * x2 + b[2] * x + c[2]).toFloat()
            val hardIronX = if (withHardIron) {
                (a[3] * x2 + b[3] * x + c[3]).toFloat()
            } else {
                null
            }
            val hardIronY = if (withHardIron) {
                (a[4] * x2 + b[4] * x + c[4]).toFloat()
            } else {
                null
            }
            val hardIronZ = if (withHardIron) {
                (a[5] * x2 + b[5] * x + c[5]).toFloat()
            } else {
                null
            }
            val timestamp = System.nanoTime()
            return MagnetometerSensorMeasurement(
                bx,
                by,
                bz,
                hardIronX,
                hardIronY,
                hardIronZ,
                timestamp,
                SensorAccuracy.HIGH
            )
        }

        fun generateMeasurement(withHardIron: Boolean = true): MagnetometerSensorMeasurement {
            val randomizer = UniformRandomizer()
            val a = randomizer.nextDoubles(SIZE)
            val b = randomizer.nextDoubles(SIZE)
            val c = randomizer.nextDoubles(SIZE)
            val x = randomizer.nextDouble()
            return generateMeasurement(a, b, c, x, withHardIron)
        }
    }
}