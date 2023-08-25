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

import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.android.navigation.inertial.getPrivateProperty
import com.irurueta.statistics.UniformRandomizer
import io.mockk.clearAllMocks
import io.mockk.unmockkAll
import org.junit.After
import org.junit.Assert.*
import org.junit.Test

class AccelerometerLinearSensorMeasurementInterpolatorTest {

    @After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
    }

    @Test
    fun constructor_whenEmpty_setsExpectedProperties() {
        val interpolator = AccelerometerLinearSensorMeasurementInterpolator()

        assertTrue(interpolator.copyIfNotInitialized)
    }

    @Test
    fun constructor_whenNotEmpty_setsExpectedProperties() {
        val interpolator = AccelerometerLinearSensorMeasurementInterpolator(false)

        assertFalse(interpolator.copyIfNotInitialized)
    }

    @Test
    fun push_whenOnce_setsExpectedValues() {
        val interpolator = AccelerometerLinearSensorMeasurementInterpolator()

        // check initial values
        val hasMeasurement1a: Boolean? = getPrivateProperty(
            LinearSensorMeasurementInterpolator::class,
            interpolator,
            "hasMeasurement1"
        )
        requireNotNull(hasMeasurement1a)
        assertFalse(hasMeasurement1a)

        val hasMeasurement0a: Boolean? = getPrivateProperty(
            LinearSensorMeasurementInterpolator::class,
            interpolator,
            "hasMeasurement0"
        )
        requireNotNull(hasMeasurement0a)
        assertFalse(hasMeasurement0a)

        val measurement1a = generateMeasurement()

        interpolator.push(measurement1a)

        // check
        val measurement1b: AccelerometerSensorMeasurement? = getPrivateProperty(
            LinearSensorMeasurementInterpolator::class,
            interpolator,
            "measurement1"
        )
        requireNotNull(measurement1b)
        assertMeasurement(measurement1a, measurement1b)

        val hasMeasurement1b: Boolean? = getPrivateProperty(
            LinearSensorMeasurementInterpolator::class,
            interpolator,
            "hasMeasurement1"
        )
        requireNotNull(hasMeasurement1b)
        assertTrue(hasMeasurement1b)

        val hasMeasurement0b: Boolean? = getPrivateProperty(
            LinearSensorMeasurementInterpolator::class,
            interpolator,
            "hasMeasurement0"
        )
        requireNotNull(hasMeasurement0b)
        assertFalse(hasMeasurement0b)

        // reset
        interpolator.reset()

        // check
        val hasMeasurement1c: Boolean? = getPrivateProperty(
            LinearSensorMeasurementInterpolator::class,
            interpolator,
            "hasMeasurement1"
        )
        requireNotNull(hasMeasurement1c)
        assertFalse(hasMeasurement1c)

        val hasMeasurement0c: Boolean? = getPrivateProperty(
            LinearSensorMeasurementInterpolator::class,
            interpolator,
            "hasMeasurement0"
        )
        requireNotNull(hasMeasurement0c)
        assertFalse(hasMeasurement0c)
    }

    @Test
    fun push_whenTwice_setsExpectedValues() {
        val interpolator = AccelerometerLinearSensorMeasurementInterpolator()

        // check initial values
        val hasMeasurement1a: Boolean? = getPrivateProperty(
            LinearSensorMeasurementInterpolator::class,
            interpolator,
            "hasMeasurement1"
        )
        requireNotNull(hasMeasurement1a)
        assertFalse(hasMeasurement1a)

        val hasMeasurement0a: Boolean? = getPrivateProperty(
            LinearSensorMeasurementInterpolator::class,
            interpolator,
            "hasMeasurement0"
        )
        requireNotNull(hasMeasurement0a)
        assertFalse(hasMeasurement0a)

        val measurement0a = generateMeasurement()
        val measurement1a = generateMeasurement()

        interpolator.push(measurement0a)
        interpolator.push(measurement1a)

        // check
        val measurement1b: AccelerometerSensorMeasurement? = getPrivateProperty(
            LinearSensorMeasurementInterpolator::class,
            interpolator,
            "measurement1"
        )
        requireNotNull(measurement1b)
        assertMeasurement(measurement1a, measurement1b)

        val hasMeasurement1b: Boolean? = getPrivateProperty(
            LinearSensorMeasurementInterpolator::class,
            interpolator,
            "hasMeasurement1"
        )
        requireNotNull(hasMeasurement1b)
        assertTrue(hasMeasurement1b)

        val measurement0b: AccelerometerSensorMeasurement? = getPrivateProperty(
            LinearSensorMeasurementInterpolator::class,
            interpolator,
            "measurement0"
        )
        requireNotNull(measurement0b)
        assertMeasurement(measurement0a, measurement0b)

        val hasMeasurement0b: Boolean? = getPrivateProperty(
            LinearSensorMeasurementInterpolator::class,
            interpolator,
            "hasMeasurement0"
        )
        requireNotNull(hasMeasurement0b)
        assertTrue(hasMeasurement0b)

        // reset
        interpolator.reset()

        // check
        val hasMeasurement1c: Boolean? = getPrivateProperty(
            LinearSensorMeasurementInterpolator::class,
            interpolator,
            "hasMeasurement1"
        )
        requireNotNull(hasMeasurement1c)
        assertFalse(hasMeasurement1c)

        val hasMeasurement0c: Boolean? = getPrivateProperty(
            LinearSensorMeasurementInterpolator::class,
            interpolator,
            "hasMeasurement0"
        )
        requireNotNull(hasMeasurement0c)
        assertFalse(hasMeasurement0c)
    }

    @Test
    fun interpolate_whenNotInitializedAndCopyIfNotInitializedDisabled_returnsFalse() {
        val interpolator =
            AccelerometerLinearSensorMeasurementInterpolator(copyIfNotInitialized = false)

        val currentMeasurement = generateMeasurement()
        val timestamp = System.nanoTime()
        val result = AccelerometerSensorMeasurement()
        assertFalse(interpolator.interpolate(currentMeasurement, timestamp, result))
    }

    @Test
    fun interpolate_whenNotInitializedAndCopyIfNotInitializedEnabled_returnsFalse() {
        val interpolator =
            AccelerometerLinearSensorMeasurementInterpolator(copyIfNotInitialized = true)

        val currentMeasurement = generateMeasurement()
        val timestamp = currentMeasurement.timestamp
        val result = AccelerometerSensorMeasurement()
        assertTrue(interpolator.interpolate(currentMeasurement, timestamp, result))
        assertMeasurement(currentMeasurement, result)
    }

    @Test
    fun interpolate_whenInitialized_returnsTrue() {
        val interpolator = AccelerometerLinearSensorMeasurementInterpolator()

        val measurement0 = generateMeasurement()
        val measurement1 = generateMeasurement()

        interpolator.push(measurement0)
        interpolator.push(measurement1)

        // check with measurement
        val currentMeasurement = generateMeasurement()
        val timestamp0 = measurement0.timestamp
        val result = AccelerometerSensorMeasurement()

        assertTrue(interpolator.interpolate(currentMeasurement, timestamp0, result))
        assertMeasurement(measurement0, result)

        val timestamp1 = measurement1.timestamp
        assertTrue(interpolator.interpolate(currentMeasurement, timestamp1, result))
        assertMeasurement(measurement1, result)
    }

    @Test
    fun interpolate_whenFutureTimestamp_returnsExpectedResult() {
        val interpolator = AccelerometerLinearSensorMeasurementInterpolator()

        val randomizer = UniformRandomizer()
        val a = randomizer.nextDoubles(SIZE)
        val b = randomizer.nextDoubles(SIZE)
        val measurement0 = generateMeasurement(a, b, 0.0)
        val measurement1 = generateMeasurement(a, b, 1.0)

        interpolator.push(measurement0)
        interpolator.push(measurement1)

        // check with previous measurements
        val currentMeasurement = generateMeasurement()
        val timestamp0 = measurement0.timestamp
        val result = AccelerometerSensorMeasurement()

        assertTrue(interpolator.interpolate(currentMeasurement, timestamp0, result))
        assertMeasurement(measurement0, result)

        val timestamp1 = measurement1.timestamp
        assertTrue(interpolator.interpolate(currentMeasurement, timestamp1, result))
        assertMeasurement(measurement1, result)

        // check with future measurement
        val timestamp = System.nanoTime()
        val x = (timestamp - timestamp0).toDouble() / (timestamp1 - timestamp0).toDouble()
        val expectedMeasurement = generateMeasurement(a, b, x)
        expectedMeasurement.timestamp = timestamp

        assertTrue(interpolator.interpolate(currentMeasurement, timestamp, result))
        assertMeasurement(expectedMeasurement, result)
    }

    @Test
    fun interpolate_whenNoBias_returnsTrue() {
        val interpolator = AccelerometerLinearSensorMeasurementInterpolator()

        val measurement0 = generateMeasurement(false)
        val measurement1 = generateMeasurement(false)

        interpolator.push(measurement0)
        interpolator.push(measurement1)

        // check with measurement
        val currentMeasurement = generateMeasurement()
        val timestamp0 = measurement0.timestamp
        val result = AccelerometerSensorMeasurement()

        assertTrue(interpolator.interpolate(currentMeasurement, timestamp0, result))
        assertMeasurement(measurement0, result)

        val timestamp1 = measurement1.timestamp
        assertTrue(interpolator.interpolate(currentMeasurement, timestamp1, result))
        assertMeasurement(measurement1, result)
    }

    private companion object {

        const val SIZE = 6

        const val ABSOLUTE_ERROR = 5e-5f

        fun assertMeasurement(
            measurement1: AccelerometerSensorMeasurement,
            measurement2: AccelerometerSensorMeasurement
        ) {
            assertNotSame(measurement1, measurement2)
            assertEquals(measurement1.ax, measurement2.ax, ABSOLUTE_ERROR)
            assertEquals(measurement1.ay, measurement2.ay, ABSOLUTE_ERROR)
            assertEquals(measurement1.az, measurement2.az, ABSOLUTE_ERROR)
            assertEquals(measurement1.bx ?: 0.0f, measurement2.bx ?: 0.0f, ABSOLUTE_ERROR)
            assertEquals(measurement1.by ?: 0.0f, measurement2.by ?: 0.0f, ABSOLUTE_ERROR)
            assertEquals(measurement1.bz ?: 0.0f, measurement2.bz ?: 0.0f, ABSOLUTE_ERROR)
            assertEquals(measurement1.timestamp, measurement2.timestamp)
            assertEquals(measurement1.accuracy, measurement2.accuracy)
        }

        fun generateMeasurement(
            a: DoubleArray,
            b: DoubleArray,
            x: Double,
            withBias: Boolean = true
        ): AccelerometerSensorMeasurement {
            val ax = (a[0] * x + b[0]).toFloat()
            val ay = (a[1] * x + b[1]).toFloat()
            val az = (a[2] * x + b[2]).toFloat()
            val bx = if (withBias) {
                (a[3] * x + b[3]).toFloat()
            } else {
                null
            }
            val by = if (withBias) {
                (a[4] * x + b[4]).toFloat()
            } else {
                null
            }
            val bz = if (withBias) {
                (a[5] * x + b[5]).toFloat()
            } else {
                null
            }
            val timestamp = System.nanoTime()
            return AccelerometerSensorMeasurement(
                ax,
                ay,
                az,
                bx,
                by,
                bz,
                timestamp,
                SensorAccuracy.HIGH
            )
        }

        fun generateMeasurement(withBias: Boolean = true): AccelerometerSensorMeasurement {
            val randomizer = UniformRandomizer()
            val a = randomizer.nextDoubles(SIZE)
            val b = randomizer.nextDoubles(SIZE)
            val x = randomizer.nextDouble()
            return generateMeasurement(a, b, x, withBias)
        }
    }
}