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
package com.irurueta.android.navigation.inertial.old.collectors.interpolators

import com.irurueta.android.navigation.inertial.collectors.measurements.GravitySensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.statistics.UniformRandomizer
import org.junit.Assert.*
import org.junit.Test

class GravityLinearSensorMeasurementInterpolatorTest {

    @Test
    fun constructor_whenEmpty_setsExpectedProperties() {
        val interpolator = GravityLinearSensorMeasurementInterpolator()

        assertTrue(interpolator.copyIfNotInitialized)
    }

    @Test
    fun constructor_whenNotEmpty_setsExpectedProperties() {
        val interpolator = GravityLinearSensorMeasurementInterpolator(false)

        assertFalse(interpolator.copyIfNotInitialized)
    }

    @Test
    fun push_whenOnce_setsExpectedValues() {
        val interpolator = GravityLinearSensorMeasurementInterpolator()

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
        val measurement1b: GravitySensorMeasurement? = getPrivateProperty(
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
        val interpolator = GravityLinearSensorMeasurementInterpolator()

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
        val measurement1b: GravitySensorMeasurement? = getPrivateProperty(
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

        val measurement0b: GravitySensorMeasurement? = getPrivateProperty(
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
        val interpolator = GravityLinearSensorMeasurementInterpolator(copyIfNotInitialized = false)

        val currentMeasurement = generateMeasurement()
        val timestamp = System.nanoTime()
        val result = GravitySensorMeasurement()
        assertFalse(interpolator.interpolate(currentMeasurement, timestamp, result))
    }

    @Test
    fun interpolate_whenNotInitializedAndCopyIfNotInitializedEnabled_returnsFalse() {
        val interpolator = GravityLinearSensorMeasurementInterpolator(copyIfNotInitialized = true)

        val currentMeasurement = generateMeasurement()
        val timestamp = currentMeasurement.timestamp
        val result = GravitySensorMeasurement()
        assertTrue(interpolator.interpolate(currentMeasurement, timestamp, result))
        assertMeasurement(currentMeasurement, result)
    }

    @Test
    fun interpolate_whenInitialized_returnsTrue() {
        val interpolator = GravityLinearSensorMeasurementInterpolator()

        val measurement0 = generateMeasurement()
        val measurement1 = generateMeasurement()

        interpolator.push(measurement0)
        interpolator.push(measurement1)

        // check with measurement
        val currentMeasurement = generateMeasurement()
        val timestamp0 = measurement0.timestamp
        val result = GravitySensorMeasurement()

        assertTrue(interpolator.interpolate(currentMeasurement, timestamp0, result))
        assertMeasurement(measurement0, result)

        val timestamp1 = measurement1.timestamp
        assertTrue(interpolator.interpolate(currentMeasurement, timestamp1, result))
        assertMeasurement(measurement1, result)
    }

    @Test
    fun interpolate_whenFutureTimestamp_returnsExpectedResult() {
        val interpolator = GravityLinearSensorMeasurementInterpolator()

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
        val result = GravitySensorMeasurement()

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

    private companion object {

        const val SIZE = 3

        const val ABSOLUTE_ERROR = 5e-6f

        fun assertMeasurement(
            measurement1: GravitySensorMeasurement,
            measurement2: GravitySensorMeasurement
        ) {
            assertNotSame(measurement1, measurement2)
            assertEquals(measurement1.gx, measurement2.gx, ABSOLUTE_ERROR)
            assertEquals(measurement1.gy, measurement2.gy, ABSOLUTE_ERROR)
            assertEquals(measurement1.gz, measurement2.gz, ABSOLUTE_ERROR)
            assertEquals(measurement1.timestamp, measurement2.timestamp)
            assertEquals(measurement1.accuracy, measurement2.accuracy)
        }

        fun generateMeasurement(
            a: DoubleArray,
            b: DoubleArray,
            x: Double
        ): GravitySensorMeasurement {
            val gx = (a[0] * x + b[0]).toFloat()
            val gy = (a[1] * x + b[1]).toFloat()
            val gz = (a[2] * x + b[2]).toFloat()
            val timestamp = System.nanoTime()
            return GravitySensorMeasurement(gx, gy, gz, timestamp, SensorAccuracy.LOW)
        }

        fun generateMeasurement(): GravitySensorMeasurement {
            val randomizer = UniformRandomizer()
            val a = randomizer.nextDoubles(SIZE)
            val b = randomizer.nextDoubles(SIZE)
            val x = randomizer.nextDouble()
            return generateMeasurement(a, b, x)
        }
    }
}