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

import com.irurueta.android.navigation.inertial.collectors.AttitudeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.geometry.Quaternion
import com.irurueta.statistics.UniformRandomizer
import org.junit.Assert.*
import org.junit.Test

class AttitudeLinearSensorMeasurementInterpolatorTest {

    @Test
    fun constructor_whenEmpty_setsExpectedProperties() {
        val interpolator = AttitudeLinearSensorMeasurementInterpolator()

        assertTrue(interpolator.copyIfNotInitialized)
    }

    @Test
    fun constructor_whenNotEmpty_setsExpectedProperties() {
        val interpolator = AttitudeLinearSensorMeasurementInterpolator(false)

        assertFalse(interpolator.copyIfNotInitialized)
    }

    @Test
    fun push_whenOnce_setsExpectedValues() {
        val interpolator = AttitudeLinearSensorMeasurementInterpolator()

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
        val measurement1b: AttitudeSensorMeasurement? = getPrivateProperty(
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
        val interpolator = AttitudeLinearSensorMeasurementInterpolator()

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
        val measurement1b: AttitudeSensorMeasurement? = getPrivateProperty(
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

        val measurement0b: AttitudeSensorMeasurement? = getPrivateProperty(
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
            AttitudeLinearSensorMeasurementInterpolator(copyIfNotInitialized = false)

        val currentMeasurement = generateMeasurement()
        val timestamp = System.nanoTime()
        val result = AttitudeSensorMeasurement()
        assertFalse(interpolator.interpolate(currentMeasurement, timestamp, result))
    }

    @Test
    fun interpolate_whenNotInitializedAndCopyIfNotInitializedEnabled_returnsFalse() {
        val interpolator =
            AttitudeLinearSensorMeasurementInterpolator(copyIfNotInitialized = true)

        val currentMeasurement = generateMeasurement()
        val timestamp = currentMeasurement.timestamp
        val result = AttitudeSensorMeasurement()
        assertTrue(interpolator.interpolate(currentMeasurement, timestamp, result))
        assertMeasurement(currentMeasurement, result)
    }

    @Test
    fun interpolate_whenInitialized_returnsTrue() {
        val interpolator = AttitudeLinearSensorMeasurementInterpolator()

        val measurement0 = generateMeasurement()
        val measurement1 = generateMeasurement()

        interpolator.push(measurement0)
        interpolator.push(measurement1)

        // check with measurement
        val currentMeasurement = generateMeasurement()
        val timestamp0 = measurement0.timestamp
        val result = AttitudeSensorMeasurement()

        assertTrue(interpolator.interpolate(currentMeasurement, timestamp0, result))
        assertMeasurement(measurement0, result)

        val timestamp1 = measurement1.timestamp
        assertTrue(interpolator.interpolate(currentMeasurement, timestamp1, result))
        assertMeasurement(measurement1, result)
    }

    @Test
    fun interpolate_whenFutureTimestamp_returnsExpectedResult() {
        val interpolator = AttitudeLinearSensorMeasurementInterpolator()

        val measurement0 = generateMeasurement()
        val measurement1 = generateMeasurement()

        interpolator.push(measurement0)
        interpolator.push(measurement1)

        // check with previous measurements
        val currentMeasurement = generateMeasurement()
        val timestamp0 = measurement0.timestamp
        val result = AttitudeSensorMeasurement()

        assertTrue(interpolator.interpolate(currentMeasurement, timestamp0, result))
        assertMeasurement(measurement0, result)

        val timestamp1 = measurement1.timestamp
        assertTrue(interpolator.interpolate(currentMeasurement, timestamp1, result))
        assertMeasurement(measurement1, result)

        // check with future measurement
        val timestamp = (timestamp1 - timestamp0) / 2 + timestamp1
        val factor = (timestamp - timestamp0).toDouble() / (timestamp1 - timestamp0).toDouble()
        val delta =
            measurement1.attitude.multiplyAndReturnNew(measurement0.attitude.inverseAndReturnNew())
        delta.normalize()
        val expectedAttitude = Quaternion.slerpAndReturnNew(
            Quaternion(),
            delta,
            factor - 1.0
        ).multiplyAndReturnNew(measurement1.attitude)
        expectedAttitude.normalize()

        val headingAccuracy0 = measurement0.headingAccuracy ?: 0.0f
        val headingAccuracy1 = measurement1.headingAccuracy ?: 0.0f
        val expectedHeadingAccuracy =
            (headingAccuracy0.toDouble() + factor * (headingAccuracy1.toDouble() - headingAccuracy0.toDouble())).toFloat()

        val expectedMeasurement = AttitudeSensorMeasurement(
            expectedAttitude,
            expectedHeadingAccuracy,
            timestamp,
            measurement0.accuracy
        )

        assertTrue(interpolator.interpolate(currentMeasurement, timestamp, result))
        assertMeasurement(expectedMeasurement, result)
    }

    @Test
    fun interpolate_whenPastTimestamp_returnsExpectedResult() {
        val interpolator = AttitudeLinearSensorMeasurementInterpolator()

        val measurement0 = generateMeasurement()
        val measurement1 = generateMeasurement()

        interpolator.push(measurement0)
        interpolator.push(measurement1)

        // check with previous measurements
        val currentMeasurement = generateMeasurement()
        val timestamp0 = measurement0.timestamp
        val timestamp1 = measurement1.timestamp
        val result = AttitudeSensorMeasurement()

        // check with past measurement
        val timestamp = -(timestamp1 - timestamp0) / 2 + timestamp0
        val factor = (timestamp - timestamp0).toDouble() / (timestamp1 - timestamp0).toDouble()
        val delta =
            measurement1.attitude.multiplyAndReturnNew(measurement0.attitude.inverseAndReturnNew())
        delta.normalize()
        delta.inverse()
        delta.normalize()
        val expectedAttitude = Quaternion.slerpAndReturnNew(
            Quaternion(),
            delta,
            factor + 1.0
        ).multiplyAndReturnNew(measurement0.attitude)
        expectedAttitude.normalize()

        val headingAccuracy0 = measurement0.headingAccuracy ?: 0.0f
        val headingAccuracy1 = measurement1.headingAccuracy ?: 0.0f
        val expectedHeadingAccuracy =
            (headingAccuracy0.toDouble() + factor * (headingAccuracy1.toDouble() - headingAccuracy0.toDouble())).toFloat()

        val expectedMeasurement = AttitudeSensorMeasurement(
            expectedAttitude,
            expectedHeadingAccuracy,
            timestamp,
            measurement0.accuracy
        )

        assertTrue(interpolator.interpolate(currentMeasurement, timestamp, result))
        assertMeasurement(expectedMeasurement, result)
    }

    @Test
    fun interpolate_whenNoHeadingAccuracy_returnsTrue() {
        val interpolator = AttitudeLinearSensorMeasurementInterpolator()

        val measurement0 = generateMeasurement(false)
        val measurement1 = generateMeasurement(false)

        interpolator.push(measurement0)
        interpolator.push(measurement1)

        // check with measurement
        val currentMeasurement = generateMeasurement()
        val timestamp0 = measurement0.timestamp
        val result = AttitudeSensorMeasurement()

        assertTrue(interpolator.interpolate(currentMeasurement, timestamp0, result))
        assertMeasurement(measurement0, result)

        val timestamp1 = measurement1.timestamp
        assertTrue(interpolator.interpolate(currentMeasurement, timestamp1, result))
        assertMeasurement(measurement1, result)
    }

    private companion object {

        const val MIN_DEGREES = -90.0
        const val MAX_DEGREES = 90.0

        const val ABSOLUTE_ERROR = 5e-6f
        const val ATTITUDE_ABSOLUTE_ERROR = 1e-3

        fun assertMeasurement(
            measurement1: AttitudeSensorMeasurement,
            measurement2: AttitudeSensorMeasurement
        ) {
            assertNotSame(measurement1, measurement2)
            assertTrue(measurement1.attitude.equals(measurement2.attitude, ATTITUDE_ABSOLUTE_ERROR))
            assertEquals(
                measurement1.headingAccuracy ?: 0.0f,
                measurement2.headingAccuracy ?: 0.0f,
                ABSOLUTE_ERROR
            )
            assertEquals(measurement1.timestamp, measurement2.timestamp)
            assertEquals(measurement1.accuracy, measurement2.accuracy)
        }

        fun generateMeasurement(withHeadingAccuracy: Boolean = true): AttitudeSensorMeasurement {
            val randomizer = UniformRandomizer()
            val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
            val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
            val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
            val attitude = Quaternion(roll, pitch, yaw)

            val headingAccuracy = if (withHeadingAccuracy) {
                randomizer.nextFloat()
            } else {
                null
            }
            val timestamp = System.nanoTime()
            return AttitudeSensorMeasurement(
                attitude,
                headingAccuracy,
                timestamp,
                SensorAccuracy.MEDIUM
            )
        }
    }
}