/*
 * Copyright (C) 2026 Alberto Irurueta Carro (alberto@irurueta.com)
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

import com.irurueta.android.navigation.inertial.collectors.measurements.AttitudeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AttitudeSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorCoordinateSystem
import com.irurueta.geometry.Quaternion
import com.irurueta.statistics.UniformRandomizer
import io.mockk.Called
import io.mockk.junit4.MockKRule
import io.mockk.mockk
import io.mockk.verify
import org.junit.Assert.*
import org.junit.Rule
import org.junit.Test

class AttitudeSensorMeasurementInterpolatorTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    private val interpolator = AttitudeSensorMeasurementInterpolator()

    @Test
    fun findClosest_whenEmptyMeasurements_returnsNull() {
        assertNull(interpolator.findClosest(emptyList(), 0L))
    }

    @Test
    fun findClosest_whenTimestampBeforeMeasurements_returnsFirstMeasurement() {
        val timestamp = System.nanoTime()
        val measurement1 = createMeasurement()
        val measurement2 = createMeasurement()
        val measurements = listOf(measurement1, measurement2)

        val closest = interpolator.findClosest(measurements, timestamp)

        assertEquals(measurement1, closest)
    }

    @Test
    fun findClosest_whenTimestampAfterMeasurements_returnsSecondMeasurement() {
        val measurement1 = createMeasurement()
        val measurement2 = createMeasurement()
        val measurements = listOf(measurement1, measurement2)
        val timestamp = System.nanoTime()

        val closest = interpolator.findClosest(measurements, timestamp)

        assertEquals(measurement2, closest)
    }

    @Test
    fun findClosest_whenTimestampBetweenMeasurementsAndCloseToFirstMeasurement_returnsFirstMeasurement() {
        val measurement1 = createMeasurement()
        val measurement2 = createMeasurement()
        val measurements = listOf(measurement1, measurement2)
        val timestamp = measurement1.timestamp + 1

        val closest = interpolator.findClosest(measurements, timestamp)

        assertEquals(measurement1, closest)
    }

    @Test
    fun findClosest_whenTimestampBetweenMeasurementsAndCloseToSecondMeasurement_returnsSecondMeasurement() {
        val measurement1 = createMeasurement()
        val measurement2 = createMeasurement()
        val measurements = listOf(measurement1, measurement2)
        val timestamp = measurement2.timestamp - 1

        val closest = interpolator.findClosest(measurements, timestamp)

        assertEquals(measurement2, closest)
    }

    @Test
    fun findClosest_whenUnorderedMeasurements_returnsExpectedMeasurement() {
        val measurement1 = createMeasurement()
        val measurement2 = createMeasurement()
        val measurement3 = createMeasurement()
        val measurements = listOf(measurement3, measurement2, measurement1)
        val timestamp = measurement2.timestamp

        val closest = interpolator.findClosest(measurements, timestamp)

        assertEquals(measurement2, closest)
    }

    @Test
    fun interpolate_whenEmptyMeasurements_returnsFalse() {
        val result = mockk<AttitudeSensorMeasurement>()
        assertFalse(interpolator.interpolate(emptyList(), 0L, result))

        verify { result wasNot Called }
    }

    @Test
    fun interpolate_whenTimestampBeforeMeasurements_returnsFirstMeasurement() {
        val timestamp = System.nanoTime()
        val measurement1 = createMeasurement()
        val measurement2 = createMeasurement()
        val measurements = listOf(measurement1, measurement2)

        val result = AttitudeSensorMeasurement()
        assertTrue(interpolator.interpolate(measurements, timestamp, result))

        val expected = measurement1.copy()
        expected.timestamp = timestamp
        assertEquals(expected, result)
    }

    @Test
    fun interpolate_whenTimestampAfterMeasurements_returnsSecondMeasurement() {
        val measurement1 = createMeasurement()
        val measurement2 = createMeasurement()
        val measurements = listOf(measurement1, measurement2)
        val timestamp = System.nanoTime()

        val result = AttitudeSensorMeasurement()
        assertTrue(interpolator.interpolate(measurements, timestamp, result))

        val expected = measurement2.copy()
        expected.timestamp = timestamp
        assertEquals(expected, result)
    }

    @Test
    fun interpolate_whenTimestampBetweenMeasurements_returnsExpectedMeasurement() {
        val measurement1 = createMeasurement()
        val measurement2 = createMeasurement()
        val measurements = listOf(measurement1, measurement2)
        val diff = (measurement2.timestamp - measurement1.timestamp) / 2
        val timestamp = measurement1.timestamp + diff
        val alpha =
            (timestamp - measurement1.timestamp).toFloat() / (measurement2.timestamp - measurement1.timestamp).toFloat()

        val result = AttitudeSensorMeasurement()
        assertTrue(interpolator.interpolate(measurements, timestamp, result))

        val attitude = Quaternion()
        Quaternion.slerp(measurement1.attitude, measurement2.attitude, alpha.toDouble(), attitude)
        val headingAccuracy = measurement1.headingAccuracy!! + alpha * (measurement2.headingAccuracy!! - measurement1.headingAccuracy!!)
        assertTrue(result.attitude.equals(attitude, ABSOLUTE_ERROR.toDouble()))
        assertEquals(headingAccuracy, result.headingAccuracy!!, ABSOLUTE_ERROR)
        assertEquals(timestamp, result.timestamp)
        assertEquals(SensorAccuracy.HIGH, result.accuracy)
        assertEquals(AttitudeSensorType.ABSOLUTE_ATTITUDE, result.sensorType)
        assertEquals(SensorCoordinateSystem.ENU, result.sensorCoordinateSystem)
    }

    @Test
    fun interpolate_whenTimestampBetweenMeasurementsWithoutAccuracyInAllMeasurements_returnsExpectedMeasurement() {
        val measurement1 = createMeasurement(false)
        val measurement2 = createMeasurement(false)
        val measurements = listOf(measurement1, measurement2)
        val diff = (measurement2.timestamp - measurement1.timestamp) / 2
        val timestamp = measurement1.timestamp + diff
        val alpha =
            (timestamp - measurement1.timestamp).toFloat() / (measurement2.timestamp - measurement1.timestamp).toFloat()

        val result = AttitudeSensorMeasurement()
        assertTrue(interpolator.interpolate(measurements, timestamp, result))

        val attitude = Quaternion()
        Quaternion.slerp(measurement1.attitude, measurement2.attitude, alpha.toDouble(), attitude)
        assertTrue(result.attitude.equals(attitude, ABSOLUTE_ERROR.toDouble()))
        assertNull(result.headingAccuracy)
        assertEquals(timestamp, result.timestamp)
        assertEquals(SensorAccuracy.HIGH, result.accuracy)
        assertEquals(AttitudeSensorType.ABSOLUTE_ATTITUDE, result.sensorType)
        assertEquals(SensorCoordinateSystem.ENU, result.sensorCoordinateSystem)
    }

    @Test
    fun interpolate_whenTimestampBetweenMeasurementsWithoutAccuracyInSecondsMeasurement_returnsExpectedMeasurement() {
        val measurement1 = createMeasurement(true)
        val measurement2 = createMeasurement(false)
        val measurements = listOf(measurement1, measurement2)
        val diff = (measurement2.timestamp - measurement1.timestamp) / 2
        val timestamp = measurement1.timestamp + diff
        val alpha =
            (timestamp - measurement1.timestamp).toFloat() / (measurement2.timestamp - measurement1.timestamp).toFloat()

        val result = AttitudeSensorMeasurement()
        assertTrue(interpolator.interpolate(measurements, timestamp, result))

        val attitude = Quaternion()
        Quaternion.slerp(measurement1.attitude, measurement2.attitude, alpha.toDouble(), attitude)
        assertTrue(result.attitude.equals(attitude, ABSOLUTE_ERROR.toDouble()))
        assertNull(result.headingAccuracy)
        assertEquals(timestamp, result.timestamp)
        assertEquals(SensorAccuracy.HIGH, result.accuracy)
        assertEquals(AttitudeSensorType.ABSOLUTE_ATTITUDE, result.sensorType)
        assertEquals(SensorCoordinateSystem.ENU, result.sensorCoordinateSystem)
    }

    @Test
    fun interpolate_whenTimestampInFirstMeasurement_returnsExpectedMeasurement() {
        val measurement1 = createMeasurement()
        val measurement2 = createMeasurement()
        val measurements = listOf(measurement1, measurement2)
        val timestamp = measurement1.timestamp

        val result = AttitudeSensorMeasurement()
        assertTrue(interpolator.interpolate(measurements, timestamp, result))

        val attitude = measurement1.attitude
        val headingAccuracy = measurement1.headingAccuracy!!
        assertEquals(attitude, result.attitude)
        assertEquals(headingAccuracy, result.headingAccuracy!!, ABSOLUTE_ERROR)
        assertEquals(timestamp, result.timestamp)
        assertEquals(SensorAccuracy.HIGH, result.accuracy)
        assertEquals(AttitudeSensorType.ABSOLUTE_ATTITUDE, result.sensorType)
        assertEquals(SensorCoordinateSystem.ENU, result.sensorCoordinateSystem)
    }

    companion object {
        const val MIN_DEGREES = -90.0

        const val MAX_DEGREES = 90.0

        const val ABSOLUTE_ERROR = 1e-5f

        private fun createMeasurement(withAccuracy: Boolean = true): AttitudeSensorMeasurement {
            val randomizer = UniformRandomizer()
            val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
            val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
            val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
            val attitude = Quaternion(roll, pitch, yaw)
            val headingAccuracy = if (withAccuracy) {
                Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
            } else {
                null
            }
            val timestamp = System.nanoTime()
            return AttitudeSensorMeasurement(
                attitude,
                headingAccuracy,
                timestamp,
                SensorAccuracy.HIGH,
                AttitudeSensorType.ABSOLUTE_ATTITUDE,
                SensorCoordinateSystem.ENU
            )
        }
    }
}