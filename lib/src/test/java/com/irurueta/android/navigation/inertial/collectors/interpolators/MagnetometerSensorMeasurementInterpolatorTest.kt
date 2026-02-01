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

import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorCoordinateSystem
import com.irurueta.statistics.UniformRandomizer
import io.mockk.Called
import io.mockk.junit4.MockKRule
import io.mockk.mockk
import io.mockk.verify
import org.junit.Assert.*
import org.junit.Rule
import org.junit.Test

class MagnetometerSensorMeasurementInterpolatorTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    private val interpolator = MagnetometerSensorMeasurementInterpolator()

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
        val result = mockk<MagnetometerSensorMeasurement>()
        assertFalse(interpolator.interpolate(emptyList(), 0L, result))

        verify { result wasNot Called }
    }

    @Test
    fun interpolate_whenTimestampBeforeMeasurements_returnsFirstMeasurement() {
        val timestamp = System.nanoTime()
        val measurement1 = createMeasurement()
        val measurement2 = createMeasurement()
        val measurements = listOf(measurement1, measurement2)

        val result = MagnetometerSensorMeasurement()
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

        val result = MagnetometerSensorMeasurement()
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

        val result = MagnetometerSensorMeasurement()
        assertTrue(interpolator.interpolate(measurements, timestamp, result))

        val bx = measurement1.bx + alpha * (measurement2.bx - measurement1.bx)
        val by = measurement1.by + alpha * (measurement2.by - measurement1.by)
        val bz = measurement1.bz + alpha * (measurement2.bz - measurement1.bz)
        val hardIronX = measurement1.hardIronX!! + alpha * (measurement2.hardIronX!! - measurement1.hardIronX!!)
        val hardIronY = measurement1.hardIronY!! + alpha * (measurement2.hardIronY!! - measurement1.hardIronY!!)
        val hardIronZ = measurement1.hardIronZ!! + alpha * (measurement2.hardIronZ!! - measurement1.hardIronZ!!)
        assertEquals(bx, result.bx, ABSOLUTE_ERROR)
        assertEquals(by, result.by, ABSOLUTE_ERROR)
        assertEquals(bz, result.bz, ABSOLUTE_ERROR)
        assertEquals(hardIronX, result.hardIronX!!, ABSOLUTE_ERROR)
        assertEquals(hardIronY, result.hardIronY!!, ABSOLUTE_ERROR)
        assertEquals(hardIronZ, result.hardIronZ!!, ABSOLUTE_ERROR)
        assertEquals(timestamp, result.timestamp)
        assertEquals(SensorAccuracy.HIGH, result.accuracy)
        assertEquals(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED, result.sensorType)
        assertEquals(SensorCoordinateSystem.ENU, result.sensorCoordinateSystem)
    }

    @Test
    fun interpolate_whenTimestampBetweenMeasurementsWithoutHardIronInAllMeasurements_returnsExpectedMeasurement() {
        val measurement1 = createMeasurement(false)
        val measurement2 = createMeasurement(false)
        val measurements = listOf(measurement1, measurement2)
        val diff = (measurement2.timestamp - measurement1.timestamp) / 2
        val timestamp = measurement1.timestamp + diff
        val alpha =
            (timestamp - measurement1.timestamp).toFloat() / (measurement2.timestamp - measurement1.timestamp).toFloat()

        val result = MagnetometerSensorMeasurement()
        assertTrue(interpolator.interpolate(measurements, timestamp, result))

        val bx = measurement1.bx + alpha * (measurement2.bx - measurement1.bx)
        val by = measurement1.by + alpha * (measurement2.by - measurement1.by)
        val bz = measurement1.bz + alpha * (measurement2.bz - measurement1.bz)
        assertEquals(bx, result.bx, ABSOLUTE_ERROR)
        assertEquals(by, result.by, ABSOLUTE_ERROR)
        assertEquals(bz, result.bz, ABSOLUTE_ERROR)
        assertNull(result.hardIronX)
        assertNull(result.hardIronY)
        assertNull(result.hardIronZ)
        assertEquals(timestamp, result.timestamp)
        assertEquals(SensorAccuracy.HIGH, result.accuracy)
        assertEquals(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED, result.sensorType)
        assertEquals(SensorCoordinateSystem.ENU, result.sensorCoordinateSystem)
    }

    @Test
    fun interpolate_whenTimestampBetweenMeasurementsWithoutHardIronInSecondMeasurement_returnsExpectedMeasurement() {
        val measurement1 = createMeasurement(true)
        val measurement2 = createMeasurement(false)
        val measurements = listOf(measurement1, measurement2)
        val diff = (measurement2.timestamp - measurement1.timestamp) / 2
        val timestamp = measurement1.timestamp + diff
        val alpha =
            (timestamp - measurement1.timestamp).toFloat() / (measurement2.timestamp - measurement1.timestamp).toFloat()

        val result = MagnetometerSensorMeasurement()
        assertTrue(interpolator.interpolate(measurements, timestamp, result))

        val bx = measurement1.bx + alpha * (measurement2.bx - measurement1.bx)
        val by = measurement1.by + alpha * (measurement2.by - measurement1.by)
        val bz = measurement1.bz + alpha * (measurement2.bz - measurement1.bz)
        assertEquals(bx, result.bx, ABSOLUTE_ERROR)
        assertEquals(by, result.by, ABSOLUTE_ERROR)
        assertEquals(bz, result.bz, ABSOLUTE_ERROR)
        assertNull(result.hardIronX)
        assertNull(result.hardIronY)
        assertNull(result.hardIronZ)
        assertEquals(timestamp, result.timestamp)
        assertEquals(SensorAccuracy.HIGH, result.accuracy)
        assertEquals(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED, result.sensorType)
        assertEquals(SensorCoordinateSystem.ENU, result.sensorCoordinateSystem)
    }

    @Test
    fun interpolate_whenTimestampInFirstMeasurement_returnsExpectedMeasurement() {
        val measurement1 = createMeasurement()
        val measurement2 = createMeasurement()
        val measurements = listOf(measurement1, measurement2)
        val timestamp = measurement1.timestamp

        val result = MagnetometerSensorMeasurement()
        assertTrue(interpolator.interpolate(measurements, timestamp, result))

        val bx = measurement1.bx
        val by = measurement1.by
        val bz = measurement1.bz
        val hardIronX = measurement1.hardIronX!!
        val hardIronY = measurement1.hardIronY!!
        val hardIronZ = measurement1.hardIronZ!!
        assertEquals(bx, result.bx, ABSOLUTE_ERROR)
        assertEquals(by, result.by, ABSOLUTE_ERROR)
        assertEquals(bz, result.bz, ABSOLUTE_ERROR)
        assertEquals(hardIronX, result.hardIronX!!, ABSOLUTE_ERROR)
        assertEquals(hardIronY, result.hardIronY!!, ABSOLUTE_ERROR)
        assertEquals(hardIronZ, result.hardIronZ!!, ABSOLUTE_ERROR)
        assertEquals(timestamp, result.timestamp)
        assertEquals(SensorAccuracy.HIGH, result.accuracy)
        assertEquals(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED, result.sensorType)
        assertEquals(SensorCoordinateSystem.ENU, result.sensorCoordinateSystem)
    }

    companion object {
        const val ABSOLUTE_ERROR = 1e-5f

        private fun createMeasurement(withBias: Boolean = true): MagnetometerSensorMeasurement {
            val randomizer = UniformRandomizer()
            val bx = randomizer.nextFloat()
            val by = randomizer.nextFloat()
            val bz = randomizer.nextFloat()
            val hardIronX = if (withBias) {
                randomizer.nextFloat()
            } else {
                null
            }
            val hardIronY = if (withBias) {
                randomizer.nextFloat()
            } else {
                null
            }
            val hardIronZ = if (withBias) {
                randomizer.nextFloat()
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
                SensorAccuracy.HIGH,
                MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
                SensorCoordinateSystem.ENU
            )
        }
    }
}