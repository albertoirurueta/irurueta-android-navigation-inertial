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
package com.irurueta.android.navigation.inertial.collectors

import android.os.SystemClock
import com.irurueta.statistics.UniformRandomizer
import io.mockk.clearAllMocks
import io.mockk.unmockkAll
import org.junit.After
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class GravitySensorMeasurementTest {

    @After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
    }

    @Test
    fun constructor_whenDefault_setsExpectedValues() {
        val measurement = GravitySensorMeasurement()

        // check
        assertEquals(0.0f, measurement.gx, 0.0f)
        assertEquals(0.0f, measurement.gy, 0.0f)
        assertEquals(0.0f, measurement.gz, 0.0f)
        assertEquals(0L, measurement.timestamp)
        assertNull(measurement.accuracy)
    }

    @Test
    fun constructor_withRequiredProperties_setsExpectedValues() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()

        val measurement = GravitySensorMeasurement(gx, gy, gz, timestamp, null)

        // check
        assertEquals(gx, measurement.gx, 0.0f)
        assertEquals(gy, measurement.gy, 0.0f)
        assertEquals(gz, measurement.gz, 0.0f)
        assertEquals(timestamp, measurement.timestamp)
        assertNull(measurement.accuracy)
    }

    @Test
    fun constructor_withAllProperties_setsExpectedValues() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()

        val measurement =
            GravitySensorMeasurement(gx, gy, gz, timestamp, SensorAccuracy.MEDIUM)

        // check
        assertEquals(gx, measurement.gx, 0.0f)
        assertEquals(gy, measurement.gy, 0.0f)
        assertEquals(gz, measurement.gz, 0.0f)
        assertEquals(timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, measurement.accuracy)
    }

    @Test
    fun constructor_whenCopy_setsExpectedValues() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()

        val measurement1 =
            GravitySensorMeasurement(gx, gy, gz, timestamp, SensorAccuracy.MEDIUM)

        val measurement2 = GravitySensorMeasurement(measurement1)

        // check
        assertEquals(gx, measurement1.gx, 0.0f)
        assertEquals(gy, measurement1.gy, 0.0f)
        assertEquals(gz, measurement1.gz, 0.0f)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, measurement1.accuracy)

        assertEquals(measurement1.gx, measurement2.gx, 0.0f)
        assertEquals(measurement1.gy, measurement2.gy, 0.0f)
        assertEquals(measurement1.gz, measurement2.gz, 0.0f)
        assertEquals(measurement1.timestamp, measurement2.timestamp)
        assertEquals(measurement1.accuracy, measurement2.accuracy)
    }

    @Test
    fun gx_setsExpectedValue() {
        val measurement = GravitySensorMeasurement()

        // check default value
        assertEquals(0.0f, measurement.gx, 0.0f)

        // set new value
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        measurement.gx = gx

        // check
        assertEquals(gx, measurement.gx, 0.0f)
    }

    @Test
    fun gy_setsExpectedValue() {
        val measurement = GravitySensorMeasurement()

        // check default value
        assertEquals(0.0f, measurement.gy, 0.0f)

        // set new value
        val randomizer = UniformRandomizer()
        val gy = randomizer.nextFloat()
        measurement.gy = gy

        // check
        assertEquals(gy, measurement.gy, 0.0f)
    }

    @Test
    fun gz_setsExpectedValue() {
        val measurement = GravitySensorMeasurement()

        // check default value
        assertEquals(0.0f, measurement.gz, 0.0f)

        // set new value
        val randomizer = UniformRandomizer()
        val gz = randomizer.nextFloat()
        measurement.gz = gz

        // check
        assertEquals(gz, measurement.gz, 0.0f)
    }

    @Test
    fun timestamp_setsExpectedValue() {
        val measurement = GravitySensorMeasurement()

        // check default value
        assertEquals(0L, measurement.timestamp)

        // set new value
        val timestamp = SystemClock.elapsedRealtimeNanos()
        measurement.timestamp = timestamp

        // check
        assertEquals(timestamp, measurement.timestamp)
    }

    @Test
    fun accuracy_setsExpectedValue() {
        val measurement = GravitySensorMeasurement()

        // check default value
        assertNull(measurement.accuracy)

        // set new value
        measurement.accuracy = SensorAccuracy.HIGH

        // check
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)

        // set to null
        measurement.accuracy = null

        // check
        @Suppress("KotlinConstantConditions")
        assertNull(measurement.accuracy)
    }

    @Test
    fun copyFrom_makesExpectedCopy() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()

        val measurement1 =
            GravitySensorMeasurement(gx, gy, gz, timestamp, SensorAccuracy.MEDIUM)

        val measurement2 = GravitySensorMeasurement()
        measurement2.copyFrom(measurement1)

        // check
        assertEquals(gx, measurement1.gx, 0.0f)
        assertEquals(gy, measurement1.gy, 0.0f)
        assertEquals(gz, measurement1.gz, 0.0f)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, measurement1.accuracy)

        assertEquals(measurement1.gx, measurement2.gx, 0.0f)
        assertEquals(measurement1.gy, measurement2.gy, 0.0f)
        assertEquals(measurement1.gz, measurement2.gz, 0.0f)
        assertEquals(measurement1.timestamp, measurement2.timestamp)
        assertEquals(measurement1.accuracy, measurement2.accuracy)
    }

    @Test
    fun copyTo_makesExpectedCopy() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()

        val measurement1 =
            GravitySensorMeasurement(gx, gy, gz, timestamp, SensorAccuracy.MEDIUM)

        val measurement2 = GravitySensorMeasurement()
        measurement1.copyTo(measurement2)

        // check
        assertEquals(gx, measurement1.gx, 0.0f)
        assertEquals(gy, measurement1.gy, 0.0f)
        assertEquals(gz, measurement1.gz, 0.0f)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, measurement1.accuracy)

        assertEquals(measurement1.gx, measurement2.gx, 0.0f)
        assertEquals(measurement1.gy, measurement2.gy, 0.0f)
        assertEquals(measurement1.gz, measurement2.gz, 0.0f)
        assertEquals(measurement1.timestamp, measurement2.timestamp)
        assertEquals(measurement1.accuracy, measurement2.accuracy)
    }

    @Test
    fun copy_makesExpectedCopy() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()

        val measurement1 =
            GravitySensorMeasurement(gx, gy, gz, timestamp, SensorAccuracy.MEDIUM)

        val measurement2 = measurement1.copy()

        // check
        assertEquals(gx, measurement1.gx, 0.0f)
        assertEquals(gy, measurement1.gy, 0.0f)
        assertEquals(gz, measurement1.gz, 0.0f)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, measurement1.accuracy)

        assertEquals(measurement1.gx, measurement2.gx, 0.0f)
        assertEquals(measurement1.gy, measurement2.gy, 0.0f)
        assertEquals(measurement1.gz, measurement2.gz, 0.0f)
        assertEquals(measurement1.timestamp, measurement2.timestamp)
        assertEquals(measurement1.accuracy, measurement2.accuracy)
    }
}