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
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class GyroscopeSensorMeasurementTest {

    @Test
    fun constructor_whenDefault_setsExpectedValues() {
        val measurement = GyroscopeSensorMeasurement()

        // check
        assertEquals(0.0f, measurement.wx, 0.0f)
        assertEquals(0.0f, measurement.wy, 0.0f)
        assertEquals(0.0f, measurement.wz, 0.0f)
        assertNull(measurement.bx)
        assertNull(measurement.by)
        assertNull(measurement.bz)
        assertEquals(0L, measurement.timestamp)
        assertNull(measurement.accuracy)
    }

    @Test
    fun constructor_withRequiredProperties_setsExpectedValues() {
        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()

        val measurement =
            GyroscopeSensorMeasurement(wx, wy, wz, null, null, null, timestamp, null)

        // check
        assertEquals(wx, measurement.wx, 0.0f)
        assertEquals(wy, measurement.wy, 0.0f)
        assertEquals(wz, measurement.wz, 0.0f)
        assertNull(measurement.bx)
        assertNull(measurement.by)
        assertNull(measurement.bz)
        assertEquals(timestamp, measurement.timestamp)
        assertNull(measurement.accuracy)
    }

    @Test
    fun constructor_withAllProperties_setsExpectedValues() {
        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()

        val measurement = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH
        )

        // check
        assertEquals(wx, measurement.wx, 0.0f)
        assertEquals(wy, measurement.wy, 0.0f)
        assertEquals(wz, measurement.wz, 0.0f)
        assertEquals(bx, measurement.bx)
        assertEquals(by, measurement.by)
        assertEquals(bz, measurement.bz)
        assertEquals(timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
    }

    @Test
    fun constructor_whenCopy_setsExpectedValues() {
        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()

        val measurement1 = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH
        )

        val measurement2 = GyroscopeSensorMeasurement(measurement1)

        // check
        assertEquals(wx, measurement1.wx, 0.0f)
        assertEquals(wy, measurement1.wy, 0.0f)
        assertEquals(wz, measurement1.wz, 0.0f)
        assertEquals(bx, measurement1.bx)
        assertEquals(by, measurement1.by)
        assertEquals(bz, measurement1.bz)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement1.accuracy)

        assertEquals(measurement1.wx, measurement2.wx, 0.0f)
        assertEquals(measurement1.wy, measurement2.wy, 0.0f)
        assertEquals(measurement1.wz, measurement2.wz, 0.0f)
        assertEquals(measurement1.bx, measurement2.bx)
        assertEquals(measurement1.by, measurement2.by)
        assertEquals(measurement1.bz, measurement2.bz)
        assertEquals(measurement1.timestamp, measurement2.timestamp)
        assertEquals(measurement1.accuracy, measurement2.accuracy)
    }

    @Test
    fun wx_setsExpectedValue() {
        val measurement = GyroscopeSensorMeasurement()

        // check default value
        assertEquals(0.0f, measurement.wx, 0.0f)

        // set new value
        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        measurement.wx = wx

        // check
        assertEquals(wx, measurement.wx, 0.0f)
    }

    @Test
    fun wy_setsExpectedValue() {
        val measurement = GyroscopeSensorMeasurement()

        // check default value
        assertEquals(0.0f, measurement.wy, 0.0f)

        // set new value
        val randomizer = UniformRandomizer()
        val wy = randomizer.nextFloat()
        measurement.wy = wy

        // check
        assertEquals(wy, measurement.wy, 0.0f)
    }

    @Test
    fun wz_setsExpectedValue() {
        val measurement = GyroscopeSensorMeasurement()

        // check default value
        assertEquals(0.0f, measurement.wz, 0.0f)

        // set new value
        val randomizer = UniformRandomizer()
        val wz = randomizer.nextFloat()
        measurement.wz = wz

        // check
        assertEquals(wz, measurement.wz, 0.0f)
    }

    @Test
    fun bx_setsExpectedValue() {
        val measurement = GyroscopeSensorMeasurement()

        // check default value
        assertNull(measurement.bx)

        // set new value
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        measurement.bx = bx

        // check
        assertEquals(bx, measurement.bx)

        // set to null
        measurement.bx = null

        // check
        @Suppress("KotlinConstantConditions")
        assertNull(measurement.bx)
    }

    @Test
    fun by_setsExpectedValue() {
        val measurement = GyroscopeSensorMeasurement()

        // check default value
        assertNull(measurement.by)

        // set new value
        val randomizer = UniformRandomizer()
        val by = randomizer.nextFloat()
        measurement.by = by

        // check
        assertEquals(by, measurement.by)

        // set to null
        measurement.by = null

        // check
        @Suppress("KotlinConstantConditions")
        assertNull(measurement.by)
    }

    @Test
    fun bz_setsExpectedValue() {
        val measurement = GyroscopeSensorMeasurement()

        // check default value
        assertNull(measurement.bz)

        // set new value
        val randomizer = UniformRandomizer()
        val bz = randomizer.nextFloat()
        measurement.bz = bz

        // check
        assertEquals(bz, measurement.bz)

        // set to null
        measurement.bz = null

        // check
        @Suppress("KotlinConstantConditions")
        assertNull(measurement.bz)
    }

    @Test
    fun timestamp_setsExpectedValue() {
        val measurement = GyroscopeSensorMeasurement()

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
        val measurement = GyroscopeSensorMeasurement()

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
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()

        val measurement1 = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH
        )

        val measurement2 = GyroscopeSensorMeasurement()
        measurement2.copyFrom(measurement1)

        // check
        assertEquals(wx, measurement1.wx, 0.0f)
        assertEquals(wy, measurement1.wy, 0.0f)
        assertEquals(wz, measurement1.wz, 0.0f)
        assertEquals(bx, measurement1.bx)
        assertEquals(by, measurement1.by)
        assertEquals(bz, measurement1.bz)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement1.accuracy)

        assertEquals(measurement1.wx, measurement2.wx, 0.0f)
        assertEquals(measurement1.wy, measurement2.wy, 0.0f)
        assertEquals(measurement1.wz, measurement2.wz, 0.0f)
        assertEquals(measurement1.bx, measurement2.bx)
        assertEquals(measurement1.by, measurement2.by)
        assertEquals(measurement1.bz, measurement2.bz)
        assertEquals(measurement1.timestamp, measurement2.timestamp)
        assertEquals(measurement1.accuracy, measurement2.accuracy)
    }

    @Test
    fun copyTo_makesExpectedCopy() {
        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()

        val measurement1 = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH
        )

        val measurement2 = GyroscopeSensorMeasurement()
        measurement1.copyTo(measurement2)

        // check
        assertEquals(wx, measurement1.wx, 0.0f)
        assertEquals(wy, measurement1.wy, 0.0f)
        assertEquals(wz, measurement1.wz, 0.0f)
        assertEquals(bx, measurement1.bx)
        assertEquals(by, measurement1.by)
        assertEquals(bz, measurement1.bz)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement1.accuracy)

        assertEquals(measurement1.wx, measurement2.wx, 0.0f)
        assertEquals(measurement1.wy, measurement2.wy, 0.0f)
        assertEquals(measurement1.wz, measurement2.wz, 0.0f)
        assertEquals(measurement1.bx, measurement2.bx)
        assertEquals(measurement1.by, measurement2.by)
        assertEquals(measurement1.bz, measurement2.bz)
        assertEquals(measurement1.timestamp, measurement2.timestamp)
        assertEquals(measurement1.accuracy, measurement2.accuracy)
    }

    @Test
    fun copy_makesExpectedCopy() {
        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()

        val measurement1 = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH
        )

        val measurement2 = measurement1.copy()

        // check
        assertEquals(wx, measurement1.wx, 0.0f)
        assertEquals(wy, measurement1.wy, 0.0f)
        assertEquals(wz, measurement1.wz, 0.0f)
        assertEquals(bx, measurement1.bx)
        assertEquals(by, measurement1.by)
        assertEquals(bz, measurement1.bz)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement1.accuracy)

        assertEquals(measurement1.wx, measurement2.wx, 0.0f)
        assertEquals(measurement1.wy, measurement2.wy, 0.0f)
        assertEquals(measurement1.wz, measurement2.wz, 0.0f)
        assertEquals(measurement1.bx, measurement2.bx)
        assertEquals(measurement1.by, measurement2.by)
        assertEquals(measurement1.bz, measurement2.bz)
        assertEquals(measurement1.timestamp, measurement2.timestamp)
        assertEquals(measurement1.accuracy, measurement2.accuracy)
    }
}