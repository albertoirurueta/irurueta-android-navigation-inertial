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
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class AccelerometerSensorMeasurementTest {

    @After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
    }

    @Test
    fun constructor_whenDefault_setsExpectedValues() {
        val measurement = AccelerometerSensorMeasurement()

        // check
        assertEquals(0.0f, measurement.ax, 0.0f)
        assertEquals(0.0f, measurement.ay, 0.0f)
        assertEquals(0.0f, measurement.az, 0.0f)
        assertNull(measurement.bx)
        assertNull(measurement.by)
        assertNull(measurement.bz)
        assertEquals(0L, measurement.timestamp)
        assertNull(measurement.accuracy)
        assertEquals(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED, measurement.sensorType)
    }

    @Test
    fun constructor_withRequiredProperties_setsExpectedValues() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()

        val measurement =
            AccelerometerSensorMeasurement(
                ax,
                ay,
                az,
                null,
                null,
                null,
                timestamp,
                null,
                AccelerometerSensorType.ACCELEROMETER
            )

        // check
        assertEquals(ax, measurement.ax, 0.0f)
        assertEquals(ay, measurement.ay, 0.0f)
        assertEquals(az, measurement.az, 0.0f)
        assertNull(measurement.bx)
        assertNull(measurement.by)
        assertNull(measurement.bz)
        assertEquals(timestamp, measurement.timestamp)
        assertNull(measurement.accuracy)
        assertEquals(AccelerometerSensorType.ACCELEROMETER, measurement.sensorType)
    }

    @Test
    fun constructor_withAllProperties_setsExpectedValues() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()

        val measurement = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        )

        // check
        assertEquals(ax, measurement.ax, 0.0f)
        assertEquals(ay, measurement.ay, 0.0f)
        assertEquals(az, measurement.az, 0.0f)
        assertEquals(bx, measurement.bx)
        assertEquals(by, measurement.by)
        assertEquals(bz, measurement.bz)
        assertEquals(timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
        assertEquals(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED, measurement.sensorType)
    }

    @Test
    fun constructor_whenCopy_setsExpectedValues() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()

        val measurement1 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        )

        val measurement2 = AccelerometerSensorMeasurement(measurement1)

        // check
        assertEquals(ax, measurement1.ax, 0.0f)
        assertEquals(ay, measurement1.ay, 0.0f)
        assertEquals(az, measurement1.az, 0.0f)
        assertEquals(bx, measurement1.bx)
        assertEquals(by, measurement1.by)
        assertEquals(bz, measurement1.bz)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement1.accuracy)
        assertEquals(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED, measurement1.sensorType)

        assertEquals(measurement1.ax, measurement2.ax, 0.0f)
        assertEquals(measurement1.ay, measurement2.ay, 0.0f)
        assertEquals(measurement1.az, measurement2.az, 0.0f)
        assertEquals(measurement1.bx, measurement2.bx)
        assertEquals(measurement1.by, measurement2.by)
        assertEquals(measurement1.bz, measurement2.bz)
        assertEquals(measurement1.timestamp, measurement2.timestamp)
        assertEquals(measurement1.accuracy, measurement2.accuracy)
        assertEquals(measurement1.sensorType, measurement2.sensorType)
    }

    @Test
    fun ax_setsExpectedValue() {
        val measurement = AccelerometerSensorMeasurement()

        // check default value
        assertEquals(0.0f, measurement.ax, 0.0f)

        // set new value
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        measurement.ax = ax

        // check
        assertEquals(ax, measurement.ax, 0.0f)
    }

    @Test
    fun ay_setsExpectedValue() {
        val measurement = AccelerometerSensorMeasurement()

        // check default value
        assertEquals(0.0f, measurement.ay, 0.0f)

        // set new value
        val randomizer = UniformRandomizer()
        val ay = randomizer.nextFloat()
        measurement.ay = ay

        // check
        assertEquals(ay, measurement.ay, 0.0f)
    }

    @Test
    fun az_setsExpectedValue() {
        val measurement = AccelerometerSensorMeasurement()

        // check default value
        assertEquals(0.0f, measurement.az, 0.0f)

        // set new value
        val randomizer = UniformRandomizer()
        val az = randomizer.nextFloat()
        measurement.az = az

        // check
        assertEquals(az, measurement.az, 0.0f)
    }

    @Test
    fun bx_setsExpectedValue() {
        val measurement = AccelerometerSensorMeasurement()

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
        val measurement = AccelerometerSensorMeasurement()

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
        val measurement = AccelerometerSensorMeasurement()

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
        val measurement = AccelerometerSensorMeasurement()

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
        val measurement = AccelerometerSensorMeasurement()

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
    fun sensorType_setsExpectedValue() {
        val measurement = AccelerometerSensorMeasurement()

        // check default value
        assertEquals(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED, measurement.sensorType)

        // set new value
        measurement.sensorType = AccelerometerSensorType.ACCELEROMETER

        // check
        assertEquals(AccelerometerSensorType.ACCELEROMETER, measurement.sensorType)
    }

    @Test
    fun copyFrom_makesExpectedCopy() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()

        val measurement1 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER
        )

        val measurement2 = AccelerometerSensorMeasurement()
        measurement2.copyFrom(measurement1)

        // check
        assertEquals(ax, measurement1.ax, 0.0f)
        assertEquals(ay, measurement1.ay, 0.0f)
        assertEquals(az, measurement1.az, 0.0f)
        assertEquals(bx, measurement1.bx)
        assertEquals(by, measurement1.by)
        assertEquals(bz, measurement1.bz)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement1.accuracy)
        assertEquals(AccelerometerSensorType.ACCELEROMETER, measurement1.sensorType)

        assertEquals(measurement1.ax, measurement2.ax, 0.0f)
        assertEquals(measurement1.ay, measurement2.ay, 0.0f)
        assertEquals(measurement1.az, measurement2.az, 0.0f)
        assertEquals(measurement1.bx, measurement2.bx)
        assertEquals(measurement1.by, measurement2.by)
        assertEquals(measurement1.bz, measurement2.bz)
        assertEquals(measurement1.timestamp, measurement2.timestamp)
        assertEquals(measurement1.accuracy, measurement2.accuracy)
        assertEquals(measurement1.sensorType, measurement2.sensorType)
    }

    @Test
    fun copyTo_makesExpectedCopy() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()

        val measurement1 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER
        )

        val measurement2 = AccelerometerSensorMeasurement()
        measurement1.copyTo(measurement2)

        // check
        assertEquals(ax, measurement1.ax, 0.0f)
        assertEquals(ay, measurement1.ay, 0.0f)
        assertEquals(az, measurement1.az, 0.0f)
        assertEquals(bx, measurement1.bx)
        assertEquals(by, measurement1.by)
        assertEquals(bz, measurement1.bz)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement1.accuracy)
        assertEquals(AccelerometerSensorType.ACCELEROMETER, measurement1.sensorType)

        assertEquals(measurement1.ax, measurement2.ax, 0.0f)
        assertEquals(measurement1.ay, measurement2.ay, 0.0f)
        assertEquals(measurement1.az, measurement2.az, 0.0f)
        assertEquals(measurement1.bx, measurement2.bx)
        assertEquals(measurement1.by, measurement2.by)
        assertEquals(measurement1.bz, measurement2.bz)
        assertEquals(measurement1.timestamp, measurement2.timestamp)
        assertEquals(measurement1.accuracy, measurement2.accuracy)
        assertEquals(measurement1.sensorType, measurement2.sensorType)
    }

    @Test
    fun copy_makesExpectedCopy() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()

        val measurement1 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER
        )

        val measurement2 = measurement1.copy()

        // check
        assertEquals(ax, measurement1.ax, 0.0f)
        assertEquals(ay, measurement1.ay, 0.0f)
        assertEquals(az, measurement1.az, 0.0f)
        assertEquals(bx, measurement1.bx)
        assertEquals(by, measurement1.by)
        assertEquals(bz, measurement1.bz)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement1.accuracy)
        assertEquals(AccelerometerSensorType.ACCELEROMETER, measurement1.sensorType)

        assertEquals(measurement1.ax, measurement2.ax, 0.0f)
        assertEquals(measurement1.ay, measurement2.ay, 0.0f)
        assertEquals(measurement1.az, measurement2.az, 0.0f)
        assertEquals(measurement1.bx, measurement2.bx)
        assertEquals(measurement1.by, measurement2.by)
        assertEquals(measurement1.bz, measurement2.bz)
        assertEquals(measurement1.timestamp, measurement2.timestamp)
        assertEquals(measurement1.accuracy, measurement2.accuracy)
        assertEquals(measurement1.sensorType, measurement2.sensorType)
    }
}

