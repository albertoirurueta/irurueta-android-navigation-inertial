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
class MagnetometerSensorMeasurementTest {

    @Test
    fun constructor_whenDefault_setsExpectedValues() {
        val measurement = MagnetometerSensorMeasurement()

        // check
        assertEquals(0.0f, measurement.bx, 0.0f)
        assertEquals(0.0f, measurement.by, 0.0f)
        assertEquals(0.0f, measurement.bz, 0.0f)
        assertNull(measurement.hardIronX)
        assertNull(measurement.hardIronY)
        assertNull(measurement.hardIronZ)
        assertEquals(0L, measurement.timestamp)
        assertNull(measurement.accuracy)
        assertEquals(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED, measurement.sensorType)
    }

    @Test
    fun constructor_withRequiredProperties_setsExpectedValues() {
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()

        val measurement =
            MagnetometerSensorMeasurement(
                bx,
                by,
                bz,
                null,
                null,
                null,
                timestamp,
                null,
                MagnetometerSensorType.MAGNETOMETER
            )

        // check
        assertEquals(bx, measurement.bx, 0.0f)
        assertEquals(by, measurement.by, 0.0f)
        assertEquals(bz, measurement.bz, 0.0f)
        assertNull(measurement.hardIronX)
        assertNull(measurement.hardIronY)
        assertNull(measurement.hardIronZ)
        assertEquals(timestamp, measurement.timestamp)
        assertNull(measurement.accuracy)
        assertEquals(MagnetometerSensorType.MAGNETOMETER, measurement.sensorType)
    }

    @Test
    fun constructor_withAllProperties_setsExpectedValues() {
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()

        val measurement =
            MagnetometerSensorMeasurement(
                bx,
                by,
                bz,
                hardIronX,
                hardIronY,
                hardIronZ,
                timestamp,
                SensorAccuracy.LOW,
                MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED
            )

        // check
        assertEquals(bx, measurement.bx, 0.0f)
        assertEquals(by, measurement.by, 0.0f)
        assertEquals(bz, measurement.bz, 0.0f)
        assertEquals(hardIronX, measurement.hardIronX)
        assertEquals(hardIronY, measurement.hardIronY)
        assertEquals(hardIronZ, measurement.hardIronZ)
        assertEquals(timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.LOW, measurement.accuracy)
        assertEquals(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED, measurement.sensorType)
    }

    @Test
    fun constructor_whenCopy_setsExpectedValues() {
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()

        val measurement1 =
            MagnetometerSensorMeasurement(
                bx,
                by,
                bz,
                hardIronX,
                hardIronY,
                hardIronZ,
                timestamp,
                SensorAccuracy.LOW,
                MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED
            )

        val measurement2 = MagnetometerSensorMeasurement(measurement1)

        // check
        assertEquals(bx, measurement1.bx, 0.0f)
        assertEquals(by, measurement1.by, 0.0f)
        assertEquals(bz, measurement1.bz, 0.0f)
        assertEquals(hardIronX, measurement1.hardIronX)
        assertEquals(hardIronY, measurement1.hardIronY)
        assertEquals(hardIronZ, measurement1.hardIronZ)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.LOW, measurement1.accuracy)
        assertEquals(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED, measurement1.sensorType)

        assertEquals(measurement1.bx, measurement2.bx, 0.0f)
        assertEquals(measurement1.by, measurement2.by, 0.0f)
        assertEquals(measurement1.bz, measurement2.bz, 0.0f)
        assertEquals(measurement1.hardIronX, measurement2.hardIronX)
        assertEquals(measurement1.hardIronY, measurement2.hardIronY)
        assertEquals(measurement1.hardIronZ, measurement2.hardIronZ)
        assertEquals(measurement1.timestamp, measurement2.timestamp)
        assertEquals(measurement1.accuracy, measurement2.accuracy)
        assertEquals(measurement1.sensorType, measurement2.sensorType)
    }

    @Test
    fun bx_setsExpectedValue() {
        val measurement = MagnetometerSensorMeasurement()

        // check default value
        assertEquals(0.0f, measurement.bx, 0.0f)

        // set new value
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        measurement.bx = bx

        // check
        assertEquals(bx, measurement.bx, 0.0f)
    }

    @Test
    fun by_setsExpectedValue() {
        val measurement = MagnetometerSensorMeasurement()

        // check default value
        assertEquals(0.0f, measurement.by, 0.0f)

        // set new value
        val randomizer = UniformRandomizer()
        val by = randomizer.nextFloat()
        measurement.by = by

        // check
        assertEquals(by, measurement.by, 0.0f)
    }

    @Test
    fun bz_setsExpectedValue() {
        val measurement = MagnetometerSensorMeasurement()

        // check default value
        assertEquals(0.0f, measurement.bz, 0.0f)

        // set new value
        val randomizer = UniformRandomizer()
        val bz = randomizer.nextFloat()
        measurement.bz = bz

        // check
        assertEquals(bz, measurement.bz, 0.0f)
    }

    @Test
    fun hardIronX_setsExpectedValue() {
        val measurement = MagnetometerSensorMeasurement()

        // check default value
        assertNull(measurement.hardIronX)

        // set new value
        val randomizer = UniformRandomizer()
        val hardIronX = randomizer.nextFloat()
        measurement.hardIronX = hardIronX

        // check
        assertEquals(hardIronX, measurement.hardIronX)

        // set to null
        measurement.hardIronX = null

        // check
        @Suppress("KotlinConstantConditions")
        assertNull(measurement.hardIronX)
    }

    @Test
    fun hardIronY_setsExpectedValue() {
        val measurement = MagnetometerSensorMeasurement()

        // check default value
        assertNull(measurement.hardIronY)

        // set new value
        val randomizer = UniformRandomizer()
        val hardIronY = randomizer.nextFloat()
        measurement.hardIronY = hardIronY

        // check
        assertEquals(hardIronY, measurement.hardIronY)

        // set to null
        measurement.hardIronY = null

        @Suppress("KotlinConstantConditions")
        assertNull(measurement.hardIronY)
    }

    @Test
    fun hardIronZ_setsExpectedValue() {
        val measurement = MagnetometerSensorMeasurement()

        // check default value
        assertNull(measurement.hardIronZ)

        // set new value
        val randomizer = UniformRandomizer()
        val hardIronZ = randomizer.nextFloat()
        measurement.hardIronZ = hardIronZ

        // check
        assertEquals(hardIronZ, measurement.hardIronZ)

        // set to null
        measurement.hardIronZ = null

        @Suppress("KotlinConstantConditions")
        assertNull(measurement.hardIronZ)
    }

    @Test
    fun timestamp_setsExpectedValue() {
        val measurement = MagnetometerSensorMeasurement()

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
        val measurement = MagnetometerSensorMeasurement()

        // check default value
        assertNull(measurement.accuracy)

        // set new value
        measurement.accuracy = SensorAccuracy.LOW

        // check
        assertEquals(SensorAccuracy.LOW, measurement.accuracy)

        // set to null
        measurement.accuracy = null

        @Suppress("KotlinConstantConditions")
        assertNull(measurement.accuracy)
    }

    @Test
    fun sensorType_setsExpectedValue() {
        val measurement = MagnetometerSensorMeasurement()

        // check default value
        assertEquals(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED, measurement.sensorType)

        // set new value
        measurement.sensorType = MagnetometerSensorType.MAGNETOMETER

        // check
        assertEquals(MagnetometerSensorType.MAGNETOMETER, measurement.sensorType)
    }

    @Test
    fun copyFrom_makesExpectedCopy() {
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()

        val measurement1 =
            MagnetometerSensorMeasurement(
                bx,
                by,
                bz,
                hardIronX,
                hardIronY,
                hardIronZ,
                timestamp,
                SensorAccuracy.LOW,
                MagnetometerSensorType.MAGNETOMETER
            )

        val measurement2 = MagnetometerSensorMeasurement()
        measurement2.copyFrom(measurement1)

        // check
        assertEquals(bx, measurement1.bx, 0.0f)
        assertEquals(by, measurement1.by, 0.0f)
        assertEquals(bz, measurement1.bz, 0.0f)
        assertEquals(hardIronX, measurement1.hardIronX)
        assertEquals(hardIronY, measurement1.hardIronY)
        assertEquals(hardIronZ, measurement1.hardIronZ)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.LOW, measurement1.accuracy)
        assertEquals(MagnetometerSensorType.MAGNETOMETER, measurement1.sensorType)

        assertEquals(measurement1.bx, measurement2.bx, 0.0f)
        assertEquals(measurement1.by, measurement2.by, 0.0f)
        assertEquals(measurement1.bz, measurement2.bz, 0.0f)
        assertEquals(measurement1.hardIronX, measurement2.hardIronX)
        assertEquals(measurement1.hardIronY, measurement2.hardIronY)
        assertEquals(measurement1.hardIronZ, measurement2.hardIronZ)
        assertEquals(measurement1.timestamp, measurement2.timestamp)
        assertEquals(measurement1.accuracy, measurement2.accuracy)
        assertEquals(measurement1.sensorType, measurement2.sensorType)
    }

    @Test
    fun copyTo_makesExpectedCopy() {
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()

        val measurement1 =
            MagnetometerSensorMeasurement(
                bx,
                by,
                bz,
                hardIronX,
                hardIronY,
                hardIronZ,
                timestamp,
                SensorAccuracy.LOW,
                MagnetometerSensorType.MAGNETOMETER
            )

        val measurement2 = MagnetometerSensorMeasurement()
        measurement1.copyTo(measurement2)

        // check
        assertEquals(bx, measurement1.bx, 0.0f)
        assertEquals(by, measurement1.by, 0.0f)
        assertEquals(bz, measurement1.bz, 0.0f)
        assertEquals(hardIronX, measurement1.hardIronX)
        assertEquals(hardIronY, measurement1.hardIronY)
        assertEquals(hardIronZ, measurement1.hardIronZ)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.LOW, measurement1.accuracy)
        assertEquals(MagnetometerSensorType.MAGNETOMETER, measurement1.sensorType)

        assertEquals(measurement1.bx, measurement2.bx, 0.0f)
        assertEquals(measurement1.by, measurement2.by, 0.0f)
        assertEquals(measurement1.bz, measurement2.bz, 0.0f)
        assertEquals(measurement1.hardIronX, measurement2.hardIronX)
        assertEquals(measurement1.hardIronY, measurement2.hardIronY)
        assertEquals(measurement1.hardIronZ, measurement2.hardIronZ)
        assertEquals(measurement1.timestamp, measurement2.timestamp)
        assertEquals(measurement1.accuracy, measurement2.accuracy)
        assertEquals(measurement1.sensorType, measurement2.sensorType)
    }

    @Test
    fun copy_makesExpectedCopy() {
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()

        val measurement1 =
            MagnetometerSensorMeasurement(
                bx,
                by,
                bz,
                hardIronX,
                hardIronY,
                hardIronZ,
                timestamp,
                SensorAccuracy.LOW,
                MagnetometerSensorType.MAGNETOMETER
            )

        val measurement2 = measurement1.copy()

        // check
        assertEquals(bx, measurement1.bx, 0.0f)
        assertEquals(by, measurement1.by, 0.0f)
        assertEquals(bz, measurement1.bz, 0.0f)
        assertEquals(hardIronX, measurement1.hardIronX)
        assertEquals(hardIronY, measurement1.hardIronY)
        assertEquals(hardIronZ, measurement1.hardIronZ)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.LOW, measurement1.accuracy)
        assertEquals(MagnetometerSensorType.MAGNETOMETER, measurement1.sensorType)

        assertEquals(measurement1.bx, measurement2.bx, 0.0f)
        assertEquals(measurement1.by, measurement2.by, 0.0f)
        assertEquals(measurement1.bz, measurement2.bz, 0.0f)
        assertEquals(measurement1.hardIronX, measurement2.hardIronX)
        assertEquals(measurement1.hardIronY, measurement2.hardIronY)
        assertEquals(measurement1.hardIronZ, measurement2.hardIronZ)
        assertEquals(measurement1.timestamp, measurement2.timestamp)
        assertEquals(measurement1.accuracy, measurement2.accuracy)
        assertEquals(measurement1.sensorType, measurement2.sensorType)
    }
}