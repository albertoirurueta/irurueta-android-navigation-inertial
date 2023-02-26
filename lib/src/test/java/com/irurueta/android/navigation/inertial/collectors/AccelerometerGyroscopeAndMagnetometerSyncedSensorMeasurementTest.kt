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

import com.irurueta.statistics.UniformRandomizer
import io.mockk.clearAllMocks
import io.mockk.unmockkAll
import org.junit.After
import org.junit.Assert.*
import org.junit.Test

class AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurementTest {

    @After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
    }

    @Test
    fun constructor_whenNoParameters_setsExpectedValues() {
        val syncedMeasurement = AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement()

        // check
        assertNull(syncedMeasurement.accelerometerMeasurement)
        assertNull(syncedMeasurement.gyroscopeMeasurement)
        assertNull(syncedMeasurement.magnetometerMeasurement)
        assertEquals(0L, syncedMeasurement.timestamp)
    }

    @Test
    fun constructor_whenAllParameters_setsExpectedValues() {
        val accelerometerMeasurement = AccelerometerSensorMeasurement()
        val gyroscopeMeasurement = GyroscopeSensorMeasurement()
        val magnetometerMeasurement = MagnetometerSensorMeasurement()
        val timestamp = UniformRandomizer().nextLong()
        val syncedMeasurement = AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement(
            accelerometerMeasurement,
            gyroscopeMeasurement,
            magnetometerMeasurement,
            timestamp
        )

        // check
        assertSame(accelerometerMeasurement, syncedMeasurement.accelerometerMeasurement)
        assertSame(gyroscopeMeasurement, syncedMeasurement.gyroscopeMeasurement)
        assertSame(magnetometerMeasurement, syncedMeasurement.magnetometerMeasurement)
        assertEquals(timestamp, syncedMeasurement.timestamp)
    }

    @Test
    fun accelerometerMeasurement_setsExpectedValue() {
        val syncedMeasurement =
            AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement()

        // check default value
        assertNull(syncedMeasurement.accelerometerMeasurement)

        // set new value
        val accelerometerMeasurement = AccelerometerSensorMeasurement()
        syncedMeasurement.accelerometerMeasurement = accelerometerMeasurement

        // check
        assertSame(accelerometerMeasurement, syncedMeasurement.accelerometerMeasurement)
    }

    @Test
    fun gyroscopeMeasurement_setsExpectedValue() {
        val syncedMeasurement =
            AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement()

        // check default value
        assertNull(syncedMeasurement.gyroscopeMeasurement)

        // set new value
        val gyroscopeMeasurement = GyroscopeSensorMeasurement()
        syncedMeasurement.gyroscopeMeasurement = gyroscopeMeasurement

        // check
        assertSame(gyroscopeMeasurement, syncedMeasurement.gyroscopeMeasurement)
    }

    @Test
    fun magnetometerMeasurement_setsExpectedValue() {
        val syncedMeasurement =
            AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement()

        // check default value
        assertNull(syncedMeasurement.magnetometerMeasurement)

        // set new value
        val magnetometerMeasurement = MagnetometerSensorMeasurement()
        syncedMeasurement.magnetometerMeasurement = magnetometerMeasurement

        // check
        assertSame(magnetometerMeasurement, syncedMeasurement.magnetometerMeasurement)
    }

    @Test
    fun timestamp_setsExpectedValue() {
        val syncedMeasurement =
            AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement()

        // check default value
        assertEquals(0L, syncedMeasurement.timestamp)

        // set new value
        val timestamp = UniformRandomizer().nextLong()
        syncedMeasurement.timestamp = timestamp

        // check
        assertEquals(timestamp, syncedMeasurement.timestamp)
    }
}