/*
 * Copyright (C) 2025 Alberto Irurueta Carro (alberto@irurueta.com)
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

package com.irurueta.android.navigation.inertial.collectors.converters

import com.irurueta.android.navigation.inertial.collectors.measurements.GravitySensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorCoordinateSystem
import com.irurueta.statistics.UniformRandomizer
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertThrows
import org.junit.Test

class GravitySensorMeasurementCoordinateSystemConverterTest {

    @Test
    fun toNedOrThrow_whenGravitySensorMeasurementWithoutOptionalValues_returnsExpectedValues() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()

        val timestamp = System.nanoTime()

        val enuMeasurement = GravitySensorMeasurement(
            gx = gx,
            gy = gy,
            gz = gz,
            timestamp = timestamp,
            sensorCoordinateSystem = SensorCoordinateSystem.ENU
        )

        val nedMeasurement1 = GravitySensorMeasurement()
        GravitySensorMeasurementCoordinateSystemConverter.toNedOrThrow(
            enuMeasurement,
            nedMeasurement1
        )
        val nedMeasurement2 = GravitySensorMeasurementCoordinateSystemConverter.toNedOrThrow(
            enuMeasurement
        )

        // check
        assertEquals(enuMeasurement.gy, nedMeasurement1.gx, 0.0f)
        assertEquals(enuMeasurement.gx, nedMeasurement1.gy, 0.0f)
        assertEquals(-enuMeasurement.gz, nedMeasurement1.gz, 0.0f)
        assertEquals(enuMeasurement.timestamp, nedMeasurement1.timestamp)
        assertNull(nedMeasurement1.accuracy)
        assertEquals(
            SensorCoordinateSystem.NED,
            nedMeasurement1.sensorCoordinateSystem
        )
        assertEquals(nedMeasurement1, nedMeasurement2)
    }

    @Test
    fun toNedOrThrow_whenGravitySensorMeasurementWithAllValues_throwsIllegalArgumentException() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()

        val timestamp = System.nanoTime()

        val enuMeasurement = GravitySensorMeasurement(
            gx = gx,
            gy = gy,
            gz = gz,
            timestamp = timestamp,
            accuracy = SensorAccuracy.HIGH,
            sensorCoordinateSystem = SensorCoordinateSystem.ENU
        )

        val nedMeasurement1 = GravitySensorMeasurement()
        GravitySensorMeasurementCoordinateSystemConverter.toNedOrThrow(
            enuMeasurement,
            nedMeasurement1
        )
        val nedMeasurement2 = GravitySensorMeasurementCoordinateSystemConverter.toNedOrThrow(
            enuMeasurement
        )

        // check
        assertEquals(enuMeasurement.gy, nedMeasurement1.gx, 0.0f)
        assertEquals(enuMeasurement.gx, nedMeasurement1.gy, 0.0f)
        assertEquals(-enuMeasurement.gz, nedMeasurement1.gz, 0.0f)
        assertEquals(enuMeasurement.timestamp, nedMeasurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, nedMeasurement1.accuracy)
        assertEquals(
            SensorCoordinateSystem.NED,
            nedMeasurement1.sensorCoordinateSystem
        )
        assertEquals(nedMeasurement1, nedMeasurement2)
    }

    @Test
    fun toNedOrThrow_whenInvalidGravityCoordinateSystem_throwsIllegalArgumentException() {
        val measurement = GravitySensorMeasurement(
            sensorCoordinateSystem = SensorCoordinateSystem.NED
        )

        assertThrows(IllegalArgumentException::class.java) {
            GravitySensorMeasurementCoordinateSystemConverter.toNedOrThrow(
                measurement,
                GravitySensorMeasurement()
            )
        }
        assertThrows(IllegalArgumentException::class.java) {
            GravitySensorMeasurementCoordinateSystemConverter.toNedOrThrow(measurement)
        }
    }

    @Test
    fun toEnuOrThrow_whenGravitySensorMeasurementWithoutOptionalValues_returnsExpectedValues() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()

        val timestamp = System.nanoTime()

        val nedMeasurement = GravitySensorMeasurement(
            gx = gx,
            gy = gy,
            gz = gz,
            timestamp = timestamp,
            sensorCoordinateSystem = SensorCoordinateSystem.NED
        )

        val enuMeasurement1 = GravitySensorMeasurement()
        GravitySensorMeasurementCoordinateSystemConverter.toEnuOrThrow(
            nedMeasurement,
            enuMeasurement1
        )
        val enuMeasurement2 = GravitySensorMeasurementCoordinateSystemConverter.toEnuOrThrow(
            nedMeasurement
        )

        // check
        assertEquals(nedMeasurement.gy, enuMeasurement1.gx, 0.0f)
        assertEquals(nedMeasurement.gx, enuMeasurement1.gy, 0.0f)
        assertEquals(-nedMeasurement.gz, enuMeasurement1.gz, 0.0f)
        assertEquals(nedMeasurement.timestamp, enuMeasurement1.timestamp)
        assertNull(enuMeasurement1.accuracy)
        assertEquals(
            SensorCoordinateSystem.ENU,
            enuMeasurement1.sensorCoordinateSystem
        )
        assertEquals(enuMeasurement1, enuMeasurement2)
    }

    @Test
    fun toEnuOrThrow_whenGravitySensorMeasurementWithAllValues_throwsIllegalArgumentException() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()

        val timestamp = System.nanoTime()

        val nedMeasurement = GravitySensorMeasurement(
            gx = gx,
            gy = gy,
            gz = gz,
            timestamp = timestamp,
            accuracy = SensorAccuracy.HIGH,
            sensorCoordinateSystem = SensorCoordinateSystem.NED
        )

        val enuMeasurement1 = GravitySensorMeasurement()
        GravitySensorMeasurementCoordinateSystemConverter.toEnuOrThrow(
            nedMeasurement,
            enuMeasurement1
        )
        val enuMeasurement2 = GravitySensorMeasurementCoordinateSystemConverter.toEnuOrThrow(
            nedMeasurement
        )

        // check
        assertEquals(nedMeasurement.gy, enuMeasurement1.gx, 0.0f)
        assertEquals(nedMeasurement.gx, enuMeasurement1.gy, 0.0f)
        assertEquals(-nedMeasurement.gz, enuMeasurement1.gz, 0.0f)
        assertEquals(nedMeasurement.timestamp, enuMeasurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, enuMeasurement1.accuracy)
        assertEquals(
            SensorCoordinateSystem.ENU,
            enuMeasurement1.sensorCoordinateSystem
        )
        assertEquals(enuMeasurement1, enuMeasurement2)
    }

    @Test
    fun toEnuOrThrow_whenInvalidGravityCoordinateSystem_throwsIllegalArgumentException() {
        val measurement = GravitySensorMeasurement(
            sensorCoordinateSystem = SensorCoordinateSystem.ENU
        )

        assertThrows(IllegalArgumentException::class.java) {
            GravitySensorMeasurementCoordinateSystemConverter.toEnuOrThrow(
                measurement,
                GravitySensorMeasurement()
            )
        }
        assertThrows(IllegalArgumentException::class.java) {
            GravitySensorMeasurementCoordinateSystemConverter.toEnuOrThrow(measurement)
        }
    }

    @Test
    fun toNed_whenAlreadyNED_copiesInput() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()

        val timestamp = System.nanoTime()

        val nedMeasurement1 = GravitySensorMeasurement(
            gx = gx,
            gy = gy,
            gz = gz,
            timestamp = timestamp,
            sensorCoordinateSystem = SensorCoordinateSystem.NED
        )

        val nedMeasurement2 = GravitySensorMeasurement()
        GravitySensorMeasurementCoordinateSystemConverter.toNed(
            nedMeasurement1,
            nedMeasurement2
        )
        val nedMeasurement3 = GravitySensorMeasurementCoordinateSystemConverter.toNed(
            nedMeasurement1)

        // check
        assertEquals(nedMeasurement1, nedMeasurement2)
        assertEquals(nedMeasurement1, nedMeasurement3)
    }

    @Test
    fun toNed_whenENU_returnsExpectedValues() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()

        val timestamp = System.nanoTime()

        val enuMeasurement = GravitySensorMeasurement(
            gx = gx,
            gy = gy,
            gz = gz,
            timestamp = timestamp,
            sensorCoordinateSystem = SensorCoordinateSystem.ENU
        )

        val nedMeasurement1 = GravitySensorMeasurement()
        GravitySensorMeasurementCoordinateSystemConverter.toNed(
            enuMeasurement,
            nedMeasurement1
        )
        val nedMeasurement2 = GravitySensorMeasurementCoordinateSystemConverter.toNed(
            enuMeasurement
        )

        // check
        assertEquals(enuMeasurement.gy, nedMeasurement1.gx, 0.0f)
        assertEquals(enuMeasurement.gx, nedMeasurement1.gy, 0.0f)
        assertEquals(-enuMeasurement.gz, nedMeasurement1.gz, 0.0f)
        assertEquals(enuMeasurement.timestamp, nedMeasurement1.timestamp)
        assertNull(nedMeasurement1.accuracy)
        assertEquals(
            SensorCoordinateSystem.NED,
            nedMeasurement1.sensorCoordinateSystem
        )
        assertEquals(nedMeasurement1, nedMeasurement2)
    }

    @Test
    fun toEnu_whenAlreadyENU_copiesInput() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()

        val timestamp = System.nanoTime()

        val enuMeasurement1 = GravitySensorMeasurement(
            gx = gx,
            gy = gy,
            gz = gz,
            timestamp = timestamp,
            sensorCoordinateSystem = SensorCoordinateSystem.ENU
        )

        val enuMeasurement2 = GravitySensorMeasurement()
        GravitySensorMeasurementCoordinateSystemConverter.toEnu(
            enuMeasurement1,
            enuMeasurement2
        )
        val enuMeasurement3 = GravitySensorMeasurementCoordinateSystemConverter.toEnu(
            enuMeasurement1)

        // check
        assertEquals(enuMeasurement1, enuMeasurement2)
        assertEquals(enuMeasurement1, enuMeasurement3)
    }

    @Test
    fun toEnu_whenNED_returnsExpectedValues() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()

        val timestamp = System.nanoTime()

        val nedMeasurement = GravitySensorMeasurement(
            gx = gx,
            gy = gy,
            gz = gz,
            timestamp = timestamp,
            sensorCoordinateSystem = SensorCoordinateSystem.NED
        )

        val enuMeasurement1 = GravitySensorMeasurement()
        GravitySensorMeasurementCoordinateSystemConverter.toEnu(
            nedMeasurement,
            enuMeasurement1
        )
        val enuMeasurement2 = GravitySensorMeasurementCoordinateSystemConverter.toEnu(
            nedMeasurement
        )

        // check
        assertEquals(nedMeasurement.gy, enuMeasurement1.gx, 0.0f)
        assertEquals(nedMeasurement.gx, enuMeasurement1.gy, 0.0f)
        assertEquals(-nedMeasurement.gz, enuMeasurement1.gz, 0.0f)
        assertEquals(nedMeasurement.timestamp, enuMeasurement1.timestamp)
        assertNull(enuMeasurement1.accuracy)
        assertEquals(
            SensorCoordinateSystem.ENU,
            enuMeasurement1.sensorCoordinateSystem
        )
        assertEquals(enuMeasurement1, enuMeasurement2)
    }
}