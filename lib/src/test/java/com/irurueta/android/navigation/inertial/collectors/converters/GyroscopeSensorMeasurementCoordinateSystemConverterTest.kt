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

import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorCoordinateSystem
import com.irurueta.statistics.UniformRandomizer
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertThrows
import org.junit.Assert.fail
import org.junit.Test

class GyroscopeSensorMeasurementCoordinateSystemConverterTest {

    @Test
    fun toNedOrThrow_whenGyroscopeSensorMeasurementWithoutOptionalValues_returnsExpectedValues() {
        val randomizer = UniformRandomizer()
        val wX = randomizer.nextFloat()
        val wY = randomizer.nextFloat()
        val wZ = randomizer.nextFloat()

        val timestamp = System.nanoTime()

        val enuMeasurement = GyroscopeSensorMeasurement(
            wx = wX,
            wy = wY,
            wz = wZ,
            timestamp = timestamp,
            sensorType = GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            sensorCoordinateSystem = SensorCoordinateSystem.ENU
        )

        val nedMeasurement1 = GyroscopeSensorMeasurement()
        GyroscopeSensorMeasurementCoordinateSystemConverter.toNedOrThrow(
            enuMeasurement,
            nedMeasurement1
        )
        val nedMeasurement2 = GyroscopeSensorMeasurementCoordinateSystemConverter.toNedOrThrow(
            enuMeasurement
        )

        // check
        assertEquals(enuMeasurement.wy, nedMeasurement1.wx, 0.0f)
        assertEquals(enuMeasurement.wx, nedMeasurement1.wy, 0.0f)
        assertEquals(-enuMeasurement.wz, nedMeasurement1.wz, 0.0f)
        assertNull(nedMeasurement1.bx)
        assertNull(nedMeasurement1.by)
        assertNull(nedMeasurement1.bz)
        assertEquals(enuMeasurement.timestamp, nedMeasurement1.timestamp)
        assertNull(nedMeasurement1.accuracy)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            nedMeasurement1.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.NED,
            nedMeasurement1.sensorCoordinateSystem
        )
        assertEquals(nedMeasurement1, nedMeasurement2)
    }

    @Test
    fun toNedOrThrow_whenGyroscopeSensorMeasurementWithAllValues_returnsExpectedValues() {
        val randomizer = UniformRandomizer()
        val wX = randomizer.nextFloat()
        val wY = randomizer.nextFloat()
        val wZ = randomizer.nextFloat()

        val bX = randomizer.nextFloat()
        val bY = randomizer.nextFloat()
        val bZ = randomizer.nextFloat()

        val timestamp = System.nanoTime()

        val enuMeasurement = GyroscopeSensorMeasurement(
            wx = wX,
            wy = wY,
            wz = wZ,
            bx = bX,
            by = bY,
            bz = bZ,
            timestamp = timestamp,
            accuracy = SensorAccuracy.HIGH,
            sensorType = GyroscopeSensorType.GYROSCOPE,
            sensorCoordinateSystem = SensorCoordinateSystem.ENU
        )

        val nedMeasurement1 = GyroscopeSensorMeasurement()
        GyroscopeSensorMeasurementCoordinateSystemConverter.toNedOrThrow(
            enuMeasurement,
            nedMeasurement1
        )
        val nedMeasurement2 = GyroscopeSensorMeasurementCoordinateSystemConverter.toNedOrThrow(
            enuMeasurement
        )

        // check
        assertEquals(enuMeasurement.wy, nedMeasurement1.wx, 0.0f)
        assertEquals(enuMeasurement.wx, nedMeasurement1.wy, 0.0f)
        assertEquals(-enuMeasurement.wz, nedMeasurement1.wz, 0.0f)
        val nedBx = (nedMeasurement1.bx ?: fail()) as Float
        val nedBy = (nedMeasurement1.by ?: fail()) as Float
        val nedBz = (nedMeasurement1.bz ?: fail()) as Float
        assertEquals(bY, nedBx, 0.0f)
        assertEquals(bX, nedBy, 0.0f)
        assertEquals(-bZ, nedBz, 0.0f)
        assertEquals(enuMeasurement.timestamp, nedMeasurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, nedMeasurement1.accuracy)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            nedMeasurement1.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.NED,
            nedMeasurement1.sensorCoordinateSystem
        )
        assertEquals(nedMeasurement1, nedMeasurement2)
    }

    @Test
    fun toNedOrThrow_whenGyroscopeInvalidCoordinateSystem_throwsIllegalArgumentException() {
        val measurement = GyroscopeSensorMeasurement(
            sensorCoordinateSystem = SensorCoordinateSystem.NED
        )

        assertThrows(IllegalArgumentException::class.java) {
            GyroscopeSensorMeasurementCoordinateSystemConverter.toNedOrThrow(
                measurement,
                GyroscopeSensorMeasurement()
            )
        }
        assertThrows(IllegalArgumentException::class.java) {
            GyroscopeSensorMeasurementCoordinateSystemConverter.toNedOrThrow(measurement)
        }
    }

    @Test
    fun toEnuOrThrow_whenGyroscopeSensorMeasurementWithoutOptionalValues_returnsExpectedValues() {
        val randomizer = UniformRandomizer()
        val wX = randomizer.nextFloat()
        val wY = randomizer.nextFloat()
        val wZ = randomizer.nextFloat()

        val timestamp = System.nanoTime()

        val nedMeasurement = GyroscopeSensorMeasurement(
            wx = wX,
            wy = wY,
            wz = wZ,
            timestamp = timestamp,
            sensorType = GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            sensorCoordinateSystem = SensorCoordinateSystem.NED
        )

        val enuMeasurement1 = GyroscopeSensorMeasurement()
        GyroscopeSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(
            nedMeasurement,
            enuMeasurement1
        )
        val enuMeasurement2 = GyroscopeSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(
            nedMeasurement
        )

        // check
        assertEquals(nedMeasurement.wy, enuMeasurement1.wx, 0.0f)
        assertEquals(nedMeasurement.wx, enuMeasurement1.wy, 0.0f)
        assertEquals(-nedMeasurement.wz, enuMeasurement1.wz, 0.0f)
        assertNull(enuMeasurement1.bx)
        assertNull(enuMeasurement1.by)
        assertNull(enuMeasurement1.bz)
        assertEquals(nedMeasurement.timestamp, enuMeasurement1.timestamp)
        assertNull(enuMeasurement1.accuracy)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            enuMeasurement1.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.ENU,
            enuMeasurement1.sensorCoordinateSystem
        )
        assertEquals(enuMeasurement1, enuMeasurement2)
    }

    @Test
    fun toEnuOrThrow_whenGyroscopeSensorMeasurementWithAllValues_returnsExpectedValues() {
        val randomizer = UniformRandomizer()
        val wX = randomizer.nextFloat()
        val wY = randomizer.nextFloat()
        val wZ = randomizer.nextFloat()

        val bX = randomizer.nextFloat()
        val bY = randomizer.nextFloat()
        val bZ = randomizer.nextFloat()

        val timestamp = System.nanoTime()

        val nedMeasurement = GyroscopeSensorMeasurement(
            wx = wX,
            wy = wY,
            wz = wZ,
            bx = bX,
            by = bY,
            bz = bZ,
            timestamp = timestamp,
            accuracy = SensorAccuracy.HIGH,
            sensorType = GyroscopeSensorType.GYROSCOPE,
            sensorCoordinateSystem = SensorCoordinateSystem.NED
        )

        val enuMeasurement1 = GyroscopeSensorMeasurement()
        GyroscopeSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(
            nedMeasurement,
            enuMeasurement1
        )
        val enuMeasurement2 = GyroscopeSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(
            nedMeasurement
        )

        // check
        assertEquals(nedMeasurement.wy, enuMeasurement1.wx, 0.0f)
        assertEquals(nedMeasurement.wx, enuMeasurement1.wy, 0.0f)
        assertEquals(-nedMeasurement.wz, enuMeasurement1.wz, 0.0f)
        val enuBx = (enuMeasurement1.bx ?: fail()) as Float
        val enuBy = (enuMeasurement1.by ?: fail()) as Float
        val enuBz = (enuMeasurement1.bz ?: fail()) as Float
        assertEquals(bY, enuBx, 0.0f)
        assertEquals(bX, enuBy, 0.0f)
        assertEquals(-bZ, enuBz, 0.0f)
        assertEquals(nedMeasurement.timestamp, enuMeasurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, enuMeasurement1.accuracy)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            enuMeasurement1.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.ENU,
            enuMeasurement1.sensorCoordinateSystem
        )
        assertEquals(enuMeasurement1, enuMeasurement2)
    }

    @Test
    fun toEnuOrThrow_whenInvalidGyroscopeCoordinateSystem_throwsIllegalArgumentException() {
        val measurement = GyroscopeSensorMeasurement(
            sensorCoordinateSystem = SensorCoordinateSystem.ENU
        )

        assertThrows(IllegalArgumentException::class.java) {
            GyroscopeSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(
                measurement,
                GyroscopeSensorMeasurement()
            )
        }
        assertThrows(IllegalArgumentException::class.java) {
            GyroscopeSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(measurement)
        }
    }

    @Test
    fun toNed_whenAlreadyNED_copiesInput() {
        val randomizer = UniformRandomizer()
        val wX = randomizer.nextFloat()
        val wY = randomizer.nextFloat()
        val wZ = randomizer.nextFloat()

        val bX = randomizer.nextFloat()
        val bY = randomizer.nextFloat()
        val bZ = randomizer.nextFloat()

        val timestamp = System.nanoTime()

        val nedMeasurement1 = GyroscopeSensorMeasurement(
            wx = wX,
            wy = wY,
            wz = wZ,
            bx = bX,
            by = bY,
            bz = bZ,
            timestamp = timestamp,
            accuracy = SensorAccuracy.HIGH,
            sensorType = GyroscopeSensorType.GYROSCOPE,
            sensorCoordinateSystem = SensorCoordinateSystem.NED
        )

        val nedMeasurement2 = GyroscopeSensorMeasurement()
        GyroscopeSensorMeasurementCoordinateSystemConverter.toNed(
            nedMeasurement1,
            nedMeasurement2
        )
        val nedMeasurement3 = GyroscopeSensorMeasurementCoordinateSystemConverter.toNed(
            nedMeasurement1
        )

        // check
        assertEquals(nedMeasurement1, nedMeasurement2)
        assertEquals(nedMeasurement1, nedMeasurement3)
    }

    @Test
    fun toNed_whenENU_returnsExpectedValues() {
        val randomizer = UniformRandomizer()
        val wX = randomizer.nextFloat()
        val wY = randomizer.nextFloat()
        val wZ = randomizer.nextFloat()

        val bX = randomizer.nextFloat()
        val bY = randomizer.nextFloat()
        val bZ = randomizer.nextFloat()

        val timestamp = System.nanoTime()

        val enuMeasurement = GyroscopeSensorMeasurement(
            wx = wX,
            wy = wY,
            wz = wZ,
            bx = bX,
            by = bY,
            bz = bZ,
            timestamp = timestamp,
            accuracy = SensorAccuracy.HIGH,
            sensorType = GyroscopeSensorType.GYROSCOPE,
            sensorCoordinateSystem = SensorCoordinateSystem.ENU
        )

        val nedMeasurement1 = GyroscopeSensorMeasurement()
        GyroscopeSensorMeasurementCoordinateSystemConverter.toNed(
            enuMeasurement,
            nedMeasurement1
        )
        val nedMeasurement2 = GyroscopeSensorMeasurementCoordinateSystemConverter.toNed(
            enuMeasurement
        )

        // check
        assertEquals(enuMeasurement.wy, nedMeasurement1.wx, 0.0f)
        assertEquals(enuMeasurement.wx, nedMeasurement1.wy, 0.0f)
        assertEquals(-enuMeasurement.wz, nedMeasurement1.wz, 0.0f)
        val nedBx = (nedMeasurement1.bx ?: fail()) as Float
        val nedBy = (nedMeasurement1.by ?: fail()) as Float
        val nedBz = (nedMeasurement1.bz ?: fail()) as Float
        assertEquals(bY, nedBx, 0.0f)
        assertEquals(bX, nedBy, 0.0f)
        assertEquals(-bZ, nedBz, 0.0f)
        assertEquals(enuMeasurement.timestamp, nedMeasurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, nedMeasurement1.accuracy)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            nedMeasurement1.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.NED,
            nedMeasurement1.sensorCoordinateSystem
        )
        assertEquals(nedMeasurement1, nedMeasurement2)
    }

    @Test
    fun toEnu_whenAlreadyENU_copiesInput() {
        val randomizer = UniformRandomizer()
        val wX = randomizer.nextFloat()
        val wY = randomizer.nextFloat()
        val wZ = randomizer.nextFloat()

        val bX = randomizer.nextFloat()
        val bY = randomizer.nextFloat()
        val bZ = randomizer.nextFloat()

        val timestamp = System.nanoTime()

        val enuMeasurement1 = GyroscopeSensorMeasurement(
            wx = wX,
            wy = wY,
            wz = wZ,
            bx = bX,
            by = bY,
            bz = bZ,
            timestamp = timestamp,
            accuracy = SensorAccuracy.HIGH,
            sensorType = GyroscopeSensorType.GYROSCOPE,
            sensorCoordinateSystem = SensorCoordinateSystem.ENU
        )

        val enuMeasurement2 = GyroscopeSensorMeasurement()
        GyroscopeSensorMeasurementCoordinateSystemConverter.toEnu(
            enuMeasurement1,
            enuMeasurement2
        )
        val enuMeasurement3 = GyroscopeSensorMeasurementCoordinateSystemConverter.toEnu(
            enuMeasurement1
        )

        // check
        assertEquals(enuMeasurement1, enuMeasurement2)
        assertEquals(enuMeasurement1, enuMeasurement3)
    }

    @Test
    fun toEnu_whenNED_returnsExpectedValues() {
        val randomizer = UniformRandomizer()
        val wX = randomizer.nextFloat()
        val wY = randomizer.nextFloat()
        val wZ = randomizer.nextFloat()

        val bX = randomizer.nextFloat()
        val bY = randomizer.nextFloat()
        val bZ = randomizer.nextFloat()

        val timestamp = System.nanoTime()

        val nedMeasurement = GyroscopeSensorMeasurement(
            wx = wX,
            wy = wY,
            wz = wZ,
            bx = bX,
            by = bY,
            bz = bZ,
            timestamp = timestamp,
            accuracy = SensorAccuracy.HIGH,
            sensorType = GyroscopeSensorType.GYROSCOPE,
            sensorCoordinateSystem = SensorCoordinateSystem.NED
        )

        val enuMeasurement1 = GyroscopeSensorMeasurement()
        GyroscopeSensorMeasurementCoordinateSystemConverter.toEnu(
            nedMeasurement,
            enuMeasurement1
        )
        val enuMeasurement2 = GyroscopeSensorMeasurementCoordinateSystemConverter.toEnu(
            nedMeasurement
        )

        // check
        assertEquals(nedMeasurement.wy, enuMeasurement1.wx, 0.0f)
        assertEquals(nedMeasurement.wx, enuMeasurement1.wy, 0.0f)
        assertEquals(-nedMeasurement.wz, enuMeasurement1.wz, 0.0f)
        val enuBx = (enuMeasurement1.bx ?: fail()) as Float
        val enuBy = (enuMeasurement1.by ?: fail()) as Float
        val enuBz = (enuMeasurement1.bz ?: fail()) as Float
        assertEquals(bY, enuBx, 0.0f)
        assertEquals(bX, enuBy, 0.0f)
        assertEquals(-bZ, enuBz, 0.0f)
        assertEquals(nedMeasurement.timestamp, enuMeasurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, enuMeasurement1.accuracy)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            enuMeasurement1.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.ENU,
            enuMeasurement1.sensorCoordinateSystem
        )
        assertEquals(enuMeasurement1, enuMeasurement2)
    }
}