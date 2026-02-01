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

import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorCoordinateSystem
import com.irurueta.statistics.UniformRandomizer
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertThrows
import org.junit.Assert.fail
import org.junit.Test

class MagnetometerSensorMeasurementCoordinateSystemConverterTest {

    @Test
    fun toNedOrThrow_whenMagnetometerSensorMeasurementWithoutOptionalValues_returnsExpectedValues() {
        val randomizer = UniformRandomizer()
        val bX = randomizer.nextFloat()
        val bY = randomizer.nextFloat()
        val bZ = randomizer.nextFloat()

        val timestamp = System.nanoTime()

        val enuMeasurement = MagnetometerSensorMeasurement(
            bx = bX,
            by = bY,
            bz = bZ,
            timestamp = timestamp,
            sensorType = MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            sensorCoordinateSystem = SensorCoordinateSystem.ENU
        )

        val nedMeasurement1 = MagnetometerSensorMeasurement()
        MagnetometerSensorMeasurementCoordinateSystemConverter.toNedOrThrow(
            enuMeasurement,
            nedMeasurement1
        )
        val nedMeasurement2 = MagnetometerSensorMeasurementCoordinateSystemConverter.toNedOrThrow(
            enuMeasurement
        )

        // check
        assertEquals(enuMeasurement.by, nedMeasurement1.bx, 0.0f)
        assertEquals(enuMeasurement.bx, nedMeasurement1.by, 0.0f)
        assertEquals(-enuMeasurement.bz, nedMeasurement1.bz, 0.0f)
        assertNull(nedMeasurement1.hardIronX)
        assertNull(nedMeasurement1.hardIronY)
        assertNull(nedMeasurement1.hardIronZ)
        assertEquals(enuMeasurement.timestamp, nedMeasurement1.timestamp)
        assertNull(nedMeasurement1.accuracy)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            nedMeasurement1.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.NED,
            nedMeasurement1.sensorCoordinateSystem
        )
        assertEquals(nedMeasurement1, nedMeasurement2)
    }

    @Test
    fun toNedOrThrow_whenMagnetometerSensorMeasurementWithAllValues_returnsExpectedValues() {
        val randomizer = UniformRandomizer()
        val bX = randomizer.nextFloat()
        val bY = randomizer.nextFloat()
        val bZ = randomizer.nextFloat()

        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()

        val timestamp = System.nanoTime()

        val enuMeasurement = MagnetometerSensorMeasurement(
            bx = bX,
            by = bY,
            bz = bZ,
            hardIronX = hardIronX,
            hardIronY = hardIronY,
            hardIronZ = hardIronZ,
            timestamp = timestamp,
            accuracy = SensorAccuracy.HIGH,
            sensorType = MagnetometerSensorType.MAGNETOMETER,
            sensorCoordinateSystem = SensorCoordinateSystem.ENU
        )

        val nedMeasurement1 = MagnetometerSensorMeasurement()
        MagnetometerSensorMeasurementCoordinateSystemConverter.toNedOrThrow(
            enuMeasurement,
            nedMeasurement1
        )
        val nedMeasurement2 = MagnetometerSensorMeasurementCoordinateSystemConverter.toNedOrThrow(
            enuMeasurement
        )

        // check
        assertEquals(enuMeasurement.by, nedMeasurement1.bx, 0.0f)
        assertEquals(enuMeasurement.bx, nedMeasurement1.by, 0.0f)
        assertEquals(-enuMeasurement.bz, nedMeasurement1.bz, 0.0f)
        val nedHardIronX = (nedMeasurement1.hardIronX ?: fail()) as Float
        val nedHardIronY = (nedMeasurement1.hardIronY ?: fail()) as Float
        val nedHardIronZ = (nedMeasurement1.hardIronZ ?: fail()) as Float
        assertEquals(hardIronY, nedHardIronX, 0.0f)
        assertEquals(hardIronX, nedHardIronY, 0.0f)
        assertEquals(-hardIronZ, nedHardIronZ, 0.0f)
        assertEquals(enuMeasurement.timestamp, nedMeasurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, nedMeasurement1.accuracy)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER,
            nedMeasurement1.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.NED,
            nedMeasurement1.sensorCoordinateSystem
        )
        assertEquals(nedMeasurement1, nedMeasurement2)
    }

    @Test
    fun toNedOrThrow_whenMagnetometerInvalidCoordinateSystem_throwsIllegalArgumentException() {
        val measurement = MagnetometerSensorMeasurement(
            sensorCoordinateSystem = SensorCoordinateSystem.NED
        )

        assertThrows(IllegalArgumentException::class.java) {
            MagnetometerSensorMeasurementCoordinateSystemConverter.toNedOrThrow(
                measurement,
                MagnetometerSensorMeasurement()
            )
        }
        assertThrows(IllegalArgumentException::class.java) {
            MagnetometerSensorMeasurementCoordinateSystemConverter.toNedOrThrow(measurement)
        }
    }

    @Test
    fun toEnuOrThrow_whenMagnetometerSensorMeasurementWithoutOptionalValues_returnsExpectedValues() {
        val randomizer = UniformRandomizer()
        val bX = randomizer.nextFloat()
        val bY = randomizer.nextFloat()
        val bZ = randomizer.nextFloat()

        val timestamp = System.nanoTime()

        val nedMeasurement = MagnetometerSensorMeasurement(
            bx = bX,
            by = bY,
            bz = bZ,
            timestamp = timestamp,
            sensorType = MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            sensorCoordinateSystem = SensorCoordinateSystem.NED
        )

        val enuMeasurement1 = MagnetometerSensorMeasurement()
        MagnetometerSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(
            nedMeasurement,
            enuMeasurement1
        )
        val enuMeasurement2 = MagnetometerSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(
            nedMeasurement
        )

        // check
        assertEquals(nedMeasurement.by, enuMeasurement1.bx, 0.0f)
        assertEquals(nedMeasurement.bx, enuMeasurement1.by, 0.0f)
        assertEquals(-nedMeasurement.bz, enuMeasurement1.bz, 0.0f)
        assertNull(enuMeasurement1.hardIronX)
        assertNull(enuMeasurement1.hardIronY)
        assertNull(enuMeasurement1.hardIronZ)
        assertEquals(nedMeasurement.timestamp, enuMeasurement1.timestamp)
        assertNull(enuMeasurement1.accuracy)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            enuMeasurement1.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.ENU,
            enuMeasurement1.sensorCoordinateSystem
        )
        assertEquals(enuMeasurement1, enuMeasurement2)
    }

    @Test
    fun toEnuOrThrow_whenMagnetometerSensorMeasurementWithAllValues_returnsExpectedValues() {
        val randomizer = UniformRandomizer()
        val bX = randomizer.nextFloat()
        val bY = randomizer.nextFloat()
        val bZ = randomizer.nextFloat()

        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()

        val timestamp = System.nanoTime()

        val nedMeasurement = MagnetometerSensorMeasurement(
            bx = bX,
            by = bY,
            bz = bZ,
            hardIronX = hardIronX,
            hardIronY = hardIronY,
            hardIronZ = hardIronZ,
            timestamp = timestamp,
            accuracy = SensorAccuracy.HIGH,
            sensorType = MagnetometerSensorType.MAGNETOMETER,
            sensorCoordinateSystem = SensorCoordinateSystem.NED
        )

        val enuMeasurement1 = MagnetometerSensorMeasurement()
        MagnetometerSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(
            nedMeasurement,
            enuMeasurement1
        )
        val enuMeasurement2 = MagnetometerSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(
            nedMeasurement
        )

        // check
        assertEquals(nedMeasurement.by, enuMeasurement1.bx, 0.0f)
        assertEquals(nedMeasurement.bx, enuMeasurement1.by, 0.0f)
        assertEquals(-nedMeasurement.bz, enuMeasurement1.bz, 0.0f)
        val enuHardIronX = (enuMeasurement1.hardIronX ?: fail()) as Float
        val enuHardIronY = (enuMeasurement1.hardIronY ?: fail()) as Float
        val enuHardIronZ = (enuMeasurement1.hardIronZ ?: fail()) as Float
        assertEquals(hardIronY, enuHardIronX, 0.0f)
        assertEquals(hardIronX, enuHardIronY, 0.0f)
        assertEquals(-hardIronZ, enuHardIronZ, 0.0f)
        assertEquals(nedMeasurement.timestamp, enuMeasurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, enuMeasurement1.accuracy)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER,
            enuMeasurement1.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.ENU,
            enuMeasurement1.sensorCoordinateSystem
        )
        assertEquals(enuMeasurement1, enuMeasurement2)
    }

    @Test
    fun toEnuOrThrow_whenInvalidMagnetometerCoordinateSystem_throwsIllegalArgumentException() {
        val measurement = MagnetometerSensorMeasurement(
            sensorCoordinateSystem = SensorCoordinateSystem.ENU
        )

        assertThrows(IllegalArgumentException::class.java) {
            MagnetometerSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(
                measurement,
                MagnetometerSensorMeasurement()
            )
        }
        assertThrows(IllegalArgumentException::class.java) {
            MagnetometerSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(measurement)
        }
    }

    @Test
    fun toNed_whenAlreadyNED_copiesInput() {
        val randomizer = UniformRandomizer()
        val bX = randomizer.nextFloat()
        val bY = randomizer.nextFloat()
        val bZ = randomizer.nextFloat()

        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()

        val timestamp = System.nanoTime()

        val nedMeasurement = MagnetometerSensorMeasurement(
            bx = bX,
            by = bY,
            bz = bZ,
            hardIronX = hardIronX,
            hardIronY = hardIronY,
            hardIronZ = hardIronZ,
            timestamp = timestamp,
            accuracy = SensorAccuracy.HIGH,
            sensorType = MagnetometerSensorType.MAGNETOMETER,
            sensorCoordinateSystem = SensorCoordinateSystem.NED
        )

        val nedMeasurement2 = MagnetometerSensorMeasurement()
        MagnetometerSensorMeasurementCoordinateSystemConverter.toNed(
            nedMeasurement,
            nedMeasurement2
        )
        val nedMeasurement3 =
            MagnetometerSensorMeasurementCoordinateSystemConverter.toNed(nedMeasurement)

        // check
        assertEquals(nedMeasurement, nedMeasurement2)
        assertEquals(nedMeasurement, nedMeasurement3)
    }

    @Test
    fun toNed_whenENU_returnsExpectedValues() {
        val randomizer = UniformRandomizer()
        val bX = randomizer.nextFloat()
        val bY = randomizer.nextFloat()
        val bZ = randomizer.nextFloat()

        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()

        val timestamp = System.nanoTime()

        val enuMeasurement = MagnetometerSensorMeasurement(
            bx = bX,
            by = bY,
            bz = bZ,
            hardIronX = hardIronX,
            hardIronY = hardIronY,
            hardIronZ = hardIronZ,
            timestamp = timestamp,
            accuracy = SensorAccuracy.HIGH,
            sensorType = MagnetometerSensorType.MAGNETOMETER,
            sensorCoordinateSystem = SensorCoordinateSystem.ENU
        )

        val nedMeasurement1 = MagnetometerSensorMeasurement()
        MagnetometerSensorMeasurementCoordinateSystemConverter.toNed(
            enuMeasurement,
            nedMeasurement1
        )
        val nedMeasurement2 =
            MagnetometerSensorMeasurementCoordinateSystemConverter.toNed(enuMeasurement)

        // check
        assertEquals(enuMeasurement.by, nedMeasurement1.bx, 0.0f)
        assertEquals(enuMeasurement.bx, nedMeasurement1.by, 0.0f)
        assertEquals(-enuMeasurement.bz, nedMeasurement1.bz, 0.0f)
        val nedHardIronX = (nedMeasurement1.hardIronX ?: fail()) as Float
        val nedHardIronY = (nedMeasurement1.hardIronY ?: fail()) as Float
        val nedHardIronZ = (nedMeasurement1.hardIronZ ?: fail()) as Float
        assertEquals(hardIronY, nedHardIronX, 0.0f)
        assertEquals(hardIronX, nedHardIronY, 0.0f)
        assertEquals(-hardIronZ, nedHardIronZ, 0.0f)
        assertEquals(enuMeasurement.timestamp, nedMeasurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, nedMeasurement1.accuracy)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER,
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
        val bX = randomizer.nextFloat()
        val bY = randomizer.nextFloat()
        val bZ = randomizer.nextFloat()

        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()

        val timestamp = System.nanoTime()

        val enuMeasurement = MagnetometerSensorMeasurement(
            bx = bX,
            by = bY,
            bz = bZ,
            hardIronX = hardIronX,
            hardIronY = hardIronY,
            hardIronZ = hardIronZ,
            timestamp = timestamp,
            accuracy = SensorAccuracy.HIGH,
            sensorType = MagnetometerSensorType.MAGNETOMETER,
            sensorCoordinateSystem = SensorCoordinateSystem.ENU
        )

        val enuMeasurement2 = MagnetometerSensorMeasurement()
        MagnetometerSensorMeasurementCoordinateSystemConverter.toEnu(
            enuMeasurement,
            enuMeasurement2
        )
        val enuMeasurement3 =
            MagnetometerSensorMeasurementCoordinateSystemConverter.toEnu(enuMeasurement)

        // check
        assertEquals(enuMeasurement, enuMeasurement2)
        assertEquals(enuMeasurement, enuMeasurement3)
    }

    @Test
    fun toEnu_whenNED_returnsExpectedValues() {
        val randomizer = UniformRandomizer()
        val bX = randomizer.nextFloat()
        val bY = randomizer.nextFloat()
        val bZ = randomizer.nextFloat()

        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()

        val timestamp = System.nanoTime()

        val nedMeasurement = MagnetometerSensorMeasurement(
            bx = bX,
            by = bY,
            bz = bZ,
            hardIronX = hardIronX,
            hardIronY = hardIronY,
            hardIronZ = hardIronZ,
            timestamp = timestamp,
            accuracy = SensorAccuracy.HIGH,
            sensorType = MagnetometerSensorType.MAGNETOMETER,
            sensorCoordinateSystem = SensorCoordinateSystem.NED
        )

        val enuMeasurement1 = MagnetometerSensorMeasurement()
        MagnetometerSensorMeasurementCoordinateSystemConverter.toEnu(
            nedMeasurement,
            enuMeasurement1
        )
        val enuMeasurement2 =
            MagnetometerSensorMeasurementCoordinateSystemConverter.toEnu(nedMeasurement)

        // check
        assertEquals(nedMeasurement.by, enuMeasurement1.bx, 0.0f)
        assertEquals(nedMeasurement.bx, enuMeasurement1.by, 0.0f)
        assertEquals(-nedMeasurement.bz, enuMeasurement1.bz, 0.0f)
        val enuHardIronX = (enuMeasurement1.hardIronX ?: fail()) as Float
        val enuHardIronY = (enuMeasurement1.hardIronY ?: fail()) as Float
        val enuHardIronZ = (enuMeasurement1.hardIronZ ?: fail()) as Float
        assertEquals(hardIronY, enuHardIronX, 0.0f)
        assertEquals(hardIronX, enuHardIronY, 0.0f)
        assertEquals(-hardIronZ, enuHardIronZ, 0.0f)
        assertEquals(nedMeasurement.timestamp, enuMeasurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, enuMeasurement1.accuracy)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER,
            enuMeasurement1.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.ENU,
            enuMeasurement1.sensorCoordinateSystem
        )
        assertEquals(enuMeasurement1, enuMeasurement2)
    }
}