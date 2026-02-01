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

import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorCoordinateSystem
import com.irurueta.statistics.UniformRandomizer
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertThrows
import org.junit.Assert.fail
import org.junit.Test

class AccelerometerSensorMeasurementCoordinateSystemConverterTest {

    @Test
    fun toNedOrThrow_whenAccelerometerSensorMeasurementWithoutOptionalValues_returnsExpectedValues() {
        val randomizer = UniformRandomizer()
        val aX = randomizer.nextFloat()
        val aY = randomizer.nextFloat()
        val aZ = randomizer.nextFloat()

        val timestamp = System.nanoTime()

        val enuMeasurement = AccelerometerSensorMeasurement(
            ax = aX,
            ay = aY,
            az = aZ,
            timestamp = timestamp,
            sensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            sensorCoordinateSystem = SensorCoordinateSystem.ENU
        )

        val nedMeasurement1 = AccelerometerSensorMeasurement()
        AccelerometerSensorMeasurementCoordinateSystemConverter.toNedOrThrow(
            enuMeasurement,
            nedMeasurement1
        )
        val nedMeasurement2 = AccelerometerSensorMeasurementCoordinateSystemConverter.toNedOrThrow(
            enuMeasurement
        )

        // check
        assertEquals(enuMeasurement.ay, nedMeasurement1.ax, 0.0f)
        assertEquals(enuMeasurement.ax, nedMeasurement1.ay, 0.0f)
        assertEquals(-enuMeasurement.az, nedMeasurement1.az, 0.0f)
        assertNull(nedMeasurement1.bx)
        assertNull(nedMeasurement1.by)
        assertNull(nedMeasurement1.bz)
        assertEquals(enuMeasurement.timestamp, nedMeasurement1.timestamp)
        assertNull(nedMeasurement1.accuracy)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            nedMeasurement1.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.NED,
            nedMeasurement1.sensorCoordinateSystem
        )
        assertEquals(nedMeasurement1, nedMeasurement2)
    }

    @Test
    fun toNedOrThrow_whenAccelerometerSensorMeasurementWithAllValues_returnsExpectedValues() {
        val randomizer = UniformRandomizer()
        val aX = randomizer.nextFloat()
        val aY = randomizer.nextFloat()
        val aZ = randomizer.nextFloat()

        val bX = randomizer.nextFloat()
        val bY = randomizer.nextFloat()
        val bZ = randomizer.nextFloat()

        val timestamp = System.nanoTime()

        val enuMeasurement = AccelerometerSensorMeasurement(
            ax = aX,
            ay = aY,
            az = aZ,
            bx = bX,
            by = bY,
            bz = bZ,
            timestamp = timestamp,
            accuracy = SensorAccuracy.HIGH,
            sensorType = AccelerometerSensorType.ACCELEROMETER,
            sensorCoordinateSystem = SensorCoordinateSystem.ENU
        )

        val nedMeasurement1 = AccelerometerSensorMeasurement()
        AccelerometerSensorMeasurementCoordinateSystemConverter.toNedOrThrow(
            enuMeasurement,
            nedMeasurement1
        )
        val nedMeasurement2 = AccelerometerSensorMeasurementCoordinateSystemConverter.toNedOrThrow(
            enuMeasurement
        )

        // check
        assertEquals(enuMeasurement.ay, nedMeasurement1.ax, 0.0f)
        assertEquals(enuMeasurement.ax, nedMeasurement1.ay, 0.0f)
        assertEquals(-enuMeasurement.az, nedMeasurement1.az, 0.0f)
        val nedBx = (nedMeasurement1.bx ?: fail()) as Float
        val nedBy = (nedMeasurement1.by ?: fail()) as Float
        val nedBz = (nedMeasurement1.bz ?: fail()) as Float
        assertEquals(bY, nedBx, 0.0f)
        assertEquals(bX, nedBy, 0.0f)
        assertEquals(-bZ, nedBz, 0.0f)
        assertEquals(enuMeasurement.timestamp, nedMeasurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, nedMeasurement1.accuracy)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            nedMeasurement1.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.NED,
            nedMeasurement1.sensorCoordinateSystem
        )
        assertEquals(nedMeasurement1, nedMeasurement2)
    }

    @Test
    fun toNedOrThrow_whenAccelerometerInvalidCoordinateSystem_throwsIllegalArgumentException() {
        val measurement = AccelerometerSensorMeasurement(
            sensorCoordinateSystem = SensorCoordinateSystem.NED
        )

        assertThrows(IllegalArgumentException::class.java) {
            AccelerometerSensorMeasurementCoordinateSystemConverter.toNedOrThrow(
                measurement,
                AccelerometerSensorMeasurement()
            )
        }
        assertThrows(IllegalArgumentException::class.java) {
            AccelerometerSensorMeasurementCoordinateSystemConverter.toNedOrThrow(measurement)
        }
    }

    @Test
    fun toEnuOrThrow_whenAccelerometerSensorMeasurementWithoutOptionalValues_returnsExpectedValues() {
        val randomizer = UniformRandomizer()
        val aX = randomizer.nextFloat()
        val aY = randomizer.nextFloat()
        val aZ = randomizer.nextFloat()

        val timestamp = System.nanoTime()

        val nedMeasurement = AccelerometerSensorMeasurement(
            ax = aX,
            ay = aY,
            az = aZ,
            timestamp = timestamp,
            sensorType = AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            sensorCoordinateSystem = SensorCoordinateSystem.NED
        )

        val enuMeasurement1 = AccelerometerSensorMeasurement()
        AccelerometerSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(
            nedMeasurement,
            enuMeasurement1
        )
        val enuMeasurement2 = AccelerometerSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(
            nedMeasurement
        )

        // check
        assertEquals(nedMeasurement.ay, enuMeasurement1.ax, 0.0f)
        assertEquals(nedMeasurement.ax, enuMeasurement1.ay, 0.0f)
        assertEquals(-nedMeasurement.az, enuMeasurement1.az, 0.0f)
        assertNull(enuMeasurement1.bx)
        assertNull(enuMeasurement1.by)
        assertNull(enuMeasurement1.bz)
        assertEquals(nedMeasurement.timestamp, enuMeasurement1.timestamp)
        assertNull(enuMeasurement1.accuracy)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            enuMeasurement1.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.ENU,
            enuMeasurement1.sensorCoordinateSystem
        )
        assertEquals(enuMeasurement1, enuMeasurement2)
    }

    @Test
    fun toEnuOrThrow_whenAccelerometerSensorMeasurementWithAllValues_returnsExpectedValues() {
        val randomizer = UniformRandomizer()
        val aX = randomizer.nextFloat()
        val aY = randomizer.nextFloat()
        val aZ = randomizer.nextFloat()

        val bX = randomizer.nextFloat()
        val bY = randomizer.nextFloat()
        val bZ = randomizer.nextFloat()

        val timestamp = System.nanoTime()

        val nedMeasurement = AccelerometerSensorMeasurement(
            ax = aX,
            ay = aY,
            az = aZ,
            bx = bX,
            by = bY,
            bz = bZ,
            timestamp = timestamp,
            accuracy = SensorAccuracy.HIGH,
            sensorType = AccelerometerSensorType.ACCELEROMETER,
            sensorCoordinateSystem = SensorCoordinateSystem.NED
        )

        val enuMeasurement1 = AccelerometerSensorMeasurement()
        AccelerometerSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(
            nedMeasurement,
            enuMeasurement1
        )
        val enuMeasurement2 = AccelerometerSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(
            nedMeasurement
        )

        // check
        assertEquals(nedMeasurement.ay, enuMeasurement1.ax, 0.0f)
        assertEquals(nedMeasurement.ax, enuMeasurement1.ay, 0.0f)
        assertEquals(-nedMeasurement.az, enuMeasurement1.az, 0.0f)
        val enuBx = (enuMeasurement1.bx ?: fail()) as Float
        val enuBy = (enuMeasurement1.by ?: fail()) as Float
        val enuBz = (enuMeasurement1.bz ?: fail()) as Float
        assertEquals(bY, enuBx, 0.0f)
        assertEquals(bX, enuBy, 0.0f)
        assertEquals(-bZ, enuBz, 0.0f)
        assertEquals(nedMeasurement.timestamp, enuMeasurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, enuMeasurement1.accuracy)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            enuMeasurement1.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.ENU,
            enuMeasurement1.sensorCoordinateSystem
        )
        assertEquals(enuMeasurement1, enuMeasurement2)
    }

    @Test
    fun toEnuOrThrow_whenInvalidAccelerometerCoordinateSystem_throwsIllegalArgumentException() {
        val measurement = AccelerometerSensorMeasurement(
            sensorCoordinateSystem = SensorCoordinateSystem.ENU
        )

        assertThrows(IllegalArgumentException::class.java) {
            AccelerometerSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(
                measurement,
                AccelerometerSensorMeasurement()
            )
        }
        assertThrows(IllegalArgumentException::class.java) {
            AccelerometerSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(measurement)
        }
    }

    @Test
    fun toNed_whenAlreadyNED_copiesInput() {
        val randomizer = UniformRandomizer()
        val aX = randomizer.nextFloat()
        val aY = randomizer.nextFloat()
        val aZ = randomizer.nextFloat()

        val bX = randomizer.nextFloat()
        val bY = randomizer.nextFloat()
        val bZ = randomizer.nextFloat()

        val timestamp = System.nanoTime()

        val nedMeasurement1 = AccelerometerSensorMeasurement(
            ax = aX,
            ay = aY,
            az = aZ,
            bx = bX,
            by = bY,
            bz = bZ,
            timestamp = timestamp,
            accuracy = SensorAccuracy.HIGH,
            sensorType = AccelerometerSensorType.ACCELEROMETER,
            sensorCoordinateSystem = SensorCoordinateSystem.NED
        )

        val nedMeasurement2 = AccelerometerSensorMeasurement()
        AccelerometerSensorMeasurementCoordinateSystemConverter.toNed(
            nedMeasurement1,
            nedMeasurement2
        )
        val nedMeasurement3 = AccelerometerSensorMeasurementCoordinateSystemConverter.toNed(
            nedMeasurement1
        )

        // check
        assertEquals(nedMeasurement1, nedMeasurement2)
        assertEquals(nedMeasurement1, nedMeasurement3)
    }

    @Test
    fun toNed_whenENU_returnsExpectedValues() {
        val randomizer = UniformRandomizer()
        val aX = randomizer.nextFloat()
        val aY = randomizer.nextFloat()
        val aZ = randomizer.nextFloat()

        val bX = randomizer.nextFloat()
        val bY = randomizer.nextFloat()
        val bZ = randomizer.nextFloat()

        val timestamp = System.nanoTime()

        val enuMeasurement = AccelerometerSensorMeasurement(
            ax = aX,
            ay = aY,
            az = aZ,
            bx = bX,
            by = bY,
            bz = bZ,
            timestamp = timestamp,
            accuracy = SensorAccuracy.HIGH,
            sensorType = AccelerometerSensorType.ACCELEROMETER,
            sensorCoordinateSystem = SensorCoordinateSystem.ENU
        )

        val nedMeasurement1 = AccelerometerSensorMeasurement()
        AccelerometerSensorMeasurementCoordinateSystemConverter.toNed(
            enuMeasurement,
            nedMeasurement1
        )
        val nedMeasurement2 = AccelerometerSensorMeasurementCoordinateSystemConverter.toNed(
            enuMeasurement
        )

        // check
        assertEquals(enuMeasurement.ay, nedMeasurement1.ax, 0.0f)
        assertEquals(enuMeasurement.ax, nedMeasurement1.ay, 0.0f)
        assertEquals(-enuMeasurement.az, nedMeasurement1.az, 0.0f)
        val nedBx = (nedMeasurement1.bx ?: fail()) as Float
        val nedBy = (nedMeasurement1.by ?: fail()) as Float
        val nedBz = (nedMeasurement1.bz ?: fail()) as Float
        assertEquals(bY, nedBx, 0.0f)
        assertEquals(bX, nedBy, 0.0f)
        assertEquals(-bZ, nedBz, 0.0f)
        assertEquals(enuMeasurement.timestamp, nedMeasurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, nedMeasurement1.accuracy)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
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
        val aX = randomizer.nextFloat()
        val aY = randomizer.nextFloat()
        val aZ = randomizer.nextFloat()

        val bX = randomizer.nextFloat()
        val bY = randomizer.nextFloat()
        val bZ = randomizer.nextFloat()

        val timestamp = System.nanoTime()

        val enuMeasurement1 = AccelerometerSensorMeasurement(
            ax = aX,
            ay = aY,
            az = aZ,
            bx = bX,
            by = bY,
            bz = bZ,
            timestamp = timestamp,
            accuracy = SensorAccuracy.HIGH,
            sensorType = AccelerometerSensorType.ACCELEROMETER,
            sensorCoordinateSystem = SensorCoordinateSystem.ENU
        )

        val enuMeasurement2 = AccelerometerSensorMeasurement()
        AccelerometerSensorMeasurementCoordinateSystemConverter.toEnu(
            enuMeasurement1,
            enuMeasurement2
        )
        val enuMeasurement3 = AccelerometerSensorMeasurementCoordinateSystemConverter.toEnu(
            enuMeasurement1
        )

        // check
        assertEquals(enuMeasurement1, enuMeasurement2)
        assertEquals(enuMeasurement1, enuMeasurement3)
    }

    @Test
    fun toEnu_whenNED_returnsExpectedValues() {
        val randomizer = UniformRandomizer()
        val aX = randomizer.nextFloat()
        val aY = randomizer.nextFloat()
        val aZ = randomizer.nextFloat()

        val bX = randomizer.nextFloat()
        val bY = randomizer.nextFloat()
        val bZ = randomizer.nextFloat()

        val timestamp = System.nanoTime()

        val nedMeasurement = AccelerometerSensorMeasurement(
            ax = aX,
            ay = aY,
            az = aZ,
            bx = bX,
            by = bY,
            bz = bZ,
            timestamp = timestamp,
            accuracy = SensorAccuracy.HIGH,
            sensorType = AccelerometerSensorType.ACCELEROMETER,
            sensorCoordinateSystem = SensorCoordinateSystem.NED
        )

        val enuMeasurement1 = AccelerometerSensorMeasurement()
        AccelerometerSensorMeasurementCoordinateSystemConverter.toEnu(
            nedMeasurement,
            enuMeasurement1
        )
        val enuMeasurement2 = AccelerometerSensorMeasurementCoordinateSystemConverter.toEnu(
            nedMeasurement
        )

        // check
        assertEquals(nedMeasurement.ay, enuMeasurement1.ax, 0.0f)
        assertEquals(nedMeasurement.ax, enuMeasurement1.ay, 0.0f)
        assertEquals(-nedMeasurement.az, enuMeasurement1.az, 0.0f)
        val nedBx = (enuMeasurement1.bx ?: fail()) as Float
        val nedBy = (enuMeasurement1.by ?: fail()) as Float
        val nedBz = (enuMeasurement1.bz ?: fail()) as Float
        assertEquals(bY, nedBx, 0.0f)
        assertEquals(bX, nedBy, 0.0f)
        assertEquals(-bZ, nedBz, 0.0f)
        assertEquals(nedMeasurement.timestamp, enuMeasurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, enuMeasurement1.accuracy)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            enuMeasurement1.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.ENU,
            enuMeasurement1.sensorCoordinateSystem
        )
        assertEquals(enuMeasurement1, enuMeasurement2)
    }
}