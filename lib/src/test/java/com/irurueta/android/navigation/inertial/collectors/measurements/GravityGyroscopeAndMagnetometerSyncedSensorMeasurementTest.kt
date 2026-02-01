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

package com.irurueta.android.navigation.inertial.collectors.measurements

import com.irurueta.statistics.UniformRandomizer
import org.junit.Assert.*
import org.junit.Test

class GravityGyroscopeAndMagnetometerSyncedSensorMeasurementTest {

    @Test
    fun constructor_whenDefault_setsExpectedValues() {
        val syncedMeasurement = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement()

        // check
        assertNull(syncedMeasurement.gravityMeasurement)
        assertNull(syncedMeasurement.gyroscopeMeasurement)
        assertNull(syncedMeasurement.magnetometerMeasurement)
        assertEquals(0L, syncedMeasurement.timestamp)
    }

    @Test
    fun constructor_whenAllParameters_setsExpectedValues() {
        val gravityMeasurement = GravitySensorMeasurement()
        val gyroscopeMeasurement = GyroscopeSensorMeasurement()
        val magnetometerMeasurement = MagnetometerSensorMeasurement()
        val timestamp = System.nanoTime()
        val syncedMeasurement = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            gravityMeasurement,
            gyroscopeMeasurement,
            magnetometerMeasurement,
            timestamp
        )

        // check
        assertSame(gravityMeasurement, syncedMeasurement.gravityMeasurement)
        assertSame(gyroscopeMeasurement, syncedMeasurement.gyroscopeMeasurement)
        assertSame(magnetometerMeasurement, syncedMeasurement.magnetometerMeasurement)
        assertEquals(timestamp, syncedMeasurement.timestamp)
    }

    @Test
    fun constructor_whenCopy_setsExpectedValues() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val gravityMeasurement = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            timestamp,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.NED
        )
        val gyroscopeMeasurement = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            wbx,
            wby,
            wbz,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
        val magnetometerMeasurement = MagnetometerSensorMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            timestamp,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )

        val syncedMeasurement1 = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            gravityMeasurement,
            gyroscopeMeasurement,
            magnetometerMeasurement,
            timestamp
        )

        val syncedMeasurement2 = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            syncedMeasurement1
        )

        // check
        assertEquals(gravityMeasurement, syncedMeasurement2.gravityMeasurement)
        assertNotSame(gravityMeasurement, syncedMeasurement2.gravityMeasurement)
        assertEquals(gyroscopeMeasurement, syncedMeasurement2.gyroscopeMeasurement)
        assertNotSame(gyroscopeMeasurement, syncedMeasurement2.gyroscopeMeasurement)
        assertEquals(magnetometerMeasurement, syncedMeasurement2.magnetometerMeasurement)
        assertNotSame(magnetometerMeasurement, syncedMeasurement2.magnetometerMeasurement)
        assertEquals(timestamp, syncedMeasurement2.timestamp)
    }

    @Test
    fun gravityMeasurement_setsExpectedValue() {
        val syncedMeasurement = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement()

        // check default value
        assertNull(syncedMeasurement.gravityMeasurement)

        // set new value
        val gravityMeasurement = GravitySensorMeasurement()
        syncedMeasurement.gravityMeasurement = gravityMeasurement
    }

    @Test
    fun gyroscopeMeasurement_setsExpectedValue() {
        val syncedMeasurement = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement()

        // check default value
        assertNull(syncedMeasurement.gyroscopeMeasurement)

        // set new value
        val gyroscopeMeasurement = GyroscopeSensorMeasurement()
        syncedMeasurement.gyroscopeMeasurement = gyroscopeMeasurement
    }

    @Test
    fun magnetometerMeasurement_setsExpectedValue() {
        val syncedMeasurement = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement()

        // check default value
        assertNull(syncedMeasurement.magnetometerMeasurement)

        // set new value
        val magnetometerMeasurement = MagnetometerSensorMeasurement()
        syncedMeasurement.magnetometerMeasurement = magnetometerMeasurement
    }

    @Test
    fun timestamp_setsExpectedValue() {
        val syncedMeasurement = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement()

        // check default value
        assertEquals(0L, syncedMeasurement.timestamp)

        // set new value
        val timestamp = System.nanoTime()
        syncedMeasurement.timestamp = timestamp

        // check
        assertEquals(timestamp, syncedMeasurement.timestamp)
    }

    @Test
    fun copyFrom_whenAllData_makesExpectedCopy() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val gravityMeasurement = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            timestamp,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.NED
        )
        val gyroscopeMeasurement = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            wbx,
            wby,
            wbz,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
        val magnetometerMeasurement = MagnetometerSensorMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            timestamp,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )

        val syncedMeasurement1 = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            gravityMeasurement,
            gyroscopeMeasurement,
            magnetometerMeasurement,
            timestamp
        )

        val syncedMeasurement2 = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement()
        syncedMeasurement2.copyFrom(syncedMeasurement1)

        // check
        assertEquals(gravityMeasurement, syncedMeasurement2.gravityMeasurement)
        assertNotSame(gravityMeasurement, syncedMeasurement2.gravityMeasurement)
        assertEquals(gyroscopeMeasurement, syncedMeasurement2.gyroscopeMeasurement)
        assertNotSame(gyroscopeMeasurement, syncedMeasurement2.gyroscopeMeasurement)
        assertEquals(magnetometerMeasurement, syncedMeasurement2.magnetometerMeasurement)
        assertNotSame(magnetometerMeasurement, syncedMeasurement2.magnetometerMeasurement)
        assertEquals(timestamp, syncedMeasurement2.timestamp)
    }

    @Test
    fun copyFrom_whenMissingMeasurements_makesExpectedCopy() {
        val timestamp = System.nanoTime()

        val syncedMeasurement1 = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            timestamp = timestamp
        )

        val syncedMeasurement2 = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement()
        syncedMeasurement2.copyFrom(syncedMeasurement1)

        // check
        assertNull(syncedMeasurement2.gravityMeasurement)
        assertNull(syncedMeasurement2.gyroscopeMeasurement)
        assertNull(syncedMeasurement2.magnetometerMeasurement)
        assertEquals(timestamp, syncedMeasurement2.timestamp)
    }

    @Test
    fun copyTo_makesExpectedCopy() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val gravityMeasurement = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            timestamp,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.NED
        )
        val gyroscopeMeasurement = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            wbx,
            wby,
            wbz,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
        val magnetometerMeasurement = MagnetometerSensorMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            timestamp,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )

        val syncedMeasurement1 = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            gravityMeasurement,
            gyroscopeMeasurement,
            magnetometerMeasurement,
            timestamp
        )

        val syncedMeasurement2 = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement()
        syncedMeasurement1.copyTo(syncedMeasurement2)

        // check
        assertEquals(gravityMeasurement, syncedMeasurement2.gravityMeasurement)
        assertNotSame(gravityMeasurement, syncedMeasurement2.gravityMeasurement)
        assertEquals(gyroscopeMeasurement, syncedMeasurement2.gyroscopeMeasurement)
        assertNotSame(gyroscopeMeasurement, syncedMeasurement2.gyroscopeMeasurement)
        assertEquals(magnetometerMeasurement, syncedMeasurement2.magnetometerMeasurement)
        assertNotSame(magnetometerMeasurement, syncedMeasurement2.magnetometerMeasurement)
        assertEquals(timestamp, syncedMeasurement2.timestamp)
    }

    @Test
    fun copy_makesExpectedCopy() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val gravityMeasurement = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            timestamp,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.NED
        )
        val gyroscopeMeasurement = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            wbx,
            wby,
            wbz,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
        val magnetometerMeasurement = MagnetometerSensorMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            timestamp,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )

        val syncedMeasurement1 = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            gravityMeasurement,
            gyroscopeMeasurement,
            magnetometerMeasurement,
            timestamp
        )

        val syncedMeasurement2 = syncedMeasurement1.copy()

        // check
        assertEquals(gravityMeasurement, syncedMeasurement2.gravityMeasurement)
        assertNotSame(gravityMeasurement, syncedMeasurement2.gravityMeasurement)
        assertEquals(gyroscopeMeasurement, syncedMeasurement2.gyroscopeMeasurement)
        assertNotSame(gyroscopeMeasurement, syncedMeasurement2.gyroscopeMeasurement)
        assertEquals(magnetometerMeasurement, syncedMeasurement2.magnetometerMeasurement)
        assertNotSame(magnetometerMeasurement, syncedMeasurement2.magnetometerMeasurement)
        assertEquals(timestamp, syncedMeasurement2.timestamp)
    }

    @Test
    fun toNedOrThrow_whenAllMeasurementsAreENU_returnsExpectedValue() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val enuGravityMeasurement = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            timestamp,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.ENU
        )
        val enuGyroscopeMeasurement = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            wbx,
            wby,
            wbz,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )
        val enuMagnetometerMeasurement = MagnetometerSensorMeasurement(
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

        val enuSyncedMeasurement = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            enuGravityMeasurement,
            enuGyroscopeMeasurement,
            enuMagnetometerMeasurement,
            timestamp
        )

        val nedSyncedMeasurement1 = enuSyncedMeasurement.toNedOrThrow()
        val nedSyncedMeasurement2 = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            GravitySensorMeasurement(),
            GyroscopeSensorMeasurement(),
            MagnetometerSensorMeasurement(),
            timestamp
        )
        enuSyncedMeasurement.toNedOrThrow(nedSyncedMeasurement2)

        // check
        val nedGravityMeasurement = enuGravityMeasurement.toNedOrThrow()
        val nedGyroscopeMeasurement = enuGyroscopeMeasurement.toNedOrThrow()
        val nedMagnetometerMeasurement = enuMagnetometerMeasurement.toNedOrThrow()

        assertEquals(nedSyncedMeasurement1, nedSyncedMeasurement2)
        assertEquals(nedGravityMeasurement, nedSyncedMeasurement2.gravityMeasurement)
        assertEquals(nedGyroscopeMeasurement, nedSyncedMeasurement2.gyroscopeMeasurement)
        assertEquals(nedMagnetometerMeasurement, nedSyncedMeasurement2.magnetometerMeasurement)
        assertEquals(timestamp, nedSyncedMeasurement2.timestamp)
    }

    @Test
    fun toNedOrThrow_whenNoMeasurements_returnsExpectedValue() {
        val timestamp = System.nanoTime()

        val enuSyncedMeasurement = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            timestamp = timestamp
        )

        val nedSyncedMeasurement1 = enuSyncedMeasurement.toNedOrThrow()
        val nedSyncedMeasurement2 = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            GravitySensorMeasurement(),
            GyroscopeSensorMeasurement(),
            MagnetometerSensorMeasurement(),
            timestamp
        )
        enuSyncedMeasurement.toNedOrThrow(nedSyncedMeasurement2)

        // check
        assertEquals(nedSyncedMeasurement1, nedSyncedMeasurement2)
        assertNull(nedSyncedMeasurement2.gravityMeasurement)
        assertNull(nedSyncedMeasurement2.gyroscopeMeasurement)
        assertNull(nedSyncedMeasurement2.magnetometerMeasurement)
        assertEquals(timestamp, nedSyncedMeasurement2.timestamp)
    }

    @Test
    fun toNedOrThrow_whenNEDGravityMeasurement_throwsIllegalArgumentException() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val wrongGravityMeasurement = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            timestamp,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.NED
        )
        val enuGyroscopeMeasurement = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            wbx,
            wby,
            wbz,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )
        val enuMagnetometerMeasurement = MagnetometerSensorMeasurement(
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

        val wrongSyncedMeasurement = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            wrongGravityMeasurement,
            enuGyroscopeMeasurement,
            enuMagnetometerMeasurement,
            timestamp
        )

        assertThrows(IllegalArgumentException::class.java) {
            wrongSyncedMeasurement.toNedOrThrow(wrongSyncedMeasurement)
        }
        assertThrows(IllegalArgumentException::class.java) {
            wrongSyncedMeasurement.toNedOrThrow()
        }
    }

    @Test
    fun toNedOrThrow_whenNEDGyroscopeMeasurement_throwsIllegalArgumentException() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val enuGravityMeasurement = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            timestamp,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.ENU
        )
        val wrongGyroscopeMeasurement = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            wbx,
            wby,
            wbz,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
        val enuMagnetometerMeasurement = MagnetometerSensorMeasurement(
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

        val wrongSyncedMeasurement = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            enuGravityMeasurement,
            wrongGyroscopeMeasurement,
            enuMagnetometerMeasurement,
            timestamp
        )

        assertThrows(IllegalArgumentException::class.java) {
            wrongSyncedMeasurement.toNedOrThrow(wrongSyncedMeasurement)
        }
        assertThrows(IllegalArgumentException::class.java) {
            wrongSyncedMeasurement.toNedOrThrow()
        }
    }

    @Test
    fun toNedOrThrow_whenNEDMagnetometerMeasurement_throwsIllegalArgumentException() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val enuGravityMeasurement = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            timestamp,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.ENU
        )
        val enuGyroscopeMeasurement = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            wbx,
            wby,
            wbz,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )
        val wrongMagnetometerMeasurement = MagnetometerSensorMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            timestamp,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )

        val wrongSyncedMeasurement = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            enuGravityMeasurement,
            enuGyroscopeMeasurement,
            wrongMagnetometerMeasurement,
            timestamp
        )

        assertThrows(IllegalArgumentException::class.java) {
            wrongSyncedMeasurement.toNedOrThrow(wrongSyncedMeasurement)
        }
        assertThrows(IllegalArgumentException::class.java) {
            wrongSyncedMeasurement.toNedOrThrow()
        }
    }

    @Test
    fun toEnuOrThrow_whenAllMeasurementsAreNED_returnsExpectedValue() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val nedGravityMeasurement = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            timestamp,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.NED
        )
        val nedGyroscopeMeasurement = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            wbx,
            wby,
            wbz,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
        val nedMagnetometerMeasurement = MagnetometerSensorMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            timestamp,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )

        val nedSyncedMeasurement = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            nedGravityMeasurement,
            nedGyroscopeMeasurement,
            nedMagnetometerMeasurement,
            timestamp
        )

        val enuSyncedMeasurement1 = nedSyncedMeasurement.toEnuOrThrow()
        val enuSyncedMeasurement2 = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            GravitySensorMeasurement(),
            GyroscopeSensorMeasurement(),
            MagnetometerSensorMeasurement(),
            timestamp
        )
        nedSyncedMeasurement.toEnuOrThrow(enuSyncedMeasurement2)

        // check
        val enuGravityMeasurement = nedGravityMeasurement.toEnuOrThrow()
        val enuGyroscopeMeasurement = nedGyroscopeMeasurement.toEnuOrThrow()
        val enuMagnetometerMeasurement = nedMagnetometerMeasurement.toEnuOrThrow()

        assertEquals(enuSyncedMeasurement1, enuSyncedMeasurement2)
        assertEquals(enuGravityMeasurement, enuSyncedMeasurement2.gravityMeasurement)
        assertEquals(enuGyroscopeMeasurement, enuSyncedMeasurement2.gyroscopeMeasurement)
        assertEquals(enuMagnetometerMeasurement, enuSyncedMeasurement2.magnetometerMeasurement)
        assertEquals(timestamp, enuSyncedMeasurement2.timestamp)
    }

    @Test
    fun toEnuOrThrow_whenNoMeasurements_returnsExpectedValue() {
        val timestamp = System.nanoTime()

        val nedSyncedMeasurement = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            timestamp = timestamp
        )

        val enuSyncedMeasurement1 = nedSyncedMeasurement.toEnuOrThrow()
        val enuSyncedMeasurement2 = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            GravitySensorMeasurement(),
            GyroscopeSensorMeasurement(),
            MagnetometerSensorMeasurement(),
            timestamp
        )
        nedSyncedMeasurement.toEnuOrThrow(enuSyncedMeasurement2)

        // check
        assertEquals(enuSyncedMeasurement1, enuSyncedMeasurement2)
        assertNull(enuSyncedMeasurement2.gravityMeasurement)
        assertNull(enuSyncedMeasurement2.gyroscopeMeasurement)
        assertNull(enuSyncedMeasurement2.magnetometerMeasurement)
        assertEquals(timestamp, enuSyncedMeasurement2.timestamp)
    }

    @Test
    fun toEnuOrThrow_whenENUGravityMeasurement_throwsIllegalArgumentException() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val wrongGravityMeasurement = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            timestamp,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.ENU
        )
        val nedGyroscopeMeasurement = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            wbx,
            wby,
            wbz,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
        val nedMagnetometerMeasurement = MagnetometerSensorMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            timestamp,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )

        val wrongSyncedMeasurement = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            wrongGravityMeasurement,
            nedGyroscopeMeasurement,
            nedMagnetometerMeasurement,
            timestamp
        )

        assertThrows(IllegalArgumentException::class.java) {
            wrongSyncedMeasurement.toNedOrThrow(wrongSyncedMeasurement)
        }
        assertThrows(IllegalArgumentException::class.java) {
            wrongSyncedMeasurement.toNedOrThrow()
        }
    }

    @Test
    fun toEnuOrThrow_whenENUGyroscopeMeasurement_throwsIllegalArgumentException() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val nedGravityMeasurement = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            timestamp,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.NED
        )
        val wrongGyroscopeMeasurement = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            wbx,
            wby,
            wbz,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )
        val nedMagnetometerMeasurement = MagnetometerSensorMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            timestamp,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )

        val wrongSyncedMeasurement = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            nedGravityMeasurement,
            wrongGyroscopeMeasurement,
            nedMagnetometerMeasurement,
            timestamp
        )

        assertThrows(IllegalArgumentException::class.java) {
            wrongSyncedMeasurement.toNedOrThrow(wrongSyncedMeasurement)
        }
        assertThrows(IllegalArgumentException::class.java) {
            wrongSyncedMeasurement.toNedOrThrow()
        }
    }

    @Test
    fun toEnuOrThrow_whenENUMagnetometerMeasurement_throwsIllegalArgumentException() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val nedGravityMeasurement = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            timestamp,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.NED
        )
        val nedGyroscopeMeasurement = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            wbx,
            wby,
            wbz,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
        val wrongMagnetometerMeasurement = MagnetometerSensorMeasurement(
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

        val wrongSyncedMeasurement = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            nedGravityMeasurement,
            nedGyroscopeMeasurement,
            wrongMagnetometerMeasurement,
            timestamp
        )

        assertThrows(IllegalArgumentException::class.java) {
            wrongSyncedMeasurement.toNedOrThrow(wrongSyncedMeasurement)
        }
        assertThrows(IllegalArgumentException::class.java) {
            wrongSyncedMeasurement.toNedOrThrow()
        }
    }

    @Test
    fun toNed_whenAllMeasurementsAreENU_returnsExpectedValue() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val enuGravityMeasurement = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            timestamp,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.ENU
        )
        val enuGyroscopeMeasurement = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            wbx,
            wby,
            wbz,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )
        val enuMagnetometerMeasurement = MagnetometerSensorMeasurement(
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

        val enuSyncedMeasurement = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            enuGravityMeasurement,
            enuGyroscopeMeasurement,
            enuMagnetometerMeasurement,
            timestamp
        )

        val nedSyncedMeasurement1 = enuSyncedMeasurement.toNed()
        val nedSyncedMeasurement2 = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            GravitySensorMeasurement(),
            GyroscopeSensorMeasurement(),
            MagnetometerSensorMeasurement(),
            timestamp
        )
        enuSyncedMeasurement.toNed(nedSyncedMeasurement2)

        // check
        val nedGravityMeasurement = enuGravityMeasurement.toNed()
        val nedGyroscopeMeasurement = enuGyroscopeMeasurement.toNed()
        val nedMagnetometerMeasurement = enuMagnetometerMeasurement.toNed()

        assertEquals(nedSyncedMeasurement1, nedSyncedMeasurement2)
        assertEquals(nedGravityMeasurement, nedSyncedMeasurement2.gravityMeasurement)
        assertEquals(nedGyroscopeMeasurement, nedSyncedMeasurement2.gyroscopeMeasurement)
        assertEquals(nedMagnetometerMeasurement, nedSyncedMeasurement2.magnetometerMeasurement)
        assertEquals(timestamp, nedSyncedMeasurement2.timestamp)
    }

    @Test
    fun toNed_whenNoMeasurements_returnsExpectedValue() {
        val timestamp = System.nanoTime()

        val enuSyncedMeasurement = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            timestamp = timestamp
        )

        val nedSyncedMeasurement1 = enuSyncedMeasurement.toNed()
        val nedSyncedMeasurement2 = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            GravitySensorMeasurement(),
            GyroscopeSensorMeasurement(),
            MagnetometerSensorMeasurement(),
            timestamp
        )
        enuSyncedMeasurement.toNed(nedSyncedMeasurement2)

        // check
        assertEquals(nedSyncedMeasurement1, nedSyncedMeasurement2)
        assertNull(nedSyncedMeasurement2.gravityMeasurement)
        assertNull(nedSyncedMeasurement2.gyroscopeMeasurement)
        assertNull(nedSyncedMeasurement2.magnetometerMeasurement)
        assertEquals(timestamp, nedSyncedMeasurement2.timestamp)
    }

    @Test
    fun toEnu_whenAllMeasurementsAreNED_returnsExpectedValue() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val nedGravityMeasurement = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            timestamp,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.NED
        )
        val nedGyroscopeMeasurement = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            wbx,
            wby,
            wbz,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
        val nedMagnetometerMeasurement = MagnetometerSensorMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            timestamp,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )

        val nedSyncedMeasurement = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            nedGravityMeasurement,
            nedGyroscopeMeasurement,
            nedMagnetometerMeasurement,
            timestamp
        )

        val enuSyncedMeasurement1 = nedSyncedMeasurement.toEnu()
        val enuSyncedMeasurement2 = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            GravitySensorMeasurement(),
            GyroscopeSensorMeasurement(),
            MagnetometerSensorMeasurement(),
            timestamp
        )
        nedSyncedMeasurement.toEnu(enuSyncedMeasurement2)

        // check
        val enuGravityMeasurement = nedGravityMeasurement.toEnu()
        val enuGyroscopeMeasurement = nedGyroscopeMeasurement.toEnu()
        val enuMagnetometerMeasurement = nedMagnetometerMeasurement.toEnu()

        assertEquals(enuSyncedMeasurement1, enuSyncedMeasurement2)
        assertEquals(enuGravityMeasurement, enuSyncedMeasurement2.gravityMeasurement)
        assertEquals(enuGyroscopeMeasurement, enuSyncedMeasurement2.gyroscopeMeasurement)
        assertEquals(enuMagnetometerMeasurement, enuSyncedMeasurement2.magnetometerMeasurement)
        assertEquals(timestamp, enuSyncedMeasurement2.timestamp)
    }

    @Test
    fun toEnu_whenNoMeasurements_returnsExpectedValue() {
        val timestamp = System.nanoTime()

        val nedSyncedMeasurement = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            timestamp = timestamp
        )

        val enuSyncedMeasurement1 = nedSyncedMeasurement.toEnu()
        val enuSyncedMeasurement2 = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            GravitySensorMeasurement(),
            GyroscopeSensorMeasurement(),
            MagnetometerSensorMeasurement(),
            timestamp
        )
        nedSyncedMeasurement.toEnu(enuSyncedMeasurement2)

        // check
        assertEquals(enuSyncedMeasurement1, enuSyncedMeasurement2)
        assertNull(enuSyncedMeasurement2.gravityMeasurement)
        assertNull(enuSyncedMeasurement2.gyroscopeMeasurement)
        assertNull(enuSyncedMeasurement2.magnetometerMeasurement)
        assertEquals(timestamp, enuSyncedMeasurement2.timestamp)
    }

    @Test
    fun equals_whenNull_returnsFalse() {
        val syncedMeasurement = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement()
        assertFalse(syncedMeasurement.equals(null))
    }

    @Test
    fun equals_whenSameInstance_returnsTrue() {
        val syncedMeasurement = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement()
        @Suppress("ReplaceCallWithBinaryOperator")
        assertTrue(syncedMeasurement.equals(syncedMeasurement))
    }

    @Test
    fun equals_whenDifferentType_returnsFalse() {
        val syncedMeasurement = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement()
        assertFalse(syncedMeasurement == Any())
    }

    @Test
    fun equals_whenDifferentContent_returnsFalse() {
        val randomizer = UniformRandomizer()
        val gx1 = randomizer.nextFloat()
        val gy1 = randomizer.nextFloat()
        val gz1 = randomizer.nextFloat()
        val wx1 = randomizer.nextFloat()
        val wy1 = randomizer.nextFloat()
        val wz1 = randomizer.nextFloat()
        val wbx1 = randomizer.nextFloat()
        val wby1 = randomizer.nextFloat()
        val wbz1 = randomizer.nextFloat()
        val bx1 = randomizer.nextFloat()
        val by1 = randomizer.nextFloat()
        val bz1 = randomizer.nextFloat()
        val hardIronX1 = randomizer.nextFloat()
        val hardIronY1 = randomizer.nextFloat()
        val hardIronZ1 = randomizer.nextFloat()
        val timestamp1 = System.nanoTime()

        val gravityMeasurement1 = GravitySensorMeasurement(
            gx1,
            gy1,
            gz1,
            timestamp1,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.NED
        )
        val gyroscopeMeasurement1 = GyroscopeSensorMeasurement(
            wx1,
            wy1,
            wz1,
            wbx1,
            wby1,
            wbz1,
            timestamp1,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
        val magnetometerMeasurement1 = MagnetometerSensorMeasurement(
            bx1,
            by1,
            bz1,
            hardIronX1,
            hardIronY1,
            hardIronZ1,
            timestamp1,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )

        val syncedMeasurement1 = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            gravityMeasurement1,
            gyroscopeMeasurement1,
            magnetometerMeasurement1,
            timestamp1
        )

        val gx2 = randomizer.nextFloat()
        val gy2 = randomizer.nextFloat()
        val gz2 = randomizer.nextFloat()
        val wx2 = randomizer.nextFloat()
        val wy2 = randomizer.nextFloat()
        val wz2 = randomizer.nextFloat()
        val wbx2 = randomizer.nextFloat()
        val wby2 = randomizer.nextFloat()
        val wbz2 = randomizer.nextFloat()
        val bx2 = randomizer.nextFloat()
        val by2 = randomizer.nextFloat()
        val bz2 = randomizer.nextFloat()
        val hardIronX2 = randomizer.nextFloat()
        val hardIronY2 = randomizer.nextFloat()
        val hardIronZ2 = randomizer.nextFloat()
        val timestamp2 = System.nanoTime()

        val gravityMeasurement2 = GravitySensorMeasurement(
            gx2,
            gy2,
            gz2,
            timestamp2,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.NED
        )
        val gyroscopeMeasurement2 = GyroscopeSensorMeasurement(
            wx2,
            wy2,
            wz2,
            wbx2,
            wby2,
            wbz2,
            timestamp2,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
        val magnetometerMeasurement2 = MagnetometerSensorMeasurement(
            bx2,
            by2,
            bz2,
            hardIronX2,
            hardIronY2,
            hardIronZ2,
            timestamp2,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )

        val syncedMeasurement2 = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            gravityMeasurement2,
            gyroscopeMeasurement2,
            magnetometerMeasurement2,
            timestamp2
        )

        assertFalse(syncedMeasurement1 == syncedMeasurement2)
    }

    @Test
    fun equals_whenDifferentTimestamp_returnsFalse() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp1 = System.nanoTime()

        val gravityMeasurement = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            timestamp1,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.NED
        )
        val gyroscopeMeasurement = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            wbx,
            wby,
            wbz,
            timestamp1,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
        val magnetometerMeasurement = MagnetometerSensorMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            timestamp1,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )

        val syncedMeasurement1 = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            gravityMeasurement,
            gyroscopeMeasurement,
            magnetometerMeasurement,
            timestamp1
        )

        val timestamp2 = System.nanoTime()

        val syncedMeasurement2 = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            gravityMeasurement,
            gyroscopeMeasurement,
            magnetometerMeasurement,
            timestamp2
        )

        assertFalse(syncedMeasurement1 == syncedMeasurement2)
    }

    @Test
    fun equals_whenDifferentGravity_returnsFalse() {
        val randomizer = UniformRandomizer()
        val gx1 = randomizer.nextFloat()
        val gy1 = randomizer.nextFloat()
        val gz1 = randomizer.nextFloat()
        val wx1 = randomizer.nextFloat()
        val wy1 = randomizer.nextFloat()
        val wz1 = randomizer.nextFloat()
        val wbx1 = randomizer.nextFloat()
        val wby1 = randomizer.nextFloat()
        val wbz1 = randomizer.nextFloat()
        val bx1 = randomizer.nextFloat()
        val by1 = randomizer.nextFloat()
        val bz1 = randomizer.nextFloat()
        val hardIronX1 = randomizer.nextFloat()
        val hardIronY1 = randomizer.nextFloat()
        val hardIronZ1 = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val gravityMeasurement1 = GravitySensorMeasurement(
            gx1,
            gy1,
            gz1,
            timestamp,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.NED
        )
        val gyroscopeMeasurement1 = GyroscopeSensorMeasurement(
            wx1,
            wy1,
            wz1,
            wbx1,
            wby1,
            wbz1,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
        val magnetometerMeasurement1 = MagnetometerSensorMeasurement(
            bx1,
            by1,
            bz1,
            hardIronX1,
            hardIronY1,
            hardIronZ1,
            timestamp,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )

        val syncedMeasurement1 = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            gravityMeasurement1,
            gyroscopeMeasurement1,
            magnetometerMeasurement1,
            timestamp
        )

        val gx2 = randomizer.nextFloat()
        val gy2 = randomizer.nextFloat()
        val gz2 = randomizer.nextFloat()
        val wx2 = randomizer.nextFloat()
        val wy2 = randomizer.nextFloat()
        val wz2 = randomizer.nextFloat()
        val wbx2 = randomizer.nextFloat()
        val wby2 = randomizer.nextFloat()
        val wbz2 = randomizer.nextFloat()
        val bx2 = randomizer.nextFloat()
        val by2 = randomizer.nextFloat()
        val bz2 = randomizer.nextFloat()
        val hardIronX2 = randomizer.nextFloat()
        val hardIronY2 = randomizer.nextFloat()
        val hardIronZ2 = randomizer.nextFloat()

        val gravityMeasurement2 = GravitySensorMeasurement(
            gx2,
            gy2,
            gz2,
            timestamp,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.NED
        )
        val gyroscopeMeasurement2 = GyroscopeSensorMeasurement(
            wx2,
            wy2,
            wz2,
            wbx2,
            wby2,
            wbz2,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
        val magnetometerMeasurement2 = MagnetometerSensorMeasurement(
            bx2,
            by2,
            bz2,
            hardIronX2,
            hardIronY2,
            hardIronZ2,
            timestamp,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )

        val syncedMeasurement2 = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            gravityMeasurement2,
            gyroscopeMeasurement2,
            magnetometerMeasurement2,
            timestamp
        )

        assertFalse(syncedMeasurement1 == syncedMeasurement2)
    }

    @Test
    fun equals_whenDifferentGyroscope_returnsFalse() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val wx1 = randomizer.nextFloat()
        val wy1 = randomizer.nextFloat()
        val wz1 = randomizer.nextFloat()
        val wbx1 = randomizer.nextFloat()
        val wby1 = randomizer.nextFloat()
        val wbz1 = randomizer.nextFloat()
        val bx1 = randomizer.nextFloat()
        val by1 = randomizer.nextFloat()
        val bz1 = randomizer.nextFloat()
        val hardIronX1 = randomizer.nextFloat()
        val hardIronY1 = randomizer.nextFloat()
        val hardIronZ1 = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val gravityMeasurement = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            timestamp,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.NED
        )
        val gyroscopeMeasurement1 = GyroscopeSensorMeasurement(
            wx1,
            wy1,
            wz1,
            wbx1,
            wby1,
            wbz1,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
        val magnetometerMeasurement1 = MagnetometerSensorMeasurement(
            bx1,
            by1,
            bz1,
            hardIronX1,
            hardIronY1,
            hardIronZ1,
            timestamp,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )

        val syncedMeasurement1 = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            gravityMeasurement,
            gyroscopeMeasurement1,
            magnetometerMeasurement1,
            timestamp
        )

        val wx2 = randomizer.nextFloat()
        val wy2 = randomizer.nextFloat()
        val wz2 = randomizer.nextFloat()
        val wbx2 = randomizer.nextFloat()
        val wby2 = randomizer.nextFloat()
        val wbz2 = randomizer.nextFloat()
        val bx2 = randomizer.nextFloat()
        val by2 = randomizer.nextFloat()
        val bz2 = randomizer.nextFloat()
        val hardIronX2 = randomizer.nextFloat()
        val hardIronY2 = randomizer.nextFloat()
        val hardIronZ2 = randomizer.nextFloat()

        val gyroscopeMeasurement2 = GyroscopeSensorMeasurement(
            wx2,
            wy2,
            wz2,
            wbx2,
            wby2,
            wbz2,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
        val magnetometerMeasurement2 = MagnetometerSensorMeasurement(
            bx2,
            by2,
            bz2,
            hardIronX2,
            hardIronY2,
            hardIronZ2,
            timestamp,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )

        val syncedMeasurement2 = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            gravityMeasurement,
            gyroscopeMeasurement2,
            magnetometerMeasurement2,
            timestamp
        )

        assertFalse(syncedMeasurement1 == syncedMeasurement2)
    }

    @Test
    fun equals_whenDifferentMagnetometer_returnsFalse() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val bx1 = randomizer.nextFloat()
        val by1 = randomizer.nextFloat()
        val bz1 = randomizer.nextFloat()
        val hardIronX1 = randomizer.nextFloat()
        val hardIronY1 = randomizer.nextFloat()
        val hardIronZ1 = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val gravityMeasurement = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            timestamp,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.NED
        )
        val gyroscopeMeasurement = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            wbx,
            wby,
            wbz,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
        val magnetometerMeasurement1 = MagnetometerSensorMeasurement(
            bx1,
            by1,
            bz1,
            hardIronX1,
            hardIronY1,
            hardIronZ1,
            timestamp,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )

        val syncedMeasurement1 = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            gravityMeasurement,
            gyroscopeMeasurement,
            magnetometerMeasurement1,
            timestamp
        )

        val bx2 = randomizer.nextFloat()
        val by2 = randomizer.nextFloat()
        val bz2 = randomizer.nextFloat()
        val hardIronX2 = randomizer.nextFloat()
        val hardIronY2 = randomizer.nextFloat()
        val hardIronZ2 = randomizer.nextFloat()

        val magnetometerMeasurement2 = MagnetometerSensorMeasurement(
            bx2,
            by2,
            bz2,
            hardIronX2,
            hardIronY2,
            hardIronZ2,
            timestamp,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )

        val syncedMeasurement2 = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            gravityMeasurement,
            gyroscopeMeasurement,
            magnetometerMeasurement2,
            timestamp
        )

        assertFalse(syncedMeasurement1 == syncedMeasurement2)
    }

    @Test
    fun equals_whenEqualContent_returnsTrue() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val gravityMeasurement = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            timestamp,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.NED
        )
        val gyroscopeMeasurement = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            wbx,
            wby,
            wbz,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
        val magnetometerMeasurement = MagnetometerSensorMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            timestamp,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )

        val syncedMeasurement1 = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            gravityMeasurement,
            gyroscopeMeasurement,
            magnetometerMeasurement,
            timestamp
        )

        val syncedMeasurement2 = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            gravityMeasurement,
            gyroscopeMeasurement,
            magnetometerMeasurement,
            timestamp
        )

        assertTrue(syncedMeasurement1 == syncedMeasurement2)
    }

    @Test
    fun hashCode_whenEqualObjects_returnsSameValue() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val gravityMeasurement = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            timestamp,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.NED
        )
        val gyroscopeMeasurement = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            wbx,
            wby,
            wbz,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
        val magnetometerMeasurement = MagnetometerSensorMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            timestamp,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )

        val syncedMeasurement1 = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            gravityMeasurement,
            gyroscopeMeasurement,
            magnetometerMeasurement,
            timestamp
        )

        val syncedMeasurement2 = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            gravityMeasurement,
            gyroscopeMeasurement,
            magnetometerMeasurement,
            timestamp
        )

        assertTrue(syncedMeasurement1 == syncedMeasurement2)
        assertEquals(syncedMeasurement1.hashCode(), syncedMeasurement2.hashCode())
    }

    @Test
    fun hashCode_whenDifferentObjects_returnsDifferentValues() {
        val randomizer = UniformRandomizer()
        val gx1 = randomizer.nextFloat()
        val gy1 = randomizer.nextFloat()
        val gz1 = randomizer.nextFloat()
        val wx1 = randomizer.nextFloat()
        val wy1 = randomizer.nextFloat()
        val wz1 = randomizer.nextFloat()
        val wbx1 = randomizer.nextFloat()
        val wby1 = randomizer.nextFloat()
        val wbz1 = randomizer.nextFloat()
        val bx1 = randomizer.nextFloat()
        val by1 = randomizer.nextFloat()
        val bz1 = randomizer.nextFloat()
        val hardIronX1 = randomizer.nextFloat()
        val hardIronY1 = randomizer.nextFloat()
        val hardIronZ1 = randomizer.nextFloat()
        val timestamp1 = System.nanoTime()

        val gravityMeasurement1 = GravitySensorMeasurement(
            gx1,
            gy1,
            gz1,
            timestamp1,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.NED
        )
        val gyroscopeMeasurement1 = GyroscopeSensorMeasurement(
            wx1,
            wy1,
            wz1,
            wbx1,
            wby1,
            wbz1,
            timestamp1,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
        val magnetometerMeasurement1 = MagnetometerSensorMeasurement(
            bx1,
            by1,
            bz1,
            hardIronX1,
            hardIronY1,
            hardIronZ1,
            timestamp1,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )

        val syncedMeasurement1 = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            gravityMeasurement1,
            gyroscopeMeasurement1,
            magnetometerMeasurement1,
            timestamp1
        )

        val gx2 = randomizer.nextFloat()
        val gy2 = randomizer.nextFloat()
        val gz2 = randomizer.nextFloat()
        val wx2 = randomizer.nextFloat()
        val wy2 = randomizer.nextFloat()
        val wz2 = randomizer.nextFloat()
        val wbx2 = randomizer.nextFloat()
        val wby2 = randomizer.nextFloat()
        val wbz2 = randomizer.nextFloat()
        val bx2 = randomizer.nextFloat()
        val by2 = randomizer.nextFloat()
        val bz2 = randomizer.nextFloat()
        val hardIronX2 = randomizer.nextFloat()
        val hardIronY2 = randomizer.nextFloat()
        val hardIronZ2 = randomizer.nextFloat()
        val timestamp2 = System.nanoTime()

        val gravityMeasurement2 = GravitySensorMeasurement(
            gx2,
            gy2,
            gz2,
            timestamp2,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.NED
        )
        val gyroscopeMeasurement2 = GyroscopeSensorMeasurement(
            wx2,
            wy2,
            wz2,
            wbx2,
            wby2,
            wbz2,
            timestamp2,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
        val magnetometerMeasurement2 = MagnetometerSensorMeasurement(
            bx2,
            by2,
            bz2,
            hardIronX2,
            hardIronY2,
            hardIronZ2,
            timestamp2,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )

        val syncedMeasurement2 = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(
            gravityMeasurement2,
            gyroscopeMeasurement2,
            magnetometerMeasurement2,
            timestamp2
        )

        assertFalse(syncedMeasurement1 == syncedMeasurement2)
        assertNotEquals(syncedMeasurement1.hashCode(), syncedMeasurement2.hashCode())
    }
}