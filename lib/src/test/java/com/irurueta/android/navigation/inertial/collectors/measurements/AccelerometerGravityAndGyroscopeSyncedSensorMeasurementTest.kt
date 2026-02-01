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

class AccelerometerGravityAndGyroscopeSyncedSensorMeasurementTest {

    @Test
    fun constructor_whenDefault_setsExpectedValues() {
        val syncedMeasurement = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement()

        // check
        assertNull(syncedMeasurement.accelerometerMeasurement)
        assertNull(syncedMeasurement.gravityMeasurement)
        assertNull(syncedMeasurement.gyroscopeMeasurement)
    }

    @Test
    fun constructor_whenAllParameters_setsExpectedValues() {
        val accelerometerMeasurement = AccelerometerSensorMeasurement()
        val gravityMeasurement = GravitySensorMeasurement()
        val gyroscopeMeasurement = GyroscopeSensorMeasurement()
        val timestamp = System.nanoTime()
        val syncedMeasurement = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement,
            gravityMeasurement,
            gyroscopeMeasurement,
            timestamp
        )

        // check
        assertSame(accelerometerMeasurement, syncedMeasurement.accelerometerMeasurement)
        assertSame(gyroscopeMeasurement, syncedMeasurement.gyroscopeMeasurement)
        assertEquals(timestamp, syncedMeasurement.timestamp)
    }

    @Test
    fun constructor_whenCopy_setsExpectedValues() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val accelerometerMeasurement = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            abx,
            aby,
            abz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
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

        val syncedMeasurement1 = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement,
            gravityMeasurement,
            gyroscopeMeasurement,
            timestamp
        )

        val syncedMeasurement2 = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(syncedMeasurement1)

        // check
        assertEquals(accelerometerMeasurement, syncedMeasurement2.accelerometerMeasurement)
        assertNotSame(accelerometerMeasurement, syncedMeasurement2.accelerometerMeasurement)
        assertEquals(gravityMeasurement, syncedMeasurement2.gravityMeasurement)
        assertNotSame(gravityMeasurement, syncedMeasurement2.gravityMeasurement)
        assertEquals(gyroscopeMeasurement, syncedMeasurement2.gyroscopeMeasurement)
        assertNotSame(gyroscopeMeasurement, syncedMeasurement2.gyroscopeMeasurement)
        assertEquals(timestamp, syncedMeasurement2.timestamp)
    }

    @Test
    fun accelerometerMeasurement_setsExpectedValue() {
        val syncedMeasurement = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement()

        // check default value
        assertNull(syncedMeasurement.accelerometerMeasurement)

        // set new value
        val accelerometerMeasurement = AccelerometerSensorMeasurement()
        syncedMeasurement.accelerometerMeasurement = accelerometerMeasurement

        // check
        assertSame(accelerometerMeasurement, syncedMeasurement.accelerometerMeasurement)
    }

    @Test
    fun gravityMeasurement_setsExpectedValue() {
        val syncedMeasurement = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement()

        // check default value
        assertNull(syncedMeasurement.gravityMeasurement)

        // set new value
        val gravityMeasurement = GravitySensorMeasurement()
        syncedMeasurement.gravityMeasurement = gravityMeasurement

        // check
        assertSame(gravityMeasurement, syncedMeasurement.gravityMeasurement)
    }

    @Test
    fun gyroscopeMeasurement_setsExpectedValue() {
        val syncedMeasurement = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement()

        // check default value
        assertNull(syncedMeasurement.gyroscopeMeasurement)

        // set new value
        val gyroscopeMeasurement = GyroscopeSensorMeasurement()
        syncedMeasurement.gyroscopeMeasurement = gyroscopeMeasurement

        // check
        assertSame(gyroscopeMeasurement, syncedMeasurement.gyroscopeMeasurement)
    }

    @Test
    fun timestamp_setsExpectedValue() {
        val syncedMeasurement = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement()

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
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val accelerometerMeasurement = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            abx,
            aby,
            abz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
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

        val syncedMeasurement1 = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement,
            gravityMeasurement,
            gyroscopeMeasurement,
            timestamp
        )

        val syncedMeasurement2 = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement()
        syncedMeasurement2.copyFrom(syncedMeasurement1)

        // check
        assertEquals(accelerometerMeasurement, syncedMeasurement2.accelerometerMeasurement)
        assertNotSame(accelerometerMeasurement, syncedMeasurement2.accelerometerMeasurement)
        assertEquals(gravityMeasurement, syncedMeasurement2.gravityMeasurement)
        assertNotSame(gravityMeasurement, syncedMeasurement2.gravityMeasurement)
        assertEquals(gyroscopeMeasurement, syncedMeasurement2.gyroscopeMeasurement)
        assertNotSame(gyroscopeMeasurement, syncedMeasurement2.gyroscopeMeasurement)
        assertEquals(timestamp, syncedMeasurement2.timestamp)
    }

    @Test
    fun copyFrom_whenMissingMeasurements_makesExpectedCopy() {
        val timestamp = System.nanoTime()

        val syncedMeasurement = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            timestamp = timestamp
        )

        val syncedMeasurement2 = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement()
        syncedMeasurement2.copyFrom(syncedMeasurement)

        // check
        assertNull(syncedMeasurement2.accelerometerMeasurement)
        assertNull(syncedMeasurement2.gravityMeasurement)
        assertNull(syncedMeasurement2.gyroscopeMeasurement)
        assertEquals(timestamp, syncedMeasurement2.timestamp)
    }

    @Test
    fun copyTo_makesExpectedCopy() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val accelerometerMeasurement = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            abx,
            aby,
            abz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
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

        val syncedMeasurement1 = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement,
            gravityMeasurement,
            gyroscopeMeasurement,
            timestamp
        )

        val syncedMeasurement2 = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement()
        syncedMeasurement1.copyTo(syncedMeasurement2)

        // check
        assertEquals(accelerometerMeasurement, syncedMeasurement2.accelerometerMeasurement)
        assertNotSame(accelerometerMeasurement, syncedMeasurement2.accelerometerMeasurement)
        assertEquals(gravityMeasurement, syncedMeasurement2.gravityMeasurement)
        assertNotSame(gravityMeasurement, syncedMeasurement2.gravityMeasurement)
        assertEquals(gyroscopeMeasurement, syncedMeasurement2.gyroscopeMeasurement)
        assertNotSame(gyroscopeMeasurement, syncedMeasurement2.gyroscopeMeasurement)
        assertEquals(timestamp, syncedMeasurement2.timestamp)
    }

    @Test
    fun copy_makesExpectedCopy() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val accelerometerMeasurement = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            abx,
            aby,
            abz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
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

        val syncedMeasurement1 = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement,
            gravityMeasurement,
            gyroscopeMeasurement,
            timestamp
        )

        val syncedMeasurement2 = syncedMeasurement1.copy()

        // check
        assertEquals(accelerometerMeasurement, syncedMeasurement2.accelerometerMeasurement)
        assertNotSame(accelerometerMeasurement, syncedMeasurement2.accelerometerMeasurement)
        assertEquals(gravityMeasurement, syncedMeasurement2.gravityMeasurement)
        assertNotSame(gravityMeasurement, syncedMeasurement2.gravityMeasurement)
        assertEquals(gyroscopeMeasurement, syncedMeasurement2.gyroscopeMeasurement)
        assertNotSame(gyroscopeMeasurement, syncedMeasurement2.gyroscopeMeasurement)
        assertEquals(timestamp, syncedMeasurement2.timestamp)
    }

    @Test
    fun toNedOrThrow_whenAllMeasurementsAreENU_returnsExpectedValue() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val enuAccelerometerMeasurement = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            abx,
            aby,
            abz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )
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

        val enuSyncedMeasurement = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            enuAccelerometerMeasurement,
            enuGravityMeasurement,
            enuGyroscopeMeasurement,
            timestamp
        )

        val nedSyncedMeasurement1 = enuSyncedMeasurement.toNedOrThrow()
        val nedSyncedMeasurement2 = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            AccelerometerSensorMeasurement(),
            GravitySensorMeasurement(),
            GyroscopeSensorMeasurement(),
            timestamp
        )
        enuSyncedMeasurement.toNedOrThrow(nedSyncedMeasurement2)

        // check
        val nedAccelerometerMeasurement = enuAccelerometerMeasurement.toNedOrThrow()
        val nedGravityMeasurement = enuGravityMeasurement.toNedOrThrow()
        val nedGyroscopeMeasurement = enuGyroscopeMeasurement.toNedOrThrow()

        assertEquals(nedSyncedMeasurement1, nedSyncedMeasurement2)
        assertEquals(nedAccelerometerMeasurement, nedSyncedMeasurement2.accelerometerMeasurement)
        assertEquals(nedGravityMeasurement, nedSyncedMeasurement2.gravityMeasurement)
        assertEquals(nedGyroscopeMeasurement, nedSyncedMeasurement2.gyroscopeMeasurement)
        assertEquals(timestamp, nedSyncedMeasurement2.timestamp)
    }

    @Test
    fun toNedOrThrow_whenNoMeasurements_returnsExpectedValue() {
        val timestamp = System.nanoTime()

        val enuSyncedMeasurement = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            timestamp = timestamp
        )

        val nedSyncedMeasurement1 = enuSyncedMeasurement.toNedOrThrow()
        val nedSyncedMeasurement2 = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            AccelerometerSensorMeasurement(),
            GravitySensorMeasurement(),
            GyroscopeSensorMeasurement(),
            timestamp
        )
        enuSyncedMeasurement.toNedOrThrow(nedSyncedMeasurement2)

        // check
        assertEquals(nedSyncedMeasurement1, nedSyncedMeasurement2)
        assertNull(nedSyncedMeasurement2.accelerometerMeasurement)
        assertNull(nedSyncedMeasurement2.gravityMeasurement)
        assertNull(nedSyncedMeasurement2.gyroscopeMeasurement)
        assertEquals(timestamp, nedSyncedMeasurement2.timestamp)
    }

    @Test
    fun toNedOrThrow_whenNEDAccelerometerMeasurement_throwsIllegalArgumentException() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val wrongAccelerometerMeasurement = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            abx,
            aby,
            abz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
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

        val wrongSyncedMeasurement = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            wrongAccelerometerMeasurement,
            enuGravityMeasurement,
            enuGyroscopeMeasurement,
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
    fun toNedOrThrow_whenNEDGravityMeasurement_throwsIllegalArgumentException() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val accelerometerMeasurement = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            abx,
            aby,
            abz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )
        val nedGravityMeasurement = GravitySensorMeasurement(
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

        val wrongSyncedMeasurement = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement,
            nedGravityMeasurement,
            enuGyroscopeMeasurement,
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
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val accelerometerMeasurement = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            abx,
            aby,
            abz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )
        val gravityMeasurement = GravitySensorMeasurement(
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

        val wrongSyncedMeasurement = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement,
            gravityMeasurement,
            wrongGyroscopeMeasurement,
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
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val nedAccelerometerMeasurement = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            abx,
            aby,
            abz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
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

        val nedSyncedMeasurement = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            nedAccelerometerMeasurement,
            nedGravityMeasurement,
            nedGyroscopeMeasurement,
            timestamp
        )

        val enuSyncedMeasurement1 = nedSyncedMeasurement.toEnuOrThrow()
        val enuSyncedMeasurement2 = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            AccelerometerSensorMeasurement(),
            GravitySensorMeasurement(),
            GyroscopeSensorMeasurement(),
            timestamp
        )
        nedSyncedMeasurement.toEnuOrThrow(enuSyncedMeasurement2)

        // check
        val enuAccelerometerMeasurement = nedAccelerometerMeasurement.toEnuOrThrow()
        val enuGravityMeasurement = nedGravityMeasurement.toEnuOrThrow()
        val enuGyroscopeMeasurement = nedGyroscopeMeasurement.toEnuOrThrow()

        assertEquals(enuSyncedMeasurement1, enuSyncedMeasurement2)
        assertEquals(enuAccelerometerMeasurement, enuSyncedMeasurement2.accelerometerMeasurement)
        assertEquals(enuGravityMeasurement, enuSyncedMeasurement2.gravityMeasurement)
        assertEquals(enuGyroscopeMeasurement, enuSyncedMeasurement2.gyroscopeMeasurement)
        assertEquals(timestamp, enuSyncedMeasurement2.timestamp)
    }

    @Test
    fun toEnuOrThrow_whenNoMeasurements_returnsExpectedValue() {
        val timestamp = System.nanoTime()

        val nedSyncedMeasurement = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            timestamp = timestamp
        )

        val enuSyncedMeasurement1 = nedSyncedMeasurement.toEnuOrThrow()
        val enuSyncedMeasurement2 = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            AccelerometerSensorMeasurement(),
            GravitySensorMeasurement(),
            GyroscopeSensorMeasurement(),
            timestamp
        )
        nedSyncedMeasurement.toEnuOrThrow(enuSyncedMeasurement2)

        // check
        assertEquals(enuSyncedMeasurement1, enuSyncedMeasurement2)
        assertNull(enuSyncedMeasurement2.accelerometerMeasurement)
        assertNull(enuSyncedMeasurement2.gravityMeasurement)
        assertNull(enuSyncedMeasurement2.gyroscopeMeasurement)
        assertEquals(timestamp, enuSyncedMeasurement2.timestamp)
    }

    @Test
    fun toEnuOrThrow_whenENUAccelerometerMeasurement_throwsIllegalArgumentException() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val wrongAccelerometerMeasurement = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            abx,
            aby,
            abz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )
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

        val wrongSyncedMeasurement = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            wrongAccelerometerMeasurement,
            nedGravityMeasurement,
            nedGyroscopeMeasurement,
            timestamp
        )

        assertThrows(IllegalArgumentException::class.java) {
            wrongSyncedMeasurement.toEnuOrThrow(wrongSyncedMeasurement)
        }
        assertThrows(IllegalArgumentException::class.java) {
            wrongSyncedMeasurement.toEnuOrThrow()
        }
    }

    @Test
    fun toEnuOrThrow_whenENUGravityMeasurement_throwsIllegalArgumentException() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val accelerometerMeasurement = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            abx,
            aby,
            abz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
        val wrongGravityMeasurement = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            timestamp,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.ENU
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

        val wrongSyncedMeasurement = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement,
            wrongGravityMeasurement,
            gyroscopeMeasurement,
            timestamp
        )

        assertThrows(IllegalArgumentException::class.java) {
            wrongSyncedMeasurement.toEnuOrThrow(wrongSyncedMeasurement)
        }
        assertThrows(IllegalArgumentException::class.java) {
            wrongSyncedMeasurement.toEnuOrThrow()
        }
    }

    @Test
    fun toEnuOrThrow_whenENUGyroscopeMeasurement_throwsIllegalArgumentException() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val accelerometerMeasurement = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            abx,
            aby,
            abz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
        val gravityMeasurement = GravitySensorMeasurement(
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

        val wrongSyncedMeasurement = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement,
            gravityMeasurement,
            wrongGyroscopeMeasurement,
            timestamp
        )

        assertThrows(IllegalArgumentException::class.java) {
            wrongSyncedMeasurement.toEnuOrThrow(wrongSyncedMeasurement)
        }
        assertThrows(IllegalArgumentException::class.java) {
            wrongSyncedMeasurement.toEnuOrThrow()
        }
    }

    @Test
    fun toNed_whenAllMeasurementsAreENU_returnsExpectedValue() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val enuAccelerometerMeasurement = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            abx,
            aby,
            abz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )
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

        val enuSyncedMeasurement = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            enuAccelerometerMeasurement,
            enuGravityMeasurement,
            enuGyroscopeMeasurement,
            timestamp
        )

        val nedSyncedMeasurement1 = enuSyncedMeasurement.toNed()
        val nedSyncedMeasurement2 = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            AccelerometerSensorMeasurement(),
            GravitySensorMeasurement(),
            GyroscopeSensorMeasurement(),
            timestamp
        )
        enuSyncedMeasurement.toNed(nedSyncedMeasurement2)

        // check
        val nedAccelerometerMeasurement = enuAccelerometerMeasurement.toNed()
        val nedGravityMeasurement = enuGravityMeasurement.toNed()
        val nedGyroscopeMeasurement = enuGyroscopeMeasurement.toNed()

        assertEquals(nedSyncedMeasurement1, nedSyncedMeasurement2)
        assertEquals(nedAccelerometerMeasurement, nedSyncedMeasurement2.accelerometerMeasurement)
        assertEquals(nedGravityMeasurement, nedSyncedMeasurement2.gravityMeasurement)
        assertEquals(nedGyroscopeMeasurement, nedSyncedMeasurement2.gyroscopeMeasurement)
        assertEquals(timestamp, nedSyncedMeasurement2.timestamp)
    }

    @Test
    fun toNed_whenNoMeasurements_returnsExpectedValue() {
        val timestamp = System.nanoTime()

        val enuSyncedMeasurement = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            timestamp = timestamp
        )

        val nedSyncedMeasurement1 = enuSyncedMeasurement.toNedOrThrow()
        val nedSyncedMeasurement2 = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            AccelerometerSensorMeasurement(),
            GravitySensorMeasurement(),
            GyroscopeSensorMeasurement(),
            timestamp
        )
        enuSyncedMeasurement.toNed(nedSyncedMeasurement2)

        // check
        assertEquals(nedSyncedMeasurement1, nedSyncedMeasurement2)
        assertNull(nedSyncedMeasurement2.accelerometerMeasurement)
        assertNull(nedSyncedMeasurement2.gravityMeasurement)
        assertNull(nedSyncedMeasurement2.gyroscopeMeasurement)
        assertEquals(timestamp, nedSyncedMeasurement2.timestamp)
    }

    @Test
    fun toEnu_whenAllMeasurementsAreNED_returnsExpectedValue() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val nedAccelerometerMeasurement = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            abx,
            aby,
            abz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
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

        val nedSyncedMeasurement = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            nedAccelerometerMeasurement,
            nedGravityMeasurement,
            nedGyroscopeMeasurement,
            timestamp
        )

        val enuSyncedMeasurement1 = nedSyncedMeasurement.toEnu()
        val enuSyncedMeasurement2 = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            AccelerometerSensorMeasurement(),
            GravitySensorMeasurement(),
            GyroscopeSensorMeasurement(),
            timestamp
        )
        nedSyncedMeasurement.toEnu(enuSyncedMeasurement2)

        // check
        val enuAccelerometerMeasurement = nedAccelerometerMeasurement.toEnu()
        val enuGravityMeasurement = nedGravityMeasurement.toEnu()
        val enuGyroscopeMeasurement = nedGyroscopeMeasurement.toEnu()

        assertEquals(enuSyncedMeasurement1, enuSyncedMeasurement2)
        assertEquals(enuAccelerometerMeasurement, enuSyncedMeasurement2.accelerometerMeasurement)
        assertEquals(enuGravityMeasurement, enuSyncedMeasurement2.gravityMeasurement)
        assertEquals(enuGyroscopeMeasurement, enuSyncedMeasurement2.gyroscopeMeasurement)
        assertEquals(timestamp, enuSyncedMeasurement2.timestamp)
    }

    @Test
    fun toEnu_whenNoMeasurements_returnsExpectedValue() {
        val timestamp = System.nanoTime()

        val nedSyncedMeasurement = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            timestamp = timestamp
        )

        val enuSyncedMeasurement1 = nedSyncedMeasurement.toEnu()
        val enuSyncedMeasurement2 = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            AccelerometerSensorMeasurement(),
            GravitySensorMeasurement(),
            GyroscopeSensorMeasurement(),
            timestamp
        )
        nedSyncedMeasurement.toEnu(enuSyncedMeasurement2)

        // check
        assertEquals(enuSyncedMeasurement1, enuSyncedMeasurement2)
        assertNull(enuSyncedMeasurement2.accelerometerMeasurement)
        assertNull(enuSyncedMeasurement2.gravityMeasurement)
        assertNull(enuSyncedMeasurement2.gyroscopeMeasurement)
        assertEquals(timestamp, enuSyncedMeasurement2.timestamp)
    }

    @Test
    fun equals_whenNull_returnsFalse() {
        val syncedMeasurement = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement()
        assertFalse(syncedMeasurement.equals(null))
    }

    @Test
    fun equals_whenSameInstance_returnsTrue() {
        val syncedMeasurement = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement()
        @Suppress("ReplaceCallWithBinaryOperator")
        assertTrue(syncedMeasurement.equals(syncedMeasurement))
    }

    @Test
    fun equals_whenDifferentType_returnsFalse() {
        val syncedMeasurement = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement()
        assertFalse(syncedMeasurement.equals(Any()))
    }

    @Test
    fun equals_whenDifferentContent_returnsFalse() {
        val randomizer = UniformRandomizer()
        val ax1 = randomizer.nextFloat()
        val ay1 = randomizer.nextFloat()
        val az1 = randomizer.nextFloat()
        val abx1 = randomizer.nextFloat()
        val aby1 = randomizer.nextFloat()
        val abz1 = randomizer.nextFloat()
        val gx1 = randomizer.nextFloat()
        val gy1 = randomizer.nextFloat()
        val gz1 = randomizer.nextFloat()
        val wx1 = randomizer.nextFloat()
        val wy1 = randomizer.nextFloat()
        val wz1 = randomizer.nextFloat()
        val wbx1 = randomizer.nextFloat()
        val wby1 = randomizer.nextFloat()
        val wbz1 = randomizer.nextFloat()
        val timestamp1 = System.nanoTime()

        val accelerometerMeasurement1 = AccelerometerSensorMeasurement(
            ax1,
            ay1,
            az1,
            abx1,
            aby1,
            abz1,
            timestamp1,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
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

        val syncedMeasurement1 = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement1,
            gravityMeasurement1,
            gyroscopeMeasurement1,
            timestamp1
        )

        val ax2 = randomizer.nextFloat()
        val ay2 = randomizer.nextFloat()
        val az2 = randomizer.nextFloat()
        val abx2 = randomizer.nextFloat()
        val aby2 = randomizer.nextFloat()
        val abz2 = randomizer.nextFloat()
        val gx2 = randomizer.nextFloat()
        val gy2 = randomizer.nextFloat()
        val gz2 = randomizer.nextFloat()
        val wx2 = randomizer.nextFloat()
        val wy2 = randomizer.nextFloat()
        val wz2 = randomizer.nextFloat()
        val wbx2 = randomizer.nextFloat()
        val wby2 = randomizer.nextFloat()
        val wbz2 = randomizer.nextFloat()
        val timestamp2 = System.nanoTime()

        val accelerometerMeasurement2 = AccelerometerSensorMeasurement(
            ax2,
            ay2,
            az2,
            abx2,
            aby2,
            abz2,
            timestamp2,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
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

        val syncedMeasurement2 = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement2,
            gravityMeasurement2,
            gyroscopeMeasurement2,
            timestamp2
        )

        assertFalse(syncedMeasurement1 == syncedMeasurement2)
    }

    @Test
    fun equals_whenDifferentTimestamp_returnsFalse() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val timestamp1 = System.nanoTime()

        val accelerometerMeasurement = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            abx,
            aby,
            abz,
            timestamp1,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
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

        val syncedMeasurement1 = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement,
            gravityMeasurement,
            gyroscopeMeasurement,
            timestamp1
        )

        val timestamp2 = System.nanoTime()

        val syncedMeasurement2 = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement,
            gravityMeasurement,
            gyroscopeMeasurement,
            timestamp2
        )

        assertFalse(syncedMeasurement1 == syncedMeasurement2)
    }

    @Test
    fun equals_whenDifferentAccelerometer_returnsFalse() {
        val randomizer = UniformRandomizer()
        val ax1 = randomizer.nextFloat()
        val ay1 = randomizer.nextFloat()
        val az1 = randomizer.nextFloat()
        val abx1 = randomizer.nextFloat()
        val aby1 = randomizer.nextFloat()
        val abz1 = randomizer.nextFloat()
        val gx1 = randomizer.nextFloat()
        val gy1 = randomizer.nextFloat()
        val gz1 = randomizer.nextFloat()
        val wx1 = randomizer.nextFloat()
        val wy1 = randomizer.nextFloat()
        val wz1 = randomizer.nextFloat()
        val wbx1 = randomizer.nextFloat()
        val wby1 = randomizer.nextFloat()
        val wbz1 = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val accelerometerMeasurement1 = AccelerometerSensorMeasurement(
            ax1,
            ay1,
            az1,
            abx1,
            aby1,
            abz1,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
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

        val syncedMeasurement1 = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement1,
            gravityMeasurement1,
            gyroscopeMeasurement1,
            timestamp
        )

        val ax2 = randomizer.nextFloat()
        val ay2 = randomizer.nextFloat()
        val az2 = randomizer.nextFloat()
        val abx2 = randomizer.nextFloat()
        val aby2 = randomizer.nextFloat()
        val abz2 = randomizer.nextFloat()
        val gx2 = randomizer.nextFloat()
        val gy2 = randomizer.nextFloat()
        val gz2 = randomizer.nextFloat()
        val wx2 = randomizer.nextFloat()
        val wy2 = randomizer.nextFloat()
        val wz2 = randomizer.nextFloat()
        val wbx2 = randomizer.nextFloat()
        val wby2 = randomizer.nextFloat()
        val wbz2 = randomizer.nextFloat()

        val accelerometerMeasurement2 = AccelerometerSensorMeasurement(
            ax2,
            ay2,
            az2,
            abx2,
            aby2,
            abz2,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
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

        val syncedMeasurement2 = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement2,
            gravityMeasurement2,
            gyroscopeMeasurement2,
            timestamp
        )

        assertFalse(syncedMeasurement1 == syncedMeasurement2)
    }

    @Test
    fun equals_whenDifferentGravity_returnsFalse() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val gx1 = randomizer.nextFloat()
        val gy1 = randomizer.nextFloat()
        val gz1 = randomizer.nextFloat()
        val wx1 = randomizer.nextFloat()
        val wy1 = randomizer.nextFloat()
        val wz1 = randomizer.nextFloat()
        val wbx1 = randomizer.nextFloat()
        val wby1 = randomizer.nextFloat()
        val wbz1 = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val accelerometerMeasurement = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            abx,
            aby,
            abz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
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

        val syncedMeasurement1 = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement,
            gravityMeasurement1,
            gyroscopeMeasurement1,
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

        val syncedMeasurement2 = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement,
            gravityMeasurement2,
            gyroscopeMeasurement2,
            timestamp
        )

        assertFalse(syncedMeasurement1 == syncedMeasurement2)
    }

    @Test
    fun equals_whenDifferentGyroscope_returnsFalse() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val wx1 = randomizer.nextFloat()
        val wy1 = randomizer.nextFloat()
        val wz1 = randomizer.nextFloat()
        val wbx1 = randomizer.nextFloat()
        val wby1 = randomizer.nextFloat()
        val wbz1 = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val accelerometerMeasurement = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            abx,
            aby,
            abz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
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

        val syncedMeasurement1 = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement,
            gravityMeasurement,
            gyroscopeMeasurement1,
            timestamp
        )

        val wx2 = randomizer.nextFloat()
        val wy2 = randomizer.nextFloat()
        val wz2 = randomizer.nextFloat()
        val wbx2 = randomizer.nextFloat()
        val wby2 = randomizer.nextFloat()
        val wbz2 = randomizer.nextFloat()

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

        val syncedMeasurement2 = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement,
            gravityMeasurement,
            gyroscopeMeasurement2,
            timestamp
        )

        assertFalse(syncedMeasurement1 == syncedMeasurement2)
    }

    @Test
    fun equals_whenEqualContent_returnsTrue() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val accelerometerMeasurement1 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            abx,
            aby,
            abz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
        val gravityMeasurement1 = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            timestamp,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.NED
        )
        val gyroscopeMeasurement1 = GyroscopeSensorMeasurement(
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

        val syncedMeasurement1 = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement1,
            gravityMeasurement1,
            gyroscopeMeasurement1,
            timestamp
        )

        val accelerometerMeasurement2 = accelerometerMeasurement1.copy()
        val gravityMeasurement2 = gravityMeasurement1.copy()
        val gyroscopeMeasurement2 = gyroscopeMeasurement1.copy()

        val syncedMeasurement2 = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement2,
            gravityMeasurement2,
            gyroscopeMeasurement2,
            timestamp
        )

        assertTrue(syncedMeasurement1 == syncedMeasurement2)
    }

    @Test
    fun hashCode_whenEqualObjects_returnsSameValue() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val accelerometerMeasurement1 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            abx,
            aby,
            abz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
        val gravityMeasurement1 = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            timestamp,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.NED
        )
        val gyroscopeMeasurement1 = GyroscopeSensorMeasurement(
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

        val syncedMeasurement1 = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement1,
            gravityMeasurement1,
            gyroscopeMeasurement1,
            timestamp
        )

        val accelerometerMeasurement2 = accelerometerMeasurement1.copy()
        val gravityMeasurement2 = gravityMeasurement1.copy()
        val gyroscopeMeasurement2 = gyroscopeMeasurement1.copy()

        val syncedMeasurement2 = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement2,
            gravityMeasurement2,
            gyroscopeMeasurement2,
            timestamp
        )

        assertTrue(syncedMeasurement1 == syncedMeasurement2)
        assertEquals(syncedMeasurement1.hashCode(), syncedMeasurement2.hashCode())
    }

    @Test
    fun hashCode_whenDifferentObjects_returnsFalse() {
        val randomizer = UniformRandomizer()
        val ax1 = randomizer.nextFloat()
        val ay1 = randomizer.nextFloat()
        val az1 = randomizer.nextFloat()
        val abx1 = randomizer.nextFloat()
        val aby1 = randomizer.nextFloat()
        val abz1 = randomizer.nextFloat()
        val gx1 = randomizer.nextFloat()
        val gy1 = randomizer.nextFloat()
        val gz1 = randomizer.nextFloat()
        val wx1 = randomizer.nextFloat()
        val wy1 = randomizer.nextFloat()
        val wz1 = randomizer.nextFloat()
        val wbx1 = randomizer.nextFloat()
        val wby1 = randomizer.nextFloat()
        val wbz1 = randomizer.nextFloat()
        val timestamp1 = System.nanoTime()

        val accelerometerMeasurement1 = AccelerometerSensorMeasurement(
            ax1,
            ay1,
            az1,
            abx1,
            aby1,
            abz1,
            timestamp1,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
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

        val syncedMeasurement1 = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement1,
            gravityMeasurement1,
            gyroscopeMeasurement1,
            timestamp1
        )

        val ax2 = randomizer.nextFloat()
        val ay2 = randomizer.nextFloat()
        val az2 = randomizer.nextFloat()
        val abx2 = randomizer.nextFloat()
        val aby2 = randomizer.nextFloat()
        val abz2 = randomizer.nextFloat()
        val gx2 = randomizer.nextFloat()
        val gy2 = randomizer.nextFloat()
        val gz2 = randomizer.nextFloat()
        val wx2 = randomizer.nextFloat()
        val wy2 = randomizer.nextFloat()
        val wz2 = randomizer.nextFloat()
        val wbx2 = randomizer.nextFloat()
        val wby2 = randomizer.nextFloat()
        val wbz2 = randomizer.nextFloat()
        val timestamp2 = System.nanoTime()

        val accelerometerMeasurement2 = AccelerometerSensorMeasurement(
            ax2,
            ay2,
            az2,
            abx2,
            aby2,
            abz2,
            timestamp2,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )
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

        val syncedMeasurement2 = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(
            accelerometerMeasurement2,
            gravityMeasurement2,
            gyroscopeMeasurement2,
            timestamp2
        )

        assertFalse(syncedMeasurement1 == syncedMeasurement2)
        assertNotEquals(syncedMeasurement1.hashCode(), syncedMeasurement2.hashCode())
    }
}