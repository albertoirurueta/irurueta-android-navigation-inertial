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

import com.irurueta.geometry.Quaternion
import com.irurueta.statistics.UniformRandomizer
import org.junit.Assert.*
import org.junit.Test

class AttitudeAndAccelerometerSyncedSensorMeasurementTest {

    @Test
    fun constructor_whenDefault_setsExpectedValues() {
        val syncedMeasurement = AttitudeAndAccelerometerSyncedSensorMeasurement()

        // check
        assertNull(syncedMeasurement.attitudeMeasurement)
        assertNull(syncedMeasurement.accelerometerMeasurement)
        assertEquals(0L, syncedMeasurement.timestamp)
    }

    @Test
    fun constructor_whenAllParameters_setsExpectedValues() {
        val attitudeMeasurement = AttitudeSensorMeasurement()
        val accelerometerMeasurement = AccelerometerSensorMeasurement()
        val timestamp = System.nanoTime()
        val syncedMeasurement = AttitudeAndAccelerometerSyncedSensorMeasurement(
            attitudeMeasurement,
            accelerometerMeasurement,
            timestamp
        )

        // check
        assertSame(attitudeMeasurement, syncedMeasurement.attitudeMeasurement)
        assertSame(accelerometerMeasurement, syncedMeasurement.accelerometerMeasurement)
        assertEquals(timestamp, syncedMeasurement.timestamp)
    }

    @Test
    fun constructor_whenCopy_setsExpectedValues() {
        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val attitudeMeasurement = AttitudeSensorMeasurement(
            attitude,
            headingAccuracy,
            timestamp,
            SensorAccuracy.HIGH,
            AttitudeSensorType.ABSOLUTE_ATTITUDE,
            SensorCoordinateSystem.NED
        )
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

        val syncedMeasurement1 = AttitudeAndAccelerometerSyncedSensorMeasurement(
            attitudeMeasurement,
            accelerometerMeasurement,
            timestamp
        )

        val syncedMeasurement2 = AttitudeAndAccelerometerSyncedSensorMeasurement(syncedMeasurement1)

        // check
        assertEquals(attitudeMeasurement, syncedMeasurement2.attitudeMeasurement)
        assertNotSame(attitudeMeasurement, syncedMeasurement2.attitudeMeasurement)
        assertEquals(accelerometerMeasurement, syncedMeasurement2.accelerometerMeasurement)
        assertNotSame(accelerometerMeasurement, syncedMeasurement2.accelerometerMeasurement)
        assertEquals(timestamp, syncedMeasurement2.timestamp)
    }

    @Test
    fun attitudeMeasurement_setsExpectedValue() {
        val syncedMeasurement = AttitudeAndAccelerometerSyncedSensorMeasurement()

        // check default value
        assertNull(syncedMeasurement.attitudeMeasurement)

        // set new value
        val attitudeMeasurement = AttitudeSensorMeasurement()
        syncedMeasurement.attitudeMeasurement = attitudeMeasurement

        // check
        assertSame(attitudeMeasurement, syncedMeasurement.attitudeMeasurement)
    }

    @Test
    fun accelerometerMeasurement_setsExpectedValue() {
        val syncedMeasurement = AttitudeAndAccelerometerSyncedSensorMeasurement()

        // check default value
        assertNull(syncedMeasurement.accelerometerMeasurement)

        // set new value
        val accelerometerMeasurement = AccelerometerSensorMeasurement()
        syncedMeasurement.accelerometerMeasurement = accelerometerMeasurement

        // check
        assertSame(accelerometerMeasurement, syncedMeasurement.accelerometerMeasurement)
    }

    @Test
    fun timestamp_setsExpectedValue() {
        val syncedMeasurement = AttitudeAndAccelerometerSyncedSensorMeasurement()

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
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val attitudeMeasurement = AttitudeSensorMeasurement(
            attitude,
            headingAccuracy,
            timestamp,
            SensorAccuracy.HIGH,
            AttitudeSensorType.ABSOLUTE_ATTITUDE,
            SensorCoordinateSystem.NED
        )
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

        val syncedMeasurement1 = AttitudeAndAccelerometerSyncedSensorMeasurement(
            attitudeMeasurement,
            accelerometerMeasurement,
            timestamp
        )

        val syncedMeasurement2 = AttitudeAndAccelerometerSyncedSensorMeasurement()
        syncedMeasurement2.copyFrom(syncedMeasurement1)

        // check
        assertEquals(attitudeMeasurement, syncedMeasurement2.attitudeMeasurement)
        assertNotSame(attitudeMeasurement, syncedMeasurement2.attitudeMeasurement)
        assertEquals(accelerometerMeasurement, syncedMeasurement2.accelerometerMeasurement)
        assertNotSame(accelerometerMeasurement, syncedMeasurement2.accelerometerMeasurement)
        assertEquals(timestamp, syncedMeasurement2.timestamp)
    }

    @Test
    fun copyFrom_whenMissingMeasurements_makesExpectedCopy() {
        val timestamp = System.nanoTime()

        val syncedMeasurement1 = AttitudeAndAccelerometerSyncedSensorMeasurement(
            timestamp = timestamp
        )

        val syncedMeasurement2 = AttitudeAndAccelerometerSyncedSensorMeasurement()
        syncedMeasurement2.copyFrom(syncedMeasurement1)

        // check
        assertNull(syncedMeasurement2.attitudeMeasurement)
        assertNull(syncedMeasurement2.accelerometerMeasurement)
        assertEquals(timestamp, syncedMeasurement2.timestamp)
    }

    @Test
    fun copyTo_makesExpectedCopy() {
        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val attitudeMeasurement = AttitudeSensorMeasurement(
            attitude,
            headingAccuracy,
            timestamp,
            SensorAccuracy.HIGH,
            AttitudeSensorType.ABSOLUTE_ATTITUDE,
            SensorCoordinateSystem.NED
        )
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

        val syncedMeasurement1 = AttitudeAndAccelerometerSyncedSensorMeasurement(
            attitudeMeasurement,
            accelerometerMeasurement,
            timestamp
        )

        val syncedMeasurement2 = AttitudeAndAccelerometerSyncedSensorMeasurement()
        syncedMeasurement1.copyTo(syncedMeasurement2)

        // check
        assertEquals(attitudeMeasurement, syncedMeasurement2.attitudeMeasurement)
        assertNotSame(attitudeMeasurement, syncedMeasurement2.attitudeMeasurement)
        assertEquals(accelerometerMeasurement, syncedMeasurement2.accelerometerMeasurement)
        assertNotSame(accelerometerMeasurement, syncedMeasurement2.accelerometerMeasurement)
        assertEquals(timestamp, syncedMeasurement2.timestamp)
    }

    @Test
    fun copy_makesExpectedCopy() {
        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val attitudeMeasurement = AttitudeSensorMeasurement(
            attitude,
            headingAccuracy,
            timestamp,
            SensorAccuracy.HIGH,
            AttitudeSensorType.ABSOLUTE_ATTITUDE,
            SensorCoordinateSystem.NED
        )
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

        val syncedMeasurement1 = AttitudeAndAccelerometerSyncedSensorMeasurement(
            attitudeMeasurement,
            accelerometerMeasurement,
            timestamp
        )

        val syncedMeasurement2 = syncedMeasurement1.copy()

        // check
        assertEquals(attitudeMeasurement, syncedMeasurement2.attitudeMeasurement)
        assertNotSame(attitudeMeasurement, syncedMeasurement2.attitudeMeasurement)
        assertEquals(accelerometerMeasurement, syncedMeasurement2.accelerometerMeasurement)
        assertNotSame(accelerometerMeasurement, syncedMeasurement2.accelerometerMeasurement)
        assertEquals(timestamp, syncedMeasurement2.timestamp)
    }

    @Test
    fun toNedOrThrow_whenAllMeasurementsAreENU_returnsExpectedValue() {
        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val enuAttitudeMeasurement = AttitudeSensorMeasurement(
            attitude,
            headingAccuracy,
            timestamp,
            SensorAccuracy.HIGH,
            AttitudeSensorType.ABSOLUTE_ATTITUDE,
            SensorCoordinateSystem.ENU
        )
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

        val enuSyncedMeasurement = AttitudeAndAccelerometerSyncedSensorMeasurement(
            enuAttitudeMeasurement,
            enuAccelerometerMeasurement,
            timestamp
        )

        val nedSyncedMeasurement1 = enuSyncedMeasurement.toNedOrThrow()
        val nedSyncedMeasurement2 = AttitudeAndAccelerometerSyncedSensorMeasurement(
            AttitudeSensorMeasurement(),
            AccelerometerSensorMeasurement(),
            timestamp
        )
        enuSyncedMeasurement.toNedOrThrow(nedSyncedMeasurement2)

        // check
        val nedAttitudeMeasurement = enuAttitudeMeasurement.toNedOrThrow()
        val nedAccelerometerMeasurement = enuAccelerometerMeasurement.toNedOrThrow()

        assertEquals(nedSyncedMeasurement1, nedSyncedMeasurement2)
        assertEquals(nedAttitudeMeasurement, nedSyncedMeasurement2.attitudeMeasurement)
        assertEquals(nedAccelerometerMeasurement, nedSyncedMeasurement2.accelerometerMeasurement)
        assertEquals(timestamp, nedSyncedMeasurement2.timestamp)
    }

    @Test
    fun toNedOrThrow_whenNoMeasurements_returnsExpectedValue() {
        val timestamp = System.nanoTime()

        val enuSyncedMeasurement = AttitudeAndAccelerometerSyncedSensorMeasurement(
            timestamp = timestamp
        )

        val nedSyncedMeasurement1 = enuSyncedMeasurement.toNedOrThrow()
        val nedSyncedMeasurement2 = AttitudeAndAccelerometerSyncedSensorMeasurement(
            AttitudeSensorMeasurement(),
            AccelerometerSensorMeasurement(),
            timestamp
        )
        enuSyncedMeasurement.toNedOrThrow(nedSyncedMeasurement2)

        // check
        assertEquals(nedSyncedMeasurement1, nedSyncedMeasurement2)
        assertNull(nedSyncedMeasurement2.attitudeMeasurement)
        assertNull(nedSyncedMeasurement2.accelerometerMeasurement)
        assertEquals(timestamp, nedSyncedMeasurement2.timestamp)
    }

    @Test
    fun toNedOrThrow_whenNEDAttitudeMeasurement_throwsIllegalArgumentException() {
        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val wrongAttitudeMeasurement = AttitudeSensorMeasurement(
            attitude,
            headingAccuracy,
            timestamp,
            SensorAccuracy.HIGH,
            AttitudeSensorType.ABSOLUTE_ATTITUDE,
            SensorCoordinateSystem.NED
        )
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

        val wrongSyncedMeasurement = AttitudeAndAccelerometerSyncedSensorMeasurement(
            wrongAttitudeMeasurement,
            enuAccelerometerMeasurement,
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
    fun toNedOrThrow_whenNEDAccelerometerMeasurement_throwsIllegalArgumentException() {
        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val enuAttitudeMeasurement = AttitudeSensorMeasurement(
            attitude,
            headingAccuracy,
            timestamp,
            SensorAccuracy.HIGH,
            AttitudeSensorType.ABSOLUTE_ATTITUDE,
            SensorCoordinateSystem.ENU
        )
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

        val wrongSyncedMeasurement = AttitudeAndAccelerometerSyncedSensorMeasurement(
            enuAttitudeMeasurement,
            wrongAccelerometerMeasurement,
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
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val nedAttitudeMeasurement = AttitudeSensorMeasurement(
            attitude,
            headingAccuracy,
            timestamp,
            SensorAccuracy.HIGH,
            AttitudeSensorType.ABSOLUTE_ATTITUDE,
            SensorCoordinateSystem.NED
        )
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

        val nedSyncedMeasurement = AttitudeAndAccelerometerSyncedSensorMeasurement(
            nedAttitudeMeasurement,
            nedAccelerometerMeasurement,
            timestamp
        )

        val enuSyncedMeasurement1 = nedSyncedMeasurement.toEnuOrThrow()
        val enuSyncedMeasurement2 = AttitudeAndAccelerometerSyncedSensorMeasurement(
            AttitudeSensorMeasurement(),
            AccelerometerSensorMeasurement(),
            timestamp
        )
        nedSyncedMeasurement.toEnuOrThrow(enuSyncedMeasurement2)

        // check
        val enuAttitudeMeasurement = nedAttitudeMeasurement.toEnuOrThrow()
        val enuAccelerometerMeasurement = nedAccelerometerMeasurement.toEnuOrThrow()

        assertEquals(enuSyncedMeasurement1, enuSyncedMeasurement2)
        assertEquals(enuAttitudeMeasurement, enuSyncedMeasurement2.attitudeMeasurement)
        assertEquals(enuAccelerometerMeasurement, enuSyncedMeasurement2.accelerometerMeasurement)
        assertEquals(timestamp, enuSyncedMeasurement2.timestamp)
    }

    @Test
    fun toEnuOrThrow_whenNoMeasurements_returnsExpectedValue() {
        val timestamp = System.nanoTime()

        val nedSyncedMeasurement = AttitudeAndAccelerometerSyncedSensorMeasurement(
            timestamp = timestamp
        )

        val enuSyncedMeasurement1 = nedSyncedMeasurement.toEnuOrThrow()
        val enuSyncedMeasurement2 = AttitudeAndAccelerometerSyncedSensorMeasurement(
            AttitudeSensorMeasurement(),
            AccelerometerSensorMeasurement(),
            timestamp
        )
        nedSyncedMeasurement.toEnuOrThrow(enuSyncedMeasurement2)

        // check
        assertEquals(enuSyncedMeasurement1, enuSyncedMeasurement2)
        assertNull(enuSyncedMeasurement2.attitudeMeasurement)
        assertNull(enuSyncedMeasurement2.accelerometerMeasurement)
        assertEquals(timestamp, enuSyncedMeasurement2.timestamp)
    }

    @Test
    fun toEnuOrThrow_whenENUAttitudeMeasurement_throwsIllegalArgumentException() {
        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val wrongAttitudeMeasurement = AttitudeSensorMeasurement(
            attitude,
            headingAccuracy,
            timestamp,
            SensorAccuracy.HIGH,
            AttitudeSensorType.ABSOLUTE_ATTITUDE,
            SensorCoordinateSystem.ENU
        )
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

        val wrongSyncedMeasurement = AttitudeAndAccelerometerSyncedSensorMeasurement(
            wrongAttitudeMeasurement,
            nedAccelerometerMeasurement,
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
    fun toEnuOrThrow_whenENUAccelerometerMeasurement_throwsIllegalArgumentException() {
        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val nedAttitudeMeasurement = AttitudeSensorMeasurement(
            attitude,
            headingAccuracy,
            timestamp,
            SensorAccuracy.HIGH,
            AttitudeSensorType.ABSOLUTE_ATTITUDE,
            SensorCoordinateSystem.NED
        )
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

        val wrongSyncedMeasurement = AttitudeAndAccelerometerSyncedSensorMeasurement(
            nedAttitudeMeasurement,
            wrongAccelerometerMeasurement,
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
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val enuAttitudeMeasurement = AttitudeSensorMeasurement(
            attitude,
            headingAccuracy,
            timestamp,
            SensorAccuracy.HIGH,
            AttitudeSensorType.ABSOLUTE_ATTITUDE,
            SensorCoordinateSystem.ENU
        )
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

        val enuSyncedMeasurement = AttitudeAndAccelerometerSyncedSensorMeasurement(
            enuAttitudeMeasurement,
            enuAccelerometerMeasurement,
            timestamp
        )

        val nedSyncedMeasurement1 = enuSyncedMeasurement.toNed()
        val nedSyncedMeasurement2 = AttitudeAndAccelerometerSyncedSensorMeasurement(
            AttitudeSensorMeasurement(),
            AccelerometerSensorMeasurement(),
            timestamp
        )
        enuSyncedMeasurement.toNed(nedSyncedMeasurement2)

        // check
        val nedAttitudeMeasurement = enuAttitudeMeasurement.toNed()
        val nedAccelerometerMeasurement = enuAccelerometerMeasurement.toNed()

        assertEquals(nedSyncedMeasurement1, nedSyncedMeasurement2)
        assertEquals(nedAttitudeMeasurement, nedSyncedMeasurement2.attitudeMeasurement)
        assertEquals(nedAccelerometerMeasurement, nedSyncedMeasurement2.accelerometerMeasurement)
        assertEquals(timestamp, nedSyncedMeasurement2.timestamp)
    }

    @Test
    fun toNed_whenNoMeasurements_returnsExpectedValue() {
        val timestamp = System.nanoTime()

        val enuSyncedMeasurement = AttitudeAndAccelerometerSyncedSensorMeasurement(
            timestamp = timestamp
        )

        val nedSyncedMeasurement1 = enuSyncedMeasurement.toNed()
        val nedSyncedMeasurement2 = AttitudeAndAccelerometerSyncedSensorMeasurement(
            AttitudeSensorMeasurement(),
            AccelerometerSensorMeasurement(),
            timestamp
        )
        enuSyncedMeasurement.toNed(nedSyncedMeasurement2)

        // check
        assertEquals(nedSyncedMeasurement1, nedSyncedMeasurement2)
        assertNull(nedSyncedMeasurement2.attitudeMeasurement)
        assertNull(nedSyncedMeasurement2.accelerometerMeasurement)
        assertEquals(timestamp, nedSyncedMeasurement2.timestamp)
    }

    @Test
    fun toEnu_whenAllMeasurementsAreNED_returnsExpectedValue() {
        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val nedAttitudeMeasurement = AttitudeSensorMeasurement(
            attitude,
            headingAccuracy,
            timestamp,
            SensorAccuracy.HIGH,
            AttitudeSensorType.ABSOLUTE_ATTITUDE,
            SensorCoordinateSystem.NED
        )
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

        val nedSyncedMeasurement = AttitudeAndAccelerometerSyncedSensorMeasurement(
            nedAttitudeMeasurement,
            nedAccelerometerMeasurement,
            timestamp
        )

        val enuSyncedMeasurement1 = nedSyncedMeasurement.toEnu()
        val enuSyncedMeasurement2 = AttitudeAndAccelerometerSyncedSensorMeasurement(
            AttitudeSensorMeasurement(),
            AccelerometerSensorMeasurement(),
            timestamp
        )
        nedSyncedMeasurement.toEnu(enuSyncedMeasurement2)

        // check
        val enuAttitudeMeasurement = nedAttitudeMeasurement.toEnu()
        val enuAccelerometerMeasurement = nedAccelerometerMeasurement.toEnu()

        assertEquals(enuSyncedMeasurement1, enuSyncedMeasurement2)
        assertEquals(enuAttitudeMeasurement, enuSyncedMeasurement2.attitudeMeasurement)
        assertEquals(enuAccelerometerMeasurement, enuSyncedMeasurement2.accelerometerMeasurement)
        assertEquals(timestamp, enuSyncedMeasurement2.timestamp)
    }

    @Test
    fun toEnu_whenNoMeasurements_returnsExpectedValue() {
        val timestamp = System.nanoTime()

        val nedSyncedMeasurement = AttitudeAndAccelerometerSyncedSensorMeasurement(
            timestamp = timestamp
        )

        val enuSyncedMeasurement1 = nedSyncedMeasurement.toEnu()
        val enuSyncedMeasurement2 = AttitudeAndAccelerometerSyncedSensorMeasurement(
            AttitudeSensorMeasurement(),
            AccelerometerSensorMeasurement(),
            timestamp
        )
        nedSyncedMeasurement.toEnu(enuSyncedMeasurement2)

        // check
        assertEquals(enuSyncedMeasurement1, enuSyncedMeasurement2)
        assertNull(enuSyncedMeasurement2.attitudeMeasurement)
        assertNull(enuSyncedMeasurement2.accelerometerMeasurement)
        assertEquals(timestamp, enuSyncedMeasurement2.timestamp)
    }

    @Test
    fun equals_whenNull_returnsFalse() {
        val syncedMeasurement = AttitudeAndAccelerometerSyncedSensorMeasurement()
        assertFalse(syncedMeasurement.equals(null))
    }

    @Test
    fun equals_whenSameInstance_returnsTrue() {
        val syncedMeasurement = AttitudeAndAccelerometerSyncedSensorMeasurement()
        @Suppress("ReplaceCallWithBinaryOperator")
        assertTrue(syncedMeasurement.equals(syncedMeasurement))
    }

    @Test
    fun equals_whenDifferentType_returnsFalse() {
        val syncedMeasurement = AttitudeAndAccelerometerSyncedSensorMeasurement()
        assertFalse(syncedMeasurement == Any())
    }

    @Test
    fun equals_whenDifferentContent_returnsFalse() {
        val randomizer = UniformRandomizer()
        val roll1 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch1 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw1 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude1 = Quaternion(roll1, pitch1, yaw1)
        val headingAccuracy1 = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val ax1 = randomizer.nextFloat()
        val ay1 = randomizer.nextFloat()
        val az1 = randomizer.nextFloat()
        val abx1 = randomizer.nextFloat()
        val aby1 = randomizer.nextFloat()
        val abz1 = randomizer.nextFloat()
        val timestamp1 = System.nanoTime()

        val attitudeMeasurement1 = AttitudeSensorMeasurement(
            attitude1,
            headingAccuracy1,
            timestamp1,
            SensorAccuracy.HIGH,
            AttitudeSensorType.ABSOLUTE_ATTITUDE,
            SensorCoordinateSystem.ENU
        )
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
            SensorCoordinateSystem.ENU
        )

        val syncedMeasurement1 = AttitudeAndAccelerometerSyncedSensorMeasurement(
            attitudeMeasurement1,
            accelerometerMeasurement1,
            timestamp1
        )

        val roll2 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch2 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw2 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude2 = Quaternion(roll2, pitch2, yaw2)
        val headingAccuracy2 = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val ax2 = randomizer.nextFloat()
        val ay2 = randomizer.nextFloat()
        val az2 = randomizer.nextFloat()
        val abx2 = randomizer.nextFloat()
        val aby2 = randomizer.nextFloat()
        val abz2 = randomizer.nextFloat()
        val timestamp2 = System.nanoTime()

        val attitudeMeasurement2 = AttitudeSensorMeasurement(
            attitude2,
            headingAccuracy2,
            timestamp2,
            SensorAccuracy.HIGH,
            AttitudeSensorType.ABSOLUTE_ATTITUDE,
            SensorCoordinateSystem.ENU
        )
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
            SensorCoordinateSystem.ENU
        )

        val syncedMeasurement2 = AttitudeAndAccelerometerSyncedSensorMeasurement(
            attitudeMeasurement2,
            accelerometerMeasurement2,
            timestamp2
        )

        assertFalse(syncedMeasurement1 == syncedMeasurement2)
    }

    @Test
    fun equals_whenDifferentTimestamp_returnsFalse() {
        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val timestamp1 = System.nanoTime()

        val attitudeMeasurement = AttitudeSensorMeasurement(
            attitude,
            headingAccuracy,
            timestamp1,
            SensorAccuracy.HIGH,
            AttitudeSensorType.ABSOLUTE_ATTITUDE,
            SensorCoordinateSystem.ENU
        )
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
            SensorCoordinateSystem.ENU
        )

        val syncedMeasurement1 = AttitudeAndAccelerometerSyncedSensorMeasurement(
            attitudeMeasurement,
            accelerometerMeasurement,
            timestamp1
        )

        val timestamp2 = System.nanoTime()

        val syncedMeasurement2 = AttitudeAndAccelerometerSyncedSensorMeasurement(
            attitudeMeasurement,
            accelerometerMeasurement,
            timestamp2
        )

        assertFalse(syncedMeasurement1 == syncedMeasurement2)
    }

    @Test
    fun equals_whenDifferentAttitude_returnsFalse() {
        val randomizer = UniformRandomizer()
        val roll1 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch1 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw1 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude1 = Quaternion(roll1, pitch1, yaw1)
        val headingAccuracy1 = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val ax1 = randomizer.nextFloat()
        val ay1 = randomizer.nextFloat()
        val az1 = randomizer.nextFloat()
        val abx1 = randomizer.nextFloat()
        val aby1 = randomizer.nextFloat()
        val abz1 = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val attitudeMeasurement1 = AttitudeSensorMeasurement(
            attitude1,
            headingAccuracy1,
            timestamp,
            SensorAccuracy.HIGH,
            AttitudeSensorType.ABSOLUTE_ATTITUDE,
            SensorCoordinateSystem.ENU
        )
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
            SensorCoordinateSystem.ENU
        )

        val syncedMeasurement1 = AttitudeAndAccelerometerSyncedSensorMeasurement(
            attitudeMeasurement1,
            accelerometerMeasurement1,
            timestamp
        )

        val roll2 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch2 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw2 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude2 = Quaternion(roll2, pitch2, yaw2)
        val headingAccuracy2 = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val ax2 = randomizer.nextFloat()
        val ay2 = randomizer.nextFloat()
        val az2 = randomizer.nextFloat()
        val abx2 = randomizer.nextFloat()
        val aby2 = randomizer.nextFloat()
        val abz2 = randomizer.nextFloat()

        val attitudeMeasurement2 = AttitudeSensorMeasurement(
            attitude2,
            headingAccuracy2,
            timestamp,
            SensorAccuracy.HIGH,
            AttitudeSensorType.ABSOLUTE_ATTITUDE,
            SensorCoordinateSystem.ENU
        )
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
            SensorCoordinateSystem.ENU
        )

        val syncedMeasurement2 = AttitudeAndAccelerometerSyncedSensorMeasurement(
            attitudeMeasurement2,
            accelerometerMeasurement2,
            timestamp
        )

        assertFalse(syncedMeasurement1 == syncedMeasurement2)
    }

    @Test
    fun equals_whenDifferentAccelerometer_returnsFalse() {
        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val ax1 = randomizer.nextFloat()
        val ay1 = randomizer.nextFloat()
        val az1 = randomizer.nextFloat()
        val abx1 = randomizer.nextFloat()
        val aby1 = randomizer.nextFloat()
        val abz1 = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val attitudeMeasurement = AttitudeSensorMeasurement(
            attitude,
            headingAccuracy,
            timestamp,
            SensorAccuracy.HIGH,
            AttitudeSensorType.ABSOLUTE_ATTITUDE,
            SensorCoordinateSystem.ENU
        )
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
            SensorCoordinateSystem.ENU
        )

        val syncedMeasurement1 = AttitudeAndAccelerometerSyncedSensorMeasurement(
            attitudeMeasurement,
            accelerometerMeasurement1,
            timestamp
        )

        val ax2 = randomizer.nextFloat()
        val ay2 = randomizer.nextFloat()
        val az2 = randomizer.nextFloat()
        val abx2 = randomizer.nextFloat()
        val aby2 = randomizer.nextFloat()
        val abz2 = randomizer.nextFloat()

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
            SensorCoordinateSystem.ENU
        )

        val syncedMeasurement2 = AttitudeAndAccelerometerSyncedSensorMeasurement(
            attitudeMeasurement,
            accelerometerMeasurement2,
            timestamp
        )

        assertFalse(syncedMeasurement1 == syncedMeasurement2)
    }

    @Test
    fun equals_whenEqualContent_returnsTrue() {
        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val attitudeMeasurement = AttitudeSensorMeasurement(
            attitude,
            headingAccuracy,
            timestamp,
            SensorAccuracy.HIGH,
            AttitudeSensorType.ABSOLUTE_ATTITUDE,
            SensorCoordinateSystem.ENU
        )
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

        val syncedMeasurement1 = AttitudeAndAccelerometerSyncedSensorMeasurement(
            attitudeMeasurement,
            accelerometerMeasurement,
            timestamp
        )

        val accelerometerMeasurement2 = accelerometerMeasurement.copy()
        val attitudeMeasurement2 = attitudeMeasurement.copy()

        val syncedMeasurement2 = AttitudeAndAccelerometerSyncedSensorMeasurement(
            attitudeMeasurement2,
            accelerometerMeasurement2,
            timestamp
        )

        assertTrue(syncedMeasurement1 == syncedMeasurement2)
    }

    @Test
    fun hashCode_whenEqualObjects_returnsSameValue() {
        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val attitudeMeasurement = AttitudeSensorMeasurement(
            attitude,
            headingAccuracy,
            timestamp,
            SensorAccuracy.HIGH,
            AttitudeSensorType.ABSOLUTE_ATTITUDE,
            SensorCoordinateSystem.ENU
        )
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

        val syncedMeasurement1 = AttitudeAndAccelerometerSyncedSensorMeasurement(
            attitudeMeasurement,
            accelerometerMeasurement,
            timestamp
        )

        val accelerometerMeasurement2 = accelerometerMeasurement.copy()
        val attitudeMeasurement2 = attitudeMeasurement.copy()

        val syncedMeasurement2 = AttitudeAndAccelerometerSyncedSensorMeasurement(
            attitudeMeasurement2,
            accelerometerMeasurement2,
            timestamp
        )

        assertTrue(syncedMeasurement1 == syncedMeasurement2)
        assertEquals(syncedMeasurement1.hashCode(), syncedMeasurement2.hashCode())
    }

    @Test
    fun hashCode_whenDifferentObjects_returnsDifferentValues() {
        val randomizer = UniformRandomizer()
        val roll1 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch1 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw1 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude1 = Quaternion(roll1, pitch1, yaw1)
        val headingAccuracy1 = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val ax1 = randomizer.nextFloat()
        val ay1 = randomizer.nextFloat()
        val az1 = randomizer.nextFloat()
        val abx1 = randomizer.nextFloat()
        val aby1 = randomizer.nextFloat()
        val abz1 = randomizer.nextFloat()
        val timestamp1 = System.nanoTime()

        val attitudeMeasurement1 = AttitudeSensorMeasurement(
            attitude1,
            headingAccuracy1,
            timestamp1,
            SensorAccuracy.HIGH,
            AttitudeSensorType.ABSOLUTE_ATTITUDE,
            SensorCoordinateSystem.ENU
        )
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
            SensorCoordinateSystem.ENU
        )

        val syncedMeasurement1 = AttitudeAndAccelerometerSyncedSensorMeasurement(
            attitudeMeasurement1,
            accelerometerMeasurement1,
            timestamp1
        )

        val roll2 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch2 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw2 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude2 = Quaternion(roll2, pitch2, yaw2)
        val headingAccuracy2 = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val ax2 = randomizer.nextFloat()
        val ay2 = randomizer.nextFloat()
        val az2 = randomizer.nextFloat()
        val abx2 = randomizer.nextFloat()
        val aby2 = randomizer.nextFloat()
        val abz2 = randomizer.nextFloat()
        val timestamp2 = System.nanoTime()

        val attitudeMeasurement2 = AttitudeSensorMeasurement(
            attitude2,
            headingAccuracy2,
            timestamp2,
            SensorAccuracy.HIGH,
            AttitudeSensorType.ABSOLUTE_ATTITUDE,
            SensorCoordinateSystem.ENU
        )
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
            SensorCoordinateSystem.ENU
        )

        val syncedMeasurement2 = AttitudeAndAccelerometerSyncedSensorMeasurement(
            attitudeMeasurement2,
            accelerometerMeasurement2,
            timestamp2
        )

        assertFalse(syncedMeasurement1 == syncedMeasurement2)
        assertNotEquals(syncedMeasurement1.hashCode(), syncedMeasurement2.hashCode())
    }

    private companion object {
        const val MIN_DEGREES = -90.0

        const val MAX_DEGREES = 90.0
    }
}