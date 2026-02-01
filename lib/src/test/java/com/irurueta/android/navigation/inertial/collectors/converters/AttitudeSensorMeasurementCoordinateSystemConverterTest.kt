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

import com.irurueta.android.navigation.inertial.collectors.measurements.AttitudeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AttitudeSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorCoordinateSystem
import com.irurueta.geometry.Quaternion
import com.irurueta.statistics.UniformRandomizer
import org.junit.Assert.assertEquals
import org.junit.Assert.assertNull
import org.junit.Assert.assertThrows
import org.junit.Test

class AttitudeSensorMeasurementCoordinateSystemConverterTest {

    @Test
    fun toNedOrThrow_whenAttitudeSensorMeasurementWithoutOptionalValues_returnsExpectedValues() {
        val enuQ = getQuaternion()
        val timestamp = System.nanoTime()

        val enuMeasurement = AttitudeSensorMeasurement(
            attitude = enuQ,
            timestamp = timestamp,
            sensorType = AttitudeSensorType.ABSOLUTE_ATTITUDE,
            sensorCoordinateSystem = SensorCoordinateSystem.ENU
        )

        val nedMeasurement1 = AttitudeSensorMeasurement()
        AttitudeSensorMeasurementCoordinateSystemConverter.toNedOrThrow(
            enuMeasurement,
            nedMeasurement1
        )
        val nedMeasurement2 = AttitudeSensorMeasurementCoordinateSystemConverter.toNedOrThrow(
            enuMeasurement
        )

        // check
        val nedQ = QuaternionCoordinateSystemConverter.convertAndReturnNew(enuQ)
        assertEquals(nedQ, nedMeasurement1.attitude)
        assertNull(nedMeasurement1.headingAccuracy)
        assertEquals(enuMeasurement.timestamp, nedMeasurement1.timestamp)
        assertNull(nedMeasurement1.accuracy)
        assertEquals(
            AttitudeSensorType.ABSOLUTE_ATTITUDE,
            nedMeasurement1.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.NED,
            nedMeasurement1.sensorCoordinateSystem
        )
        assertEquals(nedMeasurement1, nedMeasurement2)
    }

    @Test
    fun toNedOrThrow_whenAttitudeSensorMeasurementWithAllValues_returnsExpectedValues() {
        val enuQ = getQuaternion()
        val timestamp = System.nanoTime()

        val enuMeasurement = AttitudeSensorMeasurement(
            attitude = enuQ,
            headingAccuracy = HEADING_ACCURACY,
            timestamp = timestamp,
            accuracy = SensorAccuracy.HIGH,
            sensorType = AttitudeSensorType.ABSOLUTE_ATTITUDE,
            sensorCoordinateSystem = SensorCoordinateSystem.ENU
        )

        val nedMeasurement1 = AttitudeSensorMeasurement()
        AttitudeSensorMeasurementCoordinateSystemConverter.toNedOrThrow(
            enuMeasurement,
            nedMeasurement1
        )
        val nedMeasurement2 = AttitudeSensorMeasurementCoordinateSystemConverter.toNedOrThrow(
            enuMeasurement
        )

        // check
        val nedQ = QuaternionCoordinateSystemConverter.convertAndReturnNew(enuQ)
        assertEquals(nedQ, nedMeasurement1.attitude)
        assertEquals(HEADING_ACCURACY, nedMeasurement1.headingAccuracy)
        assertEquals(enuMeasurement.timestamp, nedMeasurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, nedMeasurement1.accuracy)
        assertEquals(
            AttitudeSensorType.ABSOLUTE_ATTITUDE,
            nedMeasurement1.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.NED,
            nedMeasurement1.sensorCoordinateSystem
        )
        assertEquals(nedMeasurement1, nedMeasurement2)
    }

    @Test
    fun toNedOrThrow_whenAttitudeInvalidCoordinateSystem_throwsIllegalArgumentException() {
        val measurement = AttitudeSensorMeasurement(
            sensorCoordinateSystem = SensorCoordinateSystem.NED
        )

        assertThrows(IllegalArgumentException::class.java) {
            AttitudeSensorMeasurementCoordinateSystemConverter.toNedOrThrow(
                measurement,
                AttitudeSensorMeasurement()
            )
        }
        assertThrows(IllegalArgumentException::class.java) {
            AttitudeSensorMeasurementCoordinateSystemConverter.toNedOrThrow(measurement)
        }
    }

    @Test
    fun toEnuOrThrow_whenAttitudeSensorMeasurementWithoutOptionalValues_returnsExpectedValues() {
        val nedQ = getQuaternion()
        val timestamp = System.nanoTime()

        val nedMeasurement = AttitudeSensorMeasurement(
            attitude = nedQ,
            timestamp = timestamp,
            sensorType = AttitudeSensorType.ABSOLUTE_ATTITUDE,
            sensorCoordinateSystem = SensorCoordinateSystem.NED
        )

        val enuMeasurement1 = AttitudeSensorMeasurement()
        AttitudeSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(
            nedMeasurement,
            enuMeasurement1
        )
        val enuMeasurement2 = AttitudeSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(
            nedMeasurement
        )

        // check
        val enuQ = QuaternionCoordinateSystemConverter.convertAndReturnNew(nedQ)
        assertEquals(enuQ, enuMeasurement1.attitude)
        assertNull(enuMeasurement1.headingAccuracy)
        assertEquals(nedMeasurement.timestamp, enuMeasurement1.timestamp)
        assertNull(enuMeasurement1.accuracy)
        assertEquals(
            AttitudeSensorType.ABSOLUTE_ATTITUDE,
            enuMeasurement1.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.ENU,
            enuMeasurement1.sensorCoordinateSystem
        )
        assertEquals(enuMeasurement1, enuMeasurement2)
    }

    @Test
    fun toEnuOrThrow_whenAttitudeSensorMeasurementWithAllValues_returnsExpectedValues() {
        val nedQ = getQuaternion()
        val timestamp = System.nanoTime()

        val nedMeasurement = AttitudeSensorMeasurement(
            attitude = nedQ,
            headingAccuracy = HEADING_ACCURACY,
            timestamp = timestamp,
            accuracy = SensorAccuracy.HIGH,
            sensorType = AttitudeSensorType.ABSOLUTE_ATTITUDE,
            sensorCoordinateSystem = SensorCoordinateSystem.NED
        )

        val enuMeasurement1 = AttitudeSensorMeasurement()
        AttitudeSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(
            nedMeasurement,
            enuMeasurement1
        )
        val enuMeasurement2 = AttitudeSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(
            nedMeasurement
        )

        // check
        val enuQ = QuaternionCoordinateSystemConverter.convertAndReturnNew(nedQ)
        assertEquals(enuQ, enuMeasurement1.attitude)
        assertEquals(HEADING_ACCURACY, enuMeasurement1.headingAccuracy)
        assertEquals(nedMeasurement.timestamp, enuMeasurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, enuMeasurement1.accuracy)
        assertEquals(
            AttitudeSensorType.ABSOLUTE_ATTITUDE,
            enuMeasurement1.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.ENU,
            enuMeasurement1.sensorCoordinateSystem
        )
        assertEquals(enuMeasurement1, enuMeasurement2)
    }

    @Test
    fun toEnuOrThrow_whenInvalidAttitudeCoordinateSystem_throwsIllegalArgumentException() {
        val measurement = AttitudeSensorMeasurement(
            sensorCoordinateSystem = SensorCoordinateSystem.ENU
        )

        assertThrows(IllegalArgumentException::class.java) {
            AttitudeSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(
                measurement,
                AttitudeSensorMeasurement()
            )
        }
        assertThrows(IllegalArgumentException::class.java) {
            AttitudeSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(measurement)
        }
    }

    @Test
    fun toNed_whenAlreadyNED_copiesInput() {
        val nedQ = getQuaternion()
        val timestamp = System.nanoTime()

        val nedMeasurement1 = AttitudeSensorMeasurement(
            attitude = nedQ,
            headingAccuracy = HEADING_ACCURACY,
            timestamp = timestamp,
            accuracy = SensorAccuracy.HIGH,
            sensorType = AttitudeSensorType.ABSOLUTE_ATTITUDE,
            sensorCoordinateSystem = SensorCoordinateSystem.NED
        )

        val nedMeasurement2 = AttitudeSensorMeasurement()
        AttitudeSensorMeasurementCoordinateSystemConverter.toNed(
            nedMeasurement1,
            nedMeasurement2
        )
        val nedMeasurement3 = AttitudeSensorMeasurementCoordinateSystemConverter.toNed(
            nedMeasurement1
        )

        // check
        assertEquals(nedMeasurement1, nedMeasurement2)
        assertEquals(nedMeasurement1, nedMeasurement3)
    }

    @Test
    fun toNed_whenENU_returnsExpectedValues() {
        val enuQ = getQuaternion()
        val timestamp = System.nanoTime()

        val enuMeasurement = AttitudeSensorMeasurement(
            attitude = enuQ,
            headingAccuracy = HEADING_ACCURACY,
            timestamp = timestamp,
            accuracy = SensorAccuracy.HIGH,
            sensorType = AttitudeSensorType.ABSOLUTE_ATTITUDE,
            sensorCoordinateSystem = SensorCoordinateSystem.ENU
        )

        val nedMeasurement1 = AttitudeSensorMeasurement()
        AttitudeSensorMeasurementCoordinateSystemConverter.toNed(
            enuMeasurement,
            nedMeasurement1
        )
        val nedMeasurement2 = AttitudeSensorMeasurementCoordinateSystemConverter.toNed(
            enuMeasurement
        )

        // check
        val nedQ = QuaternionCoordinateSystemConverter.convertAndReturnNew(enuQ)
        assertEquals(nedQ, nedMeasurement1.attitude)
        assertEquals(HEADING_ACCURACY, nedMeasurement1.headingAccuracy)
        assertEquals(enuMeasurement.timestamp, nedMeasurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, nedMeasurement1.accuracy)
        assertEquals(
            AttitudeSensorType.ABSOLUTE_ATTITUDE,
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
        val enuQ = getQuaternion()
        val timestamp = System.nanoTime()

        val enuMeasurement1 = AttitudeSensorMeasurement(
            attitude = enuQ,
            headingAccuracy = HEADING_ACCURACY,
            timestamp = timestamp,
            accuracy = SensorAccuracy.HIGH,
            sensorType = AttitudeSensorType.ABSOLUTE_ATTITUDE,
            sensorCoordinateSystem = SensorCoordinateSystem.ENU
        )

        val enuMeasurement2 = AttitudeSensorMeasurement()
        AttitudeSensorMeasurementCoordinateSystemConverter.toEnu(
            enuMeasurement1,
            enuMeasurement2
        )
        val enuMeasurement3 = AttitudeSensorMeasurementCoordinateSystemConverter.toEnu(
            enuMeasurement1
        )

        // check
        assertEquals(enuMeasurement1, enuMeasurement2)
        assertEquals(enuMeasurement1, enuMeasurement3)
    }

    @Test
    fun toEnu_whenNED_returnsExpectedValues() {
        val nedQ = getQuaternion()
        val timestamp = System.nanoTime()

        val nedMeasurement = AttitudeSensorMeasurement(
            attitude = nedQ,
            headingAccuracy = HEADING_ACCURACY,
            timestamp = timestamp,
            accuracy = SensorAccuracy.HIGH,
            sensorType = AttitudeSensorType.ABSOLUTE_ATTITUDE,
            sensorCoordinateSystem = SensorCoordinateSystem.NED
        )

        val enuMeasurement1 = AttitudeSensorMeasurement()
        AttitudeSensorMeasurementCoordinateSystemConverter.toEnu(
            nedMeasurement,
            enuMeasurement1
        )
        val enuMeasurement2 = AttitudeSensorMeasurementCoordinateSystemConverter.toEnu(
            nedMeasurement
        )

        // check
        val enuQ = QuaternionCoordinateSystemConverter.convertAndReturnNew(nedQ)
        assertEquals(enuQ, enuMeasurement1.attitude)
        assertEquals(HEADING_ACCURACY, enuMeasurement1.headingAccuracy)
        assertEquals(nedMeasurement.timestamp, enuMeasurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, enuMeasurement1.accuracy)
        assertEquals(
            AttitudeSensorType.ABSOLUTE_ATTITUDE,
            enuMeasurement1.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.ENU,
            enuMeasurement1.sensorCoordinateSystem
        )
        assertEquals(enuMeasurement1, enuMeasurement2)
    }

    private companion object {
        const val MIN_DEGREES = -45.0
        const val MAX_DEGREES = 45.0

        const val HEADING_ACCURACY = (Math.PI / 2.0).toFloat()

        fun getQuaternion(): Quaternion {
            val randomizer = UniformRandomizer()
            val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
            val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
            val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
            return Quaternion(roll, pitch, yaw)
        }
    }
}