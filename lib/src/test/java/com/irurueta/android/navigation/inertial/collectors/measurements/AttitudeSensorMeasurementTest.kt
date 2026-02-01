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

package com.irurueta.android.navigation.inertial.collectors.measurements

import com.irurueta.android.navigation.inertial.collectors.converters.AttitudeSensorMeasurementCoordinateSystemConverter
import com.irurueta.geometry.Quaternion
import com.irurueta.statistics.UniformRandomizer
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotEquals
import org.junit.Assert.assertNotSame
import org.junit.Assert.assertNull
import org.junit.Assert.assertSame
import org.junit.Assert.assertThrows
import org.junit.Assert.assertTrue
import org.junit.Test

class AttitudeSensorMeasurementTest {

    @Test
    fun constructor_whenDefault_setsExpectedValues() {
        val measurement = AttitudeSensorMeasurement()

        // check
        assertEquals(Quaternion(), measurement.attitude)
        assertNull(measurement.headingAccuracy)
        assertEquals(0L, measurement.timestamp)
        assertNull(measurement.accuracy)
        assertEquals(AttitudeSensorType.ABSOLUTE_ATTITUDE, measurement.sensorType)
    }

    @Test
    fun constructor_withRequiredProperties_setsExpectedValues() {
        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        val timestamp = System.nanoTime()

        val measurement =
            AttitudeSensorMeasurement(
                attitude,
                null,
                timestamp,
                null,
                AttitudeSensorType.RELATIVE_ATTITUDE
            )

        // check
        assertSame(attitude, measurement.attitude)
        assertNull(measurement.headingAccuracy)
        assertEquals(timestamp, measurement.timestamp)
        assertNull(measurement.accuracy)
        assertEquals(AttitudeSensorType.RELATIVE_ATTITUDE, measurement.sensorType)
    }

    @Test
    fun constructor_withAllProperties_setsExpectedValues() {
        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val timestamp = System.nanoTime()

        val measurement =
            AttitudeSensorMeasurement(
                attitude,
                headingAccuracy,
                timestamp,
                SensorAccuracy.HIGH,
                AttitudeSensorType.RELATIVE_ATTITUDE
            )

        // check
        assertSame(attitude, measurement.attitude)
        assertEquals(headingAccuracy, measurement.headingAccuracy)
        assertEquals(timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
        assertEquals(AttitudeSensorType.RELATIVE_ATTITUDE, measurement.sensorType)
    }

    @Test
    fun constructor_whenCopy_setsExpectedValues() {
        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val timestamp = System.nanoTime()

        val measurement1 =
            AttitudeSensorMeasurement(
                attitude,
                headingAccuracy,
                timestamp,
                SensorAccuracy.HIGH,
                AttitudeSensorType.ABSOLUTE_ATTITUDE
            )

        val measurement2 = AttitudeSensorMeasurement(measurement1)

        // check
        assertSame(attitude, measurement1.attitude)
        assertEquals(headingAccuracy, measurement1.headingAccuracy)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement1.accuracy)
        assertEquals(AttitudeSensorType.ABSOLUTE_ATTITUDE, measurement1.sensorType)

        assertEquals(measurement1.attitude, measurement2.attitude)
        assertEquals(measurement1.headingAccuracy, measurement2.headingAccuracy)
        assertEquals(measurement1.timestamp, measurement2.timestamp)
        assertEquals(measurement1.accuracy, measurement2.accuracy)
        assertEquals(measurement1.sensorType, measurement2.sensorType)
    }

    @Test
    fun attitude_setsExpectedValue() {
        val measurement = AttitudeSensorMeasurement()

        // check default value
        assertEquals(Quaternion(), measurement.attitude)

        // set new value
        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        measurement.attitude = attitude

        // check
        assertEquals(attitude, measurement.attitude)
    }

    @Test
    fun headingAccuracy_setsExpectedValue() {
        val measurement = AttitudeSensorMeasurement()

        // check default value
        assertNull(measurement.headingAccuracy)

        // set new value
        val randomizer = UniformRandomizer()
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        measurement.headingAccuracy = headingAccuracy

        // check
        assertEquals(headingAccuracy, measurement.headingAccuracy)
    }

    @Test
    fun timestamp_setsExpectedValue() {
        val measurement = AttitudeSensorMeasurement()

        // check default value
        assertEquals(0L, measurement.timestamp)

        // set new value
        val timestamp = System.nanoTime()
        measurement.timestamp = timestamp

        // check
        assertEquals(timestamp, measurement.timestamp)
    }

    @Test
    fun accuracy_setsExpectedValue() {
        val measurement = AttitudeSensorMeasurement()

        // check default value
        assertNull(measurement.accuracy)

        // set new value
        measurement.accuracy = SensorAccuracy.HIGH

        // check
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)

        // set to null
        measurement.accuracy = null

        // check
        @Suppress("KotlinConstantConditions")
        (assertNull(measurement.accuracy))
    }

    @Test
    fun sensorType_setsExpectedValue() {
        val measurement = AttitudeSensorMeasurement()

        // check default value
        assertEquals(AttitudeSensorType.ABSOLUTE_ATTITUDE, measurement.sensorType)

        // set new value
        measurement.sensorType = AttitudeSensorType.RELATIVE_ATTITUDE

        // check
        assertEquals(AttitudeSensorType.RELATIVE_ATTITUDE, measurement.sensorType)
    }

    @Test
    fun copyFrom_makesExpectedCopy() {
        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val timestamp = System.nanoTime()

        val measurement1 =
            AttitudeSensorMeasurement(
                attitude,
                headingAccuracy,
                timestamp,
                SensorAccuracy.HIGH,
                AttitudeSensorType.ABSOLUTE_ATTITUDE
            )

        val measurement2 = AttitudeSensorMeasurement()
        measurement2.copyFrom(measurement1)

        // check
        assertSame(attitude, measurement1.attitude)
        assertEquals(headingAccuracy, measurement1.headingAccuracy)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement1.accuracy)
        assertEquals(AttitudeSensorType.ABSOLUTE_ATTITUDE, measurement1.sensorType)

        assertEquals(measurement1.attitude, measurement2.attitude)
        assertEquals(measurement1.headingAccuracy, measurement2.headingAccuracy)
        assertEquals(measurement1.timestamp, measurement2.timestamp)
        assertEquals(measurement1.accuracy, measurement2.accuracy)
        assertEquals(measurement1.sensorType, measurement2.sensorType)
    }

    @Test
    fun copyTo_makesExpectedCopy() {
        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val timestamp = System.nanoTime()

        val measurement1 =
            AttitudeSensorMeasurement(
                attitude,
                headingAccuracy,
                timestamp,
                SensorAccuracy.HIGH,
                AttitudeSensorType.RELATIVE_ATTITUDE
            )

        val measurement2 = AttitudeSensorMeasurement()
        measurement1.copyTo(measurement2)

        // check
        assertSame(attitude, measurement1.attitude)
        assertEquals(headingAccuracy, measurement1.headingAccuracy)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement1.accuracy)
        assertEquals(AttitudeSensorType.RELATIVE_ATTITUDE, measurement1.sensorType)

        assertEquals(measurement1.attitude, measurement2.attitude)
        assertEquals(measurement1.headingAccuracy, measurement2.headingAccuracy)
        assertEquals(measurement1.timestamp, measurement2.timestamp)
        assertEquals(measurement1.accuracy, measurement2.accuracy)
        assertEquals(measurement1.sensorType, measurement2.sensorType)
    }

    @Test
    fun copy_makesExpectedCopy() {
        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val timestamp = System.nanoTime()

        val measurement1 =
            AttitudeSensorMeasurement(
                attitude,
                headingAccuracy,
                timestamp,
                SensorAccuracy.HIGH,
                AttitudeSensorType.ABSOLUTE_ATTITUDE
            )

        val measurement2 = measurement1.copy()

        // check
        assertSame(attitude, measurement1.attitude)
        assertEquals(headingAccuracy, measurement1.headingAccuracy)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement1.accuracy)
        assertEquals(AttitudeSensorType.ABSOLUTE_ATTITUDE, measurement1.sensorType)

        assertEquals(measurement1.attitude, measurement2.attitude)
        assertEquals(measurement1.headingAccuracy, measurement2.headingAccuracy)
        assertEquals(measurement1.timestamp, measurement2.timestamp)
        assertEquals(measurement1.accuracy, measurement2.accuracy)
        assertEquals(measurement1.sensorType, measurement2.sensorType)
    }

    @Test
    fun toNedOrThrow_whenENU_returnsExpectedValues() {
        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val timestamp = System.nanoTime()

        val measurement =
            AttitudeSensorMeasurement(
                attitude,
                headingAccuracy,
                timestamp,
                SensorAccuracy.HIGH,
                AttitudeSensorType.ABSOLUTE_ATTITUDE
            )

        // copy
        val nedMeasurement1 = measurement.toNedOrThrow()
        val nedMeasurement2 = AttitudeSensorMeasurement()
        measurement.toNedOrThrow(nedMeasurement2)

        val nedMeasurement3 = AttitudeSensorMeasurementCoordinateSystemConverter.toNedOrThrow(
            measurement)

        // check
        assertEquals(nedMeasurement1, nedMeasurement2)
        assertEquals(nedMeasurement1, nedMeasurement3)
    }

    @Test
    fun toNedOrThrow_whenNED_throwsIllegalArgumentException() {
        val measurement = AttitudeSensorMeasurement(
            sensorCoordinateSystem = SensorCoordinateSystem.NED
        )

        assertThrows(IllegalArgumentException::class.java) {
            measurement.toNedOrThrow(measurement)
        }
        assertThrows(IllegalArgumentException::class.java) {
            measurement.toNedOrThrow()
        }
    }

    @Test
    fun toEnuOrThrow_whenNED_returnsExpectedValues() {
        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val timestamp = System.nanoTime()

        val measurement =
            AttitudeSensorMeasurement(
                attitude,
                headingAccuracy,
                timestamp,
                SensorAccuracy.HIGH,
                AttitudeSensorType.ABSOLUTE_ATTITUDE,
                SensorCoordinateSystem.NED
            )

        // copy
        val enuMeasurement1 = measurement.toEnuOrThrow()
        val enuMeasurement2 = AttitudeSensorMeasurement()
        measurement.toEnuOrThrow(enuMeasurement2)

        val enuMeasurement3 = AttitudeSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(
            measurement)

        // check
        assertEquals(enuMeasurement1, enuMeasurement2)
        assertEquals(enuMeasurement1, enuMeasurement3)
    }

    @Test
    fun toEnuOrThrow_whenENU_throwsIllegalArgumentException() {
        val measurement = AttitudeSensorMeasurement(
            sensorCoordinateSystem = SensorCoordinateSystem.ENU
        )

        assertThrows(IllegalArgumentException::class.java) {
            measurement.toEnuOrThrow(measurement)
        }
        assertThrows(IllegalArgumentException::class.java) {
            measurement.toEnuOrThrow()
        }
    }

    @Test
    fun toNed_whenNED_returnsCopy() {
        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val timestamp = System.nanoTime()

        val measurement =
            AttitudeSensorMeasurement(
                attitude,
                headingAccuracy,
                timestamp,
                SensorAccuracy.HIGH,
                AttitudeSensorType.ABSOLUTE_ATTITUDE,
                SensorCoordinateSystem.NED
            )

        val result1 = measurement.toNed()
        val result2 = AttitudeSensorMeasurement()
        measurement.toNed(result2)

        // check
        assertEquals(measurement, result1)
        assertEquals(measurement, result2)
        assertNotSame(measurement, result1)
        assertNotSame(measurement, result2)
    }

    @Test
    fun toNed_whenENU_returnsConvertedValue() {
        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val timestamp = System.nanoTime()

        val measurement =
            AttitudeSensorMeasurement(
                attitude,
                headingAccuracy,
                timestamp,
                SensorAccuracy.HIGH,
                AttitudeSensorType.ABSOLUTE_ATTITUDE,
                SensorCoordinateSystem.ENU
            )

        val result1 = measurement.toNed()
        val result2 = AttitudeSensorMeasurement()
        measurement.toNed(result2)

        val expected = AttitudeSensorMeasurementCoordinateSystemConverter.toNedOrThrow(measurement)

        // check
        assertEquals(expected, result1)
        assertEquals(expected, result2)
    }

    @Test
    fun toEnu_whenENU_returnsCopy() {
        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val timestamp = System.nanoTime()

        val measurement =
            AttitudeSensorMeasurement(
                attitude,
                headingAccuracy,
                timestamp,
                SensorAccuracy.HIGH,
                AttitudeSensorType.ABSOLUTE_ATTITUDE,
                SensorCoordinateSystem.ENU
            )

        val result1 = measurement.toEnu()
        val result2 = AttitudeSensorMeasurement()
        measurement.toEnu(result2)

        // check
        assertEquals(measurement, result1)
        assertEquals(measurement, result2)
        assertNotSame(measurement, result1)
        assertNotSame(measurement, result2)
    }

    @Test
    fun toEnu_whenNED_returnsConvertedValue() {
        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val timestamp = System.nanoTime()

        val measurement =
            AttitudeSensorMeasurement(
                attitude,
                headingAccuracy,
                timestamp,
                SensorAccuracy.HIGH,
                AttitudeSensorType.ABSOLUTE_ATTITUDE,
                SensorCoordinateSystem.NED
            )

        val result1 = measurement.toEnu()
        val result2 = AttitudeSensorMeasurement()
        measurement.toEnu(result2)

        val expected = AttitudeSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(measurement)

        // check
        assertEquals(expected, result1)
        assertEquals(expected, result2)
    }

    @Test
    fun equals_whenNull_returnsFalse() {
        val measurement = AttitudeSensorMeasurement()
        assertFalse(measurement.equals(null))
    }

    @Test
    fun equals_whenSameInstance_returnsTrue() {
        val measurement = AttitudeSensorMeasurement()
        @Suppress("ReplaceCallWithBinaryOperator")
        assertTrue(measurement.equals(measurement))
    }

    @Test
    fun equals_whenDifferentType_returnsFalse() {
        val measurement = AttitudeSensorMeasurement()
        assertFalse(measurement == Any())
    }

    @Test
    fun equals_whenDifferentContent_returnsFalse() {
        val randomizer = UniformRandomizer()
        val roll1 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch1 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw1 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude1 = Quaternion(roll1, pitch1, yaw1)
        val headingAccuracy1 = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val timestamp1 = System.nanoTime()

        val roll2 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch2 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw2 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude2 = Quaternion(roll2, pitch2, yaw2)
        val headingAccuracy2 = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val timestamp2 = System.nanoTime()

        val measurement1 =
            AttitudeSensorMeasurement(
                attitude1,
                headingAccuracy1,
                timestamp1,
                SensorAccuracy.HIGH,
                AttitudeSensorType.ABSOLUTE_ATTITUDE
            )

        val measurement2 =
            AttitudeSensorMeasurement(
                attitude2,
                headingAccuracy2,
                timestamp2,
                SensorAccuracy.LOW,
                AttitudeSensorType.RELATIVE_ATTITUDE
            )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentSensorAccuracy_returnsFalse() {
        val randomizer = UniformRandomizer()
        val roll1 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch1 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw1 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude1 = Quaternion(roll1, pitch1, yaw1)
        val headingAccuracy1 = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val timestamp = System.nanoTime()

        val roll2 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch2 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw2 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude2 = Quaternion(roll2, pitch2, yaw2)
        val headingAccuracy2 = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()

        val measurement1 =
            AttitudeSensorMeasurement(
                attitude1,
                headingAccuracy1,
                timestamp,
                SensorAccuracy.HIGH,
                AttitudeSensorType.ABSOLUTE_ATTITUDE
            )

        val measurement2 =
            AttitudeSensorMeasurement(
                attitude2,
                headingAccuracy2,
                timestamp,
                SensorAccuracy.LOW,
                AttitudeSensorType.RELATIVE_ATTITUDE
            )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentAttitude_returnsFalse() {
        val randomizer = UniformRandomizer()
        val roll1 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch1 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw1 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude1 = Quaternion(roll1, pitch1, yaw1)
        val headingAccuracy1 = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val timestamp = System.nanoTime()

        val roll2 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch2 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw2 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude2 = Quaternion(roll2, pitch2, yaw2)
        val headingAccuracy2 = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()

        val measurement1 =
            AttitudeSensorMeasurement(
                attitude1,
                headingAccuracy1,
                timestamp,
                SensorAccuracy.HIGH,
                AttitudeSensorType.ABSOLUTE_ATTITUDE
            )

        val measurement2 =
            AttitudeSensorMeasurement(
                attitude2,
                headingAccuracy2,
                timestamp,
                SensorAccuracy.HIGH,
                AttitudeSensorType.RELATIVE_ATTITUDE
            )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentHeadingAccuracy_returnsFalse() {
        val randomizer = UniformRandomizer()
        val roll1 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch1 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw1 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll1, pitch1, yaw1)
        val headingAccuracy1 = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val timestamp = System.nanoTime()

        val headingAccuracy2 = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()

        val measurement1 =
            AttitudeSensorMeasurement(
                attitude,
                headingAccuracy1,
                timestamp,
                SensorAccuracy.HIGH,
                AttitudeSensorType.ABSOLUTE_ATTITUDE
            )

        val measurement2 =
            AttitudeSensorMeasurement(
                attitude,
                headingAccuracy2,
                timestamp,
                SensorAccuracy.HIGH,
                AttitudeSensorType.RELATIVE_ATTITUDE
            )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentTimestamp_returnsFalse() {
        val randomizer = UniformRandomizer()
        val roll1 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch1 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw1 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll1, pitch1, yaw1)
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val timestamp1 = System.nanoTime()

        val timestamp2 = System.nanoTime()

        val measurement1 =
            AttitudeSensorMeasurement(
                attitude,
                headingAccuracy,
                timestamp1,
                SensorAccuracy.HIGH,
                AttitudeSensorType.ABSOLUTE_ATTITUDE
            )

        val measurement2 =
            AttitudeSensorMeasurement(
                attitude,
                headingAccuracy,
                timestamp2,
                SensorAccuracy.HIGH,
                AttitudeSensorType.RELATIVE_ATTITUDE
            )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentSensorType_returnsFalse() {
        val randomizer = UniformRandomizer()
        val roll1 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch1 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw1 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll1, pitch1, yaw1)
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val timestamp = System.nanoTime()

        val measurement1 =
            AttitudeSensorMeasurement(
                attitude,
                headingAccuracy,
                timestamp,
                SensorAccuracy.HIGH,
                AttitudeSensorType.ABSOLUTE_ATTITUDE
            )

        val measurement2 =
            AttitudeSensorMeasurement(
                attitude,
                headingAccuracy,
                timestamp,
                SensorAccuracy.HIGH,
                AttitudeSensorType.RELATIVE_ATTITUDE
            )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenEqualContent_returnsTrue() {
        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val timestamp = System.nanoTime()

        val measurement1 =
            AttitudeSensorMeasurement(
                attitude,
                headingAccuracy,
                timestamp,
                SensorAccuracy.HIGH,
                AttitudeSensorType.ABSOLUTE_ATTITUDE
            )

        val measurement2 =
            AttitudeSensorMeasurement(
                Quaternion(attitude),
                headingAccuracy,
                timestamp,
                SensorAccuracy.HIGH,
                AttitudeSensorType.ABSOLUTE_ATTITUDE
            )

        assertTrue(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun hashCode_whenEqualObjects_returnsSameValue() {
        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val timestamp = System.nanoTime()

        val measurement1 =
            AttitudeSensorMeasurement(
                attitude,
                headingAccuracy,
                timestamp,
                SensorAccuracy.HIGH,
                AttitudeSensorType.ABSOLUTE_ATTITUDE
            )

        val measurement2 =
            AttitudeSensorMeasurement(
                Quaternion(attitude),
                headingAccuracy,
                timestamp,
                SensorAccuracy.HIGH,
                AttitudeSensorType.ABSOLUTE_ATTITUDE
            )

        assertEquals(measurement1.hashCode(), measurement2.hashCode())
    }

    @Test
    fun hashCode_whenDifferentObjects_returnsDifferentValue() {
        val randomizer = UniformRandomizer()
        val roll1 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch1 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw1 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude1 = Quaternion(roll1, pitch1, yaw1)
        val headingAccuracy1 = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val timestamp1 = System.nanoTime()

        val roll2 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch2 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw2 = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude2 = Quaternion(roll2, pitch2, yaw2)
        val headingAccuracy2 = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val timestamp2 = System.nanoTime()

        val measurement1 =
            AttitudeSensorMeasurement(
                attitude1,
                headingAccuracy1,
                timestamp1,
                SensorAccuracy.HIGH,
                AttitudeSensorType.ABSOLUTE_ATTITUDE
            )

        val measurement2 =
            AttitudeSensorMeasurement(
                attitude2,
                headingAccuracy2,
                timestamp2,
                SensorAccuracy.LOW,
                AttitudeSensorType.RELATIVE_ATTITUDE
            )

        assertNotEquals(measurement1.hashCode(), measurement2.hashCode())
    }

    private companion object {
        const val MIN_DEGREES = -90.0

        const val MAX_DEGREES = 90.0
    }
}