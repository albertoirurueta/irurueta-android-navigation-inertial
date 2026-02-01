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

import com.irurueta.android.navigation.inertial.collectors.converters.GyroscopeSensorMeasurementCoordinateSystemConverter
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.AngularSpeedUnit
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotEquals
import org.junit.Assert.assertNotSame
import org.junit.Assert.assertNull
import org.junit.Assert.assertThrows
import org.junit.Test
import kotlin.math.sqrt

class GyroscopeSensorMeasurementTest {

    @Test
    fun constructor_whenDefault_setsExpectedValues() {
        val measurement = GyroscopeSensorMeasurement()

        // check
        assertEquals(0.0f, measurement.wx, 0.0f)
        assertEquals(0.0f, measurement.wy, 0.0f)
        assertEquals(0.0f, measurement.wz, 0.0f)
        assertNull(measurement.bx)
        assertNull(measurement.by)
        assertNull(measurement.bz)
        assertEquals(0L, measurement.timestamp)
        assertNull(measurement.accuracy)
        assertEquals(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            measurement.sensorType)
        assertEquals(SensorCoordinateSystem.ENU,
            measurement.sensorCoordinateSystem)
    }

    @Test
    fun constructor_withRequiredProperties_setsExpectedValues() {
        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement =
            GyroscopeSensorMeasurement(
                wx,
                wy,
                wz,
                null,
                null,
                null,
                timestamp,
                null,
                GyroscopeSensorType.GYROSCOPE,
                SensorCoordinateSystem.NED
            )

        // check
        assertEquals(wx, measurement.wx, 0.0f)
        assertEquals(wy, measurement.wy, 0.0f)
        assertEquals(wz, measurement.wz, 0.0f)
        assertNull(measurement.bx)
        assertNull(measurement.by)
        assertNull(measurement.bz)
        assertEquals(timestamp, measurement.timestamp)
        assertNull(measurement.accuracy)
        assertEquals(GyroscopeSensorType.GYROSCOPE, measurement.sensorType)
        assertEquals(SensorCoordinateSystem.NED,
            measurement.sensorCoordinateSystem)
    }

    @Test
    fun constructor_withAllProperties_setsExpectedValues() {
        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )

        // check
        assertEquals(wx, measurement.wx, 0.0f)
        assertEquals(wy, measurement.wy, 0.0f)
        assertEquals(wz, measurement.wz, 0.0f)
        assertEquals(bx, measurement.bx)
        assertEquals(by, measurement.by)
        assertEquals(bz, measurement.bz)
        assertEquals(timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
        assertEquals(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            measurement.sensorType)
        assertEquals(SensorCoordinateSystem.NED,
            measurement.sensorCoordinateSystem)
    }

    @Test
    fun constructor_whenCopy_setsExpectedValues() {
        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement1 = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )

        val measurement2 = GyroscopeSensorMeasurement(measurement1)

        // check
        assertEquals(wx, measurement1.wx, 0.0f)
        assertEquals(wy, measurement1.wy, 0.0f)
        assertEquals(wz, measurement1.wz, 0.0f)
        assertEquals(bx, measurement1.bx)
        assertEquals(by, measurement1.by)
        assertEquals(bz, measurement1.bz)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement1.accuracy)
        assertEquals(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            measurement1.sensorType)
        assertEquals(SensorCoordinateSystem.NED,
            measurement1.sensorCoordinateSystem)

        assertEquals(measurement1.wx, measurement2.wx, 0.0f)
        assertEquals(measurement1.wy, measurement2.wy, 0.0f)
        assertEquals(measurement1.wz, measurement2.wz, 0.0f)
        assertEquals(measurement1.bx, measurement2.bx)
        assertEquals(measurement1.by, measurement2.by)
        assertEquals(measurement1.bz, measurement2.bz)
        assertEquals(measurement1.timestamp, measurement2.timestamp)
        assertEquals(measurement1.accuracy, measurement2.accuracy)
        assertEquals(measurement1.sensorType, measurement2.sensorType)
        assertEquals(measurement1.sensorCoordinateSystem,
            measurement2.sensorCoordinateSystem)
    }

    @Test
    fun wx_setsExpectedValue() {
        val measurement = GyroscopeSensorMeasurement()

        // check default value
        assertEquals(0.0f, measurement.wx, 0.0f)

        // set new value
        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        measurement.wx = wx

        // check
        assertEquals(wx, measurement.wx, 0.0f)
    }

    @Test
    fun wy_setsExpectedValue() {
        val measurement = GyroscopeSensorMeasurement()

        // check default value
        assertEquals(0.0f, measurement.wy, 0.0f)

        // set new value
        val randomizer = UniformRandomizer()
        val wy = randomizer.nextFloat()
        measurement.wy = wy

        // check
        assertEquals(wy, measurement.wy, 0.0f)
    }

    @Test
    fun wz_setsExpectedValue() {
        val measurement = GyroscopeSensorMeasurement()

        // check default value
        assertEquals(0.0f, measurement.wz, 0.0f)

        // set new value
        val randomizer = UniformRandomizer()
        val wz = randomizer.nextFloat()
        measurement.wz = wz

        // check
        assertEquals(wz, measurement.wz, 0.0f)
    }

    @Test
    fun bx_setsExpectedValue() {
        val measurement = GyroscopeSensorMeasurement()

        // check default value
        assertNull(measurement.bx)

        // set new value
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        measurement.bx = bx

        // check
        assertEquals(bx, measurement.bx)

        // set to null
        measurement.bx = null

        // check
        @Suppress("KotlinConstantConditions")
        (assertNull(measurement.bx))
    }

    @Test
    fun by_setsExpectedValue() {
        val measurement = GyroscopeSensorMeasurement()

        // check default value
        assertNull(measurement.by)

        // set new value
        val randomizer = UniformRandomizer()
        val by = randomizer.nextFloat()
        measurement.by = by

        // check
        assertEquals(by, measurement.by)

        // set to null
        measurement.by = null

        // check
        @Suppress("KotlinConstantConditions")
        (assertNull(measurement.by))
    }

    @Test
    fun bz_setsExpectedValue() {
        val measurement = GyroscopeSensorMeasurement()

        // check default value
        assertNull(measurement.bz)

        // set new value
        val randomizer = UniformRandomizer()
        val bz = randomizer.nextFloat()
        measurement.bz = bz

        // check
        assertEquals(bz, measurement.bz)

        // set to null
        measurement.bz = null

        // check
        @Suppress("KotlinConstantConditions")
        (assertNull(measurement.bz))
    }

    @Test
    fun timestamp_setsExpectedValue() {
        val measurement = GyroscopeSensorMeasurement()

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
        val measurement = GyroscopeSensorMeasurement()

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
        val measurement = GyroscopeSensorMeasurement()

        // check default value
        assertEquals(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED, measurement.sensorType)

        // set new value
        measurement.sensorType = GyroscopeSensorType.GYROSCOPE

        // check
        assertEquals(GyroscopeSensorType.GYROSCOPE, measurement.sensorType)
    }

    @Test
    fun sensorCoordinateSystem_setsExpectedValue() {
        val measurement = GyroscopeSensorMeasurement()

        // check default value
        assertEquals(SensorCoordinateSystem.ENU,
            measurement.sensorCoordinateSystem)

        // set new value
        measurement.sensorCoordinateSystem = SensorCoordinateSystem.NED

        // check
        assertEquals(SensorCoordinateSystem.NED,
            measurement.sensorCoordinateSystem)
    }

    @Test
    fun copyFrom_makesExpectedCopy() {
        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement1 = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE,
            SensorCoordinateSystem.NED
        )

        val measurement2 = GyroscopeSensorMeasurement()
        measurement2.copyFrom(measurement1)

        // check
        assertEquals(wx, measurement1.wx, 0.0f)
        assertEquals(wy, measurement1.wy, 0.0f)
        assertEquals(wz, measurement1.wz, 0.0f)
        assertEquals(bx, measurement1.bx)
        assertEquals(by, measurement1.by)
        assertEquals(bz, measurement1.bz)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement1.accuracy)
        assertEquals(GyroscopeSensorType.GYROSCOPE, measurement1.sensorType)
        assertEquals(SensorCoordinateSystem.NED,
            measurement1.sensorCoordinateSystem)

        assertEquals(measurement1.wx, measurement2.wx, 0.0f)
        assertEquals(measurement1.wy, measurement2.wy, 0.0f)
        assertEquals(measurement1.wz, measurement2.wz, 0.0f)
        assertEquals(measurement1.bx, measurement2.bx)
        assertEquals(measurement1.by, measurement2.by)
        assertEquals(measurement1.bz, measurement2.bz)
        assertEquals(measurement1.timestamp, measurement2.timestamp)
        assertEquals(measurement1.accuracy, measurement2.accuracy)
        assertEquals(measurement1.sensorType, measurement2.sensorType)
        assertEquals(measurement1.sensorCoordinateSystem,
            measurement2.sensorCoordinateSystem)
    }

    @Test
    fun copyTo_makesExpectedCopy() {
        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement1 = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE,
            SensorCoordinateSystem.NED
        )

        val measurement2 = GyroscopeSensorMeasurement()
        measurement1.copyTo(measurement2)

        // check
        assertEquals(wx, measurement1.wx, 0.0f)
        assertEquals(wy, measurement1.wy, 0.0f)
        assertEquals(wz, measurement1.wz, 0.0f)
        assertEquals(bx, measurement1.bx)
        assertEquals(by, measurement1.by)
        assertEquals(bz, measurement1.bz)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement1.accuracy)
        assertEquals(GyroscopeSensorType.GYROSCOPE, measurement1.sensorType)
        assertEquals(SensorCoordinateSystem.NED,
            measurement1.sensorCoordinateSystem)

        assertEquals(measurement1.wx, measurement2.wx, 0.0f)
        assertEquals(measurement1.wy, measurement2.wy, 0.0f)
        assertEquals(measurement1.wz, measurement2.wz, 0.0f)
        assertEquals(measurement1.bx, measurement2.bx)
        assertEquals(measurement1.by, measurement2.by)
        assertEquals(measurement1.bz, measurement2.bz)
        assertEquals(measurement1.timestamp, measurement2.timestamp)
        assertEquals(measurement1.accuracy, measurement2.accuracy)
        assertEquals(measurement1.sensorType, measurement2.sensorType)
        assertEquals(measurement1.sensorCoordinateSystem,
            measurement2.sensorCoordinateSystem)
    }

    @Test
    fun copy_makesExpectedCopy() {
        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement1 = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE,
            SensorCoordinateSystem.NED
        )

        val measurement2 = measurement1.copy()

        // check
        assertEquals(wx, measurement1.wx, 0.0f)
        assertEquals(wy, measurement1.wy, 0.0f)
        assertEquals(wz, measurement1.wz, 0.0f)
        assertEquals(bx, measurement1.bx)
        assertEquals(by, measurement1.by)
        assertEquals(bz, measurement1.bz)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement1.accuracy)
        assertEquals(GyroscopeSensorType.GYROSCOPE, measurement1.sensorType)
        assertEquals(SensorCoordinateSystem.NED,
            measurement1.sensorCoordinateSystem)

        assertEquals(measurement1.wx, measurement2.wx, 0.0f)
        assertEquals(measurement1.wy, measurement2.wy, 0.0f)
        assertEquals(measurement1.wz, measurement2.wz, 0.0f)
        assertEquals(measurement1.bx, measurement2.bx)
        assertEquals(measurement1.by, measurement2.by)
        assertEquals(measurement1.bz, measurement2.bz)
        assertEquals(measurement1.timestamp, measurement2.timestamp)
        assertEquals(measurement1.accuracy, measurement2.accuracy)
        assertEquals(measurement1.sensorType, measurement2.sensorType)
        assertEquals(measurement1.sensorCoordinateSystem,
            measurement2.sensorCoordinateSystem)
    }

    @Test
    fun toNedOrThrow_whenENU_returnsExpectedValues() {
        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        val nedMeasurement1 = measurement.toNedOrThrow()
        val nedMeasurement2 = GyroscopeSensorMeasurement()
        measurement.toNedOrThrow(nedMeasurement2)

        val nedMeasurement3 = GyroscopeSensorMeasurementCoordinateSystemConverter.toNedOrThrow(
            measurement)

        // check
        assertEquals(nedMeasurement1, nedMeasurement2)
        assertEquals(nedMeasurement1, nedMeasurement3)
    }

    @Test
    fun toNedOrThrow_whenNED_throwsIllegalArgumentException() {
        val measurement = GyroscopeSensorMeasurement(
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
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )

        val enuMeasurement1 = measurement.toEnuOrThrow()
        val enuMeasurement2 = GyroscopeSensorMeasurement()
        measurement.toEnuOrThrow(enuMeasurement2)

        val enuMeasurement3 = GyroscopeSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(
            measurement)

        // check
        assertEquals(enuMeasurement1, enuMeasurement2)
        assertEquals(enuMeasurement1, enuMeasurement3)
    }

    @Test
    fun toEnuOrThrow_whenENU_throwsIllegalArgumentException() {
        val measurement = GyroscopeSensorMeasurement(
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
    fun toNed_whenNED_copiesValues() {
        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )

        val nedMeasurement1 = measurement.toNed()
        val nedMeasurement2 = GyroscopeSensorMeasurement()
        measurement.toNed(nedMeasurement2)
        val nedMeasurement3 = GyroscopeSensorMeasurement()
        GyroscopeSensorMeasurementCoordinateSystemConverter.toNed(
            measurement, nedMeasurement3)

        // check
        assertEquals(measurement, nedMeasurement1)
        assertEquals(measurement, nedMeasurement2)
        assertEquals(measurement, nedMeasurement3)
    }

    @Test
    fun toNed_whenENU_convertsToNED() {
        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        val result1 = measurement.toNed()
        val result2 = GyroscopeSensorMeasurement()
        GyroscopeSensorMeasurementCoordinateSystemConverter.toNed(
            measurement, result2)

        val expected = GyroscopeSensorMeasurementCoordinateSystemConverter.toNedOrThrow(
            measurement)

        // check
        assertEquals(expected, result1)
        assertEquals(expected, result2)
    }

    @Test
    fun toEnu_whenENU_copiesValues() {
        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        val enuMeasurement1 = measurement.toEnu()
        val enuMeasurement2 = GyroscopeSensorMeasurement()
        measurement.toEnu(enuMeasurement2)
        val enuMeasurement3 = GyroscopeSensorMeasurement()
        GyroscopeSensorMeasurementCoordinateSystemConverter.toEnu(
            measurement, enuMeasurement3)

        // check
        assertEquals(measurement, enuMeasurement1)
        assertEquals(measurement, enuMeasurement2)
        assertEquals(measurement, enuMeasurement3)
    }

    @Test
    fun toEnu_whenNED_convertsToENU() {
        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )

        val result1 = measurement.toEnu()
        val result2 = GyroscopeSensorMeasurement()
        measurement.toEnu(result2)

        val expected = GyroscopeSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(
            measurement)

        // check
        assertEquals(expected, result1)
        assertEquals(expected, result2)
    }

    @Test
    fun toTriad_whenNoBias_returnsExpectedValues() {
        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            null,
            null,
            null,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        val triad1 = measurement.toTriad()
        val triad2 = AngularSpeedTriad()
        measurement.toTriad(triad2)

        // check
        assertEquals(wx.toDouble(), triad1.valueX, 0.0)
        assertEquals(wy.toDouble(), triad1.valueY, 0.0)
        assertEquals(wz.toDouble(), triad1.valueZ, 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad1.unit)
        assertEquals(triad1, triad2)
    }

    @Test
    fun toTriad_whenBias_returnsExpectedValues() {
        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        val triad1 = measurement.toTriad()
        val triad2 = AngularSpeedTriad()
        measurement.toTriad(triad2)

        // check
        assertEquals(wx.toDouble() + bx.toDouble(), triad1.valueX, 0.0)
        assertEquals(wy.toDouble() + by.toDouble(), triad1.valueY, 0.0)
        assertEquals(wz.toDouble() + bz.toDouble(), triad1.valueZ, 0.0)
        assertEquals(AngularSpeedUnit.RADIANS_PER_SECOND, triad1.unit)
        assertEquals(triad1, triad2)
    }

    @Test
    fun toNorm_whenNoBias_returnsExpectedValue() {
        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            null,
            null,
            null,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        val norm = measurement.toNorm()

        // check
        val wx2 = wx.toDouble() * wx.toDouble()
        val wy2 = wy.toDouble() * wy.toDouble()
        val wz2 = wz.toDouble() * wz.toDouble()
        val expected = sqrt(wx2 + wy2 + wz2)
        assertEquals(expected, norm, 0.0)
    }

    @Test
    fun toNorm_whenBias_returnsExpectedValue() {
        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        val norm = measurement.toNorm()

        // check
        val x = wx.toDouble() + bx.toDouble()
        val y = wy.toDouble() + by.toDouble()
        val z = wz.toDouble() + bz.toDouble()
        val expected = sqrt(x * x + y * y + z * z)
        assertEquals(expected, norm, 0.0)
    }

    @Test
    fun equals_whenNull_returnsFalse() {
        val measurement = GyroscopeSensorMeasurement()
        assertFalse(measurement.equals(null))
    }

    @Test
    fun equals_whenSameInstance_returnsTrue() {
        val measurement = GyroscopeSensorMeasurement()
        @Suppress("ReplaceCallWithBinaryOperator")
        assert(measurement.equals(measurement))
    }

    @Test
    fun equals_whenDifferentType_returnsFalse() {
        val measurement = GyroscopeSensorMeasurement()
        assertFalse(measurement == Any())
    }

    @Test
    fun equals_whenDifferentContent_returnsFalse() {
        val randomizer = UniformRandomizer()
        val wx1 = randomizer.nextFloat()
        val wy1 = randomizer.nextFloat()
        val wz1 = randomizer.nextFloat()
        val bx1 = randomizer.nextFloat()
        val by1 = randomizer.nextFloat()
        val bz1 = randomizer.nextFloat()
        val timestamp1 = System.nanoTime()

        val wx2 = randomizer.nextFloat()
        val wy2 = randomizer.nextFloat()
        val wz2 = randomizer.nextFloat()
        val bx2 = randomizer.nextFloat()
        val by2 = randomizer.nextFloat()
        val bz2 = randomizer.nextFloat()
        val timestamp2 = System.nanoTime()

        val measurement1 = GyroscopeSensorMeasurement(
            wx1,
            wy1,
            wz1,
            bx1,
            by1,
            bz1,
            timestamp1,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE,
            SensorCoordinateSystem.NED
        )

        val measurement2 = GyroscopeSensorMeasurement(
            wx2,
            wy2,
            wz2,
            bx2,
            by2,
            bz2,
            timestamp2,
            SensorAccuracy.LOW,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentSensorAccuracy_returnsFalse() {
        val randomizer = UniformRandomizer()
        val wx1 = randomizer.nextFloat()
        val wy1 = randomizer.nextFloat()
        val wz1 = randomizer.nextFloat()
        val bx1 = randomizer.nextFloat()
        val by1 = randomizer.nextFloat()
        val bz1 = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val wx2 = randomizer.nextFloat()
        val wy2 = randomizer.nextFloat()
        val wz2 = randomizer.nextFloat()
        val bx2 = randomizer.nextFloat()
        val by2 = randomizer.nextFloat()
        val bz2 = randomizer.nextFloat()

        val measurement1 = GyroscopeSensorMeasurement(
            wx1,
            wy1,
            wz1,
            bx1,
            by1,
            bz1,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE,
            SensorCoordinateSystem.NED
        )

        val measurement2 = GyroscopeSensorMeasurement(
            wx2,
            wy2,
            wz2,
            bx2,
            by2,
            bz2,
            timestamp,
            SensorAccuracy.LOW,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentWx_returnsFalse() {
        val randomizer = UniformRandomizer()
        val wx1 = randomizer.nextFloat()
        val wy1 = randomizer.nextFloat()
        val wz1 = randomizer.nextFloat()
        val bx1 = randomizer.nextFloat()
        val by1 = randomizer.nextFloat()
        val bz1 = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val wx2 = randomizer.nextFloat()
        val wy2 = randomizer.nextFloat()
        val wz2 = randomizer.nextFloat()
        val bx2 = randomizer.nextFloat()
        val by2 = randomizer.nextFloat()
        val bz2 = randomizer.nextFloat()

        val measurement1 = GyroscopeSensorMeasurement(
            wx1,
            wy1,
            wz1,
            bx1,
            by1,
            bz1,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE,
            SensorCoordinateSystem.NED
        )

        val measurement2 = GyroscopeSensorMeasurement(
            wx2,
            wy2,
            wz2,
            bx2,
            by2,
            bz2,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentWy_returnsFalse() {
        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy1 = randomizer.nextFloat()
        val wz1 = randomizer.nextFloat()
        val bx1 = randomizer.nextFloat()
        val by1 = randomizer.nextFloat()
        val bz1 = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val wy2 = randomizer.nextFloat()
        val wz2 = randomizer.nextFloat()
        val bx2 = randomizer.nextFloat()
        val by2 = randomizer.nextFloat()
        val bz2 = randomizer.nextFloat()

        val measurement1 = GyroscopeSensorMeasurement(
            wx,
            wy1,
            wz1,
            bx1,
            by1,
            bz1,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE,
            SensorCoordinateSystem.NED
        )

        val measurement2 = GyroscopeSensorMeasurement(
            wx,
            wy2,
            wz2,
            bx2,
            by2,
            bz2,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentWz_returnsFalse() {
        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz1 = randomizer.nextFloat()
        val bx1 = randomizer.nextFloat()
        val by1 = randomizer.nextFloat()
        val bz1 = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val wz2 = randomizer.nextFloat()
        val bx2 = randomizer.nextFloat()
        val by2 = randomizer.nextFloat()
        val bz2 = randomizer.nextFloat()

        val measurement1 = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz1,
            bx1,
            by1,
            bz1,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE,
            SensorCoordinateSystem.NED
        )

        val measurement2 = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz2,
            bx2,
            by2,
            bz2,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentBx_returnsFalse() {
        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx1 = randomizer.nextFloat()
        val by1 = randomizer.nextFloat()
        val bz1 = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val bx2 = randomizer.nextFloat()
        val by2 = randomizer.nextFloat()
        val bz2 = randomizer.nextFloat()

        val measurement1 = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            bx1,
            by1,
            bz1,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE,
            SensorCoordinateSystem.NED
        )

        val measurement2 = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            bx2,
            by2,
            bz2,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentBy_returnsFalse() {
        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by1 = randomizer.nextFloat()
        val bz1 = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val by2 = randomizer.nextFloat()
        val bz2 = randomizer.nextFloat()

        val measurement1 = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            bx,
            by1,
            bz1,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE,
            SensorCoordinateSystem.NED
        )

        val measurement2 = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            bx,
            by2,
            bz2,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentBz_returnsFalse() {
        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz1 = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val bz2 = randomizer.nextFloat()

        val measurement1 = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz1,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE,
            SensorCoordinateSystem.NED
        )

        val measurement2 = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz2,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentTimestamp_returnsFalse() {
        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp1 = System.nanoTime()

        val timestamp2 = System.nanoTime()

        val measurement1 = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp1,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE,
            SensorCoordinateSystem.NED
        )

        val measurement2 = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp2,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentSensorType_returnsFalse() {
        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement1 = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE,
            SensorCoordinateSystem.NED
        )

        val measurement2 = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentCoordinateSystem_returnsFalse() {
        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement1 = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )

        val measurement2 = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenEqualContent_returnsTrue() {
        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement1 = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE,
            SensorCoordinateSystem.NED
        )

        val measurement2 = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE,
            SensorCoordinateSystem.NED
        )

        assert(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun hashCode_whenEqualObjects_returnsSameValue() {
        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement1 = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE,
            SensorCoordinateSystem.NED
        )

        val measurement2 = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE,
            SensorCoordinateSystem.NED
        )

        assertEquals(measurement1.hashCode(), measurement2.hashCode())
    }

    @Test
    fun hashCode_whenDifferentObjects_returnsDifferentValue() {
        val randomizer = UniformRandomizer()
        val wx1 = randomizer.nextFloat()
        val wy1 = randomizer.nextFloat()
        val wz1 = randomizer.nextFloat()
        val bx1 = randomizer.nextFloat()
        val by1 = randomizer.nextFloat()
        val bz1 = randomizer.nextFloat()
        val timestamp1 = System.nanoTime()

        val wx2 = randomizer.nextFloat()
        val wy2 = randomizer.nextFloat()
        val wz2 = randomizer.nextFloat()
        val bx2 = randomizer.nextFloat()
        val by2 = randomizer.nextFloat()
        val bz2 = randomizer.nextFloat()
        val timestamp2 = System.nanoTime()

        val measurement1 = GyroscopeSensorMeasurement(
            wx1,
            wy1,
            wz1,
            bx1,
            by1,
            bz1,
            timestamp1,
            SensorAccuracy.HIGH,
            GyroscopeSensorType.GYROSCOPE,
            SensorCoordinateSystem.NED
        )

        val measurement2 = GyroscopeSensorMeasurement(
            wx2,
            wy2,
            wz2,
            bx2,
            by2,
            bz2,
            timestamp2,
            SensorAccuracy.LOW,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        assertNotEquals(measurement1.hashCode(), measurement2.hashCode())
    }
}