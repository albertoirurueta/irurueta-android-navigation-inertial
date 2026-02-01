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

import com.irurueta.android.navigation.inertial.collectors.converters.AccelerometerSensorMeasurementCoordinateSystemConverter
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.AccelerationUnit
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotEquals
import org.junit.Assert.assertNotSame
import org.junit.Assert.assertNull
import org.junit.Assert.assertThrows
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.sqrt

class AccelerometerSensorMeasurementTest {

    @Test
    fun constructor_whenDefault_setsExpectedValues() {
        val measurement = AccelerometerSensorMeasurement()

        // check
        assertEquals(0.0f, measurement.ax, 0.0f)
        assertEquals(0.0f, measurement.ay, 0.0f)
        assertEquals(0.0f, measurement.az, 0.0f)
        assertNull(measurement.bx)
        assertNull(measurement.by)
        assertNull(measurement.bz)
        assertEquals(0L, measurement.timestamp)
        assertNull(measurement.accuracy)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            measurement.sensorType
        )
        assertEquals(SensorCoordinateSystem.ENU,
            measurement.sensorCoordinateSystem)
    }

    @Test
    fun constructor_withRequiredProperties_setsExpectedValues() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement =
            AccelerometerSensorMeasurement(
                ax,
                ay,
                az,
                null,
                null,
                null,
                timestamp,
                null,
                AccelerometerSensorType.ACCELEROMETER,
                SensorCoordinateSystem.NED
            )

        // check
        assertEquals(ax, measurement.ax, 0.0f)
        assertEquals(ay, measurement.ay, 0.0f)
        assertEquals(az, measurement.az, 0.0f)
        assertNull(measurement.bx)
        assertNull(measurement.by)
        assertNull(measurement.bz)
        assertEquals(timestamp, measurement.timestamp)
        assertNull(measurement.accuracy)
        assertEquals(AccelerometerSensorType.ACCELEROMETER,
            measurement.sensorType)
        assertEquals(SensorCoordinateSystem.NED,
            measurement.sensorCoordinateSystem)
    }

    @Test
    fun constructor_withAllProperties_setsExpectedValues() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )

        // check
        assertEquals(ax, measurement.ax, 0.0f)
        assertEquals(ay, measurement.ay, 0.0f)
        assertEquals(az, measurement.az, 0.0f)
        assertEquals(bx, measurement.bx)
        assertEquals(by, measurement.by)
        assertEquals(bz, measurement.bz)
        assertEquals(timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            measurement.sensorType
        )
        assertEquals(SensorCoordinateSystem.NED,
            measurement.sensorCoordinateSystem)
    }

    @Test
    fun constructor_whenCopy_setsExpectedValues() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement1 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )

        val measurement2 = AccelerometerSensorMeasurement(measurement1)

        // check
        assertEquals(ax, measurement1.ax, 0.0f)
        assertEquals(ay, measurement1.ay, 0.0f)
        assertEquals(az, measurement1.az, 0.0f)
        assertEquals(bx, measurement1.bx)
        assertEquals(by, measurement1.by)
        assertEquals(bz, measurement1.bz)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement1.accuracy)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            measurement1.sensorType
        )
        assertEquals(SensorCoordinateSystem.NED,
            measurement1.sensorCoordinateSystem)

        assertEquals(measurement1.ax, measurement2.ax, 0.0f)
        assertEquals(measurement1.ay, measurement2.ay, 0.0f)
        assertEquals(measurement1.az, measurement2.az, 0.0f)
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
    fun ax_setsExpectedValue() {
        val measurement = AccelerometerSensorMeasurement()

        // check default value
        assertEquals(0.0f, measurement.ax, 0.0f)

        // set new value
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        measurement.ax = ax

        // check
        assertEquals(ax, measurement.ax, 0.0f)
    }

    @Test
    fun ay_setsExpectedValue() {
        val measurement = AccelerometerSensorMeasurement()

        // check default value
        assertEquals(0.0f, measurement.ay, 0.0f)

        // set new value
        val randomizer = UniformRandomizer()
        val ay = randomizer.nextFloat()
        measurement.ay = ay

        // check
        assertEquals(ay, measurement.ay, 0.0f)
    }

    @Test
    fun az_setsExpectedValue() {
        val measurement = AccelerometerSensorMeasurement()

        // check default value
        assertEquals(0.0f, measurement.az, 0.0f)

        // set new value
        val randomizer = UniformRandomizer()
        val az = randomizer.nextFloat()
        measurement.az = az

        // check
        assertEquals(az, measurement.az, 0.0f)
    }

    @Test
    fun bx_setsExpectedValue() {
        val measurement = AccelerometerSensorMeasurement()

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
        val measurement = AccelerometerSensorMeasurement()

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
        val measurement = AccelerometerSensorMeasurement()

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
        val measurement = AccelerometerSensorMeasurement()

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
        val measurement = AccelerometerSensorMeasurement()

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
        val measurement = AccelerometerSensorMeasurement()

        // check default value
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            measurement.sensorType
        )

        // set new value
        measurement.sensorType = AccelerometerSensorType.ACCELEROMETER

        // check
        assertEquals(AccelerometerSensorType.ACCELEROMETER, measurement.sensorType)
    }

    @Test
    fun sensorCoordinateSystem_setsExpectedValue() {
        val measurement = AccelerometerSensorMeasurement()

        // check default value
        assertEquals(
            SensorCoordinateSystem.ENU,
            measurement.sensorCoordinateSystem
        )

        // set new value
        measurement.sensorCoordinateSystem = SensorCoordinateSystem.NED

        // check
        assertEquals(SensorCoordinateSystem.NED,
            measurement.sensorCoordinateSystem)
    }

    @Test
    fun copyFrom_makesExpectedCopy() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement1 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER,
            SensorCoordinateSystem.NED
        )

        val measurement2 = AccelerometerSensorMeasurement()
        measurement2.copyFrom(measurement1)

        // check
        assertEquals(ax, measurement1.ax, 0.0f)
        assertEquals(ay, measurement1.ay, 0.0f)
        assertEquals(az, measurement1.az, 0.0f)
        assertEquals(bx, measurement1.bx)
        assertEquals(by, measurement1.by)
        assertEquals(bz, measurement1.bz)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement1.accuracy)
        assertEquals(AccelerometerSensorType.ACCELEROMETER,
            measurement1.sensorType)
        assertEquals(SensorCoordinateSystem.NED,
            measurement1.sensorCoordinateSystem)

        assertEquals(measurement1.ax, measurement2.ax, 0.0f)
        assertEquals(measurement1.ay, measurement2.ay, 0.0f)
        assertEquals(measurement1.az, measurement2.az, 0.0f)
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
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement1 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER,
            SensorCoordinateSystem.NED
        )

        val measurement2 = AccelerometerSensorMeasurement()
        measurement1.copyTo(measurement2)

        // check
        assertEquals(ax, measurement1.ax, 0.0f)
        assertEquals(ay, measurement1.ay, 0.0f)
        assertEquals(az, measurement1.az, 0.0f)
        assertEquals(bx, measurement1.bx)
        assertEquals(by, measurement1.by)
        assertEquals(bz, measurement1.bz)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement1.accuracy)
        assertEquals(AccelerometerSensorType.ACCELEROMETER, measurement1.sensorType)
        assertEquals(SensorCoordinateSystem.NED,
            measurement1.sensorCoordinateSystem)

        assertEquals(measurement1.ax, measurement2.ax, 0.0f)
        assertEquals(measurement1.ay, measurement2.ay, 0.0f)
        assertEquals(measurement1.az, measurement2.az, 0.0f)
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
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement1 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER,
            SensorCoordinateSystem.NED
        )

        val measurement2 = measurement1.copy()

        // check
        assertEquals(ax, measurement1.ax, 0.0f)
        assertEquals(ay, measurement1.ay, 0.0f)
        assertEquals(az, measurement1.az, 0.0f)
        assertEquals(bx, measurement1.bx)
        assertEquals(by, measurement1.by)
        assertEquals(bz, measurement1.bz)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement1.accuracy)
        assertEquals(AccelerometerSensorType.ACCELEROMETER,
            measurement1.sensorType)
        assertEquals(SensorCoordinateSystem.NED,
            measurement1.sensorCoordinateSystem)

        assertEquals(measurement1.ax, measurement2.ax, 0.0f)
        assertEquals(measurement1.ay, measurement2.ay, 0.0f)
        assertEquals(measurement1.az, measurement2.az, 0.0f)
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
    fun toNedOrThrow_whenENU_returnsExpectedValue() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        val nedMeasurement1 = measurement.toNedOrThrow()
        val nedMeasurement2 = AccelerometerSensorMeasurement()
        measurement.toNedOrThrow(nedMeasurement2)

        val nedMeasurement3 = AccelerometerSensorMeasurementCoordinateSystemConverter.toNedOrThrow(
            measurement)

        // check
        assertEquals(nedMeasurement1, nedMeasurement2)
        assertEquals(nedMeasurement1, nedMeasurement3)
    }

    @Test
    fun toNedOrThrow_whenNED_throwsIllegalArgumentException() {
        val measurement = AccelerometerSensorMeasurement(
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
    fun toEnuOrThrow_whenNED_returnsExpectedValue() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )

        val enuMeasurement1 = measurement.toEnuOrThrow()
        val enuMeasurement2 = AccelerometerSensorMeasurement()
        measurement.toEnuOrThrow(enuMeasurement2)

        val enuMeasurement3 = AccelerometerSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(
            measurement)

        // check
        assertEquals(enuMeasurement1, enuMeasurement2)
        assertEquals(enuMeasurement1, enuMeasurement3)
    }

    @Test
    fun toEnuOrThrow_whenENU_throwsIllegalArgumentException() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.ENU
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
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )

        val result1 = measurement.toNed()
        val result2 = AccelerometerSensorMeasurement()
        measurement.toNed(result2)

        // check
        assertEquals(measurement, result1)
        assertEquals(measurement, result2)
    }

    @Test
    fun toNed_whenENU_returnsConvertedValue() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        val result1 = measurement.toNed()
        val result2 = AccelerometerSensorMeasurement()
        measurement.toNed(result2)

        val result3 = AccelerometerSensorMeasurementCoordinateSystemConverter.toNedOrThrow(
            measurement)

        // check
        assertEquals(result3, result1)
        assertEquals(result3, result2)
    }

    @Test
    fun toEnu_whenENU_returnsCopy() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        val result1 = measurement.toEnu()
        val result2 = AccelerometerSensorMeasurement()
        measurement.toEnu(result2)

        // check
        assertEquals(measurement, result1)
        assertEquals(measurement, result2)
    }

    @Test
    fun toEnu_whenNED_returnsConvertedValue() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )

        val result1 = measurement.toEnu()
        val result2 = AccelerometerSensorMeasurement()
        measurement.toEnu(result2)

        val result3 = AccelerometerSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(
            measurement)

        // check
        assertEquals(result3, result1)
        assertEquals(result3, result2)
    }

    @Test
    fun toTriad_whenNoBias_returnsExpectedValue() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            null,
            null,
            null,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER,
            SensorCoordinateSystem.ENU
        )

        val triad1 = measurement.toTriad()
        val triad2 = AccelerationTriad()
        measurement.toTriad(triad2)

        // check
        assertEquals(ax.toDouble(), triad1.valueX, 0.0)
        assertEquals(ay.toDouble(), triad1.valueY, 0.0)
        assertEquals(az.toDouble(), triad1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.unit)
        assertEquals(triad1, triad2)
    }

    @Test
    fun toTriad_whenBias_returnsExpectedValue() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        val triad1 = measurement.toTriad()
        val triad2 = AccelerationTriad()
        measurement.toTriad(triad2)

        // check
        assertEquals(ax.toDouble() + bx.toDouble(), triad1.valueX, 0.0)
        assertEquals(ay.toDouble() + by.toDouble(), triad1.valueY, 0.0)
        assertEquals(az.toDouble() + bz.toDouble(), triad1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.unit)
        assertEquals(triad1, triad2)
    }

    @Test
    fun toNorm_whenNoBias_returnsExpectedValue() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            null,
            null,
            null,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER,
            SensorCoordinateSystem.ENU
        )

        val norm = measurement.toNorm()

        // check
        val ax2 = ax.toDouble() * ax.toDouble()
        val ay2 = ay.toDouble() * ay.toDouble()
        val az2 = az.toDouble() * az.toDouble()
        val expected = sqrt(ax2 + ay2 + az2)
        assertEquals(expected, norm, 0.0)
    }

    @Test
    fun toNorm_whenBias_returnsExpectedValue() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        val norm = measurement.toNorm()

        // check
        val x = ax.toDouble() + bx.toDouble()
        val y = ay.toDouble() + by.toDouble()
        val z = az.toDouble() + bz.toDouble()
        val expected = sqrt(x * x + y * y + z * z)
        assertEquals(expected, norm, 0.0)
    }

    @Test
    fun equals_whenNull_returnsFalse() {
        val measurement = AccelerometerSensorMeasurement()
        assertFalse(measurement.equals(null))
    }

    @Test
    fun equals_whenSameInstance_returnsTrue() {
        val measurement = AccelerometerSensorMeasurement()
        @Suppress("ReplaceCallWithBinaryOperator")
        assertTrue(measurement.equals(measurement))
    }

    @Test
    fun equals_whenDifferentType_returnsFalse() {
        val measurement = AccelerometerSensorMeasurement()
        assertFalse(measurement == Any())
    }

    @Test
    fun equals_whenDifferentContent_returnsFalse() {
        val randomizer = UniformRandomizer()
        val ax1 = randomizer.nextFloat()
        val ay1 = randomizer.nextFloat()
        val az1 = randomizer.nextFloat()
        val bx1 = randomizer.nextFloat()
        val by1 = randomizer.nextFloat()
        val bz1 = randomizer.nextFloat()
        val timestamp1 = System.nanoTime()

        val ax2 = randomizer.nextFloat()
        val ay2 = randomizer.nextFloat()
        val az2 = randomizer.nextFloat()
        val bx2 = randomizer.nextFloat()
        val by2 = randomizer.nextFloat()
        val bz2 = randomizer.nextFloat()
        val timestamp2 = System.nanoTime()

        val measurement1 = AccelerometerSensorMeasurement(
            ax1,
            ay1,
            az1,
            bx1,
            by1,
            bz1,
            timestamp1,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER,
            SensorCoordinateSystem.NED
        )

        val measurement2 = AccelerometerSensorMeasurement(
            ax2,
            ay2,
            az2,
            bx2,
            by2,
            bz2,
            timestamp2,
            SensorAccuracy.LOW,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentSensorAccuracy_returnsFalse() {
        val randomizer = UniformRandomizer()
        val ax1 = randomizer.nextFloat()
        val ay1 = randomizer.nextFloat()
        val az1 = randomizer.nextFloat()
        val bx1 = randomizer.nextFloat()
        val by1 = randomizer.nextFloat()
        val bz1 = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val ax2 = randomizer.nextFloat()
        val ay2 = randomizer.nextFloat()
        val az2 = randomizer.nextFloat()
        val bx2 = randomizer.nextFloat()
        val by2 = randomizer.nextFloat()
        val bz2 = randomizer.nextFloat()

        val measurement1 = AccelerometerSensorMeasurement(
            ax1,
            ay1,
            az1,
            bx1,
            by1,
            bz1,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER,
            SensorCoordinateSystem.NED
        )

        val measurement2 = AccelerometerSensorMeasurement(
            ax2,
            ay2,
            az2,
            bx2,
            by2,
            bz2,
            timestamp,
            SensorAccuracy.LOW,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentAx_returnsFalse() {
        val randomizer = UniformRandomizer()
        val ax1 = randomizer.nextFloat()
        val ay1 = randomizer.nextFloat()
        val az1 = randomizer.nextFloat()
        val bx1 = randomizer.nextFloat()
        val by1 = randomizer.nextFloat()
        val bz1 = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val ax2 = randomizer.nextFloat()
        val ay2 = randomizer.nextFloat()
        val az2 = randomizer.nextFloat()
        val bx2 = randomizer.nextFloat()
        val by2 = randomizer.nextFloat()
        val bz2 = randomizer.nextFloat()

        val measurement1 = AccelerometerSensorMeasurement(
            ax1,
            ay1,
            az1,
            bx1,
            by1,
            bz1,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER,
            SensorCoordinateSystem.NED
        )

        val measurement2 = AccelerometerSensorMeasurement(
            ax2,
            ay2,
            az2,
            bx2,
            by2,
            bz2,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentAy_returnsFalse() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay1 = randomizer.nextFloat()
        val az1 = randomizer.nextFloat()
        val bx1 = randomizer.nextFloat()
        val by1 = randomizer.nextFloat()
        val bz1 = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val ay2 = randomizer.nextFloat()
        val az2 = randomizer.nextFloat()
        val bx2 = randomizer.nextFloat()
        val by2 = randomizer.nextFloat()
        val bz2 = randomizer.nextFloat()

        val measurement1 = AccelerometerSensorMeasurement(
            ax,
            ay1,
            az1,
            bx1,
            by1,
            bz1,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER,
            SensorCoordinateSystem.NED
        )

        val measurement2 = AccelerometerSensorMeasurement(
            ax,
            ay2,
            az2,
            bx2,
            by2,
            bz2,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentAz_returnsFalse() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az1 = randomizer.nextFloat()
        val bx1 = randomizer.nextFloat()
        val by1 = randomizer.nextFloat()
        val bz1 = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val az2 = randomizer.nextFloat()
        val bx2 = randomizer.nextFloat()
        val by2 = randomizer.nextFloat()
        val bz2 = randomizer.nextFloat()

        val measurement1 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az1,
            bx1,
            by1,
            bz1,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER,
            SensorCoordinateSystem.NED
        )

        val measurement2 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az2,
            bx2,
            by2,
            bz2,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentBx_returnsFalse() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx1 = randomizer.nextFloat()
        val by1 = randomizer.nextFloat()
        val bz1 = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val bx2 = randomizer.nextFloat()
        val by2 = randomizer.nextFloat()
        val bz2 = randomizer.nextFloat()

        val measurement1 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            bx1,
            by1,
            bz1,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER,
            SensorCoordinateSystem.NED
        )

        val measurement2 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            bx2,
            by2,
            bz2,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentBy_returnsFalse() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by1 = randomizer.nextFloat()
        val bz1 = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val by2 = randomizer.nextFloat()
        val bz2 = randomizer.nextFloat()

        val measurement1 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            bx,
            by1,
            bz1,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER,
            SensorCoordinateSystem.NED
        )

        val measurement2 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            bx,
            by2,
            bz2,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentBz_returnsFalse() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz1 = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val bz2 = randomizer.nextFloat()

        val measurement1 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            bz1,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER,
            SensorCoordinateSystem.NED
        )

        val measurement2 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            bz2,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentTimestamp_returnsFalse() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp1 = System.nanoTime()

        val timestamp2 = System.nanoTime()

        val measurement1 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            bz,
            timestamp1,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER,
            SensorCoordinateSystem.NED
        )

        val measurement2 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            bz,
            timestamp2,
            SensorAccuracy.LOW,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentSensorType_returnsFalse() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement1 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER,
            SensorCoordinateSystem.NED
        )

        val measurement2 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentCoordinateSystem_returnsFalse() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement1 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.NED
        )

        val measurement2 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenEqualContent_returnsTrue() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement1 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER,
            SensorCoordinateSystem.NED
        )

        val measurement2 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER,
            SensorCoordinateSystem.NED
        )

        assertTrue(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun hashCode_whenEqualObjects_returnsSameValue() {
        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement1 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER,
            SensorCoordinateSystem.NED
        )

        val measurement2 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER,
            SensorCoordinateSystem.NED
        )

        assertEquals(measurement1.hashCode(), measurement2.hashCode())
    }

    @Test
    fun hashCode_whenDifferentObjects_returnsDifferentValue() {
        val randomizer = UniformRandomizer()
        val ax1 = randomizer.nextFloat()
        val ay1 = randomizer.nextFloat()
        val az1 = randomizer.nextFloat()
        val bx1 = randomizer.nextFloat()
        val by1 = randomizer.nextFloat()
        val bz1 = randomizer.nextFloat()
        val timestamp1 = System.nanoTime()

        val ax2 = randomizer.nextFloat()
        val ay2 = randomizer.nextFloat()
        val az2 = randomizer.nextFloat()
        val bx2 = randomizer.nextFloat()
        val by2 = randomizer.nextFloat()
        val bz2 = randomizer.nextFloat()
        val timestamp2 = System.nanoTime()

        val measurement1 = AccelerometerSensorMeasurement(
            ax1,
            ay1,
            az1,
            bx1,
            by1,
            bz1,
            timestamp1,
            SensorAccuracy.HIGH,
            AccelerometerSensorType.ACCELEROMETER,
            SensorCoordinateSystem.NED
        )

        val measurement2 = AccelerometerSensorMeasurement(
            ax2,
            ay2,
            az2,
            bx2,
            by2,
            bz2,
            timestamp2,
            SensorAccuracy.LOW,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        assertNotEquals(measurement1.hashCode(), measurement2.hashCode())
    }
}