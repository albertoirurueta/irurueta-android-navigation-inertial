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

import com.irurueta.android.navigation.inertial.collectors.converters.MagnetometerSensorMeasurementCoordinateSystemConverter
import com.irurueta.navigation.inertial.calibration.MagneticFluxDensityTriad
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.MagneticFluxDensityUnit
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotEquals
import org.junit.Assert.assertNotSame
import org.junit.Assert.assertNull
import org.junit.Assert.assertThrows
import org.junit.Assert.assertTrue
import org.junit.Test
import kotlin.math.sqrt

class MagnetometerSensorMeasurementTest {

    @Test
    fun constructor_whenDefault_setsExpectedValues() {
        val measurement = MagnetometerSensorMeasurement()

        // check
        assertEquals(0.0f, measurement.bx, 0.0f)
        assertEquals(0.0f, measurement.by, 0.0f)
        assertEquals(0.0f, measurement.bz, 0.0f)
        assertNull(measurement.hardIronX)
        assertNull(measurement.hardIronY)
        assertNull(measurement.hardIronZ)
        assertEquals(0L, measurement.timestamp)
        assertNull(measurement.accuracy)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            measurement.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.ENU,
            measurement.sensorCoordinateSystem
        )
    }

    @Test
    fun constructor_withRequiredProperties_setsExpectedValues() {
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement =
            MagnetometerSensorMeasurement(
                bx,
                by,
                bz,
                null,
                null,
                null,
                timestamp,
                null,
                MagnetometerSensorType.MAGNETOMETER,
                SensorCoordinateSystem.NED
            )

        // check
        assertEquals(bx, measurement.bx, 0.0f)
        assertEquals(by, measurement.by, 0.0f)
        assertEquals(bz, measurement.bz, 0.0f)
        assertNull(measurement.hardIronX)
        assertNull(measurement.hardIronY)
        assertNull(measurement.hardIronZ)
        assertEquals(timestamp, measurement.timestamp)
        assertNull(measurement.accuracy)
        assertEquals(MagnetometerSensorType.MAGNETOMETER, measurement.sensorType)
        assertEquals(
            SensorCoordinateSystem.NED,
            measurement.sensorCoordinateSystem
        )
    }

    @Test
    fun constructor_withAllProperties_setsExpectedValues() {
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement =
            MagnetometerSensorMeasurement(
                bx,
                by,
                bz,
                hardIronX,
                hardIronY,
                hardIronZ,
                timestamp,
                SensorAccuracy.LOW,
                MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
                SensorCoordinateSystem.NED
            )

        // check
        assertEquals(bx, measurement.bx, 0.0f)
        assertEquals(by, measurement.by, 0.0f)
        assertEquals(bz, measurement.bz, 0.0f)
        assertEquals(hardIronX, measurement.hardIronX)
        assertEquals(hardIronY, measurement.hardIronY)
        assertEquals(hardIronZ, measurement.hardIronZ)
        assertEquals(timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.LOW, measurement.accuracy)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            measurement.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.NED,
            measurement.sensorCoordinateSystem
        )
    }

    @Test
    fun constructor_whenCopy_setsExpectedValues() {
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement1 =
            MagnetometerSensorMeasurement(
                bx,
                by,
                bz,
                hardIronX,
                hardIronY,
                hardIronZ,
                timestamp,
                SensorAccuracy.LOW,
                MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
                SensorCoordinateSystem.NED
            )

        val measurement2 = MagnetometerSensorMeasurement(measurement1)

        // check
        assertEquals(bx, measurement1.bx, 0.0f)
        assertEquals(by, measurement1.by, 0.0f)
        assertEquals(bz, measurement1.bz, 0.0f)
        assertEquals(hardIronX, measurement1.hardIronX)
        assertEquals(hardIronY, measurement1.hardIronY)
        assertEquals(hardIronZ, measurement1.hardIronZ)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.LOW, measurement1.accuracy)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            measurement1.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.NED,
            measurement1.sensorCoordinateSystem
        )

        assertEquals(measurement1.bx, measurement2.bx, 0.0f)
        assertEquals(measurement1.by, measurement2.by, 0.0f)
        assertEquals(measurement1.bz, measurement2.bz, 0.0f)
        assertEquals(measurement1.hardIronX, measurement2.hardIronX)
        assertEquals(measurement1.hardIronY, measurement2.hardIronY)
        assertEquals(measurement1.hardIronZ, measurement2.hardIronZ)
        assertEquals(measurement1.timestamp, measurement2.timestamp)
        assertEquals(measurement1.accuracy, measurement2.accuracy)
        assertEquals(measurement1.sensorType, measurement2.sensorType)
        assertEquals(
            measurement1.sensorCoordinateSystem,
            measurement2.sensorCoordinateSystem
        )
    }

    @Test
    fun bx_setsExpectedValue() {
        val measurement = MagnetometerSensorMeasurement()

        // check default value
        assertEquals(0.0f, measurement.bx, 0.0f)

        // set new value
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        measurement.bx = bx

        // check
        assertEquals(bx, measurement.bx, 0.0f)
    }

    @Test
    fun by_setsExpectedValue() {
        val measurement = MagnetometerSensorMeasurement()

        // check default value
        assertEquals(0.0f, measurement.by, 0.0f)

        // set new value
        val randomizer = UniformRandomizer()
        val by = randomizer.nextFloat()
        measurement.by = by

        // check
        assertEquals(by, measurement.by, 0.0f)
    }

    @Test
    fun bz_setsExpectedValue() {
        val measurement = MagnetometerSensorMeasurement()

        // check default value
        assertEquals(0.0f, measurement.bz, 0.0f)

        // set new value
        val randomizer = UniformRandomizer()
        val bz = randomizer.nextFloat()
        measurement.bz = bz

        // check
        assertEquals(bz, measurement.bz, 0.0f)
    }

    @Test
    fun hardIronX_setsExpectedValue() {
        val measurement = MagnetometerSensorMeasurement()

        // check default value
        assertNull(measurement.hardIronX)

        // set new value
        val randomizer = UniformRandomizer()
        val hardIronX = randomizer.nextFloat()
        measurement.hardIronX = hardIronX

        // check
        assertEquals(hardIronX, measurement.hardIronX)

        // set to null
        measurement.hardIronX = null

        // check
        @Suppress("KotlinConstantConditions")
        (assertNull(measurement.hardIronX))
    }

    @Test
    fun hardIronY_setsExpectedValue() {
        val measurement = MagnetometerSensorMeasurement()

        // check default value
        assertNull(measurement.hardIronY)

        // set new value
        val randomizer = UniformRandomizer()
        val hardIronY = randomizer.nextFloat()
        measurement.hardIronY = hardIronY

        // check
        assertEquals(hardIronY, measurement.hardIronY)

        // set to null
        measurement.hardIronY = null

        @Suppress("KotlinConstantConditions")
        (assertNull(measurement.hardIronY))
    }

    @Test
    fun hardIronZ_setsExpectedValue() {
        val measurement = MagnetometerSensorMeasurement()

        // check default value
        assertNull(measurement.hardIronZ)

        // set new value
        val randomizer = UniformRandomizer()
        val hardIronZ = randomizer.nextFloat()
        measurement.hardIronZ = hardIronZ

        // check
        assertEquals(hardIronZ, measurement.hardIronZ)

        // set to null
        measurement.hardIronZ = null

        @Suppress("KotlinConstantConditions")
        (assertNull(measurement.hardIronZ))
    }

    @Test
    fun timestamp_setsExpectedValue() {
        val measurement = MagnetometerSensorMeasurement()

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
        val measurement = MagnetometerSensorMeasurement()

        // check default value
        assertNull(measurement.accuracy)

        // set new value
        measurement.accuracy = SensorAccuracy.LOW

        // check
        assertEquals(SensorAccuracy.LOW, measurement.accuracy)

        // set to null
        measurement.accuracy = null

        @Suppress("KotlinConstantConditions")
        (assertNull(measurement.accuracy))
    }

    @Test
    fun sensorType_setsExpectedValue() {
        val measurement = MagnetometerSensorMeasurement()

        // check default value
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            measurement.sensorType
        )

        // set new value
        measurement.sensorType = MagnetometerSensorType.MAGNETOMETER

        // check
        assertEquals(MagnetometerSensorType.MAGNETOMETER, measurement.sensorType)
    }

    @Test
    fun sensorCoordinateSystem_setsExpectedValue() {
        val measurement = MagnetometerSensorMeasurement()

        // check default value
        assertEquals(
            SensorCoordinateSystem.ENU,
            measurement.sensorCoordinateSystem
        )

        // set new value
        measurement.sensorCoordinateSystem = SensorCoordinateSystem.NED

        // check
        assertEquals(
            SensorCoordinateSystem.NED,
            measurement.sensorCoordinateSystem
        )
    }

    @Test
    fun copyFrom_makesExpectedCopy() {
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement1 =
            MagnetometerSensorMeasurement(
                bx,
                by,
                bz,
                hardIronX,
                hardIronY,
                hardIronZ,
                timestamp,
                SensorAccuracy.LOW,
                MagnetometerSensorType.MAGNETOMETER,
                SensorCoordinateSystem.NED
            )

        val measurement2 = MagnetometerSensorMeasurement()
        measurement2.copyFrom(measurement1)

        // check
        assertEquals(bx, measurement1.bx, 0.0f)
        assertEquals(by, measurement1.by, 0.0f)
        assertEquals(bz, measurement1.bz, 0.0f)
        assertEquals(hardIronX, measurement1.hardIronX)
        assertEquals(hardIronY, measurement1.hardIronY)
        assertEquals(hardIronZ, measurement1.hardIronZ)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.LOW, measurement1.accuracy)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER,
            measurement1.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.NED,
            measurement1.sensorCoordinateSystem
        )

        assertEquals(measurement1.bx, measurement2.bx, 0.0f)
        assertEquals(measurement1.by, measurement2.by, 0.0f)
        assertEquals(measurement1.bz, measurement2.bz, 0.0f)
        assertEquals(measurement1.hardIronX, measurement2.hardIronX)
        assertEquals(measurement1.hardIronY, measurement2.hardIronY)
        assertEquals(measurement1.hardIronZ, measurement2.hardIronZ)
        assertEquals(measurement1.timestamp, measurement2.timestamp)
        assertEquals(measurement1.accuracy, measurement2.accuracy)
        assertEquals(measurement1.sensorType, measurement2.sensorType)
        assertEquals(
            measurement1.sensorCoordinateSystem,
            measurement2.sensorCoordinateSystem
        )
    }

    @Test
    fun copyTo_makesExpectedCopy() {
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement1 =
            MagnetometerSensorMeasurement(
                bx,
                by,
                bz,
                hardIronX,
                hardIronY,
                hardIronZ,
                timestamp,
                SensorAccuracy.LOW,
                MagnetometerSensorType.MAGNETOMETER,
                SensorCoordinateSystem.NED
            )

        val measurement2 = MagnetometerSensorMeasurement()
        measurement1.copyTo(measurement2)

        // check
        assertEquals(bx, measurement1.bx, 0.0f)
        assertEquals(by, measurement1.by, 0.0f)
        assertEquals(bz, measurement1.bz, 0.0f)
        assertEquals(hardIronX, measurement1.hardIronX)
        assertEquals(hardIronY, measurement1.hardIronY)
        assertEquals(hardIronZ, measurement1.hardIronZ)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.LOW, measurement1.accuracy)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER,
            measurement1.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.NED,
            measurement1.sensorCoordinateSystem
        )

        assertEquals(measurement1.bx, measurement2.bx, 0.0f)
        assertEquals(measurement1.by, measurement2.by, 0.0f)
        assertEquals(measurement1.bz, measurement2.bz, 0.0f)
        assertEquals(measurement1.hardIronX, measurement2.hardIronX)
        assertEquals(measurement1.hardIronY, measurement2.hardIronY)
        assertEquals(measurement1.hardIronZ, measurement2.hardIronZ)
        assertEquals(measurement1.timestamp, measurement2.timestamp)
        assertEquals(measurement1.accuracy, measurement2.accuracy)
        assertEquals(measurement1.sensorType, measurement2.sensorType)
        assertEquals(
            measurement1.sensorCoordinateSystem,
            measurement2.sensorCoordinateSystem
        )
    }

    @Test
    fun copy_makesExpectedCopy() {
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement1 =
            MagnetometerSensorMeasurement(
                bx,
                by,
                bz,
                hardIronX,
                hardIronY,
                hardIronZ,
                timestamp,
                SensorAccuracy.LOW,
                MagnetometerSensorType.MAGNETOMETER,
                SensorCoordinateSystem.NED
            )

        val measurement2 = measurement1.copy()

        // check
        assertEquals(bx, measurement1.bx, 0.0f)
        assertEquals(by, measurement1.by, 0.0f)
        assertEquals(bz, measurement1.bz, 0.0f)
        assertEquals(hardIronX, measurement1.hardIronX)
        assertEquals(hardIronY, measurement1.hardIronY)
        assertEquals(hardIronZ, measurement1.hardIronZ)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.LOW, measurement1.accuracy)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER,
            measurement1.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.NED,
            measurement1.sensorCoordinateSystem
        )

        assertEquals(measurement1.bx, measurement2.bx, 0.0f)
        assertEquals(measurement1.by, measurement2.by, 0.0f)
        assertEquals(measurement1.bz, measurement2.bz, 0.0f)
        assertEquals(measurement1.hardIronX, measurement2.hardIronX)
        assertEquals(measurement1.hardIronY, measurement2.hardIronY)
        assertEquals(measurement1.hardIronZ, measurement2.hardIronZ)
        assertEquals(measurement1.timestamp, measurement2.timestamp)
        assertEquals(measurement1.accuracy, measurement2.accuracy)
        assertEquals(measurement1.sensorType, measurement2.sensorType)
        assertEquals(
            measurement1.sensorCoordinateSystem,
            measurement2.sensorCoordinateSystem
        )
    }

    @Test
    fun toNedOrThrow_whenENU_returnsExpectedValue() {
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement =
            MagnetometerSensorMeasurement(
                bx,
                by,
                bz,
                hardIronX,
                hardIronY,
                hardIronZ,
                timestamp,
                SensorAccuracy.LOW,
                MagnetometerSensorType.MAGNETOMETER,
                SensorCoordinateSystem.ENU
            )

        val nedMeasurement1 = measurement.toNedOrThrow()
        val nedMeasurement2 = MagnetometerSensorMeasurement()
        measurement.toNedOrThrow(nedMeasurement2)

        val nedMeasurement3 = MagnetometerSensorMeasurementCoordinateSystemConverter.toNedOrThrow(measurement)

        // check
        assertEquals(nedMeasurement1, nedMeasurement2)
        assertEquals(nedMeasurement1, nedMeasurement3)
    }

    @Test
    fun toNedOrThrow_whenNED_throwsIllegalArgumentException() {
        val measurement = MagnetometerSensorMeasurement(
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
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement =
            MagnetometerSensorMeasurement(
                bx,
                by,
                bz,
                hardIronX,
                hardIronY,
                hardIronZ,
                timestamp,
                SensorAccuracy.LOW,
                MagnetometerSensorType.MAGNETOMETER,
                SensorCoordinateSystem.NED
            )

        val enuMeasurement1 = measurement.toEnuOrThrow()
        val enuMeasurement2 = MagnetometerSensorMeasurement()
        measurement.toEnuOrThrow(enuMeasurement2)

        val enuMeasurement3 = MagnetometerSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(
            measurement)

        // check
        assertEquals(enuMeasurement1, enuMeasurement2)
        assertEquals(enuMeasurement1, enuMeasurement3)
    }

    @Test
    fun toEnuOrThrow_whenENU_throwsIllegalArgumentException() {
        val measurement = MagnetometerSensorMeasurement(
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
    fun toNed_whenNED_copiesMeasurement() {
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement1 =
            MagnetometerSensorMeasurement(
                bx,
                by,
                bz,
                hardIronX,
                hardIronY,
                hardIronZ,
                timestamp,
                SensorAccuracy.LOW,
                MagnetometerSensorType.MAGNETOMETER,
                SensorCoordinateSystem.NED
            )

        val measurement2 = MagnetometerSensorMeasurement()
        measurement1.toNed(measurement2)
        val measurement3 = measurement1.toNed()

        // check
        assertEquals(measurement1, measurement2)
        assertEquals(measurement1, measurement3)
    }

    @Test
    fun toNed_whenENU_convertsToNED() {
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement =
            MagnetometerSensorMeasurement(
                bx,
                by,
                bz,
                hardIronX,
                hardIronY,
                hardIronZ,
                timestamp,
                SensorAccuracy.LOW,
                MagnetometerSensorType.MAGNETOMETER,
                SensorCoordinateSystem.ENU
            )

        val result1 = measurement.toNed()
        val result2 = MagnetometerSensorMeasurement()
        measurement.toNed(result2)
        val expected = MagnetometerSensorMeasurementCoordinateSystemConverter.toNedOrThrow(
            measurement
        )

        // check
        assertEquals(expected, result1)
        assertEquals(expected, result2)
    }

    @Test
    fun toEnu_whenENU_copiesMeasurement() {
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement =
            MagnetometerSensorMeasurement(
                bx,
                by,
                bz,
                hardIronX,
                hardIronY,
                hardIronZ,
                timestamp,
                SensorAccuracy.LOW,
                MagnetometerSensorType.MAGNETOMETER,
                SensorCoordinateSystem.ENU
            )

        val result1 = measurement.toEnu()
        val result2 = MagnetometerSensorMeasurement()
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
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement =
            MagnetometerSensorMeasurement(
                bx,
                by,
                bz,
                hardIronX,
                hardIronY,
                hardIronZ,
                timestamp,
                SensorAccuracy.LOW,
                MagnetometerSensorType.MAGNETOMETER,
                SensorCoordinateSystem.NED
            )

        val result1 = measurement.toEnu()
        val result2 = MagnetometerSensorMeasurement()
        measurement.toEnu(result2)

        val expected = MagnetometerSensorMeasurementCoordinateSystemConverter.toEnuOrThrow(
            measurement
        )

        // check
        assertEquals(expected, result1)
        assertEquals(expected, result2)
    }

    @Test
    fun toTriad_whenNoHardIron_returnsExpectedValues() {
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement =
            MagnetometerSensorMeasurement(
                bx,
                by,
                bz,
                null,
                null,
                null,
                timestamp,
                SensorAccuracy.LOW,
                MagnetometerSensorType.MAGNETOMETER,
                SensorCoordinateSystem.NED
            )

        val triad1 = measurement.toTriad()
        val triad2 = MagneticFluxDensityTriad()
        measurement.toTriad(triad2)

        // check
        assertEquals(bx.toDouble(), triad1.valueX, 0.0)
        assertEquals(by.toDouble(), triad1.valueY, 0.0)
        assertEquals(bz.toDouble(), triad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.MICROTESLA, triad1.unit)
        assertEquals(triad1, triad2)
    }

    @Test
    fun toTriad_whenHardIron_returnsExpectedValues() {
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement =
            MagnetometerSensorMeasurement(
                bx,
                by,
                bz,
                hardIronX,
                hardIronY,
                hardIronZ,
                timestamp,
                SensorAccuracy.LOW,
                MagnetometerSensorType.MAGNETOMETER,
                SensorCoordinateSystem.NED
            )

        val triad1 = measurement.toTriad()
        val triad2 = MagneticFluxDensityTriad()
        measurement.toTriad(triad2)

        // check
        assertEquals(bx.toDouble() + hardIronX.toDouble(), triad1.valueX, 0.0)
        assertEquals(by.toDouble() + hardIronY.toDouble(), triad1.valueY, 0.0)
        assertEquals(bz + hardIronZ.toDouble(), triad1.valueZ, 0.0)
        assertEquals(MagneticFluxDensityUnit.MICROTESLA, triad1.unit)
        assertEquals(triad1, triad2)
    }

    @Test
    fun toNorm_whenNoHardIron_returnsExpectedValue() {
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement =
            MagnetometerSensorMeasurement(
                bx,
                by,
                bz,
                null,
                null,
                null,
                timestamp,
                SensorAccuracy.LOW,
                MagnetometerSensorType.MAGNETOMETER,
                SensorCoordinateSystem.NED
            )

        val norm = measurement.toNorm()

        // check
        val bx2 = bx.toDouble() * bx.toDouble()
        val by2 = by.toDouble() * by.toDouble()
        val bz2 = bz.toDouble() * bz.toDouble()
        val expected = sqrt(bx2 + by2 + bz2)
        assertEquals(expected, norm, 0.0)
    }

    @Test
    fun toNorm_whenHardIron_returnsExpectedValue() {
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement =
            MagnetometerSensorMeasurement(
                bx,
                by,
                bz,
                hardIronX,
                hardIronY,
                hardIronZ,
                timestamp,
                SensorAccuracy.LOW,
                MagnetometerSensorType.MAGNETOMETER,
                SensorCoordinateSystem.NED
            )

        val norm = measurement.toNorm()

        // check
        val x = bx.toDouble() + hardIronX.toDouble()
        val y = by.toDouble() + hardIronY.toDouble()
        val z = bz.toDouble() + hardIronZ.toDouble()
        val expected = sqrt(x * x + y * y + z * z)
        assertEquals(expected, norm, 0.0)
    }

    @Test
    fun equals_whenNull_returnsFalse() {
        val measurement = MagnetometerSensorMeasurement()
        assertFalse(measurement.equals(null))
    }

    @Test
    fun equals_whenSameInstance_returnsTrue() {
        val measurement = MagnetometerSensorMeasurement()
        @Suppress("ReplaceCallWithBinaryOperator")
        assertTrue(measurement.equals(measurement))
    }

    @Test
    fun equals_whenDifferentClass_returnsFalse() {
        val measurement = MagnetometerSensorMeasurement()
        assertFalse(measurement == Any())
    }

    @Test
    fun equals_whenDifferentContent_returnsFalse() {
        val randomizer = UniformRandomizer()
        val bx1 = randomizer.nextFloat()
        val by1 = randomizer.nextFloat()
        val bz1 = randomizer.nextFloat()
        val hardIronX1 = randomizer.nextFloat()
        val hardIronY1 = randomizer.nextFloat()
        val hardIronZ1 = randomizer.nextFloat()
        val timestamp1 = System.nanoTime()

        val bx2 = randomizer.nextFloat()
        val by2 = randomizer.nextFloat()
        val bz2 = randomizer.nextFloat()
        val hardIronX2 = randomizer.nextFloat()
        val hardIronY2 = randomizer.nextFloat()
        val hardIronZ2 = randomizer.nextFloat()
        val timestamp2 = System.nanoTime()

        val measurement1 = MagnetometerSensorMeasurement(
            bx1,
            by1,
            bz1,
            hardIronX1,
            hardIronY1,
            hardIronZ1,
            timestamp1,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER,
            SensorCoordinateSystem.NED
        )

        val measurement2 = MagnetometerSensorMeasurement(
            bx2,
            by2,
            bz2,
            hardIronX2,
            hardIronY2,
            hardIronZ2,
            timestamp2,
            SensorAccuracy.LOW,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentSensorAccuracy_returnsFalse() {
        val randomizer = UniformRandomizer()
        val bx1 = randomizer.nextFloat()
        val by1 = randomizer.nextFloat()
        val bz1 = randomizer.nextFloat()
        val hardIronX1 = randomizer.nextFloat()
        val hardIronY1 = randomizer.nextFloat()
        val hardIronZ1 = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val bx2 = randomizer.nextFloat()
        val by2 = randomizer.nextFloat()
        val bz2 = randomizer.nextFloat()
        val hardIronX2 = randomizer.nextFloat()
        val hardIronY2 = randomizer.nextFloat()
        val hardIronZ2 = randomizer.nextFloat()

        val measurement1 = MagnetometerSensorMeasurement(
            bx1,
            by1,
            bz1,
            hardIronX1,
            hardIronY1,
            hardIronZ1,
            timestamp,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER,
            SensorCoordinateSystem.NED
        )

        val measurement2 = MagnetometerSensorMeasurement(
            bx2,
            by2,
            bz2,
            hardIronX2,
            hardIronY2,
            hardIronZ2,
            timestamp,
            SensorAccuracy.LOW,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentBx_returnsFalse() {
        val randomizer = UniformRandomizer()
        val bx1 = randomizer.nextFloat()
        val by1 = randomizer.nextFloat()
        val bz1 = randomizer.nextFloat()
        val hardIronX1 = randomizer.nextFloat()
        val hardIronY1 = randomizer.nextFloat()
        val hardIronZ1 = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val bx2 = randomizer.nextFloat()
        val by2 = randomizer.nextFloat()
        val bz2 = randomizer.nextFloat()
        val hardIronX2 = randomizer.nextFloat()
        val hardIronY2 = randomizer.nextFloat()
        val hardIronZ2 = randomizer.nextFloat()

        val measurement1 = MagnetometerSensorMeasurement(
            bx1,
            by1,
            bz1,
            hardIronX1,
            hardIronY1,
            hardIronZ1,
            timestamp,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER,
            SensorCoordinateSystem.NED
        )

        val measurement2 = MagnetometerSensorMeasurement(
            bx2,
            by2,
            bz2,
            hardIronX2,
            hardIronY2,
            hardIronZ2,
            timestamp,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentBy_returnsFalse() {
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by1 = randomizer.nextFloat()
        val bz1 = randomizer.nextFloat()
        val hardIronX1 = randomizer.nextFloat()
        val hardIronY1 = randomizer.nextFloat()
        val hardIronZ1 = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val by2 = randomizer.nextFloat()
        val bz2 = randomizer.nextFloat()
        val hardIronX2 = randomizer.nextFloat()
        val hardIronY2 = randomizer.nextFloat()
        val hardIronZ2 = randomizer.nextFloat()

        val measurement1 = MagnetometerSensorMeasurement(
            bx,
            by1,
            bz1,
            hardIronX1,
            hardIronY1,
            hardIronZ1,
            timestamp,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER,
            SensorCoordinateSystem.NED
        )

        val measurement2 = MagnetometerSensorMeasurement(
            bx,
            by2,
            bz2,
            hardIronX2,
            hardIronY2,
            hardIronZ2,
            timestamp,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentBz_returnsFalse() {
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz1 = randomizer.nextFloat()
        val hardIronX1 = randomizer.nextFloat()
        val hardIronY1 = randomizer.nextFloat()
        val hardIronZ1 = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val bz2 = randomizer.nextFloat()
        val hardIronX2 = randomizer.nextFloat()
        val hardIronY2 = randomizer.nextFloat()
        val hardIronZ2 = randomizer.nextFloat()

        val measurement1 = MagnetometerSensorMeasurement(
            bx,
            by,
            bz1,
            hardIronX1,
            hardIronY1,
            hardIronZ1,
            timestamp,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER,
            SensorCoordinateSystem.NED
        )

        val measurement2 = MagnetometerSensorMeasurement(
            bx,
            by,
            bz2,
            hardIronX2,
            hardIronY2,
            hardIronZ2,
            timestamp,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentHardIronX_returnsFalse() {
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX1 = randomizer.nextFloat()
        val hardIronY1 = randomizer.nextFloat()
        val hardIronZ1 = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val hardIronX2 = randomizer.nextFloat()
        val hardIronY2 = randomizer.nextFloat()
        val hardIronZ2 = randomizer.nextFloat()

        val measurement1 = MagnetometerSensorMeasurement(
            bx,
            by,
            bz,
            hardIronX1,
            hardIronY1,
            hardIronZ1,
            timestamp,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER,
            SensorCoordinateSystem.NED
        )

        val measurement2 = MagnetometerSensorMeasurement(
            bx,
            by,
            bz,
            hardIronX2,
            hardIronY2,
            hardIronZ2,
            timestamp,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentHardIronY_returnsFalse() {
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY1 = randomizer.nextFloat()
        val hardIronZ1 = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val hardIronY2 = randomizer.nextFloat()
        val hardIronZ2 = randomizer.nextFloat()

        val measurement1 = MagnetometerSensorMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY1,
            hardIronZ1,
            timestamp,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER,
            SensorCoordinateSystem.NED
        )

        val measurement2 = MagnetometerSensorMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY2,
            hardIronZ2,
            timestamp,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentHardIronZ_returnsFalse() {
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ1 = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val hardIronZ2 = randomizer.nextFloat()

        val measurement1 = MagnetometerSensorMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ1,
            timestamp,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER,
            SensorCoordinateSystem.NED
        )

        val measurement2 = MagnetometerSensorMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ2,
            timestamp,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentTimestamp_returnsFalse() {
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp1 = System.nanoTime()

        val timestamp2 = System.nanoTime()

        val measurement1 = MagnetometerSensorMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            timestamp1,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER,
            SensorCoordinateSystem.NED
        )

        val measurement2 = MagnetometerSensorMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            timestamp2,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentSensorType_returnsFalse() {
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement1 = MagnetometerSensorMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            timestamp,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER,
            SensorCoordinateSystem.NED
        )

        val measurement2 = MagnetometerSensorMeasurement(
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

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentCoordinateSystem_returnsFalse() {
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement1 = MagnetometerSensorMeasurement(
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

        val measurement2 = MagnetometerSensorMeasurement(
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

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenEqualContent_returnsTrue() {
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement1 = MagnetometerSensorMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            timestamp,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER,
            SensorCoordinateSystem.NED
        )

        val measurement2 = MagnetometerSensorMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            timestamp,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER,
            SensorCoordinateSystem.NED
        )

        // check
        assertTrue(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun hashCode_whenEqualsObjects_returnsSameValue() {
        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement1 = MagnetometerSensorMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            timestamp,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER,
            SensorCoordinateSystem.NED
        )

        val measurement2 = MagnetometerSensorMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            timestamp,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER,
            SensorCoordinateSystem.NED
        )

        // check
        assertEquals(measurement1.hashCode(), measurement2.hashCode())
    }

    @Test
    fun hashCode_whenDifferentObjects_returnsDifferentValue() {
        val randomizer = UniformRandomizer()
        val bx1 = randomizer.nextFloat()
        val by1 = randomizer.nextFloat()
        val bz1 = randomizer.nextFloat()
        val hardIronX1 = randomizer.nextFloat()
        val hardIronY1 = randomizer.nextFloat()
        val hardIronZ1 = randomizer.nextFloat()
        val timestamp1 = System.nanoTime()

        val bx2 = randomizer.nextFloat()
        val by2 = randomizer.nextFloat()
        val bz2 = randomizer.nextFloat()
        val hardIronX2 = randomizer.nextFloat()
        val hardIronY2 = randomizer.nextFloat()
        val hardIronZ2 = randomizer.nextFloat()
        val timestamp2 = System.nanoTime()

        val measurement1 = MagnetometerSensorMeasurement(
            bx1,
            by1,
            bz1,
            hardIronX1,
            hardIronY1,
            hardIronZ1,
            timestamp1,
            SensorAccuracy.HIGH,
            MagnetometerSensorType.MAGNETOMETER,
            SensorCoordinateSystem.NED
        )

        val measurement2 = MagnetometerSensorMeasurement(
            bx2,
            by2,
            bz2,
            hardIronX2,
            hardIronY2,
            hardIronZ2,
            timestamp2,
            SensorAccuracy.LOW,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            SensorCoordinateSystem.ENU
        )

        // check
        assertNotEquals(measurement1.hashCode(), measurement2.hashCode())
    }
}