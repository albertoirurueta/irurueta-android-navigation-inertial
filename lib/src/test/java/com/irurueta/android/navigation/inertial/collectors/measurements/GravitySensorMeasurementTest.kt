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

import com.irurueta.android.navigation.inertial.collectors.converters.GravitySensorMeasurementCoordinateSystemConverter
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

class GravitySensorMeasurementTest {

    @Test
    fun constructor_whenDefault_setsExpectedValues() {
        val measurement = GravitySensorMeasurement()

        // check
        assertEquals(0.0f, measurement.gx, 0.0f)
        assertEquals(0.0f, measurement.gy, 0.0f)
        assertEquals(0.0f, measurement.gz, 0.0f)
        assertEquals(0L, measurement.timestamp)
        assertNull(measurement.accuracy)
        assertEquals(
            SensorCoordinateSystem.ENU,
            measurement.sensorCoordinateSystem
        )
    }

    @Test
    fun constructor_withRequiredProperties_setsExpectedValues() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement =
            GravitySensorMeasurement(
                gx, gy, gz, timestamp, null, SensorCoordinateSystem.NED
            )

        // check
        assertEquals(gx, measurement.gx, 0.0f)
        assertEquals(gy, measurement.gy, 0.0f)
        assertEquals(gz, measurement.gz, 0.0f)
        assertEquals(timestamp, measurement.timestamp)
        assertNull(measurement.accuracy)
        assertEquals(
            SensorCoordinateSystem.NED,
            measurement.sensorCoordinateSystem
        )
    }

    @Test
    fun constructor_withAllProperties_setsExpectedValues() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement =
            GravitySensorMeasurement(
                gx,
                gy,
                gz,
                timestamp,
                SensorAccuracy.MEDIUM,
                SensorCoordinateSystem.NED
            )

        // check
        assertEquals(gx, measurement.gx, 0.0f)
        assertEquals(gy, measurement.gy, 0.0f)
        assertEquals(gz, measurement.gz, 0.0f)
        assertEquals(timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, measurement.accuracy)
        assertEquals(
            SensorCoordinateSystem.NED,
            measurement.sensorCoordinateSystem
        )
    }

    @Test
    fun constructor_whenCopy_setsExpectedValues() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement1 =
            GravitySensorMeasurement(
                gx,
                gy,
                gz,
                timestamp,
                SensorAccuracy.MEDIUM,
                SensorCoordinateSystem.NED
            )

        val measurement2 = GravitySensorMeasurement(measurement1)

        // check
        assertEquals(gx, measurement1.gx, 0.0f)
        assertEquals(gy, measurement1.gy, 0.0f)
        assertEquals(gz, measurement1.gz, 0.0f)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, measurement1.accuracy)
        assertEquals(
            SensorCoordinateSystem.NED,
            measurement1.sensorCoordinateSystem
        )

        assertEquals(measurement1.gx, measurement2.gx, 0.0f)
        assertEquals(measurement1.gy, measurement2.gy, 0.0f)
        assertEquals(measurement1.gz, measurement2.gz, 0.0f)
        assertEquals(measurement1.timestamp, measurement2.timestamp)
        assertEquals(measurement1.accuracy, measurement2.accuracy)
        assertEquals(
            measurement1.sensorCoordinateSystem,
            measurement2.sensorCoordinateSystem
        )
    }

    @Test
    fun gx_setsExpectedValue() {
        val measurement = GravitySensorMeasurement()

        // check default value
        assertEquals(0.0f, measurement.gx, 0.0f)

        // set new value
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        measurement.gx = gx

        // check
        assertEquals(gx, measurement.gx, 0.0f)
    }

    @Test
    fun gy_setsExpectedValue() {
        val measurement = GravitySensorMeasurement()

        // check default value
        assertEquals(0.0f, measurement.gy, 0.0f)

        // set new value
        val randomizer = UniformRandomizer()
        val gy = randomizer.nextFloat()
        measurement.gy = gy

        // check
        assertEquals(gy, measurement.gy, 0.0f)
    }

    @Test
    fun gz_setsExpectedValue() {
        val measurement = GravitySensorMeasurement()

        // check default value
        assertEquals(0.0f, measurement.gz, 0.0f)

        // set new value
        val randomizer = UniformRandomizer()
        val gz = randomizer.nextFloat()
        measurement.gz = gz

        // check
        assertEquals(gz, measurement.gz, 0.0f)
    }

    @Test
    fun timestamp_setsExpectedValue() {
        val measurement = GravitySensorMeasurement()

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
        val measurement = GravitySensorMeasurement()

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
    fun sensorCoordinateSystem_setsExpectedValue() {
        val measurement = GravitySensorMeasurement()

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
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement1 =
            GravitySensorMeasurement(
                gx,
                gy,
                gz,
                timestamp,
                SensorAccuracy.MEDIUM,
                SensorCoordinateSystem.NED
            )

        val measurement2 = GravitySensorMeasurement()
        measurement2.copyFrom(measurement1)

        // check
        assertEquals(gx, measurement1.gx, 0.0f)
        assertEquals(gy, measurement1.gy, 0.0f)
        assertEquals(gz, measurement1.gz, 0.0f)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, measurement1.accuracy)
        assertEquals(
            SensorCoordinateSystem.NED,
            measurement1.sensorCoordinateSystem
        )

        assertEquals(measurement1.gx, measurement2.gx, 0.0f)
        assertEquals(measurement1.gy, measurement2.gy, 0.0f)
        assertEquals(measurement1.gz, measurement2.gz, 0.0f)
        assertEquals(measurement1.timestamp, measurement2.timestamp)
        assertEquals(measurement1.accuracy, measurement2.accuracy)
        assertEquals(
            measurement1.sensorCoordinateSystem,
            measurement2.sensorCoordinateSystem
        )
    }

    @Test
    fun copyTo_makesExpectedCopy() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement1 =
            GravitySensorMeasurement(
                gx,
                gy,
                gz,
                timestamp,
                SensorAccuracy.MEDIUM,
                SensorCoordinateSystem.NED
            )

        val measurement2 = GravitySensorMeasurement()
        measurement1.copyTo(measurement2)

        // check
        assertEquals(gx, measurement1.gx, 0.0f)
        assertEquals(gy, measurement1.gy, 0.0f)
        assertEquals(gz, measurement1.gz, 0.0f)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, measurement1.accuracy)
        assertEquals(
            SensorCoordinateSystem.NED,
            measurement1.sensorCoordinateSystem
        )

        assertEquals(measurement1.gx, measurement2.gx, 0.0f)
        assertEquals(measurement1.gy, measurement2.gy, 0.0f)
        assertEquals(measurement1.gz, measurement2.gz, 0.0f)
        assertEquals(measurement1.timestamp, measurement2.timestamp)
        assertEquals(measurement1.accuracy, measurement2.accuracy)
        assertEquals(
            measurement1.sensorCoordinateSystem,
            measurement2.sensorCoordinateSystem
        )
    }

    @Test
    fun copy_makesExpectedCopy() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement1 =
            GravitySensorMeasurement(
                gx,
                gy,
                gz,
                timestamp,
                SensorAccuracy.MEDIUM,
                SensorCoordinateSystem.NED
            )

        val measurement2 = measurement1.copy()

        // check
        assertEquals(gx, measurement1.gx, 0.0f)
        assertEquals(gy, measurement1.gy, 0.0f)
        assertEquals(gz, measurement1.gz, 0.0f)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, measurement1.accuracy)
        assertEquals(
            SensorCoordinateSystem.NED,
            measurement1.sensorCoordinateSystem
        )

        assertEquals(measurement1.gx, measurement2.gx, 0.0f)
        assertEquals(measurement1.gy, measurement2.gy, 0.0f)
        assertEquals(measurement1.gz, measurement2.gz, 0.0f)
        assertEquals(measurement1.timestamp, measurement2.timestamp)
        assertEquals(measurement1.accuracy, measurement2.accuracy)
        assertEquals(
            measurement1.sensorCoordinateSystem,
            measurement2.sensorCoordinateSystem
        )
    }

    @Test
    fun toNedOrThrow_whenENU_returnsExpectedValue() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement =
            GravitySensorMeasurement(
                gx,
                gy,
                gz,
                timestamp,
                SensorAccuracy.MEDIUM,
                SensorCoordinateSystem.ENU
            )

        val nedMeasurement1 = measurement.toNedOrThrow()
        val nedMeasurement2 = GravitySensorMeasurement()
        measurement.toNedOrThrow(nedMeasurement2)

        val nedMeasurement3 = GravitySensorMeasurementCoordinateSystemConverter.toNedOrThrow(
            measurement)

        // check
        assertEquals(nedMeasurement1, nedMeasurement2)
        assertEquals(nedMeasurement1, nedMeasurement3)
    }

    @Test
    fun toNedOrThrow_whenNED_throwsIllegalArgumentException() {
        val measurement = GravitySensorMeasurement(
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
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement =
            GravitySensorMeasurement(
                gx,
                gy,
                gz,
                timestamp,
                SensorAccuracy.MEDIUM,
                SensorCoordinateSystem.NED
            )

        val enuMeasurement1 = measurement.toEnuOrThrow()
        val enuMeasurement2 = GravitySensorMeasurement()
        measurement.toEnuOrThrow(enuMeasurement2)

        val enuMeasurement3 = GravitySensorMeasurementCoordinateSystemConverter.toEnuOrThrow(
            measurement)

        // check
        assertEquals(enuMeasurement1, enuMeasurement2)
        assertEquals(enuMeasurement1, enuMeasurement3)
    }

    @Test
    fun toEnuOrThrow_whenENU_throwsIllegalArgumentException() {
        val measurement = GravitySensorMeasurement(
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
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement =
            GravitySensorMeasurement(
                gx,
                gy,
                gz,
                timestamp,
                SensorAccuracy.MEDIUM,
                SensorCoordinateSystem.NED
            )

        val result1 = measurement.toNed()
        val result2 = GravitySensorMeasurement()
        measurement.toNed(result2)

        // check
        assertEquals(measurement, result1)
        assertEquals(measurement, result2)
    }

    @Test
    fun toNed_whenENU_returnsConvertedValue() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement =
            GravitySensorMeasurement(
                gx,
                gy,
                gz,
                timestamp,
                SensorAccuracy.MEDIUM,
                SensorCoordinateSystem.ENU
            )

        val result1 = measurement.toNed()
        val result2 = GravitySensorMeasurement()
        measurement.toNed(result2)

        val expected = GravitySensorMeasurementCoordinateSystemConverter.toNedOrThrow(
            measurement)

        // check
        assertEquals(expected, result1)
        assertEquals(expected, result2)
    }

    @Test
    fun toEnu_whenENU_returnsCopy() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement =
            GravitySensorMeasurement(
                gx,
                gy,
                gz,
                timestamp,
                SensorAccuracy.MEDIUM,
                SensorCoordinateSystem.ENU
            )

        val result1 = measurement.toEnu()
        val result2 = GravitySensorMeasurement()
        measurement.toEnu(result2)

        // check
        assertEquals(measurement, result1)
        assertEquals(measurement, result2)
    }

    @Test
    fun toEnu_whenNED_returnsConvertedValue() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement =
            GravitySensorMeasurement(
                gx,
                gy,
                gz,
                timestamp,
                SensorAccuracy.MEDIUM,
                SensorCoordinateSystem.NED
            )

        val result1 = measurement.toEnu()
        val result2 = GravitySensorMeasurement()
        measurement.toEnu(result2)

        val expected = GravitySensorMeasurementCoordinateSystemConverter.toEnuOrThrow(
            measurement)

        // check
        assertEquals(expected, result1)
        assertEquals(expected, result2)
    }

    @Test
    fun toTriad_returnsExpectedValue() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            timestamp,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.ENU
        )

        val triad1 = measurement.toTriad()
        val triad2 = AccelerationTriad()
        measurement.toTriad(triad2)

        // check
        assertEquals(gx.toDouble(), triad1.valueX, 0.0)
        assertEquals(gy.toDouble(), triad1.valueY, 0.0)
        assertEquals(gz.toDouble(), triad1.valueZ, 0.0)
        assertEquals(AccelerationUnit.METERS_PER_SQUARED_SECOND, triad1.unit)
        assertEquals(triad1, triad2)
    }

    @Test
    fun toNorm_returnsExpectedValue() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            timestamp,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.ENU
        )

        val norm = measurement.toNorm()

        // check
        val gx2 = gx.toDouble() * gx.toDouble()
        val gy2 = gy.toDouble() * gy.toDouble()
        val gz2 = gz.toDouble() * gz.toDouble()
        val expected = sqrt(gx2 + gy2 + gz2)
        assertEquals(expected, norm, 0.0)
    }

    @Test
    fun equals_whenNull_returnsFalse() {
        val measurement = GravitySensorMeasurement()
        assertFalse(measurement.equals(null))
    }

    @Test
    fun equals_whenSameInstance_returnsTrue() {
        val measurement = GravitySensorMeasurement()
        @Suppress("ReplaceCallWithBinaryOperator")
        assert(measurement.equals(measurement))
    }

    @Test
    fun equals_whenDifferentType_returnsFalse() {
        val measurement = GravitySensorMeasurement()
        assertFalse(measurement == Any())
    }

    @Test
    fun equals_whenDifferentContent_returnsFalse() {
        val randomizer = UniformRandomizer()
        val gx1 = randomizer.nextFloat()
        val gy1 = randomizer.nextFloat()
        val gz1 = randomizer.nextFloat()
        val timestamp1 = System.nanoTime()

        val gx2 = gx1 + 1.0f
        val gy2 = gy1 + 1.0f
        val gz2 = gz1 + 1.0f
        val timestamp2 = timestamp1 + 1L

        val measurement1 = GravitySensorMeasurement(
            gx1,
            gy1,
            gz1,
            timestamp1,
            SensorAccuracy.MEDIUM,
            SensorCoordinateSystem.NED
        )

        val measurement2 = GravitySensorMeasurement(
            gx2,
            gy2,
            gz2,
            timestamp2,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.ENU
        )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentSensorAccuracy_returnsFalse() {
        val randomizer = UniformRandomizer()
        val gx1 = randomizer.nextFloat()
        val gy1 = randomizer.nextFloat()
        val gz1 = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val gx2 = gx1 + 1.0f
        val gy2 = gy1 + 1.0f
        val gz2 = gz1 + 1.0f

        val measurement1 = GravitySensorMeasurement(
            gx1,
            gy1,
            gz1,
            timestamp,
            SensorAccuracy.MEDIUM,
            SensorCoordinateSystem.NED
        )

        val measurement2 = GravitySensorMeasurement(
            gx2,
            gy2,
            gz2,
            timestamp,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.ENU
        )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentGx_returnsFalse() {
        val randomizer = UniformRandomizer()
        val gx1 = randomizer.nextFloat()
        val gy1 = randomizer.nextFloat()
        val gz1 = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val gx2 = gx1 + 1.0f
        val gy2 = gy1 + 1.0f
        val gz2 = gz1 + 1.0f

        val measurement1 = GravitySensorMeasurement(
            gx1,
            gy1,
            gz1,
            timestamp,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.NED
        )

        val measurement2 = GravitySensorMeasurement(
            gx2,
            gy2,
            gz2,
            timestamp,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.ENU
        )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentGy_returnsFalse() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy1 = randomizer.nextFloat()
        val gz1 = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val gy2 = gy1 + 1.0f
        val gz2 = gz1 + 1.0f

        val measurement1 = GravitySensorMeasurement(
            gx,
            gy1,
            gz1,
            timestamp,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.NED
        )

        val measurement2 = GravitySensorMeasurement(
            gx,
            gy2,
            gz2,
            timestamp,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.ENU
        )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentGz_returnsFalse() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz1 = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val gz2 = gz1 + 1.0f

        val measurement1 = GravitySensorMeasurement(
            gx,
            gy,
            gz1,
            timestamp,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.NED
        )

        val measurement2 = GravitySensorMeasurement(
            gx,
            gy,
            gz2,
            timestamp,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.ENU
        )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentTimestamp_returnsFalse() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val timestamp1 = System.nanoTime()

        val timestamp2 = timestamp1 + 1L

        val measurement1 = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            timestamp1,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.NED
        )

        val measurement2 = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            timestamp2,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.NED
        )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenDifferentCoordinateSystem_returnsFalse() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement1 = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            timestamp,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.ENU
        )

        val measurement2 = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            timestamp,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.NED
        )

        assertFalse(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun equals_whenEqualContent_returnsTrue() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement1 = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            timestamp,
            SensorAccuracy.MEDIUM,
            SensorCoordinateSystem.NED
        )

        val measurement2 = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            timestamp,
            SensorAccuracy.MEDIUM,
            SensorCoordinateSystem.NED
        )

        assertTrue(measurement1 == measurement2)
        assertNotSame(measurement1, measurement2)
    }

    @Test
    fun hashCode_whenEqualObjects_returnsSameValue() {
        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val timestamp = System.nanoTime()

        val measurement1 = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            timestamp,
            SensorAccuracy.MEDIUM,
            SensorCoordinateSystem.NED
        )

        val measurement2 = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            timestamp,
            SensorAccuracy.MEDIUM,
            SensorCoordinateSystem.NED
        )

        assertEquals(measurement1.hashCode(), measurement2.hashCode())
    }

    @Test
    fun hashCode_whenDifferentObjects_returnsDifferentValue() {
        val randomizer = UniformRandomizer()
        val gx1 = randomizer.nextFloat()
        val gy1 = randomizer.nextFloat()
        val gz1 = randomizer.nextFloat()
        val timestamp1 = System.nanoTime()

        val gx2 = randomizer.nextFloat()
        val gy2 = randomizer.nextFloat()
        val gz2 = randomizer.nextFloat()
        val timestamp2 = System.nanoTime()

        val measurement1 = GravitySensorMeasurement(
            gx1,
            gy1,
            gz1,
            timestamp1,
            SensorAccuracy.MEDIUM,
            SensorCoordinateSystem.NED
        )

        val measurement2 = GravitySensorMeasurement(
            gx2,
            gy2,
            gz2,
            timestamp2,
            SensorAccuracy.HIGH,
            SensorCoordinateSystem.ENU
        )

        assertNotEquals(measurement1.hashCode(), measurement2.hashCode())
    }
}