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

import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorManager
import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorCoordinateSystem
import com.irurueta.statistics.UniformRandomizer
import io.mockk.every
import io.mockk.impl.annotations.MockK
import io.mockk.junit4.MockKRule
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNull
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test

class MagnetometerSensorEventMeasurementConverterTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK
    private lateinit var sensor: Sensor

    @MockK
    private lateinit var event: SensorEvent

    @Test
    fun convert_whenNoSensorEvent_returnsFalse() {
        val measurement = MagnetometerSensorMeasurement()
        assertFalse(MagnetometerSensorEventMeasurementConverter.convert(null, measurement))
    }

    @Test
    fun convert_whenUnknownSensorType_returnsFalse() {
        every { sensor.type }.returns(Sensor.TYPE_GYROSCOPE)
        event.sensor = sensor
        val measurement = MagnetometerSensorMeasurement()

        assertFalse(
            MagnetometerSensorEventMeasurementConverter.convert(
                event,
                measurement
            )
        )
    }

    @Test
    fun convert_whenMagnetometerSensorType_returnsTrue() {
        every { sensor.type }.returns(Sensor.TYPE_MAGNETIC_FIELD)
        event.sensor = sensor

        val timestamp = System.nanoTime()
        event.timestamp = timestamp

        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val values = floatArrayOf(bx, by, bz, 4.0f, 5.0f, 6.0f)
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, values)

        event.accuracy = SensorManager.SENSOR_STATUS_ACCURACY_LOW

        val measurement = MagnetometerSensorMeasurement()

        assertTrue(MagnetometerSensorEventMeasurementConverter.convert(event, measurement))

        // check measurement values
        assertEquals(bx, measurement.bx, 0.0f)
        assertEquals(by, measurement.by, 0.0f)
        assertEquals(bz, measurement.bz, 0.0f)
        assertNull(measurement.hardIronX)
        assertNull(measurement.hardIronY)
        assertNull(measurement.hardIronZ)
        assertEquals(timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.LOW, measurement.accuracy)
        assertEquals(MagnetometerSensorType.MAGNETOMETER, measurement.sensorType)
        assertEquals(
            SensorCoordinateSystem.ENU,
            measurement.sensorCoordinateSystem
        )
    }

    @Test
    fun convert_whenMagnetometerUncalibratedSensorType_returnsTrue() {
        every { sensor.type }.returns(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED)
        event.sensor = sensor

        val timestamp = System.nanoTime()
        event.timestamp = timestamp

        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val values = floatArrayOf(bx, by, bz, hardIronX, hardIronY, hardIronZ)
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, values)

        event.accuracy = SensorManager.SENSOR_STATUS_ACCURACY_LOW

        val measurement = MagnetometerSensorMeasurement()

        assertTrue(
            MagnetometerSensorEventMeasurementConverter.convert(
                event,
                measurement
            )
        )

        // check measurement values
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
            SensorCoordinateSystem.ENU,
            measurement.sensorCoordinateSystem
        )
    }

    @Test
    fun convert_whenNoSensorEvent_returnsNull() {
        assertNull(MagnetometerSensorEventMeasurementConverter.convert(null))
    }

    @Test
    fun convert_whenUnknownSensorType_returnsNull() {
        every { sensor.type }.returns(Sensor.TYPE_GYROSCOPE)
        event.sensor = sensor
        assertNull(MagnetometerSensorEventMeasurementConverter.convert(event))
    }

    @Test
    fun convert_whenMagnetometerSensorType_returnsExpectedResult() {
        every { sensor.type }.returns(Sensor.TYPE_MAGNETIC_FIELD)
        event.sensor = sensor

        val timestamp = System.nanoTime()
        event.timestamp = timestamp

        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val values = floatArrayOf(bx, by, bz, 4.0f, 5.0f, 6.0f)
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, values)

        event.accuracy = SensorManager.SENSOR_STATUS_ACCURACY_LOW

        val measurement = MagnetometerSensorEventMeasurementConverter.convert(event)

        // check measurement values
        requireNotNull(measurement)
        assertEquals(bx, measurement.bx, 0.0f)
        assertEquals(by, measurement.by, 0.0f)
        assertEquals(bz, measurement.bz, 0.0f)
        assertNull(measurement.hardIronX)
        assertNull(measurement.hardIronY)
        assertNull(measurement.hardIronZ)
        assertEquals(timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.LOW, measurement.accuracy)
        assertEquals(MagnetometerSensorType.MAGNETOMETER, measurement.sensorType)
        assertEquals(
            SensorCoordinateSystem.ENU,
            measurement.sensorCoordinateSystem
        )
    }

    @Test
    fun convert_whenMagnetometerUncalibratedSensorType_returnsExpectedResult() {
        every { sensor.type }.returns(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED)
        event.sensor = sensor

        val timestamp = System.nanoTime()
        event.timestamp = timestamp

        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val values = floatArrayOf(bx, by, bz, hardIronX, hardIronY, hardIronZ)
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, values)

        event.accuracy = SensorManager.SENSOR_STATUS_ACCURACY_LOW

        val measurement = MagnetometerSensorEventMeasurementConverter.convert(event)

        // check measurement values
        requireNotNull(measurement)
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
            SensorCoordinateSystem.ENU,
            measurement.sensorCoordinateSystem
        )
    }
}