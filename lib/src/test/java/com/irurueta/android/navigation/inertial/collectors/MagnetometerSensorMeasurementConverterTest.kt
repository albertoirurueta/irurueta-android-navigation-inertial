/*
 * Copyright (C) 2022 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.android.navigation.inertial.collectors

import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorManager
import com.irurueta.statistics.UniformRandomizer
import io.mockk.clearAllMocks
import io.mockk.every
import io.mockk.mockk
import org.junit.After
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class MagnetometerSensorMeasurementConverterTest {

    @After
    fun tearDown() {
        clearAllMocks()
    }

    @Test
    fun convert_whenNoSensorEvent_returnsFalse() {
        val measurement = MagnetometerSensorMeasurement()
        assertFalse(MagnetometerSensorMeasurementConverter.convert(null, measurement))
    }

    @Test
    fun convert_whenUnknownSensorType_returnsFalse() {
        val sensor = mockk<Sensor>()
        every { sensor.type }.returns(Sensor.TYPE_GYROSCOPE)
        val event = mockk<SensorEvent>()
        event.sensor = sensor
        val measurement = MagnetometerSensorMeasurement()

        assertFalse(MagnetometerSensorMeasurementConverter.convert(event, measurement))
    }

    @Test
    fun convert_whenMagnetometerSensorTypeWithoutStartOffset_returnsTrue() {
        val sensor = mockk<Sensor>()
        every { sensor.type }.returns(Sensor.TYPE_MAGNETIC_FIELD)
        val event = mockk<SensorEvent>()
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

        assertTrue(MagnetometerSensorMeasurementConverter.convert(event, measurement))

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
    }

    @Test
    fun convert_whenMagnetometerUncalibratedSensorTypeWithoutStartOffset_returnsTrue() {
        val sensor = mockk<Sensor>()
        every { sensor.type }.returns(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED)
        val event = mockk<SensorEvent>()
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

        assertTrue(MagnetometerSensorMeasurementConverter.convert(event, measurement))

        // check measurement values
        assertEquals(bx, measurement.bx, 0.0f)
        assertEquals(by, measurement.by, 0.0f)
        assertEquals(bz, measurement.bz, 0.0f)
        assertEquals(hardIronX, measurement.hardIronX)
        assertEquals(hardIronY, measurement.hardIronY)
        assertEquals(hardIronZ, measurement.hardIronZ)
        assertEquals(timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.LOW, measurement.accuracy)
        assertEquals(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED, measurement.sensorType)
    }

    @Test
    fun convert_whenMagnetometerSensorTypeWithStartOffset_returnsTrue() {
        val sensor = mockk<Sensor>()
        every { sensor.type }.returns(Sensor.TYPE_MAGNETIC_FIELD)
        val event = mockk<SensorEvent>()
        event.sensor = sensor

        val timestamp = System.nanoTime()
        event.timestamp = timestamp

        val randomizer = UniformRandomizer()
        val startOffset = randomizer.nextLong(-MAX_OFFSET_NANOS, MAX_OFFSET_NANOS)
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val values = floatArrayOf(bx, by, bz, 4.0f, 5.0f, 6.0f)
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, values)

        event.accuracy = SensorManager.SENSOR_STATUS_ACCURACY_LOW

        val measurement = MagnetometerSensorMeasurement()

        assertTrue(MagnetometerSensorMeasurementConverter.convert(event, measurement, startOffset))

        // check measurement values
        assertEquals(bx, measurement.bx, 0.0f)
        assertEquals(by, measurement.by, 0.0f)
        assertEquals(bz, measurement.bz, 0.0f)
        assertNull(measurement.hardIronX)
        assertNull(measurement.hardIronY)
        assertNull(measurement.hardIronZ)
        assertEquals(timestamp + startOffset, measurement.timestamp)
        assertEquals(SensorAccuracy.LOW, measurement.accuracy)
        assertEquals(MagnetometerSensorType.MAGNETOMETER, measurement.sensorType)
    }

    @Test
    fun convert_whenMagnetometerUncalibratedSensorTypeWithStartOffset_returnsTrue() {
        val sensor = mockk<Sensor>()
        every { sensor.type }.returns(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED)
        val event = mockk<SensorEvent>()
        event.sensor = sensor

        val timestamp = System.nanoTime()
        event.timestamp = timestamp

        val randomizer = UniformRandomizer()
        val startOffset = randomizer.nextLong(-MAX_OFFSET_NANOS, MAX_OFFSET_NANOS)
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

        assertTrue(MagnetometerSensorMeasurementConverter.convert(event, measurement, startOffset))

        // check measurement values
        assertEquals(bx, measurement.bx, 0.0f)
        assertEquals(by, measurement.by, 0.0f)
        assertEquals(bz, measurement.bz, 0.0f)
        assertEquals(hardIronX, measurement.hardIronX)
        assertEquals(hardIronY, measurement.hardIronY)
        assertEquals(hardIronZ, measurement.hardIronZ)
        assertEquals(timestamp + startOffset, measurement.timestamp)
        assertEquals(SensorAccuracy.LOW, measurement.accuracy)
        assertEquals(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED, measurement.sensorType)
    }

    private companion object {
        const val MAX_OFFSET_NANOS = 1000000L
    }
}