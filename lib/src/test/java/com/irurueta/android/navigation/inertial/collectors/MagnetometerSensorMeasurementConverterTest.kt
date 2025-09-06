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
//import io.mockk.clearAllMocks
//import io.mockk.every
//import io.mockk.impl.annotations.MockK
//import io.mockk.junit4.MockKRule
//import io.mockk.unmockkAll
import org.junit.After
import org.junit.Assert.*
//import org.junit.Ignore
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith
import org.mockito.Mock
import org.mockito.junit.MockitoJUnit
import org.mockito.junit.MockitoRule
import org.mockito.kotlin.whenever
import org.robolectric.RobolectricTestRunner

//@Ignore("Possible memory leak when running this test")
@RunWith(RobolectricTestRunner::class)
class MagnetometerSensorMeasurementConverterTest {

    @get:Rule
    val mockitoRule: MockitoRule = MockitoJUnit.rule()

//    @get:Rule
//    val mockkRule = MockKRule(this)

//    @MockK
    @Mock
    private lateinit var sensor: Sensor

//    @MockK
    @Mock
    private lateinit var event: SensorEvent

    /*@After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
        System.gc()
    }*/

    @Test
    fun convert_whenNoSensorEvent_returnsFalse() {
        val measurement = MagnetometerSensorMeasurement()
        assertFalse(MagnetometerSensorMeasurementConverter.convert(null, measurement))
    }

    @Test
    fun convert_whenUnknownSensorType_returnsFalse() {
        whenever(sensor.type).thenReturn(Sensor.TYPE_GYROSCOPE)
//        every { sensor.type }.returns(Sensor.TYPE_GYROSCOPE)
        event.sensor = sensor
        val measurement = MagnetometerSensorMeasurement()

        assertFalse(MagnetometerSensorMeasurementConverter.convert(event, measurement))
    }

    @Test
    fun convert_whenMagnetometerSensorTypeWithoutStartOffset_returnsTrue() {
        whenever(sensor.type).thenReturn(Sensor.TYPE_MAGNETIC_FIELD)
//        every { sensor.type }.returns(Sensor.TYPE_MAGNETIC_FIELD)
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
        whenever(sensor.type).thenReturn(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED)
//        every { sensor.type }.returns(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED)
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
        whenever(sensor.type).thenReturn(Sensor.TYPE_MAGNETIC_FIELD)
//        every { sensor.type }.returns(Sensor.TYPE_MAGNETIC_FIELD)
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
        whenever(sensor.type).thenReturn(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED)
//        every { sensor.type }.returns(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED)
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