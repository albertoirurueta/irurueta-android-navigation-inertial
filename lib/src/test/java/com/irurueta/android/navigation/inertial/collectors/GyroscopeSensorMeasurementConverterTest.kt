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
//import org.junit.After
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
class GyroscopeSensorMeasurementConverterTest {

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
        val measurement = GyroscopeSensorMeasurement()
        assertFalse(GyroscopeSensorMeasurementConverter.convert(null, measurement))
    }

    @Test
    fun convert_whenUnknownSensorType_returnsFalse() {
        whenever(sensor.type).thenReturn(Sensor.TYPE_ACCELEROMETER)
//        every { sensor.type }.returns(Sensor.TYPE_ACCELEROMETER)
        event.sensor = sensor
        val measurement = GyroscopeSensorMeasurement()

        assertFalse(GyroscopeSensorMeasurementConverter.convert(event, measurement))
    }

    @Test
    fun convert_whenGyroscopeSensorTypeWithoutStartOffset_returnsTrue() {
        whenever(sensor.type).thenReturn(Sensor.TYPE_GYROSCOPE)
//        every { sensor.type }.returns(Sensor.TYPE_GYROSCOPE)
        event.sensor = sensor

        val timestamp = System.nanoTime()
        event.timestamp = timestamp

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val values = floatArrayOf(wx, wy, wz, 4.0f, 5.0f, 6.0f)
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, values)

        event.accuracy = SensorManager.SENSOR_STATUS_ACCURACY_HIGH

        val measurement = GyroscopeSensorMeasurement()

        assertTrue(GyroscopeSensorMeasurementConverter.convert(event, measurement))

        // check measurement values
        assertEquals(wx, measurement.wx, 0.0f)
        assertEquals(wy, measurement.wy, 0.0f)
        assertEquals(wz, measurement.wz, 0.0f)
        assertNull(measurement.bx)
        assertNull(measurement.by)
        assertNull(measurement.bz)
        assertEquals(timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
        assertEquals(GyroscopeSensorType.GYROSCOPE, measurement.sensorType)
    }

    @Test
    fun convert_whenGyroscopeUncalibratedSensorTypeWithoutStartOffset_returnsTrue() {
        whenever(sensor.type).thenReturn(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
//        every { sensor.type }.returns(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
        event.sensor = sensor

        val timestamp = System.nanoTime()
        event.timestamp = timestamp

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val values = floatArrayOf(wx, wy, wz, bx, by, bz)
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, values)

        event.accuracy = SensorManager.SENSOR_STATUS_ACCURACY_HIGH

        val measurement = GyroscopeSensorMeasurement()

        assertTrue(GyroscopeSensorMeasurementConverter.convert(event, measurement))

        // check measurement values
        assertEquals(wx, measurement.wx, 0.0f)
        assertEquals(wy, measurement.wy, 0.0f)
        assertEquals(wz, measurement.wz, 0.0f)
        assertEquals(bx, measurement.bx)
        assertEquals(by, measurement.by)
        assertEquals(bz, measurement.bz)
        assertEquals(timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
        assertEquals(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED, measurement.sensorType)
    }

    @Test
    fun convert_whenGyroscopeSensorTypeWithStartOffset_returnsTrue() {
        whenever(sensor.type).thenReturn(Sensor.TYPE_GYROSCOPE)
//        every { sensor.type }.returns(Sensor.TYPE_GYROSCOPE)
        event.sensor = sensor

        val timestamp = System.nanoTime()
        event.timestamp = timestamp

        val randomizer = UniformRandomizer()
        val startOffset = randomizer.nextLong(-MAX_OFFSET_NANOS, MAX_OFFSET_NANOS)
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val values = floatArrayOf(wx, wy, wz, 4.0f, 5.0f, 6.0f)
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, values)

        event.accuracy = SensorManager.SENSOR_STATUS_ACCURACY_HIGH

        val measurement = GyroscopeSensorMeasurement()

        assertTrue(GyroscopeSensorMeasurementConverter.convert(event, measurement, startOffset))

        // check measurement values
        assertEquals(wx, measurement.wx, 0.0f)
        assertEquals(wy, measurement.wy, 0.0f)
        assertEquals(wz, measurement.wz, 0.0f)
        assertNull(measurement.bx)
        assertNull(measurement.by)
        assertNull(measurement.bz)
        assertEquals(timestamp + startOffset, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
        assertEquals(GyroscopeSensorType.GYROSCOPE, measurement.sensorType)
    }

    @Test
    fun convert_whenGyroscopeUncalibratedSensorTypeWithStartOffset_returnsTrue() {
        whenever(sensor.type).thenReturn(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
//        every { sensor.type }.returns(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
        event.sensor = sensor

        val timestamp = System.nanoTime()
        event.timestamp = timestamp

        val randomizer = UniformRandomizer()
        val startOffset = randomizer.nextLong(-MAX_OFFSET_NANOS, MAX_OFFSET_NANOS)
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val values = floatArrayOf(wx, wy, wz, bx, by, bz)
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, values)

        event.accuracy = SensorManager.SENSOR_STATUS_ACCURACY_HIGH

        val measurement = GyroscopeSensorMeasurement()

        assertTrue(GyroscopeSensorMeasurementConverter.convert(event, measurement, startOffset))

        // check measurement values
        assertEquals(wx, measurement.wx, 0.0f)
        assertEquals(wy, measurement.wy, 0.0f)
        assertEquals(wz, measurement.wz, 0.0f)
        assertEquals(bx, measurement.bx)
        assertEquals(by, measurement.by)
        assertEquals(bz, measurement.bz)
        assertEquals(timestamp + startOffset, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
        assertEquals(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED, measurement.sensorType)
    }

    private companion object {
        const val MAX_OFFSET_NANOS = 1000000L
    }
}