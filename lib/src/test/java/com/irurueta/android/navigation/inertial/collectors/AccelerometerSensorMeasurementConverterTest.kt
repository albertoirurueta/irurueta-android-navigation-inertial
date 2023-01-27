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
import io.mockk.every
import io.mockk.mockk
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class AccelerometerSensorMeasurementConverterTest {

    @Test
    fun convert_whenNoSensorEvent_returnsFalse() {
        val measurement = AccelerometerSensorMeasurement()
        assertFalse(AccelerometerSensorMeasurementConverter.convert(null, measurement))
    }

    @Test
    fun convert_whenUnknownSensorType_returnsFalse() {
        val sensor = mockk<Sensor>()
        every { sensor.type }.returns(Sensor.TYPE_GYROSCOPE)
        val event = mockk<SensorEvent>()
        event.sensor = sensor
        val measurement = AccelerometerSensorMeasurement()

        assertFalse(AccelerometerSensorMeasurementConverter.convert(event, measurement))
    }

    @Test
    fun convert_whenAccelerometerSensorTypeWithoutStartOffset_returnsTrue() {
        val sensor = mockk<Sensor>()
        every { sensor.type }.returns(Sensor.TYPE_ACCELEROMETER)
        val event = mockk<SensorEvent>()
        event.sensor = sensor

        val timestamp = System.nanoTime()
        event.timestamp = timestamp

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val values = floatArrayOf(ax, ay, az, 4.0f, 5.0f, 6.0f)
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, values)

        event.accuracy = SensorManager.SENSOR_STATUS_ACCURACY_HIGH

        val measurement = AccelerometerSensorMeasurement()

        assertTrue(AccelerometerSensorMeasurementConverter.convert(event, measurement))

        // check measurement values
        assertEquals(ax, measurement.ax, 0.0f)
        assertEquals(ay, measurement.ay, 0.0f)
        assertEquals(az, measurement.az, 0.0f)
        assertNull(measurement.bx)
        assertNull(measurement.by)
        assertNull(measurement.bz)
        assertEquals(timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
        assertEquals(AccelerometerSensorType.ACCELEROMETER, measurement.sensorType)
    }

    @Test
    fun convert_whenAccelerometerUncalibratedSensorTypeWithoutStartOffset_returnsTrue() {
        val sensor = mockk<Sensor>()
        every { sensor.type }.returns(Constants.TYPE_ACCELEROMETER_UNCALIBRATED)
        val event = mockk<SensorEvent>()
        event.sensor = sensor

        val timestamp = System.nanoTime()
        event.timestamp = timestamp

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val values = floatArrayOf(ax, ay, az, bx, by, bz)
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, values)

        event.accuracy = SensorManager.SENSOR_STATUS_ACCURACY_HIGH

        val measurement = AccelerometerSensorMeasurement()

        assertTrue(AccelerometerSensorMeasurementConverter.convert(event, measurement))

        // check measurement values
        assertEquals(ax, measurement.ax, 0.0f)
        assertEquals(ay, measurement.ay, 0.0f)
        assertEquals(az, measurement.az, 0.0f)
        assertEquals(bx, measurement.bx)
        assertEquals(by, measurement.by)
        assertEquals(bz, measurement.bz)
        assertEquals(timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
        assertEquals(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED, measurement.sensorType)
    }

    @Test
    fun convert_whenAccelerometerSensorTypeWithStartOffset_returnsTrue() {
        val sensor = mockk<Sensor>()
        every { sensor.type }.returns(Sensor.TYPE_ACCELEROMETER)
        val event = mockk<SensorEvent>()
        event.sensor = sensor

        val timestamp = System.nanoTime()
        event.timestamp = timestamp

        val randomizer = UniformRandomizer()
        val startOffset = randomizer.nextLong(-MAX_OFFSET_NANOS, MAX_OFFSET_NANOS)
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val values = floatArrayOf(ax, ay, az, 4.0f, 5.0f, 6.0f)
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, values)

        event.accuracy = SensorManager.SENSOR_STATUS_ACCURACY_HIGH

        val measurement = AccelerometerSensorMeasurement()

        assertTrue(AccelerometerSensorMeasurementConverter.convert(event, measurement, startOffset))

        // check measurement values
        assertEquals(ax, measurement.ax, 0.0f)
        assertEquals(ay, measurement.ay, 0.0f)
        assertEquals(az, measurement.az, 0.0f)
        assertNull(measurement.bx)
        assertNull(measurement.by)
        assertNull(measurement.bz)
        assertEquals(timestamp + startOffset, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
        assertEquals(AccelerometerSensorType.ACCELEROMETER, measurement.sensorType)
    }

    @Test
    fun convert_whenAccelerometerUncalibratedSensorTypeWithStartOffset_returnsTrue() {
        val sensor = mockk<Sensor>()
        every { sensor.type }.returns(Constants.TYPE_ACCELEROMETER_UNCALIBRATED)
        val event = mockk<SensorEvent>()
        event.sensor = sensor

        val timestamp = System.nanoTime()
        event.timestamp = timestamp

        val randomizer = UniformRandomizer()
        val startOffset = randomizer.nextLong(-MAX_OFFSET_NANOS, MAX_OFFSET_NANOS)
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val values = floatArrayOf(ax, ay, az, bx, by, bz)
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, values)

        event.accuracy = SensorManager.SENSOR_STATUS_ACCURACY_HIGH

        val measurement = AccelerometerSensorMeasurement()

        assertTrue(AccelerometerSensorMeasurementConverter.convert(event, measurement, startOffset))

        // check measurement values
        assertEquals(ax, measurement.ax, 0.0f)
        assertEquals(ay, measurement.ay, 0.0f)
        assertEquals(az, measurement.az, 0.0f)
        assertEquals(bx, measurement.bx)
        assertEquals(by, measurement.by)
        assertEquals(bz, measurement.bz)
        assertEquals(timestamp + startOffset, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
        assertEquals(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED, measurement.sensorType)
    }

    private companion object {
        const val MAX_OFFSET_NANOS = 1000000L
    }
}