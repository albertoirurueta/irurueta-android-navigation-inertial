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
class GravitySensorMeasurementConverterTest {

    @Test
    fun convert_whenNoSensorEvent_returnsFalse() {
        val measurement = GravitySensorMeasurement()
        assertFalse(GravitySensorMeasurementConverter.convert(null, measurement))
    }

    @Test
    fun convert_whenUnknownSensorType_returnsFalse() {
        val sensor = mockk<Sensor>()
        every { sensor.type }.returns(Sensor.TYPE_ACCELEROMETER)
        val event = mockk<SensorEvent>()
        event.sensor = sensor
        val measurement = GravitySensorMeasurement()

        assertFalse(GravitySensorMeasurementConverter.convert(event, measurement))
    }

    @Test
    fun convert_withoutStartOffset_returnsTrue() {
        val sensor = mockk<Sensor>()
        every { sensor.type }.returns(Sensor.TYPE_GRAVITY)
        val event = mockk<SensorEvent>()
        event.sensor = sensor

        val timestamp = System.nanoTime()
        event.timestamp = timestamp

        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val values = floatArrayOf(gx, gy, gz)
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, values)

        event.accuracy = SensorManager.SENSOR_STATUS_ACCURACY_MEDIUM

        val measurement = GravitySensorMeasurement()

        assertTrue(GravitySensorMeasurementConverter.convert(event, measurement))

        // check measurement values
        assertEquals(gx, measurement.gx, 0.0f)
        assertEquals(gy, measurement.gy, 0.0f)
        assertEquals(gz, measurement.gz, 0.0f)
        assertEquals(timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, measurement.accuracy)
    }

    @Test
    fun convert_withStartOffset_returnsTrue() {
        val sensor = mockk<Sensor>()
        every { sensor.type }.returns(Sensor.TYPE_GRAVITY)
        val event = mockk<SensorEvent>()
        event.sensor = sensor

        val timestamp = System.nanoTime()
        event.timestamp = timestamp

        val randomizer = UniformRandomizer()
        val startOffset = randomizer.nextLong(-MAX_OFFSET_NANOS, MAX_OFFSET_NANOS)
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val values = floatArrayOf(gx, gy, gz)
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, values)

        event.accuracy = SensorManager.SENSOR_STATUS_ACCURACY_MEDIUM

        val measurement = GravitySensorMeasurement()

        assertTrue(GravitySensorMeasurementConverter.convert(event, measurement, startOffset))

        // check measurement values
        assertEquals(gx, measurement.gx, 0.0f)
        assertEquals(gy, measurement.gy, 0.0f)
        assertEquals(gz, measurement.gz, 0.0f)
        assertEquals(timestamp + startOffset, measurement.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, measurement.accuracy)
    }

    private companion object {
        const val MAX_OFFSET_NANOS = 1000000L
    }
}