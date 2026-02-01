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
import com.irurueta.android.navigation.inertial.collectors.measurements.GravitySensorMeasurement
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

class GravitySensorEventMeasurementConverterTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK
    private lateinit var sensor: Sensor

    @MockK
    private lateinit var event: SensorEvent

    @Test
    fun convert_whenNoSensorEvent_returnsFalse() {
        val measurement = GravitySensorMeasurement()
        assertFalse(
            GravitySensorEventMeasurementConverter.convert(
                null,
                measurement
            )
        )
    }

    @Test
    fun convert_whenUnknownSensorType_returnsFalse() {
        every { sensor.type }.returns(Sensor.TYPE_ACCELEROMETER)
        event.sensor = sensor
        val measurement = GravitySensorMeasurement()

        assertFalse(GravitySensorEventMeasurementConverter.convert(event, measurement))
    }

    @Test
    fun convert_whenValidSensorType_returnsTrue() {
        every { sensor.type }.returns(Sensor.TYPE_GRAVITY)
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

        assertTrue(GravitySensorEventMeasurementConverter.convert(event, measurement))

        // check measurement values
        assertEquals(gx, measurement.gx, 0.0f)
        assertEquals(gy, measurement.gy, 0.0f)
        assertEquals(gz, measurement.gz, 0.0f)
        assertEquals(timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, measurement.accuracy)
        assertEquals(
            SensorCoordinateSystem.ENU,
            measurement.sensorCoordinateSystem
        )
    }

    @Test
    fun convert_whenNoSensorEvent_returnsExpectedResult() {
        assertNull(GravitySensorEventMeasurementConverter.convert(null))
    }

    @Test
    fun convert_whenUnknownSensorType_returnsExpectedResult() {
        every { sensor.type }.returns(Sensor.TYPE_ACCELEROMETER)
        event.sensor = sensor
        assertNull(GravitySensorEventMeasurementConverter.convert(event))
    }

    @Test
    fun convert_whenValidSensorType_returnsExpectedResult() {
        every { sensor.type }.returns(Sensor.TYPE_GRAVITY)
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

        val measurement = GravitySensorEventMeasurementConverter.convert(event)

        // check measurement values
        requireNotNull(measurement)
        assertEquals(gx, measurement.gx, 0.0f)
        assertEquals(gy, measurement.gy, 0.0f)
        assertEquals(gz, measurement.gz, 0.0f)
        assertEquals(timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, measurement.accuracy)
        assertEquals(
            SensorCoordinateSystem.ENU,
            measurement.sensorCoordinateSystem
        )
    }
}