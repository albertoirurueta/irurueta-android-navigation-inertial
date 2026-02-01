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
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorType
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

class GyroscopeSensorEventMeasurementConverterTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK
    private lateinit var sensor: Sensor

    @MockK
    private lateinit var event: SensorEvent

    @Test
    fun convert_whenNoSensorEvent_returnsFalse() {
        val measurement = GyroscopeSensorMeasurement()
        assertFalse(
            GyroscopeSensorEventMeasurementConverter.convert(
                null,
                measurement
            )
        )
    }

    @Test
    fun convert_whenUnknownSensorType_returnsFalse() {
        every { sensor.type }.returns(Sensor.TYPE_ACCELEROMETER)
        event.sensor = sensor
        val measurement = GyroscopeSensorMeasurement()

        assertFalse(GyroscopeSensorEventMeasurementConverter.convert(event, measurement))
    }

    @Test
    fun convert_whenGyroscopeSensorTypeWithoutStartOffset_returnsTrue() {
        every { sensor.type }.returns(Sensor.TYPE_GYROSCOPE)
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

        assertTrue(GyroscopeSensorEventMeasurementConverter.convert(event, measurement))

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
        assertEquals(
            SensorCoordinateSystem.ENU,
            measurement.sensorCoordinateSystem
        )
    }

    @Test
    fun convert_whenGyroscopeUncalibratedSensorTypeWithoutStartOffset_returnsTrue() {
        every { sensor.type }.returns(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
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

        assertTrue(GyroscopeSensorEventMeasurementConverter.convert(event, measurement))

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
        assertEquals(
            SensorCoordinateSystem.ENU,
            measurement.sensorCoordinateSystem
        )
    }

    @Test
    fun convert_whenNoSensorEvent_returnsNull() {
        assertNull(GyroscopeSensorEventMeasurementConverter.convert(null))
    }

    @Test
    fun convert_whenUnknownSensorType_returnsNull() {
        every { sensor.type }.returns(Sensor.TYPE_ACCELEROMETER)
        event.sensor = sensor
        assertNull(GyroscopeSensorEventMeasurementConverter.convert(event))
    }

    @Test
    fun convert_whenGyroscopeSensorTypeWithoutStartOffset_returnsExpectedResult() {
        every { sensor.type }.returns(Sensor.TYPE_GYROSCOPE)
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

        val measurement = GyroscopeSensorEventMeasurementConverter.convert(event)

        // check measurement values
        requireNotNull(measurement)
        assertEquals(wx, measurement.wx, 0.0f)
        assertEquals(wy, measurement.wy, 0.0f)
        assertEquals(wz, measurement.wz, 0.0f)
        assertNull(measurement.bx)
        assertNull(measurement.by)
        assertNull(measurement.bz)
        assertEquals(timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
        assertEquals(GyroscopeSensorType.GYROSCOPE, measurement.sensorType)
        assertEquals(
            SensorCoordinateSystem.ENU,
            measurement.sensorCoordinateSystem
        )
    }

    @Test
    fun convert_whenGyroscopeUncalibratedSensorTypeWithoutStartOffset_returnsExpectedResult() {
        every { sensor.type }.returns(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
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

        val measurement = GyroscopeSensorEventMeasurementConverter.convert(event)

        // check measurement values
        requireNotNull(measurement)
        assertEquals(wx, measurement.wx, 0.0f)
        assertEquals(wy, measurement.wy, 0.0f)
        assertEquals(wz, measurement.wz, 0.0f)
        assertEquals(bx, measurement.bx)
        assertEquals(by, measurement.by)
        assertEquals(bz, measurement.bz)
        assertEquals(timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
        assertEquals(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED, measurement.sensorType)
        assertEquals(
            SensorCoordinateSystem.ENU,
            measurement.sensorCoordinateSystem
        )
    }

}