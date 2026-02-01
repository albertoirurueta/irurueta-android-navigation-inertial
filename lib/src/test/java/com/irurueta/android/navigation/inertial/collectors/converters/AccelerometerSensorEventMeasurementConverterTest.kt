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
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
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

class AccelerometerSensorEventMeasurementConverterTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK
    private lateinit var sensor: Sensor

    @MockK
    private lateinit var event: SensorEvent

    @Test
    fun convert_whenNoSensorEvent_returnsFalse() {
        val measurement = AccelerometerSensorMeasurement()
        assertFalse(
            AccelerometerSensorEventMeasurementConverter.convert(
                null, measurement
            )
        )
    }

    @Test
    fun convert_whenUnknownSensorType_returnsFalse() {
        every { sensor.type }.returns(Sensor.TYPE_GYROSCOPE)
        event.sensor = sensor
        val measurement = AccelerometerSensorMeasurement()

        assertFalse(AccelerometerSensorEventMeasurementConverter.convert(event, measurement))
    }

    @Test
    fun convert_whenAccelerometerSensorType_returnsTrue() {
        every { sensor.type }.returns(Sensor.TYPE_ACCELEROMETER)
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

        assertTrue(AccelerometerSensorEventMeasurementConverter.convert(event, measurement))

        // check measurement values
        assertEquals(ax, measurement.ax, 0.0f)
        assertEquals(ay, measurement.ay, 0.0f)
        assertEquals(az, measurement.az, 0.0f)
        assertNull(measurement.bx)
        assertNull(measurement.by)
        assertNull(measurement.bz)
        assertEquals(timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            measurement.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.ENU,
            measurement.sensorCoordinateSystem
        )
    }

    @Test
    fun convert_whenAccelerometerUncalibratedSensorType_returnsTrue() {
        every { sensor.type }.returns(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED)
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

        assertTrue(AccelerometerSensorEventMeasurementConverter.convert(event, measurement))

        // check measurement values
        assertEquals(ax, measurement.ax, 0.0f)
        assertEquals(ay, measurement.ay, 0.0f)
        assertEquals(az, measurement.az, 0.0f)
        assertEquals(bx, measurement.bx)
        assertEquals(by, measurement.by)
        assertEquals(bz, measurement.bz)
        assertEquals(timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            measurement.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.ENU,
            measurement.sensorCoordinateSystem
        )
    }

    @Test
    fun convert_whenNoSensorEvent_returnsNull() {
        assertNull(AccelerometerSensorEventMeasurementConverter.convert(null))
    }

    @Test
    fun convert_whenUnknownSensorType_returnsNull() {
        every { sensor.type }.returns(Sensor.TYPE_GYROSCOPE)
        event.sensor = sensor
        assertNull(AccelerometerSensorEventMeasurementConverter.convert(event))
    }

    @Test
    fun convert_whenAccelerometerSensorType_returnsExpectedResult() {
        every { sensor.type }.returns(Sensor.TYPE_ACCELEROMETER)
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

        val measurement = AccelerometerSensorEventMeasurementConverter.convert(event)

        // check measurement values
        requireNotNull(measurement)
        assertEquals(ax, measurement.ax, 0.0f)
        assertEquals(ay, measurement.ay, 0.0f)
        assertEquals(az, measurement.az, 0.0f)
        assertNull(measurement.bx)
        assertNull(measurement.by)
        assertNull(measurement.bz)
        assertEquals(timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            measurement.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.ENU,
            measurement.sensorCoordinateSystem
        )
    }

    @Test
    fun convert_whenAccelerometerUncalibratedSensorType_returnsExpectedResult() {
        every { sensor.type }.returns(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED)
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

        val measurement = AccelerometerSensorEventMeasurementConverter.convert(event)

        // check measurement values
        requireNotNull(measurement)
        assertEquals(ax, measurement.ax, 0.0f)
        assertEquals(ay, measurement.ay, 0.0f)
        assertEquals(az, measurement.az, 0.0f)
        assertEquals(bx, measurement.bx)
        assertEquals(by, measurement.by)
        assertEquals(bz, measurement.bz)
        assertEquals(timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            measurement.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.ENU,
            measurement.sensorCoordinateSystem
        )
    }
}