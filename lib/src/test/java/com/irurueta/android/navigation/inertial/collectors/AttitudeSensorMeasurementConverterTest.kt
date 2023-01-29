/*
 * Copyright (C) 2023 Alberto Irurueta Carro (alberto@irurueta.com)
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
import com.irurueta.geometry.Quaternion
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
class AttitudeSensorMeasurementConverterTest {

    @After
    fun tearDown() {
        clearAllMocks()
    }

    @Test
    fun convert_whenNoSensorEvent_returnsFalse() {
        val measurement = AttitudeSensorMeasurement()
        assertFalse(AttitudeSensorMeasurementConverter.convert(null, measurement))
    }

    @Test
    fun convert_whenUnknownSensorType_returnsFalse() {
        val sensor = mockk<Sensor>()
        every { sensor.type }.returns(Sensor.TYPE_GYROSCOPE)
        val event = mockk<SensorEvent>()
        event.sensor = sensor
        val measurement = AttitudeSensorMeasurement()

        assertFalse(AttitudeSensorMeasurementConverter.convert(event, measurement))
    }

    @Test
    fun convert_whenAbsoluteAttitudeNoHeadingAccuracyAndNoStartOffset_returnsTrue() {
        val sensor = mockk<Sensor>()
        every { sensor.type }.returns(Sensor.TYPE_ROTATION_VECTOR)
        val event = mockk<SensorEvent>()
        event.sensor = sensor

        val timestamp = System.nanoTime()
        event.timestamp = timestamp

        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        attitude.normalize()
        val values = floatArrayOf(
            attitude.b.toFloat(),
            attitude.c.toFloat(),
            attitude.d.toFloat(),
            attitude.a.toFloat()
        )
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, values)

        event.accuracy = SensorManager.SENSOR_STATUS_ACCURACY_HIGH

        val measurement = AttitudeSensorMeasurement()

        assertTrue(AttitudeSensorMeasurementConverter.convert(event, measurement))

        // check measurement values
        assertTrue(attitude.equals(measurement.attitude, ABSOLUTE_ERROR))
        assertNull(measurement.headingAccuracy)
        assertEquals(timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
        assertEquals(AttitudeSensorType.ABSOLUTE_ATTITUDE, measurement.sensorType)
    }

    @Test
    fun convert_whenAbsoluteAttitudeHeadingAccuracyAndNoStartOffset_returnsTrue() {
        val sensor = mockk<Sensor>()
        every { sensor.type }.returns(Sensor.TYPE_ROTATION_VECTOR)
        val event = mockk<SensorEvent>()
        event.sensor = sensor

        val timestamp = System.nanoTime()
        event.timestamp = timestamp

        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        attitude.normalize()
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val values = floatArrayOf(
            attitude.b.toFloat(),
            attitude.c.toFloat(),
            attitude.d.toFloat(),
            attitude.a.toFloat(),
            headingAccuracy
        )
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, values)

        event.accuracy = SensorManager.SENSOR_STATUS_ACCURACY_HIGH

        val measurement = AttitudeSensorMeasurement()

        assertTrue(AttitudeSensorMeasurementConverter.convert(event, measurement))

        // check measurement values
        assertTrue(attitude.equals(measurement.attitude, ABSOLUTE_ERROR))
        assertEquals(headingAccuracy, measurement.headingAccuracy)
        assertEquals(timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
        assertEquals(AttitudeSensorType.ABSOLUTE_ATTITUDE, measurement.sensorType)
    }

    @Test
    fun convert_whenAbsoluteAttitudeNoHeadingAccuracyAndStartOffset_returnsTrue() {
        val sensor = mockk<Sensor>()
        every { sensor.type }.returns(Sensor.TYPE_ROTATION_VECTOR)
        val event = mockk<SensorEvent>()
        event.sensor = sensor

        val timestamp = System.nanoTime()
        event.timestamp = timestamp

        val randomizer = UniformRandomizer()
        val startOffset = randomizer.nextLong(-MAX_OFFSET_NANOS, MAX_OFFSET_NANOS)
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        attitude.normalize()
        val values = floatArrayOf(
            attitude.b.toFloat(),
            attitude.c.toFloat(),
            attitude.d.toFloat(),
            attitude.a.toFloat()
        )
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, values)

        event.accuracy = SensorManager.SENSOR_STATUS_ACCURACY_HIGH

        val measurement = AttitudeSensorMeasurement()

        assertTrue(AttitudeSensorMeasurementConverter.convert(event, measurement, startOffset))

        // check measurement values
        assertTrue(attitude.equals(measurement.attitude, ABSOLUTE_ERROR))
        assertNull(measurement.headingAccuracy)
        assertEquals(timestamp + startOffset, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
        assertEquals(AttitudeSensorType.ABSOLUTE_ATTITUDE, measurement.sensorType)
    }

    @Test
    fun convert_whenAbsoluteAttitudeHeadingAccuracyAndStartOffset_returnsTrue() {
        val sensor = mockk<Sensor>()
        every { sensor.type }.returns(Sensor.TYPE_ROTATION_VECTOR)
        val event = mockk<SensorEvent>()
        event.sensor = sensor

        val timestamp = System.nanoTime()
        event.timestamp = timestamp

        val randomizer = UniformRandomizer()
        val startOffset = randomizer.nextLong(-MAX_OFFSET_NANOS, MAX_OFFSET_NANOS)
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        attitude.normalize()
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val values = floatArrayOf(
            attitude.b.toFloat(),
            attitude.c.toFloat(),
            attitude.d.toFloat(),
            attitude.a.toFloat(),
            headingAccuracy
        )
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, values)

        event.accuracy = SensorManager.SENSOR_STATUS_ACCURACY_HIGH

        val measurement = AttitudeSensorMeasurement()

        assertTrue(AttitudeSensorMeasurementConverter.convert(event, measurement, startOffset))

        // check measurement values
        assertTrue(attitude.equals(measurement.attitude, ABSOLUTE_ERROR))
        assertEquals(headingAccuracy, measurement.headingAccuracy)
        assertEquals(timestamp + startOffset, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
        assertEquals(AttitudeSensorType.ABSOLUTE_ATTITUDE, measurement.sensorType)
    }

    @Test
    fun convert_whenRelativeAttitudeNoHeadingAccuracyAndNoStartOffset_returnsTrue() {
        val sensor = mockk<Sensor>()
        every { sensor.type }.returns(Sensor.TYPE_GAME_ROTATION_VECTOR)
        val event = mockk<SensorEvent>()
        event.sensor = sensor

        val timestamp = System.nanoTime()
        event.timestamp = timestamp

        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        attitude.normalize()
        val values = floatArrayOf(
            attitude.b.toFloat(),
            attitude.c.toFloat(),
            attitude.d.toFloat(),
            attitude.a.toFloat()
        )
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, values)

        event.accuracy = SensorManager.SENSOR_STATUS_ACCURACY_HIGH

        val measurement = AttitudeSensorMeasurement()

        assertTrue(AttitudeSensorMeasurementConverter.convert(event, measurement))

        // check measurement values
        assertTrue(attitude.equals(measurement.attitude, ABSOLUTE_ERROR))
        assertNull(measurement.headingAccuracy)
        assertEquals(timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
        assertEquals(AttitudeSensorType.RELATIVE_ATTITUDE, measurement.sensorType)
    }

    @Test
    fun convert_whenRelativeAttitudeHeadingAccuracyAndNoStartOffset_returnsTrue() {
        val sensor = mockk<Sensor>()
        every { sensor.type }.returns(Sensor.TYPE_GAME_ROTATION_VECTOR)
        val event = mockk<SensorEvent>()
        event.sensor = sensor

        val timestamp = System.nanoTime()
        event.timestamp = timestamp

        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        attitude.normalize()
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val values = floatArrayOf(
            attitude.b.toFloat(),
            attitude.c.toFloat(),
            attitude.d.toFloat(),
            attitude.a.toFloat(),
            headingAccuracy
        )
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, values)

        event.accuracy = SensorManager.SENSOR_STATUS_ACCURACY_HIGH

        val measurement = AttitudeSensorMeasurement()

        assertTrue(AttitudeSensorMeasurementConverter.convert(event, measurement))

        // check measurement values
        assertTrue(attitude.equals(measurement.attitude, ABSOLUTE_ERROR))
        assertEquals(headingAccuracy, measurement.headingAccuracy)
        assertEquals(timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
        assertEquals(AttitudeSensorType.RELATIVE_ATTITUDE, measurement.sensorType)
    }

    @Test
    fun convert_whenRelativeAttitudeNoHeadingAccuracyAndStartOffset_returnsTrue() {
        val sensor = mockk<Sensor>()
        every { sensor.type }.returns(Sensor.TYPE_GAME_ROTATION_VECTOR)
        val event = mockk<SensorEvent>()
        event.sensor = sensor

        val timestamp = System.nanoTime()
        event.timestamp = timestamp

        val randomizer = UniformRandomizer()
        val startOffset = randomizer.nextLong(-MAX_OFFSET_NANOS, MAX_OFFSET_NANOS)
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        attitude.normalize()
        val values = floatArrayOf(
            attitude.b.toFloat(),
            attitude.c.toFloat(),
            attitude.d.toFloat(),
            attitude.a.toFloat()
        )
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, values)

        event.accuracy = SensorManager.SENSOR_STATUS_ACCURACY_HIGH

        val measurement = AttitudeSensorMeasurement()

        assertTrue(AttitudeSensorMeasurementConverter.convert(event, measurement, startOffset))

        // check measurement values
        assertTrue(attitude.equals(measurement.attitude, ABSOLUTE_ERROR))
        assertNull(measurement.headingAccuracy)
        assertEquals(timestamp + startOffset, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
        assertEquals(AttitudeSensorType.RELATIVE_ATTITUDE, measurement.sensorType)
    }

    @Test
    fun convert_whenRelativeAttitudeHeadingAccuracyAndStartOffset_returnsTrue() {
        val sensor = mockk<Sensor>()
        every { sensor.type }.returns(Sensor.TYPE_GAME_ROTATION_VECTOR)
        val event = mockk<SensorEvent>()
        event.sensor = sensor

        val timestamp = System.nanoTime()
        event.timestamp = timestamp

        val randomizer = UniformRandomizer()
        val startOffset = randomizer.nextLong(-MAX_OFFSET_NANOS, MAX_OFFSET_NANOS)
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        attitude.normalize()
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val values = floatArrayOf(
            attitude.b.toFloat(),
            attitude.c.toFloat(),
            attitude.d.toFloat(),
            attitude.a.toFloat(),
            headingAccuracy
        )
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, values)

        event.accuracy = SensorManager.SENSOR_STATUS_ACCURACY_HIGH

        val measurement = AttitudeSensorMeasurement()

        assertTrue(AttitudeSensorMeasurementConverter.convert(event, measurement, startOffset))

        // check measurement values
        assertTrue(attitude.equals(measurement.attitude, ABSOLUTE_ERROR))
        assertEquals(headingAccuracy, measurement.headingAccuracy)
        assertEquals(timestamp + startOffset, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
        assertEquals(AttitudeSensorType.RELATIVE_ATTITUDE, measurement.sensorType)
    }

    private companion object {
        const val MIN_DEGREES = -90.0

        const val MAX_DEGREES = 90.0

        const val ABSOLUTE_ERROR = 1e-6

        const val MAX_OFFSET_NANOS = 1_000_000L
    }
}