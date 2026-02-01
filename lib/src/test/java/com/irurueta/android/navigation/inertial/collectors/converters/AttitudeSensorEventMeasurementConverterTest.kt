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
import com.irurueta.android.navigation.inertial.collectors.measurements.AttitudeSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AttitudeSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorCoordinateSystem
import com.irurueta.geometry.Quaternion
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

class AttitudeSensorEventMeasurementConverterTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK
    private lateinit var sensor: Sensor

    @MockK
    private lateinit var event: SensorEvent

    @Test
    fun convert_whenNoSensorEvent_returnsFalse() {
        val measurement = AttitudeSensorMeasurement()
        assertFalse(
            AttitudeSensorEventMeasurementConverter.convert(
                null,
                measurement
            )
        )
    }

    @Test
    fun convert_whenUnknownSensorType_returnsFalse() {
        every { sensor.type }.returns(Sensor.TYPE_GYROSCOPE)
        event.sensor = sensor
        val measurement = AttitudeSensorMeasurement()

        assertFalse(AttitudeSensorEventMeasurementConverter.convert(event, measurement))
    }

    @Test
    fun convert_whenAbsoluteAttitudeNoHeadingAccuracy_returnsTrue() {
        every { sensor.type }.returns(Sensor.TYPE_ROTATION_VECTOR)
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

        assertTrue(AttitudeSensorEventMeasurementConverter.convert(event, measurement))

        // check measurement values
        assertTrue(attitude.equals(measurement.attitude, ABSOLUTE_ERROR))
        assertNull(measurement.headingAccuracy)
        assertEquals(timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
        assertEquals(
            AttitudeSensorType.ABSOLUTE_ATTITUDE,
            measurement.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.ENU,
            measurement.sensorCoordinateSystem
        )
    }

    @Test
    fun convert_whenAbsoluteAttitudeHeadingAccuracy_returnsTrue() {
        every { sensor.type }.returns(Sensor.TYPE_ROTATION_VECTOR)
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

        assertTrue(AttitudeSensorEventMeasurementConverter.convert(event, measurement))

        // check measurement values
        assertTrue(attitude.equals(measurement.attitude, ABSOLUTE_ERROR))
        assertEquals(headingAccuracy, measurement.headingAccuracy)
        assertEquals(timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
        assertEquals(
            AttitudeSensorType.ABSOLUTE_ATTITUDE,
            measurement.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.ENU,
            measurement.sensorCoordinateSystem
        )

    }

    @Test
    fun convert_whenRelativeAttitudeNoHeadingAccuracyAndNoStartOffset_returnsTrue() {
        every { sensor.type }.returns(Sensor.TYPE_GAME_ROTATION_VECTOR)
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

        assertTrue(AttitudeSensorEventMeasurementConverter.convert(event, measurement))

        // check measurement values
        assertTrue(attitude.equals(measurement.attitude, ABSOLUTE_ERROR))
        assertNull(measurement.headingAccuracy)
        assertEquals(timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
        assertEquals(
            AttitudeSensorType.RELATIVE_ATTITUDE,
            measurement.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.ENU,
            measurement.sensorCoordinateSystem
        )
    }

    @Test
    fun convert_whenRelativeAttitudeHeadingAccuracyAndNoStartOffset_returnsTrue() {
        every { sensor.type }.returns(Sensor.TYPE_GAME_ROTATION_VECTOR)
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

        assertTrue(AttitudeSensorEventMeasurementConverter.convert(event, measurement))

        // check measurement values
        assertTrue(attitude.equals(measurement.attitude, ABSOLUTE_ERROR))
        assertEquals(headingAccuracy, measurement.headingAccuracy)
        assertEquals(timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
        assertEquals(
            AttitudeSensorType.RELATIVE_ATTITUDE,
            measurement.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.ENU,
            measurement.sensorCoordinateSystem
        )
    }

    @Test
    fun convert_whenNoSensorEvent_returnsNull() {
        assertNull(AttitudeSensorEventMeasurementConverter.convert(null))
    }

    @Test
    fun convert_whenUnknownSensorType_returnsNull() {
        every { sensor.type }.returns(Sensor.TYPE_GYROSCOPE)
        event.sensor = sensor
        assertNull(AttitudeSensorEventMeasurementConverter.convert(event))
    }

    @Test
    fun convert_whenAbsoluteAttitudeNoHeadingAccuracy_returnsExpectedResult() {
        every { sensor.type }.returns(Sensor.TYPE_ROTATION_VECTOR)
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

        val measurement = AttitudeSensorEventMeasurementConverter.convert(event)

        // check measurement values
        requireNotNull(measurement)
        assertTrue(attitude.equals(measurement.attitude, ABSOLUTE_ERROR))
        assertNull(measurement.headingAccuracy)
        assertEquals(timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
        assertEquals(
            AttitudeSensorType.ABSOLUTE_ATTITUDE,
            measurement.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.ENU,
            measurement.sensorCoordinateSystem
        )
    }

    @Test
    fun convert_whenAbsoluteAttitudeHeadingAccuracy_returnsExpectedResult() {
        every { sensor.type }.returns(Sensor.TYPE_ROTATION_VECTOR)
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

        val measurement = AttitudeSensorEventMeasurementConverter.convert(event)

        // check measurement values
        requireNotNull(measurement)
        assertTrue(attitude.equals(measurement.attitude, ABSOLUTE_ERROR))
        assertEquals(headingAccuracy, measurement.headingAccuracy)
        assertEquals(timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
        assertEquals(
            AttitudeSensorType.ABSOLUTE_ATTITUDE,
            measurement.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.ENU,
            measurement.sensorCoordinateSystem
        )

    }

    @Test
    fun convert_whenRelativeAttitudeNoHeadingAccuracyAndNoStartOffset_returnsExpectedResult() {
        every { sensor.type }.returns(Sensor.TYPE_GAME_ROTATION_VECTOR)
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

        val measurement = AttitudeSensorEventMeasurementConverter.convert(event)

        // check measurement values
        requireNotNull(measurement)
        assertTrue(attitude.equals(measurement.attitude, ABSOLUTE_ERROR))
        assertNull(measurement.headingAccuracy)
        assertEquals(timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
        assertEquals(
            AttitudeSensorType.RELATIVE_ATTITUDE,
            measurement.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.ENU,
            measurement.sensorCoordinateSystem
        )
    }

    @Test
    fun convert_whenRelativeAttitudeHeadingAccuracyAndNoStartOffset_returnsExpectedResult() {
        every { sensor.type }.returns(Sensor.TYPE_GAME_ROTATION_VECTOR)
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

        val measurement = AttitudeSensorEventMeasurementConverter.convert(event)

        // check measurement values
        requireNotNull(measurement)
        assertTrue(attitude.equals(measurement.attitude, ABSOLUTE_ERROR))
        assertEquals(headingAccuracy, measurement.headingAccuracy)
        assertEquals(timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
        assertEquals(
            AttitudeSensorType.RELATIVE_ATTITUDE,
            measurement.sensorType
        )
        assertEquals(
            SensorCoordinateSystem.ENU,
            measurement.sensorCoordinateSystem
        )
    }

    private companion object {
        const val MIN_DEGREES = -90.0

        const val MAX_DEGREES = 90.0

        const val ABSOLUTE_ERROR = 1e-6
    }
}