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

import android.os.SystemClock
import com.irurueta.geometry.Quaternion
import com.irurueta.statistics.UniformRandomizer
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class AttitudeSensorMeasurementTest {

    @Test
    fun constructor_whenDefault_setsExpectedValues() {
        val measurement = AttitudeSensorMeasurement()

        // check
        assertEquals(Quaternion(), measurement.attitude)
        assertNull(measurement.headingAccuracy)
        assertEquals(0L, measurement.timestamp)
        assertNull(measurement.accuracy)
    }

    @Test
    fun constructor_withRequiredProperties_setsExpectedValues() {
        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        val timestamp = SystemClock.elapsedRealtimeNanos()

        val measurement =
            AttitudeSensorMeasurement(attitude, null, timestamp, null)

        // check
        assertSame(attitude, measurement.attitude)
        assertNull(measurement.headingAccuracy)
        assertEquals(timestamp, measurement.timestamp)
        assertNull(measurement.accuracy)
    }

    @Test
    fun constructor_withAllProperties_setsExpectedValues() {
        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()

        val measurement =
            AttitudeSensorMeasurement(attitude, headingAccuracy, timestamp, SensorAccuracy.HIGH)

        // check
        assertSame(attitude, measurement.attitude)
        assertEquals(headingAccuracy, measurement.headingAccuracy)
        assertEquals(timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
    }

    @Test
    fun constructor_whenCopy_setsExpectedValues() {
        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()

        val measurement1 =
            AttitudeSensorMeasurement(attitude, headingAccuracy, timestamp, SensorAccuracy.HIGH)

        val measurement2 = AttitudeSensorMeasurement(measurement1)

        // check
        assertSame(attitude, measurement1.attitude)
        assertEquals(headingAccuracy, measurement1.headingAccuracy)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement1.accuracy)

        assertEquals(measurement1.attitude, measurement2.attitude)
        assertEquals(measurement1.headingAccuracy, measurement2.headingAccuracy)
        assertEquals(measurement1.timestamp, measurement2.timestamp)
        assertEquals(measurement1.accuracy, measurement2.accuracy)
    }

    @Test
    fun attitude_setsExpectedValue() {
        val measurement = AttitudeSensorMeasurement()

        // check default value
        assertEquals(Quaternion(), measurement.attitude)

        // set new value
        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        measurement.attitude = attitude

        // check
        assertEquals(attitude, measurement.attitude)
    }

    @Test
    fun headingAccuracy_setsExpectedValue() {
        val measurement = AttitudeSensorMeasurement()

        // check default value
        assertNull(measurement.headingAccuracy)

        // set new value
        val randomizer = UniformRandomizer()
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        measurement.headingAccuracy = headingAccuracy

        // check
        assertEquals(headingAccuracy, measurement.headingAccuracy)
    }

    @Test
    fun timestamp_setsExpectedValue() {
        val measurement = AttitudeSensorMeasurement()

        // check default value
        assertEquals(0L, measurement.timestamp)

        // set new value
        val timestamp = SystemClock.elapsedRealtimeNanos()
        measurement.timestamp = timestamp

        // check
        assertEquals(timestamp, measurement.timestamp)
    }

    @Test
    fun accuracy_setsExpectedValue() {
        val measurement = AttitudeSensorMeasurement()

        // check default value
        assertNull(measurement.accuracy)

        // set new value
        measurement.accuracy = SensorAccuracy.HIGH

        // check
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)

        // set to null
        measurement.accuracy = null

        // check
        @Suppress("KotlinConstantConditions")
        assertNull(measurement.accuracy)
    }

    @Test
    fun copyFrom_makesExpectedCopy() {
        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()

        val measurement1 =
            AttitudeSensorMeasurement(attitude, headingAccuracy, timestamp, SensorAccuracy.HIGH)

        val measurement2 = AttitudeSensorMeasurement()
        measurement2.copyFrom(measurement1)

        // check
        assertSame(attitude, measurement1.attitude)
        assertEquals(headingAccuracy, measurement1.headingAccuracy)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement1.accuracy)

        assertEquals(measurement1.attitude, measurement2.attitude)
        assertEquals(measurement1.headingAccuracy, measurement2.headingAccuracy)
        assertEquals(measurement1.timestamp, measurement2.timestamp)
        assertEquals(measurement1.accuracy, measurement2.accuracy)
    }

    @Test
    fun copyTo_makesExpectedCopy() {
        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()

        val measurement1 =
            AttitudeSensorMeasurement(attitude, headingAccuracy, timestamp, SensorAccuracy.HIGH)

        val measurement2 = AttitudeSensorMeasurement()
        measurement1.copyTo(measurement2)

        // check
        assertSame(attitude, measurement1.attitude)
        assertEquals(headingAccuracy, measurement1.headingAccuracy)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement1.accuracy)

        assertEquals(measurement1.attitude, measurement2.attitude)
        assertEquals(measurement1.headingAccuracy, measurement2.headingAccuracy)
        assertEquals(measurement1.timestamp, measurement2.timestamp)
        assertEquals(measurement1.accuracy, measurement2.accuracy)
    }

    @Test
    fun copy_makesExpectedCopy() {
        val randomizer = UniformRandomizer()
        val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
        val attitude = Quaternion(roll, pitch, yaw)
        val headingAccuracy = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()

        val measurement1 =
            AttitudeSensorMeasurement(attitude, headingAccuracy, timestamp, SensorAccuracy.HIGH)

        val measurement2 = measurement1.copy()

        // check
        assertSame(attitude, measurement1.attitude)
        assertEquals(headingAccuracy, measurement1.headingAccuracy)
        assertEquals(timestamp, measurement1.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement1.accuracy)

        assertEquals(measurement1.attitude, measurement2.attitude)
        assertEquals(measurement1.headingAccuracy, measurement2.headingAccuracy)
        assertEquals(measurement1.timestamp, measurement2.timestamp)
        assertEquals(measurement1.accuracy, measurement2.accuracy)
    }

    private companion object {
        const val MIN_DEGREES = -90.0

        const val MAX_DEGREES = 90.0
    }
}