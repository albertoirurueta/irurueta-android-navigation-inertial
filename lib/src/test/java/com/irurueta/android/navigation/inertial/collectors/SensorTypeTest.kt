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

import android.os.Build
import io.mockk.clearAllMocks
import org.junit.After
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import org.robolectric.annotation.Config

@RunWith(RobolectricTestRunner::class)
class SensorTypeTest {

    @After
    fun tearDown() {
        clearAllMocks()
    }

    @Config(sdk = [Build.VERSION_CODES.O])
    @Test
    fun sensorType_fromIntWhenSdkO_returnsExpectedValues() {
        assertEquals(10, SensorType.values().size)
        assertEquals(
            SensorType.ACCELEROMETER,
            SensorType.from(SensorType.ACCELEROMETER.value)
        )
        assertEquals(
            SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorType.from(SensorType.ACCELEROMETER_UNCALIBRATED.value)
        )
        assertEquals(
            SensorType.LINEAR_ACCELERATION,
            SensorType.from(SensorType.LINEAR_ACCELERATION.value)
        )
        assertEquals(
            SensorType.GYROSCOPE,
            SensorType.from(SensorType.GYROSCOPE.value)
        )
        assertEquals(
            SensorType.GYROSCOPE_UNCALIBRATED,
            SensorType.from(SensorType.GYROSCOPE_UNCALIBRATED.value)
        )
        assertEquals(
            SensorType.MAGNETOMETER,
            SensorType.from(SensorType.MAGNETOMETER.value)
        )
        assertEquals(
            SensorType.MAGNETOMETER_UNCALIBRATED,
            SensorType.from(SensorType.MAGNETOMETER_UNCALIBRATED.value)
        )
        assertEquals(
            SensorType.GRAVITY,
            SensorType.from(SensorType.GRAVITY.value)
        )
        assertEquals(
            SensorType.ABSOLUTE_ATTITUDE,
            SensorType.from(SensorType.ABSOLUTE_ATTITUDE.value)
        )
        assertEquals(
            SensorType.RELATIVE_ATTITUDE,
            SensorType.from(SensorType.RELATIVE_ATTITUDE.value)
        )
    }

    @Config(sdk = [Build.VERSION_CODES.N])
    @Test
    fun sensorType_fromIntWhenSdkN_returnsExpectedValues() {
        assertEquals(10, SensorType.values().size)
        assertEquals(
            SensorType.ACCELEROMETER,
            SensorType.from(SensorType.ACCELEROMETER.value)
        )
        assertNull(SensorType.from(SensorType.ACCELEROMETER_UNCALIBRATED.value))
        assertEquals(
            SensorType.LINEAR_ACCELERATION,
            SensorType.from(SensorType.LINEAR_ACCELERATION.value)
        )
        assertEquals(
            SensorType.GYROSCOPE,
            SensorType.from(SensorType.GYROSCOPE.value)
        )
        assertEquals(
            SensorType.GYROSCOPE_UNCALIBRATED,
            SensorType.from(SensorType.GYROSCOPE_UNCALIBRATED.value)
        )
        assertEquals(
            SensorType.MAGNETOMETER,
            SensorType.from(SensorType.MAGNETOMETER.value)
        )
        assertEquals(
            SensorType.MAGNETOMETER_UNCALIBRATED,
            SensorType.from(SensorType.MAGNETOMETER_UNCALIBRATED.value)
        )
        assertEquals(
            SensorType.GRAVITY,
            SensorType.from(SensorType.GRAVITY.value)
        )
        assertEquals(
            SensorType.ABSOLUTE_ATTITUDE,
            SensorType.from(SensorType.ABSOLUTE_ATTITUDE.value)
        )
        assertEquals(
            SensorType.RELATIVE_ATTITUDE,
            SensorType.from(SensorType.RELATIVE_ATTITUDE.value)
        )
    }

    @Test
    fun sensorType_fromAccelerometerSensorType_returnsExpectedValue() {
        assertEquals(
            SensorType.ACCELEROMETER,
            SensorType.from(AccelerometerSensorType.ACCELEROMETER)
        )
        assertEquals(
            SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorType.from(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED)
        )
    }

    @Test
    fun sensorType_fromGyroscopeSensorType_returnsExpectedValue() {
        assertEquals(
            SensorType.GYROSCOPE,
            SensorType.from(GyroscopeSensorType.GYROSCOPE)
        )
        assertEquals(
            SensorType.GYROSCOPE_UNCALIBRATED,
            SensorType.from(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED)
        )
    }

    @Test
    fun sensorType_fromMagnetometerSensorType_returnsExpectedValue() {
        assertEquals(
            SensorType.MAGNETOMETER,
            SensorType.from(MagnetometerSensorType.MAGNETOMETER)
        )
        assertEquals(
            SensorType.MAGNETOMETER_UNCALIBRATED,
            SensorType.from(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED)
        )
    }

    @Test
    fun sensorType_fromAttitudeSensorType_returnsExpectedValue() {
        assertEquals(
            SensorType.ABSOLUTE_ATTITUDE,
            SensorType.from(AttitudeSensorType.ABSOLUTE_ATTITUDE)
        )
        assertEquals(
            SensorType.RELATIVE_ATTITUDE,
            SensorType.from(AttitudeSensorType.RELATIVE_ATTITUDE)
        )
    }
}