/*
 * Copyright (C) 2021 Alberto Irurueta Carro (alberto@irurueta.com)
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
package com.irurueta.android.navigation.inertial

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorManager
import android.os.Build
import androidx.test.core.app.ApplicationProvider
import io.mockk.every
import io.mockk.spyk
import io.mockk.verify
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import org.robolectric.annotation.Config

@RunWith(RobolectricTestRunner::class)
class SensorAvailabilityServiceTest {

    @Config(sdk = [Build.VERSION_CODES.O])
    @Test
    fun constructor_whenSdkO_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val service = SensorAvailabilityService(context)

        assertSame(context, service.context)
        assertFalse(service.accelerometerAvailable)
        assertFalse(service.linearAccelerometerAvailable)
        assertFalse(service.uncalibratedAccelerometerAvailable)
        assertFalse(service.gyroscopeAvailable)
        assertFalse(service.uncalibratedGyroscopeAvailable)
        assertFalse(service.magnetometerAvailable)
        assertFalse(service.uncalibratedMagnetometerAvailable)
        assertFalse(service.gravityAvailable)
        assertFalse(service.absoluteAttitudeAvailable)
        assertFalse(service.relativeAttitudeAvailable)
    }

    @Config(sdk = [Build.VERSION_CODES.N])
    @Test
    fun constructor_whenSdkN_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val service = SensorAvailabilityService(context)

        assertSame(context, service.context)
        assertFalse(service.accelerometerAvailable)
        assertFalse(service.linearAccelerometerAvailable)
        assertFalse(service.uncalibratedAccelerometerAvailable)
        assertFalse(service.gyroscopeAvailable)
        assertFalse(service.uncalibratedGyroscopeAvailable)
        assertFalse(service.magnetometerAvailable)
        assertFalse(service.uncalibratedMagnetometerAvailable)
        assertFalse(service.gravityAvailable)
        assertFalse(service.absoluteAttitudeAvailable)
        assertFalse(service.relativeAttitudeAvailable)
    }

    @Config(sdk = [Build.VERSION_CODES.O])
    @Test
    fun hasSensor_whenSdkO_returnsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val service = SensorAvailabilityService(contextSpy)

        assertFalse(service.hasSensor(SensorAvailabilityService.SensorType.ACCELEROMETER))
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ACCELEROMETER) }

        assertFalse(service.hasSensor(SensorAvailabilityService.SensorType.LINEAR_ACCELERATION))
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION) }

        assertFalse(
            service.hasSensor(
                SensorAvailabilityService.SensorType.ACCELEROMETER_UNCALIBRATED
            )
        )
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(TYPE_ACCELEROMETER_UNCALIBRATED) }

        assertFalse(service.hasSensor(SensorAvailabilityService.SensorType.GYROSCOPE))
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE) }

        assertFalse(service.hasSensor(SensorAvailabilityService.SensorType.GYROSCOPE_UNCALIBRATED))
        verify(exactly = 1) {
            sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
        }

        assertFalse(service.hasSensor(SensorAvailabilityService.SensorType.MAGNETOMETER))
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD) }

        assertFalse(
            service.hasSensor(
                SensorAvailabilityService.SensorType.MAGNETOMETER_UNCALIBRATED
            )
        )
        verify(exactly = 1) {
            sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED)
        }

        assertFalse(service.hasSensor(SensorAvailabilityService.SensorType.GRAVITY))
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }

        assertFalse(service.hasSensor(SensorAvailabilityService.SensorType.ABSOLUTE_ATTITUDE))
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) }

        assertFalse(service.hasSensor(SensorAvailabilityService.SensorType.RELATIVE_ATTITUDE))
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR) }

        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
    }

    @Config(sdk = [Build.VERSION_CODES.N])
    @Test
    fun hasSensor_whenSdkN_returnsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val service = SensorAvailabilityService(contextSpy)

        assertFalse(service.hasSensor(SensorAvailabilityService.SensorType.ACCELEROMETER))
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ACCELEROMETER) }

        assertFalse(service.hasSensor(SensorAvailabilityService.SensorType.LINEAR_ACCELERATION))
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION) }

        assertFalse(
            service.hasSensor(
                SensorAvailabilityService.SensorType.ACCELEROMETER_UNCALIBRATED
            )
        )
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(TYPE_ACCELEROMETER_UNCALIBRATED) }

        assertFalse(service.hasSensor(SensorAvailabilityService.SensorType.GYROSCOPE))
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE) }

        assertFalse(service.hasSensor(SensorAvailabilityService.SensorType.GYROSCOPE_UNCALIBRATED))
        verify(exactly = 1) {
            sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
        }

        assertFalse(service.hasSensor(SensorAvailabilityService.SensorType.MAGNETOMETER))
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD) }

        assertFalse(
            service.hasSensor(
                SensorAvailabilityService.SensorType.MAGNETOMETER_UNCALIBRATED
            )
        )
        verify(exactly = 1) {
            sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED)
        }

        assertFalse(service.hasSensor(SensorAvailabilityService.SensorType.GRAVITY))
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }

        assertFalse(service.hasSensor(SensorAvailabilityService.SensorType.ABSOLUTE_ATTITUDE))
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) }

        assertFalse(service.hasSensor(SensorAvailabilityService.SensorType.RELATIVE_ATTITUDE))
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR) }

        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
    }

    @Config(sdk = [Build.VERSION_CODES.O])
    @Test
    fun sensorType_fromValuesSdkO_returnsExpectedValues() {
        assertEquals(10, SensorAvailabilityService.SensorType.values().size)
        assertEquals(
            SensorAvailabilityService.SensorType.ACCELEROMETER,
            SensorAvailabilityService.SensorType.from(
                Sensor.TYPE_ACCELEROMETER
            )
        )
        assertEquals(
            SensorAvailabilityService.SensorType.LINEAR_ACCELERATION,
            SensorAvailabilityService.SensorType.from(Sensor.TYPE_LINEAR_ACCELERATION)
        )
        assertEquals(
            SensorAvailabilityService.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorAvailabilityService.SensorType.from(TYPE_ACCELEROMETER_UNCALIBRATED)
        )
        assertEquals(
            SensorAvailabilityService.SensorType.GYROSCOPE,
            SensorAvailabilityService.SensorType.from(Sensor.TYPE_GYROSCOPE)
        )
        assertEquals(
            SensorAvailabilityService.SensorType.GYROSCOPE_UNCALIBRATED,
            SensorAvailabilityService.SensorType.from(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
        )
        assertEquals(
            SensorAvailabilityService.SensorType.MAGNETOMETER,
            SensorAvailabilityService.SensorType.from(Sensor.TYPE_MAGNETIC_FIELD)
        )
        assertEquals(
            SensorAvailabilityService.SensorType.MAGNETOMETER_UNCALIBRATED,
            SensorAvailabilityService.SensorType.from(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED)
        )
        assertEquals(
            SensorAvailabilityService.SensorType.GRAVITY,
            SensorAvailabilityService.SensorType.from(Sensor.TYPE_GRAVITY)
        )
        assertEquals(
            SensorAvailabilityService.SensorType.ABSOLUTE_ATTITUDE,
            SensorAvailabilityService.SensorType.from(Sensor.TYPE_ROTATION_VECTOR)
        )
        assertEquals(
            SensorAvailabilityService.SensorType.RELATIVE_ATTITUDE,
            SensorAvailabilityService.SensorType.from(Sensor.TYPE_GAME_ROTATION_VECTOR)
        )
    }

    @Config(sdk = [Build.VERSION_CODES.N])
    @Test
    fun sensorType_fromValuesSdkN_returnsExpectedValues() {
        assertEquals(10, SensorAvailabilityService.SensorType.values().size)
        assertEquals(
            SensorAvailabilityService.SensorType.ACCELEROMETER,
            SensorAvailabilityService.SensorType.from(
                Sensor.TYPE_ACCELEROMETER
            )
        )
        assertEquals(
            SensorAvailabilityService.SensorType.LINEAR_ACCELERATION,
            SensorAvailabilityService.SensorType.from(Sensor.TYPE_LINEAR_ACCELERATION)
        )
        assertNull(SensorAvailabilityService.SensorType.from(TYPE_ACCELEROMETER_UNCALIBRATED))
        assertEquals(
            SensorAvailabilityService.SensorType.GYROSCOPE,
            SensorAvailabilityService.SensorType.from(Sensor.TYPE_GYROSCOPE)
        )
        assertEquals(
            SensorAvailabilityService.SensorType.GYROSCOPE_UNCALIBRATED,
            SensorAvailabilityService.SensorType.from(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
        )
        assertEquals(
            SensorAvailabilityService.SensorType.MAGNETOMETER,
            SensorAvailabilityService.SensorType.from(Sensor.TYPE_MAGNETIC_FIELD)
        )
        assertEquals(
            SensorAvailabilityService.SensorType.MAGNETOMETER_UNCALIBRATED,
            SensorAvailabilityService.SensorType.from(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED)
        )
        assertEquals(
            SensorAvailabilityService.SensorType.GRAVITY,
            SensorAvailabilityService.SensorType.from(Sensor.TYPE_GRAVITY)
        )
        assertEquals(
            SensorAvailabilityService.SensorType.ABSOLUTE_ATTITUDE,
            SensorAvailabilityService.SensorType.from(Sensor.TYPE_ROTATION_VECTOR)
        )
        assertEquals(
            SensorAvailabilityService.SensorType.RELATIVE_ATTITUDE,
            SensorAvailabilityService.SensorType.from(Sensor.TYPE_GAME_ROTATION_VECTOR)
        )
    }

    companion object {
        const val TYPE_ACCELEROMETER_UNCALIBRATED = 35
    }
}