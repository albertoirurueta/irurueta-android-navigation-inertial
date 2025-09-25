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
import com.irurueta.android.navigation.inertial.collectors.SensorType
import io.mockk.every
import io.mockk.impl.annotations.SpyK
import io.mockk.junit4.MockKRule
import io.mockk.verify
import org.junit.Assert.assertFalse
import org.junit.Assert.assertSame
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import org.robolectric.annotation.Config

@RunWith(RobolectricTestRunner::class)
class SensorAvailabilityServiceTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    private val context = ApplicationProvider.getApplicationContext<Context>()

    private val sensorManager: SensorManager? =
        context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?

    @SpyK
    private var sensorManagerSpy = sensorManager!!

    @SpyK
    private var contextSpy = context

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
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val service = SensorAvailabilityService(contextSpy)

        assertFalse(service.hasSensor(SensorType.ACCELEROMETER))
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ACCELEROMETER) }

        assertFalse(service.hasSensor(SensorType.LINEAR_ACCELERATION))
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION) }

        assertFalse(
            service.hasSensor(
                SensorType.ACCELEROMETER_UNCALIBRATED
            )
        )
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(TYPE_ACCELEROMETER_UNCALIBRATED) }

        assertFalse(service.hasSensor(SensorType.GYROSCOPE))
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE) }

        assertFalse(service.hasSensor(SensorType.GYROSCOPE_UNCALIBRATED))
        verify(exactly = 1) {
            sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
        }

        assertFalse(service.hasSensor(SensorType.MAGNETOMETER))
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD) }

        assertFalse(
            service.hasSensor(
                SensorType.MAGNETOMETER_UNCALIBRATED
            )
        )
        verify(exactly = 1) {
            sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED)
        }

        assertFalse(service.hasSensor(SensorType.GRAVITY))
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }

        assertFalse(service.hasSensor(SensorType.ABSOLUTE_ATTITUDE))
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) }

        assertFalse(service.hasSensor(SensorType.RELATIVE_ATTITUDE))
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR) }

        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
    }

    @Config(sdk = [Build.VERSION_CODES.N])
    @Test
    fun hasSensor_whenSdkN_returnsExpectedValues() {
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val service = SensorAvailabilityService(contextSpy)

        assertFalse(service.hasSensor(SensorType.ACCELEROMETER))
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ACCELEROMETER) }

        assertFalse(service.hasSensor(SensorType.LINEAR_ACCELERATION))
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION) }

        assertFalse(
            service.hasSensor(SensorType.ACCELEROMETER_UNCALIBRATED)
        )
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(TYPE_ACCELEROMETER_UNCALIBRATED) }

        assertFalse(service.hasSensor(SensorType.GYROSCOPE))
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE) }

        assertFalse(service.hasSensor(SensorType.GYROSCOPE_UNCALIBRATED))
        verify(exactly = 1) {
            sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
        }

        assertFalse(service.hasSensor(SensorType.MAGNETOMETER))
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD) }

        assertFalse(
            service.hasSensor(SensorType.MAGNETOMETER_UNCALIBRATED)
        )
        verify(exactly = 1) {
            sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED)
        }

        assertFalse(service.hasSensor(SensorType.GRAVITY))
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }

        assertFalse(service.hasSensor(SensorType.ABSOLUTE_ATTITUDE))
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) }

        assertFalse(service.hasSensor(SensorType.RELATIVE_ATTITUDE))
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR) }

        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
    }

    companion object {
        const val TYPE_ACCELEROMETER_UNCALIBRATED = 35
    }
}