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
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import android.os.Build
import androidx.test.core.app.ApplicationProvider
import io.mockk.*
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import org.robolectric.annotation.Config

@RunWith(RobolectricTestRunner::class)
class AccelerometerSensorTest {

    @Test
    fun constructor_whenContext_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensor = AccelerometerSensor(context)

        // check values
        assertSame(context, sensor.context)
        assertEquals(AccelerometerSensor.AccelerometerSensorType.ACCELEROMETER, sensor.sensorType)
        assertEquals(SensorDelay.FASTEST, sensor.sensorDelay)
        assertNull(sensor.accelerometerMeasurementListener)
        assertNull(sensor.accelerometerAccuracyChangedListener)
        assertNull(sensor.sensor)
        assertFalse(sensor.sensorAvailable)

        val sensorManager: SensorManager? = sensor.getPrivateProperty("sensorManager")
        assertNotNull(sensorManager)
    }

    @Test
    fun constructor_whenSensorType_setExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensor = AccelerometerSensor(
            context,
            AccelerometerSensor.AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        )

        // check values
        assertSame(context, sensor.context)
        assertEquals(
            AccelerometerSensor.AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            sensor.sensorType
        )
        assertEquals(SensorDelay.FASTEST, sensor.sensorDelay)
        assertNull(sensor.accelerometerMeasurementListener)
        assertNull(sensor.accelerometerAccuracyChangedListener)
        assertNull(sensor.sensor)
        assertFalse(sensor.sensorAvailable)

        val sensorManager: SensorManager? = sensor.getPrivateProperty("sensorManager")
        assertNotNull(sensorManager)
    }

    @Test
    fun constructor_whenSensorDelay_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensor = AccelerometerSensor(
            context,
            AccelerometerSensor.AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL
        )

        // check values
        assertSame(context, sensor.context)
        assertEquals(
            AccelerometerSensor.AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            sensor.sensorType
        )
        assertEquals(SensorDelay.NORMAL, sensor.sensorDelay)
        assertNull(sensor.accelerometerMeasurementListener)
        assertNull(sensor.accelerometerAccuracyChangedListener)
        assertNull(sensor.sensor)
        assertFalse(sensor.sensorAvailable)

        val sensorManager: SensorManager? = sensor.getPrivateProperty("sensorManager")
        assertNotNull(sensorManager)
    }

    @Test
    fun constructor_whenMeasurementListener_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val accelerometerMeasurementListener =
            mockk<AccelerometerSensor.OnAccelerometerMeasurementListener>()
        val sensor = AccelerometerSensor(
            context,
            AccelerometerSensor.AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            accelerometerMeasurementListener
        )

        // check values
        assertSame(context, sensor.context)
        assertEquals(
            AccelerometerSensor.AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            sensor.sensorType
        )
        assertEquals(SensorDelay.NORMAL, sensor.sensorDelay)
        assertSame(accelerometerMeasurementListener, sensor.accelerometerMeasurementListener)
        assertNull(sensor.accelerometerAccuracyChangedListener)
        assertNull(sensor.sensor)
        assertFalse(sensor.sensorAvailable)

        val sensorManager: SensorManager? = sensor.getPrivateProperty("sensorManager")
        assertNotNull(sensorManager)
    }

    @Test
    fun constructor_whenAccuracyListener_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val accelerometerMeasurementListener =
            mockk<AccelerometerSensor.OnAccelerometerMeasurementListener>()
        val accelerometerAccuracyChangedListener =
            mockk<AccelerometerSensor.OnAccelerometerAccuracyChangedListener>()
        val sensor = AccelerometerSensor(
            context,
            AccelerometerSensor.AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            accelerometerMeasurementListener,
            accelerometerAccuracyChangedListener
        )

        // check values
        assertSame(context, sensor.context)
        assertEquals(
            AccelerometerSensor.AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            sensor.sensorType
        )
        assertEquals(SensorDelay.NORMAL, sensor.sensorDelay)
        assertSame(accelerometerMeasurementListener, sensor.accelerometerMeasurementListener)
        assertSame(
            accelerometerAccuracyChangedListener,
            sensor.accelerometerAccuracyChangedListener
        )
        assertNull(sensor.sensor)
        assertFalse(sensor.sensorAvailable)

        val sensorManager: SensorManager? = sensor.getPrivateProperty("sensorManager")
        assertNotNull(sensorManager)
    }

    @Test
    fun accelerometerMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensor = AccelerometerSensor(context)

        assertNull(sensor.accelerometerMeasurementListener)

        // set new value
        val accelerometerMeasurementListener =
            mockk<AccelerometerSensor.OnAccelerometerMeasurementListener>()
        sensor.accelerometerMeasurementListener = accelerometerMeasurementListener

        // check
        assertSame(accelerometerMeasurementListener, sensor.accelerometerMeasurementListener)
    }

    @Test
    fun accelerometerAccuracyChangedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensor = AccelerometerSensor(context)

        assertNull(sensor.accelerometerAccuracyChangedListener)

        // set new value
        val accelerometerAccuracyChangedListener =
            mockk<AccelerometerSensor.OnAccelerometerAccuracyChangedListener>()
        sensor.accelerometerAccuracyChangedListener = accelerometerAccuracyChangedListener

        // check
        assertSame(
            accelerometerAccuracyChangedListener,
            sensor.accelerometerAccuracyChangedListener
        )
    }

    @Test
    fun sensor_whenSensorTypeAccelerometer_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ACCELEROMETER) }.returns(sensorMock)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val sensor = AccelerometerSensor(
            contextSpy,
            AccelerometerSensor.AccelerometerSensorType.ACCELEROMETER
        )

        assertSame(sensorMock, sensor.sensor)
        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ACCELEROMETER) }
    }

    @Test
    fun sensor_whenSensorTypeAccelerometerUncalibrated_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(TYPE_ACCELEROMETER_UNCALIBRATED) }
            .returns(sensorMock)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val sensor = AccelerometerSensor(
            contextSpy,
            AccelerometerSensor.AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        )

        assertSame(sensorMock, sensor.sensor)
        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) {
            sensorManagerSpy.getDefaultSensor(TYPE_ACCELEROMETER_UNCALIBRATED)
        }
    }

    @Test
    fun sensorAvailable_whenSensorTypeAccelerometer_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val sensor = AccelerometerSensor(
            contextSpy,
            AccelerometerSensor.AccelerometerSensorType.ACCELEROMETER
        )

        assertFalse(sensor.sensorAvailable)
        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ACCELEROMETER) }
    }

    @Config(sdk = [Build.VERSION_CODES.O])
    @Test
    fun sensorAvailable_whenSensorTypeAccelerometerUncalibratedSdkO_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(TYPE_ACCELEROMETER_UNCALIBRATED) }
            .returns(sensorMock)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val sensor = AccelerometerSensor(
            contextSpy,
            AccelerometerSensor.AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        )

        assertTrue(sensor.sensorAvailable)
        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(TYPE_ACCELEROMETER_UNCALIBRATED) }
    }

    @Config(sdk = [Build.VERSION_CODES.N])
    @Test
    fun sensorAvailable_whenSensorTypeAccelerometerUncalibratedSdkN_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(TYPE_ACCELEROMETER_UNCALIBRATED) }
            .returns(sensorMock)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val sensor = AccelerometerSensor(
            contextSpy,
            AccelerometerSensor.AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        )

        assertFalse(sensor.sensorAvailable)
        verify(exactly = 0) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 0) { sensorManagerSpy.getDefaultSensor(TYPE_ACCELEROMETER_UNCALIBRATED) }
    }

    @Test
    fun start_whenNoSensorManager_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val sensor = AccelerometerSensor(contextSpy)
        assertFalse(sensor.start())

        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
    }

    @Test
    fun start_whenNoSensor_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ACCELEROMETER) }
            .returns(null)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val sensor = AccelerometerSensor(contextSpy)
        assertFalse(sensor.start())

        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 0) {
            sensorManagerSpy.registerListener(
                any(),
                any<Sensor>(),
                sensor.sensorDelay.value
            )
        }
    }

    @Test
    fun start_whenSensorManager_registersListener() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ACCELEROMETER) }
            .returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val sensor = AccelerometerSensor(contextSpy)
        assertTrue(sensor.start())

        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(
                capture(slot),
                sensorMock,
                sensor.sensorDelay.value
            )
        }

        val eventListener = slot.captured
        assertNotNull(eventListener)
    }

    @Test
    fun stop_whenNoSensorManager_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val sensor = AccelerometerSensor(contextSpy)
        sensor.stop()
    }

    @Test
    fun stop_whenSensorManager_unregistersListener() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ACCELEROMETER) }
            .returns(sensorMock)
        justRun { sensorManagerSpy.unregisterListener(any(), any<Sensor>()) }
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val sensor = AccelerometerSensor(contextSpy)
        sensor.stop()

        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        val slot = slot<SensorEventListener>()
        verify(exactly = 1) { sensorManagerSpy.unregisterListener(capture(slot), sensorMock) }

        val eventListener = slot.captured
        assertNotNull(eventListener)
    }

    @Test
    fun onSensorChanged_whenNoEvent_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ACCELEROMETER) }
            .returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val accelerometerMeasurementListener =
            mockk<AccelerometerSensor.OnAccelerometerMeasurementListener>()
        val sensor = AccelerometerSensor(
            contextSpy,
            accelerometerMeasurementListener = accelerometerMeasurementListener
        )
        assertTrue(sensor.start())

        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(
                capture(slot),
                sensorMock,
                sensor.sensorDelay.value
            )
        }

        val eventListener = slot.captured

        eventListener.onSensorChanged(null)
        verify { accelerometerMeasurementListener wasNot Called }
    }

    @Test
    fun onSensorChanged_whenNoSensorType_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ACCELEROMETER) }
            .returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val accelerometerMeasurementListener =
            mockk<AccelerometerSensor.OnAccelerometerMeasurementListener>()
        val sensor = AccelerometerSensor(
            contextSpy,
            accelerometerMeasurementListener = accelerometerMeasurementListener
        )
        assertTrue(sensor.start())

        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(
                capture(slot),
                sensorMock,
                sensor.sensorDelay.value
            )
        }

        val eventListener = slot.captured

        every { sensorMock.type }.returns(-1)
        val event = mockk<SensorEvent>()
        event.sensor = sensorMock
        eventListener.onSensorChanged(event)

        verify { accelerometerMeasurementListener wasNot Called }
    }

    @Test
    fun onSensorChanged_whenNoListener_doesNotNotify() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ACCELEROMETER) }
            .returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val sensor = AccelerometerSensor(contextSpy)
        assertTrue(sensor.start())

        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(
                capture(slot),
                sensorMock,
                sensor.sensorDelay.value
            )
        }

        val eventListener = slot.captured

        every { sensorMock.type }
            .returns(AccelerometerSensor.AccelerometerSensorType.ACCELEROMETER.value)
        val event = mockk<SensorEvent>()
        event.sensor = sensorMock
        event.timestamp = System.nanoTime()
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, floatArrayOf(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f))
        eventListener.onSensorChanged(event)

        verify(exactly = 1) { sensorMock.type }
    }

    @Test
    fun onSensorChanged_whenListenerAccelerometerSensor_notifiesEvent() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ACCELEROMETER) }
            .returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val accelerometerMeasurementListener =
            mockk<AccelerometerSensor.OnAccelerometerMeasurementListener>(relaxUnitFun = true)
        val sensor = AccelerometerSensor(
            contextSpy,
            accelerometerMeasurementListener = accelerometerMeasurementListener
        )
        assertTrue(sensor.start())

        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(
                capture(slot),
                sensorMock,
                sensor.sensorDelay.value
            )
        }

        val eventListener = slot.captured

        every { sensorMock.type }
            .returns(AccelerometerSensor.AccelerometerSensorType.ACCELEROMETER.value)
        val event = mockk<SensorEvent>()
        event.sensor = sensorMock
        event.timestamp = System.nanoTime()
        event.accuracy = SensorAccuracy.HIGH.value
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, floatArrayOf(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f))
        eventListener.onSensorChanged(event)

        verify(exactly = 1) {
            accelerometerMeasurementListener.onAccelerometerMeasurement(
                1.0f, 2.0f, 3.0f, null, null, null, event.timestamp, SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun onSensorChanged_whenListenerAccelerometerUncalibratedSensor_notifiesEvent() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(TYPE_ACCELEROMETER_UNCALIBRATED) }
            .returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val accelerometerMeasurementListener =
            mockk<AccelerometerSensor.OnAccelerometerMeasurementListener>(relaxUnitFun = true)
        val sensor = AccelerometerSensor(
            contextSpy,
            AccelerometerSensor.AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            accelerometerMeasurementListener = accelerometerMeasurementListener
        )
        assertTrue(sensor.start())

        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(
                capture(slot),
                sensorMock,
                sensor.sensorDelay.value
            )
        }

        val eventListener = slot.captured

        every { sensorMock.type }
            .returns(AccelerometerSensor.AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value)
        val event = mockk<SensorEvent>()
        event.sensor = sensorMock
        event.timestamp = System.nanoTime()
        event.accuracy = SensorAccuracy.HIGH.value
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, floatArrayOf(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f))
        eventListener.onSensorChanged(event)

        verify(exactly = 1) {
            accelerometerMeasurementListener.onAccelerometerMeasurement(
                1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f, event.timestamp, SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun onAccuracyChanged_whenNoSensor_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ACCELEROMETER) }
            .returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val accelerometerAccuracyChangedListener =
            mockk<AccelerometerSensor.OnAccelerometerAccuracyChangedListener>(relaxUnitFun = true)
        val sensor = AccelerometerSensor(
            contextSpy,
            accelerometerAccuracyChangedListener = accelerometerAccuracyChangedListener
        )
        assertTrue(sensor.start())

        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(
                capture(slot),
                sensorMock,
                sensor.sensorDelay.value
            )
        }

        val eventListener = slot.captured

        eventListener.onAccuracyChanged(null, SensorAccuracy.HIGH.value)

        verify { accelerometerAccuracyChangedListener wasNot Called }
    }

    @Test
    fun onAccuracyChanged_whenNoSensorType_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ACCELEROMETER) }
            .returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val accelerometerAccuracyChangedListener =
            mockk<AccelerometerSensor.OnAccelerometerAccuracyChangedListener>(relaxUnitFun = true)
        val sensor = AccelerometerSensor(
            contextSpy,
            accelerometerAccuracyChangedListener = accelerometerAccuracyChangedListener
        )
        assertTrue(sensor.start())

        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(
                capture(slot),
                sensorMock,
                sensor.sensorDelay.value
            )
        }

        val eventListener = slot.captured

        every { sensorMock.type }.returns(-1)
        eventListener.onAccuracyChanged(sensorMock, SensorAccuracy.HIGH.value)

        verify { accelerometerAccuracyChangedListener wasNot Called }
    }

    @Test
    fun onAccuracyChanged_whenListener_notifiesEvent() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ACCELEROMETER) }
            .returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val accelerometerAccuracyChangedListener =
            mockk<AccelerometerSensor.OnAccelerometerAccuracyChangedListener>(relaxUnitFun = true)
        val sensor = AccelerometerSensor(
            contextSpy,
            accelerometerAccuracyChangedListener = accelerometerAccuracyChangedListener
        )
        assertTrue(sensor.start())

        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(
                capture(slot),
                sensorMock,
                sensor.sensorDelay.value
            )
        }

        val eventListener = slot.captured

        every { sensorMock.type }
            .returns(AccelerometerSensor.AccelerometerSensorType.ACCELEROMETER.value)
        eventListener.onAccuracyChanged(sensorMock, SensorAccuracy.HIGH.value)

        verify(exactly = 1) {
            accelerometerAccuracyChangedListener.onAccelerometerAccuracyChanged(SensorAccuracy.HIGH)
        }
    }

    @Config(sdk = [Build.VERSION_CODES.O])
    @Test
    fun accelerometerSensorType_fromValueSdkO_returnsExpected() {
        assertEquals(2, AccelerometerSensor.AccelerometerSensorType.values().size)
        assertEquals(
            AccelerometerSensor.AccelerometerSensorType.ACCELEROMETER,
            AccelerometerSensor.AccelerometerSensorType.from(Sensor.TYPE_ACCELEROMETER)
        )
        assertEquals(
            AccelerometerSensor.AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            AccelerometerSensor.AccelerometerSensorType.from(TYPE_ACCELEROMETER_UNCALIBRATED)
        )
    }

    @Config(sdk = [Build.VERSION_CODES.N])
    @Test
    fun accelerometerSensorType_fromValueSdkN_returnsExpected() {
        assertEquals(2, AccelerometerSensor.AccelerometerSensorType.values().size)
        assertEquals(
            AccelerometerSensor.AccelerometerSensorType.ACCELEROMETER,
            AccelerometerSensor.AccelerometerSensorType.from(Sensor.TYPE_ACCELEROMETER)
        )
        assertNull(
            AccelerometerSensor.AccelerometerSensorType.from(TYPE_ACCELEROMETER_UNCALIBRATED)
        )
    }

    companion object {
        const val TYPE_ACCELEROMETER_UNCALIBRATED = 35
    }
}