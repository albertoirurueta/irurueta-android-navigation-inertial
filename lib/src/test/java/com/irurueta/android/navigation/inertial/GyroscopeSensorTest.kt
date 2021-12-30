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
import androidx.test.core.app.ApplicationProvider
import io.mockk.*
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class GyroscopeSensorTest {

    @Test
    fun constructor_whenContext_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensor = GyroscopeSensor(context)

        // check values
        assertSame(context, sensor.context)
        assertEquals(GyroscopeSensor.GyroscopeSensorType.GYROSCOPE, sensor.sensorType)
        assertEquals(SensorDelay.FASTEST, sensor.sensorDelay)
        assertNull(sensor.gyroscopeMeasurementListener)
        assertNull(sensor.gyroscopeAccuracyChangedListener)
        assertNull(sensor.sensor)
        assertFalse(sensor.sensorAvailable)

        val sensorManager: SensorManager? = sensor.getPrivateProperty("sensorManager")
        assertNotNull(sensorManager)
    }

    @Test
    fun constructor_whenSensorType_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensor =
            GyroscopeSensor(context, GyroscopeSensor.GyroscopeSensorType.GYROSCOPE_UNCALIBRATED)

        // check values
        assertSame(context, sensor.context)
        assertEquals(GyroscopeSensor.GyroscopeSensorType.GYROSCOPE_UNCALIBRATED, sensor.sensorType)
        assertEquals(SensorDelay.FASTEST, sensor.sensorDelay)
        assertNull(sensor.gyroscopeMeasurementListener)
        assertNull(sensor.gyroscopeAccuracyChangedListener)
        assertNull(sensor.sensor)
        assertFalse(sensor.sensorAvailable)

        val sensorManager: SensorManager? = sensor.getPrivateProperty("sensorManager")
        assertNotNull(sensorManager)
    }

    @Test
    fun constructor_whenSensorDelay_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensor =
            GyroscopeSensor(
                context, GyroscopeSensor.GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
                SensorDelay.NORMAL
            )

        // check values
        assertSame(context, sensor.context)
        assertEquals(GyroscopeSensor.GyroscopeSensorType.GYROSCOPE_UNCALIBRATED, sensor.sensorType)
        assertEquals(SensorDelay.NORMAL, sensor.sensorDelay)
        assertNull(sensor.gyroscopeMeasurementListener)
        assertNull(sensor.gyroscopeAccuracyChangedListener)
        assertNull(sensor.sensor)
        assertFalse(sensor.sensorAvailable)

        val sensorManager: SensorManager? = sensor.getPrivateProperty("sensorManager")
        assertNotNull(sensorManager)
    }

    @Test
    fun constructor_whenMeasurementListener_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val gyroscopeMeasurementListener = mockk<GyroscopeSensor.OnGyroscopeMeasurementListener>()
        val sensor =
            GyroscopeSensor(
                context, GyroscopeSensor.GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
                SensorDelay.NORMAL, gyroscopeMeasurementListener
            )

        // check values
        assertSame(context, sensor.context)
        assertEquals(GyroscopeSensor.GyroscopeSensorType.GYROSCOPE_UNCALIBRATED, sensor.sensorType)
        assertEquals(SensorDelay.NORMAL, sensor.sensorDelay)
        assertSame(gyroscopeMeasurementListener, sensor.gyroscopeMeasurementListener)
        assertNull(sensor.gyroscopeAccuracyChangedListener)
        assertNull(sensor.sensor)
        assertFalse(sensor.sensorAvailable)

        val sensorManager: SensorManager? = sensor.getPrivateProperty("sensorManager")
        assertNotNull(sensorManager)
    }

    @Test
    fun constructor_whenAccuracyListener_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val gyroscopeMeasurementListener = mockk<GyroscopeSensor.OnGyroscopeMeasurementListener>()
        val gyroscopeAccuracyChangedListener =
            mockk<GyroscopeSensor.OnGyroscopeAccuracyChangedListener>()
        val sensor =
            GyroscopeSensor(
                context, GyroscopeSensor.GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
                SensorDelay.NORMAL, gyroscopeMeasurementListener, gyroscopeAccuracyChangedListener
            )

        // check values
        assertSame(context, sensor.context)
        assertEquals(GyroscopeSensor.GyroscopeSensorType.GYROSCOPE_UNCALIBRATED, sensor.sensorType)
        assertEquals(SensorDelay.NORMAL, sensor.sensorDelay)
        assertSame(gyroscopeMeasurementListener, sensor.gyroscopeMeasurementListener)
        assertSame(gyroscopeAccuracyChangedListener, sensor.gyroscopeAccuracyChangedListener)
        assertNull(sensor.sensor)
        assertFalse(sensor.sensorAvailable)

        val sensorManager: SensorManager? = sensor.getPrivateProperty("sensorManager")
        assertNotNull(sensorManager)
    }

    @Test
    fun gyroscopeMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensor = GyroscopeSensor(context)

        assertNull(sensor.gyroscopeMeasurementListener)

        // set new value
        val gyroscopeMeasurementListener = mockk<GyroscopeSensor.OnGyroscopeMeasurementListener>()
        sensor.gyroscopeMeasurementListener = gyroscopeMeasurementListener

        // check
        assertSame(gyroscopeMeasurementListener, sensor.gyroscopeMeasurementListener)
    }

    @Test
    fun gyroscopeAccuracyChangedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensor = GyroscopeSensor(context)

        assertNull(sensor.gyroscopeAccuracyChangedListener)

        // set new value
        val gyroscopeAccuracyChangedListener =
            mockk<GyroscopeSensor.OnGyroscopeAccuracyChangedListener>()
        sensor.gyroscopeAccuracyChangedListener = gyroscopeAccuracyChangedListener

        // check
        assertSame(gyroscopeAccuracyChangedListener, sensor.gyroscopeAccuracyChangedListener)
    }

    @Test
    fun sensor_whenSensorTypeGyroscope_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE) }.returns(sensorMock)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val sensor = GyroscopeSensor(contextSpy, GyroscopeSensor.GyroscopeSensorType.GYROSCOPE)

        assertSame(sensorMock, sensor.sensor)
        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE) }
    }

    @Test
    fun sensor_whenSensorTypeGyroscopeUncalibrated_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED) }.returns(
            sensorMock
        )
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val sensor =
            GyroscopeSensor(contextSpy, GyroscopeSensor.GyroscopeSensorType.GYROSCOPE_UNCALIBRATED)

        assertSame(sensorMock, sensor.sensor)
        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) {
            sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
        }
    }

    @Test
    fun sensorAvailable_whenSensorTypeGyroscope_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val sensor = GyroscopeSensor(contextSpy, GyroscopeSensor.GyroscopeSensorType.GYROSCOPE)

        assertFalse(sensor.sensorAvailable)
        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE) }
    }

    @Test
    fun sensorAvailable_whenSensorTypeGyroscopeUncalibrated_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val sensor = GyroscopeSensor(
            contextSpy,
            GyroscopeSensor.GyroscopeSensorType.GYROSCOPE_UNCALIBRATED
        )

        assertFalse(sensor.sensorAvailable)
        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) {
            sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
        }
    }

    @Test
    fun start_whenNoSensorManager_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val sensor = GyroscopeSensor(contextSpy)
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
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE) }.returns(null)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val sensor = GyroscopeSensor(contextSpy)
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
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE) }.returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val sensor = GyroscopeSensor(contextSpy)
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

        val sensor = GyroscopeSensor(contextSpy)
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
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE) }
            .returns(sensorMock)
        justRun { sensorManagerSpy.unregisterListener(any(), any<Sensor>()) }
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val sensor = GyroscopeSensor(contextSpy)
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
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE) }.returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val gyroscopeMeasurementListener = mockk<GyroscopeSensor.OnGyroscopeMeasurementListener>()
        val sensor = GyroscopeSensor(
            contextSpy,
            gyroscopeMeasurementListener = gyroscopeMeasurementListener
        )
        assertTrue(sensor.start())

        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(capture(slot), sensorMock, sensor.sensorDelay.value)
        }

        val eventListener = slot.captured

        eventListener.onSensorChanged(null)
        verify { gyroscopeMeasurementListener wasNot Called }
    }

    @Test
    fun onSensorChanged_whenNoSensorType_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE) }.returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val gyroscopeMeasurementListener = mockk<GyroscopeSensor.OnGyroscopeMeasurementListener>()
        val sensor = GyroscopeSensor(
            contextSpy,
            gyroscopeMeasurementListener = gyroscopeMeasurementListener
        )
        assertTrue(sensor.start())

        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(capture(slot), sensorMock, sensor.sensorDelay.value)
        }

        val eventListener = slot.captured

        every { sensorMock.type }.returns(-1)
        val event = mockk<SensorEvent>()
        event.sensor = sensorMock
        eventListener.onSensorChanged(event)

        verify { gyroscopeMeasurementListener wasNot Called }
    }

    @Test
    fun onSensorChanged_whenNoListener_doesNotNotify() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE) }.returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val sensor = GyroscopeSensor(contextSpy)
        assertTrue(sensor.start())

        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(capture(slot), sensorMock, sensor.sensorDelay.value)
        }

        val eventListener = slot.captured

        every { sensorMock.type }.returns(-1)
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
    fun onSensorChanged_whenListenerGyroscopeSensor_notifiesEvent() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE) }.returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val gyroscopeMeasurementListener =
            mockk<GyroscopeSensor.OnGyroscopeMeasurementListener>(relaxUnitFun = true)
        val sensor = GyroscopeSensor(
            contextSpy,
            gyroscopeMeasurementListener = gyroscopeMeasurementListener
        )
        assertTrue(sensor.start())

        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(capture(slot), sensorMock, sensor.sensorDelay.value)
        }

        val eventListener = slot.captured

        every { sensorMock.type }.returns(GyroscopeSensor.GyroscopeSensorType.GYROSCOPE.value)
        val event = mockk<SensorEvent>()
        event.sensor = sensorMock
        event.timestamp = System.nanoTime()
        event.accuracy = SensorAccuracy.HIGH.value
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, floatArrayOf(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f))
        eventListener.onSensorChanged(event)

        verify(exactly = 1) {
            gyroscopeMeasurementListener.onGyroscopeMeasurement(
                1.0f,
                2.0f,
                3.0f,
                null,
                null,
                null,
                event.timestamp,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun onSensorChanged_whenListenerGyroscopeUncalibratedSensor_notifiesEvent() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED) }.returns(
            sensorMock
        )
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val gyroscopeMeasurementListener =
            mockk<GyroscopeSensor.OnGyroscopeMeasurementListener>(relaxUnitFun = true)
        val sensor = GyroscopeSensor(
            contextSpy,
            GyroscopeSensor.GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            gyroscopeMeasurementListener = gyroscopeMeasurementListener
        )
        assertTrue(sensor.start())

        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(capture(slot), sensorMock, sensor.sensorDelay.value)
        }

        val eventListener = slot.captured

        every { sensorMock.type }
            .returns(GyroscopeSensor.GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value)
        val event = mockk<SensorEvent>()
        event.sensor = sensorMock
        event.timestamp = System.nanoTime()
        event.accuracy = SensorAccuracy.HIGH.value
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, floatArrayOf(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f))
        eventListener.onSensorChanged(event)

        verify(exactly = 1) {
            gyroscopeMeasurementListener.onGyroscopeMeasurement(
                1.0f,
                2.0f,
                3.0f,
                4.0f,
                5.0f,
                6.0f,
                event.timestamp,
                SensorAccuracy.HIGH
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
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE) }
            .returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val gyroscopeAccuracyChangedListener =
            mockk<GyroscopeSensor.OnGyroscopeAccuracyChangedListener>()
        val sensor = GyroscopeSensor(
            contextSpy,
            gyroscopeAccuracyChangedListener = gyroscopeAccuracyChangedListener
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

        verify { gyroscopeAccuracyChangedListener wasNot Called }
    }

    @Test
    fun onAccuracyChanged_whenNoSensorType_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE) }
            .returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val gyroscopeAccuracyChangedListener =
            mockk<GyroscopeSensor.OnGyroscopeAccuracyChangedListener>()
        val sensor = GyroscopeSensor(
            contextSpy,
            gyroscopeAccuracyChangedListener = gyroscopeAccuracyChangedListener
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

        verify { gyroscopeAccuracyChangedListener wasNot Called }
    }

    @Test
    fun onAccuracyChanged_whenListener_notifiesEvent() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE) }
            .returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val gyroscopeAccuracyChangedListener =
            mockk<GyroscopeSensor.OnGyroscopeAccuracyChangedListener>(relaxUnitFun = true)
        val sensor = GyroscopeSensor(
            contextSpy,
            gyroscopeAccuracyChangedListener = gyroscopeAccuracyChangedListener
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

        every { sensorMock.type }.returns(GyroscopeSensor.GyroscopeSensorType.GYROSCOPE.value)
        eventListener.onAccuracyChanged(sensorMock, SensorAccuracy.HIGH.value)

        verify(exactly = 1) {
            gyroscopeAccuracyChangedListener.onGyroscopeAccuracyChanged(
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun gyroscopeSensorType_fromValues_returnsExpected() {
        assertEquals(2, GyroscopeSensor.GyroscopeSensorType.values().size)
        assertEquals(
            GyroscopeSensor.GyroscopeSensorType.GYROSCOPE,
            GyroscopeSensor.GyroscopeSensorType.from(Sensor.TYPE_GYROSCOPE)
        )
        assertEquals(
            GyroscopeSensor.GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            GyroscopeSensor.GyroscopeSensorType.from(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
        )
    }
}