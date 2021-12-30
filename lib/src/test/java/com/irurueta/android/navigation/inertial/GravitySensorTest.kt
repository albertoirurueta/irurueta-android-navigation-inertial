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
import kotlin.math.sqrt

@RunWith(RobolectricTestRunner::class)
class GravitySensorTest {

    @Test
    fun constructor_whenContext_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensor = GravitySensor(context)

        // check values
        assertSame(context, sensor.context)
        assertEquals(SensorDelay.FASTEST, sensor.sensorDelay)
        assertNull(sensor.gravityMeasurementListener)
        assertNull(sensor.gravityAccuracyChangedListener)
        assertNull(sensor.sensor)
        assertFalse(sensor.sensorAvailable)

        val sensorManager: SensorManager? = sensor.getPrivateProperty("sensorManager")
        assertNotNull(sensorManager)
    }

    @Test
    fun constructor_whenSensorDelay_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensor = GravitySensor(context, SensorDelay.GAME)

        // check values
        assertSame(context, sensor.context)
        assertEquals(SensorDelay.GAME, sensor.sensorDelay)
        assertNull(sensor.gravityMeasurementListener)
        assertNull(sensor.gravityAccuracyChangedListener)
        assertNull(sensor.sensor)
        assertFalse(sensor.sensorAvailable)

        val sensorManager: SensorManager? = sensor.getPrivateProperty("sensorManager")
        assertNotNull(sensorManager)
    }

    @Test
    fun constructor_whenMeasurementListener_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val gravityMeasurementListener = mockk<GravitySensor.OnGravityMeasurementListener>()
        val sensor = GravitySensor(context, SensorDelay.UI, gravityMeasurementListener)

        // check values
        assertSame(context, sensor.context)
        assertEquals(SensorDelay.UI, sensor.sensorDelay)
        assertSame(gravityMeasurementListener, sensor.gravityMeasurementListener)
        assertNull(sensor.gravityAccuracyChangedListener)
        assertNull(sensor.sensor)
        assertFalse(sensor.sensorAvailable)

        val sensorManager: SensorManager? = sensor.getPrivateProperty("sensorManager")
        assertNotNull(sensorManager)
    }

    @Test
    fun constructor_whenAccuracyListener_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val gravityMeasurementListener = mockk<GravitySensor.OnGravityMeasurementListener>()
        val gravityAccuracyListener = mockk<GravitySensor.OnGravityAccuracyChangedListener>()
        val sensor = GravitySensor(
            context,
            SensorDelay.FASTEST,
            gravityMeasurementListener,
            gravityAccuracyListener
        )

        // check values
        assertSame(context, sensor.context)
        assertEquals(SensorDelay.FASTEST, sensor.sensorDelay)
        assertSame(gravityMeasurementListener, sensor.gravityMeasurementListener)
        assertSame(gravityAccuracyListener, sensor.gravityAccuracyChangedListener)
        assertNull(sensor.sensor)
        assertFalse(sensor.sensorAvailable)

        val sensorManager: SensorManager? = sensor.getPrivateProperty("sensorManager")
        assertNotNull(sensorManager)
    }

    @Test
    fun gravityMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensor = GravitySensor(context)

        assertNull(sensor.gravityMeasurementListener)

        // set new value
        val gravityMeasurementListener = mockk<GravitySensor.OnGravityMeasurementListener>()
        sensor.gravityMeasurementListener = gravityMeasurementListener

        // check
        assertSame(gravityMeasurementListener, sensor.gravityMeasurementListener)
    }

    @Test
    fun gravityAccuracyChangedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensor = GravitySensor(context)

        assertNull(sensor.gravityAccuracyChangedListener)

        // set new value
        val gravityAccuracyChangedListener = mockk<GravitySensor.OnGravityAccuracyChangedListener>()
        sensor.gravityAccuracyChangedListener = gravityAccuracyChangedListener

        // check
        assertSame(gravityAccuracyChangedListener, sensor.gravityAccuracyChangedListener)
    }

    @Test
    fun sensor_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(sensorMock)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val sensor = GravitySensor(contextSpy)

        assertSame(sensorMock, sensor.sensor)
        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }
    }

    @Test
    fun sensorAvailable_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val sensor = GravitySensor(contextSpy)

        assertFalse(sensor.sensorAvailable)
        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }
    }

    @Test
    fun start_whenNoSensorManager_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val sensor = GravitySensor(contextSpy)
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
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(null)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val sensor = GravitySensor(contextSpy)
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
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val sensor = GravitySensor(contextSpy)
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

        val sensor = GravitySensor(contextSpy)
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
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(sensorMock)
        justRun { sensorManagerSpy.unregisterListener(any(), any<Sensor>()) }
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val sensor = GravitySensor(contextSpy)
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
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val gravityMeasurementListener = mockk<GravitySensor.OnGravityMeasurementListener>()
        val sensor =
            GravitySensor(contextSpy, gravityMeasurementListener = gravityMeasurementListener)
        assertTrue(sensor.start())

        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(capture(slot), sensorMock, sensor.sensorDelay.value)
        }

        val eventListener = slot.captured

        eventListener.onSensorChanged(null)
        verify { gravityMeasurementListener wasNot Called }
    }

    @Test
    fun onSensorChanged_whenNoSensorType_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val gravityMeasurementListener = mockk<GravitySensor.OnGravityMeasurementListener>()
        val sensor =
            GravitySensor(contextSpy, gravityMeasurementListener = gravityMeasurementListener)
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

        verify { gravityMeasurementListener wasNot Called }
    }

    @Test
    fun onSensorChanged_whenNoListener_doesNotNotify() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val sensor = GravitySensor(contextSpy)
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

        every { sensorMock.type }.returns(Sensor.TYPE_GRAVITY)
        val event = mockk<SensorEvent>()
        event.sensor = sensorMock
        event.timestamp = System.nanoTime()
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, floatArrayOf(1.0f, 2.0f, 3.0f))
        eventListener.onSensorChanged(event)

        verify(exactly = 1) { sensorMock.type }
    }

    @Test
    fun onSensorChanged_whenListener_notifiesEvent() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val gravityMeasurementListener =
            mockk<GravitySensor.OnGravityMeasurementListener>(relaxUnitFun = true)
        val sensor =
            GravitySensor(contextSpy, gravityMeasurementListener = gravityMeasurementListener)
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

        every { sensorMock.type }.returns(Sensor.TYPE_GRAVITY)
        val event = mockk<SensorEvent>()
        event.sensor = sensorMock
        event.timestamp = System.nanoTime()
        event.accuracy = SensorAccuracy.HIGH.value
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, floatArrayOf(1.0f, 2.0f, 3.0f))
        eventListener.onSensorChanged(event)

        verify(exactly = 1) {
            gravityMeasurementListener.onGravityMeasurement(
                1.0f,
                2.0f,
                3.0f,
                sqrt(1.0 + 4.0 + 9.0),
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
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val gravityAccuracyChangedListener =
            mockk<GravitySensor.OnGravityAccuracyChangedListener>(relaxUnitFun = true)
        val sensor = GravitySensor(
            contextSpy,
            gravityAccuracyChangedListener = gravityAccuracyChangedListener
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

        verify { gravityAccuracyChangedListener wasNot Called }
    }

    @Test
    fun onAccuracyChanged_whenNoSensorType_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val gravityAccuracyChangedListener =
            mockk<GravitySensor.OnGravityAccuracyChangedListener>(relaxUnitFun = true)
        val sensor = GravitySensor(
            contextSpy,
            gravityAccuracyChangedListener = gravityAccuracyChangedListener
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

        verify { gravityAccuracyChangedListener wasNot Called }
    }

    @Test
    fun onAccuracyChanged_whenListener_notifiesEvent() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val gravityAccuracyChangedListener =
            mockk<GravitySensor.OnGravityAccuracyChangedListener>(relaxUnitFun = true)
        val sensor = GravitySensor(
            contextSpy,
            gravityAccuracyChangedListener = gravityAccuracyChangedListener
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

        every { sensorMock.type }.returns(Sensor.TYPE_GRAVITY)
        eventListener.onAccuracyChanged(sensorMock, SensorAccuracy.HIGH.value)

        verify(exactly = 1) {
            gravityAccuracyChangedListener.onGravityAccuracyChanged(SensorAccuracy.HIGH)
        }
    }
}