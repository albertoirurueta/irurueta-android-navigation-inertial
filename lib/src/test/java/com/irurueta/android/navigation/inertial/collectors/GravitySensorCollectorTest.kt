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
package com.irurueta.android.navigation.inertial.collectors

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import androidx.test.core.app.ApplicationProvider
import com.irurueta.android.testutils.getPrivateProperty
import io.mockk.*
import io.mockk.impl.annotations.MockK
import io.mockk.impl.annotations.SpyK
import io.mockk.junit4.MockKRule
import org.junit.After
import org.junit.Assert.*
import org.junit.Ignore
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import kotlin.math.sqrt

@Ignore("possible memory leak")
@RunWith(RobolectricTestRunner::class)
class GravitySensorCollectorTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK(relaxUnitFun = true)
    private lateinit var measurementListener: GravitySensorCollector.OnMeasurementListener

    @MockK
    private lateinit var accuracyListener: SensorCollector.OnAccuracyChangedListener

    @MockK(relaxUnitFun = true)
    private lateinit var accuracyChangedListener: SensorCollector.OnAccuracyChangedListener

    @MockK
    private lateinit var event: SensorEvent

    @MockK
    private lateinit var sensor: Sensor

    private val context = ApplicationProvider.getApplicationContext<Context>()
    private val sensorManager: SensorManager? =
        context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?

    @SpyK
    private var sensorManagerSpy = sensorManager!!

    @SpyK
    private var contextSpy = context

    @After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
    }

    @Test
    fun constructor_whenContext_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = GravitySensorCollector(context)

        // check values
        assertSame(context, collector.context)
        assertEquals(SensorDelay.FASTEST, collector.sensorDelay)
        assertNull(collector.measurementListener)
        assertNull(collector.accuracyChangedListener)
        assertNull(collector.sensor)
        assertFalse(collector.sensorAvailable)

        val sensorManager: SensorManager? =
            getPrivateProperty(SensorCollector::class, collector, "sensorManager")
        assertNotNull(sensorManager)
    }

    @Test
    fun constructor_whenSensorDelay_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = GravitySensorCollector(context, SensorDelay.GAME)

        // check values
        assertSame(context, collector.context)
        assertEquals(SensorDelay.GAME, collector.sensorDelay)
        assertNull(collector.measurementListener)
        assertNull(collector.accuracyChangedListener)
        assertNull(collector.sensor)
        assertFalse(collector.sensorAvailable)

        val sensorManager: SensorManager? =
            getPrivateProperty(SensorCollector::class, collector, "sensorManager")
        assertNotNull(sensorManager)
    }

    @Test
    fun constructor_whenMeasurementListener_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = GravitySensorCollector(context, SensorDelay.UI, measurementListener)

        // check values
        assertSame(context, collector.context)
        assertEquals(SensorDelay.UI, collector.sensorDelay)
        assertSame(measurementListener, collector.measurementListener)
        assertNull(collector.accuracyChangedListener)
        assertNull(collector.sensor)
        assertFalse(collector.sensorAvailable)

        val sensorManager: SensorManager? =
            getPrivateProperty(SensorCollector::class, collector, "sensorManager")
        assertNotNull(sensorManager)
    }

    @Test
    fun constructor_whenAccuracyListener_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = GravitySensorCollector(
            context,
            SensorDelay.FASTEST,
            measurementListener,
            accuracyListener
        )

        // check values
        assertSame(context, collector.context)
        assertEquals(SensorDelay.FASTEST, collector.sensorDelay)
        assertSame(measurementListener, collector.measurementListener)
        assertSame(accuracyListener, collector.accuracyChangedListener)
        assertNull(collector.sensor)
        assertFalse(collector.sensorAvailable)

        val sensorManager: SensorManager? =
            getPrivateProperty(SensorCollector::class, collector, "sensorManager")
        assertNotNull(sensorManager)
    }

    @Test
    fun measurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = GravitySensorCollector(context)

        assertNull(collector.measurementListener)

        // set new value
        collector.measurementListener = measurementListener

        // check
        assertSame(measurementListener, collector.measurementListener)
    }

    @Test
    fun accuracyChangedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = GravitySensorCollector(context)

        assertNull(collector.accuracyChangedListener)

        // set new value
        collector.accuracyChangedListener = accuracyChangedListener

        // check
        assertSame(accuracyChangedListener, collector.accuracyChangedListener)
    }

    @Test
    fun sensor_returnsExpectedValue() {
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(sensor)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = GravitySensorCollector(contextSpy)

        assertSame(sensor, collector.sensor)
        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }
    }

    @Test
    fun sensorAvailable_returnsExpectedValue() {
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = GravitySensorCollector(contextSpy)

        assertFalse(collector.sensorAvailable)
        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }
    }

    @Test
    fun start_whenNoSensorManager_returnsFalse() {
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = GravitySensorCollector(contextSpy)
        assertFalse(collector.start())

        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
    }

    @Test
    fun start_whenNoSensor_returnsFalse() {
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(null)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = GravitySensorCollector(contextSpy)
        assertFalse(collector.start())

        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 0) {
            sensorManagerSpy.registerListener(
                any(),
                any<Sensor>(),
                collector.sensorDelay.value
            )
        }
    }

    @Test
    fun start_whenSensorManager_registersListener() {
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(sensor)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = GravitySensorCollector(contextSpy)
        assertTrue(collector.start())

        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(
                capture(slot),
                sensor,
                collector.sensorDelay.value
            )
        }

        val eventListener = slot.captured
        assertNotNull(eventListener)
    }

    @Test
    fun stop_whenNoSensorManager_makesNoAction() {
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = GravitySensorCollector(contextSpy)
        collector.stop()
    }

    @Test
    fun stop_whenSensorManager_unregistersListener() {
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(sensor)
        justRun { sensorManagerSpy.unregisterListener(any(), any<Sensor>()) }
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = GravitySensorCollector(contextSpy)
        collector.stop()

        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        val slot = slot<SensorEventListener>()
        verify(exactly = 1) { sensorManagerSpy.unregisterListener(capture(slot), sensor) }

        val eventListener = slot.captured
        assertNotNull(eventListener)
    }

    @Test
    fun onSensorChanged_whenNoEvent_makesNoAction() {
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(sensor)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector =
            GravitySensorCollector(contextSpy, measurementListener = measurementListener)
        assertTrue(collector.start())

        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(
                capture(slot),
                sensor,
                collector.sensorDelay.value
            )
        }

        val eventListener = slot.captured

        eventListener.onSensorChanged(null)
        verify { measurementListener wasNot Called }
    }

    @Test
    fun onSensorChanged_whenNoSensorType_makesNoAction() {
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(sensor)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector =
            GravitySensorCollector(contextSpy, measurementListener = measurementListener)
        assertTrue(collector.start())

        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(
                capture(slot),
                sensor,
                collector.sensorDelay.value
            )
        }

        val eventListener = slot.captured

        every { sensor.type }.returns(-1)
        event.sensor = sensor
        eventListener.onSensorChanged(event)

        verify { measurementListener wasNot Called }
    }

    @Test
    fun onSensorChanged_whenNoListener_doesNotNotify() {
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(sensor)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = GravitySensorCollector(contextSpy)
        assertTrue(collector.start())

        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(
                capture(slot),
                sensor,
                collector.sensorDelay.value
            )
        }

        val eventListener = slot.captured

        every { sensor.type }.returns(Sensor.TYPE_GRAVITY)
        event.sensor = sensor
        event.timestamp = System.nanoTime()
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, floatArrayOf(1.0f, 2.0f, 3.0f))
        eventListener.onSensorChanged(event)

        verify(exactly = 1) { sensor.type }
    }

    @Test
    fun onSensorChanged_whenListener_notifiesEvent() {
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(sensor)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector =
            GravitySensorCollector(contextSpy, measurementListener = measurementListener)
        assertTrue(collector.start())

        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(
                capture(slot),
                sensor,
                collector.sensorDelay.value
            )
        }

        val eventListener = slot.captured

        every { sensor.type }.returns(Sensor.TYPE_GRAVITY)
        event.sensor = sensor
        event.timestamp = System.nanoTime()
        event.accuracy = SensorAccuracy.HIGH.value
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, floatArrayOf(1.0f, 2.0f, 3.0f))
        eventListener.onSensorChanged(event)

        verify(exactly = 1) {
            measurementListener.onMeasurement(
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
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(sensor)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = GravitySensorCollector(
            contextSpy,
            accuracyChangedListener = accuracyChangedListener
        )
        assertTrue(collector.start())

        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(
                capture(slot),
                sensor,
                collector.sensorDelay.value
            )
        }

        val eventListener = slot.captured

        eventListener.onAccuracyChanged(null, SensorAccuracy.HIGH.value)

        verify { accuracyChangedListener wasNot Called }
    }

    @Test
    fun onAccuracyChanged_whenNoSensorType_makesNoAction() {
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(sensor)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = GravitySensorCollector(
            contextSpy,
            accuracyChangedListener = accuracyChangedListener
        )
        assertTrue(collector.start())

        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(
                capture(slot),
                sensor,
                collector.sensorDelay.value
            )
        }

        val eventListener = slot.captured

        every { sensor.type }.returns(-1)
        eventListener.onAccuracyChanged(sensor, SensorAccuracy.HIGH.value)

        verify { accuracyChangedListener wasNot Called }
    }

    @Test
    fun onAccuracyChanged_whenListener_notifiesEvent() {
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(sensor)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = GravitySensorCollector(
            contextSpy,
            accuracyChangedListener = accuracyChangedListener
        )
        assertTrue(collector.start())

        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(
                capture(slot),
                sensor,
                collector.sensorDelay.value
            )
        }

        val eventListener = slot.captured

        every { sensor.type }.returns(Sensor.TYPE_GRAVITY)
        eventListener.onAccuracyChanged(sensor, SensorAccuracy.HIGH.value)

        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(SensorAccuracy.HIGH)
        }
    }
}