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
//import io.mockk.*
//import io.mockk.impl.annotations.MockK
//import io.mockk.impl.annotations.SpyK
//import io.mockk.junit4.MockKRule
import org.junit.After
import org.junit.Assert.*
//import org.junit.Ignore
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith
import org.mockito.ArgumentCaptor
import org.mockito.Captor
import org.mockito.Mock
import org.mockito.Spy
import org.mockito.junit.MockitoJUnit
import org.mockito.junit.MockitoRule
import org.mockito.kotlin.any
import org.mockito.kotlin.capture
import org.mockito.kotlin.doNothing
import org.mockito.kotlin.doReturn
import org.mockito.kotlin.eq
import org.mockito.kotlin.never
import org.mockito.kotlin.only
import org.mockito.kotlin.times
import org.mockito.kotlin.verify
import org.mockito.kotlin.verifyNoInteractions
import org.mockito.kotlin.whenever
import org.robolectric.RobolectricTestRunner
import kotlin.math.sqrt

//@Ignore("Possible memory leak when running this test")
@RunWith(RobolectricTestRunner::class)
class GravitySensorCollectorTest {

    @get:Rule
    val mockitoRule: MockitoRule = MockitoJUnit.rule()

//    @get:Rule
//    val mockkRule = MockKRule(this)

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var measurementListener: GravitySensorCollector.OnMeasurementListener

//    @MockK
    @Mock
    private lateinit var accuracyListener: SensorCollector.OnAccuracyChangedListener

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var accuracyChangedListener: SensorCollector.OnAccuracyChangedListener

//    @MockK
    @Mock
    private lateinit var event: SensorEvent

//    @MockK
    @Mock
    private lateinit var sensor: Sensor

    private val context = ApplicationProvider.getApplicationContext<Context>()
    private val sensorManager: SensorManager? =
        context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?

//    @SpyK
    @Spy
    private var sensorManagerSpy = sensorManager!!

//    @SpyK
    @Spy
    private var contextSpy = context

    @Captor
    private lateinit var sensorEventListenerCaptor: ArgumentCaptor<SensorEventListener>

    /*@After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
        System.gc()
    }*/

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
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_GRAVITY)
//        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(sensor)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = GravitySensorCollector(contextSpy)

        assertSame(sensor, collector.sensor)
        verify(contextSpy, only()).getSystemService(Context.SENSOR_SERVICE)
//        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(sensorManagerSpy, only()).getDefaultSensor(Sensor.TYPE_GRAVITY)
//        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }
    }

    @Test
    fun sensorAvailable_returnsExpectedValue() {
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = GravitySensorCollector(contextSpy)

        assertFalse(collector.sensorAvailable)
        verify(contextSpy, only()).getSystemService(Context.SENSOR_SERVICE)
//        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(sensorManagerSpy, times(1)).getDefaultSensor(Sensor.TYPE_GRAVITY)
//        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }
    }

    @Test
    fun start_whenNoSensorManager_returnsFalse() {
        doReturn(null).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = GravitySensorCollector(contextSpy)
        assertFalse(collector.start())

        verify(contextSpy, only()).getSystemService(Context.SENSOR_SERVICE)
//        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
    }

    @Test
    fun start_whenNoSensor_returnsFalse() {
        doReturn(null).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_GRAVITY)
//        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(null)
        doReturn(true).whenever(sensorManagerSpy).registerListener(any(), any<Sensor>(), any())
//        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = GravitySensorCollector(contextSpy)
        assertFalse(collector.start())

        verify(contextSpy, only()).getSystemService(Context.SENSOR_SERVICE)
//        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(sensorManagerSpy, never()).registerListener(
            any(),
            any<Sensor>(),
            eq(collector.sensorDelay.value)
        )
/*        verify(exactly = 0) {
            sensorManagerSpy.registerListener(
                any(),
                any<Sensor>(),
                collector.sensorDelay.value
            )
        }*/
    }

    @Test
    fun start_whenSensorManager_registersListener() {
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_GRAVITY)
//        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(sensor)
        doReturn(true).whenever(sensorManagerSpy).registerListener(any(), any<Sensor>(), any())
//        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = GravitySensorCollector(contextSpy)
        assertTrue(collector.start())

        verify(contextSpy, only()).getSystemService(Context.SENSOR_SERVICE)
//        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(sensorManagerSpy, times(1)).registerListener(
            capture(sensorEventListenerCaptor),
            eq(sensor),
            eq(collector.sensorDelay.value)
        )
/*        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(
                capture(slot),
                sensor,
                collector.sensorDelay.value
            )
        }*/

        val eventListener = sensorEventListenerCaptor.value
//        val eventListener = slot.captured
        assertNotNull(eventListener)
    }

    @Test
    fun stop_whenNoSensorManager_makesNoAction() {
        doReturn(null).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = GravitySensorCollector(contextSpy)
        collector.stop()
    }

    @Test
    fun stop_whenSensorManager_unregistersListener() {
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_GRAVITY)
//        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(sensor)
        doNothing().whenever(sensorManagerSpy).unregisterListener(any(), any<Sensor>())
//        justRun { sensorManagerSpy.unregisterListener(any(), any<Sensor>()) }
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = GravitySensorCollector(contextSpy)
        collector.stop()

        verify(contextSpy, only()).getSystemService(Context.SENSOR_SERVICE)
//        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(sensorManagerSpy, times(1)).unregisterListener(capture(sensorEventListenerCaptor), eq(sensor))
//        val slot = slot<SensorEventListener>()
//        verify(exactly = 1) { sensorManagerSpy.unregisterListener(capture(slot), sensor) }

        val eventListener = sensorEventListenerCaptor.value
//        val eventListener = slot.captured
        assertNotNull(eventListener)
    }

    @Test
    fun onSensorChanged_whenNoEvent_makesNoAction() {
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_GRAVITY)
//        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(sensor)
        doReturn(true).whenever(sensorManagerSpy).registerListener(any(), any<Sensor>(), any())
//        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector =
            GravitySensorCollector(contextSpy, measurementListener = measurementListener)
        assertTrue(collector.start())

        verify(sensorManagerSpy, times(1)).registerListener(
            capture(sensorEventListenerCaptor),
            eq(sensor),
            eq(collector.sensorDelay.value)
        )
/*        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(
                capture(slot),
                sensor,
                collector.sensorDelay.value
            )
        }*/

        val eventListener = sensorEventListenerCaptor.value
//        val eventListener = slot.captured

        eventListener.onSensorChanged(null)
        verifyNoInteractions(measurementListener)
//        verify { measurementListener wasNot Called }
    }

    @Test
    fun onSensorChanged_whenNoSensorType_makesNoAction() {
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_GRAVITY)
//        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(sensor)
        doReturn(true).whenever(sensorManagerSpy).registerListener(any(), any<Sensor>(), any())
//        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector =
            GravitySensorCollector(contextSpy, measurementListener = measurementListener)
        assertTrue(collector.start())

        verify(sensorManagerSpy, times(1)).registerListener(
            capture(sensorEventListenerCaptor),
            eq(sensor),
            eq(collector.sensorDelay.value)
        )
/*        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(
                capture(slot),
                sensor,
                collector.sensorDelay.value
            )
        }*/

        val eventListener = sensorEventListenerCaptor.value
//        val eventListener = slot.captured

        whenever(sensor.type).thenReturn(-1)
//        every { sensor.type }.returns(-1)
        event.sensor = sensor
        eventListener.onSensorChanged(event)

        verifyNoInteractions(measurementListener)
//        verify { measurementListener wasNot Called }
    }

    @Test
    fun onSensorChanged_whenNoListener_doesNotNotify() {
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_GRAVITY)
//        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(sensor)
        doReturn(true).whenever(sensorManagerSpy).registerListener(any(), any<Sensor>(), any())
//        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = GravitySensorCollector(contextSpy)
        assertTrue(collector.start())

        verify(sensorManagerSpy, times(1)).registerListener(
            capture(sensorEventListenerCaptor),
            eq(sensor),
            eq(collector.sensorDelay.value)
        )
/*        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(
                capture(slot),
                sensor,
                collector.sensorDelay.value
            )
        }*/

        val eventListener = sensorEventListenerCaptor.value
//        val eventListener = slot.captured

        whenever(sensor.type).thenReturn(Sensor.TYPE_GRAVITY)
//        every { sensor.type }.returns(Sensor.TYPE_GRAVITY)
        event.sensor = sensor
        event.timestamp = System.nanoTime()
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, floatArrayOf(1.0f, 2.0f, 3.0f))
        eventListener.onSensorChanged(event)

        verify(sensor, only()).type
//        verify(exactly = 1) { sensor.type }
    }

    @Test
    fun onSensorChanged_whenListener_notifiesEvent() {
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_GRAVITY)
//        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(sensor)
        doReturn(true).whenever(sensorManagerSpy).registerListener(any(), any<Sensor>(), any())
//        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector =
            GravitySensorCollector(contextSpy, measurementListener = measurementListener)
        assertTrue(collector.start())

        verify(sensorManagerSpy, times(1)).registerListener(
            capture(sensorEventListenerCaptor),
            eq(sensor),
            eq(collector.sensorDelay.value)
        )
/*        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(
                capture(slot),
                sensor,
                collector.sensorDelay.value
            )
        }*/

        val eventListener = sensorEventListenerCaptor.value
//        val eventListener = slot.captured

        whenever(sensor.type).thenReturn(Sensor.TYPE_GRAVITY)
//        every { sensor.type }.returns(Sensor.TYPE_GRAVITY)
        event.sensor = sensor
        event.timestamp = System.nanoTime()
        event.accuracy = SensorAccuracy.HIGH.value
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, floatArrayOf(1.0f, 2.0f, 3.0f))
        eventListener.onSensorChanged(event)

        verify(measurementListener, only()).onMeasurement(
            1.0f,
            2.0f,
            3.0f,
            sqrt(1.0 + 4.0 + 9.0),
            event.timestamp,
            SensorAccuracy.HIGH
        )
/*        verify(exactly = 1) {
            measurementListener.onMeasurement(
                1.0f,
                2.0f,
                3.0f,
                sqrt(1.0 + 4.0 + 9.0),
                event.timestamp,
                SensorAccuracy.HIGH
            )
        }*/
    }

    @Test
    fun onAccuracyChanged_whenNoSensor_makesNoAction() {
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_GRAVITY)
//        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(sensor)
        doReturn(true).whenever(sensorManagerSpy).registerListener(any(), any<Sensor>(), any())
//        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = GravitySensorCollector(
            contextSpy,
            accuracyChangedListener = accuracyChangedListener
        )
        assertTrue(collector.start())

        verify(sensorManagerSpy, times(1)).registerListener(
            capture(sensorEventListenerCaptor),
            eq(sensor),
            eq(collector.sensorDelay.value)
        )
/*        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(
                capture(slot),
                sensor,
                collector.sensorDelay.value
            )
        }*/

        val eventListener = sensorEventListenerCaptor.value
//        val eventListener = slot.captured

        eventListener.onAccuracyChanged(null, SensorAccuracy.HIGH.value)

        verifyNoInteractions(accuracyChangedListener)
//        verify { accuracyChangedListener wasNot Called }
    }

    @Test
    fun onAccuracyChanged_whenNoSensorType_makesNoAction() {
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_GRAVITY)
//        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(sensor)
        doReturn(true).whenever(sensorManagerSpy).registerListener(any(), any<Sensor>(), any())
//        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = GravitySensorCollector(
            contextSpy,
            accuracyChangedListener = accuracyChangedListener
        )
        assertTrue(collector.start())

        verify(sensorManagerSpy, times(1)).registerListener(
            capture(sensorEventListenerCaptor),
            eq(sensor),
            eq(collector.sensorDelay.value)
        )
/*        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(
                capture(slot),
                sensor,
                collector.sensorDelay.value
            )
        }*/

        val eventListener = sensorEventListenerCaptor.value
//        val eventListener = slot.captured

        whenever(sensor.type).thenReturn(-1)
//        every { sensor.type }.returns(-1)
        eventListener.onAccuracyChanged(sensor, SensorAccuracy.HIGH.value)

        verifyNoInteractions(accuracyChangedListener)
//        verify { accuracyChangedListener wasNot Called }
    }

    @Test
    fun onAccuracyChanged_whenListener_notifiesEvent() {
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_GRAVITY)
//        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GRAVITY) }.returns(sensor)
        doReturn(true).whenever(sensorManagerSpy).registerListener(any(), any<Sensor>(), any())
//        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = GravitySensorCollector(
            contextSpy,
            accuracyChangedListener = accuracyChangedListener
        )
        assertTrue(collector.start())

        verify(sensorManagerSpy, times(1)).registerListener(
            capture(sensorEventListenerCaptor),
            eq(sensor),
            eq(collector.sensorDelay.value)
        )
/*        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(
                capture(slot),
                sensor,
                collector.sensorDelay.value
            )
        }*/

        val eventListener = sensorEventListenerCaptor.value
//        val eventListener = slot.captured

        whenever(sensor.type).thenReturn(Sensor.TYPE_GRAVITY)
//        every { sensor.type }.returns(Sensor.TYPE_GRAVITY)
        eventListener.onAccuracyChanged(sensor, SensorAccuracy.HIGH.value)

        verify(accuracyChangedListener, only()).onAccuracyChanged(SensorAccuracy.HIGH)
/*        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(SensorAccuracy.HIGH)
        }*/
    }
}