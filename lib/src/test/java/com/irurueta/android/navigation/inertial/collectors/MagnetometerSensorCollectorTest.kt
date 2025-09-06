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
//import io.mockk.junit4.MockKRule
//import org.junit.After
import org.junit.Assert.*
//import org.junit.Ignore
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith
import org.mockito.ArgumentCaptor
import org.mockito.Captor
import org.mockito.Mock
import org.mockito.junit.MockitoJUnit
import org.mockito.junit.MockitoRule
import org.mockito.kotlin.any
import org.mockito.kotlin.capture
import org.mockito.kotlin.doNothing
import org.mockito.kotlin.doReturn
import org.mockito.kotlin.eq
import org.mockito.kotlin.never
import org.mockito.kotlin.only
import org.mockito.kotlin.spy
import org.mockito.kotlin.times
import org.mockito.kotlin.verify
import org.mockito.kotlin.verifyNoInteractions
import org.mockito.kotlin.whenever
import org.robolectric.RobolectricTestRunner

//@Ignore("Possible memory leak when running this test")
@RunWith(RobolectricTestRunner::class)
class MagnetometerSensorCollectorTest {

    @get:Rule
    val mockitoRule: MockitoRule = MockitoJUnit.rule()

//    @get:Rule
//    val mockkRule = MockKRule(this)

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var measurementListener: MagnetometerSensorCollector.OnMeasurementListener

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var accuracyChangedListener: SensorCollector.OnAccuracyChangedListener

//    @MockK
    @Mock
    private lateinit var sensor: Sensor

//    @MockK
    @Mock
    private lateinit var event: SensorEvent

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
        val collector = MagnetometerSensorCollector(context)

        // check values
        assertSame(context, collector.context)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            collector.sensorType
        )
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
    fun constructor_whenSensorType_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = MagnetometerSensorCollector(
            context,
            MagnetometerSensorType.MAGNETOMETER
        )

        // check values
        assertSame(context, collector.context)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER,
            collector.sensorType
        )
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
    fun constructor_whenSensorDelay_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = MagnetometerSensorCollector(
            context,
            MagnetometerSensorType.MAGNETOMETER,
            SensorDelay.NORMAL
        )

        // check values
        assertSame(context, collector.context)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER,
            collector.sensorType
        )
        assertEquals(SensorDelay.NORMAL, collector.sensorDelay)
        assertNull(collector.measurementListener)
        assertNull(collector.accuracyChangedListener)
        assertNull(collector.sensor)
        assertFalse(collector.sensorAvailable)

        val sensorManager: SensorManager? =
            getPrivateProperty(SensorCollector::class, collector, "sensorManager")
        assertNotNull(sensorManager)
    }

    @Test
    fun constructor_whenMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = MagnetometerSensorCollector(
            context,
            MagnetometerSensorType.MAGNETOMETER,
            SensorDelay.NORMAL,
            measurementListener
        )

        // check values
        assertSame(context, collector.context)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER,
            collector.sensorType
        )
        assertEquals(SensorDelay.NORMAL, collector.sensorDelay)
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
        val collector = MagnetometerSensorCollector(
            context,
            MagnetometerSensorType.MAGNETOMETER,
            SensorDelay.NORMAL,
            measurementListener,
            accuracyChangedListener
        )

        // check values
        assertSame(context, collector.context)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER,
            collector.sensorType
        )
        assertEquals(SensorDelay.NORMAL, collector.sensorDelay)
        assertSame(measurementListener, collector.measurementListener)
        assertSame(accuracyChangedListener, collector.accuracyChangedListener)
        assertNull(collector.sensor)
        assertFalse(collector.sensorAvailable)

        val sensorManager: SensorManager? =
            getPrivateProperty(SensorCollector::class, collector, "sensorManager")
        assertNotNull(sensorManager)
    }

    @Test
    fun measurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = MagnetometerSensorCollector(context)

        assertNull(collector.measurementListener)

        // set new value
        collector.measurementListener = measurementListener

        // check
        assertSame(measurementListener, collector.measurementListener)
    }

    @Test
    fun accuracyChangedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = MagnetometerSensorCollector(context)

        assertNull(collector.accuracyChangedListener)

        // set new value
        collector.accuracyChangedListener = accuracyChangedListener

        // check
        assertSame(accuracyChangedListener, collector.accuracyChangedListener)
    }

    @Test
    fun sensor_whenSensorTypeMagnetometer_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spy(sensorManager)
//        val sensorManagerSpy = spyk(sensorManager)
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)
//        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD) }.returns(sensor)
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = MagnetometerSensorCollector(
            contextSpy,
            MagnetometerSensorType.MAGNETOMETER
        )

        assertSame(sensor, collector.sensor)
        verify(contextSpy, only()).getSystemService(Context.SENSOR_SERVICE)
//        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(sensorManagerSpy, only()).getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)
//        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD) }
    }

    @Test
    fun sensor_whenSensorTypeMagnetometerUncalibrated_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spy(sensorManager)
//        val sensorManagerSpy = spyk(sensorManager)
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED)
/*        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED) }.returns(
            sensor
        )*/
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = MagnetometerSensorCollector(
            contextSpy,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED
        )

        assertSame(sensor, collector.sensor)
        verify(contextSpy, only()).getSystemService(Context.SENSOR_SERVICE)
//        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(sensorManagerSpy, only()).getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED)
/*        verify(exactly = 1) {
            sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED)
        }*/
    }

    @Test
    fun sensorAvailable_whenSensorTypeMagnetometer_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spy(sensorManager)
//        val sensorManagerSpy = spyk(sensorManager)
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = MagnetometerSensorCollector(
            contextSpy,
            MagnetometerSensorType.MAGNETOMETER
        )

        assertFalse(collector.sensorAvailable)
        verify(contextSpy, only()).getSystemService(Context.SENSOR_SERVICE)
//        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(sensorManagerSpy, times(1)).getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)
//        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD) }
    }

    @Test
    fun sensorAvailable_whenSensorTypeMagnetometerUncalibrated_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spy(sensorManager)
//        val sensorManagerSpy = spyk(sensorManager)
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED)
/*        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED) }
            .returns(sensor)*/
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = MagnetometerSensorCollector(
            contextSpy,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED
        )

        assertTrue(collector.sensorAvailable)
        verify(contextSpy, only()).getSystemService(Context.SENSOR_SERVICE)
//        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(sensorManagerSpy, only()).getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED)
/*        verify(exactly = 1) {
            sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED)
        }*/
    }

    @Test
    fun start_whenNoSensorManager_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(null).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = MagnetometerSensorCollector(contextSpy)
        assertFalse(collector.start())

        verify(contextSpy, only()).getSystemService(Context.SENSOR_SERVICE)
//        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
    }

    @Test
    fun start_whenNoSensor_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spy(sensorManager)
//        val sensorManagerSpy = spyk(sensorManager)
        doReturn(null).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD)
//        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD) }.returns(null)
        doReturn(true).whenever(sensorManagerSpy).registerListener(any(), any<Sensor>(), any())
//        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = MagnetometerSensorCollector(contextSpy)
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spy(sensorManager)
//        val sensorManagerSpy = spyk(sensorManager)
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED)
/*        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED) }.returns(
            sensor
        )*/
        doReturn(true).whenever(sensorManagerSpy).registerListener(any(), any<Sensor>(), any())
//        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = MagnetometerSensorCollector(contextSpy)
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(null).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = MagnetometerSensorCollector(contextSpy)
        collector.stop()
    }

    @Test
    fun stop_whenSensorManager_unregistersListener() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spy(sensorManager)
//        val sensorManagerSpy = spyk(sensorManager)
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED)
/*        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED) }.returns(
            sensor
        )*/
        doNothing().whenever(sensorManagerSpy).unregisterListener(any(), any<Sensor>())
//        justRun { sensorManagerSpy.unregisterListener(any(), any<Sensor>()) }
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = MagnetometerSensorCollector(contextSpy)
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spy(sensorManager)
//        val sensorManagerSpy = spyk(sensorManager)
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED)
/*        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED) }.returns(
            sensor
        )*/
        doReturn(true).whenever(sensorManagerSpy).registerListener(any(), any<Sensor>(), any())
//        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = MagnetometerSensorCollector(
            contextSpy,
            measurementListener = measurementListener
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

        eventListener.onSensorChanged(null)
        verifyNoInteractions(measurementListener)
//        verify { measurementListener wasNot Called }
    }

    @Test
    fun onSensorChanged_whenNoSensorType_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spy(sensorManager)
//        val sensorManagerSpy = spyk(sensorManager)
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED)
/*        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED) }.returns(
            sensor
        )*/
        doReturn(true).whenever(sensorManagerSpy).registerListener(any(), any<Sensor>(), any())
//        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = MagnetometerSensorCollector(
            contextSpy,
            measurementListener = measurementListener
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
        event.sensor = sensor
        eventListener.onSensorChanged(event)

        verifyNoInteractions(measurementListener)
//        verify { measurementListener wasNot Called }
    }

    @Test
    fun onSensorChanged_whenNoListener_doesNotNotify() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spy(sensorManager)
//        val sensorManagerSpy = spyk(sensorManager)
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED)
/*        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED) }.returns(
            sensor
        )*/
        doReturn(true).whenever(sensorManagerSpy).registerListener(any(), any<Sensor>(), any())
//        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = MagnetometerSensorCollector(contextSpy)
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

        whenever(sensor.type).thenReturn(MagnetometerSensorType.MAGNETOMETER.value)
/*        every { sensor.type }
            .returns(MagnetometerSensorType.MAGNETOMETER.value)*/
        event.sensor = sensor
        event.timestamp = System.nanoTime()
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, floatArrayOf(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f))
        eventListener.onSensorChanged(event)

        verify(sensor, only()).type
//        verify(exactly = 1) { sensor.type }
    }

    @Test
    fun onSensorChanged_whenListenerMagnetometerSensor_notifiesEvent() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spy(sensorManager)
//        val sensorManagerSpy = spyk(sensorManager)
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED)
/*        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED) }.returns(
            sensor
        )*/
        doReturn(true).whenever(sensorManagerSpy).registerListener(any(), any<Sensor>(), any())
//        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = MagnetometerSensorCollector(
            contextSpy,
            measurementListener = measurementListener
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

        whenever(sensor.type).thenReturn(MagnetometerSensorType.MAGNETOMETER.value)
/*        every { sensor.type }
            .returns(MagnetometerSensorType.MAGNETOMETER.value)*/
        event.sensor = sensor
        event.timestamp = System.nanoTime()
        event.accuracy = SensorAccuracy.HIGH.value
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, floatArrayOf(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f))
        eventListener.onSensorChanged(event)

        verify(measurementListener, only()).onMeasurement(
            1.0f,
            2.0f,
            3.0f,
            null,
            null,
            null,
            event.timestamp,
            SensorAccuracy.HIGH
        )
/*        verify(exactly = 1) {
            measurementListener.onMeasurement(
                1.0f,
                2.0f,
                3.0f,
                null,
                null,
                null,
                event.timestamp,
                SensorAccuracy.HIGH
            )
        }*/
    }

    @Test
    fun onSensorChanged_whenListenerMagnetometerUncalibratedSensor_notifiesEvent() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spy(sensorManager)
//        val sensorManagerSpy = spyk(sensorManager)
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED)
/*        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED) }
            .returns(sensor)*/
        doReturn(true).whenever(sensorManagerSpy).registerListener(any(), any<Sensor>(), any())
//        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = MagnetometerSensorCollector(
            contextSpy,
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            measurementListener = measurementListener
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

        whenever(sensor.type).thenReturn(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value)
/*        every { sensor.type }
            .returns(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value)*/
        event.sensor = sensor
        event.timestamp = System.nanoTime()
        event.accuracy = SensorAccuracy.HIGH.value
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, floatArrayOf(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f))
        eventListener.onSensorChanged(event)

        verify(measurementListener, only()).onMeasurement(
            1.0f,
            2.0f,
            3.0f,
            4.0f,
            5.0f,
            6.0f,
            event.timestamp,
            SensorAccuracy.HIGH
        )
/*        verify(exactly = 1) {
            measurementListener.onMeasurement(
                1.0f,
                2.0f,
                3.0f,
                4.0f,
                5.0f,
                6.0f,
                event.timestamp,
                SensorAccuracy.HIGH
            )
        }*/
    }

    @Test
    fun onAccuracyChanged_whenNoSensor_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spy(sensorManager)
//        val sensorManagerSpy = spyk(sensorManager)
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED)
/*        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED) }.returns(
            sensor
        )*/
        doReturn(true).whenever(sensorManagerSpy).registerListener(any(), any<Sensor>(), any())
//        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spy(context)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        val contextSpy = spyk(context)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = MagnetometerSensorCollector(
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
        verify {
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spy(sensorManager)
//        val sensorManagerSpy = spyk(sensorManager)
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED)
/*        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED) }.returns(
            sensor
        )*/
        doReturn(true).whenever(sensorManagerSpy).registerListener(any(), any<Sensor>(), any())
//        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = MagnetometerSensorCollector(
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spy(sensorManager)
//        val sensorManagerSpy = spyk(sensorManager)
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED)
/*        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED) }.returns(
            sensor
        )*/
        doReturn(true).whenever(sensorManagerSpy).registerListener(any(), any<Sensor>(), any())
//        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = MagnetometerSensorCollector(
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

        whenever(sensor.type).thenReturn(MagnetometerSensorType.MAGNETOMETER.value)
/*        every { sensor.type }
            .returns(MagnetometerSensorType.MAGNETOMETER.value)*/
        eventListener.onAccuracyChanged(sensor, SensorAccuracy.HIGH.value)

        verify(accuracyChangedListener, only()).onAccuracyChanged(
            SensorAccuracy.HIGH
        )
/*        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                SensorAccuracy.HIGH
            )
        }*/
    }

    @Test
    fun magnetometerSensorType_fromValues_returnsExpected() {
        assertEquals(2, MagnetometerSensorType.entries.size)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER,
            MagnetometerSensorType.from(Sensor.TYPE_MAGNETIC_FIELD)
        )
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            MagnetometerSensorType.from(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED)
        )
    }
}