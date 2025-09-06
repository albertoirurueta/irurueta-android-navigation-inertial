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

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import androidx.test.core.app.ApplicationProvider
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.android.testutils.setPrivateProperty
import com.irurueta.geometry.Quaternion
import com.irurueta.statistics.UniformRandomizer
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
class AttitudeSensorCollector2Test {

    @get:Rule
    val mockitoRule: MockitoRule = MockitoJUnit.rule()

//    @get:Rule
//    val mockkRule = MockKRule(this)

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var accuracyChangedListener:
            SensorCollector2.OnAccuracyChangedListener<AttitudeSensorMeasurement, AttitudeSensorCollector2>

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var measurementListener:
            SensorCollector2.OnMeasurementListener<AttitudeSensorMeasurement, AttitudeSensorCollector2>

//    @MockK
    @Mock
    private lateinit var sensor: Sensor

//    @MockK
    @Mock
    private lateinit var event: SensorEvent

    @Captor
    private lateinit var sensorEventListenerCaptor: ArgumentCaptor<SensorEventListener>

    @Captor
    private lateinit var attitudeSensorMeasurementCaptor: ArgumentCaptor<AttitudeSensorMeasurement>

    /*@After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
        System.gc()
    }*/

    @Test
    fun constructor_whenRequiredParameters_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = AttitudeSensorCollector2(context)

        // check values
        assertSame(context, collector.context)
        assertEquals(AttitudeSensorType.ABSOLUTE_ATTITUDE, collector.sensorType)
        assertEquals(SensorDelay.FASTEST, collector.sensorDelay)
        assertTrue(collector.startOffsetEnabled)
        assertNull(collector.accuracyChangedListener)
        assertNull(collector.measurementListener)
        assertNull(collector.sensor)
        assertFalse(collector.sensorAvailable)
        assertEquals(0L, collector.startTimestamp)
        assertNull(collector.startOffset)
        assertFalse(collector.running)
        assertEquals(0, collector.numberOfProcessedMeasurements)
        assertEquals(0L, collector.mostRecentTimestamp)
    }

    @Test
    fun constructor_whenAllParameters_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = AttitudeSensorCollector2(
            context,
            AttitudeSensorType.RELATIVE_ATTITUDE,
            SensorDelay.NORMAL,
            startOffsetEnabled = false,
            accuracyChangedListener,
            measurementListener
        )

        // check values
        assertSame(context, collector.context)
        assertEquals(AttitudeSensorType.RELATIVE_ATTITUDE, collector.sensorType)
        assertEquals(SensorDelay.NORMAL, collector.sensorDelay)
        assertFalse(collector.startOffsetEnabled)
        assertSame(accuracyChangedListener, collector.accuracyChangedListener)
        assertSame(measurementListener, collector.measurementListener)
        assertNull(collector.sensor)
        assertFalse(collector.sensorAvailable)
        assertEquals(0L, collector.startTimestamp)
        assertNull(collector.startOffset)
        assertFalse(collector.running)
        assertEquals(0, collector.numberOfProcessedMeasurements)
        assertEquals(0L, collector.mostRecentTimestamp)
    }

    @Test
    fun accuracyChangedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = AttitudeSensorCollector2(context)

        // check default value
        assertNull(collector.accuracyChangedListener)

        // set new value
        collector.accuracyChangedListener = accuracyChangedListener

        // check
        assertSame(accuracyChangedListener, collector.accuracyChangedListener)
    }

    @Test
    fun measurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = AttitudeSensorCollector2(context)

        // check default value
        assertNull(collector.measurementListener)

        // set new value
        collector.measurementListener = measurementListener

        // check
        assertSame(measurementListener, collector.measurementListener)
    }

    @Test
    fun sensor_whenSensorTypeAbsoluteAttitude_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spy(sensorManager)
//        val sensorManagerSpy = spyk(sensorManager)
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)
//        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) }.returns(sensor)
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = AttitudeSensorCollector2(contextSpy, AttitudeSensorType.ABSOLUTE_ATTITUDE)

        assertSame(sensor, collector.sensor)
        verify(contextSpy, only()).getSystemService(Context.SENSOR_SERVICE)
//        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(sensorManagerSpy, only()).getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)
//        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) }
    }

    @Test
    fun sensor_whenSensorTypeRelativeAttitude_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spy(sensorManager)
//        val sensorManagerSpy = spyk(sensorManager)
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR)
/*        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR) }.returns(
            sensor
        )*/
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = AttitudeSensorCollector2(contextSpy, AttitudeSensorType.RELATIVE_ATTITUDE)

        assertSame(sensor, collector.sensor)
        verify(contextSpy, only()).getSystemService(Context.SENSOR_SERVICE)
//        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(sensorManagerSpy, only()).getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR)
//        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR) }
    }

    @Test
    fun sensorAvailable_whenSensorTypeAbsoluteAttitude_returnsExpectedValue() {
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

        val collector = AttitudeSensorCollector2(contextSpy, AttitudeSensorType.ABSOLUTE_ATTITUDE)

        assertFalse(collector.sensorAvailable)
        verify(contextSpy, only()).getSystemService(Context.SENSOR_SERVICE)
//        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(sensorManagerSpy, times(1)).getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)
//        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) }
    }

    @Test
    fun sensorAvailable_whenSensorTypeRelativeAttitude_returnsExpectedValue() {
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

        val collector = AttitudeSensorCollector2(contextSpy, AttitudeSensorType.RELATIVE_ATTITUDE)

        assertFalse(collector.sensorAvailable)
        verify(contextSpy, only()).getSystemService(Context.SENSOR_SERVICE)
//        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(sensorManagerSpy, times(1)).getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR)
//        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR) }
    }

    @Test
    fun start_whenNoSensorManager_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(null).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = AttitudeSensorCollector2(contextSpy)
        assertEquals(0L, collector.startTimestamp)
        assertFalse(collector.start())

        verify(contextSpy, only()).getSystemService(Context.SENSOR_SERVICE)
//        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }

        assertFalse(collector.running)
        assertEquals(0L, collector.startTimestamp)
    }

    @Test
    fun start_whenNoSensor_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spy(sensorManager)
//        val sensorManagerSpy = spyk(sensorManager)
        doReturn(null).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)
//        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) }.returns(null)
        doReturn(true).whenever(sensorManagerSpy).registerListener(any(), any<Sensor>(), any())
//        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = AttitudeSensorCollector2(contextSpy)
        assertEquals(0L, collector.startTimestamp)
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

        assertFalse(collector.running)
        assertEquals(0L, collector.startTimestamp)
    }

    @Test
    fun start_whenRegisterListenerFails_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spy(sensorManager)
//        val sensorManagerSpy = spyk(sensorManager)
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)
//        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) }.returns(sensor)
        doReturn(false).whenever(sensorManagerSpy).registerListener(any(), any<Sensor>(), any())
//        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(false)
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = AttitudeSensorCollector2(contextSpy)
        assertEquals(0L, collector.startTimestamp)
        assertFalse(collector.start())

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

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SensorCollector2::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)
        assertSame(sensorEventListener, eventListener)

        assertFalse(collector.running)
        assertNotEquals(0L, collector.startTimestamp)
    }

    @Test
    fun start_whenRegisterListenerSucceeds_registersListener() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spy(sensorManager)
//        val sensorManagerSpy = spyk(sensorManager)
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)
//        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) }.returns(sensor)
        doReturn(true).whenever(sensorManagerSpy).registerListener(any(), any<Sensor>(), any())
//        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = AttitudeSensorCollector2(contextSpy)
        assertEquals(0L, collector.startTimestamp)
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

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SensorCollector2::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)
        assertSame(sensorEventListener, eventListener)

        assertTrue(collector.running)
        assertNotEquals(0L, collector.startTimestamp)
    }

    @Test
    fun start_whenStartTimestampProvided_registersListenerAndSetsStartTimestamp() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spy(sensorManager)
//        val sensorManagerSpy = spyk(sensorManager)
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)
//        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) }.returns(sensor)
        doReturn(true).whenever(sensorManagerSpy).registerListener(any(), any<Sensor>(), any())
//        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = AttitudeSensorCollector2(contextSpy)
        assertEquals(0L, collector.startTimestamp)
        val startTimestamp = System.nanoTime()
        assertTrue(collector.start(startTimestamp))

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

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SensorCollector2::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)
        assertSame(sensorEventListener, eventListener)

        assertTrue(collector.running)
        assertEquals(startTimestamp, collector.startTimestamp)
    }

    @Test
    fun stop_whenNoSensorManager_resetsParameters() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(null).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = AttitudeSensorCollector2(contextSpy)

        // set initial values
        val randomizer = UniformRandomizer()
        val startOffset1 = randomizer.nextLong()
        setPrivateProperty(SensorCollector2::class, collector, "startOffset", startOffset1)

        val numberOfProcessedMeasurements1 = randomizer.nextInt()
        setPrivateProperty(
            SensorCollector2::class,
            collector,
            "numberOfProcessedMeasurements",
            numberOfProcessedMeasurements1
        )

        setPrivateProperty(SensorCollector2::class, collector, "running", true)

        collector.stop()

        val startOffset2: Long? =
            getPrivateProperty(SensorCollector2::class, collector, "startOffset")
        assertNull(startOffset2)

        val numberOfProcessedMeasurements2: Int? =
            getPrivateProperty(SensorCollector2::class, collector, "numberOfProcessedMeasurements")
        assertEquals(0, numberOfProcessedMeasurements2)

        assertFalse(collector.running)
    }

    @Test
    fun stop_whenSensorManager_unregistersListenerAndResetsParameters() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spy(sensorManager)
//        val sensorManagerSpy = spyk(sensorManager)
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)
//        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) }.returns(sensor)
        doNothing().whenever(sensorManagerSpy).unregisterListener(any(), any<Sensor>())
//        justRun { sensorManagerSpy.unregisterListener(any(), any<Sensor>()) }
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = AttitudeSensorCollector2(contextSpy)

        // set initial values
        val randomizer = UniformRandomizer()
        val startOffset1 = randomizer.nextLong()
        setPrivateProperty(SensorCollector2::class, collector, "startOffset", startOffset1)

        val numberOfProcessedMeasurements1 = randomizer.nextInt()
        setPrivateProperty(
            SensorCollector2::class,
            collector,
            "numberOfProcessedMeasurements",
            numberOfProcessedMeasurements1
        )

        setPrivateProperty(SensorCollector2::class, collector, "running", true)

        collector.stop()

        // check that values have been reset
        val startOffset2: Long? =
            getPrivateProperty(SensorCollector2::class, collector, "startOffset")
        assertNull(startOffset2)

        val numberOfProcessedMeasurements2: Int? =
            getPrivateProperty(SensorCollector2::class, collector, "numberOfProcessedMeasurements")
        assertEquals(0, numberOfProcessedMeasurements2)

        assertFalse(collector.running)

        verify(contextSpy, only()).getSystemService(Context.SENSOR_SERVICE)
//        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(sensorManagerSpy, times(1)).unregisterListener(capture(sensorEventListenerCaptor), eq(sensor))
/*        val slot = slot<SensorEventListener>()
        verify(exactly = 1) { sensorManagerSpy.unregisterListener(capture(slot), sensor) }*/

        val eventListener = sensorEventListenerCaptor.value
//        val eventListener = slot.captured

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SensorCollector2::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)
        assertSame(sensorEventListener, eventListener)
    }

    @Test
    fun onSensorChanged_whenNoEvent_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = AttitudeSensorCollector2(context, measurementListener = measurementListener)

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SensorCollector2::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onSensorChanged(null)

        // check
        verifyNoInteractions(measurementListener)
//        verify { measurementListener wasNot Called }

        assertNull(collector.startOffset)
        assertEquals(0, collector.numberOfProcessedMeasurements)
        assertEquals(0L, collector.mostRecentTimestamp)
    }

    @Test
    fun onSensorChanged_whenNoSensorType_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spy(sensorManager)
//        val sensorManagerSpy = spyk(sensorManager)
        whenever(sensor.type).thenReturn(Sensor.TYPE_GYROSCOPE)
//        every { sensor.type }.returns(Sensor.TYPE_GYROSCOPE)
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)
//        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) }.returns(sensor)
        doReturn(true).whenever(sensorManagerSpy).registerListener(any(), any<Sensor>(), any())
//        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = AttitudeSensorCollector2(
            contextSpy,
            startOffsetEnabled = false,
            measurementListener = measurementListener
        )

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SensorCollector2::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        event.sensor = sensor
        sensorEventListener.onSensorChanged(event)

        // check
        verifyNoInteractions(measurementListener)
//        verify { measurementListener wasNot Called }

        assertNull(collector.startOffset)
        assertEquals(0, collector.numberOfProcessedMeasurements)
        assertEquals(0L, collector.mostRecentTimestamp)
    }

    @Test
    fun onSensorChanged_whenStartOffsetEnabled_setsStartOffset() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spy(sensorManager)
//        val sensorManagerSpy = spyk(sensorManager)
        whenever(sensor.type).thenReturn(Sensor.TYPE_ROTATION_VECTOR)
//        every { sensor.type }.returns(Sensor.TYPE_ROTATION_VECTOR)
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)
//        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) }.returns(sensor)
        doReturn(true).whenever(sensorManagerSpy).registerListener(any(), any<Sensor>(), any())
//        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = AttitudeSensorCollector2(
            contextSpy,
            startOffsetEnabled = true,
            measurementListener = measurementListener
        )

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SensorCollector2::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        // set start time
        val startTimestamp = System.nanoTime()
        setPrivateProperty(SensorCollector2::class, collector, "startTimestamp", startTimestamp)

        assertNull(collector.startOffset)

        event.sensor = sensor
        event.timestamp = System.nanoTime()
        event.accuracy = SensorManager.SENSOR_STATUS_ACCURACY_HIGH
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, floatArrayOf(1.0f, 2.0f, 3.0f, 4.0f, 5.0f))
        sensorEventListener.onSensorChanged(event)

        // check
        val startOffset = event.timestamp - startTimestamp
        assertNotNull(collector.startOffset)
        assertEquals(startOffset, collector.startOffset)
        verify(measurementListener, only()).onMeasurement(eq(collector), capture(attitudeSensorMeasurementCaptor))
/*        val slot = slot<AttitudeSensorMeasurement>()
        verify(exactly = 1) { measurementListener.onMeasurement(collector, capture(slot)) }*/

        val measurement = attitudeSensorMeasurementCaptor.value
//        val measurement = slot.captured
        assertTrue(
            measurement.attitude.equals(
                Quaternion(doubleArrayOf(4.0, 1.0, 2.0, 3.0)),
                ABSOLUTE_ERROR
            )
        )
        val headingAccuracy = measurement.headingAccuracy
        requireNotNull(headingAccuracy)
        assertEquals(5.0f, headingAccuracy, 0.0f)
        assertEquals(event.timestamp + startOffset, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
        assertEquals(AttitudeSensorType.ABSOLUTE_ATTITUDE, measurement.sensorType)

        assertEquals(1, collector.numberOfProcessedMeasurements)
        assertEquals(measurement.timestamp, collector.mostRecentTimestamp)
    }

    @Test
    fun onAccuracyChanged_whenNoSensor_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector =
            AttitudeSensorCollector2(context, accuracyChangedListener = accuracyChangedListener)

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SensorCollector2::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onAccuracyChanged(null, SensorAccuracy.HIGH.value)

        verifyNoInteractions(accuracyChangedListener)
//        verify { accuracyChangedListener wasNot Called }
    }

    @Test
    fun onAccuracyChanged_whenUnsupportedSensorType_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector =
            AttitudeSensorCollector2(context, accuracyChangedListener = accuracyChangedListener)

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SensorCollector2::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        whenever(sensor.type).thenReturn(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value)
//        every { sensor.type }.returns(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value)
        sensorEventListener.onAccuracyChanged(sensor, SensorAccuracy.HIGH.value)

        verifyNoInteractions(accuracyChangedListener)
//        verify { accuracyChangedListener wasNot Called }
    }

    @Test
    fun onAccuracyChanged_whenNoListener_executes() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = AttitudeSensorCollector2(context)

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SensorCollector2::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        whenever(sensor.type).thenReturn(AttitudeSensorType.ABSOLUTE_ATTITUDE.value)
//        every { sensor.type }.returns(AttitudeSensorType.ABSOLUTE_ATTITUDE.value)
        sensorEventListener.onAccuracyChanged(sensor, SensorAccuracy.HIGH.value)
    }

    @Test
    fun onAccuracyChanged_whenListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = AttitudeSensorCollector2(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SensorCollector2::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        whenever(sensor.type).thenReturn(AttitudeSensorType.ABSOLUTE_ATTITUDE.value)
//        every { sensor.type }.returns(AttitudeSensorType.ABSOLUTE_ATTITUDE.value)
        sensorEventListener.onAccuracyChanged(sensor, SensorAccuracy.HIGH.value)

        verify(accuracyChangedListener, only()).onAccuracyChanged(
            collector,
            SensorAccuracy.HIGH
        )
/*        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                collector,
                SensorAccuracy.HIGH
            )
        }*/
    }

    private companion object {
        const val ABSOLUTE_ERROR = 1e-6
    }
}