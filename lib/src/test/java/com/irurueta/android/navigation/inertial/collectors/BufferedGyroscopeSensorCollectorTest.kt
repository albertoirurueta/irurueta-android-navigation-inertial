/*
 * Copyright (C) 2022 Alberto Irurueta Carro (alberto@irurueta.com)
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
import java.util.*

//@Ignore("Possible memory leak when running this test")
@RunWith(RobolectricTestRunner::class)
class BufferedGyroscopeSensorCollectorTest {

    @get:Rule
    val mockitoRule: MockitoRule = MockitoJUnit.rule()

//    @get:Rule
//    val mockkRule = MockKRule(this)

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var accuracyChangedListener:
            BufferedSensorCollector.OnAccuracyChangedListener<GyroscopeSensorMeasurement, BufferedGyroscopeSensorCollector>

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var bufferFilledListener:
            BufferedSensorCollector.OnBufferFilledListener<GyroscopeSensorMeasurement, BufferedGyroscopeSensorCollector>

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var measurementListener:
            BufferedSensorCollector.OnMeasurementListener<GyroscopeSensorMeasurement, BufferedGyroscopeSensorCollector>

//    @MockK
    @Mock
    private lateinit var sensor: Sensor

//    @MockK
    @Mock
    private lateinit var event: SensorEvent

    @Captor
    private lateinit var sensorEventListenerCaptor: ArgumentCaptor<SensorEventListener>

    @Captor
    private lateinit var gyroscopeSensorMeasurementCaptor: ArgumentCaptor<GyroscopeSensorMeasurement>

    /*@After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
        System.gc()
    }*/

    @Test
    fun constructor_whenRequiredParameters_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = BufferedGyroscopeSensorCollector(context)

        // check values
        assertSame(context, collector.context)
        assertEquals(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED, collector.sensorType)
        assertEquals(SensorDelay.FASTEST, collector.sensorDelay)
        assertEquals(BufferedSensorCollector.DEFAULT_CAPACITY, collector.capacity)
        assertTrue(collector.startOffsetEnabled)
        assertTrue(collector.stopWhenFilledBuffer)
        assertTrue(collector.skipWhenProcessing)
        assertNull(collector.accuracyChangedListener)
        assertNull(collector.bufferFilledListener)
        assertNull(collector.measurementListener)
        assertNull(collector.sensor)
        assertFalse(collector.sensorAvailable)
        assertEquals(0L, collector.startTimestamp)
        assertNull(collector.startOffset)
        assertTrue(collector.bufferedMeasurements.isEmpty())
        assertEquals(0.0f, collector.usage, 0.0f)
        assertFalse(collector.running)
        assertFalse(collector.processing)
        assertEquals(0, collector.numberOfProcessedMeasurements)
        assertEquals(0L, collector.mostRecentTimestamp)
        assertNull(collector.oldestTimestampInBuffer)

        // check internal fields initialization
        val buffer: ArrayDeque<GyroscopeSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "buffer")
        requireNotNull(buffer)
        assertTrue(buffer.isEmpty())

        val availableMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "availableMeasurements")
        requireNotNull(availableMeasurements)
        assertEquals(BufferedSensorCollector.DEFAULT_CAPACITY, availableMeasurements.size)

        val measurementsBeforeTimestamp: ArrayDeque<GyroscopeSensorMeasurement>? =
            getPrivateProperty(
                BufferedSensorCollector::class,
                collector,
                "measurementsBeforeTimestamp"
            )
        requireNotNull(measurementsBeforeTimestamp)
        assertTrue(measurementsBeforeTimestamp.isEmpty())

        val measurementsBeforePosition: ArrayDeque<GyroscopeSensorMeasurement>? =
            getPrivateProperty(
                BufferedSensorCollector::class,
                collector,
                "measurementsBeforePosition"
            )
        requireNotNull(measurementsBeforePosition)
        assertTrue(measurementsBeforePosition.isEmpty())

        val availableMeasurementsBeforeTimestamp: ArrayList<GyroscopeSensorMeasurement>? =
            getPrivateProperty(
                BufferedSensorCollector::class,
                collector,
                "availableMeasurementsBeforeTimestamp"
            )
        requireNotNull(availableMeasurementsBeforeTimestamp)
        assertEquals(
            BufferedSensorCollector.DEFAULT_CAPACITY,
            availableMeasurementsBeforeTimestamp.size
        )

        val availableMeasurementsBeforePosition: ArrayList<GyroscopeSensorMeasurement>? =
            getPrivateProperty(
                BufferedSensorCollector::class,
                collector,
                "availableMeasurementsBeforePosition"
            )
        requireNotNull(availableMeasurementsBeforePosition)
        assertEquals(
            BufferedSensorCollector.DEFAULT_CAPACITY,
            availableMeasurementsBeforePosition.size
        )
    }

    @Test
    fun constructor_whenAllParameters_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = BufferedGyroscopeSensorCollector(
            context,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            CAPACITY,
            startOffsetEnabled = false,
            stopWhenFilledBuffer = false,
            skipWhenProcessing = false,
            accuracyChangedListener,
            bufferFilledListener,
            measurementListener
        )

        // check values
        assertSame(context, collector.context)
        assertEquals(GyroscopeSensorType.GYROSCOPE, collector.sensorType)
        assertEquals(SensorDelay.NORMAL, collector.sensorDelay)
        assertEquals(CAPACITY, collector.capacity)
        assertFalse(collector.startOffsetEnabled)
        assertFalse(collector.stopWhenFilledBuffer)
        assertFalse(collector.skipWhenProcessing)
        assertSame(accuracyChangedListener, collector.accuracyChangedListener)
        assertSame(bufferFilledListener, collector.bufferFilledListener)
        assertSame(measurementListener, collector.measurementListener)
        assertNull(collector.sensor)
        assertFalse(collector.sensorAvailable)
        assertEquals(0L, collector.startTimestamp)
        assertNull(collector.startOffset)
        assertTrue(collector.bufferedMeasurements.isEmpty())
        assertEquals(0.0f, collector.usage, 0.0f)
        assertFalse(collector.running)
        assertFalse(collector.processing)
        assertEquals(0, collector.numberOfProcessedMeasurements)
        assertEquals(0L, collector.mostRecentTimestamp)
        assertNull(collector.oldestTimestampInBuffer)

        // check internal fields initialization
        val buffer: ArrayDeque<GyroscopeSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "buffer")
        requireNotNull(buffer)
        assertTrue(buffer.isEmpty())

        val availableMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "availableMeasurements")
        requireNotNull(availableMeasurements)
        assertEquals(CAPACITY, availableMeasurements.size)

        val measurementsBeforeTimestamp: ArrayDeque<GyroscopeSensorMeasurement>? =
            getPrivateProperty(
                BufferedSensorCollector::class,
                collector,
                "measurementsBeforeTimestamp"
            )
        requireNotNull(measurementsBeforeTimestamp)
        assertTrue(measurementsBeforeTimestamp.isEmpty())

        val measurementsBeforePosition: ArrayDeque<GyroscopeSensorMeasurement>? =
            getPrivateProperty(
                BufferedSensorCollector::class,
                collector,
                "measurementsBeforePosition"
            )
        requireNotNull(measurementsBeforePosition)
        assertTrue(measurementsBeforePosition.isEmpty())

        val availableMeasurementsBeforeTimestamp: ArrayList<GyroscopeSensorMeasurement>? =
            getPrivateProperty(
                BufferedSensorCollector::class,
                collector,
                "availableMeasurementsBeforeTimestamp"
            )
        requireNotNull(availableMeasurementsBeforeTimestamp)
        assertEquals(CAPACITY, availableMeasurementsBeforeTimestamp.size)

        val availableMeasurementsBeforePosition: ArrayList<GyroscopeSensorMeasurement>? =
            getPrivateProperty(
                BufferedSensorCollector::class,
                collector,
                "availableMeasurementsBeforePosition"
            )
        requireNotNull(availableMeasurementsBeforePosition)
        assertEquals(CAPACITY, availableMeasurementsBeforePosition.size)
    }

    @Test(expected = IllegalArgumentException::class)
    fun constructor_whenZeroCapacity_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        BufferedGyroscopeSensorCollector(context, capacity = 0)
    }

    @Test
    fun accuracyChangedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = BufferedGyroscopeSensorCollector(context)

        // check default value
        assertNull(collector.accuracyChangedListener)

        // set new value
        collector.accuracyChangedListener = accuracyChangedListener

        // check
        assertSame(accuracyChangedListener, collector.accuracyChangedListener)
    }

    @Test
    fun bufferFilledListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = BufferedGyroscopeSensorCollector(context)

        // check default value
        assertNull(collector.bufferFilledListener)

        // set new value
        collector.bufferFilledListener = bufferFilledListener

        // check
        assertSame(bufferFilledListener, collector.bufferFilledListener)
    }

    @Test
    fun measurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = BufferedGyroscopeSensorCollector(context)

        // check default value
        assertNull(collector.measurementListener)

        // set new value
        collector.measurementListener = measurementListener

        // check
        assertSame(measurementListener, collector.measurementListener)
    }

    @Test
    fun sensor_whenSensorTypeGyroscope_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spy(sensorManager)
//        val sensorManagerSpy = spyk(sensorManager)
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_GYROSCOPE)
//        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE) }.returns(sensor)
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector =
            BufferedGyroscopeSensorCollector(contextSpy, GyroscopeSensorType.GYROSCOPE)

        assertSame(sensor, collector.sensor)
        verify(contextSpy, only()).getSystemService(Context.SENSOR_SERVICE)
//        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(sensorManagerSpy, only()).getDefaultSensor(Sensor.TYPE_GYROSCOPE)
//        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE) }
    }

    @Test
    fun sensor_whenSensorTypeAccelerometerUncalibrated_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spy(sensorManager)
//        val sensorManagerSpy = spyk(sensorManager)
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
/*        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED) }.returns(
            sensor
        )*/
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = BufferedGyroscopeSensorCollector(
            contextSpy,
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED
        )

        assertSame(sensor, collector.sensor)
        verify(contextSpy, only()).getSystemService(Context.SENSOR_SERVICE)
//        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(sensorManagerSpy, only()).getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
//        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED) }
    }

    @Test
    fun sensorAvailable_whenSensorTypeGyroscope_returnsExpectedValue() {
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

        val collector =
            BufferedGyroscopeSensorCollector(contextSpy, GyroscopeSensorType.GYROSCOPE)

        assertFalse(collector.sensorAvailable)
        verify(contextSpy, only()).getSystemService(Context.SENSOR_SERVICE)
//        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(sensorManagerSpy, times(1)).getDefaultSensor(Sensor.TYPE_GYROSCOPE)
//        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE) }
    }

    @Test
    fun sensorAvailable_whenSensorTypeGyroscopeUncalibrated_returnsExpectedValue() {
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

        val collector =
            BufferedGyroscopeSensorCollector(contextSpy, GyroscopeSensorType.GYROSCOPE_UNCALIBRATED)

        assertFalse(collector.sensorAvailable)
        verify(contextSpy, only()).getSystemService(Context.SENSOR_SERVICE)
//        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(sensorManagerSpy, times(1)).getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
//        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED) }
    }

    @Test
    fun start_whenNoSensorManager_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(null).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = BufferedGyroscopeSensorCollector(contextSpy)
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
        doReturn(null).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
/*        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED) }
            .returns(null)*/
        doReturn(true).whenever(sensorManagerSpy).registerListener(any(), any<Sensor>(), any())
//        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = BufferedGyroscopeSensorCollector(contextSpy)
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
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
/*        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED) }
            .returns(sensor)*/
        doReturn(false).whenever(sensorManagerSpy).registerListener(any(), any<Sensor>(), any())
//        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(false)
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = BufferedGyroscopeSensorCollector(contextSpy)
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
            getPrivateProperty(BufferedSensorCollector::class, collector, "sensorEventListener")
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
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
/*        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED) }
            .returns(sensor)*/
        doReturn(true).whenever(sensorManagerSpy).registerListener(any(), any<Sensor>(), any())
//        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = BufferedGyroscopeSensorCollector(contextSpy)
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
            getPrivateProperty(BufferedSensorCollector::class, collector, "sensorEventListener")
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
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
/*        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED) }
            .returns(sensor)*/
        doReturn(true).whenever(sensorManagerSpy).registerListener(any(), any<Sensor>(), any())
//        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = BufferedGyroscopeSensorCollector(contextSpy)
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
            getPrivateProperty(BufferedSensorCollector::class, collector, "sensorEventListener")
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

        val collector = BufferedGyroscopeSensorCollector(contextSpy)

        // set initial values
        val buffer: ArrayDeque<GyroscopeSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "buffer")
        requireNotNull(buffer)
        buffer.add(GyroscopeSensorMeasurement())
        assertEquals(1, buffer.size)

        val randomizer = UniformRandomizer()
        val startOffset1 = randomizer.nextLong()
        setPrivateProperty(BufferedSensorCollector::class, collector, "startOffset", startOffset1)

        val numberOfProcessedMeasurements1 = randomizer.nextInt()
        setPrivateProperty(
            BufferedSensorCollector::class,
            collector,
            "numberOfProcessedMeasurements",
            numberOfProcessedMeasurements1
        )

        val mostRecentTimestamp = System.nanoTime()
        setPrivateProperty(
            BufferedSensorCollector::class,
            collector,
            "mostRecentTimestamp",
            mostRecentTimestamp
        )

        setPrivateProperty(BufferedSensorCollector::class, collector, "running", true)

        collector.stop()

        // check that values have been reset
        assertTrue(buffer.isEmpty())

        val startOffset2: Long? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "startOffset")
        assertNull(startOffset2)

        val numberOfProcessedMeasurements2: Int? = getPrivateProperty(
            BufferedSensorCollector::class,
            collector,
            "numberOfProcessedMeasurements"
        )
        assertEquals(0, numberOfProcessedMeasurements2)

        val mostRecentTimestamp2: Long? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "mostRecentTimestamp")
        assertEquals(0L, mostRecentTimestamp2)

        assertFalse(collector.running)
        assertFalse(collector.processing)
    }

    @Test
    fun stop_whenSensorManager_unregistersListenerAndResetsParameters() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spy(sensorManager)
//        val sensorManagerSpy = spyk(sensorManager)
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
/*        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED) }
            .returns(sensor)*/
        doNothing().whenever(sensorManagerSpy).unregisterListener(any(), any<Sensor>())
//        justRun { sensorManagerSpy.unregisterListener(any(), any<Sensor>()) }
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = BufferedGyroscopeSensorCollector(contextSpy)

        // set initial values
        val buffer: ArrayDeque<GyroscopeSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "buffer")
        requireNotNull(buffer)
        buffer.add(GyroscopeSensorMeasurement())
        assertEquals(1, buffer.size)

        val randomizer = UniformRandomizer()
        val startOffset1 = randomizer.nextLong()
        setPrivateProperty(BufferedSensorCollector::class, collector, "startOffset", startOffset1)

        val numberOfProcessedMeasurements1 = randomizer.nextInt()
        setPrivateProperty(
            BufferedSensorCollector::class,
            collector,
            "numberOfProcessedMeasurements",
            numberOfProcessedMeasurements1
        )

        val mostRecentTimestamp = System.nanoTime()
        setPrivateProperty(
            BufferedSensorCollector::class,
            collector,
            "mostRecentTimestamp",
            mostRecentTimestamp
        )

        setPrivateProperty(BufferedSensorCollector::class, collector, "running", true)

        collector.stop()

        // check that values have been reset
        assertTrue(buffer.isEmpty())

        val startOffset2: Long? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "startOffset")
        assertNull(startOffset2)

        val numberOfProcessedMeasurements2: Int? = getPrivateProperty(
            BufferedSensorCollector::class,
            collector,
            "numberOfProcessedMeasurements"
        )
        assertEquals(0, numberOfProcessedMeasurements2)

        val mostRecentTimestamp2: Long? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "mostRecentTimestamp")
        assertEquals(0L, mostRecentTimestamp2)

        assertFalse(collector.running)
        assertFalse(collector.processing)

        verify(contextSpy, only()).getSystemService(Context.SENSOR_SERVICE)
//        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(sensorManagerSpy, times(1)).unregisterListener(capture(sensorEventListenerCaptor), eq(sensor))
//        val slot = slot<SensorEventListener>()
//        verify(exactly = 1) { sensorManagerSpy.unregisterListener(capture(slot), sensor) }

        val eventListener = sensorEventListenerCaptor.value
//        val eventListener = slot.captured

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)
        assertSame(sensorEventListener, eventListener)
    }

    @Test
    fun onSensorChanged_whenNoEvent_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = BufferedGyroscopeSensorCollector(context)

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onSensorChanged(null)

        // check
        assertEquals(0.0f, collector.usage, 0.0f)
        assertTrue(collector.bufferedMeasurements.isEmpty())
        assertNull(collector.startOffset)
        assertEquals(0, collector.numberOfProcessedMeasurements)

        // check internal fields
        val buffer: ArrayDeque<GyroscopeSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "buffer")
        requireNotNull(buffer)
        assertTrue(buffer.isEmpty())

        val availableMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "availableMeasurements")
        requireNotNull(availableMeasurements)
        assertEquals(BufferedSensorCollector.DEFAULT_CAPACITY, availableMeasurements.size)

        val measurementsBeforeTimestamp: ArrayDeque<GyroscopeSensorMeasurement>? =
            getPrivateProperty(
                BufferedSensorCollector::class,
                collector,
                "measurementsBeforeTimestamp"
            )
        requireNotNull(measurementsBeforeTimestamp)
        assertTrue(measurementsBeforeTimestamp.isEmpty())

        val measurementsBeforePosition: ArrayDeque<GyroscopeSensorMeasurement>? =
            getPrivateProperty(
                BufferedSensorCollector::class,
                collector,
                "measurementsBeforePosition"
            )
        requireNotNull(measurementsBeforePosition)
        assertTrue(measurementsBeforePosition.isEmpty())

        val availableMeasurementsBeforeTimestamp: ArrayList<GyroscopeSensorMeasurement>? =
            getPrivateProperty(
                BufferedSensorCollector::class,
                collector,
                "availableMeasurementsBeforeTimestamp"
            )
        requireNotNull(availableMeasurementsBeforeTimestamp)
        assertEquals(
            BufferedSensorCollector.DEFAULT_CAPACITY,
            availableMeasurementsBeforeTimestamp.size
        )

        val availableMeasurementsBeforePosition: ArrayList<GyroscopeSensorMeasurement>? =
            getPrivateProperty(
                BufferedSensorCollector::class,
                collector,
                "availableMeasurementsBeforePosition"
            )
        requireNotNull(availableMeasurementsBeforePosition)
        assertEquals(
            BufferedSensorCollector.DEFAULT_CAPACITY,
            availableMeasurementsBeforePosition.size
        )
    }

    @Test
    fun onSensorChanged_whenSkipProcessingAndProcessing_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = BufferedGyroscopeSensorCollector(context)

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        setPrivateProperty(BufferedSensorCollector::class, collector, "processing", true)

        sensorEventListener.onSensorChanged(event)

        // check
        assertEquals(0.0f, collector.usage, 0.0f)
        assertTrue(collector.bufferedMeasurements.isEmpty())
        assertNull(collector.startOffset)
        assertEquals(0, collector.numberOfProcessedMeasurements)

        // check internal fields
        val buffer: ArrayDeque<GyroscopeSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "buffer")
        requireNotNull(buffer)
        assertTrue(buffer.isEmpty())

        val availableMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "availableMeasurements")
        requireNotNull(availableMeasurements)
        assertEquals(BufferedSensorCollector.DEFAULT_CAPACITY, availableMeasurements.size)

        val measurementsBeforeTimestamp: ArrayDeque<GyroscopeSensorMeasurement>? =
            getPrivateProperty(
                BufferedSensorCollector::class,
                collector,
                "measurementsBeforeTimestamp"
            )
        requireNotNull(measurementsBeforeTimestamp)
        assertTrue(measurementsBeforeTimestamp.isEmpty())

        val measurementsBeforePosition: ArrayDeque<GyroscopeSensorMeasurement>? =
            getPrivateProperty(
                BufferedSensorCollector::class,
                collector,
                "measurementsBeforePosition"
            )
        requireNotNull(measurementsBeforePosition)
        assertTrue(measurementsBeforePosition.isEmpty())

        val availableMeasurementsBeforeTimestamp: ArrayList<GyroscopeSensorMeasurement>? =
            getPrivateProperty(
                BufferedSensorCollector::class,
                collector,
                "availableMeasurementsBeforeTimestamp"
            )
        requireNotNull(availableMeasurementsBeforeTimestamp)
        assertEquals(
            BufferedSensorCollector.DEFAULT_CAPACITY,
            availableMeasurementsBeforeTimestamp.size
        )

        val availableMeasurementsBeforePosition: ArrayList<GyroscopeSensorMeasurement>? =
            getPrivateProperty(
                BufferedSensorCollector::class,
                collector,
                "availableMeasurementsBeforePosition"
            )
        requireNotNull(availableMeasurementsBeforePosition)
        assertEquals(
            BufferedSensorCollector.DEFAULT_CAPACITY,
            availableMeasurementsBeforePosition.size
        )
    }

    @Test
    fun onSensorChanged_whenNoSensorType_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spy(sensorManager)
//        val sensorManagerSpy = spyk(sensorManager)
        whenever(sensor.type).thenReturn(Sensor.TYPE_ACCELEROMETER)
//        every { sensor.type }.returns(Sensor.TYPE_ACCELEROMETER)
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
/*        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED) }
            .returns(sensor)*/
        doReturn(true).whenever(sensorManagerSpy).registerListener(any(), any<Sensor>(), any())
//        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = BufferedGyroscopeSensorCollector(
            contextSpy,
            startOffsetEnabled = false,
            measurementListener = measurementListener
        )

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        event.sensor = sensor
        sensorEventListener.onSensorChanged(event)

        // check
        verifyNoInteractions(measurementListener)
//        verify { measurementListener wasNot Called }

        assertEquals(0.0f, collector.usage, 0.0f)
        assertTrue(collector.bufferedMeasurements.isEmpty())
        assertNull(collector.startOffset)
        assertEquals(0, collector.numberOfProcessedMeasurements)

        // check internal fields
        val buffer: ArrayDeque<GyroscopeSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "buffer")
        requireNotNull(buffer)
        assertTrue(buffer.isEmpty())

        val availableMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "availableMeasurements")
        requireNotNull(availableMeasurements)
        assertEquals(BufferedSensorCollector.DEFAULT_CAPACITY, availableMeasurements.size)

        val measurementsBeforeTimestamp: ArrayDeque<GyroscopeSensorMeasurement>? =
            getPrivateProperty(
                BufferedSensorCollector::class,
                collector,
                "measurementsBeforeTimestamp"
            )
        requireNotNull(measurementsBeforeTimestamp)
        assertTrue(measurementsBeforeTimestamp.isEmpty())

        val measurementsBeforePosition: ArrayDeque<GyroscopeSensorMeasurement>? =
            getPrivateProperty(
                BufferedSensorCollector::class,
                collector,
                "measurementsBeforePosition"
            )
        requireNotNull(measurementsBeforePosition)
        assertTrue(measurementsBeforePosition.isEmpty())

        val availableMeasurementsBeforeTimestamp: ArrayList<GyroscopeSensorMeasurement>? =
            getPrivateProperty(
                BufferedSensorCollector::class,
                collector,
                "availableMeasurementsBeforeTimestamp"
            )
        requireNotNull(availableMeasurementsBeforeTimestamp)
        assertEquals(
            BufferedSensorCollector.DEFAULT_CAPACITY,
            availableMeasurementsBeforeTimestamp.size
        )

        val availableMeasurementsBeforePosition: ArrayList<GyroscopeSensorMeasurement>? =
            getPrivateProperty(
                BufferedSensorCollector::class,
                collector,
                "availableMeasurementsBeforePosition"
            )
        requireNotNull(availableMeasurementsBeforePosition)
        assertEquals(
            BufferedSensorCollector.DEFAULT_CAPACITY,
            availableMeasurementsBeforePosition.size
        )
    }

    @Test
    fun onSensorChanged_whenStartOffsetEnabled_setsStartOffset() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spy(sensorManager)
//        val sensorManagerSpy = spyk(sensorManager)
        whenever(sensor.type).thenReturn(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
//        every { sensor.type }.returns(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
/*        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED) }
            .returns(sensor)*/
        doReturn(true).whenever(sensorManagerSpy).registerListener(any(), any<Sensor>(), any())
//        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = BufferedGyroscopeSensorCollector(
            contextSpy,
            startOffsetEnabled = true,
            measurementListener = measurementListener
        )

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        // set start time
        val startTimestamp = System.nanoTime()
        setPrivateProperty(
            BufferedSensorCollector::class,
            collector,
            "startTimestamp",
            startTimestamp
        )

        assertNull(collector.startOffset)

        event.sensor = sensor
        event.timestamp = System.nanoTime()
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, floatArrayOf(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f))
        sensorEventListener.onSensorChanged(event)

        // check
        val startOffset = event.timestamp - startTimestamp
        assertNotNull(collector.startOffset)
        assertEquals(startOffset, collector.startOffset)
        verify(measurementListener, only()).onMeasurement(eq(collector), capture(gyroscopeSensorMeasurementCaptor), eq(0))
//        val slot = slot<GyroscopeSensorMeasurement>()
//        verify(exactly = 1) { measurementListener.onMeasurement(collector, capture(slot), 0) }

        val measurement = gyroscopeSensorMeasurementCaptor.value
//        val measurement = slot.captured

        assertNotEquals(0.0f, collector.usage, 0.0f)
        assertEquals(1, collector.bufferedMeasurements.size)
        assertEquals(1, collector.numberOfProcessedMeasurements)
        assertEquals(event.timestamp + startOffset, collector.mostRecentTimestamp)
        assertEquals(event.timestamp + startOffset, collector.oldestTimestampInBuffer)

        // check internal fields
        val buffer: ArrayDeque<GyroscopeSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "buffer")
        requireNotNull(buffer)
        assertEquals(1, buffer.size)
        assertSame(measurement, buffer.peek())

        val availableMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "availableMeasurements")
        requireNotNull(availableMeasurements)
        assertEquals(BufferedSensorCollector.DEFAULT_CAPACITY - 1, availableMeasurements.size)

        val measurementsBeforeTimestamp: ArrayDeque<GyroscopeSensorMeasurement>? =
            getPrivateProperty(
                BufferedSensorCollector::class,
                collector,
                "measurementsBeforeTimestamp"
            )
        requireNotNull(measurementsBeforeTimestamp)
        assertTrue(measurementsBeforeTimestamp.isEmpty())

        val measurementsBeforePosition: ArrayDeque<GyroscopeSensorMeasurement>? =
            getPrivateProperty(
                BufferedSensorCollector::class,
                collector,
                "measurementsBeforePosition"
            )
        requireNotNull(measurementsBeforePosition)
        assertTrue(measurementsBeforePosition.isEmpty())

        val availableMeasurementsBeforeTimestamp: ArrayList<GyroscopeSensorMeasurement>? =
            getPrivateProperty(
                BufferedSensorCollector::class,
                collector,
                "availableMeasurementsBeforeTimestamp"
            )
        requireNotNull(availableMeasurementsBeforeTimestamp)
        assertEquals(
            BufferedSensorCollector.DEFAULT_CAPACITY,
            availableMeasurementsBeforeTimestamp.size
        )

        val availableMeasurementsBeforePosition: ArrayList<GyroscopeSensorMeasurement>? =
            getPrivateProperty(
                BufferedSensorCollector::class,
                collector,
                "availableMeasurementsBeforePosition"
            )
        requireNotNull(availableMeasurementsBeforePosition)
        assertEquals(
            BufferedSensorCollector.DEFAULT_CAPACITY,
            availableMeasurementsBeforePosition.size
        )
    }

    @Test
    fun onSensorChanged_whenBufferFilledAndStopWhenFilledBufferEnabled_notifiesAndStops() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spy(sensorManager)
//        val sensorManagerSpy = spyk(sensorManager)
        whenever(sensor.type).thenReturn(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
//        every { sensor.type }.returns(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
/*        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED) }
            .returns(sensor)*/
        doReturn(true).whenever(sensorManagerSpy).registerListener(any(), any<Sensor>(), any())
//        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = BufferedGyroscopeSensorCollector(
            contextSpy,
            stopWhenFilledBuffer = true,
            bufferFilledListener = bufferFilledListener,
            measurementListener = measurementListener
        )

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        // clear available measurements and fill buffer
        val buffer: ArrayDeque<GyroscopeSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "buffer")
        requireNotNull(buffer)

        val availableMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "availableMeasurements")
        requireNotNull(availableMeasurements)

        buffer.addAll(availableMeasurements)
        availableMeasurements.clear()

        assertEquals(BufferedSensorCollector.DEFAULT_CAPACITY, buffer.size)
        assertTrue(availableMeasurements.isEmpty())

        // set as running
        setPrivateProperty(BufferedSensorCollector::class, collector, "running", true)
        assertTrue(collector.running)

        event.sensor = sensor
        event.timestamp = System.nanoTime()
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, floatArrayOf(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f))
        sensorEventListener.onSensorChanged(event)

        // check
        verifyNoInteractions(measurementListener)
//        verify { measurementListener wasNot Called }
        verify(bufferFilledListener, only()).onBufferFilled(collector)
//        verify(exactly = 1) { bufferFilledListener.onBufferFilled(collector) }

        assertTrue(buffer.isEmpty())
        assertEquals(BufferedSensorCollector.DEFAULT_CAPACITY, availableMeasurements.size)
        assertEquals(0L, collector.mostRecentTimestamp)
        assertNull(collector.oldestTimestampInBuffer)
        assertEquals(0, collector.numberOfProcessedMeasurements)
        assertFalse(collector.running)
    }

    @Test
    fun onSensorChanged_whenBufferFilledAndStopWhenFilledBufferDisabled_notifiesAndLosesOldMeasurement() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spy(sensorManager)
//        val sensorManagerSpy = spyk(sensorManager)
        whenever(sensor.type).thenReturn(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
//        every { sensor.type }.returns(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
        doReturn(sensor).whenever(sensorManagerSpy).getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED)
/*        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GYROSCOPE_UNCALIBRATED) }
            .returns(sensor)*/
        doReturn(true).whenever(sensorManagerSpy).registerListener(any(), any<Sensor>(), any())
//        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spy(context)
//        val contextSpy = spyk(context)
        doReturn(sensorManagerSpy).whenever(contextSpy).getSystemService(Context.SENSOR_SERVICE)
//        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = BufferedGyroscopeSensorCollector(
            contextSpy,
            stopWhenFilledBuffer = false,
            startOffsetEnabled = false,
            bufferFilledListener = bufferFilledListener,
            measurementListener = measurementListener
        )

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        // clear available measurements and fill buffer
        val buffer: ArrayDeque<GyroscopeSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "buffer")
        requireNotNull(buffer)

        val availableMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "availableMeasurements")
        requireNotNull(availableMeasurements)

        buffer.addAll(availableMeasurements)
        availableMeasurements.clear()

        assertEquals(BufferedSensorCollector.DEFAULT_CAPACITY, buffer.size)
        assertTrue(availableMeasurements.isEmpty())

        // set as running
        setPrivateProperty(BufferedSensorCollector::class, collector, "running", true)
        assertTrue(collector.running)

        event.sensor = sensor
        event.timestamp = System.nanoTime()
        event.accuracy = SensorAccuracy.HIGH.value
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, floatArrayOf(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f))
        sensorEventListener.onSensorChanged(event)

        // check
        verify(bufferFilledListener, only()).onBufferFilled(collector)
//        verify(exactly = 1) { bufferFilledListener.onBufferFilled(collector) }
        verify(measurementListener, only()).onMeasurement(eq(collector), capture(gyroscopeSensorMeasurementCaptor), eq(buffer.size - 1))
//        val slot = slot<GyroscopeSensorMeasurement>()
//        verify { measurementListener.onMeasurement(collector, capture(slot), buffer.size - 1) }

        val measurement = gyroscopeSensorMeasurementCaptor.value
//        val measurement = slot.captured
        assertEquals(1.0f, measurement.wx, 0.0f)
        assertEquals(2.0f, measurement.wy, 0.0f)
        assertEquals(3.0f, measurement.wz, 0.0f)
        assertEquals(4.0f, measurement.bx)
        assertEquals(5.0f, measurement.by)
        assertEquals(6.0f, measurement.bz)
        assertEquals(event.timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
        assertEquals(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED, measurement.sensorType)

        assertEquals(BufferedSensorCollector.DEFAULT_CAPACITY, buffer.size)
        assertTrue(availableMeasurements.isEmpty())
        assertEquals(event.timestamp, collector.mostRecentTimestamp)
        assertEquals(0L, collector.oldestTimestampInBuffer)
        assertEquals(1, collector.numberOfProcessedMeasurements)
        assertTrue(collector.running)
    }

    @Test
    fun onAccuracyChanged_whenNoSensor_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = BufferedGyroscopeSensorCollector(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onAccuracyChanged(null, SensorAccuracy.HIGH.value)

        verifyNoInteractions(accuracyChangedListener)
//        verify { accuracyChangedListener wasNot Called }
    }

    @Test
    fun onAccuracyChanged_whenUnsupportedSensorType_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = BufferedGyroscopeSensorCollector(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        whenever(sensor.type).thenReturn(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value)
//        every { sensor.type }.returns(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value)
        sensorEventListener.onAccuracyChanged(sensor, SensorAccuracy.HIGH.value)

        verifyNoInteractions(accuracyChangedListener)
//        verify { accuracyChangedListener wasNot Called }
    }

    @Test
    fun onAccuracyChanged_whenNoListener_executes() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = BufferedGyroscopeSensorCollector(context)

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        whenever(sensor.type).thenReturn(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value)
//        every { sensor.type }.returns(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value)
        sensorEventListener.onAccuracyChanged(sensor, SensorAccuracy.HIGH.value)
    }

    @Test
    fun onAccuracyChanged_whenListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = BufferedGyroscopeSensorCollector(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        whenever(sensor.type).thenReturn(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value)
//        every { sensor.type }.returns(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value)
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

    @Test
    fun getMeasurementsBeforeTimestamp_whenEmptyBuffer_returnsEmptyList() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = BufferedGyroscopeSensorCollector(context)

        val buffer: ArrayDeque<GyroscopeSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "buffer")
        requireNotNull(buffer)
        assertTrue(buffer.isEmpty())

        val timestamp = System.nanoTime()
        val measurements = collector.getMeasurementsBeforeTimestamp(timestamp)
        assertTrue(measurements.isEmpty())
    }

    @Test
    fun getMeasurementsBeforeTimestamp_whenNonEmptyBuffer_returnsExpectedResult() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = BufferedGyroscopeSensorCollector(context)

        val buffer: ArrayDeque<GyroscopeSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "buffer")
        requireNotNull(buffer)
        assertTrue(buffer.isEmpty())

        val availableMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "availableMeasurements")
        requireNotNull(availableMeasurements)
        assertEquals(BufferedSensorCollector.DEFAULT_CAPACITY, availableMeasurements.size)

        // add measurements to buffer and clear available measurements
        availableMeasurements.clear()

        val timestamp = System.nanoTime()
        val measurement1 = GyroscopeSensorMeasurement(
            1.0f,
            1.0f,
            1.0f,
            1.0f,
            1.0f,
            1.0f,
            timestamp - 1,
            SensorAccuracy.HIGH
        )
        val measurement2 = GyroscopeSensorMeasurement(
            2.0f,
            2.0f,
            2.0f,
            2.0f,
            2.0f,
            2.0f,
            timestamp,
            SensorAccuracy.HIGH
        )
        val measurement3 = GyroscopeSensorMeasurement(
            3.0f,
            3.0f,
            3.0f,
            3.0f,
            3.0f,
            3.0f,
            timestamp + 1,
            SensorAccuracy.HIGH
        )
        buffer.add(measurement1)
        buffer.add(measurement2)
        buffer.add(measurement3)

        val measurements = ArrayList(collector.getMeasurementsBeforeTimestamp(timestamp))
        assertEquals(2, measurements.size)

        assertNotSame(measurement1, measurements[0])
        assertEquals(1.0f, measurements[0].wx)
        assertEquals(1.0f, measurements[0].wy)
        assertEquals(1.0f, measurements[0].wz)
        assertEquals(1.0f, measurements[0].bx)
        assertEquals(1.0f, measurements[0].by)
        assertEquals(1.0f, measurements[0].bz)
        assertEquals(timestamp - 1, measurements[0].timestamp)
        assertEquals(SensorAccuracy.HIGH, measurements[0].accuracy)

        assertNotSame(measurement2, measurements[1])
        assertEquals(2.0f, measurements[1].wx)
        assertEquals(2.0f, measurements[1].wy)
        assertEquals(2.0f, measurements[1].wz)
        assertEquals(2.0f, measurements[1].bx)
        assertEquals(2.0f, measurements[1].by)
        assertEquals(2.0f, measurements[1].bz)
        assertEquals(timestamp, measurements[1].timestamp)
        assertEquals(SensorAccuracy.HIGH, measurements[1].accuracy)

        assertEquals(1, buffer.size)
        assertEquals(2, availableMeasurements.size)
    }

    @Test
    fun getMeasurementsBeforePosition_whenEmptyBuffer_returnsEmptyList() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = BufferedGyroscopeSensorCollector(context)

        val buffer: ArrayDeque<GyroscopeSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "buffer")
        requireNotNull(buffer)
        assertTrue(buffer.isEmpty())

        val measurements = collector.getMeasurementsBeforePosition(0)
        assertTrue(measurements.isEmpty())
    }

    @Test
    fun getMeasurementsBeforePosition_whenNonEmptyBuffer_returnsExpectedResult() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = BufferedGyroscopeSensorCollector(context)

        val buffer: ArrayDeque<GyroscopeSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "buffer")
        requireNotNull(buffer)
        assertTrue(buffer.isEmpty())

        val availableMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "availableMeasurements")
        requireNotNull(availableMeasurements)
        assertEquals(BufferedSensorCollector.DEFAULT_CAPACITY, availableMeasurements.size)

        // add measurements to buffer and clear available measurements
        availableMeasurements.clear()

        val timestamp = System.nanoTime()
        val measurement1 = GyroscopeSensorMeasurement(
            1.0f,
            1.0f,
            1.0f,
            1.0f,
            1.0f,
            1.0f,
            timestamp - 1,
            SensorAccuracy.HIGH
        )
        val measurement2 = GyroscopeSensorMeasurement(
            2.0f,
            2.0f,
            2.0f,
            2.0f,
            2.0f,
            2.0f,
            timestamp,
            SensorAccuracy.HIGH
        )
        val measurement3 = GyroscopeSensorMeasurement(
            3.0f,
            3.0f,
            3.0f,
            3.0f,
            3.0f,
            3.0f,
            timestamp + 1,
            SensorAccuracy.HIGH
        )
        buffer.add(measurement1)
        buffer.add(measurement2)
        buffer.add(measurement3)

        val measurements = ArrayList(collector.getMeasurementsBeforePosition(1))
        assertEquals(2, measurements.size)

        assertNotSame(measurement1, measurements[0])
        assertEquals(1.0f, measurements[0].wx)
        assertEquals(1.0f, measurements[0].wy)
        assertEquals(1.0f, measurements[0].wz)
        assertEquals(1.0f, measurements[0].bx)
        assertEquals(1.0f, measurements[0].by)
        assertEquals(1.0f, measurements[0].bz)
        assertEquals(timestamp - 1, measurements[0].timestamp)
        assertEquals(SensorAccuracy.HIGH, measurements[0].accuracy)

        assertNotSame(measurement2, measurements[1])
        assertEquals(2.0f, measurements[1].wx)
        assertEquals(2.0f, measurements[1].wy)
        assertEquals(2.0f, measurements[1].wz)
        assertEquals(2.0f, measurements[1].bx)
        assertEquals(2.0f, measurements[1].by)
        assertEquals(2.0f, measurements[1].bz)
        assertEquals(timestamp, measurements[1].timestamp)
        assertEquals(SensorAccuracy.HIGH, measurements[1].accuracy)

        assertEquals(1, buffer.size)
        assertEquals(2, availableMeasurements.size)
    }

    private companion object {
        const val CAPACITY = 1
    }
}