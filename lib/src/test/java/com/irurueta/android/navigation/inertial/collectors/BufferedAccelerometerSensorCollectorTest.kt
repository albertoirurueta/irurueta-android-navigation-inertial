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
import android.os.Build
import androidx.test.core.app.ApplicationProvider
import com.irurueta.android.navigation.inertial.getPrivateProperty
import com.irurueta.android.navigation.inertial.setPrivateProperty
import com.irurueta.statistics.UniformRandomizer
import io.mockk.*
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import org.robolectric.annotation.Config
import java.util.*

@RunWith(RobolectricTestRunner::class)
class BufferedAccelerometerSensorCollectorTest {

    @Test
    fun constructor_whenRequiredParameters_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = BufferedAccelerometerSensorCollector(context)

        // check values
        assertSame(context, collector.context)
        assertEquals(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED, collector.sensorType)
        assertEquals(SensorDelay.FASTEST, collector.sensorDelay)
        assertEquals(BufferedSensorCollector.DEFAULT_CAPACITY, collector.capacity)
        assertTrue(collector.startOffsetEnabled)
        assertTrue(collector.stopWhenFilledBuffer)
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
        assertEquals(0, collector.numberOfProcessedMeasurements)
        assertEquals(0L, collector.mostRecentTimestamp)
        assertNull(collector.oldestTimestampInBuffer)

        // check internal fields initialization
        val buffer: ArrayDeque<AccelerometerSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "buffer")
        requireNotNull(buffer)
        assertTrue(buffer.isEmpty())

        val availableMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "availableMeasurements")
        requireNotNull(availableMeasurements)
        assertEquals(BufferedSensorCollector.DEFAULT_CAPACITY, availableMeasurements.size)

        val measurementsBeforeTimestamp: ArrayDeque<AccelerometerSensorMeasurement>? =
            getPrivateProperty(
                BufferedSensorCollector::class,
                collector,
                "measurementsBeforeTimestamp"
            )
        requireNotNull(measurementsBeforeTimestamp)
        assertTrue(measurementsBeforeTimestamp.isEmpty())

        val measurementsBeforePosition: ArrayDeque<AccelerometerSensorMeasurement>? =
            getPrivateProperty(
                BufferedSensorCollector::class,
                collector,
                "measurementsBeforePosition"
            )
        requireNotNull(measurementsBeforePosition)
        assertTrue(measurementsBeforePosition.isEmpty())

        val availableMeasurementsBeforeTimestamp: ArrayList<AccelerometerSensorMeasurement>? =
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

        val availableMeasurementsBeforePosition: ArrayList<AccelerometerSensorMeasurement>? =
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
        val accuracyChangedListener =
            mockk<BufferedSensorCollector.OnAccuracyChangedListener<AccelerometerSensorMeasurement, BufferedAccelerometerSensorCollector>>()
        val bufferFilledListener =
            mockk<BufferedSensorCollector.OnBufferFilledListener<AccelerometerSensorMeasurement, BufferedAccelerometerSensorCollector>>()
        val measurementListener =
            mockk<BufferedSensorCollector.OnMeasurementListener<AccelerometerSensorMeasurement, BufferedAccelerometerSensorCollector>>()
        val collector = BufferedAccelerometerSensorCollector(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            SensorDelay.NORMAL,
            CAPACITY,
            startOffsetEnabled = false,
            stopWhenFilledBuffer = false,
            accuracyChangedListener,
            bufferFilledListener,
            measurementListener
        )

        // check values
        assertSame(context, collector.context)
        assertEquals(AccelerometerSensorType.ACCELEROMETER, collector.sensorType)
        assertEquals(SensorDelay.NORMAL, collector.sensorDelay)
        assertEquals(CAPACITY, collector.capacity)
        assertFalse(collector.startOffsetEnabled)
        assertFalse(collector.stopWhenFilledBuffer)
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
        assertEquals(0, collector.numberOfProcessedMeasurements)
        assertEquals(0L, collector.mostRecentTimestamp)
        assertNull(collector.oldestTimestampInBuffer)

        // check internal fields initialization
        val buffer: ArrayDeque<AccelerometerSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "buffer")
        requireNotNull(buffer)
        assertTrue(buffer.isEmpty())

        val availableMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "availableMeasurements")
        requireNotNull(availableMeasurements)
        assertEquals(CAPACITY, availableMeasurements.size)

        val measurementsBeforeTimestamp: ArrayDeque<AccelerometerSensorMeasurement>? =
            getPrivateProperty(
                BufferedSensorCollector::class,
                collector,
                "measurementsBeforeTimestamp"
            )
        requireNotNull(measurementsBeforeTimestamp)
        assertTrue(measurementsBeforeTimestamp.isEmpty())

        val measurementsBeforePosition: ArrayDeque<AccelerometerSensorMeasurement>? =
            getPrivateProperty(
                BufferedSensorCollector::class,
                collector,
                "measurementsBeforePosition"
            )
        requireNotNull(measurementsBeforePosition)
        assertTrue(measurementsBeforePosition.isEmpty())

        val availableMeasurementsBeforeTimestamp: ArrayList<AccelerometerSensorMeasurement>? =
            getPrivateProperty(
                BufferedSensorCollector::class,
                collector,
                "availableMeasurementsBeforeTimestamp"
            )
        requireNotNull(availableMeasurementsBeforeTimestamp)
        assertEquals(CAPACITY, availableMeasurementsBeforeTimestamp.size)

        val availableMeasurementsBeforePosition: ArrayList<AccelerometerSensorMeasurement>? =
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
        BufferedAccelerometerSensorCollector(context, capacity = 0)
    }

    @Test
    fun accuracyChangedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = BufferedAccelerometerSensorCollector(context)

        // check default value
        assertNull(collector.accuracyChangedListener)

        // set new value
        val accuracyChangedListener =
            mockk<BufferedSensorCollector.OnAccuracyChangedListener<AccelerometerSensorMeasurement, BufferedAccelerometerSensorCollector>>()
        collector.accuracyChangedListener = accuracyChangedListener

        // check
        assertSame(accuracyChangedListener, collector.accuracyChangedListener)
    }

    @Test
    fun bufferFilledListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = BufferedAccelerometerSensorCollector(context)

        // check default value
        assertNull(collector.bufferFilledListener)

        // set new value
        val bufferFilledListener =
            mockk<BufferedSensorCollector.OnBufferFilledListener<AccelerometerSensorMeasurement, BufferedAccelerometerSensorCollector>>()
        collector.bufferFilledListener = bufferFilledListener

        // check
        assertSame(bufferFilledListener, collector.bufferFilledListener)
    }

    @Test
    fun measurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = BufferedAccelerometerSensorCollector(context)

        // check default value
        assertNull(collector.measurementListener)

        // set new value
        val measurementListener =
            mockk<BufferedSensorCollector.OnMeasurementListener<AccelerometerSensorMeasurement, BufferedAccelerometerSensorCollector>>()
        collector.measurementListener = measurementListener

        // check
        assertSame(measurementListener, collector.measurementListener)
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

        val collector =
            BufferedAccelerometerSensorCollector(contextSpy, AccelerometerSensorType.ACCELEROMETER)

        assertSame(sensorMock, collector.sensor)
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
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED) }.returns(
            sensorMock
        )
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = BufferedAccelerometerSensorCollector(
            contextSpy,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        )

        assertSame(sensorMock, collector.sensor)
        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED) }
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

        val collector =
            BufferedAccelerometerSensorCollector(contextSpy, AccelerometerSensorType.ACCELEROMETER)

        assertFalse(collector.sensorAvailable)
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
        every { sensorManagerSpy.getDefaultSensor(Constants.TYPE_ACCELEROMETER_UNCALIBRATED) }
            .returns(sensorMock)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = BufferedAccelerometerSensorCollector(
            contextSpy,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        )

        assertTrue(collector.sensorAvailable)
        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Constants.TYPE_ACCELEROMETER_UNCALIBRATED) }
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
        every { sensorManagerSpy.getDefaultSensor(Constants.TYPE_ACCELEROMETER_UNCALIBRATED) }
            .returns(sensorMock)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = BufferedAccelerometerSensorCollector(
            contextSpy,
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED
        )

        assertFalse(collector.sensorAvailable)
        verify(exactly = 0) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 0) { sensorManagerSpy.getDefaultSensor(Constants.TYPE_ACCELEROMETER_UNCALIBRATED) }
    }

    @Test
    fun start_whenNoSensorManager_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = BufferedAccelerometerSensorCollector(contextSpy)
        assertEquals(0L, collector.startTimestamp)
        assertFalse(collector.start())

        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }

        assertFalse(collector.running)
        assertEquals(0L, collector.startTimestamp)
    }

    @Test
    fun start_whenNoSensor_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED) }
            .returns(null)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = BufferedAccelerometerSensorCollector(contextSpy)
        assertEquals(0L, collector.startTimestamp)
        assertFalse(collector.start())

        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 0) {
            sensorManagerSpy.registerListener(
                any(),
                any<Sensor>(),
                collector.sensorDelay.value
            )
        }

        assertFalse(collector.running)
        assertEquals(0L, collector.startTimestamp)
    }

    @Test
    fun start_whenRegisterListenerFails_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED) }
            .returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(false)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = BufferedAccelerometerSensorCollector(contextSpy)
        assertEquals(0L, collector.startTimestamp)
        assertFalse(collector.start())

        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(
                capture(slot),
                sensorMock,
                collector.sensorDelay.value
            )
        }

        val eventListener = slot.captured

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
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED) }
            .returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = BufferedAccelerometerSensorCollector(contextSpy)
        assertEquals(0L, collector.startTimestamp)
        assertTrue(collector.start())

        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(
                capture(slot),
                sensorMock,
                collector.sensorDelay.value
            )
        }

        val eventListener = slot.captured

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
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED) }
            .returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = BufferedAccelerometerSensorCollector(contextSpy)
        assertEquals(0L, collector.startTimestamp)
        val startTimestamp = System.nanoTime()
        assertTrue(collector.start(startTimestamp))

        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(
                capture(slot),
                sensorMock,
                collector.sensorDelay.value
            )
        }

        val eventListener = slot.captured

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
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = BufferedAccelerometerSensorCollector(contextSpy)

        // set initial values
        val buffer: ArrayDeque<AccelerometerSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "buffer")
        requireNotNull(buffer)
        buffer.add(AccelerometerSensorMeasurement())
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
    }

    @Test
    fun stop_whenSensorManager_unregistersListenerAndResetsParameters() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED) }
            .returns(sensorMock)
        justRun { sensorManagerSpy.unregisterListener(any(), any<Sensor>()) }
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = BufferedAccelerometerSensorCollector(contextSpy)

        // set initial values
        val buffer: ArrayDeque<AccelerometerSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "buffer")
        requireNotNull(buffer)
        buffer.add(AccelerometerSensorMeasurement())
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

        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        val slot = slot<SensorEventListener>()
        verify(exactly = 1) { sensorManagerSpy.unregisterListener(capture(slot), sensorMock) }

        val eventListener = slot.captured

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)
        assertSame(sensorEventListener, eventListener)
    }

    @Test
    fun onSensorChanged_whenNoEvent_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = BufferedAccelerometerSensorCollector(context)

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
        val buffer: ArrayDeque<AccelerometerSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "buffer")
        requireNotNull(buffer)
        assertTrue(buffer.isEmpty())

        val availableMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "availableMeasurements")
        requireNotNull(availableMeasurements)
        assertEquals(BufferedSensorCollector.DEFAULT_CAPACITY, availableMeasurements.size)

        val measurementsBeforeTimestamp: ArrayDeque<AccelerometerSensorMeasurement>? =
            getPrivateProperty(
                BufferedSensorCollector::class,
                collector,
                "measurementsBeforeTimestamp"
            )
        requireNotNull(measurementsBeforeTimestamp)
        assertTrue(measurementsBeforeTimestamp.isEmpty())

        val measurementsBeforePosition: ArrayDeque<AccelerometerSensorMeasurement>? =
            getPrivateProperty(
                BufferedSensorCollector::class,
                collector,
                "measurementsBeforePosition"
            )
        requireNotNull(measurementsBeforePosition)
        assertTrue(measurementsBeforePosition.isEmpty())

        val availableMeasurementsBeforeTimestamp: ArrayList<AccelerometerSensorMeasurement>? =
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

        val availableMeasurementsBeforePosition: ArrayList<AccelerometerSensorMeasurement>? =
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
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorMock.type }.returns(Sensor.TYPE_GYROSCOPE)
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED) }
            .returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val measurementListener =
            mockk<BufferedSensorCollector.OnMeasurementListener<AccelerometerSensorMeasurement, BufferedAccelerometerSensorCollector>>()
        val collector = BufferedAccelerometerSensorCollector(
            contextSpy,
            startOffsetEnabled = false,
            measurementListener = measurementListener
        )

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        val event = mockk<SensorEvent>()
        event.sensor = sensorMock
        sensorEventListener.onSensorChanged(event)

        // check
        verify { measurementListener wasNot Called }

        assertEquals(0.0f, collector.usage, 0.0f)
        assertTrue(collector.bufferedMeasurements.isEmpty())
        assertNull(collector.startOffset)
        assertEquals(0, collector.numberOfProcessedMeasurements)

        // check internal fields
        val buffer: ArrayDeque<AccelerometerSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "buffer")
        requireNotNull(buffer)
        assertTrue(buffer.isEmpty())

        val availableMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "availableMeasurements")
        requireNotNull(availableMeasurements)
        assertEquals(BufferedSensorCollector.DEFAULT_CAPACITY, availableMeasurements.size)

        val measurementsBeforeTimestamp: ArrayDeque<AccelerometerSensorMeasurement>? =
            getPrivateProperty(
                BufferedSensorCollector::class,
                collector,
                "measurementsBeforeTimestamp"
            )
        requireNotNull(measurementsBeforeTimestamp)
        assertTrue(measurementsBeforeTimestamp.isEmpty())

        val measurementsBeforePosition: ArrayDeque<AccelerometerSensorMeasurement>? =
            getPrivateProperty(
                BufferedSensorCollector::class,
                collector,
                "measurementsBeforePosition"
            )
        requireNotNull(measurementsBeforePosition)
        assertTrue(measurementsBeforePosition.isEmpty())

        val availableMeasurementsBeforeTimestamp: ArrayList<AccelerometerSensorMeasurement>? =
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

        val availableMeasurementsBeforePosition: ArrayList<AccelerometerSensorMeasurement>? =
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
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorMock.type }.returns(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED)
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED) }
            .returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val measurementListener =
            mockk<BufferedSensorCollector.OnMeasurementListener<AccelerometerSensorMeasurement, BufferedAccelerometerSensorCollector>>(
                relaxUnitFun = true
            )
        val collector = BufferedAccelerometerSensorCollector(
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

        val event = mockk<SensorEvent>()
        event.sensor = sensorMock
        event.timestamp = System.nanoTime()
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, floatArrayOf(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f))
        sensorEventListener.onSensorChanged(event)

        // check
        val startOffset = event.timestamp - startTimestamp
        assertNotNull(collector.startOffset)
        assertEquals(startOffset, collector.startOffset)
        val slot = slot<AccelerometerSensorMeasurement>()
        verify(exactly = 1) { measurementListener.onMeasurement(collector, capture(slot), 0) }

        val measurement = slot.captured

        assertNotEquals(0.0f, collector.usage, 0.0f)
        assertEquals(1, collector.bufferedMeasurements.size)
        assertEquals(1, collector.numberOfProcessedMeasurements)
        assertEquals(event.timestamp + startOffset, collector.mostRecentTimestamp)
        assertEquals(event.timestamp + startOffset, collector.oldestTimestampInBuffer)

        // check internal fields
        val buffer: ArrayDeque<AccelerometerSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "buffer")
        requireNotNull(buffer)
        assertEquals(1, buffer.size)
        assertSame(measurement, buffer.peek())

        val availableMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "availableMeasurements")
        requireNotNull(availableMeasurements)
        assertEquals(BufferedSensorCollector.DEFAULT_CAPACITY - 1, availableMeasurements.size)

        val measurementsBeforeTimestamp: ArrayDeque<AccelerometerSensorMeasurement>? =
            getPrivateProperty(
                BufferedSensorCollector::class,
                collector,
                "measurementsBeforeTimestamp"
            )
        requireNotNull(measurementsBeforeTimestamp)
        assertTrue(measurementsBeforeTimestamp.isEmpty())

        val measurementsBeforePosition: ArrayDeque<AccelerometerSensorMeasurement>? =
            getPrivateProperty(
                BufferedSensorCollector::class,
                collector,
                "measurementsBeforePosition"
            )
        requireNotNull(measurementsBeforePosition)
        assertTrue(measurementsBeforePosition.isEmpty())

        val availableMeasurementsBeforeTimestamp: ArrayList<AccelerometerSensorMeasurement>? =
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

        val availableMeasurementsBeforePosition: ArrayList<AccelerometerSensorMeasurement>? =
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
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorMock.type }.returns(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED)
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED) }
            .returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val bufferFilledListener =
            mockk<BufferedSensorCollector.OnBufferFilledListener<AccelerometerSensorMeasurement, BufferedAccelerometerSensorCollector>>(
                relaxUnitFun = true
            )
        val measurementListener =
            mockk<BufferedSensorCollector.OnMeasurementListener<AccelerometerSensorMeasurement, BufferedAccelerometerSensorCollector>>()
        val collector = BufferedAccelerometerSensorCollector(
            contextSpy,
            stopWhenFilledBuffer = true,
            bufferFilledListener = bufferFilledListener,
            measurementListener = measurementListener
        )

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        // clear available measurements and fill buffer
        val buffer: ArrayDeque<AccelerometerSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "buffer")
        requireNotNull(buffer)

        val availableMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "availableMeasurements")
        requireNotNull(availableMeasurements)

        buffer.addAll(availableMeasurements)
        availableMeasurements.clear()

        assertEquals(BufferedSensorCollector.DEFAULT_CAPACITY, buffer.size)
        assertTrue(availableMeasurements.isEmpty())

        // set as running
        setPrivateProperty(BufferedSensorCollector::class, collector, "running", true)
        assertTrue(collector.running)

        val event = mockk<SensorEvent>()
        event.sensor = sensorMock
        event.timestamp = System.nanoTime()
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, floatArrayOf(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f))
        sensorEventListener.onSensorChanged(event)

        // check
        verify { measurementListener wasNot Called }
        verify(exactly = 1) { bufferFilledListener.onBufferFilled(collector) }

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
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorMock.type }.returns(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED)
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED) }
            .returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val bufferFilledListener =
            mockk<BufferedSensorCollector.OnBufferFilledListener<AccelerometerSensorMeasurement, BufferedAccelerometerSensorCollector>>(
                relaxUnitFun = true
            )
        val measurementListener =
            mockk<BufferedSensorCollector.OnMeasurementListener<AccelerometerSensorMeasurement, BufferedAccelerometerSensorCollector>>(
                relaxUnitFun = true
            )
        val collector = BufferedAccelerometerSensorCollector(
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
        val buffer: ArrayDeque<AccelerometerSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "buffer")
        requireNotNull(buffer)

        val availableMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "availableMeasurements")
        requireNotNull(availableMeasurements)

        buffer.addAll(availableMeasurements)
        availableMeasurements.clear()

        assertEquals(BufferedSensorCollector.DEFAULT_CAPACITY, buffer.size)
        assertTrue(availableMeasurements.isEmpty())

        // set as running
        setPrivateProperty(BufferedSensorCollector::class, collector, "running", true)
        assertTrue(collector.running)

        val event = mockk<SensorEvent>()
        event.sensor = sensorMock
        event.timestamp = System.nanoTime()
        event.accuracy = SensorAccuracy.HIGH.value
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, floatArrayOf(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f))
        sensorEventListener.onSensorChanged(event)

        // check
        verify(exactly = 1) { bufferFilledListener.onBufferFilled(collector) }
        val slot = slot<AccelerometerSensorMeasurement>()
        verify { measurementListener.onMeasurement(collector, capture(slot), buffer.size - 1) }

        val measurement = slot.captured
        assertEquals(1.0f, measurement.ax, 0.0f)
        assertEquals(2.0f, measurement.ay, 0.0f)
        assertEquals(3.0f, measurement.az, 0.0f)
        assertEquals(4.0f, measurement.bx)
        assertEquals(5.0f, measurement.by)
        assertEquals(6.0f, measurement.bz)
        assertEquals(event.timestamp, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)

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
        val accuracyChangedListener =
            mockk<BufferedSensorCollector.OnAccuracyChangedListener<AccelerometerSensorMeasurement, BufferedAccelerometerSensorCollector>>()
        val collector = BufferedAccelerometerSensorCollector(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onAccuracyChanged(null, SensorAccuracy.HIGH.value)

        verify { accuracyChangedListener wasNot Called }
    }

    @Test
    fun onAccuracyChanged_whenUnsupportedSensorType_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val accuracyChangedListener =
            mockk<BufferedSensorCollector.OnAccuracyChangedListener<AccelerometerSensorMeasurement, BufferedAccelerometerSensorCollector>>()
        val collector = BufferedAccelerometerSensorCollector(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        val sensor = mockk<Sensor>()
        every { sensor.type }.returns(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value)
        sensorEventListener.onAccuracyChanged(sensor, SensorAccuracy.HIGH.value)

        verify { accuracyChangedListener wasNot Called }
    }

    @Test
    fun onAccuracyChanged_whenNoListener_executes() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = BufferedAccelerometerSensorCollector(context)

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        val sensor = mockk<Sensor>()
        every { sensor.type }.returns(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value)
        sensorEventListener.onAccuracyChanged(sensor, SensorAccuracy.HIGH.value)
    }

    @Test
    fun onAccuracyChanged_whenListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val accuracyChangedListener =
            mockk<BufferedSensorCollector.OnAccuracyChangedListener<AccelerometerSensorMeasurement, BufferedAccelerometerSensorCollector>>(
                relaxUnitFun = true
            )
        val collector = BufferedAccelerometerSensorCollector(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        val sensor = mockk<Sensor>()
        every { sensor.type }.returns(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value)
        sensorEventListener.onAccuracyChanged(sensor, SensorAccuracy.HIGH.value)

        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                collector,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun getMeasurementsBeforeTimestamp_whenEmptyBuffer_returnsEmptyList() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = BufferedAccelerometerSensorCollector(context)

        val buffer: ArrayDeque<AccelerometerSensorMeasurement>? =
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
        val collector = BufferedAccelerometerSensorCollector(context)

        val buffer: ArrayDeque<AccelerometerSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "buffer")
        requireNotNull(buffer)
        assertTrue(buffer.isEmpty())

        val availableMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "availableMeasurements")
        requireNotNull(availableMeasurements)
        assertEquals(BufferedSensorCollector.DEFAULT_CAPACITY, availableMeasurements.size)

        // add measurements to buffer and clear available measurements
        availableMeasurements.clear()

        val timestamp = System.nanoTime()
        val measurement1 = AccelerometerSensorMeasurement(
            1.0f,
            1.0f,
            1.0f,
            1.0f,
            1.0f,
            1.0f,
            timestamp - 1,
            SensorAccuracy.HIGH
        )
        val measurement2 = AccelerometerSensorMeasurement(
            2.0f,
            2.0f,
            2.0f,
            2.0f,
            2.0f,
            2.0f,
            timestamp,
            SensorAccuracy.HIGH
        )
        val measurement3 = AccelerometerSensorMeasurement(
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
        assertEquals(1.0f, measurements[0].ax)
        assertEquals(1.0f, measurements[0].ay)
        assertEquals(1.0f, measurements[0].az)
        assertEquals(1.0f, measurements[0].bx)
        assertEquals(1.0f, measurements[0].by)
        assertEquals(1.0f, measurements[0].bz)
        assertEquals(timestamp - 1, measurements[0].timestamp)
        assertEquals(SensorAccuracy.HIGH, measurements[0].accuracy)

        assertNotSame(measurement2, measurements[1])
        assertEquals(2.0f, measurements[1].ax)
        assertEquals(2.0f, measurements[1].ay)
        assertEquals(2.0f, measurements[1].az)
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
        val collector = BufferedAccelerometerSensorCollector(context)

        val buffer: ArrayDeque<AccelerometerSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "buffer")
        requireNotNull(buffer)
        assertTrue(buffer.isEmpty())

        val measurements = collector.getMeasurementsBeforePosition(0)
        assertTrue(measurements.isEmpty())
    }

    @Test
    fun getMeasurementsBeforePosition_whenNonEmptyBuffer_returnsExpectedResult() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = BufferedAccelerometerSensorCollector(context)

        val buffer: ArrayDeque<AccelerometerSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "buffer")
        requireNotNull(buffer)
        assertTrue(buffer.isEmpty())

        val availableMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            getPrivateProperty(BufferedSensorCollector::class, collector, "availableMeasurements")
        requireNotNull(availableMeasurements)
        assertEquals(BufferedSensorCollector.DEFAULT_CAPACITY, availableMeasurements.size)

        // add measurements to buffer and clear available measurements
        availableMeasurements.clear()

        val timestamp = System.nanoTime()
        val measurement1 = AccelerometerSensorMeasurement(
            1.0f,
            1.0f,
            1.0f,
            1.0f,
            1.0f,
            1.0f,
            timestamp - 1,
            SensorAccuracy.HIGH
        )
        val measurement2 = AccelerometerSensorMeasurement(
            2.0f,
            2.0f,
            2.0f,
            2.0f,
            2.0f,
            2.0f,
            timestamp,
            SensorAccuracy.HIGH
        )
        val measurement3 = AccelerometerSensorMeasurement(
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
        assertEquals(1.0f, measurements[0].ax)
        assertEquals(1.0f, measurements[0].ay)
        assertEquals(1.0f, measurements[0].az)
        assertEquals(1.0f, measurements[0].bx)
        assertEquals(1.0f, measurements[0].by)
        assertEquals(1.0f, measurements[0].bz)
        assertEquals(timestamp - 1, measurements[0].timestamp)
        assertEquals(SensorAccuracy.HIGH, measurements[0].accuracy)

        assertNotSame(measurement2, measurements[1])
        assertEquals(2.0f, measurements[1].ax)
        assertEquals(2.0f, measurements[1].ay)
        assertEquals(2.0f, measurements[1].az)
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