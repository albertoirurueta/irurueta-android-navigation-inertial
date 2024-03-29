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
import android.os.Build
import androidx.test.core.app.ApplicationProvider
import com.irurueta.android.navigation.inertial.getPrivateProperty
import com.irurueta.android.navigation.inertial.setPrivateProperty
import com.irurueta.statistics.UniformRandomizer
import io.mockk.*
import org.junit.After
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import org.robolectric.annotation.Config

@RunWith(RobolectricTestRunner::class)
class AccelerometerSensorCollector2Test {

    @After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
    }

    @Test
    fun constructor_whenRequiredParameters_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = AccelerometerSensorCollector2(context)

        // check values
        assertSame(context, collector.context)
        assertEquals(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED, collector.sensorType)
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
        val accuracyChangedListener =
            mockk<SensorCollector2.OnAccuracyChangedListener<AccelerometerSensorMeasurement, AccelerometerSensorCollector2>>()
        val measurementListener =
            mockk<SensorCollector2.OnMeasurementListener<AccelerometerSensorMeasurement, AccelerometerSensorCollector2>>()
        val collector = AccelerometerSensorCollector2(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            SensorDelay.NORMAL,
            startOffsetEnabled = false,
            accuracyChangedListener,
            measurementListener
        )

        // check values
        assertSame(context, collector.context)
        assertEquals(AccelerometerSensorType.ACCELEROMETER, collector.sensorType)
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
        val collector = AccelerometerSensorCollector2(context)

        // check default value
        assertNull(collector.accuracyChangedListener)

        // set new value
        val accuracyChangedListener =
            mockk<SensorCollector2.OnAccuracyChangedListener<AccelerometerSensorMeasurement, AccelerometerSensorCollector2>>()
        collector.accuracyChangedListener = accuracyChangedListener

        // check
        assertSame(accuracyChangedListener, collector.accuracyChangedListener)
    }

    @Test
    fun measurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = AccelerometerSensorCollector2(context)

        // check default value
        assertNull(collector.measurementListener)

        // set new value
        val measurementListener =
            mockk<SensorCollector2.OnMeasurementListener<AccelerometerSensorMeasurement, AccelerometerSensorCollector2>>()
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
            AccelerometerSensorCollector2(contextSpy, AccelerometerSensorType.ACCELEROMETER)

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

        val collector = AccelerometerSensorCollector2(
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
            AccelerometerSensorCollector2(contextSpy, AccelerometerSensorType.ACCELEROMETER)

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

        val collector = AccelerometerSensorCollector2(
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

        val collector = AccelerometerSensorCollector2(
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

        val collector = AccelerometerSensorCollector2(contextSpy)
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

        val collector = AccelerometerSensorCollector2(contextSpy)
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

        val collector = AccelerometerSensorCollector2(contextSpy)
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
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED) }
            .returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = AccelerometerSensorCollector2(contextSpy)
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
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED) }
            .returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = AccelerometerSensorCollector2(contextSpy)
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
            getPrivateProperty(SensorCollector2::class, collector, "sensorEventListener")
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

        val collector = AccelerometerSensorCollector2(contextSpy)

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
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED) }
            .returns(sensorMock)
        justRun { sensorManagerSpy.unregisterListener(any(), any<Sensor>()) }
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = AccelerometerSensorCollector2(contextSpy)

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

        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        val slot = slot<SensorEventListener>()
        verify(exactly = 1) { sensorManagerSpy.unregisterListener(capture(slot), sensorMock) }

        val eventListener = slot.captured

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SensorCollector2::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)
        assertSame(sensorEventListener, eventListener)
    }

    @Test
    fun onSensorChanged_whenNoEvent_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val measurementListener =
            mockk<SensorCollector2.OnMeasurementListener<AccelerometerSensorMeasurement, AccelerometerSensorCollector2>>()
        val collector =
            AccelerometerSensorCollector2(context, measurementListener = measurementListener)

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SensorCollector2::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onSensorChanged(null)

        // check
        verify { measurementListener wasNot Called }

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
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorMock.type }.returns(Sensor.TYPE_GYROSCOPE)
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED) }
            .returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val measurementListener =
            mockk<SensorCollector2.OnMeasurementListener<AccelerometerSensorMeasurement, AccelerometerSensorCollector2>>()
        val collector = AccelerometerSensorCollector2(
            contextSpy,
            startOffsetEnabled = false,
            measurementListener = measurementListener
        )

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SensorCollector2::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        val event = mockk<SensorEvent>()
        event.sensor = sensorMock
        sensorEventListener.onSensorChanged(event)

        // check
        verify { measurementListener wasNot Called }

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
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorMock.type }.returns(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED)
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ACCELEROMETER_UNCALIBRATED) }
            .returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val measurementListener =
            mockk<SensorCollector2.OnMeasurementListener<AccelerometerSensorMeasurement, AccelerometerSensorCollector2>>(
                relaxUnitFun = true
            )
        val collector = AccelerometerSensorCollector2(
            contextSpy,
            startOffsetEnabled = true,
            measurementListener = measurementListener
        )

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SensorCollector2::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        // set start time
        val startTimestamp = System.nanoTime()
        setPrivateProperty(
            SensorCollector2::class,
            collector,
            "startTimestamp",
            startTimestamp
        )

        assertNull(collector.startOffset)

        val event = mockk<SensorEvent>()
        event.sensor = sensorMock
        event.timestamp = System.nanoTime()
        event.accuracy = SensorManager.SENSOR_STATUS_ACCURACY_HIGH
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, floatArrayOf(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f))
        sensorEventListener.onSensorChanged(event)

        // check
        val startOffset = event.timestamp - startTimestamp
        assertNotNull(collector.startOffset)
        assertEquals(startOffset, collector.startOffset)
        val slot = slot<AccelerometerSensorMeasurement>()
        verify(exactly = 1) { measurementListener.onMeasurement(collector, capture(slot)) }

        val measurement = slot.captured
        assertEquals(1.0f, measurement.ax, 0.0f)
        assertEquals(2.0f, measurement.ay, 0.0f)
        assertEquals(3.0f, measurement.az, 0.0f)
        assertEquals(4.0f, measurement.bx)
        assertEquals(5.0f, measurement.by)
        assertEquals(6.0f, measurement.bz)
        assertEquals(event.timestamp + startOffset, measurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, measurement.accuracy)
        assertEquals(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED, measurement.sensorType)

        assertEquals(1, collector.numberOfProcessedMeasurements)
        assertEquals(measurement.timestamp, collector.mostRecentTimestamp)
    }

    @Test
    fun onAccuracyChanged_whenNoSensor_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val accuracyChangedListener =
            mockk<SensorCollector2.OnAccuracyChangedListener<AccelerometerSensorMeasurement, AccelerometerSensorCollector2>>()
        val collector = AccelerometerSensorCollector2(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SensorCollector2::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        sensorEventListener.onAccuracyChanged(null, SensorAccuracy.HIGH.value)

        verify { accuracyChangedListener wasNot Called }
    }

    @Test
    fun onAccuracyChanged_whenUnsupportedSensorType_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val accuracyChangedListener =
            mockk<SensorCollector2.OnAccuracyChangedListener<AccelerometerSensorMeasurement, AccelerometerSensorCollector2>>()
        val collector = AccelerometerSensorCollector2(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SensorCollector2::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        val sensor = mockk<Sensor>()
        every { sensor.type }.returns(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value)
        sensorEventListener.onAccuracyChanged(sensor, SensorAccuracy.HIGH.value)

        verify { accuracyChangedListener wasNot Called }
    }

    @Test
    fun onAccuracyChanged_whenNoListener_executes() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = AccelerometerSensorCollector2(context)

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SensorCollector2::class, collector, "sensorEventListener")
        requireNotNull(sensorEventListener)

        val sensor = mockk<Sensor>()
        every { sensor.type }.returns(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value)
        sensorEventListener.onAccuracyChanged(sensor, SensorAccuracy.HIGH.value)
    }

    @Test
    fun onAccuracyChanged_whenListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val accuracyChangedListener =
            mockk<SensorCollector2.OnAccuracyChangedListener<AccelerometerSensorMeasurement, AccelerometerSensorCollector2>>(
                relaxUnitFun = true
            )
        val collector = AccelerometerSensorCollector2(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val sensorEventListener: SensorEventListener? =
            getPrivateProperty(SensorCollector2::class, collector, "sensorEventListener")
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
}