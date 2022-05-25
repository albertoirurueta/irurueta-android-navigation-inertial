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
import com.irurueta.android.navigation.inertial.getPrivateProperty
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.statistics.UniformRandomizer
import io.mockk.*
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class AttitudeSensorCollectorTest {

    @Test
    fun constructor_whenContext_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = AttitudeSensorCollector(context)

        // check values
        assertSame(context, collector.context)
        assertEquals(
            AttitudeSensorCollector.SensorType.ABSOLUTE_ATTITUDE,
            collector.sensorType
        )
        assertEquals(SensorDelay.FASTEST, collector.sensorDelay)
        assertFalse(collector.estimateCoordinateTransformation)
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
        val collector = AttitudeSensorCollector(
            context,
            AttitudeSensorCollector.SensorType.GEOMAGNETIC_ABSOLUTE_ATTITUDE
        )

        // check values
        assertSame(context, collector.context)
        assertEquals(
            AttitudeSensorCollector.SensorType.GEOMAGNETIC_ABSOLUTE_ATTITUDE,
            collector.sensorType
        )
        assertEquals(SensorDelay.FASTEST, collector.sensorDelay)
        assertFalse(collector.estimateCoordinateTransformation)
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
        val collector = AttitudeSensorCollector(
            context,
            AttitudeSensorCollector.SensorType.GEOMAGNETIC_ABSOLUTE_ATTITUDE,
            SensorDelay.NORMAL
        )

        // check values
        assertSame(context, collector.context)
        assertEquals(
            AttitudeSensorCollector.SensorType.GEOMAGNETIC_ABSOLUTE_ATTITUDE,
            collector.sensorType
        )
        assertEquals(SensorDelay.NORMAL, collector.sensorDelay)
        assertFalse(collector.estimateCoordinateTransformation)
        assertNull(collector.measurementListener)
        assertNull(collector.accuracyChangedListener)
        assertNull(collector.sensor)
        assertFalse(collector.sensorAvailable)

        val sensorManager: SensorManager? =
            getPrivateProperty(SensorCollector::class, collector, "sensorManager")
        assertNotNull(sensorManager)
    }

    @Test
    fun constructor_whenEstimateCoordinateTransformation_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = AttitudeSensorCollector(
            context,
            AttitudeSensorCollector.SensorType.GEOMAGNETIC_ABSOLUTE_ATTITUDE,
            SensorDelay.NORMAL,
            estimateCoordinateTransformation = true
        )

        // check values
        assertSame(context, collector.context)
        assertEquals(
            AttitudeSensorCollector.SensorType.GEOMAGNETIC_ABSOLUTE_ATTITUDE,
            collector.sensorType
        )
        assertEquals(SensorDelay.NORMAL, collector.sensorDelay)
        assertTrue(collector.estimateCoordinateTransformation)
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
        val measurementListener = mockk<AttitudeSensorCollector.OnMeasurementListener>()
        val collector = AttitudeSensorCollector(
            context,
            AttitudeSensorCollector.SensorType.GEOMAGNETIC_ABSOLUTE_ATTITUDE,
            SensorDelay.NORMAL,
            estimateCoordinateTransformation = true,
            measurementListener
        )

        // check values
        assertSame(context, collector.context)
        assertEquals(
            AttitudeSensorCollector.SensorType.GEOMAGNETIC_ABSOLUTE_ATTITUDE,
            collector.sensorType
        )
        assertEquals(SensorDelay.NORMAL, collector.sensorDelay)
        assertTrue(collector.estimateCoordinateTransformation)
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
        val measurementListener = mockk<AttitudeSensorCollector.OnMeasurementListener>()
        val accuracyChangedListener =
            mockk<SensorCollector.OnAccuracyChangedListener>()
        val collector = AttitudeSensorCollector(
            context,
            AttitudeSensorCollector.SensorType.GEOMAGNETIC_ABSOLUTE_ATTITUDE,
            SensorDelay.NORMAL,
            estimateCoordinateTransformation = true,
            measurementListener,
            accuracyChangedListener
        )

        // check values
        assertSame(context, collector.context)
        assertEquals(
            AttitudeSensorCollector.SensorType.GEOMAGNETIC_ABSOLUTE_ATTITUDE,
            collector.sensorType
        )
        assertEquals(SensorDelay.NORMAL, collector.sensorDelay)
        assertTrue(collector.estimateCoordinateTransformation)
        assertSame(measurementListener, collector.measurementListener)
        assertSame(
            accuracyChangedListener,
            collector.accuracyChangedListener
        )
        assertNull(collector.sensor)
        assertFalse(collector.sensorAvailable)

        val sensorManager: SensorManager? =
            getPrivateProperty(SensorCollector::class, collector, "sensorManager")
        assertNotNull(sensorManager)
    }

    @Test
    fun estimateCoordinateTransformation_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = AttitudeSensorCollector(context)

        // check default value
        assertFalse(collector.estimateCoordinateTransformation)

        // set new value
        collector.estimateCoordinateTransformation = true

        // check
        assertTrue(collector.estimateCoordinateTransformation)
    }

    @Test
    fun measurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = AttitudeSensorCollector(context)

        // check default value
        assertNull(collector.measurementListener)

        // set new value
        val measurementListener = mockk<AttitudeSensorCollector.OnMeasurementListener>()
        collector.measurementListener = measurementListener

        // check
        assertSame(measurementListener, collector.measurementListener)
    }

    @Test
    fun accuracyChangedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = AttitudeSensorCollector(context)

        // check default value
        assertNull(collector.accuracyChangedListener)

        // set new value
        val accuracyChangedListener = mockk<SensorCollector.OnAccuracyChangedListener>()
        collector.accuracyChangedListener = accuracyChangedListener

        // check
        assertSame(accuracyChangedListener, collector.accuracyChangedListener)
    }

    @Test
    fun sensor_whenSensorTypeRelativeAttitude_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR) }.returns(
            sensorMock
        )
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = AttitudeSensorCollector(
            contextSpy,
            AttitudeSensorCollector.SensorType.RELATIVE_ATTITUdE
        )

        assertSame(sensorMock, collector.sensor)
        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR) }
    }

    @Test
    fun sensor_whenSensorTypeAbsoluteAttitude_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) }.returns(
            sensorMock
        )
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = AttitudeSensorCollector(
            contextSpy,
            AttitudeSensorCollector.SensorType.ABSOLUTE_ATTITUDE
        )

        assertSame(sensorMock, collector.sensor)
        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) }
    }

    @Test
    fun sensor_whenSensorTypeGeomagneticAbsoluteAttitude_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GEOMAGNETIC_ROTATION_VECTOR) }.returns(
            sensorMock
        )
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = AttitudeSensorCollector(
            contextSpy,
            AttitudeSensorCollector.SensorType.GEOMAGNETIC_ABSOLUTE_ATTITUDE
        )

        assertSame(sensorMock, collector.sensor)
        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GEOMAGNETIC_ROTATION_VECTOR) }
    }

    @Test
    fun sensorAvailable_whenSensorTypeRelativeAttitude_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = AttitudeSensorCollector(
            contextSpy,
            AttitudeSensorCollector.SensorType.RELATIVE_ATTITUdE
        )

        assertFalse(collector.sensorAvailable)
        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR) }
    }

    @Test
    fun sensorAvailable_whenSensorTypeAbsoluteAttitude_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = AttitudeSensorCollector(
            contextSpy,
            AttitudeSensorCollector.SensorType.ABSOLUTE_ATTITUDE
        )

        assertFalse(collector.sensorAvailable)
        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) }
    }

    @Test
    fun sensorAvailable_whenSensorTypeGeomagneticAbsoluteAttitude_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = AttitudeSensorCollector(
            contextSpy,
            AttitudeSensorCollector.SensorType.GEOMAGNETIC_ABSOLUTE_ATTITUDE
        )

        assertFalse(collector.sensorAvailable)
        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GEOMAGNETIC_ROTATION_VECTOR) }
    }

    @Test
    fun start_whenNoSensorManager_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = AttitudeSensorCollector(contextSpy)
        assertFalse(collector.start())

        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
    }

    @Test
    fun start_whenNoSensor_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) }.returns(null)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = AttitudeSensorCollector(contextSpy)
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) }.returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = AttitudeSensorCollector(contextSpy)
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
        assertNotNull(eventListener)
    }

    @Test
    fun stop_whenNoSensorManager_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = AttitudeSensorCollector(contextSpy)
        collector.stop()
    }

    @Test
    fun stop_whenSensorManager_unregistersListener() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) }.returns(sensorMock)
        justRun { sensorManagerSpy.unregisterListener(any(), any<Sensor>()) }
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = AttitudeSensorCollector(contextSpy)
        collector.stop()

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
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) }.returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val measurementListener = mockk<AttitudeSensorCollector.OnMeasurementListener>()
        val collector =
            AttitudeSensorCollector(contextSpy, measurementListener = measurementListener)
        assertTrue(collector.start())

        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(
                capture(slot),
                sensorMock,
                collector.sensorDelay.value
            )
        }

        val eventListener = slot.captured

        eventListener.onSensorChanged(null)
        verify { measurementListener wasNot Called }
    }

    @Test
    fun onSensorChanged_whenNoSensorType_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) }.returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val measurementListener = mockk<AttitudeSensorCollector.OnMeasurementListener>()
        val collector =
            AttitudeSensorCollector(contextSpy, measurementListener = measurementListener)
        assertTrue(collector.start())

        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(
                capture(slot),
                sensorMock,
                collector.sensorDelay.value
            )
        }

        val eventListener = slot.captured

        every { sensorMock.type }.returns(-1)
        val constructor = SensorEvent::class.java.getConstructor(Integer.TYPE)
        val event = constructor.newInstance(5)
        event.sensor = sensorMock
        eventListener.onSensorChanged(event)

        verify { measurementListener wasNot Called }
    }

    @Test
    fun onSensorChanged_whenNoListener_doesNotNotify() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) }.returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = AttitudeSensorCollector(contextSpy)
        assertTrue(collector.start())

        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(
                capture(slot),
                sensorMock,
                collector.sensorDelay.value
            )
        }

        val eventListener = slot.captured

        every { sensorMock.type }.returns(-1)
        val constructor = SensorEvent::class.java.getConstructor(Integer.TYPE)
        val event = constructor.newInstance(5)
        event.sensor = sensorMock
        event.timestamp = System.nanoTime()
        eventListener.onSensorChanged(event)

        verify(exactly = 1) { sensorMock.type }
    }

    @Test
    fun onSensorChanged_whenListenerRelativeAttitude_notifiesEvent() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR) }.returns(
            sensorMock
        )
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val measurementListener =
            mockk<AttitudeSensorCollector.OnMeasurementListener>(relaxUnitFun = true)
        val collector =
            AttitudeSensorCollector(
                contextSpy,
                AttitudeSensorCollector.SensorType.RELATIVE_ATTITUdE,
                measurementListener = measurementListener
            )
        assertTrue(collector.start())

        val slotEventListener = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(
                capture(slotEventListener),
                sensorMock,
                collector.sensorDelay.value
            )
        }

        val eventListener = slotEventListener.captured

        every { sensorMock.type }.returns(AttitudeSensorCollector.SensorType.RELATIVE_ATTITUdE.value)
        val event = mockk<SensorEvent>()
        event.sensor = sensorMock
        event.timestamp = System.nanoTime()
        event.accuracy = SensorAccuracy.HIGH.value
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true

        val attitude = createQuaternion()
        val a = attitude.a.toFloat()
        val b = attitude.b.toFloat()
        val c = attitude.c.toFloat()
        val d = attitude.d.toFloat()

        val randomizer = UniformRandomizer()
        val accuracy = Math.toRadians(randomizer.nextDouble()).toFloat()
        val values = floatArrayOf(b, c, d, a, accuracy)
        valuesField.set(event, values)
        eventListener.onSensorChanged(event)

        verify(exactly = 1) { sensorMock.type }
        val slot = slot<Quaternion>()
        verify(exactly = 1) {
            measurementListener.onMeasurement(
                capture(slot),
                null,
                accuracy.toDouble(),
                event.timestamp,
                SensorAccuracy.HIGH
            )
        }

        val quaternion = slot.captured
        assertTrue(attitude.equals(quaternion.conjugateAndReturnNew(), ABSOLUTE_ERROR))
    }

    @Test
    fun onSensorChanged_whenListenerAbsoluteAttitude_notifiesEvent() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) }.returns(
            sensorMock
        )
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val measurementListener =
            mockk<AttitudeSensorCollector.OnMeasurementListener>(relaxUnitFun = true)
        val collector =
            AttitudeSensorCollector(
                contextSpy,
                AttitudeSensorCollector.SensorType.ABSOLUTE_ATTITUDE,
                measurementListener = measurementListener
            )
        assertTrue(collector.start())

        val slotEventListener = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(
                capture(slotEventListener),
                sensorMock,
                collector.sensorDelay.value
            )
        }

        val eventListener = slotEventListener.captured

        every { sensorMock.type }.returns(AttitudeSensorCollector.SensorType.ABSOLUTE_ATTITUDE.value)
        val event = mockk<SensorEvent>()
        event.sensor = sensorMock
        event.timestamp = System.nanoTime()
        event.accuracy = SensorAccuracy.HIGH.value
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true

        val attitude = createQuaternion()
        val a = attitude.a.toFloat()
        val b = attitude.b.toFloat()
        val c = attitude.c.toFloat()
        val d = attitude.d.toFloat()

        val randomizer = UniformRandomizer()
        val accuracy = Math.toRadians(randomizer.nextDouble()).toFloat()
        val values = floatArrayOf(b, c, d, a, accuracy)
        valuesField.set(event, values)
        eventListener.onSensorChanged(event)

        verify(exactly = 1) { sensorMock.type }
        val slot = slot<Quaternion>()
        verify(exactly = 1) {
            measurementListener.onMeasurement(
                capture(slot),
                null,
                accuracy.toDouble(),
                event.timestamp,
                SensorAccuracy.HIGH
            )
        }

        val quaternion = slot.captured
        assertTrue(attitude.equals(quaternion.conjugateAndReturnNew(), ABSOLUTE_ERROR))
    }

    @Test
    fun onSensorChanged_whenListenerGeomagneticAbsoluteAttitude_notifiesEvent() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GEOMAGNETIC_ROTATION_VECTOR) }.returns(
            sensorMock
        )
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val measurementListener =
            mockk<AttitudeSensorCollector.OnMeasurementListener>(relaxUnitFun = true)
        val collector =
            AttitudeSensorCollector(
                contextSpy,
                AttitudeSensorCollector.SensorType.GEOMAGNETIC_ABSOLUTE_ATTITUDE,
                measurementListener = measurementListener
            )
        assertTrue(collector.start())

        val slotEventListener = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(
                capture(slotEventListener),
                sensorMock,
                collector.sensorDelay.value
            )
        }

        val eventListener = slotEventListener.captured

        every { sensorMock.type }.returns(AttitudeSensorCollector.SensorType.GEOMAGNETIC_ABSOLUTE_ATTITUDE.value)
        val event = mockk<SensorEvent>()
        event.sensor = sensorMock
        event.timestamp = System.nanoTime()
        event.accuracy = SensorAccuracy.HIGH.value
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true

        val attitude = createQuaternion()
        val a = attitude.a.toFloat()
        val b = attitude.b.toFloat()
        val c = attitude.c.toFloat()
        val d = attitude.d.toFloat()

        val randomizer = UniformRandomizer()
        val accuracy = Math.toRadians(randomizer.nextDouble()).toFloat()
        val values = floatArrayOf(b, c, d, a, accuracy)
        valuesField.set(event, values)
        eventListener.onSensorChanged(event)

        verify(exactly = 1) { sensorMock.type }
        val slot = slot<Quaternion>()
        verify(exactly = 1) {
            measurementListener.onMeasurement(
                capture(slot),
                null,
                accuracy.toDouble(),
                event.timestamp,
                SensorAccuracy.HIGH
            )
        }

        val quaternion = slot.captured
        assertTrue(attitude.equals(quaternion.conjugateAndReturnNew(), ABSOLUTE_ERROR))
    }

    @Test
    fun onSensorChanged_whenListenerAndEstimateCoordinateTransformation_notifiesEvent() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) }.returns(
            sensorMock
        )
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val measurementListener =
            mockk<AttitudeSensorCollector.OnMeasurementListener>(relaxUnitFun = true)
        val collector =
            AttitudeSensorCollector(
                contextSpy,
                AttitudeSensorCollector.SensorType.ABSOLUTE_ATTITUDE,
                estimateCoordinateTransformation = true,
                measurementListener = measurementListener
            )
        assertTrue(collector.start())

        val slotEventListener = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(
                capture(slotEventListener),
                sensorMock,
                collector.sensorDelay.value
            )
        }

        val eventListener = slotEventListener.captured

        every { sensorMock.type }.returns(AttitudeSensorCollector.SensorType.ABSOLUTE_ATTITUDE.value)
        val event = mockk<SensorEvent>()
        event.sensor = sensorMock
        event.timestamp = System.nanoTime()
        event.accuracy = SensorAccuracy.HIGH.value
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true

        val attitude = createQuaternion()
        val a = attitude.a.toFloat()
        val b = attitude.b.toFloat()
        val c = attitude.c.toFloat()
        val d = attitude.d.toFloat()

        val randomizer = UniformRandomizer()
        val accuracy = Math.toRadians(randomizer.nextDouble()).toFloat()
        val values = floatArrayOf(b, c, d, a, accuracy)
        valuesField.set(event, values)
        eventListener.onSensorChanged(event)

        verify(exactly = 1) { sensorMock.type }
        val slotQuaternion = slot<Quaternion>()
        val slotTransformation = slot<CoordinateTransformation>()
        verify(exactly = 1) {
            measurementListener.onMeasurement(
                capture(slotQuaternion),
                capture(slotTransformation),
                accuracy.toDouble(),
                event.timestamp,
                SensorAccuracy.HIGH
            )
        }

        val quaternion1 = slotQuaternion.captured
        assertTrue(attitude.equals(quaternion1.conjugateAndReturnNew(), ABSOLUTE_ERROR))
        val transformation = slotTransformation.captured
        val quaternion2 = Quaternion()
        transformation.asRotation(quaternion2)

        assertEquals(quaternion1, quaternion2)
    }

    @Test
    fun onAccuracyChanged_whenNoSensor_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) }.returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val accuracyChangedListener =
            mockk<SensorCollector.OnAccuracyChangedListener>(relaxUnitFun = true)
        val collector = AttitudeSensorCollector(
            contextSpy,
            accuracyChangedListener = accuracyChangedListener
        )
        assertTrue(collector.start())

        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(
                capture(slot),
                sensorMock,
                collector.sensorDelay.value
            )
        }

        val eventListener = slot.captured

        eventListener.onAccuracyChanged(null, SensorAccuracy.HIGH.value)

        verify { accuracyChangedListener wasNot Called }
    }

    @Test
    fun onAccuracyChanged_whenNoSensorType_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) }.returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val accuracyChangedListener =
            mockk<SensorCollector.OnAccuracyChangedListener>(relaxUnitFun = true)
        val collector = AttitudeSensorCollector(
            contextSpy,
            accuracyChangedListener = accuracyChangedListener
        )
        assertTrue(collector.start())

        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(
                capture(slot),
                sensorMock,
                collector.sensorDelay.value
            )
        }

        val eventListener = slot.captured

        every { sensorMock.type }.returns(-1)
        eventListener.onAccuracyChanged(null, SensorAccuracy.HIGH.value)

        verify { accuracyChangedListener wasNot Called }
    }

    @Test
    fun onAccuracyChanged_whenListener_notifiesEvent() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) }.returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val accuracyChangedListener =
            mockk<SensorCollector.OnAccuracyChangedListener>(relaxUnitFun = true)
        val collector = AttitudeSensorCollector(
            contextSpy,
            accuracyChangedListener = accuracyChangedListener
        )
        assertTrue(collector.start())

        val slot = slot<SensorEventListener>()
        verify(exactly = 1) {
            sensorManagerSpy.registerListener(
                capture(slot),
                sensorMock,
                collector.sensorDelay.value
            )
        }

        val eventListener = slot.captured

        every { sensorMock.type }.returns(AttitudeSensorCollector.SensorType.ABSOLUTE_ATTITUDE.value)
        eventListener.onAccuracyChanged(sensorMock, SensorAccuracy.HIGH.value)

        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(SensorAccuracy.HIGH)
        }
    }

    @Test
    fun sensorType_fromValue_returnsExpected() {
        assertEquals(3, AttitudeSensorCollector.SensorType.values().size)
        assertEquals(
            AttitudeSensorCollector.SensorType.RELATIVE_ATTITUdE,
            AttitudeSensorCollector.SensorType.from(Sensor.TYPE_GAME_ROTATION_VECTOR)
        )
        assertEquals(
            AttitudeSensorCollector.SensorType.ABSOLUTE_ATTITUDE,
            AttitudeSensorCollector.SensorType.from(Sensor.TYPE_ROTATION_VECTOR)
        )
        assertEquals(
            AttitudeSensorCollector.SensorType.GEOMAGNETIC_ABSOLUTE_ATTITUDE,
            AttitudeSensorCollector.SensorType.from(Sensor.TYPE_GEOMAGNETIC_ROTATION_VECTOR)
        )
    }

    private companion object {
        const val ABSOLUTE_ERROR = 1e-6

        const val MIN_ANGLE_DEGREES = -90.0
        const val MAX_ANGLE_DEGREES = 90.0

        fun createQuaternion(): Quaternion {
            val randomizer = UniformRandomizer()
            val roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            return Quaternion(roll, pitch, yaw)
        }
    }

}