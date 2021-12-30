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
import com.irurueta.geometry.Quaternion
import com.irurueta.geometry.Rotation3D
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType
import com.irurueta.statistics.UniformRandomizer
import io.mockk.*
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class AttitudeSensorTest {

    @Test
    fun constructor_whenContext_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensor = AttitudeSensor(context)

        // check values
        assertSame(context, sensor.context)
        assertEquals(AttitudeSensor.AttitudeSensorType.ABSOLUTE_ATTITUDE, sensor.sensorType)
        assertEquals(SensorDelay.FASTEST, sensor.sensorDelay)
        assertNull(sensor.attitudeMeasurementListener)
        assertNull(sensor.attitudeAccuracyChangedListener)
        assertNull(sensor.sensor)
        assertFalse(sensor.sensorAvailable)

        val sensorManager: SensorManager? = sensor.getPrivateProperty("sensorManager")
        assertNotNull(sensorManager)
    }

    @Test
    fun constructor_whenSensorType_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensor = AttitudeSensor(context, AttitudeSensor.AttitudeSensorType.RELATIVE_ATTITUDE)

        // check values
        assertSame(context, sensor.context)
        assertEquals(AttitudeSensor.AttitudeSensorType.RELATIVE_ATTITUDE, sensor.sensorType)
        assertEquals(SensorDelay.FASTEST, sensor.sensorDelay)
        assertNull(sensor.attitudeMeasurementListener)
        assertNull(sensor.attitudeAccuracyChangedListener)
        assertNull(sensor.sensor)
        assertFalse(sensor.sensorAvailable)

        val sensorManager: SensorManager? = sensor.getPrivateProperty("sensorManager")
        assertNotNull(sensorManager)
    }

    @Test
    fun constructor_whenSensorDelay_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensor = AttitudeSensor(
            context,
            AttitudeSensor.AttitudeSensorType.RELATIVE_ATTITUDE,
            SensorDelay.NORMAL
        )

        // check values
        assertSame(context, sensor.context)
        assertEquals(AttitudeSensor.AttitudeSensorType.RELATIVE_ATTITUDE, sensor.sensorType)
        assertEquals(SensorDelay.NORMAL, sensor.sensorDelay)
        assertNull(sensor.attitudeMeasurementListener)
        assertNull(sensor.attitudeAccuracyChangedListener)
        assertNull(sensor.sensor)
        assertFalse(sensor.sensorAvailable)

        val sensorManager: SensorManager? = sensor.getPrivateProperty("sensorManager")
        assertNotNull(sensorManager)
    }

    @Test
    fun constructor_whenMeasurementListener_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val attitudeMeasurementListener = mockk<AttitudeSensor.OnAttitudeMeasurementListener>()
        val sensor = AttitudeSensor(
            context, AttitudeSensor.AttitudeSensorType.RELATIVE_ATTITUDE,
            SensorDelay.NORMAL, attitudeMeasurementListener
        )

        // check values
        assertSame(context, sensor.context)
        assertEquals(AttitudeSensor.AttitudeSensorType.RELATIVE_ATTITUDE, sensor.sensorType)
        assertEquals(SensorDelay.NORMAL, sensor.sensorDelay)
        assertSame(attitudeMeasurementListener, sensor.attitudeMeasurementListener)
        assertNull(sensor.attitudeAccuracyChangedListener)
        assertNull(sensor.sensor)
        assertFalse(sensor.sensorAvailable)

        val sensorManager: SensorManager? = sensor.getPrivateProperty("sensorManager")
        assertNotNull(sensorManager)
    }

    @Test
    fun constructor_whenAccuracyListener_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val attitudeMeasurementListener = mockk<AttitudeSensor.OnAttitudeMeasurementListener>()
        val attitudeAccuracyChangedListener =
            mockk<AttitudeSensor.OnAttitudeAccuracyChangedListener>()
        val sensor = AttitudeSensor(
            context,
            AttitudeSensor.AttitudeSensorType.RELATIVE_ATTITUDE,
            SensorDelay.NORMAL,
            attitudeMeasurementListener,
            attitudeAccuracyChangedListener
        )

        // check values
        assertSame(context, sensor.context)
        assertEquals(AttitudeSensor.AttitudeSensorType.RELATIVE_ATTITUDE, sensor.sensorType)
        assertEquals(SensorDelay.NORMAL, sensor.sensorDelay)
        assertSame(attitudeMeasurementListener, sensor.attitudeMeasurementListener)
        assertSame(attitudeAccuracyChangedListener, sensor.attitudeAccuracyChangedListener)
        assertNull(sensor.sensor)
        assertFalse(sensor.sensorAvailable)

        val sensorManager: SensorManager? = sensor.getPrivateProperty("sensorManager")
        assertNotNull(sensorManager)
    }

    @Test
    fun attitudeMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensor = AttitudeSensor(context)

        assertNull(sensor.attitudeMeasurementListener)

        // set new value
        val attitudeMeasurementListener = mockk<AttitudeSensor.OnAttitudeMeasurementListener>()
        sensor.attitudeMeasurementListener = attitudeMeasurementListener

        // check
        assertSame(attitudeMeasurementListener, sensor.attitudeMeasurementListener)
    }

    @Test
    fun attitudeAccuracyChangedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensor = AttitudeSensor(context)

        assertNull(sensor.attitudeAccuracyChangedListener)

        // set new value
        val attitudeAccuracyChangedListener =
            mockk<AttitudeSensor.OnAttitudeAccuracyChangedListener>()
        sensor.attitudeAccuracyChangedListener = attitudeAccuracyChangedListener

        // check
        assertSame(attitudeAccuracyChangedListener, sensor.attitudeAccuracyChangedListener)
    }

    @Test
    fun sensor_whenSensorTypeAbsoluteAttitude_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) }.returns(sensorMock)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val sensor = AttitudeSensor(contextSpy, AttitudeSensor.AttitudeSensorType.ABSOLUTE_ATTITUDE)

        assertSame(sensorMock, sensor.sensor)
        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) }
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

        val sensor = AttitudeSensor(contextSpy, AttitudeSensor.AttitudeSensorType.RELATIVE_ATTITUDE)

        assertSame(sensorMock, sensor.sensor)
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

        val sensor = AttitudeSensor(contextSpy, AttitudeSensor.AttitudeSensorType.ABSOLUTE_ATTITUDE)

        assertFalse(sensor.sensorAvailable)
        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) }
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

        val sensor = AttitudeSensor(contextSpy, AttitudeSensor.AttitudeSensorType.RELATIVE_ATTITUDE)

        assertFalse(sensor.sensorAvailable)
        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_GAME_ROTATION_VECTOR) }
    }

    @Test
    fun start_whenNoSensorManager_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val sensor = AttitudeSensor(contextSpy)
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
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) }.returns(null)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val sensor = AttitudeSensor(contextSpy)
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
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) }.returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val sensor = AttitudeSensor(contextSpy)
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

        val sensor = AttitudeSensor(contextSpy)
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
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) }.returns(sensorMock)
        justRun { sensorManagerSpy.unregisterListener(any(), any<Sensor>()) }
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val sensor = AttitudeSensor(contextSpy)
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
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR) }.returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val attitudeMeasurementListener = mockk<AttitudeSensor.OnAttitudeMeasurementListener>()
        val sensor =
            AttitudeSensor(contextSpy, attitudeMeasurementListener = attitudeMeasurementListener)
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

        eventListener.onSensorChanged(null)
        verify { attitudeMeasurementListener wasNot Called }
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

        val attitudeMeasurementListener = mockk<AttitudeSensor.OnAttitudeMeasurementListener>()
        val sensor =
            AttitudeSensor(contextSpy, attitudeMeasurementListener = attitudeMeasurementListener)
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

        verify { attitudeMeasurementListener wasNot Called }
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

        val sensor = AttitudeSensor(contextSpy)
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

        every { sensorMock.type }.returns(AttitudeSensor.AttitudeSensorType.ABSOLUTE_ATTITUDE.value)
        val event = mockk<SensorEvent>()
        event.sensor = sensorMock
        event.timestamp = System.nanoTime()
        val quaternion = getQuaternion()
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(
            event,
            floatArrayOf(
                quaternion.b.toFloat(),
                quaternion.c.toFloat(),
                quaternion.d.toFloat(),
                quaternion.a.toFloat(),
                -1.0f
            )
        )
        eventListener.onSensorChanged(event)

        verify(exactly = 1) { sensorMock.type }
    }

    @Test
    fun onSensorChanged_whenListenerAndNoHeadingAccuracy_notifiesEvent() {
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

        val attitudeMeasurementListener =
            mockk<AttitudeSensor.OnAttitudeMeasurementListener>(relaxUnitFun = true)
        val sensor =
            AttitudeSensor(contextSpy, attitudeMeasurementListener = attitudeMeasurementListener)
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

        every { sensorMock.type }.returns(AttitudeSensor.AttitudeSensorType.ABSOLUTE_ATTITUDE.value)
        val event = mockk<SensorEvent>()
        event.sensor = sensorMock
        event.timestamp = System.nanoTime()
        event.accuracy = SensorAccuracy.HIGH.value
        val quaternion = getQuaternion()
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(
            event,
            floatArrayOf(
                quaternion.b.toFloat(),
                quaternion.c.toFloat(),
                quaternion.d.toFloat(),
                quaternion.a.toFloat(),
                -1.0f
            )
        )
        eventListener.onSensorChanged(event)

        val rotationSlot = slot<Rotation3D>()
        val coordinateTransformationSlot = slot<CoordinateTransformation>()
        verify(exactly = 1) {
            attitudeMeasurementListener.onAttitudeMeasurement(
                capture(rotationSlot),
                capture(coordinateTransformationSlot),
                null,
                event.timestamp,
                SensorAccuracy.HIGH
            )
        }

        val rotation = rotationSlot.captured
        assertTrue(rotation.equals(quaternion, ABSOLUTE_ERROR))

        val coordinateTransformation = coordinateTransformationSlot.captured
        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, coordinateTransformation.sourceType)
        assertEquals(FrameType.BODY_FRAME, coordinateTransformation.destinationType)
        assertEquals(rotation.asInhomogeneousMatrix(), coordinateTransformation.matrix)
    }

    @Test
    fun onSensorChanged_whenListenerAndHeadingAccuracy_notifiesEvent() {
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

        val attitudeMeasurementListener =
            mockk<AttitudeSensor.OnAttitudeMeasurementListener>(relaxUnitFun = true)
        val sensor =
            AttitudeSensor(contextSpy, attitudeMeasurementListener = attitudeMeasurementListener)
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

        every { sensorMock.type }.returns(AttitudeSensor.AttitudeSensorType.ABSOLUTE_ATTITUDE.value)
        val event = mockk<SensorEvent>()
        event.sensor = sensorMock
        event.timestamp = System.nanoTime()
        event.accuracy = SensorAccuracy.HIGH.value
        val quaternion = getQuaternion()
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(
            event,
            floatArrayOf(
                quaternion.b.toFloat(),
                quaternion.c.toFloat(),
                quaternion.d.toFloat(),
                quaternion.a.toFloat(),
                HEADING_ACCURACY_RADIANS
            )
        )
        eventListener.onSensorChanged(event)

        val rotationSlot = slot<Rotation3D>()
        val coordinateTransformationSlot = slot<CoordinateTransformation>()
        verify(exactly = 1) {
            attitudeMeasurementListener.onAttitudeMeasurement(
                capture(rotationSlot),
                capture(coordinateTransformationSlot),
                HEADING_ACCURACY_RADIANS,
                event.timestamp,
                SensorAccuracy.HIGH
            )
        }

        val rotation = rotationSlot.captured
        assertTrue(rotation.equals(quaternion, ABSOLUTE_ERROR))

        val coordinateTransformation = coordinateTransformationSlot.captured
        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, coordinateTransformation.sourceType)
        assertEquals(FrameType.BODY_FRAME, coordinateTransformation.destinationType)
        assertEquals(rotation.asInhomogeneousMatrix(), coordinateTransformation.matrix)
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

        val attitudeAccuracyChangedListener =
            mockk<AttitudeSensor.OnAttitudeAccuracyChangedListener>(relaxUnitFun = true)
        val sensor = AttitudeSensor(
            contextSpy,
            attitudeAccuracyChangedListener = attitudeAccuracyChangedListener
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

        verify { attitudeAccuracyChangedListener wasNot Called }
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

        val attitudeAccuracyChangedListener =
            mockk<AttitudeSensor.OnAttitudeAccuracyChangedListener>(relaxUnitFun = true)
        val sensor = AttitudeSensor(
            contextSpy,
            attitudeAccuracyChangedListener = attitudeAccuracyChangedListener
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

        verify { attitudeAccuracyChangedListener wasNot Called }
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

        val attitudeAccuracyChangedListener =
            mockk<AttitudeSensor.OnAttitudeAccuracyChangedListener>(relaxUnitFun = true)
        val sensor = AttitudeSensor(
            contextSpy,
            attitudeAccuracyChangedListener = attitudeAccuracyChangedListener
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

        every { sensorMock.type }.returns(AttitudeSensor.AttitudeSensorType.ABSOLUTE_ATTITUDE.value)
        eventListener.onAccuracyChanged(sensorMock, SensorAccuracy.HIGH.value)

        verify(exactly = 1) {
            attitudeAccuracyChangedListener.onAttitudeAccuracyChanged(
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun attitudeSensorType_fromValue_returnsExpected() {
        assertEquals(2, AttitudeSensor.AttitudeSensorType.values().size)
        assertEquals(
            AttitudeSensor.AttitudeSensorType.ABSOLUTE_ATTITUDE,
            AttitudeSensor.AttitudeSensorType.from(Sensor.TYPE_ROTATION_VECTOR)
        )
        assertEquals(
            AttitudeSensor.AttitudeSensorType.RELATIVE_ATTITUDE,
            AttitudeSensor.AttitudeSensorType.from(Sensor.TYPE_GAME_ROTATION_VECTOR)
        )
    }

    private fun getQuaternion(): Quaternion {
        val randomizer = UniformRandomizer()

        val roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        val pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
        val yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))

        return Quaternion(roll, pitch, yaw)
    }

    companion object {
        const val MIN_ANGLE_DEGREES = -90.0

        const val MAX_ANGLE_DEGREES = 90.0

        const val ABSOLUTE_ERROR = 1e-6

        val HEADING_ACCURACY_RADIANS = Math.toRadians(5.0).toFloat()
    }
}