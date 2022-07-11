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
import android.location.Location
import android.view.Display
import android.view.Surface
import androidx.test.core.app.ApplicationProvider
import com.irurueta.android.navigation.inertial.getPrivateProperty
import com.irurueta.android.navigation.inertial.toNEDPosition
import com.irurueta.geometry.EuclideanTransformation3D
import com.irurueta.geometry.InhomogeneousPoint3D
import com.irurueta.geometry.Point3D
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.*
import com.irurueta.navigation.frames.converters.ECEFtoNEDPositionVelocityConverter
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter
import com.irurueta.statistics.UniformRandomizer
import io.mockk.*
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class RelativeDevicePoseSensorCollectorTest {

    @Test
    fun constructor_whenContext_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = RelativeDevicePoseSensorCollector(context)

        // check values
        assertSame(context, collector.context)
        assertEquals(SensorDelay.FASTEST, collector.sensorDelay)
        assertEquals(0.0, collector.timeInterval, 0.0)
        assertNull(collector.measurementListener)
        assertNull(collector.accuracyChangedListener)
        assertNull(collector.location)
        assertNull(collector.sensor)
        assertFalse(collector.sensorAvailable)

        val sensorManager: SensorManager? =
            getPrivateProperty(SensorCollector::class, collector, "sensorManager")
        assertNotNull(sensorManager)
        assertNull(collector.getPrivateProperty("startPosition"))
    }

    @Test
    fun constructor_whenSensorDelay_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = RelativeDevicePoseSensorCollector(
            context,
            SensorDelay.NORMAL
        )

        // check values
        assertSame(context, collector.context)
        assertEquals(SensorDelay.NORMAL, collector.sensorDelay)
        assertEquals(0.0, collector.timeInterval, 0.0)
        assertNull(collector.measurementListener)
        assertNull(collector.accuracyChangedListener)
        assertNull(collector.location)
        assertNull(collector.sensor)
        assertFalse(collector.sensorAvailable)

        val sensorManager: SensorManager? =
            getPrivateProperty(SensorCollector::class, collector, "sensorManager")
        assertNotNull(sensorManager)
        assertNull(collector.getPrivateProperty("startPosition"))
    }

    @Test
    fun constructor_whenLocation_setsExpectedValues() {
        val location = createLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = RelativeDevicePoseSensorCollector(
            context,
            SensorDelay.NORMAL,
            location
        )

        // check values
        assertSame(context, collector.context)
        assertEquals(SensorDelay.NORMAL, collector.sensorDelay)
        assertEquals(0.0, collector.timeInterval, 0.0)
        assertNull(collector.measurementListener)
        assertNull(collector.accuracyChangedListener)
        assertSame(location, collector.location)
        assertNull(collector.sensor)
        assertFalse(collector.sensorAvailable)

        val sensorManager: SensorManager? =
            getPrivateProperty(SensorCollector::class, collector, "sensorManager")
        assertNotNull(sensorManager)
        val startPosition: NEDPosition? = collector.getPrivateProperty("startPosition")
        requireNotNull(startPosition)
        assertEquals(startPosition, location.toNEDPosition())
    }

    @Test
    fun constructor_whenTimeInterval_setsExpectedValues() {
        val location = createLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = RelativeDevicePoseSensorCollector(
            context,
            SensorDelay.NORMAL,
            location,
            TIME_INTERVAL
        )

        // check values
        assertSame(context, collector.context)
        assertEquals(SensorDelay.NORMAL, collector.sensorDelay)
        assertEquals(TIME_INTERVAL, collector.timeInterval, 0.0)
        assertNull(collector.measurementListener)
        assertNull(collector.accuracyChangedListener)
        assertSame(location, collector.location)
        assertNull(collector.sensor)
        assertFalse(collector.sensorAvailable)

        val sensorManager: SensorManager? =
            getPrivateProperty(SensorCollector::class, collector, "sensorManager")
        assertNotNull(sensorManager)
        val startPosition: NEDPosition? = collector.getPrivateProperty("startPosition")
        requireNotNull(startPosition)
        assertEquals(startPosition, location.toNEDPosition())
    }

    @Test
    fun constructor_whenMeasurementListener_setsExpectedValues() {
        val location = createLocation()
        val measurementListener = mockk<RelativeDevicePoseSensorCollector.OnMeasurementListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = RelativeDevicePoseSensorCollector(
            context,
            SensorDelay.NORMAL,
            location,
            TIME_INTERVAL,
            measurementListener
        )

        // check values
        assertSame(context, collector.context)
        assertEquals(SensorDelay.NORMAL, collector.sensorDelay)
        assertEquals(TIME_INTERVAL, collector.timeInterval, 0.0)
        assertSame(measurementListener, collector.measurementListener)
        assertNull(collector.accuracyChangedListener)
        assertSame(location, collector.location)
        assertNull(collector.sensor)
        assertFalse(collector.sensorAvailable)

        val sensorManager: SensorManager? =
            getPrivateProperty(SensorCollector::class, collector, "sensorManager")
        assertNotNull(sensorManager)
        val startPosition: NEDPosition? = collector.getPrivateProperty("startPosition")
        requireNotNull(startPosition)
        assertEquals(startPosition, location.toNEDPosition())
    }

    @Test
    fun constructor_whenAccuracyChangedListener_setsExpectedValues() {
        val location = createLocation()
        val measurementListener = mockk<RelativeDevicePoseSensorCollector.OnMeasurementListener>()
        val accuracyChangedListener = mockk<SensorCollector.OnAccuracyChangedListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = RelativeDevicePoseSensorCollector(
            context,
            SensorDelay.NORMAL,
            location,
            TIME_INTERVAL,
            measurementListener,
            accuracyChangedListener
        )

        // check values
        assertSame(context, collector.context)
        assertEquals(SensorDelay.NORMAL, collector.sensorDelay)
        assertEquals(TIME_INTERVAL, collector.timeInterval, 0.0)
        assertSame(measurementListener, collector.measurementListener)
        assertSame(accuracyChangedListener, collector.accuracyChangedListener)
        assertSame(location, collector.location)
        assertNull(collector.sensor)
        assertFalse(collector.sensorAvailable)

        val sensorManager: SensorManager? =
            getPrivateProperty(SensorCollector::class, collector, "sensorManager")
        assertNotNull(sensorManager)
        val startPosition: NEDPosition? = collector.getPrivateProperty("startPosition")
        requireNotNull(startPosition)
        assertEquals(startPosition, location.toNEDPosition())
    }

    @Test
    fun timeInterval_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = RelativeDevicePoseSensorCollector(context)

        // check default value
        assertEquals(0.0, collector.timeInterval, 0.0)

        // set new value
        collector.timeInterval = TIME_INTERVAL

        // check
        assertEquals(TIME_INTERVAL, collector.timeInterval, 0.0)
    }

    @Test
    fun measurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = RelativeDevicePoseSensorCollector(context)

        // check default value
        assertNull(collector.measurementListener)

        // set new value
        val measurementListener = mockk<RelativeDevicePoseSensorCollector.OnMeasurementListener>()
        collector.measurementListener = measurementListener

        // check
        assertSame(measurementListener, collector.measurementListener)
    }

    @Test
    fun accuracyChangedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = RelativeDevicePoseSensorCollector(context)

        // check default value
        assertNull(collector.accuracyChangedListener)

        // set new value
        val accuracyChangedListener = mockk<SensorCollector.OnAccuracyChangedListener>()
        collector.accuracyChangedListener = accuracyChangedListener

        // check
        assertSame(accuracyChangedListener, collector.accuracyChangedListener)
    }

    @Test
    fun location_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val collector = RelativeDevicePoseSensorCollector(context)

        // check default value
        assertNull(collector.location)
        assertNull(collector.getPrivateProperty("startPosition"))

        // set new value
        val location = createLocation()
        collector.location = location

        // check
        assertSame(location, collector.location)
        val startPosition: NEDPosition? = collector.getPrivateProperty("startPosition")
        requireNotNull(startPosition)
        assertEquals(startPosition, location.toNEDPosition())

        // set as null
        collector.location = null

        // check
        assertNull(collector.location)
        assertNull(collector.getPrivateProperty("startPosition"))
    }

    @Test
    fun sensor_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_POSE_6DOF) }.returns(sensorMock)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = RelativeDevicePoseSensorCollector(contextSpy)

        assertSame(sensorMock, collector.sensor)
        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_POSE_6DOF) }
    }

    @Test
    fun sensorAvailable_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_POSE_6DOF) }.returns(sensorMock)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = RelativeDevicePoseSensorCollector(contextSpy)

        assertTrue(collector.sensorAvailable)
        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_POSE_6DOF) }
    }

    @Test
    fun start_whenNoSensorManager_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val collector = RelativeDevicePoseSensorCollector(contextSpy)
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
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_POSE_6DOF) }.returns(null)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = RelativeDevicePoseSensorCollector(contextSpy)
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
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_POSE_6DOF) }.returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = RelativeDevicePoseSensorCollector(contextSpy)
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

        val collector = RelativeDevicePoseSensorCollector(contextSpy)
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
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_POSE_6DOF) }.returns(sensorMock)
        justRun { sensorManagerSpy.unregisterListener(any(), any<Sensor>()) }
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = RelativeDevicePoseSensorCollector(contextSpy)
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
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_POSE_6DOF) }.returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val measurementListener = mockk<RelativeDevicePoseSensorCollector.OnMeasurementListener>()
        val collector =
            RelativeDevicePoseSensorCollector(contextSpy, measurementListener = measurementListener)
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
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_POSE_6DOF) }.returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val measurementListener = mockk<RelativeDevicePoseSensorCollector.OnMeasurementListener>()
        val collector =
            RelativeDevicePoseSensorCollector(contextSpy, measurementListener = measurementListener)
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
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_POSE_6DOF) }.returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val collector = RelativeDevicePoseSensorCollector(contextSpy)
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
    fun onSensorChanged_whenListenerAndLocation_notifiesEvent() {
        val location = createLocation()
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_0)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)

        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_POSE_6DOF) }.returns(
            sensorMock
        )
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val measurementListener =
            mockk<RelativeDevicePoseSensorCollector.OnMeasurementListener>(relaxUnitFun = true)
        val collector =
            RelativeDevicePoseSensorCollector(
                context,
                location = location,
                measurementListener = measurementListener,
                timeInterval = TIME_INTERVAL
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

        every { sensorMock.type }.returns(Sensor.TYPE_POSE_6DOF)
        val event = mockk<SensorEvent>()
        event.sensor = sensorMock
        event.timestamp = System.nanoTime()
        event.accuracy = SensorAccuracy.HIGH.value
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true

        val startNedPosition = location.toNEDPosition()
        val startEcefPosition = ECEFPosition()
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
            startNedPosition,
            NEDVelocity(),
            startEcefPosition,
            ECEFVelocity()
        )

        val attitude1 = createQuaternion()
        val attitude2 = createQuaternion()
        val invAttitude1 = attitude1.inverseAndReturnNew()
        val deltaAttitude = invAttitude1.combineAndReturnNew(attitude2)

        val translation1 = createTranslation()
        val translation2 = createTranslation()
        val deltaTranslation = InhomogeneousPoint3D(
            translation2.inhomX - translation1.inhomX,
            translation2.inhomY - translation1.inhomY,
            translation2.inhomZ - translation1.inhomZ
        )

        val transformation2 = EuclideanTransformation3D(attitude2, translation2.asArray())

        val a2 = attitude2.a.toFloat()
        val b2 = attitude2.b.toFloat()
        val c2 = attitude2.c.toFloat()
        val d2 = attitude2.d.toFloat()

        val x2 = translation2.inhomX.toFloat()
        val y2 = translation2.inhomY.toFloat()
        val z2 = translation2.inhomZ.toFloat()

        val aDelta = deltaAttitude.a.toFloat()
        val bDelta = deltaAttitude.b.toFloat()
        val cDelta = deltaAttitude.c.toFloat()
        val dDelta = deltaAttitude.d.toFloat()

        val xDelta = deltaTranslation.inhomX.toFloat()
        val yDelta = deltaTranslation.inhomY.toFloat()
        val zDelta = deltaTranslation.inhomZ.toFloat()

        val values = floatArrayOf(
            b2,
            c2,
            d2,
            a2,
            x2,
            y2,
            z2,
            bDelta,
            cDelta,
            dDelta,
            aDelta,
            xDelta,
            yDelta,
            zDelta,
            SEQUENCE_NUMBER
        )

        valuesField.set(event, values)
        eventListener.onSensorChanged(event)

        verify(exactly = 1) { sensorMock.type }
        val quaternionSlot = slot<Quaternion>()
        val translationSlot = slot<Point3D>()
        val frameSlot = slot<NEDFrame>()
        val transformationSlot = slot<EuclideanTransformation3D>()
        verify(exactly = 1) {
            measurementListener.onMeasurement(
                capture(quaternionSlot),
                capture(translationSlot),
                capture(frameSlot),
                capture(transformationSlot),
                event.timestamp,
                SensorAccuracy.HIGH
            )
        }

        val quaternion = quaternionSlot.captured
        val translation = translationSlot.captured
        val frame = frameSlot.captured
        val transformation = transformationSlot.captured
        assertEquals(quaternion, transformation.rotation)
        assertEquals(translation, transformation.translationPoint)
        assertTrue(
            transformation.asMatrix()
                .equals(transformation2.inverseAndReturnNew().asMatrix(), ABSOLUTE_ERROR)
        )
        val endEcefPosition = ECEFPosition(
            startEcefPosition.x + translation.inhomX,
            startEcefPosition.y + translation.inhomY,
            startEcefPosition.z + translation.inhomZ
        )
        val deltaTranslationResult: InhomogeneousPoint3D? =
            collector.getPrivateProperty("deltaTranslation")
        requireNotNull(deltaTranslationResult)
        val endEcefVelocity = ECEFVelocity(
            deltaTranslationResult.inhomX / TIME_INTERVAL,
            deltaTranslationResult.inhomY / TIME_INTERVAL,
            deltaTranslationResult.inhomZ / TIME_INTERVAL
        )
        val endNedPosition = NEDPosition()
        val endNedVelocity = NEDVelocity()
        ECEFtoNEDPositionVelocityConverter.convertECEFtoNED(
            endEcefPosition,
            endEcefVelocity,
            endNedPosition,
            endNedVelocity
        )
        assertTrue(frame.position.equals(endNedPosition, ABSOLUTE_ERROR))
        assertTrue(
            attitude2.equals(
                quaternion.conjugateAndReturnNew(),
                ABSOLUTE_ERROR
            )
        )
        val c = CoordinateTransformation(
            quaternion.asInhomogeneousMatrix(),
            FrameType.BODY_FRAME,
            FrameType.LOCAL_NAVIGATION_FRAME
        )
        val frame2 = NEDFrame(endNedPosition, endNedVelocity, c)
        assertTrue(frame.equals(frame2, ABSOLUTE_ERROR))
        assertEquals(frame.coordinateTransformation, frame2.coordinateTransformation)
        assertEquals(frame.position, frame2.position)
    }

    @Test
    fun onSensorChanged_whenListenerAndNoLocation_notifiesEvent() {
        val display = mockk<Display>()
        every { display.rotation }.returns(Surface.ROTATION_0)
        val context = spyk(ApplicationProvider.getApplicationContext())
        every { context.display }.returns(display)

        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_POSE_6DOF) }.returns(
            sensorMock
        )
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val measurementListener =
            mockk<RelativeDevicePoseSensorCollector.OnMeasurementListener>(relaxUnitFun = true)
        val collector =
            RelativeDevicePoseSensorCollector(
                context,
                measurementListener = measurementListener,
                timeInterval = TIME_INTERVAL
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

        every { sensorMock.type }.returns(Sensor.TYPE_POSE_6DOF)
        val event = mockk<SensorEvent>()
        event.sensor = sensorMock
        event.timestamp = System.nanoTime()
        event.accuracy = SensorAccuracy.HIGH.value
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true

        val attitude1 = createQuaternion()
        val attitude2 = createQuaternion()
        val invAttitude1 = attitude1.inverseAndReturnNew()
        val deltaAttitude = invAttitude1.combineAndReturnNew(attitude2)

        val translation1 = createTranslation()
        val translation2 = createTranslation()
        val deltaTranslation = InhomogeneousPoint3D(
            translation2.inhomX - translation1.inhomX,
            translation2.inhomY - translation1.inhomY,
            translation2.inhomZ - translation1.inhomZ
        )

        val transformation2 = EuclideanTransformation3D(attitude2, translation2.asArray())

        val a2 = attitude2.a.toFloat()
        val b2 = attitude2.b.toFloat()
        val c2 = attitude2.c.toFloat()
        val d2 = attitude2.d.toFloat()

        val x2 = translation2.inhomX.toFloat()
        val y2 = translation2.inhomY.toFloat()
        val z2 = translation2.inhomZ.toFloat()

        val aDelta = deltaAttitude.a.toFloat()
        val bDelta = deltaAttitude.b.toFloat()
        val cDelta = deltaAttitude.c.toFloat()
        val dDelta = deltaAttitude.d.toFloat()

        val xDelta = deltaTranslation.inhomX.toFloat()
        val yDelta = deltaTranslation.inhomY.toFloat()
        val zDelta = deltaTranslation.inhomZ.toFloat()

        val values = floatArrayOf(
            b2,
            c2,
            d2,
            a2,
            x2,
            y2,
            z2,
            bDelta,
            cDelta,
            dDelta,
            aDelta,
            xDelta,
            yDelta,
            zDelta,
            SEQUENCE_NUMBER
        )

        valuesField.set(event, values)
        eventListener.onSensorChanged(event)

        verify(exactly = 1) { sensorMock.type }
        val quaternionSlot = slot<Quaternion>()
        val translationSlot = slot<Point3D>()
        val transformationSlot = slot<EuclideanTransformation3D>()
        verify(exactly = 1) {
            measurementListener.onMeasurement(
                capture(quaternionSlot),
                capture(translationSlot),
                null,
                capture(transformationSlot),
                event.timestamp,
                SensorAccuracy.HIGH
            )
        }

        val quaternion = quaternionSlot.captured
        val translation = translationSlot.captured
        val transformation = transformationSlot.captured
        assertEquals(quaternion, transformation.rotation)
        assertEquals(translation, transformation.translationPoint)
        assertTrue(
            transformation.asMatrix()
                .equals(transformation2.inverseAndReturnNew().asMatrix(), 10.0 * ABSOLUTE_ERROR)
        )
        val deltaTranslationResult: InhomogeneousPoint3D? =
            collector.getPrivateProperty("deltaTranslation")
        requireNotNull(deltaTranslationResult)
        assertTrue(
            attitude2.equals(
                quaternion.conjugateAndReturnNew(),
                ABSOLUTE_ERROR
            )
        )
    }

    @Test
    fun onAccuracyChanged_whenNoSensor_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_POSE_6DOF) }.returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val accuracyChangedListener =
            mockk<SensorCollector.OnAccuracyChangedListener>(relaxUnitFun = true)
        val collector = RelativeDevicePoseSensorCollector(
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
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_POSE_6DOF) }.returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val accuracyChangedListener =
            mockk<SensorCollector.OnAccuracyChangedListener>(relaxUnitFun = true)
        val collector = RelativeDevicePoseSensorCollector(
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
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_POSE_6DOF) }.returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val accuracyChangedListener =
            mockk<SensorCollector.OnAccuracyChangedListener>(relaxUnitFun = true)
        val collector = RelativeDevicePoseSensorCollector(
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

        every { sensorMock.type }.returns(Sensor.TYPE_POSE_6DOF)
        eventListener.onAccuracyChanged(sensorMock, SensorAccuracy.HIGH.value)

        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(SensorAccuracy.HIGH)
        }
    }

    private companion object {
        const val ABSOLUTE_ERROR = 1e-6

        const val TIME_INTERVAL = 0.02

        const val MIN_LATITUDE_DEGREES = -90.0
        const val MAX_LATITUDE_DEGREES = 90.0

        const val MIN_LONGITUDE_DEGREES = -180.0
        const val MAX_LONGITUDE_DEGREES = 180.0

        const val MIN_HEIGHT = -100.0
        const val MAX_HEIGHT = 100.0

        const val MIN_ANGLE_DEGREES = -90.0
        const val MAX_ANGLE_DEGREES = 90.0

        const val MIN_POS = -10.0
        const val MAX_POS = 10.0

        const val SEQUENCE_NUMBER = 1.0f

        fun createLocation(): Location {
            val randomizer = UniformRandomizer()
            val location = mockk<Location>()
            every { location.latitude }.returns(
                randomizer.nextDouble(
                    MIN_LATITUDE_DEGREES,
                    MAX_LATITUDE_DEGREES
                )
            )
            every { location.longitude }.returns(
                randomizer.nextDouble(
                    MIN_LONGITUDE_DEGREES,
                    MAX_LONGITUDE_DEGREES
                )
            )
            every { location.altitude }.returns(randomizer.nextDouble(MIN_HEIGHT, MAX_HEIGHT))
            return location
        }

        fun createQuaternion(): Quaternion {
            val randomizer = UniformRandomizer()
            val roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            return Quaternion(roll, pitch, yaw)
        }

        fun createTranslation(): Point3D {
            val randomizer = UniformRandomizer()
            val x = randomizer.nextDouble(MIN_POS, MAX_POS)
            val y = randomizer.nextDouble(MIN_POS, MAX_POS)
            val z = randomizer.nextDouble(MIN_POS, MAX_POS)
            return InhomogeneousPoint3D(x, y, z)
        }
    }
}