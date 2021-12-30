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
import io.mockk.*
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class MagnetometerSensorTest {

    @Test
    fun constructor_whenContext_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensor = MagnetometerSensor(context)

        // check values
        assertSame(context, sensor.context)
        assertEquals(MagnetometerSensor.MagnetometerSensorType.MAGNETOMETER, sensor.sensorType)
        assertEquals(SensorDelay.FASTEST, sensor.sensorDelay)
        assertNull(sensor.magnetometerMeasurementListener)
        assertNull(sensor.magnetometerAccuracyChangedListener)
        assertNull(sensor.sensor)
        assertFalse(sensor.sensorAvailable)

        val sensorManager: SensorManager? = sensor.getPrivateProperty("sensorManager")
        assertNotNull(sensorManager)
    }

    @Test
    fun constructor_whenSensorType_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensor = MagnetometerSensor(
            context,
            MagnetometerSensor.MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED
        )

        // check values
        assertSame(context, sensor.context)
        assertEquals(
            MagnetometerSensor.MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            sensor.sensorType
        )
        assertEquals(SensorDelay.FASTEST, sensor.sensorDelay)
        assertNull(sensor.magnetometerMeasurementListener)
        assertNull(sensor.magnetometerAccuracyChangedListener)
        assertNull(sensor.sensor)
        assertFalse(sensor.sensorAvailable)

        val sensorManager: SensorManager? = sensor.getPrivateProperty("sensorManager")
        assertNotNull(sensorManager)
    }

    @Test
    fun constructor_whenSensorDelay_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensor = MagnetometerSensor(
            context,
            MagnetometerSensor.MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            SensorDelay.NORMAL
        )

        // check values
        assertSame(context, sensor.context)
        assertEquals(
            MagnetometerSensor.MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            sensor.sensorType
        )
        assertEquals(SensorDelay.NORMAL, sensor.sensorDelay)
        assertNull(sensor.magnetometerMeasurementListener)
        assertNull(sensor.magnetometerAccuracyChangedListener)
        assertNull(sensor.sensor)
        assertFalse(sensor.sensorAvailable)

        val sensorManager: SensorManager? = sensor.getPrivateProperty("sensorManager")
        assertNotNull(sensorManager)
    }

    @Test
    fun constructor_whenMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val magnetometerMeasurementListener =
            mockk<MagnetometerSensor.OnMagnetometerMeasurementListener>()
        val sensor = MagnetometerSensor(
            context,
            MagnetometerSensor.MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            magnetometerMeasurementListener
        )

        // check values
        assertSame(context, sensor.context)
        assertEquals(
            MagnetometerSensor.MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            sensor.sensorType
        )
        assertEquals(SensorDelay.NORMAL, sensor.sensorDelay)
        assertSame(magnetometerMeasurementListener, sensor.magnetometerMeasurementListener)
        assertNull(sensor.magnetometerAccuracyChangedListener)
        assertNull(sensor.sensor)
        assertFalse(sensor.sensorAvailable)

        val sensorManager: SensorManager? = sensor.getPrivateProperty("sensorManager")
        assertNotNull(sensorManager)
    }

    @Test
    fun constructor_whenAccuracyListener_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val magnetometerMeasurementListener =
            mockk<MagnetometerSensor.OnMagnetometerMeasurementListener>()
        val magnetometerAccuracyChangedListener =
            mockk<MagnetometerSensor.OnMagnetometerAccuracyChangedListener>()
        val sensor = MagnetometerSensor(
            context,
            MagnetometerSensor.MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            SensorDelay.NORMAL,
            magnetometerMeasurementListener,
            magnetometerAccuracyChangedListener
        )

        // check values
        assertSame(context, sensor.context)
        assertEquals(
            MagnetometerSensor.MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            sensor.sensorType
        )
        assertEquals(SensorDelay.NORMAL, sensor.sensorDelay)
        assertSame(magnetometerMeasurementListener, sensor.magnetometerMeasurementListener)
        assertSame(magnetometerAccuracyChangedListener, sensor.magnetometerAccuracyChangedListener)
        assertNull(sensor.sensor)
        assertFalse(sensor.sensorAvailable)

        val sensorManager: SensorManager? = sensor.getPrivateProperty("sensorManager")
        assertNotNull(sensorManager)
    }

    @Test
    fun magnetometerMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensor = MagnetometerSensor(context)

        assertNull(sensor.magnetometerMeasurementListener)

        // set new value
        val magnetometerMeasurementListener =
            mockk<MagnetometerSensor.OnMagnetometerMeasurementListener>()
        sensor.magnetometerMeasurementListener = magnetometerMeasurementListener

        // check
        assertSame(magnetometerMeasurementListener, sensor.magnetometerMeasurementListener)
    }

    @Test
    fun magnetometerAccuracyChangedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensor = MagnetometerSensor(context)

        assertNull(sensor.magnetometerAccuracyChangedListener)

        // set new value
        val magnetometerAccuracyChangedListener =
            mockk<MagnetometerSensor.OnMagnetometerAccuracyChangedListener>()
        sensor.magnetometerAccuracyChangedListener = magnetometerAccuracyChangedListener

        // check
        assertSame(magnetometerAccuracyChangedListener, sensor.magnetometerAccuracyChangedListener)
    }

    @Test
    fun sensor_whenSensorTypeMagnetometer_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD) }.returns(sensorMock)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val sensor =
            MagnetometerSensor(contextSpy, MagnetometerSensor.MagnetometerSensorType.MAGNETOMETER)

        assertSame(sensorMock, sensor.sensor)
        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD) }
    }

    @Test
    fun sensor_whenSensorTypeMagnetometerUncalibrated_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED) }.returns(
            sensorMock
        )
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val sensor = MagnetometerSensor(
            contextSpy,
            MagnetometerSensor.MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED
        )

        assertSame(sensorMock, sensor.sensor)
        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) {
            sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED)
        }
    }

    @Test
    fun sensorAvailable_whenSensorTypeMagnetometer_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val sensor =
            MagnetometerSensor(contextSpy, MagnetometerSensor.MagnetometerSensorType.MAGNETOMETER)

        assertFalse(sensor.sensorAvailable)
        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD) }
    }

    @Test
    fun sensorAvailable_whenSensorTypeMagnetometerUncalibrated_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED) }
            .returns(sensorMock)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val sensor =
            MagnetometerSensor(
                contextSpy,
                MagnetometerSensor.MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED
            )

        assertTrue(sensor.sensorAvailable)
        verify(exactly = 1) { contextSpy.getSystemService(Context.SENSOR_SERVICE) }
        verify(exactly = 1) {
            sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED)
        }
    }

    @Test
    fun start_whenNoSensorManager_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(null)

        val sensor = MagnetometerSensor(contextSpy)
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
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD) }.returns(null)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val sensor = MagnetometerSensor(contextSpy)
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
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD) }.returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val sensor = MagnetometerSensor(contextSpy)
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

        val sensor = MagnetometerSensor(contextSpy)
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
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD) }.returns(sensorMock)
        justRun { sensorManagerSpy.unregisterListener(any(), any<Sensor>()) }
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val sensor = MagnetometerSensor(contextSpy)
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
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD) }.returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val magnetometerMeasurementListener =
            mockk<MagnetometerSensor.OnMagnetometerMeasurementListener>()
        val sensor = MagnetometerSensor(
            contextSpy,
            magnetometerMeasurementListener = magnetometerMeasurementListener
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

        eventListener.onSensorChanged(null)
        verify { magnetometerMeasurementListener wasNot Called }
    }

    @Test
    fun onSensorChanged_whenNoSensorType_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD) }.returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val magnetometerMeasurementListener =
            mockk<MagnetometerSensor.OnMagnetometerMeasurementListener>()
        val sensor = MagnetometerSensor(
            contextSpy,
            magnetometerMeasurementListener = magnetometerMeasurementListener
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
        val event = mockk<SensorEvent>()
        event.sensor = sensorMock
        eventListener.onSensorChanged(event)

        verify { magnetometerMeasurementListener wasNot Called }
    }

    @Test
    fun onSensorChanged_whenNoListener_doesNotNotify() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD) }.returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val sensor = MagnetometerSensor(contextSpy)
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

        every { sensorMock.type }
            .returns(MagnetometerSensor.MagnetometerSensorType.MAGNETOMETER.value)
        val event = mockk<SensorEvent>()
        event.sensor = sensorMock
        event.timestamp = System.nanoTime()
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, floatArrayOf(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f))
        eventListener.onSensorChanged(event)

        verify(exactly = 1) { sensorMock.type }
    }

    @Test
    fun onSensorChanged_whenListenerMagnetometerSensor_notifiesEvent() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD) }.returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val magnetometerMeasurementListener =
            mockk<MagnetometerSensor.OnMagnetometerMeasurementListener>(relaxUnitFun = true)
        val sensor = MagnetometerSensor(
            contextSpy,
            magnetometerMeasurementListener = magnetometerMeasurementListener
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

        every { sensorMock.type }
            .returns(MagnetometerSensor.MagnetometerSensorType.MAGNETOMETER.value)
        val event = mockk<SensorEvent>()
        event.sensor = sensorMock
        event.timestamp = System.nanoTime()
        event.accuracy = SensorAccuracy.HIGH.value
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, floatArrayOf(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f))
        eventListener.onSensorChanged(event)

        verify(exactly = 1) {
            magnetometerMeasurementListener.onMagnetometerMeasurement(
                1.0f,
                2.0f,
                3.0f,
                null,
                null,
                null,
                event.timestamp,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun onSensorChanged_whenListenerMagnetometerUncalibratedSensor_notifiesEvent() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED) }
            .returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val magnetometerMeasurementListener =
            mockk<MagnetometerSensor.OnMagnetometerMeasurementListener>(relaxUnitFun = true)
        val sensor = MagnetometerSensor(
            contextSpy,
            MagnetometerSensor.MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            magnetometerMeasurementListener = magnetometerMeasurementListener
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

        every { sensorMock.type }
            .returns(MagnetometerSensor.MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value)
        val event = mockk<SensorEvent>()
        event.sensor = sensorMock
        event.timestamp = System.nanoTime()
        event.accuracy = SensorAccuracy.HIGH.value
        val valuesField = SensorEvent::class.java.getDeclaredField("values")
        valuesField.isAccessible = true
        valuesField.set(event, floatArrayOf(1.0f, 2.0f, 3.0f, 4.0f, 5.0f, 6.0f))
        eventListener.onSensorChanged(event)

        verify(exactly = 1) {
            magnetometerMeasurementListener.onMagnetometerMeasurement(
                1.0f,
                2.0f,
                3.0f,
                4.0f,
                5.0f,
                6.0f,
                event.timestamp,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun onAccuracyChanged_whenNoSensor_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD) }.returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val magnetometerAccuracyChangedListener =
            mockk<MagnetometerSensor.OnMagnetometerAccuracyChangedListener>(relaxUnitFun = true)
        val sensor = MagnetometerSensor(
            contextSpy,
            magnetometerAccuracyChangedListener = magnetometerAccuracyChangedListener
        )
        assertTrue(sensor.start())

        val slot = slot<SensorEventListener>()
        verify {
            sensorManagerSpy.registerListener(capture(slot), sensorMock, sensor.sensorDelay.value)
        }

        val eventListener = slot.captured

        eventListener.onAccuracyChanged(null, SensorAccuracy.HIGH.value)

        verify { magnetometerAccuracyChangedListener wasNot Called }
    }

    @Test
    fun onAccuracyChanged_whenNoSensorType_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD) }.returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val magnetometerAccuracyChangedListener =
            mockk<MagnetometerSensor.OnMagnetometerAccuracyChangedListener>(relaxUnitFun = true)
        val sensor = MagnetometerSensor(
            contextSpy,
            magnetometerAccuracyChangedListener = magnetometerAccuracyChangedListener
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

        verify { magnetometerAccuracyChangedListener wasNot Called }
    }

    @Test
    fun onAccuracyChanged_whenListener_notifiesEvent() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val sensorManager: SensorManager? =
            context.getSystemService(Context.SENSOR_SERVICE) as SensorManager?
        requireNotNull(sensorManager)
        val sensorManagerSpy = spyk(sensorManager)
        val sensorMock = mockk<Sensor>()
        every { sensorManagerSpy.getDefaultSensor(Sensor.TYPE_MAGNETIC_FIELD) }.returns(sensorMock)
        every { sensorManagerSpy.registerListener(any(), any<Sensor>(), any()) }.returns(true)
        val contextSpy = spyk(context)
        every { contextSpy.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManagerSpy)

        val magnetometerAccuracyChangedListener =
            mockk<MagnetometerSensor.OnMagnetometerAccuracyChangedListener>(relaxUnitFun = true)
        val sensor = MagnetometerSensor(
            contextSpy,
            magnetometerAccuracyChangedListener = magnetometerAccuracyChangedListener
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

        every { sensorMock.type }
            .returns(MagnetometerSensor.MagnetometerSensorType.MAGNETOMETER.value)
        eventListener.onAccuracyChanged(sensorMock, SensorAccuracy.HIGH.value)

        verify(exactly = 1) {
            magnetometerAccuracyChangedListener.onMagnetometerAccuracyChanged(
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun magnetometerSensorType_fromValues_returnsExpected() {
        assertEquals(2, MagnetometerSensor.MagnetometerSensorType.values().size)
        assertEquals(
            MagnetometerSensor.MagnetometerSensorType.MAGNETOMETER,
            MagnetometerSensor.MagnetometerSensorType.from(Sensor.TYPE_MAGNETIC_FIELD)
        )
        assertEquals(
            MagnetometerSensor.MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            MagnetometerSensor.MagnetometerSensorType.from(Sensor.TYPE_MAGNETIC_FIELD_UNCALIBRATED)
        )
    }
}