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
import androidx.test.core.app.ApplicationProvider
import com.irurueta.android.navigation.inertial.getPrivateProperty
import com.irurueta.android.navigation.inertial.setPrivateProperty
import io.mockk.every
import io.mockk.mockk
import io.mockk.spyk
import io.mockk.verify
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import java.util.ArrayDeque

@RunWith(RobolectricTestRunner::class)
class AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncerTest {

    @Test
    fun constructor_whenRequiredParameters_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

        // check
        assertSame(context, syncer.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            syncer.accelerometerSensorType
        )
        assertEquals(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED, syncer.gyroscopeSensorType)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            syncer.magnetometerSensorType
        )
        assertEquals(SensorDelay.FASTEST, syncer.accelerometerSensorDelay)
        assertEquals(SensorDelay.FASTEST, syncer.magnetometerSensorDelay)
        assertEquals(SensorDelay.FASTEST, syncer.gyroscopeSensorDelay)
        assertEquals(
            AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer.DEFAULT_ACCELEROMETER_CAPACITY,
            syncer.accelerometerCapacity
        )
        assertEquals(
            AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer.DEFAULT_GYROSCOPE_CAPACITY,
            syncer.gyroscopeCapacity
        )
        assertEquals(
            AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer.DEFAULT_MAGNETOMETER_CAPACITY,
            syncer.magnetometerCapacity
        )
        assertFalse(syncer.accelerometerStartOffsetEnabled)
        assertFalse(syncer.gyroscopeStartOffsetEnabled)
        assertFalse(syncer.magnetometerStartOffsetEnabled)
        assertTrue(syncer.stopWhenFilledBuffer)
        assertEquals(
            AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer.DEFAULT_STALE_OFFSET_NANOS,
            syncer.staleOffsetNanos
        )
        assertTrue(syncer.staleDetectionEnabled)
        assertNull(syncer.accuracyChangedListener)
        assertNull(syncer.bufferFilledListener)
        assertNull(syncer.syncedMeasurementListener)
        assertNull(syncer.staleDetectedMeasurementsListener)
        assertEquals(0L, syncer.startTimestamp)
        assertFalse(syncer.running)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        assertNull(syncer.mostRecentTimestamp)
        assertNull(syncer.oldestTimestamp)
        assertNull(syncer.accelerometerSensor)
        assertNull(syncer.gyroscopeSensor)
        assertNull(syncer.magnetometerSensor)
        assertFalse(syncer.accelerometerSensorAvailable)
        assertFalse(syncer.gyroscopeSensorAvailable)
        assertFalse(syncer.magnetometerSensorAvailable)
        assertNull(syncer.accelerometerStartOffset)
        assertNull(syncer.gyroscopeStartOffset)
        assertNull(syncer.magnetometerStartOffset)
        assertEquals(0.0f, syncer.accelerometerCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.gyroscopeCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.magnetometerCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.accelerometerUsage, 0.0f)
        assertEquals(0.0f, syncer.gyroscopeUsage, 0.0f)
        assertEquals(0.0f, syncer.magnetometerUsage, 0.0f)
    }

    @Test(expected = IllegalArgumentException::class)
    fun constructor_whenZeroAccelerometerCapacity_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer(context, accelerometerCapacity = 0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun constructor_whenZeroGyroscopeCapacity_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer(context, gyroscopeCapacity = 0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun constructor_whenZeroMagnetometerCapacity_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer(context, magnetometerCapacity = 0)
    }

    @Test
    fun constructor_whenAllParameters_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val accuracyChangedListener =
            mockk<SensorMeasurementSyncer.OnAccuracyChangedListener<AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement, AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer>>()
        val bufferFilledListener =
            mockk<SensorMeasurementSyncer.OnBufferFilledListener<AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement, AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer>>()
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement, AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer>>()
        val staleDetectedMeasurementsListener =
            mockk<SensorMeasurementSyncer.OnStaleDetectedMeasurementsListener<AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement, AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer>>()
        val syncer = AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            GyroscopeSensorType.GYROSCOPE,
            MagnetometerSensorType.MAGNETOMETER,
            SensorDelay.NORMAL,
            SensorDelay.UI,
            SensorDelay.GAME,
            accelerometerCapacity = 2,
            gyroscopeCapacity = 3,
            magnetometerCapacity = 4,
            accelerometerStartOffsetEnabled = true,
            gyroscopeStartOffsetEnabled = true,
            magnetometerStartOffsetEnabled = true,
            stopWhenFilledBuffer = false,
            staleOffsetNanos = 123456789L,
            staleDetectionEnabled = false,
            accuracyChangedListener = accuracyChangedListener,
            bufferFilledListener = bufferFilledListener,
            syncedMeasurementListener = syncedMeasurementListener,
            staleDetectedMeasurementsListener = staleDetectedMeasurementsListener
        )

        // check
        assertSame(context, syncer.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            syncer.accelerometerSensorType
        )
        assertEquals(GyroscopeSensorType.GYROSCOPE, syncer.gyroscopeSensorType)
        assertEquals(MagnetometerSensorType.MAGNETOMETER, syncer.magnetometerSensorType)
        assertEquals(SensorDelay.NORMAL, syncer.accelerometerSensorDelay)
        assertEquals(SensorDelay.UI, syncer.gyroscopeSensorDelay)
        assertEquals(SensorDelay.GAME, syncer.magnetometerSensorDelay)
        assertEquals(2, syncer.accelerometerCapacity)
        assertEquals(3, syncer.gyroscopeCapacity)
        assertEquals(4, syncer.magnetometerCapacity)
        assertTrue(syncer.accelerometerStartOffsetEnabled)
        assertTrue(syncer.gyroscopeStartOffsetEnabled)
        assertTrue(syncer.magnetometerStartOffsetEnabled)
        assertFalse(syncer.stopWhenFilledBuffer)
        assertEquals(123456789L, syncer.staleOffsetNanos)
        assertFalse(syncer.staleDetectionEnabled)
        assertSame(accuracyChangedListener, syncer.accuracyChangedListener)
        assertSame(bufferFilledListener, syncer.bufferFilledListener)
        assertSame(syncedMeasurementListener, syncer.syncedMeasurementListener)
        assertSame(staleDetectedMeasurementsListener, syncer.staleDetectedMeasurementsListener)
        assertEquals(0L, syncer.startTimestamp)
        assertFalse(syncer.running)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        assertNull(syncer.mostRecentTimestamp)
        assertNull(syncer.oldestTimestamp)
        assertNull(syncer.accelerometerSensor)
        assertNull(syncer.gyroscopeSensor)
        assertNull(syncer.magnetometerSensor)
        assertFalse(syncer.accelerometerSensorAvailable)
        assertFalse(syncer.gyroscopeSensorAvailable)
        assertFalse(syncer.magnetometerSensorAvailable)
        assertNull(syncer.accelerometerStartOffset)
        assertNull(syncer.gyroscopeStartOffset)
        assertNull(syncer.magnetometerStartOffset)
        assertEquals(0.0f, syncer.accelerometerCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.gyroscopeCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.magnetometerCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.accelerometerUsage, 0.0f)
        assertEquals(0.0f, syncer.gyroscopeUsage, 0.0f)
        assertEquals(0.0f, syncer.magnetometerUsage, 0.0f)
    }

    @Test
    fun accuracyChangedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

        // check default value
        assertNull(syncer.accuracyChangedListener)

        // set new value
        val accuracyChangedListener =
            mockk<SensorMeasurementSyncer.OnAccuracyChangedListener<AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement, AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer>>()
        syncer.accuracyChangedListener = accuracyChangedListener

        // check
        assertSame(accuracyChangedListener, syncer.accuracyChangedListener)
    }

    @Test
    fun bufferFilledListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

        // check default value
        assertNull(syncer.bufferFilledListener)

        // set new value
        val bufferFilledListener =
            mockk<SensorMeasurementSyncer.OnBufferFilledListener<AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement, AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer>>()
        syncer.bufferFilledListener = bufferFilledListener

        // check
        assertSame(bufferFilledListener, syncer.bufferFilledListener)
    }

    @Test
    fun syncedMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

        // check default value
        assertNull(syncer.syncedMeasurementListener)

        // set new value
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement, AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer>>()
        syncer.syncedMeasurementListener = syncedMeasurementListener

        // check
        assertSame(syncedMeasurementListener, syncer.syncedMeasurementListener)
    }

    @Test
    fun staleDetectedMeasurementsListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

        // check default value
        assertNull(syncer.staleDetectedMeasurementsListener)

        // set new value
        val staleDetectedMeasurementsListener =
            mockk<SensorMeasurementSyncer.OnStaleDetectedMeasurementsListener<AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement, AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer>>()
        syncer.staleDetectedMeasurementsListener = staleDetectedMeasurementsListener

        // check
        assertSame(staleDetectedMeasurementsListener, syncer.staleDetectedMeasurementsListener)
    }

    @Test
    fun accelerometerSensor_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        assertNull(syncer.accelerometerSensor)
        verify(exactly = 1) { accelerometerSensorCollectorSpy.sensor }
    }

    @Test
    fun gyroscopeSensor_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        assertNull(syncer.gyroscopeSensor)
        verify(exactly = 1) { gyroscopeSensorCollectorSpy.sensor }
    }

    @Test
    fun magnetometerSensor_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        assertNull(syncer.magnetometerSensor)
        verify(exactly = 1) { magnetometerSensorCollectorSpy.sensor }
    }

    @Test
    fun accelerometerSensorAvailable_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        assertFalse(syncer.accelerometerSensorAvailable)
        verify(exactly = 1) { accelerometerSensorCollectorSpy.sensorAvailable }
    }

    @Test
    fun gyroscopeSensorAvailable_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        assertFalse(syncer.gyroscopeSensorAvailable)
        verify(exactly = 1) { gyroscopeSensorCollectorSpy.sensorAvailable }
    }

    @Test
    fun magnetometerSensorAvailable_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        assertFalse(syncer.magnetometerSensorAvailable)
        verify(exactly = 1) { magnetometerSensorCollectorSpy.sensorAvailable }
    }

    @Test
    fun accelerometerStartOffset_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        assertNull(syncer.accelerometerStartOffset)
        verify(exactly = 1) { accelerometerSensorCollectorSpy.startOffset }
    }

    @Test
    fun gyroscopeStartOffset_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        assertNull(syncer.gyroscopeStartOffset)
        verify(exactly = 1) { gyroscopeSensorCollectorSpy.startOffset }
    }

    @Test
    fun magnetometerStartOffset_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        assertNull(syncer.magnetometerStartOffset)
        verify(exactly = 1) { magnetometerSensorCollectorSpy.startOffset }
    }

    @Test
    fun accelerometerCollectorUsage_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        assertEquals(0.0f, syncer.accelerometerCollectorUsage, 0.0f)
        verify(exactly = 1) { accelerometerSensorCollectorSpy.usage }
    }

    @Test
    fun gyroscopeCollectorUsage_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        assertEquals(0.0f, syncer.gyroscopeCollectorUsage, 0.0f)
        verify(exactly = 1) { gyroscopeSensorCollectorSpy.usage }
    }

    @Test
    fun magnetometerCollectorUsage_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        assertEquals(0.0f, syncer.magnetometerCollectorUsage, 0.0f)
        verify(exactly = 1) { magnetometerSensorCollectorSpy.usage }
    }

    @Test
    fun accelerometerUsage_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

        // check default value
        assertEquals(0.0f, syncer.accelerometerUsage, 0.0f)

        // add measurement
        val accelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("accelerometerMeasurements")
        requireNotNull(accelerometerMeasurements)
        assertTrue(accelerometerMeasurements.isEmpty())
        accelerometerMeasurements.add(AccelerometerSensorMeasurement())

        // check
        assertEquals(1.0f / syncer.accelerometerCapacity.toFloat(), syncer.accelerometerUsage, 0.0f)
    }

    @Test
    fun gyroscopeUsage_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

        // check default value
        assertEquals(0.0f, syncer.gyroscopeUsage, 0.0f)

        // add measurement
        val gyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("gyroscopeMeasurements")
        requireNotNull(gyroscopeMeasurements)
        assertTrue(gyroscopeMeasurements.isEmpty())
        gyroscopeMeasurements.add(GyroscopeSensorMeasurement())

        // check
        assertEquals(1.0f / syncer.gyroscopeCapacity.toFloat(), syncer.gyroscopeUsage, 0.0f)
    }

    @Test
    fun magnetometerUsage_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

        // check default value
        assertEquals(0.0f, syncer.magnetometerUsage, 0.0f)

        // add measurement
        val magnetometerMeasurements: ArrayDeque<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("magnetometerMeasurements")
        requireNotNull(magnetometerMeasurements)
        assertTrue(magnetometerMeasurements.isEmpty())
        magnetometerMeasurements.add(MagnetometerSensorMeasurement())

        // check
        assertEquals(1.0f / syncer.magnetometerCapacity.toFloat(), syncer.magnetometerUsage, 0.0f)
    }

    @Test(expected = IllegalStateException::class)
    fun start_whenAlreadyRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

        // set as running
        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "running", true)
        assertTrue(syncer.running)

        syncer.start()
    }

    @Test
    fun start_whenNotRunningAndNoTimestamp_setsCurrentStartTimestamp() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

        assertFalse(syncer.running)
        assertEquals(0L, syncer.startTimestamp)

        assertFalse(syncer.start())

        // check
        assertFalse(syncer.running)
        assertNotEquals(0L, syncer.startTimestamp)
    }

    @Test
    fun start_whenNotRunningAndTimestampProvided_setsProvidedTimestamp() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

        assertFalse(syncer.running)
        assertEquals(0L, syncer.startTimestamp)

        val timestamp = System.nanoTime()
        assertFalse(syncer.start(timestamp))

        // check
        assertFalse(syncer.running)
        assertEquals(timestamp, syncer.startTimestamp)
    }

    @Test
    fun start_whenAccelerometerCollectorFails_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.start(any()) }.returns(false)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)
        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        every { gyroscopeSensorCollectorSpy.start(any()) }.returns(true)
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)
        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)
        every { magnetometerSensorCollectorSpy.start(any()) }.returns(true)
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        assertFalse(syncer.running)

        val timestamp = System.nanoTime()
        assertFalse(syncer.start(timestamp))

        assertFalse(syncer.running)
        assertEquals(timestamp, syncer.startTimestamp)

        verify(exactly = 1) { accelerometerSensorCollectorSpy.start(timestamp) }
        verify(exactly = 0) { gyroscopeSensorCollectorSpy.start(any()) }
        verify(exactly = 0) { magnetometerSensorCollectorSpy.start(any()) }
    }

    @Test
    fun start_whenGyroscopeCollectorFails_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.start(any()) }.returns(true)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)
        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        every { gyroscopeSensorCollectorSpy.start(any()) }.returns(false)
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)
        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)
        every { magnetometerSensorCollectorSpy.start(any()) }.returns(true)
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        assertFalse(syncer.running)

        val timestamp = System.nanoTime()
        assertFalse(syncer.start(timestamp))

        assertFalse(syncer.running)
        assertEquals(timestamp, syncer.startTimestamp)

        verify(exactly = 1) { accelerometerSensorCollectorSpy.start(timestamp) }
        verify(exactly = 1) { gyroscopeSensorCollectorSpy.start(any()) }
        verify(exactly = 0) { magnetometerSensorCollectorSpy.start(timestamp) }
    }

    @Test
    fun start_whenMagnetometerCollectorFails_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.start(any()) }.returns(true)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)
        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        every { gyroscopeSensorCollectorSpy.start(any()) }.returns(true)
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)
        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)
        every { magnetometerSensorCollectorSpy.start(any()) }.returns(false)
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        assertFalse(syncer.running)

        val timestamp = System.nanoTime()
        assertFalse(syncer.start(timestamp))

        assertFalse(syncer.running)
        assertEquals(timestamp, syncer.startTimestamp)

        verify(exactly = 1) { accelerometerSensorCollectorSpy.start(timestamp) }
        verify(exactly = 1) { gyroscopeSensorCollectorSpy.start(any()) }
        verify(exactly = 1) { magnetometerSensorCollectorSpy.start(timestamp) }
    }

    @Test
    fun start_whenCollectorsSucceed_returnsTrue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.start(any()) }.returns(true)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)
        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        every { gyroscopeSensorCollectorSpy.start(any()) }.returns(true)
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)
        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)
        every { magnetometerSensorCollectorSpy.start(any()) }.returns(true)
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        assertFalse(syncer.running)

        val timestamp = System.nanoTime()
        assertTrue(syncer.start(timestamp))

        assertTrue(syncer.running)
        assertEquals(timestamp, syncer.startTimestamp)

        verify(exactly = 1) { accelerometerSensorCollectorSpy.start(timestamp) }
        verify(exactly = 1) { gyroscopeSensorCollectorSpy.start(any()) }
        verify(exactly = 1) { magnetometerSensorCollectorSpy.start(timestamp) }
    }

    @Test
    fun stop_stopsCollectorsInitializesCachesAndResetsProperties() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)
        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)
        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        val accelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("accelerometerMeasurements")
        requireNotNull(accelerometerMeasurements)
        accelerometerMeasurements.add(AccelerometerSensorMeasurement())

        val gyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("gyroscopeMeasurements")
        requireNotNull(gyroscopeMeasurements)
        gyroscopeMeasurements.add(GyroscopeSensorMeasurement())

        val magnetometerMeasurements: ArrayDeque<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("magnetometerMeasurements")
        requireNotNull(magnetometerMeasurements)
        magnetometerMeasurements.add(MagnetometerSensorMeasurement())

        val foundGyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("foundGyroscopeMeasurements")
        requireNotNull(foundGyroscopeMeasurements)
        foundGyroscopeMeasurements.add(GyroscopeSensorMeasurement())

        val foundMagnetometerMeasurements: ArrayDeque<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("foundMagnetometerMeasurements")
        requireNotNull(foundMagnetometerMeasurements)
        foundMagnetometerMeasurements.add(MagnetometerSensorMeasurement())

        // set variables that will be later reset
        setPrivateProperty(
            SensorMeasurementSyncer::class,
            syncer,
            "numberOfProcessedMeasurements",
            1
        )
        setPrivateProperty(
            SensorMeasurementSyncer::class,
            syncer,
            "mostRecentTimestamp",
            System.nanoTime()
        )
        setPrivateProperty(
            SensorMeasurementSyncer::class,
            syncer,
            "oldestTimestamp",
            System.nanoTime()
        )
        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "running", true)

        syncer.setPrivateProperty("hasPreviousAccelerometerMeasurement", true)
        syncer.setPrivateProperty("hasPreviousGyroscopeMeasurement", true)
        syncer.setPrivateProperty("hasPreviousMagnetometerMeasurement", true)
        syncer.setPrivateProperty("lastNotifiedTimestamp", 1L)
        syncer.setPrivateProperty("lastNotifiedAccelerometerTimestamp", 2L)
        syncer.setPrivateProperty("lastNotifiedGyroscopeTimestamp", 3L)
        syncer.setPrivateProperty("lastNotifiedMagnetometerTimestamp", 4L)

        syncer.stop()

        // check
        verify(exactly = 1) { accelerometerSensorCollectorSpy.stop() }
        verify(exactly = 1) { gyroscopeSensorCollectorSpy.stop() }
        verify(exactly = 1) { magnetometerSensorCollectorSpy.stop() }
        assertTrue(accelerometerMeasurements.isEmpty())
        assertTrue(gyroscopeMeasurements.isEmpty())
        assertTrue(magnetometerMeasurements.isEmpty())
        assertTrue(foundGyroscopeMeasurements.isEmpty())
        assertTrue(foundMagnetometerMeasurements.isEmpty())

        val numberOfProcessedMeasurements: Int? = getPrivateProperty(
            SensorMeasurementSyncer::class,
            syncer,
            "numberOfProcessedMeasurements"
        )
        requireNotNull(numberOfProcessedMeasurements)
        assertEquals(0, numberOfProcessedMeasurements)

        val mostRecentTimestamp: Long? =
            getPrivateProperty(SensorMeasurementSyncer::class, syncer, "mostRecentTimestamp")
        assertNull(mostRecentTimestamp)

        val oldestTimestamp: Long? =
            getPrivateProperty(SensorMeasurementSyncer::class, syncer, "oldestTimestamp")
        assertNull(oldestTimestamp)

        assertFalse(syncer.running)

        val hasPreviousAccelerometerMeasurement: Boolean? =
            syncer.getPrivateProperty("hasPreviousAccelerometerMeasurement")
        requireNotNull(hasPreviousAccelerometerMeasurement)
        assertFalse(hasPreviousAccelerometerMeasurement)

        val hasPreviousGyroscopeMeasurement: Boolean? =
            syncer.getPrivateProperty("hasPreviousGyroscopeMeasurement")
        requireNotNull(hasPreviousGyroscopeMeasurement)
        assertFalse(hasPreviousGyroscopeMeasurement)

        val hasPreviousMagnetometerMeasurement: Boolean? =
            syncer.getPrivateProperty("hasPreviousMagnetometerMeasurement")
        requireNotNull(hasPreviousMagnetometerMeasurement)
        assertFalse(hasPreviousMagnetometerMeasurement)

        val lastNotifiedTimestamp: Long? = syncer.getPrivateProperty("lastNotifiedTimestamp")
        requireNotNull(lastNotifiedTimestamp)
        assertEquals(0L, lastNotifiedTimestamp)

        val lastNotifiedAccelerometerTimestamp: Long? =
            syncer.getPrivateProperty("lastNotifiedAccelerometerTimestamp")
        requireNotNull(lastNotifiedAccelerometerTimestamp)
        assertEquals(0L, lastNotifiedAccelerometerTimestamp)

        val lastNotifiedGyroscopeTimestamp: Long? =
            syncer.getPrivateProperty("lastNotifiedGyroscopeTimestamp")
        requireNotNull(lastNotifiedGyroscopeTimestamp)
        assertEquals(0L, lastNotifiedGyroscopeTimestamp)

        val lastNotifiedMagnetometerTimestamp: Long? =
            syncer.getPrivateProperty("lastNotifiedMagnetometerTimestamp")
        requireNotNull(lastNotifiedMagnetometerTimestamp)
        assertEquals(0L, lastNotifiedMagnetometerTimestamp)
    }

    @Test
    fun accelerometerSensorCollector_whenAccuracyChangedListenerAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)

        val listener = accelerometerSensorCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(accelerometerSensorCollector, SensorAccuracy.HIGH)
    }

    @Test
    fun accelerometerSensorCollector_whenAccuracyChangedListenerAndListenerAvailable_notifies() {
        val accuracyChangedListener =
            mockk<SensorMeasurementSyncer.OnAccuracyChangedListener<AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement, AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)

        val listener = accelerometerSensorCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(accelerometerSensorCollector, SensorAccuracy.HIGH)

        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                syncer,
                SensorMeasurementSyncer.SensorType.ACCELEROMETER_UNCALIBRATED,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun accelerometerSensorCollector_whenBufferFilledAndStopWhenFilledBuffer_stops() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer =
            AccelerometerGyroscopeAndMagnetometerSensorMeasurementSyncer(
                context,
                stopWhenFilledBuffer = true
            )

        // set as running
        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "running", true)
        assertTrue(syncer.running)
        assertTrue(syncer.stopWhenFilledBuffer)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)

        val listener = accelerometerSensorCollector.bufferFilledListener
        requireNotNull(listener)

        listener.onBufferFilled(accelerometerSensorCollector)

        assertFalse(syncer.running)
    }

}