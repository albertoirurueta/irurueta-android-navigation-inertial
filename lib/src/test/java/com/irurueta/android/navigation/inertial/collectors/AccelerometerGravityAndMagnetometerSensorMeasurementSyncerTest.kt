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
import com.irurueta.statistics.UniformRandomizer
import io.mockk.*
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import java.util.*

@RunWith(RobolectricTestRunner::class)
class AccelerometerGravityAndMagnetometerSensorMeasurementSyncerTest {

    @Test
    fun constructor_whenRequiredParameters_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(context)

        // check
        assertSame(context, syncer.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            syncer.accelerometerSensorType
        )
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            syncer.magnetometerSensorType
        )
        assertEquals(SensorDelay.FASTEST, syncer.accelerometerSensorDelay)
        assertEquals(SensorDelay.FASTEST, syncer.gravitySensorDelay)
        assertEquals(SensorDelay.FASTEST, syncer.magnetometerSensorDelay)
        assertEquals(
            AccelerometerGravityAndMagnetometerSensorMeasurementSyncer.DEFAULT_ACCELEROMETER_CAPACITY,
            syncer.accelerometerCapacity
        )
        assertEquals(
            AccelerometerGravityAndMagnetometerSensorMeasurementSyncer.DEFAULT_GRAVITY_CAPACITY,
            syncer.gravityCapacity
        )
        assertEquals(
            AccelerometerGravityAndMagnetometerSensorMeasurementSyncer.DEFAULT_MAGNETOMETER_CAPACITY,
            syncer.magnetometerCapacity
        )
        assertFalse(syncer.accelerometerStartOffsetEnabled)
        assertFalse(syncer.gravityStartOffsetEnabled)
        assertFalse(syncer.magnetometerStartOffsetEnabled)
        assertTrue(syncer.stopWhenFilledBuffer)
        assertEquals(
            AccelerometerGravityAndMagnetometerSensorMeasurementSyncer.DEFAULT_STALE_OFFSET_NANOS,
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
        assertNull(syncer.gravitySensor)
        assertNull(syncer.magnetometerSensor)
        assertFalse(syncer.accelerometerSensorAvailable)
        assertFalse(syncer.gravitySensorAvailable)
        assertFalse(syncer.magnetometerSensorAvailable)
        assertNull(syncer.accelerometerStartOffset)
        assertNull(syncer.gravityStartOffset)
        assertNull(syncer.magnetometerStartOffset)
        assertEquals(0.0f, syncer.accelerometerCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.gravityCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.magnetometerCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.accelerometerUsage, 0.0f)
        assertEquals(0.0f, syncer.gravityUsage, 0.0f)
        assertEquals(0.0f, syncer.magnetometerUsage, 0.0f)
    }

    @Test(expected = IllegalArgumentException::class)
    fun constructor_whenZeroAccelerometerCapacity_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(
            context,
            accelerometerCapacity = 0
        )
    }

    @Test(expected = IllegalArgumentException::class)
    fun constructor_whenZeroGravityCapacity_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(context, gravityCapacity = 0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun constructor_whenZeroMagnetometerCapacity_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(
            context,
            magnetometerCapacity = 0
        )
    }

    @Test
    fun constructor_whenAllParameters_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val accuracyChangedListener =
            mockk<SensorMeasurementSyncer.OnAccuracyChangedListener<AccelerometerGravityAndMagnetometerSyncedSensorMeasurement, AccelerometerGravityAndMagnetometerSensorMeasurementSyncer>>()
        val bufferFilledListener =
            mockk<SensorMeasurementSyncer.OnBufferFilledListener<AccelerometerGravityAndMagnetometerSyncedSensorMeasurement, AccelerometerGravityAndMagnetometerSensorMeasurementSyncer>>()
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerGravityAndMagnetometerSyncedSensorMeasurement, AccelerometerGravityAndMagnetometerSensorMeasurementSyncer>>()
        val staleDetectedMeasurementsListener =
            mockk<SensorMeasurementSyncer.OnStaleDetectedMeasurementsListener<AccelerometerGravityAndMagnetometerSyncedSensorMeasurement, AccelerometerGravityAndMagnetometerSensorMeasurementSyncer>>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            MagnetometerSensorType.MAGNETOMETER,
            SensorDelay.NORMAL,
            SensorDelay.UI,
            SensorDelay.GAME,
            accelerometerCapacity = 2,
            gravityCapacity = 4,
            magnetometerCapacity = 3,
            accelerometerStartOffsetEnabled = true,
            gravityStartOffsetEnabled = true,
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
        assertEquals(AccelerometerSensorType.ACCELEROMETER, syncer.accelerometerSensorType)
        assertEquals(MagnetometerSensorType.MAGNETOMETER, syncer.magnetometerSensorType)
        assertEquals(SensorDelay.NORMAL, syncer.accelerometerSensorDelay)
        assertEquals(SensorDelay.UI, syncer.gravitySensorDelay)
        assertEquals(SensorDelay.GAME, syncer.magnetometerSensorDelay)
        assertEquals(2, syncer.accelerometerCapacity)
        assertEquals(4, syncer.gravityCapacity)
        assertEquals(3, syncer.magnetometerCapacity)
        assertTrue(syncer.accelerometerStartOffsetEnabled)
        assertTrue(syncer.gravityStartOffsetEnabled)
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
        assertNull(syncer.gravitySensor)
        assertNull(syncer.magnetometerSensor)
        assertFalse(syncer.accelerometerSensorAvailable)
        assertFalse(syncer.gravitySensorAvailable)
        assertFalse(syncer.magnetometerSensorAvailable)
        assertNull(syncer.accelerometerStartOffset)
        assertNull(syncer.gravityStartOffset)
        assertNull(syncer.magnetometerStartOffset)
        assertEquals(0.0f, syncer.accelerometerCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.gravityCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.magnetometerCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.accelerometerUsage, 0.0f)
        assertEquals(0.0f, syncer.gravityUsage, 0.0f)
        assertEquals(0.0f, syncer.magnetometerUsage, 0.0f)
    }

    @Test
    fun accuracyChangedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(context)

        // check default value
        assertNull(syncer.accuracyChangedListener)

        // set new value
        val accuracyChangedListener =
            mockk<SensorMeasurementSyncer.OnAccuracyChangedListener<AccelerometerGravityAndMagnetometerSyncedSensorMeasurement, AccelerometerGravityAndMagnetometerSensorMeasurementSyncer>>()
        syncer.accuracyChangedListener = accuracyChangedListener

        // check
        assertSame(accuracyChangedListener, syncer.accuracyChangedListener)
    }

    @Test
    fun bufferFilledListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(context)

        // check default value
        assertNull(syncer.bufferFilledListener)

        // set new value
        val bufferFilledListener =
            mockk<SensorMeasurementSyncer.OnBufferFilledListener<AccelerometerGravityAndMagnetometerSyncedSensorMeasurement, AccelerometerGravityAndMagnetometerSensorMeasurementSyncer>>()
        syncer.bufferFilledListener = bufferFilledListener

        // check
        assertSame(bufferFilledListener, syncer.bufferFilledListener)
    }

    @Test
    fun syncedMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(context)

        // check default value
        assertNull(syncer.syncedMeasurementListener)

        // set new value
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerGravityAndMagnetometerSyncedSensorMeasurement, AccelerometerGravityAndMagnetometerSensorMeasurementSyncer>>()
        syncer.syncedMeasurementListener = syncedMeasurementListener

        // check
        assertSame(syncedMeasurementListener, syncer.syncedMeasurementListener)
    }

    @Test
    fun staleDetectedMeasurementsListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(context)

        // check default value
        assertNull(syncer.staleDetectedMeasurementsListener)

        // set new value
        val staleDetectedMeasurementsListener =
            mockk<SensorMeasurementSyncer.OnStaleDetectedMeasurementsListener<AccelerometerGravityAndMagnetometerSyncedSensorMeasurement, AccelerometerGravityAndMagnetometerSensorMeasurementSyncer>>()
        syncer.staleDetectedMeasurementsListener = staleDetectedMeasurementsListener

        // check
        assertSame(staleDetectedMeasurementsListener, syncer.staleDetectedMeasurementsListener)
    }

    @Test
    fun accelerometerSensor_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(context)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        assertNull(syncer.accelerometerSensor)
        verify(exactly = 1) { accelerometerSensorCollectorSpy.sensor }
    }

    @Test
    fun gravitySensor_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(context)

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)

        assertNull(syncer.gravitySensor)
        verify(exactly = 1) { gravitySensorCollectorSpy.sensor }
    }

    @Test
    fun magnetometerSensor_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(context)

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
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(context)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        assertFalse(syncer.accelerometerSensorAvailable)
        verify(exactly = 1) { accelerometerSensorCollectorSpy.sensorAvailable }
    }

    @Test
    fun gravitySensorAvailable_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(context)

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)

        assertFalse(syncer.gravitySensorAvailable)
        verify(exactly = 1) { gravitySensorCollectorSpy.sensorAvailable }
    }

    @Test
    fun magnetometerSensorAvailable_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(context)

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
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(context)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        assertNull(syncer.accelerometerStartOffset)
        verify(exactly = 1) { accelerometerSensorCollectorSpy.startOffset }
    }

    @Test
    fun gravityStartOffset_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(context)

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)

        assertNull(syncer.gravityStartOffset)
        verify(exactly = 1) { gravitySensorCollectorSpy.startOffset }
    }

    @Test
    fun magnetometerStartOffset_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(context)

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
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(context)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        assertEquals(0.0f, syncer.accelerometerCollectorUsage, 0.0f)
        verify(exactly = 1) { accelerometerSensorCollectorSpy.usage }
    }

    @Test
    fun gravityCollectorUsage_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(context)

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)

        assertEquals(0.0f, syncer.gravityCollectorUsage, 0.0f)
        verify(exactly = 1) { gravitySensorCollectorSpy.usage }
    }

    @Test
    fun magnetometerCollectorUsage_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(context)

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
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(context)

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
    fun gravityUsage_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(context)

        // check default value
        assertEquals(0.0f, syncer.gravityUsage, 0.0f)

        // add measurement
        val gravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("gravityMeasurements")
        requireNotNull(gravityMeasurements)
        assertTrue(gravityMeasurements.isEmpty())
        gravityMeasurements.add(GravitySensorMeasurement())

        // check
        assertEquals(1.0f / syncer.gravityCapacity.toFloat(), syncer.gravityUsage, 0.0f)
    }

    @Test
    fun magnetometerUsage_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(context)

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
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(context)

        // set as running
        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "running", true)
        assertTrue(syncer.running)

        syncer.start()
    }

    @Test
    fun start_whenNotRunningAndNoTimestamp_setsCurrentStartTimestamp() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(context)

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
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(context)

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
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(context)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.start(any()) }.returns(false)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)
        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)
        every { gravitySensorCollectorSpy.start(any()) }.returns(true)
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
        verify(exactly = 0) { gravitySensorCollectorSpy.start(any()) }
        verify(exactly = 0) { magnetometerSensorCollectorSpy.start(any()) }
    }

    @Test
    fun start_whenGravityCollectorFails_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(context)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.start(any()) }.returns(true)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)
        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)
        every { gravitySensorCollectorSpy.start(any()) }.returns(false)
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)
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
        verify(exactly = 1) { gravitySensorCollectorSpy.start(timestamp) }
        verify(exactly = 0) { magnetometerSensorCollectorSpy.start(any()) }
    }

    @Test
    fun start_whenMagnetometerCollectorFails_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(context)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.start(any()) }.returns(true)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)
        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)
        every { gravitySensorCollectorSpy.start(any()) }.returns(true)
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)
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
        verify(exactly = 1) { gravitySensorCollectorSpy.start(timestamp) }
        verify(exactly = 1) { magnetometerSensorCollectorSpy.start(any()) }
    }

    @Test
    fun start_whenCollectorsSucceed_returnsTrue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(context)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.start(any()) }.returns(true)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)
        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)
        every { gravitySensorCollectorSpy.start(any()) }.returns(true)
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)
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
        verify(exactly = 1) { gravitySensorCollectorSpy.start(timestamp) }
        verify(exactly = 1) { magnetometerSensorCollectorSpy.start(any()) }
    }

    @Test
    fun stop_stopsCollectorsInitializesCachesAndResetsProperties() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(context)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)
        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)
        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        val accelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("accelerometerMeasurements")
        requireNotNull(accelerometerMeasurements)
        accelerometerMeasurements.add(AccelerometerSensorMeasurement())

        val gravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("gravityMeasurements")
        requireNotNull(gravityMeasurements)
        gravityMeasurements.add(GravitySensorMeasurement())

        val magnetometerMeasurements: ArrayDeque<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("magnetometerMeasurements")
        requireNotNull(magnetometerMeasurements)
        magnetometerMeasurements.add(MagnetometerSensorMeasurement())

        val foundGravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("foundGravityMeasurements")
        requireNotNull(foundGravityMeasurements)
        foundGravityMeasurements.add(GravitySensorMeasurement())

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
        syncer.setPrivateProperty("hasPreviousGravityMeasurement", true)
        syncer.setPrivateProperty("hasPreviousMagnetometerMeasurement", true)
        syncer.setPrivateProperty("lastNotifiedTimestamp", 1L)
        syncer.setPrivateProperty("lastNotifiedAccelerometerTimestamp", 2L)
        syncer.setPrivateProperty("lastNotifiedGravityTimestamp", 4L)
        syncer.setPrivateProperty("lastNotifiedMagnetometerTimestamp", 3L)

        syncer.stop()

        // check
        verify(exactly = 1) { accelerometerSensorCollectorSpy.stop() }
        verify(exactly = 1) { gravitySensorCollectorSpy.stop() }
        verify(exactly = 1) { magnetometerSensorCollectorSpy.stop() }
        assertTrue(accelerometerMeasurements.isEmpty())
        assertTrue(gravityMeasurements.isEmpty())
        assertTrue(magnetometerMeasurements.isEmpty())
        assertTrue(foundGravityMeasurements.isEmpty())
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

        val hasPreviousGravityMeasurement: Boolean? =
            syncer.getPrivateProperty("hasPreviousGravityMeasurement")
        requireNotNull(hasPreviousGravityMeasurement)
        assertFalse(hasPreviousGravityMeasurement)

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

        val lastNotifiedGravityTimestamp: Long? =
            syncer.getPrivateProperty("lastNotifiedGravityTimestamp")
        requireNotNull(lastNotifiedGravityTimestamp)
        assertEquals(0L, lastNotifiedGravityTimestamp)

        val lastNotifiedMagnetometerTimestamp: Long? =
            syncer.getPrivateProperty("lastNotifiedMagnetometerTimestamp")
        requireNotNull(lastNotifiedMagnetometerTimestamp)
        assertEquals(0L, lastNotifiedMagnetometerTimestamp)
    }

    @Test
    fun accelerometerSensorCollector_whenAccuracyChangedListenerAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(context)

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
            mockk<SensorMeasurementSyncer.OnAccuracyChangedListener<AccelerometerGravityAndMagnetometerSyncedSensorMeasurement, AccelerometerGravityAndMagnetometerSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(
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
            AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(
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

    @Test
    fun accelerometerSensorCollector_whenBufferFilledAndListenerAvailable_notifies() {
        val bufferFilledListener =
            mockk<SensorMeasurementSyncer.OnBufferFilledListener<AccelerometerGravityAndMagnetometerSyncedSensorMeasurement, AccelerometerGravityAndMagnetometerSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(
            context,
            stopWhenFilledBuffer = false,
            bufferFilledListener = bufferFilledListener
        )

        // set as running
        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "running", true)
        assertTrue(syncer.running)
        assertFalse(syncer.stopWhenFilledBuffer)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)

        val listener = accelerometerSensorCollector.bufferFilledListener
        requireNotNull(listener)

        listener.onBufferFilled(accelerometerSensorCollector)

        verify(exactly = 1) {
            bufferFilledListener.onBufferFilled(
                syncer,
                SensorMeasurementSyncer.SensorType.ACCELEROMETER_UNCALIBRATED
            )
        }
        assertTrue(syncer.running)
    }

    @Test
    fun accelerometerSensorCollector_whenMeasurementAndNoMeasurementsAvailable_callsCollector() {
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerGravityAndMagnetometerSyncedSensorMeasurement, AccelerometerGravityAndMagnetometerSensorMeasurementSyncer>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(
            context,
            syncedMeasurementListener = syncedMeasurementListener
        )

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(any()) }.returns(
            ArrayDeque()
        )
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        val listener = accelerometerSensorCollectorSpy.measurementListener
        requireNotNull(listener)

        listener.onMeasurement(accelerometerSensorCollectorSpy, AccelerometerSensorMeasurement(), 0)

        verify(exactly = 1) { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(0) }
        verify { syncedMeasurementListener wasNot Called }
        assertNull(syncer.mostRecentTimestamp)
    }

    @Test
    fun accelerometerSensorCollector_whenMeasurementAndMeasurementsAvailable_updatesMostRecentTimestampCopiesMeasurementsAndProcessesThem() {
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerGravityAndMagnetometerSyncedSensorMeasurement, AccelerometerGravityAndMagnetometerSensorMeasurementSyncer>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(
            context,
            syncedMeasurementListener = syncedMeasurementListener
        )

        assertNull(syncer.mostRecentTimestamp)
        assertNull(syncer.oldestTimestamp)

        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "running", true)
        assertTrue(syncer.running)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()
        val accelerometerMeasurement =
            AccelerometerSensorMeasurement(ax, ay, az, bx, by, bz, timestamp, SensorAccuracy.HIGH)
        val measurementsBeforePosition = ArrayDeque<AccelerometerSensorMeasurement>()
        measurementsBeforePosition.add(accelerometerMeasurement)
        every { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(any()) }.returns(
            measurementsBeforePosition
        )
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        val listener = accelerometerSensorCollectorSpy.measurementListener
        requireNotNull(listener)

        listener.onMeasurement(accelerometerSensorCollectorSpy, AccelerometerSensorMeasurement(), 0)

        verify(exactly = 1) { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(0) }
        assertEquals(timestamp, syncer.mostRecentTimestamp)

        val accelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("accelerometerMeasurements")
        requireNotNull(accelerometerMeasurements)
        assertEquals(1, accelerometerMeasurements.size)
        val accelerometerMeasurement2 = accelerometerMeasurements.peek()
        requireNotNull(accelerometerMeasurement2)
        assertNotSame(accelerometerMeasurement, accelerometerMeasurement2)
        assertEquals(ax, accelerometerMeasurement2.ax, 0.0f)
        assertEquals(ay, accelerometerMeasurement2.ay, 0.0f)
        assertEquals(az, accelerometerMeasurement2.az, 0.0f)
        assertEquals(bx, accelerometerMeasurement2.bx)
        assertEquals(by, accelerometerMeasurement2.by)
        assertEquals(bz, accelerometerMeasurement2.bz)
        assertEquals(timestamp, accelerometerMeasurement2.timestamp)
        assertEquals(SensorAccuracy.HIGH, accelerometerMeasurement2.accuracy)

        assertEquals(timestamp, syncer.oldestTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)

        val alreadyProcessedAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())

        val alreadyProcessedMagnetometerMeasurements: ArrayDeque<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedMagnetometerMeasurements")
        requireNotNull(alreadyProcessedMagnetometerMeasurements)
        assertTrue(alreadyProcessedMagnetometerMeasurements.isEmpty())

        verify { syncedMeasurementListener wasNot Called }
        assertTrue(syncer.running)
    }

    @Test
    fun gravitySensorCollector_whenAccuracyChangedListenerAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(context)

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)

        val listener = gravitySensorCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(gravitySensorCollector, SensorAccuracy.HIGH)
    }

    @Test
    fun gravitySensorCollector_whenAccuracyChangedListenerAndListenerAvailable_notifies() {
        val accuracyChangedListener =
            mockk<SensorMeasurementSyncer.OnAccuracyChangedListener<AccelerometerGravityAndMagnetometerSyncedSensorMeasurement, AccelerometerGravityAndMagnetometerSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)

        val listener = gravitySensorCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(gravitySensorCollector, SensorAccuracy.HIGH)

        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                syncer,
                SensorMeasurementSyncer.SensorType.GRAVITY,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun gravitySensorCollector_whenBufferFilledAndStopWhenFilledBuffer_stops() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer =
            AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(
                context,
                stopWhenFilledBuffer = true
            )

        // set as running
        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "running", true)
        assertTrue(syncer.running)
        assertTrue(syncer.stopWhenFilledBuffer)

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)

        val listener = gravitySensorCollector.bufferFilledListener
        requireNotNull(listener)

        listener.onBufferFilled(gravitySensorCollector)

        assertFalse(syncer.running)
    }

    @Test
    fun gravitySensorCollector_whenBufferFilledAndListenerAvailable_notifies() {
        val bufferFilledListener =
            mockk<SensorMeasurementSyncer.OnBufferFilledListener<AccelerometerGravityAndMagnetometerSyncedSensorMeasurement, AccelerometerGravityAndMagnetometerSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(
            context,
            stopWhenFilledBuffer = false,
            bufferFilledListener = bufferFilledListener
        )

        // set as running
        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "running", true)
        assertTrue(syncer.running)
        assertFalse(syncer.stopWhenFilledBuffer)

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)

        val listener = gravitySensorCollector.bufferFilledListener
        requireNotNull(listener)

        listener.onBufferFilled(gravitySensorCollector)

        verify(exactly = 1) {
            bufferFilledListener.onBufferFilled(
                syncer,
                SensorMeasurementSyncer.SensorType.GRAVITY
            )
        }
        assertTrue(syncer.running)
    }

    @Test
    fun gravitySensorCollector_whenMeasurementAndMostRecentTimestampNotDefined_makesNoAction() {
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerGravityAndMagnetometerSyncedSensorMeasurement, AccelerometerGravityAndMagnetometerSensorMeasurementSyncer>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(
            context,
            syncedMeasurementListener = syncedMeasurementListener
        )

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)

        val listener = gravitySensorCollector.measurementListener
        requireNotNull(listener)

        listener.onMeasurement(gravitySensorCollectorSpy, GravitySensorMeasurement(), 0)

        verify { gravitySensorCollectorSpy wasNot Called }
        verify { syncedMeasurementListener wasNot Called }
        assertNull(syncer.mostRecentTimestamp)
    }

    @Test
    fun gravitySensorCollector_whenMeasurementMostRecentTimestampDefinedAndNoMeasurementsAvailable_callsCollector() {
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerGravityAndMagnetometerSyncedSensorMeasurement, AccelerometerGravityAndMagnetometerSensorMeasurementSyncer>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(
            context,
            syncedMeasurementListener = syncedMeasurementListener
        )

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)
        every { gravitySensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            ArrayDeque()
        )
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)

        // set most recent timestamp
        val mostRecentTimestamp = System.nanoTime()
        setPrivateProperty(
            SensorMeasurementSyncer::class,
            syncer,
            "mostRecentTimestamp",
            mostRecentTimestamp
        )

        val listener = gravitySensorCollector.measurementListener
        requireNotNull(listener)

        listener.onMeasurement(gravitySensorCollectorSpy, GravitySensorMeasurement(), 0)

        verify(exactly = 1) {
            gravitySensorCollectorSpy.getMeasurementsBeforeTimestamp(
                mostRecentTimestamp
            )
        }
        verify { syncedMeasurementListener wasNot Called }
        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
    }

    @Test
    fun gravitySensorCollector_whenMeasurementMostRecentTimestampDefinedAndMeasurementsAvailable_copiesMeasurements() {
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerGravityAndMagnetometerSyncedSensorMeasurement, AccelerometerGravityAndMagnetometerSensorMeasurementSyncer>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(
            context,
            syncedMeasurementListener = syncedMeasurementListener
        )

        assertNull(syncer.mostRecentTimestamp)
        assertNull(syncer.oldestTimestamp)

        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "running", true)
        assertTrue(syncer.running)

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)

        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val timestamp = System.nanoTime()
        val gravityMeasurement =
            GravitySensorMeasurement(gx, gy, gz, timestamp, SensorAccuracy.HIGH)
        val measurementsBeforeTimestamp = ArrayDeque<GravitySensorMeasurement>()
        measurementsBeforeTimestamp.add(gravityMeasurement)
        every { gravitySensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            measurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)

        // set most recent timestamp
        val mostRecentTimestamp = System.nanoTime()
        setPrivateProperty(
            SensorMeasurementSyncer::class,
            syncer,
            "mostRecentTimestamp",
            mostRecentTimestamp
        )

        val listener = gravitySensorCollector.measurementListener
        requireNotNull(listener)

        listener.onMeasurement(gravitySensorCollectorSpy, GravitySensorMeasurement(), 0)

        verify(exactly = 1) {
            gravitySensorCollectorSpy.getMeasurementsBeforeTimestamp(
                mostRecentTimestamp
            )
        }

        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
        val gravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("gravityMeasurements")
        requireNotNull(gravityMeasurements)
        assertEquals(1, gravityMeasurements.size)
        val gravityMeasurement2 = gravityMeasurements.peek()
        requireNotNull(gravityMeasurement2)
        assertNotSame(gravityMeasurement, gravityMeasurement2)
        assertEquals(gx, gravityMeasurement2.gx, 0.0f)
        assertEquals(gy, gravityMeasurement2.gy, 0.0f)
        assertEquals(gz, gravityMeasurement2.gz, 0.0f)
        assertEquals(timestamp, gravityMeasurement2.timestamp)
        assertEquals(SensorAccuracy.HIGH, gravityMeasurement2.accuracy)

        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
        assertNull(syncer.oldestTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)

        val alreadyProcessedAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())

        val alreadyProcessedGravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGravityMeasurements")
        requireNotNull(alreadyProcessedGravityMeasurements)
        assertTrue(alreadyProcessedGravityMeasurements.isEmpty())

        verify { syncedMeasurementListener wasNot Called }
        assertTrue(syncer.running)
    }

    @Test
    fun magnetometerSensorCollector_whenAccuracyChangedListenerAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(context)

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)

        val listener = magnetometerSensorCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(magnetometerSensorCollector, SensorAccuracy.MEDIUM)
    }

    @Test
    fun magnetometerSensorCollector_whenAccuracyChangedListenerAndListenerAvailable_notifies() {
        val accuracyChangedListener =
            mockk<SensorMeasurementSyncer.OnAccuracyChangedListener<AccelerometerGravityAndMagnetometerSyncedSensorMeasurement, AccelerometerGravityAndMagnetometerSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)

        val listener = magnetometerSensorCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(magnetometerSensorCollector, SensorAccuracy.MEDIUM)

        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                syncer,
                SensorMeasurementSyncer.SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorAccuracy.MEDIUM
            )
        }
    }

    @Test
    fun magnetometerSensorCollector_whenBufferFilledAndStopWhenFilledBuffer_stops() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(
            context,
            stopWhenFilledBuffer = true
        )

        // set as running
        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "running", true)
        assertTrue(syncer.running)
        assertTrue(syncer.stopWhenFilledBuffer)

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)

        val listener = magnetometerSensorCollector.bufferFilledListener
        requireNotNull(listener)

        listener.onBufferFilled(magnetometerSensorCollector)

        assertFalse(syncer.running)
    }

    @Test
    fun magnetometerSensorCollector_whenBufferFilledAndListenerAvailable_notifies() {
        val bufferFilledListener =
            mockk<SensorMeasurementSyncer.OnBufferFilledListener<AccelerometerGravityAndMagnetometerSyncedSensorMeasurement, AccelerometerGravityAndMagnetometerSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(
            context,
            stopWhenFilledBuffer = false,
            bufferFilledListener = bufferFilledListener
        )

        // set as running
        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "running", true)
        assertTrue(syncer.running)
        assertFalse(syncer.stopWhenFilledBuffer)

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)

        val listener = magnetometerSensorCollector.bufferFilledListener
        requireNotNull(listener)

        listener.onBufferFilled(magnetometerSensorCollector)

        verify(exactly = 1) {
            bufferFilledListener.onBufferFilled(
                syncer,
                SensorMeasurementSyncer.SensorType.MAGNETOMETER_UNCALIBRATED
            )
        }
        assertTrue(syncer.running)
    }

    @Test
    fun magnetometerSensorCollector_whenMeasurementAndMostRecentTimestampNotDefined_makesNoAction() {
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerGravityAndMagnetometerSyncedSensorMeasurement, AccelerometerGravityAndMagnetometerSensorMeasurementSyncer>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(
            context,
            syncedMeasurementListener = syncedMeasurementListener
        )

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        val listener = magnetometerSensorCollector.measurementListener
        requireNotNull(listener)

        listener.onMeasurement(magnetometerSensorCollectorSpy, MagnetometerSensorMeasurement(), 0)

        verify { magnetometerSensorCollectorSpy wasNot Called }
        verify { syncedMeasurementListener wasNot Called }
        assertNull(syncer.mostRecentTimestamp)
    }

    @Test
    fun magnetometerSensorCollector_whenMeasurementMostRecentTimestampDefinedAndNoMeasurementsAvailable_callsCollector() {
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerGravityAndMagnetometerSyncedSensorMeasurement, AccelerometerGravityAndMagnetometerSensorMeasurementSyncer>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(
            context,
            syncedMeasurementListener = syncedMeasurementListener
        )

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)
        every { magnetometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            ArrayDeque()
        )
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        // set most recent timestamp
        val mostRecentTimestamp = System.nanoTime()
        setPrivateProperty(
            SensorMeasurementSyncer::class,
            syncer,
            "mostRecentTimestamp",
            mostRecentTimestamp
        )

        val listener = magnetometerSensorCollector.measurementListener
        requireNotNull(listener)

        listener.onMeasurement(magnetometerSensorCollectorSpy, MagnetometerSensorMeasurement(), 0)

        verify(exactly = 1) {
            magnetometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(
                mostRecentTimestamp
            )
        }
        verify { syncedMeasurementListener wasNot Called }
        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
    }

    @Test
    fun magnetometerSensorCollector_whenMeasurementMostRecentTimestampDefinedAndMeasurementsAvailable_copiesMeasurements() {
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerGravityAndMagnetometerSyncedSensorMeasurement, AccelerometerGravityAndMagnetometerSensorMeasurementSyncer>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(
            context,
            syncedMeasurementListener = syncedMeasurementListener
        )

        assertNull(syncer.mostRecentTimestamp)
        assertNull(syncer.oldestTimestamp)

        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "running", true)
        assertTrue(syncer.running)

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)

        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = System.nanoTime()
        val magnetometerMeasurement = MagnetometerSensorMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            timestamp,
            SensorAccuracy.HIGH
        )
        val measurementsBeforeTimestamp = ArrayDeque<MagnetometerSensorMeasurement>()
        measurementsBeforeTimestamp.add(magnetometerMeasurement)
        every { magnetometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            measurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        // set most recent timestamp
        val mostRecentTimestamp = System.nanoTime()
        setPrivateProperty(
            SensorMeasurementSyncer::class,
            syncer,
            "mostRecentTimestamp",
            mostRecentTimestamp
        )

        val listener = magnetometerSensorCollector.measurementListener
        requireNotNull(listener)

        listener.onMeasurement(magnetometerSensorCollectorSpy, MagnetometerSensorMeasurement(), 0)

        verify(exactly = 1) {
            magnetometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(
                mostRecentTimestamp
            )
        }

        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
        val magnetometerMeasurements: ArrayDeque<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("magnetometerMeasurements")
        requireNotNull(magnetometerMeasurements)
        assertEquals(1, magnetometerMeasurements.size)
        val magnetometerMeasurement2 = magnetometerMeasurements.peek()
        requireNotNull(magnetometerMeasurement2)
        assertNotSame(magnetometerMeasurement, magnetometerMeasurement2)
        assertEquals(bx, magnetometerMeasurement2.bx, 0.0f)
        assertEquals(by, magnetometerMeasurement2.by, 0.0f)
        assertEquals(bz, magnetometerMeasurement2.bz, 0.0f)
        assertEquals(hardIronX, magnetometerMeasurement2.hardIronX)
        assertEquals(hardIronY, magnetometerMeasurement2.hardIronY)
        assertEquals(hardIronZ, magnetometerMeasurement2.hardIronZ)
        assertEquals(timestamp, magnetometerMeasurement2.timestamp)
        assertEquals(SensorAccuracy.HIGH, magnetometerMeasurement2.accuracy)

        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
        assertNull(syncer.oldestTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)

        val alreadyProcessedAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())

        val alreadyProcessedMagnetometerMeasurements: ArrayDeque<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedMagnetometerMeasurements")
        requireNotNull(alreadyProcessedMagnetometerMeasurements)
        assertTrue(alreadyProcessedMagnetometerMeasurements.isEmpty())

        verify { syncedMeasurementListener wasNot Called }
        assertTrue(syncer.running)
    }

    @Test
    fun sensorCollectors_whenAllMeasurements_notifiesSyncedMeasurement() {
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerGravityAndMagnetometerSyncedSensorMeasurement, AccelerometerGravityAndMagnetometerSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(
            context,
            syncedMeasurementListener = syncedMeasurementListener
        )

        assertNull(syncer.mostRecentTimestamp)
        assertNull(syncer.oldestTimestamp)

        // set as running
        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "running", true)
        assertTrue(syncer.running)

        // set most recent timestamp
        val mostRecentTimestamp = System.nanoTime()
        val accelerometerTimestamp = mostRecentTimestamp - 1
        setPrivateProperty(
            SensorMeasurementSyncer::class,
            syncer,
            "mostRecentTimestamp",
            mostRecentTimestamp
        )

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)

        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val magnetometerTimestamp = accelerometerTimestamp - 1
        val magnetometerMeasurement1 = MagnetometerSensorMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            magnetometerTimestamp,
            SensorAccuracy.MEDIUM
        )
        val magnetometerMeasurementsBeforeTimestamp = ArrayDeque<MagnetometerSensorMeasurement>()
        magnetometerMeasurementsBeforeTimestamp.add(magnetometerMeasurement1)
        every { magnetometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            magnetometerMeasurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)

        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val gravityTimestamp = accelerometerTimestamp - 1
        val gravityMeasurement1 =
            GravitySensorMeasurement(gx, gy, gz, gravityTimestamp, SensorAccuracy.HIGH)
        val gravityMeasurementsBeforeTimestamp = ArrayDeque<GravitySensorMeasurement>()
        gravityMeasurementsBeforeTimestamp.add(gravityMeasurement1)
        every { gravitySensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            gravityMeasurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)

        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val accelerometerMeasurement1 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            abx,
            aby,
            abz,
            accelerometerTimestamp,
            SensorAccuracy.HIGH
        )
        val measurementsBeforePosition = ArrayDeque<AccelerometerSensorMeasurement>()
        measurementsBeforePosition.add(accelerometerMeasurement1)
        every { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(any()) }.returns(
            measurementsBeforePosition
        )
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        val accelerometerListener = accelerometerSensorCollectorSpy.measurementListener
        requireNotNull(accelerometerListener)

        val gravityListener = gravitySensorCollector.measurementListener
        requireNotNull(gravityListener)

        val magnetometerListener = magnetometerSensorCollector.measurementListener
        requireNotNull(magnetometerListener)

        // process magnetometer measurement
        magnetometerListener.onMeasurement(
            magnetometerSensorCollectorSpy,
            MagnetometerSensorMeasurement(),
            0
        )

        assertNull(syncer.oldestTimestamp)
        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        assertTrue(syncer.running)

        val magnetometerMeasurements: ArrayDeque<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("magnetometerMeasurements")
        requireNotNull(magnetometerMeasurements)
        assertEquals(1, magnetometerMeasurements.size)
        val magnetometerMeasurement2 = magnetometerMeasurements.peek()
        requireNotNull(magnetometerMeasurement2)
        assertNotSame(magnetometerMeasurement1, magnetometerMeasurement2)
        assertEquals(bx, magnetometerMeasurement2.bx, 0.0f)
        assertEquals(by, magnetometerMeasurement2.by, 0.0f)
        assertEquals(bz, magnetometerMeasurement2.bz, 0.0f)
        assertEquals(hardIronX, magnetometerMeasurement2.hardIronX)
        assertEquals(hardIronY, magnetometerMeasurement2.hardIronY)
        assertEquals(hardIronZ, magnetometerMeasurement2.hardIronZ)
        assertEquals(magnetometerTimestamp, magnetometerMeasurement2.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, magnetometerMeasurement2.accuracy)

        val alreadyProcessedMagnetometerMeasurements: ArrayDeque<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedMagnetometerMeasurements")
        requireNotNull(alreadyProcessedMagnetometerMeasurements)
        assertTrue(alreadyProcessedMagnetometerMeasurements.isEmpty())

        // process gravity measurement
        gravityListener.onMeasurement(gravitySensorCollectorSpy, GravitySensorMeasurement(), 0)

        assertNull(syncer.oldestTimestamp)
        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        assertTrue(syncer.running)

        val gravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("gravityMeasurements")
        requireNotNull(gravityMeasurements)
        assertEquals(1, gravityMeasurements.size)
        val gravityMeasurement2 = gravityMeasurements.peek()
        requireNotNull(gravityMeasurement2)
        assertNotSame(gravityMeasurement1, gravityMeasurement2)
        assertEquals(gx, gravityMeasurement2.gx, 0.0f)
        assertEquals(gy, gravityMeasurement2.gy, 0.0f)
        assertEquals(gz, gravityMeasurement2.gz, 0.0f)
        assertEquals(gravityTimestamp, gravityMeasurement2.timestamp)
        assertEquals(SensorAccuracy.HIGH, gravityMeasurement2.accuracy)

        val alreadyProcessedGravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGravityMeasurements")
        requireNotNull(alreadyProcessedGravityMeasurements)
        assertTrue(alreadyProcessedGravityMeasurements.isEmpty())

        // process accelerometer measurement
        accelerometerListener.onMeasurement(
            accelerometerSensorCollectorSpy,
            AccelerometerSensorMeasurement(),
            0
        )

        assertEquals(accelerometerTimestamp, syncer.oldestTimestamp)
        assertEquals(accelerometerTimestamp, syncer.mostRecentTimestamp)
        assertEquals(1, syncer.numberOfProcessedMeasurements)
        assertTrue(syncer.running)
        verify(exactly = 1) { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(0) }

        val accelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("accelerometerMeasurements")
        requireNotNull(accelerometerMeasurements)
        assertTrue(accelerometerMeasurements.isEmpty())
        assertTrue(magnetometerMeasurements.isEmpty())

        val alreadyProcessedAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())
        assertTrue(alreadyProcessedMagnetometerMeasurements.isEmpty())

        val slot = slot<AccelerometerGravityAndMagnetometerSyncedSensorMeasurement>()
        verify(exactly = 1) {
            syncedMeasurementListener.onSyncedMeasurements(
                syncer,
                capture(slot)
            )
        }

        val syncedMeasurement = slot.captured
        assertEquals(magnetometerTimestamp, syncedMeasurement.timestamp)
        val syncedAccelerometerMeasurement = syncedMeasurement.accelerometerMeasurement
        requireNotNull(syncedAccelerometerMeasurement)
        assertEquals(ax, syncedAccelerometerMeasurement.ax, 0.0f)
        assertEquals(ay, syncedAccelerometerMeasurement.ay, 0.0f)
        assertEquals(az, syncedAccelerometerMeasurement.az, 0.0f)
        assertEquals(abx, syncedAccelerometerMeasurement.bx)
        assertEquals(aby, syncedAccelerometerMeasurement.by)
        assertEquals(abz, syncedAccelerometerMeasurement.bz)
        assertEquals(accelerometerTimestamp, syncedAccelerometerMeasurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, syncedAccelerometerMeasurement.accuracy)
        val syncedGravityMeasurement = syncedMeasurement.gravityMeasurement
        requireNotNull(syncedGravityMeasurement)
        assertEquals(gx, syncedGravityMeasurement.gx, 0.0f)
        assertEquals(gy, syncedGravityMeasurement.gy, 0.0f)
        assertEquals(gz, syncedGravityMeasurement.gz, 0.0f)
        assertEquals(gravityTimestamp, syncedGravityMeasurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, syncedGravityMeasurement.accuracy)
        val syncedMagnetometerMeasurement = syncedMeasurement.magnetometerMeasurement
        requireNotNull(syncedMagnetometerMeasurement)
        assertEquals(bx, syncedMagnetometerMeasurement.bx, 0.0f)
        assertEquals(by, syncedMagnetometerMeasurement.by, 0.0f)
        assertEquals(bz, syncedMagnetometerMeasurement.bz, 0.0f)
        assertEquals(hardIronX, syncedMagnetometerMeasurement.hardIronX)
        assertEquals(hardIronY, syncedMagnetometerMeasurement.hardIronY)
        assertEquals(hardIronZ, syncedMagnetometerMeasurement.hardIronZ)
        assertEquals(magnetometerTimestamp, syncedMagnetometerMeasurement.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, syncedMagnetometerMeasurement.accuracy)
    }

    @Test
    fun cleanupStaleMeasurements_whenStaleMeasurementsAvailable_notifies() {
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerGravityAndMagnetometerSyncedSensorMeasurement, AccelerometerGravityAndMagnetometerSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val staleDetectedMeasurementsListener =
            mockk<SensorMeasurementSyncer.OnStaleDetectedMeasurementsListener<AccelerometerGravityAndMagnetometerSyncedSensorMeasurement, AccelerometerGravityAndMagnetometerSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(
            context,
            syncedMeasurementListener = syncedMeasurementListener,
            staleDetectedMeasurementsListener = staleDetectedMeasurementsListener
        )

        assertTrue(syncer.staleDetectionEnabled)
        assertNull(syncer.mostRecentTimestamp)
        assertNull(syncer.oldestTimestamp)

        // set as running
        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "running", true)
        assertTrue(syncer.running)

        // set most recent timestamp
        val mostRecentTimestamp = System.nanoTime()
        val accelerometerTimestamp = mostRecentTimestamp - 1
        setPrivateProperty(
            SensorMeasurementSyncer::class,
            syncer,
            "mostRecentTimestamp",
            mostRecentTimestamp
        )

        // add stale measurements
        val staleTimestamp =
            mostRecentTimestamp - 2 * AccelerometerAndMagnetometerSensorMeasurementSyncer.DEFAULT_STALE_OFFSET_NANOS
        val magnetometerMeasurements: ArrayDeque<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("magnetometerMeasurements")
        requireNotNull(magnetometerMeasurements)
        val randomizer = UniformRandomizer()
        val bx1 = randomizer.nextFloat()
        val by1 = randomizer.nextFloat()
        val bz1 = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val staleMagnetometerMeasurement = MagnetometerSensorMeasurement(
            bx1,
            by1,
            bz1,
            hardIronX,
            hardIronY,
            hardIronZ,
            staleTimestamp,
            SensorAccuracy.MEDIUM
        )
        magnetometerMeasurements.add(staleMagnetometerMeasurement)

        val gravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("gravityMeasurements")
        requireNotNull(gravityMeasurements)
        val gx1 = randomizer.nextFloat()
        val gy1 = randomizer.nextFloat()
        val gz1 = randomizer.nextFloat()
        val staleGravityMeasurement =
            GravitySensorMeasurement(gx1, gy1, gz1, staleTimestamp, SensorAccuracy.MEDIUM)
        gravityMeasurements.add(staleGravityMeasurement)

        val accelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("accelerometerMeasurements")
        requireNotNull(accelerometerMeasurements)
        val ax1 = randomizer.nextFloat()
        val ay1 = randomizer.nextFloat()
        val az1 = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val staleAccelerometerMeasurement = AccelerometerSensorMeasurement(
            ax1,
            ay1,
            az1,
            abx,
            aby,
            abz,
            staleTimestamp,
            SensorAccuracy.HIGH
        )
        accelerometerMeasurements.add(staleAccelerometerMeasurement)

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)

        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val magnetometerTimestamp = accelerometerTimestamp - 1
        val magnetometerMeasurement1 = MagnetometerSensorMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            magnetometerTimestamp,
            SensorAccuracy.MEDIUM
        )
        val magnetometerMeasurementsBeforeTimestamp = ArrayDeque<MagnetometerSensorMeasurement>()
        magnetometerMeasurementsBeforeTimestamp.add(magnetometerMeasurement1)
        every { magnetometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            magnetometerMeasurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)

        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val gravityTimestamp = accelerometerTimestamp - 1
        val gravityMeasurement1 =
            GravitySensorMeasurement(gx, gy, gz, gravityTimestamp, SensorAccuracy.MEDIUM)
        val gravityMeasurementsBeforeTimestamp = ArrayDeque<GravitySensorMeasurement>()
        gravityMeasurementsBeforeTimestamp.add(gravityMeasurement1)
        every { gravitySensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            gravityMeasurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)

        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val accelerometerMeasurement1 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            abx,
            aby,
            abz,
            accelerometerTimestamp,
            SensorAccuracy.HIGH
        )
        val measurementsBeforePosition = ArrayDeque<AccelerometerSensorMeasurement>()
        measurementsBeforePosition.add(accelerometerMeasurement1)
        every { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(any()) }.returns(
            measurementsBeforePosition
        )
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        // set previous measurements
        syncer.setPrivateProperty("hasPreviousAccelerometerMeasurement", true)
        val previousAccelerometerMeasurement: AccelerometerSensorMeasurement? =
            syncer.getPrivateProperty("previousAccelerometerMeasurement")
        requireNotNull(previousAccelerometerMeasurement)
        previousAccelerometerMeasurement.timestamp = accelerometerTimestamp - 1

        val accelerometerListener = accelerometerSensorCollectorSpy.measurementListener
        requireNotNull(accelerometerListener)

        val gravityListener = gravitySensorCollectorSpy.measurementListener
        requireNotNull(gravityListener)

        val magnetometerListener = magnetometerSensorCollector.measurementListener
        requireNotNull(magnetometerListener)

        // process magnetometer measurement
        magnetometerListener.onMeasurement(
            magnetometerSensorCollectorSpy,
            MagnetometerSensorMeasurement(),
            0
        )

        assertNull(syncer.oldestTimestamp)
        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        assertTrue(syncer.running)

        assertEquals(2, magnetometerMeasurements.size)
        val magnetometerMeasurement2 = magnetometerMeasurements.first
        requireNotNull(magnetometerMeasurement2)
        assertEquals(bx1, magnetometerMeasurement2.bx, 0.0f)
        assertEquals(by1, magnetometerMeasurement2.by, 0.0f)
        assertEquals(bz1, magnetometerMeasurement2.bz, 0.0f)
        assertEquals(hardIronX, magnetometerMeasurement2.hardIronX)
        assertEquals(hardIronY, magnetometerMeasurement2.hardIronY)
        assertEquals(hardIronZ, magnetometerMeasurement2.hardIronZ)
        assertEquals(staleTimestamp, magnetometerMeasurement2.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, magnetometerMeasurement2.accuracy)
        val gyroscopeMeasurement3 = magnetometerMeasurements.last
        requireNotNull(gyroscopeMeasurement3)
        assertEquals(bx, gyroscopeMeasurement3.bx, 0.0f)
        assertEquals(by, gyroscopeMeasurement3.by, 0.0f)
        assertEquals(bz, gyroscopeMeasurement3.bz, 0.0f)
        assertEquals(hardIronX, gyroscopeMeasurement3.hardIronX)
        assertEquals(hardIronY, gyroscopeMeasurement3.hardIronY)
        assertEquals(hardIronZ, gyroscopeMeasurement3.hardIronZ)
        assertEquals(magnetometerTimestamp, gyroscopeMeasurement3.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, gyroscopeMeasurement3.accuracy)

        val alreadyProcessedMagnetometerMeasurements: ArrayDeque<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedMagnetometerMeasurements")
        requireNotNull(alreadyProcessedMagnetometerMeasurements)
        assertTrue(alreadyProcessedMagnetometerMeasurements.isEmpty())

        // process gravity measurement
        gravityListener.onMeasurement(gravitySensorCollectorSpy, GravitySensorMeasurement(), 0)

        assertNull(syncer.oldestTimestamp)
        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        assertTrue(syncer.running)

        assertEquals(2, gravityMeasurements.size)
        val gravityMeasurement2 = gravityMeasurements.first
        requireNotNull(gravityMeasurement2)
        assertEquals(gx1, gravityMeasurement2.gx, 0.0f)
        assertEquals(gy1, gravityMeasurement2.gy, 0.0f)
        assertEquals(gz1, gravityMeasurement2.gz, 0.0f)
        assertEquals(staleTimestamp, gravityMeasurement2.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, gravityMeasurement2.accuracy)
        val gravityMeasurement3 = gravityMeasurements.last
        requireNotNull(gravityMeasurement3)
        assertEquals(gx, gravityMeasurement3.gx, 0.0f)
        assertEquals(gy, gravityMeasurement3.gy, 0.0f)
        assertEquals(gz, gravityMeasurement3.gz, 0.0f)
        assertEquals(gravityTimestamp, gravityMeasurement3.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, gravityMeasurement3.accuracy)

        val alreadyProcessedGravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGravityMeasurements")
        requireNotNull(alreadyProcessedGravityMeasurements)
        assertTrue(alreadyProcessedGravityMeasurements.isEmpty())

        // process accelerometer measurement
        accelerometerListener.onMeasurement(
            accelerometerSensorCollectorSpy,
            AccelerometerSensorMeasurement(),
            0
        )

        assertEquals(staleTimestamp, syncer.oldestTimestamp)
        assertEquals(accelerometerTimestamp, syncer.mostRecentTimestamp)
        assertEquals(2, syncer.numberOfProcessedMeasurements)
        assertTrue(syncer.running)
        verify(exactly = 1) { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(0) }

        assertTrue(accelerometerMeasurements.isEmpty())
        assertTrue(gravityMeasurements.isEmpty())
        assertTrue(magnetometerMeasurements.isEmpty())

        val alreadyProcessedAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())
        assertTrue(alreadyProcessedMagnetometerMeasurements.isEmpty())
        assertTrue(alreadyProcessedGravityMeasurements.isEmpty())

        verify(exactly = 1) {
            staleDetectedMeasurementsListener.onStaleMeasurements(
                syncer,
                SensorMeasurementSyncer.SensorType.ACCELEROMETER_UNCALIBRATED,
                any()
            )
        }
    }

    @Test
    fun cleanupStaleMeasurements_whenStaleDetectionDisabled_doesNotNotify() {
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerGravityAndMagnetometerSyncedSensorMeasurement, AccelerometerGravityAndMagnetometerSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val staleDetectedMeasurementsListener =
            mockk<SensorMeasurementSyncer.OnStaleDetectedMeasurementsListener<AccelerometerGravityAndMagnetometerSyncedSensorMeasurement, AccelerometerGravityAndMagnetometerSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(
            context,
            staleDetectionEnabled = false,
            syncedMeasurementListener = syncedMeasurementListener,
            staleDetectedMeasurementsListener = staleDetectedMeasurementsListener
        )

        assertFalse(syncer.staleDetectionEnabled)
        assertNull(syncer.mostRecentTimestamp)
        assertNull(syncer.oldestTimestamp)

        // set as running
        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "running", true)
        assertTrue(syncer.running)

        // set most recent timestamp
        val mostRecentTimestamp = System.nanoTime()
        val accelerometerTimestamp = mostRecentTimestamp - 1
        setPrivateProperty(
            SensorMeasurementSyncer::class,
            syncer,
            "mostRecentTimestamp",
            mostRecentTimestamp
        )

        // add stale measurements
        val staleTimestamp =
            mostRecentTimestamp - 2 * AccelerometerAndGyroscopeSensorMeasurementSyncer.DEFAULT_STALE_OFFSET_NANOS
        val magnetometerMeasurements: ArrayDeque<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("magnetometerMeasurements")
        requireNotNull(magnetometerMeasurements)
        val randomizer = UniformRandomizer()
        val bx1 = randomizer.nextFloat()
        val by1 = randomizer.nextFloat()
        val bz1 = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val staleMagnetometerMeasurement = MagnetometerSensorMeasurement(
            bx1,
            by1,
            bz1,
            hardIronX,
            hardIronY,
            hardIronZ,
            staleTimestamp,
            SensorAccuracy.MEDIUM
        )
        magnetometerMeasurements.add(staleMagnetometerMeasurement)

        val gravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("gravityMeasurements")
        requireNotNull(gravityMeasurements)
        val gx1 = randomizer.nextFloat()
        val gy1 = randomizer.nextFloat()
        val gz1 = randomizer.nextFloat()
        val staleGravityMeasurement =
            GravitySensorMeasurement(gx1, gy1, gz1, staleTimestamp, SensorAccuracy.MEDIUM)
        gravityMeasurements.add(staleGravityMeasurement)

        val accelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("accelerometerMeasurements")
        requireNotNull(accelerometerMeasurements)
        val ax1 = randomizer.nextFloat()
        val ay1 = randomizer.nextFloat()
        val az1 = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val staleAccelerometerMeasurement = AccelerometerSensorMeasurement(
            ax1,
            ay1,
            az1,
            abx,
            aby,
            abz,
            staleTimestamp,
            SensorAccuracy.HIGH
        )
        accelerometerMeasurements.add(staleAccelerometerMeasurement)

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)

        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val magnetometerTimestamp = accelerometerTimestamp - 1
        val magnetometerMeasurement1 = MagnetometerSensorMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            magnetometerTimestamp,
            SensorAccuracy.MEDIUM
        )
        val magnetometerMeasurementsBeforeTimestamp = ArrayDeque<MagnetometerSensorMeasurement>()
        magnetometerMeasurementsBeforeTimestamp.add(magnetometerMeasurement1)
        every { magnetometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            magnetometerMeasurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)

        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val gravityTimestamp = accelerometerTimestamp - 1
        val gravityMeasurement1 =
            GravitySensorMeasurement(gx, gy, gz, gravityTimestamp, SensorAccuracy.MEDIUM)
        val gravityMeasurementsBeforeTimestamp = ArrayDeque<GravitySensorMeasurement>()
        gravityMeasurementsBeforeTimestamp.add(gravityMeasurement1)
        every { gravitySensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            gravityMeasurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)

        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val accelerometerMeasurement1 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            abx,
            aby,
            abz,
            accelerometerTimestamp,
            SensorAccuracy.HIGH
        )
        val measurementsBeforePosition = ArrayDeque<AccelerometerSensorMeasurement>()
        measurementsBeforePosition.add(accelerometerMeasurement1)
        every { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(any()) }.returns(
            measurementsBeforePosition
        )
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        // set previous measurements
        syncer.setPrivateProperty("hasPreviousAccelerometerMeasurement", true)
        val previousAccelerometerMeasurement: AccelerometerSensorMeasurement? =
            syncer.getPrivateProperty("previousAccelerometerMeasurement")
        requireNotNull(previousAccelerometerMeasurement)
        previousAccelerometerMeasurement.timestamp = accelerometerTimestamp - 1

        val accelerometerListener = accelerometerSensorCollectorSpy.measurementListener
        requireNotNull(accelerometerListener)

        val gravityListener = gravitySensorCollectorSpy.measurementListener
        requireNotNull(gravityListener)

        val magnetometerListener = magnetometerSensorCollector.measurementListener
        requireNotNull(magnetometerListener)

        // process magnetometer measurement
        magnetometerListener.onMeasurement(
            magnetometerSensorCollectorSpy,
            MagnetometerSensorMeasurement(),
            0
        )

        assertNull(syncer.oldestTimestamp)
        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        assertTrue(syncer.running)

        assertEquals(2, magnetometerMeasurements.size)
        val magnetometerMeasurement2 = magnetometerMeasurements.first
        requireNotNull(magnetometerMeasurement2)
        assertEquals(bx1, magnetometerMeasurement2.bx, 0.0f)
        assertEquals(by1, magnetometerMeasurement2.by, 0.0f)
        assertEquals(bz1, magnetometerMeasurement2.bz, 0.0f)
        assertEquals(hardIronX, magnetometerMeasurement2.hardIronX)
        assertEquals(hardIronY, magnetometerMeasurement2.hardIronY)
        assertEquals(hardIronZ, magnetometerMeasurement2.hardIronZ)
        assertEquals(staleTimestamp, magnetometerMeasurement2.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, magnetometerMeasurement2.accuracy)
        val magnetometerMeasurement3 = magnetometerMeasurements.last
        requireNotNull(magnetometerMeasurement3)
        assertEquals(bx, magnetometerMeasurement3.bx, 0.0f)
        assertEquals(by, magnetometerMeasurement3.by, 0.0f)
        assertEquals(bz, magnetometerMeasurement3.bz, 0.0f)
        assertEquals(hardIronX, magnetometerMeasurement3.hardIronX)
        assertEquals(hardIronY, magnetometerMeasurement3.hardIronY)
        assertEquals(hardIronZ, magnetometerMeasurement3.hardIronZ)
        assertEquals(magnetometerTimestamp, magnetometerMeasurement3.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, magnetometerMeasurement3.accuracy)

        val alreadyProcessedMagnetometerMeasurements: ArrayDeque<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedMagnetometerMeasurements")
        requireNotNull(alreadyProcessedMagnetometerMeasurements)
        assertTrue(alreadyProcessedMagnetometerMeasurements.isEmpty())

        // process gravity measurement
        gravityListener.onMeasurement(gravitySensorCollectorSpy, GravitySensorMeasurement(), 0)

        assertNull(syncer.oldestTimestamp)
        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        assertTrue(syncer.running)

        assertEquals(2, gravityMeasurements.size)
        val gravityMeasurement2 = gravityMeasurements.first
        requireNotNull(gravityMeasurement2)
        assertEquals(gx1, gravityMeasurement2.gx, 0.0f)
        assertEquals(gy1, gravityMeasurement2.gy, 0.0f)
        assertEquals(gz1, gravityMeasurement2.gz, 0.0f)
        assertEquals(staleTimestamp, gravityMeasurement2.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, gravityMeasurement2.accuracy)
        val gravityMeasurement3 = gravityMeasurements.last
        requireNotNull(gravityMeasurement3)
        assertEquals(gx, gravityMeasurement3.gx, 0.0f)
        assertEquals(gy, gravityMeasurement3.gy, 0.0f)
        assertEquals(gz, gravityMeasurement3.gz, 0.0f)
        assertEquals(gravityTimestamp, gravityMeasurement3.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, gravityMeasurement3.accuracy)

        val alreadyProcessedGravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGravityMeasurements")
        requireNotNull(alreadyProcessedGravityMeasurements)
        assertTrue(alreadyProcessedGravityMeasurements.isEmpty())

        // process accelerometer measurement
        accelerometerListener.onMeasurement(
            accelerometerSensorCollectorSpy,
            AccelerometerSensorMeasurement(),
            0
        )

        assertEquals(staleTimestamp, syncer.oldestTimestamp)
        assertEquals(accelerometerTimestamp, syncer.mostRecentTimestamp)
        assertEquals(2, syncer.numberOfProcessedMeasurements)
        assertTrue(syncer.running)
        verify(exactly = 1) { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(0) }

        assertEquals(1, accelerometerMeasurements.size)
        val accelerometerMeasurement2 = accelerometerMeasurements.first()
        requireNotNull(accelerometerMeasurement2)
        assertSame(staleAccelerometerMeasurement, accelerometerMeasurement2)
        assertTrue(gravityMeasurements.isEmpty())
        assertTrue(magnetometerMeasurements.isEmpty())

        val alreadyProcessedAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertFalse(alreadyProcessedAccelerometerMeasurements.isEmpty())
        assertFalse(alreadyProcessedGravityMeasurements.isEmpty())
        assertTrue(alreadyProcessedMagnetometerMeasurements.isEmpty())

        verify { staleDetectedMeasurementsListener wasNot Called }
    }

    @Test
    fun sensorCollectors_whenPreviousGyroscopeMeasurement_notifies() {
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerGravityAndMagnetometerSyncedSensorMeasurement, AccelerometerGravityAndMagnetometerSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(
            context,
            syncedMeasurementListener = syncedMeasurementListener
        )

        assertNull(syncer.mostRecentTimestamp)
        assertNull(syncer.oldestTimestamp)

        // set as running
        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "running", true)
        assertTrue(syncer.running)

        // set most recent timestamp
        val mostRecentTimestamp = System.nanoTime()
        val accelerometerTimestamp = mostRecentTimestamp - 1
        setPrivateProperty(
            SensorMeasurementSyncer::class,
            syncer,
            "mostRecentTimestamp",
            mostRecentTimestamp
        )

        // set previous accelerometer, gravity and gyroscope timestamp
        syncer.setPrivateProperty("hasPreviousAccelerometerMeasurement", true)
        val previousAccelerometerMeasurement: AccelerometerSensorMeasurement? =
            syncer.getPrivateProperty("previousAccelerometerMeasurement")
        requireNotNull(previousAccelerometerMeasurement)
        previousAccelerometerMeasurement.timestamp = mostRecentTimestamp - 1
        syncer.setPrivateProperty("hasPreviousMagnetometerMeasurement", true)
        syncer.setPrivateProperty("lastNotifiedTimestamp", accelerometerTimestamp - 1)
        syncer.setPrivateProperty("lastNotifiedAccelerometerTimestamp", accelerometerTimestamp - 1)
        syncer.setPrivateProperty("lastNotifiedGyroscopeTimestamp", accelerometerTimestamp - 2)
        syncer.setPrivateProperty("hasPreviousGravityMeasurement", true)
        val previousGravityMeasurement: GravitySensorMeasurement? =
            syncer.getPrivateProperty("previousGravityMeasurement")
        requireNotNull(previousGravityMeasurement)
        val randomizer = UniformRandomizer()
        val gx1 = randomizer.nextFloat()
        val gy1 = randomizer.nextFloat()
        val gz1 = randomizer.nextFloat()
        previousGravityMeasurement.gx = gx1
        previousGravityMeasurement.gy = gy1
        previousGravityMeasurement.gz = gz1
        previousGravityMeasurement.timestamp = accelerometerTimestamp - 1
        previousGravityMeasurement.accuracy = SensorAccuracy.HIGH

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)

        val gx2 = randomizer.nextFloat()
        val gy2 = randomizer.nextFloat()
        val gz2 = randomizer.nextFloat()
        val gravityTimestamp = accelerometerTimestamp - 1
        val gravityMeasurement1 =
            GravitySensorMeasurement(gx2, gy2, gz2, gravityTimestamp, SensorAccuracy.HIGH)
        val gravityMeasurementsBeforeTimestamp = ArrayDeque<GravitySensorMeasurement>()
        gravityMeasurementsBeforeTimestamp.add(gravityMeasurement1)
        every { gravitySensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            gravityMeasurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)

        val previousMagnetometerMeasurement: MagnetometerSensorMeasurement? =
            syncer.getPrivateProperty("previousMagnetometerMeasurement")
        requireNotNull(previousMagnetometerMeasurement)
        val bx1 = randomizer.nextFloat()
        val by1 = randomizer.nextFloat()
        val bz1 = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        previousMagnetometerMeasurement.bx = bx1
        previousMagnetometerMeasurement.by = by1
        previousMagnetometerMeasurement.bz = bz1
        previousMagnetometerMeasurement.hardIronX = hardIronX
        previousMagnetometerMeasurement.hardIronY = hardIronY
        previousMagnetometerMeasurement.hardIronZ = hardIronZ
        previousMagnetometerMeasurement.timestamp = accelerometerTimestamp - 1
        previousMagnetometerMeasurement.accuracy = SensorAccuracy.MEDIUM

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)

        val bx2 = randomizer.nextFloat()
        val by2 = randomizer.nextFloat()
        val bz2 = randomizer.nextFloat()
        val gyroscopeTimestamp = accelerometerTimestamp - 1
        val magnetometerMeasurement1 = MagnetometerSensorMeasurement(
            bx2,
            by2,
            bz2,
            hardIronX,
            hardIronY,
            hardIronZ,
            gyroscopeTimestamp,
            SensorAccuracy.MEDIUM
        )
        val magnetometerMeasurementsBeforeTimestamp = ArrayDeque<MagnetometerSensorMeasurement>()
        magnetometerMeasurementsBeforeTimestamp.add(magnetometerMeasurement1)
        every { magnetometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            magnetometerMeasurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)

        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val accelerometerMeasurement1 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            abx,
            aby,
            abz,
            accelerometerTimestamp,
            SensorAccuracy.HIGH
        )
        val measurementsBeforePosition = ArrayDeque<AccelerometerSensorMeasurement>()
        measurementsBeforePosition.add(accelerometerMeasurement1)
        every { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(any()) }.returns(
            measurementsBeforePosition
        )
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        val accelerometerListener = accelerometerSensorCollectorSpy.measurementListener
        requireNotNull(accelerometerListener)

        val gravityListener = gravitySensorCollectorSpy.measurementListener
        requireNotNull(gravityListener)

        val gyroscopeListener = magnetometerSensorCollector.measurementListener
        requireNotNull(gyroscopeListener)

        // process magnetometer measurement
        gyroscopeListener.onMeasurement(
            magnetometerSensorCollectorSpy,
            MagnetometerSensorMeasurement(),
            0
        )

        assertNull(syncer.oldestTimestamp)
        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        assertTrue(syncer.running)

        val magnetometerMeasurements: ArrayDeque<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("magnetometerMeasurements")
        requireNotNull(magnetometerMeasurements)
        assertEquals(1, magnetometerMeasurements.size)
        val magnetometerMeasurement2 = magnetometerMeasurements.peek()
        requireNotNull(magnetometerMeasurement2)
        assertNotSame(magnetometerMeasurement1, magnetometerMeasurement2)
        assertEquals(bx2, magnetometerMeasurement2.bx, 0.0f)
        assertEquals(by2, magnetometerMeasurement2.by, 0.0f)
        assertEquals(bz2, magnetometerMeasurement2.bz, 0.0f)
        assertEquals(hardIronX, magnetometerMeasurement2.hardIronX)
        assertEquals(hardIronY, magnetometerMeasurement2.hardIronY)
        assertEquals(hardIronZ, magnetometerMeasurement2.hardIronZ)
        assertEquals(gyroscopeTimestamp, magnetometerMeasurement2.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, magnetometerMeasurement2.accuracy)

        val alreadyProcessedMagnetometerMeasurements: ArrayDeque<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedMagnetometerMeasurements")
        requireNotNull(alreadyProcessedMagnetometerMeasurements)
        assertTrue(alreadyProcessedMagnetometerMeasurements.isEmpty())

        // process gravity measurement
        gravityListener.onMeasurement(gravitySensorCollectorSpy, GravitySensorMeasurement(), 0)

        assertNull(syncer.oldestTimestamp)
        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        assertTrue(syncer.running)

        val gravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("gravityMeasurements")
        requireNotNull(gravityMeasurements)
        assertEquals(1, gravityMeasurements.size)
        val gravityMeasurement2 = gravityMeasurements.peek()
        requireNotNull(gravityMeasurement2)
        assertNotSame(gravityMeasurement1, gravityMeasurement2)
        assertEquals(gx2, gravityMeasurement2.gx, 0.0f)
        assertEquals(gy2, gravityMeasurement2.gy, 0.0f)
        assertEquals(gz2, gravityMeasurement2.gz, 0.0f)
        assertEquals(gravityTimestamp, gravityMeasurement2.timestamp)
        assertEquals(SensorAccuracy.HIGH, gravityMeasurement2.accuracy)

        val alreadyProcessedGravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGravityMeasurements")
        requireNotNull(alreadyProcessedGravityMeasurements)
        assertTrue(alreadyProcessedGravityMeasurements.isEmpty())

        // process accelerometer measurement
        accelerometerListener.onMeasurement(
            accelerometerSensorCollectorSpy,
            AccelerometerSensorMeasurement(),
            0
        )

        assertEquals(accelerometerTimestamp, syncer.oldestTimestamp)
        assertEquals(accelerometerTimestamp, syncer.mostRecentTimestamp)
        assertEquals(1, syncer.numberOfProcessedMeasurements)
        assertTrue(syncer.running)
        verify(exactly = 1) { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(0) }

        val accelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("accelerometerMeasurements")
        requireNotNull(accelerometerMeasurements)
        assertTrue(accelerometerMeasurements.isEmpty())
        assertEquals(1, magnetometerMeasurements.size)
        assertSame(magnetometerMeasurement2, magnetometerMeasurements.first())

        val alreadyProcessedAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())
        assertTrue(alreadyProcessedMagnetometerMeasurements.isEmpty())

        val slot = slot<AccelerometerGravityAndMagnetometerSyncedSensorMeasurement>()
        verify(exactly = 1) {
            syncedMeasurementListener.onSyncedMeasurements(
                syncer,
                capture(slot)
            )
        }

        val syncedMeasurement = slot.captured
        assertEquals(accelerometerTimestamp, syncedMeasurement.timestamp)
        val syncedAccelerometerMeasurement = syncedMeasurement.accelerometerMeasurement
        requireNotNull(syncedAccelerometerMeasurement)
        assertEquals(ax, syncedAccelerometerMeasurement.ax, 0.0f)
        assertEquals(ay, syncedAccelerometerMeasurement.ay, 0.0f)
        assertEquals(az, syncedAccelerometerMeasurement.az, 0.0f)
        assertEquals(abx, syncedAccelerometerMeasurement.bx)
        assertEquals(aby, syncedAccelerometerMeasurement.by)
        assertEquals(abz, syncedAccelerometerMeasurement.bz)
        assertEquals(accelerometerTimestamp, syncedAccelerometerMeasurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, syncedAccelerometerMeasurement.accuracy)
        val syncedGravityMeasurement = syncedMeasurement.gravityMeasurement
        requireNotNull(syncedGravityMeasurement)
        assertEquals(gx1, syncedGravityMeasurement.gx, 0.0f)
        assertEquals(gy1, syncedGravityMeasurement.gy, 0.0f)
        assertEquals(gz1, syncedGravityMeasurement.gz, 0.0f)
        assertEquals(gravityTimestamp, syncedGravityMeasurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, syncedGravityMeasurement.accuracy)
        val syncedMagnetometerMeasurement = syncedMeasurement.magnetometerMeasurement
        requireNotNull(syncedMagnetometerMeasurement)
        assertEquals(bx1, syncedMagnetometerMeasurement.bx, 0.0f)
        assertEquals(by1, syncedMagnetometerMeasurement.by, 0.0f)
        assertEquals(bz1, syncedMagnetometerMeasurement.bz, 0.0f)
        assertEquals(hardIronX, syncedMagnetometerMeasurement.hardIronX)
        assertEquals(hardIronY, syncedMagnetometerMeasurement.hardIronY)
        assertEquals(hardIronZ, syncedMagnetometerMeasurement.hardIronZ)
        assertEquals(gyroscopeTimestamp, syncedMagnetometerMeasurement.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, syncedMagnetometerMeasurement.accuracy)
    }

    @Test
    fun sensorCollectors_whenNoPreviousGyroscopeMeasurement_notifies() {
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerGravityAndMagnetometerSyncedSensorMeasurement, AccelerometerGravityAndMagnetometerSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityAndMagnetometerSensorMeasurementSyncer(
            context,
            syncedMeasurementListener = syncedMeasurementListener
        )

        assertNull(syncer.mostRecentTimestamp)
        assertNull(syncer.oldestTimestamp)

        // set as running
        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "running", true)
        assertTrue(syncer.running)

        // set most recent timestamp
        val mostRecentTimestamp = System.nanoTime()
        val accelerometerTimestamp = mostRecentTimestamp - 1
        setPrivateProperty(
            SensorMeasurementSyncer::class,
            syncer,
            "mostRecentTimestamp",
            mostRecentTimestamp
        )

        // set previous accelerometer, gravity and magnetometer timestamp
        syncer.setPrivateProperty("hasPreviousAccelerometerMeasurement", true)
        val previousAccelerometerMeasurement: AccelerometerSensorMeasurement? =
            syncer.getPrivateProperty("previousAccelerometerMeasurement")
        requireNotNull(previousAccelerometerMeasurement)
        previousAccelerometerMeasurement.timestamp = mostRecentTimestamp - 1
        syncer.setPrivateProperty("hasPreviousMagnetometerMeasurement", true)
        syncer.setPrivateProperty("lastNotifiedTimestamp", accelerometerTimestamp - 1)
        syncer.setPrivateProperty("lastNotifiedAccelerometerTimestamp", accelerometerTimestamp - 1)
        syncer.setPrivateProperty("lastNotifiedMagnetometerTimestamp", accelerometerTimestamp - 2)
        syncer.setPrivateProperty("hasPreviousGravityMeasurement", true)
        val previousGravityMeasurement: GravitySensorMeasurement? =
            syncer.getPrivateProperty("previousGravityMeasurement")
        requireNotNull(previousGravityMeasurement)
        val randomizer = UniformRandomizer()
        val gx1 = randomizer.nextFloat()
        val gy1 = randomizer.nextFloat()
        val gz1 = randomizer.nextFloat()
        previousGravityMeasurement.gx = gx1
        previousGravityMeasurement.gy = gy1
        previousGravityMeasurement.gz = gz1
        previousGravityMeasurement.timestamp = accelerometerTimestamp - 1
        previousGravityMeasurement.accuracy = SensorAccuracy.HIGH

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)

        val gx2 = randomizer.nextFloat()
        val gy2 = randomizer.nextFloat()
        val gz2 = randomizer.nextFloat()
        val gravityMeasurement1 =
            GravitySensorMeasurement(gx2, gy2, gz2, accelerometerTimestamp, SensorAccuracy.HIGH)
        val gravityMeasurementsBeforeTimestamp = ArrayDeque<GravitySensorMeasurement>()
        gravityMeasurementsBeforeTimestamp.add(gravityMeasurement1)
        every { gravitySensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            gravityMeasurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)

        val previousMagnetometerMeasurement: MagnetometerSensorMeasurement? =
            syncer.getPrivateProperty("previousMagnetometerMeasurement")
        requireNotNull(previousMagnetometerMeasurement)
        val bx1 = randomizer.nextFloat()
        val by1 = randomizer.nextFloat()
        val bz1 = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        previousMagnetometerMeasurement.bx = bx1
        previousMagnetometerMeasurement.by = by1
        previousMagnetometerMeasurement.bz = bz1
        previousMagnetometerMeasurement.hardIronX = hardIronX
        previousMagnetometerMeasurement.hardIronY = hardIronY
        previousMagnetometerMeasurement.hardIronZ = hardIronZ
        previousMagnetometerMeasurement.timestamp = accelerometerTimestamp - 1
        previousMagnetometerMeasurement.accuracy = SensorAccuracy.MEDIUM

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)

        val bx2 = randomizer.nextFloat()
        val by2 = randomizer.nextFloat()
        val bz2 = randomizer.nextFloat()
        val magnetometerTimestamp = accelerometerTimestamp - 3
        val magnetometerMeasurement1 = MagnetometerSensorMeasurement(
            bx2,
            by2,
            bz2,
            hardIronX,
            hardIronY,
            hardIronZ,
            magnetometerTimestamp,
            SensorAccuracy.MEDIUM
        )
        val magnetometerMeasurementsBeforeTimestamp = ArrayDeque<MagnetometerSensorMeasurement>()
        magnetometerMeasurementsBeforeTimestamp.add(magnetometerMeasurement1)
        every { magnetometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            magnetometerMeasurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)

        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val accelerometerMeasurement1 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            abx,
            aby,
            abz,
            accelerometerTimestamp,
            SensorAccuracy.HIGH
        )
        val measurementsBeforePosition = ArrayDeque<AccelerometerSensorMeasurement>()
        measurementsBeforePosition.add(accelerometerMeasurement1)
        every { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(any()) }.returns(
            measurementsBeforePosition
        )
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        val accelerometerListener = accelerometerSensorCollectorSpy.measurementListener
        requireNotNull(accelerometerListener)

        val gravityListener = gravitySensorCollectorSpy.measurementListener
        requireNotNull(gravityListener)

        val magnetometerListener = magnetometerSensorCollector.measurementListener
        requireNotNull(magnetometerListener)

        // process magnetometer measurement
        magnetometerListener.onMeasurement(
            magnetometerSensorCollectorSpy,
            MagnetometerSensorMeasurement(),
            0
        )

        assertNull(syncer.oldestTimestamp)
        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        assertTrue(syncer.running)

        val magnetometerMeasurements: ArrayDeque<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("magnetometerMeasurements")
        requireNotNull(magnetometerMeasurements)
        assertEquals(1, magnetometerMeasurements.size)
        val magnetometerMeasurement2 = magnetometerMeasurements.peek()
        requireNotNull(magnetometerMeasurement2)
        assertNotSame(magnetometerMeasurement1, magnetometerMeasurement2)
        assertEquals(bx2, magnetometerMeasurement2.bx, 0.0f)
        assertEquals(by2, magnetometerMeasurement2.by, 0.0f)
        assertEquals(bz2, magnetometerMeasurement2.bz, 0.0f)
        assertEquals(hardIronX, magnetometerMeasurement2.hardIronX)
        assertEquals(hardIronY, magnetometerMeasurement2.hardIronY)
        assertEquals(hardIronZ, magnetometerMeasurement2.hardIronZ)
        assertEquals(magnetometerTimestamp, magnetometerMeasurement2.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, magnetometerMeasurement2.accuracy)

        val alreadyProcessedMagnetometerMeasurements: ArrayDeque<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedMagnetometerMeasurements")
        requireNotNull(alreadyProcessedMagnetometerMeasurements)
        assertTrue(alreadyProcessedMagnetometerMeasurements.isEmpty())

        // process gravity measurement
        gravityListener.onMeasurement(gravitySensorCollectorSpy, GravitySensorMeasurement(), 0)

        assertNull(syncer.oldestTimestamp)
        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        assertTrue(syncer.running)

        val gravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("gravityMeasurements")
        requireNotNull(gravityMeasurements)
        assertEquals(1, gravityMeasurements.size)
        val gravityMeasurement2 = gravityMeasurements.peek()
        requireNotNull(gravityMeasurement2)
        assertNotSame(gravityMeasurement1, gravityMeasurement2)
        assertEquals(gx2, gravityMeasurement2.gx, 0.0f)
        assertEquals(gy2, gravityMeasurement2.gy, 0.0f)
        assertEquals(gz2, gravityMeasurement2.gz, 0.0f)
        assertEquals(accelerometerTimestamp, gravityMeasurement2.timestamp)
        assertEquals(SensorAccuracy.HIGH, gravityMeasurement2.accuracy)

        val alreadyProcessedGravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGravityMeasurements")
        requireNotNull(alreadyProcessedGravityMeasurements)
        assertTrue(alreadyProcessedGravityMeasurements.isEmpty())

        // process accelerometer measurement
        accelerometerListener.onMeasurement(
            accelerometerSensorCollectorSpy,
            AccelerometerSensorMeasurement(),
            0
        )

        assertEquals(accelerometerTimestamp, syncer.oldestTimestamp)
        assertEquals(accelerometerTimestamp, syncer.mostRecentTimestamp)
        assertEquals(1, syncer.numberOfProcessedMeasurements)
        assertTrue(syncer.running)
        verify(exactly = 1) { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(0) }

        val accelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("accelerometerMeasurements")
        requireNotNull(accelerometerMeasurements)
        assertTrue(accelerometerMeasurements.isEmpty())
        assertEquals(1, magnetometerMeasurements.size)
        assertSame(magnetometerMeasurement2, magnetometerMeasurements.first())

        val alreadyProcessedAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())
        assertTrue(alreadyProcessedMagnetometerMeasurements.isEmpty())

        val slot = slot<AccelerometerGravityAndMagnetometerSyncedSensorMeasurement>()
        verify(exactly = 1) {
            syncedMeasurementListener.onSyncedMeasurements(
                syncer,
                capture(slot)
            )
        }

        val syncedMeasurement = slot.captured
        assertEquals(accelerometerTimestamp, syncedMeasurement.timestamp)
        val syncedAccelerometerMeasurement = syncedMeasurement.accelerometerMeasurement
        requireNotNull(syncedAccelerometerMeasurement)
        assertEquals(ax, syncedAccelerometerMeasurement.ax, 0.0f)
        assertEquals(ay, syncedAccelerometerMeasurement.ay, 0.0f)
        assertEquals(az, syncedAccelerometerMeasurement.az, 0.0f)
        assertEquals(abx, syncedAccelerometerMeasurement.bx)
        assertEquals(aby, syncedAccelerometerMeasurement.by)
        assertEquals(abz, syncedAccelerometerMeasurement.bz)
        assertEquals(accelerometerTimestamp, syncedAccelerometerMeasurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, syncedAccelerometerMeasurement.accuracy)
        val syncedGravityMeasurement = syncedMeasurement.gravityMeasurement
        requireNotNull(syncedGravityMeasurement)
        assertEquals(gx2, syncedGravityMeasurement.gx, 0.0f)
        assertEquals(gy2, syncedGravityMeasurement.gy, 0.0f)
        assertEquals(gz2, syncedGravityMeasurement.gz, 0.0f)
        assertEquals(accelerometerTimestamp, syncedGravityMeasurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, syncedGravityMeasurement.accuracy)
        val syncedMagnetometerMeasurement = syncedMeasurement.magnetometerMeasurement
        requireNotNull(syncedMagnetometerMeasurement)
        assertEquals(bx1, syncedMagnetometerMeasurement.bx, 0.0f)
        assertEquals(by1, syncedMagnetometerMeasurement.by, 0.0f)
        assertEquals(bz1, syncedMagnetometerMeasurement.bz, 0.0f)
        assertEquals(hardIronX, syncedMagnetometerMeasurement.hardIronX)
        assertEquals(hardIronY, syncedMagnetometerMeasurement.hardIronY)
        assertEquals(hardIronZ, syncedMagnetometerMeasurement.hardIronZ)
        assertEquals(accelerometerTimestamp - 1, syncedMagnetometerMeasurement.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, syncedMagnetometerMeasurement.accuracy)
    }
}