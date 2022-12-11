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
import android.os.Build
import androidx.test.core.app.ApplicationProvider
import com.irurueta.android.navigation.inertial.callPrivateFunc
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
class AccelerometerAndMagnetometerSensorMeasurementSyncerTest {

    @Test
    fun constructor_whenRequiredParameters_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(context)

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
        assertEquals(SensorDelay.FASTEST, syncer.magnetometerSensorDelay)
        assertEquals(BufferedSensorCollector.DEFAULT_CAPACITY, syncer.accelerometerCapacity)
        assertEquals(BufferedSensorCollector.DEFAULT_CAPACITY, syncer.magnetometerCapacity)
        assertTrue(syncer.accelerometerStartOffsetEnabled)
        assertTrue(syncer.magnetometerStartOffsetEnabled)
        assertTrue(syncer.stopWhenFilledBuffer)
        assertFalse(syncer.outOfOrderDetectionEnabled)
        assertTrue(syncer.stopWhenOutOfOrder)
        assertNull(syncer.accuracyChangedListener)
        assertNull(syncer.bufferFilledListener)
        assertNull(syncer.outOfOrderMeasurementListener)
        assertNull(syncer.syncedMeasurementListener)
        assertEquals(0L, syncer.startTimestamp)
        assertFalse(syncer.running)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        assertNull(syncer.mostRecentTimestamp)
        assertNull(syncer.oldestTimestamp)
        assertNull(syncer.accelerometerSensor)
        assertNull(syncer.magnetometerSensor)
        assertFalse(syncer.accelerometerSensorAvailable)
        assertFalse(syncer.magnetometerSensorAvailable)
        assertNull(syncer.accelerometerStartOffset)
        assertNull(syncer.magnetometerStartOffset)
        assertEquals(0.0f, syncer.accelerometerUsage, 0.0f)
        assertEquals(0.0f, syncer.magnetometerUsage, 0.0f)
    }

    @Test
    fun constructor_whenAllParameters_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val accuracyChangedListener =
            mockk<SensorMeasurementSyncer.OnAccuracyChangedListener<AccelerometerAndMagnetometerSyncedSensorMeasurement, AccelerometerAndMagnetometerSensorMeasurementSyncer>>()
        val bufferFilledListener =
            mockk<SensorMeasurementSyncer.OnBufferFilledListener<AccelerometerAndMagnetometerSyncedSensorMeasurement, AccelerometerAndMagnetometerSensorMeasurementSyncer>>()
        val outOfOrderMeasurementListener =
            mockk<SensorMeasurementSyncer.OnOutOfOrderMeasurementListener<AccelerometerAndMagnetometerSyncedSensorMeasurement, AccelerometerAndMagnetometerSensorMeasurementSyncer>>()
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerAndMagnetometerSyncedSensorMeasurement, AccelerometerAndMagnetometerSensorMeasurementSyncer>>()
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            MagnetometerSensorType.MAGNETOMETER,
            SensorDelay.NORMAL,
            SensorDelay.GAME,
            accelerometerCapacity = 2,
            magnetometerCapacity = 3,
            accelerometerStartOffsetEnabled = false,
            magnetometerStartOffsetEnabled = false,
            stopWhenFilledBuffer = false,
            outOfOrderDetectionEnabled = true,
            stopWhenOutOfOrder = false,
            accuracyChangedListener = accuracyChangedListener,
            bufferFilledListener = bufferFilledListener,
            outOfOrderMeasurementListener = outOfOrderMeasurementListener,
            syncedMeasurementListener = syncedMeasurementListener
        )

        // check
        assertSame(context, syncer.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            syncer.accelerometerSensorType
        )
        assertEquals(MagnetometerSensorType.MAGNETOMETER, syncer.magnetometerSensorType)
        assertEquals(SensorDelay.NORMAL, syncer.accelerometerSensorDelay)
        assertEquals(SensorDelay.GAME, syncer.magnetometerSensorDelay)
        assertEquals(2, syncer.accelerometerCapacity)
        assertEquals(3, syncer.magnetometerCapacity)
        assertFalse(syncer.accelerometerStartOffsetEnabled)
        assertFalse(syncer.magnetometerStartOffsetEnabled)
        assertFalse(syncer.stopWhenFilledBuffer)
        assertTrue(syncer.outOfOrderDetectionEnabled)
        assertFalse(syncer.stopWhenOutOfOrder)
        assertSame(accuracyChangedListener, syncer.accuracyChangedListener)
        assertSame(bufferFilledListener, syncer.bufferFilledListener)
        assertSame(outOfOrderMeasurementListener, syncer.outOfOrderMeasurementListener)
        assertSame(syncedMeasurementListener, syncer.syncedMeasurementListener)
        assertEquals(0L, syncer.startTimestamp)
        assertFalse(syncer.running)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        assertNull(syncer.mostRecentTimestamp)
        assertNull(syncer.oldestTimestamp)
        assertNull(syncer.accelerometerSensor)
        assertNull(syncer.magnetometerSensor)
        assertFalse(syncer.accelerometerSensorAvailable)
        assertFalse(syncer.magnetometerSensorAvailable)
        assertNull(syncer.accelerometerStartOffset)
        assertNull(syncer.magnetometerStartOffset)
        assertEquals(0.0f, syncer.accelerometerUsage, 0.0f)
        assertEquals(0.0f, syncer.magnetometerUsage, 0.0f)
    }

    @Test
    fun accuracyChangedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(context)

        // check default value
        assertNull(syncer.accuracyChangedListener)

        // set new value
        val accuracyChangedListener =
            mockk<SensorMeasurementSyncer.OnAccuracyChangedListener<AccelerometerAndMagnetometerSyncedSensorMeasurement, AccelerometerAndMagnetometerSensorMeasurementSyncer>>()
        syncer.accuracyChangedListener = accuracyChangedListener

        // check
        assertSame(accuracyChangedListener, syncer.accuracyChangedListener)
    }

    @Test
    fun bufferFilledListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(context)

        // check default value
        assertNull(syncer.bufferFilledListener)

        // set new value
        val bufferFilledListener =
            mockk<SensorMeasurementSyncer.OnBufferFilledListener<AccelerometerAndMagnetometerSyncedSensorMeasurement, AccelerometerAndMagnetometerSensorMeasurementSyncer>>()
        syncer.bufferFilledListener = bufferFilledListener

        // check
        assertSame(bufferFilledListener, syncer.bufferFilledListener)
    }

    @Test
    fun outOfOrderMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(context)

        // check default value
        assertNull(syncer.outOfOrderMeasurementListener)

        // set new value
        val outOfOrderMeasurementListener =
            mockk<SensorMeasurementSyncer.OnOutOfOrderMeasurementListener<AccelerometerAndMagnetometerSyncedSensorMeasurement, AccelerometerAndMagnetometerSensorMeasurementSyncer>>()
        syncer.outOfOrderMeasurementListener = outOfOrderMeasurementListener

        // check
        assertSame(outOfOrderMeasurementListener, syncer.outOfOrderMeasurementListener)
    }

    @Test
    fun syncedMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(context)

        // check default value
        assertNull(syncer.syncedMeasurementListener)

        // set new value
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerAndMagnetometerSyncedSensorMeasurement, AccelerometerAndMagnetometerSensorMeasurementSyncer>>()
        syncer.syncedMeasurementListener = syncedMeasurementListener

        // check
        assertSame(syncedMeasurementListener, syncer.syncedMeasurementListener)
    }

    @Test(expected = IllegalStateException::class)
    fun start_whenAlreadyRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(context)

        // set as running
        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "running", true)
        assertTrue(syncer.running)

        syncer.start()
    }

    @Test
    fun start_whenNotRunningAndNoTimestamp_setsCurrentStartTimestamp() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(context)

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
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(context)

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
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(context)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.start(any()) }.returns(false)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)
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
        verify(exactly = 0) { magnetometerSensorCollectorSpy.start(any()) }
    }

    @Test
    fun start_whenMagnetometerCollectorFails_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(context)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.start(any()) }.returns(true)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)
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
        verify(exactly = 1) { magnetometerSensorCollectorSpy.start(timestamp) }
    }

    @Test
    fun start_whenCollectorsSucceed_returnsTrue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(context)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.start(any()) }.returns(true)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)
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
        verify(exactly = 1) { magnetometerSensorCollectorSpy.start(timestamp) }
    }

    @Test
    fun stop_stopsCollectorsInitializesCachesAndResetsProperties() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(context)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)
        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        val availableAccelerometerMeasurements: LinkedList<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("availableAccelerometerMeasurements")
        requireNotNull(availableAccelerometerMeasurements)
        availableAccelerometerMeasurements.clear()

        val availableMagnetometerMeasurements: LinkedList<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("availableMagnetometerMeasurements")
        requireNotNull(availableMagnetometerMeasurements)
        availableMagnetometerMeasurements.clear()

        val accelerometerMeasurements: LinkedList<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("accelerometerMeasurements")
        requireNotNull(accelerometerMeasurements)
        accelerometerMeasurements.add(AccelerometerSensorMeasurement())

        val magnetometerMeasurements: LinkedList<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("magnetometerMeasurements")
        requireNotNull(magnetometerMeasurements)
        magnetometerMeasurements.add(MagnetometerSensorMeasurement())

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

        syncer.stop()

        // check
        verify(exactly = 1) { accelerometerSensorCollectorSpy.stop() }
        verify(exactly = 1) { magnetometerSensorCollectorSpy.stop() }
        assertEquals(syncer.accelerometerCapacity, availableAccelerometerMeasurements.size)
        assertEquals(syncer.magnetometerCapacity, availableMagnetometerMeasurements.size)
        assertTrue(accelerometerMeasurements.isEmpty())
        assertTrue(magnetometerMeasurements.isEmpty())

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
    }

    @Test
    fun accelerometerSensorCollector_whenAccuracyChangedListenerAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(context)

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
            mockk<SensorMeasurementSyncer.OnAccuracyChangedListener<AccelerometerAndMagnetometerSyncedSensorMeasurement, AccelerometerAndMagnetometerSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(
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
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(
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
            mockk<SensorMeasurementSyncer.OnBufferFilledListener<AccelerometerAndMagnetometerSyncedSensorMeasurement, AccelerometerAndMagnetometerSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(
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
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerAndMagnetometerSyncedSensorMeasurement, AccelerometerAndMagnetometerSensorMeasurementSyncer>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(
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
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerAndMagnetometerSyncedSensorMeasurement, AccelerometerAndMagnetometerSensorMeasurementSyncer>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(
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

        val availableAccelerometerMeasurements: LinkedList<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("availableAccelerometerMeasurements")
        requireNotNull(availableAccelerometerMeasurements)
        assertEquals(syncer.accelerometerCapacity - 1, availableAccelerometerMeasurements.size)
        val accelerometerMeasurements: LinkedList<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("accelerometerMeasurements")
        requireNotNull(accelerometerMeasurements)
        assertEquals(1, accelerometerMeasurements.size)
        assertNotSame(accelerometerMeasurement, accelerometerMeasurements[0])
        assertEquals(ax, accelerometerMeasurements[0].ax, 0.0f)
        assertEquals(ay, accelerometerMeasurements[0].ay, 0.0f)
        assertEquals(az, accelerometerMeasurements[0].az, 0.0f)
        assertEquals(bx, accelerometerMeasurements[0].bx)
        assertEquals(by, accelerometerMeasurements[0].by)
        assertEquals(bz, accelerometerMeasurements[0].bz)
        assertEquals(timestamp, accelerometerMeasurements[0].timestamp)
        assertEquals(SensorAccuracy.HIGH, accelerometerMeasurements[0].accuracy)

        assertEquals(timestamp, syncer.oldestTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)

        val alreadyProcessedAccelerometerMeasurements: LinkedList<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())

        val alreadyProcessedMagnetometerMeasurements: LinkedList<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedMagnetometerMeasurements")
        requireNotNull(alreadyProcessedMagnetometerMeasurements)
        assertTrue(alreadyProcessedMagnetometerMeasurements.isEmpty())

        verify { syncedMeasurementListener wasNot Called }
        assertTrue(syncer.running)
    }

    @Test
    fun accelerometerSensorCollector_whenMeasurementMeasurementsAvailableAndEmptyCache_updatesMostRecentTimestampCopiesMeasurementsAndProcessesThem() {
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerAndMagnetometerSyncedSensorMeasurement, AccelerometerAndMagnetometerSensorMeasurementSyncer>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(
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

        val availableAccelerometerMeasurements: LinkedList<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("availableAccelerometerMeasurements")
        requireNotNull(availableAccelerometerMeasurements)
        availableAccelerometerMeasurements.clear()

        val listener = accelerometerSensorCollectorSpy.measurementListener
        requireNotNull(listener)

        listener.onMeasurement(accelerometerSensorCollectorSpy, AccelerometerSensorMeasurement(), 0)

        verify(exactly = 1) { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(0) }
        assertEquals(timestamp, syncer.mostRecentTimestamp)

        val accelerometerMeasurements: LinkedList<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("accelerometerMeasurements")
        requireNotNull(accelerometerMeasurements)
        assertEquals(1, accelerometerMeasurements.size)
        assertNotSame(accelerometerMeasurement, accelerometerMeasurements[0])
        assertEquals(ax, accelerometerMeasurements[0].ax, 0.0f)
        assertEquals(ay, accelerometerMeasurements[0].ay, 0.0f)
        assertEquals(az, accelerometerMeasurements[0].az, 0.0f)
        assertEquals(bx, accelerometerMeasurements[0].bx)
        assertEquals(by, accelerometerMeasurements[0].by)
        assertEquals(bz, accelerometerMeasurements[0].bz)
        assertEquals(timestamp, accelerometerMeasurements[0].timestamp)
        assertEquals(SensorAccuracy.HIGH, accelerometerMeasurements[0].accuracy)

        assertEquals(timestamp, syncer.oldestTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)

        val alreadyProcessedAccelerometerMeasurements: LinkedList<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())

        val alreadyProcessedMagnetometerMeasurements: LinkedList<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedMagnetometerMeasurements")
        requireNotNull(alreadyProcessedMagnetometerMeasurements)
        assertTrue(alreadyProcessedMagnetometerMeasurements.isEmpty())

        verify { syncedMeasurementListener wasNot Called }
        assertTrue(syncer.running)
    }

    @Test
    fun magnetometerSensorCollector_whenAccuracyChangedListenerAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(context)

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)

        val listener = magnetometerSensorCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(magnetometerSensorCollector, SensorAccuracy.LOW)
    }

    @Test
    fun magnetometerSensorCollector_whenAccuracyChangedListenerAndListenerAvailable_notifies() {
        val accuracyChangedListener =
            mockk<SensorMeasurementSyncer.OnAccuracyChangedListener<AccelerometerAndMagnetometerSyncedSensorMeasurement, AccelerometerAndMagnetometerSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)

        val listener = magnetometerSensorCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(magnetometerSensorCollector, SensorAccuracy.LOW)

        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                syncer,
                SensorMeasurementSyncer.SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorAccuracy.LOW
            )
        }
    }

    @Test
    fun magnetometerSensorCollector_whenBufferFilledAndStopWhenFilledBuffer_stops() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(
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
            mockk<SensorMeasurementSyncer.OnBufferFilledListener<AccelerometerAndMagnetometerSyncedSensorMeasurement, AccelerometerAndMagnetometerSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(
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
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerAndMagnetometerSyncedSensorMeasurement, AccelerometerAndMagnetometerSensorMeasurementSyncer>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(
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
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerAndMagnetometerSyncedSensorMeasurement, AccelerometerAndMagnetometerSensorMeasurementSyncer>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(
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
    fun magnetometerSensorCollector_whenMeasurementMostRecentTimestampDefinedAndMeasurementsAvailable_copiesMeasurementsAndProcessesThem() {
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerAndMagnetometerSyncedSensorMeasurement, AccelerometerAndMagnetometerSensorMeasurementSyncer>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(
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
            SensorAccuracy.MEDIUM
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

        val availableMagnetometerMeasurements: LinkedList<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("availableMagnetometerMeasurements")
        requireNotNull(availableMagnetometerMeasurements)
        assertEquals(syncer.magnetometerCapacity - 1, availableMagnetometerMeasurements.size)

        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
        val magnetometerMeasurements: LinkedList<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("magnetometerMeasurements")
        requireNotNull(magnetometerMeasurements)
        assertEquals(1, magnetometerMeasurements.size)
        assertNotSame(magnetometerMeasurements, magnetometerMeasurements[0])
        assertEquals(bx, magnetometerMeasurements[0].bx, 0.0f)
        assertEquals(by, magnetometerMeasurements[0].by, 0.0f)
        assertEquals(bz, magnetometerMeasurements[0].bz, 0.0f)
        assertEquals(hardIronX, magnetometerMeasurements[0].hardIronX)
        assertEquals(hardIronY, magnetometerMeasurements[0].hardIronY)
        assertEquals(hardIronZ, magnetometerMeasurements[0].hardIronZ)
        assertEquals(timestamp, magnetometerMeasurements[0].timestamp)
        assertEquals(SensorAccuracy.MEDIUM, magnetometerMeasurements[0].accuracy)

        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
        assertNull(syncer.oldestTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)

        val alreadyProcessedAccelerometerMeasurements: LinkedList<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())

        val alreadyProcessedMagnetometerMeasurements: LinkedList<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedMagnetometerMeasurements")
        requireNotNull(alreadyProcessedMagnetometerMeasurements)
        assertTrue(alreadyProcessedMagnetometerMeasurements.isEmpty())

        verify { syncedMeasurementListener wasNot Called }
        assertTrue(syncer.running)
    }

    @Test
    fun gyroscopeSensorCollector_whenMeasurementMostRecentTimestampDefinedMeasurementsAvailableAndEmptyCache_copiesMeasurementsAndProcessesThem() {
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerAndMagnetometerSyncedSensorMeasurement, AccelerometerAndMagnetometerSensorMeasurementSyncer>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(
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
            SensorAccuracy.MEDIUM
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

        val availableMagnetometerMeasurements: LinkedList<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("availableMagnetometerMeasurements")
        requireNotNull(availableMagnetometerMeasurements)
        availableMagnetometerMeasurements.clear()

        val listener = magnetometerSensorCollector.measurementListener
        requireNotNull(listener)

        listener.onMeasurement(magnetometerSensorCollectorSpy, MagnetometerSensorMeasurement(), 0)

        verify(exactly = 1) {
            magnetometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(
                mostRecentTimestamp
            )
        }

        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
        val magnetometerMeasurements: LinkedList<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("magnetometerMeasurements")
        requireNotNull(magnetometerMeasurements)
        assertEquals(1, magnetometerMeasurements.size)
        assertNotSame(magnetometerMeasurements, magnetometerMeasurements[0])
        assertEquals(bx, magnetometerMeasurements[0].bx, 0.0f)
        assertEquals(by, magnetometerMeasurements[0].by, 0.0f)
        assertEquals(bz, magnetometerMeasurements[0].bz, 0.0f)
        assertEquals(hardIronX, magnetometerMeasurements[0].hardIronX)
        assertEquals(hardIronY, magnetometerMeasurements[0].hardIronY)
        assertEquals(hardIronZ, magnetometerMeasurements[0].hardIronZ)
        assertEquals(timestamp, magnetometerMeasurements[0].timestamp)
        assertEquals(SensorAccuracy.MEDIUM, magnetometerMeasurements[0].accuracy)

        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
        assertNull(syncer.oldestTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)

        val alreadyProcessedAccelerometerMeasurements: LinkedList<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())

        val alreadyProcessedMagnetometerMeasurements: LinkedList<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedMagnetometerMeasurements")
        requireNotNull(alreadyProcessedMagnetometerMeasurements)
        assertTrue(alreadyProcessedMagnetometerMeasurements.isEmpty())

        verify { syncedMeasurementListener wasNot Called }
        assertTrue(syncer.running)
    }

    @Test
    fun sensorCollectors_whenAllMeasurementsAndOutOfOrder_notifiesOutOfOrder() {
        val outOfOrderMeasurementListener =
            mockk<SensorMeasurementSyncer.OnOutOfOrderMeasurementListener<AccelerometerAndMagnetometerSyncedSensorMeasurement, AccelerometerAndMagnetometerSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerAndMagnetometerSyncedSensorMeasurement, AccelerometerAndMagnetometerSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(
            context,
            outOfOrderDetectionEnabled = true,
            stopWhenOutOfOrder = false,
            outOfOrderMeasurementListener = outOfOrderMeasurementListener,
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
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val accelerometerTimestamp = System.nanoTime()
        val accelerometerMeasurement =
            AccelerometerSensorMeasurement(
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
        measurementsBeforePosition.add(accelerometerMeasurement)
        every { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(any()) }.returns(
            measurementsBeforePosition
        )
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)

        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val magnetometerTimestamp = accelerometerTimestamp - 1
        val magnetometerMeasurement = MagnetometerSensorMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            magnetometerTimestamp,
            SensorAccuracy.MEDIUM
        )
        val measurementsBeforeTimestamp = ArrayDeque<MagnetometerSensorMeasurement>()
        measurementsBeforeTimestamp.add(magnetometerMeasurement)
        every { magnetometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            measurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        val accelerometerListener = accelerometerSensorCollectorSpy.measurementListener
        requireNotNull(accelerometerListener)

        val magnetometerListener = magnetometerSensorCollector.measurementListener
        requireNotNull(magnetometerListener)

        // process accelerometer measurement
        accelerometerListener.onMeasurement(
            accelerometerSensorCollectorSpy,
            AccelerometerSensorMeasurement(),
            0
        )

        assertEquals(accelerometerTimestamp, syncer.oldestTimestamp)
        assertEquals(accelerometerTimestamp, syncer.mostRecentTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        verify(exactly = 1) { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(0) }

        val availableAccelerometerMeasurements: LinkedList<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("availableAccelerometerMeasurements")
        requireNotNull(availableAccelerometerMeasurements)
        assertEquals(syncer.accelerometerCapacity - 1, availableAccelerometerMeasurements.size)
        val accelerometerMeasurements: LinkedList<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("accelerometerMeasurements")
        requireNotNull(accelerometerMeasurements)
        assertEquals(1, accelerometerMeasurements.size)
        assertNotSame(accelerometerMeasurement, accelerometerMeasurements[0])
        assertEquals(ax, accelerometerMeasurements[0].ax, 0.0f)
        assertEquals(ay, accelerometerMeasurements[0].ay, 0.0f)
        assertEquals(az, accelerometerMeasurements[0].az, 0.0f)
        assertEquals(abx, accelerometerMeasurements[0].bx)
        assertEquals(aby, accelerometerMeasurements[0].by)
        assertEquals(abz, accelerometerMeasurements[0].bz)
        assertEquals(accelerometerTimestamp, accelerometerMeasurements[0].timestamp)
        assertEquals(SensorAccuracy.HIGH, accelerometerMeasurements[0].accuracy)

        val alreadyProcessedAccelerometerMeasurements: LinkedList<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())

        val alreadyProcessedMagnetometerMeasurements: LinkedList<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedMagnetometerMeasurements")
        requireNotNull(alreadyProcessedMagnetometerMeasurements)
        assertTrue(alreadyProcessedMagnetometerMeasurements.isEmpty())

        assertTrue(syncer.running)

        // process magnetometer measurement
        magnetometerListener.onMeasurement(
            magnetometerSensorCollectorSpy,
            MagnetometerSensorMeasurement(),
            0
        )

        assertEquals(accelerometerTimestamp, syncer.oldestTimestamp)
        assertEquals(1, syncer.numberOfProcessedMeasurements)

        val availableMagnetometerMeasurements: LinkedList<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("availableMagnetometerMeasurements")
        requireNotNull(availableMagnetometerMeasurements)

        val magnetometerMeasurements: LinkedList<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("magnetometerMeasurements")
        requireNotNull(magnetometerMeasurements)

        assertTrue(accelerometerMeasurements.isEmpty())
        assertEquals(syncer.accelerometerCapacity, availableAccelerometerMeasurements.size)
        assertTrue(magnetometerMeasurements.isEmpty())
        assertEquals(syncer.magnetometerCapacity, availableMagnetometerMeasurements.size)

        // verify out of order was notified
        verify(exactly = 1) {
            outOfOrderMeasurementListener.onOutOfOrderMeasurement(
                syncer,
                SensorMeasurementSyncer.SensorType.MAGNETOMETER_UNCALIBRATED,
                any()
            )
        }

        // verify syncer was not stopped
        assertTrue(syncer.running)

        val slot = slot<AccelerometerAndMagnetometerSyncedSensorMeasurement>()
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
    fun sensorCollectors_whenAllMeasurementsOutOfOrderAndStopWhenOutOfOrder_notifiesOutOfOrderAndStops() {
        val outOfOrderMeasurementListener =
            mockk<SensorMeasurementSyncer.OnOutOfOrderMeasurementListener<AccelerometerAndMagnetometerSyncedSensorMeasurement, AccelerometerAndMagnetometerSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerAndMagnetometerSyncedSensorMeasurement, AccelerometerAndMagnetometerSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(
            context,
            outOfOrderDetectionEnabled = true,
            stopWhenOutOfOrder = true,
            outOfOrderMeasurementListener = outOfOrderMeasurementListener,
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
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val accelerometerTimestamp = System.nanoTime()
        val accelerometerMeasurement =
            AccelerometerSensorMeasurement(
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
        measurementsBeforePosition.add(accelerometerMeasurement)
        every { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(any()) }.returns(
            measurementsBeforePosition
        )
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)

        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val magnetometerTimestamp = accelerometerTimestamp - 1
        val magnetometerMeasurement = MagnetometerSensorMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            magnetometerTimestamp,
            SensorAccuracy.MEDIUM
        )
        val measurementsBeforeTimestamp = ArrayDeque<MagnetometerSensorMeasurement>()
        measurementsBeforeTimestamp.add(magnetometerMeasurement)
        every { magnetometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            measurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        val accelerometerListener = accelerometerSensorCollectorSpy.measurementListener
        requireNotNull(accelerometerListener)

        val magnetometerListener = magnetometerSensorCollector.measurementListener
        requireNotNull(magnetometerListener)

        // process accelerometer measurement
        accelerometerListener.onMeasurement(
            accelerometerSensorCollectorSpy,
            AccelerometerSensorMeasurement(),
            0
        )

        assertEquals(accelerometerTimestamp, syncer.oldestTimestamp)
        assertEquals(accelerometerTimestamp, syncer.mostRecentTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        verify(exactly = 1) { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(0) }

        val availableAccelerometerMeasurements: LinkedList<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("availableAccelerometerMeasurements")
        requireNotNull(availableAccelerometerMeasurements)
        assertEquals(syncer.accelerometerCapacity - 1, availableAccelerometerMeasurements.size)
        val accelerometerMeasurements: LinkedList<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("accelerometerMeasurements")
        requireNotNull(accelerometerMeasurements)
        assertEquals(1, accelerometerMeasurements.size)
        assertNotSame(accelerometerMeasurement, accelerometerMeasurements[0])
        assertEquals(ax, accelerometerMeasurements[0].ax, 0.0f)
        assertEquals(ay, accelerometerMeasurements[0].ay, 0.0f)
        assertEquals(az, accelerometerMeasurements[0].az, 0.0f)
        assertEquals(abx, accelerometerMeasurements[0].bx)
        assertEquals(aby, accelerometerMeasurements[0].by)
        assertEquals(abz, accelerometerMeasurements[0].bz)
        assertEquals(accelerometerTimestamp, accelerometerMeasurements[0].timestamp)
        assertEquals(SensorAccuracy.HIGH, accelerometerMeasurements[0].accuracy)

        val alreadyProcessedAccelerometerMeasurements: LinkedList<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())

        val alreadyProcessedMagnetometerMeasurements: LinkedList<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedMagnetometerMeasurements")
        requireNotNull(alreadyProcessedMagnetometerMeasurements)
        assertTrue(alreadyProcessedMagnetometerMeasurements.isEmpty())

        assertTrue(syncer.running)

        // process magnetometer measurement
        magnetometerListener.onMeasurement(
            magnetometerSensorCollectorSpy,
            MagnetometerSensorMeasurement(),
            0
        )

        val availableMagnetometerMeasurements: LinkedList<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("availableMagnetometerMeasurements")
        requireNotNull(availableMagnetometerMeasurements)
        assertEquals(syncer.magnetometerCapacity, availableMagnetometerMeasurements.size)

        val magnetometerMeasurements: LinkedList<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("magnetometerMeasurements")
        requireNotNull(magnetometerMeasurements)
        assertTrue(magnetometerMeasurements.isEmpty())

        assertTrue(accelerometerMeasurements.isEmpty())
        assertEquals(syncer.accelerometerCapacity, availableAccelerometerMeasurements.size)

        // verify out of order was notified
        verify(exactly = 1) {
            outOfOrderMeasurementListener.onOutOfOrderMeasurement(
                syncer,
                SensorMeasurementSyncer.SensorType.MAGNETOMETER_UNCALIBRATED,
                any()
            )
        }

        // verify syncer was stopped
        assertFalse(syncer.running)
        assertNull(syncer.oldestTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)

        verify { syncedMeasurementListener wasNot Called }
    }

    @Test
    fun sensorCollectors_whenAllMeasurements_notifiesSyncedMeasurement() {
        val outOfOrderMeasurementListener =
            mockk<SensorMeasurementSyncer.OnOutOfOrderMeasurementListener<AccelerometerAndMagnetometerSyncedSensorMeasurement, AccelerometerAndMagnetometerSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerAndMagnetometerSyncedSensorMeasurement, AccelerometerAndMagnetometerSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(
            context,
            outOfOrderDetectionEnabled = false,
            stopWhenOutOfOrder = false,
            outOfOrderMeasurementListener = outOfOrderMeasurementListener,
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
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val accelerometerTimestamp = System.nanoTime()
        val accelerometerMeasurement =
            AccelerometerSensorMeasurement(
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
        measurementsBeforePosition.add(accelerometerMeasurement)
        every { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(any()) }.returns(
            measurementsBeforePosition
        )
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)

        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val magnetometerTimestamp = accelerometerTimestamp - 1
        val magnetometerMeasurement = MagnetometerSensorMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            magnetometerTimestamp,
            SensorAccuracy.MEDIUM
        )
        val measurementsBeforeTimestamp = ArrayDeque<MagnetometerSensorMeasurement>()
        measurementsBeforeTimestamp.add(magnetometerMeasurement)
        every { magnetometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            measurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        val accelerometerListener = accelerometerSensorCollectorSpy.measurementListener
        requireNotNull(accelerometerListener)

        val magnetometerListener = magnetometerSensorCollector.measurementListener
        requireNotNull(magnetometerListener)

        // process accelerometer measurement
        accelerometerListener.onMeasurement(
            accelerometerSensorCollectorSpy,
            AccelerometerSensorMeasurement(),
            0
        )

        assertEquals(accelerometerTimestamp, syncer.oldestTimestamp)
        assertEquals(accelerometerTimestamp, syncer.mostRecentTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        verify(exactly = 1) { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(0) }

        val availableAccelerometerMeasurements: LinkedList<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("availableAccelerometerMeasurements")
        requireNotNull(availableAccelerometerMeasurements)
        assertEquals(syncer.accelerometerCapacity - 1, availableAccelerometerMeasurements.size)
        val accelerometerMeasurements: LinkedList<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("accelerometerMeasurements")
        requireNotNull(accelerometerMeasurements)
        assertEquals(1, accelerometerMeasurements.size)
        assertNotSame(accelerometerMeasurement, accelerometerMeasurements[0])
        assertEquals(ax, accelerometerMeasurements[0].ax, 0.0f)
        assertEquals(ay, accelerometerMeasurements[0].ay, 0.0f)
        assertEquals(az, accelerometerMeasurements[0].az, 0.0f)
        assertEquals(abx, accelerometerMeasurements[0].bx)
        assertEquals(aby, accelerometerMeasurements[0].by)
        assertEquals(abz, accelerometerMeasurements[0].bz)
        assertEquals(accelerometerTimestamp, accelerometerMeasurements[0].timestamp)
        assertEquals(SensorAccuracy.HIGH, accelerometerMeasurements[0].accuracy)

        val alreadyProcessedAccelerometerMeasurements: LinkedList<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())

        val alreadyProcessedMagnetometerMeasurements: LinkedList<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedMagnetometerMeasurements")
        requireNotNull(alreadyProcessedMagnetometerMeasurements)
        assertTrue(alreadyProcessedMagnetometerMeasurements.isEmpty())

        assertTrue(syncer.running)

        // process magnetometer measurement
        magnetometerListener.onMeasurement(
            magnetometerSensorCollectorSpy,
            MagnetometerSensorMeasurement(),
            0
        )

        assertEquals(accelerometerTimestamp, syncer.oldestTimestamp)
        assertEquals(1, syncer.numberOfProcessedMeasurements)

        val availableMagnetometerMeasurements: LinkedList<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("availableMagnetometerMeasurements")
        requireNotNull(availableMagnetometerMeasurements)

        val magnetometerMeasurements: LinkedList<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("magnetometerMeasurements")
        requireNotNull(magnetometerMeasurements)
        assertTrue(magnetometerMeasurements.isEmpty())

        assertTrue(accelerometerMeasurements.isEmpty())
        assertEquals(syncer.accelerometerCapacity, availableAccelerometerMeasurements.size)
        assertTrue(magnetometerMeasurements.isEmpty())
        assertEquals(syncer.magnetometerCapacity, availableMagnetometerMeasurements.size)

        // verify out of order was not notified
        verify { outOfOrderMeasurementListener wasNot Called }

        // verify syncer was not stopped
        assertTrue(syncer.running)

        val slot = slot<AccelerometerAndMagnetometerSyncedSensorMeasurement>()
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
    fun sensorCollectors_whenAllMeasurementsAndPreviousAccelerometerTimestamp_notifiesSyncedMeasurement() {
        val outOfOrderMeasurementListener =
            mockk<SensorMeasurementSyncer.OnOutOfOrderMeasurementListener<AccelerometerAndMagnetometerSyncedSensorMeasurement, AccelerometerAndMagnetometerSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerAndMagnetometerSyncedSensorMeasurement, AccelerometerAndMagnetometerSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(
            context,
            outOfOrderDetectionEnabled = true,
            stopWhenOutOfOrder = false,
            outOfOrderMeasurementListener = outOfOrderMeasurementListener,
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
        val ax1 = randomizer.nextFloat()
        val ay1 = randomizer.nextFloat()
        val az1 = randomizer.nextFloat()
        val abx1 = randomizer.nextFloat()
        val aby1 = randomizer.nextFloat()
        val abz1 = randomizer.nextFloat()
        val accelerometerTimestamp1 = System.nanoTime()
        val accelerometerMeasurement1 =
            AccelerometerSensorMeasurement(
                ax1,
                ay1,
                az1,
                abx1,
                aby1,
                abz1,
                accelerometerTimestamp1,
                SensorAccuracy.HIGH
            )

        val ax2 = randomizer.nextFloat()
        val ay2 = randomizer.nextFloat()
        val az2 = randomizer.nextFloat()
        val abx2 = randomizer.nextFloat()
        val aby2 = randomizer.nextFloat()
        val abz2 = randomizer.nextFloat()
        val accelerometerTimestamp2 = accelerometerTimestamp1 + 1
        val accelerometerMeasurement2 =
            AccelerometerSensorMeasurement(
                ax2,
                ay2,
                az2,
                abx2,
                aby2,
                abz2,
                accelerometerTimestamp2,
                SensorAccuracy.HIGH
            )

        val measurementsBeforePosition = ArrayDeque<AccelerometerSensorMeasurement>()
        measurementsBeforePosition.add(accelerometerMeasurement1)
        measurementsBeforePosition.add(accelerometerMeasurement2)
        every { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(any()) }.returns(
            measurementsBeforePosition
        )
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)

        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val magnetometerTimestamp = accelerometerTimestamp2 - 1
        val magnetometerMeasurement = MagnetometerSensorMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            magnetometerTimestamp,
            SensorAccuracy.MEDIUM
        )
        val measurementsBeforeTimestamp = ArrayDeque<MagnetometerSensorMeasurement>()
        measurementsBeforeTimestamp.add(magnetometerMeasurement)
        every { magnetometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            measurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        val accelerometerListener = accelerometerSensorCollectorSpy.measurementListener
        requireNotNull(accelerometerListener)

        val magnetometerListener = magnetometerSensorCollector.measurementListener
        requireNotNull(magnetometerListener)

        // process accelerometer measurement
        accelerometerListener.onMeasurement(
            accelerometerSensorCollectorSpy,
            AccelerometerSensorMeasurement(),
            0
        )

        assertEquals(accelerometerTimestamp1, syncer.oldestTimestamp)
        assertEquals(accelerometerTimestamp2, syncer.mostRecentTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        verify(exactly = 1) { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(0) }

        val availableAccelerometerMeasurements: LinkedList<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("availableAccelerometerMeasurements")
        requireNotNull(availableAccelerometerMeasurements)
        assertEquals(syncer.accelerometerCapacity - 2, availableAccelerometerMeasurements.size)
        val accelerometerMeasurements: LinkedList<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("accelerometerMeasurements")
        requireNotNull(accelerometerMeasurements)
        assertEquals(2, accelerometerMeasurements.size)
        assertNotSame(accelerometerMeasurement1, accelerometerMeasurements[0])
        assertEquals(ax1, accelerometerMeasurements[0].ax, 0.0f)
        assertEquals(ay1, accelerometerMeasurements[0].ay, 0.0f)
        assertEquals(az1, accelerometerMeasurements[0].az, 0.0f)
        assertEquals(abx1, accelerometerMeasurements[0].bx)
        assertEquals(aby1, accelerometerMeasurements[0].by)
        assertEquals(abz1, accelerometerMeasurements[0].bz)
        assertEquals(accelerometerTimestamp1, accelerometerMeasurements[0].timestamp)
        assertEquals(SensorAccuracy.HIGH, accelerometerMeasurements[0].accuracy)

        assertNotSame(accelerometerMeasurement2, accelerometerMeasurements[1])
        assertEquals(ax2, accelerometerMeasurements[1].ax, 0.0f)
        assertEquals(ay2, accelerometerMeasurements[1].ay, 0.0f)
        assertEquals(az2, accelerometerMeasurements[1].az, 0.0f)
        assertEquals(abx2, accelerometerMeasurements[1].bx)
        assertEquals(aby2, accelerometerMeasurements[1].by)
        assertEquals(abz2, accelerometerMeasurements[1].bz)
        assertEquals(accelerometerTimestamp2, accelerometerMeasurements[1].timestamp)
        assertEquals(SensorAccuracy.HIGH, accelerometerMeasurements[1].accuracy)

        val alreadyProcessedAccelerometerMeasurements: LinkedList<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())

        val alreadyProcessedMagnetometerMeasurements: LinkedList<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedMagnetometerMeasurements")
        requireNotNull(alreadyProcessedMagnetometerMeasurements)
        assertTrue(alreadyProcessedMagnetometerMeasurements.isEmpty())

        assertTrue(syncer.running)

        // process magnetometer measurement
        magnetometerListener.onMeasurement(
            magnetometerSensorCollectorSpy,
            MagnetometerSensorMeasurement(),
            0
        )

        assertEquals(accelerometerTimestamp1, syncer.oldestTimestamp)
        assertEquals(1, syncer.numberOfProcessedMeasurements)

        val availableMagnetometerMeasurements: LinkedList<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("availableMagnetometerMeasurements")
        requireNotNull(availableMagnetometerMeasurements)

        val magnetometerMeasurements: LinkedList<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("magnetometerMeasurements")
        requireNotNull(magnetometerMeasurements)
        assertTrue(magnetometerMeasurements.isEmpty())

        assertEquals(1, accelerometerMeasurements.size)
        assertNotSame(accelerometerMeasurement1, accelerometerMeasurements[0])
        assertEquals(ax1, accelerometerMeasurements[0].ax, 0.0f)
        assertEquals(ay1, accelerometerMeasurements[0].ay, 0.0f)
        assertEquals(az1, accelerometerMeasurements[0].az, 0.0f)
        assertEquals(abx1, accelerometerMeasurements[0].bx)
        assertEquals(aby1, accelerometerMeasurements[0].by)
        assertEquals(abz1, accelerometerMeasurements[0].bz)
        assertEquals(accelerometerTimestamp1, accelerometerMeasurements[0].timestamp)
        assertEquals(SensorAccuracy.HIGH, accelerometerMeasurements[0].accuracy)

        assertEquals(syncer.accelerometerCapacity - 1, availableAccelerometerMeasurements.size)

        assertTrue(magnetometerMeasurements.isEmpty())
        assertEquals(syncer.magnetometerCapacity, availableMagnetometerMeasurements.size)

        // verify out of order was not notified
        verify { outOfOrderMeasurementListener wasNot Called }

        // verify syncer was not stopped
        assertTrue(syncer.running)

        val slot = slot<AccelerometerAndMagnetometerSyncedSensorMeasurement>()
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
        assertEquals(ax2, syncedAccelerometerMeasurement.ax, 0.0f)
        assertEquals(ay2, syncedAccelerometerMeasurement.ay, 0.0f)
        assertEquals(az2, syncedAccelerometerMeasurement.az, 0.0f)
        assertEquals(abx2, syncedAccelerometerMeasurement.bx)
        assertEquals(aby2, syncedAccelerometerMeasurement.by)
        assertEquals(abz2, syncedAccelerometerMeasurement.bz)
        assertEquals(accelerometerTimestamp2, syncedAccelerometerMeasurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, syncedAccelerometerMeasurement.accuracy)
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
    fun sensorCollectors_whenMagnetometerTimestampGreaterThanAccelerometerTimestamp_doesNotNotify() {
        val outOfOrderMeasurementListener =
            mockk<SensorMeasurementSyncer.OnOutOfOrderMeasurementListener<AccelerometerAndMagnetometerSyncedSensorMeasurement, AccelerometerAndMagnetometerSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerAndMagnetometerSyncedSensorMeasurement, AccelerometerAndMagnetometerSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(
            context,
            outOfOrderDetectionEnabled = false,
            stopWhenOutOfOrder = false,
            outOfOrderMeasurementListener = outOfOrderMeasurementListener,
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
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val accelerometerTimestamp = System.nanoTime()
        val accelerometerMeasurement =
            AccelerometerSensorMeasurement(
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
        measurementsBeforePosition.add(accelerometerMeasurement)
        every { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(any()) }.returns(
            measurementsBeforePosition
        )
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)

        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val magnetometerTimestamp = accelerometerTimestamp + 1
        val magnetometerMeasurement = MagnetometerSensorMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            magnetometerTimestamp,
            SensorAccuracy.MEDIUM
        )
        val measurementsBeforeTimestamp = ArrayDeque<MagnetometerSensorMeasurement>()
        measurementsBeforeTimestamp.add(magnetometerMeasurement)
        every { magnetometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            measurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        val accelerometerListener = accelerometerSensorCollectorSpy.measurementListener
        requireNotNull(accelerometerListener)

        val magnetometerListener = magnetometerSensorCollector.measurementListener
        requireNotNull(magnetometerListener)

        // process accelerometer measurement
        accelerometerListener.onMeasurement(
            accelerometerSensorCollectorSpy,
            AccelerometerSensorMeasurement(),
            0
        )

        assertEquals(accelerometerTimestamp, syncer.oldestTimestamp)
        assertEquals(accelerometerTimestamp, syncer.mostRecentTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        verify(exactly = 1) { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(0) }

        val availableAccelerometerMeasurements: LinkedList<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("availableAccelerometerMeasurements")
        requireNotNull(availableAccelerometerMeasurements)
        assertEquals(syncer.accelerometerCapacity - 1, availableAccelerometerMeasurements.size)
        val accelerometerMeasurements: LinkedList<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("accelerometerMeasurements")
        requireNotNull(accelerometerMeasurements)
        assertEquals(1, accelerometerMeasurements.size)
        assertNotSame(accelerometerMeasurement, accelerometerMeasurements[0])
        assertEquals(ax, accelerometerMeasurements[0].ax, 0.0f)
        assertEquals(ay, accelerometerMeasurements[0].ay, 0.0f)
        assertEquals(az, accelerometerMeasurements[0].az, 0.0f)
        assertEquals(abx, accelerometerMeasurements[0].bx)
        assertEquals(aby, accelerometerMeasurements[0].by)
        assertEquals(abz, accelerometerMeasurements[0].bz)
        assertEquals(accelerometerTimestamp, accelerometerMeasurements[0].timestamp)
        assertEquals(SensorAccuracy.HIGH, accelerometerMeasurements[0].accuracy)

        val alreadyProcessedAccelerometerMeasurements: LinkedList<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())

        val alreadyProcessedMagnetometerMeasurements: LinkedList<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedMagnetometerMeasurements")
        requireNotNull(alreadyProcessedMagnetometerMeasurements)
        assertTrue(alreadyProcessedMagnetometerMeasurements.isEmpty())

        assertTrue(syncer.running)

        // process magnetometer measurement
        magnetometerListener.onMeasurement(
            magnetometerSensorCollectorSpy,
            MagnetometerSensorMeasurement(),
            0
        )

        assertEquals(accelerometerTimestamp, syncer.oldestTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)

        val availableMagnetometerMeasurements: LinkedList<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("availableMagnetometerMeasurements")
        requireNotNull(availableMagnetometerMeasurements)
        assertEquals(syncer.magnetometerCapacity - 1, availableMagnetometerMeasurements.size)
        val magnetometerMeasurements: LinkedList<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("magnetometerMeasurements")
        requireNotNull(magnetometerMeasurements)
        assertEquals(1, magnetometerMeasurements.size)
        assertNotSame(magnetometerMeasurement, magnetometerMeasurements[0])
        assertEquals(bx, magnetometerMeasurements[0].bx, 0.0f)
        assertEquals(by, magnetometerMeasurements[0].by, 0.0f)
        assertEquals(bz, magnetometerMeasurements[0].bz, 0.0f)
        assertEquals(hardIronX, magnetometerMeasurements[0].hardIronX)
        assertEquals(hardIronY, magnetometerMeasurements[0].hardIronY)
        assertEquals(hardIronZ, magnetometerMeasurements[0].hardIronZ)
        assertEquals(magnetometerTimestamp, magnetometerMeasurements[0].timestamp)
        assertEquals(SensorAccuracy.MEDIUM, magnetometerMeasurements[0].accuracy)

        assertEquals(1, accelerometerMeasurements.size)
        assertNotSame(accelerometerMeasurement, accelerometerMeasurements[0])
        assertEquals(ax, accelerometerMeasurements[0].ax, 0.0f)
        assertEquals(ay, accelerometerMeasurements[0].ay, 0.0f)
        assertEquals(az, accelerometerMeasurements[0].az, 0.0f)
        assertEquals(abx, accelerometerMeasurements[0].bx)
        assertEquals(aby, accelerometerMeasurements[0].by)
        assertEquals(abz, accelerometerMeasurements[0].bz)
        assertEquals(accelerometerTimestamp, accelerometerMeasurements[0].timestamp)
        assertEquals(SensorAccuracy.HIGH, accelerometerMeasurements[0].accuracy)
        assertEquals(syncer.accelerometerCapacity - 1, availableAccelerometerMeasurements.size)

        // verify out of order was not notified
        verify { outOfOrderMeasurementListener wasNot Called }

        // verify syncer was not stopped
        assertTrue(syncer.running)

        // verify no synced measurement was notified
        verify { syncedMeasurementListener wasNot Called }
    }

    @Test
    fun resizeAvailableMeasurementsIfNeeded_whenAccelerometerCapacityExceeded_resizesAvailableAccelerometerMeasurements() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(context)

        val availableAccelerometerMeasurements: LinkedList<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("availableAccelerometerMeasurements")
        requireNotNull(availableAccelerometerMeasurements)
        assertEquals(syncer.accelerometerCapacity, availableAccelerometerMeasurements.size)

        val availableMagnetometerMeasurements: LinkedList<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("availableMagnetometerMeasurements")
        requireNotNull(availableMagnetometerMeasurements)
        assertEquals(syncer.magnetometerCapacity, availableMagnetometerMeasurements.size)

        // add accelerometer measurement
        availableAccelerometerMeasurements.add(AccelerometerSensorMeasurement())
        assertEquals(syncer.accelerometerCapacity + 1, availableAccelerometerMeasurements.size)

        callPrivateFunc(
            AccelerometerAndMagnetometerSensorMeasurementSyncer::class,
            syncer,
            "resizeAvailableMeasurementsIfNeeded"
        )

        // check
        assertEquals(syncer.accelerometerCapacity, availableAccelerometerMeasurements.size)
        assertEquals(syncer.magnetometerCapacity, availableMagnetometerMeasurements.size)
    }

    @Test
    fun resizeAvailableMeasurementsIfNeeded_whenMagnetometerCapacityExceeded_resizesAvailableAccelerometerMeasurements() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndMagnetometerSensorMeasurementSyncer(context)

        val availableAccelerometerMeasurements: LinkedList<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("availableAccelerometerMeasurements")
        requireNotNull(availableAccelerometerMeasurements)
        assertEquals(syncer.accelerometerCapacity, availableAccelerometerMeasurements.size)

        val availableMagnetometerMeasurements: LinkedList<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("availableMagnetometerMeasurements")
        requireNotNull(availableMagnetometerMeasurements)
        assertEquals(syncer.magnetometerCapacity, availableMagnetometerMeasurements.size)

        // add magnetometer measurement
        availableMagnetometerMeasurements.add(MagnetometerSensorMeasurement())
        assertEquals(syncer.magnetometerCapacity + 1, availableMagnetometerMeasurements.size)

        callPrivateFunc(
            AccelerometerAndMagnetometerSensorMeasurementSyncer::class,
            syncer,
            "resizeAvailableMeasurementsIfNeeded"
        )

        // check
        assertEquals(syncer.accelerometerCapacity, availableAccelerometerMeasurements.size)
        assertEquals(syncer.magnetometerCapacity, availableMagnetometerMeasurements.size)
    }

    @Config(sdk = [Build.VERSION_CODES.O])
    @Test
    fun sensorType_fromIntWhenSdkO_returnsExpectedValues() {
        assertEquals(7, SensorMeasurementSyncer.SensorType.values().size)
        assertEquals(
            SensorMeasurementSyncer.SensorType.ACCELEROMETER,
            SensorMeasurementSyncer.SensorType.from(SensorMeasurementSyncer.SensorType.ACCELEROMETER.value)
        )
        assertEquals(
            SensorMeasurementSyncer.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorMeasurementSyncer.SensorType.from(SensorMeasurementSyncer.SensorType.ACCELEROMETER_UNCALIBRATED.value)
        )
        assertEquals(
            SensorMeasurementSyncer.SensorType.GYROSCOPE,
            SensorMeasurementSyncer.SensorType.from(SensorMeasurementSyncer.SensorType.GYROSCOPE.value)
        )
        assertEquals(
            SensorMeasurementSyncer.SensorType.GYROSCOPE_UNCALIBRATED,
            SensorMeasurementSyncer.SensorType.from(SensorMeasurementSyncer.SensorType.GYROSCOPE_UNCALIBRATED.value)
        )
        assertEquals(
            SensorMeasurementSyncer.SensorType.MAGNETOMETER,
            SensorMeasurementSyncer.SensorType.from(SensorMeasurementSyncer.SensorType.MAGNETOMETER.value)
        )
        assertEquals(
            SensorMeasurementSyncer.SensorType.MAGNETOMETER_UNCALIBRATED,
            SensorMeasurementSyncer.SensorType.from(SensorMeasurementSyncer.SensorType.MAGNETOMETER_UNCALIBRATED.value)
        )
        assertEquals(
            SensorMeasurementSyncer.SensorType.GRAVITY,
            SensorMeasurementSyncer.SensorType.from(SensorMeasurementSyncer.SensorType.GRAVITY.value)
        )
    }

    @Config(sdk = [Build.VERSION_CODES.N])
    @Test
    fun sensorType_fromIntWhenSdkN_returnsExpectedValues() {
        assertEquals(7, SensorMeasurementSyncer.SensorType.values().size)
        assertEquals(
            SensorMeasurementSyncer.SensorType.ACCELEROMETER,
            SensorMeasurementSyncer.SensorType.from(SensorMeasurementSyncer.SensorType.ACCELEROMETER.value)
        )
        assertNull(SensorMeasurementSyncer.SensorType.from(SensorMeasurementSyncer.SensorType.ACCELEROMETER_UNCALIBRATED.value))
        assertEquals(
            SensorMeasurementSyncer.SensorType.GYROSCOPE,
            SensorMeasurementSyncer.SensorType.from(SensorMeasurementSyncer.SensorType.GYROSCOPE.value)
        )
        assertEquals(
            SensorMeasurementSyncer.SensorType.GYROSCOPE_UNCALIBRATED,
            SensorMeasurementSyncer.SensorType.from(SensorMeasurementSyncer.SensorType.GYROSCOPE_UNCALIBRATED.value)
        )
        assertEquals(
            SensorMeasurementSyncer.SensorType.MAGNETOMETER,
            SensorMeasurementSyncer.SensorType.from(SensorMeasurementSyncer.SensorType.MAGNETOMETER.value)
        )
        assertEquals(
            SensorMeasurementSyncer.SensorType.MAGNETOMETER_UNCALIBRATED,
            SensorMeasurementSyncer.SensorType.from(SensorMeasurementSyncer.SensorType.MAGNETOMETER_UNCALIBRATED.value)
        )
        assertEquals(
            SensorMeasurementSyncer.SensorType.GRAVITY,
            SensorMeasurementSyncer.SensorType.from(SensorMeasurementSyncer.SensorType.GRAVITY.value)
        )
    }

    @Test
    fun sensorType_fromAccelerometerSensorType_returnsExpectedValue() {
        assertEquals(
            SensorMeasurementSyncer.SensorType.ACCELEROMETER,
            SensorMeasurementSyncer.SensorType.from(AccelerometerSensorType.ACCELEROMETER)
        )
        assertEquals(
            SensorMeasurementSyncer.SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorMeasurementSyncer.SensorType.from(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED)
        )
    }

    @Test
    fun sensorType_fromGyroscopeSensorType_returnsExpectedValue() {
        assertEquals(
            SensorMeasurementSyncer.SensorType.GYROSCOPE,
            SensorMeasurementSyncer.SensorType.from(GyroscopeSensorType.GYROSCOPE)
        )
        assertEquals(
            SensorMeasurementSyncer.SensorType.GYROSCOPE_UNCALIBRATED,
            SensorMeasurementSyncer.SensorType.from(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED)
        )
    }

    @Test
    fun sensorType_fromMagnetometerSensorType_returnsExpectedValue() {
        assertEquals(
            SensorMeasurementSyncer.SensorType.MAGNETOMETER,
            SensorMeasurementSyncer.SensorType.from(MagnetometerSensorType.MAGNETOMETER)
        )
        assertEquals(
            SensorMeasurementSyncer.SensorType.MAGNETOMETER_UNCALIBRATED,
            SensorMeasurementSyncer.SensorType.from(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED)
        )
    }
}