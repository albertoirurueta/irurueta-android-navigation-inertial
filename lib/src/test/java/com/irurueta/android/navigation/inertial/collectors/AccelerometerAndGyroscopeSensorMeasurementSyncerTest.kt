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
class AccelerometerAndGyroscopeSensorMeasurementSyncerTest {

    @Test
    fun constructor_whenRequiredParameters_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(context)

        // check
        assertSame(context, syncer.context)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            syncer.accelerometerSensorType
        )
        assertEquals(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED, syncer.gyroscopeSensorType)
        assertEquals(SensorDelay.FASTEST, syncer.accelerometerSensorDelay)
        assertEquals(SensorDelay.FASTEST, syncer.gyroscopeSensorDelay)
        assertEquals(BufferedSensorCollector.DEFAULT_CAPACITY, syncer.accelerometerCapacity)
        assertEquals(BufferedSensorCollector.DEFAULT_CAPACITY, syncer.gyroscopeCapacity)
        assertTrue(syncer.accelerometerStartOffsetEnabled)
        assertTrue(syncer.gyroscopeStartOffsetEnabled)
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
        assertNull(syncer.gyroscopeSensor)
        assertFalse(syncer.accelerometerSensorAvailable)
        assertFalse(syncer.gyroscopeSensorAvailable)
        assertNull(syncer.accelerometerStartOffset)
        assertNull(syncer.gyroscopeStartOffset)
        assertEquals(0.0f, syncer.accelerometerUsage, 0.0f)
        assertEquals(0.0f, syncer.gyroscopeUsage, 0.0f)
    }

    @Test
    fun constructor_whenAllParameters_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val accuracyChangedListener =
            mockk<SensorMeasurementSyncer.OnAccuracyChangedListener<AccelerometerAndGyroscopeSyncedSensorMeasurement, AccelerometerAndGyroscopeSensorMeasurementSyncer>>()
        val bufferFilledListener =
            mockk<SensorMeasurementSyncer.OnBufferFilledListener<AccelerometerAndGyroscopeSyncedSensorMeasurement, AccelerometerAndGyroscopeSensorMeasurementSyncer>>()
        val outOfOrderMeasurementListener =
            mockk<SensorMeasurementSyncer.OnOutOfOrderMeasurementListener<AccelerometerAndGyroscopeSyncedSensorMeasurement, AccelerometerAndGyroscopeSensorMeasurementSyncer>>()
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerAndGyroscopeSyncedSensorMeasurement, AccelerometerAndGyroscopeSensorMeasurementSyncer>>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            SensorDelay.GAME,
            accelerometerCapacity = 2,
            gyroscopeCapacity = 3,
            accelerometerStartOffsetEnabled = false,
            gyroscopeStartOffsetEnabled = false,
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
        assertEquals(GyroscopeSensorType.GYROSCOPE, syncer.gyroscopeSensorType)
        assertEquals(SensorDelay.NORMAL, syncer.accelerometerSensorDelay)
        assertEquals(SensorDelay.GAME, syncer.gyroscopeSensorDelay)
        assertEquals(2, syncer.accelerometerCapacity)
        assertEquals(3, syncer.gyroscopeCapacity)
        assertFalse(syncer.accelerometerStartOffsetEnabled)
        assertFalse(syncer.gyroscopeStartOffsetEnabled)
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
        assertNull(syncer.gyroscopeSensor)
        assertFalse(syncer.accelerometerSensorAvailable)
        assertFalse(syncer.gyroscopeSensorAvailable)
        assertNull(syncer.accelerometerStartOffset)
        assertNull(syncer.gyroscopeStartOffset)
        assertEquals(0.0f, syncer.accelerometerUsage, 0.0f)
        assertEquals(0.0f, syncer.gyroscopeUsage, 0.0f)
    }

    @Test
    fun accuracyChangedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(context)

        // check default value
        assertNull(syncer.accuracyChangedListener)

        // set new value
        val accuracyChangedListener =
            mockk<SensorMeasurementSyncer.OnAccuracyChangedListener<AccelerometerAndGyroscopeSyncedSensorMeasurement, AccelerometerAndGyroscopeSensorMeasurementSyncer>>()
        syncer.accuracyChangedListener = accuracyChangedListener

        // check
        assertSame(accuracyChangedListener, syncer.accuracyChangedListener)
    }

    @Test
    fun bufferFilledListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(context)

        // check default value
        assertNull(syncer.bufferFilledListener)

        // set new value
        val bufferFilledListener =
            mockk<SensorMeasurementSyncer.OnBufferFilledListener<AccelerometerAndGyroscopeSyncedSensorMeasurement, AccelerometerAndGyroscopeSensorMeasurementSyncer>>()
        syncer.bufferFilledListener = bufferFilledListener

        // check
        assertSame(bufferFilledListener, syncer.bufferFilledListener)
    }

    @Test
    fun outOfOrderMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(context)

        // check default value
        assertNull(syncer.outOfOrderMeasurementListener)

        // set new value
        val outOfOrderMeasurementListener =
            mockk<SensorMeasurementSyncer.OnOutOfOrderMeasurementListener<AccelerometerAndGyroscopeSyncedSensorMeasurement, AccelerometerAndGyroscopeSensorMeasurementSyncer>>()
        syncer.outOfOrderMeasurementListener = outOfOrderMeasurementListener

        // check
        assertSame(outOfOrderMeasurementListener, syncer.outOfOrderMeasurementListener)
    }

    @Test
    fun syncedMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(context)

        // check default value
        assertNull(syncer.syncedMeasurementListener)

        // set new value
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerAndGyroscopeSyncedSensorMeasurement, AccelerometerAndGyroscopeSensorMeasurementSyncer>>()
        syncer.syncedMeasurementListener = syncedMeasurementListener

        // check
        assertSame(syncedMeasurementListener, syncer.syncedMeasurementListener)
    }

    @Test(expected = IllegalStateException::class)
    fun start_whenAlreadyRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(context)

        // set as running
        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "running", true)
        assertTrue(syncer.running)

        syncer.start()
    }

    @Test
    fun start_whenNotRunningAndNoTimestamp_setsCurrentStartTimestamp() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(context)

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
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(context)

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
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(context)

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

        assertFalse(syncer.running)

        val timestamp = System.nanoTime()
        assertFalse(syncer.start(timestamp))

        assertFalse(syncer.running)
        assertEquals(timestamp, syncer.startTimestamp)

        verify(exactly = 1) { accelerometerSensorCollectorSpy.start(timestamp) }
        verify(exactly = 0) { gyroscopeSensorCollectorSpy.start(any()) }
    }

    @Test
    fun start_whenGyroscopeCollectorFails_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(context)

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

        assertFalse(syncer.running)

        val timestamp = System.nanoTime()
        assertFalse(syncer.start(timestamp))

        assertFalse(syncer.running)
        assertEquals(timestamp, syncer.startTimestamp)

        verify(exactly = 1) { accelerometerSensorCollectorSpy.start(timestamp) }
        verify(exactly = 1) { gyroscopeSensorCollectorSpy.start(timestamp) }
    }

    @Test
    fun start_whenCollectorsSucceed_returnsTrue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(context)

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

        assertFalse(syncer.running)

        val timestamp = System.nanoTime()
        assertTrue(syncer.start(timestamp))

        assertTrue(syncer.running)
        assertEquals(timestamp, syncer.startTimestamp)

        verify(exactly = 1) { accelerometerSensorCollectorSpy.start(timestamp) }
        verify(exactly = 1) { gyroscopeSensorCollectorSpy.start(timestamp) }
    }

    @Test
    fun stop_stopsCollectorsInitializesCachesAndResetsProperties() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(context)

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

        val availableAccelerometerMeasurements: LinkedList<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("availableAccelerometerMeasurements")
        requireNotNull(availableAccelerometerMeasurements)
        availableAccelerometerMeasurements.clear()

        val availableGyroscopeMeasurements: LinkedList<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("availableGyroscopeMeasurements")
        requireNotNull(availableGyroscopeMeasurements)
        availableGyroscopeMeasurements.clear()

        val accelerometerMeasurements: LinkedList<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("accelerometerMeasurements")
        requireNotNull(accelerometerMeasurements)
        accelerometerMeasurements.add(AccelerometerSensorMeasurement())

        val gyroscopeMeasurements: LinkedList<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("gyroscopeMeasurements")
        requireNotNull(gyroscopeMeasurements)
        gyroscopeMeasurements.add(GyroscopeSensorMeasurement())

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
        verify(exactly = 1) { gyroscopeSensorCollectorSpy.stop() }
        assertEquals(syncer.accelerometerCapacity, availableAccelerometerMeasurements.size)
        assertEquals(syncer.gyroscopeCapacity, availableGyroscopeMeasurements.size)
        assertTrue(accelerometerMeasurements.isEmpty())
        assertTrue(gyroscopeMeasurements.isEmpty())

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
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(context)

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
            mockk<SensorMeasurementSyncer.OnAccuracyChangedListener<AccelerometerAndGyroscopeSyncedSensorMeasurement, AccelerometerAndGyroscopeSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(
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
            AccelerometerAndGyroscopeSensorMeasurementSyncer(context, stopWhenFilledBuffer = true)

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
            mockk<SensorMeasurementSyncer.OnBufferFilledListener<AccelerometerAndGyroscopeSyncedSensorMeasurement, AccelerometerAndGyroscopeSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(
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
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerAndGyroscopeSyncedSensorMeasurement, AccelerometerAndGyroscopeSensorMeasurementSyncer>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(
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
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerAndGyroscopeSyncedSensorMeasurement, AccelerometerAndGyroscopeSensorMeasurementSyncer>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(
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

        val alreadyProcessedGyroscopeMeasurements: LinkedList<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGyroscopeMeasurements")
        requireNotNull(alreadyProcessedGyroscopeMeasurements)
        assertTrue(alreadyProcessedGyroscopeMeasurements.isEmpty())

        verify { syncedMeasurementListener wasNot Called }
        assertTrue(syncer.running)
    }

    @Test
    fun accelerometerSensorCollector_whenMeasurementMeasurementsAvailableAndEmptyCache_updatesMostRecentTimestampCopiesMeasurementsAndProcessesThem() {
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerAndGyroscopeSyncedSensorMeasurement, AccelerometerAndGyroscopeSensorMeasurementSyncer>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(
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

        val alreadyProcessedGyroscopeMeasurements: LinkedList<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGyroscopeMeasurements")
        requireNotNull(alreadyProcessedGyroscopeMeasurements)
        assertTrue(alreadyProcessedGyroscopeMeasurements.isEmpty())

        verify { syncedMeasurementListener wasNot Called }
        assertTrue(syncer.running)
    }

    @Test
    fun gyroscopeSensorCollector_whenAccuracyChangedListenerAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(context)

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)

        val listener = gyroscopeSensorCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(gyroscopeSensorCollector, SensorAccuracy.HIGH)
    }

    @Test
    fun gyroscopeSensorCollector_whenAccuracyChangedListenerAndListenerAvailable_notifies() {
        val accuracyChangedListener =
            mockk<SensorMeasurementSyncer.OnAccuracyChangedListener<AccelerometerAndGyroscopeSyncedSensorMeasurement, AccelerometerAndGyroscopeSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)

        val listener = gyroscopeSensorCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(gyroscopeSensorCollector, SensorAccuracy.HIGH)

        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                syncer,
                SensorMeasurementSyncer.SensorType.GYROSCOPE_UNCALIBRATED,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun gyroscopeSensorCollector_whenBufferFilledAndStopWhenFilledBuffer_stops() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer =
            AccelerometerAndGyroscopeSensorMeasurementSyncer(context, stopWhenFilledBuffer = true)

        // set as running
        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "running", true)
        assertTrue(syncer.running)
        assertTrue(syncer.stopWhenFilledBuffer)

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)

        val listener = gyroscopeSensorCollector.bufferFilledListener
        requireNotNull(listener)

        listener.onBufferFilled(gyroscopeSensorCollector)

        assertFalse(syncer.running)
    }

    @Test
    fun gyroscopeSensorCollector_whenBufferFilledAndListenerAvailable_notifies() {
        val bufferFilledListener =
            mockk<SensorMeasurementSyncer.OnBufferFilledListener<AccelerometerAndGyroscopeSyncedSensorMeasurement, AccelerometerAndGyroscopeSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer =
            AccelerometerAndGyroscopeSensorMeasurementSyncer(
                context,
                stopWhenFilledBuffer = false,
                bufferFilledListener = bufferFilledListener
            )

        // set as running
        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "running", true)
        assertTrue(syncer.running)
        assertFalse(syncer.stopWhenFilledBuffer)

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)

        val listener = gyroscopeSensorCollector.bufferFilledListener
        requireNotNull(listener)

        listener.onBufferFilled(gyroscopeSensorCollector)

        verify(exactly = 1) {
            bufferFilledListener.onBufferFilled(
                syncer,
                SensorMeasurementSyncer.SensorType.GYROSCOPE_UNCALIBRATED
            )
        }
        assertTrue(syncer.running)
    }

    @Test
    fun gyroscopeSensorCollector_whenMeasurementAndMostRecentTimestampNotDefined_makesNoAction() {
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerAndGyroscopeSyncedSensorMeasurement, AccelerometerAndGyroscopeSensorMeasurementSyncer>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(
            context,
            syncedMeasurementListener = syncedMeasurementListener
        )

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        val listener = gyroscopeSensorCollector.measurementListener
        requireNotNull(listener)

        listener.onMeasurement(gyroscopeSensorCollectorSpy, GyroscopeSensorMeasurement(), 0)

        verify { gyroscopeSensorCollectorSpy wasNot Called }
        verify { syncedMeasurementListener wasNot Called }
        assertNull(syncer.mostRecentTimestamp)
    }

    @Test
    fun gyroscopeSensorCollector_whenMeasurementMostRecentTimestampDefinedAndNoMeasurementsAvailable_callsCollector() {
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerAndGyroscopeSyncedSensorMeasurement, AccelerometerAndGyroscopeSensorMeasurementSyncer>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(
            context,
            syncedMeasurementListener = syncedMeasurementListener
        )

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        every { gyroscopeSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            ArrayDeque()
        )
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        // set most recent timestamp
        val mostRecentTimestamp = System.nanoTime()
        setPrivateProperty(
            SensorMeasurementSyncer::class,
            syncer,
            "mostRecentTimestamp",
            mostRecentTimestamp
        )

        val listener = gyroscopeSensorCollector.measurementListener
        requireNotNull(listener)

        listener.onMeasurement(gyroscopeSensorCollectorSpy, GyroscopeSensorMeasurement(), 0)

        verify(exactly = 1) {
            gyroscopeSensorCollectorSpy.getMeasurementsBeforeTimestamp(
                mostRecentTimestamp
            )
        }
        verify { syncedMeasurementListener wasNot Called }
        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
    }

    @Test
    fun gyroscopeSensorCollector_whenMeasurementMostRecentTimestampDefinedAndMeasurementsAvailable_copiesMeasurementsAndProcessesThem() {
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerAndGyroscopeSyncedSensorMeasurement, AccelerometerAndGyroscopeSensorMeasurementSyncer>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(
            context,
            syncedMeasurementListener = syncedMeasurementListener
        )

        assertNull(syncer.mostRecentTimestamp)
        assertNull(syncer.oldestTimestamp)

        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "running", true)
        assertTrue(syncer.running)

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()
        val gyroscopeMeasurement =
            GyroscopeSensorMeasurement(wx, wy, wz, bx, by, bz, timestamp, SensorAccuracy.HIGH)
        val measurementsBeforeTimestamp = ArrayDeque<GyroscopeSensorMeasurement>()
        measurementsBeforeTimestamp.add(gyroscopeMeasurement)
        every { gyroscopeSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            measurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        // set most recent timestamp
        val mostRecentTimestamp = System.nanoTime()
        setPrivateProperty(
            SensorMeasurementSyncer::class,
            syncer,
            "mostRecentTimestamp",
            mostRecentTimestamp
        )

        val listener = gyroscopeSensorCollector.measurementListener
        requireNotNull(listener)

        listener.onMeasurement(gyroscopeSensorCollectorSpy, GyroscopeSensorMeasurement(), 0)

        verify(exactly = 1) {
            gyroscopeSensorCollectorSpy.getMeasurementsBeforeTimestamp(
                mostRecentTimestamp
            )
        }

        val availableGyroscopeMeasurements: LinkedList<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("availableGyroscopeMeasurements")
        requireNotNull(availableGyroscopeMeasurements)
        assertEquals(syncer.gyroscopeCapacity - 1, availableGyroscopeMeasurements.size)

        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
        val gyroscopeMeasurements: LinkedList<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("gyroscopeMeasurements")
        requireNotNull(gyroscopeMeasurements)
        assertEquals(1, gyroscopeMeasurements.size)
        assertNotSame(gyroscopeMeasurement, gyroscopeMeasurements[0])
        assertEquals(wx, gyroscopeMeasurements[0].wx, 0.0f)
        assertEquals(wy, gyroscopeMeasurements[0].wy, 0.0f)
        assertEquals(wz, gyroscopeMeasurements[0].wz, 0.0f)
        assertEquals(bx, gyroscopeMeasurements[0].bx)
        assertEquals(by, gyroscopeMeasurements[0].by)
        assertEquals(bz, gyroscopeMeasurements[0].bz)
        assertEquals(timestamp, gyroscopeMeasurements[0].timestamp)
        assertEquals(SensorAccuracy.HIGH, gyroscopeMeasurements[0].accuracy)

        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
        assertNull(syncer.oldestTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)

        val alreadyProcessedAccelerometerMeasurements: LinkedList<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())

        val alreadyProcessedGyroscopeMeasurements: LinkedList<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGyroscopeMeasurements")
        requireNotNull(alreadyProcessedGyroscopeMeasurements)
        assertTrue(alreadyProcessedGyroscopeMeasurements.isEmpty())

        verify { syncedMeasurementListener wasNot Called }
        assertTrue(syncer.running)
    }

    @Test
    fun gyroscopeSensorCollector_whenMeasurementMostRecentTimestampDefinedMeasurementsAvailableAndEmptyCache_copiesMeasurementsAndProcessesThem() {
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerAndGyroscopeSyncedSensorMeasurement, AccelerometerAndGyroscopeSensorMeasurementSyncer>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(
            context,
            syncedMeasurementListener = syncedMeasurementListener
        )

        assertNull(syncer.mostRecentTimestamp)
        assertNull(syncer.oldestTimestamp)

        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "running", true)
        assertTrue(syncer.running)

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = System.nanoTime()
        val gyroscopeMeasurement =
            GyroscopeSensorMeasurement(wx, wy, wz, bx, by, bz, timestamp, SensorAccuracy.HIGH)
        val measurementsBeforeTimestamp = ArrayDeque<GyroscopeSensorMeasurement>()
        measurementsBeforeTimestamp.add(gyroscopeMeasurement)
        every { gyroscopeSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            measurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        // set most recent timestamp
        val mostRecentTimestamp = System.nanoTime()
        setPrivateProperty(
            SensorMeasurementSyncer::class,
            syncer,
            "mostRecentTimestamp",
            mostRecentTimestamp
        )

        val availableGyroscopeMeasurements: LinkedList<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("availableGyroscopeMeasurements")
        requireNotNull(availableGyroscopeMeasurements)
        availableGyroscopeMeasurements.clear()

        val listener = gyroscopeSensorCollector.measurementListener
        requireNotNull(listener)

        listener.onMeasurement(gyroscopeSensorCollectorSpy, GyroscopeSensorMeasurement(), 0)

        verify(exactly = 1) {
            gyroscopeSensorCollectorSpy.getMeasurementsBeforeTimestamp(
                mostRecentTimestamp
            )
        }

        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
        val gyroscopeMeasurements: LinkedList<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("gyroscopeMeasurements")
        requireNotNull(gyroscopeMeasurements)
        assertEquals(1, gyroscopeMeasurements.size)
        assertNotSame(gyroscopeMeasurement, gyroscopeMeasurements[0])
        assertEquals(wx, gyroscopeMeasurements[0].wx, 0.0f)
        assertEquals(wy, gyroscopeMeasurements[0].wy, 0.0f)
        assertEquals(wz, gyroscopeMeasurements[0].wz, 0.0f)
        assertEquals(bx, gyroscopeMeasurements[0].bx)
        assertEquals(by, gyroscopeMeasurements[0].by)
        assertEquals(bz, gyroscopeMeasurements[0].bz)
        assertEquals(timestamp, gyroscopeMeasurements[0].timestamp)
        assertEquals(SensorAccuracy.HIGH, gyroscopeMeasurements[0].accuracy)

        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
        assertNull(syncer.oldestTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)

        val alreadyProcessedAccelerometerMeasurements: LinkedList<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())

        val alreadyProcessedGyroscopeMeasurements: LinkedList<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGyroscopeMeasurements")
        requireNotNull(alreadyProcessedGyroscopeMeasurements)
        assertTrue(alreadyProcessedGyroscopeMeasurements.isEmpty())

        verify { syncedMeasurementListener wasNot Called }
        assertTrue(syncer.running)
    }

    // TODO: fun sensorCollectors_whenAllMeasurementsAndOutOfOrder_notifiesOutOfOrder()
    // TODO: fun sensorCollectors_whenAllMeasurementsOutOfOrderAndStopWhenOutOfOrder_notifiesOutOfOrderAndStops()
    // TODO: fun sensorCollectors_whenAllMeasurements_notifiesSyncedMeasurement()

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