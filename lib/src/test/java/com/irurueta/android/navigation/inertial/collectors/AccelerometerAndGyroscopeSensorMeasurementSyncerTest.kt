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
import com.irurueta.android.navigation.inertial.collectors.interpolators.AccelerometerDirectSensorMeasurementInterpolator
import com.irurueta.android.navigation.inertial.collectors.interpolators.AccelerometerQuadraticSensorMeasurementInterpolator
import com.irurueta.android.navigation.inertial.collectors.interpolators.GyroscopeDirectSensorMeasurementInterpolator
import com.irurueta.android.navigation.inertial.collectors.interpolators.GyroscopeQuadraticSensorMeasurementInterpolator
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.android.testutils.setPrivateProperty
import com.irurueta.statistics.UniformRandomizer
//import io.mockk.*
//import io.mockk.impl.annotations.MockK
//import io.mockk.junit4.MockKRule
//import org.junit.After
import org.junit.Assert.*
//import org.junit.Ignore
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith
import org.mockito.ArgumentCaptor
import org.mockito.Captor
import org.mockito.Mock
import org.mockito.junit.MockitoJUnit
import org.mockito.junit.MockitoRule
import org.mockito.kotlin.any
import org.mockito.kotlin.capture
import org.mockito.kotlin.doAnswer
import org.mockito.kotlin.doReturn
import org.mockito.kotlin.eq
import org.mockito.kotlin.never
import org.mockito.kotlin.only
import org.mockito.kotlin.spy
import org.mockito.kotlin.times
import org.mockito.kotlin.verify
import org.mockito.kotlin.verifyNoInteractions
import org.mockito.kotlin.whenever
import org.robolectric.RobolectricTestRunner
import java.util.*

//@Ignore("Possible memory leak when running this test")
@RunWith(RobolectricTestRunner::class)
class AccelerometerAndGyroscopeSensorMeasurementSyncerTest {

    @get:Rule
    val mockitoRule: MockitoRule = MockitoJUnit.rule()

//    @get:Rule
//    val mockkRule = MockKRule(this)

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var accuracyChangedListener:
            SensorMeasurementSyncer.OnAccuracyChangedListener<AccelerometerAndGyroscopeSyncedSensorMeasurement, AccelerometerAndGyroscopeSensorMeasurementSyncer>

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var bufferFilledListener:
            SensorMeasurementSyncer.OnBufferFilledListener<AccelerometerAndGyroscopeSyncedSensorMeasurement, AccelerometerAndGyroscopeSensorMeasurementSyncer>

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var syncedMeasurementListener:
            SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerAndGyroscopeSyncedSensorMeasurement, AccelerometerAndGyroscopeSensorMeasurementSyncer>

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var staleDetectedMeasurementsListener:
            SensorMeasurementSyncer.OnStaleDetectedMeasurementsListener<AccelerometerAndGyroscopeSyncedSensorMeasurement, AccelerometerAndGyroscopeSensorMeasurementSyncer>

    @Captor
    private lateinit var accelerometerAndGyroscopeSyncedSensorMeasurementCaptor: ArgumentCaptor<AccelerometerAndGyroscopeSyncedSensorMeasurement>

    /*@After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
        System.gc()
    }*/

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
        assertEquals(
            AccelerometerAndGyroscopeSensorMeasurementSyncer.DEFAULT_ACCELEROMETER_CAPACITY,
            syncer.accelerometerCapacity
        )
        assertEquals(
            AccelerometerAndGyroscopeSensorMeasurementSyncer.DEFAULT_GYROSCOPE_CAPACITY,
            syncer.gyroscopeCapacity
        )
        assertFalse(syncer.accelerometerStartOffsetEnabled)
        assertFalse(syncer.gyroscopeStartOffsetEnabled)
        assertTrue(syncer.stopWhenFilledBuffer)
        assertEquals(
            AccelerometerAndGyroscopeSensorMeasurementSyncer.DEFAULT_STALE_OFFSET_NANOS,
            syncer.staleOffsetNanos
        )
        assertTrue(syncer.staleDetectionEnabled)
        assertTrue(syncer.skipWhenProcessing)
        assertNull(syncer.accuracyChangedListener)
        assertNull(syncer.bufferFilledListener)
        assertNull(syncer.syncedMeasurementListener)
        assertNull(syncer.staleDetectedMeasurementsListener)
        assertNotNull(syncer.accelerometerInterpolator)
        assertTrue(syncer.accelerometerInterpolator is AccelerometerQuadraticSensorMeasurementInterpolator)
        assertNotNull(syncer.gyroscopeInterpolator)
        assertTrue(syncer.gyroscopeInterpolator is GyroscopeQuadraticSensorMeasurementInterpolator)
        assertEquals(0L, syncer.startTimestamp)
        assertFalse(syncer.running)
        assertFalse(syncer.processing)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        assertNull(syncer.mostRecentTimestamp)
        assertNull(syncer.oldestTimestamp)
        assertNull(syncer.accelerometerSensor)
        assertNull(syncer.gyroscopeSensor)
        assertFalse(syncer.accelerometerSensorAvailable)
        assertFalse(syncer.gyroscopeSensorAvailable)
        assertNull(syncer.accelerometerStartOffset)
        assertNull(syncer.gyroscopeStartOffset)
        assertEquals(0.0f, syncer.accelerometerCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.gyroscopeCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.accelerometerUsage, 0.0f)
        assertEquals(0.0f, syncer.gyroscopeUsage, 0.0f)
    }

    @Test(expected = IllegalArgumentException::class)
    fun constructor_whenZeroAccelerometerCapacity_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        AccelerometerAndGyroscopeSensorMeasurementSyncer(context, accelerometerCapacity = 0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun constructor_whenZeroGyroscopeCapacity_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        AccelerometerAndGyroscopeSensorMeasurementSyncer(context, gyroscopeCapacity = 0)
    }

    @Test
    fun constructor_whenAllParameters_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val accelerometerInterpolator = AccelerometerDirectSensorMeasurementInterpolator()
        val gyroscopeInterpolator = GyroscopeDirectSensorMeasurementInterpolator()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            SensorDelay.GAME,
            accelerometerCapacity = 2,
            gyroscopeCapacity = 3,
            accelerometerStartOffsetEnabled = true,
            gyroscopeStartOffsetEnabled = true,
            stopWhenFilledBuffer = false,
            staleOffsetNanos = 123456789L,
            staleDetectionEnabled = false,
            skipWhenProcessing = false,
            accuracyChangedListener = accuracyChangedListener,
            bufferFilledListener = bufferFilledListener,
            syncedMeasurementListener = syncedMeasurementListener,
            staleDetectedMeasurementsListener = staleDetectedMeasurementsListener,
            accelerometerInterpolator = accelerometerInterpolator,
            gyroscopeInterpolator = gyroscopeInterpolator
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
        assertTrue(syncer.accelerometerStartOffsetEnabled)
        assertTrue(syncer.gyroscopeStartOffsetEnabled)
        assertFalse(syncer.stopWhenFilledBuffer)
        assertEquals(123456789L, syncer.staleOffsetNanos)
        assertFalse(syncer.staleDetectionEnabled)
        assertFalse(syncer.skipWhenProcessing)
        assertSame(accuracyChangedListener, syncer.accuracyChangedListener)
        assertSame(bufferFilledListener, syncer.bufferFilledListener)
        assertSame(syncedMeasurementListener, syncer.syncedMeasurementListener)
        assertSame(staleDetectedMeasurementsListener, syncer.staleDetectedMeasurementsListener)
        assertSame(accelerometerInterpolator, syncer.accelerometerInterpolator)
        assertSame(gyroscopeInterpolator, syncer.gyroscopeInterpolator)
        assertEquals(0L, syncer.startTimestamp)
        assertFalse(syncer.running)
        assertFalse(syncer.processing)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        assertNull(syncer.mostRecentTimestamp)
        assertNull(syncer.oldestTimestamp)
        assertNull(syncer.accelerometerSensor)
        assertNull(syncer.gyroscopeSensor)
        assertFalse(syncer.accelerometerSensorAvailable)
        assertFalse(syncer.gyroscopeSensorAvailable)
        assertNull(syncer.accelerometerStartOffset)
        assertNull(syncer.gyroscopeStartOffset)
        assertEquals(0.0f, syncer.accelerometerCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.gyroscopeCollectorUsage, 0.0f)
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
        syncer.bufferFilledListener = bufferFilledListener

        // check
        assertSame(bufferFilledListener, syncer.bufferFilledListener)
    }

    @Test
    fun syncedMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(context)

        // check default value
        assertNull(syncer.syncedMeasurementListener)

        // set new value
        syncer.syncedMeasurementListener = syncedMeasurementListener

        // check
        assertSame(syncedMeasurementListener, syncer.syncedMeasurementListener)
    }

    @Test
    fun staleDetectedMeasurementsListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(context)

        // check default value
        assertNull(syncer.staleDetectedMeasurementsListener)

        // set new value
        syncer.staleDetectedMeasurementsListener = staleDetectedMeasurementsListener

        // check
        assertSame(staleDetectedMeasurementsListener, syncer.staleDetectedMeasurementsListener)
    }

    @Test
    fun accelerometerSensor_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(context)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spy(accelerometerSensorCollector)
//        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        assertNull(syncer.accelerometerSensor)
        verify(accelerometerSensorCollectorSpy, only()).sensor
//        verify(exactly = 1) { accelerometerSensorCollectorSpy.sensor }
    }

    @Test
    fun gyroscopeSensor_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(context)

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spy(gyroscopeSensorCollector)
//        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        assertNull(syncer.gyroscopeSensor)
        verify(gyroscopeSensorCollectorSpy, only()).sensor
//        verify(exactly = 1) { gyroscopeSensorCollectorSpy.sensor }
    }

    @Test
    fun accelerometerSensorAvailable_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(context)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spy(accelerometerSensorCollector)
//        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        assertFalse(syncer.accelerometerSensorAvailable)
        verify(accelerometerSensorCollectorSpy, only()).sensorAvailable
//        verify(exactly = 1) { accelerometerSensorCollectorSpy.sensorAvailable }
    }

    @Test
    fun gyroscopeSensorAvailable_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(context)

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spy(gyroscopeSensorCollector)
//        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        assertFalse(syncer.gyroscopeSensorAvailable)
        verify(gyroscopeSensorCollectorSpy, only()).sensorAvailable
//        verify(exactly = 1) { gyroscopeSensorCollectorSpy.sensorAvailable }
    }

    @Test
    fun accelerometerStartOffset_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(context)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spy(accelerometerSensorCollector)
//        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        assertNull(syncer.accelerometerStartOffset)
        verify(accelerometerSensorCollectorSpy, only()).startOffset
//        verify(exactly = 1) { accelerometerSensorCollectorSpy.startOffset }
    }

    @Test
    fun gyroscopeStartOffset_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(context)

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spy(gyroscopeSensorCollector)
//        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        assertNull(syncer.gyroscopeStartOffset)
        verify(gyroscopeSensorCollectorSpy, only()).startOffset
//        verify(exactly = 1) { gyroscopeSensorCollectorSpy.startOffset }
    }

    @Test
    fun accelerometerCollectorUsage_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(context)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spy(accelerometerSensorCollector)
//        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        assertEquals(0.0f, syncer.accelerometerCollectorUsage, 0.0f)
        verify(accelerometerSensorCollectorSpy, only()).usage
//        verify(exactly = 1) { accelerometerSensorCollectorSpy.usage }
    }

    @Test
    fun gyroscopeCollectorUsage_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(context)

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spy(gyroscopeSensorCollector)
//        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        assertEquals(0.0f, syncer.gyroscopeCollectorUsage, 0.0f)
        verify(gyroscopeSensorCollectorSpy, only()).usage
//        verify(exactly = 1) { gyroscopeSensorCollectorSpy.usage }
    }

    @Test
    fun accelerometerUsage_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(context)

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
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(context)

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

    @Test(expected = IllegalStateException::class)
    fun start_whenAlreadyRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(context)

        // set as running
        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "running", true)
        assertTrue(syncer.running)
        assertFalse(syncer.processing)

        syncer.start()
    }

    @Test
    fun start_whenNotRunningAndNoTimestamp_clearsResetsAndSetsCurrentStartTimestamp() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(context)

        assertFalse(syncer.running)
        assertFalse(syncer.processing)
        assertEquals(0L, syncer.startTimestamp)

        // set variables that will be later reset
        val accelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("accelerometerMeasurements")
        requireNotNull(accelerometerMeasurements)
        accelerometerMeasurements.add(AccelerometerSensorMeasurement())

        val gyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("gyroscopeMeasurements")
        requireNotNull(gyroscopeMeasurements)
        gyroscopeMeasurements.add(GyroscopeSensorMeasurement())

        val alreadyProcessedAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        alreadyProcessedAccelerometerMeasurements.add(AccelerometerSensorMeasurement())

        val alreadyProcessedGyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGyroscopeMeasurements")
        requireNotNull(alreadyProcessedGyroscopeMeasurements)
        alreadyProcessedGyroscopeMeasurements.add(GyroscopeSensorMeasurement())

        val foundGyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("foundGyroscopeMeasurements")
        requireNotNull(foundGyroscopeMeasurements)
        foundGyroscopeMeasurements.add(GyroscopeSensorMeasurement())

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

        syncer.setPrivateProperty("hasPreviousAccelerometerMeasurement", true)
        syncer.setPrivateProperty("hasPreviousGyroscopeMeasurement", true)
        syncer.setPrivateProperty("lastNotifiedTimestamp", 1L)
        syncer.setPrivateProperty("lastNotifiedAccelerometerTimestamp", 2L)
        syncer.setPrivateProperty("lastNotifiedGyroscopeTimestamp", 3L)

        assertFalse(syncer.running)
        assertFalse(syncer.processing)
        assertEquals(0L, syncer.startTimestamp)

        assertFalse(syncer.start())

        // check
        assertFalse(syncer.running)
        assertFalse(syncer.processing)
        assertNotEquals(0L, syncer.startTimestamp)

        assertTrue(accelerometerMeasurements.isEmpty())
        assertTrue(gyroscopeMeasurements.isEmpty())
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())
        assertTrue(alreadyProcessedGyroscopeMeasurements.isEmpty())
        assertTrue(foundGyroscopeMeasurements.isEmpty())

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
        assertFalse(syncer.processing)

        val hasPreviousAccelerometerMeasurement: Boolean? =
            syncer.getPrivateProperty("hasPreviousAccelerometerMeasurement")
        requireNotNull(hasPreviousAccelerometerMeasurement)
        assertFalse(hasPreviousAccelerometerMeasurement)

        val hasPreviousGyroscopeMeasurement: Boolean? =
            syncer.getPrivateProperty("hasPreviousGyroscopeMeasurement")
        requireNotNull(hasPreviousGyroscopeMeasurement)
        assertFalse(hasPreviousGyroscopeMeasurement)

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
    }

    @Test
    fun start_whenNotRunningAndTimestampProvided_setsProvidedTimestamp() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(context)

        assertFalse(syncer.running)
        assertFalse(syncer.processing)
        assertEquals(0L, syncer.startTimestamp)

        val timestamp = System.nanoTime()
        assertFalse(syncer.start(timestamp))

        // check
        assertFalse(syncer.running)
        assertFalse(syncer.processing)
        assertEquals(timestamp, syncer.startTimestamp)
    }

    @Test
    fun start_whenAccelerometerCollectorFails_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(context)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spy(accelerometerSensorCollector)
//        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        doReturn(false).whenever(accelerometerSensorCollectorSpy).start(any())
//        every { accelerometerSensorCollectorSpy.start(any()) }.returns(false)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)
        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spy(gyroscopeSensorCollector)
//        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        doReturn(true).whenever(gyroscopeSensorCollectorSpy).start(any())
//        every { gyroscopeSensorCollectorSpy.start(any()) }.returns(true)
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        assertFalse(syncer.running)
        assertFalse(syncer.processing)

        val timestamp = System.nanoTime()
        assertFalse(syncer.start(timestamp))

        assertFalse(syncer.running)
        assertFalse(syncer.processing)
        assertEquals(timestamp, syncer.startTimestamp)

        verify(accelerometerSensorCollectorSpy, times(1)).start(timestamp)
        verify(gyroscopeSensorCollectorSpy, never()).start(any())
//        verify(exactly = 1) { accelerometerSensorCollectorSpy.start(timestamp) }
//        verify(exactly = 0) { gyroscopeSensorCollectorSpy.start(any()) }
    }

    @Test
    fun start_whenGyroscopeCollectorFails_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(context)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spy(accelerometerSensorCollector)
        doReturn(true).whenever(accelerometerSensorCollectorSpy).start(any())
//        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
//        every { accelerometerSensorCollectorSpy.start(any()) }.returns(true)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)
        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spy(gyroscopeSensorCollector)
        doReturn(false).whenever(gyroscopeSensorCollectorSpy).start(any())
//        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
//        every { gyroscopeSensorCollectorSpy.start(any()) }.returns(false)
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        assertFalse(syncer.running)
        assertFalse(syncer.processing)

        val timestamp = System.nanoTime()
        assertFalse(syncer.start(timestamp))

        assertFalse(syncer.running)
        assertFalse(syncer.processing)
        assertEquals(timestamp, syncer.startTimestamp)

        verify(accelerometerSensorCollectorSpy, times(1)).start(timestamp)
        verify(gyroscopeSensorCollectorSpy, times(1)).start(timestamp)
//        verify(exactly = 1) { accelerometerSensorCollectorSpy.start(timestamp) }
//        verify(exactly = 1) { gyroscopeSensorCollectorSpy.start(timestamp) }
    }

    @Test
    fun start_whenCollectorsSucceed_returnsTrue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(context)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spy(accelerometerSensorCollector)
        doReturn(true).whenever(accelerometerSensorCollectorSpy).start(any())
//        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
//        every { accelerometerSensorCollectorSpy.start(any()) }.returns(true)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)
        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spy(gyroscopeSensorCollector)
        doReturn(true).whenever(gyroscopeSensorCollectorSpy).start(any())
//        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
//        every { gyroscopeSensorCollectorSpy.start(any()) }.returns(true)
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        assertFalse(syncer.running)
        assertFalse(syncer.processing)

        val timestamp = System.nanoTime()
        assertTrue(syncer.start(timestamp))

        assertTrue(syncer.running)
        assertFalse(syncer.processing)
        assertEquals(timestamp, syncer.startTimestamp)

        verify(accelerometerSensorCollectorSpy, only()).start(timestamp)
        verify(gyroscopeSensorCollectorSpy, only()).start(timestamp)
//        verify(exactly = 1) { accelerometerSensorCollectorSpy.start(timestamp) }
//        verify(exactly = 1) { gyroscopeSensorCollectorSpy.start(timestamp) }
    }

    @Test
    fun stop_stopsCollectorsInitializesCachesAndResetsProperties() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(context)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spy(accelerometerSensorCollector)
//        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)
        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spy(gyroscopeSensorCollector)
//        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        val accelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("accelerometerMeasurements")
        requireNotNull(accelerometerMeasurements)
        accelerometerMeasurements.add(AccelerometerSensorMeasurement())

        val gyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("gyroscopeMeasurements")
        requireNotNull(gyroscopeMeasurements)
        gyroscopeMeasurements.add(GyroscopeSensorMeasurement())

        val alreadyProcessedAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        alreadyProcessedAccelerometerMeasurements.add(AccelerometerSensorMeasurement())

        val alreadyProcessedGyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGyroscopeMeasurements")
        requireNotNull(alreadyProcessedGyroscopeMeasurements)
        alreadyProcessedGyroscopeMeasurements.add(GyroscopeSensorMeasurement())

        val foundGyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("foundGyroscopeMeasurements")
        requireNotNull(foundGyroscopeMeasurements)
        foundGyroscopeMeasurements.add(GyroscopeSensorMeasurement())

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
        syncer.setPrivateProperty("lastNotifiedTimestamp", 1L)
        syncer.setPrivateProperty("lastNotifiedAccelerometerTimestamp", 2L)
        syncer.setPrivateProperty("lastNotifiedGyroscopeTimestamp", 3L)

        val stopping1: Boolean? =
            getPrivateProperty(SensorMeasurementSyncer::class, syncer, "stopping")
        requireNotNull(stopping1)
        assertFalse(stopping1)

        syncer.stop()

        // check
        verify(accelerometerSensorCollectorSpy, times(1)).stop()
//        verify(exactly = 1) { accelerometerSensorCollectorSpy.stop() }
        verify(gyroscopeSensorCollectorSpy, times(1)).stop()
//        verify(exactly = 1) { gyroscopeSensorCollectorSpy.stop() }
        assertEquals(1, accelerometerMeasurements.size)
        assertEquals(1, gyroscopeMeasurements.size)
        assertEquals(1, alreadyProcessedAccelerometerMeasurements.size)
        assertEquals(1, alreadyProcessedGyroscopeMeasurements.size)
        assertEquals(1, foundGyroscopeMeasurements.size)

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
        assertFalse(syncer.processing)

        val hasPreviousAccelerometerMeasurement: Boolean? =
            syncer.getPrivateProperty("hasPreviousAccelerometerMeasurement")
        requireNotNull(hasPreviousAccelerometerMeasurement)
        assertFalse(hasPreviousAccelerometerMeasurement)

        val hasPreviousGyroscopeMeasurement: Boolean? =
            syncer.getPrivateProperty("hasPreviousGyroscopeMeasurement")
        requireNotNull(hasPreviousGyroscopeMeasurement)
        assertFalse(hasPreviousGyroscopeMeasurement)

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

        val stopping2: Boolean? =
            getPrivateProperty(SensorMeasurementSyncer::class, syncer, "stopping")
        requireNotNull(stopping2)
        assertTrue(stopping2)
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

        verify(accuracyChangedListener, only()).onAccuracyChanged(
            syncer,
            SensorType.ACCELEROMETER_UNCALIBRATED,
            SensorAccuracy.HIGH
        )
/*        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                syncer,
                SensorType.ACCELEROMETER_UNCALIBRATED,
                SensorAccuracy.HIGH
            )
        }*/
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

        verify(bufferFilledListener, only()).onBufferFilled(
            syncer,
            SensorType.ACCELEROMETER_UNCALIBRATED
        )
/*        verify(exactly = 1) {
            bufferFilledListener.onBufferFilled(
                syncer,
                SensorType.ACCELEROMETER_UNCALIBRATED
            )
        }*/
        assertTrue(syncer.running)
    }

    @Test
    fun accelerometerSensorCollector_whenSkipWhenProcessingAndProcessing_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(
            context,
            skipWhenProcessing = true,
            syncedMeasurementListener = syncedMeasurementListener
        )

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spy(accelerometerSensorCollector)
        doReturn(ArrayDeque<AccelerometerSensorMeasurement>()).whenever(accelerometerSensorCollectorSpy).getMeasurementsBeforePosition(any())
//        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
/*        every { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(any()) }.returns(
            ArrayDeque()
        )*/
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)
        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "processing", true)

        val listener = accelerometerSensorCollectorSpy.measurementListener
        requireNotNull(listener)

        listener.onMeasurement(accelerometerSensorCollectorSpy, AccelerometerSensorMeasurement(), 0)

        verify(accelerometerSensorCollectorSpy, never()).getMeasurementsBeforePosition(0)
        verifyNoInteractions(syncedMeasurementListener)
//        verify(exactly = 0) { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(0) }
//        verify { syncedMeasurementListener wasNot Called }
        assertNull(syncer.mostRecentTimestamp)
    }

    @Test
    fun accelerometerSensorCollector_whenMeasurementAndNoMeasurementsAvailable_callsCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(
            context,
            syncedMeasurementListener = syncedMeasurementListener
        )

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spy(accelerometerSensorCollector)
        doReturn(ArrayDeque<AccelerometerSensorMeasurement>()).whenever(accelerometerSensorCollectorSpy).getMeasurementsBeforePosition(any())
//        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
/*        every { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(any()) }.returns(
            ArrayDeque<AccelerometerSensorMeasurement>()
        )*/
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        val listener = accelerometerSensorCollectorSpy.measurementListener
        requireNotNull(listener)

        listener.onMeasurement(accelerometerSensorCollectorSpy, AccelerometerSensorMeasurement(), 0)

        verify(accelerometerSensorCollectorSpy, times(1)).getMeasurementsBeforePosition(0)
//        verify(exactly = 1) { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(0) }
        verifyNoInteractions(syncedMeasurementListener)
//        verify { syncedMeasurementListener wasNot Called }
        assertNull(syncer.mostRecentTimestamp)
    }

    @Test
    fun accelerometerSensorCollector_whenMeasurementAndMeasurementsAvailable_updatesMostRecentTimestampCopiesMeasurementsAndProcessesThem() {
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
        val accelerometerSensorCollectorSpy = spy(accelerometerSensorCollector)
//        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)

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
        doReturn(measurementsBeforePosition).whenever(accelerometerSensorCollectorSpy).getMeasurementsBeforePosition(any())
/*        every { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(any()) }.returns(
            measurementsBeforePosition
        )*/
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        val listener = accelerometerSensorCollectorSpy.measurementListener
        requireNotNull(listener)

        listener.onMeasurement(accelerometerSensorCollectorSpy, AccelerometerSensorMeasurement(), 0)

        verify(accelerometerSensorCollectorSpy, times(1)).getMeasurementsBeforePosition(0)
//        verify(exactly = 1) { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(0) }
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

        val alreadyProcessedGyroscopeMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGyroscopeMeasurements")
        requireNotNull(alreadyProcessedGyroscopeMeasurements)
        assertTrue(alreadyProcessedGyroscopeMeasurements.isEmpty())

        verifyNoInteractions(syncedMeasurementListener)
//        verify { syncedMeasurementListener wasNot Called }
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

        verify(accuracyChangedListener, only()).onAccuracyChanged(
            syncer,
            SensorType.GYROSCOPE_UNCALIBRATED,
            SensorAccuracy.HIGH
        )
/*        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                syncer,
                SensorType.GYROSCOPE_UNCALIBRATED,
                SensorAccuracy.HIGH
            )
        }*/
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

        verify(bufferFilledListener, only()).onBufferFilled(
            syncer,
            SensorType.GYROSCOPE_UNCALIBRATED
        )
/*        verify(exactly = 1) {
            bufferFilledListener.onBufferFilled(
                syncer,
                SensorType.GYROSCOPE_UNCALIBRATED
            )
        }*/
        assertTrue(syncer.running)
    }

    @Test
    fun gyroscopeSensorCollector_whenSkipWhenProcessingAndProcessing_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(
            context,
            skipWhenProcessing = true,
            syncedMeasurementListener = syncedMeasurementListener
        )

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spy(gyroscopeSensorCollector)
//        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)
        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "processing", true)

        val listener = gyroscopeSensorCollector.measurementListener
        requireNotNull(listener)

        listener.onMeasurement(gyroscopeSensorCollectorSpy, GyroscopeSensorMeasurement(), 0)

        verifyNoInteractions(gyroscopeSensorCollectorSpy)
        verifyNoInteractions(syncedMeasurementListener)
//        verify { gyroscopeSensorCollectorSpy wasNot Called }
//        verify { syncedMeasurementListener wasNot Called }
        assertNull(syncer.mostRecentTimestamp)
    }

    @Test
    fun gyroscopeSensorCollector_whenMeasurementAndMostRecentTimestampNotDefined_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(
            context,
            syncedMeasurementListener = syncedMeasurementListener
        )

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spy(gyroscopeSensorCollector)
//        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        val listener = gyroscopeSensorCollector.measurementListener
        requireNotNull(listener)

        listener.onMeasurement(gyroscopeSensorCollectorSpy, GyroscopeSensorMeasurement(), 0)

        verifyNoInteractions(gyroscopeSensorCollectorSpy)
        verifyNoInteractions(syncedMeasurementListener)
//        verify { gyroscopeSensorCollectorSpy wasNot Called }
//        verify { syncedMeasurementListener wasNot Called }
        assertNull(syncer.mostRecentTimestamp)
    }

    @Test
    fun gyroscopeSensorCollector_whenMeasurementMostRecentTimestampDefinedAndNoMeasurementsAvailable_callsCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(
            context,
            syncedMeasurementListener = syncedMeasurementListener
        )

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spy(gyroscopeSensorCollector)
        doReturn(ArrayDeque<GyroscopeSensorMeasurement>()).whenever(gyroscopeSensorCollectorSpy).getMeasurementsBeforeTimestamp(any())
//        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
/*        every { gyroscopeSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            ArrayDeque()
        )*/
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

        verify(gyroscopeSensorCollectorSpy, only()).getMeasurementsBeforeTimestamp(
            mostRecentTimestamp
        )
        verifyNoInteractions(syncedMeasurementListener)
/*        verify(exactly = 1) {
            gyroscopeSensorCollectorSpy.getMeasurementsBeforeTimestamp(
                mostRecentTimestamp
            )
        }*/
//        verify { syncedMeasurementListener wasNot Called }
        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
    }

    @Test
    fun gyroscopeSensorCollector_whenMeasurementMostRecentTimestampDefinedAndMeasurementsAvailable_copiesMeasurements() {
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
        val gyroscopeSensorCollectorSpy = spy(gyroscopeSensorCollector)
//        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)

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
        doReturn(measurementsBeforeTimestamp).whenever(gyroscopeSensorCollectorSpy).getMeasurementsBeforeTimestamp(any())
/*        every { gyroscopeSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            measurementsBeforeTimestamp
        )*/
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

        verify(gyroscopeSensorCollectorSpy, only()).getMeasurementsBeforeTimestamp(
            mostRecentTimestamp
        )
/*        verify(exactly = 1) {
            gyroscopeSensorCollectorSpy.getMeasurementsBeforeTimestamp(
                mostRecentTimestamp
            )
        }*/

        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
        val gyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("gyroscopeMeasurements")
        requireNotNull(gyroscopeMeasurements)
        assertEquals(1, gyroscopeMeasurements.size)
        val gyroscopeMeasurement2 = gyroscopeMeasurements.peek()
        requireNotNull(gyroscopeMeasurement2)
        assertNotSame(gyroscopeMeasurement, gyroscopeMeasurement2)
        assertEquals(wx, gyroscopeMeasurement2.wx, 0.0f)
        assertEquals(wy, gyroscopeMeasurement2.wy, 0.0f)
        assertEquals(wz, gyroscopeMeasurement2.wz, 0.0f)
        assertEquals(bx, gyroscopeMeasurement2.bx)
        assertEquals(by, gyroscopeMeasurement2.by)
        assertEquals(bz, gyroscopeMeasurement2.bz)
        assertEquals(timestamp, gyroscopeMeasurement2.timestamp)
        assertEquals(SensorAccuracy.HIGH, gyroscopeMeasurement2.accuracy)
        assertEquals(syncer.gyroscopeSensorType, gyroscopeMeasurement2.sensorType)

        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
        assertNull(syncer.oldestTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)

        val alreadyProcessedAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())

        val alreadyProcessedGyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGyroscopeMeasurements")
        requireNotNull(alreadyProcessedGyroscopeMeasurements)
        assertTrue(alreadyProcessedGyroscopeMeasurements.isEmpty())

        verifyNoInteractions(syncedMeasurementListener)
//        verify { syncedMeasurementListener wasNot Called }
        assertTrue(syncer.running)
    }

    @Test
    fun sensorCollectors_whenAllMeasurements_notifiesSyncedMeasurement() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(
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

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spy(gyroscopeSensorCollector)
//        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val gyroscopeTimestamp = accelerometerTimestamp - 1
        val gyroscopeMeasurement1 =
            GyroscopeSensorMeasurement(
                wx,
                wy,
                wz,
                wbx,
                wby,
                wbz,
                gyroscopeTimestamp,
                SensorAccuracy.HIGH,
                syncer.gyroscopeSensorType
            )
        val measurementsBeforeTimestamp = ArrayDeque<GyroscopeSensorMeasurement>()
        measurementsBeforeTimestamp.add(gyroscopeMeasurement1)
        doReturn(measurementsBeforeTimestamp).whenever(gyroscopeSensorCollectorSpy).getMeasurementsBeforeTimestamp(any())
/*        every { gyroscopeSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            measurementsBeforeTimestamp
        )*/
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spy(accelerometerSensorCollector)
//        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)

        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val accelerometerMeasurement1 =
            AccelerometerSensorMeasurement(
                ax,
                ay,
                az,
                abx,
                aby,
                abz,
                accelerometerTimestamp,
                SensorAccuracy.HIGH,
                syncer.accelerometerSensorType
            )
        val measurementsBeforePosition = ArrayDeque<AccelerometerSensorMeasurement>()
        measurementsBeforePosition.add(accelerometerMeasurement1)
        doReturn(measurementsBeforePosition).whenever(accelerometerSensorCollectorSpy).getMeasurementsBeforePosition(any())
/*        every { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(any()) }.returns(
            measurementsBeforePosition
        )*/
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        val accelerometerListener = accelerometerSensorCollectorSpy.measurementListener
        requireNotNull(accelerometerListener)

        val gyroscopeListener = gyroscopeSensorCollector.measurementListener
        requireNotNull(gyroscopeListener)

        // process gyroscope measurement
        gyroscopeListener.onMeasurement(
            gyroscopeSensorCollectorSpy,
            GyroscopeSensorMeasurement(),
            0
        )

        assertNull(syncer.oldestTimestamp)
        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        assertTrue(syncer.running)

        val gyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("gyroscopeMeasurements")
        requireNotNull(gyroscopeMeasurements)
        assertEquals(1, gyroscopeMeasurements.size)
        val gyroscopeMeasurement2 = gyroscopeMeasurements.peek()
        requireNotNull(gyroscopeMeasurement2)
        assertNotSame(gyroscopeMeasurement1, gyroscopeMeasurement2)
        assertEquals(wx, gyroscopeMeasurement2.wx, 0.0f)
        assertEquals(wy, gyroscopeMeasurement2.wy, 0.0f)
        assertEquals(wz, gyroscopeMeasurement2.wz, 0.0f)
        assertEquals(wbx, gyroscopeMeasurement2.bx)
        assertEquals(wby, gyroscopeMeasurement2.by)
        assertEquals(wbz, gyroscopeMeasurement2.bz)
        assertEquals(gyroscopeTimestamp, gyroscopeMeasurement2.timestamp)
        assertEquals(SensorAccuracy.HIGH, gyroscopeMeasurement2.accuracy)
        assertEquals(syncer.gyroscopeSensorType, gyroscopeMeasurement2.sensorType)

        val alreadyProcessedGyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGyroscopeMeasurements")
        requireNotNull(alreadyProcessedGyroscopeMeasurements)
        assertTrue(alreadyProcessedGyroscopeMeasurements.isEmpty())

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
        verify(accelerometerSensorCollectorSpy, times(1)).getMeasurementsBeforePosition(0)
//        verify(exactly = 1) { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(0) }

        val accelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("accelerometerMeasurements")
        requireNotNull(accelerometerMeasurements)
        assertTrue(accelerometerMeasurements.isEmpty())
        assertTrue(gyroscopeMeasurements.isEmpty())

        val alreadyProcessedAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())
        assertTrue(alreadyProcessedGyroscopeMeasurements.isEmpty())

//        val captor = ArgumentCaptor.captor<AccelerometerAndGyroscopeSyncedSensorMeasurement>()
//        val slot = slot<AccelerometerAndGyroscopeSyncedSensorMeasurement>()
        verify(syncedMeasurementListener, only()).onSyncedMeasurements(
            eq(syncer),
            capture(accelerometerAndGyroscopeSyncedSensorMeasurementCaptor)
        )
/*        verify(exactly = 1) {
            syncedMeasurementListener.onSyncedMeasurements(
                syncer,
                capture(slot)
            )
        }*/

        val syncedMeasurement = accelerometerAndGyroscopeSyncedSensorMeasurementCaptor.value
//        val syncedMeasurement = slot.captured
        assertEquals(gyroscopeTimestamp, syncedMeasurement.timestamp)
        val syncedAccelerometerMeasurement = syncedMeasurement.accelerometerMeasurement
        requireNotNull(syncedAccelerometerMeasurement)
        assertEquals(ax, syncedAccelerometerMeasurement.ax, 0.0f)
        assertEquals(ay, syncedAccelerometerMeasurement.ay, 0.0f)
        assertEquals(az, syncedAccelerometerMeasurement.az, 0.0f)
        assertEquals(abx, syncedAccelerometerMeasurement.bx)
        assertEquals(aby, syncedAccelerometerMeasurement.by)
        assertEquals(abz, syncedAccelerometerMeasurement.bz)
        assertEquals(gyroscopeTimestamp, syncedAccelerometerMeasurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, syncedAccelerometerMeasurement.accuracy)
        assertEquals(syncer.accelerometerSensorType, syncedAccelerometerMeasurement.sensorType)
        val syncedGyroscopeMeasurement = syncedMeasurement.gyroscopeMeasurement
        requireNotNull(syncedGyroscopeMeasurement)
        assertEquals(wx, syncedGyroscopeMeasurement.wx, 0.0f)
        assertEquals(wy, syncedGyroscopeMeasurement.wy, 0.0f)
        assertEquals(wz, syncedGyroscopeMeasurement.wz, 0.0f)
        assertEquals(wbx, syncedGyroscopeMeasurement.bx)
        assertEquals(wby, syncedGyroscopeMeasurement.by)
        assertEquals(wbz, syncedGyroscopeMeasurement.bz)
        assertEquals(gyroscopeTimestamp, syncedGyroscopeMeasurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, syncedGyroscopeMeasurement.accuracy)
        assertEquals(syncer.gyroscopeSensorType, syncedGyroscopeMeasurement.sensorType)
    }

    @Test
    fun cleanupStaleMeasurements_whenStaleMeasurementsAvailable_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(
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
            mostRecentTimestamp - 2 * AccelerometerAndGyroscopeSensorMeasurementSyncer.DEFAULT_STALE_OFFSET_NANOS
        val gyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("gyroscopeMeasurements")
        requireNotNull(gyroscopeMeasurements)
        val randomizer = UniformRandomizer()
        val wx1 = randomizer.nextFloat()
        val wy1 = randomizer.nextFloat()
        val wz1 = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val staleGyroscopeMeasurement =
            GyroscopeSensorMeasurement(
                wx1,
                wy1,
                wz1,
                wbx,
                wby,
                wbz,
                staleTimestamp,
                SensorAccuracy.HIGH,
                syncer.gyroscopeSensorType
            )
        gyroscopeMeasurements.add(staleGyroscopeMeasurement)

        val accelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("accelerometerMeasurements")
        requireNotNull(accelerometerMeasurements)
        val ax1 = randomizer.nextFloat()
        val ay1 = randomizer.nextFloat()
        val az1 = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val staleAccelerometerMeasurement =
            AccelerometerSensorMeasurement(
                ax1,
                ay1,
                az1,
                abx,
                aby,
                abz,
                staleTimestamp,
                SensorAccuracy.HIGH,
                syncer.accelerometerSensorType
            )
        accelerometerMeasurements.add(staleAccelerometerMeasurement)

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spy(gyroscopeSensorCollector)
//        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)

        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val gyroscopeTimestamp = accelerometerTimestamp - 1
        val gyroscopeMeasurement1 =
            GyroscopeSensorMeasurement(
                wx,
                wy,
                wz,
                wbx,
                wby,
                wbz,
                gyroscopeTimestamp,
                SensorAccuracy.HIGH,
                syncer.gyroscopeSensorType
            )
        val measurementsBeforeTimestamp = ArrayDeque<GyroscopeSensorMeasurement>()
        measurementsBeforeTimestamp.add(gyroscopeMeasurement1)
        doReturn(measurementsBeforeTimestamp).whenever(gyroscopeSensorCollectorSpy).getMeasurementsBeforeTimestamp(any())
/*        every { gyroscopeSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            measurementsBeforeTimestamp
        )*/
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spy(accelerometerSensorCollector)
//        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)

        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val accelerometerMeasurement1 =
            AccelerometerSensorMeasurement(
                ax,
                ay,
                az,
                abx,
                aby,
                abz,
                accelerometerTimestamp,
                SensorAccuracy.HIGH,
                syncer.accelerometerSensorType
            )
        val measurementsBeforePosition = ArrayDeque<AccelerometerSensorMeasurement>()
        measurementsBeforePosition.add(accelerometerMeasurement1)
        doReturn(measurementsBeforePosition).whenever(accelerometerSensorCollectorSpy).getMeasurementsBeforePosition(any())
/*        every { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(any()) }.returns(
            measurementsBeforePosition
        )*/
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        // set previous measurements
        syncer.setPrivateProperty("hasPreviousAccelerometerMeasurement", true)
        val previousAccelerometerMeasurement: AccelerometerSensorMeasurement? =
            syncer.getPrivateProperty("previousAccelerometerMeasurement")
        requireNotNull(previousAccelerometerMeasurement)
        previousAccelerometerMeasurement.timestamp = accelerometerTimestamp - 1

        val accelerometerListener = accelerometerSensorCollectorSpy.measurementListener
        requireNotNull(accelerometerListener)

        val gyroscopeListener = gyroscopeSensorCollector.measurementListener
        requireNotNull(gyroscopeListener)

        // process gyroscope measurement
        gyroscopeListener.onMeasurement(
            gyroscopeSensorCollectorSpy,
            GyroscopeSensorMeasurement(),
            0
        )

        assertNull(syncer.oldestTimestamp)
        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        assertTrue(syncer.running)

        assertEquals(2, gyroscopeMeasurements.size)
        val gyroscopeMeasurement2 = gyroscopeMeasurements.first
        requireNotNull(gyroscopeMeasurement2)
        assertEquals(wx1, gyroscopeMeasurement2.wx, 0.0f)
        assertEquals(wy1, gyroscopeMeasurement2.wy, 0.0f)
        assertEquals(wz1, gyroscopeMeasurement2.wz, 0.0f)
        assertEquals(wbx, gyroscopeMeasurement2.bx)
        assertEquals(wby, gyroscopeMeasurement2.by)
        assertEquals(wbz, gyroscopeMeasurement2.bz)
        assertEquals(staleTimestamp, gyroscopeMeasurement2.timestamp)
        assertEquals(SensorAccuracy.HIGH, gyroscopeMeasurement2.accuracy)
        assertEquals(syncer.gyroscopeSensorType, gyroscopeMeasurement2.sensorType)
        val gyroscopeMeasurement3 = gyroscopeMeasurements.last
        requireNotNull(gyroscopeMeasurement3)
        assertEquals(wx, gyroscopeMeasurement3.wx, 0.0f)
        assertEquals(wy, gyroscopeMeasurement3.wy, 0.0f)
        assertEquals(wz, gyroscopeMeasurement3.wz, 0.0f)
        assertEquals(wbx, gyroscopeMeasurement3.bx)
        assertEquals(wby, gyroscopeMeasurement3.by)
        assertEquals(wbz, gyroscopeMeasurement3.bz)
        assertEquals(gyroscopeTimestamp, gyroscopeMeasurement3.timestamp)
        assertEquals(SensorAccuracy.HIGH, gyroscopeMeasurement3.accuracy)
        assertEquals(syncer.gyroscopeSensorType, gyroscopeMeasurement2.sensorType)

        val alreadyProcessedGyroscopeMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGyroscopeMeasurements")
        requireNotNull(alreadyProcessedGyroscopeMeasurements)
        assertTrue(alreadyProcessedGyroscopeMeasurements.isEmpty())

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
        verify(accelerometerSensorCollectorSpy, times(1)).getMeasurementsBeforePosition(0)
//        verify(exactly = 1) { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(0) }

        assertTrue(accelerometerMeasurements.isEmpty())
        assertTrue(gyroscopeMeasurements.isEmpty())

        val alreadyProcessedAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())
        assertTrue(alreadyProcessedGyroscopeMeasurements.isEmpty())

        verify(staleDetectedMeasurementsListener, only()).onStaleMeasurements(
            eq(syncer),
            eq(SensorType.ACCELEROMETER_UNCALIBRATED),
            any()
        )
/*        verify(exactly = 1) {
            staleDetectedMeasurementsListener.onStaleMeasurements(
                syncer,
                SensorType.ACCELEROMETER_UNCALIBRATED,
                any()
            )
        }*/
    }

    @Test
    fun cleanupStaleMeasurements_whenStaleDetectionDisabled_doesNotNotify() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(
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
        val gyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("gyroscopeMeasurements")
        requireNotNull(gyroscopeMeasurements)
        val randomizer = UniformRandomizer()
        val wx1 = randomizer.nextFloat()
        val wy1 = randomizer.nextFloat()
        val wz1 = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val staleGyroscopeMeasurement =
            GyroscopeSensorMeasurement(
                wx1,
                wy1,
                wz1,
                wbx,
                wby,
                wbz,
                staleTimestamp,
                SensorAccuracy.HIGH,
                syncer.gyroscopeSensorType
            )
        gyroscopeMeasurements.add(staleGyroscopeMeasurement)

        val accelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("accelerometerMeasurements")
        requireNotNull(accelerometerMeasurements)
        val ax1 = randomizer.nextFloat()
        val ay1 = randomizer.nextFloat()
        val az1 = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val staleAccelerometerMeasurement =
            AccelerometerSensorMeasurement(
                ax1,
                ay1,
                az1,
                abx,
                aby,
                abz,
                staleTimestamp,
                SensorAccuracy.HIGH,
                syncer.accelerometerSensorType
            )
        accelerometerMeasurements.add(staleAccelerometerMeasurement)

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spy(gyroscopeSensorCollector)
//        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)

        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val gyroscopeTimestamp = accelerometerTimestamp - 1
        val gyroscopeMeasurement1 =
            GyroscopeSensorMeasurement(
                wx,
                wy,
                wz,
                wbx,
                wby,
                wbz,
                gyroscopeTimestamp,
                SensorAccuracy.HIGH,
                syncer.gyroscopeSensorType
            )
        val measurementsBeforeTimestamp = ArrayDeque<GyroscopeSensorMeasurement>()
        measurementsBeforeTimestamp.add(gyroscopeMeasurement1)
        doReturn(measurementsBeforeTimestamp).whenever(gyroscopeSensorCollectorSpy).getMeasurementsBeforeTimestamp(any())
/*        every { gyroscopeSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            measurementsBeforeTimestamp
        )*/
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spy(accelerometerSensorCollector)
//        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)

        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val accelerometerMeasurement1 =
            AccelerometerSensorMeasurement(
                ax,
                ay,
                az,
                abx,
                aby,
                abz,
                accelerometerTimestamp,
                SensorAccuracy.HIGH,
                syncer.accelerometerSensorType
            )
        val measurementsBeforePosition = ArrayDeque<AccelerometerSensorMeasurement>()
        measurementsBeforePosition.add(accelerometerMeasurement1)
        doReturn(measurementsBeforePosition).whenever(accelerometerSensorCollectorSpy).getMeasurementsBeforePosition(any())
/*        every { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(any()) }.returns(
            measurementsBeforePosition
        )*/
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        // set previous measurements
        syncer.setPrivateProperty("hasPreviousAccelerometerMeasurement", true)
        val previousAccelerometerMeasurement: AccelerometerSensorMeasurement? =
            syncer.getPrivateProperty("previousAccelerometerMeasurement")
        requireNotNull(previousAccelerometerMeasurement)
        previousAccelerometerMeasurement.timestamp = accelerometerTimestamp - 1

        val accelerometerListener = accelerometerSensorCollectorSpy.measurementListener
        requireNotNull(accelerometerListener)

        val gyroscopeListener = gyroscopeSensorCollector.measurementListener
        requireNotNull(gyroscopeListener)

        // process gyroscope measurement
        gyroscopeListener.onMeasurement(
            gyroscopeSensorCollectorSpy,
            GyroscopeSensorMeasurement(),
            0
        )

        assertNull(syncer.oldestTimestamp)
        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        assertTrue(syncer.running)

        assertEquals(2, gyroscopeMeasurements.size)
        val gyroscopeMeasurement2 = gyroscopeMeasurements.first
        requireNotNull(gyroscopeMeasurement2)
        assertEquals(wx1, gyroscopeMeasurement2.wx, 0.0f)
        assertEquals(wy1, gyroscopeMeasurement2.wy, 0.0f)
        assertEquals(wz1, gyroscopeMeasurement2.wz, 0.0f)
        assertEquals(wbx, gyroscopeMeasurement2.bx)
        assertEquals(wby, gyroscopeMeasurement2.by)
        assertEquals(wbz, gyroscopeMeasurement2.bz)
        assertEquals(staleTimestamp, gyroscopeMeasurement2.timestamp)
        assertEquals(SensorAccuracy.HIGH, gyroscopeMeasurement2.accuracy)
        assertEquals(syncer.gyroscopeSensorType, gyroscopeMeasurement2.sensorType)
        val gyroscopeMeasurement3 = gyroscopeMeasurements.last
        requireNotNull(gyroscopeMeasurement3)
        assertEquals(wx, gyroscopeMeasurement3.wx, 0.0f)
        assertEquals(wy, gyroscopeMeasurement3.wy, 0.0f)
        assertEquals(wz, gyroscopeMeasurement3.wz, 0.0f)
        assertEquals(wbx, gyroscopeMeasurement3.bx)
        assertEquals(wby, gyroscopeMeasurement3.by)
        assertEquals(wbz, gyroscopeMeasurement3.bz)
        assertEquals(gyroscopeTimestamp, gyroscopeMeasurement3.timestamp)
        assertEquals(SensorAccuracy.HIGH, gyroscopeMeasurement3.accuracy)
        assertEquals(syncer.gyroscopeSensorType, gyroscopeMeasurement2.sensorType)

        val alreadyProcessedGyroscopeMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGyroscopeMeasurements")
        requireNotNull(alreadyProcessedGyroscopeMeasurements)
        assertTrue(alreadyProcessedGyroscopeMeasurements.isEmpty())

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
        verify(accelerometerSensorCollectorSpy, times(1)).getMeasurementsBeforePosition(0)
//        verify(exactly = 1) { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(0) }

        assertEquals(1, accelerometerMeasurements.size)
        val accelerometerMeasurement2 = accelerometerMeasurements.first()
        requireNotNull(accelerometerMeasurement2)
        assertSame(staleAccelerometerMeasurement, accelerometerMeasurement2)
        assertTrue(gyroscopeMeasurements.isEmpty())

        val alreadyProcessedAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertFalse(alreadyProcessedAccelerometerMeasurements.isEmpty())
        assertTrue(alreadyProcessedGyroscopeMeasurements.isEmpty())

        verifyNoInteractions(staleDetectedMeasurementsListener)
//        verify { staleDetectedMeasurementsListener wasNot Called }
    }

    @Test
    fun sensorCollectors_whenPreviousGyroscopeMeasurement_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(
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

        // set previous accelerometer and gyroscope timestamp
        syncer.setPrivateProperty("hasPreviousAccelerometerMeasurement", true)
        val previousAccelerometerMeasurement: AccelerometerSensorMeasurement? =
            syncer.getPrivateProperty("previousAccelerometerMeasurement")
        requireNotNull(previousAccelerometerMeasurement)
        previousAccelerometerMeasurement.timestamp = mostRecentTimestamp - 1
        syncer.setPrivateProperty("hasPreviousGyroscopeMeasurement", true)
        syncer.setPrivateProperty("lastNotifiedTimestamp", accelerometerTimestamp - 1)
        syncer.setPrivateProperty("lastNotifiedAccelerometerTimestamp", accelerometerTimestamp - 1)
        syncer.setPrivateProperty("lastNotifiedGyroscopeTimestamp", accelerometerTimestamp - 2)
        val previousGyroscopeMeasurement: GyroscopeSensorMeasurement? =
            syncer.getPrivateProperty("previousGyroscopeMeasurement")
        requireNotNull(previousGyroscopeMeasurement)
        val randomizer = UniformRandomizer()
        val wx1 = randomizer.nextFloat()
        val wy1 = randomizer.nextFloat()
        val wz1 = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        previousGyroscopeMeasurement.wx = wx1
        previousGyroscopeMeasurement.wy = wy1
        previousGyroscopeMeasurement.wz = wz1
        previousGyroscopeMeasurement.bx = wbx
        previousGyroscopeMeasurement.by = wby
        previousGyroscopeMeasurement.bz = wbz
        previousGyroscopeMeasurement.timestamp = accelerometerTimestamp - 1
        previousGyroscopeMeasurement.accuracy = SensorAccuracy.HIGH

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spy(gyroscopeSensorCollector)
//        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)

        val wx2 = randomizer.nextFloat()
        val wy2 = randomizer.nextFloat()
        val wz2 = randomizer.nextFloat()
        val gyroscopeTimestamp = accelerometerTimestamp - 1
        val gyroscopeMeasurement1 =
            GyroscopeSensorMeasurement(
                wx2,
                wy2,
                wz2,
                wbx,
                wby,
                wbz,
                gyroscopeTimestamp,
                SensorAccuracy.HIGH,
                syncer.gyroscopeSensorType
            )
        val measurementsBeforeTimestamp = ArrayDeque<GyroscopeSensorMeasurement>()
        measurementsBeforeTimestamp.add(gyroscopeMeasurement1)
        doReturn(measurementsBeforeTimestamp).whenever(gyroscopeSensorCollectorSpy).getMeasurementsBeforeTimestamp(any())
/*        every { gyroscopeSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            measurementsBeforeTimestamp
        )*/
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spy(accelerometerSensorCollector)
//        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)

        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        val accelerometerMeasurement1 =
            AccelerometerSensorMeasurement(
                ax,
                ay,
                az,
                abx,
                aby,
                abz,
                accelerometerTimestamp,
                SensorAccuracy.HIGH,
                syncer.accelerometerSensorType
            )
        val measurementsBeforePosition = ArrayDeque<AccelerometerSensorMeasurement>()
        measurementsBeforePosition.add(accelerometerMeasurement1)
        doReturn(measurementsBeforePosition).whenever(accelerometerSensorCollectorSpy).getMeasurementsBeforePosition(any())
/*        every { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(any()) }.returns(
            measurementsBeforePosition
        )*/
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        val accelerometerListener = accelerometerSensorCollectorSpy.measurementListener
        requireNotNull(accelerometerListener)

        val gyroscopeListener = gyroscopeSensorCollector.measurementListener
        requireNotNull(gyroscopeListener)

        // process gyroscope measurement
        gyroscopeListener.onMeasurement(
            gyroscopeSensorCollectorSpy,
            GyroscopeSensorMeasurement(),
            0
        )

        assertNull(syncer.oldestTimestamp)
        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        assertTrue(syncer.running)

        val gyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("gyroscopeMeasurements")
        requireNotNull(gyroscopeMeasurements)
        assertEquals(1, gyroscopeMeasurements.size)
        val gyroscopeMeasurement2 = gyroscopeMeasurements.peek()
        requireNotNull(gyroscopeMeasurement2)
        assertNotSame(gyroscopeMeasurement1, gyroscopeMeasurement2)
        assertEquals(wx2, gyroscopeMeasurement2.wx, 0.0f)
        assertEquals(wy2, gyroscopeMeasurement2.wy, 0.0f)
        assertEquals(wz2, gyroscopeMeasurement2.wz, 0.0f)
        assertEquals(wbx, gyroscopeMeasurement2.bx)
        assertEquals(wby, gyroscopeMeasurement2.by)
        assertEquals(wbz, gyroscopeMeasurement2.bz)
        assertEquals(gyroscopeTimestamp, gyroscopeMeasurement2.timestamp)
        assertEquals(SensorAccuracy.HIGH, gyroscopeMeasurement2.accuracy)
        assertEquals(syncer.gyroscopeSensorType, gyroscopeMeasurement2.sensorType)

        val alreadyProcessedGyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGyroscopeMeasurements")
        requireNotNull(alreadyProcessedGyroscopeMeasurements)
        assertTrue(alreadyProcessedGyroscopeMeasurements.isEmpty())

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
        verify(accelerometerSensorCollectorSpy, times(1)).getMeasurementsBeforePosition(0)
//        verify(exactly = 1) { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(0) }

        val accelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("accelerometerMeasurements")
        requireNotNull(accelerometerMeasurements)
        assertTrue(accelerometerMeasurements.isEmpty())
        assertEquals(1, gyroscopeMeasurements.size)
        assertSame(gyroscopeMeasurement2, gyroscopeMeasurements.first())

        val alreadyProcessedAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())
        assertTrue(alreadyProcessedGyroscopeMeasurements.isEmpty())

        verify(syncedMeasurementListener, only()).onSyncedMeasurements(eq(syncer), capture(accelerometerAndGyroscopeSyncedSensorMeasurementCaptor))
/*        val slot = slot<AccelerometerAndGyroscopeSyncedSensorMeasurement>()
        verify(exactly = 1) {
            syncedMeasurementListener.onSyncedMeasurements(
                syncer,
                capture(slot)
            )
        }*/

        val syncedMeasurement = accelerometerAndGyroscopeSyncedSensorMeasurementCaptor.value
//        val syncedMeasurement = slot.captured
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
        assertEquals(syncer.accelerometerSensorType, syncedAccelerometerMeasurement.sensorType)
        val syncedGyroscopeMeasurement = syncedMeasurement.gyroscopeMeasurement
        requireNotNull(syncedGyroscopeMeasurement)
        assertEquals(wx1, syncedGyroscopeMeasurement.wx, 0.0f)
        assertEquals(wy1, syncedGyroscopeMeasurement.wy, 0.0f)
        assertEquals(wz1, syncedGyroscopeMeasurement.wz, 0.0f)
        assertEquals(wbx, syncedGyroscopeMeasurement.bx)
        assertEquals(wby, syncedGyroscopeMeasurement.by)
        assertEquals(wbz, syncedGyroscopeMeasurement.bz)
        assertEquals(accelerometerTimestamp, syncedGyroscopeMeasurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, syncedGyroscopeMeasurement.accuracy)
        assertEquals(syncer.gyroscopeSensorType, syncedGyroscopeMeasurement.sensorType)
    }

    @Test
    fun sensorCollectors_whenStopping_clearCollectionsAndResets() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerAndGyroscopeSensorMeasurementSyncer(
            context,
            syncedMeasurementListener = syncedMeasurementListener
        )

        assertNull(syncer.mostRecentTimestamp)
        assertNull(syncer.oldestTimestamp)

        // set as running and stopping
        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "running", true)
        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "stopping", true)
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

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spy(gyroscopeSensorCollector)
//        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val gyroscopeTimestamp = accelerometerTimestamp - 1
        val gyroscopeMeasurement1 = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            wbx,
            wby,
            wbz,
            gyroscopeTimestamp,
            SensorAccuracy.HIGH
        )
        val measurementsBeforeTimestamp = ArrayDeque<GyroscopeSensorMeasurement>()
        measurementsBeforeTimestamp.add(gyroscopeMeasurement1)
        doReturn(measurementsBeforeTimestamp).whenever(gyroscopeSensorCollectorSpy).getMeasurementsBeforeTimestamp(any())
/*        every { gyroscopeSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            measurementsBeforeTimestamp
        )*/
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spy(accelerometerSensorCollector)
//        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)

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
            SensorAccuracy.HIGH,
            syncer.accelerometerSensorType
        )
        val measurementsBeforePosition = ArrayDeque<AccelerometerSensorMeasurement>()
        measurementsBeforePosition.add(accelerometerMeasurement1)
        doReturn(measurementsBeforePosition).whenever(accelerometerSensorCollectorSpy).getMeasurementsBeforePosition(any())
/*        every { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(any()) }.returns(
            measurementsBeforePosition
        )*/
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        val accelerometerListener = accelerometerSensorCollectorSpy.measurementListener
        requireNotNull(accelerometerListener)

        val gyroscopeListener = gyroscopeSensorCollector.measurementListener
        requireNotNull(gyroscopeListener)

        // process gyroscope measurement
        gyroscopeListener.onMeasurement(
            gyroscopeSensorCollectorSpy,
            GyroscopeSensorMeasurement(),
            0
        )

        assertNull(syncer.oldestTimestamp)
        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        assertTrue(syncer.running)

        val gyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("gyroscopeMeasurements")
        requireNotNull(gyroscopeMeasurements)
        assertEquals(1, gyroscopeMeasurements.size)
        val gyroscopeMeasurement2 = gyroscopeMeasurements.peek()
        requireNotNull(gyroscopeMeasurement2)
        assertNotSame(gyroscopeMeasurement1, gyroscopeMeasurement2)
        assertEquals(wx, gyroscopeMeasurement2.wx, 0.0f)
        assertEquals(wy, gyroscopeMeasurement2.wy, 0.0f)
        assertEquals(wz, gyroscopeMeasurement2.wz, 0.0f)
        assertEquals(wbx, gyroscopeMeasurement2.bx)
        assertEquals(wby, gyroscopeMeasurement2.by)
        assertEquals(wbz, gyroscopeMeasurement2.bz)
        assertEquals(gyroscopeTimestamp, gyroscopeMeasurement2.timestamp)
        assertEquals(SensorAccuracy.HIGH, gyroscopeMeasurement2.accuracy)
        assertEquals(syncer.gyroscopeSensorType, gyroscopeMeasurement2.sensorType)

        val alreadyProcessedGyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGyroscopeMeasurements")
        requireNotNull(alreadyProcessedGyroscopeMeasurements)
        assertTrue(alreadyProcessedGyroscopeMeasurements.isEmpty())

        // process accelerometer measurement
        accelerometerListener.onMeasurement(
            accelerometerSensorCollectorSpy,
            AccelerometerSensorMeasurement(),
            0
        )

        verify(accelerometerSensorCollectorSpy, times(1)).getMeasurementsBeforePosition(0)
//        verify(exactly = 1) { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(0) }

        assertNull(syncer.oldestTimestamp)
        assertNull(syncer.mostRecentTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        assertFalse(syncer.running)

        val accelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("accelerometerMeasurements")
        requireNotNull(accelerometerMeasurements)
        val alreadyProcessedAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        val foundGyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("foundGyroscopeMeasurements")
        requireNotNull(foundGyroscopeMeasurements)
        assertTrue(accelerometerMeasurements.isEmpty())
        assertTrue(gyroscopeMeasurements.isEmpty())
        assertTrue(alreadyProcessedGyroscopeMeasurements.isEmpty())
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())
        assertTrue(foundGyroscopeMeasurements.isEmpty())

        verify(syncedMeasurementListener, only()).onSyncedMeasurements(
            eq(syncer),
            capture(accelerometerAndGyroscopeSyncedSensorMeasurementCaptor)
        )
/*        val slot = slot<AccelerometerAndGyroscopeSyncedSensorMeasurement>()
        verify(exactly = 1) {
            syncedMeasurementListener.onSyncedMeasurements(
                syncer,
                capture(slot)
            )
        }*/

        val syncedMeasurement = accelerometerAndGyroscopeSyncedSensorMeasurementCaptor.value
        assertEquals(gyroscopeTimestamp, syncedMeasurement.timestamp)
        val syncedAccelerometerMeasurement = syncedMeasurement.accelerometerMeasurement
        requireNotNull(syncedAccelerometerMeasurement)
        assertEquals(ax, syncedAccelerometerMeasurement.ax, 0.0f)
        assertEquals(ay, syncedAccelerometerMeasurement.ay, 0.0f)
        assertEquals(az, syncedAccelerometerMeasurement.az, 0.0f)
        assertEquals(abx, syncedAccelerometerMeasurement.bx)
        assertEquals(aby, syncedAccelerometerMeasurement.by)
        assertEquals(abz, syncedAccelerometerMeasurement.bz)
        assertEquals(gyroscopeTimestamp, syncedAccelerometerMeasurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, syncedAccelerometerMeasurement.accuracy)
        assertEquals(syncer.accelerometerSensorType, syncedAccelerometerMeasurement.sensorType)
        val syncedGyroscopeMeasurement = syncedMeasurement.gyroscopeMeasurement
        requireNotNull(syncedGyroscopeMeasurement)
        assertEquals(wx, syncedGyroscopeMeasurement.wx, 0.0f)
        assertEquals(wy, syncedGyroscopeMeasurement.wy, 0.0f)
        assertEquals(wz, syncedGyroscopeMeasurement.wz, 0.0f)
        assertEquals(wbx, syncedGyroscopeMeasurement.bx)
        assertEquals(wby, syncedGyroscopeMeasurement.by)
        assertEquals(wbz, syncedGyroscopeMeasurement.bz)
        assertEquals(gyroscopeTimestamp, syncedGyroscopeMeasurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, syncedGyroscopeMeasurement.accuracy)
        assertEquals(syncer.gyroscopeSensorType, syncedGyroscopeMeasurement.sensorType)
    }
}