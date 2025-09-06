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
import androidx.test.core.app.ApplicationProvider
import com.irurueta.android.navigation.inertial.collectors.interpolators.GravityDirectSensorMeasurementInterpolator
import com.irurueta.android.navigation.inertial.collectors.interpolators.GravityQuadraticSensorMeasurementInterpolator
import com.irurueta.android.navigation.inertial.collectors.interpolators.MagnetometerDirectSensorMeasurementInterpolator
import com.irurueta.android.navigation.inertial.collectors.interpolators.MagnetometerQuadraticSensorMeasurementInterpolator
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
class GravityAndMagnetometerSensorMeasurementSyncerTest {

    @get:Rule
    val mockitoRule: MockitoRule = MockitoJUnit.rule()

//    @get:Rule
//    val mockkRule = MockKRule(this)

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var accuracyChangedListener:
            SensorMeasurementSyncer.OnAccuracyChangedListener<GravityAndMagnetometerSyncedSensorMeasurement, GravityAndMagnetometerSensorMeasurementSyncer>

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var bufferFilledListener:
            SensorMeasurementSyncer.OnBufferFilledListener<GravityAndMagnetometerSyncedSensorMeasurement, GravityAndMagnetometerSensorMeasurementSyncer>

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var syncedMeasurementListener:
            SensorMeasurementSyncer.OnSyncedMeasurementsListener<GravityAndMagnetometerSyncedSensorMeasurement, GravityAndMagnetometerSensorMeasurementSyncer>

//    @MockK(relaxUnitFun = true)
    @Mock
    private lateinit var staleDetectedMeasurementsListener:
            SensorMeasurementSyncer.OnStaleDetectedMeasurementsListener<GravityAndMagnetometerSyncedSensorMeasurement, GravityAndMagnetometerSensorMeasurementSyncer>

    @Captor
    private lateinit var gravityAndMagnetometerSyncedSensorMeasurementCaptor: ArgumentCaptor<GravityAndMagnetometerSyncedSensorMeasurement>

    /*@After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
        System.gc()
    }*/

    @Test
    fun constructor_whenRequiredParameters_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(context)

        // check
        assertSame(context, syncer.context)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            syncer.magnetometerSensorType
        )
        assertEquals(SensorDelay.FASTEST, syncer.gravitySensorDelay)
        assertEquals(SensorDelay.FASTEST, syncer.magnetometerSensorDelay)
        assertEquals(
            GravityAndMagnetometerSensorMeasurementSyncer.DEFAULT_GRAVITY_CAPACITY,
            syncer.gravityCapacity
        )
        assertEquals(
            GravityAndMagnetometerSensorMeasurementSyncer.DEFAULT_MAGNETOMETER_CAPACITY,
            syncer.magnetometerCapacity
        )
        assertFalse(syncer.gravityStartOffsetEnabled)
        assertFalse(syncer.magnetometerStartOffsetEnabled)
        assertTrue(syncer.stopWhenFilledBuffer)
        assertEquals(
            GravityAndMagnetometerSensorMeasurementSyncer.DEFAULT_STALE_OFFSET_NANOS,
            syncer.staleOffsetNanos
        )
        assertTrue(syncer.staleDetectionEnabled)
        assertTrue(syncer.skipWhenProcessing)
        assertNull(syncer.accuracyChangedListener)
        assertNull(syncer.bufferFilledListener)
        assertNull(syncer.syncedMeasurementListener)
        assertNull(syncer.staleDetectedMeasurementsListener)
        assertNotNull(syncer.gravityInterpolator)
        assertTrue(syncer.gravityInterpolator is GravityQuadraticSensorMeasurementInterpolator)
        assertNotNull(syncer.magnetometerInterpolator)
        assertTrue(syncer.magnetometerInterpolator is MagnetometerQuadraticSensorMeasurementInterpolator)
        assertEquals(0L, syncer.startTimestamp)
        assertFalse(syncer.running)
        assertFalse(syncer.processing)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        assertNull(syncer.mostRecentTimestamp)
        assertNull(syncer.oldestTimestamp)
        assertNull(syncer.gravitySensor)
        assertNull(syncer.magnetometerSensor)
        assertFalse(syncer.gravitySensorAvailable)
        assertFalse(syncer.magnetometerSensorAvailable)
        assertNull(syncer.gravityStartOffset)
        assertNull(syncer.magnetometerStartOffset)
        assertEquals(0.0f, syncer.gravityCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.magnetometerCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.gravityUsage, 0.0f)
        assertEquals(0.0f, syncer.magnetometerUsage, 0.0f)
    }

    @Test(expected = IllegalArgumentException::class)
    fun constructor_whenZeroGravityCapacity_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        GravityAndMagnetometerSensorMeasurementSyncer(context, gravityCapacity = 0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun constructor_whenZeroMagnetometerCapacity_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        GravityAndMagnetometerSensorMeasurementSyncer(context, magnetometerCapacity = 0)
    }

    @Test
    fun constructor_whenAllParameters_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val gravityInterpolator = GravityDirectSensorMeasurementInterpolator()
        val magnetometerInterpolator = MagnetometerDirectSensorMeasurementInterpolator()
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(
            context,
            MagnetometerSensorType.MAGNETOMETER,
            SensorDelay.NORMAL,
            SensorDelay.GAME,
            gravityCapacity = 2,
            magnetometerCapacity = 3,
            gravityStartOffsetEnabled = true,
            magnetometerStartOffsetEnabled = true,
            stopWhenFilledBuffer = false,
            staleOffsetNanos = 123456789L,
            staleDetectionEnabled = false,
            skipWhenProcessing = false,
            accuracyChangedListener = accuracyChangedListener,
            bufferFilledListener = bufferFilledListener,
            syncedMeasurementListener = syncedMeasurementListener,
            staleDetectedMeasurementsListener = staleDetectedMeasurementsListener,
            gravityInterpolator = gravityInterpolator,
            magnetometerInterpolator = magnetometerInterpolator
        )

        // check
        assertSame(context, syncer.context)
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER,
            syncer.magnetometerSensorType
        )
        assertEquals(SensorDelay.NORMAL, syncer.gravitySensorDelay)
        assertEquals(SensorDelay.GAME, syncer.magnetometerSensorDelay)
        assertEquals(2, syncer.gravityCapacity)
        assertEquals(3, syncer.magnetometerCapacity)
        assertTrue(syncer.gravityStartOffsetEnabled)
        assertTrue(syncer.magnetometerStartOffsetEnabled)
        assertFalse(syncer.stopWhenFilledBuffer)
        assertEquals(123456789L, syncer.staleOffsetNanos)
        assertFalse(syncer.staleDetectionEnabled)
        assertFalse(syncer.skipWhenProcessing)
        assertSame(accuracyChangedListener, syncer.accuracyChangedListener)
        assertSame(bufferFilledListener, syncer.bufferFilledListener)
        assertSame(syncedMeasurementListener, syncer.syncedMeasurementListener)
        assertSame(staleDetectedMeasurementsListener, syncer.staleDetectedMeasurementsListener)
        assertSame(gravityInterpolator, syncer.gravityInterpolator)
        assertSame(magnetometerInterpolator, syncer.magnetometerInterpolator)
        assertEquals(0L, syncer.startTimestamp)
        assertFalse(syncer.running)
        assertFalse(syncer.processing)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        assertNull(syncer.mostRecentTimestamp)
        assertNull(syncer.oldestTimestamp)
        assertNull(syncer.gravitySensor)
        assertNull(syncer.magnetometerSensor)
        assertFalse(syncer.gravitySensorAvailable)
        assertFalse(syncer.magnetometerSensorAvailable)
        assertNull(syncer.gravityStartOffset)
        assertNull(syncer.magnetometerStartOffset)
        assertEquals(0.0f, syncer.gravityCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.magnetometerCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.gravityUsage, 0.0f)
        assertEquals(0.0f, syncer.magnetometerUsage, 0.0f)
    }

    @Test
    fun accuracyChangedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(context)

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
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(context)

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
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(context)

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
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(context)

        // check default value
        assertNull(syncer.staleDetectedMeasurementsListener)

        // set new value
        syncer.staleDetectedMeasurementsListener = staleDetectedMeasurementsListener

        // check
        assertSame(staleDetectedMeasurementsListener, syncer.staleDetectedMeasurementsListener)
    }

    @Test
    fun gravitySensor_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(context)

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spy(gravitySensorCollector)
//        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)

        assertNull(syncer.gravitySensor)
        verify(gravitySensorCollectorSpy, only()).sensor
//        verify(exactly = 1) { gravitySensorCollectorSpy.sensor }
    }

    @Test
    fun magnetometerSensor_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(context)

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spy(magnetometerSensorCollector)
//        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        assertNull(syncer.magnetometerSensor)
        verify(magnetometerSensorCollectorSpy, only()).sensor
//        verify(exactly = 1) { magnetometerSensorCollectorSpy.sensor }
    }

    @Test
    fun gravitySensorAvailable_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(context)

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spy(gravitySensorCollector)
//        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)

        assertFalse(syncer.gravitySensorAvailable)
        verify(gravitySensorCollectorSpy, only()).sensorAvailable
//        verify(exactly = 1) { gravitySensorCollectorSpy.sensorAvailable }
    }

    @Test
    fun magnetometerSensorAvailable_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(context)

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spy(magnetometerSensorCollector)
//        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        assertFalse(syncer.magnetometerSensorAvailable)
        verify(magnetometerSensorCollectorSpy, only()).sensorAvailable
//        verify(exactly = 1) { magnetometerSensorCollectorSpy.sensorAvailable }
    }

    @Test
    fun gravityStartOffset_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(context)

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spy(gravitySensorCollector)
//        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)

        assertNull(syncer.gravityStartOffset)
        verify(gravitySensorCollectorSpy, only()).startOffset
//        verify(exactly = 1) { gravitySensorCollectorSpy.startOffset }
    }

    @Test
    fun magnetometerStartOffset_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(context)

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spy(magnetometerSensorCollector)
//        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        assertNull(syncer.magnetometerStartOffset)
        verify(magnetometerSensorCollectorSpy, only()).startOffset
//        verify(exactly = 1) { magnetometerSensorCollectorSpy.startOffset }
    }

    @Test
    fun gravityCollectorUsage_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(context)

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spy(gravitySensorCollector)
//        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)

        assertEquals(0.0f, syncer.gravityCollectorUsage, 0.0f)
        verify(gravitySensorCollectorSpy, only()).usage
//        verify(exactly = 1) { gravitySensorCollectorSpy.usage }
    }

    @Test
    fun magnetometerCollectorUsage_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(context)

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spy(magnetometerSensorCollector)
//        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        assertEquals(0.0f, syncer.magnetometerCollectorUsage, 0.0f)
        verify(magnetometerSensorCollectorSpy, only()).usage
//        verify(exactly = 1) { magnetometerSensorCollectorSpy.usage }
    }

    @Test
    fun gravityUsage_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(context)

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
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(context)

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
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(context)

        // set as running
        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "running", true)
        assertTrue(syncer.running)
        assertFalse(syncer.processing)

        syncer.start()
    }

    @Test
    fun start_whenNotRunningAndNoTimestamp_clearsResetsAndSetsCurrentStartTimestamp() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(context)

        assertFalse(syncer.running)
        assertFalse(syncer.processing)
        assertEquals(0L, syncer.startTimestamp)

        // set variables that will be later reset
        val gravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("gravityMeasurements")
        requireNotNull(gravityMeasurements)
        gravityMeasurements.add(GravitySensorMeasurement())

        val magnetometerMeasurements: ArrayDeque<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("magnetometerMeasurements")
        requireNotNull(magnetometerMeasurements)
        magnetometerMeasurements.add(MagnetometerSensorMeasurement())

        val alreadyProcessedGravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGravityMeasurements")
        requireNotNull(alreadyProcessedGravityMeasurements)
        alreadyProcessedGravityMeasurements.add(GravitySensorMeasurement())

        val alreadyProcessedMagnetometerMeasurements: ArrayDeque<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedMagnetometerMeasurements")
        requireNotNull(alreadyProcessedMagnetometerMeasurements)
        alreadyProcessedMagnetometerMeasurements.add(MagnetometerSensorMeasurement())

        val foundMagnetometerMeasurements: ArrayDeque<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("foundMagnetometerMeasurements")
        requireNotNull(foundMagnetometerMeasurements)
        foundMagnetometerMeasurements.add(MagnetometerSensorMeasurement())

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

        syncer.setPrivateProperty("hasPreviousGravityMeasurement", true)
        syncer.setPrivateProperty("hasPreviousMagnetometerMeasurement", true)
        syncer.setPrivateProperty("lastNotifiedTimestamp", 1L)
        syncer.setPrivateProperty("lastNotifiedGravityTimestamp", 2L)
        syncer.setPrivateProperty("lastNotifiedMagnetometerTimestamp", 3L)

        assertFalse(syncer.running)
        assertFalse(syncer.processing)
        assertEquals(0L, syncer.startTimestamp)

        assertFalse(syncer.start())

        // check
        assertFalse(syncer.running)
        assertFalse(syncer.processing)
        assertNotEquals(0L, syncer.startTimestamp)

        assertTrue(gravityMeasurements.isEmpty())
        assertTrue(magnetometerMeasurements.isEmpty())
        assertTrue(alreadyProcessedGravityMeasurements.isEmpty())
        assertTrue(alreadyProcessedMagnetometerMeasurements.isEmpty())
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
        assertFalse(syncer.processing)

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
    fun start_whenNotRunningAndTimestampProvided_setsProvidedTimestamp() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(context)

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
    fun start_whenGravityCollectorFails_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(context)

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spy(gravitySensorCollector)
//        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)
        doReturn(false).whenever(gravitySensorCollectorSpy).start(any())
//        every { gravitySensorCollectorSpy.start(any()) }.returns(false)
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)
        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spy(magnetometerSensorCollector)
//        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)
        doReturn(true).whenever(magnetometerSensorCollectorSpy).start(any())
//        every { magnetometerSensorCollectorSpy.start(any()) }.returns(true)
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        assertFalse(syncer.running)
        assertFalse(syncer.processing)

        val timestamp = System.nanoTime()
        assertFalse(syncer.start(timestamp))

        assertFalse(syncer.running)
        assertFalse(syncer.processing)
        assertEquals(timestamp, syncer.startTimestamp)

        verify(gravitySensorCollectorSpy, times(1)).start(timestamp)
//        verify(exactly = 1) { gravitySensorCollectorSpy.start(timestamp) }
        verify(magnetometerSensorCollectorSpy, never()).start(any())
//        verify(exactly = 0) { magnetometerSensorCollectorSpy.start(any()) }
    }

    @Test
    fun start_whenMagnetometerCollectorFails_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(context)

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spy(gravitySensorCollector)
//        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)
        doReturn(true).whenever(gravitySensorCollectorSpy).start(any())
//        every { gravitySensorCollectorSpy.start(any()) }.returns(true)
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)
        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spy(magnetometerSensorCollector)
//        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)
        doReturn(false).whenever(magnetometerSensorCollectorSpy).start(any())
//        every { magnetometerSensorCollectorSpy.start(any()) }.returns(false)
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        assertFalse(syncer.running)
        assertFalse(syncer.processing)

        val timestamp = System.nanoTime()
        assertFalse(syncer.start(timestamp))

        assertFalse(syncer.running)
        assertFalse(syncer.processing)
        assertEquals(timestamp, syncer.startTimestamp)

        verify(gravitySensorCollectorSpy, times(1)).start(timestamp)
//        verify(exactly = 1) { gravitySensorCollectorSpy.start(timestamp) }
        verify(magnetometerSensorCollectorSpy, times(1)).start(timestamp)
//        verify(exactly = 1) { magnetometerSensorCollectorSpy.start(timestamp) }
    }

    @Test
    fun start_whenCollectorsSucceed_returnsTrue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(context)

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spy(gravitySensorCollector)
//        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)
        doReturn(true).whenever(gravitySensorCollectorSpy).start(any())
//        every { gravitySensorCollectorSpy.start(any()) }.returns(true)
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)
        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spy(magnetometerSensorCollector)
//        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)
        doReturn(true).whenever(magnetometerSensorCollectorSpy).start(any())
//        every { magnetometerSensorCollectorSpy.start(any()) }.returns(true)
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        assertFalse(syncer.running)
        assertFalse(syncer.processing)

        val timestamp = System.nanoTime()
        assertTrue(syncer.start(timestamp))

        assertTrue(syncer.running)
        assertFalse(syncer.processing)
        assertEquals(timestamp, syncer.startTimestamp)

        verify(gravitySensorCollectorSpy, only()).start(timestamp)
//        verify(exactly = 1) { gravitySensorCollectorSpy.start(timestamp) }
        verify(magnetometerSensorCollectorSpy, only()).start(timestamp)
//        verify(exactly = 1) { magnetometerSensorCollectorSpy.start(timestamp) }
    }

    @Test
    fun stop_stopsCollectorsInitializesCachesAndResetsProperties() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(context)

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spy(gravitySensorCollector)
//        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)
        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spy(magnetometerSensorCollector)
//        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        val gravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("gravityMeasurements")
        requireNotNull(gravityMeasurements)
        gravityMeasurements.add(GravitySensorMeasurement())

        val magnetometerMeasurements: ArrayDeque<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("magnetometerMeasurements")
        requireNotNull(magnetometerMeasurements)
        magnetometerMeasurements.add(MagnetometerSensorMeasurement())

        val alreadyProcessedGravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGravityMeasurements")
        requireNotNull(alreadyProcessedGravityMeasurements)
        alreadyProcessedGravityMeasurements.add(GravitySensorMeasurement())

        val alreadyProcessedMagnetometerMeasurements: ArrayDeque<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedMagnetometerMeasurements")
        requireNotNull(alreadyProcessedMagnetometerMeasurements)
        alreadyProcessedMagnetometerMeasurements.add(MagnetometerSensorMeasurement())

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

        syncer.setPrivateProperty("hasPreviousGravityMeasurement", true)
        syncer.setPrivateProperty("hasPreviousMagnetometerMeasurement", true)
        syncer.setPrivateProperty("lastNotifiedTimestamp", 1L)
        syncer.setPrivateProperty("lastNotifiedGravityTimestamp", 2L)
        syncer.setPrivateProperty("lastNotifiedMagnetometerTimestamp", 3L)

        val stopping1: Boolean? =
            getPrivateProperty(SensorMeasurementSyncer::class, syncer, "stopping")
        requireNotNull(stopping1)
        assertFalse(stopping1)

        syncer.stop()

        // check
        verify(gravitySensorCollectorSpy, times(1)).stop()
//        verify(exactly = 1) { gravitySensorCollectorSpy.stop() }
        verify(magnetometerSensorCollectorSpy, times(1)).stop()
//        verify(exactly = 1) { magnetometerSensorCollectorSpy.stop() }
        assertEquals(1, gravityMeasurements.size)
        assertEquals(1, magnetometerMeasurements.size)
        assertEquals(1, alreadyProcessedGravityMeasurements.size)
        assertEquals(1, alreadyProcessedMagnetometerMeasurements.size)
        assertEquals(1, foundMagnetometerMeasurements.size)

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

        val lastNotifiedGravityTimestamp: Long? =
            syncer.getPrivateProperty("lastNotifiedGravityTimestamp")
        requireNotNull(lastNotifiedGravityTimestamp)
        assertEquals(0L, lastNotifiedGravityTimestamp)

        val lastNotifiedMagnetometerTimestamp: Long? =
            syncer.getPrivateProperty("lastNotifiedMagnetometerTimestamp")
        requireNotNull(lastNotifiedMagnetometerTimestamp)
        assertEquals(0L, lastNotifiedMagnetometerTimestamp)

        val stopping2: Boolean? =
            getPrivateProperty(SensorMeasurementSyncer::class, syncer, "stopping")
        requireNotNull(stopping2)
        assertTrue(stopping2)
    }

    @Test
    fun gravitySensorCollector_whenAccuracyChangedListenerAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(context)

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)

        val listener = gravitySensorCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(gravitySensorCollector, SensorAccuracy.MEDIUM)
    }

    @Test
    fun gravitySensorCollector_whenAccuracyChangedListenerAndListenerAvailable_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)

        val listener = gravitySensorCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(gravitySensorCollector, SensorAccuracy.MEDIUM)

        verify(accuracyChangedListener, only()).onAccuracyChanged(
            syncer,
            SensorType.GRAVITY,
            SensorAccuracy.MEDIUM
        )
/*        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                syncer,
                SensorType.GRAVITY,
                SensorAccuracy.MEDIUM
            )
        }*/
    }

    @Test
    fun gravitySensorCollector_whenBufferFilledAndStopWhenFilledBuffer_stops() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(
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

        verify(bufferFilledListener, only()).onBufferFilled(
            syncer,
            SensorType.GRAVITY
        )
/*        verify(exactly = 1) {
            bufferFilledListener.onBufferFilled(
                syncer,
                SensorType.GRAVITY
            )
        }*/
        assertTrue(syncer.running)
    }

    @Test
    fun gravitySensorCollector_whenSkipWhenProcessingAndProcessing_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(
            context,
            skipWhenProcessing = true,
            syncedMeasurementListener = syncedMeasurementListener
        )

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spy(gravitySensorCollector)
//        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)
        doReturn(ArrayDeque<GravitySensorMeasurement>()).whenever(gravitySensorCollectorSpy).getMeasurementsBeforePosition(any())
/*        every { gravitySensorCollectorSpy.getMeasurementsBeforePosition(any()) }.returns(
            ArrayDeque<GravitySensorMeasurement>()
        )*/
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)
        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "processing", true)

        val listener = gravitySensorCollectorSpy.measurementListener
        requireNotNull(listener)

        listener.onMeasurement(gravitySensorCollectorSpy, GravitySensorMeasurement(), 0)

        verify(gravitySensorCollectorSpy, never()).getMeasurementsBeforePosition(0)
//        verify(exactly = 0) { gravitySensorCollectorSpy.getMeasurementsBeforePosition(0) }
        verifyNoInteractions(syncedMeasurementListener)
//        verify { syncedMeasurementListener wasNot Called }
        assertNull(syncer.mostRecentTimestamp)
    }

    @Test
    fun gravitySensorCollector_whenMeasurementAndNoMeasurementsAvailable_callsCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(
            context,
            syncedMeasurementListener = syncedMeasurementListener
        )

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spy(gravitySensorCollector)
//        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)
        doReturn(ArrayDeque<GravitySensorMeasurement>()).whenever(gravitySensorCollectorSpy).getMeasurementsBeforePosition(any())
/*        every { gravitySensorCollectorSpy.getMeasurementsBeforePosition(any()) }.returns(
            ArrayDeque<GravitySensorMeasurement>()
        )*/
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)

        val listener = gravitySensorCollectorSpy.measurementListener
        requireNotNull(listener)

        listener.onMeasurement(gravitySensorCollectorSpy, GravitySensorMeasurement(), 0)

        verify(gravitySensorCollectorSpy, times(1)).getMeasurementsBeforePosition(0)
//        verify(exactly = 1) { gravitySensorCollectorSpy.getMeasurementsBeforePosition(0) }
        verifyNoInteractions(syncedMeasurementListener)
//        verify { syncedMeasurementListener wasNot Called }
        assertNull(syncer.mostRecentTimestamp)
    }

    @Test
    fun gravitySensorCollector_whenMeasurementAndMeasurementsAvailable_updatesMostRecentTimestampCopiesMeasurementsAndProcessesThem() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(
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
        val gravitySensorCollectorSpy = spy(gravitySensorCollector)
//        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)

        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val timestamp = System.nanoTime()
        val gravityMeasurement =
            GravitySensorMeasurement(gx, gy, gz, timestamp, SensorAccuracy.MEDIUM)
        val measurementsBeforePosition = ArrayDeque<GravitySensorMeasurement>()
        measurementsBeforePosition.add(gravityMeasurement)
        doReturn(measurementsBeforePosition).whenever(gravitySensorCollectorSpy).getMeasurementsBeforePosition(any())
/*        every { gravitySensorCollectorSpy.getMeasurementsBeforePosition(any()) }.returns(
            measurementsBeforePosition
        )*/
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)

        val listener = gravitySensorCollectorSpy.measurementListener
        requireNotNull(listener)

        listener.onMeasurement(gravitySensorCollectorSpy, GravitySensorMeasurement(), 0)

        verify(gravitySensorCollectorSpy, times(1)).getMeasurementsBeforePosition(0)
//        verify(exactly = 1) { gravitySensorCollectorSpy.getMeasurementsBeforePosition(0) }
        assertEquals(timestamp, syncer.mostRecentTimestamp)

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
        assertEquals(SensorAccuracy.MEDIUM, gravityMeasurement2.accuracy)

        assertEquals(timestamp, syncer.oldestTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)

        val alreadyProcessedGravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGravityMeasurements")
        requireNotNull(alreadyProcessedGravityMeasurements)
        assertTrue(alreadyProcessedGravityMeasurements.isEmpty())

        val alreadyProcessedMagnetometerMeasurements: ArrayDeque<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedMagnetometerMeasurements")
        requireNotNull(alreadyProcessedMagnetometerMeasurements)
        assertTrue(alreadyProcessedMagnetometerMeasurements.isEmpty())

        verifyNoInteractions(syncedMeasurementListener)
//        verify { syncedMeasurementListener wasNot Called }
        assertTrue(syncer.running)
    }

    @Test
    fun magnetometerSensorCollector_whenAccuracyChangedListenerAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(context)

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)

        val listener = magnetometerSensorCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(magnetometerSensorCollector, SensorAccuracy.LOW)
    }

    @Test
    fun magnetometerSensorCollector_whenAccuracyChangedListenerAndListenerAvailable_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)

        val listener = magnetometerSensorCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(magnetometerSensorCollector, SensorAccuracy.LOW)

        verify(accuracyChangedListener, only()).onAccuracyChanged(
            syncer,
            SensorType.MAGNETOMETER_UNCALIBRATED,
            SensorAccuracy.LOW
        )
/*        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                syncer,
                SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorAccuracy.LOW
            )
        }*/
    }

    @Test
    fun magnetometerSensorCollector_whenBufferFilledAndStopWhenFilledBuffer_stops() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(
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

        verify(bufferFilledListener, only()).onBufferFilled(
            syncer,
            SensorType.MAGNETOMETER_UNCALIBRATED
        )
/*        verify(exactly = 1) {
            bufferFilledListener.onBufferFilled(
                syncer,
                SensorType.MAGNETOMETER_UNCALIBRATED
            )
        }*/
        assertTrue(syncer.running)
    }

    @Test
    fun magnetometerSensorCollector_whenSkipWhenProcessingAndProcessing_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(
            context,
            skipWhenProcessing = true,
            syncedMeasurementListener = syncedMeasurementListener
        )

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spy(magnetometerSensorCollector)
//        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)
        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "processing", true)

        val listener = magnetometerSensorCollector.measurementListener
        requireNotNull(listener)

        listener.onMeasurement(magnetometerSensorCollectorSpy, MagnetometerSensorMeasurement(), 0)

        verifyNoInteractions(magnetometerSensorCollectorSpy)
//        verify { magnetometerSensorCollectorSpy wasNot Called }
        verifyNoInteractions(syncedMeasurementListener)
//        verify { syncedMeasurementListener wasNot Called }
        assertNull(syncer.mostRecentTimestamp)
    }

    @Test
    fun magnetometerSensorCollector_whenMeasurementAndMostRecentTimestampNotDefined_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(
            context,
            syncedMeasurementListener = syncedMeasurementListener
        )

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spy(magnetometerSensorCollector)
//        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        val listener = magnetometerSensorCollector.measurementListener
        requireNotNull(listener)

        listener.onMeasurement(magnetometerSensorCollectorSpy, MagnetometerSensorMeasurement(), 0)

        verifyNoInteractions(magnetometerSensorCollectorSpy)
//        verify { magnetometerSensorCollectorSpy wasNot Called }
        verifyNoInteractions(syncedMeasurementListener)
//        verify { syncedMeasurementListener wasNot Called }
        assertNull(syncer.mostRecentTimestamp)
    }

    @Test
    fun magnetometerSensorCollector_whenMeasurementMostRecentTimestampDefinedAndNoMeasurementsAvailable_callsCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(
            context,
            syncedMeasurementListener = syncedMeasurementListener
        )

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spy(magnetometerSensorCollector)
//        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)
        doReturn(ArrayDeque<MagnetometerSensorMeasurement>()).whenever(magnetometerSensorCollectorSpy).getMeasurementsBeforeTimestamp(any())
/*        every { magnetometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            ArrayDeque<MagnetometerSensorMeasurement>()
        )*/
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

        verify(magnetometerSensorCollectorSpy, only()).getMeasurementsBeforeTimestamp(
            mostRecentTimestamp
        )
/*        verify(exactly = 1) {
            magnetometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(
                mostRecentTimestamp
            )
        }*/
        verifyNoInteractions(syncedMeasurementListener)
//        verify { syncedMeasurementListener wasNot Called }
        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
    }

    @Test
    fun magnetometerSensorCollector_whenMeasurementMostRecentTimestampDefinedAndMeasurementsAvailable_copiesMeasurements() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(
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
        val magnetometerSensorCollectorSpy = spy(magnetometerSensorCollector)
//        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)

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
            SensorAccuracy.MEDIUM,
            syncer.magnetometerSensorType
        )
        val measurementsBeforeTimestamp = ArrayDeque<MagnetometerSensorMeasurement>()
        measurementsBeforeTimestamp.add(magnetometerMeasurement)
        doReturn(measurementsBeforeTimestamp).whenever(magnetometerSensorCollectorSpy).getMeasurementsBeforeTimestamp(any())
/*        every { magnetometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            measurementsBeforeTimestamp
        )*/
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

        verify(magnetometerSensorCollectorSpy, only()).getMeasurementsBeforeTimestamp(
            mostRecentTimestamp
        )
/*        verify(exactly = 1) {
            magnetometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(
                mostRecentTimestamp
            )
        }*/

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
        assertEquals(SensorAccuracy.MEDIUM, magnetometerMeasurement2.accuracy)
        assertEquals(syncer.magnetometerSensorType, magnetometerMeasurement2.sensorType)

        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
        assertNull(syncer.oldestTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)

        val alreadyProcessedGravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGravityMeasurements")
        requireNotNull(alreadyProcessedGravityMeasurements)
        assertTrue(alreadyProcessedGravityMeasurements.isEmpty())

        val alreadyProcessedMagnetometerMeasurements: ArrayDeque<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedMagnetometerMeasurements")
        requireNotNull(alreadyProcessedMagnetometerMeasurements)
        assertTrue(alreadyProcessedMagnetometerMeasurements.isEmpty())

        verifyNoInteractions(syncedMeasurementListener)
//        verify { syncedMeasurementListener wasNot Called }
        assertTrue(syncer.running)
    }

    @Test
    fun sensorCollectors_whenAllMeasurements_notifiesSyncedMeasurement() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(
            context,
            syncedMeasurementListener = syncedMeasurementListener
        )

        assertNull(syncer.mostRecentTimestamp)
        assertNull(syncer.oldestTimestamp)

        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "running", true)
        assertTrue(syncer.running)

        // set most recent timestamp
        val mostRecentTimestamp = System.nanoTime()
        val gravityTimestamp = mostRecentTimestamp - 1
        setPrivateProperty(
            SensorMeasurementSyncer::class,
            syncer,
            "mostRecentTimestamp",
            mostRecentTimestamp
        )

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spy(magnetometerSensorCollector)
//        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)

        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val magnetometerTimestamp = gravityTimestamp - 1
        val magnetometerMeasurement1 = MagnetometerSensorMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            magnetometerTimestamp,
            SensorAccuracy.MEDIUM,
            syncer.magnetometerSensorType
        )
        val measurementsBeforeTimestamp = ArrayDeque<MagnetometerSensorMeasurement>()
        measurementsBeforeTimestamp.add(magnetometerMeasurement1)
        doReturn(measurementsBeforeTimestamp).whenever(magnetometerSensorCollectorSpy).getMeasurementsBeforeTimestamp(any())
/*        every { magnetometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            measurementsBeforeTimestamp
        )*/
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spy(gravitySensorCollector)
//        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)

        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val gravityMeasurement = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            gravityTimestamp,
            SensorAccuracy.HIGH
        )
        val measurementsBeforePosition = ArrayDeque<GravitySensorMeasurement>()
        measurementsBeforePosition.add(gravityMeasurement)
        doReturn(measurementsBeforePosition).whenever(gravitySensorCollectorSpy).getMeasurementsBeforePosition(any())
/*        every { gravitySensorCollectorSpy.getMeasurementsBeforePosition(any()) }.returns(
            measurementsBeforePosition
        )*/
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)

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
        assertEquals(bx, magnetometerMeasurement2.bx, 0.0f)
        assertEquals(by, magnetometerMeasurement2.by, 0.0f)
        assertEquals(bz, magnetometerMeasurement2.bz, 0.0f)
        assertEquals(hardIronX, magnetometerMeasurement2.hardIronX)
        assertEquals(hardIronY, magnetometerMeasurement2.hardIronY)
        assertEquals(hardIronZ, magnetometerMeasurement2.hardIronZ)
        assertEquals(magnetometerTimestamp, magnetometerMeasurement2.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, magnetometerMeasurement2.accuracy)
        assertEquals(syncer.magnetometerSensorType, magnetometerMeasurement2.sensorType)

        val alreadyProcessedMagnetometerMeasurements: ArrayDeque<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedMagnetometerMeasurements")
        requireNotNull(alreadyProcessedMagnetometerMeasurements)
        assertTrue(alreadyProcessedMagnetometerMeasurements.isEmpty())

        // process gravity measurement
        gravityListener.onMeasurement(
            gravitySensorCollectorSpy,
            GravitySensorMeasurement(),
            0
        )

        assertEquals(gravityTimestamp, syncer.oldestTimestamp)
        assertEquals(gravityTimestamp, syncer.mostRecentTimestamp)
        assertEquals(1, syncer.numberOfProcessedMeasurements)
        assertTrue(syncer.running)
        verify(gravitySensorCollectorSpy, times(1)).getMeasurementsBeforePosition(0)
//        verify(exactly = 1) { gravitySensorCollectorSpy.getMeasurementsBeforePosition(0) }

        val gravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("gravityMeasurements")
        requireNotNull(gravityMeasurements)
        assertTrue(gravityMeasurements.isEmpty())
        assertTrue(magnetometerMeasurements.isEmpty())

        val alreadyProcessedGravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGravityMeasurements")
        requireNotNull(alreadyProcessedGravityMeasurements)
        assertTrue(alreadyProcessedGravityMeasurements.isEmpty())
        assertTrue(alreadyProcessedMagnetometerMeasurements.isEmpty())

        verify(syncedMeasurementListener, only()).onSyncedMeasurements(
            eq(syncer),
            capture(gravityAndMagnetometerSyncedSensorMeasurementCaptor)
        )
/*        val slot = slot<GravityAndMagnetometerSyncedSensorMeasurement>()
        verify(exactly = 1) {
            syncedMeasurementListener.onSyncedMeasurements(
                syncer,
                capture(slot)
            )
        }*/

        val syncedMeasurement = gravityAndMagnetometerSyncedSensorMeasurementCaptor.value
//        val syncedMeasurement = slot.captured
        assertEquals(magnetometerTimestamp, syncedMeasurement.timestamp)
        val syncedGravityMeasurement = syncedMeasurement.gravityMeasurement
        requireNotNull(syncedGravityMeasurement)
        assertEquals(gx, syncedGravityMeasurement.gx, 0.0f)
        assertEquals(gy, syncedGravityMeasurement.gy, 0.0f)
        assertEquals(gz, syncedGravityMeasurement.gz, 0.0f)
        assertEquals(magnetometerTimestamp, syncedGravityMeasurement.timestamp)
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
        assertEquals(syncer.magnetometerSensorType, syncedMagnetometerMeasurement.sensorType)
    }

    @Test
    fun cleanupStaleMeasurements_whenStaleMeasurementsAvailable_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(
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
        val gravityTimestamp = mostRecentTimestamp - 1
        setPrivateProperty(
            SensorMeasurementSyncer::class,
            syncer,
            "mostRecentTimestamp",
            mostRecentTimestamp
        )

        // add stale measurements
        val staleTimestamp =
            mostRecentTimestamp - 2 * GravityAndMagnetometerSensorMeasurementSyncer.DEFAULT_STALE_OFFSET_NANOS
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
            SensorAccuracy.MEDIUM,
            syncer.magnetometerSensorType
        )
        magnetometerMeasurements.add(staleMagnetometerMeasurement)

        val gravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("gravityMeasurements")
        requireNotNull(gravityMeasurements)
        val gx1 = randomizer.nextFloat()
        val gy1 = randomizer.nextFloat()
        val gz1 = randomizer.nextFloat()
        val staleGravityMeasurement = GravitySensorMeasurement(
            gx1,
            gy1,
            gz1,
            staleTimestamp,
            SensorAccuracy.HIGH
        )
        gravityMeasurements.add(staleGravityMeasurement)

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spy(magnetometerSensorCollector)
//        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)

        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val magnetometerTimestamp = gravityTimestamp - 1
        val magnetometerMeasurement1 = MagnetometerSensorMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            magnetometerTimestamp,
            SensorAccuracy.MEDIUM,
            syncer.magnetometerSensorType
        )
        val measurementsBeforeTimestamp = ArrayDeque<MagnetometerSensorMeasurement>()
        measurementsBeforeTimestamp.add(magnetometerMeasurement1)
        doReturn(measurementsBeforeTimestamp).whenever(magnetometerSensorCollectorSpy).getMeasurementsBeforeTimestamp(any())
/*        every { magnetometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            measurementsBeforeTimestamp
        )*/
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spy(gravitySensorCollector)
//        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)

        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val gravityMeasurement1 = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            gravityTimestamp,
            SensorAccuracy.HIGH
        )
        val measurementsBeforePosition = ArrayDeque<GravitySensorMeasurement>()
        measurementsBeforePosition.add(gravityMeasurement1)
        doReturn(measurementsBeforePosition).whenever(gravitySensorCollectorSpy).getMeasurementsBeforePosition(any())
/*        every { gravitySensorCollectorSpy.getMeasurementsBeforePosition(any()) }.returns(
            measurementsBeforePosition
        )*/
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)

        // set previous measurements
        syncer.setPrivateProperty("hasPreviousGravityMeasurement", true)
        val previousGravityMeasurement: GravitySensorMeasurement? =
            syncer.getPrivateProperty("previousGravityMeasurement")
        requireNotNull(previousGravityMeasurement)
        previousGravityMeasurement.timestamp = gravityTimestamp - 1

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
        assertEquals(syncer.magnetometerSensorType, magnetometerMeasurement2.sensorType)
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
        assertEquals(syncer.magnetometerSensorType, magnetometerMeasurement3.sensorType)

        val alreadyProcessedMagnetometerMeasurements: ArrayDeque<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedMagnetometerMeasurements")
        requireNotNull(alreadyProcessedMagnetometerMeasurements)
        assertTrue(alreadyProcessedMagnetometerMeasurements.isEmpty())

        // process gravity measurement
        gravityListener.onMeasurement(
            gravitySensorCollectorSpy,
            GravitySensorMeasurement(),
            0
        )

        assertEquals(staleTimestamp, syncer.oldestTimestamp)
        assertEquals(gravityTimestamp, syncer.mostRecentTimestamp)
        assertEquals(2, syncer.numberOfProcessedMeasurements)
        assertTrue(syncer.running)
        verify(gravitySensorCollectorSpy, times(1)).getMeasurementsBeforePosition(0)
//        verify(exactly = 1) { gravitySensorCollectorSpy.getMeasurementsBeforePosition(0) }

        assertTrue(gravityMeasurements.isEmpty())
        assertTrue(magnetometerMeasurements.isEmpty())

        val alreadyProcessedGravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGravityMeasurements")
        requireNotNull(alreadyProcessedGravityMeasurements)
        assertTrue(alreadyProcessedGravityMeasurements.isEmpty())
        assertTrue(alreadyProcessedMagnetometerMeasurements.isEmpty())

        verify(staleDetectedMeasurementsListener, only()).onStaleMeasurements(
            eq(syncer),
            eq(SensorType.GRAVITY),
            any()
        )
/*        verify(exactly = 1) {
            staleDetectedMeasurementsListener.onStaleMeasurements(
                syncer,
                SensorType.GRAVITY,
                any()
            )
        }*/
    }

    @Test
    fun cleanupStaleMeasurements_whenStaleDetectionDisabled_doesNotNotify() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(
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
        val gravityTimestamp = mostRecentTimestamp - 1
        setPrivateProperty(
            SensorMeasurementSyncer::class,
            syncer,
            "mostRecentTimestamp",
            mostRecentTimestamp
        )

        // add stale measurements
        val staleTimestamp =
            mostRecentTimestamp - 2 * GravityAndMagnetometerSensorMeasurementSyncer.DEFAULT_STALE_OFFSET_NANOS
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
            SensorAccuracy.MEDIUM,
            syncer.magnetometerSensorType
        )
        magnetometerMeasurements.add(staleMagnetometerMeasurement)

        val gravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("gravityMeasurements")
        requireNotNull(gravityMeasurements)
        val gx1 = randomizer.nextFloat()
        val gy1 = randomizer.nextFloat()
        val gz1 = randomizer.nextFloat()
        val staleGravityMeasurement = GravitySensorMeasurement(
            gx1,
            gy1,
            gz1,
            staleTimestamp,
            SensorAccuracy.HIGH
        )
        gravityMeasurements.add(staleGravityMeasurement)

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spy(magnetometerSensorCollector)
//        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)

        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val magnetometerTimestamp = gravityTimestamp - 1
        val magnetometerMeasurement1 = MagnetometerSensorMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            magnetometerTimestamp,
            SensorAccuracy.MEDIUM,
            syncer.magnetometerSensorType
        )
        val measurementsBeforeTimestamp = ArrayDeque<MagnetometerSensorMeasurement>()
        measurementsBeforeTimestamp.add(magnetometerMeasurement1)
        doReturn(measurementsBeforeTimestamp).whenever(magnetometerSensorCollectorSpy).getMeasurementsBeforeTimestamp(any())
/*        every { magnetometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            measurementsBeforeTimestamp
        )*/
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spy(gravitySensorCollector)
//        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)

        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val gravityMeasurement1 = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            gravityTimestamp,
            SensorAccuracy.HIGH
        )
        val measurementsBeforePosition = ArrayDeque<GravitySensorMeasurement>()
        measurementsBeforePosition.add(gravityMeasurement1)
        doReturn(measurementsBeforePosition).whenever(gravitySensorCollectorSpy).getMeasurementsBeforePosition(any())
/*        every { gravitySensorCollectorSpy.getMeasurementsBeforePosition(any()) }.returns(
            measurementsBeforePosition
        )*/
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)

        // set previous measurements
        syncer.setPrivateProperty("hasPreviousGravityMeasurement", true)
        val previousGravityMeasurement: GravitySensorMeasurement? =
            syncer.getPrivateProperty("previousGravityMeasurement")
        requireNotNull(previousGravityMeasurement)
        previousGravityMeasurement.timestamp = gravityTimestamp - 1

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
        assertEquals(syncer.magnetometerSensorType, magnetometerMeasurement2.sensorType)
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
        assertEquals(syncer.magnetometerSensorType, magnetometerMeasurement3.sensorType)

        val alreadyProcessedMagnetometerMeasurements: ArrayDeque<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedMagnetometerMeasurements")
        requireNotNull(alreadyProcessedMagnetometerMeasurements)
        assertTrue(alreadyProcessedMagnetometerMeasurements.isEmpty())

        // process gravity measurement
        gravityListener.onMeasurement(
            gravitySensorCollectorSpy,
            GravitySensorMeasurement(),
            0
        )

        assertEquals(staleTimestamp, syncer.oldestTimestamp)
        assertEquals(gravityTimestamp, syncer.mostRecentTimestamp)
        assertEquals(2, syncer.numberOfProcessedMeasurements)
        assertTrue(syncer.running)
        verify(gravitySensorCollectorSpy, times(1)).getMeasurementsBeforePosition(0)
//        verify(exactly = 1) { gravitySensorCollectorSpy.getMeasurementsBeforePosition(0) }

        assertEquals(1, gravityMeasurements.size)
        val gravityMeasurement2 = gravityMeasurements.first()
        requireNotNull(gravityMeasurement2)
        assertSame(staleGravityMeasurement, gravityMeasurement2)
        assertTrue(magnetometerMeasurements.isEmpty())

        val alreadyProcessedGravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGravityMeasurements")
        requireNotNull(alreadyProcessedGravityMeasurements)
        assertFalse(alreadyProcessedGravityMeasurements.isEmpty())
        assertTrue(alreadyProcessedMagnetometerMeasurements.isEmpty())

        verifyNoInteractions(staleDetectedMeasurementsListener)
//        verify { staleDetectedMeasurementsListener wasNot Called }
    }

    @Test
    fun sensorCollectors_whenPreviousMagnetometerMeasurement_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(
            context,
            syncedMeasurementListener = syncedMeasurementListener
        )

        assertNull(syncer.mostRecentTimestamp)
        assertNull(syncer.oldestTimestamp)

        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "running", true)
        assertTrue(syncer.running)

        // set most recent timestamp
        val mostRecentTimestamp = System.nanoTime()
        val gravityTimestamp = mostRecentTimestamp - 1
        setPrivateProperty(
            SensorMeasurementSyncer::class,
            syncer,
            "mostRecentTimestamp",
            mostRecentTimestamp
        )

        // set previous gravity and magnetometer timestamp
        syncer.setPrivateProperty("hasPreviousGravityMeasurement", true)
        val previousGravityMeasurement: GravitySensorMeasurement? =
            syncer.getPrivateProperty("previousGravityMeasurement")
        requireNotNull(previousGravityMeasurement)
        previousGravityMeasurement.timestamp = mostRecentTimestamp - 1
        syncer.setPrivateProperty("hasPreviousMagnetometerMeasurement", true)
        syncer.setPrivateProperty("lastNotifiedTimestamp", gravityTimestamp - 1)
        syncer.setPrivateProperty("lastNotifiedGravityTimestamp", gravityTimestamp - 1)
        syncer.setPrivateProperty("lastNotifiedMagnetometerTimestamp", gravityTimestamp - 2)
        val previousMagnetometerMeasurement: MagnetometerSensorMeasurement? =
            syncer.getPrivateProperty("previousMagnetometerMeasurement")
        requireNotNull(previousMagnetometerMeasurement)
        val randomizer = UniformRandomizer()
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
        previousMagnetometerMeasurement.timestamp = gravityTimestamp - 1
        previousMagnetometerMeasurement.accuracy = SensorAccuracy.MEDIUM
        previousMagnetometerMeasurement.sensorType = syncer.magnetometerSensorType

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spy(magnetometerSensorCollector)
//        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)

        val bx2 = randomizer.nextFloat()
        val by2 = randomizer.nextFloat()
        val bz2 = randomizer.nextFloat()
        val magnetometerTimestamp = gravityTimestamp - 1
        val magnetometerMeasurement1 = MagnetometerSensorMeasurement(
            bx2,
            by2,
            bz2,
            hardIronX,
            hardIronY,
            hardIronZ,
            magnetometerTimestamp,
            SensorAccuracy.MEDIUM,
            syncer.magnetometerSensorType
        )
        val measurementsBeforeTimestamp = ArrayDeque<MagnetometerSensorMeasurement>()
        measurementsBeforeTimestamp.add(magnetometerMeasurement1)
        doReturn(measurementsBeforeTimestamp).whenever(magnetometerSensorCollectorSpy).getMeasurementsBeforeTimestamp(any())
/*        every { magnetometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            measurementsBeforeTimestamp
        )*/
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spy(gravitySensorCollector)
//        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)

        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val gravityMeasurement = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            gravityTimestamp,
            SensorAccuracy.HIGH
        )
        val measurementsBeforePosition = ArrayDeque<GravitySensorMeasurement>()
        measurementsBeforePosition.add(gravityMeasurement)
        doReturn(measurementsBeforePosition).whenever(gravitySensorCollectorSpy).getMeasurementsBeforePosition(any())
/*        every { gravitySensorCollectorSpy.getMeasurementsBeforePosition(any()) }.returns(
            measurementsBeforePosition
        )*/
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)

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
        assertEquals(syncer.magnetometerSensorType, magnetometerMeasurement2.sensorType)

        val alreadyProcessedMagnetometerMeasurements: ArrayDeque<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedMagnetometerMeasurements")
        requireNotNull(alreadyProcessedMagnetometerMeasurements)
        assertTrue(alreadyProcessedMagnetometerMeasurements.isEmpty())

        // process gravity measurement
        gravityListener.onMeasurement(
            gravitySensorCollectorSpy,
            GravitySensorMeasurement(),
            0
        )

        assertEquals(gravityTimestamp, syncer.oldestTimestamp)
        assertEquals(gravityTimestamp, syncer.mostRecentTimestamp)
        assertEquals(1, syncer.numberOfProcessedMeasurements)
        assertTrue(syncer.running)
        verify(gravitySensorCollectorSpy, times(1)).getMeasurementsBeforePosition(0)
//        verify(exactly = 1) { gravitySensorCollectorSpy.getMeasurementsBeforePosition(0) }

        val gravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("gravityMeasurements")
        requireNotNull(gravityMeasurements)
        assertTrue(gravityMeasurements.isEmpty())
        assertEquals(1, magnetometerMeasurements.size)
        assertSame(magnetometerMeasurement2, magnetometerMeasurements.first())

        val alreadyProcessedGravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGravityMeasurements")
        requireNotNull(alreadyProcessedGravityMeasurements)
        assertTrue(alreadyProcessedGravityMeasurements.isEmpty())
        assertTrue(alreadyProcessedMagnetometerMeasurements.isEmpty())

        verify(syncedMeasurementListener, only()).onSyncedMeasurements(
            eq(syncer),
            capture(gravityAndMagnetometerSyncedSensorMeasurementCaptor)
        )
/*        val slot = slot<GravityAndMagnetometerSyncedSensorMeasurement>()
        verify(exactly = 1) {
            syncedMeasurementListener.onSyncedMeasurements(
                syncer,
                capture(slot)
            )
        }*/

        val syncedMeasurement = gravityAndMagnetometerSyncedSensorMeasurementCaptor.value
//        val syncedMeasurement = slot.captured
        assertEquals(gravityTimestamp, syncedMeasurement.timestamp)
        val syncedGravityMeasurement = syncedMeasurement.gravityMeasurement
        requireNotNull(syncedGravityMeasurement)
        assertEquals(gx, syncedGravityMeasurement.gx, 0.0f)
        assertEquals(gy, syncedGravityMeasurement.gy, 0.0f)
        assertEquals(gz, syncedGravityMeasurement.gz, 0.0f)
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
        assertEquals(gravityTimestamp, syncedMagnetometerMeasurement.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, syncedMagnetometerMeasurement.accuracy)
        assertEquals(syncer.magnetometerSensorType, syncedMagnetometerMeasurement.sensorType)
    }

    @Test
    fun sensorCollectors_whenStopping_clearCollectionsAndResets() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = GravityAndMagnetometerSensorMeasurementSyncer(
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
        val gravityTimestamp = mostRecentTimestamp - 1
        setPrivateProperty(
            SensorMeasurementSyncer::class,
            syncer,
            "mostRecentTimestamp",
            mostRecentTimestamp
        )

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spy(magnetometerSensorCollector)
//        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)

        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val magnetometerTimestamp = gravityTimestamp - 1
        val magnetometerMeasurement1 = MagnetometerSensorMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            magnetometerTimestamp,
            SensorAccuracy.MEDIUM,
            syncer.magnetometerSensorType
        )
        val measurementsBeforeTimestamp = ArrayDeque<MagnetometerSensorMeasurement>()
        measurementsBeforeTimestamp.add(magnetometerMeasurement1)
        doReturn(measurementsBeforeTimestamp).whenever(magnetometerSensorCollectorSpy).getMeasurementsBeforeTimestamp(any())
/*        every { magnetometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            measurementsBeforeTimestamp
        )*/
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spy(gravitySensorCollector)
//        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)

        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val gravityMeasurement = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            gravityTimestamp,
            SensorAccuracy.HIGH
        )
        val measurementsBeforePosition = ArrayDeque<GravitySensorMeasurement>()
        measurementsBeforePosition.add(gravityMeasurement)
        doReturn(measurementsBeforePosition).whenever(gravitySensorCollectorSpy).getMeasurementsBeforePosition(any())
/*        every { gravitySensorCollectorSpy.getMeasurementsBeforePosition(any()) }.returns(
            measurementsBeforePosition
        )*/
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)

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
        assertEquals(bx, magnetometerMeasurement2.bx, 0.0f)
        assertEquals(by, magnetometerMeasurement2.by, 0.0f)
        assertEquals(bz, magnetometerMeasurement2.bz, 0.0f)
        assertEquals(hardIronX, magnetometerMeasurement2.hardIronX)
        assertEquals(hardIronY, magnetometerMeasurement2.hardIronY)
        assertEquals(hardIronZ, magnetometerMeasurement2.hardIronZ)
        assertEquals(magnetometerTimestamp, magnetometerMeasurement2.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, magnetometerMeasurement2.accuracy)
        assertEquals(syncer.magnetometerSensorType, magnetometerMeasurement2.sensorType)

        val alreadyProcessedMagnetometerMeasurements: ArrayDeque<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedMagnetometerMeasurements")
        requireNotNull(alreadyProcessedMagnetometerMeasurements)
        assertTrue(alreadyProcessedMagnetometerMeasurements.isEmpty())

        // process gravity measurement
        gravityListener.onMeasurement(
            gravitySensorCollectorSpy,
            GravitySensorMeasurement(),
            0
        )

        verify(gravitySensorCollectorSpy, times(1)).getMeasurementsBeforePosition(0)
//        verify(exactly = 1) { gravitySensorCollectorSpy.getMeasurementsBeforePosition(0) }

        assertNull(syncer.oldestTimestamp)
        assertNull(syncer.mostRecentTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        assertFalse(syncer.running)

        val gravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("gravityMeasurements")
        requireNotNull(gravityMeasurements)
        val alreadyProcessedGravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGravityMeasurements")
        requireNotNull(alreadyProcessedGravityMeasurements)
        val foundMagnetometerMeasurements: ArrayDeque<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("foundMagnetometerMeasurements")
        requireNotNull(foundMagnetometerMeasurements)
        assertTrue(gravityMeasurements.isEmpty())
        assertTrue(magnetometerMeasurements.isEmpty())
        assertTrue(alreadyProcessedGravityMeasurements.isEmpty())
        assertTrue(alreadyProcessedMagnetometerMeasurements.isEmpty())
        assertTrue(foundMagnetometerMeasurements.isEmpty())

        verify(syncedMeasurementListener, only()).onSyncedMeasurements(
            eq(syncer),
            capture(gravityAndMagnetometerSyncedSensorMeasurementCaptor)
        )
/*        val slot = slot<GravityAndMagnetometerSyncedSensorMeasurement>()
        verify(exactly = 1) {
            syncedMeasurementListener.onSyncedMeasurements(
                syncer,
                capture(slot)
            )
        }*/

        val syncedMeasurement = gravityAndMagnetometerSyncedSensorMeasurementCaptor.value
//        val syncedMeasurement = slot.captured
        assertEquals(magnetometerTimestamp, syncedMeasurement.timestamp)
        val syncedGravityMeasurement = syncedMeasurement.gravityMeasurement
        requireNotNull(syncedGravityMeasurement)
        assertEquals(gx, syncedGravityMeasurement.gx, 0.0f)
        assertEquals(gy, syncedGravityMeasurement.gy, 0.0f)
        assertEquals(gz, syncedGravityMeasurement.gz, 0.0f)
        assertEquals(magnetometerTimestamp, syncedGravityMeasurement.timestamp)
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
        assertEquals(syncer.magnetometerSensorType, syncedMagnetometerMeasurement.sensorType)
    }
}