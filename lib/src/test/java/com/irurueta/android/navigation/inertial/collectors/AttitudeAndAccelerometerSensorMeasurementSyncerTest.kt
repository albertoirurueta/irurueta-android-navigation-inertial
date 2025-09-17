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
import com.irurueta.android.navigation.inertial.collectors.interpolators.AttitudeDirectSensorMeasurementInterpolator
import com.irurueta.android.navigation.inertial.collectors.interpolators.AttitudeLinearSensorMeasurementInterpolator
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.android.testutils.setPrivateProperty
import com.irurueta.geometry.Quaternion
import com.irurueta.statistics.UniformRandomizer
import io.mockk.Called
import io.mockk.every
import io.mockk.impl.annotations.MockK
import io.mockk.junit4.MockKRule
import io.mockk.slot
import io.mockk.spyk
import io.mockk.verify
import org.junit.Assert.assertEquals
import org.junit.Assert.assertFalse
import org.junit.Assert.assertNotEquals
import org.junit.Assert.assertNotNull
import org.junit.Assert.assertNotSame
import org.junit.Assert.assertNull
import org.junit.Assert.assertSame
import org.junit.Assert.assertTrue
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import java.util.ArrayDeque

@RunWith(RobolectricTestRunner::class)
class AttitudeAndAccelerometerSensorMeasurementSyncerTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK(relaxUnitFun = true)
    private lateinit var accuracyChangedListener:
            SensorMeasurementSyncer.OnAccuracyChangedListener<AttitudeAndAccelerometerSyncedSensorMeasurement, AttitudeAndAccelerometerSensorMeasurementSyncer>

    @MockK(relaxUnitFun = true)
    private lateinit var bufferFilledListener:
            SensorMeasurementSyncer.OnBufferFilledListener<AttitudeAndAccelerometerSyncedSensorMeasurement, AttitudeAndAccelerometerSensorMeasurementSyncer>

    @MockK(relaxUnitFun = true)
    private lateinit var syncedMeasurementListener:
            SensorMeasurementSyncer.OnSyncedMeasurementsListener<AttitudeAndAccelerometerSyncedSensorMeasurement, AttitudeAndAccelerometerSensorMeasurementSyncer>

    @MockK(relaxUnitFun = true)
    private lateinit var staleDetectedMeasurementsListener:
            SensorMeasurementSyncer.OnStaleDetectedMeasurementsListener<AttitudeAndAccelerometerSyncedSensorMeasurement, AttitudeAndAccelerometerSensorMeasurementSyncer>

    @Test
    fun constructor_whenRequiredParameters_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(context)

        // check
        assertSame(context, syncer.context)
        assertEquals(AttitudeSensorType.ABSOLUTE_ATTITUDE, syncer.attitudeSensorType)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            syncer.accelerometerSensorType
        )
        assertEquals(SensorDelay.FASTEST, syncer.attitudeSensorDelay)
        assertEquals(SensorDelay.FASTEST, syncer.accelerometerSensorDelay)
        assertEquals(
            AttitudeAndAccelerometerSensorMeasurementSyncer.DEFAULT_ATTITUDE_CAPACITY,
            syncer.attitudeCapacity
        )
        assertEquals(
            AttitudeAndAccelerometerSensorMeasurementSyncer.DEFAULT_ACCELEROMETER_CAPACITY,
            syncer.accelerometerCapacity
        )
        assertFalse(syncer.attitudeStartOffsetEnabled)
        assertFalse(syncer.accelerometerStartOffsetEnabled)
        assertTrue(syncer.stopWhenFilledBuffer)
        assertEquals(
            AttitudeAndAccelerometerSensorMeasurementSyncer.DEFAULT_STALE_OFFSET_NANOS,
            syncer.staleOffsetNanos
        )
        assertTrue(syncer.staleDetectionEnabled)
        assertTrue(syncer.skipWhenProcessing)
        assertNull(syncer.accuracyChangedListener)
        assertNull(syncer.bufferFilledListener)
        assertNull(syncer.syncedMeasurementListener)
        assertNull(syncer.staleDetectedMeasurementsListener)
        assertNotNull(syncer.attitudeInterpolator)
        assertTrue(syncer.attitudeInterpolator is AttitudeLinearSensorMeasurementInterpolator)
        assertNotNull(syncer.accelerometerInterpolator)
        assertTrue(syncer.accelerometerInterpolator is AccelerometerQuadraticSensorMeasurementInterpolator)
        assertEquals(0L, syncer.startTimestamp)
        assertFalse(syncer.running)
        assertFalse(syncer.processing)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        assertNull(syncer.mostRecentTimestamp)
        assertNull(syncer.oldestTimestamp)
        assertNull(syncer.attitudeSensor)
        assertNull(syncer.accelerometerSensor)
        assertFalse(syncer.attitudeSensorAvailable)
        assertFalse(syncer.accelerometerSensorAvailable)
        assertNull(syncer.attitudeStartOffset)
        assertNull(syncer.accelerometerStartOffset)
        assertEquals(0.0f, syncer.attitudeCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.accelerometerCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.attitudeUsage, 0.0f)
        assertEquals(0.0f, syncer.accelerometerUsage, 0.0f)
    }

    @Test(expected = IllegalArgumentException::class)
    fun constructor_whenZeroAttitudeCapacity_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        AttitudeAndAccelerometerSensorMeasurementSyncer(context, attitudeCapacity = 0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun constructor_whenZeroAccelerometerCapacity_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        AttitudeAndAccelerometerSensorMeasurementSyncer(context, accelerometerCapacity = 0)
    }

    @Test
    fun constructor_whenAllParameters_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val attitudeInterpolator = AttitudeDirectSensorMeasurementInterpolator()
        val accelerometerInterpolator = AccelerometerDirectSensorMeasurementInterpolator()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(
            context,
            AttitudeSensorType.RELATIVE_ATTITUDE,
            AccelerometerSensorType.ACCELEROMETER,
            SensorDelay.NORMAL,
            SensorDelay.UI,
            attitudeCapacity = 2,
            accelerometerCapacity = 4,
            attitudeStartOffsetEnabled = true,
            accelerometerStartOffsetEnabled = true,
            stopWhenFilledBuffer = false,
            staleOffsetNanos = 123456789L,
            staleDetectionEnabled = false,
            skipWhenProcessing = false,
            accuracyChangedListener = accuracyChangedListener,
            bufferFilledListener = bufferFilledListener,
            syncedMeasurementListener = syncedMeasurementListener,
            staleDetectedMeasurementsListener = staleDetectedMeasurementsListener,
            attitudeInterpolator = attitudeInterpolator,
            accelerometerInterpolator = accelerometerInterpolator
        )

        // check
        assertSame(context, syncer.context)
        assertEquals(AttitudeSensorType.RELATIVE_ATTITUDE, syncer.attitudeSensorType)
        assertEquals(AccelerometerSensorType.ACCELEROMETER, syncer.accelerometerSensorType)
        assertEquals(SensorDelay.NORMAL, syncer.attitudeSensorDelay)
        assertEquals(SensorDelay.UI, syncer.accelerometerSensorDelay)
        assertEquals(2, syncer.attitudeCapacity)
        assertEquals(4, syncer.accelerometerCapacity)
        assertTrue(syncer.attitudeStartOffsetEnabled)
        assertTrue(syncer.accelerometerStartOffsetEnabled)
        assertFalse(syncer.stopWhenFilledBuffer)
        assertEquals(123456789L, syncer.staleOffsetNanos)
        assertFalse(syncer.staleDetectionEnabled)
        assertFalse(syncer.skipWhenProcessing)
        assertSame(accuracyChangedListener, syncer.accuracyChangedListener)
        assertSame(bufferFilledListener, syncer.bufferFilledListener)
        assertSame(syncedMeasurementListener, syncer.syncedMeasurementListener)
        assertSame(staleDetectedMeasurementsListener, syncer.staleDetectedMeasurementsListener)
        assertSame(attitudeInterpolator, syncer.attitudeInterpolator)
        assertSame(accelerometerInterpolator, syncer.accelerometerInterpolator)
        assertEquals(0L, syncer.startTimestamp)
        assertFalse(syncer.running)
        assertFalse(syncer.processing)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        assertNull(syncer.mostRecentTimestamp)
        assertNull(syncer.oldestTimestamp)
        assertNull(syncer.attitudeSensor)
        assertNull(syncer.accelerometerSensor)
        assertFalse(syncer.attitudeSensorAvailable)
        assertFalse(syncer.accelerometerSensorAvailable)
        assertNull(syncer.attitudeStartOffset)
        assertNull(syncer.accelerometerStartOffset)
        assertEquals(0.0f, syncer.attitudeCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.accelerometerCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.attitudeUsage, 0.0f)
        assertEquals(0.0f, syncer.accelerometerUsage, 0.0f)
    }

    @Test
    fun accuracyChangedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(context)

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
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(context)

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
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(context)

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
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(context)

        // check default value
        assertNull(syncer.staleDetectedMeasurementsListener)

        // set new value
        syncer.staleDetectedMeasurementsListener = staleDetectedMeasurementsListener

        // check
        assertSame(staleDetectedMeasurementsListener, syncer.staleDetectedMeasurementsListener)
    }

    @Test
    fun attitudeSensor_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(context)

        val attitudeSensorCollector: BufferedAttitudeSensorCollector? =
            syncer.getPrivateProperty("attitudeSensorCollector")
        requireNotNull(attitudeSensorCollector)
        val attitudeSensorCollectorSpy = spyk(attitudeSensorCollector)
        syncer.setPrivateProperty("attitudeSensorCollector", attitudeSensorCollectorSpy)

        assertNull(syncer.attitudeSensor)
        verify(exactly = 1) { attitudeSensorCollectorSpy.sensor }
    }

    @Test
    fun accelerometerSensor_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(context)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        assertNull(syncer.accelerometerSensor)
        verify(exactly = 1) { accelerometerSensorCollectorSpy.sensor }
    }

    @Test
    fun attitudeSensorAvailable_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(context)

        val attitudeSensorCollector: BufferedAttitudeSensorCollector? =
            syncer.getPrivateProperty("attitudeSensorCollector")
        requireNotNull(attitudeSensorCollector)
        val attitudeSensorCollectorSpy = spyk(attitudeSensorCollector)
        syncer.setPrivateProperty("attitudeSensorCollector", attitudeSensorCollectorSpy)

        assertFalse(syncer.attitudeSensorAvailable)
        verify(exactly = 1) { attitudeSensorCollectorSpy.sensorAvailable }
    }

    @Test
    fun accelerometerSensorAvailable_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(context)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        assertFalse(syncer.accelerometerSensorAvailable)
        verify(exactly = 1) { accelerometerSensorCollectorSpy.sensorAvailable }
    }

    @Test
    fun attitudeStartOffset_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(context)

        val attitudeSensorCollector: BufferedAttitudeSensorCollector? =
            syncer.getPrivateProperty("attitudeSensorCollector")
        requireNotNull(attitudeSensorCollector)
        val attitudeSensorCollectorSpy = spyk(attitudeSensorCollector)
        syncer.setPrivateProperty("attitudeSensorCollector", attitudeSensorCollectorSpy)

        assertNull(syncer.attitudeStartOffset)
        verify(exactly = 1) { attitudeSensorCollectorSpy.startOffset }
    }

    @Test
    fun accelerometerStartOffset_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(context)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        assertNull(syncer.accelerometerStartOffset)
        verify(exactly = 1) { accelerometerSensorCollectorSpy.startOffset }
    }

    @Test
    fun attitudeCollectorUsage_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(context)

        val attitudeSensorCollector: BufferedAttitudeSensorCollector? =
            syncer.getPrivateProperty("attitudeSensorCollector")
        requireNotNull(attitudeSensorCollector)
        val attitudeSensorCollectorSpy = spyk(attitudeSensorCollector)
        syncer.setPrivateProperty("attitudeSensorCollector", attitudeSensorCollectorSpy)

        assertEquals(0.0f, syncer.attitudeCollectorUsage, 0.0f)
        verify(exactly = 1) { attitudeSensorCollectorSpy.usage }
    }

    @Test
    fun accelerometerCollectorUsage_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(context)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        assertEquals(0.0f, syncer.accelerometerCollectorUsage, 0.0f)
        verify(exactly = 1) { accelerometerSensorCollectorSpy.usage }
    }

    @Test
    fun attitudeUsage_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(context)

        // check default value
        assertEquals(0.0f, syncer.attitudeUsage, 0.0f)

        // add measurement
        val attitudeMeasurements: ArrayDeque<AttitudeSensorMeasurement>? =
            syncer.getPrivateProperty("attitudeMeasurements")
        requireNotNull(attitudeMeasurements)
        assertTrue(attitudeMeasurements.isEmpty())
        attitudeMeasurements.add(AttitudeSensorMeasurement())

        // check
        assertEquals(1.0f / syncer.attitudeCapacity.toFloat(), syncer.attitudeUsage, 0.0f)
    }

    @Test
    fun accelerometerUsage_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(context)

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

    @Test(expected = IllegalStateException::class)
    fun start_whenAlreadyRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(context)

        // set as running
        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "running", true)
        assertTrue(syncer.running)
        assertFalse(syncer.processing)

        syncer.start()
    }

    @Test
    fun start_whenNotRunningAndNoTimestamp_clearsResetAndSetsCurrentStartTimestamp() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(context)

        assertFalse(syncer.running)
        assertFalse(syncer.processing)
        assertEquals(0L, syncer.startTimestamp)

        // sets variables that will be later reset
        val attitudeMeasurements: ArrayDeque<AttitudeSensorMeasurement>? =
            syncer.getPrivateProperty("attitudeMeasurements")
        requireNotNull(attitudeMeasurements)
        attitudeMeasurements.add(AttitudeSensorMeasurement())

        val accelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("accelerometerMeasurements")
        requireNotNull(accelerometerMeasurements)
        accelerometerMeasurements.add(AccelerometerSensorMeasurement())

        val alreadyProcessedAttitudeMeasurements: ArrayDeque<AttitudeSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAttitudeMeasurements")
        requireNotNull(alreadyProcessedAttitudeMeasurements)
        alreadyProcessedAttitudeMeasurements.add(AttitudeSensorMeasurement())

        val alreadyProcessedAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        alreadyProcessedAccelerometerMeasurements.add(AccelerometerSensorMeasurement())

        val foundAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("foundAccelerometerMeasurements")
        requireNotNull(foundAccelerometerMeasurements)
        foundAccelerometerMeasurements.add(AccelerometerSensorMeasurement())

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

        syncer.setPrivateProperty("hasPreviousAttitudeMeasurement", true)
        syncer.setPrivateProperty("hasPreviousAccelerometerMeasurement", true)
        syncer.setPrivateProperty("lastNotifiedTimestamp", 1L)
        syncer.setPrivateProperty("lastNotifiedAttitudeTimestamp", 3L)
        syncer.setPrivateProperty("lastNotifiedAccelerometerTimestamp", 2L)

        assertFalse(syncer.running)
        assertEquals(0L, syncer.startTimestamp)

        assertFalse(syncer.start())

        // check
        assertFalse(syncer.running)
        assertNotEquals(0L, syncer.startTimestamp)

        assertTrue(attitudeMeasurements.isEmpty())
        assertTrue(accelerometerMeasurements.isEmpty())
        assertTrue(alreadyProcessedAttitudeMeasurements.isEmpty())
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())
        assertTrue(foundAccelerometerMeasurements.isEmpty())

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

        val hasPreviousAttitudeMeasurement: Boolean? =
            syncer.getPrivateProperty("hasPreviousAttitudeMeasurement")
        requireNotNull(hasPreviousAttitudeMeasurement)
        assertFalse(hasPreviousAttitudeMeasurement)

        val hasPreviousAccelerometerMeasurement: Boolean? =
            syncer.getPrivateProperty("hasPreviousAccelerometerMeasurement")
        requireNotNull(hasPreviousAccelerometerMeasurement)
        assertFalse(hasPreviousAccelerometerMeasurement)

        val lastNotifiedTimestamp: Long? = syncer.getPrivateProperty("lastNotifiedTimestamp")
        requireNotNull(lastNotifiedTimestamp)
        assertEquals(0L, lastNotifiedTimestamp)

        val lastNotifiedAttitudeTimestamp: Long? =
            syncer.getPrivateProperty("lastNotifiedAttitudeTimestamp")
        requireNotNull(lastNotifiedAttitudeTimestamp)
        assertEquals(0L, lastNotifiedAttitudeTimestamp)

        val lastNotifiedAccelerometerTimestamp: Long? =
            syncer.getPrivateProperty("lastNotifiedAccelerometerTimestamp")
        requireNotNull(lastNotifiedAccelerometerTimestamp)
        assertEquals(0L, lastNotifiedAccelerometerTimestamp)
    }

    @Test
    fun start_whenNotRunningAndTimestampProvided_setsProvidedTimestamp() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(context)

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
    fun start_whenAttitudeCollectorFails_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(context)

        val attitudeSensorCollector: BufferedAttitudeSensorCollector? =
            syncer.getPrivateProperty("attitudeSensorCollector")
        requireNotNull(attitudeSensorCollector)
        val attitudeSensorCollectorSpy = spyk(attitudeSensorCollector)
        every { attitudeSensorCollectorSpy.start(any()) }.returns(false)
        syncer.setPrivateProperty("attitudeSensorCollector", attitudeSensorCollectorSpy)
        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.start(any()) }.returns(true)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        assertFalse(syncer.running)
        assertFalse(syncer.processing)

        val timestamp = System.nanoTime()
        assertFalse(syncer.start(timestamp))

        assertFalse(syncer.running)
        assertFalse(syncer.processing)
        assertEquals(timestamp, syncer.startTimestamp)

        verify(exactly = 1) { attitudeSensorCollectorSpy.start(timestamp) }
        verify(exactly = 0) { accelerometerSensorCollectorSpy.start(any()) }
    }

    @Test
    fun start_whenAccelerometerCollectorFails_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(context)

        val attitudeSensorCollector: BufferedAttitudeSensorCollector? =
            syncer.getPrivateProperty("attitudeSensorCollector")
        requireNotNull(attitudeSensorCollector)
        val attitudeSensorCollectorSpy = spyk(attitudeSensorCollector)
        every { attitudeSensorCollectorSpy.start(any()) }.returns(true)
        syncer.setPrivateProperty("attitudeSensorCollector", attitudeSensorCollectorSpy)
        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.start(any()) }.returns(false)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        assertFalse(syncer.running)
        assertFalse(syncer.processing)

        val timestamp = System.nanoTime()
        assertFalse(syncer.start(timestamp))

        assertFalse(syncer.running)
        assertFalse(syncer.processing)
        assertEquals(timestamp, syncer.startTimestamp)

        verify(exactly = 1) { attitudeSensorCollectorSpy.start(timestamp) }
        verify(exactly = 1) { accelerometerSensorCollectorSpy.start(any()) }
    }

    @Test
    fun start_whenCollectorsSucceed_returnsTrue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(context)

        val attitudeSensorCollector: BufferedAttitudeSensorCollector? =
            syncer.getPrivateProperty("attitudeSensorCollector")
        requireNotNull(attitudeSensorCollector)
        val attitudeSensorCollectorSpy = spyk(attitudeSensorCollector)
        every { attitudeSensorCollectorSpy.start(any()) }.returns(true)
        syncer.setPrivateProperty("attitudeSensorCollector", attitudeSensorCollectorSpy)
        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.start(any()) }.returns(true)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        assertFalse(syncer.running)
        assertFalse(syncer.processing)

        val timestamp = System.nanoTime()
        assertTrue(syncer.start(timestamp))

        assertTrue(syncer.running)
        assertFalse(syncer.processing)
        assertEquals(timestamp, syncer.startTimestamp)

        verify(exactly = 1) { attitudeSensorCollectorSpy.start(timestamp) }
        verify(exactly = 1) { accelerometerSensorCollectorSpy.start(any()) }
    }

    @Test
    fun stop_stopsCollectorsInitializesCachesAndResetsProperties() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(context)

        val attitudeSensorCollector: BufferedAttitudeSensorCollector? =
            syncer.getPrivateProperty("attitudeSensorCollector")
        requireNotNull(attitudeSensorCollector)
        val attitudeSensorCollectorSpy = spyk(attitudeSensorCollector)
        syncer.setPrivateProperty("attitudeSensorCollector", attitudeSensorCollectorSpy)
        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        val attitudeMeasurements: ArrayDeque<AttitudeSensorMeasurement>? =
            syncer.getPrivateProperty("attitudeMeasurements")
        requireNotNull(attitudeMeasurements)
        attitudeMeasurements.add(AttitudeSensorMeasurement())

        val accelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("accelerometerMeasurements")
        requireNotNull(accelerometerMeasurements)
        accelerometerMeasurements.add(AccelerometerSensorMeasurement())

        val alreadyProcessedAttitudeMeasurements: ArrayDeque<AttitudeSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAttitudeMeasurements")
        requireNotNull(alreadyProcessedAttitudeMeasurements)
        alreadyProcessedAttitudeMeasurements.add(AttitudeSensorMeasurement())

        val alreadyProcessedAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        alreadyProcessedAccelerometerMeasurements.add(AccelerometerSensorMeasurement())

        val foundAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("foundAccelerometerMeasurements")
        requireNotNull(foundAccelerometerMeasurements)
        foundAccelerometerMeasurements.add(AccelerometerSensorMeasurement())

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

        syncer.setPrivateProperty("hasPreviousAttitudeMeasurement", true)
        syncer.setPrivateProperty("hasPreviousAccelerometerMeasurement", true)
        syncer.setPrivateProperty("lastNotifiedTimestamp", 1L)
        syncer.setPrivateProperty("lastNotifiedAttitudeTimestamp", 4L)
        syncer.setPrivateProperty("lastNotifiedAccelerometerTimestamp", 2L)

        val stopping1: Boolean? =
            getPrivateProperty(SensorMeasurementSyncer::class, syncer, "stopping")
        requireNotNull(stopping1)
        assertFalse(stopping1)

        syncer.stop()

        // check
        verify(exactly = 1) { attitudeSensorCollectorSpy.stop() }
        verify(exactly = 1) { accelerometerSensorCollectorSpy.stop() }
        assertEquals(1, attitudeMeasurements.size)
        assertEquals(1, accelerometerMeasurements.size)
        assertEquals(1, alreadyProcessedAttitudeMeasurements.size)
        assertEquals(1, alreadyProcessedAccelerometerMeasurements.size)
        assertEquals(1, foundAccelerometerMeasurements.size)

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

        val hasPreviousAttitudeMeasurement: Boolean? =
            syncer.getPrivateProperty("hasPreviousAttitudeMeasurement")
        requireNotNull(hasPreviousAttitudeMeasurement)
        assertFalse(hasPreviousAttitudeMeasurement)

        val hasPreviousAccelerometerMeasurement: Boolean? =
            syncer.getPrivateProperty("hasPreviousAccelerometerMeasurement")
        requireNotNull(hasPreviousAccelerometerMeasurement)
        assertFalse(hasPreviousAccelerometerMeasurement)

        val lastNotifiedTimestamp: Long? = syncer.getPrivateProperty("lastNotifiedTimestamp")
        requireNotNull(lastNotifiedTimestamp)
        assertEquals(0L, lastNotifiedTimestamp)

        val lastNotifiedAttitudeTimestamp: Long? =
            syncer.getPrivateProperty("lastNotifiedAttitudeTimestamp")
        requireNotNull(lastNotifiedAttitudeTimestamp)
        assertEquals(0L, lastNotifiedAttitudeTimestamp)

        val lastNotifiedAccelerometerTimestamp: Long? =
            syncer.getPrivateProperty("lastNotifiedAccelerometerTimestamp")
        requireNotNull(lastNotifiedAccelerometerTimestamp)
        assertEquals(0L, lastNotifiedAccelerometerTimestamp)

        val stopping2: Boolean? =
            getPrivateProperty(SensorMeasurementSyncer::class, syncer, "stopping")
        requireNotNull(stopping2)
        assertTrue(stopping2)
    }

    @Test
    fun attitudeSensorCollector_whenAccuracyChangedListenerAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(context)

        val attitudeSensorCollector: BufferedAttitudeSensorCollector? =
            syncer.getPrivateProperty("attitudeSensorCollector")
        requireNotNull(attitudeSensorCollector)

        val listener = attitudeSensorCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(attitudeSensorCollector, SensorAccuracy.HIGH)
    }

    @Test
    fun attitudeSensorCollector_whenAccuracyChangedListenerAndListenerAvailable_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val attitudeSensorCollector: BufferedAttitudeSensorCollector? =
            syncer.getPrivateProperty("attitudeSensorCollector")
        requireNotNull(attitudeSensorCollector)

        val listener = attitudeSensorCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(attitudeSensorCollector, SensorAccuracy.HIGH)

        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                syncer,
                SensorType.ABSOLUTE_ATTITUDE,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun attitudeSensorCollector_whenBufferFilledAndStopWhenFilledBuffer_stops() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(
            context,
            stopWhenFilledBuffer = true
        )

        // set as running
        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "running", true)
        assertTrue(syncer.running)
        assertTrue(syncer.stopWhenFilledBuffer)

        val attitudeSensorCollector: BufferedAttitudeSensorCollector? =
            syncer.getPrivateProperty("attitudeSensorCollector")
        requireNotNull(attitudeSensorCollector)

        val listener = attitudeSensorCollector.bufferFilledListener
        requireNotNull(listener)

        listener.onBufferFilled(attitudeSensorCollector)

        assertFalse(syncer.running)
    }

    @Test
    fun attitudeSensorCollector_whenBufferFilledAndListenerAvailable_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(
            context,
            stopWhenFilledBuffer = false,
            bufferFilledListener = bufferFilledListener
        )

        // set as running
        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "running", true)
        assertTrue(syncer.running)
        assertFalse(syncer.stopWhenFilledBuffer)

        val attitudeSensorCollector: BufferedAttitudeSensorCollector? =
            syncer.getPrivateProperty("attitudeSensorCollector")
        requireNotNull(attitudeSensorCollector)

        val listener = attitudeSensorCollector.bufferFilledListener
        requireNotNull(listener)

        listener.onBufferFilled(attitudeSensorCollector)

        verify(exactly = 1) {
            bufferFilledListener.onBufferFilled(syncer, SensorType.ABSOLUTE_ATTITUDE)
        }
        assertTrue(syncer.running)
    }

    @Test
    fun attitudeSensorCollector_whenSkipWhenProcessingAndProcessing_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(
            context,
            skipWhenProcessing = true,
            syncedMeasurementListener = syncedMeasurementListener
        )

        val attitudeSensorCollector: BufferedAttitudeSensorCollector? =
            syncer.getPrivateProperty("attitudeSensorCollector")
        requireNotNull(attitudeSensorCollector)
        val attitudeSensorCollectorSpy = spyk(attitudeSensorCollector)
        every { attitudeSensorCollectorSpy.getMeasurementsBeforePosition(any()) }.returns(
            ArrayDeque<AttitudeSensorMeasurement>()
        )
        syncer.setPrivateProperty("attitudeSensorCollector", attitudeSensorCollectorSpy)
        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "processing", true)

        val listener = attitudeSensorCollectorSpy.measurementListener
        requireNotNull(listener)

        listener.onMeasurement(attitudeSensorCollectorSpy, AttitudeSensorMeasurement(), 0)

        verify(exactly = 0) { attitudeSensorCollectorSpy.getMeasurementsBeforePosition(0) }
        verify { syncedMeasurementListener wasNot Called }
        assertNull(syncer.mostRecentTimestamp)
    }

    @Test
    fun attitudeSensorCollector_whenMeasurementAndNoMeasurementsAvailable_callsCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(
            context,
            syncedMeasurementListener = syncedMeasurementListener
        )

        val attitudeSensorCollector: BufferedAttitudeSensorCollector? =
            syncer.getPrivateProperty("attitudeSensorCollector")
        requireNotNull(attitudeSensorCollector)
        val attitudeSensorCollectorSpy = spyk(attitudeSensorCollector)
        every { attitudeSensorCollectorSpy.getMeasurementsBeforePosition(any()) }.returns(
            ArrayDeque<AttitudeSensorMeasurement>()
        )
        syncer.setPrivateProperty("attitudeSensorCollector", attitudeSensorCollectorSpy)

        val listener = attitudeSensorCollectorSpy.measurementListener
        requireNotNull(listener)

        listener.onMeasurement(attitudeSensorCollectorSpy, AttitudeSensorMeasurement(), 0)

        verify(exactly = 1) { attitudeSensorCollectorSpy.getMeasurementsBeforePosition(0) }
        verify { syncedMeasurementListener wasNot Called }
        assertNull(syncer.mostRecentTimestamp)
    }

    @Test
    fun attitudeSensorCollector_whenMeasurementAndMeasurementsAvailable_updatesMostRecentTimestampCopiesMeasurementsAndProcessesThem() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(
            context,
            syncedMeasurementListener = syncedMeasurementListener
        )

        assertNull(syncer.mostRecentTimestamp)
        assertNull(syncer.oldestTimestamp)

        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "running", true)
        assertTrue(syncer.running)

        val attitudeSensorCollector: BufferedAttitudeSensorCollector? =
            syncer.getPrivateProperty("attitudeSensorCollector")
        requireNotNull(attitudeSensorCollector)
        val attitudeSensorCollectorSpy = spyk(attitudeSensorCollector)

        val randomizer = UniformRandomizer()
        val attitude = getAttitude()
        val headingAccuracyRadians =
            Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val timestamp = System.nanoTime()
        val attitudeMeasurement = AttitudeSensorMeasurement(
            attitude,
            headingAccuracyRadians,
            timestamp,
            SensorAccuracy.HIGH,
            syncer.attitudeSensorType
        )
        val measurementsBeforePosition = ArrayDeque<AttitudeSensorMeasurement>()
        measurementsBeforePosition.add(attitudeMeasurement)
        every { attitudeSensorCollectorSpy.getMeasurementsBeforePosition(any()) }.returns(
            measurementsBeforePosition
        )
        syncer.setPrivateProperty("attitudeSensorCollector", attitudeSensorCollectorSpy)

        val listener = attitudeSensorCollectorSpy.measurementListener
        requireNotNull(listener)

        listener.onMeasurement(attitudeSensorCollectorSpy, AttitudeSensorMeasurement(), 0)

        verify(exactly = 1) { attitudeSensorCollectorSpy.getMeasurementsBeforePosition(0) }
        assertEquals(timestamp, syncer.mostRecentTimestamp)

        val attitudeMeasurements: ArrayDeque<AttitudeSensorMeasurement>? =
            syncer.getPrivateProperty("attitudeMeasurements")
        requireNotNull(attitudeMeasurements)
        assertEquals(1, attitudeMeasurements.size)
        val attitudeMeasurement2 = attitudeMeasurements.peek()
        requireNotNull(attitudeMeasurement2)
        assertNotSame(attitudeMeasurement, attitudeMeasurement2)
        assertEquals(attitude, attitudeMeasurement2.attitude)
        assertEquals(headingAccuracyRadians, attitudeMeasurement2.headingAccuracy)
        assertEquals(timestamp, attitudeMeasurement2.timestamp)
        assertEquals(SensorAccuracy.HIGH, attitudeMeasurement2.accuracy)
        assertEquals(syncer.attitudeSensorType, attitudeMeasurement2.sensorType)

        assertEquals(timestamp, syncer.oldestTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)

        val alreadyProcessedAttitudeMeasurements: ArrayDeque<AttitudeSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAttitudeMeasurements")
        requireNotNull(alreadyProcessedAttitudeMeasurements)
        assertTrue(alreadyProcessedAttitudeMeasurements.isEmpty())

        val alreadyProcessedAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())

        verify { syncedMeasurementListener wasNot Called }
        assertTrue(syncer.running)
    }

    @Test
    fun accelerometerSensorCollector_whenAccuracyChangedListenerAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(context)

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
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(
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
                SensorType.ACCELEROMETER_UNCALIBRATED,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun accelerometerSensorCollector_whenBufferFilledAndStopWhenFilledBuffer_stops() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(
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
                SensorType.ACCELEROMETER_UNCALIBRATED
            )
        }
        assertTrue(syncer.running)
    }

    @Test
    fun accelerometerSensorCollector_whenSkipWhenProcessingAndProcessing_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(
            context,
            skipWhenProcessing = true,
            syncedMeasurementListener = syncedMeasurementListener
        )

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)
        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "processing", true)

        val listener = accelerometerSensorCollector.measurementListener
        requireNotNull(listener)

        listener.onMeasurement(accelerometerSensorCollectorSpy, AccelerometerSensorMeasurement(), 0)

        verify { accelerometerSensorCollectorSpy wasNot Called }
        verify { syncedMeasurementListener wasNot Called }
        assertNull(syncer.mostRecentTimestamp)
    }

    @Test
    fun accelerometerSensorCollector_whenMeasurementAndMostRecentTimestampNotDefined_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(
            context,
            syncedMeasurementListener = syncedMeasurementListener
        )

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        val listener = accelerometerSensorCollector.measurementListener
        requireNotNull(listener)

        listener.onMeasurement(accelerometerSensorCollectorSpy, AccelerometerSensorMeasurement(), 0)

        verify { accelerometerSensorCollectorSpy wasNot Called }
        verify { syncedMeasurementListener wasNot Called }
        assertNull(syncer.mostRecentTimestamp)
    }

    @Test
    fun accelerometerSensorCollector_whenMeasurementMostRecentTimestampDefinedAndNoMeasurementsAvailable_callsCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(
            context,
            syncedMeasurementListener = syncedMeasurementListener
        )

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            ArrayDeque<AccelerometerSensorMeasurement>()
        )
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        // set most recent timestamp
        val mostRecentTimestamp = System.nanoTime()
        setPrivateProperty(
            SensorMeasurementSyncer::class,
            syncer,
            "mostRecentTimestamp",
            mostRecentTimestamp
        )

        val listener = accelerometerSensorCollector.measurementListener
        requireNotNull(listener)

        listener.onMeasurement(accelerometerSensorCollectorSpy, AccelerometerSensorMeasurement(), 0)

        verify(exactly = 1) {
            accelerometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(
                mostRecentTimestamp
            )
        }
        verify { syncedMeasurementListener wasNot Called }
        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
    }

    @Test
    fun accelerometerSensorCollector_whenMeasurementMostRecentTimestampDefinedAndMeasurementsAvailable_copiesMeasurements() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(
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
        val measurementsBeforeTimestamp = ArrayDeque<AccelerometerSensorMeasurement>()
        measurementsBeforeTimestamp.add(accelerometerMeasurement)
        every { accelerometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            measurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        // set most recent timestamp
        val mostRecentTimestamp = System.nanoTime()
        setPrivateProperty(
            SensorMeasurementSyncer::class,
            syncer,
            "mostRecentTimestamp",
            mostRecentTimestamp
        )

        val listener = accelerometerSensorCollector.measurementListener
        requireNotNull(listener)

        listener.onMeasurement(accelerometerSensorCollectorSpy, AccelerometerSensorMeasurement(), 0)

        verify(exactly = 1) {
            accelerometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(
                mostRecentTimestamp
            )
        }

        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
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
        assertEquals(timestamp, accelerometerMeasurement2.timestamp)
        assertEquals(SensorAccuracy.HIGH, accelerometerMeasurement2.accuracy)

        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
        assertNull(syncer.oldestTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)

        val alreadyProcessedAttitudeMeasurements: ArrayDeque<AttitudeSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAttitudeMeasurements")
        requireNotNull(alreadyProcessedAttitudeMeasurements)
        assertTrue(alreadyProcessedAttitudeMeasurements.isEmpty())

        val alreadyProcessedAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())

        verify { syncedMeasurementListener wasNot Called }
        assertTrue(syncer.running)
    }

    @Test
    fun sensorCollectors_whenAllMeasurements_notifiesSyncedMeasurement() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(
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
        val attitudeTimestamp = mostRecentTimestamp - 1
        setPrivateProperty(
            SensorMeasurementSyncer::class,
            syncer,
            "mostRecentTimestamp",
            mostRecentTimestamp
        )

        val randomizer = UniformRandomizer()

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
        val accelerometerTimestamp = attitudeTimestamp - 1
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
        val accelerometerMeasurementsBeforeTimestamp = ArrayDeque<AccelerometerSensorMeasurement>()
        accelerometerMeasurementsBeforeTimestamp.add(accelerometerMeasurement1)
        every { accelerometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            accelerometerMeasurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        val attitudeSensorCollector: BufferedAttitudeSensorCollector? =
            syncer.getPrivateProperty("attitudeSensorCollector")
        requireNotNull(attitudeSensorCollector)
        val attitudeSensorCollectorSpy = spyk(attitudeSensorCollector)

        val headingAccuracyRadians =
            Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val attitude = getAttitude()
        val attitudeMeasurement1 = AttitudeSensorMeasurement(
            attitude,
            headingAccuracyRadians,
            attitudeTimestamp,
            SensorAccuracy.HIGH,
            syncer.attitudeSensorType
        )
        val measurementsBeforePosition = ArrayDeque<AttitudeSensorMeasurement>()
        measurementsBeforePosition.add(attitudeMeasurement1)
        every { attitudeSensorCollectorSpy.getMeasurementsBeforePosition(any()) }.returns(
            measurementsBeforePosition
        )
        syncer.setPrivateProperty("attitudeSensorCollector", attitudeSensorCollectorSpy)

        val attitudeListener = attitudeSensorCollectorSpy.measurementListener
        requireNotNull(attitudeListener)

        val accelerometerListener = accelerometerSensorCollector.measurementListener
        requireNotNull(accelerometerListener)

        // process accelerometer measurement
        accelerometerListener.onMeasurement(
            accelerometerSensorCollectorSpy,
            AccelerometerSensorMeasurement(),
            0
        )

        assertNull(syncer.oldestTimestamp)
        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        assertTrue(syncer.running)

        val accelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("accelerometerMeasurements")
        requireNotNull(accelerometerMeasurements)
        assertEquals(1, accelerometerMeasurements.size)
        val accelerometerMeasurement2 = accelerometerMeasurements.peek()
        requireNotNull(accelerometerMeasurement2)
        assertNotSame(accelerometerMeasurement1, accelerometerMeasurement2)
        assertEquals(ax, accelerometerMeasurement2.ax, 0.0f)
        assertEquals(ay, accelerometerMeasurement2.ay, 0.0f)
        assertEquals(az, accelerometerMeasurement2.az, 0.0f)
        assertEquals(accelerometerTimestamp, accelerometerMeasurement2.timestamp)
        assertEquals(SensorAccuracy.HIGH, accelerometerMeasurement2.accuracy)

        val alreadyProcessedAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())

        // process attitude measurement
        attitudeListener.onMeasurement(
            attitudeSensorCollectorSpy,
            AttitudeSensorMeasurement(),
            0
        )

        assertEquals(attitudeTimestamp, syncer.oldestTimestamp)
        assertEquals(attitudeTimestamp, syncer.mostRecentTimestamp)
        assertEquals(1, syncer.numberOfProcessedMeasurements)
        assertTrue(syncer.running)
        verify(exactly = 1) { attitudeSensorCollectorSpy.getMeasurementsBeforePosition(0) }

        val attitudeMeasurements: ArrayDeque<AttitudeSensorMeasurement>? =
            syncer.getPrivateProperty("attitudeMeasurements")
        requireNotNull(attitudeMeasurements)
        assertTrue(attitudeMeasurements.isEmpty())

        val alreadyProcessedAttitudeMeasurements: ArrayDeque<AttitudeSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAttitudeMeasurements")
        requireNotNull(alreadyProcessedAttitudeMeasurements)
        assertTrue(alreadyProcessedAttitudeMeasurements.isEmpty())
        assertTrue(alreadyProcessedAttitudeMeasurements.isEmpty())

        val slot = slot<AttitudeAndAccelerometerSyncedSensorMeasurement>()
        verify(exactly = 1) {
            syncedMeasurementListener.onSyncedMeasurements(
                syncer,
                capture(slot)
            )
        }

        val syncedMeasurement = slot.captured
        assertEquals(accelerometerTimestamp, syncedMeasurement.timestamp)
        val syncedAttitudeMeasurement = syncedMeasurement.attitudeMeasurement
        requireNotNull(syncedAttitudeMeasurement)
        assertEquals(attitude, syncedAttitudeMeasurement.attitude)
        assertEquals(headingAccuracyRadians, syncedAttitudeMeasurement.headingAccuracy)
        assertEquals(accelerometerTimestamp, syncedAttitudeMeasurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, syncedAttitudeMeasurement.accuracy)
        assertEquals(syncer.attitudeSensorType, syncedAttitudeMeasurement.sensorType)
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
    }

    @Test
    fun cleanupStaleMeasurements_whenStaleMeasurementsAvailable_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(
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
        val attitudeTimestamp = mostRecentTimestamp - 1
        setPrivateProperty(
            SensorMeasurementSyncer::class,
            syncer,
            "mostRecentTimestamp",
            mostRecentTimestamp
        )

        // add stale measurements
        val staleTimestamp =
            mostRecentTimestamp - 2 * AttitudeAndAccelerometerSensorMeasurementSyncer.DEFAULT_STALE_OFFSET_NANOS
        val randomizer = UniformRandomizer()
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
            SensorAccuracy.MEDIUM
        )
        accelerometerMeasurements.add(staleAccelerometerMeasurement)

        val attitudeMeasurements: ArrayDeque<AttitudeSensorMeasurement>? =
            syncer.getPrivateProperty("attitudeMeasurements")
        requireNotNull(attitudeMeasurements)
        val attitude1 = getAttitude()
        val headingAccuracyRadians1 =
            Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val staleAttitudeMeasurement = AttitudeSensorMeasurement(
            attitude1,
            headingAccuracyRadians1,
            staleTimestamp,
            SensorAccuracy.HIGH,
            syncer.attitudeSensorType
        )
        attitudeMeasurements.add(staleAttitudeMeasurement)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)

        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val accelerometerTimestamp = attitudeTimestamp - 1
        val accelerometerMeasurement1 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            abx,
            aby,
            abz,
            accelerometerTimestamp,
            SensorAccuracy.MEDIUM
        )
        val accelerometerMeasurementsBeforeTimestamp = ArrayDeque<AccelerometerSensorMeasurement>()
        accelerometerMeasurementsBeforeTimestamp.add(accelerometerMeasurement1)
        every { accelerometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            accelerometerMeasurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        val attitudeSensorCollector: BufferedAttitudeSensorCollector? =
            syncer.getPrivateProperty("attitudeSensorCollector")
        requireNotNull(attitudeSensorCollector)
        val attitudeSensorCollectorSpy = spyk(attitudeSensorCollector)

        val attitude = getAttitude()
        val headingAccuracyRadians =
            Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val attitudeMeasurement1 = AttitudeSensorMeasurement(
            attitude,
            headingAccuracyRadians,
            attitudeTimestamp,
            SensorAccuracy.HIGH,
            syncer.attitudeSensorType
        )
        val measurementsBeforePosition = ArrayDeque<AttitudeSensorMeasurement>()
        measurementsBeforePosition.add(attitudeMeasurement1)
        every { attitudeSensorCollectorSpy.getMeasurementsBeforePosition(any()) }.returns(
            measurementsBeforePosition
        )
        syncer.setPrivateProperty("attitudeSensorCollector", attitudeSensorCollectorSpy)

        // set previous measurements
        syncer.setPrivateProperty("hasPreviousAttitudeMeasurement", true)
        val previousAttitudeMeasurement: AttitudeSensorMeasurement? =
            syncer.getPrivateProperty("previousAttitudeMeasurement")
        requireNotNull(previousAttitudeMeasurement)
        previousAttitudeMeasurement.timestamp = attitudeTimestamp - 1

        val attitudeListener = attitudeSensorCollectorSpy.measurementListener
        requireNotNull(attitudeListener)

        val accelerometerListener = accelerometerSensorCollectorSpy.measurementListener
        requireNotNull(accelerometerListener)

        // process accelerometer measurement
        accelerometerListener.onMeasurement(
            accelerometerSensorCollectorSpy,
            AccelerometerSensorMeasurement(),
            0
        )

        assertNull(syncer.oldestTimestamp)
        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        assertTrue(syncer.running)

        assertEquals(2, accelerometerMeasurements.size)
        val accelerometerMeasurement2 = accelerometerMeasurements.first
        requireNotNull(accelerometerMeasurement2)
        assertEquals(ax1, accelerometerMeasurement2.ax, 0.0f)
        assertEquals(ay1, accelerometerMeasurement2.ay, 0.0f)
        assertEquals(az1, accelerometerMeasurement2.az, 0.0f)
        assertEquals(abx, accelerometerMeasurement2.bx)
        assertEquals(aby, accelerometerMeasurement2.by)
        assertEquals(abz, accelerometerMeasurement2.bz)
        assertEquals(staleTimestamp, accelerometerMeasurement2.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, accelerometerMeasurement2.accuracy)
        val accelerometerMeasurement3 = accelerometerMeasurements.last
        requireNotNull(accelerometerMeasurement3)
        assertEquals(ax, accelerometerMeasurement3.ax, 0.0f)
        assertEquals(ay, accelerometerMeasurement3.ay, 0.0f)
        assertEquals(az, accelerometerMeasurement3.az, 0.0f)
        assertEquals(abx, accelerometerMeasurement3.bx)
        assertEquals(aby, accelerometerMeasurement3.by)
        assertEquals(abz, accelerometerMeasurement3.bz)
        assertEquals(accelerometerTimestamp, accelerometerMeasurement3.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, accelerometerMeasurement3.accuracy)

        val alreadyProcessedAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())

        // process attitude measurement
        attitudeListener.onMeasurement(
            attitudeSensorCollectorSpy,
            AttitudeSensorMeasurement(),
            0
        )

        assertEquals(staleTimestamp, syncer.oldestTimestamp)
        assertEquals(attitudeTimestamp, syncer.mostRecentTimestamp)
        assertEquals(2, syncer.numberOfProcessedMeasurements)
        assertTrue(syncer.running)
        verify(exactly = 1) { attitudeSensorCollectorSpy.getMeasurementsBeforePosition(0) }

        assertTrue(attitudeMeasurements.isEmpty())
        assertTrue(accelerometerMeasurements.isEmpty())

        val alreadyProcessedAttitudeMeasurements: ArrayDeque<AttitudeSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAttitudeMeasurements")
        requireNotNull(alreadyProcessedAttitudeMeasurements)
        assertTrue(alreadyProcessedAttitudeMeasurements.isEmpty())
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())

        verify(exactly = 1) {
            staleDetectedMeasurementsListener.onStaleMeasurements(
                syncer,
                SensorType.ABSOLUTE_ATTITUDE,
                any()
            )
        }
    }

    @Test
    fun cleanupStaleMeasurements_whenStaleDetectionDisabled_doesNotNotify() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(
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
        val attitudeTimestamp = mostRecentTimestamp - 1
        setPrivateProperty(
            SensorMeasurementSyncer::class,
            syncer,
            "mostRecentTimestamp",
            mostRecentTimestamp
        )

        // add stale measurements
        val staleTimestamp =
            mostRecentTimestamp - 2 * AttitudeAndAccelerometerSensorMeasurementSyncer.DEFAULT_STALE_OFFSET_NANOS
        val randomizer = UniformRandomizer()
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
            SensorAccuracy.MEDIUM
        )
        accelerometerMeasurements.add(staleAccelerometerMeasurement)

        val attitudeMeasurements: ArrayDeque<AttitudeSensorMeasurement>? =
            syncer.getPrivateProperty("attitudeMeasurements")
        requireNotNull(attitudeMeasurements)
        val attitude1 = getAttitude()
        val headingAccuracyRadians1 =
            Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val staleAttitudeMeasurement = AttitudeSensorMeasurement(
            attitude1,
            headingAccuracyRadians1,
            staleTimestamp,
            SensorAccuracy.HIGH,
            syncer.attitudeSensorType
        )
        attitudeMeasurements.add(staleAttitudeMeasurement)

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)

        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val accelerometerTimestamp = attitudeTimestamp - 1
        val accelerometerMeasurement1 = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            abx,
            aby,
            abz,
            accelerometerTimestamp,
            SensorAccuracy.MEDIUM
        )
        val accelerometerMeasurementsBeforeTimestamp = ArrayDeque<AccelerometerSensorMeasurement>()
        accelerometerMeasurementsBeforeTimestamp.add(accelerometerMeasurement1)
        every { accelerometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            accelerometerMeasurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        val attitudeSensorCollector: BufferedAttitudeSensorCollector? =
            syncer.getPrivateProperty("attitudeSensorCollector")
        requireNotNull(attitudeSensorCollector)
        val attitudeSensorCollectorSpy = spyk(attitudeSensorCollector)

        val attitude = getAttitude()
        val headingAccuracyRadians =
            Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val attitudeMeasurement1 = AttitudeSensorMeasurement(
            attitude,
            headingAccuracyRadians,
            attitudeTimestamp,
            SensorAccuracy.HIGH,
            syncer.attitudeSensorType
        )
        val measurementsBeforePosition = ArrayDeque<AttitudeSensorMeasurement>()
        measurementsBeforePosition.add(attitudeMeasurement1)
        every { attitudeSensorCollectorSpy.getMeasurementsBeforePosition(any()) }.returns(
            measurementsBeforePosition
        )
        syncer.setPrivateProperty("attitudeSensorCollector", attitudeSensorCollectorSpy)

        // set previous measurements
        syncer.setPrivateProperty("hasPreviousAttitudeMeasurement", true)
        val previousAttitudeMeasurement: AttitudeSensorMeasurement? =
            syncer.getPrivateProperty("previousAttitudeMeasurement")
        requireNotNull(previousAttitudeMeasurement)
        previousAttitudeMeasurement.timestamp = attitudeTimestamp - 1

        val attitudeListener = attitudeSensorCollectorSpy.measurementListener
        requireNotNull(attitudeListener)

        val accelerometerListener = accelerometerSensorCollectorSpy.measurementListener
        requireNotNull(accelerometerListener)

        // process accelerometer measurement
        accelerometerListener.onMeasurement(
            accelerometerSensorCollectorSpy,
            AccelerometerSensorMeasurement(),
            0
        )

        assertNull(syncer.oldestTimestamp)
        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        assertTrue(syncer.running)

        assertEquals(2, accelerometerMeasurements.size)
        val accelerometerMeasurement2 = accelerometerMeasurements.first
        requireNotNull(accelerometerMeasurement2)
        assertEquals(ax1, accelerometerMeasurement2.ax, 0.0f)
        assertEquals(ay1, accelerometerMeasurement2.ay, 0.0f)
        assertEquals(az1, accelerometerMeasurement2.az, 0.0f)
        assertEquals(abx, accelerometerMeasurement2.bx)
        assertEquals(aby, accelerometerMeasurement2.by)
        assertEquals(abz, accelerometerMeasurement2.bz)
        assertEquals(staleTimestamp, accelerometerMeasurement2.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, accelerometerMeasurement2.accuracy)
        val accelerometerMeasurement3 = accelerometerMeasurements.last
        requireNotNull(accelerometerMeasurement3)
        assertEquals(ax, accelerometerMeasurement3.ax, 0.0f)
        assertEquals(ay, accelerometerMeasurement3.ay, 0.0f)
        assertEquals(az, accelerometerMeasurement3.az, 0.0f)
        assertEquals(abx, accelerometerMeasurement3.bx)
        assertEquals(aby, accelerometerMeasurement3.by)
        assertEquals(abz, accelerometerMeasurement3.bz)
        assertEquals(accelerometerTimestamp, accelerometerMeasurement3.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, accelerometerMeasurement3.accuracy)

        val alreadyProcessedAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())

        // process attitude measurement
        attitudeListener.onMeasurement(
            attitudeSensorCollectorSpy,
            AttitudeSensorMeasurement(),
            0
        )

        assertEquals(staleTimestamp, syncer.oldestTimestamp)
        assertEquals(attitudeTimestamp, syncer.mostRecentTimestamp)
        assertEquals(2, syncer.numberOfProcessedMeasurements)
        assertTrue(syncer.running)
        verify(exactly = 1) { attitudeSensorCollectorSpy.getMeasurementsBeforePosition(0) }

        assertEquals(1, attitudeMeasurements.size)
        val attitudeMeasurement2 = attitudeMeasurements.first()
        requireNotNull(attitudeMeasurement2)
        assertSame(staleAttitudeMeasurement, attitudeMeasurement2)
        assertTrue(accelerometerMeasurements.isEmpty())

        val alreadyProcessedAttitudeMeasurements: ArrayDeque<AttitudeSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAttitudeMeasurements")
        requireNotNull(alreadyProcessedAttitudeMeasurements)
        assertFalse(alreadyProcessedAttitudeMeasurements.isEmpty())
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())

        verify { staleDetectedMeasurementsListener wasNot Called }
    }

    @Test
    fun sensorCollectors_whenPreviousAccelerometerMeasurement_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(
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
        val attitudeTimestamp = mostRecentTimestamp - 1
        setPrivateProperty(
            SensorMeasurementSyncer::class,
            syncer,
            "mostRecentTimestamp",
            mostRecentTimestamp
        )

        // set previous attitude, accelerometer and gyroscope timestamp
        syncer.setPrivateProperty("hasPreviousAttitudeMeasurement", true)
        val previousAttitudeMeasurement: AttitudeSensorMeasurement? =
            syncer.getPrivateProperty("previousAttitudeMeasurement")
        requireNotNull(previousAttitudeMeasurement)
        previousAttitudeMeasurement.timestamp = mostRecentTimestamp - 1
        syncer.setPrivateProperty("lastNotifiedTimestamp", attitudeTimestamp - 1)
        syncer.setPrivateProperty("lastNotifiedAttitudeTimestamp", attitudeTimestamp - 1)
        syncer.setPrivateProperty("lastNotifiedAccelerometerTimestamp", attitudeTimestamp - 2)
        syncer.setPrivateProperty("hasPreviousAccelerometerMeasurement", true)
        val previousAccelerometerMeasurement: AccelerometerSensorMeasurement? =
            syncer.getPrivateProperty("previousAccelerometerMeasurement")
        requireNotNull(previousAccelerometerMeasurement)
        val randomizer = UniformRandomizer()
        val ax1 = randomizer.nextFloat()
        val ay1 = randomizer.nextFloat()
        val az1 = randomizer.nextFloat()
        val abx = randomizer.nextFloat()
        val aby = randomizer.nextFloat()
        val abz = randomizer.nextFloat()
        previousAccelerometerMeasurement.ax = ax1
        previousAccelerometerMeasurement.ay = ay1
        previousAccelerometerMeasurement.az = az1
        previousAccelerometerMeasurement.bx = abx
        previousAccelerometerMeasurement.by = aby
        previousAccelerometerMeasurement.bz = abz
        previousAccelerometerMeasurement.timestamp = attitudeTimestamp - 1
        previousAccelerometerMeasurement.accuracy = SensorAccuracy.HIGH

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)

        val ax2 = randomizer.nextFloat()
        val ay2 = randomizer.nextFloat()
        val az2 = randomizer.nextFloat()
        val accelerometerTimestamp = attitudeTimestamp - 1
        val accelerometerMeasurement1 = AccelerometerSensorMeasurement(
            ax2,
            ay2,
            az2,
            abx,
            aby,
            abz,
            accelerometerTimestamp,
            SensorAccuracy.HIGH
        )
        val accelerometerMeasurementsBeforeTimestamp = ArrayDeque<AccelerometerSensorMeasurement>()
        accelerometerMeasurementsBeforeTimestamp.add(accelerometerMeasurement1)
        every { accelerometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            accelerometerMeasurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        val attitudeSensorCollector: BufferedAttitudeSensorCollector? =
            syncer.getPrivateProperty("attitudeSensorCollector")
        requireNotNull(attitudeSensorCollector)
        val attitudeSensorCollectorSpy = spyk(attitudeSensorCollector)

        val attitude = getAttitude()
        val headingAccuracyRadians =
            Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val attitudeMeasurement1 = AttitudeSensorMeasurement(
            attitude,
            headingAccuracyRadians,
            attitudeTimestamp,
            SensorAccuracy.HIGH,
            syncer.attitudeSensorType
        )
        val measurementsBeforePosition = ArrayDeque<AttitudeSensorMeasurement>()
        measurementsBeforePosition.add(attitudeMeasurement1)
        every { attitudeSensorCollectorSpy.getMeasurementsBeforePosition(any()) }.returns(
            measurementsBeforePosition
        )
        syncer.setPrivateProperty("attitudeSensorCollector", attitudeSensorCollectorSpy)

        val attitudeListener = attitudeSensorCollectorSpy.measurementListener
        requireNotNull(attitudeListener)

        val accelerometerListener = accelerometerSensorCollectorSpy.measurementListener
        requireNotNull(accelerometerListener)

        // process accelerometer measurement
        accelerometerListener.onMeasurement(
            accelerometerSensorCollectorSpy,
            AccelerometerSensorMeasurement(),
            0
        )

        assertNull(syncer.oldestTimestamp)
        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        assertTrue(syncer.running)

        val accelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("accelerometerMeasurements")
        requireNotNull(accelerometerMeasurements)
        assertEquals(1, accelerometerMeasurements.size)
        val accelerometerMeasurement2 = accelerometerMeasurements.peek()
        requireNotNull(accelerometerMeasurement2)
        assertNotSame(accelerometerMeasurement1, accelerometerMeasurement2)
        assertEquals(ax2, accelerometerMeasurement2.ax, 0.0f)
        assertEquals(ay2, accelerometerMeasurement2.ay, 0.0f)
        assertEquals(az2, accelerometerMeasurement2.az, 0.0f)
        assertEquals(abx, accelerometerMeasurement2.bx)
        assertEquals(aby, accelerometerMeasurement2.by)
        assertEquals(abz, accelerometerMeasurement2.bz)
        assertEquals(accelerometerTimestamp, accelerometerMeasurement2.timestamp)
        assertEquals(SensorAccuracy.HIGH, accelerometerMeasurement2.accuracy)

        val alreadyProcessedAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())

        // process attitude measurement
        attitudeListener.onMeasurement(
            attitudeSensorCollectorSpy,
            AttitudeSensorMeasurement(),
            0
        )

        assertEquals(attitudeTimestamp, syncer.oldestTimestamp)
        assertEquals(attitudeTimestamp, syncer.mostRecentTimestamp)
        assertEquals(1, syncer.numberOfProcessedMeasurements)
        assertTrue(syncer.running)
        verify(exactly = 1) { attitudeSensorCollectorSpy.getMeasurementsBeforePosition(0) }

        val attitudeMeasurements: ArrayDeque<AttitudeSensorMeasurement>? =
            syncer.getPrivateProperty("attitudeMeasurements")
        requireNotNull(attitudeMeasurements)
        assertTrue(attitudeMeasurements.isEmpty())
        assertEquals(1, accelerometerMeasurements.size)
        assertSame(accelerometerMeasurement2, accelerometerMeasurements.first())

        val alreadyProcessedAttitudeMeasurements: ArrayDeque<AttitudeSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAttitudeMeasurements")
        requireNotNull(alreadyProcessedAttitudeMeasurements)
        assertTrue(alreadyProcessedAttitudeMeasurements.isEmpty())
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())

        val slot = slot<AttitudeAndAccelerometerSyncedSensorMeasurement>()
        verify(exactly = 1) {
            syncedMeasurementListener.onSyncedMeasurements(
                syncer,
                capture(slot)
            )
        }

        val syncedMeasurement = slot.captured
        assertEquals(attitudeTimestamp, syncedMeasurement.timestamp)
        val syncedAttitudeMeasurement = syncedMeasurement.attitudeMeasurement
        requireNotNull(syncedAttitudeMeasurement)
        assertEquals(attitude, syncedAttitudeMeasurement.attitude)
        assertEquals(headingAccuracyRadians, syncedAttitudeMeasurement.headingAccuracy)
        assertEquals(attitudeTimestamp, syncedAttitudeMeasurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, syncedAttitudeMeasurement.accuracy)
        assertEquals(syncer.attitudeSensorType, syncedAttitudeMeasurement.sensorType)
        val syncedAccelerometerMeasurement = syncedMeasurement.accelerometerMeasurement
        requireNotNull(syncedAccelerometerMeasurement)
        assertEquals(ax1, syncedAccelerometerMeasurement.ax, 0.0f)
        assertEquals(ay1, syncedAccelerometerMeasurement.ay, 0.0f)
        assertEquals(az1, syncedAccelerometerMeasurement.az, 0.0f)
        assertEquals(abx, syncedAccelerometerMeasurement.bx)
        assertEquals(aby, syncedAccelerometerMeasurement.by)
        assertEquals(abz, syncedAccelerometerMeasurement.bz)
        assertEquals(attitudeTimestamp, syncedAccelerometerMeasurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, syncedAccelerometerMeasurement.accuracy)
    }

    @Test
    fun sensorCollectors_whenStopping_clearCollectionsAndResets() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAndAccelerometerSensorMeasurementSyncer(
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
        val attitudeTimestamp = mostRecentTimestamp - 1
        setPrivateProperty(
            SensorMeasurementSyncer::class,
            syncer,
            "mostRecentTimestamp",
            mostRecentTimestamp
        )

        val randomizer = UniformRandomizer()
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
        val accelerometerTimestamp = attitudeTimestamp - 1
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
        val accelerometerMeasurementsBeforeTimestamp = ArrayDeque<AccelerometerSensorMeasurement>()
        accelerometerMeasurementsBeforeTimestamp.add(accelerometerMeasurement1)
        every { accelerometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            accelerometerMeasurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        val attitudeSensorCollector: BufferedAttitudeSensorCollector? =
            syncer.getPrivateProperty("attitudeSensorCollector")
        requireNotNull(attitudeSensorCollector)
        val attitudeSensorCollectorSpy = spyk(attitudeSensorCollector)

        val attitude = getAttitude()
        val headingAccuracyRadians =
            Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val attitudeMeasurement1 = AttitudeSensorMeasurement(
            attitude,
            headingAccuracyRadians,
            attitudeTimestamp,
            SensorAccuracy.HIGH,
            syncer.attitudeSensorType
        )
        val measurementsBeforePosition = ArrayDeque<AttitudeSensorMeasurement>()
        measurementsBeforePosition.add(attitudeMeasurement1)
        every { attitudeSensorCollectorSpy.getMeasurementsBeforePosition(any()) }.returns(
            measurementsBeforePosition
        )
        syncer.setPrivateProperty("attitudeSensorCollector", attitudeSensorCollectorSpy)

        val attitudeListener = attitudeSensorCollectorSpy.measurementListener
        requireNotNull(attitudeListener)

        val accelerometerListener = accelerometerSensorCollector.measurementListener
        requireNotNull(accelerometerListener)

        // process accelerometer measurement
        accelerometerListener.onMeasurement(
            accelerometerSensorCollectorSpy,
            AccelerometerSensorMeasurement(),
            0
        )

        assertNull(syncer.oldestTimestamp)
        assertEquals(mostRecentTimestamp, syncer.mostRecentTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        assertTrue(syncer.running)

        val accelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("accelerometerMeasurements")
        requireNotNull(accelerometerMeasurements)
        assertEquals(1, accelerometerMeasurements.size)
        val accelerometerMeasurement2 = accelerometerMeasurements.peek()
        requireNotNull(accelerometerMeasurement2)
        assertNotSame(accelerometerMeasurement1, accelerometerMeasurement2)
        assertEquals(ax, accelerometerMeasurement2.ax, 0.0f)
        assertEquals(ay, accelerometerMeasurement2.ay, 0.0f)
        assertEquals(az, accelerometerMeasurement2.az, 0.0f)
        assertEquals(abx, accelerometerMeasurement2.bx)
        assertEquals(aby, accelerometerMeasurement2.by)
        assertEquals(abz, accelerometerMeasurement2.bz)
        assertEquals(accelerometerTimestamp, accelerometerMeasurement2.timestamp)
        assertEquals(SensorAccuracy.HIGH, accelerometerMeasurement2.accuracy)

        val alreadyProcessedAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())

        // process attitude measurement
        attitudeListener.onMeasurement(
            attitudeSensorCollectorSpy,
            AttitudeSensorMeasurement(),
            0
        )

        verify(exactly = 1) { attitudeSensorCollectorSpy.getMeasurementsBeforePosition(0) }

        assertNull(syncer.oldestTimestamp)
        assertNull(syncer.mostRecentTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        assertFalse(syncer.running)

        val attitudeMeasurements: ArrayDeque<AttitudeSensorMeasurement>? =
            syncer.getPrivateProperty("attitudeMeasurements")
        requireNotNull(attitudeMeasurements)
        val alreadyProcessedAttitudeMeasurements: ArrayDeque<AttitudeSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAttitudeMeasurements")
        requireNotNull(alreadyProcessedAttitudeMeasurements)
        val foundAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("foundAccelerometerMeasurements")
        requireNotNull(foundAccelerometerMeasurements)
        assertTrue(attitudeMeasurements.isEmpty())
        assertTrue(accelerometerMeasurements.isEmpty())
        assertTrue(alreadyProcessedAttitudeMeasurements.isEmpty())
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())
        assertTrue(foundAccelerometerMeasurements.isEmpty())

        val slot = slot<AttitudeAndAccelerometerSyncedSensorMeasurement>()
        verify(exactly = 1) {
            syncedMeasurementListener.onSyncedMeasurements(
                syncer,
                capture(slot)
            )
        }

        val syncedMeasurement = slot.captured
        assertEquals(accelerometerTimestamp, syncedMeasurement.timestamp)
        val syncedAttitudeMeasurement = syncedMeasurement.attitudeMeasurement
        requireNotNull(syncedAttitudeMeasurement)
        assertEquals(attitude, syncedAttitudeMeasurement.attitude)
        assertEquals(headingAccuracyRadians, syncedAttitudeMeasurement.headingAccuracy)
        assertEquals(accelerometerTimestamp, syncedAttitudeMeasurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, syncedAttitudeMeasurement.accuracy)
        assertEquals(syncer.attitudeSensorType, syncedAttitudeMeasurement.sensorType)
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
    }

    private companion object {
        const val MIN_DEGREES = -90.0

        const val MAX_DEGREES = 90.0

        fun getAttitude(): Quaternion {
            val randomizer = UniformRandomizer()
            val roll = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
            val pitch = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
            val yaw = Math.toRadians(randomizer.nextDouble(MIN_DEGREES, MAX_DEGREES))
            return Quaternion(roll, pitch, yaw)
        }
    }
}