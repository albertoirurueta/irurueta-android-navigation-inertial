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
import com.irurueta.android.navigation.inertial.collectors.interpolators.*
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.android.testutils.setPrivateProperty
import com.irurueta.statistics.UniformRandomizer
import io.mockk.*
import io.mockk.impl.annotations.MockK
import io.mockk.junit4.MockKRule
import org.junit.After
import org.junit.Assert.*
import org.junit.Ignore
import org.junit.Rule
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import java.util.*

@Ignore("possible memory leak")
@RunWith(RobolectricTestRunner::class)
class AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncerTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK(relaxUnitFun = true)
    private lateinit var accuracyChangedListener:
            SensorMeasurementSyncer.OnAccuracyChangedListener<AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorMeasurement, AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer>

    @MockK(relaxUnitFun = true)
    private lateinit var bufferFilledListener:
            SensorMeasurementSyncer.OnBufferFilledListener<AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorMeasurement, AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer>

    @MockK(relaxUnitFun = true)
    private lateinit var syncedMeasurementListener:
            SensorMeasurementSyncer.OnSyncedMeasurementsListener<AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorMeasurement, AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer>

    @MockK(relaxUnitFun = true)
    private lateinit var staleDetectedMeasurementsListener:
            SensorMeasurementSyncer.OnStaleDetectedMeasurementsListener<AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorMeasurement, AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer>

    @After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
    }

    @Test
    fun constructor_whenRequiredParameters_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

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
        assertEquals(SensorDelay.FASTEST, syncer.gravitySensorDelay)
        assertEquals(SensorDelay.FASTEST, syncer.gyroscopeSensorDelay)
        assertEquals(SensorDelay.FASTEST, syncer.magnetometerSensorDelay)
        assertEquals(
            AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer.DEFAULT_ACCELEROMETER_CAPACITY,
            syncer.accelerometerCapacity
        )
        assertEquals(
            AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer.DEFAULT_GRAVITY_CAPACITY,
            syncer.gravityCapacity
        )
        assertEquals(
            AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer.DEFAULT_GYROSCOPE_CAPACITY,
            syncer.gyroscopeCapacity
        )
        assertEquals(
            AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer.DEFAULT_MAGNETOMETER_CAPACITY,
            syncer.magnetometerCapacity
        )
        assertFalse(syncer.accelerometerStartOffsetEnabled)
        assertFalse(syncer.gravityStartOffsetEnabled)
        assertFalse(syncer.gyroscopeStartOffsetEnabled)
        assertFalse(syncer.magnetometerStartOffsetEnabled)
        assertTrue(syncer.stopWhenFilledBuffer)
        assertEquals(
            AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer.DEFAULT_STALE_OFFSET_NANOS,
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
        assertNotNull(syncer.gravityInterpolator)
        assertTrue(syncer.gravityInterpolator is GravityQuadraticSensorMeasurementInterpolator)
        assertNotNull(syncer.gyroscopeInterpolator)
        assertTrue(syncer.gyroscopeInterpolator is GyroscopeQuadraticSensorMeasurementInterpolator)
        assertNotNull(syncer.magnetometerInterpolator)
        assertTrue(syncer.magnetometerInterpolator is MagnetometerQuadraticSensorMeasurementInterpolator)
        assertEquals(0L, syncer.startTimestamp)
        assertFalse(syncer.running)
        assertFalse(syncer.processing)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        assertNull(syncer.mostRecentTimestamp)
        assertNull(syncer.oldestTimestamp)
        assertNull(syncer.accelerometerSensor)
        assertNull(syncer.gravitySensor)
        assertNull(syncer.gyroscopeSensor)
        assertNull(syncer.magnetometerSensor)
        assertFalse(syncer.accelerometerSensorAvailable)
        assertFalse(syncer.gravitySensorAvailable)
        assertFalse(syncer.gyroscopeSensorAvailable)
        assertFalse(syncer.magnetometerSensorAvailable)
        assertNull(syncer.accelerometerStartOffset)
        assertNull(syncer.gravityStartOffset)
        assertNull(syncer.gyroscopeStartOffset)
        assertNull(syncer.magnetometerStartOffset)
        assertEquals(0.0f, syncer.accelerometerCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.gravityCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.gyroscopeCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.magnetometerCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.accelerometerUsage, 0.0f)
        assertEquals(0.0f, syncer.gravityUsage, 0.0f)
        assertEquals(0.0f, syncer.gyroscopeUsage, 0.0f)
        assertEquals(0.0f, syncer.magnetometerUsage, 0.0f)
    }

    @Test(expected = IllegalArgumentException::class)
    fun constructor_whenZeroAccelerometerCapacity_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
            context,
            accelerometerCapacity = 0
        )
    }

    @Test(expected = IllegalArgumentException::class)
    fun constructor_whenZeroGravityCapacity_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
            context,
            gravityCapacity = 0
        )
    }

    @Test(expected = IllegalArgumentException::class)
    fun constructor_whenZeroGyroscopeCapacity_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
            context,
            gyroscopeCapacity = 0
        )
    }

    @Test(expected = IllegalArgumentException::class)
    fun constructor_whenZeroMagnetometerCapacity_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
            context,
            magnetometerCapacity = 0
        )
    }

    @Test
    fun constructor_whenAllParameters_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val accelerometerInterpolator = AccelerometerDirectSensorMeasurementInterpolator()
        val gravityInterpolator = GravityDirectSensorMeasurementInterpolator()
        val gyroscopeInterpolator = GyroscopeDirectSensorMeasurementInterpolator()
        val magnetometerInterpolator = MagnetometerDirectSensorMeasurementInterpolator()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
            context,
            AccelerometerSensorType.ACCELEROMETER,
            GyroscopeSensorType.GYROSCOPE,
            MagnetometerSensorType.MAGNETOMETER,
            SensorDelay.NORMAL,
            SensorDelay.UI,
            SensorDelay.GAME,
            SensorDelay.NORMAL,
            accelerometerCapacity = 2,
            gravityCapacity = 4,
            gyroscopeCapacity = 3,
            magnetometerCapacity = 5,
            accelerometerStartOffsetEnabled = true,
            gravityStartOffsetEnabled = true,
            gyroscopeStartOffsetEnabled = true,
            magnetometerStartOffsetEnabled = true,
            stopWhenFilledBuffer = false,
            staleOffsetNanos = 123456789L,
            staleDetectionEnabled = false,
            skipWhenProcessing = false,
            accuracyChangedListener = accuracyChangedListener,
            bufferFilledListener = bufferFilledListener,
            syncedMeasurementListener = syncedMeasurementListener,
            staleDetectedMeasurementsListener = staleDetectedMeasurementsListener,
            accelerometerInterpolator = accelerometerInterpolator,
            gravityInterpolator = gravityInterpolator,
            gyroscopeInterpolator = gyroscopeInterpolator,
            magnetometerInterpolator = magnetometerInterpolator
        )

        // check
        assertSame(context, syncer.context)
        assertEquals(AccelerometerSensorType.ACCELEROMETER, syncer.accelerometerSensorType)
        assertEquals(GyroscopeSensorType.GYROSCOPE, syncer.gyroscopeSensorType)
        assertEquals(MagnetometerSensorType.MAGNETOMETER, syncer.magnetometerSensorType)
        assertEquals(SensorDelay.NORMAL, syncer.accelerometerSensorDelay)
        assertEquals(SensorDelay.UI, syncer.gravitySensorDelay)
        assertEquals(SensorDelay.GAME, syncer.gyroscopeSensorDelay)
        assertEquals(SensorDelay.NORMAL, syncer.magnetometerSensorDelay)
        assertEquals(2, syncer.accelerometerCapacity)
        assertEquals(4, syncer.gravityCapacity)
        assertEquals(3, syncer.gyroscopeCapacity)
        assertEquals(5, syncer.magnetometerCapacity)
        assertTrue(syncer.accelerometerStartOffsetEnabled)
        assertTrue(syncer.gravityStartOffsetEnabled)
        assertTrue(syncer.gyroscopeStartOffsetEnabled)
        assertTrue(syncer.magnetometerStartOffsetEnabled)
        assertFalse(syncer.stopWhenFilledBuffer)
        assertEquals(123456789L, syncer.staleOffsetNanos)
        assertFalse(syncer.staleDetectionEnabled)
        assertFalse(syncer.skipWhenProcessing)
        assertSame(accuracyChangedListener, syncer.accuracyChangedListener)
        assertSame(bufferFilledListener, syncer.bufferFilledListener)
        assertSame(syncedMeasurementListener, syncer.syncedMeasurementListener)
        assertSame(staleDetectedMeasurementsListener, syncer.staleDetectedMeasurementsListener)
        assertSame(accelerometerInterpolator, syncer.accelerometerInterpolator)
        assertSame(gravityInterpolator, syncer.gravityInterpolator)
        assertSame(gyroscopeInterpolator, syncer.gyroscopeInterpolator)
        assertSame(magnetometerInterpolator, syncer.magnetometerInterpolator)
        assertEquals(0L, syncer.startTimestamp)
        assertFalse(syncer.running)
        assertFalse(syncer.processing)
        assertEquals(0, syncer.numberOfProcessedMeasurements)
        assertNull(syncer.mostRecentTimestamp)
        assertNull(syncer.oldestTimestamp)
        assertNull(syncer.accelerometerSensor)
        assertNull(syncer.gravitySensor)
        assertNull(syncer.gyroscopeSensor)
        assertNull(syncer.magnetometerSensor)
        assertFalse(syncer.accelerometerSensorAvailable)
        assertFalse(syncer.gravitySensorAvailable)
        assertFalse(syncer.gyroscopeSensorAvailable)
        assertFalse(syncer.magnetometerSensorAvailable)
        assertNull(syncer.accelerometerStartOffset)
        assertNull(syncer.gravityStartOffset)
        assertNull(syncer.gyroscopeStartOffset)
        assertNull(syncer.magnetometerStartOffset)
        assertEquals(0.0f, syncer.accelerometerCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.gravityCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.gyroscopeCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.magnetometerCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.accelerometerUsage, 0.0f)
        assertEquals(0.0f, syncer.gravityUsage, 0.0f)
        assertEquals(0.0f, syncer.gyroscopeUsage, 0.0f)
        assertEquals(0.0f, syncer.magnetometerUsage, 0.0f)
    }

    @Test
    fun accuracyChangedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

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
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

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
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

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
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

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
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

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
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)

        assertNull(syncer.gravitySensor)
        verify(exactly = 1) { gravitySensorCollectorSpy.sensor }
    }

    @Test
    fun gyroscopeSensor_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

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
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

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
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

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
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)

        assertFalse(syncer.gravitySensorAvailable)
        verify(exactly = 1) { gravitySensorCollectorSpy.sensorAvailable }
    }

    @Test
    fun gyroscopeSensorAvailable_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

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
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

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
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

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
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)

        assertNull(syncer.gravityStartOffset)
        verify(exactly = 1) { gravitySensorCollectorSpy.startOffset }
    }

    @Test
    fun gyroscopeStartOffset_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

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
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

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
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

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
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)

        assertEquals(0.0f, syncer.gravityCollectorUsage, 0.0f)
        verify(exactly = 1) { gravitySensorCollectorSpy.usage }
    }

    @Test
    fun gyroscopeCollectorUsage_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

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
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

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
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

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
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

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
    fun gyroscopeUsage_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

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
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

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
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

        // set as running
        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "running", true)
        assertTrue(syncer.running)
        assertFalse(syncer.processing)

        syncer.start()
    }

    @Test
    fun start_whenNotRunningAndNoTimestamp_clearsResetsAndSetsCurrentStartTimestamp() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

        assertFalse(syncer.running)
        assertFalse(syncer.processing)
        assertEquals(0L, syncer.startTimestamp)

        // set variables that will be later reset
        val accelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("accelerometerMeasurements")
        requireNotNull(accelerometerMeasurements)
        accelerometerMeasurements.add(AccelerometerSensorMeasurement())

        val gravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("gravityMeasurements")
        requireNotNull(gravityMeasurements)
        gravityMeasurements.add(GravitySensorMeasurement())

        val gyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("gyroscopeMeasurements")
        requireNotNull(gyroscopeMeasurements)
        gyroscopeMeasurements.add(GyroscopeSensorMeasurement())

        val magnetometerMeasurements: ArrayDeque<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("magnetometerMeasurements")
        requireNotNull(magnetometerMeasurements)
        magnetometerMeasurements.add(MagnetometerSensorMeasurement())

        val alreadyProcessedAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        alreadyProcessedAccelerometerMeasurements.add(AccelerometerSensorMeasurement())

        val alreadyProcessedGravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGravityMeasurements")
        requireNotNull(alreadyProcessedGravityMeasurements)
        alreadyProcessedGravityMeasurements.add(GravitySensorMeasurement())

        val alreadyProcessedGyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGyroscopeMeasurements")
        requireNotNull(alreadyProcessedGyroscopeMeasurements)
        alreadyProcessedGyroscopeMeasurements.add(GyroscopeSensorMeasurement())

        val alreadyProcessedMagnetometerMeasurements: ArrayDeque<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedMagnetometerMeasurements")
        requireNotNull(alreadyProcessedMagnetometerMeasurements)
        alreadyProcessedMagnetometerMeasurements.add(MagnetometerSensorMeasurement())

        val foundGravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("foundGravityMeasurements")
        requireNotNull(foundGravityMeasurements)
        foundGravityMeasurements.add(GravitySensorMeasurement())

        val foundGyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("foundGyroscopeMeasurements")
        requireNotNull(foundGyroscopeMeasurements)
        foundGyroscopeMeasurements.add(GyroscopeSensorMeasurement())

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

        syncer.setPrivateProperty("hasPreviousAccelerometerMeasurement", true)
        syncer.setPrivateProperty("hasPreviousGravityMeasurement", true)
        syncer.setPrivateProperty("hasPreviousGyroscopeMeasurement", true)
        syncer.setPrivateProperty("hasPreviousMagnetometerMeasurement", true)
        syncer.setPrivateProperty("lastNotifiedTimestamp", 1L)
        syncer.setPrivateProperty("lastNotifiedAccelerometerTimestamp", 2L)
        syncer.setPrivateProperty("lastNotifiedGravityTimestamp", 3L)
        syncer.setPrivateProperty("lastNotifiedGyroscopeTimestamp", 4L)
        syncer.setPrivateProperty("lastNotifiedMagnetometerTimestamp", 5L)

        assertFalse(syncer.running)
        assertFalse(syncer.processing)
        assertEquals(0L, syncer.startTimestamp)

        assertFalse(syncer.start())

        // check
        assertFalse(syncer.running)
        assertFalse(syncer.processing)
        assertNotEquals(0L, syncer.startTimestamp)

        assertTrue(accelerometerMeasurements.isEmpty())
        assertTrue(gravityMeasurements.isEmpty())
        assertTrue(gyroscopeMeasurements.isEmpty())
        assertTrue(magnetometerMeasurements.isEmpty())
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())
        assertTrue(alreadyProcessedGravityMeasurements.isEmpty())
        assertTrue(alreadyProcessedGyroscopeMeasurements.isEmpty())
        assertTrue(alreadyProcessedMagnetometerMeasurements.isEmpty())
        assertTrue(foundGravityMeasurements.isEmpty())
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
        assertFalse(syncer.processing)

        val hasPreviousAccelerometerMeasurement: Boolean? =
            syncer.getPrivateProperty("hasPreviousAccelerometerMeasurement")
        requireNotNull(hasPreviousAccelerometerMeasurement)
        assertFalse(hasPreviousAccelerometerMeasurement)

        val hasPreviousGravityMeasurement: Boolean? =
            syncer.getPrivateProperty("hasPreviousGravityMeasurement")
        requireNotNull(hasPreviousGravityMeasurement)
        assertFalse(hasPreviousGravityMeasurement)

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

        val lastNotifiedGravityTimestamp: Long? =
            syncer.getPrivateProperty("lastNotifiedGravityTimestamp")
        requireNotNull(lastNotifiedGravityTimestamp)
        assertEquals(0L, lastNotifiedGravityTimestamp)

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
    fun start_whenNotRunningAndTimestampProvided_setsProvidedTimestamp() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

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
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

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
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)
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
        assertFalse(syncer.processing)

        val timestamp = System.nanoTime()
        assertFalse(syncer.start(timestamp))

        assertFalse(syncer.running)
        assertFalse(syncer.processing)
        assertEquals(timestamp, syncer.startTimestamp)

        verify(exactly = 1) { accelerometerSensorCollectorSpy.start(timestamp) }
        verify(exactly = 0) { gravitySensorCollectorSpy.start(any()) }
        verify(exactly = 0) { gyroscopeSensorCollectorSpy.start(any()) }
        verify(exactly = 0) { magnetometerSensorCollectorSpy.start(any()) }
    }

    @Test
    fun start_whenGravityCollectorFails_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

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
        assertFalse(syncer.processing)

        val timestamp = System.nanoTime()
        assertFalse(syncer.start(timestamp))

        assertFalse(syncer.running)
        assertFalse(syncer.processing)
        assertEquals(timestamp, syncer.startTimestamp)

        verify(exactly = 1) { accelerometerSensorCollectorSpy.start(timestamp) }
        verify(exactly = 1) { gravitySensorCollectorSpy.start(timestamp) }
        verify(exactly = 0) { gyroscopeSensorCollectorSpy.start(any()) }
        verify(exactly = 0) { magnetometerSensorCollectorSpy.start(any()) }
    }

    @Test
    fun start_whenGyroscopeCollectorFails_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

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
        assertFalse(syncer.processing)

        val timestamp = System.nanoTime()
        assertFalse(syncer.start(timestamp))

        assertFalse(syncer.running)
        assertFalse(syncer.processing)
        assertEquals(timestamp, syncer.startTimestamp)

        verify(exactly = 1) { accelerometerSensorCollectorSpy.start(timestamp) }
        verify(exactly = 1) { gravitySensorCollectorSpy.start(timestamp) }
        verify(exactly = 1) { gyroscopeSensorCollectorSpy.start(any()) }
        verify(exactly = 0) { magnetometerSensorCollectorSpy.start(any()) }
    }

    @Test
    fun start_whenMagnetometerCollectorFails_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

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
        assertFalse(syncer.processing)

        val timestamp = System.nanoTime()
        assertFalse(syncer.start(timestamp))

        assertFalse(syncer.running)
        assertFalse(syncer.processing)
        assertEquals(timestamp, syncer.startTimestamp)

        verify(exactly = 1) { accelerometerSensorCollectorSpy.start(timestamp) }
        verify(exactly = 1) { gravitySensorCollectorSpy.start(timestamp) }
        verify(exactly = 1) { gyroscopeSensorCollectorSpy.start(any()) }
        verify(exactly = 1) { magnetometerSensorCollectorSpy.start(any()) }
    }

    @Test
    fun start_whenCollectorsSucceed_returnsTrue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

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
        assertFalse(syncer.processing)

        val timestamp = System.nanoTime()
        assertTrue(syncer.start(timestamp))

        assertTrue(syncer.running)
        assertFalse(syncer.processing)
        assertEquals(timestamp, syncer.startTimestamp)

        verify(exactly = 1) { accelerometerSensorCollectorSpy.start(timestamp) }
        verify(exactly = 1) { gravitySensorCollectorSpy.start(timestamp) }
        verify(exactly = 1) { gyroscopeSensorCollectorSpy.start(any()) }
        verify(exactly = 1) { magnetometerSensorCollectorSpy.start(any()) }
    }

    @Test
    fun stop_stopsCollectorsInitializesCachesAndResetsProperties() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

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

        val gravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("gravityMeasurements")
        requireNotNull(gravityMeasurements)
        gravityMeasurements.add(GravitySensorMeasurement())

        val gyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("gyroscopeMeasurements")
        requireNotNull(gyroscopeMeasurements)
        gyroscopeMeasurements.add(GyroscopeSensorMeasurement())

        val magnetometerMeasurements: ArrayDeque<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("magnetometerMeasurements")
        requireNotNull(magnetometerMeasurements)
        magnetometerMeasurements.add(MagnetometerSensorMeasurement())

        val alreadyProcessedAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        alreadyProcessedAccelerometerMeasurements.add(AccelerometerSensorMeasurement())

        val alreadyProcessedGravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGravityMeasurements")
        requireNotNull(alreadyProcessedGravityMeasurements)
        alreadyProcessedGravityMeasurements.add(GravitySensorMeasurement())

        val alreadyProcessedGyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGyroscopeMeasurements")
        requireNotNull(alreadyProcessedGyroscopeMeasurements)
        alreadyProcessedGyroscopeMeasurements.add(GyroscopeSensorMeasurement())

        val alreadyProcessedMagnetometerMeasurements: ArrayDeque<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedMagnetometerMeasurements")
        requireNotNull(alreadyProcessedMagnetometerMeasurements)
        alreadyProcessedMagnetometerMeasurements.add(MagnetometerSensorMeasurement())

        val foundGravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("foundGravityMeasurements")
        requireNotNull(foundGravityMeasurements)
        foundGravityMeasurements.add(GravitySensorMeasurement())

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
        syncer.setPrivateProperty("hasPreviousGravityMeasurement", true)
        syncer.setPrivateProperty("hasPreviousGyroscopeMeasurement", true)
        syncer.setPrivateProperty("hasPreviousMagnetometerMeasurement", true)
        syncer.setPrivateProperty("lastNotifiedTimestamp", 1L)
        syncer.setPrivateProperty("lastNotifiedAccelerometerTimestamp", 2L)
        syncer.setPrivateProperty("lastNotifiedGravityTimestamp", 4L)
        syncer.setPrivateProperty("lastNotifiedGyroscopeTimestamp", 3L)
        syncer.setPrivateProperty("lastNotifiedMagnetometerTimestamp", 5L)

        val stopping1: Boolean? =
            getPrivateProperty(SensorMeasurementSyncer::class, syncer, "stopping")
        requireNotNull(stopping1)
        assertFalse(stopping1)

        syncer.stop()

        // check
        verify(exactly = 1) { accelerometerSensorCollectorSpy.stop() }
        verify(exactly = 1) { gravitySensorCollectorSpy.stop() }
        verify(exactly = 1) { gyroscopeSensorCollectorSpy.stop() }
        verify(exactly = 1) { magnetometerSensorCollectorSpy.stop() }
        assertEquals(1, accelerometerMeasurements.size)
        assertEquals(1, gravityMeasurements.size)
        assertEquals(1, gyroscopeMeasurements.size)
        assertEquals(1, magnetometerMeasurements.size)
        assertEquals(1, alreadyProcessedAccelerometerMeasurements.size)
        assertEquals(1, alreadyProcessedGravityMeasurements.size)
        assertEquals(1, alreadyProcessedGyroscopeMeasurements.size)
        assertEquals(1, alreadyProcessedMagnetometerMeasurements.size)
        assertEquals(1, foundGravityMeasurements.size)
        assertEquals(1, foundGyroscopeMeasurements.size)
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

        val hasPreviousAccelerometerMeasurement: Boolean? =
            syncer.getPrivateProperty("hasPreviousAccelerometerMeasurement")
        requireNotNull(hasPreviousAccelerometerMeasurement)
        assertFalse(hasPreviousAccelerometerMeasurement)

        val hasPreviousGravityMeasurement: Boolean? =
            syncer.getPrivateProperty("hasPreviousGravityMeasurement")
        requireNotNull(hasPreviousGravityMeasurement)
        assertFalse(hasPreviousGravityMeasurement)

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

        val lastNotifiedGravityTimestamp: Long? =
            syncer.getPrivateProperty("lastNotifiedGravityTimestamp")
        requireNotNull(lastNotifiedGravityTimestamp)
        assertEquals(0L, lastNotifiedGravityTimestamp)

        val lastNotifiedGyroscopeTimestamp: Long? =
            syncer.getPrivateProperty("lastNotifiedGyroscopeTimestamp")
        requireNotNull(lastNotifiedGyroscopeTimestamp)
        assertEquals(0L, lastNotifiedGyroscopeTimestamp)

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
    fun accelerometerSensorCollector_whenAccuracyChangedListenerAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

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
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
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
        val syncer =
            AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
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
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
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
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
            context,
            skipWhenProcessing = true,
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
        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "processing", true)

        val listener = accelerometerSensorCollectorSpy.measurementListener
        requireNotNull(listener)

        listener.onMeasurement(accelerometerSensorCollectorSpy, AccelerometerSensorMeasurement(), 0)

        verify(exactly = 0) { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(0) }
        verify { syncedMeasurementListener wasNot Called }
        assertNull(syncer.mostRecentTimestamp)
    }

    @Test
    fun accelerometerSensorCollector_whenMeasurementAndNoMeasurementsAvailable_callsCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
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
        val accelerometerMeasurement = AccelerometerSensorMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            syncer.accelerometerSensorType
        )
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
        assertEquals(syncer.accelerometerSensorType, accelerometerMeasurement2.sensorType)

        assertEquals(timestamp, syncer.oldestTimestamp)
        assertEquals(0, syncer.numberOfProcessedMeasurements)

        val alreadyProcessedAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())

        val alreadyProcessedGravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGravityMeasurements")
        requireNotNull(alreadyProcessedGravityMeasurements)
        assertTrue(alreadyProcessedGravityMeasurements.isEmpty())

        val alreadyProcessedGyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGyroscopeMeasurements")
        requireNotNull(alreadyProcessedGyroscopeMeasurements)
        assertTrue(alreadyProcessedGyroscopeMeasurements.isEmpty())

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
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)

        val listener = gravitySensorCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(gravitySensorCollector, SensorAccuracy.HIGH)
    }

    @Test
    fun gravitySensorCollector_whenAccuracyChangedListenerAndListenerAvailable_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
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
                SensorType.GRAVITY,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun gravitySensorCollector_whenBufferFilledAndStopWhenFilledBuffer_stops() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer =
            AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
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
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
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
                SensorType.GRAVITY
            )
        }
        assertTrue(syncer.running)
    }

    @Test
    fun gravitySensorCollector_whenSkipWhenProcessingAndProcessing_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
            context,
            skipWhenProcessing = true,
            syncedMeasurementListener = syncedMeasurementListener
        )

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)
        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "processing", true)

        val listener = gravitySensorCollector.measurementListener
        requireNotNull(listener)

        listener.onMeasurement(gravitySensorCollectorSpy, GravitySensorMeasurement(), 0)

        verify { gravitySensorCollectorSpy wasNot Called }
        verify { syncedMeasurementListener wasNot Called }
        assertNull(syncer.mostRecentTimestamp)
    }

    @Test
    fun gravitySensorCollector_whenMeasurementAndMostRecentTimestampNotDefined_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
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
    fun gyroscopeSensorCollector_whenAccuracyChangedListenerAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

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
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
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
                SensorType.GYROSCOPE_UNCALIBRATED,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun gyroscopeSensorCollector_whenBufferFilledAndStopWhenFilledBuffer_stops() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer =
            AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
                context,
                stopWhenFilledBuffer = true
            )

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
            AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
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
                SensorType.GYROSCOPE_UNCALIBRATED
            )
        }
        assertTrue(syncer.running)
    }

    @Test
    fun gyroscopeSensorCollector_whenSkipWhenProcessingAndProcessing_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
            context,
            skipWhenProcessing = true,
            syncedMeasurementListener = syncedMeasurementListener
        )

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)
        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "processing", true)

        val listener = gyroscopeSensorCollector.measurementListener
        requireNotNull(listener)

        listener.onMeasurement(gyroscopeSensorCollectorSpy, GyroscopeSensorMeasurement(), 0)

        verify { gyroscopeSensorCollectorSpy wasNot Called }
        verify { syncedMeasurementListener wasNot Called }
        assertNull(syncer.mostRecentTimestamp)
    }

    @Test
    fun gyroscopeSensorCollector_whenMeasurementAndMostRecentTimestampNotDefined_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
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
    fun gyroscopeSensorCollector_whenMeasurementMostRecentTimestampDefinedAndMeasurementsAvailable_copiesMeasurements() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
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
        val gyroscopeMeasurement = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp,
            SensorAccuracy.HIGH,
            syncer.gyroscopeSensorType
        )
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

        verify { syncedMeasurementListener wasNot Called }
        assertTrue(syncer.running)
    }

    @Test
    fun magnetometerSensorCollector_whenAccuracyChangedListenerAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(context)

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)

        val listener = magnetometerSensorCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(magnetometerSensorCollector, SensorAccuracy.MEDIUM)
    }

    @Test
    fun magnetometerSensorCollector_whenAccuracyChangedListenerAndListenerAvailable_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
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
                SensorType.MAGNETOMETER_UNCALIBRATED,
                SensorAccuracy.MEDIUM
            )
        }
    }

    @Test
    fun magnetometerSensorCollector_whenBufferFilledAndStopWhenFilledBuffer_stops() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
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
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
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
                SensorType.MAGNETOMETER_UNCALIBRATED
            )
        }
        assertTrue(syncer.running)
    }

    @Test
    fun magnetometerSensorCollector_whenMeasurementAndMostRecentTimestampNotDefined_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
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
            SensorAccuracy.HIGH,
            syncer.magnetometerSensorType
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
        assertEquals(syncer.magnetometerSensorType, magnetometerMeasurement2.sensorType)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
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
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)

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
            SensorAccuracy.HIGH,
            syncer.gyroscopeSensorType
        )
        val gyroscopeMeasurementsBeforeTimestamp = ArrayDeque<GyroscopeSensorMeasurement>()
        gyroscopeMeasurementsBeforeTimestamp.add(gyroscopeMeasurement1)
        every { gyroscopeSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            gyroscopeMeasurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

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
            SensorAccuracy.HIGH,
            syncer.accelerometerSensorType
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
        assertTrue(gyroscopeMeasurements.isEmpty())

        val alreadyProcessedAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())
        assertTrue(alreadyProcessedGyroscopeMeasurements.isEmpty())

        val slot = slot<AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorMeasurement>()
        verify(exactly = 1) {
            syncedMeasurementListener.onSyncedMeasurements(
                syncer,
                capture(slot)
            )
        }

        val syncedMeasurement = slot.captured
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
        assertEquals(syncer.magnetometerSensorType, syncedMagnetometerMeasurement.sensorType)
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
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
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
        val staleGyroscopeMeasurement = GyroscopeSensorMeasurement(
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

        val gravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("gravityMeasurements")
        requireNotNull(gravityMeasurements)
        val gx1 = randomizer.nextFloat()
        val gy1 = randomizer.nextFloat()
        val gz1 = randomizer.nextFloat()
        val staleGravityMeasurement =
            GravitySensorMeasurement(gx1, gy1, gz1, staleTimestamp, SensorAccuracy.MEDIUM)
        gravityMeasurements.add(staleGravityMeasurement)

        val magnetometerMeasurements: ArrayDeque<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("magnetometerMeasurements")
        requireNotNull(magnetometerMeasurements)
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
            SensorAccuracy.HIGH,
            syncer.accelerometerSensorType
        )
        accelerometerMeasurements.add(staleAccelerometerMeasurement)

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)

        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val gyroscopeTimestamp = accelerometerTimestamp - 1
        val gyroscopeMeasurement1 = GyroscopeSensorMeasurement(
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
        val gyroscopeMeasurementsBeforeTimestamp = ArrayDeque<GyroscopeSensorMeasurement>()
        gyroscopeMeasurementsBeforeTimestamp.add(gyroscopeMeasurement1)
        every { gyroscopeSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            gyroscopeMeasurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

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
            SensorAccuracy.MEDIUM,
            syncer.magnetometerSensorType
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

        val magnetometerListener = magnetometerSensorCollectorSpy.measurementListener
        requireNotNull(magnetometerListener)

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
        assertEquals(syncer.gyroscopeSensorType, gyroscopeMeasurement3.sensorType)

        val alreadyProcessedGyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGyroscopeMeasurements")
        requireNotNull(alreadyProcessedGyroscopeMeasurements)
        assertTrue(alreadyProcessedGyroscopeMeasurements.isEmpty())

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
        assertTrue(gyroscopeMeasurements.isEmpty())
        assertTrue(magnetometerMeasurements.isEmpty())

        val alreadyProcessedAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())
        assertTrue(alreadyProcessedGyroscopeMeasurements.isEmpty())
        assertTrue(alreadyProcessedGravityMeasurements.isEmpty())
        assertTrue(alreadyProcessedMagnetometerMeasurements.isEmpty())

        verify(exactly = 1) {
            staleDetectedMeasurementsListener.onStaleMeasurements(
                syncer,
                SensorType.ACCELEROMETER_UNCALIBRATED,
                any()
            )
        }
    }

    @Test
    fun cleanupStaleMeasurements_whenStaleDetectionDisabled_doesNotNotify() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
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
        val staleGyroscopeMeasurement = GyroscopeSensorMeasurement(
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

        val gravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("gravityMeasurements")
        requireNotNull(gravityMeasurements)
        val gx1 = randomizer.nextFloat()
        val gy1 = randomizer.nextFloat()
        val gz1 = randomizer.nextFloat()
        val staleGravityMeasurement =
            GravitySensorMeasurement(gx1, gy1, gz1, staleTimestamp, SensorAccuracy.MEDIUM)
        gravityMeasurements.add(staleGravityMeasurement)

        val magnetometerMeasurements: ArrayDeque<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("magnetometerMeasurements")
        requireNotNull(magnetometerMeasurements)
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
            SensorAccuracy.HIGH,
            syncer.accelerometerSensorType
        )
        accelerometerMeasurements.add(staleAccelerometerMeasurement)

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)

        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val gyroscopeTimestamp = accelerometerTimestamp - 1
        val gyroscopeMeasurement1 = GyroscopeSensorMeasurement(
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
        val gyroscopeMeasurementsBeforeTimestamp = ArrayDeque<GyroscopeSensorMeasurement>()
        gyroscopeMeasurementsBeforeTimestamp.add(gyroscopeMeasurement1)
        every { gyroscopeSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            gyroscopeMeasurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

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
            SensorAccuracy.MEDIUM,
            syncer.magnetometerSensorType
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

        val magnetometerListener = magnetometerSensorCollectorSpy.measurementListener
        requireNotNull(magnetometerListener)

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
        assertEquals(syncer.gyroscopeSensorType, gyroscopeMeasurement3.sensorType)

        val alreadyProcessedGyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGyroscopeMeasurements")
        requireNotNull(alreadyProcessedGyroscopeMeasurements)
        assertTrue(alreadyProcessedGyroscopeMeasurements.isEmpty())

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
        assertTrue(gyroscopeMeasurements.isEmpty())
        assertTrue(magnetometerMeasurements.isEmpty())

        val alreadyProcessedAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertFalse(alreadyProcessedAccelerometerMeasurements.isEmpty())
        assertFalse(alreadyProcessedGravityMeasurements.isEmpty())
        assertTrue(alreadyProcessedMagnetometerMeasurements.isEmpty())
        assertFalse(alreadyProcessedGyroscopeMeasurements.isEmpty())

        verify { staleDetectedMeasurementsListener wasNot Called }
    }

    @Test
    fun sensorCollectors_whenPreviousGyroscopeMeasurement_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
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
        syncer.setPrivateProperty("hasPreviousGyroscopeMeasurement", true)
        syncer.setPrivateProperty("lastNotifiedTimestamp", accelerometerTimestamp - 1)
        syncer.setPrivateProperty("lastNotifiedAccelerometerTimestamp", accelerometerTimestamp - 1)
        syncer.setPrivateProperty("lastNotifiedGyroscopeTimestamp", accelerometerTimestamp - 2)
        syncer.setPrivateProperty("hasPreviousGravityMeasurement", true)
        syncer.setPrivateProperty("hasPreviousMagnetometerMeasurement", true)
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
        previousMagnetometerMeasurement.timestamp = accelerometerTimestamp - 1
        previousMagnetometerMeasurement.accuracy = SensorAccuracy.MEDIUM
        previousMagnetometerMeasurement.sensorType = syncer.magnetometerSensorType

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)

        val bx2 = randomizer.nextFloat()
        val by2 = randomizer.nextFloat()
        val bz2 = randomizer.nextFloat()
        val magnetometerTimestamp = accelerometerTimestamp - 1
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
        val magnetometerMeasurementsBeforeTimestamp = ArrayDeque<MagnetometerSensorMeasurement>()
        magnetometerMeasurementsBeforeTimestamp.add(magnetometerMeasurement1)
        every { magnetometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            magnetometerMeasurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        val previousGravityMeasurement: GravitySensorMeasurement? =
            syncer.getPrivateProperty("previousGravityMeasurement")
        requireNotNull(previousGravityMeasurement)
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

        val previousGyroscopeMeasurement: GyroscopeSensorMeasurement? =
            syncer.getPrivateProperty("previousGyroscopeMeasurement")
        requireNotNull(previousGyroscopeMeasurement)
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
        previousGyroscopeMeasurement.sensorType = syncer.gyroscopeSensorType

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)

        val wx2 = randomizer.nextFloat()
        val wy2 = randomizer.nextFloat()
        val wz2 = randomizer.nextFloat()
        val gyroscopeTimestamp = accelerometerTimestamp - 1
        val gyroscopeMeasurement1 = GyroscopeSensorMeasurement(
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
        val gyroscopeMeasurementsBeforeTimestamp = ArrayDeque<GyroscopeSensorMeasurement>()
        gyroscopeMeasurementsBeforeTimestamp.add(gyroscopeMeasurement1)
        every { gyroscopeSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            gyroscopeMeasurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

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
            SensorAccuracy.HIGH,
            syncer.accelerometerSensorType
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

        val magnetometerListener = magnetometerSensorCollectorSpy.measurementListener
        requireNotNull(magnetometerListener)

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

        val alreadyProcessedGyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGyroscopeMeasurements")
        requireNotNull(alreadyProcessedGyroscopeMeasurements)
        assertTrue(alreadyProcessedGyroscopeMeasurements.isEmpty())

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
        assertEquals(1, gyroscopeMeasurements.size)
        assertSame(gyroscopeMeasurement2, gyroscopeMeasurements.first())

        val alreadyProcessedAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())
        assertTrue(alreadyProcessedGyroscopeMeasurements.isEmpty())

        val slot = slot<AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorMeasurement>()
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
        assertEquals(syncer.accelerometerSensorType, syncedAccelerometerMeasurement.sensorType)
        val syncedGravityMeasurement = syncedMeasurement.gravityMeasurement
        requireNotNull(syncedGravityMeasurement)
        assertEquals(gx1, syncedGravityMeasurement.gx, 0.0f)
        assertEquals(gy1, syncedGravityMeasurement.gy, 0.0f)
        assertEquals(gz1, syncedGravityMeasurement.gz, 0.0f)
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
        assertEquals(accelerometerTimestamp, syncedMagnetometerMeasurement.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, syncedMagnetometerMeasurement.accuracy)
        assertEquals(syncer.magnetometerSensorType, syncedMagnetometerMeasurement.sensorType)
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
    fun sensorCollectors_whenNoPreviousGyroscopeMeasurement_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
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
        syncer.setPrivateProperty("hasPreviousGyroscopeMeasurement", true)
        syncer.setPrivateProperty("lastNotifiedTimestamp", accelerometerTimestamp - 1)
        syncer.setPrivateProperty("lastNotifiedAccelerometerTimestamp", accelerometerTimestamp - 1)
        syncer.setPrivateProperty("lastNotifiedGyroscopeTimestamp", accelerometerTimestamp - 2)
        syncer.setPrivateProperty("hasPreviousGravityMeasurement", true)
        syncer.setPrivateProperty("hasPreviousMagnetometerMeasurement", true)
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
        previousMagnetometerMeasurement.timestamp = accelerometerTimestamp - 1
        previousMagnetometerMeasurement.accuracy = SensorAccuracy.MEDIUM
        previousMagnetometerMeasurement.sensorType = syncer.magnetometerSensorType

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)

        val bx2 = randomizer.nextFloat()
        val by2 = randomizer.nextFloat()
        val bz2 = randomizer.nextFloat()
        val magnetometerMeasurement1 = MagnetometerSensorMeasurement(
            bx2,
            by2,
            bz2,
            hardIronX,
            hardIronY,
            hardIronZ,
            accelerometerTimestamp,
            SensorAccuracy.MEDIUM,
            syncer.magnetometerSensorType
        )
        val magnetometerMeasurementsBeforeTimestamp = ArrayDeque<MagnetometerSensorMeasurement>()
        magnetometerMeasurementsBeforeTimestamp.add(magnetometerMeasurement1)
        every { magnetometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            magnetometerMeasurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        val previousGravityMeasurement: GravitySensorMeasurement? =
            syncer.getPrivateProperty("previousGravityMeasurement")
        requireNotNull(previousGravityMeasurement)
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

        val previousGyroscopeMeasurement: GyroscopeSensorMeasurement? =
            syncer.getPrivateProperty("previousGyroscopeMeasurement")
        requireNotNull(previousGyroscopeMeasurement)
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
        previousGyroscopeMeasurement.sensorType = syncer.gyroscopeSensorType

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)

        val wx2 = randomizer.nextFloat()
        val wy2 = randomizer.nextFloat()
        val wz2 = randomizer.nextFloat()
        val gyroscopeTimestamp = accelerometerTimestamp - 3
        val gyroscopeMeasurement1 = GyroscopeSensorMeasurement(
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
        val gyroscopeMeasurementsBeforeTimestamp = ArrayDeque<GyroscopeSensorMeasurement>()
        gyroscopeMeasurementsBeforeTimestamp.add(gyroscopeMeasurement1)
        every { gyroscopeSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            gyroscopeMeasurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

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
            SensorAccuracy.HIGH,
            syncer.accelerometerSensorType
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

        val magnetometerListener = magnetometerSensorCollectorSpy.measurementListener
        requireNotNull(magnetometerListener)

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
        assertEquals(accelerometerTimestamp, magnetometerMeasurement2.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, magnetometerMeasurement2.accuracy)
        assertEquals(syncer.magnetometerSensorType, magnetometerMeasurement2.sensorType)

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
        assertEquals(1, gyroscopeMeasurements.size)
        assertSame(gyroscopeMeasurement2, gyroscopeMeasurements.first())

        val alreadyProcessedAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())
        assertTrue(alreadyProcessedGyroscopeMeasurements.isEmpty())

        val slot = slot<AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorMeasurement>()
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
        assertEquals(syncer.accelerometerSensorType, syncedAccelerometerMeasurement.sensorType)
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
        assertEquals(accelerometerTimestamp, syncedMagnetometerMeasurement.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, syncedMagnetometerMeasurement.accuracy)
        assertEquals(syncer.magnetometerSensorType, syncedMagnetometerMeasurement.sensorType)
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
    fun sensorCollectors_whenNoPreviousMagnetometerMeasurement_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
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
        syncer.setPrivateProperty("hasPreviousGyroscopeMeasurement", true)
        syncer.setPrivateProperty("lastNotifiedTimestamp", accelerometerTimestamp - 2)
        syncer.setPrivateProperty("lastNotifiedAccelerometerTimestamp", accelerometerTimestamp - 1)
        syncer.setPrivateProperty("lastNotifiedGyroscopeTimestamp", accelerometerTimestamp - 2)
        syncer.setPrivateProperty("hasPreviousGravityMeasurement", true)
        syncer.setPrivateProperty("hasPreviousMagnetometerMeasurement", true)
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
        previousMagnetometerMeasurement.timestamp = accelerometerTimestamp - 1
        previousMagnetometerMeasurement.accuracy = SensorAccuracy.MEDIUM
        previousMagnetometerMeasurement.sensorType = syncer.magnetometerSensorType

        val magnetometerSensorCollector: BufferedMagnetometerSensorCollector? =
            syncer.getPrivateProperty("magnetometerSensorCollector")
        requireNotNull(magnetometerSensorCollector)
        val magnetometerSensorCollectorSpy = spyk(magnetometerSensorCollector)

        val bx2 = randomizer.nextFloat()
        val by2 = randomizer.nextFloat()
        val bz2 = randomizer.nextFloat()
        val magnetometerMeasurement1 = MagnetometerSensorMeasurement(
            bx2,
            by2,
            bz2,
            hardIronX,
            hardIronY,
            hardIronZ,
            accelerometerTimestamp,
            SensorAccuracy.MEDIUM,
            syncer.magnetometerSensorType
        )
        val magnetometerMeasurementsBeforeTimestamp = ArrayDeque<MagnetometerSensorMeasurement>()
        magnetometerMeasurementsBeforeTimestamp.add(magnetometerMeasurement1)
        every { magnetometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            magnetometerMeasurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("magnetometerSensorCollector", magnetometerSensorCollectorSpy)

        val previousGravityMeasurement: GravitySensorMeasurement? =
            syncer.getPrivateProperty("previousGravityMeasurement")
        requireNotNull(previousGravityMeasurement)
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

        val previousGyroscopeMeasurement: GyroscopeSensorMeasurement? =
            syncer.getPrivateProperty("previousGyroscopeMeasurement")
        requireNotNull(previousGyroscopeMeasurement)
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
        previousGyroscopeMeasurement.sensorType = syncer.gyroscopeSensorType

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)

        val wx2 = randomizer.nextFloat()
        val wy2 = randomizer.nextFloat()
        val wz2 = randomizer.nextFloat()
        val gyroscopeTimestamp = accelerometerTimestamp - 1
        val gyroscopeMeasurement1 = GyroscopeSensorMeasurement(
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
        val gyroscopeMeasurementsBeforeTimestamp = ArrayDeque<GyroscopeSensorMeasurement>()
        gyroscopeMeasurementsBeforeTimestamp.add(gyroscopeMeasurement1)
        every { gyroscopeSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            gyroscopeMeasurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

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
            SensorAccuracy.HIGH,
            syncer.accelerometerSensorType
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

        val magnetometerListener = magnetometerSensorCollectorSpy.measurementListener
        requireNotNull(magnetometerListener)

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
        assertEquals(accelerometerTimestamp, magnetometerMeasurement2.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, magnetometerMeasurement2.accuracy)
        assertEquals(syncer.magnetometerSensorType, magnetometerMeasurement2.sensorType)

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
        assertTrue(gyroscopeMeasurements.isEmpty())

        val alreadyProcessedAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())
        assertTrue(alreadyProcessedGyroscopeMeasurements.isEmpty())

        val slot = slot<AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorMeasurement>()
        verify(exactly = 1) {
            syncedMeasurementListener.onSyncedMeasurements(
                syncer,
                capture(slot)
            )
        }

        val syncedMeasurement = slot.captured
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
        val syncedGravityMeasurement = syncedMeasurement.gravityMeasurement
        requireNotNull(syncedGravityMeasurement)
        assertEquals(gx2, syncedGravityMeasurement.gx, 0.0f)
        assertEquals(gy2, syncedGravityMeasurement.gy, 0.0f)
        assertEquals(gz2, syncedGravityMeasurement.gz, 0.0f)
        assertEquals(gyroscopeTimestamp, syncedGravityMeasurement.timestamp)
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
        assertEquals(syncer.magnetometerSensorType, syncedMagnetometerMeasurement.sensorType)
        val syncedGyroscopeMeasurement = syncedMeasurement.gyroscopeMeasurement
        requireNotNull(syncedGyroscopeMeasurement)
        assertEquals(wx2, syncedGyroscopeMeasurement.wx, 0.0f)
        assertEquals(wy2, syncedGyroscopeMeasurement.wy, 0.0f)
        assertEquals(wz2, syncedGyroscopeMeasurement.wz, 0.0f)
        assertEquals(wbx, syncedGyroscopeMeasurement.bx)
        assertEquals(wby, syncedGyroscopeMeasurement.by)
        assertEquals(wbz, syncedGyroscopeMeasurement.bz)
        assertEquals(accelerometerTimestamp - 1, syncedGyroscopeMeasurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, syncedGyroscopeMeasurement.accuracy)
        assertEquals(syncer.gyroscopeSensorType, syncedGyroscopeMeasurement.sensorType)
    }

    @Test
    fun sensorCollectors_whenStopping_clearCollectionsAndResets() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AccelerometerGravityGyroscopeAndMagnetometerSensorMeasurementSyncer(
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
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)

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
            SensorAccuracy.HIGH,
            syncer.gyroscopeSensorType
        )
        val gyroscopeMeasurementsBeforeTimestamp = ArrayDeque<GyroscopeSensorMeasurement>()
        gyroscopeMeasurementsBeforeTimestamp.add(gyroscopeMeasurement1)
        every { gyroscopeSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            gyroscopeMeasurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

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
            SensorAccuracy.HIGH,
            syncer.accelerometerSensorType
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

        // process accelerometer measurement
        accelerometerListener.onMeasurement(
            accelerometerSensorCollectorSpy,
            AccelerometerSensorMeasurement(),
            0
        )

        verify(exactly = 1) { accelerometerSensorCollectorSpy.getMeasurementsBeforePosition(0) }

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
        val foundGravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("foundGravityMeasurements")
        requireNotNull(foundGravityMeasurements)
        val foundGyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("foundGyroscopeMeasurements")
        requireNotNull(foundGyroscopeMeasurements)
        val foundMagnetometerMeasurements: ArrayDeque<MagnetometerSensorMeasurement>? =
            syncer.getPrivateProperty("foundMagnetometerMeasurements")
        requireNotNull(foundMagnetometerMeasurements)
        assertTrue(accelerometerMeasurements.isEmpty())
        assertTrue(gravityMeasurements.isEmpty())
        assertTrue(gyroscopeMeasurements.isEmpty())
        assertTrue(magnetometerMeasurements.isEmpty())
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())
        assertTrue(alreadyProcessedGravityMeasurements.isEmpty())
        assertTrue(alreadyProcessedGyroscopeMeasurements.isEmpty())
        assertTrue(alreadyProcessedMagnetometerMeasurements.isEmpty())
        assertTrue(foundGravityMeasurements.isEmpty())
        assertTrue(foundGyroscopeMeasurements.isEmpty())
        assertTrue(foundMagnetometerMeasurements.isEmpty())

        val slot = slot<AccelerometerGravityGyroscopeAndMagnetometerSyncedSensorMeasurement>()
        verify(exactly = 1) {
            syncedMeasurementListener.onSyncedMeasurements(
                syncer,
                capture(slot)
            )
        }

        val syncedMeasurement = slot.captured
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
        assertEquals(syncer.magnetometerSensorType, syncedMagnetometerMeasurement.sensorType)
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