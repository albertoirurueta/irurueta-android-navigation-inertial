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
import com.irurueta.android.navigation.inertial.getPrivateProperty
import com.irurueta.android.navigation.inertial.setPrivateProperty
import com.irurueta.geometry.Quaternion
import com.irurueta.statistics.UniformRandomizer
import io.mockk.*
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import java.util.*

@RunWith(RobolectricTestRunner::class)
class AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncerTest {

    @Test
    fun constructor_whenRequiredParameters_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

        // check
        assertSame(context, syncer.context)
        assertEquals(AttitudeSensorType.ABSOLUTE_ATTITUDE, syncer.attitudeSensorType)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            syncer.accelerometerSensorType
        )
        assertEquals(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED, syncer.gyroscopeSensorType)
        assertEquals(SensorDelay.FASTEST, syncer.attitudeSensorDelay)
        assertEquals(SensorDelay.FASTEST, syncer.accelerometerSensorDelay)
        assertEquals(SensorDelay.FASTEST, syncer.gravitySensorDelay)
        assertEquals(SensorDelay.FASTEST, syncer.gyroscopeSensorDelay)
        assertEquals(
            AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer.DEFAULT_ATTITUDE_CAPACITY,
            syncer.attitudeCapacity
        )
        assertEquals(
            AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer.DEFAULT_ACCELEROMETER_CAPACITY,
            syncer.accelerometerCapacity
        )
        assertEquals(
            AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer.DEFAULT_GRAVITY_CAPACITY,
            syncer.gravityCapacity
        )
        assertEquals(
            AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer.DEFAULT_GYROSCOPE_CAPACITY,
            syncer.gyroscopeCapacity
        )
        assertFalse(syncer.attitudeStartOffsetEnabled)
        assertFalse(syncer.accelerometerStartOffsetEnabled)
        assertFalse(syncer.gravityStartOffsetEnabled)
        assertFalse(syncer.gyroscopeStartOffsetEnabled)
        assertTrue(syncer.stopWhenFilledBuffer)
        assertEquals(
            AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer.DEFAULT_STALE_OFFSET_NANOS,
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
        assertNull(syncer.attitudeSensor)
        assertNull(syncer.accelerometerSensor)
        assertNull(syncer.gravitySensor)
        assertNull(syncer.gyroscopeSensor)
        assertFalse(syncer.attitudeSensorAvailable)
        assertFalse(syncer.accelerometerSensorAvailable)
        assertFalse(syncer.gravitySensorAvailable)
        assertFalse(syncer.gyroscopeSensorAvailable)
        assertNull(syncer.attitudeStartOffset)
        assertNull(syncer.accelerometerStartOffset)
        assertNull(syncer.gravityStartOffset)
        assertNull(syncer.gyroscopeStartOffset)
        assertEquals(0.0f, syncer.attitudeCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.accelerometerCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.gravityCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.gyroscopeCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.attitudeUsage, 0.0f)
        assertEquals(0.0f, syncer.accelerometerUsage, 0.0f)
        assertEquals(0.0f, syncer.gravityUsage, 0.0f)
        assertEquals(0.0f, syncer.gyroscopeUsage, 0.0f)
    }

    @Test(expected = IllegalArgumentException::class)
    fun constructor_whenZeroAttitudeCapacity_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
            context,
            attitudeCapacity = 0
        )
    }

    @Test(expected = IllegalArgumentException::class)
    fun constructor_whenZeroAccelerometerCapacity_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
            context,
            accelerometerCapacity = 0
        )
    }

    @Test(expected = IllegalArgumentException::class)
    fun constructor_whenZeroGravityCapacity_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
            context,
            gravityCapacity = 0
        )
    }

    @Test(expected = IllegalArgumentException::class)
    fun constructor_whenZeroGyroscopeCapacity_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
            context,
            gyroscopeCapacity = 0
        )
    }

    @Test
    fun constructor_whenAllParameters_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val accuracyChangedListener =
            mockk<SensorMeasurementSyncer.OnAccuracyChangedListener<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer>>()
        val bufferFilledListener =
            mockk<SensorMeasurementSyncer.OnBufferFilledListener<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer>>()
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer>>()
        val staleDetectedMeasurementsListener =
            mockk<SensorMeasurementSyncer.OnStaleDetectedMeasurementsListener<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer>>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
            context,
            AttitudeSensorType.RELATIVE_ATTITUDE,
            AccelerometerSensorType.ACCELEROMETER,
            GyroscopeSensorType.GYROSCOPE,
            SensorDelay.NORMAL,
            SensorDelay.UI,
            SensorDelay.GAME,
            SensorDelay.NORMAL,
            attitudeCapacity = 2,
            accelerometerCapacity = 3,
            gravityCapacity = 4,
            gyroscopeCapacity = 5,
            attitudeStartOffsetEnabled = true,
            accelerometerStartOffsetEnabled = true,
            gravityStartOffsetEnabled = true,
            gyroscopeStartOffsetEnabled = true,
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
        assertEquals(AttitudeSensorType.RELATIVE_ATTITUDE, syncer.attitudeSensorType)
        assertEquals(AccelerometerSensorType.ACCELEROMETER, syncer.accelerometerSensorType)
        assertEquals(GyroscopeSensorType.GYROSCOPE, syncer.gyroscopeSensorType)
        assertEquals(SensorDelay.NORMAL, syncer.attitudeSensorDelay)
        assertEquals(SensorDelay.UI, syncer.accelerometerSensorDelay)
        assertEquals(SensorDelay.GAME, syncer.gravitySensorDelay)
        assertEquals(SensorDelay.NORMAL, syncer.gyroscopeSensorDelay)
        assertEquals(2, syncer.attitudeCapacity)
        assertEquals(3, syncer.accelerometerCapacity)
        assertEquals(4, syncer.gravityCapacity)
        assertEquals(5, syncer.gyroscopeCapacity)
        assertTrue(syncer.attitudeStartOffsetEnabled)
        assertTrue(syncer.accelerometerStartOffsetEnabled)
        assertTrue(syncer.gravityStartOffsetEnabled)
        assertTrue(syncer.gyroscopeStartOffsetEnabled)
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
        assertNull(syncer.attitudeSensor)
        assertNull(syncer.accelerometerSensor)
        assertNull(syncer.gravitySensor)
        assertNull(syncer.gyroscopeSensor)
        assertFalse(syncer.attitudeSensorAvailable)
        assertFalse(syncer.accelerometerSensorAvailable)
        assertFalse(syncer.gravitySensorAvailable)
        assertFalse(syncer.gyroscopeSensorAvailable)
        assertNull(syncer.attitudeStartOffset)
        assertNull(syncer.accelerometerStartOffset)
        assertNull(syncer.gravityStartOffset)
        assertNull(syncer.gyroscopeStartOffset)
        assertEquals(0.0f, syncer.attitudeCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.accelerometerCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.gravityCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.gyroscopeCollectorUsage, 0.0f)
        assertEquals(0.0f, syncer.attitudeUsage, 0.0f)
        assertEquals(0.0f, syncer.accelerometerUsage, 0.0f)
        assertEquals(0.0f, syncer.gravityUsage, 0.0f)
        assertEquals(0.0f, syncer.gyroscopeUsage, 0.0f)
    }

    @Test
    fun accuracyChangedListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

        // check default value
        assertNull(syncer.accuracyChangedListener)

        // set new value
        val accuracyChangedListener =
            mockk<SensorMeasurementSyncer.OnAccuracyChangedListener<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer>>()
        syncer.accuracyChangedListener = accuracyChangedListener

        // check
        assertSame(accuracyChangedListener, syncer.accuracyChangedListener)
    }

    @Test
    fun bufferFilledListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

        // check default value
        assertNull(syncer.bufferFilledListener)

        // set new value
        val bufferFilledListener =
            mockk<SensorMeasurementSyncer.OnBufferFilledListener<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer>>()
        syncer.bufferFilledListener = bufferFilledListener

        // check
        assertSame(bufferFilledListener, syncer.bufferFilledListener)
    }

    @Test
    fun syncedMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

        // check default value
        assertNull(syncer.syncedMeasurementListener)

        // set new value
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer>>()
        syncer.syncedMeasurementListener = syncedMeasurementListener

        // check
        assertSame(syncedMeasurementListener, syncer.syncedMeasurementListener)
    }

    @Test
    fun staleDetectedMeasurementsListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

        // check default value
        assertNull(syncer.staleDetectedMeasurementsListener)

        // set new value
        val staleDetectedMeasurementsListener =
            mockk<SensorMeasurementSyncer.OnStaleDetectedMeasurementsListener<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer>>()
        syncer.staleDetectedMeasurementsListener = staleDetectedMeasurementsListener

        // check
        assertSame(staleDetectedMeasurementsListener, syncer.staleDetectedMeasurementsListener)
    }

    @Test
    fun attitudeSensor_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

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
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

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
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

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
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        assertNull(syncer.gyroscopeSensor)
        verify(exactly = 1) { gyroscopeSensorCollectorSpy.sensor }
    }

    @Test
    fun attitudeSensorAvailable_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

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
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

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
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

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
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        assertFalse(syncer.gyroscopeSensorAvailable)
        verify(exactly = 1) { gyroscopeSensorCollectorSpy.sensorAvailable }
    }

    @Test
    fun attitudeStartOffset_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

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
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

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
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

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
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        assertNull(syncer.gyroscopeStartOffset)
        verify(exactly = 1) { gyroscopeSensorCollectorSpy.startOffset }
    }

    @Test
    fun attitudeCollectorUsage_callsInternalCollector() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

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
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

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
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

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
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        assertEquals(0.0f, syncer.gyroscopeCollectorUsage, 0.0f)
        verify(exactly = 1) { gyroscopeSensorCollectorSpy.usage }
    }

    @Test
    fun attitudeUsage_getsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

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
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

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
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

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
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

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
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

        // set as running
        setPrivateProperty(SensorMeasurementSyncer::class, syncer, "running", true)
        assertTrue(syncer.running)

        syncer.start()
    }

    @Test
    fun start_whenNotRunningAndNoTimestamp_clearsResetsAndSetsCurrentStartTimestamp() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

        assertFalse(syncer.running)
        assertEquals(0L, syncer.startTimestamp)

        // set variables that will be later reset
        val attitudeMeasurements: ArrayDeque<AttitudeSensorMeasurement>? =
            syncer.getPrivateProperty("attitudeMeasurements")
        requireNotNull(attitudeMeasurements)
        attitudeMeasurements.add(AttitudeSensorMeasurement())

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

        val alreadyProcessedAttitudeMeasurements: ArrayDeque<AttitudeSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAttitudeMeasurements")
        requireNotNull(alreadyProcessedAttitudeMeasurements)
        alreadyProcessedAttitudeMeasurements.add(AttitudeSensorMeasurement())

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

        val foundAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("foundAccelerometerMeasurements")
        requireNotNull(foundAccelerometerMeasurements)
        foundAccelerometerMeasurements.add(AccelerometerSensorMeasurement())

        val foundGravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("foundGravityMeasurements")
        requireNotNull(foundGravityMeasurements)
        foundGravityMeasurements.add(GravitySensorMeasurement())

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

        syncer.setPrivateProperty("hasPreviousAttitudeMeasurement", true)
        syncer.setPrivateProperty("hasPreviousAccelerometerMeasurement", true)
        syncer.setPrivateProperty("hasPreviousGravityMeasurement", true)
        syncer.setPrivateProperty("hasPreviousGyroscopeMeasurement", true)
        syncer.setPrivateProperty("lastNotifiedTimestamp", 1L)
        syncer.setPrivateProperty("lastNotifiedAttitudeTimestamp", 2L)
        syncer.setPrivateProperty("lastNotifiedAccelerometerTimestamp", 3L)
        syncer.setPrivateProperty("lastNotifiedGravityTimestamp", 4L)
        syncer.setPrivateProperty("lastNotifiedGyroscopeTimestamp", 5L)

        assertFalse(syncer.running)
        assertEquals(0L, syncer.startTimestamp)

        assertFalse(syncer.start())

        // check
        assertFalse(syncer.running)
        assertNotEquals(0L, syncer.startTimestamp)

        assertTrue(attitudeMeasurements.isEmpty())
        assertTrue(accelerometerMeasurements.isEmpty())
        assertTrue(gravityMeasurements.isEmpty())
        assertTrue(gyroscopeMeasurements.isEmpty())
        assertTrue(alreadyProcessedAttitudeMeasurements.isEmpty())
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())
        assertTrue(alreadyProcessedGravityMeasurements.isEmpty())
        assertTrue(alreadyProcessedGyroscopeMeasurements.isEmpty())
        assertTrue(foundAccelerometerMeasurements.isEmpty())
        assertTrue(foundGravityMeasurements.isEmpty())
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

        val hasPreviousAttitudeMeasurement: Boolean? =
            syncer.getPrivateProperty("hasPreviousAttitudeMeasurement")
        requireNotNull(hasPreviousAttitudeMeasurement)
        assertFalse(hasPreviousAttitudeMeasurement)

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

        val lastNotifiedGravityTimestamp: Long? =
            syncer.getPrivateProperty("lastNotifiedGravityTimestamp")
        requireNotNull(lastNotifiedGravityTimestamp)
        assertEquals(0L, lastNotifiedGravityTimestamp)

        val lastNotifiedGyroscopeTimestamp: Long? =
            syncer.getPrivateProperty("lastNotifiedGyroscopeTimestamp")
        requireNotNull(lastNotifiedGyroscopeTimestamp)
        assertEquals(0L, lastNotifiedGyroscopeTimestamp)
    }

    @Test
    fun start_whenNotRunningAndTimestampProvided_setsProvidedTimestamp() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

        assertFalse(syncer.running)
        assertEquals(0L, syncer.startTimestamp)

        val timestamp = System.nanoTime()
        assertFalse(syncer.start(timestamp))

        // check
        assertFalse(syncer.running)
        assertEquals(timestamp, syncer.startTimestamp)
    }

    @Test
    fun start_whenAttitudeCollectorFails_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

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

        assertFalse(syncer.running)

        val timestamp = System.nanoTime()
        assertFalse(syncer.start(timestamp))

        assertFalse(syncer.running)
        assertEquals(timestamp, syncer.startTimestamp)

        verify(exactly = 1) { attitudeSensorCollectorSpy.start(timestamp) }
        verify(exactly = 0) { accelerometerSensorCollectorSpy.start(timestamp) }
        verify(exactly = 0) { gravitySensorCollectorSpy.start(any()) }
        verify(exactly = 0) { gyroscopeSensorCollectorSpy.start(any()) }
    }

    @Test
    fun start_whenAccelerometerCollectorFails_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

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

        assertFalse(syncer.running)

        val timestamp = System.nanoTime()
        assertFalse(syncer.start(timestamp))

        assertFalse(syncer.running)
        assertEquals(timestamp, syncer.startTimestamp)

        verify(exactly = 1) { attitudeSensorCollectorSpy.start(timestamp) }
        verify(exactly = 1) { accelerometerSensorCollectorSpy.start(timestamp) }
        verify(exactly = 0) { gravitySensorCollectorSpy.start(any()) }
        verify(exactly = 0) { gyroscopeSensorCollectorSpy.start(any()) }
    }

    @Test
    fun start_whenGravityCollectorFails_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

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

        assertFalse(syncer.running)

        val timestamp = System.nanoTime()
        assertFalse(syncer.start(timestamp))

        assertFalse(syncer.running)
        assertEquals(timestamp, syncer.startTimestamp)

        verify(exactly = 1) { attitudeSensorCollectorSpy.start(timestamp) }
        verify(exactly = 1) { accelerometerSensorCollectorSpy.start(timestamp) }
        verify(exactly = 1) { gravitySensorCollectorSpy.start(any()) }
        verify(exactly = 0) { gyroscopeSensorCollectorSpy.start(any()) }
    }

    @Test
    fun start_whenGyroscopeCollectorFails_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

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

        assertFalse(syncer.running)

        val timestamp = System.nanoTime()
        assertFalse(syncer.start(timestamp))

        assertFalse(syncer.running)
        assertEquals(timestamp, syncer.startTimestamp)

        verify(exactly = 1) { attitudeSensorCollectorSpy.start(timestamp) }
        verify(exactly = 1) { accelerometerSensorCollectorSpy.start(timestamp) }
        verify(exactly = 1) { gravitySensorCollectorSpy.start(any()) }
        verify(exactly = 1) { gyroscopeSensorCollectorSpy.start(any()) }
    }

    @Test
    fun start_whenCollectorsSucceed_returnsTrue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

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

        assertFalse(syncer.running)

        val timestamp = System.nanoTime()
        assertTrue(syncer.start(timestamp))

        assertTrue(syncer.running)
        assertEquals(timestamp, syncer.startTimestamp)

        verify(exactly = 1) { attitudeSensorCollectorSpy.start(timestamp) }
        verify(exactly = 1) { accelerometerSensorCollectorSpy.start(timestamp) }
        verify(exactly = 1) { gravitySensorCollectorSpy.start(any()) }
        verify(exactly = 1) { gyroscopeSensorCollectorSpy.start(any()) }
    }

    @Test
    fun stop_stopsCollectorsInitializesCachesAndResetsProperties() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

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

        val attitudeMeasurements: ArrayDeque<AttitudeSensorMeasurement>? =
            syncer.getPrivateProperty("attitudeMeasurements")
        requireNotNull(attitudeMeasurements)
        attitudeMeasurements.add(AttitudeSensorMeasurement())

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

        val alreadyProcessedAttitudeMeasurements: ArrayDeque<AttitudeSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAttitudeMeasurements")
        requireNotNull(alreadyProcessedAttitudeMeasurements)
        alreadyProcessedAttitudeMeasurements.add(AttitudeSensorMeasurement())

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

        val foundAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("foundAccelerometerMeasurements")
        requireNotNull(foundAccelerometerMeasurements)
        foundAccelerometerMeasurements.add(AccelerometerSensorMeasurement())

        val foundGravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("foundGravityMeasurements")
        requireNotNull(foundGravityMeasurements)
        foundGravityMeasurements.add(GravitySensorMeasurement())

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

        syncer.setPrivateProperty("hasPreviousAttitudeMeasurement", true)
        syncer.setPrivateProperty("hasPreviousAccelerometerMeasurement", true)
        syncer.setPrivateProperty("hasPreviousGravityMeasurement", true)
        syncer.setPrivateProperty("hasPreviousGyroscopeMeasurement", true)
        syncer.setPrivateProperty("lastNotifiedTimestamp", 1L)
        syncer.setPrivateProperty("lastNotifiedAttitudeTimestamp", 2L)
        syncer.setPrivateProperty("lastNotifiedAccelerometerTimestamp", 3L)
        syncer.setPrivateProperty("lastNotifiedGravityTimestamp", 4L)
        syncer.setPrivateProperty("lastNotifiedGyroscopeTimestamp", 5L)

        val stopping1: Boolean? =
            getPrivateProperty(SensorMeasurementSyncer::class, syncer, "stopping")
        requireNotNull(stopping1)
        assertFalse(stopping1)

        syncer.stop()

        // check
        verify(exactly = 1) { attitudeSensorCollectorSpy.stop() }
        verify(exactly = 1) { accelerometerSensorCollectorSpy.stop() }
        verify(exactly = 1) { gravitySensorCollectorSpy.stop() }
        verify(exactly = 1) { gyroscopeSensorCollectorSpy.stop() }
        assertEquals(1, attitudeMeasurements.size)
        assertEquals(1, accelerometerMeasurements.size)
        assertEquals(1, gravityMeasurements.size)
        assertEquals(1, gyroscopeMeasurements.size)
        assertEquals(1, alreadyProcessedAttitudeMeasurements.size)
        assertEquals(1, alreadyProcessedAccelerometerMeasurements.size)
        assertEquals(1, alreadyProcessedGravityMeasurements.size)
        assertEquals(1, alreadyProcessedGyroscopeMeasurements.size)
        assertEquals(1, foundAccelerometerMeasurements.size)
        assertEquals(1, foundGravityMeasurements.size)
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

        val hasPreviousAttitudeMeasurement: Boolean? =
            syncer.getPrivateProperty("hasPreviousAttitudeMeasurement")
        requireNotNull(hasPreviousAttitudeMeasurement)
        assertFalse(hasPreviousAttitudeMeasurement)

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

        val lastNotifiedGravityTimestamp: Long? =
            syncer.getPrivateProperty("lastNotifiedGravityTimestamp")
        requireNotNull(lastNotifiedGravityTimestamp)
        assertEquals(0L, lastNotifiedGravityTimestamp)

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
    fun attitudeSensorCollector_whenAccuracyChangedListenerAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

        val attitudeSensorCollector: BufferedAttitudeSensorCollector? =
            syncer.getPrivateProperty("attitudeSensorCollector")
        requireNotNull(attitudeSensorCollector)

        val listener = attitudeSensorCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(attitudeSensorCollector, SensorAccuracy.HIGH)
    }

    @Test
    fun attitudeSensorCollector_whenAccuracyChangedListenerAndListenerAvailable_notifies() {
        val accuracyChangedListener =
            mockk<SensorMeasurementSyncer.OnAccuracyChangedListener<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
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
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
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
        val bufferFilledListener =
            mockk<SensorMeasurementSyncer.OnBufferFilledListener<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
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
            bufferFilledListener.onBufferFilled(
                syncer,
                SensorType.ABSOLUTE_ATTITUDE
            )
        }
        assertTrue(syncer.running)
    }

    @Test
    fun attitudeSensorCollector_whenMeasurementAndNoMeasurementsAvailable_callsCollector() {
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
            context,
            syncedMeasurementListener = syncedMeasurementListener
        )

        val attitudeSensorCollector: BufferedAttitudeSensorCollector? =
            syncer.getPrivateProperty("attitudeSensorCollector")
        requireNotNull(attitudeSensorCollector)
        val attitudeSensorCollectorSpy = spyk(attitudeSensorCollector)
        every { attitudeSensorCollectorSpy.getMeasurementsBeforePosition(any()) }.returns(
            ArrayDeque()
        )
        syncer.setPrivateProperty("attitudeSensorCollector", attitudeSensorCollectorSpy)

        val listener = attitudeSensorCollector.measurementListener
        requireNotNull(listener)

        listener.onMeasurement(attitudeSensorCollectorSpy, AttitudeSensorMeasurement(), 0)

        verify(exactly = 1) {
            attitudeSensorCollectorSpy.getMeasurementsBeforePosition(0)
        }
        verify { syncedMeasurementListener wasNot Called }
        assertNull(syncer.mostRecentTimestamp)
    }

    @Test
    fun attitudeSensorCollector_whenMeasurementAndMeasurementsAvailable_updatesMostRecentTimestampCopiesMeasurementsAndProcessesThem() {
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
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

        val attitude = getAttitude()
        val randomizer = UniformRandomizer()
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

        val listener = attitudeSensorCollector.measurementListener
        requireNotNull(listener)

        listener.onMeasurement(attitudeSensorCollectorSpy, AttitudeSensorMeasurement(), 0)

        verify(exactly = 1) {
            attitudeSensorCollectorSpy.getMeasurementsBeforePosition(0)
        }
        assertEquals(timestamp, syncer.mostRecentTimestamp)

        val attitudeMeasurements: ArrayDeque<AttitudeSensorMeasurement>? =
            syncer.getPrivateProperty("attitudeMeasurements")
        requireNotNull(attitudeMeasurements)
        assertEquals(1, attitudeMeasurements.size)
        val attitudeMeasurement2 = attitudeMeasurements.peek()
        requireNotNull(attitudeMeasurement2)
        assertNotSame(attitudeMeasurement, attitudeMeasurement2)
        assertEquals(attitude, attitudeMeasurement2.attitude)
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

        val alreadyProcessedGravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGravityMeasurements")
        requireNotNull(alreadyProcessedGravityMeasurements)
        assertTrue(alreadyProcessedGravityMeasurements.isEmpty())

        val alreadyProcessedGyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGyroscopeMeasurements")
        requireNotNull(alreadyProcessedGyroscopeMeasurements)
        assertTrue(alreadyProcessedGyroscopeMeasurements.isEmpty())

        verify { syncedMeasurementListener wasNot Called }
        assertTrue(syncer.running)
    }

    @Test
    fun accelerometerSensorCollector_whenAccuracyChangedListenerAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

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
            mockk<SensorMeasurementSyncer.OnAccuracyChangedListener<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
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
            AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
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
            mockk<SensorMeasurementSyncer.OnBufferFilledListener<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
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
    fun accelerometerSensorCollector_whenMeasurementAndMostRecentTimestampNotDefined_makesNoAction() {
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
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
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
            context,
            syncedMeasurementListener = syncedMeasurementListener
        )

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            ArrayDeque()
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
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
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
        assertEquals(bx, accelerometerMeasurement2.bx)
        assertEquals(by, accelerometerMeasurement2.by)
        assertEquals(bz, accelerometerMeasurement2.bz)
        assertEquals(timestamp, accelerometerMeasurement2.timestamp)
        assertEquals(SensorAccuracy.HIGH, accelerometerMeasurement2.accuracy)
        assertEquals(syncer.accelerometerSensorType, accelerometerMeasurement2.sensorType)

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
    fun gravitySensorCollector_whenAccuracyChangedListenerAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

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
            mockk<SensorMeasurementSyncer.OnAccuracyChangedListener<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
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
            AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
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
            mockk<SensorMeasurementSyncer.OnBufferFilledListener<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
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
    fun gravitySensorCollector_whenMeasurementAndMostRecentTimestampNotDefined_makesNoAction() {
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
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
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
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
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
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
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(context)

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
            mockk<SensorMeasurementSyncer.OnAccuracyChangedListener<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
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
            AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
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
        val bufferFilledListener =
            mockk<SensorMeasurementSyncer.OnBufferFilledListener<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer =
            AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
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
    fun gyroscopeSensorCollector_whenMeasurementAndMostRecentTimestampNotDefined_makesNoAction() {
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
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
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
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
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer>>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
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
    fun sensorCollectors_whenAllMeasurements_notifiesSyncedMeasurement() {
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
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

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)

        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val gravityTimestamp = attitudeTimestamp - 1
        val gravityMeasurement1 = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            gravityTimestamp,
            SensorAccuracy.HIGH
        )
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

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)

        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val gyroscopeTimestamp = attitudeTimestamp - 1
        val gyroscopeMeasurement1 = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            wbx,
            wby,
            wbz,
            gyroscopeTimestamp,
            SensorAccuracy.MEDIUM,
            syncer.gyroscopeSensorType
        )
        val gyroscopeMeasurementsBeforeTimestamp = ArrayDeque<GyroscopeSensorMeasurement>()
        gyroscopeMeasurementsBeforeTimestamp.add(gyroscopeMeasurement1)
        every { gyroscopeSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            gyroscopeMeasurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

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

        val gyroscopeListener = gyroscopeSensorCollector.measurementListener
        requireNotNull(gyroscopeListener)

        val gravityListener = gravitySensorCollector.measurementListener
        requireNotNull(gravityListener)

        // process gravity measurement
        gravityListener.onMeasurement(
            gravitySensorCollectorSpy,
            GravitySensorMeasurement(),
            0
        )

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
        assertEquals(SensorAccuracy.MEDIUM, gyroscopeMeasurement2.accuracy)
        assertEquals(syncer.gyroscopeSensorType, gyroscopeMeasurement2.sensorType)

        val alreadyProcessedGyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGyroscopeMeasurements")
        requireNotNull(alreadyProcessedGyroscopeMeasurements)
        assertTrue(alreadyProcessedGyroscopeMeasurements.isEmpty())

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
        assertTrue(gravityMeasurements.isEmpty())

        val alreadyProcessedAttitudeMeasurements: ArrayDeque<AttitudeSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAttitudeMeasurements")
        requireNotNull(alreadyProcessedAttitudeMeasurements)
        assertTrue(alreadyProcessedAttitudeMeasurements.isEmpty())
        assertTrue(alreadyProcessedGravityMeasurements.isEmpty())

        val slot = slot<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement>()
        verify(exactly = 1) {
            syncedMeasurementListener.onSyncedMeasurements(
                syncer,
                capture(slot)
            )
        }

        val syncedMeasurement = slot.captured
        assertEquals(gyroscopeTimestamp, syncedMeasurement.timestamp)
        val syncedAttitudeMeasurement = syncedMeasurement.attitudeMeasurement
        requireNotNull(syncedAttitudeMeasurement)
        assertEquals(attitude, syncedAttitudeMeasurement.attitude)
        assertEquals(headingAccuracyRadians, syncedAttitudeMeasurement.headingAccuracy)
        assertEquals(attitudeTimestamp, syncedAttitudeMeasurement.timestamp)
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
        assertEquals(syncer.accelerometerSensorType, syncedAccelerometerMeasurement.sensorType)
        val syncedGravityMeasurement = syncedMeasurement.gravityMeasurement
        requireNotNull(syncedGravityMeasurement)
        assertEquals(gx, syncedGravityMeasurement.gx, 0.0f)
        assertEquals(gy, syncedGravityMeasurement.gy, 0.0f)
        assertEquals(gz, syncedGravityMeasurement.gz, 0.0f)
        assertEquals(gravityTimestamp, syncedGravityMeasurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, syncedGravityMeasurement.accuracy)
        val syncedGyroscopeMeasurement = syncedMeasurement.gyroscopeMeasurement
        requireNotNull(syncedGyroscopeMeasurement)
        assertEquals(wx, syncedGyroscopeMeasurement.wx, 0.0f)
        assertEquals(wy, syncedGyroscopeMeasurement.wy, 0.0f)
        assertEquals(wz, syncedGyroscopeMeasurement.wz, 0.0f)
        assertEquals(wbx, syncedGyroscopeMeasurement.bx)
        assertEquals(wby, syncedGyroscopeMeasurement.by)
        assertEquals(wbz, syncedGyroscopeMeasurement.bz)
        assertEquals(gyroscopeTimestamp, syncedGyroscopeMeasurement.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, syncedGyroscopeMeasurement.accuracy)
        assertEquals(syncer.gyroscopeSensorType, syncedGyroscopeMeasurement.sensorType)
    }

    @Test
    fun cleanupStaleMeasurements_whenStaleMeasurementsAvailable_notifies() {
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val staleDetectedMeasurementsListener =
            mockk<SensorMeasurementSyncer.OnStaleDetectedMeasurementsListener<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
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
            mostRecentTimestamp - 2 * AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer.DEFAULT_STALE_OFFSET_NANOS
        val gravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("gravityMeasurements")
        requireNotNull(gravityMeasurements)
        val randomizer = UniformRandomizer()
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

        val gyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("gyroscopeMeasurements")
        requireNotNull(gyroscopeMeasurements)
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
            SensorAccuracy.MEDIUM,
            syncer.gyroscopeSensorType
        )
        gyroscopeMeasurements.add(staleGyroscopeMeasurement)

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

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)

        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val gravityTimestamp = attitudeTimestamp - 1
        val gravityMeasurement1 = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            gravityTimestamp,
            SensorAccuracy.HIGH
        )
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

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)

        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val gyroscopeTimestamp = attitudeTimestamp - 1
        val gyroscopeMeasurement1 = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            wbx,
            wby,
            wbz,
            gyroscopeTimestamp,
            SensorAccuracy.MEDIUM,
            syncer.gyroscopeSensorType
        )
        val gyroscopeMeasurementsBeforeTimestamp = ArrayDeque<GyroscopeSensorMeasurement>()
        gyroscopeMeasurementsBeforeTimestamp.add(gyroscopeMeasurement1)
        every { gyroscopeSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            gyroscopeMeasurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

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

        val gyroscopeListener = gyroscopeSensorCollectorSpy.measurementListener
        requireNotNull(gyroscopeListener)

        val gravityListener = gravitySensorCollector.measurementListener
        requireNotNull(gravityListener)

        // process gravity measurement
        gravityListener.onMeasurement(
            gravitySensorCollectorSpy,
            GravitySensorMeasurement(),
            0
        )

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
        assertEquals(SensorAccuracy.HIGH, gravityMeasurement2.accuracy)
        val gravityMeasurement3 = gravityMeasurements.last
        requireNotNull(gravityMeasurement3)
        assertEquals(gx, gravityMeasurement3.gx, 0.0f)
        assertEquals(gy, gravityMeasurement3.gy, 0.0f)
        assertEquals(gz, gravityMeasurement3.gz, 0.0f)
        assertEquals(gravityTimestamp, gravityMeasurement3.timestamp)
        assertEquals(SensorAccuracy.HIGH, gravityMeasurement3.accuracy)

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
        assertEquals(SensorAccuracy.MEDIUM, gyroscopeMeasurement2.accuracy)
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
        assertEquals(SensorAccuracy.MEDIUM, gyroscopeMeasurement3.accuracy)
        assertEquals(syncer.gyroscopeSensorType, gyroscopeMeasurement3.sensorType)

        val alreadyProcessedGyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGyroscopeMeasurements")
        requireNotNull(alreadyProcessedGyroscopeMeasurements)
        assertTrue(alreadyProcessedGyroscopeMeasurements.isEmpty())

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
        assertTrue(gravityMeasurements.isEmpty())
        assertTrue(gyroscopeMeasurements.isEmpty())

        val alreadyProcessedAttitudeMeasurements: ArrayDeque<AttitudeSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAttitudeMeasurements")
        requireNotNull(alreadyProcessedAttitudeMeasurements)
        assertTrue(alreadyProcessedAttitudeMeasurements.isEmpty())
        assertTrue(alreadyProcessedGravityMeasurements.isEmpty())
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())
        assertTrue(alreadyProcessedGyroscopeMeasurements.isEmpty())

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
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val staleDetectedMeasurementsListener =
            mockk<SensorMeasurementSyncer.OnStaleDetectedMeasurementsListener<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
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
            mostRecentTimestamp - 2 * AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer.DEFAULT_STALE_OFFSET_NANOS
        val gravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("gravityMeasurements")
        requireNotNull(gravityMeasurements)
        val randomizer = UniformRandomizer()
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

        val gyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("gyroscopeMeasurements")
        requireNotNull(gyroscopeMeasurements)
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
            SensorAccuracy.MEDIUM,
            syncer.gyroscopeSensorType
        )
        gyroscopeMeasurements.add(staleGyroscopeMeasurement)

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

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)

        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val gravityTimestamp = attitudeTimestamp - 1
        val gravityMeasurement1 = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            gravityTimestamp,
            SensorAccuracy.HIGH
        )
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

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)

        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val gyroscopeTimestamp = attitudeTimestamp - 1
        val gyroscopeMeasurement1 = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            wbx,
            wby,
            wbz,
            gyroscopeTimestamp,
            SensorAccuracy.MEDIUM,
            syncer.gyroscopeSensorType
        )
        val gyroscopeMeasurementsBeforeTimestamp = ArrayDeque<GyroscopeSensorMeasurement>()
        gyroscopeMeasurementsBeforeTimestamp.add(gyroscopeMeasurement1)
        every { gyroscopeSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            gyroscopeMeasurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

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

        val gyroscopeListener = gyroscopeSensorCollectorSpy.measurementListener
        requireNotNull(gyroscopeListener)

        val gravityListener = gravitySensorCollector.measurementListener
        requireNotNull(gravityListener)

        // process gravity measurement
        gravityListener.onMeasurement(
            gravitySensorCollectorSpy,
            GravitySensorMeasurement(),
            0
        )

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
        assertEquals(SensorAccuracy.HIGH, gravityMeasurement2.accuracy)
        val gravityMeasurement3 = gravityMeasurements.last
        requireNotNull(gravityMeasurement3)
        assertEquals(gx, gravityMeasurement3.gx, 0.0f)
        assertEquals(gy, gravityMeasurement3.gy, 0.0f)
        assertEquals(gz, gravityMeasurement3.gz, 0.0f)
        assertEquals(gravityTimestamp, gravityMeasurement3.timestamp)
        assertEquals(SensorAccuracy.HIGH, gravityMeasurement3.accuracy)

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
        assertEquals(syncer.accelerometerSensorType, accelerometerMeasurement2.sensorType)
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
        assertEquals(syncer.accelerometerSensorType, accelerometerMeasurement3.sensorType)

        val alreadyProcessedAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())

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
        assertEquals(SensorAccuracy.MEDIUM, gyroscopeMeasurement2.accuracy)
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
        assertEquals(SensorAccuracy.MEDIUM, gyroscopeMeasurement3.accuracy)
        assertEquals(syncer.gyroscopeSensorType, gyroscopeMeasurement3.sensorType)

        val alreadyProcessedGyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGyroscopeMeasurements")
        requireNotNull(alreadyProcessedGyroscopeMeasurements)
        assertTrue(alreadyProcessedGyroscopeMeasurements.isEmpty())

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
        assertTrue(gravityMeasurements.isEmpty())
        assertTrue(gyroscopeMeasurements.isEmpty())

        val alreadyProcessedAttitudeMeasurements: ArrayDeque<AttitudeSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAttitudeMeasurements")
        requireNotNull(alreadyProcessedAttitudeMeasurements)
        assertFalse(alreadyProcessedAttitudeMeasurements.isEmpty())
        assertFalse(alreadyProcessedAccelerometerMeasurements.isEmpty())
        assertTrue(alreadyProcessedGyroscopeMeasurements.isEmpty())
        assertFalse(alreadyProcessedGravityMeasurements.isEmpty())

        verify { staleDetectedMeasurementsListener wasNot Called }
    }

    @Test
    fun sensorCollectors_whenPreviousGravityMeasurement_notifies() {
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
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

        // set previous attitude, accelerometer, gravity and gyroscope timestamp
        syncer.setPrivateProperty("hasPreviousAttitudeMeasurement", true)
        val previousAttitudeMeasurement: AttitudeSensorMeasurement? =
            syncer.getPrivateProperty("previousAttitudeMeasurement")
        requireNotNull(previousAttitudeMeasurement)
        previousAttitudeMeasurement.timestamp = mostRecentTimestamp - 1
        syncer.setPrivateProperty("hasPreviousGravityMeasurement", true)
        syncer.setPrivateProperty("lastNotifiedTimestamp", attitudeTimestamp - 1)
        syncer.setPrivateProperty("lastNotifiedAttitudeTimestamp", attitudeTimestamp - 1)
        syncer.setPrivateProperty("lastNotifiedGravityTimestamp", attitudeTimestamp - 2)
        syncer.setPrivateProperty("hasPreviousAccelerometerMeasurement", true)
        syncer.setPrivateProperty("hasPreviousGyroscopeMeasurement", true)
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
        previousGyroscopeMeasurement.timestamp = attitudeTimestamp - 1
        previousGyroscopeMeasurement.accuracy = SensorAccuracy.MEDIUM
        previousGyroscopeMeasurement.sensorType = syncer.gyroscopeSensorType

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)

        val wx2 = randomizer.nextFloat()
        val wy2 = randomizer.nextFloat()
        val wz2 = randomizer.nextFloat()
        val gyroscopeTimestamp = attitudeTimestamp - 1
        val gyroscopeMeasurement1 = GyroscopeSensorMeasurement(
            wx2,
            wy2,
            wz2,
            wbx,
            wby,
            wbz,
            gyroscopeTimestamp,
            SensorAccuracy.MEDIUM,
            syncer.gyroscopeSensorType
        )
        val gyroscopeMeasurementsBeforeTimestamp = ArrayDeque<GyroscopeSensorMeasurement>()
        gyroscopeMeasurementsBeforeTimestamp.add(gyroscopeMeasurement1)
        every { gyroscopeSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            gyroscopeMeasurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        val previousAccelerometerMeasurement: AccelerometerSensorMeasurement? =
            syncer.getPrivateProperty("previousAccelerometerMeasurement")
        requireNotNull(previousAccelerometerMeasurement)
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

        val previousGravityMeasurement: GravitySensorMeasurement? =
            syncer.getPrivateProperty("previousGravityMeasurement")
        requireNotNull(previousGravityMeasurement)
        val gx1 = randomizer.nextFloat()
        val gy1 = randomizer.nextFloat()
        val gz1 = randomizer.nextFloat()
        previousGravityMeasurement.gx = gx1
        previousGravityMeasurement.gy = gy1
        previousGravityMeasurement.gz = gz1
        previousGravityMeasurement.timestamp = attitudeTimestamp - 1
        previousGravityMeasurement.accuracy = SensorAccuracy.HIGH

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)

        val gx2 = randomizer.nextFloat()
        val gy2 = randomizer.nextFloat()
        val gz2 = randomizer.nextFloat()
        val gravityTimestamp = attitudeTimestamp - 1
        val gravityMeasurement1 = GravitySensorMeasurement(
            gx2,
            gy2,
            gz2,
            gravityTimestamp,
            SensorAccuracy.HIGH
        )
        val gravityMeasurementsBeforeTimestamp = ArrayDeque<GravitySensorMeasurement>()
        gravityMeasurementsBeforeTimestamp.add(gravityMeasurement1)
        every { gravitySensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            gravityMeasurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)

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

        val gyroscopeListener = gyroscopeSensorCollectorSpy.measurementListener
        requireNotNull(gyroscopeListener)

        val gravityListener = gravitySensorCollector.measurementListener
        requireNotNull(gravityListener)

        // process gravity measurement
        gravityListener.onMeasurement(
            gravitySensorCollectorSpy,
            GravitySensorMeasurement(),
            0
        )

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
        assertEquals(SensorAccuracy.MEDIUM, gyroscopeMeasurement2.accuracy)
        assertEquals(syncer.gyroscopeSensorType, gyroscopeMeasurement2.sensorType)

        val alreadyProcessedGyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGyroscopeMeasurements")
        requireNotNull(alreadyProcessedGyroscopeMeasurements)
        assertTrue(alreadyProcessedGyroscopeMeasurements.isEmpty())

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
        assertEquals(1, gravityMeasurements.size)
        assertSame(gravityMeasurement2, gravityMeasurements.first())

        val alreadyProcessedAttitudeMeasurements: ArrayDeque<AttitudeSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAttitudeMeasurements")
        requireNotNull(alreadyProcessedAttitudeMeasurements)
        assertTrue(alreadyProcessedAttitudeMeasurements.isEmpty())
        assertTrue(alreadyProcessedGravityMeasurements.isEmpty())

        val slot = slot<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement>()
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
        assertEquals(gyroscopeTimestamp, syncedGyroscopeMeasurement.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, syncedGyroscopeMeasurement.accuracy)
        assertEquals(syncer.gyroscopeSensorType, syncedGyroscopeMeasurement.sensorType)
        val syncedGravityMeasurement = syncedMeasurement.gravityMeasurement
        requireNotNull(syncedGravityMeasurement)
        assertEquals(gx1, syncedGravityMeasurement.gx, 0.0f)
        assertEquals(gy1, syncedGravityMeasurement.gy, 0.0f)
        assertEquals(gz1, syncedGravityMeasurement.gz, 0.0f)
        assertEquals(gravityTimestamp, syncedGravityMeasurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, syncedGravityMeasurement.accuracy)
    }

    @Test
    fun sensorCollectors_whenNoPreviousGyroscopeMeasurement_notifies() {
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
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

        // set previous attitude, accelerometer, gravity and gyroscope timestamp
        syncer.setPrivateProperty("hasPreviousAttitudeMeasurement", true)
        val previousAttitudeMeasurement: AttitudeSensorMeasurement? =
            syncer.getPrivateProperty("previousAttitudeMeasurement")
        requireNotNull(previousAttitudeMeasurement)
        previousAttitudeMeasurement.timestamp = mostRecentTimestamp - 1
        syncer.setPrivateProperty("hasPreviousGravityMeasurement", true)
        syncer.setPrivateProperty("lastNotifiedTimestamp", attitudeTimestamp - 1)
        syncer.setPrivateProperty("lastNotifiedAttitudeTimestamp", attitudeTimestamp - 1)
        syncer.setPrivateProperty("lastNotifiedGravityTimestamp", attitudeTimestamp - 2)
        syncer.setPrivateProperty("hasPreviousAccelerometerMeasurement", true)
        syncer.setPrivateProperty("hasPreviousGyroscopeMeasurement", true)
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
        previousGyroscopeMeasurement.timestamp = attitudeTimestamp - 1
        previousGyroscopeMeasurement.accuracy = SensorAccuracy.MEDIUM
        previousGyroscopeMeasurement.sensorType = syncer.gyroscopeSensorType

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)

        val wx2 = randomizer.nextFloat()
        val wy2 = randomizer.nextFloat()
        val wz2 = randomizer.nextFloat()
        val gyroscopeMeasurement1 = GyroscopeSensorMeasurement(
            wx2,
            wy2,
            wz2,
            wbx,
            wby,
            wbz,
            attitudeTimestamp,
            SensorAccuracy.MEDIUM,
            syncer.gyroscopeSensorType
        )
        val gyroscopeMeasurementsBeforeTimestamp = ArrayDeque<GyroscopeSensorMeasurement>()
        gyroscopeMeasurementsBeforeTimestamp.add(gyroscopeMeasurement1)
        every { gyroscopeSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            gyroscopeMeasurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        val previousAccelerometerMeasurement: AccelerometerSensorMeasurement? =
            syncer.getPrivateProperty("previousAccelerometerMeasurement")
        requireNotNull(previousAccelerometerMeasurement)
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
        val accelerometerMeasurement1 = AccelerometerSensorMeasurement(
            ax2,
            ay2,
            az2,
            abx,
            aby,
            abz,
            attitudeTimestamp,
            SensorAccuracy.HIGH
        )
        val accelerometerMeasurementsBeforeTimestamp = ArrayDeque<AccelerometerSensorMeasurement>()
        accelerometerMeasurementsBeforeTimestamp.add(accelerometerMeasurement1)
        every { accelerometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            accelerometerMeasurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        val previousGravityMeasurement: GravitySensorMeasurement? =
            syncer.getPrivateProperty("previousGravityMeasurement")
        requireNotNull(previousGravityMeasurement)
        val gx1 = randomizer.nextFloat()
        val gy1 = randomizer.nextFloat()
        val gz1 = randomizer.nextFloat()
        previousGravityMeasurement.gx = gx1
        previousGravityMeasurement.gy = gy1
        previousGravityMeasurement.gz = gz1
        previousGravityMeasurement.timestamp = attitudeTimestamp - 1
        previousGravityMeasurement.accuracy = SensorAccuracy.HIGH

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)

        val gx2 = randomizer.nextFloat()
        val gy2 = randomizer.nextFloat()
        val gz2 = randomizer.nextFloat()
        val gravityTimestamp = attitudeTimestamp - 3
        val gravityMeasurement1 = GravitySensorMeasurement(
            gx2,
            gy2,
            gz2,
            gravityTimestamp,
            SensorAccuracy.HIGH
        )
        val gravityMeasurementsBeforeTimestamp = ArrayDeque<GravitySensorMeasurement>()
        gravityMeasurementsBeforeTimestamp.add(gravityMeasurement1)
        every { gravitySensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            gravityMeasurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)

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

        val gyroscopeListener = gyroscopeSensorCollectorSpy.measurementListener
        requireNotNull(gyroscopeListener)

        val gravityListener = gravitySensorCollector.measurementListener
        requireNotNull(gravityListener)

        // process gravity measurement
        gravityListener.onMeasurement(
            gravitySensorCollectorSpy,
            GravitySensorMeasurement(),
            0
        )

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
        assertEquals(attitudeTimestamp, gyroscopeMeasurement2.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, gyroscopeMeasurement2.accuracy)
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
        assertEquals(attitudeTimestamp, accelerometerMeasurement2.timestamp)
        assertEquals(SensorAccuracy.HIGH, accelerometerMeasurement2.accuracy)
        assertEquals(syncer.accelerometerSensorType, accelerometerMeasurement2.sensorType)

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
        assertEquals(1, gravityMeasurements.size)
        assertSame(gravityMeasurement2, gravityMeasurements.first())

        val alreadyProcessedAttitudeMeasurements: ArrayDeque<AttitudeSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAttitudeMeasurements")
        requireNotNull(alreadyProcessedAttitudeMeasurements)
        assertTrue(alreadyProcessedAttitudeMeasurements.isEmpty())
        assertTrue(alreadyProcessedGravityMeasurements.isEmpty())

        val slot = slot<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement>()
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
        assertEquals(ax2, syncedAccelerometerMeasurement.ax, 0.0f)
        assertEquals(ay2, syncedAccelerometerMeasurement.ay, 0.0f)
        assertEquals(az2, syncedAccelerometerMeasurement.az, 0.0f)
        assertEquals(abx, syncedAccelerometerMeasurement.bx)
        assertEquals(aby, syncedAccelerometerMeasurement.by)
        assertEquals(abz, syncedAccelerometerMeasurement.bz)
        assertEquals(attitudeTimestamp, syncedAccelerometerMeasurement.timestamp)
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
        assertEquals(attitudeTimestamp - 1, syncedGyroscopeMeasurement.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, syncedGyroscopeMeasurement.accuracy)
        assertEquals(syncer.gyroscopeSensorType, syncedGyroscopeMeasurement.sensorType)
        val syncedGravityMeasurement = syncedMeasurement.gravityMeasurement
        requireNotNull(syncedGravityMeasurement)
        assertEquals(gx1, syncedGravityMeasurement.gx, 0.0f)
        assertEquals(gy1, syncedGravityMeasurement.gy, 0.0f)
        assertEquals(gz1, syncedGravityMeasurement.gz, 0.0f)
        assertEquals(attitudeTimestamp - 1, syncedGravityMeasurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, syncedGravityMeasurement.accuracy)
    }

    @Test
    fun sensorCollectors_whenNoPreviousMagnetometerMeasurement_notifies() {
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
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

        // set previous attitude, accelerometer, gravity and gyroscope timestamp
        syncer.setPrivateProperty("hasPreviousAttitudeMeasurement", true)
        val previousAttitudeMeasurement: AttitudeSensorMeasurement? =
            syncer.getPrivateProperty("previousAttitudeMeasurement")
        requireNotNull(previousAttitudeMeasurement)
        previousAttitudeMeasurement.timestamp = mostRecentTimestamp - 1
        syncer.setPrivateProperty("hasPreviousGravityMeasurement", true)
        syncer.setPrivateProperty("lastNotifiedTimestamp", attitudeTimestamp - 2)
        syncer.setPrivateProperty("lastNotifiedAttitudeTimestamp", attitudeTimestamp - 1)
        syncer.setPrivateProperty("lastNotifiedGravityTimestamp", attitudeTimestamp - 2)
        syncer.setPrivateProperty("hasPreviousAccelerometerMeasurement", true)
        syncer.setPrivateProperty("hasPreviousGyroscopeMeasurement", true)
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
        previousGyroscopeMeasurement.timestamp = attitudeTimestamp - 1
        previousGyroscopeMeasurement.accuracy = SensorAccuracy.MEDIUM
        previousGyroscopeMeasurement.sensorType = syncer.gyroscopeSensorType

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)

        val wx2 = randomizer.nextFloat()
        val wy2 = randomizer.nextFloat()
        val wz2 = randomizer.nextFloat()
        val gyroscopeMeasurement1 = GyroscopeSensorMeasurement(
            wx2,
            wy2,
            wz2,
            wbx,
            wby,
            wbz,
            attitudeTimestamp,
            SensorAccuracy.MEDIUM,
            syncer.gyroscopeSensorType
        )
        val gyroscopeMeasurementsBeforeTimestamp = ArrayDeque<GyroscopeSensorMeasurement>()
        gyroscopeMeasurementsBeforeTimestamp.add(gyroscopeMeasurement1)
        every { gyroscopeSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            gyroscopeMeasurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        val previousAccelerometerMeasurement: AccelerometerSensorMeasurement? =
            syncer.getPrivateProperty("previousAccelerometerMeasurement")
        requireNotNull(previousAccelerometerMeasurement)
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
        previousAccelerometerMeasurement.sensorType = syncer.accelerometerSensorType

        val accelerometerSensorCollector: BufferedAccelerometerSensorCollector? =
            syncer.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)

        val ax2 = randomizer.nextFloat()
        val ay2 = randomizer.nextFloat()
        val az2 = randomizer.nextFloat()
        val accelerometerMeasurement1 = AccelerometerSensorMeasurement(
            ax2,
            ay2,
            az2,
            abx,
            aby,
            abz,
            attitudeTimestamp,
            SensorAccuracy.HIGH
        )
        val accelerometerMeasurementsBeforeTimestamp = ArrayDeque<AccelerometerSensorMeasurement>()
        accelerometerMeasurementsBeforeTimestamp.add(accelerometerMeasurement1)
        every { accelerometerSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            accelerometerMeasurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        val previousGravityMeasurement: GravitySensorMeasurement? =
            syncer.getPrivateProperty("previousGravityMeasurement")
        requireNotNull(previousGravityMeasurement)
        val gx1 = randomizer.nextFloat()
        val gy1 = randomizer.nextFloat()
        val gz1 = randomizer.nextFloat()
        previousGravityMeasurement.gx = gx1
        previousGravityMeasurement.gy = gy1
        previousGravityMeasurement.gz = gz1
        previousGravityMeasurement.timestamp = attitudeTimestamp - 1
        previousGravityMeasurement.accuracy = SensorAccuracy.HIGH

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)

        val gx2 = randomizer.nextFloat()
        val gy2 = randomizer.nextFloat()
        val gz2 = randomizer.nextFloat()
        val gravityTimestamp = attitudeTimestamp - 1
        val gravityMeasurement1 = GravitySensorMeasurement(
            gx2,
            gy2,
            gz2,
            gravityTimestamp,
            SensorAccuracy.HIGH
        )
        val gravityMeasurementsBeforeTimestamp = ArrayDeque<GravitySensorMeasurement>()
        gravityMeasurementsBeforeTimestamp.add(gravityMeasurement1)
        every { gravitySensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            gravityMeasurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("gravitySensorCollector", gravitySensorCollectorSpy)

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

        val gyroscopeListener = gyroscopeSensorCollectorSpy.measurementListener
        requireNotNull(gyroscopeListener)

        val gravityListener = gravitySensorCollector.measurementListener
        requireNotNull(gravityListener)

        // process gravity measurement
        gravityListener.onMeasurement(
            gravitySensorCollectorSpy,
            GravitySensorMeasurement(),
            0
        )

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
        assertEquals(attitudeTimestamp, gyroscopeMeasurement2.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, gyroscopeMeasurement2.accuracy)
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
        assertEquals(attitudeTimestamp, accelerometerMeasurement2.timestamp)
        assertEquals(SensorAccuracy.HIGH, accelerometerMeasurement2.accuracy)
        assertEquals(syncer.accelerometerSensorType, accelerometerMeasurement2.sensorType)

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
        assertTrue(gravityMeasurements.isEmpty())

        val alreadyProcessedAttitudeMeasurements: ArrayDeque<AttitudeSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAttitudeMeasurements")
        requireNotNull(alreadyProcessedAttitudeMeasurements)
        assertTrue(alreadyProcessedAttitudeMeasurements.isEmpty())
        assertTrue(alreadyProcessedGravityMeasurements.isEmpty())

        val slot = slot<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement>()
        verify(exactly = 1) {
            syncedMeasurementListener.onSyncedMeasurements(
                syncer,
                capture(slot)
            )
        }

        val syncedMeasurement = slot.captured
        assertEquals(gravityTimestamp, syncedMeasurement.timestamp)
        val syncedAttitudeMeasurement = syncedMeasurement.attitudeMeasurement
        requireNotNull(syncedAttitudeMeasurement)
        assertEquals(attitude, syncedAttitudeMeasurement.attitude)
        assertEquals(headingAccuracyRadians, syncedAttitudeMeasurement.headingAccuracy)
        assertEquals(attitudeTimestamp, syncedAttitudeMeasurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, syncedAttitudeMeasurement.accuracy)
        assertEquals(syncer.attitudeSensorType, syncedAttitudeMeasurement.sensorType)
        val syncedAccelerometerMeasurement = syncedMeasurement.accelerometerMeasurement
        requireNotNull(syncedAccelerometerMeasurement)
        assertEquals(ax2, syncedAccelerometerMeasurement.ax, 0.0f)
        assertEquals(ay2, syncedAccelerometerMeasurement.ay, 0.0f)
        assertEquals(az2, syncedAccelerometerMeasurement.az, 0.0f)
        assertEquals(abx, syncedAccelerometerMeasurement.bx)
        assertEquals(aby, syncedAccelerometerMeasurement.by)
        assertEquals(abz, syncedAccelerometerMeasurement.bz)
        assertEquals(attitudeTimestamp, syncedAccelerometerMeasurement.timestamp)
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
        assertEquals(attitudeTimestamp - 1, syncedGyroscopeMeasurement.timestamp)
        assertEquals(SensorAccuracy.MEDIUM, syncedGyroscopeMeasurement.accuracy)
        assertEquals(syncer.gyroscopeSensorType, syncedGyroscopeMeasurement.sensorType)
        val syncedGravityMeasurement = syncedMeasurement.gravityMeasurement
        requireNotNull(syncedGravityMeasurement)
        assertEquals(gx2, syncedGravityMeasurement.gx, 0.0f)
        assertEquals(gy2, syncedGravityMeasurement.gy, 0.0f)
        assertEquals(gz2, syncedGravityMeasurement.gz, 0.0f)
        assertEquals(attitudeTimestamp - 1, syncedGravityMeasurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, syncedGravityMeasurement.accuracy)
    }

    @Test
    fun sensorCollectors_whenStopping_clearCollectionsAndResets() {
        val syncedMeasurementListener =
            mockk<SensorMeasurementSyncer.OnSyncedMeasurementsListener<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement, AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer>>(
                relaxUnitFun = true
            )
        val context = ApplicationProvider.getApplicationContext<Context>()
        val syncer = AttitudeAccelerometerGravityAndGyroscopeSensorMeasurementSyncer(
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

        val gravitySensorCollector: BufferedGravitySensorCollector? =
            syncer.getPrivateProperty("gravitySensorCollector")
        requireNotNull(gravitySensorCollector)
        val gravitySensorCollectorSpy = spyk(gravitySensorCollector)

        val randomizer = UniformRandomizer()
        val gx = randomizer.nextFloat()
        val gy = randomizer.nextFloat()
        val gz = randomizer.nextFloat()
        val gravityTimestamp = attitudeTimestamp - 1
        val gravityMeasurement1 = GravitySensorMeasurement(
            gx,
            gy,
            gz,
            gravityTimestamp,
            SensorAccuracy.HIGH
        )
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

        val gyroscopeSensorCollector: BufferedGyroscopeSensorCollector? =
            syncer.getPrivateProperty("gyroscopeSensorCollector")
        requireNotNull(gyroscopeSensorCollector)
        val gyroscopeSensorCollectorSpy = spyk(gyroscopeSensorCollector)

        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val wbx = randomizer.nextFloat()
        val wby = randomizer.nextFloat()
        val wbz = randomizer.nextFloat()
        val gyroscopeTimestamp = attitudeTimestamp - 1
        val gyroscopeMeasurement1 = GyroscopeSensorMeasurement(
            wx,
            wy,
            wz,
            wbx,
            wby,
            wbz,
            gyroscopeTimestamp,
            SensorAccuracy.MEDIUM,
            syncer.gyroscopeSensorType
        )
        val gyroscopeMeasurementsBeforeTimestamp = ArrayDeque<GyroscopeSensorMeasurement>()
        gyroscopeMeasurementsBeforeTimestamp.add(gyroscopeMeasurement1)
        every { gyroscopeSensorCollectorSpy.getMeasurementsBeforeTimestamp(any()) }.returns(
            gyroscopeMeasurementsBeforeTimestamp
        )
        syncer.setPrivateProperty("gyroscopeSensorCollector", gyroscopeSensorCollectorSpy)

        val attitudeSensorCollector: BufferedAttitudeSensorCollector? =
            syncer.getPrivateProperty("attitudeSensorCollector")
        requireNotNull(attitudeSensorCollector)
        val attitudeSensorCollectorSpy = spyk(attitudeSensorCollector)

        val attitude = getAttitude()
        val headingAcuracyRadians = Math.toRadians(randomizer.nextDouble(0.0, MAX_DEGREES)).toFloat()
        val attitudeMeasurement1 = AttitudeSensorMeasurement(
            attitude,
            headingAcuracyRadians,
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

        val gyroscopeListener = gyroscopeSensorCollector.measurementListener
        requireNotNull(gyroscopeListener)

        val gravityListener = gravitySensorCollector.measurementListener
        requireNotNull(gravityListener)

        // process gravity measurement
        gravityListener.onMeasurement(
            gravitySensorCollectorSpy,
            GravitySensorMeasurement(),
            0
        )

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
        accelerometerListener.onMeasurement(accelerometerSensorCollectorSpy, AccelerometerSensorMeasurement(), 0)

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
        assertEquals(syncer.accelerometerSensorType, accelerometerMeasurement2.sensorType)

        val alreadyProcessedAccelerometerMeasurements: ArrayDeque<AccelerometerSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedAccelerometerMeasurements")
        requireNotNull(alreadyProcessedAccelerometerMeasurements)
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())

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
        assertEquals(SensorAccuracy.MEDIUM, gyroscopeMeasurement2.accuracy)
        assertEquals(syncer.gyroscopeSensorType, gyroscopeMeasurement2.sensorType)

        val alreadyProcessedGyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("alreadyProcessedGyroscopeMeasurements")
        requireNotNull(alreadyProcessedGyroscopeMeasurements)
        assertTrue(alreadyProcessedGyroscopeMeasurements.isEmpty())

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
        val foundGravityMeasurements: ArrayDeque<GravitySensorMeasurement>? =
            syncer.getPrivateProperty("foundGravityMeasurements")
        requireNotNull(foundGravityMeasurements)
        val foundGyroscopeMeasurements: ArrayDeque<GyroscopeSensorMeasurement>? =
            syncer.getPrivateProperty("foundGyroscopeMeasurements")
        requireNotNull(foundGyroscopeMeasurements)
        assertTrue(attitudeMeasurements.isEmpty())
        assertTrue(accelerometerMeasurements.isEmpty())
        assertTrue(gravityMeasurements.isEmpty())
        assertTrue(gyroscopeMeasurements.isEmpty())
        assertTrue(alreadyProcessedAttitudeMeasurements.isEmpty())
        assertTrue(alreadyProcessedAccelerometerMeasurements.isEmpty())
        assertTrue(alreadyProcessedGravityMeasurements.isEmpty())
        assertTrue(alreadyProcessedGyroscopeMeasurements.isEmpty())
        assertTrue(foundAccelerometerMeasurements.isEmpty())
        assertTrue(foundGravityMeasurements.isEmpty())
        assertTrue(foundGyroscopeMeasurements.isEmpty())

        val slot = slot<AttitudeAccelerometerGravityAndGyroscopeSyncedSensorMeasurement>()
        verify(exactly = 1) {
            syncedMeasurementListener.onSyncedMeasurements(
                syncer,
                capture(slot)
            )
        }

        val syncedMeasurement = slot.captured
        assertEquals(gravityTimestamp, syncedMeasurement.timestamp)
        val syncedAttitudeMeasurement = syncedMeasurement.attitudeMeasurement
        requireNotNull(syncedAttitudeMeasurement)
        assertEquals(attitude, syncedAttitudeMeasurement.attitude)
        assertEquals(headingAcuracyRadians, syncedAttitudeMeasurement.headingAccuracy)
        assertEquals(attitudeTimestamp, syncedAttitudeMeasurement.timestamp)
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
        assertEquals(SensorAccuracy.MEDIUM, syncedGyroscopeMeasurement.accuracy)
        assertEquals(syncer.gyroscopeSensorType, syncedGyroscopeMeasurement.sensorType)
        val syncedGravityMeasurement = syncedMeasurement.gravityMeasurement
        requireNotNull(syncedGravityMeasurement)
        assertEquals(gx, syncedGravityMeasurement.gx, 0.0f)
        assertEquals(gy, syncedGravityMeasurement.gy, 0.0f)
        assertEquals(gz, syncedGravityMeasurement.gz, 0.0f)
        assertEquals(gravityTimestamp, syncedGravityMeasurement.timestamp)
        assertEquals(SensorAccuracy.HIGH, syncedGravityMeasurement.accuracy)
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