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
package com.irurueta.android.navigation.inertial.estimators.attitude

import android.content.Context
import android.location.Location
import androidx.test.core.app.ApplicationProvider
import com.irurueta.android.navigation.inertial.collectors.*
import com.irurueta.android.navigation.inertial.estimators.filter.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.MeanAveragingFilter
import com.irurueta.android.navigation.inertial.getPrivateProperty
import com.irurueta.android.navigation.inertial.processors.AccelerometerLeveledRelativeAttitudeProcessor
import com.irurueta.android.navigation.inertial.processors.BaseLeveledRelativeAttitudeProcessor
import com.irurueta.android.navigation.inertial.processors.LeveledRelativeAttitudeProcessor
import com.irurueta.android.navigation.inertial.setPrivateProperty
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType
import com.irurueta.statistics.UniformRandomizer
import io.mockk.*
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class LeveledRelativeAttitudeEstimator2Test {

    @Test
    fun constructor_whenRequiredProperties_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(context)

        // check
        assertSame(context, estimator.context)
        assertNull(estimator.location)
        assertEquals(SensorDelay.GAME, estimator.sensorDelay)
        assertTrue(estimator.useAccelerometer)
        assertTrue(estimator.startOffsetEnabled)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            estimator.accelerometerSensorType
        )
        assertNotNull(estimator.accelerometerAveragingFilter)
        assertTrue(estimator.accelerometerAveragingFilter is LowPassAveragingFilter)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            estimator.gyroscopeSensorType
        )
        assertFalse(estimator.useAccurateLevelingEstimator)
        assertTrue(estimator.useAccurateRelativeGyroscopeAttitudeEstimator)
        assertFalse(estimator.estimateCoordinateTransformation)
        assertTrue(estimator.estimateEulerAngles)
        assertNull(estimator.attitudeAvailableListener)
        assertNull(estimator.accuracyChangedListener)
        assertNull(estimator.bufferFilledListener)
        assertEquals(0.0, estimator.gyroscopeTimeIntervalSeconds, 0.0)
        assertFalse(estimator.running)
        assertTrue(estimator.useIndirectInterpolation)
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_INTERPOLATION_VALUE,
            estimator.interpolationValue,
            0.0
        )
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            estimator.indirectInterpolationWeight,
            0.0
        )
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_OUTLIER_THRESHOLD,
            estimator.outlierThreshold,
            0.0
        )
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            estimator.outlierPanicThreshold,
            0.0
        )
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_PANIC_COUNTER_THRESHOLD,
            estimator.panicCounterThreshold
        )

        // check internal properties
        val gravityAndGyroscopeSyncer: GravityAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("gravityAndGyroscopeSyncer")
        requireNotNull(gravityAndGyroscopeSyncer)

        assertSame(context, gravityAndGyroscopeSyncer.context)
        assertEquals(estimator.gyroscopeSensorType, gravityAndGyroscopeSyncer.gyroscopeSensorType)
        assertEquals(estimator.sensorDelay, gravityAndGyroscopeSyncer.gravitySensorDelay)
        assertEquals(estimator.sensorDelay, gravityAndGyroscopeSyncer.gyroscopeSensorDelay)
        assertEquals(
            GravityAndGyroscopeSensorMeasurementSyncer.DEFAULT_GRAVITY_CAPACITY,
            gravityAndGyroscopeSyncer.gravityCapacity
        )
        assertEquals(
            GravityAndGyroscopeSensorMeasurementSyncer.DEFAULT_GYROSCOPE_CAPACITY,
            gravityAndGyroscopeSyncer.gyroscopeCapacity
        )
        assertEquals(
            estimator.startOffsetEnabled,
            gravityAndGyroscopeSyncer.gravityStartOffsetEnabled
        )
        assertEquals(
            estimator.startOffsetEnabled,
            gravityAndGyroscopeSyncer.gyroscopeStartOffsetEnabled
        )
        assertFalse(gravityAndGyroscopeSyncer.stopWhenFilledBuffer)
        assertEquals(
            GravityAndGyroscopeSensorMeasurementSyncer.DEFAULT_STALE_OFFSET_NANOS,
            gravityAndGyroscopeSyncer.staleOffsetNanos
        )
        assertTrue(gravityAndGyroscopeSyncer.staleDetectionEnabled)
        assertNotNull(gravityAndGyroscopeSyncer.accuracyChangedListener)
        assertNotNull(gravityAndGyroscopeSyncer.bufferFilledListener)
        assertNotNull(gravityAndGyroscopeSyncer.syncedMeasurementListener)

        val accelerometerAndGyroscopeSyncer: AccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerAndGyroscopeSyncer")
        requireNotNull(accelerometerAndGyroscopeSyncer)

        assertSame(context, accelerometerAndGyroscopeSyncer.context)
        assertEquals(
            estimator.gyroscopeSensorType,
            accelerometerAndGyroscopeSyncer.gyroscopeSensorType
        )
        assertEquals(
            estimator.sensorDelay,
            accelerometerAndGyroscopeSyncer.accelerometerSensorDelay
        )
        assertEquals(estimator.sensorDelay, accelerometerAndGyroscopeSyncer.gyroscopeSensorDelay)
        assertEquals(
            AccelerometerAndGyroscopeSensorMeasurementSyncer.DEFAULT_ACCELEROMETER_CAPACITY,
            accelerometerAndGyroscopeSyncer.accelerometerCapacity
        )
        assertEquals(
            AccelerometerAndGyroscopeSensorMeasurementSyncer.DEFAULT_GYROSCOPE_CAPACITY,
            accelerometerAndGyroscopeSyncer.gyroscopeCapacity
        )
        assertEquals(
            estimator.startOffsetEnabled,
            accelerometerAndGyroscopeSyncer.accelerometerStartOffsetEnabled
        )
        assertEquals(
            estimator.startOffsetEnabled,
            accelerometerAndGyroscopeSyncer.gyroscopeStartOffsetEnabled
        )
        assertFalse(accelerometerAndGyroscopeSyncer.stopWhenFilledBuffer)
        assertEquals(
            AccelerometerAndGyroscopeSensorMeasurementSyncer.DEFAULT_STALE_OFFSET_NANOS,
            accelerometerAndGyroscopeSyncer.staleOffsetNanos
        )
        assertTrue(accelerometerAndGyroscopeSyncer.staleDetectionEnabled)
        assertNotNull(accelerometerAndGyroscopeSyncer.accuracyChangedListener)
        assertNotNull(accelerometerAndGyroscopeSyncer.bufferFilledListener)
        assertNotNull(accelerometerAndGyroscopeSyncer.syncedMeasurementListener)
    }

    @Test
    fun constructor_whenAllProperties_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val accelerometerAveragingFilter = MeanAveragingFilter()
        val attitudeAvailableListener =
            mockk<LeveledRelativeAttitudeEstimator2.OnAttitudeAvailableListener>()
        val accuracyChangedListener =
            mockk<LeveledRelativeAttitudeEstimator2.OnAccuracyChangedListener>()
        val bufferFilledListener = mockk<LeveledRelativeAttitudeEstimator2.OnBufferFilledListener>()
        val estimator = LeveledRelativeAttitudeEstimator2(
            context,
            location,
            SensorDelay.NORMAL,
            useAccelerometer = false,
            startOffsetEnabled = false,
            AccelerometerSensorType.ACCELEROMETER,
            accelerometerAveragingFilter,
            GyroscopeSensorType.GYROSCOPE,
            useAccurateLevelingEstimator = true,
            useAccurateRelativeGyroscopeAttitudeEstimator = false,
            estimateCoordinateTransformation = true,
            estimateEulerAngles = false,
            attitudeAvailableListener,
            accuracyChangedListener,
            bufferFilledListener
        )

        // check
        assertSame(context, estimator.context)
        assertSame(location, estimator.location)
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertFalse(estimator.useAccelerometer)
        assertFalse(estimator.startOffsetEnabled)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            estimator.accelerometerSensorType
        )
        assertSame(accelerometerAveragingFilter, estimator.accelerometerAveragingFilter)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            estimator.gyroscopeSensorType
        )
        assertTrue(estimator.useAccurateLevelingEstimator)
        assertFalse(estimator.useAccurateRelativeGyroscopeAttitudeEstimator)
        assertTrue(estimator.estimateCoordinateTransformation)
        assertFalse(estimator.estimateEulerAngles)
        assertSame(attitudeAvailableListener, estimator.attitudeAvailableListener)
        assertSame(accuracyChangedListener, estimator.accuracyChangedListener)
        assertSame(bufferFilledListener, estimator.bufferFilledListener)
        assertEquals(0.0, estimator.gyroscopeTimeIntervalSeconds, 0.0)
        assertFalse(estimator.running)
        assertTrue(estimator.useIndirectInterpolation)
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_INTERPOLATION_VALUE,
            estimator.interpolationValue,
            0.0
        )
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            estimator.indirectInterpolationWeight,
            0.0
        )
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_OUTLIER_THRESHOLD,
            estimator.outlierThreshold,
            0.0
        )
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            estimator.outlierPanicThreshold,
            0.0
        )
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_PANIC_COUNTER_THRESHOLD,
            estimator.panicCounterThreshold
        )

        // check internal properties
        val gravityAndGyroscopeSyncer: GravityAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("gravityAndGyroscopeSyncer")
        requireNotNull(gravityAndGyroscopeSyncer)

        assertSame(context, gravityAndGyroscopeSyncer.context)
        assertEquals(estimator.gyroscopeSensorType, gravityAndGyroscopeSyncer.gyroscopeSensorType)
        assertEquals(estimator.sensorDelay, gravityAndGyroscopeSyncer.gravitySensorDelay)
        assertEquals(estimator.sensorDelay, gravityAndGyroscopeSyncer.gyroscopeSensorDelay)
        assertEquals(
            GravityAndGyroscopeSensorMeasurementSyncer.DEFAULT_GRAVITY_CAPACITY,
            gravityAndGyroscopeSyncer.gravityCapacity
        )
        assertEquals(
            GravityAndGyroscopeSensorMeasurementSyncer.DEFAULT_GYROSCOPE_CAPACITY,
            gravityAndGyroscopeSyncer.gyroscopeCapacity
        )
        assertEquals(
            estimator.startOffsetEnabled,
            gravityAndGyroscopeSyncer.gravityStartOffsetEnabled
        )
        assertEquals(
            estimator.startOffsetEnabled,
            gravityAndGyroscopeSyncer.gyroscopeStartOffsetEnabled
        )
        assertFalse(gravityAndGyroscopeSyncer.stopWhenFilledBuffer)
        assertEquals(
            GravityAndGyroscopeSensorMeasurementSyncer.DEFAULT_STALE_OFFSET_NANOS,
            gravityAndGyroscopeSyncer.staleOffsetNanos
        )
        assertTrue(gravityAndGyroscopeSyncer.staleDetectionEnabled)
        assertNotNull(gravityAndGyroscopeSyncer.accuracyChangedListener)
        assertNotNull(gravityAndGyroscopeSyncer.bufferFilledListener)
        assertNotNull(gravityAndGyroscopeSyncer.syncedMeasurementListener)

        val accelerometerAndGyroscopeSyncer: AccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerAndGyroscopeSyncer")
        requireNotNull(accelerometerAndGyroscopeSyncer)

        assertSame(context, accelerometerAndGyroscopeSyncer.context)
        assertEquals(
            estimator.gyroscopeSensorType,
            accelerometerAndGyroscopeSyncer.gyroscopeSensorType
        )
        assertEquals(
            estimator.sensorDelay,
            accelerometerAndGyroscopeSyncer.accelerometerSensorDelay
        )
        assertEquals(estimator.sensorDelay, accelerometerAndGyroscopeSyncer.gyroscopeSensorDelay)
        assertEquals(
            AccelerometerAndGyroscopeSensorMeasurementSyncer.DEFAULT_ACCELEROMETER_CAPACITY,
            accelerometerAndGyroscopeSyncer.accelerometerCapacity
        )
        assertEquals(
            AccelerometerAndGyroscopeSensorMeasurementSyncer.DEFAULT_GYROSCOPE_CAPACITY,
            accelerometerAndGyroscopeSyncer.gyroscopeCapacity
        )
        assertEquals(
            estimator.startOffsetEnabled,
            accelerometerAndGyroscopeSyncer.accelerometerStartOffsetEnabled
        )
        assertEquals(
            estimator.startOffsetEnabled,
            accelerometerAndGyroscopeSyncer.gyroscopeStartOffsetEnabled
        )
        assertFalse(accelerometerAndGyroscopeSyncer.stopWhenFilledBuffer)
        assertEquals(
            AccelerometerAndGyroscopeSensorMeasurementSyncer.DEFAULT_STALE_OFFSET_NANOS,
            accelerometerAndGyroscopeSyncer.staleOffsetNanos
        )
        assertTrue(accelerometerAndGyroscopeSyncer.staleDetectionEnabled)
        assertNotNull(accelerometerAndGyroscopeSyncer.accuracyChangedListener)
        assertNotNull(accelerometerAndGyroscopeSyncer.bufferFilledListener)
        assertNotNull(accelerometerAndGyroscopeSyncer.syncedMeasurementListener)
    }

    @Test
    fun location_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(context)

        // check default value
        assertNull(estimator.location)
        assertFalse(estimator.running)

        // set new value
        val location = getLocation()
        estimator.location = location

        // check
        assertSame(location, estimator.location)

        // set new value
        estimator.location = null

        // check
        assertNull(estimator.location)
    }

    @Test
    fun location_whenRunningAndNotNull_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(context)

        assertFalse(estimator.running)
        assertNull(estimator.location)

        // set as running
        estimator.setPrivateProperty("running", true)

        assertTrue(estimator.running)

        val location = getLocation()
        estimator.location = location

        // check
        assertSame(location, estimator.location)
    }

    @Test(expected = IllegalStateException::class)
    fun location_whenRunningAndNull_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = LeveledRelativeAttitudeEstimator2(context, location)

        assertFalse(estimator.running)
        assertSame(location, estimator.location)

        // set as running
        estimator.setPrivateProperty("running", true)

        assertTrue(estimator.running)

        // set new value
        estimator.location = null
    }

    @Test(expected = IllegalStateException::class)
    fun useAccurateLevelingEstimator_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(context)

        assertFalse(estimator.useAccurateLevelingEstimator)
        assertFalse(estimator.running)

        // set as running
        estimator.setPrivateProperty("running", true)

        assertTrue(estimator.running)

        estimator.useAccurateLevelingEstimator = true
    }

    @Test
    fun useAccurateLevelingEstimator_whenNotRunningNoLocationAndSetToFalse_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(context)

        assertFalse(estimator.useAccurateLevelingEstimator)
        assertNull(estimator.location)
        assertFalse(estimator.running)

        estimator.useAccurateLevelingEstimator = false

        // check
        assertFalse(estimator.useAccurateLevelingEstimator)
    }

    @Test(expected = IllegalStateException::class)
    fun useAccurateLevelingEstimator_whenNotRunningNoLocationAndSetToTrue_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(context)

        assertFalse(estimator.useAccurateLevelingEstimator)
        assertNull(estimator.location)
        assertFalse(estimator.running)

        estimator.useAccurateLevelingEstimator = true
    }

    @Test
    fun useAccurateLevelingEstimator_whenNotRunningAndLocationAndSetToTrue_setsExpectedValueAndUpdatesProcessors() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = LeveledRelativeAttitudeEstimator2(context, location)

        val accelerometerProcessor: AccelerometerLeveledRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: LeveledRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        assertFalse(estimator.useAccurateLevelingEstimator)
        assertFalse(estimator.running)
        assertSame(location, estimator.location)

        // set to true
        estimator.useAccurateLevelingEstimator = true

        // check
        assertTrue(estimator.useAccurateLevelingEstimator)
        assertTrue(accelerometerProcessor.useAccurateLevelingProcessor)
        assertTrue(gravityProcessor.useAccurateLevelingProcessor)

        // set to false
        estimator.useAccurateLevelingEstimator = false

        // check
        assertFalse(estimator.useAccurateLevelingEstimator)
        assertFalse(accelerometerProcessor.useAccurateLevelingProcessor)
        assertFalse(gravityProcessor.useAccurateLevelingProcessor)
    }

    @Test(expected = IllegalStateException::class)
    fun useAccurateRelativeGyroscopeAttitudeEstimator_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(context)

        assertFalse(estimator.running)

        // set as running
        estimator.setPrivateProperty("running", true)

        assertTrue(estimator.running)

        estimator.useAccurateRelativeGyroscopeAttitudeEstimator = true
    }

    @Test
    fun useAccurateRelativeGyroscopeAttitudeEstimator_whenNotRunning_setsExpectedValueAndUpdatesProcessors() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(context)

        val accelerometerProcessor: AccelerometerLeveledRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: LeveledRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        assertFalse(estimator.running)
        assertTrue(estimator.useAccurateRelativeGyroscopeAttitudeEstimator)
        assertTrue(accelerometerProcessor.useAccurateRelativeGyroscopeAttitudeProcessor)
        assertTrue(gravityProcessor.useAccurateRelativeGyroscopeAttitudeProcessor)

        // set new value
        estimator.useAccurateRelativeGyroscopeAttitudeEstimator = false

        // check
        assertFalse(estimator.useAccurateRelativeGyroscopeAttitudeEstimator)
        assertFalse(accelerometerProcessor.useAccurateRelativeGyroscopeAttitudeProcessor)
        assertFalse(gravityProcessor.useAccurateRelativeGyroscopeAttitudeProcessor)
    }

    @Test
    fun useIndirectInterpolation_setsExpectedValueAndUpdatesProcessors() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(context)

        val accelerometerProcessor: AccelerometerLeveledRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: LeveledRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        // check default value
        assertTrue(estimator.useIndirectInterpolation)
        assertTrue(accelerometerProcessor.useIndirectInterpolation)
        assertTrue(gravityProcessor.useIndirectInterpolation)

        // set new value
        estimator.useIndirectInterpolation = false

        // check
        assertFalse(estimator.useIndirectInterpolation)
        assertFalse(accelerometerProcessor.useIndirectInterpolation)
        assertFalse(gravityProcessor.useIndirectInterpolation)
    }

    @Test
    fun interpolationValue_whenOutOfRange_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(context)

        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_INTERPOLATION_VALUE,
            estimator.interpolationValue,
            0.0
        )

        assertThrows(IllegalArgumentException::class.java) {
            estimator.interpolationValue = -1.0
        }
        assertThrows(IllegalArgumentException::class.java) {
            estimator.interpolationValue = 2.0
        }
    }

    @Test
    fun interpolationValue_whenValid_setsExpectedValueAndUpdatesProcessors() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(context)

        val accelerometerProcessor: AccelerometerLeveledRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: LeveledRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_INTERPOLATION_VALUE,
            estimator.interpolationValue,
            0.0
        )
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_INTERPOLATION_VALUE,
            accelerometerProcessor.interpolationValue,
            0.0
        )
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_INTERPOLATION_VALUE,
            gravityProcessor.interpolationValue,
            0.0
        )

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        estimator.interpolationValue = value

        // check
        assertEquals(value, estimator.interpolationValue, 0.0)
        assertEquals(value, accelerometerProcessor.interpolationValue, 0.0)
        assertEquals(value, gravityProcessor.interpolationValue, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun indirectInterpolationWeight_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(context)

        estimator.indirectInterpolationWeight = 0.0
    }

    @Test
    fun indirectInterpolationWeight_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(context)

        val accelerometerProcessor: AccelerometerLeveledRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: LeveledRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        // check default value
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            estimator.indirectInterpolationWeight,
            0.0
        )
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            accelerometerProcessor.indirectInterpolationWeight,
            0.0
        )
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            gravityProcessor.indirectInterpolationWeight,
            0.0
        )

        // sets new value
        val randomizer = UniformRandomizer()
        val indirectInterpolationWeight = randomizer.nextDouble()
        estimator.indirectInterpolationWeight = indirectInterpolationWeight

        // check
        assertEquals(indirectInterpolationWeight, estimator.indirectInterpolationWeight, 0.0)
        assertEquals(
            indirectInterpolationWeight,
            accelerometerProcessor.indirectInterpolationWeight,
            0.0
        )
        assertEquals(indirectInterpolationWeight, gravityProcessor.indirectInterpolationWeight, 0.0)
    }

    @Test
    fun gyroscopeAverageTimeInterval_whenUseAccelerometer_returnsProcessorValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(context, useAccelerometer = true)

        val accelerometerProcessor: AccelerometerLeveledRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)
        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        every { accelerometerProcessorSpy.timeIntervalSeconds }.returns(TIME_INTERVAL)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)

        assertEquals(TIME_INTERVAL, estimator.gyroscopeTimeIntervalSeconds, 0.0)
        verify(exactly = 1) { accelerometerProcessorSpy.timeIntervalSeconds }
    }

    @Test
    fun gyroscopeAverageTimeInterval_whenAccelerometerNotUsed_returnsProcessorValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(context, useAccelerometer = false)

        val gravityProcessor: LeveledRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        every { gravityProcessorSpy.timeIntervalSeconds }.returns(TIME_INTERVAL)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        assertEquals(TIME_INTERVAL, estimator.gyroscopeTimeIntervalSeconds, 0.0)
        verify(exactly = 1) { gravityProcessorSpy.timeIntervalSeconds }
    }

    @Test(expected = IllegalStateException::class)
    fun outlierThreshold_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(context)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        val randomizer = UniformRandomizer()
        val outlierThreshold = randomizer.nextDouble()
        estimator.outlierThreshold = outlierThreshold
    }

    @Test
    fun outlierThreshold_whenOutOfRange_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(context)

        // check default value
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_OUTLIER_THRESHOLD,
            estimator.outlierThreshold,
            0.0
        )
        assertFalse(estimator.running)

        // set invalid values
        assertThrows(IllegalArgumentException::class.java) { estimator.outlierThreshold = -1.0 }
        assertThrows(IllegalArgumentException::class.java) { estimator.outlierThreshold = 2.0 }
    }

    @Test
    fun outlierThreshold_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(context)

        val accelerometerProcessor: AccelerometerLeveledRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: LeveledRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        // check default value
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_OUTLIER_THRESHOLD,
            estimator.outlierThreshold,
            0.0
        )
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_OUTLIER_THRESHOLD,
            accelerometerProcessor.outlierThreshold,
            0.0
        )
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_OUTLIER_THRESHOLD,
            gravityProcessor.outlierThreshold,
            0.0
        )
        assertFalse(estimator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val outlierThreshold = randomizer.nextDouble()

        estimator.outlierThreshold = outlierThreshold

        // check
        assertEquals(outlierThreshold, estimator.outlierThreshold, 0.0)
        assertEquals(outlierThreshold, accelerometerProcessor.outlierThreshold, 0.0)
        assertEquals(outlierThreshold, gravityProcessor.outlierThreshold, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun outlierPanicThreshold_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(context)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        val randomizer = UniformRandomizer()
        val outlierPanicThreshold = randomizer.nextDouble()
        estimator.outlierPanicThreshold = outlierPanicThreshold
    }

    @Test
    fun outlierPanicThreshold_whenOutOfRange_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(context)

        // check default value
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            estimator.outlierPanicThreshold,
            0.0
        )
        assertFalse(estimator.running)

        // set invalid values
        assertThrows(IllegalArgumentException::class.java) {
            estimator.outlierPanicThreshold = -1.0
        }
        assertThrows(IllegalArgumentException::class.java) { estimator.outlierPanicThreshold = 2.0 }
    }

    @Test
    fun outlierPanicThreshold_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(context)

        val accelerometerProcessor: AccelerometerLeveledRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: LeveledRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        // check default value
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            estimator.outlierPanicThreshold,
            0.0
        )
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            accelerometerProcessor.outlierPanicThreshold,
            0.0
        )
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            gravityProcessor.outlierPanicThreshold,
            0.0
        )
        assertFalse(estimator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val outlierPanicThreshold = randomizer.nextDouble()
        estimator.outlierPanicThreshold = outlierPanicThreshold

        // check
        assertEquals(outlierPanicThreshold, estimator.outlierPanicThreshold, 0.0)
        assertEquals(outlierPanicThreshold, accelerometerProcessor.outlierPanicThreshold, 0.0)
        assertEquals(outlierPanicThreshold, gravityProcessor.outlierPanicThreshold, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun panicCounterThreshold_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(context)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        estimator.panicCounterThreshold = 1
    }

    @Test(expected = IllegalArgumentException::class)
    fun panicCounterThreshold_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(context)

        assertFalse(estimator.running)

        estimator.panicCounterThreshold = 0
    }

    @Test
    fun panicCounterThreshold_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(context)

        val accelerometerProcessor: AccelerometerLeveledRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: LeveledRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        // check default value
        assertFalse(estimator.running)
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_PANIC_COUNTER_THRESHOLD,
            estimator.panicCounterThreshold
        )
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_PANIC_COUNTER_THRESHOLD,
            accelerometerProcessor.panicCounterThreshold
        )
        assertEquals(
            BaseLeveledRelativeAttitudeProcessor.DEFAULT_PANIC_COUNTER_THRESHOLD,
            gravityProcessor.panicCounterThreshold
        )

        estimator.panicCounterThreshold = 2

        // check
        assertEquals(2, estimator.panicCounterThreshold)
        assertEquals(2, accelerometerProcessor.panicCounterThreshold)
        assertEquals(2, gravityProcessor.panicCounterThreshold)
    }

    @Test(expected = IllegalStateException::class)
    fun start_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(context)

        assertFalse(estimator.running)

        // set as running
        estimator.setPrivateProperty("running", true)

        assertTrue(estimator.running)

        estimator.start()
    }

    @Test
    fun start_whenNotRunningAndUseAccelerometer_resetsAndStartsSyncer() {
        val timestamp = System.nanoTime()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(context, useAccelerometer = true)

        // setup spies
        val accelerometerProcessor: AccelerometerLeveledRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)
        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)

        val gravityProcessor: LeveledRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val accelerometerAndGyroscopeSyncer: AccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerAndGyroscopeSyncer")
        requireNotNull(accelerometerAndGyroscopeSyncer)
        val accelerometerAndGyroscopeSyncerSpy = spyk(accelerometerAndGyroscopeSyncer)
        every { accelerometerAndGyroscopeSyncerSpy.start(timestamp) }.returns(true)
        estimator.setPrivateProperty(
            "accelerometerAndGyroscopeSyncer",
            accelerometerAndGyroscopeSyncerSpy
        )

        val gravityAndGyroscopeSyncer: GravityAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("gravityAndGyroscopeSyncer")
        requireNotNull(gravityAndGyroscopeSyncer)
        val gravityAndGyroscopeSyncerSpy = spyk(gravityAndGyroscopeSyncer)
        estimator.setPrivateProperty("gravityAndGyroscopeSyncer", gravityAndGyroscopeSyncerSpy)

        assertFalse(estimator.running)

        assertTrue(estimator.start(timestamp))

        verify(exactly = 1) { accelerometerProcessorSpy.reset() }
        verify(exactly = 1) { accelerometerAndGyroscopeSyncerSpy.start(timestamp) }
        verify { gravityProcessorSpy wasNot Called }
        verify { gravityAndGyroscopeSyncerSpy wasNot Called }
    }

    @Test
    fun start_whenNotRunningAndAccelerometerNotUsed_resetsAndStartsSyncer() {
        val timestamp = System.nanoTime()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(context, useAccelerometer = false)

        // setup spies
        val accelerometerProcessor: AccelerometerLeveledRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)
        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)

        val gravityProcessor: LeveledRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val accelerometerAndGyroscopeSyncer: AccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerAndGyroscopeSyncer")
        requireNotNull(accelerometerAndGyroscopeSyncer)
        val accelerometerAndGyroscopeSyncerSpy = spyk(accelerometerAndGyroscopeSyncer)
        estimator.setPrivateProperty(
            "accelerometerAndGyroscopeSyncer",
            accelerometerAndGyroscopeSyncerSpy
        )

        val gravityAndGyroscopeSyncer: GravityAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("gravityAndGyroscopeSyncer")
        requireNotNull(gravityAndGyroscopeSyncer)
        val gravityAndGyroscopeSyncerSpy = spyk(gravityAndGyroscopeSyncer)
        every { gravityAndGyroscopeSyncerSpy.start(timestamp) }.returns(true)
        estimator.setPrivateProperty("gravityAndGyroscopeSyncer", gravityAndGyroscopeSyncerSpy)

        assertFalse(estimator.running)

        assertTrue(estimator.start(timestamp))

        verify(exactly = 1) { gravityProcessorSpy.reset() }
        verify(exactly = 1) { gravityAndGyroscopeSyncerSpy.start(timestamp) }
        verify { accelerometerProcessorSpy wasNot Called }
        verify { accelerometerAndGyroscopeSyncerSpy wasNot Called }
    }

    @Test
    fun stop_stopsInternalSyncers() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(context)

        // setup spies
        val accelerometerAndGyroscopeSyncer: AccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerAndGyroscopeSyncer")
        requireNotNull(accelerometerAndGyroscopeSyncer)
        val accelerometerAndGyroscopeSyncerSpy = spyk(accelerometerAndGyroscopeSyncer)
        estimator.setPrivateProperty(
            "accelerometerAndGyroscopeSyncer",
            accelerometerAndGyroscopeSyncerSpy
        )

        val gravityAndGyroscopeSyncer: GravityAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("gravityAndGyroscopeSyncer")
        requireNotNull(gravityAndGyroscopeSyncer)
        val gravityAndGyroscopeSyncerSpy = spyk(gravityAndGyroscopeSyncer)
        estimator.setPrivateProperty("gravityAndGyroscopeSyncer", gravityAndGyroscopeSyncerSpy)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        // stop
        estimator.stop()

        // check
        assertFalse(estimator.running)
        verify(exactly = 1) { accelerometerAndGyroscopeSyncerSpy.stop() }
        verify(exactly = 1) { gravityAndGyroscopeSyncerSpy.stop() }
    }

    @Test
    fun gravitySyncer_whenAccuracyChangedAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(context)

        val gravityAndGyroscopeSyncer: GravityAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("gravityAndGyroscopeSyncer")
        requireNotNull(gravityAndGyroscopeSyncer)

        val listener = gravityAndGyroscopeSyncer.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(
            gravityAndGyroscopeSyncer,
            SensorType.GRAVITY,
            SensorAccuracy.MEDIUM
        )
    }

    @Test
    fun gravitySyncer_whenAccuracyChangedAndListener_notifies() {
        val accuracyChangedListener =
            mockk<LeveledRelativeAttitudeEstimator2.OnAccuracyChangedListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val gravityAndGyroscopeSyncer: GravityAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("gravityAndGyroscopeSyncer")
        requireNotNull(gravityAndGyroscopeSyncer)

        val listener = gravityAndGyroscopeSyncer.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(
            gravityAndGyroscopeSyncer,
            SensorType.GRAVITY,
            SensorAccuracy.MEDIUM
        )

        // check
        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                estimator,
                SensorType.GRAVITY,
                SensorAccuracy.MEDIUM
            )
        }
    }

    @Test
    fun gravitySyncer_whenBufferFilledAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(context)

        val gravityAndGyroscopeSyncer: GravityAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("gravityAndGyroscopeSyncer")
        requireNotNull(gravityAndGyroscopeSyncer)

        val listener = gravityAndGyroscopeSyncer.bufferFilledListener
        requireNotNull(listener)
        listener.onBufferFilled(
            gravityAndGyroscopeSyncer,
            SensorType.GRAVITY
        )
    }

    @Test
    fun gravitySyncer_whenBufferFilledAndListener_notifies() {
        val bufferFilledListener =
            mockk<LeveledRelativeAttitudeEstimator2.OnBufferFilledListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(
            context,
            bufferFilledListener = bufferFilledListener
        )

        val gravityAndGyroscopeSyncer: GravityAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("gravityAndGyroscopeSyncer")
        requireNotNull(gravityAndGyroscopeSyncer)

        val listener = gravityAndGyroscopeSyncer.bufferFilledListener
        requireNotNull(listener)
        listener.onBufferFilled(
            gravityAndGyroscopeSyncer,
            SensorType.GRAVITY
        )

        // check
        verify(exactly = 1) {
            bufferFilledListener.onBufferFilled(
                estimator,
                SensorType.GRAVITY
            )
        }
    }

    @Test
    fun gravitySyncer_whenSyncedMeasurementAndNotProcessed_makesNoAction() {
        val attitudeAvailableListener =
            mockk<LeveledRelativeAttitudeEstimator2.OnAttitudeAvailableListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(
            context,
            attitudeAvailableListener = attitudeAvailableListener
        )

        val gravityAndGyroscopeSyncer: GravityAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("gravityAndGyroscopeSyncer")
        requireNotNull(gravityAndGyroscopeSyncer)

        val measurement = GravityAndGyroscopeSyncedSensorMeasurement()

        val gravityProcessor: LeveledRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        every { gravityProcessorSpy.process(measurement) }.returns(false)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val listener = gravityAndGyroscopeSyncer.syncedMeasurementListener
        requireNotNull(listener)
        listener.onSyncedMeasurements(gravityAndGyroscopeSyncer, measurement)

        verify { attitudeAvailableListener wasNot Called }
    }

    @Test
    fun gravitySyncer_whenSyncedMeasurementProcessedAndNoListener_updatesFusedAttitude() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(context)

        val gravityAndGyroscopeSyncer: GravityAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("gravityAndGyroscopeSyncer")
        requireNotNull(gravityAndGyroscopeSyncer)

        val measurement = GravityAndGyroscopeSyncedSensorMeasurement()
        val fusedAttitude1 = getAttitude()

        val gravityProcessor: LeveledRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        every { gravityProcessorSpy.process(measurement) }.returns(true)
        every { gravityProcessorSpy.fusedAttitude }.returns(fusedAttitude1)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val listener = gravityAndGyroscopeSyncer.syncedMeasurementListener
        requireNotNull(listener)
        listener.onSyncedMeasurements(gravityAndGyroscopeSyncer, measurement)

        val fusedAttitude2: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude2)
        assertEquals(fusedAttitude1, fusedAttitude2)
    }

    @Test
    fun gravitySyncer_whenSyncedMeasurementProcessedEstimateCoordinateTransformationAndEulerAnglesDisableAndListener_updatesFusedAttitudeAndNotifies() {
        val attitudeAvailableListener =
            mockk<LeveledRelativeAttitudeEstimator2.OnAttitudeAvailableListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(
            context,
            estimateCoordinateTransformation = false,
            estimateEulerAngles = false,
            attitudeAvailableListener = attitudeAvailableListener
        )

        val gravityAndGyroscopeSyncer: GravityAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("gravityAndGyroscopeSyncer")
        requireNotNull(gravityAndGyroscopeSyncer)

        val timestamp = System.nanoTime()
        val measurement = GravityAndGyroscopeSyncedSensorMeasurement(timestamp = timestamp)
        val fusedAttitude1 = getAttitude()

        val gravityProcessor: LeveledRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        every { gravityProcessorSpy.process(measurement) }.returns(true)
        every { gravityProcessorSpy.fusedAttitude }.returns(fusedAttitude1)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val listener = gravityAndGyroscopeSyncer.syncedMeasurementListener
        requireNotNull(listener)
        listener.onSyncedMeasurements(gravityAndGyroscopeSyncer, measurement)

        val fusedAttitude2: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude2)
        assertEquals(fusedAttitude1, fusedAttitude2)

        verify(exactly = 1) {
            attitudeAvailableListener.onAttitudeAvailable(
                estimator,
                fusedAttitude2,
                timestamp,
                null,
                null,
                null,
                null
            )
        }
    }

    @Test
    fun gravitySyncer_whenSyncedMeasurementProcessedEstimateCoordinateTransformationAndEulerAnglesEnableAndListener_updatesFusedAttitudeAndNotifies() {
        val attitudeAvailableListener =
            mockk<LeveledRelativeAttitudeEstimator2.OnAttitudeAvailableListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(
            context,
            estimateCoordinateTransformation = true,
            estimateEulerAngles = true,
            attitudeAvailableListener = attitudeAvailableListener
        )

        val gravityAndGyroscopeSyncer: GravityAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("gravityAndGyroscopeSyncer")
        requireNotNull(gravityAndGyroscopeSyncer)

        val timestamp = System.nanoTime()
        val measurement = GravityAndGyroscopeSyncedSensorMeasurement(timestamp = timestamp)
        val fusedAttitude1 = getAttitude()

        val gravityProcessor: LeveledRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        every { gravityProcessorSpy.process(measurement) }.returns(true)
        every { gravityProcessorSpy.fusedAttitude }.returns(fusedAttitude1)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val listener = gravityAndGyroscopeSyncer.syncedMeasurementListener
        requireNotNull(listener)
        listener.onSyncedMeasurements(gravityAndGyroscopeSyncer, measurement)

        val fusedAttitude2: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude2)
        assertEquals(fusedAttitude1, fusedAttitude2)

        val coordinateTransformation: CoordinateTransformation? =
            estimator.getPrivateProperty("coordinateTransformation")
        requireNotNull(coordinateTransformation)
        assertEquals(
            CoordinateTransformation(
                fusedAttitude1,
                FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME
            ), coordinateTransformation
        )

        val eulerAngles1: DoubleArray? = estimator.getPrivateProperty("eulerAngles")
        requireNotNull(eulerAngles1)
        val eulerAngles2 = fusedAttitude1.toEulerAngles()
        assertArrayEquals(eulerAngles1, eulerAngles2, 0.0)

        verify(exactly = 1) {
            attitudeAvailableListener.onAttitudeAvailable(
                estimator,
                fusedAttitude2,
                timestamp,
                eulerAngles2[0],
                eulerAngles2[1],
                eulerAngles2[2],
                coordinateTransformation
            )
        }
    }

    @Test
    fun accelerometerSyncer_whenAccuracyChangedAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(context)

        val accelerometerAndGyroscopeSyncer: AccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerAndGyroscopeSyncer")
        requireNotNull(accelerometerAndGyroscopeSyncer)

        val listener = accelerometerAndGyroscopeSyncer.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(
            accelerometerAndGyroscopeSyncer,
            SensorType.GRAVITY,
            SensorAccuracy.MEDIUM
        )
    }

    @Test
    fun accelerometerSyncer_whenAccuracyChangedAndListener_notifies() {
        val accuracyChangedListener =
            mockk<LeveledRelativeAttitudeEstimator2.OnAccuracyChangedListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val accelerometerAndGyroscopeSyncer: AccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerAndGyroscopeSyncer")
        requireNotNull(accelerometerAndGyroscopeSyncer)

        val listener = accelerometerAndGyroscopeSyncer.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(
            accelerometerAndGyroscopeSyncer,
            SensorType.GRAVITY,
            SensorAccuracy.MEDIUM
        )

        // check
        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                estimator,
                SensorType.GRAVITY,
                SensorAccuracy.MEDIUM
            )
        }
    }

    @Test
    fun accelerometerSyncer_whenBufferFilledAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(context)

        val accelerometerAndGyroscopeSyncer: AccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerAndGyroscopeSyncer")
        requireNotNull(accelerometerAndGyroscopeSyncer)

        val listener = accelerometerAndGyroscopeSyncer.bufferFilledListener
        requireNotNull(listener)
        listener.onBufferFilled(
            accelerometerAndGyroscopeSyncer,
            SensorType.GRAVITY
        )
    }

    @Test
    fun accelerometerSyncer_whenBufferFilledAndListener_notifies() {
        val bufferFilledListener =
            mockk<LeveledRelativeAttitudeEstimator2.OnBufferFilledListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(
            context,
            bufferFilledListener = bufferFilledListener
        )

        val accelerometerAndGyroscopeSyncer: AccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerAndGyroscopeSyncer")
        requireNotNull(accelerometerAndGyroscopeSyncer)

        val listener = accelerometerAndGyroscopeSyncer.bufferFilledListener
        requireNotNull(listener)
        listener.onBufferFilled(
            accelerometerAndGyroscopeSyncer,
            SensorType.GRAVITY
        )

        // check
        verify(exactly = 1) {
            bufferFilledListener.onBufferFilled(
                estimator,
                SensorType.GRAVITY
            )
        }
    }

    @Test
    fun accelerometerSyncer_whenSyncedMeasurementAndNotProcessed_makesNoAction() {
        val attitudeAvailableListener =
            mockk<LeveledRelativeAttitudeEstimator2.OnAttitudeAvailableListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(
            context,
            attitudeAvailableListener = attitudeAvailableListener
        )

        val accelerometerAndGyroscopeSyncer: AccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerAndGyroscopeSyncer")
        requireNotNull(accelerometerAndGyroscopeSyncer)

        val measurement = AccelerometerAndGyroscopeSyncedSensorMeasurement()

        val accelerometerProcessor: AccelerometerLeveledRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)
        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        every { accelerometerProcessorSpy.process(measurement) }.returns(false)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)

        val listener = accelerometerAndGyroscopeSyncer.syncedMeasurementListener
        requireNotNull(listener)
        listener.onSyncedMeasurements(accelerometerAndGyroscopeSyncer, measurement)

        verify { attitudeAvailableListener wasNot Called }
    }

    @Test
    fun accelerometerSyncer_whenSyncedMeasurementProcessedAndNoListener_updatesFusedAttitude() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(context)

        val accelerometerAndGyroscopeSyncer: AccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerAndGyroscopeSyncer")
        requireNotNull(accelerometerAndGyroscopeSyncer)

        val measurement = AccelerometerAndGyroscopeSyncedSensorMeasurement()
        val fusedAttitude1 = getAttitude()

        val accelerometerProcessor: AccelerometerLeveledRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)
        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        every { accelerometerProcessorSpy.process(measurement) }.returns(true)
        every { accelerometerProcessorSpy.fusedAttitude }.returns(fusedAttitude1)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)

        val listener = accelerometerAndGyroscopeSyncer.syncedMeasurementListener
        requireNotNull(listener)
        listener.onSyncedMeasurements(accelerometerAndGyroscopeSyncer, measurement)

        val fusedAttitude2: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude2)
        assertEquals(fusedAttitude1, fusedAttitude2)
    }

    @Test
    fun accelerometerSyncer_whenSyncedMeasurementProcessedEstimateCoordinateTransformationAndEulerAnglesDisableAndListener_updatesFusedAttitudeAndNotifies() {
        val attitudeAvailableListener =
            mockk<LeveledRelativeAttitudeEstimator2.OnAttitudeAvailableListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(
            context,
            estimateCoordinateTransformation = false,
            estimateEulerAngles = false,
            attitudeAvailableListener = attitudeAvailableListener
        )

        val accelerometerAndGyroscopeSyncer: AccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerAndGyroscopeSyncer")
        requireNotNull(accelerometerAndGyroscopeSyncer)

        val timestamp = System.nanoTime()
        val measurement = AccelerometerAndGyroscopeSyncedSensorMeasurement(timestamp = timestamp)
        val fusedAttitude1 = getAttitude()

        val accelerometerProcessor: AccelerometerLeveledRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)
        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        every { accelerometerProcessorSpy.process(measurement) }.returns(true)
        every { accelerometerProcessorSpy.fusedAttitude }.returns(fusedAttitude1)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)

        val listener = accelerometerAndGyroscopeSyncer.syncedMeasurementListener
        requireNotNull(listener)
        listener.onSyncedMeasurements(accelerometerAndGyroscopeSyncer, measurement)

        val fusedAttitude2: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude2)
        assertEquals(fusedAttitude1, fusedAttitude2)

        verify(exactly = 1) {
            attitudeAvailableListener.onAttitudeAvailable(
                estimator,
                fusedAttitude2,
                timestamp,
                null,
                null,
                null,
                null
            )
        }
    }

    @Test
    fun accelerometerSyncer_whenSyncedMeasurementProcessedEstimateCoordinateTransformationAndEulerAnglesEnableAndListener_updatesFusedAttitudeAndNotifies() {
        val attitudeAvailableListener =
            mockk<LeveledRelativeAttitudeEstimator2.OnAttitudeAvailableListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator2(
            context,
            estimateCoordinateTransformation = true,
            estimateEulerAngles = true,
            attitudeAvailableListener = attitudeAvailableListener
        )

        val accelerometerAndGyroscopeSyncer: AccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerAndGyroscopeSyncer")
        requireNotNull(accelerometerAndGyroscopeSyncer)

        val timestamp = System.nanoTime()
        val measurement = AccelerometerAndGyroscopeSyncedSensorMeasurement(timestamp = timestamp)
        val fusedAttitude1 = getAttitude()

        val accelerometerProcessor: AccelerometerLeveledRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)
        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        every { accelerometerProcessorSpy.process(measurement) }.returns(true)
        every { accelerometerProcessorSpy.fusedAttitude }.returns(fusedAttitude1)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)

        val listener = accelerometerAndGyroscopeSyncer.syncedMeasurementListener
        requireNotNull(listener)
        listener.onSyncedMeasurements(accelerometerAndGyroscopeSyncer, measurement)

        val fusedAttitude2: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude2)
        assertEquals(fusedAttitude1, fusedAttitude2)

        val coordinateTransformation: CoordinateTransformation? =
            estimator.getPrivateProperty("coordinateTransformation")
        requireNotNull(coordinateTransformation)
        assertEquals(
            CoordinateTransformation(
                fusedAttitude1,
                FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME
            ), coordinateTransformation
        )

        val eulerAngles1: DoubleArray? = estimator.getPrivateProperty("eulerAngles")
        requireNotNull(eulerAngles1)
        val eulerAngles2 = fusedAttitude1.toEulerAngles()
        assertArrayEquals(eulerAngles1, eulerAngles2, 0.0)

        verify(exactly = 1) {
            attitudeAvailableListener.onAttitudeAvailable(
                estimator,
                fusedAttitude2,
                timestamp,
                eulerAngles2[0],
                eulerAngles2[1],
                eulerAngles2[2],
                coordinateTransformation
            )
        }
    }

    private companion object {
        const val MIN_LATITUDE_DEGREES = -90.0
        const val MAX_LATITUDE_DEGREES = 90.0

        const val MIN_LONGITUDE_DEGREES = -180.0
        const val MAX_LONGITUDE_DEGREES = 180.0

        const val MIN_HEIGHT = -100.0
        const val MAX_HEIGHT = 4000.0

        const val MIN_ANGLE_DEGREES = -45.0
        const val MAX_ANGLE_DEGREES = 45.0

        const val TIME_INTERVAL = 0.02

        fun getLocation(): Location {
            val randomizer = UniformRandomizer()
            val latitudeDegrees = randomizer.nextDouble(
                MIN_LATITUDE_DEGREES,
                MAX_LATITUDE_DEGREES
            )
            val longitudeDegrees = randomizer.nextDouble(
                MIN_LONGITUDE_DEGREES,
                MAX_LONGITUDE_DEGREES
            )
            val height = randomizer.nextDouble(
                MIN_HEIGHT,
                MAX_HEIGHT
            )

            val location = mockk<Location>()
            every { location.latitude }.returns(latitudeDegrees)
            every { location.longitude }.returns(longitudeDegrees)
            every { location.altitude }.returns(height)

            return location
        }

        fun getAttitude(): Quaternion {
            val randomizer = UniformRandomizer()
            val roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))

            return Quaternion(roll, pitch, yaw)
        }
    }
}