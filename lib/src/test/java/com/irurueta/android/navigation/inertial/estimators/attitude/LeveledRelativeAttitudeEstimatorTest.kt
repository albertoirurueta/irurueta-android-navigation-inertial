/*
 * Copyright (C) 2026 Alberto Irurueta Carro (alberto@irurueta.com)
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
import android.hardware.Sensor
import android.hardware.SensorManager
import android.location.Location
import android.os.SystemClock
import com.irurueta.android.navigation.inertial.collectors.AccelerometerAndGyroscopeSyncedSensorCollector
import com.irurueta.android.navigation.inertial.collectors.GravityAndGyroscopeSyncedSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerAndGyroscopeSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.GravityAndGyroscopeSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorType
import com.irurueta.android.navigation.inertial.processors.attitude.AccelerometerLeveledRelativeAttitudeProcessor
import com.irurueta.android.navigation.inertial.processors.attitude.BaseLeveledRelativeAttitudeProcessor
import com.irurueta.android.navigation.inertial.processors.attitude.LeveledRelativeAttitudeProcessor
import com.irurueta.android.navigation.inertial.processors.filters.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.processors.filters.MeanAveragingFilter
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.android.testutils.setPrivateProperty
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.statistics.UniformRandomizer
import com.irurueta.units.Acceleration
import com.irurueta.units.AccelerationUnit
import io.mockk.Called
import io.mockk.every
import io.mockk.impl.annotations.MockK
import io.mockk.junit4.MockKRule
import io.mockk.justRun
import io.mockk.mockkStatic
import io.mockk.spyk
import io.mockk.verify
import org.junit.Assert.*
import org.junit.Rule
import org.junit.Test

class LeveledRelativeAttitudeEstimatorTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK(relaxUnitFun = true)
    private lateinit var attitudeAvailableListener:
            LeveledRelativeAttitudeEstimator.OnAttitudeAvailableListener

    @MockK(relaxUnitFun = true)
    private lateinit var accuracyChangedListener:
            LeveledRelativeAttitudeEstimator.OnAccuracyChangedListener

    @MockK
    private lateinit var location: Location

    @MockK
    private lateinit var context: Context

    @MockK
    private lateinit var sensorManager: SensorManager

    @MockK
    private lateinit var accelerometerSensor: Sensor

    @MockK
    private lateinit var gravitySensor: Sensor

    @MockK
    private lateinit var gyroscopeSensor: Sensor

    @Test
    fun constructor_whenRequiredProperties_setsDefaultValues() {
        val estimator = LeveledRelativeAttitudeEstimator(context)

        // check
        assertSame(context, estimator.context)
        assertNull(estimator.location)
        assertTrue(estimator.adjustGravityNorm)
        assertEquals(SensorDelay.GAME, estimator.sensorDelay)
        assertTrue(estimator.useAccelerometer)
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
        val gravityAndGyroscopeCollector: GravityAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("gravityAndGyroscopeCollector")
        requireNotNull(gravityAndGyroscopeCollector)

        assertSame(context, gravityAndGyroscopeCollector.context)
        assertEquals(estimator.gyroscopeSensorType, gravityAndGyroscopeCollector.gyroscopeSensorType)
        assertEquals(estimator.sensorDelay, gravityAndGyroscopeCollector.gravitySensorDelay)
        assertEquals(estimator.sensorDelay, gravityAndGyroscopeCollector.gyroscopeSensorDelay)
        assertNotNull(gravityAndGyroscopeCollector.accuracyChangedListener)
        assertNotNull(gravityAndGyroscopeCollector.measurementListener)

        val accelerometerAndGyroscopeCollector: AccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerAndGyroscopeCollector")
        requireNotNull(accelerometerAndGyroscopeCollector)

        assertSame(context, accelerometerAndGyroscopeCollector.context)
        assertEquals(
            estimator.gyroscopeSensorType,
            accelerometerAndGyroscopeCollector.gyroscopeSensorType
        )
        assertEquals(
            estimator.sensorDelay,
            accelerometerAndGyroscopeCollector.accelerometerSensorDelay
        )
        assertEquals(estimator.sensorDelay, accelerometerAndGyroscopeCollector.gyroscopeSensorDelay)
        assertNotNull(accelerometerAndGyroscopeCollector.accuracyChangedListener)
        assertNotNull(accelerometerAndGyroscopeCollector.measurementListener)
    }

    @Test
    fun constructor_whenAllProperties_setsExpectedValues() {
        val location = getLocation()
        val accelerometerAveragingFilter = MeanAveragingFilter<AccelerationUnit, Acceleration, AccelerationTriad>()
        val estimator = LeveledRelativeAttitudeEstimator(
            context,
            location,
            SensorDelay.NORMAL,
            useAccelerometer = false,
            AccelerometerSensorType.ACCELEROMETER,
            accelerometerAveragingFilter,
            GyroscopeSensorType.GYROSCOPE,
            useAccurateLevelingEstimator = true,
            useAccurateRelativeGyroscopeAttitudeEstimator = false,
            estimateCoordinateTransformation = true,
            estimateEulerAngles = false,
            attitudeAvailableListener,
            accuracyChangedListener,
            adjustGravityNorm = false
        )

        // check
        assertSame(context, estimator.context)
        assertSame(location, estimator.location)
        assertFalse(estimator.adjustGravityNorm)
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertFalse(estimator.useAccelerometer)
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
        val gravityAndGyroscopeCollector: GravityAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("gravityAndGyroscopeCollector")
        requireNotNull(gravityAndGyroscopeCollector)

        assertSame(context, gravityAndGyroscopeCollector.context)
        assertEquals(estimator.gyroscopeSensorType, gravityAndGyroscopeCollector.gyroscopeSensorType)
        assertEquals(estimator.sensorDelay, gravityAndGyroscopeCollector.gravitySensorDelay)
        assertEquals(estimator.sensorDelay, gravityAndGyroscopeCollector.gyroscopeSensorDelay)
        assertNotNull(gravityAndGyroscopeCollector.accuracyChangedListener)
        assertNotNull(gravityAndGyroscopeCollector.measurementListener)

        val accelerometerAndGyroscopeCollector: AccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerAndGyroscopeCollector")
        requireNotNull(accelerometerAndGyroscopeCollector)

        assertSame(context, accelerometerAndGyroscopeCollector.context)
        assertEquals(
            estimator.gyroscopeSensorType,
            accelerometerAndGyroscopeCollector.gyroscopeSensorType
        )
        assertEquals(
            estimator.sensorDelay,
            accelerometerAndGyroscopeCollector.accelerometerSensorDelay
        )
        assertEquals(estimator.sensorDelay, accelerometerAndGyroscopeCollector.gyroscopeSensorDelay)
        assertNotNull(accelerometerAndGyroscopeCollector.accuracyChangedListener)
        assertNotNull(accelerometerAndGyroscopeCollector.measurementListener)
    }

    @Test
    fun location_whenNotRunning_setsExpectedValue() {
        val estimator = LeveledRelativeAttitudeEstimator(context)

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
        val estimator = LeveledRelativeAttitudeEstimator(context)

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

    @Test
    fun location_whenRunningAndNull_throwsIllegalStateException() {
        val location = getLocation()
        val estimator = LeveledRelativeAttitudeEstimator(context, location)

        assertFalse(estimator.running)
        assertSame(location, estimator.location)

        // set as running
        estimator.setPrivateProperty("running", true)

        assertTrue(estimator.running)

        // set new value
        assertThrows(IllegalStateException::class.java) {
            estimator.location = null
        }
    }

    @Test
    fun adjustGravityNorm_setsExpectedValue() {
        val estimator = LeveledRelativeAttitudeEstimator(context)

        // check default value
        assertTrue(estimator.adjustGravityNorm)

        val gravityProcessor: LeveledRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        assertTrue(gravityProcessor.adjustGravityNorm)

        val accelerometerProcessor: AccelerometerLeveledRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)
        assertTrue(accelerometerProcessor.adjustGravityNorm)

        // set new value
        estimator.adjustGravityNorm = false

        // check
        assertFalse(estimator.adjustGravityNorm)
        assertFalse(gravityProcessor.adjustGravityNorm)
        assertFalse(accelerometerProcessor.adjustGravityNorm)
    }

    @Test
    fun useAccurateLevelingEstimator_whenRunning_throwsIllegalStateException() {
        val estimator = LeveledRelativeAttitudeEstimator(context)

        assertFalse(estimator.useAccurateLevelingEstimator)
        assertFalse(estimator.running)

        // set as running
        estimator.setPrivateProperty("running", true)

        assertTrue(estimator.running)

        assertThrows(IllegalStateException::class.java) {
            estimator.useAccurateLevelingEstimator = true
        }
    }

    @Test
    fun useAccurateLevelingEstimator_whenNotRunningNoLocationAndSetToFalse_setsExpectedValue() {
        val estimator = LeveledRelativeAttitudeEstimator(context)

        assertFalse(estimator.useAccurateLevelingEstimator)
        assertNull(estimator.location)
        assertFalse(estimator.running)

        estimator.useAccurateLevelingEstimator = false

        // check
        assertFalse(estimator.useAccurateLevelingEstimator)
    }

    @Test
    fun useAccurateLevelingEstimator_whenNotRunningNoLocationAndSetToTrue_throwsIllegalStateException() {
        val estimator = LeveledRelativeAttitudeEstimator(context)

        assertFalse(estimator.useAccurateLevelingEstimator)
        assertNull(estimator.location)
        assertFalse(estimator.running)

        assertThrows(IllegalStateException::class.java) {
            estimator.useAccurateLevelingEstimator = true
        }
    }

    @Test
    fun useAccurateLevelingEstimator_whenNotRunningAndLocationAndSetToTrue_setsExpectedValueAndUpdatesProcessors() {
        val location = getLocation()
        val estimator = LeveledRelativeAttitudeEstimator(context, location)

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

    @Test
    fun useAccurateRelativeGyroscopeAttitudeEstimator_whenRunning_throwsIllegalStateException() {
        val estimator = LeveledRelativeAttitudeEstimator(context)

        assertFalse(estimator.running)

        // set as running
        estimator.setPrivateProperty("running", true)

        assertTrue(estimator.running)

        assertThrows(IllegalStateException::class.java) {
            estimator.useAccurateRelativeGyroscopeAttitudeEstimator = true
        }
    }

    @Test
    fun useAccurateRelativeGyroscopeAttitudeEstimator_whenNotRunning_setsExpectedValueAndUpdatesProcessors() {
        val estimator = LeveledRelativeAttitudeEstimator(context)

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
        val estimator = LeveledRelativeAttitudeEstimator(context)

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
        val estimator = LeveledRelativeAttitudeEstimator(context)

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
        val estimator = LeveledRelativeAttitudeEstimator(context)

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

    @Test
    fun indirectInterpolationWeight_whenInvalid_throwsIllegalArgumentException() {
        val estimator = LeveledRelativeAttitudeEstimator(context)

        assertThrows(IllegalArgumentException::class.java) {
            estimator.indirectInterpolationWeight = 0.0
        }
    }

    @Test
    fun indirectInterpolationWeight_whenValid_setsExpectedValue() {
        val estimator = LeveledRelativeAttitudeEstimator(context)

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
        val estimator = LeveledRelativeAttitudeEstimator(context, useAccelerometer = true)

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
        val estimator = LeveledRelativeAttitudeEstimator(context, useAccelerometer = false)

        val gravityProcessor: LeveledRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        every { gravityProcessorSpy.timeIntervalSeconds }.returns(TIME_INTERVAL)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        assertEquals(TIME_INTERVAL, estimator.gyroscopeTimeIntervalSeconds, 0.0)
        verify(exactly = 1) { gravityProcessorSpy.timeIntervalSeconds }
    }

    @Test
    fun outlierThreshold_whenRunning_throwsIllegalStateException() {
        val estimator = LeveledRelativeAttitudeEstimator(context)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        val randomizer = UniformRandomizer()
        val outlierThreshold = randomizer.nextDouble()
        assertThrows(IllegalStateException::class.java) {
            estimator.outlierThreshold = outlierThreshold
        }
    }

    @Test
    fun outlierThreshold_whenOutOfRange_throwsIllegalArgumentException() {
        val estimator = LeveledRelativeAttitudeEstimator(context)

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
        val estimator = LeveledRelativeAttitudeEstimator(context)

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

    @Test
    fun outlierPanicThreshold_whenRunning_throwsIllegalStateException() {
        val estimator = LeveledRelativeAttitudeEstimator(context)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        val randomizer = UniformRandomizer()
        val outlierPanicThreshold = randomizer.nextDouble()
        assertThrows(IllegalStateException::class.java) {
            estimator.outlierPanicThreshold = outlierPanicThreshold
        }
    }

    @Test
    fun outlierPanicThreshold_whenOutOfRange_throwsIllegalArgumentException() {
        val estimator = LeveledRelativeAttitudeEstimator(context)

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
        val estimator = LeveledRelativeAttitudeEstimator(context)

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

    @Test
    fun panicCounterThreshold_whenRunning_throwsIllegalStateException() {
        val estimator = LeveledRelativeAttitudeEstimator(context)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        assertThrows(IllegalStateException::class.java) {
            estimator.panicCounterThreshold = 1
        }
    }

    @Test
    fun panicCounterThreshold_whenInvalid_throwsIllegalArgumentException() {
        val estimator = LeveledRelativeAttitudeEstimator(context)

        assertFalse(estimator.running)

        assertThrows(IllegalArgumentException::class.java) {
            estimator.panicCounterThreshold = 0
        }
    }

    @Test
    fun panicCounterThreshold_whenValid_setsExpectedValue() {
        val estimator = LeveledRelativeAttitudeEstimator(context)

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

    @Test
    fun start_whenRunning_throwsIllegalStateException() {
        mockkStatic(SystemClock::class) {
            val startTimestamp = System.nanoTime()
            every { SystemClock.elapsedRealtimeNanos() }.returns(startTimestamp)

            val estimator = LeveledRelativeAttitudeEstimator(context)

            assertFalse(estimator.running)

            // set as running
            estimator.setPrivateProperty("running", true)

            assertTrue(estimator.running)

            assertThrows(IllegalStateException::class.java) {
                estimator.start()
            }
        }
    }

    @Test
    fun start_whenNotRunningAndUseAccelerometer_resetsAndStartsCollector() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }
            .returns(accelerometerSensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }
            .returns(gyroscopeSensor)
        every { sensorManager.registerListener(any(), any<Sensor>(), any()) }.returns(true)

        val timestamp = System.nanoTime()
        val estimator = LeveledRelativeAttitudeEstimator(context, useAccelerometer = true)

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

        val accelerometerAndGyroscopeCollector: AccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerAndGyroscopeCollector")
        requireNotNull(accelerometerAndGyroscopeCollector)
        val accelerometerAndGyroscopeCollectorSpy = spyk(accelerometerAndGyroscopeCollector)
        every { accelerometerAndGyroscopeCollectorSpy.start(timestamp) }.returns(true)
        estimator.setPrivateProperty(
            "accelerometerAndGyroscopeCollector",
            accelerometerAndGyroscopeCollectorSpy
        )

        val gravityAndGyroscopeCollector: GravityAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("gravityAndGyroscopeCollector")
        requireNotNull(gravityAndGyroscopeCollector)
        val gravityAndGyroscopeCollectorSpy = spyk(gravityAndGyroscopeCollector)
        estimator.setPrivateProperty("gravityAndGyroscopeCollector", gravityAndGyroscopeCollectorSpy)

        assertFalse(estimator.running)

        assertTrue(estimator.start(timestamp))

        verify(exactly = 1) { accelerometerProcessorSpy.reset() }
        verify(exactly = 1) { accelerometerAndGyroscopeCollectorSpy.start(timestamp) }
        verify { gravityProcessorSpy wasNot Called }
        verify { gravityAndGyroscopeCollectorSpy wasNot Called }
    }

    @Test
    fun start_whenNotRunningAndAccelerometerNotUsed_resetsAndStartsSyncer() {
        val timestamp = System.nanoTime()
        val estimator = LeveledRelativeAttitudeEstimator(context, useAccelerometer = false)

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

        val accelerometerAndGyroscopeCollector: AccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerAndGyroscopeCollector")
        requireNotNull(accelerometerAndGyroscopeCollector)
        val accelerometerAndGyroscopeCollectorSpy = spyk(accelerometerAndGyroscopeCollector)
        estimator.setPrivateProperty(
            "accelerometerAndGyroscopeCollector",
            accelerometerAndGyroscopeCollectorSpy
        )

        val gravityAndGyroscopeCollector: GravityAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("gravityAndGyroscopeCollector")
        requireNotNull(gravityAndGyroscopeCollector)
        val gravityAndGyroscopeCollectorSpy = spyk(gravityAndGyroscopeCollector)
        every { gravityAndGyroscopeCollectorSpy.start(timestamp) }.returns(true)
        estimator.setPrivateProperty("gravityAndGyroscopeCollector", gravityAndGyroscopeCollectorSpy)

        assertFalse(estimator.running)

        assertTrue(estimator.start(timestamp))

        verify(exactly = 1) { gravityProcessorSpy.reset() }
        verify(exactly = 1) { gravityAndGyroscopeCollectorSpy.start(timestamp) }
        verify { accelerometerProcessorSpy wasNot Called }
        verify { accelerometerAndGyroscopeCollectorSpy wasNot Called }
    }

    @Test
    fun stop_stopsInternalSyncers() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }
            .returns(accelerometerSensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }
            .returns(gyroscopeSensor)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }
            .returns(gravitySensor)
        justRun { sensorManager.unregisterListener(any(), any<Sensor>()) }

        val estimator = LeveledRelativeAttitudeEstimator(context)

        // setup spies
        val accelerometerAndGyroscopeCollector: AccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerAndGyroscopeCollector")
        requireNotNull(accelerometerAndGyroscopeCollector)
        val accelerometerAndGyroscopeCollectorSpy = spyk(accelerometerAndGyroscopeCollector)
        estimator.setPrivateProperty(
            "accelerometerAndGyroscopeCollector",
            accelerometerAndGyroscopeCollectorSpy
        )

        val gravityAndGyroscopeCollector: GravityAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("gravityAndGyroscopeCollector")
        requireNotNull(gravityAndGyroscopeCollector)
        val gravityAndGyroscopeCollectorSpy = spyk(gravityAndGyroscopeCollector)
        estimator.setPrivateProperty("gravityAndGyroscopeCollector", gravityAndGyroscopeCollectorSpy)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        // stop
        estimator.stop()

        // check
        assertFalse(estimator.running)
        verify(exactly = 1) { accelerometerAndGyroscopeCollectorSpy.stop() }
        verify(exactly = 1) { gravityAndGyroscopeCollectorSpy.stop() }
    }

    @Test
    fun gravityCollector_whenAccuracyChangedAndNoListener_makesNoAction() {
        val estimator = LeveledRelativeAttitudeEstimator(context)

        val gravityAndGyroscopeCollector: GravityAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("gravityAndGyroscopeCollector")
        requireNotNull(gravityAndGyroscopeCollector)

        val listener = gravityAndGyroscopeCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(
            gravityAndGyroscopeCollector,
            SensorType.GRAVITY,
            SensorAccuracy.MEDIUM
        )
    }

    @Test
    fun gravityCollector_whenAccuracyChangedAndListener_notifies() {
        val estimator = LeveledRelativeAttitudeEstimator(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val gravityAndGyroscopeCollector: GravityAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("gravityAndGyroscopeCollector")
        requireNotNull(gravityAndGyroscopeCollector)

        val listener = gravityAndGyroscopeCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(
            gravityAndGyroscopeCollector,
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
    fun gravityCollector_whenSyncedMeasurementAndNotProcessed_makesNoAction() {
        val estimator = LeveledRelativeAttitudeEstimator(
            context,
            attitudeAvailableListener = attitudeAvailableListener
        )

        val gravityAndGyroscopeCollector: GravityAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("gravityAndGyroscopeCollector")
        requireNotNull(gravityAndGyroscopeCollector)

        val measurement = GravityAndGyroscopeSyncedSensorMeasurement()

        val gravityProcessor: LeveledRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        every { gravityProcessorSpy.process(measurement) }.returns(false)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val listener = gravityAndGyroscopeCollector.measurementListener
        requireNotNull(listener)
        listener.onMeasurement(gravityAndGyroscopeCollector, measurement)

        verify { attitudeAvailableListener wasNot Called }
    }

    @Test
    fun gravityCollector_whenSyncedMeasurementProcessedAndNoListener_updatesFusedAttitude() {
        val estimator = LeveledRelativeAttitudeEstimator(context)

        val gravityAndGyroscopeCollector: GravityAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("gravityAndGyroscopeCollector")
        requireNotNull(gravityAndGyroscopeCollector)

        val measurement = GravityAndGyroscopeSyncedSensorMeasurement()
        val fusedAttitude1 = getAttitude()

        val gravityProcessor: LeveledRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        every { gravityProcessorSpy.process(measurement) }.returns(true)
        every { gravityProcessorSpy.fusedAttitude }.returns(fusedAttitude1)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val listener = gravityAndGyroscopeCollector.measurementListener
        requireNotNull(listener)
        listener.onMeasurement(gravityAndGyroscopeCollector, measurement)

        val fusedAttitude2: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude2)
        assertEquals(fusedAttitude1, fusedAttitude2)
    }

    @Test
    fun gravityCollector_whenSyncedMeasurementProcessedEstimateCoordinateTransformationAndEulerAnglesDisableAndListener_updatesFusedAttitudeAndNotifies() {
        val estimator = LeveledRelativeAttitudeEstimator(
            context,
            estimateCoordinateTransformation = false,
            estimateEulerAngles = false,
            attitudeAvailableListener = attitudeAvailableListener
        )

        val gravityAndGyroscopeCollector: GravityAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("gravityAndGyroscopeCollector")
        requireNotNull(gravityAndGyroscopeCollector)

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

        val listener = gravityAndGyroscopeCollector.measurementListener
        requireNotNull(listener)
        listener.onMeasurement(gravityAndGyroscopeCollector, measurement)

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
    fun gravityCollector_whenSyncedMeasurementProcessedEstimateCoordinateTransformationAndEulerAnglesEnableAndListener_updatesFusedAttitudeAndNotifies() {
        val estimator = LeveledRelativeAttitudeEstimator(
            context,
            estimateCoordinateTransformation = true,
            estimateEulerAngles = true,
            attitudeAvailableListener = attitudeAvailableListener
        )

        val gravityAndGyroscopeCollector: GravityAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("gravityAndGyroscopeCollector")
        requireNotNull(gravityAndGyroscopeCollector)

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

        val listener = gravityAndGyroscopeCollector.measurementListener
        requireNotNull(listener)
        listener.onMeasurement(gravityAndGyroscopeCollector, measurement)

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
    fun accelerometerCollector_whenAccuracyChangedAndNoListener_makesNoAction() {
        val estimator = LeveledRelativeAttitudeEstimator(context)

        val accelerometerAndGyroscopeCollector: AccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerAndGyroscopeCollector")
        requireNotNull(accelerometerAndGyroscopeCollector)

        val listener = accelerometerAndGyroscopeCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(
            accelerometerAndGyroscopeCollector,
            SensorType.GRAVITY,
            SensorAccuracy.MEDIUM
        )
    }

    @Test
    fun accelerometerCollector_whenAccuracyChangedAndListener_notifies() {
        val estimator = LeveledRelativeAttitudeEstimator(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val accelerometerAndGyroscopeCollector: AccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerAndGyroscopeCollector")
        requireNotNull(accelerometerAndGyroscopeCollector)

        val listener = accelerometerAndGyroscopeCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(
            accelerometerAndGyroscopeCollector,
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
    fun accelerometerCollector_whenSyncedMeasurementAndNotProcessed_makesNoAction() {
        val estimator = LeveledRelativeAttitudeEstimator(
            context,
            attitudeAvailableListener = attitudeAvailableListener
        )

        val accelerometerAndGyroscopeCollector: AccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerAndGyroscopeCollector")
        requireNotNull(accelerometerAndGyroscopeCollector)

        val measurement = AccelerometerAndGyroscopeSyncedSensorMeasurement()

        val accelerometerProcessor: AccelerometerLeveledRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)
        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        every { accelerometerProcessorSpy.process(measurement) }.returns(false)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)

        val listener = accelerometerAndGyroscopeCollector.measurementListener
        requireNotNull(listener)
        listener.onMeasurement(accelerometerAndGyroscopeCollector, measurement)

        verify { attitudeAvailableListener wasNot Called }
    }

    @Test
    fun accelerometerCollector_whenSyncedMeasurementProcessedAndNoListener_updatesFusedAttitude() {
        val estimator = LeveledRelativeAttitudeEstimator(context)

        val accelerometerAndGyroscopeCollector: AccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerAndGyroscopeCollector")
        requireNotNull(accelerometerAndGyroscopeCollector)

        val measurement = AccelerometerAndGyroscopeSyncedSensorMeasurement()
        val fusedAttitude1 = getAttitude()

        val accelerometerProcessor: AccelerometerLeveledRelativeAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)
        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        every { accelerometerProcessorSpy.process(measurement) }.returns(true)
        every { accelerometerProcessorSpy.fusedAttitude }.returns(fusedAttitude1)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)

        val listener = accelerometerAndGyroscopeCollector.measurementListener
        requireNotNull(listener)
        listener.onMeasurement(accelerometerAndGyroscopeCollector, measurement)

        val fusedAttitude2: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude2)
        assertEquals(fusedAttitude1, fusedAttitude2)
    }

    @Test
    fun accelerometerCollector_whenSyncedMeasurementProcessedEstimateCoordinateTransformationAndEulerAnglesDisableAndListener_updatesFusedAttitudeAndNotifies() {
        val estimator = LeveledRelativeAttitudeEstimator(
            context,
            estimateCoordinateTransformation = false,
            estimateEulerAngles = false,
            attitudeAvailableListener = attitudeAvailableListener
        )

        val accelerometerAndGyroscopeCollector: AccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerAndGyroscopeCollector")
        requireNotNull(accelerometerAndGyroscopeCollector)

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

        val listener = accelerometerAndGyroscopeCollector.measurementListener
        requireNotNull(listener)
        listener.onMeasurement(accelerometerAndGyroscopeCollector, measurement)

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
        val estimator = LeveledRelativeAttitudeEstimator(
            context,
            estimateCoordinateTransformation = true,
            estimateEulerAngles = true,
            attitudeAvailableListener = attitudeAvailableListener
        )

        val accelerometerAndGyroscopeCollector: AccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerAndGyroscopeCollector")
        requireNotNull(accelerometerAndGyroscopeCollector)

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

        val listener = accelerometerAndGyroscopeCollector.measurementListener
        requireNotNull(listener)
        listener.onMeasurement(accelerometerAndGyroscopeCollector, measurement)

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

    private fun getLocation(): Location {
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

        every { location.latitude }.returns(latitudeDegrees)
        every { location.longitude }.returns(longitudeDegrees)
        every { location.altitude }.returns(height)

        return location
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

        fun getAttitude(): Quaternion {
            val randomizer = UniformRandomizer()
            val roll = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val pitch = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))
            val yaw = Math.toRadians(randomizer.nextDouble(MIN_ANGLE_DEGREES, MAX_ANGLE_DEGREES))

            return Quaternion(roll, pitch, yaw)
        }
    }
}