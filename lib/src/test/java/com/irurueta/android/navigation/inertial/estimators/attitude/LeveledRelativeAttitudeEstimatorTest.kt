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
package com.irurueta.android.navigation.inertial.estimators.attitude

import android.content.Context
import android.location.Location
import android.os.SystemClock
import androidx.test.core.app.ApplicationProvider
import com.irurueta.android.navigation.inertial.QuaternionHelper
import com.irurueta.android.navigation.inertial.collectors.*
import com.irurueta.android.navigation.inertial.estimators.filter.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.MeanAveragingFilter
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.android.testutils.setPrivateProperty
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType
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
import kotlin.math.abs

@Ignore("possible memory leak")
@RunWith(RobolectricTestRunner::class)
class LeveledRelativeAttitudeEstimatorTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK
    private lateinit var listener: LeveledRelativeAttitudeEstimator.OnAttitudeAvailableListener

    @MockK
    private lateinit var accelerometerMeasurementListener:
            AccelerometerSensorCollector.OnMeasurementListener

    @MockK
    private lateinit var gravityMeasurementListener: GravitySensorCollector.OnMeasurementListener

    @MockK
    private lateinit var gyroscopeMeasurementListener:
            GyroscopeSensorCollector.OnMeasurementListener

    @MockK
    private lateinit var gravityEstimationListener: GravityEstimator.OnEstimationListener

    @MockK(relaxUnitFun = true)
    private lateinit var attitudeAvailableListener:
            LeveledRelativeAttitudeEstimator.OnAttitudeAvailableListener

    @MockK
    private lateinit var location: Location

    @After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
    }

    @Test
    fun constructor_whenRequiredProperties_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(context)

        // check
        assertSame(context, estimator.context)
        assertNull(estimator.location)
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
        assertNull(estimator.accelerometerMeasurementListener)
        assertNull(estimator.gravityMeasurementListener)
        assertNull(estimator.gyroscopeMeasurementListener)
        assertNull(estimator.gravityEstimationListener)
        assertEquals(0.0, estimator.gyroscopeAverageTimeInterval, 0.0)
        assertFalse(estimator.running)
        assertTrue(estimator.useIndirectInterpolation)
        assertEquals(
            LeveledRelativeAttitudeEstimator.DEFAULT_INTERPOLATION_VALUE,
            estimator.interpolationValue,
            0.0
        )
        assertEquals(
            LeveledRelativeAttitudeEstimator.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            estimator.indirectInterpolationWeight,
            0.0
        )
        assertEquals(
            LeveledRelativeAttitudeEstimator.DEFAULT_OUTLIER_THRESHOLD,
            estimator.outlierThreshold,
            0.0
        )
        assertEquals(
            LeveledRelativeAttitudeEstimator.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            estimator.outlierPanicThreshold,
            0.0
        )
        assertEquals(
            LeveledRelativeAttitudeEstimator.DEFAULT_PANIC_COUNTER_THRESHOLD,
            estimator.panicCounterThreshold
        )
    }

    @Test
    fun constructor_whenAllProperties_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val accelerometerAveragingFilter = MeanAveragingFilter()
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
            listener,
            accelerometerMeasurementListener,
            gravityMeasurementListener,
            gyroscopeMeasurementListener,
            gravityEstimationListener
        )

        // check
        assertSame(context, estimator.context)
        assertSame(location, estimator.location)
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
        assertSame(listener, estimator.attitudeAvailableListener)
        assertSame(accelerometerMeasurementListener, estimator.accelerometerMeasurementListener)
        assertSame(gravityMeasurementListener, estimator.gravityMeasurementListener)
        assertSame(gyroscopeMeasurementListener, estimator.gyroscopeMeasurementListener)
        assertSame(gravityEstimationListener, estimator.gravityEstimationListener)
        assertEquals(0.0, estimator.gyroscopeAverageTimeInterval, 0.0)
        assertFalse(estimator.running)
        assertTrue(estimator.useIndirectInterpolation)
        assertEquals(
            LeveledRelativeAttitudeEstimator.DEFAULT_INTERPOLATION_VALUE,
            estimator.interpolationValue,
            0.0
        )
        assertEquals(
            LeveledRelativeAttitudeEstimator.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            estimator.indirectInterpolationWeight,
            0.0
        )
        assertEquals(
            LeveledRelativeAttitudeEstimator.DEFAULT_OUTLIER_THRESHOLD,
            estimator.outlierThreshold,
            0.0
        )
        assertEquals(
            LeveledRelativeAttitudeEstimator.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            estimator.outlierPanicThreshold,
            0.0
        )
        assertEquals(
            LeveledRelativeAttitudeEstimator.DEFAULT_PANIC_COUNTER_THRESHOLD,
            estimator.panicCounterThreshold
        )
    }

    @Test
    fun location_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(context)

        assertFalse(estimator.running)
        assertNull(estimator.location)

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

    @Test(expected = IllegalStateException::class)
    fun location_whenRunningAndNull_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = LeveledRelativeAttitudeEstimator(context, location)

        assertFalse(estimator.running)
        assertSame(location, estimator.location)

        // set as running
        estimator.setPrivateProperty("running", true)

        assertTrue(estimator.running)

        // set new value
        estimator.location = null
    }

    @Test
    fun attitudeAvailableListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(context)

        // check default value
        assertNull(estimator.attitudeAvailableListener)

        // set new value
        estimator.attitudeAvailableListener = attitudeAvailableListener

        // check
        assertSame(attitudeAvailableListener, estimator.attitudeAvailableListener)
    }

    @Test
    fun accelerometerMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(context)

        // check default value
        assertNull(estimator.accelerometerMeasurementListener)

        // set new value
        estimator.accelerometerMeasurementListener = accelerometerMeasurementListener

        // check
        assertSame(accelerometerMeasurementListener, estimator.accelerometerMeasurementListener)

        val levelingEstimator: BaseLevelingEstimator<*, *>? =
            estimator.getPrivateProperty("levelingEstimator")
        requireNotNull(levelingEstimator)
        assertSame(
            levelingEstimator.accelerometerMeasurementListener,
            accelerometerMeasurementListener
        )
    }

    @Test
    fun gravityMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(context)

        // check default value
        assertNull(estimator.gravityMeasurementListener)

        // set new value
        estimator.gravityMeasurementListener = gravityMeasurementListener

        // check
        assertSame(gravityMeasurementListener, estimator.gravityMeasurementListener)

        val levelingEstimator: BaseLevelingEstimator<*, *>? =
            estimator.getPrivateProperty("levelingEstimator")
        requireNotNull(levelingEstimator)
        assertSame(levelingEstimator.gravityMeasurementListener, gravityMeasurementListener)
    }

    @Test
    fun gyroscopeMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(context)

        // check default value
        assertNull(estimator.gyroscopeMeasurementListener)

        // set new value
        estimator.gyroscopeMeasurementListener = gyroscopeMeasurementListener

        // check
        assertSame(gyroscopeMeasurementListener, estimator.gyroscopeMeasurementListener)
    }

    @Test
    fun gravityEstimationListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(context)

        // check default value
        assertNull(estimator.gravityEstimationListener)

        // set new value
        estimator.gravityEstimationListener = gravityEstimationListener

        // check
        assertSame(gravityEstimationListener, estimator.gravityEstimationListener)

        val levelingEstimator: BaseLevelingEstimator<*, *>? =
            estimator.getPrivateProperty("levelingEstimator")
        requireNotNull(levelingEstimator)
        assertEquals(levelingEstimator.gravityEstimationListener, gravityEstimationListener)
    }

    @Test(expected = IllegalStateException::class)
    fun useAccurateLevelingEstimator_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(context)

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
        val estimator = LeveledRelativeAttitudeEstimator(context)

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
        val estimator = LeveledRelativeAttitudeEstimator(context)

        assertFalse(estimator.useAccurateLevelingEstimator)
        assertNull(estimator.location)
        assertFalse(estimator.running)

        estimator.useAccurateLevelingEstimator = true
    }

    @Test
    fun useAccurateLevelingEstimator_whenNotRunningAndLocationAndSetToTrue_setsExpectedValueAndBuildsEstimator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = LeveledRelativeAttitudeEstimator(context, location)

        val levelingEstimator1: BaseLevelingEstimator<*, *>? =
            estimator.getPrivateProperty("levelingEstimator")
        assertNotNull(levelingEstimator1)

        assertFalse(estimator.useAccurateLevelingEstimator)
        assertFalse(estimator.running)
        assertSame(location, estimator.location)

        // set to true
        estimator.useAccurateLevelingEstimator = true

        // check
        assertTrue(estimator.useAccurateLevelingEstimator)

        val levelingEstimator2: BaseLevelingEstimator<*, *>? =
            estimator.getPrivateProperty("levelingEstimator")
        requireNotNull(levelingEstimator2)
        assertTrue(levelingEstimator2 is AccurateLevelingEstimator)
        val accurateLevelingEstimator = levelingEstimator2 as AccurateLevelingEstimator
        assertSame(context, accurateLevelingEstimator.context)
        assertSame(location, accurateLevelingEstimator.location)
        assertEquals(estimator.sensorDelay, accurateLevelingEstimator.sensorDelay)
        assertEquals(estimator.useAccelerometer, accurateLevelingEstimator.useAccelerometer)
        assertEquals(
            estimator.accelerometerSensorType,
            accurateLevelingEstimator.accelerometerSensorType
        )
        assertSame(
            estimator.accelerometerAveragingFilter,
            accurateLevelingEstimator.accelerometerAveragingFilter
        )
        assertFalse(accurateLevelingEstimator.estimateCoordinateTransformation)
        assertFalse(accurateLevelingEstimator.estimateEulerAngles)
        assertNotNull(accurateLevelingEstimator.levelingAvailableListener)

        // set to false
        estimator.useAccurateLevelingEstimator = false

        // check
        assertFalse(estimator.useAccurateLevelingEstimator)

        val levelingEstimator3: BaseLevelingEstimator<*, *>? =
            estimator.getPrivateProperty("levelingEstimator")
        requireNotNull(levelingEstimator3)
        assertTrue(levelingEstimator3 is LevelingEstimator)
        val levelingEstimator = levelingEstimator3 as LevelingEstimator
        assertSame(context, levelingEstimator.context)
        assertEquals(estimator.sensorDelay, levelingEstimator.sensorDelay)
        assertEquals(estimator.useAccelerometer, levelingEstimator.useAccelerometer)
        assertEquals(estimator.accelerometerSensorType, levelingEstimator.accelerometerSensorType)
        assertSame(
            estimator.accelerometerAveragingFilter,
            levelingEstimator.accelerometerAveragingFilter
        )
        assertFalse(levelingEstimator.estimateCoordinateTransformation)
        assertFalse(levelingEstimator.estimateEulerAngles)
        assertNotNull(levelingEstimator.levelingAvailableListener)
    }

    @Test(expected = IllegalStateException::class)
    fun useAccurateRelativeGyroscopeAttitudeEstimator_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(context)

        assertFalse(estimator.running)

        // set as running
        estimator.setPrivateProperty("running", true)

        assertTrue(estimator.running)

        estimator.useAccurateRelativeGyroscopeAttitudeEstimator = true
    }

    @Test
    fun useAccurateRelativeGyroscopeAttitudeEstimator_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(context)

        assertFalse(estimator.running)
        assertTrue(estimator.useAccurateRelativeGyroscopeAttitudeEstimator)

        // set new value
        estimator.useAccurateRelativeGyroscopeAttitudeEstimator = false

        // check
        assertFalse(estimator.useAccurateRelativeGyroscopeAttitudeEstimator)
    }

    @Test
    fun useIndirectInterpolation_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(context)

        // check default value
        assertTrue(estimator.useIndirectInterpolation)

        // set new value
        estimator.useIndirectInterpolation = false

        // check
        @Suppress("KotlinConstantConditions")
        assertFalse(estimator.useIndirectInterpolation)
    }

    @Test
    fun interpolationValue_whenOutOfRange_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(context)

        assertEquals(
            LeveledRelativeAttitudeEstimator.DEFAULT_INTERPOLATION_VALUE,
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
    fun interpolationValue_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(context)

        assertEquals(
            LeveledRelativeAttitudeEstimator.DEFAULT_INTERPOLATION_VALUE,
            estimator.interpolationValue,
            0.0
        )

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        estimator.interpolationValue = value

        // check
        assertEquals(value, estimator.interpolationValue, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun indirectInterpolationWeight_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(context)

        estimator.indirectInterpolationWeight = 0.0
    }

    @Test
    fun indirectInterpolationWeight_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(context)

        // check default value
        assertEquals(
            LeveledRelativeAttitudeEstimator.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            estimator.indirectInterpolationWeight,
            0.0
        )

        // sets new value
        val randomizer = UniformRandomizer()
        val indirectInterpolationWeight = randomizer.nextDouble()
        estimator.indirectInterpolationWeight = indirectInterpolationWeight

        // check
        assertEquals(indirectInterpolationWeight, estimator.indirectInterpolationWeight, 0.0)
    }

    @Test
    fun gyroscopeAverageTimeInterval_returnsInternalAttitudeEstimatorAverageTimeInterval() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(context)

        val attitudeEstimator: BaseRelativeGyroscopeAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(attitudeEstimator)
        val attitudeEstimatorSpy = spyk(attitudeEstimator)
        every { attitudeEstimatorSpy.averageTimeInterval }.returns(TIME_INTERVAL)
        estimator.setPrivateProperty("attitudeEstimator", attitudeEstimatorSpy)

        assertEquals(TIME_INTERVAL, estimator.gyroscopeAverageTimeInterval, 0.0)
        verify(exactly = 1) { attitudeEstimatorSpy.averageTimeInterval }
    }

    @Test
    fun running_whenInternalEstimatorsAreNotRunning_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(context)

        assertFalse(estimator.running)
    }

    @Test
    fun running_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(context)

        assertFalse(estimator.running)

        // set as running
        estimator.setPrivateProperty("running", true)

        assertTrue(estimator.running)
    }

    @Test(expected = IllegalStateException::class)
    fun outlierThreshold_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(context)

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
        val estimator = LeveledRelativeAttitudeEstimator(context)

        // check default value
        assertEquals(
            LeveledRelativeAttitudeEstimator.DEFAULT_OUTLIER_THRESHOLD,
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
        val estimator = LeveledRelativeAttitudeEstimator(context)

        // check default value
        assertEquals(
            LeveledRelativeAttitudeEstimator.DEFAULT_OUTLIER_THRESHOLD,
            estimator.outlierThreshold,
            0.0
        )
        assertFalse(estimator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val outlierThreshold = randomizer.nextDouble()

        estimator.outlierThreshold = outlierThreshold

        // check
        assertEquals(outlierThreshold, estimator.outlierThreshold, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun outlierPanicThreshold_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(context)

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
        val estimator = LeveledRelativeAttitudeEstimator(context)

        // check default value
        assertEquals(
            LeveledRelativeAttitudeEstimator.DEFAULT_OUTLIER_PANIC_THRESHOLD,
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
        val estimator = LeveledRelativeAttitudeEstimator(context)

        // check default value
        assertEquals(
            LeveledRelativeAttitudeEstimator.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            estimator.outlierPanicThreshold,
            0.0
        )
        assertFalse(estimator.running)

        // set new value
        val randomizer = UniformRandomizer()
        val outlierPanicThreshold = randomizer.nextDouble()
        estimator.outlierPanicThreshold = outlierPanicThreshold

        // check
        assertEquals(outlierPanicThreshold, estimator.outlierPanicThreshold, 0.0)
    }

    @Test(expected = IllegalStateException::class)
    fun panicCounterThreshold_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(context)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        estimator.panicCounterThreshold = 1
    }

    @Test(expected = IllegalArgumentException::class)
    fun panicCounterThreshold_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(context)

        assertFalse(estimator.running)

        estimator.panicCounterThreshold = 0
    }

    @Test
    fun panicCounterThreshold_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(context)

        assertFalse(estimator.running)
        assertEquals(
            LeveledRelativeAttitudeEstimator.DEFAULT_PANIC_COUNTER_THRESHOLD,
            estimator.panicCounterThreshold
        )

        estimator.panicCounterThreshold = 2

        // check
        assertEquals(2, estimator.panicCounterThreshold)
    }

    @Test(expected = IllegalStateException::class)
    fun start_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(context)

        assertFalse(estimator.running)

        // set as running
        estimator.setPrivateProperty("running", true)

        assertTrue(estimator.running)

        estimator.start()
    }

    @Test
    fun start_whenNotRunningAndInternalLevelingEstimatorFails_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(context)

        assertFalse(estimator.running)

        val levelingEstimator: BaseLevelingEstimator<*, *>? =
            estimator.getPrivateProperty("levelingEstimator")
        requireNotNull(levelingEstimator)
        val levelingEstimatorSpy = spyk(levelingEstimator)
        every { levelingEstimatorSpy.start() }.returns(false)
        estimator.setPrivateProperty("levelingEstimator", levelingEstimatorSpy)

        val attitudeEstimator: BaseRelativeGyroscopeAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(attitudeEstimator)
        val attitudeEstimatorSpy = spyk(attitudeEstimator)
        estimator.setPrivateProperty("attitudeEstimator", attitudeEstimatorSpy)

        assertFalse(estimator.start())

        verify(exactly = 1) { levelingEstimatorSpy.start() }
        verify(exactly = 0) { attitudeEstimatorSpy.start() }
        verify(exactly = 1) { levelingEstimatorSpy.stop() }
        verify(exactly = 1) { attitudeEstimatorSpy.stop() }
    }

    @Test
    fun start_whenNotRunningAndInternalAttitudeEstimatorFails_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(context)

        assertFalse(estimator.running)

        val levelingEstimator: BaseLevelingEstimator<*, *>? =
            estimator.getPrivateProperty("levelingEstimator")
        requireNotNull(levelingEstimator)
        val levelingEstimatorSpy = spyk(levelingEstimator)
        every { levelingEstimatorSpy.start() }.returns(true)
        estimator.setPrivateProperty("levelingEstimator", levelingEstimatorSpy)

        val attitudeEstimator: BaseRelativeGyroscopeAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(attitudeEstimator)
        val attitudeEstimatorSpy = spyk(attitudeEstimator)
        every { attitudeEstimatorSpy.start() }.returns(false)
        estimator.setPrivateProperty("attitudeEstimator", attitudeEstimatorSpy)

        assertFalse(estimator.start())

        verify(exactly = 1) { levelingEstimatorSpy.start() }
        verify(exactly = 1) { attitudeEstimatorSpy.start() }
    }

    @Test
    fun start_whenNotRunningAndInternalEstimatorSucceeds_returnsTrue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(context)

        assertFalse(estimator.running)

        val levelingEstimator: BaseLevelingEstimator<*, *>? =
            estimator.getPrivateProperty("levelingEstimator")
        requireNotNull(levelingEstimator)
        val levelingEstimatorSpy = spyk(levelingEstimator)
        every { levelingEstimatorSpy.start() }.returns(true)
        estimator.setPrivateProperty("levelingEstimator", levelingEstimatorSpy)

        val attitudeEstimator: BaseRelativeGyroscopeAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(attitudeEstimator)
        val attitudeEstimatorSpy = spyk(attitudeEstimator)
        every { attitudeEstimatorSpy.start() }.returns(true)
        estimator.setPrivateProperty("attitudeEstimator", attitudeEstimatorSpy)

        assertTrue(estimator.start())

        verify(exactly = 1) { levelingEstimatorSpy.start() }
        verify(exactly = 1) { attitudeEstimatorSpy.start() }
    }

    @Test
    fun stop_stopsInternalEstimators() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(context)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        val levelingEstimator: BaseLevelingEstimator<*, *>? =
            estimator.getPrivateProperty("levelingEstimator")
        requireNotNull(levelingEstimator)
        val levelingEstimatorSpy = spyk(levelingEstimator)
        justRun { levelingEstimatorSpy.stop() }
        estimator.setPrivateProperty("levelingEstimator", levelingEstimatorSpy)

        val attitudeEstimator: BaseRelativeGyroscopeAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(attitudeEstimator)
        val attitudeEstimatorSpy = spyk(attitudeEstimator)
        justRun { attitudeEstimatorSpy.stop() }
        estimator.setPrivateProperty("attitudeEstimator", attitudeEstimatorSpy)

        estimator.stop()

        verify(exactly = 1) { levelingEstimatorSpy.stop() }
        verify(exactly = 1) { attitudeEstimatorSpy.stop() }
        assertFalse(estimator.running)
    }

    @Test
    fun processRelativeAttitude_whenFirstAndNonAccurateRelativeAttitudeEstimator_copiesAttitudeAndSetsPreviousRelativeAttitude() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(
            context,
            useAccurateRelativeGyroscopeAttitudeEstimator = false
        )

        // check initial values
        val relativeAttitude1: Quaternion? = estimator.getPrivateProperty("relativeAttitude")
        requireNotNull(relativeAttitude1)
        assertEquals(Quaternion(), relativeAttitude1)
        val hasRelativeAttitude1: Boolean? = estimator.getPrivateProperty("hasRelativeAttitude")
        requireNotNull(hasRelativeAttitude1)
        assertFalse(hasRelativeAttitude1)
        val hasDeltaRelativeAttitude1: Boolean? =
            estimator.getPrivateProperty("hasDeltaRelativeAttitude")
        requireNotNull(hasDeltaRelativeAttitude1)
        assertFalse(hasDeltaRelativeAttitude1)
        val previousRelativeAttitude1: Quaternion? =
            estimator.getPrivateProperty("previousRelativeAttitude")
        assertNull(previousRelativeAttitude1)

        val attitudeEstimator: RelativeGyroscopeAttitudeEstimator? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(attitudeEstimator)

        val internalAttitude = spyk(getAttitude())
        val listener = attitudeEstimator.attitudeAvailableListener
        requireNotNull(listener)
        val timestamp = SystemClock.elapsedRealtimeNanos()
        listener.onAttitudeAvailable(
            attitudeEstimator,
            internalAttitude,
            timestamp,
            null,
            null,
            null,
            null
        )

        verify(exactly = 1) { internalAttitude.copyTo(relativeAttitude1) }
        assertEquals(internalAttitude, relativeAttitude1)
        val hasRelativeAttitude2: Boolean? = estimator.getPrivateProperty("hasRelativeAttitude")
        requireNotNull(hasRelativeAttitude2)
        assertTrue(hasRelativeAttitude2)
        val hasDeltaRelativeAttitude2: Boolean? =
            estimator.getPrivateProperty("hasDeltaRelativeAttitude")
        requireNotNull(hasDeltaRelativeAttitude2)
        assertFalse(hasDeltaRelativeAttitude2)
        val previousRelativeAttitude2: Quaternion? =
            estimator.getPrivateProperty("previousRelativeAttitude")
        requireNotNull(previousRelativeAttitude2)
        assertEquals(internalAttitude, previousRelativeAttitude2)
    }

    @Test
    fun processRelativeAttitude_whenFirstAndAccurateRelativeAttitudeEstimator_copiesAttitudeAndSetsPreviousRelativeAttitude() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(
            context,
            useAccurateRelativeGyroscopeAttitudeEstimator = true
        )

        // check initial values
        val relativeAttitude1: Quaternion? = estimator.getPrivateProperty("relativeAttitude")
        requireNotNull(relativeAttitude1)
        assertEquals(Quaternion(), relativeAttitude1)
        val hasRelativeAttitude1: Boolean? = estimator.getPrivateProperty("hasRelativeAttitude")
        requireNotNull(hasRelativeAttitude1)
        assertFalse(hasRelativeAttitude1)
        val hasDeltaRelativeAttitude1: Boolean? =
            estimator.getPrivateProperty("hasDeltaRelativeAttitude")
        requireNotNull(hasDeltaRelativeAttitude1)
        assertFalse(hasDeltaRelativeAttitude1)
        val previousRelativeAttitude1: Quaternion? =
            estimator.getPrivateProperty("previousRelativeAttitude")
        assertNull(previousRelativeAttitude1)

        val attitudeEstimator: AccurateRelativeGyroscopeAttitudeEstimator? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(attitudeEstimator)

        val internalAttitude = spyk(getAttitude())
        val listener = attitudeEstimator.attitudeAvailableListener
        requireNotNull(listener)
        val timestamp = SystemClock.elapsedRealtimeNanos()
        listener.onAttitudeAvailable(
            attitudeEstimator,
            internalAttitude,
            timestamp,
            null,
            null,
            null,
            null
        )

        verify(exactly = 1) { internalAttitude.copyTo(relativeAttitude1) }
        assertEquals(internalAttitude, relativeAttitude1)
        val hasRelativeAttitude2: Boolean? = estimator.getPrivateProperty("hasRelativeAttitude")
        requireNotNull(hasRelativeAttitude2)
        assertTrue(hasRelativeAttitude2)
        val hasDeltaRelativeAttitude2: Boolean? =
            estimator.getPrivateProperty("hasDeltaRelativeAttitude")
        requireNotNull(hasDeltaRelativeAttitude2)
        assertFalse(hasDeltaRelativeAttitude2)
        val previousRelativeAttitude2: Quaternion? =
            estimator.getPrivateProperty("previousRelativeAttitude")
        requireNotNull(previousRelativeAttitude2)
        assertEquals(internalAttitude, previousRelativeAttitude2)
    }

    @Test
    fun processRelativeAttitude_whenNotFirstAndNonAccurateRelativeAttitudeEstimator_copiesAttitudeAndSetsPreviousRelativeAttitude() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(
            context,
            useAccurateRelativeGyroscopeAttitudeEstimator = false
        )

        // check initial values
        val relativeAttitude1: Quaternion? = estimator.getPrivateProperty("relativeAttitude")
        requireNotNull(relativeAttitude1)
        assertEquals(Quaternion(), relativeAttitude1)
        val hasRelativeAttitude1: Boolean? = estimator.getPrivateProperty("hasRelativeAttitude")
        requireNotNull(hasRelativeAttitude1)
        assertFalse(hasRelativeAttitude1)
        val hasDeltaRelativeAttitude1: Boolean? =
            estimator.getPrivateProperty("hasDeltaRelativeAttitude")
        requireNotNull(hasDeltaRelativeAttitude1)
        assertFalse(hasDeltaRelativeAttitude1)
        val previousRelativeAttitude1: Quaternion? =
            estimator.getPrivateProperty("previousRelativeAttitude")
        assertNull(previousRelativeAttitude1)

        val attitudeEstimator: RelativeGyroscopeAttitudeEstimator? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(attitudeEstimator)

        // call listener for the 1st time
        val internalAttitude1 = spyk(getAttitude())
        val listener = attitudeEstimator.attitudeAvailableListener
        requireNotNull(listener)
        val timestamp = SystemClock.elapsedRealtimeNanos()
        listener.onAttitudeAvailable(
            attitudeEstimator,
            internalAttitude1,
            timestamp,
            null,
            null,
            null,
            null
        )

        verify(exactly = 1) { internalAttitude1.copyTo(relativeAttitude1) }
        assertEquals(internalAttitude1, relativeAttitude1)
        val hasRelativeAttitude2: Boolean? = estimator.getPrivateProperty("hasRelativeAttitude")
        requireNotNull(hasRelativeAttitude2)
        assertTrue(hasRelativeAttitude2)
        val hasDeltaRelativeAttitude2: Boolean? =
            estimator.getPrivateProperty("hasDeltaRelativeAttitude")
        requireNotNull(hasDeltaRelativeAttitude2)
        assertFalse(hasDeltaRelativeAttitude2)
        val previousRelativeAttitude2: Quaternion? =
            estimator.getPrivateProperty("previousRelativeAttitude")
        requireNotNull(previousRelativeAttitude2)
        assertEquals(internalAttitude1, previousRelativeAttitude2)

        // call listener a 2nd time
        val deltaRelativeAttitude1 = getAttitude()
        val internalAttitude2 = spyk(deltaRelativeAttitude1.combineAndReturnNew(internalAttitude1))
        listener.onAttitudeAvailable(
            attitudeEstimator,
            internalAttitude2,
            timestamp,
            null,
            null,
            null,
            null
        )

        verify(exactly = 1) { internalAttitude2.copyTo(relativeAttitude1) }
        assertEquals(internalAttitude2, relativeAttitude1)
        val hasRelativeAttitude3: Boolean? = estimator.getPrivateProperty("hasRelativeAttitude")
        requireNotNull(hasRelativeAttitude3)
        assertTrue(hasRelativeAttitude3)
        val hasDeltaRelativeAttitude3: Boolean? =
            estimator.getPrivateProperty("hasDeltaRelativeAttitude")
        requireNotNull(hasDeltaRelativeAttitude3)
        assertTrue(hasDeltaRelativeAttitude3)
        val previousRelativeAttitude3: Quaternion? =
            estimator.getPrivateProperty("previousRelativeAttitude")
        requireNotNull(previousRelativeAttitude3)
        assertEquals(internalAttitude1, previousRelativeAttitude3)
        val inversePreviousRelativeAttitude: Quaternion? =
            estimator.getPrivateProperty("inversePreviousRelativeAttitude")
        requireNotNull(inversePreviousRelativeAttitude)
        assertEquals(
            previousRelativeAttitude2.inverseAndReturnNew(),
            inversePreviousRelativeAttitude
        )
        val deltaRelativeAttitude2: Quaternion? =
            estimator.getPrivateProperty("deltaRelativeAttitude")
        requireNotNull(deltaRelativeAttitude2)
        assertEquals(deltaRelativeAttitude1, deltaRelativeAttitude2)
    }

    @Test
    fun processRelativeAttitude_whenNotFirstAndAccurateRelativeAttitudeEstimator_copiesAttitudeAndSetsPreviousRelativeAttitude() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(
            context,
            useAccurateRelativeGyroscopeAttitudeEstimator = true
        )

        // check initial values
        val relativeAttitude1: Quaternion? = estimator.getPrivateProperty("relativeAttitude")
        requireNotNull(relativeAttitude1)
        assertEquals(Quaternion(), relativeAttitude1)
        val hasRelativeAttitude1: Boolean? = estimator.getPrivateProperty("hasRelativeAttitude")
        requireNotNull(hasRelativeAttitude1)
        assertFalse(hasRelativeAttitude1)
        val hasDeltaRelativeAttitude1: Boolean? =
            estimator.getPrivateProperty("hasDeltaRelativeAttitude")
        requireNotNull(hasDeltaRelativeAttitude1)
        assertFalse(hasDeltaRelativeAttitude1)
        val previousRelativeAttitude1: Quaternion? =
            estimator.getPrivateProperty("previousRelativeAttitude")
        assertNull(previousRelativeAttitude1)

        val attitudeEstimator: AccurateRelativeGyroscopeAttitudeEstimator? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(attitudeEstimator)

        // call listener for the 1st time
        val internalAttitude1 = spyk(getAttitude())
        val listener = attitudeEstimator.attitudeAvailableListener
        requireNotNull(listener)
        val timestamp = SystemClock.elapsedRealtimeNanos()
        listener.onAttitudeAvailable(
            attitudeEstimator,
            internalAttitude1,
            timestamp,
            null,
            null,
            null,
            null
        )

        verify(exactly = 1) { internalAttitude1.copyTo(relativeAttitude1) }
        assertEquals(internalAttitude1, relativeAttitude1)
        val hasRelativeAttitude2: Boolean? = estimator.getPrivateProperty("hasRelativeAttitude")
        requireNotNull(hasRelativeAttitude2)
        assertTrue(hasRelativeAttitude2)
        val hasDeltaRelativeAttitude2: Boolean? =
            estimator.getPrivateProperty("hasDeltaRelativeAttitude")
        requireNotNull(hasDeltaRelativeAttitude2)
        assertFalse(hasDeltaRelativeAttitude2)
        val previousRelativeAttitude2: Quaternion? =
            estimator.getPrivateProperty("previousRelativeAttitude")
        requireNotNull(previousRelativeAttitude2)
        assertEquals(internalAttitude1, previousRelativeAttitude2)

        // call listener a 2nd time
        val deltaRelativeAttitude1 = getAttitude()
        val internalAttitude2 = spyk(deltaRelativeAttitude1.combineAndReturnNew(internalAttitude1))
        listener.onAttitudeAvailable(
            attitudeEstimator,
            internalAttitude2,
            timestamp,
            null,
            null,
            null,
            null
        )

        verify(exactly = 1) { internalAttitude2.copyTo(relativeAttitude1) }
        assertEquals(internalAttitude2, relativeAttitude1)
        val hasRelativeAttitude3: Boolean? = estimator.getPrivateProperty("hasRelativeAttitude")
        requireNotNull(hasRelativeAttitude3)
        assertTrue(hasRelativeAttitude3)
        val hasDeltaRelativeAttitude3: Boolean? =
            estimator.getPrivateProperty("hasDeltaRelativeAttitude")
        requireNotNull(hasDeltaRelativeAttitude3)
        assertTrue(hasDeltaRelativeAttitude3)
        val previousRelativeAttitude3: Quaternion? =
            estimator.getPrivateProperty("previousRelativeAttitude")
        requireNotNull(previousRelativeAttitude3)
        assertEquals(internalAttitude1, previousRelativeAttitude3)
        val inversePreviousRelativeAttitude: Quaternion? =
            estimator.getPrivateProperty("inversePreviousRelativeAttitude")
        requireNotNull(inversePreviousRelativeAttitude)
        assertEquals(
            previousRelativeAttitude2.inverseAndReturnNew(),
            inversePreviousRelativeAttitude
        )
        val deltaRelativeAttitude2: Quaternion? =
            estimator.getPrivateProperty("deltaRelativeAttitude")
        requireNotNull(deltaRelativeAttitude2)
        assertEquals(deltaRelativeAttitude1, deltaRelativeAttitude2)
    }

    @Test
    fun processLeveling_whenNoRelativeAttitudeAndNonAccurateLeveling_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(
            context,
            useAccurateLevelingEstimator = false,
            attitudeAvailableListener = attitudeAvailableListener
        )

        val hasRelativeAttitude1: Boolean? = estimator.getPrivateProperty("hasRelativeAttitude")
        requireNotNull(hasRelativeAttitude1)
        assertFalse(hasRelativeAttitude1)

        val levelingAttitude1: Quaternion? = estimator.getPrivateProperty("levelingAttitude")
        requireNotNull(levelingAttitude1)
        assertEquals(Quaternion(), levelingAttitude1)

        val hasDeltaRelativeAttitude1: Boolean? =
            estimator.getPrivateProperty("hasDeltaRelativeAttitude")
        requireNotNull(hasDeltaRelativeAttitude1)
        assertFalse(hasDeltaRelativeAttitude1)

        val levelingEstimator: LevelingEstimator? =
            estimator.getPrivateProperty("levelingEstimator")
        requireNotNull(levelingEstimator)

        val internalAttitude = spyk(getAttitude())
        val listener = levelingEstimator.levelingAvailableListener
        requireNotNull(listener)
        val timestamp = SystemClock.elapsedRealtimeNanos()
        listener.onLevelingAvailable(
            levelingEstimator,
            internalAttitude,
            timestamp,
            null,
            null,
            null
        )

        val levelingAttitude2: Quaternion? = estimator.getPrivateProperty("levelingAttitude")
        requireNotNull(levelingAttitude2)
        assertEquals(Quaternion(), levelingAttitude2)

        verify { attitudeAvailableListener wasNot Called }
        verify { internalAttitude wasNot Called }
    }

    @Test
    fun processLeveling_whenNoRelativeAttitudeAndAccurateLeveling_makesNoAction() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(
            context,
            location,
            useAccurateLevelingEstimator = true,
            attitudeAvailableListener = attitudeAvailableListener
        )

        val hasRelativeAttitude1: Boolean? = estimator.getPrivateProperty("hasRelativeAttitude")
        requireNotNull(hasRelativeAttitude1)
        assertFalse(hasRelativeAttitude1)

        val levelingAttitude1: Quaternion? = estimator.getPrivateProperty("levelingAttitude")
        requireNotNull(levelingAttitude1)
        assertEquals(Quaternion(), levelingAttitude1)

        val hasDeltaRelativeAttitude1: Boolean? =
            estimator.getPrivateProperty("hasDeltaRelativeAttitude")
        requireNotNull(hasDeltaRelativeAttitude1)
        assertFalse(hasDeltaRelativeAttitude1)

        val levelingEstimator: AccurateLevelingEstimator? =
            estimator.getPrivateProperty("levelingEstimator")
        requireNotNull(levelingEstimator)

        val internalAttitude = spyk(getAttitude())
        val listener = levelingEstimator.levelingAvailableListener
        requireNotNull(listener)
        val timestamp = SystemClock.elapsedRealtimeNanos()
        listener.onLevelingAvailable(
            levelingEstimator,
            internalAttitude,
            timestamp,
            null,
            null,
            null
        )

        val levelingAttitude2: Quaternion? = estimator.getPrivateProperty("levelingAttitude")
        requireNotNull(levelingAttitude2)
        assertEquals(Quaternion(), levelingAttitude2)

        verify { attitudeAvailableListener wasNot Called }
        verify { internalAttitude wasNot Called }
    }

    @Test
    fun processLeveling_whenRelativeAttitudeAndNonAccurateLeveling_copiesAttitude() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(
            context,
            useAccurateLevelingEstimator = false,
            attitudeAvailableListener = attitudeAvailableListener
        )

        estimator.setPrivateProperty("hasRelativeAttitude", true)

        val hasRelativeAttitude1: Boolean? = estimator.getPrivateProperty("hasRelativeAttitude")
        requireNotNull(hasRelativeAttitude1)
        assertTrue(hasRelativeAttitude1)

        val levelingAttitude1: Quaternion? = estimator.getPrivateProperty("levelingAttitude")
        requireNotNull(levelingAttitude1)
        assertEquals(Quaternion(), levelingAttitude1)

        val hasDeltaRelativeAttitude1: Boolean? =
            estimator.getPrivateProperty("hasDeltaRelativeAttitude")
        requireNotNull(hasDeltaRelativeAttitude1)
        assertFalse(hasDeltaRelativeAttitude1)

        val levelingEstimator: LevelingEstimator? =
            estimator.getPrivateProperty("levelingEstimator")
        requireNotNull(levelingEstimator)

        val internalAttitude = spyk(getAttitude())
        val listener = levelingEstimator.levelingAvailableListener
        requireNotNull(listener)
        val timestamp = SystemClock.elapsedRealtimeNanos()
        listener.onLevelingAvailable(
            levelingEstimator,
            internalAttitude,
            timestamp,
            null,
            null,
            null
        )

        val levelingAttitude2: Quaternion? = estimator.getPrivateProperty("levelingAttitude")
        requireNotNull(levelingAttitude2)
        assertEquals(internalAttitude, levelingAttitude2)

        verify { attitudeAvailableListener wasNot Called }
        verify(exactly = 1) { internalAttitude.copyTo(levelingAttitude2) }
    }

    @Test
    fun processLeveling_whenRelativeAttitudeAndAccurateLeveling_copiesAttitude() {
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(
            context,
            location,
            useAccurateLevelingEstimator = true,
            attitudeAvailableListener = attitudeAvailableListener
        )

        estimator.setPrivateProperty("hasRelativeAttitude", true)

        val hasRelativeAttitude1: Boolean? = estimator.getPrivateProperty("hasRelativeAttitude")
        requireNotNull(hasRelativeAttitude1)
        assertTrue(hasRelativeAttitude1)

        val levelingAttitude1: Quaternion? = estimator.getPrivateProperty("levelingAttitude")
        requireNotNull(levelingAttitude1)
        assertEquals(Quaternion(), levelingAttitude1)

        val hasDeltaRelativeAttitude1: Boolean? =
            estimator.getPrivateProperty("hasDeltaRelativeAttitude")
        requireNotNull(hasDeltaRelativeAttitude1)
        assertFalse(hasDeltaRelativeAttitude1)

        val levelingEstimator: AccurateLevelingEstimator? =
            estimator.getPrivateProperty("levelingEstimator")
        requireNotNull(levelingEstimator)

        val internalAttitude = spyk(getAttitude())
        val listener = levelingEstimator.levelingAvailableListener
        requireNotNull(listener)
        val timestamp = SystemClock.elapsedRealtimeNanos()
        listener.onLevelingAvailable(
            levelingEstimator,
            internalAttitude,
            timestamp,
            null,
            null,
            null
        )

        val levelingAttitude2: Quaternion? = estimator.getPrivateProperty("levelingAttitude")
        requireNotNull(levelingAttitude2)
        assertEquals(internalAttitude, levelingAttitude2)

        verify { attitudeAvailableListener wasNot Called }
        verify(exactly = 1) { internalAttitude.copyTo(levelingAttitude2) }
    }

    @Test
    fun processLeveling_whenDeltaRelativeAttitudeAndResetLeveling_resets() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(
            context,
            useAccurateLevelingEstimator = false,
            attitudeAvailableListener = attitudeAvailableListener
        )

        estimator.setPrivateProperty("hasRelativeAttitude", true)
        estimator.setPrivateProperty("hasDeltaRelativeAttitude", true)

        val hasRelativeAttitude1: Boolean? = estimator.getPrivateProperty("hasRelativeAttitude")
        requireNotNull(hasRelativeAttitude1)
        assertTrue(hasRelativeAttitude1)

        val levelingAttitude1: Quaternion? = estimator.getPrivateProperty("levelingAttitude")
        requireNotNull(levelingAttitude1)
        assertEquals(Quaternion(), levelingAttitude1)

        val hasDeltaRelativeAttitude1: Boolean? =
            estimator.getPrivateProperty("hasDeltaRelativeAttitude")
        requireNotNull(hasDeltaRelativeAttitude1)
        assertTrue(hasDeltaRelativeAttitude1)

        val levelingAttitude: Quaternion? = estimator.getPrivateProperty("levelingAttitude")
        requireNotNull(levelingAttitude)
        val levelingAttitudeSpy = spyk(levelingAttitude)
        estimator.setPrivateProperty("levelingAttitude", levelingAttitudeSpy)

        val relativeAttitude: Quaternion? = estimator.getPrivateProperty("relativeAttitude")
        requireNotNull(relativeAttitude)
        val attitude = getAttitude()
        attitude.copyTo(relativeAttitude)
        val relativeAttitudeSpy = spyk(relativeAttitude)
        estimator.setPrivateProperty("relativeAttitude", relativeAttitudeSpy)

        val internalFusedAttitude: Quaternion? =
            estimator.getPrivateProperty("internalFusedAttitude")
        requireNotNull(internalFusedAttitude)
        val internalFusedAttitudeSpy = spyk(internalFusedAttitude)
        estimator.setPrivateProperty("internalFusedAttitude", internalFusedAttitudeSpy)

        estimator.setPrivateProperty("panicCounter", estimator.panicCounterThreshold)
        val panicCounter1: Int? = estimator.getPrivateProperty("panicCounter")
        requireNotNull(panicCounter1)
        assertEquals(estimator.panicCounterThreshold, panicCounter1)

        val resetToLeveling: Boolean? = estimator.getPrivateProperty("resetToLeveling")
        requireNotNull(resetToLeveling)
        assertTrue(resetToLeveling)

        val levelingEstimator: LevelingEstimator? =
            estimator.getPrivateProperty("levelingEstimator")
        requireNotNull(levelingEstimator)

        val internalAttitude = spyk(getAttitude())
        val listener = levelingEstimator.levelingAvailableListener
        requireNotNull(listener)
        val timestamp = SystemClock.elapsedRealtimeNanos()
        listener.onLevelingAvailable(
            levelingEstimator,
            internalAttitude,
            timestamp,
            null,
            null,
            null
        )

        val levelingAttitude2: Quaternion? = estimator.getPrivateProperty("levelingAttitude")
        requireNotNull(levelingAttitude2)

        val angles = DoubleArray(3)
        internalAttitude.toEulerAngles(angles)
        val levelingRoll = angles[0]
        val levelingPitch = angles[1]
        attitude.toEulerAngles(angles)
        val relativeYaw = angles[2]

        verify { attitudeAvailableListener wasNot Called }
        verify(exactly = 1) { internalAttitude.copyTo(levelingAttitude2) }

        val eulerAngles: DoubleArray? = estimator.getPrivateProperty("eulerAngles")
        requireNotNull(eulerAngles)
        verify(exactly = 1) { levelingAttitudeSpy.toEulerAngles(eulerAngles) }
        verify(exactly = 1) { relativeAttitudeSpy.toEulerAngles(eulerAngles) }
        verify(exactly = 1) {
            levelingAttitudeSpy.setFromEulerAngles(
                levelingRoll,
                levelingPitch,
                relativeYaw
            )
        }
        verify { levelingAttitudeSpy.copyTo(internalFusedAttitudeSpy) }

        val panicCounter2: Int? = estimator.getPrivateProperty("panicCounter")
        requireNotNull(panicCounter2)
        assertEquals(0, panicCounter2)

        val hasRelativeAttitude2: Boolean? = estimator.getPrivateProperty("hasRelativeAttitude")
        requireNotNull(hasRelativeAttitude2)
        assertTrue(hasRelativeAttitude2)
    }

    @Test
    fun processLeveling_whenDeltaRelativeAttitudeSmallDivergenceAndDirectInterpolation_updatesFusedAttitudeAndNotifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(
            context,
            useAccurateLevelingEstimator = false,
            estimateCoordinateTransformation = false,
            estimateEulerAngles = false,
            attitudeAvailableListener = attitudeAvailableListener
        )
        estimator.useIndirectInterpolation = false

        estimator.setPrivateProperty("hasRelativeAttitude", true)
        estimator.setPrivateProperty("hasDeltaRelativeAttitude", true)

        val hasRelativeAttitude1: Boolean? = estimator.getPrivateProperty("hasRelativeAttitude")
        requireNotNull(hasRelativeAttitude1)
        assertTrue(hasRelativeAttitude1)

        val levelingAttitude1: Quaternion? = estimator.getPrivateProperty("levelingAttitude")
        requireNotNull(levelingAttitude1)
        assertEquals(Quaternion(), levelingAttitude1)

        val hasDeltaRelativeAttitude1: Boolean? =
            estimator.getPrivateProperty("hasDeltaRelativeAttitude")
        requireNotNull(hasDeltaRelativeAttitude1)
        assertTrue(hasDeltaRelativeAttitude1)

        val levelingAttitude: Quaternion? = estimator.getPrivateProperty("levelingAttitude")
        requireNotNull(levelingAttitude)
        val levelingAttitudeSpy = spyk(levelingAttitude)
        estimator.setPrivateProperty("levelingAttitude", levelingAttitudeSpy)

        val relativeAttitude: Quaternion? = estimator.getPrivateProperty("relativeAttitude")
        requireNotNull(relativeAttitude)
        val attitude = getAttitude()
        attitude.copyTo(relativeAttitude)
        val relativeAttitudeSpy = spyk(relativeAttitude)
        estimator.setPrivateProperty("relativeAttitude", relativeAttitudeSpy)

        val internalFusedAttitude: Quaternion? =
            estimator.getPrivateProperty("internalFusedAttitude")
        requireNotNull(internalFusedAttitude)
        getAttitude().copyTo(internalFusedAttitude)
        val internalFusedAttitudeSpy = spyk(internalFusedAttitude)
        estimator.setPrivateProperty("internalFusedAttitude", internalFusedAttitudeSpy)

        val previousRelativeAttitude1 = Quaternion()
        estimator.setPrivateProperty("previousRelativeAttitude", previousRelativeAttitude1)
        assertSame(
            previousRelativeAttitude1,
            estimator.getPrivateProperty("previousRelativeAttitude")
        )

        val deltaRelativeAttitude: Quaternion? =
            estimator.getPrivateProperty("deltaRelativeAttitude")
        requireNotNull(deltaRelativeAttitude)
        val randomizer = UniformRandomizer()
        val deltaRoll = Math.toRadians(randomizer.nextDouble(-DELTA_DEGREES, DELTA_DEGREES))
        val deltaPitch = Math.toRadians(randomizer.nextDouble(-DELTA_DEGREES, DELTA_DEGREES))
        val deltaYaw = Math.toRadians(randomizer.nextDouble(-DELTA_DEGREES, DELTA_DEGREES))
        deltaRelativeAttitude.setFromEulerAngles(deltaRoll, deltaPitch, deltaYaw)

        mockkObject(QuaternionHelper)
        every {
            QuaternionHelper.dotProduct(
                any(),
                any()
            )
        }.returns(estimator.outlierThreshold)

        val fusedAttitude2 = Quaternion(internalFusedAttitude)
        val fusedAttitude3 = deltaRelativeAttitude.multiplyAndReturnNew(fusedAttitude2)

        estimator.setPrivateProperty("panicCounter", 1)
        val panicCounter1: Int? = estimator.getPrivateProperty("panicCounter")
        requireNotNull(panicCounter1)
        assertEquals(1, panicCounter1)

        val resetToLeveling: Boolean? = estimator.getPrivateProperty("resetToLeveling")
        requireNotNull(resetToLeveling)
        assertFalse(resetToLeveling)

        val levelingEstimator: LevelingEstimator? =
            estimator.getPrivateProperty("levelingEstimator")
        requireNotNull(levelingEstimator)

        val internalAttitude = spyk(getAttitude())
        val listener = levelingEstimator.levelingAvailableListener
        requireNotNull(listener)
        val timestamp = SystemClock.elapsedRealtimeNanos()
        estimator.setPrivateProperty("timestamp", timestamp)
        listener.onLevelingAvailable(
            levelingEstimator,
            internalAttitude,
            timestamp,
            null,
            null,
            null
        )

        val levelingAttitude2: Quaternion? = estimator.getPrivateProperty("levelingAttitude")
        requireNotNull(levelingAttitude2)

        val angles = DoubleArray(3)
        internalAttitude.toEulerAngles(angles)
        val levelingRoll = angles[0]
        val levelingPitch = angles[1]
        attitude.toEulerAngles(angles)
        val relativeYaw = angles[2]

        verify(exactly = 1) { internalAttitude.copyTo(levelingAttitude2) }

        val eulerAngles: DoubleArray? = estimator.getPrivateProperty("eulerAngles")
        requireNotNull(eulerAngles)
        verify(exactly = 1) { levelingAttitudeSpy.toEulerAngles(eulerAngles) }
        verify(exactly = 1) { relativeAttitudeSpy.toEulerAngles(eulerAngles) }
        verify(exactly = 1) {
            levelingAttitudeSpy.setFromEulerAngles(
                levelingRoll,
                levelingPitch,
                relativeYaw
            )
        }
        verify(exactly = 0) { levelingAttitudeSpy.copyTo(internalFusedAttitudeSpy) }

        val panicCounter2: Int? = estimator.getPrivateProperty("panicCounter")
        requireNotNull(panicCounter2)
        assertEquals(0, panicCounter2)

        val hasRelativeAttitude2: Boolean? = estimator.getPrivateProperty("hasRelativeAttitude")
        requireNotNull(hasRelativeAttitude2)
        assertTrue(hasRelativeAttitude2)

        val fusedAttitude4 = Quaternion()
        Quaternion.slerp(
            fusedAttitude3,
            levelingAttitudeSpy,
            estimator.interpolationValue,
            fusedAttitude4
        )

        assertEquals(fusedAttitude4, internalFusedAttitudeSpy)

        val absDot = abs(QuaternionHelper.dotProduct(fusedAttitude3, levelingAttitudeSpy))
        assertTrue(absDot >= estimator.outlierThreshold)

        val previousRelativeAttitude2: Quaternion? =
            estimator.getPrivateProperty("previousRelativeAttitude")
        requireNotNull(previousRelativeAttitude2)
        assertEquals(relativeAttitudeSpy, previousRelativeAttitude2)

        val fusedAttitude: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude)

        verify(exactly = 1) {
            attitudeAvailableListener.onAttitudeAvailable(
                estimator,
                fusedAttitude,
                timestamp,
                null,
                null,
                null,
                null
            )
        }

        unmockkObject(QuaternionHelper)
    }

    @Test
    fun processLeveling_whenDeltaRelativeAttitudeMediumDivergenceAndDirectInterpolation_updatesFusedAttitudeAndNotifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(
            context,
            useAccurateLevelingEstimator = false,
            estimateCoordinateTransformation = false,
            estimateEulerAngles = false,
            attitudeAvailableListener = attitudeAvailableListener
        )
        estimator.useIndirectInterpolation = false

        estimator.setPrivateProperty("hasRelativeAttitude", true)
        estimator.setPrivateProperty("hasDeltaRelativeAttitude", true)

        val hasRelativeAttitude1: Boolean? = estimator.getPrivateProperty("hasRelativeAttitude")
        requireNotNull(hasRelativeAttitude1)
        assertTrue(hasRelativeAttitude1)

        val levelingAttitude1: Quaternion? = estimator.getPrivateProperty("levelingAttitude")
        requireNotNull(levelingAttitude1)
        assertEquals(Quaternion(), levelingAttitude1)

        val hasDeltaRelativeAttitude1: Boolean? =
            estimator.getPrivateProperty("hasDeltaRelativeAttitude")
        requireNotNull(hasDeltaRelativeAttitude1)
        assertTrue(hasDeltaRelativeAttitude1)

        val levelingAttitude: Quaternion? = estimator.getPrivateProperty("levelingAttitude")
        requireNotNull(levelingAttitude)
        val levelingAttitudeSpy = spyk(levelingAttitude)
        estimator.setPrivateProperty("levelingAttitude", levelingAttitudeSpy)

        val relativeAttitude: Quaternion? = estimator.getPrivateProperty("relativeAttitude")
        requireNotNull(relativeAttitude)
        val attitude = getAttitude()
        attitude.copyTo(relativeAttitude)
        val relativeAttitudeSpy = spyk(relativeAttitude)
        estimator.setPrivateProperty("relativeAttitude", relativeAttitudeSpy)

        val internalFusedAttitude: Quaternion? =
            estimator.getPrivateProperty("internalFusedAttitude")
        requireNotNull(internalFusedAttitude)
        getAttitude().copyTo(internalFusedAttitude)
        val internalFusedAttitudeSpy = spyk(internalFusedAttitude)
        estimator.setPrivateProperty("internalFusedAttitude", internalFusedAttitudeSpy)

        val previousRelativeAttitude1 = Quaternion()
        estimator.setPrivateProperty("previousRelativeAttitude", previousRelativeAttitude1)
        assertSame(
            previousRelativeAttitude1,
            estimator.getPrivateProperty("previousRelativeAttitude")
        )

        val deltaRelativeAttitude: Quaternion? =
            estimator.getPrivateProperty("deltaRelativeAttitude")
        requireNotNull(deltaRelativeAttitude)
        val randomizer = UniformRandomizer()
        val deltaRoll = Math.toRadians(randomizer.nextDouble(-DELTA_DEGREES, DELTA_DEGREES))
        val deltaPitch = Math.toRadians(randomizer.nextDouble(-DELTA_DEGREES, DELTA_DEGREES))
        val deltaYaw = Math.toRadians(randomizer.nextDouble(-DELTA_DEGREES, DELTA_DEGREES))
        deltaRelativeAttitude.setFromEulerAngles(deltaRoll, deltaPitch, deltaYaw)

        mockkObject(QuaternionHelper)
        every {
            QuaternionHelper.dotProduct(
                any(),
                any()
            )
        }.returns(0.5 * (estimator.outlierThreshold + estimator.outlierPanicThreshold))

        val fusedAttitude2 = Quaternion(internalFusedAttitude)
        val fusedAttitude3 = deltaRelativeAttitude.multiplyAndReturnNew(fusedAttitude2)

        estimator.setPrivateProperty("panicCounter", 1)
        val panicCounter1: Int? = estimator.getPrivateProperty("panicCounter")
        requireNotNull(panicCounter1)
        assertEquals(1, panicCounter1)

        val resetToLeveling: Boolean? = estimator.getPrivateProperty("resetToLeveling")
        requireNotNull(resetToLeveling)
        assertFalse(resetToLeveling)

        val levelingEstimator: LevelingEstimator? =
            estimator.getPrivateProperty("levelingEstimator")
        requireNotNull(levelingEstimator)

        val internalAttitude = spyk(getAttitude())
        val listener = levelingEstimator.levelingAvailableListener
        requireNotNull(listener)
        val timestamp = SystemClock.elapsedRealtimeNanos()
        estimator.setPrivateProperty("timestamp", timestamp)
        listener.onLevelingAvailable(
            levelingEstimator,
            internalAttitude,
            timestamp,
            null,
            null,
            null
        )

        val levelingAttitude2: Quaternion? = estimator.getPrivateProperty("levelingAttitude")
        requireNotNull(levelingAttitude2)

        val angles = DoubleArray(3)
        internalAttitude.toEulerAngles(angles)
        val levelingRoll = angles[0]
        val levelingPitch = angles[1]
        attitude.toEulerAngles(angles)
        val relativeYaw = angles[2]

        verify(exactly = 1) { internalAttitude.copyTo(levelingAttitude2) }

        val eulerAngles: DoubleArray? = estimator.getPrivateProperty("eulerAngles")
        requireNotNull(eulerAngles)
        verify(exactly = 1) { levelingAttitudeSpy.toEulerAngles(eulerAngles) }
        verify(exactly = 1) { relativeAttitudeSpy.toEulerAngles(eulerAngles) }
        verify(exactly = 1) {
            levelingAttitudeSpy.setFromEulerAngles(
                levelingRoll,
                levelingPitch,
                relativeYaw
            )
        }
        verify(exactly = 0) { levelingAttitudeSpy.copyTo(internalFusedAttitudeSpy) }

        val panicCounter2: Int? = estimator.getPrivateProperty("panicCounter")
        requireNotNull(panicCounter2)
        assertEquals(1, panicCounter2)

        val hasRelativeAttitude2: Boolean? = estimator.getPrivateProperty("hasRelativeAttitude")
        requireNotNull(hasRelativeAttitude2)
        assertTrue(hasRelativeAttitude2)

        assertEquals(fusedAttitude3, internalFusedAttitudeSpy)

        val absDot = abs(QuaternionHelper.dotProduct(fusedAttitude3, levelingAttitudeSpy))
        assertTrue(absDot < estimator.outlierThreshold)
        assertTrue(absDot > estimator.outlierPanicThreshold)

        val previousRelativeAttitude2: Quaternion? =
            estimator.getPrivateProperty("previousRelativeAttitude")
        requireNotNull(previousRelativeAttitude2)
        assertEquals(relativeAttitudeSpy, previousRelativeAttitude2)

        val fusedAttitude: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude)

        verify(exactly = 1) {
            attitudeAvailableListener.onAttitudeAvailable(
                estimator,
                fusedAttitude,
                timestamp,
                null,
                null,
                null,
                null
            )
        }

        unmockkObject(QuaternionHelper)
    }

    @Test
    fun processLeveling_whenDeltaRelativeAttitudeLargeDivergenceAndDirectInterpolation_updatesFusedAttitudeAndNotifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(
            context,
            useAccurateLevelingEstimator = false,
            estimateCoordinateTransformation = false,
            estimateEulerAngles = false,
            attitudeAvailableListener = attitudeAvailableListener
        )
        estimator.useIndirectInterpolation = false

        estimator.setPrivateProperty("hasRelativeAttitude", true)
        estimator.setPrivateProperty("hasDeltaRelativeAttitude", true)

        val hasRelativeAttitude1: Boolean? = estimator.getPrivateProperty("hasRelativeAttitude")
        requireNotNull(hasRelativeAttitude1)
        assertTrue(hasRelativeAttitude1)

        val levelingAttitude1: Quaternion? = estimator.getPrivateProperty("levelingAttitude")
        requireNotNull(levelingAttitude1)
        assertEquals(Quaternion(), levelingAttitude1)

        val hasDeltaRelativeAttitude1: Boolean? =
            estimator.getPrivateProperty("hasDeltaRelativeAttitude")
        requireNotNull(hasDeltaRelativeAttitude1)
        assertTrue(hasDeltaRelativeAttitude1)

        val levelingAttitude: Quaternion? = estimator.getPrivateProperty("levelingAttitude")
        requireNotNull(levelingAttitude)
        val levelingAttitudeSpy = spyk(levelingAttitude)
        estimator.setPrivateProperty("levelingAttitude", levelingAttitudeSpy)

        val relativeAttitude: Quaternion? = estimator.getPrivateProperty("relativeAttitude")
        requireNotNull(relativeAttitude)
        val attitude = getAttitude()
        attitude.copyTo(relativeAttitude)
        val relativeAttitudeSpy = spyk(relativeAttitude)
        estimator.setPrivateProperty("relativeAttitude", relativeAttitudeSpy)

        val internalFusedAttitude: Quaternion? =
            estimator.getPrivateProperty("internalFusedAttitude")
        requireNotNull(internalFusedAttitude)
        getAttitude().copyTo(internalFusedAttitude)
        val internalFusedAttitudeSpy = spyk(internalFusedAttitude)
        estimator.setPrivateProperty("internalFusedAttitude", internalFusedAttitudeSpy)

        val previousRelativeAttitude1 = Quaternion()
        estimator.setPrivateProperty("previousRelativeAttitude", previousRelativeAttitude1)
        assertSame(
            previousRelativeAttitude1,
            estimator.getPrivateProperty("previousRelativeAttitude")
        )

        val deltaRelativeAttitude: Quaternion? =
            estimator.getPrivateProperty("deltaRelativeAttitude")
        requireNotNull(deltaRelativeAttitude)
        val randomizer = UniformRandomizer()
        val deltaRoll = Math.toRadians(randomizer.nextDouble(-DELTA_DEGREES, DELTA_DEGREES))
        val deltaPitch = Math.toRadians(randomizer.nextDouble(-DELTA_DEGREES, DELTA_DEGREES))
        val deltaYaw = Math.toRadians(randomizer.nextDouble(-DELTA_DEGREES, DELTA_DEGREES))
        deltaRelativeAttitude.setFromEulerAngles(deltaRoll, deltaPitch, deltaYaw)

        mockkObject(QuaternionHelper)
        every {
            QuaternionHelper.dotProduct(
                any(),
                any()
            )
        }.returns(0.0)

        val fusedAttitude2 = Quaternion(internalFusedAttitude)
        val fusedAttitude3 = deltaRelativeAttitude.multiplyAndReturnNew(fusedAttitude2)

        estimator.setPrivateProperty("panicCounter", 1)
        val panicCounter1: Int? = estimator.getPrivateProperty("panicCounter")
        requireNotNull(panicCounter1)
        assertEquals(1, panicCounter1)

        val resetToLeveling: Boolean? = estimator.getPrivateProperty("resetToLeveling")
        requireNotNull(resetToLeveling)
        assertFalse(resetToLeveling)

        val levelingEstimator: LevelingEstimator? =
            estimator.getPrivateProperty("levelingEstimator")
        requireNotNull(levelingEstimator)

        val internalAttitude = spyk(getAttitude())
        val listener = levelingEstimator.levelingAvailableListener
        requireNotNull(listener)
        val timestamp = SystemClock.elapsedRealtimeNanos()
        estimator.setPrivateProperty("timestamp", timestamp)
        listener.onLevelingAvailable(
            levelingEstimator,
            internalAttitude,
            timestamp,
            null,
            null,
            null
        )

        val levelingAttitude2: Quaternion? = estimator.getPrivateProperty("levelingAttitude")
        requireNotNull(levelingAttitude2)

        val angles = DoubleArray(3)
        internalAttitude.toEulerAngles(angles)
        val levelingRoll = angles[0]
        val levelingPitch = angles[1]
        attitude.toEulerAngles(angles)
        val relativeYaw = angles[2]

        verify(exactly = 1) { internalAttitude.copyTo(levelingAttitude2) }

        val eulerAngles: DoubleArray? = estimator.getPrivateProperty("eulerAngles")
        requireNotNull(eulerAngles)
        verify(exactly = 1) { levelingAttitudeSpy.toEulerAngles(eulerAngles) }
        verify(exactly = 1) { relativeAttitudeSpy.toEulerAngles(eulerAngles) }
        verify(exactly = 1) {
            levelingAttitudeSpy.setFromEulerAngles(
                levelingRoll,
                levelingPitch,
                relativeYaw
            )
        }
        verify(exactly = 0) { levelingAttitudeSpy.copyTo(internalFusedAttitudeSpy) }

        val panicCounter2: Int? = estimator.getPrivateProperty("panicCounter")
        requireNotNull(panicCounter2)
        assertEquals(2, panicCounter2)

        val hasRelativeAttitude2: Boolean? = estimator.getPrivateProperty("hasRelativeAttitude")
        requireNotNull(hasRelativeAttitude2)
        assertTrue(hasRelativeAttitude2)

        assertEquals(fusedAttitude3, internalFusedAttitudeSpy)

        val absDot = abs(QuaternionHelper.dotProduct(fusedAttitude3, levelingAttitudeSpy))
        assertTrue(absDot < estimator.outlierThreshold)
        assertTrue(absDot < estimator.outlierPanicThreshold)

        val previousRelativeAttitude2: Quaternion? =
            estimator.getPrivateProperty("previousRelativeAttitude")
        requireNotNull(previousRelativeAttitude2)
        assertEquals(relativeAttitudeSpy, previousRelativeAttitude2)

        val fusedAttitude: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude)

        verify(exactly = 1) {
            attitudeAvailableListener.onAttitudeAvailable(
                estimator,
                fusedAttitude,
                timestamp,
                null,
                null,
                null,
                null
            )
        }

        unmockkObject(QuaternionHelper)
    }

    @Test
    fun processLeveling_whenIndirectInterpolation_updatesFusedAttitudeAndNotifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(
            context,
            useAccurateRelativeGyroscopeAttitudeEstimator = false,
            useAccurateLevelingEstimator = false,
            estimateCoordinateTransformation = false,
            estimateEulerAngles = false,
            attitudeAvailableListener = attitudeAvailableListener
        )
        estimator.useIndirectInterpolation = true

        estimator.setPrivateProperty("hasRelativeAttitude", true)
        estimator.setPrivateProperty("hasDeltaRelativeAttitude", true)

        val hasRelativeAttitude1: Boolean? = estimator.getPrivateProperty("hasRelativeAttitude")
        requireNotNull(hasRelativeAttitude1)
        assertTrue(hasRelativeAttitude1)

        val levelingAttitude1: Quaternion? = estimator.getPrivateProperty("levelingAttitude")
        requireNotNull(levelingAttitude1)
        assertEquals(Quaternion(), levelingAttitude1)

        val hasDeltaRelativeAttitude1: Boolean? =
            estimator.getPrivateProperty("hasDeltaRelativeAttitude")
        requireNotNull(hasDeltaRelativeAttitude1)
        assertTrue(hasDeltaRelativeAttitude1)

        val levelingAttitude: Quaternion? = estimator.getPrivateProperty("levelingAttitude")
        requireNotNull(levelingAttitude)
        val levelingAttitudeSpy = spyk(levelingAttitude)
        estimator.setPrivateProperty("levelingAttitude", levelingAttitudeSpy)

        val relativeAttitude: Quaternion? = estimator.getPrivateProperty("relativeAttitude")
        requireNotNull(relativeAttitude)
        val attitude = getAttitude()
        attitude.copyTo(relativeAttitude)
        val relativeAttitudeSpy = spyk(relativeAttitude)
        estimator.setPrivateProperty("relativeAttitude", relativeAttitudeSpy)

        val internalFusedAttitude: Quaternion? =
            estimator.getPrivateProperty("internalFusedAttitude")
        requireNotNull(internalFusedAttitude)
        getAttitude().copyTo(internalFusedAttitude)
        val internalFusedAttitudeSpy = spyk(internalFusedAttitude)
        estimator.setPrivateProperty("internalFusedAttitude", internalFusedAttitudeSpy)

        val previousRelativeAttitude1 = Quaternion()
        estimator.setPrivateProperty("previousRelativeAttitude", previousRelativeAttitude1)
        assertSame(
            previousRelativeAttitude1,
            estimator.getPrivateProperty("previousRelativeAttitude")
        )

        val deltaRelativeAttitude: Quaternion? =
            estimator.getPrivateProperty("deltaRelativeAttitude")
        requireNotNull(deltaRelativeAttitude)
        val randomizer = UniformRandomizer()
        val deltaRoll = Math.toRadians(randomizer.nextDouble(-DELTA_DEGREES, DELTA_DEGREES))
        val deltaPitch = Math.toRadians(randomizer.nextDouble(-DELTA_DEGREES, DELTA_DEGREES))
        val deltaYaw = Math.toRadians(randomizer.nextDouble(-DELTA_DEGREES, DELTA_DEGREES))
        deltaRelativeAttitude.setFromEulerAngles(deltaRoll, deltaPitch, deltaYaw)

        mockkObject(QuaternionHelper)
        every {
            QuaternionHelper.dotProduct(
                any(),
                any()
            )
        }.returns(estimator.outlierThreshold)

        val fusedAttitude2 = Quaternion(internalFusedAttitude)
        val fusedAttitude3 = deltaRelativeAttitude.multiplyAndReturnNew(fusedAttitude2)

        estimator.setPrivateProperty("panicCounter", 1)
        val panicCounter1: Int? = estimator.getPrivateProperty("panicCounter")
        requireNotNull(panicCounter1)
        assertEquals(1, panicCounter1)

        val resetToLeveling: Boolean? = estimator.getPrivateProperty("resetToLeveling")
        requireNotNull(resetToLeveling)
        assertFalse(resetToLeveling)

        val attitudeEstimator: RelativeGyroscopeAttitudeEstimator? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(attitudeEstimator)
        val attitudeEstimatorSpy = spyk(attitudeEstimator)
        every { attitudeEstimatorSpy.averageTimeInterval }.returns(TIME_INTERVAL)
        estimator.setPrivateProperty("attitudeEstimator", attitudeEstimatorSpy)

        val levelingEstimator: LevelingEstimator? =
            estimator.getPrivateProperty("levelingEstimator")
        requireNotNull(levelingEstimator)

        val internalAttitude = spyk(getAttitude())
        val listener = levelingEstimator.levelingAvailableListener
        requireNotNull(listener)
        val timestamp = SystemClock.elapsedRealtimeNanos()
        estimator.setPrivateProperty("timestamp", timestamp)
        listener.onLevelingAvailable(
            levelingEstimator,
            internalAttitude,
            timestamp,
            null,
            null,
            null
        )

        val levelingAttitude2: Quaternion? = estimator.getPrivateProperty("levelingAttitude")
        requireNotNull(levelingAttitude2)

        val angles = DoubleArray(3)
        internalAttitude.toEulerAngles(angles)
        val levelingRoll = angles[0]
        val levelingPitch = angles[1]
        attitude.toEulerAngles(angles)
        val relativeYaw = angles[2]

        verify(exactly = 1) { internalAttitude.copyTo(levelingAttitude2) }

        val eulerAngles: DoubleArray? = estimator.getPrivateProperty("eulerAngles")
        requireNotNull(eulerAngles)
        verify(exactly = 1) { levelingAttitudeSpy.toEulerAngles(eulerAngles) }
        verify(exactly = 1) { relativeAttitudeSpy.toEulerAngles(eulerAngles) }
        verify(exactly = 1) {
            levelingAttitudeSpy.setFromEulerAngles(
                levelingRoll,
                levelingPitch,
                relativeYaw
            )
        }
        verify(exactly = 0) { levelingAttitudeSpy.copyTo(internalFusedAttitudeSpy) }

        val panicCounter2: Int? = estimator.getPrivateProperty("panicCounter")
        requireNotNull(panicCounter2)
        assertEquals(0, panicCounter2)

        val hasRelativeAttitude2: Boolean? = estimator.getPrivateProperty("hasRelativeAttitude")
        requireNotNull(hasRelativeAttitude2)
        assertTrue(hasRelativeAttitude2)

        val fusedAttitude4 = Quaternion()
        val t = estimator.interpolationValue + estimator.indirectInterpolationWeight *
                abs(deltaRelativeAttitude.rotationAngle / TIME_INTERVAL)
        Quaternion.slerp(
            fusedAttitude3,
            levelingAttitudeSpy,
            t,
            fusedAttitude4
        )

        assertEquals(fusedAttitude4, internalFusedAttitudeSpy)

        val absDot = abs(QuaternionHelper.dotProduct(fusedAttitude3, levelingAttitudeSpy))
        assertTrue(absDot >= estimator.outlierThreshold)

        val previousRelativeAttitude2: Quaternion? =
            estimator.getPrivateProperty("previousRelativeAttitude")
        requireNotNull(previousRelativeAttitude2)
        assertEquals(relativeAttitudeSpy, previousRelativeAttitude2)

        val fusedAttitude: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude)

        verify(exactly = 1) {
            attitudeAvailableListener.onAttitudeAvailable(
                estimator,
                fusedAttitude,
                timestamp,
                null,
                null,
                null,
                null
            )
        }

        unmockkObject(QuaternionHelper)
    }

    @Test
    fun processLeveling_whenEstimateEulerAnglesAndCoordinateTransformationEnabled_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = LeveledRelativeAttitudeEstimator(
            context,
            useAccurateLevelingEstimator = false,
            estimateCoordinateTransformation = true,
            estimateEulerAngles = true,
            attitudeAvailableListener = attitudeAvailableListener
        )
        estimator.useIndirectInterpolation = false

        estimator.setPrivateProperty("hasRelativeAttitude", true)
        estimator.setPrivateProperty("hasDeltaRelativeAttitude", true)

        val hasRelativeAttitude1: Boolean? = estimator.getPrivateProperty("hasRelativeAttitude")
        requireNotNull(hasRelativeAttitude1)
        assertTrue(hasRelativeAttitude1)

        val levelingAttitude1: Quaternion? = estimator.getPrivateProperty("levelingAttitude")
        requireNotNull(levelingAttitude1)
        assertEquals(Quaternion(), levelingAttitude1)

        val hasDeltaRelativeAttitude1: Boolean? =
            estimator.getPrivateProperty("hasDeltaRelativeAttitude")
        requireNotNull(hasDeltaRelativeAttitude1)
        assertTrue(hasDeltaRelativeAttitude1)

        val levelingAttitude: Quaternion? = estimator.getPrivateProperty("levelingAttitude")
        requireNotNull(levelingAttitude)
        val levelingAttitudeSpy = spyk(levelingAttitude)
        estimator.setPrivateProperty("levelingAttitude", levelingAttitudeSpy)

        val relativeAttitude: Quaternion? = estimator.getPrivateProperty("relativeAttitude")
        requireNotNull(relativeAttitude)
        val attitude = getAttitude()
        attitude.copyTo(relativeAttitude)
        val relativeAttitudeSpy = spyk(relativeAttitude)
        estimator.setPrivateProperty("relativeAttitude", relativeAttitudeSpy)

        val internalFusedAttitude: Quaternion? =
            estimator.getPrivateProperty("internalFusedAttitude")
        requireNotNull(internalFusedAttitude)
        getAttitude().copyTo(internalFusedAttitude)
        val internalFusedAttitudeSpy = spyk(internalFusedAttitude)
        estimator.setPrivateProperty("internalFusedAttitude", internalFusedAttitudeSpy)

        val previousRelativeAttitude1 = Quaternion()
        estimator.setPrivateProperty("previousRelativeAttitude", previousRelativeAttitude1)
        assertSame(
            previousRelativeAttitude1,
            estimator.getPrivateProperty("previousRelativeAttitude")
        )

        val deltaRelativeAttitude: Quaternion? =
            estimator.getPrivateProperty("deltaRelativeAttitude")
        requireNotNull(deltaRelativeAttitude)
        val randomizer = UniformRandomizer()
        val deltaRoll = Math.toRadians(randomizer.nextDouble(-DELTA_DEGREES, DELTA_DEGREES))
        val deltaPitch = Math.toRadians(randomizer.nextDouble(-DELTA_DEGREES, DELTA_DEGREES))
        val deltaYaw = Math.toRadians(randomizer.nextDouble(-DELTA_DEGREES, DELTA_DEGREES))
        deltaRelativeAttitude.setFromEulerAngles(deltaRoll, deltaPitch, deltaYaw)

        mockkObject(QuaternionHelper)
        every {
            QuaternionHelper.dotProduct(
                any(),
                any()
            )
        }.returns(estimator.outlierThreshold)

        val fusedAttitude2 = Quaternion(internalFusedAttitude)
        val fusedAttitude3 = deltaRelativeAttitude.multiplyAndReturnNew(fusedAttitude2)

        estimator.setPrivateProperty("panicCounter", 1)
        val panicCounter1: Int? = estimator.getPrivateProperty("panicCounter")
        requireNotNull(panicCounter1)
        assertEquals(1, panicCounter1)

        val resetToLeveling: Boolean? = estimator.getPrivateProperty("resetToLeveling")
        requireNotNull(resetToLeveling)
        assertFalse(resetToLeveling)

        val levelingEstimator: LevelingEstimator? =
            estimator.getPrivateProperty("levelingEstimator")
        requireNotNull(levelingEstimator)

        val internalAttitude = spyk(getAttitude())
        val listener = levelingEstimator.levelingAvailableListener
        requireNotNull(listener)
        val timestamp = SystemClock.elapsedRealtimeNanos()
        estimator.setPrivateProperty("timestamp", timestamp)
        listener.onLevelingAvailable(
            levelingEstimator,
            internalAttitude,
            timestamp,
            null,
            null,
            null
        )

        val levelingAttitude2: Quaternion? = estimator.getPrivateProperty("levelingAttitude")
        requireNotNull(levelingAttitude2)

        val angles = DoubleArray(3)
        internalAttitude.toEulerAngles(angles)
        val levelingRoll = angles[0]
        val levelingPitch = angles[1]
        attitude.toEulerAngles(angles)
        val relativeYaw = angles[2]

        verify(exactly = 1) { internalAttitude.copyTo(levelingAttitude2) }

        val eulerAngles: DoubleArray? = estimator.getPrivateProperty("eulerAngles")
        requireNotNull(eulerAngles)
        verify(exactly = 1) { levelingAttitudeSpy.toEulerAngles(eulerAngles) }
        verify(exactly = 1) { relativeAttitudeSpy.toEulerAngles(eulerAngles) }
        verify(exactly = 1) {
            levelingAttitudeSpy.setFromEulerAngles(
                levelingRoll,
                levelingPitch,
                relativeYaw
            )
        }
        verify(exactly = 0) { levelingAttitudeSpy.copyTo(internalFusedAttitudeSpy) }

        val panicCounter2: Int? = estimator.getPrivateProperty("panicCounter")
        requireNotNull(panicCounter2)
        assertEquals(0, panicCounter2)

        val hasRelativeAttitude2: Boolean? = estimator.getPrivateProperty("hasRelativeAttitude")
        requireNotNull(hasRelativeAttitude2)
        assertTrue(hasRelativeAttitude2)

        val fusedAttitude4 = Quaternion()
        Quaternion.slerp(
            fusedAttitude3,
            levelingAttitudeSpy,
            estimator.interpolationValue,
            fusedAttitude4
        )

        assertEquals(fusedAttitude4, internalFusedAttitudeSpy)

        val absDot = abs(QuaternionHelper.dotProduct(fusedAttitude3, levelingAttitudeSpy))
        assertTrue(absDot >= estimator.outlierThreshold)

        val previousRelativeAttitude2: Quaternion? =
            estimator.getPrivateProperty("previousRelativeAttitude")
        requireNotNull(previousRelativeAttitude2)
        assertEquals(relativeAttitudeSpy, previousRelativeAttitude2)

        val rollSlot = slot<Double>()
        val pitchSlot = slot<Double>()
        val yawSlot = slot<Double>()
        val coordinateTransformationSlot = slot<CoordinateTransformation>()

        val fusedAttitude: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude)

        verify(exactly = 1) {
            attitudeAvailableListener.onAttitudeAvailable(
                estimator,
                fusedAttitude,
                timestamp,
                capture(rollSlot),
                capture(pitchSlot),
                capture(yawSlot),
                capture(coordinateTransformationSlot)
            )
        }

        assertTrue(rollSlot.isCaptured)
        assertFalse(rollSlot.isNull)
        assertNotNull(rollSlot.captured)

        assertTrue(pitchSlot.isCaptured)
        assertFalse(pitchSlot.isNull)
        assertNotNull(pitchSlot.captured)

        assertTrue(yawSlot.isCaptured)
        assertFalse(yawSlot.isNull)
        assertNotNull(yawSlot.captured)

        assertTrue(coordinateTransformationSlot.isCaptured)
        assertFalse(coordinateTransformationSlot.isNull)
        val c = coordinateTransformationSlot.captured
        assertNotNull(c)
        assertEquals(FrameType.BODY_FRAME, c.sourceType)
        assertEquals(FrameType.LOCAL_NAVIGATION_FRAME, c.destinationType)

        unmockkObject(QuaternionHelper)
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

        const val DELTA_DEGREES = 1e-6

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