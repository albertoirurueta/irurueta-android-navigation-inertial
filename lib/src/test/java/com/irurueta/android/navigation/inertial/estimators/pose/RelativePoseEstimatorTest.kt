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
package com.irurueta.android.navigation.inertial.estimators.pose

import android.content.Context
import android.location.Location
import android.os.SystemClock
import androidx.test.core.app.ApplicationProvider
import com.irurueta.algebra.Matrix
import com.irurueta.android.navigation.inertial.*
import com.irurueta.android.navigation.inertial.collectors.AccelerometerSensorCollector
import com.irurueta.android.navigation.inertial.collectors.GyroscopeSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.estimators.attitude.GravityEstimator
import com.irurueta.android.navigation.inertial.estimators.attitude.LeveledRelativeAttitudeEstimator
import com.irurueta.android.navigation.inertial.estimators.filter.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.MeanAveragingFilter
import com.irurueta.geometry.EuclideanTransformation3D
import com.irurueta.geometry.InhomogeneousPoint3D
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.statistics.UniformRandomizer
import io.mockk.*
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class RelativePoseEstimatorTest {

    @Test
    fun constructor_whenRequiredProperties_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator(context)

        // check
        assertSame(context, estimator.context)
        assertEquals(SensorDelay.GAME, estimator.sensorDelay)
        assertFalse(estimator.useAccelerometerForAttitudeEstimation)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            estimator.accelerometerSensorType
        )
        assertEquals(
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            estimator.gyroscopeSensorType
        )
        assertNotNull(estimator.accelerometerAveragingFilter)
        assertTrue(estimator.accelerometerAveragingFilter is LowPassAveragingFilter)
        assertFalse(estimator.useAccurateLevelingEstimator)
        assertTrue(estimator.useAccurateRelativeGyroscopeAttitudeEstimator)
        assertTrue(estimator.useIndirectAttitudeInterpolation)
        assertEquals(
            LeveledRelativeAttitudeEstimator.DEFAULT_INTERPOLATION_VALUE,
            estimator.attitudeInterpolationValue,
            0.0
        )
        assertEquals(
            LeveledRelativeAttitudeEstimator.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            estimator.attitudeIndirectInterpolationWeight,
            0.0
        )
        assertEquals(
            LeveledRelativeAttitudeEstimator.DEFAULT_OUTLIER_THRESHOLD,
            estimator.attitudeOutlierThreshold,
            0.0
        )
        assertEquals(
            LeveledRelativeAttitudeEstimator.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            estimator.attitudeOutlierPanicThreshold,
            0.0
        )
        assertEquals(
            LeveledRelativeAttitudeEstimator.DEFAULT_PANIC_COUNTER_THRESHOLD,
            estimator.attitudePanicCounterThreshold
        )
        assertNull(estimator.poseAvailableListener)
        assertNull(estimator.accelerometerMeasurementListener)
        assertNull(estimator.gyroscopeMeasurementListener)
        assertNull(estimator.gravityEstimationListener)
        assertEquals(SpeedTriad(), estimator.initialSpeed)
        assertNull(estimator.initialLocation)
        assertFalse(estimator.running)
        assertEquals(0.0, estimator.averageTimeInterval, 0.0)
    }

    @Test
    fun constructor_whenAllProperties_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val initialSpeed = getSpeed()
        val accelerometerAveragingFilter = MeanAveragingFilter()
        val poseAvailableListener = mockk<RelativePoseEstimator.OnPoseAvailableListener>()
        val accelerometerMeasurementListener =
            mockk<AccelerometerSensorCollector.OnMeasurementListener>()
        val gyroscopeMeasurementListener = mockk<GyroscopeSensorCollector.OnMeasurementListener>()
        val gravityEstimationListener = mockk<GravityEstimator.OnEstimationListener>()
        val estimator = RelativePoseEstimator(
            context,
            SensorDelay.NORMAL,
            useAccelerometerForAttitudeEstimation = true,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER,
            GyroscopeSensorCollector.SensorType.GYROSCOPE,
            accelerometerAveragingFilter,
            useAccurateLevelingEstimator = true,
            useAccurateRelativeGyroscopeAttitudeEstimator = false,
            poseAvailableListener,
            accelerometerMeasurementListener,
            gyroscopeMeasurementListener,
            gravityEstimationListener,
            initialSpeed,
            location
        )

        // check
        assertSame(context, estimator.context)
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertTrue(estimator.useAccelerometerForAttitudeEstimation)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER,
            estimator.accelerometerSensorType
        )
        assertEquals(
            GyroscopeSensorCollector.SensorType.GYROSCOPE,
            estimator.gyroscopeSensorType
        )
        assertSame(accelerometerAveragingFilter, estimator.accelerometerAveragingFilter)
        assertTrue(estimator.useAccurateLevelingEstimator)
        assertFalse(estimator.useAccurateRelativeGyroscopeAttitudeEstimator)
        assertTrue(estimator.useIndirectAttitudeInterpolation)
        assertEquals(
            LeveledRelativeAttitudeEstimator.DEFAULT_INTERPOLATION_VALUE,
            estimator.attitudeInterpolationValue,
            0.0
        )
        assertEquals(
            LeveledRelativeAttitudeEstimator.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            estimator.attitudeIndirectInterpolationWeight,
            0.0
        )
        assertEquals(
            LeveledRelativeAttitudeEstimator.DEFAULT_OUTLIER_THRESHOLD,
            estimator.attitudeOutlierThreshold,
            0.0
        )
        assertEquals(
            LeveledRelativeAttitudeEstimator.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            estimator.attitudeOutlierPanicThreshold,
            0.0
        )
        assertEquals(
            LeveledRelativeAttitudeEstimator.DEFAULT_PANIC_COUNTER_THRESHOLD,
            estimator.attitudePanicCounterThreshold
        )
        assertSame(poseAvailableListener, estimator.poseAvailableListener)
        assertSame(accelerometerMeasurementListener, estimator.accelerometerMeasurementListener)
        assertSame(gyroscopeMeasurementListener, estimator.gyroscopeMeasurementListener)
        assertSame(gravityEstimationListener, estimator.gravityEstimationListener)
        assertEquals(initialSpeed, estimator.initialSpeed)
        assertSame(location, estimator.initialLocation)
        assertFalse(estimator.running)
        assertEquals(0.0, estimator.averageTimeInterval, 0.0)
    }

    @Test
    fun poseAvailableListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator(context)

        // check default value
        assertNull(estimator.poseAvailableListener)

        // set new value
        val poseAvailableListener = mockk<RelativePoseEstimator.OnPoseAvailableListener>()
        estimator.poseAvailableListener = poseAvailableListener

        // check
        assertSame(poseAvailableListener, estimator.poseAvailableListener)
    }

    @Test
    fun accelerometerMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator(context)

        // check default value
        assertNull(estimator.accelerometerMeasurementListener)

        // set new value
        val accelerometerMeasurementListener =
            mockk<AccelerometerSensorCollector.OnMeasurementListener>()
        estimator.accelerometerMeasurementListener = accelerometerMeasurementListener

        // check
        assertSame(accelerometerMeasurementListener, estimator.accelerometerMeasurementListener)
    }

    @Test
    fun gyroscopeMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator(context)

        // check default value
        assertNull(estimator.gyroscopeMeasurementListener)

        // set new value
        val gyroscopeMeasurementListener = mockk<GyroscopeSensorCollector.OnMeasurementListener>()
        estimator.gyroscopeMeasurementListener = gyroscopeMeasurementListener

        // check
        assertSame(gyroscopeMeasurementListener, estimator.gyroscopeMeasurementListener)
    }

    @Test
    fun gravityEstimationListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator(context)

        // check default value
        assertNull(estimator.gravityEstimationListener)

        // set new value
        val gravityEstimationListener = mockk<GravityEstimator.OnEstimationListener>()
        estimator.gravityEstimationListener = gravityEstimationListener

        // check
        assertSame(gravityEstimationListener, estimator.gravityEstimationListener)
    }

    @Test
    fun initialSpeed_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator(context)

        // check default value
        assertEquals(SpeedTriad(), estimator.initialSpeed)

        // set new value
        val initialSpeed = getSpeed()
        estimator.initialSpeed = initialSpeed

        // check
        assertSame(initialSpeed, estimator.initialSpeed)
    }

    @Test(expected = IllegalStateException::class)
    fun initialSpeed_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator(context)

        // set as running
        estimator.setPrivateProperty("running", true)

        val initialSpeed = getSpeed()
        estimator.initialSpeed = initialSpeed
    }

    @Test
    fun initialLocation_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator(context)

        // check default value
        assertNull(estimator.initialLocation)

        // set new value
        val initialLocation = getLocation()
        estimator.initialLocation = initialLocation

        // check
        assertSame(initialLocation, estimator.initialLocation)
    }

    @Test(expected = IllegalStateException::class)
    fun initialLocation_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator(context)

        // set as running
        estimator.setPrivateProperty("running", true)

        val initialLocation = getLocation()
        estimator.initialLocation = initialLocation
    }

    @Test
    fun useAccurateLevelingEstimator_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = RelativePoseEstimator(context, initialLocation = location)

        val attitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(attitudeEstimator)

        // check default value
        assertFalse(estimator.useAccurateLevelingEstimator)
        assertFalse(attitudeEstimator.useAccurateLevelingEstimator)

        // set new value
        estimator.useAccurateLevelingEstimator = true

        // check
        assertTrue(estimator.useAccurateLevelingEstimator)
        assertTrue(attitudeEstimator.useAccurateLevelingEstimator)
    }

    @Test(expected = IllegalStateException::class)
    fun useAccurateLevelingEstimator_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = RelativePoseEstimator(context, initialLocation = location)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        estimator.useAccurateLevelingEstimator = true
    }

    @Test(expected = IllegalStateException::class)
    fun useAccurateLevelingEstimator_whenNoLocationAndTrue_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator(context)

        estimator.useAccurateLevelingEstimator = true
    }

    @Test
    fun useAccurateLevelingEstimator_whenNoLocationAndFalse_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator(context)

        estimator.useAccurateLevelingEstimator = false

        // check
        assertFalse(estimator.useAccurateLevelingEstimator)
    }

    @Test
    fun useAccurateRelativeGyroscopeAttitudeEstimator_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator(context)

        val attitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(attitudeEstimator)

        // check default value
        assertTrue(estimator.useAccurateRelativeGyroscopeAttitudeEstimator)
        assertTrue(attitudeEstimator.useAccurateRelativeGyroscopeAttitudeEstimator)
        assertFalse(estimator.running)

        // set new value
        estimator.useAccurateRelativeGyroscopeAttitudeEstimator = false

        // check
        assertFalse(estimator.useAccurateRelativeGyroscopeAttitudeEstimator)
        assertFalse(attitudeEstimator.useAccurateRelativeGyroscopeAttitudeEstimator)
    }

    @Test(expected = IllegalStateException::class)
    fun useAccurateRelativeGyroscopeAttitudeEstimator_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator(context)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        // set new value
        estimator.useAccurateRelativeGyroscopeAttitudeEstimator = true
    }

    @Test
    fun useIndirectAttitudeInterpolation_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator(context)

        // check default value
        assertTrue(estimator.useIndirectAttitudeInterpolation)

        val attitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(attitudeEstimator)
        assertTrue(attitudeEstimator.useIndirectInterpolation)

        // set new value
        estimator.useIndirectAttitudeInterpolation = false

        // check
        assertFalse(estimator.useIndirectAttitudeInterpolation)
        assertFalse(attitudeEstimator.useIndirectInterpolation)
    }

    @Test
    fun attitudeInterpolationValue_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator(context)

        // check default value
        assertEquals(
            LeveledRelativeAttitudeEstimator.DEFAULT_INTERPOLATION_VALUE,
            estimator.attitudeInterpolationValue,
            0.0
        )

        val attitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(attitudeEstimator)
        assertEquals(
            LeveledRelativeAttitudeEstimator.DEFAULT_INTERPOLATION_VALUE,
            attitudeEstimator.interpolationValue,
            0.0
        )

        // set new value
        val randomizer = UniformRandomizer()
        val attitudeInterpolationValue = randomizer.nextDouble()
        estimator.attitudeInterpolationValue = attitudeInterpolationValue

        // check
        assertEquals(attitudeInterpolationValue, estimator.attitudeInterpolationValue, 0.0)
        assertEquals(attitudeInterpolationValue, attitudeEstimator.interpolationValue, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun attitudeInterpolationValue_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator(context)

        estimator.attitudeInterpolationValue = -1.0
    }

    @Test
    fun attitudeIndirectInterpolationWeight_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator(context)

        // check default value
        assertEquals(
            LeveledRelativeAttitudeEstimator.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            estimator.attitudeIndirectInterpolationWeight,
            0.0
        )

        val attitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(attitudeEstimator)
        assertEquals(
            LeveledRelativeAttitudeEstimator.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            attitudeEstimator.indirectInterpolationWeight,
            0.0
        )

        // set new value
        val randomizer = UniformRandomizer()
        val attitudeIndirectInterpolationWeight = randomizer.nextDouble()
        estimator.attitudeIndirectInterpolationWeight = attitudeIndirectInterpolationWeight

        // check
        assertEquals(
            attitudeIndirectInterpolationWeight,
            estimator.attitudeIndirectInterpolationWeight,
            0.0
        )
        assertEquals(
            attitudeIndirectInterpolationWeight,
            attitudeEstimator.indirectInterpolationWeight,
            0.0
        )
    }

    @Test(expected = IllegalArgumentException::class)
    fun attitudeIndirectInterpolationWeight_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator(context)

        estimator.attitudeIndirectInterpolationWeight = 0.0
    }

    @Test
    fun attitudeOutlierThreshold_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator(context)

        // check default value
        assertEquals(
            LeveledRelativeAttitudeEstimator.DEFAULT_OUTLIER_THRESHOLD,
            estimator.attitudeOutlierThreshold,
            0.0
        )

        val attitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(attitudeEstimator)
        assertEquals(
            LeveledRelativeAttitudeEstimator.DEFAULT_OUTLIER_THRESHOLD,
            attitudeEstimator.outlierThreshold,
            0.0
        )

        // set new value
        val randomizer = UniformRandomizer()
        val attitudeOutlierThreshold = randomizer.nextDouble()
        estimator.attitudeOutlierThreshold = attitudeOutlierThreshold

        // check
        assertEquals(attitudeOutlierThreshold, estimator.attitudeOutlierThreshold, 0.0)
        assertEquals(attitudeOutlierThreshold, attitudeEstimator.outlierThreshold, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun attitudeOutlierThreshold_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator(context)

        estimator.attitudeOutlierThreshold = -1.0
    }

    @Test(expected = IllegalStateException::class)
    fun attitudeOutlierThreshold_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator(context)

        // set as running
        val attitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(attitudeEstimator)
        attitudeEstimator.setPrivateProperty("running", true)

        // attempt setting new value
        val randomizer = UniformRandomizer()
        val attitudeOutlierThreshold = randomizer.nextDouble()
        estimator.attitudeOutlierThreshold = attitudeOutlierThreshold
    }

    @Test
    fun attitudeOutlierPanicThreshold_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator(context)

        // check default value
        assertEquals(
            LeveledRelativeAttitudeEstimator.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            estimator.attitudeOutlierPanicThreshold,
            0.0
        )

        val attitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(attitudeEstimator)
        assertEquals(
            LeveledRelativeAttitudeEstimator.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            attitudeEstimator.outlierPanicThreshold,
            0.0
        )

        // set new value
        val randomizer = UniformRandomizer()
        val attitudeOutlierPanicThreshold = randomizer.nextDouble()
        estimator.attitudeOutlierPanicThreshold = attitudeOutlierPanicThreshold

        // check
        assertEquals(attitudeOutlierPanicThreshold, estimator.attitudeOutlierPanicThreshold, 0.0)
        assertEquals(attitudeOutlierPanicThreshold, attitudeEstimator.outlierPanicThreshold, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun attitudeOutlierPanicThreshold_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator(context)

        estimator.attitudeOutlierPanicThreshold = -1.0
    }

    @Test(expected = IllegalStateException::class)
    fun attitudeOutlierPanicThreshold_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator(context)

        // set as running
        val attitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(attitudeEstimator)
        attitudeEstimator.setPrivateProperty("running", true)

        // attempt setting new value
        val randomizer = UniformRandomizer()
        val attitudeOutlierPanicThreshold = randomizer.nextDouble()
        estimator.attitudeOutlierPanicThreshold = attitudeOutlierPanicThreshold
    }

    @Test
    fun attitudePanicCounterThreshold_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator(context)

        // check default value
        assertEquals(
            LeveledRelativeAttitudeEstimator.DEFAULT_PANIC_COUNTER_THRESHOLD,
            estimator.attitudePanicCounterThreshold
        )
        val attitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(attitudeEstimator)
        assertEquals(
            LeveledRelativeAttitudeEstimator.DEFAULT_PANIC_COUNTER_THRESHOLD,
            attitudeEstimator.panicCounterThreshold
        )

        // set new value
        estimator.attitudePanicCounterThreshold = 1

        // check
        assertEquals(1, estimator.attitudePanicCounterThreshold)
        assertEquals(1, attitudeEstimator.panicCounterThreshold)
    }

    @Test
    fun averageTimeInterval_callsInternatAttitudeEstimator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator(context)

        val attitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(attitudeEstimator)
        val attitudeEstimatorSpy = spyk(attitudeEstimator)
        every { attitudeEstimatorSpy.gyroscopeAverageTimeInterval }.returns(TIME_INTERVAL)
        estimator.setPrivateProperty("attitudeEstimator", attitudeEstimatorSpy)

        assertEquals(TIME_INTERVAL, estimator.averageTimeInterval, 0.0)
        verify(exactly = 1) { attitudeEstimatorSpy.gyroscopeAverageTimeInterval }
    }

    @Test
    fun start_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator(context)

        val attitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(attitudeEstimator)
        val attitudeEstimatorSpy = spyk(attitudeEstimator)
        estimator.setPrivateProperty("attitudeEstimator", attitudeEstimatorSpy)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        assertThrows(IllegalStateException::class.java) { estimator.start() }
        verify { attitudeEstimatorSpy wasNot Called }
    }

    @Test
    fun start_whenNotRunningAndInternalEstimatorFails_stopsAndReturnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator(context)

        val attitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(attitudeEstimator)
        val attitudeEstimatorSpy = spyk(attitudeEstimator)
        every { attitudeEstimatorSpy.start() }.returns(false)
        estimator.setPrivateProperty("attitudeEstimator", attitudeEstimatorSpy)

        assertFalse(estimator.running)

        // start
        assertFalse(estimator.start())
        assertFalse(estimator.running)

        verify(exactly = 1) { attitudeEstimatorSpy.start() }
        verify(exactly = 1) { attitudeEstimatorSpy.stop() }
    }

    @Test
    fun start_whenAccelerometerSensorCollectorFails_stopsAndReturnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator(context)

        val attitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(attitudeEstimator)
        val attitudeEstimatorSpy = spyk(attitudeEstimator)
        every { attitudeEstimatorSpy.start() }.returns(true)
        estimator.setPrivateProperty("attitudeEstimator", attitudeEstimatorSpy)

        val accelerometerSensorCollector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.start() }.returns(false)
        estimator.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        assertFalse(estimator.running)
        assertFalse(estimator.useAccelerometerForAttitudeEstimation)

        // start
        assertFalse(estimator.start())
        assertFalse(estimator.running)

        verify(exactly = 1) { attitudeEstimatorSpy.start() }
        verify(exactly = 1) { attitudeEstimatorSpy.stop() }

        verify(exactly = 1) { accelerometerSensorCollectorSpy.start() }
        verify(exactly = 1) { accelerometerSensorCollectorSpy.stop() }
    }

    @Test
    fun start_whenUseAccelerometerForAttitudeEstimationDisabledNotRunningAndInternalEstimatorSucceeds_returnsTrue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator(context)

        val attitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(attitudeEstimator)
        val attitudeEstimatorSpy = spyk(attitudeEstimator)
        every { attitudeEstimatorSpy.start() }.returns(true)
        estimator.setPrivateProperty("attitudeEstimator", attitudeEstimatorSpy)

        val accelerometerSensorCollector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.start() }.returns(true)
        estimator.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        assertFalse(estimator.running)
        assertFalse(estimator.useAccelerometerForAttitudeEstimation)

        // start
        assertTrue(estimator.start())
        assertTrue(estimator.running)

        verify(exactly = 1) { attitudeEstimatorSpy.start() }
        verify(exactly = 0) { attitudeEstimatorSpy.stop() }

        verify(exactly = 1) { accelerometerSensorCollectorSpy.start() }
    }

    @Test
    fun start_whenUseAccelerometerForAttitudeEstimationEnabledNotRunningAndInternalEstimatorSucceeds_returnsTrue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator(context, useAccelerometerForAttitudeEstimation = true)

        val attitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(attitudeEstimator)
        val attitudeEstimatorSpy = spyk(attitudeEstimator)
        every { attitudeEstimatorSpy.start() }.returns(true)
        estimator.setPrivateProperty("attitudeEstimator", attitudeEstimatorSpy)

        val accelerometerSensorCollector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.start() }.returns(true)
        estimator.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        assertFalse(estimator.running)
        assertTrue(estimator.useAccelerometerForAttitudeEstimation)

        // start
        assertTrue(estimator.start())
        assertTrue(estimator.running)

        verify(exactly = 1) { attitudeEstimatorSpy.start() }
        verify(exactly = 0) { attitudeEstimatorSpy.stop() }

        verify(exactly = 0) { accelerometerSensorCollectorSpy.start() }
    }

    @Test
    fun start_whenNotRunning_resetsInitialized() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator(context)

        // set as initialized
        estimator.setPrivateProperty("initialized", true)
        val initialized1: Boolean? = estimator.getPrivateProperty("initialized")
        requireNotNull(initialized1)
        assertTrue(initialized1)

        val attitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(attitudeEstimator)
        val attitudeEstimatorSpy = spyk(attitudeEstimator)
        every { attitudeEstimatorSpy.start() }.returns(false)
        estimator.setPrivateProperty("attitudeEstimator", attitudeEstimatorSpy)

        assertFalse(estimator.running)

        // start
        assertFalse(estimator.start())
        assertFalse(estimator.running)

        verify(exactly = 1) { attitudeEstimatorSpy.start() }
        verify(exactly = 1) { attitudeEstimatorSpy.stop() }

        val initialized2: Boolean? = estimator.getPrivateProperty("initialized")
        requireNotNull(initialized2)
        assertFalse(initialized2)
    }

    @Test
    fun stop_callsInternalEstimatorAndSetsAsNotRunning() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator(context)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        val attitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(attitudeEstimator)
        val attitudeEstimatorSpy = spyk(attitudeEstimator)
        estimator.setPrivateProperty("attitudeEstimator", attitudeEstimatorSpy)

        estimator.stop()

        // check
        assertFalse(estimator.running)
        verify(exactly = 1) { attitudeEstimatorSpy.stop() }
    }

    @Test
    fun initialize_whenNotInitialized_initialized() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialSpeed = getSpeed()
        val estimator = RelativePoseEstimator(context, initialSpeed = initialSpeed)

        val initialized1: Boolean? = estimator.getPrivateProperty("initialized")
        requireNotNull(initialized1)
        assertFalse(initialized1)

        val initialAttitude: Quaternion? = estimator.getPrivateProperty("initialAttitude")
        requireNotNull(initialAttitude)
        assertEquals(Quaternion(), initialAttitude)

        val previousAttitude: Quaternion? = estimator.getPrivateProperty("previousAttitude")
        requireNotNull(previousAttitude)
        assertEquals(Quaternion(), previousAttitude)

        val previousSpeed: SpeedTriad? = estimator.getPrivateProperty("previousSpeed")
        requireNotNull(previousSpeed)
        assertEquals(SpeedTriad(), previousSpeed)

        val previousPosition: InhomogeneousPoint3D? =
            estimator.getPrivateProperty("previousPosition")
        requireNotNull(previousPosition)
        assertEquals(InhomogeneousPoint3D(), previousPosition)

        val q = getAttitude()

        val result: Boolean? = estimator.callPrivateFuncWithResult("initialize", q)
        requireNotNull(result)
        assertFalse(result)

        // check
        assertEquals(q, initialAttitude)
        assertEquals(q, previousAttitude)
        assertEquals(initialSpeed, previousSpeed)
        assertEquals(InhomogeneousPoint3D(), previousPosition)

        val initialized2: Boolean? = estimator.getPrivateProperty("initialized")
        requireNotNull(initialized2)
        assertTrue(initialized2)
    }

    @Test
    fun initialize_whenAlreadyInitialized_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialSpeed = getSpeed()
        val estimator = RelativePoseEstimator(context, initialSpeed = initialSpeed)

        estimator.setPrivateProperty("initialized", true)
        val initialized1: Boolean? = estimator.getPrivateProperty("initialized")
        requireNotNull(initialized1)
        assertTrue(initialized1)

        val initialAttitude: Quaternion? = estimator.getPrivateProperty("initialAttitude")
        requireNotNull(initialAttitude)
        assertEquals(Quaternion(), initialAttitude)

        val previousAttitude: Quaternion? = estimator.getPrivateProperty("previousAttitude")
        requireNotNull(previousAttitude)
        assertEquals(Quaternion(), previousAttitude)

        val previousSpeed: SpeedTriad? = estimator.getPrivateProperty("previousSpeed")
        requireNotNull(previousSpeed)
        assertEquals(SpeedTriad(), previousSpeed)

        val previousPosition: InhomogeneousPoint3D? =
            estimator.getPrivateProperty("previousPosition")
        requireNotNull(previousPosition)
        val previousPositionSpy = spyk(previousPosition)
        estimator.setPrivateProperty("previousPosition", previousPositionSpy)
        assertEquals(InhomogeneousPoint3D(), previousPosition)

        val q = getAttitude()

        val result: Boolean? = estimator.callPrivateFuncWithResult("initialize", q)
        requireNotNull(result)
        assertTrue(result)

        // check
        verify { previousPositionSpy wasNot Called }

        assertEquals(Quaternion(), initialAttitude)
        assertEquals(Quaternion(), previousAttitude)
        assertEquals(SpeedTriad(), previousSpeed)
        assertEquals(InhomogeneousPoint3D(), previousPositionSpy)
    }

    @Test
    fun computeTransformation_computesExpectedTransformation() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialSpeed = getSpeed()
        val estimator = RelativePoseEstimator(context, initialSpeed = initialSpeed)

        val currentAttitude = getAttitude()

        val randomizer = UniformRandomizer()
        val currentPosition = InhomogeneousPoint3D(
            randomizer.nextDouble(MIN_DELTA_POS, MAX_DELTA_POS),
            randomizer.nextDouble(MIN_DELTA_POS, MAX_DELTA_POS),
            randomizer.nextDouble(MIN_DELTA_POS, MAX_DELTA_POS)
        )

        estimator.setPrivateProperty("currentAttitude", currentAttitude)
        estimator.setPrivateProperty("currentPosition", currentPosition)

        callPrivateFunc(RelativePoseEstimator::class, estimator, "computeTransformation")

        // check
        val transformationRotation1: Quaternion? =
            estimator.getPrivateProperty("transformationRotation")
        requireNotNull(transformationRotation1)
        val transformationRotation2 = Quaternion()
        Quaternion.product(
            currentAttitude,
            ENUtoNEDTriadConverter.conversionRotation,
            transformationRotation2
        )

        val transformationPosition: InhomogeneousPoint3D? =
            estimator.getPrivateProperty("transformationPosition")
        requireNotNull(transformationPosition)
        assertEquals(
            ENUtoNEDTriadConverter.conversionRotation.rotate(currentPosition),
            transformationPosition
        )

        val poseTransformation: EuclideanTransformation3D? =
            estimator.getPrivateProperty("poseTransformation")
        requireNotNull(poseTransformation)
        assertEquals(transformationRotation1, poseTransformation.rotation)
        assertEquals(transformationPosition, poseTransformation.translationPoint)
    }

    @Test
    fun attitudeEstimator_whenAccelerometerMeasurementNoBiasAndNoListener_updatesAcceleration() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator(context)

        // check initial value
        val specificForce: AccelerationTriad? = estimator.getPrivateProperty("specificForce")
        requireNotNull(specificForce)
        assertEquals(0.0, specificForce.valueX, 0.0)
        assertEquals(0.0, specificForce.valueY, 0.0)
        assertEquals(0.0, specificForce.valueZ, 0.0)

        val attitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(attitudeEstimator)
        val accelerometerMeasurementListener = attitudeEstimator.accelerometerMeasurementListener
        requireNotNull(accelerometerMeasurementListener)

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        accelerometerMeasurementListener.onMeasurement(
            ax,
            ay,
            az,
            null,
            null,
            null,
            timestamp,
            accuracy
        )

        // check
        assertEquals(ax.toDouble(), specificForce.valueY, 0.0)
        assertEquals(ay.toDouble(), specificForce.valueX, 0.0)
        assertEquals(az.toDouble(), -specificForce.valueZ, 0.0)
    }

    @Test
    fun attitudeEstimator_whenAccelerometerMeasurementWithBiasAndNoListener_updatesAcceleration() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator(context)

        // check initial value
        val specificForce: AccelerationTriad? = estimator.getPrivateProperty("specificForce")
        requireNotNull(specificForce)
        assertEquals(0.0, specificForce.valueX, 0.0)
        assertEquals(0.0, specificForce.valueY, 0.0)
        assertEquals(0.0, specificForce.valueZ, 0.0)

        val attitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(attitudeEstimator)
        val accelerometerMeasurementListener = attitudeEstimator.accelerometerMeasurementListener
        requireNotNull(accelerometerMeasurementListener)

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        accelerometerMeasurementListener.onMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            bz,
            timestamp,
            accuracy
        )

        // check
        assertEquals((ax - bx).toDouble(), specificForce.valueY, 0.0)
        assertEquals((ay - by).toDouble(), specificForce.valueX, 0.0)
        assertEquals((az - bz).toDouble(), -specificForce.valueZ, 0.0)
    }

    @Test
    fun attitudeEstimator_whenAccelerometerMeasurementAndListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val listener =
            mockk<AccelerometerSensorCollector.OnMeasurementListener>(relaxUnitFun = true)
        val estimator = RelativePoseEstimator(context, accelerometerMeasurementListener = listener)

        // check initial value
        val specificForce: AccelerationTriad? = estimator.getPrivateProperty("specificForce")
        requireNotNull(specificForce)
        assertEquals(0.0, specificForce.valueX, 0.0)
        assertEquals(0.0, specificForce.valueY, 0.0)
        assertEquals(0.0, specificForce.valueZ, 0.0)

        val attitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(attitudeEstimator)
        val accelerometerMeasurementListener = attitudeEstimator.accelerometerMeasurementListener
        requireNotNull(accelerometerMeasurementListener)

        val randomizer = UniformRandomizer()
        val ax = randomizer.nextFloat()
        val ay = randomizer.nextFloat()
        val az = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        accelerometerMeasurementListener.onMeasurement(
            ax,
            ay,
            az,
            bx,
            by,
            bz,
            timestamp,
            accuracy
        )

        // check
        assertEquals((ax - bx).toDouble(), specificForce.valueY, 0.0)
        assertEquals((ay - by).toDouble(), specificForce.valueX, 0.0)
        assertEquals((az - bz).toDouble(), -specificForce.valueZ, 0.0)

        verify(exactly = 1) { listener.onMeasurement(ax, ay, az, bx, by, bz, timestamp, accuracy) }
    }

    @Test
    fun attitudeEstimator_whenGyroscopeMeasurementNoBiasAndNoListener_updatesAngularSpeed() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator(context)

        // check initial value
        val angularSpeed: AngularSpeedTriad? = estimator.getPrivateProperty("angularSpeed")
        requireNotNull(angularSpeed)
        assertEquals(0.0, angularSpeed.valueX, 0.0)
        assertEquals(0.0, angularSpeed.valueY, 0.0)
        assertEquals(0.0, angularSpeed.valueZ, 0.0)

        val attitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(attitudeEstimator)
        val gyroscopeMeasurementListener = attitudeEstimator.gyroscopeMeasurementListener
        requireNotNull(gyroscopeMeasurementListener)

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        gyroscopeMeasurementListener.onMeasurement(
            wx,
            wy,
            wz,
            null,
            null,
            null,
            timestamp,
            accuracy
        )

        // check
        assertEquals(wx.toDouble(), angularSpeed.valueY, 0.0)
        assertEquals(wy.toDouble(), angularSpeed.valueX, 0.0)
        assertEquals(wz.toDouble(), -angularSpeed.valueZ, 0.0)
    }

    @Test
    fun attitudeEstimator_whenGyroscopeMeasurementWithBiasAndNoListener_updatesAngularSpeed() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator(context)

        // check initial value
        val angularSpeed: AngularSpeedTriad? = estimator.getPrivateProperty("angularSpeed")
        requireNotNull(angularSpeed)
        assertEquals(0.0, angularSpeed.valueX, 0.0)
        assertEquals(0.0, angularSpeed.valueY, 0.0)
        assertEquals(0.0, angularSpeed.valueZ, 0.0)

        val attitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(attitudeEstimator)
        val gyroscopeMeasurementListener = attitudeEstimator.gyroscopeMeasurementListener
        requireNotNull(gyroscopeMeasurementListener)

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        gyroscopeMeasurementListener.onMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp,
            accuracy
        )

        // check
        assertEquals((wx - bx).toDouble(), angularSpeed.valueY, 0.0)
        assertEquals((wy - by).toDouble(), angularSpeed.valueX, 0.0)
        assertEquals((wz - bz).toDouble(), -angularSpeed.valueZ, 0.0)
    }

    @Test
    fun attitudeEstimator_whenGyroscopeMeasurementAndListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val listener = mockk<GyroscopeSensorCollector.OnMeasurementListener>(relaxUnitFun = true)
        val estimator = RelativePoseEstimator(context, gyroscopeMeasurementListener = listener)

        // check initial value
        val angularSpeed: AngularSpeedTriad? = estimator.getPrivateProperty("angularSpeed")
        requireNotNull(angularSpeed)
        assertEquals(0.0, angularSpeed.valueX, 0.0)
        assertEquals(0.0, angularSpeed.valueY, 0.0)
        assertEquals(0.0, angularSpeed.valueZ, 0.0)

        val attitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(attitudeEstimator)
        val gyroscopeMeasurementListener = attitudeEstimator.gyroscopeMeasurementListener
        requireNotNull(gyroscopeMeasurementListener)

        val randomizer = UniformRandomizer()
        val wx = randomizer.nextFloat()
        val wy = randomizer.nextFloat()
        val wz = randomizer.nextFloat()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        gyroscopeMeasurementListener.onMeasurement(
            wx,
            wy,
            wz,
            bx,
            by,
            bz,
            timestamp,
            accuracy
        )

        // check
        assertEquals((wx - bx).toDouble(), angularSpeed.valueY, 0.0)
        assertEquals((wy - by).toDouble(), angularSpeed.valueX, 0.0)
        assertEquals((wz - bz).toDouble(), -angularSpeed.valueZ, 0.0)

        verify(exactly = 1) { listener.onMeasurement(wx, wy, wz, bx, by, bz, timestamp, accuracy) }
    }

    @Test
    fun attitudeEstimator_whenGravityMeasurementAndNoListener_updatesGravity() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator(context)

        // check initial value
        val gravity: AccelerationTriad? = estimator.getPrivateProperty("gravity")
        requireNotNull(gravity)
        assertEquals(0.0, gravity.valueX, 0.0)
        assertEquals(0.0, gravity.valueY, 0.0)
        assertEquals(0.0, gravity.valueZ, 0.0)

        val attitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(attitudeEstimator)
        val gravityEstimationListener = attitudeEstimator.gravityEstimationListener
        requireNotNull(gravityEstimationListener)

        val randomizer = UniformRandomizer()
        val fx = randomizer.nextDouble()
        val fy = randomizer.nextDouble()
        val fz = randomizer.nextDouble()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        gravityEstimationListener.onEstimation(mockk(), fx, fy, fz, timestamp)

        // check
        assertEquals(fx, gravity.valueX, 0.0)
        assertEquals(fy, gravity.valueY, 0.0)
        assertEquals(fz, gravity.valueZ, 0.0)
    }

    @Test
    fun attitudeEstimator_whenGravityMeasurementAndListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val listener = mockk<GravityEstimator.OnEstimationListener>(relaxUnitFun = true)
        val estimator = RelativePoseEstimator(context, gravityEstimationListener = listener)

        // check initial value
        val gravity: AccelerationTriad? = estimator.getPrivateProperty("gravity")
        requireNotNull(gravity)
        assertEquals(0.0, gravity.valueX, 0.0)
        assertEquals(0.0, gravity.valueY, 0.0)
        assertEquals(0.0, gravity.valueZ, 0.0)

        val attitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(attitudeEstimator)
        val gravityEstimationListener = attitudeEstimator.gravityEstimationListener
        requireNotNull(gravityEstimationListener)

        val randomizer = UniformRandomizer()
        val fx = randomizer.nextDouble()
        val fy = randomizer.nextDouble()
        val fz = randomizer.nextDouble()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        gravityEstimationListener.onEstimation(mockk(), fx, fy, fz, timestamp)

        // check
        assertEquals(fx, gravity.valueX, 0.0)
        assertEquals(fy, gravity.valueY, 0.0)
        assertEquals(fz, gravity.valueZ, 0.0)

        verify(exactly = 1) { listener.onEstimation(any(), fx, fy, fz, timestamp) }
    }

    @Test
    fun attitudeEstimator_whenNotInitialized_initializesAndSetsInitialPositionAttitudeAndSpeed() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialSpeed = getSpeed()
        val poseAvailableListener =
            mockk<RelativePoseEstimator.OnPoseAvailableListener>(relaxUnitFun = true)
        val estimator = RelativePoseEstimator(
            context,
            initialSpeed = initialSpeed,
            poseAvailableListener = poseAvailableListener
        )

        val attitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(attitudeEstimator)
        val attitudeEstimatorSpy = spyk(attitudeEstimator)
        every { attitudeEstimatorSpy.gyroscopeAverageTimeInterval }.returns(TIME_INTERVAL)

        val initialized1: Boolean? = estimator.getPrivateProperty("initialized")
        requireNotNull(initialized1)
        assertFalse(initialized1)

        val initialAttitude: Quaternion? = estimator.getPrivateProperty("initialAttitude")
        requireNotNull(initialAttitude)
        assertEquals(Quaternion(), initialAttitude)

        val previousAttitude: Quaternion? = estimator.getPrivateProperty("previousAttitude")
        requireNotNull(previousAttitude)
        assertEquals(Quaternion(), previousAttitude)

        val previousSpeed: SpeedTriad? = estimator.getPrivateProperty("previousSpeed")
        requireNotNull(previousSpeed)
        assertEquals(SpeedTriad(), previousSpeed)

        val previousPosition: InhomogeneousPoint3D? =
            estimator.getPrivateProperty("previousPosition")
        requireNotNull(previousPosition)
        assertEquals(InhomogeneousPoint3D(), previousPosition)

        val attitude = getAttitude()
        val c = CoordinateTransformation(
            attitude,
            FrameType.BODY_FRAME,
            FrameType.LOCAL_NAVIGATION_FRAME
        )

        // execute
        val attitudeAvailableListener = attitudeEstimatorSpy.attitudeAvailableListener
        requireNotNull(attitudeAvailableListener)
        attitudeAvailableListener.onAttitudeAvailable(
            attitudeEstimatorSpy,
            attitude,
            0L,
            null,
            null,
            null,
            c
        )

        // check
        assertEquals(attitude, initialAttitude)
        assertEquals(attitude, previousAttitude)
        assertEquals(initialSpeed, previousSpeed)
        assertEquals(InhomogeneousPoint3D(), previousPosition)

        val initialized2: Boolean? = estimator.getPrivateProperty("initialized")
        requireNotNull(initialized2)
        assertTrue(initialized2)

        verify { poseAvailableListener wasNot Called }
    }

    @Test
    fun attitudeEstimator_whenInitialized_computesCurrentPositionAttitudeAndSpeedAndNotifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialSpeed = getSpeed()
        val poseAvailableListener =
            mockk<RelativePoseEstimator.OnPoseAvailableListener>(relaxUnitFun = true)
        val estimator = RelativePoseEstimator(
            context,
            initialSpeed = initialSpeed,
            poseAvailableListener = poseAvailableListener
        )

        val attitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("attitudeEstimator")
        requireNotNull(attitudeEstimator)
        val attitudeEstimatorSpy = spyk(attitudeEstimator)
        every { attitudeEstimatorSpy.gyroscopeAverageTimeInterval }.returns(TIME_INTERVAL)

        // set as initialized
        estimator.setPrivateProperty("initialized", true)

        val initialized: Boolean? = estimator.getPrivateProperty("initialized")
        requireNotNull(initialized)
        assertTrue(initialized)

        val initialAttitude: Quaternion? = estimator.getPrivateProperty("initialAttitude")
        requireNotNull(initialAttitude)
        assertEquals(Quaternion(), initialAttitude)

        val previousAttitude: Quaternion? = estimator.getPrivateProperty("previousAttitude")
        requireNotNull(previousAttitude)
        assertEquals(Quaternion(), previousAttitude)
        val previousAttitude2 = Quaternion(previousAttitude)

        val previousSpeed: SpeedTriad? = estimator.getPrivateProperty("previousSpeed")
        requireNotNull(previousSpeed)
        initialSpeed.copyTo(previousSpeed)

        val previousPosition: InhomogeneousPoint3D? =
            estimator.getPrivateProperty("previousPosition")
        requireNotNull(previousPosition)
        assertEquals(InhomogeneousPoint3D(), previousPosition)

        val randomizer = UniformRandomizer()
        // set specific force
        val fx = randomizer.nextDouble()
        val fy = randomizer.nextDouble()
        val fz = randomizer.nextDouble()
        val specificForce = AccelerationTriad(fx, fy, fz)
        estimator.setPrivateProperty("specificForce", specificForce)

        // set angular speed
        val wx = randomizer.nextDouble()
        val wy = randomizer.nextDouble()
        val wz = randomizer.nextDouble()
        val angularSpeed = AngularSpeedTriad(wx, wy, wz)
        estimator.setPrivateProperty("angularSpeed", angularSpeed)

        // set gravity
        val gx = randomizer.nextDouble()
        val gy = randomizer.nextDouble()
        val gz = randomizer.nextDouble()
        val gravity = AccelerationTriad(gx, gy, gz)
        estimator.setPrivateProperty("gravity", gravity)

        // execute
        val attitude = getAttitude()
        val c = CoordinateTransformation(
            attitude,
            FrameType.BODY_FRAME,
            FrameType.LOCAL_NAVIGATION_FRAME
        )
        val timestamp = SystemClock.elapsedRealtimeNanos()

        // execute
        val attitudeAvailableListener = attitudeEstimatorSpy.attitudeAvailableListener
        requireNotNull(attitudeAvailableListener)
        attitudeAvailableListener.onAttitudeAvailable(
            attitudeEstimatorSpy,
            attitude,
            timestamp,
            null,
            null,
            null,
            c
        )

        // check
        val currentAttitude: Quaternion? = estimator.getPrivateProperty("currentAttitude")
        requireNotNull(currentAttitude)
        assertEquals(attitude, currentAttitude)

        val averageAttitude1: Quaternion? = estimator.getPrivateProperty("averageAttitude")
        requireNotNull(averageAttitude1)
        val averageAttitude2 = Quaternion.slerpAndReturnNew(previousAttitude2, currentAttitude, 0.5)
        assertEquals(averageAttitude1, averageAttitude2)

        val abb: Matrix? = estimator.getPrivateProperty("abb")
        requireNotNull(abb)
        assertEquals(3, abb.rows)
        assertEquals(1, abb.columns)
        assertEquals(specificForce.valueX - gx, abb.getElementAtIndex(0), 0.0)
        assertEquals(specificForce.valueY - gy, abb.getElementAtIndex(1), 0.0)
        assertEquals(specificForce.valueZ - gz, abb.getElementAtIndex(2), 0.0)

        val avgAttitudeMatrix: Matrix? = estimator.getPrivateProperty("avgAttitudeMatrix")
        requireNotNull(avgAttitudeMatrix)
        assertEquals(averageAttitude2.asInhomogeneousMatrix(), avgAttitudeMatrix)

        val abn: Matrix? = estimator.getPrivateProperty("abn")
        requireNotNull(abn)
        assertEquals(avgAttitudeMatrix.multiplyAndReturnNew(abb), abn)

        val ax = abn.getElementAtIndex(0)
        val ay = abn.getElementAtIndex(1)
        val az = abn.getElementAtIndex(2)

        val oldVx = initialSpeed.valueX
        val oldVy = initialSpeed.valueY
        val oldVz = initialSpeed.valueZ

        val newVx = oldVx + ax * TIME_INTERVAL
        val newVy = oldVy + ay * TIME_INTERVAL
        val newVz = oldVz + az * TIME_INTERVAL
        val currentSpeed1 = SpeedTriad(newVx, newVy, newVz)
        val currentSpeed2: SpeedTriad? = estimator.getPrivateProperty("currentSpeed")
        requireNotNull(currentSpeed2)
        assertEquals(currentSpeed1, currentSpeed2)

        // old position is initially at zero
        val newX = 0.5 * (oldVx + newVx) * TIME_INTERVAL
        val newY = 0.5 * (oldVy + newVy) * TIME_INTERVAL
        val newZ = 0.5 * (oldVz + newVz) * TIME_INTERVAL
        val currentPosition1 = InhomogeneousPoint3D(newX, newY, newZ)
        val currentPosition2: InhomogeneousPoint3D? =
            estimator.getPrivateProperty("currentPosition")
        requireNotNull(currentPosition2)
        assertEquals(currentPosition1, currentPosition2)

        val poseTransformation: EuclideanTransformation3D? =
            estimator.getPrivateProperty("poseTransformation")
        requireNotNull(poseTransformation)

        verify(exactly = 1) {
            poseAvailableListener.onPoseAvailable(
                estimator,
                timestamp,
                poseTransformation
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

        const val MIN_DELTA_POS = -5.0
        const val MAX_DELTA_POS = 5.0

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

        fun getSpeed(): SpeedTriad {
            val randomizer = UniformRandomizer()
            return SpeedTriad(
                randomizer.nextDouble(),
                randomizer.nextDouble(),
                randomizer.nextDouble()
            )
        }
    }
}