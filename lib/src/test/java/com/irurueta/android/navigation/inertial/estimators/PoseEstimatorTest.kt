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
package com.irurueta.android.navigation.inertial.estimators

import android.content.Context
import android.location.Location
import android.os.SystemClock
import androidx.test.core.app.ApplicationProvider
import com.irurueta.android.navigation.inertial.*
import com.irurueta.android.navigation.inertial.collectors.*
import com.irurueta.android.navigation.inertial.estimators.filter.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.MeanAveragingFilter
import com.irurueta.geometry.EuclideanTransformation3D
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.*
import com.irurueta.navigation.frames.converters.NEDtoECEFPositionVelocityConverter
import com.irurueta.navigation.inertial.BodyKinematics
import com.irurueta.navigation.inertial.navigators.ECEFInertialNavigator
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel
import com.irurueta.statistics.UniformRandomizer
import io.mockk.*
import org.junit.Assert.*
import org.junit.Ignore
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import java.util.*

@Ignore
@RunWith(RobolectricTestRunner::class)
class PoseEstimatorTest {

    @Test
    fun constructor_whenRequiredProperties_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = PoseEstimator(context, location)

        // check
        assertSame(context, estimator.context)
        assertSame(location, estimator.location)
        assertEquals(NEDVelocity(), estimator.initialVelocity)
        assertEquals(SensorDelay.GAME, estimator.sensorDelay)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER,
            estimator.accelerometerSensorType
        )
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER,
            estimator.magnetometerSensorType
        )
        assertNotNull(estimator.accelerometerAveragingFilter)
        assertTrue(estimator.accelerometerAveragingFilter is LowPassAveragingFilter)
        assertEquals(GyroscopeSensorCollector.SensorType.GYROSCOPE, estimator.gyroscopeSensorType)
        assertNull(estimator.worldMagneticModel)
        assertNotNull(estimator.timestamp)
        assertFalse(estimator.useWorldMagneticModel)
        assertTrue(estimator.useAccurateLevelingEstimator)
        assertTrue(estimator.useAccurateRelativeGyroscopeAttitudeEstimator)
        assertFalse(estimator.estimateInitialTransformation)
        assertFalse(estimator.estimatePreviousTransformation)
        assertNull(estimator.poseAvailableListener)
        assertNull(estimator.accelerometerMeasurementListener)
        assertNull(estimator.gyroscopeMeasurementListener)
        assertNull(estimator.magnetometerMeasurementListener)
        assertFalse(estimator.running)
        assertEquals(0.0, estimator.averageTimeInterval, 0.0)
    }

    @Test
    fun constructor_whenAllProperties_setsExpectedValues() {
        val location = getLocation()
        val initialVelocity = NEDVelocity()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val accelerometerAveragingFilter = MeanAveragingFilter()
        val worldMagneticModel = WorldMagneticModel()
        val timestamp = Date()
        val poseAvailableListener = mockk<PoseEstimator.OnPoseAvailableListener>()
        val accelerometerMeasurementListener =
            mockk<AccelerometerSensorCollector.OnMeasurementListener>()
        val gyroscopeMeasurementListener = mockk<GyroscopeSensorCollector.OnMeasurementListener>()
        val magnetometerMeasurementListener =
            mockk<MagnetometerSensorCollector.OnMeasurementListener>()
        val estimator = PoseEstimator(
            context,
            location,
            initialVelocity,
            SensorDelay.NORMAL,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            accelerometerAveragingFilter,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            worldMagneticModel,
            timestamp,
            useWorldMagneticModel = true,
            useAccurateLevelingEstimator = false,
            useAccurateRelativeGyroscopeAttitudeEstimator = false,
            estimateInitialTransformation = true,
            estimatePreviousTransformation = true,
            poseAvailableListener,
            accelerometerMeasurementListener,
            gyroscopeMeasurementListener,
            magnetometerMeasurementListener
        )

        // check
        assertSame(context, estimator.context)
        assertSame(location, estimator.location)
        assertSame(initialVelocity, estimator.initialVelocity)
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertEquals(
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            estimator.accelerometerSensorType
        )
        assertEquals(
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            estimator.magnetometerSensorType
        )
        assertSame(accelerometerAveragingFilter, estimator.accelerometerAveragingFilter)
        assertEquals(
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            estimator.gyroscopeSensorType
        )
        assertSame(worldMagneticModel, estimator.worldMagneticModel)
        assertSame(timestamp, estimator.timestamp)
        assertTrue(estimator.useWorldMagneticModel)
        assertFalse(estimator.useAccurateLevelingEstimator)
        assertFalse(estimator.useAccurateRelativeGyroscopeAttitudeEstimator)
        assertTrue(estimator.estimateInitialTransformation)
        assertTrue(estimator.estimatePreviousTransformation)
        assertSame(poseAvailableListener, estimator.poseAvailableListener)
        assertSame(accelerometerMeasurementListener, estimator.accelerometerMeasurementListener)
        assertSame(gyroscopeMeasurementListener, estimator.gyroscopeMeasurementListener)
        assertSame(magnetometerMeasurementListener, estimator.magnetometerMeasurementListener)
        assertFalse(estimator.running)
        assertEquals(0.0, estimator.averageTimeInterval, 0.0)
    }

    @Test
    fun magnetometerMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = PoseEstimator(context, location)

        // check default value
        assertNull(estimator.magnetometerMeasurementListener)

        // set new value
        val magnetometerMeasurementListener =
            mockk<MagnetometerSensorCollector.OnMeasurementListener>()
        estimator.magnetometerMeasurementListener = magnetometerMeasurementListener

        // check
        assertSame(magnetometerMeasurementListener, estimator.magnetometerMeasurementListener)
        val absoluteAttitudeEstimator: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        val listener = absoluteAttitudeEstimator.magnetometerMeasurementListener
        assertSame(magnetometerMeasurementListener, listener)
    }

    @Test
    fun location_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location1 = getLocation()
        val estimator = PoseEstimator(context, location1)

        val absoluteAttitudeEstimator: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)

        // check default values
        assertSame(location1, estimator.location)
        assertSame(location1, absoluteAttitudeEstimator.location)
        assertFalse(estimator.running)

        // set new value
        val location2 = getLocation()
        estimator.location = location2

        // check
        assertSame(location2, estimator.location)
        assertSame(location2, absoluteAttitudeEstimator.location)
    }

    @Test
    fun worldMagneticModel_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = PoseEstimator(context, location)

        val absoluteAttitudeEstimator: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)

        // check
        assertNull(estimator.worldMagneticModel)
        assertNull(absoluteAttitudeEstimator.worldMagneticModel)
        assertFalse(estimator.running)

        // set new value
        val worldMagneticModel = WorldMagneticModel()
        estimator.worldMagneticModel = worldMagneticModel

        // check
        assertSame(worldMagneticModel, estimator.worldMagneticModel)
        assertSame(worldMagneticModel, absoluteAttitudeEstimator.worldMagneticModel)
    }

    @Test(expected = IllegalStateException::class)
    fun worldMagneticModel_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = PoseEstimator(context, location)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        // set new value
        val worldMagneticModel = WorldMagneticModel()
        estimator.worldMagneticModel = worldMagneticModel
    }

    @Test
    fun timestamp_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = PoseEstimator(context, location)

        // check default value
        assertNotNull(estimator.timestamp)
        assertFalse(estimator.running)

        // set new value
        val timestamp = Date()
        estimator.timestamp = timestamp

        // check
        assertSame(timestamp, estimator.timestamp)

        val absoluteAttitudeEstimator: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        assertSame(timestamp, absoluteAttitudeEstimator.timestamp)
    }

    @Test
    fun useWorldMagneticModel_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = PoseEstimator(context, location)

        val absoluteAttitudeEstimator: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)

        // check default value
        assertFalse(estimator.useWorldMagneticModel)
        assertFalse(absoluteAttitudeEstimator.useWorldMagneticModel)
        assertFalse(estimator.running)

        // set new value
        estimator.useWorldMagneticModel = true

        // check
        assertTrue(estimator.useWorldMagneticModel)
        assertTrue(absoluteAttitudeEstimator.useWorldMagneticModel)
    }

    @Test(expected = IllegalStateException::class)
    fun useWorldMagneticModel_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = PoseEstimator(context, location)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        // set new value
        estimator.useWorldMagneticModel = true
    }

    @Test
    fun useAccurateLevelingEstimator_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = PoseEstimator(context, location)

        val absoluteAttitudeEstimator: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)

        // check default value
        assertTrue(estimator.useAccurateLevelingEstimator)
        assertTrue(absoluteAttitudeEstimator.useAccurateLevelingEstimator)
        assertFalse(estimator.running)
        assertSame(location, estimator.location)

        // set new value
        estimator.useAccurateLevelingEstimator = false

        // check
        assertFalse(estimator.useAccurateLevelingEstimator)
        assertFalse(absoluteAttitudeEstimator.useAccurateLevelingEstimator)
    }

    @Test(expected = IllegalStateException::class)
    fun useAccurateLevelingEstimator_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = PoseEstimator(context, location)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        // check
        estimator.useAccurateLevelingEstimator = true
    }

    @Test
    fun useAccurateRelativeGyroscopeAttitudeEstimator_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = PoseEstimator(context, location)

        val absoluteAttitudeEstimator: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)

        // check default values
        assertTrue(estimator.useAccurateRelativeGyroscopeAttitudeEstimator)
        assertTrue(absoluteAttitudeEstimator.useAccurateRelativeGyroscopeAttitudeEstimator)
        assertFalse(estimator.running)

        // set new value
        estimator.useAccurateRelativeGyroscopeAttitudeEstimator = false

        // check
        assertFalse(estimator.useAccurateRelativeGyroscopeAttitudeEstimator)
        assertFalse(absoluteAttitudeEstimator.useAccurateRelativeGyroscopeAttitudeEstimator)
    }

    @Test(expected = IllegalStateException::class)
    fun useAccurateRelativeGyroscopeAttitudeEstimator_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = PoseEstimator(context, location)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        // set new value
        estimator.useAccurateRelativeGyroscopeAttitudeEstimator = true
    }

    @Test
    fun averageTimeInterval_callsInternalAttitudeEstimator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = PoseEstimator(context, location)

        val absoluteAttitudeEstimator: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        val absoluteAttitudeEstimatorSpy = spyk(absoluteAttitudeEstimator)
        every { absoluteAttitudeEstimatorSpy.gyroscopeAverageTimeInterval }.returns(TIME_INTERVAL)
        estimator.setPrivateProperty("absoluteAttitudeEstimator", absoluteAttitudeEstimatorSpy)

        assertEquals(TIME_INTERVAL, estimator.averageTimeInterval, 0.0)
        verify(exactly = 1) { absoluteAttitudeEstimatorSpy.gyroscopeAverageTimeInterval }
    }

    @Test
    fun start_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = PoseEstimator(context, location)

        val absoluteAttitudeEstimator: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        val absoluteAttitudeEstimatorSpy = spyk(absoluteAttitudeEstimator)
        estimator.setPrivateProperty("absoluteAttitudeEstimator", absoluteAttitudeEstimatorSpy)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        assertThrows(IllegalStateException::class.java) { estimator.start() }
        verify { absoluteAttitudeEstimatorSpy wasNot Called }
    }

    @Test
    fun start_whenNotRunningAndInternalEstimatorFails_stopsAndReturnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = PoseEstimator(context, location)

        val absoluteAttitudeEstimator: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        val absoluteAttitudeEstimatorSpy = spyk(absoluteAttitudeEstimator)
        every { absoluteAttitudeEstimatorSpy.start() }.returns(false)
        estimator.setPrivateProperty("absoluteAttitudeEstimator", absoluteAttitudeEstimatorSpy)

        assertFalse(estimator.running)

        // start
        assertFalse(estimator.start())
        assertFalse(estimator.running)

        verify(exactly = 1) { absoluteAttitudeEstimatorSpy.start() }
        verify(exactly = 1) { absoluteAttitudeEstimatorSpy.stop() }
    }

    @Test
    fun start_whenNotRunningAndInternalEstimatorSucceeds_returnsTrue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = PoseEstimator(context, location)

        val absoluteAttitudeEstimator: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        val absoluteAttitudeEstimatorSpy = spyk(absoluteAttitudeEstimator)
        every { absoluteAttitudeEstimatorSpy.start() }.returns(true)
        estimator.setPrivateProperty("absoluteAttitudeEstimator", absoluteAttitudeEstimatorSpy)

        assertFalse(estimator.running)

        // start
        assertTrue(estimator.start())
        assertTrue(estimator.running)

        verify(exactly = 1) { absoluteAttitudeEstimatorSpy.start() }
        verify(exactly = 0) { absoluteAttitudeEstimatorSpy.stop() }
    }

    @Test
    fun start_whenNotRunning_resetsInitialized() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = PoseEstimator(context, location)

        // set as initialized
        estimator.setPrivateProperty("initialized", true)
        val initialized1: Boolean? = estimator.getPrivateProperty("initialized")
        requireNotNull(initialized1)
        assertTrue(initialized1)

        val absoluteAttitudeEstimator: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        val absoluteAttitudeEstimatorSpy = spyk(absoluteAttitudeEstimator)
        every { absoluteAttitudeEstimatorSpy.start() }.returns(false)
        estimator.setPrivateProperty("absoluteAttitudeEstimator", absoluteAttitudeEstimatorSpy)

        assertFalse(estimator.running)

        // start
        assertFalse(estimator.start())
        assertFalse(estimator.running)

        verify(exactly = 1) { absoluteAttitudeEstimatorSpy.start() }
        verify(exactly = 1) { absoluteAttitudeEstimatorSpy.stop() }

        val initialized2: Boolean? = estimator.getPrivateProperty("initialized")
        requireNotNull(initialized2)
        assertFalse(initialized2)
    }

    @Test
    fun stop_callsInternalEstimatorAndSetsAsNotRunning() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = PoseEstimator(context, location)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        val absoluteAttitudeEstimator: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        val absoluteAttitudeEstimatorSpy = spyk(absoluteAttitudeEstimator)
        estimator.setPrivateProperty("absoluteAttitudeEstimator", absoluteAttitudeEstimatorSpy)

        estimator.stop()

        // check
        assertFalse(estimator.running)
        verify(exactly = 1) { absoluteAttitudeEstimatorSpy.stop() }
    }

    @Test
    fun initialize_whenNotInitializedAndNoLocation_initializesFrame() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = PoseEstimator(context, location)

        val initialized1: Boolean? = estimator.getPrivateProperty("initialized")
        requireNotNull(initialized1)
        assertFalse(initialized1)

        val initialFrame: ECEFFrame? = estimator.getPrivateProperty("initialFrame")
        requireNotNull(initialFrame)
        assertEquals(Quaternion(), initialFrame.coordinateTransformation.asRotation())

        val q = getAttitude()
        val c = CoordinateTransformation(
            FrameType.BODY_FRAME,
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME
        )
        c.fromRotation(q)

        val result: Boolean? = estimator.callPrivateFuncWithResult("initialize", c)
        requireNotNull(result)
        assertFalse(result)

        // check
        assertEquals(c, initialFrame.coordinateTransformation)
        assertEquals(q, initialFrame.coordinateTransformation.asRotation())

        val nedPosition = location.toNEDPosition()
        val nedVelocity = NEDVelocity()
        val ecefPosition = ECEFPosition()
        val ecefVelocity = ECEFVelocity()
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
            nedPosition,
            nedVelocity,
            ecefPosition,
            ecefVelocity
        )

        assertEquals(ecefPosition, initialFrame.ecefPosition)

        val previousFrame: ECEFFrame? = estimator.getPrivateProperty("previousFrame")
        requireNotNull(previousFrame)
        assertEquals(previousFrame, initialFrame)

        val initialized2: Boolean? = estimator.getPrivateProperty("initialized")
        requireNotNull(initialized2)
        assertTrue(initialized2)
    }

    @Test
    fun initialize_whenNotInitializedAndLocation_initializesFrame() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = PoseEstimator(context, location)

        val initialized1: Boolean? = estimator.getPrivateProperty("initialized")
        requireNotNull(initialized1)
        assertFalse(initialized1)

        val initialFrame: ECEFFrame? = estimator.getPrivateProperty("initialFrame")
        requireNotNull(initialFrame)
        assertEquals(Quaternion(), initialFrame.coordinateTransformation.asRotation())

        val q = getAttitude()
        val c = CoordinateTransformation(
            FrameType.BODY_FRAME,
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME
        )
        c.fromRotation(q)

        val result: Boolean? = estimator.callPrivateFuncWithResult("initialize", c)
        requireNotNull(result)
        assertFalse(result)

        // check
        assertEquals(c, initialFrame.coordinateTransformation)
        assertEquals(q, initialFrame.coordinateTransformation.asRotation())

        val nedPosition = location.toNEDPosition()
        val nedVelocity = NEDVelocity()
        val ecefPosition = ECEFPosition()
        val ecefVelocity = ECEFVelocity()
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
            nedPosition,
            nedVelocity,
            ecefPosition,
            ecefVelocity
        )

        assertEquals(ecefPosition, initialFrame.ecefPosition)

        val previousFrame: ECEFFrame? = estimator.getPrivateProperty("previousFrame")
        requireNotNull(previousFrame)
        assertEquals(previousFrame, initialFrame)

        val initialized2: Boolean? = estimator.getPrivateProperty("initialized")
        requireNotNull(initialized2)
        assertTrue(initialized2)
    }

    @Test
    fun initialize_whenAlreadyInitialized_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = PoseEstimator(context, location)

        estimator.setPrivateProperty("initialized", true)
        val initialized1: Boolean? = estimator.getPrivateProperty("initialized")
        requireNotNull(initialized1)
        assertTrue(initialized1)

        val initialFrame: ECEFFrame? = estimator.getPrivateProperty("initialFrame")
        requireNotNull(initialFrame)
        val initialFrameSpy = spyk(initialFrame)
        estimator.setPrivateProperty("initialFrame", initialFrameSpy)

        val previousFrame: ECEFFrame? = estimator.getPrivateProperty("previousFrame")
        requireNotNull(previousFrame)
        val previousFrameSpy = spyk(previousFrame)
        estimator.setPrivateProperty("previousFrame", previousFrameSpy)

        val q = getAttitude()
        val c = CoordinateTransformation(
            FrameType.BODY_FRAME,
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME
        )
        c.fromRotation(q)

        val result: Boolean? = estimator.callPrivateFuncWithResult("initialize", c)
        requireNotNull(result)
        assertTrue(result)

        // check
        verify { initialFrameSpy wasNot Called }
        verify { previousFrameSpy wasNot Called }

        val initialized2: Boolean? = estimator.getPrivateProperty("initialized")
        requireNotNull(initialized2)
        assertTrue(initialized2)
    }

    @Test
    fun computeTransformation_whenAtCloseLocationAndNoAttitudeChange_computesExpectedTransformation() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = PoseEstimator(context, location)

        val startNedPosition = getLocation().toNEDPosition()
        val startNedVelocity = getNEDVelocity()
        val startEcefPosition = ECEFPosition()
        val ecefVelocity = ECEFVelocity()
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
            startNedPosition,
            startNedVelocity,
            startEcefPosition,
            ecefVelocity
        )
        val startAttitude = Quaternion()
        val startC = CoordinateTransformation(
            FrameType.BODY_FRAME,
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME
        )
        startC.fromRotation(startAttitude)
        val startFrame = ECEFFrame(startEcefPosition, ecefVelocity, startC)

        val randomizer = UniformRandomizer()
        val deltaX = randomizer.nextDouble(MIN_DELTA_POS, MAX_DELTA_POS)
        val deltaY = randomizer.nextDouble(MIN_DELTA_POS, MAX_DELTA_POS)
        val deltaZ = randomizer.nextDouble(MIN_DELTA_POS, MAX_DELTA_POS)
        val endEcefPosition = ECEFPosition(
            startEcefPosition.x + deltaX,
            startEcefPosition.y + deltaY,
            startEcefPosition.z + deltaZ
        )
        val endAttitude = Quaternion()
        val endC = CoordinateTransformation(
            FrameType.BODY_FRAME,
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME
        )
        startC.fromRotation(endAttitude)
        val endFrame = ECEFFrame(endEcefPosition, ecefVelocity, endC)

        val transformation = EuclideanTransformation3D()
        callPrivateFunc(
            PoseEstimator::class,
            estimator,
            "computeTransformation",
            startFrame,
            endFrame,
            transformation
        )

        val startPoint = startEcefPosition.position
        val endPoint = endEcefPosition.position

        // transform start point using transformation and check it becomes the endPoint
        val transformedPoint = transformation.transformAndReturnNew(startPoint)
        assertTrue(transformedPoint.equals(endPoint, ABSOLUTE_ERROR))
    }

    @Test
    fun computeTransformation_whenAtCloseLocationAndAttitudeChange_computesExpectedTransformation() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = PoseEstimator(context, location)

        val startNedPosition = getLocation().toNEDPosition()
        val startNedVelocity = getNEDVelocity()
        val startEcefPosition = ECEFPosition()
        val ecefVelocity = ECEFVelocity()
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
            startNedPosition,
            startNedVelocity,
            startEcefPosition,
            ecefVelocity
        )
        val startAttitude = getAttitude()
        val startC = CoordinateTransformation(
            FrameType.BODY_FRAME,
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME
        )
        startC.fromRotation(startAttitude)
        val startFrame = ECEFFrame(startEcefPosition, ecefVelocity, startC)

        val randomizer = UniformRandomizer()
        val deltaX = randomizer.nextDouble(MIN_DELTA_POS, MAX_DELTA_POS)
        val deltaY = randomizer.nextDouble(MIN_DELTA_POS, MAX_DELTA_POS)
        val deltaZ = randomizer.nextDouble(MIN_DELTA_POS, MAX_DELTA_POS)
        val endEcefPosition = ECEFPosition(
            startEcefPosition.x + deltaX,
            startEcefPosition.y + deltaY,
            startEcefPosition.z + deltaZ
        )
        val endAttitude = getAttitude()
        val endC = CoordinateTransformation(
            FrameType.BODY_FRAME,
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME
        )
        startC.fromRotation(endAttitude)
        val endFrame = ECEFFrame(endEcefPosition, ecefVelocity, endC)

        val transformation = EuclideanTransformation3D()
        callPrivateFunc(
            PoseEstimator::class,
            estimator,
            "computeTransformation",
            startFrame,
            endFrame,
            transformation
        )

        val startPoint = startEcefPosition.position
        val endPoint = endEcefPosition.position

        // transform start point using transformation and check it becomes the endPoint
        val transformedPoint = transformation.transformAndReturnNew(startPoint)
        assertTrue(transformedPoint.equals(endPoint, ABSOLUTE_ERROR))
    }

    @Test
    fun computeTransformation_whenAtRandomLocations_computesExpectedTransformation() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = PoseEstimator(context, location)

        val startNedPosition = getLocation().toNEDPosition()
        val startNedVelocity = getNEDVelocity()
        val startEcefPosition = ECEFPosition()
        val startEcefVelocity = ECEFVelocity()
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
            startNedPosition,
            startNedVelocity,
            startEcefPosition,
            startEcefVelocity
        )
        val startAttitude = getAttitude()
        val startC = CoordinateTransformation(
            FrameType.BODY_FRAME,
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME
        )
        startC.fromRotation(startAttitude)
        val startFrame = ECEFFrame(startEcefPosition, startEcefVelocity, startC)


        val endNedPosition = getLocation().toNEDPosition()
        val endNedVelocity = getNEDVelocity()
        val endEcefPosition = ECEFPosition()
        val endEcefVelocity = ECEFVelocity()
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
            endNedPosition,
            endNedVelocity,
            endEcefPosition,
            endEcefVelocity
        )
        val endAttitude = getAttitude()
        val endC = CoordinateTransformation(
            FrameType.BODY_FRAME,
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME
        )
        startC.fromRotation(endAttitude)
        val endFrame = ECEFFrame(endEcefPosition, endEcefVelocity, endC)

        val transformation = EuclideanTransformation3D()
        callPrivateFunc(
            PoseEstimator::class,
            estimator,
            "computeTransformation",
            startFrame,
            endFrame,
            transformation
        )

        val startPoint = startEcefPosition.position
        val endPoint = endEcefPosition.position

        // transform start point using transformation and check it becomes the endPoint
        val transformedPoint = transformation.transformAndReturnNew(startPoint)
        assertTrue(transformedPoint.equals(endPoint, ABSOLUTE_ERROR))
    }

    @Test
    fun absoluteAttitudeEstimator_whenAccelerometerMeasurementNoBiasAndNoListener_updatesBodyKinematics() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = PoseEstimator(context, location)

        // check initial value
        val bodyKinematics: BodyKinematics? = estimator.getPrivateProperty("bodyKinematics")
        requireNotNull(bodyKinematics)
        assertEquals(0.0, bodyKinematics.fx, 0.0)
        assertEquals(0.0, bodyKinematics.fy, 0.0)
        assertEquals(0.0, bodyKinematics.fz, 0.0)

        val absoluteAttitudeEstimator: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        val accelerometerMeasurementListener =
            absoluteAttitudeEstimator.accelerometerMeasurementListener
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
        assertEquals(ax.toDouble(), bodyKinematics.fx, 0.0)
        assertEquals(ay.toDouble(), bodyKinematics.fy, 0.0)
        assertEquals(az.toDouble(), bodyKinematics.fz, 0.0)
    }

    @Test
    fun absoluteAttitudeEstimator_whenAccelerometerMeasurementWithBiasAndNoListener_updatesBodyKinematics() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = PoseEstimator(context, location)

        // check initial value
        val bodyKinematics: BodyKinematics? = estimator.getPrivateProperty("bodyKinematics")
        requireNotNull(bodyKinematics)
        assertEquals(0.0, bodyKinematics.fx, 0.0)
        assertEquals(0.0, bodyKinematics.fy, 0.0)
        assertEquals(0.0, bodyKinematics.fz, 0.0)

        val absoluteAttitudeEstimator: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        val accelerometerMeasurementListener =
            absoluteAttitudeEstimator.accelerometerMeasurementListener
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
        assertEquals((ax - bx).toDouble(), bodyKinematics.fx, 0.0)
        assertEquals((ay - by).toDouble(), bodyKinematics.fy, 0.0)
        assertEquals((az - bz).toDouble(), bodyKinematics.fz, 0.0)
    }

    @Test
    fun absoluteAttitudeEstimator_whenAccelerometerMeasurementAndListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val listener =
            mockk<AccelerometerSensorCollector.OnMeasurementListener>(relaxUnitFun = true)
        val estimator =
            PoseEstimator(context, location, accelerometerMeasurementListener = listener)

        // check initial value
        val bodyKinematics: BodyKinematics? = estimator.getPrivateProperty("bodyKinematics")
        requireNotNull(bodyKinematics)
        assertEquals(0.0, bodyKinematics.fx, 0.0)
        assertEquals(0.0, bodyKinematics.fy, 0.0)
        assertEquals(0.0, bodyKinematics.fz, 0.0)

        val absoluteAttitudeEstimator: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        val accelerometerMeasurementListener =
            absoluteAttitudeEstimator.accelerometerMeasurementListener
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
        assertEquals((ax - bx).toDouble(), bodyKinematics.fx, 0.0)
        assertEquals((ay - by).toDouble(), bodyKinematics.fy, 0.0)
        assertEquals((az - bz).toDouble(), bodyKinematics.fz, 0.0)

        verify(exactly = 1) { listener.onMeasurement(ax, ay, az, bx, by, bz, timestamp, accuracy) }
    }

    @Test
    fun absoluteAttitudeEstimator_whenGyroscopeMeasurementNoBiasAndNoListener_updatesBodyKinematics() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = PoseEstimator(context, location)

        // check initial value
        val bodyKinematics: BodyKinematics? = estimator.getPrivateProperty("bodyKinematics")
        requireNotNull(bodyKinematics)
        assertEquals(0.0, bodyKinematics.angularRateX, 0.0)
        assertEquals(0.0, bodyKinematics.angularRateY, 0.0)
        assertEquals(0.0, bodyKinematics.angularRateZ, 0.0)

        val absoluteAttitudeEstimator: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        val gyroscopeMeasurementListener = absoluteAttitudeEstimator.gyroscopeMeasurementListener
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
        assertEquals(wx.toDouble(), bodyKinematics.angularRateX, 0.0)
        assertEquals(wy.toDouble(), bodyKinematics.angularRateY, 0.0)
        assertEquals(wz.toDouble(), bodyKinematics.angularRateZ, 0.0)
    }

    @Test
    fun absoluteAttitudeEstimator_whenGyroscopeMeasurementWithBiasAndNoListener_updatesBodyKinematics() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = PoseEstimator(context, location)

        // check initial value
        val bodyKinematics: BodyKinematics? = estimator.getPrivateProperty("bodyKinematics")
        requireNotNull(bodyKinematics)
        assertEquals(0.0, bodyKinematics.angularRateX, 0.0)
        assertEquals(0.0, bodyKinematics.angularRateY, 0.0)
        assertEquals(0.0, bodyKinematics.angularRateZ, 0.0)

        val absoluteAttitudeEstimator: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        val gyroscopeMeasurementListener = absoluteAttitudeEstimator.gyroscopeMeasurementListener
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
        assertEquals((wx - bx).toDouble(), bodyKinematics.angularRateX, 0.0)
        assertEquals((wy - by).toDouble(), bodyKinematics.angularRateY, 0.0)
        assertEquals((wz - bz).toDouble(), bodyKinematics.angularRateZ, 0.0)
    }

    @Test
    fun absoluteAttitudeEstimator_whenGyroscopeMeasurementAndListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val listener = mockk<GyroscopeSensorCollector.OnMeasurementListener>(relaxUnitFun = true)
        val estimator = PoseEstimator(context, location, gyroscopeMeasurementListener = listener)

        // check initial value
        val bodyKinematics: BodyKinematics? = estimator.getPrivateProperty("bodyKinematics")
        requireNotNull(bodyKinematics)
        assertEquals(0.0, bodyKinematics.angularRateX, 0.0)
        assertEquals(0.0, bodyKinematics.angularRateY, 0.0)
        assertEquals(0.0, bodyKinematics.angularRateZ, 0.0)

        val absoluteAttitudeEstimator: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        val gyroscopeMeasurementListener = absoluteAttitudeEstimator.gyroscopeMeasurementListener
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
        assertEquals((wx - bx).toDouble(), bodyKinematics.angularRateX, 0.0)
        assertEquals((wy - by).toDouble(), bodyKinematics.angularRateY, 0.0)
        assertEquals((wz - bz).toDouble(), bodyKinematics.angularRateZ, 0.0)

        verify(exactly = 1) { listener.onMeasurement(wx, wy, wz, bx, by, bz, timestamp, accuracy) }
    }

    @Test
    fun absoluteAttitudeEstimator_whenNotInitializedAndNoLocation_initializesAndSetsInitialFrame() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val poseAvailableListener =
            mockk<PoseEstimator.OnPoseAvailableListener>(relaxUnitFun = true)
        val estimator =
            PoseEstimator(context, location, poseAvailableListener = poseAvailableListener)

        val absoluteAttitudeEstimator: FusedGeomagneticAttitudeEstimator? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        val absoluteAttitudeEstimatorSpy = spyk(absoluteAttitudeEstimator)
        every { absoluteAttitudeEstimatorSpy.gyroscopeAverageTimeInterval }.returns(TIME_INTERVAL)

        val initialized1: Boolean? = estimator.getPrivateProperty("initialized")
        requireNotNull(initialized1)
        assertFalse(initialized1)

        val attitude = getAttitude()
        val c = CoordinateTransformation(
            FrameType.BODY_FRAME,
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME
        )
        c.fromRotation(attitude)

        // execute
        val attitudeAvailableListener = absoluteAttitudeEstimatorSpy.attitudeAvailableListener
        requireNotNull(attitudeAvailableListener)
        attitudeAvailableListener.onAttitudeAvailable(
            absoluteAttitudeEstimatorSpy,
            attitude,
            0L,
            null,
            null,
            null,
            c
        )

        // check
        val initialized2: Boolean? = estimator.getPrivateProperty("initialized")
        requireNotNull(initialized2)
        assertTrue(initialized2)

        val initialFrame: ECEFFrame? = estimator.getPrivateProperty("initialFrame")
        requireNotNull(initialFrame)
        assertEquals(c, initialFrame.coordinateTransformation)

        val nedPosition = location.toNEDPosition()
        val nedVelocity = NEDVelocity()
        val ecefPosition = ECEFPosition()
        val ecefVelocity = ECEFVelocity()
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
            nedPosition,
            nedVelocity,
            ecefPosition,
            ecefVelocity
        )

        assertEquals(ecefPosition, initialFrame.ecefPosition)

        val previousFrame: ECEFFrame? = estimator.getPrivateProperty("previousFrame")
        requireNotNull(previousFrame)
        assertEquals(initialFrame, previousFrame)

        verify { poseAvailableListener wasNot Called }
    }

    @Test
    fun absoluteAttitudeEstimator_whenNotInitializedAndLocation_initializesAndSetsInitialFrame() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val poseAvailableListener =
            mockk<PoseEstimator.OnPoseAvailableListener>(relaxUnitFun = true)
        val estimator =
            PoseEstimator(context, location, poseAvailableListener = poseAvailableListener)

        val absoluteAttitudeEstimator: FusedGeomagneticAttitudeEstimator? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        val absoluteAttitudeEstimatorSpy = spyk(absoluteAttitudeEstimator)
        every { absoluteAttitudeEstimatorSpy.gyroscopeAverageTimeInterval }.returns(TIME_INTERVAL)

        val initialized1: Boolean? = estimator.getPrivateProperty("initialized")
        requireNotNull(initialized1)
        assertFalse(initialized1)

        val attitude = getAttitude()
        val c = CoordinateTransformation(
            FrameType.BODY_FRAME,
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME
        )
        c.fromRotation(attitude)

        // execute
        val attitudeAvailableListener = absoluteAttitudeEstimatorSpy.attitudeAvailableListener
        requireNotNull(attitudeAvailableListener)
        attitudeAvailableListener.onAttitudeAvailable(
            absoluteAttitudeEstimatorSpy,
            attitude,
            0L,
            null,
            null,
            null,
            c
        )

        // check
        val initialized2: Boolean? = estimator.getPrivateProperty("initialized")
        requireNotNull(initialized2)
        assertTrue(initialized2)

        val initialFrame: ECEFFrame? = estimator.getPrivateProperty("initialFrame")
        requireNotNull(initialFrame)
        assertEquals(c, initialFrame.coordinateTransformation)

        val nedPosition = location.toNEDPosition()
        val nedVelocity = NEDVelocity()
        val ecefPosition = ECEFPosition()
        val ecefVelocity = ECEFVelocity()
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
            nedPosition,
            nedVelocity,
            ecefPosition,
            ecefVelocity
        )

        assertEquals(ecefPosition, initialFrame.ecefPosition)

        val previousFrame: ECEFFrame? = estimator.getPrivateProperty("previousFrame")
        requireNotNull(previousFrame)
        assertEquals(initialFrame, previousFrame)

        verify { poseAvailableListener wasNot Called }
    }

    @Test
    fun absoluteAttitudeEstimator_whenInitialized_computesCurrentFrameAndNotifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val poseAvailableListener =
            mockk<PoseEstimator.OnPoseAvailableListener>(relaxUnitFun = true)
        val estimator =
            PoseEstimator(
                context,
                location,
                estimateInitialTransformation = false,
                estimatePreviousTransformation = false,
                poseAvailableListener = poseAvailableListener
            )

        val absoluteAttitudeEstimator: FusedGeomagneticAttitudeEstimator? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        val absoluteAttitudeEstimatorSpy = spyk(absoluteAttitudeEstimator)
        every { absoluteAttitudeEstimatorSpy.gyroscopeAverageTimeInterval }.returns(TIME_INTERVAL)

        // set as initialized
        estimator.setPrivateProperty("initialized", true)

        val initialized: Boolean? = estimator.getPrivateProperty("initialized")
        requireNotNull(initialized)
        assertTrue(initialized)

        val startNedPosition = getLocation().toNEDPosition()
        val startNedVelocity = getNEDVelocity()
        val startEcefPosition = ECEFPosition()
        val ecefVelocity = ECEFVelocity()
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
            startNedPosition,
            startNedVelocity,
            startEcefPosition,
            ecefVelocity
        )
        val startAttitude = getAttitude()
        val startC = CoordinateTransformation(
            FrameType.BODY_FRAME,
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME
        )
        startC.fromRotation(startAttitude)
        val initialFrame = ECEFFrame(startEcefPosition, ecefVelocity, startC)
        val previousFrame = ECEFFrame(initialFrame)

        estimator.setPrivateProperty("initialFrame", initialFrame)
        estimator.setPrivateProperty("previousFrame", previousFrame)

        val attitude = getAttitude()
        val c = CoordinateTransformation(
            FrameType.BODY_FRAME,
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME
        )
        c.fromRotation(attitude)

        // set body kinematics
        val randomizer = UniformRandomizer()
        val bodyKinematics = BodyKinematics(
            randomizer.nextDouble(),
            randomizer.nextDouble(),
            randomizer.nextDouble(),
            randomizer.nextDouble(),
            randomizer.nextDouble(),
            randomizer.nextDouble()
        )
        estimator.setPrivateProperty("bodyKinematics", bodyKinematics)

        // execute
        val attitudeAvailableListener = absoluteAttitudeEstimatorSpy.attitudeAvailableListener
        requireNotNull(attitudeAvailableListener)
        attitudeAvailableListener.onAttitudeAvailable(
            absoluteAttitudeEstimatorSpy,
            attitude,
            0L,
            null,
            null,
            null,
            c
        )

        // check
        val currentFrame1 = ECEFFrame()
        ECEFInertialNavigator.navigateECEF(
            TIME_INTERVAL,
            initialFrame,
            bodyKinematics,
            currentFrame1
        )

        val currentFrame2: ECEFFrame? = estimator.getPrivateProperty("currentFrame")
        requireNotNull(currentFrame2)
        assertEquals(currentFrame1, currentFrame2)
        assertEquals(previousFrame, currentFrame2)

        verify(exactly = 1) {
            poseAvailableListener.onPoseAvailable(
                estimator,
                currentFrame2,
                previousFrame,
                initialFrame,
                Quaternion(),
                Quaternion(),
                Quaternion(),
                0L,
                null,
                null
            )
        }
    }

    @Test
    fun absoluteAttitudeEstimator_whenInitializedAndTransformationsEstimated_computesCurrentFrameTransformationsAndNotifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val poseAvailableListener =
            mockk<PoseEstimator.OnPoseAvailableListener>(relaxUnitFun = true)
        val estimator =
            PoseEstimator(
                context,
                location,
                estimateInitialTransformation = true,
                estimatePreviousTransformation = true,
                poseAvailableListener = poseAvailableListener
            )

        val absoluteAttitudeEstimator: FusedGeomagneticAttitudeEstimator? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        val absoluteAttitudeEstimatorSpy = spyk(absoluteAttitudeEstimator)
        every { absoluteAttitudeEstimatorSpy.gyroscopeAverageTimeInterval }.returns(TIME_INTERVAL)

        // set as initialized
        estimator.setPrivateProperty("initialized", true)

        val initialized: Boolean? = estimator.getPrivateProperty("initialized")
        requireNotNull(initialized)
        assertTrue(initialized)

        val startNedPosition = getLocation().toNEDPosition()
        val startNedVelocity = getNEDVelocity()
        val startEcefPosition = ECEFPosition()
        val ecefVelocity = ECEFVelocity()
        NEDtoECEFPositionVelocityConverter.convertNEDtoECEF(
            startNedPosition,
            startNedVelocity,
            startEcefPosition,
            ecefVelocity
        )
        val startAttitude = getAttitude()
        val startC = CoordinateTransformation(
            FrameType.BODY_FRAME,
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME
        )
        startC.fromRotation(startAttitude)
        val initialFrame = ECEFFrame(startEcefPosition, ecefVelocity, startC)
        val previousFrame = ECEFFrame(initialFrame)

        estimator.setPrivateProperty("initialFrame", initialFrame)
        estimator.setPrivateProperty("previousFrame", previousFrame)

        val attitude = getAttitude()
        val c = CoordinateTransformation(
            FrameType.BODY_FRAME,
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME
        )
        c.fromRotation(attitude)

        // set body kinematics
        val randomizer = UniformRandomizer()
        val bodyKinematics = BodyKinematics(
            randomizer.nextDouble(),
            randomizer.nextDouble(),
            randomizer.nextDouble(),
            randomizer.nextDouble(),
            randomizer.nextDouble(),
            randomizer.nextDouble()
        )
        estimator.setPrivateProperty("bodyKinematics", bodyKinematics)

        // execute
        val attitudeAvailableListener = absoluteAttitudeEstimatorSpy.attitudeAvailableListener
        requireNotNull(attitudeAvailableListener)
        attitudeAvailableListener.onAttitudeAvailable(
            absoluteAttitudeEstimatorSpy,
            attitude,
            0L,
            null,
            null,
            null,
            c
        )

        // check
        val currentFrame1 = ECEFFrame()
        ECEFInertialNavigator.navigateECEF(
            TIME_INTERVAL,
            initialFrame,
            bodyKinematics,
            currentFrame1
        )

        val currentFrame: ECEFFrame? = estimator.getPrivateProperty("currentFrame")
        requireNotNull(currentFrame)
        assertEquals(currentFrame1, currentFrame)
        assertEquals(previousFrame, currentFrame)

        val initialTransformationSlot1 = slot<EuclideanTransformation3D>()
        val previousTransformationSlot1 = slot<EuclideanTransformation3D>()
        verify(exactly = 1) {
            poseAvailableListener.onPoseAvailable(
                estimator,
                currentFrame,
                previousFrame,
                initialFrame,
                Quaternion(),
                Quaternion(),
                Quaternion(),
                0L,
                capture(initialTransformationSlot1),
                capture(previousTransformationSlot1)
            )
        }

        val initialTransformation1 = initialTransformationSlot1.captured
        val previousTransformation1 = previousTransformationSlot1.captured

        assertEquals(initialTransformation1.asMatrix(), previousTransformation1.asMatrix())

        val initialPoint = startEcefPosition.position
        val currentPoint1 = currentFrame.position
        val transformedPoint1 = initialTransformation1.transformAndReturnNew(initialPoint)
        assertEquals(currentPoint1, transformedPoint1)

        // execute again
        val previousFrameCopy = ECEFFrame(currentFrame)
        attitudeAvailableListener.onAttitudeAvailable(
            absoluteAttitudeEstimatorSpy,
            attitude,
            0L,
            null,
            null,
            null,
            c
        )

        // check
        val currentFrame2 = ECEFFrame()
        ECEFInertialNavigator.navigateECEF(
            TIME_INTERVAL,
            previousFrameCopy,
            bodyKinematics,
            currentFrame2
        )

        assertEquals(currentFrame2, currentFrame)
        assertEquals(previousFrame, currentFrame)

        val initialTransformationSlot2 = mutableListOf<EuclideanTransformation3D>()
        val previousTransformationSlot2 = mutableListOf<EuclideanTransformation3D>()
        verify(exactly = 2) {
            poseAvailableListener.onPoseAvailable(
                estimator,
                currentFrame2,
                previousFrame,
                initialFrame,
                Quaternion(),
                Quaternion(),
                Quaternion(),
                0L,
                capture(initialTransformationSlot2),
                capture(previousTransformationSlot2)
            )
        }

        val initialTransformation2 = initialTransformationSlot2.last()
        val previousTransformation2 = previousTransformationSlot2.last()

        assertNotEquals(initialTransformation2.asMatrix(), previousTransformation2.asMatrix())

        val currentPoint2 = currentFrame.position
        val transformedPoint2 = initialTransformation2.transformAndReturnNew(initialPoint)
        assertEquals(currentPoint2, transformedPoint2)
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

        const val ABSOLUTE_ERROR = 1e-6

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

        fun getNEDVelocity(): NEDVelocity {
            val randomizer = UniformRandomizer()
            return NEDVelocity(
                randomizer.nextDouble(),
                randomizer.nextDouble(),
                randomizer.nextDouble()
            )
        }
    }
}