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
import com.irurueta.android.navigation.inertial.*
import com.irurueta.android.navigation.inertial.collectors.*
import com.irurueta.android.navigation.inertial.estimators.attitude.AbsoluteAttitudeEstimator
import com.irurueta.android.navigation.inertial.estimators.attitude.FusedGeomagneticAttitudeEstimator
import com.irurueta.android.navigation.inertial.estimators.attitude.GravityEstimator
import com.irurueta.android.navigation.inertial.estimators.filter.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.MeanAveragingFilter
import com.irurueta.geometry.EuclideanTransformation3D
import com.irurueta.geometry.InhomogeneousPoint3D
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.*
import com.irurueta.navigation.frames.converters.ECEFtoNEDFrameConverter
import com.irurueta.navigation.frames.converters.NEDtoECEFFrameConverter
import com.irurueta.navigation.inertial.BodyKinematics
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.AngularSpeedTriad
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel
import com.irurueta.statistics.UniformRandomizer
import io.mockk.*
import org.junit.After
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import java.util.*

@RunWith(RobolectricTestRunner::class)
class EcefAbsolutePoseEstimatorTest {

    @After
    fun tearDown() {
        unmockkAll()
        clearAllMocks()
    }

    @Test
    fun constructor_whenRequiredProperties_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = EcefAbsolutePoseEstimator(context, location)

        // check
        assertSame(context, estimator.context)
        assertSame(location, estimator.initialLocation)
        assertEquals(NEDVelocity(), estimator.initialVelocity)
        assertEquals(SensorDelay.GAME, estimator.sensorDelay)
        assertFalse(estimator.useAccelerometerForAttitudeEstimation)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            estimator.accelerometerSensorType
        )
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED,
            estimator.magnetometerSensorType
        )
        assertNotNull(estimator.accelerometerAveragingFilter)
        assertTrue(estimator.accelerometerAveragingFilter is LowPassAveragingFilter)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE_UNCALIBRATED,
            estimator.gyroscopeSensorType
        )
        assertNull(estimator.worldMagneticModel)
        assertNotNull(estimator.timestamp)
        assertFalse(estimator.useWorldMagneticModel)
        assertTrue(estimator.useAccurateLevelingEstimator)
        assertTrue(estimator.useAccurateRelativeGyroscopeAttitudeEstimator)
        assertTrue(estimator.useAccurateAttitudeEstimator)
        assertTrue(estimator.useIndirectAttitudeInterpolation)
        assertEquals(
            FusedGeomagneticAttitudeEstimator.DEFAULT_INTERPOLATION_VALUE,
            estimator.attitudeInterpolationValue,
            0.0
        )
        assertEquals(
            FusedGeomagneticAttitudeEstimator.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            estimator.attitudeIndirectInterpolationWeight,
            0.0
        )
        assertEquals(
            FusedGeomagneticAttitudeEstimator.DEFAULT_OUTLIER_THRESHOLD,
            estimator.attitudeOutlierThreshold,
            0.0
        )
        assertEquals(
            FusedGeomagneticAttitudeEstimator.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            estimator.attitudeOutlierPanicThreshold,
            0.0
        )
        assertEquals(
            FusedGeomagneticAttitudeEstimator.DEFAULT_PANIC_COUNTER_THRESHOLD,
            estimator.attitudePanicCounterThreshold
        )
        assertFalse(estimator.estimatePoseTransformation)
        assertNull(estimator.poseAvailableListener)
        assertNull(estimator.accelerometerMeasurementListener)
        assertNull(estimator.gyroscopeMeasurementListener)
        assertNull(estimator.magnetometerMeasurementListener)
        assertNull(estimator.gravityEstimationListener)
        assertFalse(estimator.running)
        assertEquals(0.0, estimator.averageTimeInterval, 0.0)
        assertTrue(estimator.useLeveledRelativeAttitudeRespectStart)
    }

    @Test
    fun constructor_whenAllProperties_setsExpectedValues() {
        val location = getLocation()
        val initialVelocity = getNEDVelocity()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val accelerometerAveragingFilter = MeanAveragingFilter()
        val worldMagneticModel = WorldMagneticModel()
        val timestamp = Date()
        val poseAvailableListener = mockk<EcefAbsolutePoseEstimator.OnPoseAvailableListener>()
        val accelerometerMeasurementListener =
            mockk<AccelerometerSensorCollector.OnMeasurementListener>()
        val gyroscopeMeasurementListener = mockk<GyroscopeSensorCollector.OnMeasurementListener>()
        val magnetometerMeasurementListener =
            mockk<MagnetometerSensorCollector.OnMeasurementListener>()
        val gravityEstimationListener = mockk<GravityEstimator.OnEstimationListener>()
        val estimator = EcefAbsolutePoseEstimator(
            context,
            location,
            initialVelocity,
            SensorDelay.NORMAL,
            useAccelerometerForAttitudeEstimation = true,
            AccelerometerSensorType.ACCELEROMETER,
            MagnetometerSensorType.MAGNETOMETER,
            accelerometerAveragingFilter,
            GyroscopeSensorType.GYROSCOPE,
            worldMagneticModel,
            timestamp,
            useWorldMagneticModel = true,
            useAccurateLevelingEstimator = false,
            useAccurateRelativeGyroscopeAttitudeEstimator = false,
            useAccurateAttitudeEstimator = false,
            estimatePoseTransformation = true,
            poseAvailableListener,
            accelerometerMeasurementListener,
            gyroscopeMeasurementListener,
            magnetometerMeasurementListener,
            gravityEstimationListener
        )

        // check
        assertSame(context, estimator.context)
        assertSame(location, estimator.initialLocation)
        assertSame(initialVelocity, estimator.initialVelocity)
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertTrue(estimator.useAccelerometerForAttitudeEstimation)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            estimator.accelerometerSensorType
        )
        assertEquals(
            MagnetometerSensorType.MAGNETOMETER,
            estimator.magnetometerSensorType
        )
        assertSame(accelerometerAveragingFilter, estimator.accelerometerAveragingFilter)
        assertEquals(
            GyroscopeSensorType.GYROSCOPE,
            estimator.gyroscopeSensorType
        )
        assertSame(worldMagneticModel, estimator.worldMagneticModel)
        assertSame(timestamp, estimator.timestamp)
        assertTrue(estimator.useWorldMagneticModel)
        assertFalse(estimator.useAccurateLevelingEstimator)
        assertFalse(estimator.useAccurateRelativeGyroscopeAttitudeEstimator)
        assertFalse(estimator.useAccurateAttitudeEstimator)
        assertTrue(estimator.useIndirectAttitudeInterpolation)
        assertEquals(
            FusedGeomagneticAttitudeEstimator.DEFAULT_INTERPOLATION_VALUE,
            estimator.attitudeInterpolationValue,
            0.0
        )
        assertEquals(
            FusedGeomagneticAttitudeEstimator.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            estimator.attitudeIndirectInterpolationWeight,
            0.0
        )
        assertEquals(
            FusedGeomagneticAttitudeEstimator.DEFAULT_OUTLIER_THRESHOLD,
            estimator.attitudeOutlierThreshold,
            0.0
        )
        assertEquals(
            FusedGeomagneticAttitudeEstimator.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            estimator.attitudeOutlierPanicThreshold,
            0.0
        )
        assertEquals(
            FusedGeomagneticAttitudeEstimator.DEFAULT_PANIC_COUNTER_THRESHOLD,
            estimator.attitudePanicCounterThreshold
        )
        assertTrue(estimator.estimatePoseTransformation)
        assertSame(poseAvailableListener, estimator.poseAvailableListener)
        assertSame(accelerometerMeasurementListener, estimator.accelerometerMeasurementListener)
        assertSame(gyroscopeMeasurementListener, estimator.gyroscopeMeasurementListener)
        assertSame(magnetometerMeasurementListener, estimator.magnetometerMeasurementListener)
        assertSame(gravityEstimationListener, estimator.gravityEstimationListener)
        assertFalse(estimator.running)
        assertEquals(0.0, estimator.averageTimeInterval, 0.0)
        assertTrue(estimator.useLeveledRelativeAttitudeRespectStart)
    }

    @Test
    fun poseAvailableListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = EcefAbsolutePoseEstimator(context, location)

        // check default value
        assertNull(estimator.poseAvailableListener)

        // set new value
        val poseAvailableListener = mockk<EcefAbsolutePoseEstimator.OnPoseAvailableListener>()
        estimator.poseAvailableListener = poseAvailableListener

        // check
        assertSame(poseAvailableListener, estimator.poseAvailableListener)
    }

    @Test
    fun accelerometerMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = EcefAbsolutePoseEstimator(context, location)

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
        val location = getLocation()
        val estimator = EcefAbsolutePoseEstimator(context, location)

        // check default value
        assertNull(estimator.gyroscopeMeasurementListener)

        // set new value
        val gyroscopeMeasurementListener = mockk<GyroscopeSensorCollector.OnMeasurementListener>()
        estimator.gyroscopeMeasurementListener = gyroscopeMeasurementListener

        // check
        assertSame(gyroscopeMeasurementListener, estimator.gyroscopeMeasurementListener)
    }

    @Test
    fun magnetometerMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = EcefAbsolutePoseEstimator(context, location)

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
    fun gravityEstimationListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = EcefAbsolutePoseEstimator(context, location)

        // check default value
        assertNull(estimator.gravityEstimationListener)

        // set new value
        val gravityEstimationListener = mockk<GravityEstimator.OnEstimationListener>()
        estimator.gravityEstimationListener = gravityEstimationListener

        // check
        assertSame(gravityEstimationListener, estimator.gravityEstimationListener)
    }

    @Test
    fun location_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location1 = getLocation()
        val estimator = EcefAbsolutePoseEstimator(context, location1)

        val absoluteAttitudeEstimator: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)

        // check default values
        assertSame(location1, estimator.initialLocation)
        assertSame(location1, absoluteAttitudeEstimator.location)
        assertFalse(estimator.running)

        // set new value
        val location2 = getLocation()
        estimator.initialLocation = location2

        // check
        assertSame(location2, estimator.initialLocation)
        assertSame(location2, absoluteAttitudeEstimator.location)
    }

    @Test
    fun worldMagneticModel_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = EcefAbsolutePoseEstimator(context, location)

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
        val estimator = EcefAbsolutePoseEstimator(context, location)

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
        val estimator = EcefAbsolutePoseEstimator(context, location)

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
        val estimator = EcefAbsolutePoseEstimator(context, location)

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
        val estimator = EcefAbsolutePoseEstimator(context, location)

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
        val estimator = EcefAbsolutePoseEstimator(context, location)

        val absoluteAttitudeEstimator: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)

        // check default value
        assertTrue(estimator.useAccurateLevelingEstimator)
        assertTrue(absoluteAttitudeEstimator.useAccurateLevelingEstimator)
        assertFalse(estimator.running)
        assertSame(location, estimator.initialLocation)

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
        val estimator = EcefAbsolutePoseEstimator(context, location)

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
        val estimator = EcefAbsolutePoseEstimator(context, location)

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
        val estimator = EcefAbsolutePoseEstimator(context, location)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        // set new value
        estimator.useAccurateRelativeGyroscopeAttitudeEstimator = true
    }

    @Test
    fun useIndirectAttitudeInterpolation_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = EcefAbsolutePoseEstimator(context, location)

        // check default value
        assertTrue(estimator.useIndirectAttitudeInterpolation)

        val absoluteAttitudeEstimator: FusedGeomagneticAttitudeEstimator? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        assertTrue(absoluteAttitudeEstimator.useIndirectInterpolation)

        // set new value
        estimator.useIndirectAttitudeInterpolation = false

        // check
        assertFalse(estimator.useIndirectAttitudeInterpolation)
        assertFalse(absoluteAttitudeEstimator.useIndirectInterpolation)
    }

    @Test
    fun attitudeInterpolationValue_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = EcefAbsolutePoseEstimator(context, location)

        // check default value
        assertEquals(
            FusedGeomagneticAttitudeEstimator.DEFAULT_INTERPOLATION_VALUE,
            estimator.attitudeInterpolationValue,
            0.0
        )

        val absoluteAttitudeEstimator: FusedGeomagneticAttitudeEstimator? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        assertEquals(
            FusedGeomagneticAttitudeEstimator.DEFAULT_INTERPOLATION_VALUE,
            absoluteAttitudeEstimator.interpolationValue,
            0.0
        )

        // set new value
        val randomizer = UniformRandomizer()
        val attitudeInterpolationValue = randomizer.nextDouble()
        estimator.attitudeInterpolationValue = attitudeInterpolationValue

        // check
        assertEquals(attitudeInterpolationValue, estimator.attitudeInterpolationValue, 0.0)
        assertEquals(attitudeInterpolationValue, absoluteAttitudeEstimator.interpolationValue, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun attitudeInterpolationValue_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = EcefAbsolutePoseEstimator(context, location)

        estimator.attitudeInterpolationValue = -1.0
    }

    @Test
    fun attitudeIndirectInterpolationWeight_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = EcefAbsolutePoseEstimator(context, location)

        // check default value
        assertEquals(
            FusedGeomagneticAttitudeEstimator.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            estimator.attitudeIndirectInterpolationWeight,
            0.0
        )

        val absoluteAttitudeEstimator: FusedGeomagneticAttitudeEstimator? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        assertEquals(
            FusedGeomagneticAttitudeEstimator.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            absoluteAttitudeEstimator.indirectInterpolationWeight,
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
            absoluteAttitudeEstimator.indirectInterpolationWeight,
            0.0
        )
    }

    @Test(expected = IllegalArgumentException::class)
    fun attitudeIndirectInterpolationWeight_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = EcefAbsolutePoseEstimator(context, location)

        estimator.attitudeIndirectInterpolationWeight = 0.0
    }

    @Test
    fun attitudeOutlierThreshold_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = EcefAbsolutePoseEstimator(context, location)

        // check default value
        assertEquals(
            FusedGeomagneticAttitudeEstimator.DEFAULT_OUTLIER_THRESHOLD,
            estimator.attitudeOutlierThreshold,
            0.0
        )

        val absoluteAttitudeEstimator: FusedGeomagneticAttitudeEstimator? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        assertEquals(
            FusedGeomagneticAttitudeEstimator.DEFAULT_OUTLIER_THRESHOLD,
            absoluteAttitudeEstimator.outlierThreshold,
            0.0
        )

        // set new value
        val randomizer = UniformRandomizer()
        val attitudeOutlierThreshold = randomizer.nextDouble()
        estimator.attitudeOutlierThreshold = attitudeOutlierThreshold

        // check
        assertEquals(attitudeOutlierThreshold, estimator.attitudeOutlierThreshold, 0.0)
        assertEquals(attitudeOutlierThreshold, absoluteAttitudeEstimator.outlierThreshold, 0.0)
    }

    @Test(expected = IllegalArgumentException::class)
    fun attitudeOutlierThreshold_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = EcefAbsolutePoseEstimator(context, location)

        estimator.attitudeOutlierThreshold = -1.0
    }

    @Test(expected = IllegalStateException::class)
    fun attitudeOutlierThreshold_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = EcefAbsolutePoseEstimator(context, location)

        // set as running
        val absoluteAttitudeEstimator: FusedGeomagneticAttitudeEstimator? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        absoluteAttitudeEstimator.setPrivateProperty("running", true)

        // attempt setting new value
        val randomizer = UniformRandomizer()
        val attitudeOutlierThreshold = randomizer.nextDouble()
        estimator.attitudeOutlierThreshold = attitudeOutlierThreshold
    }

    @Test
    fun attitudeOutlierPanicThreshold_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = EcefAbsolutePoseEstimator(context, location)

        // check default value
        assertEquals(
            FusedGeomagneticAttitudeEstimator.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            estimator.attitudeOutlierPanicThreshold,
            0.0
        )

        val absoluteAttitudeEstimator: FusedGeomagneticAttitudeEstimator? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        assertEquals(
            FusedGeomagneticAttitudeEstimator.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            absoluteAttitudeEstimator.outlierPanicThreshold,
            0.0
        )

        // set new value
        val randomizer = UniformRandomizer()
        val attitudeOutlierPanicThreshold = randomizer.nextDouble()
        estimator.attitudeOutlierPanicThreshold = attitudeOutlierPanicThreshold

        // check
        assertEquals(attitudeOutlierPanicThreshold, estimator.attitudeOutlierPanicThreshold, 0.0)
        assertEquals(
            attitudeOutlierPanicThreshold,
            absoluteAttitudeEstimator.outlierPanicThreshold,
            0.0
        )
    }

    @Test(expected = IllegalArgumentException::class)
    fun attitudeOutlierPanicThreshold_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = EcefAbsolutePoseEstimator(context, location)

        estimator.attitudeOutlierPanicThreshold = -1.0
    }

    @Test(expected = IllegalStateException::class)
    fun attitudeOutlierPanicThreshold_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = EcefAbsolutePoseEstimator(context, location)

        // set as running
        val absoluteAttitudeEstimator: FusedGeomagneticAttitudeEstimator? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        absoluteAttitudeEstimator.setPrivateProperty("running", true)

        // attempt setting new value
        val randomizer = UniformRandomizer()
        val attitudeOutlierPanicThreshold = randomizer.nextDouble()
        estimator.attitudeOutlierPanicThreshold = attitudeOutlierPanicThreshold
    }

    @Test
    fun attitudePanicCounterThreshold_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = EcefAbsolutePoseEstimator(context, location)

        // check default value
        assertEquals(
            FusedGeomagneticAttitudeEstimator.DEFAULT_PANIC_COUNTER_THRESHOLD,
            estimator.attitudePanicCounterThreshold
        )
        val absoluteAttitudeEstimator: FusedGeomagneticAttitudeEstimator? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        assertEquals(
            FusedGeomagneticAttitudeEstimator.DEFAULT_PANIC_COUNTER_THRESHOLD,
            absoluteAttitudeEstimator.panicCounterThreshold
        )

        // set new value
        estimator.attitudePanicCounterThreshold = 1

        // check
        assertEquals(1, estimator.attitudePanicCounterThreshold)
        assertEquals(1, absoluteAttitudeEstimator.panicCounterThreshold)
    }

    @Test
    fun averageTimeInterval_callsInternalAttitudeEstimator() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = EcefAbsolutePoseEstimator(context, location)

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
        val estimator = EcefAbsolutePoseEstimator(context, location)

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
    fun start_whenUseAccurateAttitudeEstimatorEnabledNotRunningAndInternalEstimatorFails_stopsAndReturnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = EcefAbsolutePoseEstimator(context, location)

        val absoluteAttitudeEstimator: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        val absoluteAttitudeEstimatorSpy = spyk(absoluteAttitudeEstimator)
        every { absoluteAttitudeEstimatorSpy.start() }.returns(false)
        estimator.setPrivateProperty("absoluteAttitudeEstimator", absoluteAttitudeEstimatorSpy)

        val absoluteAttitudeEstimator2: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator2")
        requireNotNull(absoluteAttitudeEstimator2)
        val absoluteAttitudeEstimator2Spy = spyk(absoluteAttitudeEstimator2)
        every { absoluteAttitudeEstimator2Spy.start() }.returns(false)
        estimator.setPrivateProperty("absoluteAttitudeEstimator2", absoluteAttitudeEstimator2Spy)

        val accelerometerSensorCollector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.start() }.returns(true)
        estimator.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        assertFalse(estimator.running)
        assertTrue(estimator.useAccurateAttitudeEstimator)

        // start
        assertFalse(estimator.start())
        assertFalse(estimator.running)

        verify(exactly = 0) { absoluteAttitudeEstimatorSpy.start() }
        verify(exactly = 0) { absoluteAttitudeEstimatorSpy.stop() }

        verify(exactly = 1) { absoluteAttitudeEstimator2Spy.start() }
        verify(exactly = 1) { absoluteAttitudeEstimator2Spy.stop() }

        verify(exactly = 1) { accelerometerSensorCollectorSpy.stop() }
    }

    @Test
    fun start_whenUseAccurateAttitudeEstimatorDisabledNotRunningAndInternalEstimatorFails_stopsAndReturnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator =
            EcefAbsolutePoseEstimator(context, location, useAccurateAttitudeEstimator = false)

        val absoluteAttitudeEstimator: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        val absoluteAttitudeEstimatorSpy = spyk(absoluteAttitudeEstimator)
        every { absoluteAttitudeEstimatorSpy.start() }.returns(false)
        estimator.setPrivateProperty("absoluteAttitudeEstimator", absoluteAttitudeEstimatorSpy)

        val absoluteAttitudeEstimator2: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator2")
        requireNotNull(absoluteAttitudeEstimator2)
        val absoluteAttitudeEstimator2Spy = spyk(absoluteAttitudeEstimator2)
        every { absoluteAttitudeEstimator2Spy.start() }.returns(false)
        estimator.setPrivateProperty("absoluteAttitudeEstimator2", absoluteAttitudeEstimator2Spy)

        val accelerometerSensorCollector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.start() }.returns(true)
        estimator.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        assertFalse(estimator.running)
        assertFalse(estimator.useAccurateAttitudeEstimator)

        // start
        assertFalse(estimator.start())
        assertFalse(estimator.running)

        verify(exactly = 1) { absoluteAttitudeEstimatorSpy.start() }
        verify(exactly = 1) { absoluteAttitudeEstimatorSpy.stop() }

        verify(exactly = 0) { absoluteAttitudeEstimator2Spy.start() }
        verify(exactly = 0) { absoluteAttitudeEstimator2Spy.stop() }

        verify(exactly = 1) { accelerometerSensorCollectorSpy.stop() }
    }

    @Test
    fun start_whenAccelerometerSensorCollectorFails_stopsAndReturnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = EcefAbsolutePoseEstimator(context, location)

        val absoluteAttitudeEstimator: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        val absoluteAttitudeEstimatorSpy = spyk(absoluteAttitudeEstimator)
        every { absoluteAttitudeEstimatorSpy.start() }.returns(false)
        estimator.setPrivateProperty("absoluteAttitudeEstimator", absoluteAttitudeEstimatorSpy)

        val absoluteAttitudeEstimator2: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator2")
        requireNotNull(absoluteAttitudeEstimator2)
        val absoluteAttitudeEstimator2Spy = spyk(absoluteAttitudeEstimator2)
        every { absoluteAttitudeEstimator2Spy.start() }.returns(false)
        estimator.setPrivateProperty("absoluteAttitudeEstimator2", absoluteAttitudeEstimator2Spy)

        val accelerometerSensorCollector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.start() }.returns(false)
        estimator.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        assertFalse(estimator.running)
        assertTrue(estimator.useAccurateAttitudeEstimator)

        // start
        assertFalse(estimator.start())
        assertFalse(estimator.running)

        verify(exactly = 0) { absoluteAttitudeEstimatorSpy.start() }
        verify(exactly = 0) { absoluteAttitudeEstimatorSpy.stop() }

        verify(exactly = 1) { absoluteAttitudeEstimator2Spy.start() }
        verify(exactly = 1) { absoluteAttitudeEstimator2Spy.stop() }

        verify(exactly = 1) { accelerometerSensorCollectorSpy.stop() }
    }

    @Test
    fun start_whenUseAccurateAttitudeEstimatorEnabledNotRunningAndInternalEstimatorSucceeds_returnsTrue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = EcefAbsolutePoseEstimator(context, location)

        val absoluteAttitudeEstimator: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        val absoluteAttitudeEstimatorSpy = spyk(absoluteAttitudeEstimator)
        every { absoluteAttitudeEstimatorSpy.start() }.returns(true)
        estimator.setPrivateProperty("absoluteAttitudeEstimator", absoluteAttitudeEstimatorSpy)

        val absoluteAttitudeEstimator2: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator2")
        requireNotNull(absoluteAttitudeEstimator2)
        val absoluteAttitudeEstimator2Spy = spyk(absoluteAttitudeEstimator2)
        every { absoluteAttitudeEstimator2Spy.start() }.returns(true)
        estimator.setPrivateProperty("absoluteAttitudeEstimator2", absoluteAttitudeEstimator2Spy)

        val accelerometerSensorCollector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.start() }.returns(true)
        estimator.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        assertFalse(estimator.running)

        // start
        assertTrue(estimator.start())
        assertTrue(estimator.running)

        verify(exactly = 0) { absoluteAttitudeEstimatorSpy.start() }
        verify(exactly = 0) { absoluteAttitudeEstimatorSpy.stop() }

        verify(exactly = 1) { absoluteAttitudeEstimator2Spy.start() }
        verify(exactly = 0) { absoluteAttitudeEstimator2Spy.stop() }

        verify(exactly = 1) { accelerometerSensorCollectorSpy.start() }
    }

    @Test
    fun start_whenUseAccurateAttitudeEstimatorDisabledNotRunningAndInternalEstimatorSucceeds_returnsTrue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator =
            EcefAbsolutePoseEstimator(context, location, useAccurateAttitudeEstimator = false)

        val absoluteAttitudeEstimator: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        val absoluteAttitudeEstimatorSpy = spyk(absoluteAttitudeEstimator)
        every { absoluteAttitudeEstimatorSpy.start() }.returns(true)
        estimator.setPrivateProperty("absoluteAttitudeEstimator", absoluteAttitudeEstimatorSpy)

        val absoluteAttitudeEstimator2: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator2")
        requireNotNull(absoluteAttitudeEstimator2)
        val absoluteAttitudeEstimator2Spy = spyk(absoluteAttitudeEstimator2)
        every { absoluteAttitudeEstimator2Spy.start() }.returns(true)
        estimator.setPrivateProperty("absoluteAttitudeEstimator2", absoluteAttitudeEstimator2Spy)

        val accelerometerSensorCollector: AccelerometerSensorCollector? =
            estimator.getPrivateProperty("accelerometerSensorCollector")
        requireNotNull(accelerometerSensorCollector)
        val accelerometerSensorCollectorSpy = spyk(accelerometerSensorCollector)
        every { accelerometerSensorCollectorSpy.start() }.returns(true)
        estimator.setPrivateProperty("accelerometerSensorCollector", accelerometerSensorCollectorSpy)

        assertFalse(estimator.running)

        // start
        assertTrue(estimator.start())
        assertTrue(estimator.running)

        verify(exactly = 1) { absoluteAttitudeEstimatorSpy.start() }
        verify(exactly = 0) { absoluteAttitudeEstimatorSpy.stop() }

        verify(exactly = 0) { absoluteAttitudeEstimator2Spy.start() }
        verify(exactly = 0) { absoluteAttitudeEstimator2Spy.stop() }

        verify(exactly = 1) { accelerometerSensorCollectorSpy.start() }
    }

    @Test
    fun start_whenNotRunningAndUseAccurateAttitudeEstimatorEnabled_resetsInitialized() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = EcefAbsolutePoseEstimator(context, location)

        // set as initialized
        estimator.setPrivateProperty("initializedFrame", true)
        val initializedFrame1: Boolean? = estimator.getPrivateProperty("initializedFrame")
        requireNotNull(initializedFrame1)
        assertTrue(initializedFrame1)

        val absoluteAttitudeEstimator: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        val absoluteAttitudeEstimatorSpy = spyk(absoluteAttitudeEstimator)
        every { absoluteAttitudeEstimatorSpy.start() }.returns(false)
        estimator.setPrivateProperty("absoluteAttitudeEstimator", absoluteAttitudeEstimatorSpy)

        val absoluteAttitudeEstimator2: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator2")
        requireNotNull(absoluteAttitudeEstimator2)
        val absoluteAttitudeEstimator2Spy = spyk(absoluteAttitudeEstimator2)
        every { absoluteAttitudeEstimator2Spy.start() }.returns(false)
        estimator.setPrivateProperty("absoluteAttitudeEstimator2", absoluteAttitudeEstimator2Spy)

        assertFalse(estimator.running)
        assertTrue(estimator.useAccurateAttitudeEstimator)

        // start
        assertFalse(estimator.start())
        assertFalse(estimator.running)

        verify(exactly = 0) { absoluteAttitudeEstimatorSpy.start() }
        verify(exactly = 0) { absoluteAttitudeEstimatorSpy.stop() }

        verify(exactly = 1) { absoluteAttitudeEstimator2Spy.start() }
        verify(exactly = 1) { absoluteAttitudeEstimator2Spy.stop() }

        val initializedFrame2: Boolean? = estimator.getPrivateProperty("initializedFrame")
        requireNotNull(initializedFrame2)
        assertFalse(initializedFrame2)
    }

    @Test
    fun start_whenNotRunningAndUseAccurateAttitudeEstimatorDisabled_resetsInitialized() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator =
            EcefAbsolutePoseEstimator(context, location, useAccurateAttitudeEstimator = false)

        // set as initialized
        estimator.setPrivateProperty("initializedFrame", true)
        val initializedFrame1: Boolean? = estimator.getPrivateProperty("initializedFrame")
        requireNotNull(initializedFrame1)
        assertTrue(initializedFrame1)

        val absoluteAttitudeEstimator: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        val absoluteAttitudeEstimatorSpy = spyk(absoluteAttitudeEstimator)
        every { absoluteAttitudeEstimatorSpy.start() }.returns(false)
        estimator.setPrivateProperty("absoluteAttitudeEstimator", absoluteAttitudeEstimatorSpy)

        val absoluteAttitudeEstimator2: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator2")
        requireNotNull(absoluteAttitudeEstimator2)
        val absoluteAttitudeEstimator2Spy = spyk(absoluteAttitudeEstimator2)
        every { absoluteAttitudeEstimator2Spy.start() }.returns(false)
        estimator.setPrivateProperty("absoluteAttitudeEstimator2", absoluteAttitudeEstimator2Spy)

        assertFalse(estimator.running)
        assertFalse(estimator.useAccurateAttitudeEstimator)

        // start
        assertFalse(estimator.start())
        assertFalse(estimator.running)

        verify(exactly = 1) { absoluteAttitudeEstimatorSpy.start() }
        verify(exactly = 1) { absoluteAttitudeEstimatorSpy.stop() }

        verify(exactly = 0) { absoluteAttitudeEstimator2Spy.start() }
        verify(exactly = 0) { absoluteAttitudeEstimator2Spy.stop() }

        val initializedFrame2: Boolean? = estimator.getPrivateProperty("initializedFrame")
        requireNotNull(initializedFrame2)
        assertFalse(initializedFrame2)
    }

    @Test
    fun stop_whenUseAccurateAttitudeEstimatorEnabled_callsInternalEstimatorAndSetsAsNotRunning() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = EcefAbsolutePoseEstimator(context, location)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        val absoluteAttitudeEstimator: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        val absoluteAttitudeEstimatorSpy = spyk(absoluteAttitudeEstimator)
        estimator.setPrivateProperty("absoluteAttitudeEstimator", absoluteAttitudeEstimatorSpy)

        val absoluteAttitudeEstimator2: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator2")
        requireNotNull(absoluteAttitudeEstimator2)
        val absoluteAttitudeEstimator2Spy = spyk(absoluteAttitudeEstimator2)
        estimator.setPrivateProperty("absoluteAttitudeEstimator2", absoluteAttitudeEstimator2Spy)

        estimator.stop()

        // check
        assertFalse(estimator.running)
        verify(exactly = 0) { absoluteAttitudeEstimatorSpy.stop() }
        verify(exactly = 1) { absoluteAttitudeEstimator2Spy.stop() }
    }

    @Test
    fun stop_whenUseAccurateAttitudeEstimatorDisabled_callsInternalEstimatorAndSetsAsNotRunning() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator =
            EcefAbsolutePoseEstimator(context, location, useAccurateAttitudeEstimator = false)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        val absoluteAttitudeEstimator: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        val absoluteAttitudeEstimatorSpy = spyk(absoluteAttitudeEstimator)
        estimator.setPrivateProperty("absoluteAttitudeEstimator", absoluteAttitudeEstimatorSpy)

        val absoluteAttitudeEstimator2: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator2")
        requireNotNull(absoluteAttitudeEstimator2)
        val absoluteAttitudeEstimator2Spy = spyk(absoluteAttitudeEstimator2)
        estimator.setPrivateProperty("absoluteAttitudeEstimator2", absoluteAttitudeEstimator2Spy)

        estimator.stop()

        // check
        assertFalse(estimator.running)
        verify(exactly = 1) { absoluteAttitudeEstimatorSpy.stop() }
        verify(exactly = 0) { absoluteAttitudeEstimator2Spy.stop() }
    }

    @Test
    fun initialize_whenNotInitialized_initializesFrame() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val initialVelocity = getNEDVelocity()
        val estimator = EcefAbsolutePoseEstimator(context, location, initialVelocity)

        val initializedFrame1: Boolean? = estimator.getPrivateProperty("initializedFrame")
        requireNotNull(initializedFrame1)
        assertFalse(initializedFrame1)

        val initialNedFrame: NEDFrame? = estimator.getPrivateProperty("initialNedFrame")
        requireNotNull(initialNedFrame)
        assertEquals(Quaternion(), initialNedFrame.coordinateTransformation.asRotation())
        assertEquals(NEDPosition(), initialNedFrame.position)
        assertEquals(NEDVelocity(), initialNedFrame.velocity)

        val initialEcefFrame: ECEFFrame? = estimator.getPrivateProperty("initialEcefFrame")
        requireNotNull(initialEcefFrame)
        assertEquals(Quaternion(), initialEcefFrame.coordinateTransformation.asRotation())
        assertEquals(ECEFPosition(), initialEcefFrame.ecefPosition)
        assertEquals(ECEFVelocity(), initialEcefFrame.ecefVelocity)

        val initialAttitude: Quaternion? = estimator.getPrivateProperty("initialAttitude")
        requireNotNull(initialAttitude)
        assertEquals(Quaternion(), initialAttitude)

        val q = getAttitude()
        val c = CoordinateTransformation(
            FrameType.BODY_FRAME,
            FrameType.LOCAL_NAVIGATION_FRAME
        )
        c.fromRotation(q)

        val result: Boolean? = estimator.callPrivateFuncWithResult("initialize", q, c)
        requireNotNull(result)
        assertFalse(result)

        // check
        assertEquals(c, initialNedFrame.coordinateTransformation)
        assertEquals(q, initialNedFrame.coordinateTransformation.asRotation())
        assertEquals(location.toNEDPosition(), initialNedFrame.position)
        assertEquals(initialVelocity, initialNedFrame.velocity)

        val expectedInitialEcefFrame =
            NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(initialNedFrame)
        assertEquals(expectedInitialEcefFrame, initialEcefFrame)

        val previousEcefFrame: ECEFFrame? = estimator.getPrivateProperty("previousEcefFrame")
        requireNotNull(previousEcefFrame)
        assertEquals(initialEcefFrame, previousEcefFrame)

        assertEquals(q, initialAttitude)

        val initializedFrame2: Boolean? = estimator.getPrivateProperty("initializedFrame")
        requireNotNull(initializedFrame2)
        assertTrue(initializedFrame2)
    }

    @Test
    fun initialize_whenAlreadyInitialized_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = EcefAbsolutePoseEstimator(context, location)

        estimator.setPrivateProperty("initializedFrame", true)
        val initializedFrame1: Boolean? = estimator.getPrivateProperty("initializedFrame")
        requireNotNull(initializedFrame1)
        assertTrue(initializedFrame1)

        val initialNedFrame: NEDFrame? = estimator.getPrivateProperty("initialNedFrame")
        requireNotNull(initialNedFrame)
        val initialNedFrameSpy = spyk(initialNedFrame)
        estimator.setPrivateProperty("initialNedFrame", initialNedFrameSpy)

        val initialEcefFrame: ECEFFrame? = estimator.getPrivateProperty("initialEcefFrame")
        requireNotNull(initialEcefFrame)
        val initialEcefFrameSpy = spyk(initialEcefFrame)
        estimator.setPrivateProperty("initialEcefFrame", initialEcefFrameSpy)

        val q = getAttitude()
        val c = CoordinateTransformation(
            FrameType.BODY_FRAME,
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME
        )
        c.fromRotation(q)

        val result: Boolean? = estimator.callPrivateFuncWithResult("initialize", q, c)
        requireNotNull(result)
        assertTrue(result)

        // check
        verify { initialNedFrameSpy wasNot Called }
        verify { initialEcefFrameSpy wasNot Called }

        val initializedFrame2: Boolean? = estimator.getPrivateProperty("initializedFrame")
        requireNotNull(initializedFrame2)
        assertTrue(initializedFrame2)
    }

    @Test
    fun computeTransformation_whenUseLeveledRelativeAttitudeRespectStart_computesExpectedTransformation() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val initialNedVelocity = getNEDVelocity()
        val estimator = EcefAbsolutePoseEstimator(context, location, initialNedVelocity)
        estimator.useLeveledRelativeAttitudeRespectStart = true

        val initialNedAttitude = getAttitude()
        val currentNedAttitude = getAttitude()

        val initialNedFrame = NEDFrame(
            location.toNEDPosition(),
            initialNedVelocity,
            CoordinateTransformation(
                initialNedAttitude,
                FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME
            )
        )
        val currentNedFrame = NEDFrame(
            location.toNEDPosition(),
            NEDVelocity(),
            CoordinateTransformation(
                currentNedAttitude,
                FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME
            )
        )
        val initialEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(initialNedFrame)
        val currentEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(currentNedFrame)

        val randomizer = UniformRandomizer()
        val deltaEcefX = randomizer.nextDouble(MIN_DELTA_POS, MAX_DELTA_POS)
        val deltaEcefY = randomizer.nextDouble(MIN_DELTA_POS, MAX_DELTA_POS)
        val deltaEcefZ = randomizer.nextDouble(MIN_DELTA_POS, MAX_DELTA_POS)
        currentEcefFrame.setPosition(
            ECEFPosition(
                initialEcefFrame.x + deltaEcefX,
                initialEcefFrame.y + deltaEcefY,
                initialEcefFrame.z + deltaEcefZ
            )
        )

        val result = EuclideanTransformation3D()
        callPrivateFunc(
            EcefAbsolutePoseEstimator::class,
            estimator,
            "computeTransformation",
            initialEcefFrame,
            currentEcefFrame,
            initialNedAttitude,
            currentNedAttitude,
            result
        )

        val rotation = result.rotation.toQuaternion()
        val initialEulerAngles = initialNedAttitude.toEulerAngles()
        val currentEulerAngles = currentNedAttitude.toEulerAngles()
        val expectedRotation = Quaternion(
            currentEulerAngles[0],
            currentEulerAngles[1],
            currentEulerAngles[2] - initialEulerAngles[2]
        ).combineAndReturnNew(ENUtoNEDConverter.conversionRotation)
        assertEquals(expectedRotation, rotation)

        val inverseInitialEcefRotation =
            initialEcefFrame.coordinateTransformation.asRotation().inverseRotationAndReturnNew()
        val translation =
            ENUtoNEDConverter.conversionRotation.combineAndReturnNew(inverseInitialEcefRotation)
                .rotate(InhomogeneousPoint3D(deltaEcefX, deltaEcefY, deltaEcefZ))
        assertTrue(translation.equals(result.translationPoint, ABSOLUTE_ERROR))
    }

    @Test
    fun computeTransformation_whenNotUseLeveledRelativeAttitudeRespectStart_computesExpectedTransformation() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val initialNedVelocity = getNEDVelocity()
        val estimator = EcefAbsolutePoseEstimator(context, location, initialNedVelocity)
        estimator.useLeveledRelativeAttitudeRespectStart = false

        val initialNedAttitude = getAttitude()
        val currentNedAttitude = getAttitude()

        val initialNedFrame = NEDFrame(
            location.toNEDPosition(),
            initialNedVelocity,
            CoordinateTransformation(
                initialNedAttitude,
                FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME
            )
        )
        val currentNedFrame = NEDFrame(
            location.toNEDPosition(),
            NEDVelocity(),
            CoordinateTransformation(
                currentNedAttitude,
                FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME
            )
        )
        val initialEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(initialNedFrame)
        val currentEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(currentNedFrame)

        val randomizer = UniformRandomizer()
        val deltaEcefX = randomizer.nextDouble(MIN_DELTA_POS, MAX_DELTA_POS)
        val deltaEcefY = randomizer.nextDouble(MIN_DELTA_POS, MAX_DELTA_POS)
        val deltaEcefZ = randomizer.nextDouble(MIN_DELTA_POS, MAX_DELTA_POS)
        currentEcefFrame.setPosition(
            ECEFPosition(
                initialEcefFrame.x + deltaEcefX,
                initialEcefFrame.y + deltaEcefY,
                initialEcefFrame.z + deltaEcefZ
            )
        )

        val result = EuclideanTransformation3D()
        callPrivateFunc(
            EcefAbsolutePoseEstimator::class,
            estimator,
            "computeTransformation",
            initialEcefFrame,
            currentEcefFrame,
            initialNedAttitude,
            currentNedAttitude,
            result
        )

        val rotation = result.rotation.toQuaternion()
        val expectedRotation =
            currentNedAttitude.combineAndReturnNew(ENUtoNEDConverter.conversionRotation)
        assertEquals(expectedRotation, rotation)

        val inverseInitialEcefRotation =
            initialEcefFrame.coordinateTransformation.asRotation().inverseRotationAndReturnNew()
        val translation =
            ENUtoNEDConverter.conversionRotation.combineAndReturnNew(inverseInitialEcefRotation)
                .rotate(InhomogeneousPoint3D(deltaEcefX, deltaEcefY, deltaEcefZ))
        assertTrue(translation.equals(result.translationPoint, ABSOLUTE_ERROR))
    }

    @Test
    fun absoluteAttitudeEstimator_whenAccelerometerMeasurementNoBiasAndNoListener_updatesAcceleration() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = EcefAbsolutePoseEstimator(context, location)

        // check initial value
        val specificForce: AccelerationTriad? = estimator.getPrivateProperty("specificForce")
        requireNotNull(specificForce)
        assertEquals(0.0, specificForce.valueX, 0.0)
        assertEquals(0.0, specificForce.valueY, 0.0)
        assertEquals(0.0, specificForce.valueZ, 0.0)

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
        assertEquals(ax.toDouble(), specificForce.valueY, 0.0)
        assertEquals(ay.toDouble(), specificForce.valueX, 0.0)
        assertEquals(az.toDouble(), -specificForce.valueZ, 0.0)
    }

    @Test
    fun absoluteAttitudeEstimator_whenAccelerometerMeasurementWithBiasAndNoListener_updatesAcceleration() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = EcefAbsolutePoseEstimator(context, location)

        // check initial value
        val specificForce: AccelerationTriad? = estimator.getPrivateProperty("specificForce")
        requireNotNull(specificForce)
        assertEquals(0.0, specificForce.valueX, 0.0)
        assertEquals(0.0, specificForce.valueY, 0.0)
        assertEquals(0.0, specificForce.valueZ, 0.0)

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
        assertEquals((ax - bx).toDouble(), specificForce.valueY, 0.0)
        assertEquals((ay - by).toDouble(), specificForce.valueX, 0.0)
        assertEquals((az - bz).toDouble(), -specificForce.valueZ, 0.0)
    }

    @Test
    fun absoluteAttitudeEstimator_whenAccelerometerMeasurementAndListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val listener =
            mockk<AccelerometerSensorCollector.OnMeasurementListener>(relaxUnitFun = true)
        val estimator =
            EcefAbsolutePoseEstimator(
                context,
                location,
                accelerometerMeasurementListener = listener
            )

        // check initial value
        val specificForce: AccelerationTriad? = estimator.getPrivateProperty("specificForce")
        requireNotNull(specificForce)
        assertEquals(0.0, specificForce.valueX, 0.0)
        assertEquals(0.0, specificForce.valueY, 0.0)
        assertEquals(0.0, specificForce.valueZ, 0.0)

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
        assertEquals((ax - bx).toDouble(), specificForce.valueY, 0.0)
        assertEquals((ay - by).toDouble(), specificForce.valueX, 0.0)
        assertEquals((az - bz).toDouble(), -specificForce.valueZ, 0.0)

        verify(exactly = 1) { listener.onMeasurement(ax, ay, az, bx, by, bz, timestamp, accuracy) }
    }

    @Test
    fun absoluteAttitudeEstimator_whenGyroscopeMeasurementNoBiasAndNoListener_updatesAngularSpeed() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = EcefAbsolutePoseEstimator(context, location)

        // check initial value
        val angularSpeed: AngularSpeedTriad? = estimator.getPrivateProperty("angularSpeed")
        requireNotNull(angularSpeed)
        assertEquals(0.0, angularSpeed.valueX, 0.0)
        assertEquals(0.0, angularSpeed.valueY, 0.0)
        assertEquals(0.0, angularSpeed.valueZ, 0.0)

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
        assertEquals(wx.toDouble(), angularSpeed.valueY, 0.0)
        assertEquals(wy.toDouble(), angularSpeed.valueX, 0.0)
        assertEquals(wz.toDouble(), -angularSpeed.valueZ, 0.0)
    }

    @Test
    fun absoluteAttitudeEstimator_whenGyroscopeMeasurementWithBiasAndNoListener_updatesAngularSpeed() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = EcefAbsolutePoseEstimator(context, location)

        // check initial value
        val angularSpeed: AngularSpeedTriad? = estimator.getPrivateProperty("angularSpeed")
        requireNotNull(angularSpeed)
        assertEquals(0.0, angularSpeed.valueX, 0.0)
        assertEquals(0.0, angularSpeed.valueY, 0.0)
        assertEquals(0.0, angularSpeed.valueZ, 0.0)

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
        assertEquals((wx - bx).toDouble(), angularSpeed.valueY, 0.0)
        assertEquals((wy - by).toDouble(), angularSpeed.valueX, 0.0)
        assertEquals((wz - bz).toDouble(), -angularSpeed.valueZ, 0.0)
    }

    @Test
    fun absoluteAttitudeEstimator_whenGyroscopeMeasurementAndListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val listener = mockk<GyroscopeSensorCollector.OnMeasurementListener>(relaxUnitFun = true)
        val estimator =
            EcefAbsolutePoseEstimator(context, location, gyroscopeMeasurementListener = listener)

        // check initial value
        val angularSpeed: AngularSpeedTriad? = estimator.getPrivateProperty("angularSpeed")
        requireNotNull(angularSpeed)
        assertEquals(0.0, angularSpeed.valueX, 0.0)
        assertEquals(0.0, angularSpeed.valueY, 0.0)
        assertEquals(0.0, angularSpeed.valueZ, 0.0)

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
        assertEquals((wx - bx).toDouble(), angularSpeed.valueY, 0.0)
        assertEquals((wy - by).toDouble(), angularSpeed.valueX, 0.0)
        assertEquals((wz - bz).toDouble(), -angularSpeed.valueZ, 0.0)

        verify(exactly = 1) { listener.onMeasurement(wx, wy, wz, bx, by, bz, timestamp, accuracy) }
    }

    @Test
    fun absoluteAttitudeEstimator_whenGravityMeasurement_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val listener = mockk<GravityEstimator.OnEstimationListener>(relaxUnitFun = true)
        val estimator =
            EcefAbsolutePoseEstimator(context, location, gravityEstimationListener = listener)

        val absoluteAttitudeEstimator: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        val gravityEstimationListener = absoluteAttitudeEstimator.gravityEstimationListener
        requireNotNull(gravityEstimationListener)

        assertSame(listener, gravityEstimationListener)

        val randomizer = UniformRandomizer()
        val fx = randomizer.nextDouble()
        val fy = randomizer.nextDouble()
        val fz = randomizer.nextDouble()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        gravityEstimationListener.onEstimation(mockk(), fx, fy, fz, timestamp)

        verify(exactly = 1) { listener.onEstimation(any(), fx, fy, fz, timestamp) }
    }

    @Test
    fun absoluteAttitudeEstimator_whenMagnetometerMeasurement_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val listener = mockk<MagnetometerSensorCollector.OnMeasurementListener>(relaxUnitFun = true)
        val estimator =
            EcefAbsolutePoseEstimator(context, location, magnetometerMeasurementListener = listener)

        val absoluteAttitudeEstimator: AbsoluteAttitudeEstimator<*, *>? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        val magnetometerMeasurementListener =
            absoluteAttitudeEstimator.magnetometerMeasurementListener
        requireNotNull(magnetometerMeasurementListener)

        assertSame(listener, magnetometerMeasurementListener)

        val randomizer = UniformRandomizer()
        val bx = randomizer.nextFloat()
        val by = randomizer.nextFloat()
        val bz = randomizer.nextFloat()
        val hardIronX = randomizer.nextFloat()
        val hardIronY = randomizer.nextFloat()
        val hardIronZ = randomizer.nextFloat()
        val timestamp = SystemClock.elapsedRealtimeNanos()
        val accuracy = SensorAccuracy.HIGH
        magnetometerMeasurementListener.onMeasurement(
            bx,
            by,
            bz,
            hardIronX,
            hardIronY,
            hardIronZ,
            timestamp,
            accuracy
        )

        verify(exactly = 1) {
            listener.onMeasurement(
                bx,
                by,
                bz,
                hardIronX,
                hardIronY,
                hardIronZ,
                timestamp,
                accuracy
            )
        }
    }

    @Test
    fun absoluteAttitudeEstimator_whenNotInitialized_initializesAndSetsInitialFrame() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val initialVelocity = getNEDVelocity()
        val poseAvailableListener =
            mockk<EcefAbsolutePoseEstimator.OnPoseAvailableListener>(relaxUnitFun = true)
        val estimator =
            EcefAbsolutePoseEstimator(
                context,
                location,
                initialVelocity,
                poseAvailableListener = poseAvailableListener
            )

        val absoluteAttitudeEstimator: FusedGeomagneticAttitudeEstimator? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        val absoluteAttitudeEstimatorSpy = spyk(absoluteAttitudeEstimator)
        every { absoluteAttitudeEstimatorSpy.gyroscopeAverageTimeInterval }.returns(TIME_INTERVAL)

        val initializedFrame1: Boolean? = estimator.getPrivateProperty("initializedFrame")
        requireNotNull(initializedFrame1)
        assertFalse(initializedFrame1)

        val attitude = getAttitude()
        val c = CoordinateTransformation(
            attitude,
            FrameType.BODY_FRAME,
            FrameType.LOCAL_NAVIGATION_FRAME
        )

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
        val initializedFrame2: Boolean? = estimator.getPrivateProperty("initializedFrame")
        requireNotNull(initializedFrame2)
        assertTrue(initializedFrame2)

        val initialNedFrame: NEDFrame? = estimator.getPrivateProperty("initialNedFrame")
        requireNotNull(initialNedFrame)
        val initialEcefFrame: ECEFFrame? = estimator.getPrivateProperty("initialEcefFrame")
        requireNotNull(initialEcefFrame)

        assertEquals(c, initialNedFrame.coordinateTransformation)
        assertEquals(attitude, initialNedFrame.coordinateTransformation.asRotation())
        assertEquals(location.toNEDPosition(), initialNedFrame.position)
        assertEquals(initialVelocity, initialNedFrame.velocity)

        val expectedInitialEcefFrame =
            NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(initialNedFrame)
        assertEquals(expectedInitialEcefFrame, initialEcefFrame)

        val previousEcefFrame: ECEFFrame? = estimator.getPrivateProperty("previousEcefFrame")
        requireNotNull(previousEcefFrame)
        assertEquals(initialEcefFrame, previousEcefFrame)

        val initialAttitude: Quaternion? = estimator.getPrivateProperty("initialAttitude")
        requireNotNull(initialAttitude)
        assertEquals(attitude, initialAttitude)

        verify { poseAvailableListener wasNot Called }
    }

    @Test
    fun absoluteAttitudeEstimator_whenInitialized_computesCurrentFrameAndNotifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val initialVelocity = getNEDVelocity()
        val poseAvailableListener =
            mockk<EcefAbsolutePoseEstimator.OnPoseAvailableListener>(relaxUnitFun = true)
        val estimator =
            EcefAbsolutePoseEstimator(
                context,
                location,
                initialVelocity,
                estimatePoseTransformation = false,
                poseAvailableListener = poseAvailableListener
            )

        val absoluteAttitudeEstimator: FusedGeomagneticAttitudeEstimator? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        val absoluteAttitudeEstimatorSpy = spyk(absoluteAttitudeEstimator)
        every { absoluteAttitudeEstimatorSpy.gyroscopeAverageTimeInterval }.returns(TIME_INTERVAL)

        // set as initialized
        estimator.setPrivateProperty("initializedFrame", true)

        val initializedFrame: Boolean? = estimator.getPrivateProperty("initializedFrame")
        requireNotNull(initializedFrame)
        assertTrue(initializedFrame)

        val initialNedPosition = location.toNEDPosition()
        val initialAttitude = getAttitude()
        val initialC = CoordinateTransformation(
            initialAttitude,
            FrameType.BODY_FRAME,
            FrameType.LOCAL_NAVIGATION_FRAME
        )
        val initialNedFrame = NEDFrame(initialNedPosition, initialVelocity, initialC)
        val initialEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(initialNedFrame)
        val previousEcefFrame = ECEFFrame(initialEcefFrame)
        val previousEcefFrame2 = ECEFFrame(previousEcefFrame)

        estimator.setPrivateProperty("initialNedFrame", initialNedFrame)
        estimator.setPrivateProperty("initialEcefFrame", initialEcefFrame)
        estimator.setPrivateProperty("previousEcefFrame", previousEcefFrame)
        estimator.setPrivateProperty("initialAttitude", initialAttitude)

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

        // execute
        val attitude = getAttitude()
        val c = CoordinateTransformation(
            attitude,
            FrameType.BODY_FRAME,
            FrameType.LOCAL_NAVIGATION_FRAME
        )

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
        val currentAttitude: Quaternion? = estimator.getPrivateProperty("currentAttitude")
        requireNotNull(currentAttitude)
        assertEquals(attitude, currentAttitude)

        val bodyKinematics: BodyKinematics? = estimator.getPrivateProperty("bodyKinematics")
        requireNotNull(bodyKinematics)
        assertEquals(fx, bodyKinematics.fx, 0.0)
        assertEquals(fy, bodyKinematics.fy, 0.0)
        assertEquals(fz, bodyKinematics.fz, 0.0)
        assertEquals(wx, bodyKinematics.angularRateX, 0.0)
        assertEquals(wy, bodyKinematics.angularRateY, 0.0)
        assertEquals(wz, bodyKinematics.angularRateZ, 0.0)

        val currentEcefFrame1 =
            com.irurueta.navigation.inertial.navigators.ECEFInertialNavigator.navigateECEFAndReturnNew(
                TIME_INTERVAL,
                previousEcefFrame2,
                bodyKinematics
            )
        val currentNedFrame1 =
            ECEFtoNEDFrameConverter.convertECEFtoNEDAndReturnNew(currentEcefFrame1)
        currentNedFrame1.coordinateTransformation = CoordinateTransformation(
            attitude,
            FrameType.BODY_FRAME,
            FrameType.LOCAL_NAVIGATION_FRAME
        )
        NEDtoECEFFrameConverter.convertNEDtoECEF(currentNedFrame1, currentEcefFrame1)

        val currentEcefFrame2: ECEFFrame? = estimator.getPrivateProperty("currentEcefFrame")
        requireNotNull(currentEcefFrame2)
        assertEquals(currentEcefFrame1, currentEcefFrame2)
        assertEquals(previousEcefFrame, currentEcefFrame2)

        val currentNedFrame2: NEDFrame? = estimator.getPrivateProperty("currentNedFrame")
        requireNotNull(currentNedFrame2)
        assertEquals(currentNedFrame1, currentNedFrame2)
        val previousNedFrame: NEDFrame? = estimator.getPrivateProperty("previousNedFrame")
        requireNotNull(previousNedFrame)
        assertEquals(previousNedFrame, currentNedFrame2)

        verify(exactly = 1) {
            poseAvailableListener.onPoseAvailable(
                estimator,
                currentEcefFrame2,
                previousEcefFrame,
                initialEcefFrame,
                0L,
                null
            )
        }
    }

    @Test
    fun absoluteAttitudeEstimator_whenInitializedAndTransformationEstimated_computesCurrentFrameTransformationAndNotifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val initialVelocity = getNEDVelocity()
        val poseAvailableListener =
            mockk<EcefAbsolutePoseEstimator.OnPoseAvailableListener>(relaxUnitFun = true)
        val estimator =
            EcefAbsolutePoseEstimator(
                context,
                location,
                initialVelocity,
                estimatePoseTransformation = true,
                poseAvailableListener = poseAvailableListener
            )

        val absoluteAttitudeEstimator: FusedGeomagneticAttitudeEstimator? =
            estimator.getPrivateProperty("absoluteAttitudeEstimator")
        requireNotNull(absoluteAttitudeEstimator)
        val absoluteAttitudeEstimatorSpy = spyk(absoluteAttitudeEstimator)
        every { absoluteAttitudeEstimatorSpy.gyroscopeAverageTimeInterval }.returns(TIME_INTERVAL)

        // set as initialized
        estimator.setPrivateProperty("initializedFrame", true)

        val initializedFrame: Boolean? = estimator.getPrivateProperty("initializedFrame")
        requireNotNull(initializedFrame)
        assertTrue(initializedFrame)

        val initialNedPosition = getLocation().toNEDPosition()
        val initialAttitude = getAttitude()
        val initialC = CoordinateTransformation(
            FrameType.BODY_FRAME,
            FrameType.LOCAL_NAVIGATION_FRAME
        )
        initialC.fromRotation(initialAttitude)
        val initialNedFrame = NEDFrame(initialNedPosition, initialVelocity, initialC)
        val initialEcefFrame = NEDtoECEFFrameConverter.convertNEDtoECEFAndReturnNew(initialNedFrame)
        val previousEcefFrame = ECEFFrame(initialEcefFrame)
        val previousEcefFrame2 = ECEFFrame(previousEcefFrame)

        estimator.setPrivateProperty("initialNedFrame", initialNedFrame)
        estimator.setPrivateProperty("initialEcefFrame", initialEcefFrame)
        estimator.setPrivateProperty("previousEcefFrame", previousEcefFrame)
        estimator.setPrivateProperty("initialAttitude", initialAttitude)

        // set body kinematics
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

        // execute
        val attitude = getAttitude()
        val c = CoordinateTransformation(
            FrameType.BODY_FRAME,
            FrameType.EARTH_CENTERED_EARTH_FIXED_FRAME
        )
        c.fromRotation(attitude)

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
        val currentAttitude: Quaternion? = estimator.getPrivateProperty("currentAttitude")
        requireNotNull(currentAttitude)
        assertEquals(attitude, currentAttitude)

        val bodyKinematics: BodyKinematics? = estimator.getPrivateProperty("bodyKinematics")
        requireNotNull(bodyKinematics)
        assertEquals(fx, bodyKinematics.fx, 0.0)
        assertEquals(fy, bodyKinematics.fy, 0.0)
        assertEquals(fz, bodyKinematics.fz, 0.0)
        assertEquals(wx, bodyKinematics.angularRateX, 0.0)
        assertEquals(wy, bodyKinematics.angularRateY, 0.0)
        assertEquals(wz, bodyKinematics.angularRateZ, 0.0)

        val currentEcefFrame1 =
            com.irurueta.navigation.inertial.navigators.ECEFInertialNavigator.navigateECEFAndReturnNew(
                TIME_INTERVAL,
                previousEcefFrame2,
                bodyKinematics
            )
        val currentNedFrame1 =
            ECEFtoNEDFrameConverter.convertECEFtoNEDAndReturnNew(currentEcefFrame1)
        currentNedFrame1.coordinateTransformation = CoordinateTransformation(
            attitude,
            FrameType.BODY_FRAME,
            FrameType.LOCAL_NAVIGATION_FRAME
        )
        NEDtoECEFFrameConverter.convertNEDtoECEF(currentNedFrame1, currentEcefFrame1)

        val currentEcefFrame2: ECEFFrame? = estimator.getPrivateProperty("currentEcefFrame")
        requireNotNull(currentEcefFrame2)
        assertEquals(currentEcefFrame1, currentEcefFrame2)
        assertEquals(previousEcefFrame, currentEcefFrame2)

        val currentNedFrame2: NEDFrame? = estimator.getPrivateProperty("currentNedFrame")
        requireNotNull(currentNedFrame2)
        assertEquals(currentNedFrame1, currentNedFrame2)
        val previousNedFrame: NEDFrame? = estimator.getPrivateProperty("previousNedFrame")
        requireNotNull(previousNedFrame)
        assertEquals(previousNedFrame, currentNedFrame2)

        val poseTransformationSlot1 = slot<EuclideanTransformation3D>()
        verify(exactly = 1) {
            poseAvailableListener.onPoseAvailable(
                estimator,
                currentEcefFrame2,
                previousEcefFrame,
                initialEcefFrame,
                0L,
                capture(poseTransformationSlot1)
            )
        }

        val poseTransformation1 = poseTransformationSlot1.captured

        val poseTransformation2: EuclideanTransformation3D? =
            estimator.getPrivateProperty("poseTransformation")
        assertSame(poseTransformation1, poseTransformation2)

        val poseTransformation3 = EuclideanTransformation3D()
        callPrivateFunc(
            EcefAbsolutePoseEstimator::class,
            estimator,
            "computeTransformation",
            initialEcefFrame,
            currentEcefFrame2,
            initialAttitude,
            attitude,
            poseTransformation3
        )
        assertEquals(poseTransformation1.asMatrix(), poseTransformation3.asMatrix())
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