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
import com.irurueta.android.navigation.inertial.estimators.filter.MedianAveragingFilter
import com.irurueta.android.navigation.inertial.getPrivateProperty
import com.irurueta.android.navigation.inertial.setPrivateProperty
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel
import com.irurueta.statistics.UniformRandomizer
import io.mockk.*
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner
import java.util.*
import kotlin.math.abs

@RunWith(RobolectricTestRunner::class)
class FusedGeomagneticAttitudeEstimator2Test {

    @Test
    fun constructor_whenRequiredProperties_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

        // check
        assertSame(context, estimator.context)
        assertNull(estimator.location)
        assertEquals(SensorDelay.GAME, estimator.sensorDelay)
        assertFalse(estimator.useAccelerometer)
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
        assertFalse(estimator.useAccurateLevelingEstimator)
        assertFalse(estimator.useAccurateRelativeGyroscopeAttitudeEstimator)
        assertFalse(estimator.estimateCoordinateTransformation)
        assertTrue(estimator.estimateEulerAngles)
        assertNull(estimator.attitudeAvailableListener)
        assertNull(estimator.accelerometerMeasurementListener)
        assertNull(estimator.gravityMeasurementListener)
        assertNull(estimator.gyroscopeMeasurementListener)
        assertNull(estimator.magnetometerMeasurementListener)
        assertNull(estimator.gravityEstimationListener)
        assertFalse(estimator.running)
    }

    @Test
    fun constructor_whenAllProperties_setsExpectedValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val accelerometerAveragingFilter = MedianAveragingFilter()
        val worldMagneticModel = WorldMagneticModel()
        val timestamp = Date()
        val listener = mockk<FusedGeomagneticAttitudeEstimator2.OnAttitudeAvailableListener>()
        val accelerometerMeasurementListener =
            mockk<AccelerometerSensorCollector.OnMeasurementListener>()
        val gravityMeasurementListener = mockk<GravitySensorCollector.OnMeasurementListener>()
        val gyroscopeMeasurementListener = mockk<GyroscopeSensorCollector.OnMeasurementListener>()
        val magnetometerMeasurementListener =
            mockk<MagnetometerSensorCollector.OnMeasurementListener>()
        val gravityEstimationListener = mockk<GravityEstimator.OnEstimationListener>()
        val estimator = FusedGeomagneticAttitudeEstimator2(
            context,
            location,
            SensorDelay.NORMAL,
            useAccelerometer = true,
            AccelerometerSensorCollector.SensorType.ACCELEROMETER_UNCALIBRATED,
            MagnetometerSensorCollector.SensorType.MAGNETOMETER_UNCALIBRATED,
            accelerometerAveragingFilter,
            GyroscopeSensorCollector.SensorType.GYROSCOPE_UNCALIBRATED,
            worldMagneticModel,
            timestamp,
            useWorldMagneticModel = true,
            useAccurateLevelingEstimator = true,
            useAccurateRelativeGyroscopeAttitudeEstimator = true,
            estimateCoordinateTransformation = true,
            estimateEulerAngles = false,
            attitudeAvailableListener = listener,
            accelerometerMeasurementListener = accelerometerMeasurementListener,
            gravityMeasurementListener = gravityMeasurementListener,
            gyroscopeMeasurementListener = gyroscopeMeasurementListener,
            magnetometerMeasurementListener = magnetometerMeasurementListener,
            gravityEstimationListener = gravityEstimationListener
        )

        // check
        assertSame(context, estimator.context)
        assertSame(location, estimator.location)
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertTrue(estimator.useAccelerometer)
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
        assertTrue(estimator.useAccurateLevelingEstimator)
        assertTrue(estimator.useAccurateRelativeGyroscopeAttitudeEstimator)
        assertTrue(estimator.estimateCoordinateTransformation)
        assertFalse(estimator.estimateEulerAngles)
        assertSame(listener, estimator.attitudeAvailableListener)
        assertSame(accelerometerMeasurementListener, estimator.accelerometerMeasurementListener)
        assertSame(gravityMeasurementListener, estimator.gravityMeasurementListener)
        assertSame(gyroscopeMeasurementListener, estimator.gyroscopeMeasurementListener)
        assertSame(magnetometerMeasurementListener, estimator.magnetometerMeasurementListener)
        assertSame(gravityEstimationListener, estimator.gravityEstimationListener)
        assertFalse(estimator.running)
    }

    @Test
    fun location_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

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
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

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
        val estimator = FusedGeomagneticAttitudeEstimator2(context, location)

        assertFalse(estimator.running)
        assertSame(location, estimator.location)

        // set as running
        estimator.setPrivateProperty("running", true)

        assertTrue(estimator.running)

        // set new value
        estimator.location = null
    }

    @Test
    fun worldMagneticModel_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

        // check default value
        assertFalse(estimator.running)
        assertNull(estimator.worldMagneticModel)
        val geomagneticAttitudeEstimator1: GeomagneticAttitudeEstimator? =
            estimator.getPrivateProperty("geomagneticAttitudeEstimator")
        requireNotNull(geomagneticAttitudeEstimator1)
        assertNull(geomagneticAttitudeEstimator1.worldMagneticModel)

        // set new value
        val worldMagneticModel = WorldMagneticModel()
        estimator.worldMagneticModel = worldMagneticModel

        // check
        assertSame(worldMagneticModel, estimator.worldMagneticModel)
        val geomagneticAttitudeEstimator2: GeomagneticAttitudeEstimator? =
            estimator.getPrivateProperty("geomagneticAttitudeEstimator")
        requireNotNull(geomagneticAttitudeEstimator2)
        assertSame(worldMagneticModel, geomagneticAttitudeEstimator2.worldMagneticModel)
    }

    @Test(expected = IllegalStateException::class)
    fun worldMagneticModel_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

        // set as running
        estimator.setPrivateProperty("running", true)

        estimator.worldMagneticModel = WorldMagneticModel()
    }

    @Test
    fun timestamp_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

        // check default value
        assertNotNull(estimator.timestamp)

        // set new value
        val timestamp = Date()
        estimator.timestamp = timestamp

        // check
        assertSame(timestamp, estimator.timestamp)
    }

    @Test
    fun attitudeAvailableListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

        // check default value
        assertNull(estimator.attitudeAvailableListener)

        // set new value
        val listener = mockk<FusedGeomagneticAttitudeEstimator2.OnAttitudeAvailableListener>()
        estimator.attitudeAvailableListener = listener

        // check
        assertSame(listener, estimator.attitudeAvailableListener)
    }

    @Test
    fun accelerometerMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

        // check default value
        assertNull(estimator.accelerometerMeasurementListener)

        // set new value
        val accelerometerMeasurementListener =
            mockk<AccelerometerSensorCollector.OnMeasurementListener>()
        estimator.accelerometerMeasurementListener = accelerometerMeasurementListener

        // check
        assertSame(accelerometerMeasurementListener, estimator.accelerometerMeasurementListener)

        val relativeAttitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("relativeAttitudeEstimator")
        requireNotNull(relativeAttitudeEstimator)
        assertSame(
            accelerometerMeasurementListener,
            relativeAttitudeEstimator.accelerometerMeasurementListener
        )
    }

    @Test
    fun gravityMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

        // check default value
        assertNull(estimator.gravityMeasurementListener)

        // set new value
        val gravityMeasurementListener = mockk<GravitySensorCollector.OnMeasurementListener>()
        estimator.gravityMeasurementListener = gravityMeasurementListener

        // check
        assertSame(gravityMeasurementListener, estimator.gravityMeasurementListener)

        val relativeAttitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("relativeAttitudeEstimator")
        requireNotNull(relativeAttitudeEstimator)
        assertSame(gravityMeasurementListener, relativeAttitudeEstimator.gravityMeasurementListener)
    }

    @Test
    fun gyroscopeMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

        // check default value
        assertNull(estimator.gyroscopeMeasurementListener)

        // set new value
        val gyroscopeMeasurementListener = mockk<GyroscopeSensorCollector.OnMeasurementListener>()
        estimator.gyroscopeMeasurementListener = gyroscopeMeasurementListener

        // check
        assertSame(gyroscopeMeasurementListener, estimator.gyroscopeMeasurementListener)

        val relativeAttitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("relativeAttitudeEstimator")
        requireNotNull(relativeAttitudeEstimator)
        assertSame(
            gyroscopeMeasurementListener,
            relativeAttitudeEstimator.gyroscopeMeasurementListener
        )
    }

    @Test
    fun magnetometerMeasurementListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

        // check default value
        val magnetometerMeasurementListener =
            mockk<MagnetometerSensorCollector.OnMeasurementListener>()
        estimator.magnetometerMeasurementListener = magnetometerMeasurementListener

        // check
        assertSame(magnetometerMeasurementListener, estimator.magnetometerMeasurementListener)

        val geomagneticAttitudeEstimator: GeomagneticAttitudeEstimator? =
            estimator.getPrivateProperty("geomagneticAttitudeEstimator")
        requireNotNull(geomagneticAttitudeEstimator)
        assertSame(
            magnetometerMeasurementListener,
            geomagneticAttitudeEstimator.magnetometerMeasurementListener
        )
    }

    @Test
    fun gravityEstimationListener_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

        // check default value
        assertNull(estimator.gyroscopeMeasurementListener)

        // set new value
        val gravityEstimationListener = mockk<GravityEstimator.OnEstimationListener>()
        estimator.gravityEstimationListener = gravityEstimationListener

        // check
        assertSame(gravityEstimationListener, estimator.gravityEstimationListener)

        val relativeAttitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("relativeAttitudeEstimator")
        requireNotNull(relativeAttitudeEstimator)
        assertSame(gravityEstimationListener, relativeAttitudeEstimator.gravityEstimationListener)
    }

    @Test
    fun useWorldMagneticModel_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

        // check
        assertFalse(estimator.running)
        assertFalse(estimator.useWorldMagneticModel)
        val geomagneticAttitudeEstimator1: GeomagneticAttitudeEstimator? =
            estimator.getPrivateProperty("geomagneticAttitudeEstimator")
        requireNotNull(geomagneticAttitudeEstimator1)
        assertFalse(geomagneticAttitudeEstimator1.useWorldMagneticModel)

        // set new value
        estimator.useWorldMagneticModel = true

        // check
        assertTrue(estimator.useWorldMagneticModel)
        val geomagneticAttitudeEstimator2: GeomagneticAttitudeEstimator? =
            estimator.getPrivateProperty("geomagneticAttitudeEstimator")
        requireNotNull(geomagneticAttitudeEstimator2)
        assertTrue(geomagneticAttitudeEstimator2.useWorldMagneticModel)
    }

    @Test(expected = IllegalStateException::class)
    fun useWorldMagneticModel_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        estimator.useWorldMagneticModel = true
    }

    @Test(expected = IllegalStateException::class)
    fun useAccurateLevelingEstimator_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

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
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

        assertFalse(estimator.useAccurateLevelingEstimator)
        assertNull(estimator.location)
        assertFalse(estimator.running)
        val geomagneticAttitudeEstimator1: GeomagneticAttitudeEstimator? =
            estimator.getPrivateProperty("geomagneticAttitudeEstimator")
        requireNotNull(geomagneticAttitudeEstimator1)
        assertFalse(geomagneticAttitudeEstimator1.useAccurateLevelingEstimator)

        estimator.useAccurateLevelingEstimator = false

        // check
        assertFalse(estimator.useAccurateLevelingEstimator)
        val geomagneticAttitudeEstimator2: GeomagneticAttitudeEstimator? =
            estimator.getPrivateProperty("geomagneticAttitudeEstimator")
        requireNotNull(geomagneticAttitudeEstimator2)
        assertFalse(geomagneticAttitudeEstimator2.useAccurateLevelingEstimator)
    }

    @Test(expected = IllegalStateException::class)
    fun useAccurateLevelingEstimator_whenNotRunningNoLocationAndSetToTrue_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

        assertFalse(estimator.useAccurateLevelingEstimator)
        assertNull(estimator.location)
        assertFalse(estimator.running)

        estimator.useAccurateLevelingEstimator = true
    }

    @Test
    fun useAccurateLevelingEstimator_whenNotRunningAndLocationAndSetToTrue_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val location = getLocation()
        val estimator = FusedGeomagneticAttitudeEstimator2(context, location)

        // check default value
        assertFalse(estimator.running)
        assertSame(location, estimator.location)
        assertFalse(estimator.useAccurateLevelingEstimator)
        val geomagneticAttitudeEstimator1: GeomagneticAttitudeEstimator? =
            estimator.getPrivateProperty("geomagneticAttitudeEstimator")
        requireNotNull(geomagneticAttitudeEstimator1)
        assertFalse(geomagneticAttitudeEstimator1.useAccurateLevelingEstimator)

        // set new value
        estimator.useAccurateLevelingEstimator = true

        // check
        assertTrue(estimator.useAccurateLevelingEstimator)
        val geomagneticAttitudeEstimator2: GeomagneticAttitudeEstimator? =
            estimator.getPrivateProperty("geomagneticAttitudeEstimator")
        requireNotNull(geomagneticAttitudeEstimator2)
        assertTrue(geomagneticAttitudeEstimator2.useAccurateLevelingEstimator)
    }

    @Test(expected = IllegalStateException::class)
    fun useAccurateRelativeGyroscopeAttitudeEstimator_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

        assertFalse(estimator.running)

        // set as running
        estimator.setPrivateProperty("running", true)

        assertTrue(estimator.running)

        estimator.useAccurateRelativeGyroscopeAttitudeEstimator = true
    }

    @Test
    fun useAccurateRelativeGyroscopeAttitudeEstimator_whenNotRunning_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

        assertFalse(estimator.running)
        assertFalse(estimator.useAccurateRelativeGyroscopeAttitudeEstimator)

        // set new value
        estimator.useAccurateRelativeGyroscopeAttitudeEstimator = true

        // check
        assertTrue(estimator.useAccurateRelativeGyroscopeAttitudeEstimator)
    }

    @Test
    fun useIndirectInterpolation_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

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
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

        assertEquals(
            FusedGeomagneticAttitudeEstimator2.DEFAULT_INTERPOLATION_VALUE,
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
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

        assertEquals(
            FusedGeomagneticAttitudeEstimator2.DEFAULT_INTERPOLATION_VALUE,
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
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

        estimator.indirectInterpolationWeight = 0.0
    }

    @Test
    fun indirectInterpolationWeight_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

        // check default value
        assertEquals(
            FusedGeomagneticAttitudeEstimator2.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
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
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

        val relativeAttitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("relativeAttitudeEstimator")
        requireNotNull(relativeAttitudeEstimator)
        val relativeAttitudeEstimatorSpy = spyk(relativeAttitudeEstimator)
        every { relativeAttitudeEstimatorSpy.gyroscopeAverageTimeInterval }.returns(TIME_INTERVAL)
        estimator.setPrivateProperty("relativeAttitudeEstimator", relativeAttitudeEstimatorSpy)

        assertEquals(TIME_INTERVAL, estimator.gyroscopeAverageTimeInterval, 0.0)
        verify(exactly = 1) { relativeAttitudeEstimatorSpy.gyroscopeAverageTimeInterval }
    }

    @Test
    fun running_whenInternalEstimatorsAreNotRunning_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

        assertFalse(estimator.running)
    }

    @Test
    fun running_returnsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

        assertFalse(estimator.running)

        // set as running
        estimator.setPrivateProperty("running", true)

        assertTrue(estimator.running)
    }

    @Test(expected = IllegalStateException::class)
    fun outlierThreshold_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

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
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

        // check default value
        assertEquals(
            FusedGeomagneticAttitudeEstimator2.DEFAULT_OUTLIER_THRESHOLD,
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
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

        // check default value
        assertEquals(
            FusedGeomagneticAttitudeEstimator2.DEFAULT_OUTLIER_THRESHOLD,
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
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

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
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

        // check default value
        assertEquals(
            FusedGeomagneticAttitudeEstimator2.DEFAULT_OUTLIER_PANIC_THRESHOLD,
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
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

        // check default value
        assertEquals(
            FusedGeomagneticAttitudeEstimator2.DEFAULT_OUTLIER_PANIC_THRESHOLD,
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
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        estimator.panicCounterThreshold = 1
    }

    @Test(expected = IllegalArgumentException::class)
    fun panicCounterThreshold_whenInvalid_throwsIllegalArgumentException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

        assertFalse(estimator.running)

        estimator.panicCounterThreshold = 0
    }

    @Test
    fun panicCounterThreshold_whenValid_setsExpectedValue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

        assertFalse(estimator.running)
        assertEquals(
            FusedGeomagneticAttitudeEstimator2.DEFAULT_PANIC_COUNTER_THRESHOLD,
            estimator.panicCounterThreshold
        )

        estimator.panicCounterThreshold = 2

        // check
        assertEquals(2, estimator.panicCounterThreshold)
    }

    @Test(expected = IllegalStateException::class)
    fun start_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

        assertFalse(estimator.running)

        // set as running
        estimator.setPrivateProperty("running", true)

        assertTrue(estimator.running)

        estimator.start()
    }

    @Test
    fun start_whenNotRunningAndInternalGeomagneticEstimatorFails_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

        assertFalse(estimator.running)

        val geomagneticAttitudeEstimator: GeomagneticAttitudeEstimator? =
            estimator.getPrivateProperty("geomagneticAttitudeEstimator")
        requireNotNull(geomagneticAttitudeEstimator)
        val geomagneticAttitudeEstimatorSpy = spyk(geomagneticAttitudeEstimator)
        every { geomagneticAttitudeEstimatorSpy.start() }.returns(false)
        estimator.setPrivateProperty(
            "geomagneticAttitudeEstimator",
            geomagneticAttitudeEstimatorSpy
        )

        val relativeAttitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("relativeAttitudeEstimator")
        requireNotNull(relativeAttitudeEstimator)
        val relativeAttitudeEstimatorSpy = spyk(relativeAttitudeEstimator)
        estimator.setPrivateProperty("relativeAttitudeEstimator", relativeAttitudeEstimatorSpy)

        assertFalse(estimator.start())

        verify(exactly = 1) { geomagneticAttitudeEstimatorSpy.start() }
        verify(exactly = 0) { relativeAttitudeEstimatorSpy.start() }
        verify(exactly = 1) { geomagneticAttitudeEstimatorSpy.stop() }
        verify(exactly = 1) { relativeAttitudeEstimatorSpy.stop() }
    }

    @Test
    fun start_whenNotRunningAndInternalRelativeEstimatorFails_returnsFalse() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

        assertFalse(estimator.running)

        val geomagneticAttitudeEstimator: GeomagneticAttitudeEstimator? =
            estimator.getPrivateProperty("geomagneticAttitudeEstimator")
        requireNotNull(geomagneticAttitudeEstimator)
        val geomagneticAttitudeEstimatorSpy = spyk(geomagneticAttitudeEstimator)
        every { geomagneticAttitudeEstimatorSpy.start() }.returns(true)
        estimator.setPrivateProperty(
            "geomagneticAttitudeEstimator",
            geomagneticAttitudeEstimatorSpy
        )

        val relativeAttitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("relativeAttitudeEstimator")
        requireNotNull(relativeAttitudeEstimator)
        val relativeAttitudeEstimatorSpy = spyk(relativeAttitudeEstimator)
        every { relativeAttitudeEstimatorSpy.start() }.returns(false)
        estimator.setPrivateProperty("relativeAttitudeEstimator", relativeAttitudeEstimatorSpy)

        assertFalse(estimator.start())

        verify(exactly = 1) { geomagneticAttitudeEstimatorSpy.start() }
        verify(exactly = 1) { relativeAttitudeEstimatorSpy.start() }
        verify(exactly = 1) { geomagneticAttitudeEstimatorSpy.stop() }
        verify(exactly = 1) { relativeAttitudeEstimatorSpy.stop() }
    }

    @Test
    fun start_whenNotRunningAndInternalEstimatorSucceeds_returnsTrue() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

        assertFalse(estimator.running)

        val geomagneticAttitudeEstimator: GeomagneticAttitudeEstimator? =
            estimator.getPrivateProperty("geomagneticAttitudeEstimator")
        requireNotNull(geomagneticAttitudeEstimator)
        val geomagneticAttitudeEstimatorSpy = spyk(geomagneticAttitudeEstimator)
        every { geomagneticAttitudeEstimatorSpy.start() }.returns(true)
        estimator.setPrivateProperty(
            "geomagneticAttitudeEstimator",
            geomagneticAttitudeEstimatorSpy
        )

        val relativeAttitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("relativeAttitudeEstimator")
        requireNotNull(relativeAttitudeEstimator)
        val relativeAttitudeEstimatorSpy = spyk(relativeAttitudeEstimator)
        every { relativeAttitudeEstimatorSpy.start() }.returns(true)
        estimator.setPrivateProperty("relativeAttitudeEstimator", relativeAttitudeEstimatorSpy)

        assertTrue(estimator.start())

        verify(exactly = 1) { geomagneticAttitudeEstimatorSpy.start() }
        verify(exactly = 1) { relativeAttitudeEstimatorSpy.start() }
        verify(exactly = 0) { geomagneticAttitudeEstimatorSpy.stop() }
        verify(exactly = 0) { relativeAttitudeEstimatorSpy.stop() }
    }

    @Test
    fun stop_stopsInternalEstimators() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(context)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        val geomagneticAttitudeEstimator: GeomagneticAttitudeEstimator? =
            estimator.getPrivateProperty("geomagneticAttitudeEstimator")
        requireNotNull(geomagneticAttitudeEstimator)
        val geomagneticAttitudeEstimatorSpy = spyk(geomagneticAttitudeEstimator)
        justRun { geomagneticAttitudeEstimatorSpy.stop() }
        estimator.setPrivateProperty(
            "geomagneticAttitudeEstimator",
            geomagneticAttitudeEstimatorSpy
        )

        val relativeAttitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("relativeAttitudeEstimator")
        requireNotNull(relativeAttitudeEstimator)
        val relativeAttitudeEstimatorSpy = spyk(relativeAttitudeEstimator)
        justRun { relativeAttitudeEstimatorSpy.stop() }
        estimator.setPrivateProperty("relativeAttitudeEstimator", relativeAttitudeEstimatorSpy)

        estimator.stop()

        verify(exactly = 1) { geomagneticAttitudeEstimatorSpy.stop() }
        verify(exactly = 1) { relativeAttitudeEstimatorSpy.stop() }
        assertFalse(estimator.running)
    }

    @Test
    fun processRelativeAttitude_whenFirstAndNonAccurateRelativeAttitudeEstimator_copiesAttitudeAndSetsPreviousRelativeAttitude() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(
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

        val relativeAttitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("relativeAttitudeEstimator")
        requireNotNull(relativeAttitudeEstimator)

        val internalAttitude = spyk(getAttitude())
        val listener = relativeAttitudeEstimator.attitudeAvailableListener
        val timestamp = SystemClock.elapsedRealtimeNanos()
        requireNotNull(listener)
        listener.onAttitudeAvailable(
            relativeAttitudeEstimator,
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
        val estimator = FusedGeomagneticAttitudeEstimator2(
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

        val relativeAttitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("relativeAttitudeEstimator")
        requireNotNull(relativeAttitudeEstimator)

        val internalAttitude = spyk(getAttitude())
        val listener = relativeAttitudeEstimator.attitudeAvailableListener
        requireNotNull(listener)
        val timestamp = SystemClock.elapsedRealtimeNanos()
        listener.onAttitudeAvailable(
            relativeAttitudeEstimator,
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
        val estimator = FusedGeomagneticAttitudeEstimator2(
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

        val relativeAttitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("relativeAttitudeEstimator")
        requireNotNull(relativeAttitudeEstimator)

        // call listener for the 1st time
        val internalAttitude1 = spyk(getAttitude())
        val listener = relativeAttitudeEstimator.attitudeAvailableListener
        requireNotNull(listener)
        val timestamp = SystemClock.elapsedRealtimeNanos()
        listener.onAttitudeAvailable(
            relativeAttitudeEstimator,
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
            relativeAttitudeEstimator,
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
        val estimator = FusedGeomagneticAttitudeEstimator2(
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

        val relativeAttitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("relativeAttitudeEstimator")
        requireNotNull(relativeAttitudeEstimator)

        // call listener for the 1st time
        val internalAttitude1 = spyk(getAttitude())
        val listener = relativeAttitudeEstimator.attitudeAvailableListener
        requireNotNull(listener)
        val timestamp = SystemClock.elapsedRealtimeNanos()
        listener.onAttitudeAvailable(
            relativeAttitudeEstimator,
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
            relativeAttitudeEstimator,
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
    fun processGeomagneticAttitude_whenNoRelativeAttitudeAndNonAccurateLeveling_makesNoAction() {
        val attitudeAvailableListener =
            mockk<FusedGeomagneticAttitudeEstimator2.OnAttitudeAvailableListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(
            context,
            useAccurateLevelingEstimator = false,
            attitudeAvailableListener = attitudeAvailableListener
        )

        val hasRelativeAttitude1: Boolean? = estimator.getPrivateProperty("hasRelativeAttitude")
        requireNotNull(hasRelativeAttitude1)
        assertFalse(hasRelativeAttitude1)

        val geomagneticAttitude1: Quaternion? = estimator.getPrivateProperty("geomagneticAttitude")
        requireNotNull(geomagneticAttitude1)
        assertEquals(Quaternion(), geomagneticAttitude1)

        val hasDeltaRelativeAttitude1: Boolean? =
            estimator.getPrivateProperty("hasDeltaRelativeAttitude")
        requireNotNull(hasDeltaRelativeAttitude1)
        assertFalse(hasDeltaRelativeAttitude1)

        val geomagneticEstimator: GeomagneticAttitudeEstimator? =
            estimator.getPrivateProperty("geomagneticAttitudeEstimator")
        requireNotNull(geomagneticEstimator)

        val internalAttitude = spyk(getAttitude())
        val listener = geomagneticEstimator.attitudeAvailableListener
        requireNotNull(listener)
        val timestamp = SystemClock.elapsedRealtimeNanos()
        listener.onAttitudeAvailable(
            geomagneticEstimator,
            internalAttitude,
            timestamp,
            null,
            null,
            null,
            null
        )

        val geomagneticAttitude2: Quaternion? = estimator.getPrivateProperty("geomagneticAttitude")
        requireNotNull(geomagneticAttitude2)
        assertEquals(Quaternion(), geomagneticAttitude2)

        verify { attitudeAvailableListener wasNot Called }
        verify { internalAttitude wasNot Called }
    }

    @Test
    fun processGeomagneticAttitude_whenNoRelativeAttitudeAndAccurateLeveling_makesNoAction() {
        val attitudeAvailableListener =
            mockk<FusedGeomagneticAttitudeEstimator2.OnAttitudeAvailableListener>()
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(
            context,
            location,
            useAccurateLevelingEstimator = true,
            attitudeAvailableListener = attitudeAvailableListener
        )

        val hasRelativeAttitude1: Boolean? = estimator.getPrivateProperty("hasRelativeAttitude")
        requireNotNull(hasRelativeAttitude1)
        assertFalse(hasRelativeAttitude1)

        val geomagneticAttitude1: Quaternion? = estimator.getPrivateProperty("geomagneticAttitude")
        requireNotNull(geomagneticAttitude1)
        assertEquals(Quaternion(), geomagneticAttitude1)

        val hasDeltaRelativeAttitude1: Boolean? =
            estimator.getPrivateProperty("hasDeltaRelativeAttitude")
        requireNotNull(hasDeltaRelativeAttitude1)
        assertFalse(hasDeltaRelativeAttitude1)

        val geomagneticEstimator: GeomagneticAttitudeEstimator? =
            estimator.getPrivateProperty("geomagneticAttitudeEstimator")
        requireNotNull(geomagneticEstimator)

        val internalAttitude = spyk(getAttitude())
        val listener = geomagneticEstimator.attitudeAvailableListener
        requireNotNull(listener)
        val timestamp = SystemClock.elapsedRealtimeNanos()
        listener.onAttitudeAvailable(
            geomagneticEstimator,
            internalAttitude,
            timestamp,
            null,
            null,
            null,
            null
        )

        val geomagneticAttitude2: Quaternion? = estimator.getPrivateProperty("geomagneticAttitude")
        requireNotNull(geomagneticAttitude2)
        assertEquals(Quaternion(), geomagneticAttitude2)

        verify { attitudeAvailableListener wasNot Called }
        verify { internalAttitude wasNot Called }
    }

    @Test
    fun processGeomagneticAttitude_whenRelativeAttitudeAndNonAccurateLeveling_copiesAttitude() {
        val attitudeAvailableListener =
            mockk<FusedGeomagneticAttitudeEstimator2.OnAttitudeAvailableListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(
            context,
            useAccurateLevelingEstimator = false,
            attitudeAvailableListener = attitudeAvailableListener
        )

        estimator.setPrivateProperty("hasRelativeAttitude", true)

        val hasRelativeAttitude1: Boolean? = estimator.getPrivateProperty("hasRelativeAttitude")
        requireNotNull(hasRelativeAttitude1)
        assertTrue(hasRelativeAttitude1)

        val geomagneticAttitude1: Quaternion? = estimator.getPrivateProperty("geomagneticAttitude")
        requireNotNull(geomagneticAttitude1)
        assertEquals(Quaternion(), geomagneticAttitude1)

        val hasDeltaRelativeAttitude1: Boolean? =
            estimator.getPrivateProperty("hasDeltaRelativeAttitude")
        requireNotNull(hasDeltaRelativeAttitude1)
        assertFalse(hasDeltaRelativeAttitude1)

        val geomagneticEstimator: GeomagneticAttitudeEstimator? =
            estimator.getPrivateProperty("geomagneticAttitudeEstimator")
        requireNotNull(geomagneticEstimator)

        val internalAttitude = spyk(getAttitude())
        val listener = geomagneticEstimator.attitudeAvailableListener
        requireNotNull(listener)
        val timestamp = SystemClock.elapsedRealtimeNanos()
        listener.onAttitudeAvailable(
            geomagneticEstimator,
            internalAttitude,
            timestamp,
            null,
            null,
            null,
            null
        )

        val geomagneticAttitude2: Quaternion? = estimator.getPrivateProperty("geomagneticAttitude")
        requireNotNull(geomagneticAttitude2)
        assertEquals(internalAttitude, geomagneticAttitude2)

        verify { attitudeAvailableListener wasNot Called }
        verify(exactly = 1) { internalAttitude.copyTo(geomagneticAttitude2) }
    }

    @Test
    fun processGeomagneticAttitude_whenRelativeAttitudeAndAccurateLeveling_copiesAttitude() {
        val attitudeAvailableListener =
            mockk<FusedGeomagneticAttitudeEstimator2.OnAttitudeAvailableListener>()
        val location = getLocation()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(
            context,
            location,
            useAccurateLevelingEstimator = true,
            attitudeAvailableListener = attitudeAvailableListener
        )

        estimator.setPrivateProperty("hasRelativeAttitude", true)

        val hasRelativeAttitude1: Boolean? = estimator.getPrivateProperty("hasRelativeAttitude")
        requireNotNull(hasRelativeAttitude1)
        assertTrue(hasRelativeAttitude1)

        val geomagneticAttitude1: Quaternion? = estimator.getPrivateProperty("geomagneticAttitude")
        requireNotNull(geomagneticAttitude1)
        assertEquals(Quaternion(), geomagneticAttitude1)

        val hasDeltaRelativeAttitude1: Boolean? =
            estimator.getPrivateProperty("hasDeltaRelativeAttitude")
        requireNotNull(hasDeltaRelativeAttitude1)
        assertFalse(hasDeltaRelativeAttitude1)

        val geomagneticEstimator: GeomagneticAttitudeEstimator? =
            estimator.getPrivateProperty("geomagneticAttitudeEstimator")
        requireNotNull(geomagneticEstimator)

        val internalAttitude = spyk(getAttitude())
        val listener = geomagneticEstimator.attitudeAvailableListener
        requireNotNull(listener)
        val timestamp = SystemClock.elapsedRealtimeNanos()
        listener.onAttitudeAvailable(
            geomagneticEstimator,
            internalAttitude,
            timestamp,
            null,
            null,
            null,
            null
        )

        val geomagneticAttitude2: Quaternion? = estimator.getPrivateProperty("geomagneticAttitude")
        requireNotNull(geomagneticAttitude2)
        assertEquals(internalAttitude, geomagneticAttitude2)

        verify { attitudeAvailableListener wasNot Called }
        verify(exactly = 1) { internalAttitude.copyTo(geomagneticAttitude2) }
    }

    @Test
    fun processGeomagneticAttitude_whenDeltaRelativeAttitudeAndResetLeveling_resets() {
        val attitudeAvailableListener =
            mockk<FusedGeomagneticAttitudeEstimator2.OnAttitudeAvailableListener>()
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(
            context,
            useAccurateLevelingEstimator = false,
            attitudeAvailableListener = attitudeAvailableListener
        )

        estimator.setPrivateProperty("hasRelativeAttitude", true)
        estimator.setPrivateProperty("hasDeltaRelativeAttitude", true)

        val hasRelativeAttitude1: Boolean? = estimator.getPrivateProperty("hasRelativeAttitude")
        requireNotNull(hasRelativeAttitude1)
        assertTrue(hasRelativeAttitude1)

        val geomagneticAttitude1: Quaternion? = estimator.getPrivateProperty("geomagneticAttitude")
        requireNotNull(geomagneticAttitude1)
        assertEquals(Quaternion(), geomagneticAttitude1)

        val hasDeltaRelativeAttitude1: Boolean? =
            estimator.getPrivateProperty("hasDeltaRelativeAttitude")
        requireNotNull(hasDeltaRelativeAttitude1)
        assertTrue(hasDeltaRelativeAttitude1)

        val geomagneticAttitude: Quaternion? = estimator.getPrivateProperty("geomagneticAttitude")
        requireNotNull(geomagneticAttitude)
        val geomagneticAttitudeSpy = spyk(geomagneticAttitude)
        estimator.setPrivateProperty("geomagneticAttitude", geomagneticAttitudeSpy)

        val relativeAttitude: Quaternion? = estimator.getPrivateProperty("relativeAttitude")
        requireNotNull(relativeAttitude)
        val attitude = getAttitude()
        attitude.copyTo(relativeAttitude)
        val relativeAttitudeSpy = spyk(relativeAttitude)
        estimator.setPrivateProperty("relativeAttitude", relativeAttitudeSpy)

        val internalFusedAttitude: Quaternion? = estimator.getPrivateProperty("internalFusedAttitude")
        requireNotNull(internalFusedAttitude)
        val internalFusedAttitudeSpy = spyk(internalFusedAttitude)
        estimator.setPrivateProperty("internalFusedAttitude", internalFusedAttitudeSpy)

        estimator.setPrivateProperty("panicCounter", estimator.panicCounterThreshold)
        val panicCounter1: Int? = estimator.getPrivateProperty("panicCounter")
        requireNotNull(panicCounter1)
        assertEquals(estimator.panicCounterThreshold, panicCounter1)

        val resetToGeomagnetic: Boolean? = estimator.getPrivateProperty("resetToGeomagnetic")
        requireNotNull(resetToGeomagnetic)
        assertTrue(resetToGeomagnetic)

        val geomagneticEstimator: GeomagneticAttitudeEstimator? =
            estimator.getPrivateProperty("geomagneticAttitudeEstimator")
        requireNotNull(geomagneticEstimator)

        val internalAttitude = spyk(getAttitude())
        val listener = geomagneticEstimator.attitudeAvailableListener
        requireNotNull(listener)
        val timestamp = SystemClock.elapsedRealtimeNanos()
        listener.onAttitudeAvailable(
            geomagneticEstimator,
            internalAttitude,
            timestamp,
            null,
            null,
            null,
            null
        )

        val geomagneticAttitude2: Quaternion? = estimator.getPrivateProperty("geomagneticAttitude")
        requireNotNull(geomagneticAttitude2)

        verify { attitudeAvailableListener wasNot Called }
        verify(exactly = 1) { internalAttitude.copyTo(geomagneticAttitude2) }

        val eulerAngles: DoubleArray? = estimator.getPrivateProperty("eulerAngles")
        requireNotNull(eulerAngles)
        verify { geomagneticAttitudeSpy.copyTo(internalFusedAttitudeSpy) }

        val panicCounter2: Int? = estimator.getPrivateProperty("panicCounter")
        requireNotNull(panicCounter2)
        assertEquals(0, panicCounter2)

        val hasRelativeAttitude2: Boolean? = estimator.getPrivateProperty("hasRelativeAttitude")
        requireNotNull(hasRelativeAttitude2)
        assertTrue(hasRelativeAttitude2)
    }

    @Test
    fun processGeomagneticAttitude_whenDeltaRelativeAttitudeSmallDivergenceAndDirectInterpolation_updatesFusedAttitudeAndNotifies() {
        val attitudeAvailableListener =
            mockk<FusedGeomagneticAttitudeEstimator2.OnAttitudeAvailableListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(
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

        val geomagneticAttitude1: Quaternion? = estimator.getPrivateProperty("geomagneticAttitude")
        requireNotNull(geomagneticAttitude1)
        assertEquals(Quaternion(), geomagneticAttitude1)

        val hasDeltaRelativeAttitude1: Boolean? =
            estimator.getPrivateProperty("hasDeltaRelativeAttitude")
        requireNotNull(hasDeltaRelativeAttitude1)
        assertTrue(hasDeltaRelativeAttitude1)

        val geomagneticAttitude: Quaternion? = estimator.getPrivateProperty("geomagneticAttitude")
        requireNotNull(geomagneticAttitude)
        val geomagneticAttitudeSpy = spyk(geomagneticAttitude)
        estimator.setPrivateProperty("geomagneticAttitude", geomagneticAttitudeSpy)

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

        val resetToGeomagnetic: Boolean? = estimator.getPrivateProperty("resetToGeomagnetic")
        requireNotNull(resetToGeomagnetic)
        assertFalse(resetToGeomagnetic)

        val geomagneticEstimator: GeomagneticAttitudeEstimator? =
            estimator.getPrivateProperty("geomagneticAttitudeEstimator")
        requireNotNull(geomagneticEstimator)

        val internalAttitude = spyk(getAttitude())
        val listener = geomagneticEstimator.attitudeAvailableListener
        requireNotNull(listener)
        val timestamp = SystemClock.elapsedRealtimeNanos()
        estimator.setPrivateProperty("relativeAttitudeTimestamp", timestamp)
        listener.onAttitudeAvailable(
            geomagneticEstimator,
            internalAttitude,
            timestamp,
            null,
            null,
            null,
            null
        )

        val geomagneticAttitude2: Quaternion? = estimator.getPrivateProperty("geomagneticAttitude")
        requireNotNull(geomagneticAttitude2)

        verify(exactly = 1) { internalAttitude.copyTo(geomagneticAttitude2) }

        val eulerAngles: DoubleArray? = estimator.getPrivateProperty("eulerAngles")
        requireNotNull(eulerAngles)
        verify(exactly = 0) { geomagneticAttitudeSpy.copyTo(internalFusedAttitudeSpy) }

        val panicCounter2: Int? = estimator.getPrivateProperty("panicCounter")
        requireNotNull(panicCounter2)
        assertEquals(0, panicCounter2)

        val hasRelativeAttitude2: Boolean? = estimator.getPrivateProperty("hasRelativeAttitude")
        requireNotNull(hasRelativeAttitude2)
        assertTrue(hasRelativeAttitude2)

        val fusedAttitude4 = Quaternion()
        Quaternion.slerp(
            fusedAttitude3,
            geomagneticAttitudeSpy,
            estimator.interpolationValue,
            fusedAttitude4
        )

        assertEquals(fusedAttitude4, internalFusedAttitudeSpy)

        val absDot = abs(QuaternionHelper.dotProduct(fusedAttitude3, geomagneticAttitudeSpy))
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
    fun processGeomagneticAttitude_whenDeltaRelativeAttitudeMediumDivergenceAndDirectInterpolation_updatesFusedAttitudeAndNotifies() {
        val attitudeAvailableListener =
            mockk<FusedGeomagneticAttitudeEstimator2.OnAttitudeAvailableListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(
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

        val geomagneticAttitude1: Quaternion? = estimator.getPrivateProperty("geomagneticAttitude")
        requireNotNull(geomagneticAttitude1)
        assertEquals(Quaternion(), geomagneticAttitude1)

        val hasDeltaRelativeAttitude1: Boolean? =
            estimator.getPrivateProperty("hasDeltaRelativeAttitude")
        requireNotNull(hasDeltaRelativeAttitude1)
        assertTrue(hasDeltaRelativeAttitude1)

        val geomagneticAttitude: Quaternion? = estimator.getPrivateProperty("geomagneticAttitude")
        requireNotNull(geomagneticAttitude)
        val geomagneticAttitudeSpy = spyk(geomagneticAttitude)
        estimator.setPrivateProperty("geomagneticAttitude", geomagneticAttitudeSpy)

        val relativeAttitude: Quaternion? = estimator.getPrivateProperty("relativeAttitude")
        requireNotNull(relativeAttitude)
        val attitude = getAttitude()
        attitude.copyTo(relativeAttitude)
        val relativeAttitudeSpy = spyk(relativeAttitude)
        estimator.setPrivateProperty("relativeAttitude", relativeAttitudeSpy)

        val internalFusedAttitude: Quaternion? = estimator.getPrivateProperty("internalFusedAttitude")
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

        val resetToGeomagnetic: Boolean? = estimator.getPrivateProperty("resetToGeomagnetic")
        requireNotNull(resetToGeomagnetic)
        assertFalse(resetToGeomagnetic)

        val geomagneticEstimator: GeomagneticAttitudeEstimator? =
            estimator.getPrivateProperty("geomagneticAttitudeEstimator")
        requireNotNull(geomagneticEstimator)

        val internalAttitude = spyk(getAttitude())
        val listener = geomagneticEstimator.attitudeAvailableListener
        requireNotNull(listener)
        val timestamp = SystemClock.elapsedRealtimeNanos()
        estimator.setPrivateProperty("relativeAttitudeTimestamp", timestamp)
        listener.onAttitudeAvailable(
            geomagneticEstimator,
            internalAttitude,
            timestamp,
            null,
            null,
            null,
            null
        )

        val geomagneticAttitude2: Quaternion? = estimator.getPrivateProperty("geomagneticAttitude")
        requireNotNull(geomagneticAttitude2)

        verify(exactly = 1) { internalAttitude.copyTo(geomagneticAttitude2) }

        val eulerAngles: DoubleArray? = estimator.getPrivateProperty("eulerAngles")
        requireNotNull(eulerAngles)
        verify(exactly = 0) { geomagneticAttitudeSpy.copyTo(internalFusedAttitudeSpy) }

        val panicCounter2: Int? = estimator.getPrivateProperty("panicCounter")
        requireNotNull(panicCounter2)
        assertEquals(1, panicCounter2)

        val hasRelativeAttitude2: Boolean? = estimator.getPrivateProperty("hasRelativeAttitude")
        requireNotNull(hasRelativeAttitude2)
        assertTrue(hasRelativeAttitude2)

        assertEquals(fusedAttitude3, internalFusedAttitudeSpy)

        val absDot = abs(QuaternionHelper.dotProduct(fusedAttitude3, geomagneticAttitudeSpy))
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
    fun processGeomagneticAttitude_whenDeltaRelativeAttitudeLargeDivergenceAndDirectInterpolation_updatesFusedAttitudeAndNotifies() {
        val attitudeAvailableListener =
            mockk<FusedGeomagneticAttitudeEstimator2.OnAttitudeAvailableListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(
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

        val geomagneticAttitude1: Quaternion? = estimator.getPrivateProperty("geomagneticAttitude")
        requireNotNull(geomagneticAttitude1)
        assertEquals(Quaternion(), geomagneticAttitude1)

        val hasDeltaRelativeAttitude1: Boolean? =
            estimator.getPrivateProperty("hasDeltaRelativeAttitude")
        requireNotNull(hasDeltaRelativeAttitude1)
        assertTrue(hasDeltaRelativeAttitude1)

        val geomagneticAttitude: Quaternion? = estimator.getPrivateProperty("geomagneticAttitude")
        requireNotNull(geomagneticAttitude)
        val geomagneticAttitudeSpy = spyk(geomagneticAttitude)
        estimator.setPrivateProperty("geomagneticAttitude", geomagneticAttitudeSpy)

        val relativeAttitude: Quaternion? = estimator.getPrivateProperty("relativeAttitude")
        requireNotNull(relativeAttitude)
        val attitude = getAttitude()
        attitude.copyTo(relativeAttitude)
        val relativeAttitudeSpy = spyk(relativeAttitude)
        estimator.setPrivateProperty("relativeAttitude", relativeAttitudeSpy)

        val internalFusedAttitude: Quaternion? = estimator.getPrivateProperty("internalFusedAttitude")
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

        val resetToGeomagnetic: Boolean? = estimator.getPrivateProperty("resetToGeomagnetic")
        requireNotNull(resetToGeomagnetic)
        assertFalse(resetToGeomagnetic)

        val geomagneticEstimator: GeomagneticAttitudeEstimator? =
            estimator.getPrivateProperty("geomagneticAttitudeEstimator")
        requireNotNull(geomagneticEstimator)

        val internalAttitude = spyk(getAttitude())
        val listener = geomagneticEstimator.attitudeAvailableListener
        requireNotNull(listener)
        val timestamp = SystemClock.elapsedRealtimeNanos()
        estimator.setPrivateProperty("relativeAttitudeTimestamp", timestamp)
        listener.onAttitudeAvailable(
            geomagneticEstimator,
            internalAttitude,
            timestamp,
            null,
            null,
            null,
            null
        )

        val geomagneticAttitude2: Quaternion? = estimator.getPrivateProperty("geomagneticAttitude")
        requireNotNull(geomagneticAttitude2)

        verify(exactly = 1) { internalAttitude.copyTo(geomagneticAttitude2) }

        val eulerAngles: DoubleArray? = estimator.getPrivateProperty("eulerAngles")
        requireNotNull(eulerAngles)
        verify(exactly = 0) { geomagneticAttitudeSpy.copyTo(internalFusedAttitudeSpy) }

        val panicCounter2: Int? = estimator.getPrivateProperty("panicCounter")
        requireNotNull(panicCounter2)
        assertEquals(2, panicCounter2)

        val hasRelativeAttitude2: Boolean? = estimator.getPrivateProperty("hasRelativeAttitude")
        requireNotNull(hasRelativeAttitude2)
        assertTrue(hasRelativeAttitude2)

        assertEquals(fusedAttitude3, internalFusedAttitudeSpy)

        val absDot = abs(QuaternionHelper.dotProduct(fusedAttitude3, geomagneticAttitudeSpy))
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
    fun processGeomagneticAttitude_whenIndirectInterpolation_updatesFusedAttitudeAndNotifies() {
        val attitudeAvailableListener =
            mockk<FusedGeomagneticAttitudeEstimator2.OnAttitudeAvailableListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(
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

        val geomagneticAttitude1: Quaternion? = estimator.getPrivateProperty("geomagneticAttitude")
        requireNotNull(geomagneticAttitude1)
        assertEquals(Quaternion(), geomagneticAttitude1)

        val hasDeltaRelativeAttitude1: Boolean? =
            estimator.getPrivateProperty("hasDeltaRelativeAttitude")
        requireNotNull(hasDeltaRelativeAttitude1)
        assertTrue(hasDeltaRelativeAttitude1)

        val geomagneticAttitude: Quaternion? = estimator.getPrivateProperty("geomagneticAttitude")
        requireNotNull(geomagneticAttitude)
        val geomagneticAttitudeSpy = spyk(geomagneticAttitude)
        estimator.setPrivateProperty("geomagneticAttitude", geomagneticAttitudeSpy)

        val relativeAttitude: Quaternion? = estimator.getPrivateProperty("relativeAttitude")
        requireNotNull(relativeAttitude)
        val attitude = getAttitude()
        attitude.copyTo(relativeAttitude)
        val relativeAttitudeSpy = spyk(relativeAttitude)
        estimator.setPrivateProperty("relativeAttitude", relativeAttitudeSpy)

        val internalFusedAttitude: Quaternion? = estimator.getPrivateProperty("internalFusedAttitude")
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

        val resetToGeomagnetic: Boolean? = estimator.getPrivateProperty("resetToGeomagnetic")
        requireNotNull(resetToGeomagnetic)
        assertFalse(resetToGeomagnetic)

        val relativeAttitudeEstimator: LeveledRelativeAttitudeEstimator? =
            estimator.getPrivateProperty("relativeAttitudeEstimator")
        requireNotNull(relativeAttitudeEstimator)
        val relativeAttitudeEstimatorSpy = spyk(relativeAttitudeEstimator)
        every { relativeAttitudeEstimatorSpy.gyroscopeAverageTimeInterval }.returns(TIME_INTERVAL)
        estimator.setPrivateProperty("relativeAttitudeEstimator", relativeAttitudeEstimatorSpy)

        val geomagneticEstimator: GeomagneticAttitudeEstimator? =
            estimator.getPrivateProperty("geomagneticAttitudeEstimator")
        requireNotNull(geomagneticEstimator)

        val internalAttitude = spyk(getAttitude())
        val listener = geomagneticEstimator.attitudeAvailableListener
        requireNotNull(listener)
        val timestamp = SystemClock.elapsedRealtimeNanos()
        estimator.setPrivateProperty("relativeAttitudeTimestamp", timestamp)
        listener.onAttitudeAvailable(
            geomagneticEstimator,
            internalAttitude,
            timestamp,
            null,
            null,
            null,
            null
        )

        val geomagneticAttitude2: Quaternion? = estimator.getPrivateProperty("geomagneticAttitude")
        requireNotNull(geomagneticAttitude2)

        verify(exactly = 1) { internalAttitude.copyTo(geomagneticAttitude2) }

        val eulerAngles: DoubleArray? = estimator.getPrivateProperty("eulerAngles")
        requireNotNull(eulerAngles)
        verify(exactly = 0) { geomagneticAttitudeSpy.copyTo(internalFusedAttitudeSpy) }

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
            geomagneticAttitudeSpy,
            t,
            fusedAttitude4
        )


        assertEquals(fusedAttitude4, internalFusedAttitudeSpy)

        val absDot = abs(QuaternionHelper.dotProduct(fusedAttitude3, geomagneticAttitudeSpy))
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
    fun processGeomagneticAttitude_whenEstimateEulerAnglesAndCoordinateTransformationEnabled_notifies() {
        val attitudeAvailableListener =
            mockk<FusedGeomagneticAttitudeEstimator2.OnAttitudeAvailableListener>(relaxUnitFun = true)
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = FusedGeomagneticAttitudeEstimator2(
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

        val geomagneticAttitude1: Quaternion? = estimator.getPrivateProperty("geomagneticAttitude")
        requireNotNull(geomagneticAttitude1)
        assertEquals(Quaternion(), geomagneticAttitude1)

        val hasDeltaRelativeAttitude1: Boolean? =
            estimator.getPrivateProperty("hasDeltaRelativeAttitude")
        requireNotNull(hasDeltaRelativeAttitude1)
        assertTrue(hasDeltaRelativeAttitude1)

        val geomagneticAttitude: Quaternion? = estimator.getPrivateProperty("geomagneticAttitude")
        requireNotNull(geomagneticAttitude)
        val geomagneticAttitudeSpy = spyk(geomagneticAttitude)
        estimator.setPrivateProperty("geomagneticAttitude", geomagneticAttitudeSpy)

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

        val resetToGeomagnetic: Boolean? = estimator.getPrivateProperty("resetToGeomagnetic")
        requireNotNull(resetToGeomagnetic)
        assertFalse(resetToGeomagnetic)

        val geomagneticEstimator: GeomagneticAttitudeEstimator? =
            estimator.getPrivateProperty("geomagneticAttitudeEstimator")
        requireNotNull(geomagneticEstimator)

        val internalAttitude = spyk(getAttitude())
        val listener = geomagneticEstimator.attitudeAvailableListener
        requireNotNull(listener)
        val timestamp = SystemClock.elapsedRealtimeNanos()
        estimator.setPrivateProperty("relativeAttitudeTimestamp", timestamp)
        listener.onAttitudeAvailable(
            geomagneticEstimator,
            internalAttitude,
            timestamp,
            null,
            null,
            null,
            null
        )

        val geomagneticAttitude2: Quaternion? = estimator.getPrivateProperty("geomagneticAttitude")
        requireNotNull(geomagneticAttitude2)

        verify(exactly = 1) { internalAttitude.copyTo(geomagneticAttitude2) }

        val eulerAngles: DoubleArray? = estimator.getPrivateProperty("eulerAngles")
        requireNotNull(eulerAngles)
        verify(exactly = 0) { geomagneticAttitudeSpy.copyTo(internalFusedAttitudeSpy) }

        val panicCounter2: Int? = estimator.getPrivateProperty("panicCounter")
        requireNotNull(panicCounter2)
        assertEquals(0, panicCounter2)

        val hasRelativeAttitude2: Boolean? = estimator.getPrivateProperty("hasRelativeAttitude")
        requireNotNull(hasRelativeAttitude2)
        assertTrue(hasRelativeAttitude2)

        val fusedAttitude4 = Quaternion()
        Quaternion.slerp(
            fusedAttitude3,
            geomagneticAttitudeSpy,
            estimator.interpolationValue,
            fusedAttitude4
        )

        assertEquals(fusedAttitude4, internalFusedAttitudeSpy)

        val absDot = abs(QuaternionHelper.dotProduct(fusedAttitude3, geomagneticAttitudeSpy))
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