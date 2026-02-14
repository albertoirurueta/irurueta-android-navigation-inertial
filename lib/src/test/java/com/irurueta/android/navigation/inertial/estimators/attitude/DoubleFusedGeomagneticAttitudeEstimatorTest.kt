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
import com.irurueta.android.navigation.inertial.collectors.AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector
import com.irurueta.android.navigation.inertial.collectors.GravityGyroscopeAndMagnetometerSyncedSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.GravityGyroscopeAndMagnetometerSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.MagnetometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorType
import com.irurueta.android.navigation.inertial.processors.attitude.AccelerometerDoubleFusedGeomagneticAttitudeProcessor
import com.irurueta.android.navigation.inertial.processors.attitude.BaseDoubleFusedGeomagneticAttitudeProcessor
import com.irurueta.android.navigation.inertial.processors.attitude.BaseFusedGeomagneticAttitudeProcessor
import com.irurueta.android.navigation.inertial.processors.attitude.DoubleFusedGeomagneticAttitudeProcessor
import com.irurueta.android.navigation.inertial.processors.filters.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.processors.filters.MedianAveragingFilter
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.android.testutils.setPrivateProperty
import com.irurueta.geometry.Quaternion
import com.irurueta.navigation.frames.CoordinateTransformation
import com.irurueta.navigation.frames.FrameType
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.wmm.WorldMagneticModel
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
import java.util.Date

class DoubleFusedGeomagneticAttitudeEstimatorTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK(relaxUnitFun = true)
    private lateinit var attitudeListener:
            DoubleFusedGeomagneticAttitudeEstimator.OnAttitudeAvailableListener

    @MockK(relaxUnitFun = true)
    private lateinit var accuracyChangedListener:
            DoubleFusedGeomagneticAttitudeEstimator.OnAccuracyChangedListener

    @MockK
    private lateinit var location: Location

    @MockK
    private lateinit var context: Context

    @MockK
    private lateinit var sensorManager: SensorManager

    @MockK
    private lateinit var accelerometerSensor: Sensor

    @MockK
    private lateinit var gyroscopeSensor: Sensor

    @MockK
    private lateinit var magnetometerSensor: Sensor

    @MockK
    private lateinit var gravitySensor: Sensor

    @Test
    fun constructor_whenRequiredProperties_setsDefaultValues() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context)

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
        assertFalse(estimator.useAccurateLevelingEstimator)
        assertFalse(estimator.useAccurateRelativeGyroscopeAttitudeEstimator)
        assertFalse(estimator.estimateCoordinateTransformation)
        assertTrue(estimator.estimateEulerAngles)
        assertNull(estimator.attitudeAvailableListener)
        assertNull(estimator.accuracyChangedListener)
        assertTrue(estimator.useIndirectInterpolation)
        assertEquals(
            BaseDoubleFusedGeomagneticAttitudeProcessor.DEFAULT_INTERPOLATION_VALUE,
            estimator.interpolationValue,
            0.0
        )
        assertEquals(
            BaseDoubleFusedGeomagneticAttitudeProcessor.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            estimator.indirectInterpolationWeight,
            0.0
        )
        assertEquals(0.0, estimator.gyroscopeTimeIntervalSeconds, 0.0)
        assertEquals(
            BaseDoubleFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_THRESHOLD,
            estimator.outlierThreshold,
            0.0
        )
        assertEquals(
            BaseDoubleFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            estimator.outlierPanicThreshold,
            0.0
        )
        assertEquals(
            BaseDoubleFusedGeomagneticAttitudeProcessor.DEFAULT_PANIC_COUNTER_THRESHOLD,
            estimator.panicCounterThreshold
        )
        assertFalse(estimator.running)
    }

    @Test
    fun constructor_whenAllProperties_setsExpectedValues() {
        val location = getLocation()
        val accelerometerAveragingFilter = MedianAveragingFilter<AccelerationUnit, Acceleration, AccelerationTriad>()
        val worldMagneticModel = WorldMagneticModel()
        val timestamp = Date()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(
            context,
            location,
            SensorDelay.NORMAL,
            useAccelerometer = false,
            AccelerometerSensorType.ACCELEROMETER,
            MagnetometerSensorType.MAGNETOMETER,
            accelerometerAveragingFilter,
            GyroscopeSensorType.GYROSCOPE,
            worldMagneticModel,
            timestamp,
            useWorldMagneticModel = true,
            useAccurateLevelingEstimator = true,
            useAccurateRelativeGyroscopeAttitudeEstimator = true,
            estimateCoordinateTransformation = true,
            estimateEulerAngles = false,
            attitudeAvailableListener = attitudeListener,
            accuracyChangedListener = accuracyChangedListener,
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
        assertEquals(timestamp, estimator.timestamp)
        assertTrue(estimator.useWorldMagneticModel)
        assertTrue(estimator.useAccurateLevelingEstimator)
        assertTrue(estimator.useAccurateRelativeGyroscopeAttitudeEstimator)
        assertTrue(estimator.estimateCoordinateTransformation)
        assertFalse(estimator.estimateEulerAngles)
        assertSame(attitudeListener, estimator.attitudeAvailableListener)
        assertSame(accuracyChangedListener, estimator.accuracyChangedListener)
        assertTrue(estimator.useIndirectInterpolation)
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_INTERPOLATION_VALUE,
            estimator.interpolationValue,
            0.0
        )
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            estimator.indirectInterpolationWeight,
            0.0
        )
        assertEquals(0.0, estimator.gyroscopeTimeIntervalSeconds, 0.0)
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_THRESHOLD,
            estimator.outlierThreshold,
            0.0
        )
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            estimator.outlierPanicThreshold,
            0.0
        )
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_PANIC_COUNTER_THRESHOLD,
            estimator.panicCounterThreshold
        )
        assertFalse(estimator.running)
    }

    @Test
    fun location_whenNotRunning_setsExpectedValue() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context)

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
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context)

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
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context, location)

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
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context)

        // check default value
        assertTrue(estimator.adjustGravityNorm)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        assertTrue(gravityProcessor.adjustGravityNorm)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
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
    fun worldMagneticModel_whenNotRunning_setsExpectedValue() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context)

        // check default value
        assertFalse(estimator.running)
        assertNull(estimator.worldMagneticModel)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        assertNull(accelerometerProcessor.worldMagneticModel)
        assertNull(gravityProcessor.worldMagneticModel)

        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        // set new value
        val worldMagneticModel = WorldMagneticModel()
        estimator.worldMagneticModel = worldMagneticModel

        // check
        assertSame(worldMagneticModel, estimator.worldMagneticModel)

        verify { accelerometerProcessorSpy.worldMagneticModel = worldMagneticModel }
        verify { gravityProcessorSpy.worldMagneticModel = worldMagneticModel }

        assertSame(worldMagneticModel, accelerometerProcessor.worldMagneticModel)
        assertSame(worldMagneticModel, gravityProcessor.worldMagneticModel)
    }

    @Test
    fun worldMagneticModel_whenRunning_throwsIllegalStateException() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context)

        // set as running
        estimator.setPrivateProperty("running", true)

        assertThrows(IllegalStateException::class.java) {
            estimator.worldMagneticModel = WorldMagneticModel()
        }
    }

    @Test
    fun timestamp_setsExpectedValue() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context)

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
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context)

        // check default value
        assertNull(estimator.attitudeAvailableListener)

        // set new value
        estimator.attitudeAvailableListener = attitudeListener

        // check
        assertSame(attitudeListener, estimator.attitudeAvailableListener)
    }

    @Test
    fun accuracyChangedListener_setsExpectedValue() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context)

        // check default value
        assertNull(estimator.accuracyChangedListener)

        // set new value
        estimator.accuracyChangedListener = accuracyChangedListener

        // check
        assertSame(accuracyChangedListener, estimator.accuracyChangedListener)
    }

    @Test
    fun useWorldMagneticModel_whenNotRunning_setsExpectedValue() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context)

        // check
        assertFalse(estimator.running)
        assertFalse(estimator.useWorldMagneticModel)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        assertFalse(accelerometerProcessor.useWorldMagneticModel)
        assertFalse(gravityProcessor.useWorldMagneticModel)

        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        // set new value
        estimator.useWorldMagneticModel = true

        // check
        assertTrue(estimator.useWorldMagneticModel)

        verify(exactly = 1) { accelerometerProcessorSpy.useWorldMagneticModel = true }
        verify(exactly = 1) { gravityProcessorSpy.useWorldMagneticModel = true }

        assertTrue(accelerometerProcessor.useWorldMagneticModel)
        assertTrue(gravityProcessor.useWorldMagneticModel)
    }

    @Test
    fun useWorldMagneticModel_whenRunning_throwsIllegalStateException() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        assertThrows(IllegalStateException::class.java) {
            estimator.useWorldMagneticModel = true
        }
    }

    @Test
    fun useAccurateLevelingEstimator_whenRunning_throwsIllegalStateException() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context)

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
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context)

        assertFalse(estimator.useAccurateLevelingEstimator)
        assertNull(estimator.location)
        assertFalse(estimator.running)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        estimator.useAccurateLevelingEstimator = false

        // check
        assertFalse(estimator.useAccurateLevelingEstimator)
        assertFalse(accelerometerProcessor.useAccurateLevelingProcessor)
        assertFalse(gravityProcessor.useAccurateLevelingProcessor)

        verify(exactly = 1) { accelerometerProcessorSpy.useAccurateLevelingProcessor = false }
        verify(exactly = 1) { gravityProcessorSpy.useAccurateLevelingProcessor = false }
    }

    @Test
    fun useAccurateLevelingEstimator_whenNotRunningNoLocationAndSetToTrue_throwsIllegalStateException() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context)

        assertFalse(estimator.useAccurateLevelingEstimator)
        assertNull(estimator.location)
        assertFalse(estimator.running)

        assertThrows(IllegalStateException::class.java) {
            estimator.useAccurateLevelingEstimator = true
        }
    }

    @Test
    fun useAccurateLevelingEstimator_whenNotRunningAndLocationAndSetToTrue_setsExpectedValue() {
        val location = getLocation()
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context, location)

        // check default value
        assertFalse(estimator.running)
        assertSame(location, estimator.location)
        assertFalse(estimator.useAccurateLevelingEstimator)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        // set new value
        estimator.useAccurateLevelingEstimator = true

        // check
        assertTrue(estimator.useAccurateLevelingEstimator)
        assertTrue(accelerometerProcessor.useAccurateLevelingProcessor)
        assertTrue(gravityProcessor.useAccurateLevelingProcessor)

        verify(exactly = 1) { accelerometerProcessorSpy.useAccurateLevelingProcessor = true }
        verify(exactly = 1) { gravityProcessorSpy.useAccurateLevelingProcessor = true }
    }

    @Test
    fun useAccurateRelativeGyroscopeAttitudeEstimator_whenRunning_throwsIllegalStateException() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context)

        assertFalse(estimator.running)

        // set as running
        estimator.setPrivateProperty("running", true)

        assertTrue(estimator.running)

        assertThrows(IllegalStateException::class.java) {
            estimator.useAccurateRelativeGyroscopeAttitudeEstimator = true
        }
    }

    @Test
    fun useAccurateRelativeGyroscopeAttitudeEstimator_whenNotRunning_setsExpectedValue() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context)

        assertFalse(estimator.running)
        assertFalse(estimator.useAccurateRelativeGyroscopeAttitudeEstimator)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        // set new value
        estimator.useAccurateRelativeGyroscopeAttitudeEstimator = true

        // check
        assertTrue(estimator.useAccurateRelativeGyroscopeAttitudeEstimator)
        assertTrue(accelerometerProcessorSpy.useAccurateRelativeGyroscopeAttitudeProcessor)
        assertTrue(gravityProcessorSpy.useAccurateRelativeGyroscopeAttitudeProcessor)

        verify(exactly = 1) {
            accelerometerProcessorSpy.useAccurateRelativeGyroscopeAttitudeProcessor = true
        }
        verify(exactly = 1) {
            gravityProcessorSpy.useAccurateRelativeGyroscopeAttitudeProcessor = true
        }
    }

    @Test
    fun useIndirectInterpolation_setsExpectedValue() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context)

        // check default value
        assertTrue(estimator.useIndirectInterpolation)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        // set new value
        estimator.useIndirectInterpolation = false

        // check
        assertFalse(estimator.useIndirectInterpolation)
        assertFalse(accelerometerProcessorSpy.useIndirectInterpolation)
        assertFalse(gravityProcessorSpy.useIndirectInterpolation)

        verify(exactly = 1) { accelerometerProcessorSpy.useIndirectInterpolation = false }
        verify(exactly = 1) { gravityProcessorSpy.useIndirectInterpolation = false }
    }

    @Test
    fun interpolationValue_whenOutOfRange_throwsIllegalArgumentException() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context)

        assertEquals(
            BaseDoubleFusedGeomagneticAttitudeProcessor.DEFAULT_INTERPOLATION_VALUE,
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
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context)

        assertEquals(
            com.irurueta.android.navigation.inertial.old.processors.attitude.BaseDoubleFusedGeomagneticAttitudeProcessor.DEFAULT_INTERPOLATION_VALUE,
            estimator.interpolationValue,
            0.0
        )

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val randomizer = UniformRandomizer()
        val value = randomizer.nextDouble()
        estimator.interpolationValue = value

        // check
        assertEquals(value, estimator.interpolationValue, 0.0)
        assertEquals(value, accelerometerProcessorSpy.interpolationValue, 0.0)
        assertEquals(value, gravityProcessorSpy.interpolationValue, 0.0)

        verify(exactly = 1) { accelerometerProcessorSpy.interpolationValue = value }
        verify(exactly = 1) { gravityProcessorSpy.interpolationValue = value }
    }

    @Test
    fun indirectInterpolationWeight_whenInvalid_throwsIllegalArgumentException() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context)

        assertThrows(IllegalArgumentException::class.java) {
            estimator.indirectInterpolationWeight = 0.0
        }
    }

    @Test
    fun indirectInterpolationWeight_whenValid_setsExpectedValue() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context)

        // check default value
        assertEquals(
            BaseDoubleFusedGeomagneticAttitudeProcessor.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            estimator.indirectInterpolationWeight,
            0.0
        )

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        // sets new value
        val randomizer = UniformRandomizer()
        val indirectInterpolationWeight = randomizer.nextDouble()
        estimator.indirectInterpolationWeight = indirectInterpolationWeight

        // check
        assertEquals(indirectInterpolationWeight, estimator.indirectInterpolationWeight, 0.0)
        assertEquals(
            indirectInterpolationWeight,
            accelerometerProcessorSpy.indirectInterpolationWeight,
            0.0
        )
        assertEquals(
            indirectInterpolationWeight,
            gravityProcessorSpy.indirectInterpolationWeight,
            0.0
        )

        verify(exactly = 1) {
            accelerometerProcessorSpy.indirectInterpolationWeight = indirectInterpolationWeight
        }
        verify(exactly = 1) {
            gravityProcessorSpy.indirectInterpolationWeight = indirectInterpolationWeight
        }
    }

    @Test
    fun gyroscopeTimeIntervalSeconds_whenUseAccelerometer_returnsExpectedValue() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context, useAccelerometer = true)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        every { accelerometerProcessorSpy.gyroscopeTimeIntervalSeconds }.returns(
            TIME_INTERVAL
        )
        val gravityProcessorSpy = spyk(gravityProcessor)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        assertEquals(TIME_INTERVAL, estimator.gyroscopeTimeIntervalSeconds, 0.0)

        verify(exactly = 1) { accelerometerProcessorSpy.gyroscopeTimeIntervalSeconds }
        verify { gravityProcessorSpy wasNot Called }
    }

    @Test
    fun gyroscopeTimeIntervalSeconds_whenNotUseAccelerometer_returnsExpectedValue() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context, useAccelerometer = false)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        every { gravityProcessorSpy.gyroscopeTimeIntervalSeconds }.returns(
            TIME_INTERVAL
        )
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        assertEquals(TIME_INTERVAL, estimator.gyroscopeTimeIntervalSeconds, 0.0)

        verify(exactly = 1) { gravityProcessorSpy.gyroscopeTimeIntervalSeconds }
        verify { accelerometerProcessorSpy wasNot Called }
    }

    @Test
    fun outlierThreshold_whenRunning_throwsIllegalStateException() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context)

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
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context)

        // check default value
        assertEquals(
            BaseDoubleFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_THRESHOLD,
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
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context)

        // check default value
        assertEquals(
            BaseDoubleFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_THRESHOLD,
            estimator.outlierThreshold,
            0.0
        )
        assertFalse(estimator.running)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        // set new value
        val randomizer = UniformRandomizer()
        val outlierThreshold = randomizer.nextDouble()

        estimator.outlierThreshold = outlierThreshold

        // check
        assertEquals(outlierThreshold, estimator.outlierThreshold, 0.0)
        assertEquals(outlierThreshold, accelerometerProcessorSpy.outlierThreshold, 0.0)
        assertEquals(outlierThreshold, gravityProcessorSpy.outlierThreshold, 0.0)

        verify(exactly = 1) { accelerometerProcessorSpy.outlierThreshold = outlierThreshold }
        verify(exactly = 1) { gravityProcessorSpy.outlierThreshold = outlierThreshold }
    }

    @Test
    fun outlierPanicThreshold_whenRunning_throwsIllegalStateException() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context)

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
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context)

        // check default value
        assertEquals(
            BaseDoubleFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            estimator.outlierPanicThreshold,
            0.0
        )
        assertFalse(estimator.running)

        // set invalid values
        assertThrows(IllegalArgumentException::class.java) {
            estimator.outlierPanicThreshold = -1.0
        }
        assertThrows(IllegalArgumentException::class.java) {
            estimator.outlierPanicThreshold = 2.0
        }
    }

    @Test
    fun outlierPanicThreshold_whenValid_setsExpectedValue() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context)

        // check default value
        assertEquals(
            BaseDoubleFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            estimator.outlierPanicThreshold,
            0.0
        )
        assertFalse(estimator.running)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        // set new value
        val randomizer = UniformRandomizer()
        val outlierPanicThreshold = randomizer.nextDouble()
        estimator.outlierPanicThreshold = outlierPanicThreshold

        // check
        assertEquals(outlierPanicThreshold, estimator.outlierPanicThreshold, 0.0)
        assertEquals(outlierPanicThreshold, accelerometerProcessorSpy.outlierPanicThreshold, 0.0)
        assertEquals(outlierPanicThreshold, gravityProcessorSpy.outlierPanicThreshold, 0.0)

        verify(exactly = 1) {
            accelerometerProcessorSpy.outlierPanicThreshold = outlierPanicThreshold
        }
        verify(exactly = 1) { gravityProcessorSpy.outlierPanicThreshold = outlierPanicThreshold }
    }

    @Test
    fun panicCounterThreshold_whenRunning_throwsIllegalStateException() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        assertThrows(IllegalStateException::class.java) {
            estimator.panicCounterThreshold = 1
        }
    }

    @Test
    fun panicCounterThreshold_whenInvalid_throwsIllegalArgumentException() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context)

        assertFalse(estimator.running)

        assertThrows(IllegalArgumentException::class.java) {
            estimator.panicCounterThreshold = 0
        }
    }

    @Test
    fun panicCounterThreshold_whenValid_setsExpectedValue() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context)

        assertFalse(estimator.running)
        assertEquals(
            BaseDoubleFusedGeomagneticAttitudeProcessor.DEFAULT_PANIC_COUNTER_THRESHOLD,
            estimator.panicCounterThreshold
        )

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        estimator.panicCounterThreshold = 2

        // check
        assertEquals(2, estimator.panicCounterThreshold)
        assertEquals(2, accelerometerProcessorSpy.panicCounterThreshold)
        assertEquals(2, gravityProcessorSpy.panicCounterThreshold)

        verify(exactly = 1) { accelerometerProcessorSpy.panicCounterThreshold = 2 }
        verify(exactly = 1) { gravityProcessorSpy.panicCounterThreshold = 2 }
    }

    @Test
    fun start_whenRunning_throwsIllegalStateException() {
        mockkStatic(SystemClock::class) {
            val startTimestamp = System.nanoTime()
            every { SystemClock.elapsedRealtimeNanos() }.returns(startTimestamp)

            val estimator = DoubleFusedGeomagneticAttitudeEstimator(context)

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
    fun start_whenUseAccelerometerAndInternalSyncerFails_returnsFalse() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context, useAccelerometer = true)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val accelerometerGyroscopeAndMagnetometerCollector: AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerGyroscopeAndMagnetometerCollector")
        requireNotNull(accelerometerGyroscopeAndMagnetometerCollector)

        val gravityGyroscopeAndMagnetometerCollector: GravityGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("gravityGyroscopeAndMagnetometerCollector")
        requireNotNull(gravityGyroscopeAndMagnetometerCollector)

        val accelerometerGyroscopeAndMagnetometerCollectorSpy =
            spyk(accelerometerGyroscopeAndMagnetometerCollector)
        every { accelerometerGyroscopeAndMagnetometerCollectorSpy.start(any()) }.returns(false)
        val gravityGyroscopeAndMagnetometerCollectorSpy = spyk(gravityGyroscopeAndMagnetometerCollector)

        estimator.setPrivateProperty(
            "accelerometerGyroscopeAndMagnetometerCollector",
            accelerometerGyroscopeAndMagnetometerCollectorSpy
        )
        estimator.setPrivateProperty(
            "gravityGyroscopeAndMagnetometerSyncer",
            gravityGyroscopeAndMagnetometerCollectorSpy
        )

        // start
        val startTimestamp = System.nanoTime()
        assertFalse(estimator.start(startTimestamp))

        verify(exactly = 1) { accelerometerProcessorSpy.reset() }
        verify(exactly = 1) { accelerometerGyroscopeAndMagnetometerCollectorSpy.start(startTimestamp) }
        verify { gravityProcessorSpy wasNot Called }
        verify { gravityGyroscopeAndMagnetometerCollectorSpy wasNot Called }
    }

    @Test
    fun start_whenNotUseAccelerometerAndInternalCollectorFails_returnsFalse() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }
            .returns(gyroscopeSensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }
            .returns(magnetometerSensor)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }
            .returns(gravitySensor)
        every { sensorManager.registerListener(any(), any<Sensor>(), any()) }
            .returns(false)

        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context, useAccelerometer = false)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val accelerometerGyroscopeAndMagnetometerCollector: AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerGyroscopeAndMagnetometerCollector")
        requireNotNull(accelerometerGyroscopeAndMagnetometerCollector)

        val gravityGyroscopeAndMagnetometerCollector: GravityGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("gravityGyroscopeAndMagnetometerCollector")
        requireNotNull(gravityGyroscopeAndMagnetometerCollector)

        val accelerometerGyroscopeAndMagnetometerCollectorSpy =
            spyk(accelerometerGyroscopeAndMagnetometerCollector)
        val gravityGyroscopeAndMagnetometerCollectorSpy = spyk(gravityGyroscopeAndMagnetometerCollector)
        every { gravityGyroscopeAndMagnetometerCollectorSpy.start(any()) }.returns(false)

        estimator.setPrivateProperty(
            "accelerometerGyroscopeAndMagnetometerCollector",
            accelerometerGyroscopeAndMagnetometerCollectorSpy
        )
        estimator.setPrivateProperty(
            "gravityGyroscopeAndMagnetometerCollector",
            gravityGyroscopeAndMagnetometerCollectorSpy
        )

        // start
        val startTimestamp = System.nanoTime()
        assertFalse(estimator.start(startTimestamp))

        verify(exactly = 1) { gravityProcessorSpy.reset() }
        verify(exactly = 1) { gravityGyroscopeAndMagnetometerCollectorSpy.start(startTimestamp) }
        verify { accelerometerProcessorSpy wasNot Called }
        verify { accelerometerGyroscopeAndMagnetometerCollectorSpy wasNot Called }
    }

    @Test
    fun start_whenUseAccelerometerAndInternalSyncerSucceeds_returnsTrue() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context, useAccelerometer = true)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val accelerometerGyroscopeAndMagnetometerCollector: AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerGyroscopeAndMagnetometerCollector")
        requireNotNull(accelerometerGyroscopeAndMagnetometerCollector)

        val gravityGyroscopeAndMagnetometerCollector: GravityGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("gravityGyroscopeAndMagnetometerCollector")
        requireNotNull(gravityGyroscopeAndMagnetometerCollector)

        val accelerometerGyroscopeAndMagnetometerCollectorSpy =
            spyk(accelerometerGyroscopeAndMagnetometerCollector)
        every { accelerometerGyroscopeAndMagnetometerCollectorSpy.start(any()) }.returns(true)
        val gravityGyroscopeAndMagnetometerCollectorSpy = spyk(gravityGyroscopeAndMagnetometerCollector)

        estimator.setPrivateProperty(
            "accelerometerGyroscopeAndMagnetometerCollector",
            accelerometerGyroscopeAndMagnetometerCollectorSpy
        )
        estimator.setPrivateProperty(
            "gravityGyroscopeAndMagnetometerCollector",
            gravityGyroscopeAndMagnetometerCollectorSpy
        )

        // start
        val startTimestamp = System.nanoTime()
        assertTrue(estimator.start(startTimestamp))

        verify(exactly = 1) { accelerometerProcessorSpy.reset() }
        verify(exactly = 1) { accelerometerGyroscopeAndMagnetometerCollectorSpy.start(startTimestamp) }
        verify { gravityProcessorSpy wasNot Called }
        verify { gravityGyroscopeAndMagnetometerCollectorSpy wasNot Called }
    }

    @Test
    fun start_whenNotUseAccelerometerAndInternalSyncerSucceeds_returnsTrue() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context, useAccelerometer = false)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)

        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val accelerometerGyroscopeAndMagnetometerCollector: AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerGyroscopeAndMagnetometerCollector")
        requireNotNull(accelerometerGyroscopeAndMagnetometerCollector)

        val gravityGyroscopeAndMagnetometerCollector: GravityGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("gravityGyroscopeAndMagnetometerCollector")
        requireNotNull(gravityGyroscopeAndMagnetometerCollector)

        val accelerometerGyroscopeAndMagnetometerSyncerSpy =
            spyk(accelerometerGyroscopeAndMagnetometerCollector)
        val gravityGyroscopeAndMagnetometerSyncerSpy = spyk(gravityGyroscopeAndMagnetometerCollector)
        every { gravityGyroscopeAndMagnetometerSyncerSpy.start(any()) }.returns(true)

        estimator.setPrivateProperty(
            "accelerometerGyroscopeAndMagnetometerCollector",
            accelerometerGyroscopeAndMagnetometerSyncerSpy
        )
        estimator.setPrivateProperty(
            "gravityGyroscopeAndMagnetometerCollector",
            gravityGyroscopeAndMagnetometerSyncerSpy
        )

        // start
        val startTimestamp = System.nanoTime()
        assertTrue(estimator.start(startTimestamp))

        verify(exactly = 1) { gravityProcessorSpy.reset() }
        verify(exactly = 1) { gravityGyroscopeAndMagnetometerSyncerSpy.start(startTimestamp) }
        verify { accelerometerProcessorSpy wasNot Called }
        verify { accelerometerGyroscopeAndMagnetometerSyncerSpy wasNot Called }
    }

    @Test
    fun stop_stopsInternalSyncersAndUpdatesRunningStatus() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }
            .returns(accelerometerSensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }
            .returns(gyroscopeSensor)
        every { sensorManager.getDefaultSensor(MagnetometerSensorType.MAGNETOMETER_UNCALIBRATED.value) }
            .returns(magnetometerSensor)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }
            .returns(gravitySensor)
        justRun { sensorManager.unregisterListener(any(), any<Sensor>()) }

        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context)

        // set as running
        estimator.setPrivateProperty("running", true)
        assertTrue(estimator.running)

        val accelerometerGyroscopeAndMagnetometerCollector: AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerGyroscopeAndMagnetometerCollector")
        requireNotNull(accelerometerGyroscopeAndMagnetometerCollector)

        val gravityGyroscopeAndMagnetometerCollector: GravityGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("gravityGyroscopeAndMagnetometerCollector")
        requireNotNull(gravityGyroscopeAndMagnetometerCollector)

        val accelerometerGyroscopeAndMagnetometerCollectorSpy =
            spyk(accelerometerGyroscopeAndMagnetometerCollector)
        val gravityGyroscopeAndMagnetometerCollectorSpy = spyk(gravityGyroscopeAndMagnetometerCollector)

        estimator.setPrivateProperty(
            "accelerometerGyroscopeAndMagnetometerCollector",
            accelerometerGyroscopeAndMagnetometerCollectorSpy
        )
        estimator.setPrivateProperty(
            "gravityGyroscopeAndMagnetometerCollector",
            gravityGyroscopeAndMagnetometerCollectorSpy
        )

        // stop
        estimator.stop()

        // check
        assertFalse(estimator.running)

        verify(exactly = 1) { accelerometerGyroscopeAndMagnetometerCollectorSpy.stop() }
        verify(exactly = 1) { gravityGyroscopeAndMagnetometerCollectorSpy.stop() }
    }

    @Test
    fun gravityGyroscopeAndMagnetometerCollector_whenAccuracyChangedAndNoListener_makesNoAction() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context, useAccelerometer = false)

        assertNull(estimator.accuracyChangedListener)

        val gravityGyroscopeAndMagnetometerCollector: GravityGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("gravityGyroscopeAndMagnetometerCollector")
        requireNotNull(gravityGyroscopeAndMagnetometerCollector)

        val listener = gravityGyroscopeAndMagnetometerCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(
            gravityGyroscopeAndMagnetometerCollector,
            SensorType.GRAVITY,
            SensorAccuracy.MEDIUM
        )
    }

    @Test
    fun gravityGyroscopeAndMagnetometerCollector_whenAccuracyChangedAndListener_notifies() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(
            context,
            useAccelerometer = false,
            accuracyChangedListener = accuracyChangedListener
        )

        assertSame(accuracyChangedListener, estimator.accuracyChangedListener)

        val gravityGyroscopeAndMagnetometerCollector: GravityGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("gravityGyroscopeAndMagnetometerCollector")
        requireNotNull(gravityGyroscopeAndMagnetometerCollector)

        val listener = gravityGyroscopeAndMagnetometerCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(
            gravityGyroscopeAndMagnetometerCollector,
            SensorType.GRAVITY,
            SensorAccuracy.MEDIUM
        )

        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                estimator,
                SensorType.GRAVITY,
                SensorAccuracy.MEDIUM
            )
        }
    }

    @Test
    fun gravityGyroscopeAndMagnetometerCollector_whenSyncedMeasurementAndNotProcessed_makesNoAction() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context, useAccelerometer = false)

        assertNull(estimator.attitudeAvailableListener)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        every { gravityProcessorSpy.process(any()) }.returns(false)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val gravityGyroscopeAndMagnetometerCollector: GravityGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("gravityGyroscopeAndMagnetometerCollector")
        requireNotNull(gravityGyroscopeAndMagnetometerCollector)

        val listener = gravityGyroscopeAndMagnetometerCollector.measurementListener
        requireNotNull(listener)
        val syncedMeasurement = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement()
        listener.onMeasurement(
            gravityGyroscopeAndMagnetometerCollector,
            syncedMeasurement
        )

        // check
        verify(exactly = 1) { gravityProcessorSpy.process(syncedMeasurement) }
        verify(exactly = 0) { gravityProcessorSpy.fusedAttitude }
    }

    @Test
    fun gravityGyroscopeAndMagnetometerCollector_whenSyncedMeasurementProcessedAndCoordinateTransformationAndEulerAnglesEstimatedAndNoListener_makesEstimation() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(
            context,
            useAccelerometer = false,
            estimateCoordinateTransformation = true,
            estimateEulerAngles = true
        )

        assertNull(estimator.attitudeAvailableListener)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        every { gravityProcessorSpy.process(any()) }.returns(true)
        val attitude = getAttitude()
        every { gravityProcessorSpy.fusedAttitude }.returns(attitude)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val gravityGyroscopeAndMagnetometerCollector: GravityGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("gravityGyroscopeAndMagnetometerCollector")
        requireNotNull(gravityGyroscopeAndMagnetometerCollector)

        val listener = gravityGyroscopeAndMagnetometerCollector.measurementListener
        requireNotNull(listener)
        val syncedMeasurement = GravityGyroscopeAndMagnetometerSyncedSensorMeasurement()
        listener.onMeasurement(
            gravityGyroscopeAndMagnetometerCollector,
            syncedMeasurement
        )

        // check
        verify(exactly = 1) { gravityProcessorSpy.process(syncedMeasurement) }
        verify(exactly = 1) { gravityProcessorSpy.fusedAttitude }

        val fusedAttitude: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude)
        assertEquals(attitude, fusedAttitude)
        assertNotSame(attitude, fusedAttitude)

        val coordinateTransformation: CoordinateTransformation? =
            estimator.getPrivateProperty("coordinateTransformation")
        requireNotNull(coordinateTransformation)
        assertEquals(
            CoordinateTransformation(
                attitude,
                FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME
            ), coordinateTransformation
        )

        val eulerAngles: DoubleArray? =
            estimator.getPrivateProperty("eulerAngles")
        requireNotNull(eulerAngles)
        assertArrayEquals(attitude.toEulerAngles(), eulerAngles, 0.0)
    }

    @Test
    fun gravityGyroscopeAndMagnetometerSyncer_whenSyncedMeasurementProcessedAndCoordinateTransformationAndEulerAnglesEstimatedAndListener_makesEstimationAndNotifies() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(
            context,
            useAccelerometer = false,
            estimateCoordinateTransformation = true,
            estimateEulerAngles = true,
            attitudeAvailableListener = attitudeListener
        )

        assertSame(attitudeListener, estimator.attitudeAvailableListener)

        val gravityProcessor: DoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("gravityProcessor")
        requireNotNull(gravityProcessor)
        val gravityProcessorSpy = spyk(gravityProcessor)
        every { gravityProcessorSpy.process(any()) }.returns(true)
        val attitude = getAttitude()
        every { gravityProcessorSpy.fusedAttitude }.returns(attitude)
        estimator.setPrivateProperty("gravityProcessor", gravityProcessorSpy)

        val gravityGyroscopeAndMagnetometerCollector: GravityGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("gravityGyroscopeAndMagnetometerCollector")
        requireNotNull(gravityGyroscopeAndMagnetometerCollector)

        val listener = gravityGyroscopeAndMagnetometerCollector.measurementListener
        requireNotNull(listener)
        val timestamp = System.nanoTime()
        val syncedMeasurement =
            GravityGyroscopeAndMagnetometerSyncedSensorMeasurement(timestamp = timestamp)
        listener.onMeasurement(
            gravityGyroscopeAndMagnetometerCollector,
            syncedMeasurement
        )

        // check
        verify(exactly = 1) { gravityProcessorSpy.process(syncedMeasurement) }
        verify(exactly = 1) { gravityProcessorSpy.fusedAttitude }

        val fusedAttitude: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude)
        assertEquals(attitude, fusedAttitude)
        assertNotSame(attitude, fusedAttitude)

        val coordinateTransformation: CoordinateTransformation? =
            estimator.getPrivateProperty("coordinateTransformation")
        requireNotNull(coordinateTransformation)
        assertEquals(
            CoordinateTransformation(
                attitude,
                FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME
            ), coordinateTransformation
        )

        val eulerAngles: DoubleArray? =
            estimator.getPrivateProperty("eulerAngles")
        requireNotNull(eulerAngles)
        assertArrayEquals(attitude.toEulerAngles(), eulerAngles, 0.0)

        verify(exactly = 1) {
            attitudeListener.onAttitudeAvailable(
                estimator,
                fusedAttitude,
                timestamp,
                eulerAngles[0],
                eulerAngles[1],
                eulerAngles[2],
                coordinateTransformation
            )
        }
    }

    @Test
    fun gravityGyroscopeAndMagnetometerSyncer_whenSyncedMeasurementProcessedAndCoordinateTransformationAndEulerAnglesNotEstimatedAndListener_makesEstimationAndNotifies() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(
            context,
            estimateCoordinateTransformation = false,
            estimateEulerAngles = false,
            attitudeAvailableListener = attitudeListener
        )

        assertSame(attitudeListener, estimator.attitudeAvailableListener)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)
        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        every { accelerometerProcessorSpy.process(any()) }.returns(true)
        val attitude = getAttitude()
        every { accelerometerProcessorSpy.fusedAttitude }.returns(attitude)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)

        val accelerometerGyroscopeAndMagnetometerCollector: AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerGyroscopeAndMagnetometerCollector")
        requireNotNull(accelerometerGyroscopeAndMagnetometerCollector)

        val listener = accelerometerGyroscopeAndMagnetometerCollector.measurementListener
        requireNotNull(listener)
        val timestamp = System.nanoTime()
        val syncedMeasurement =
            AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement(timestamp = timestamp)
        listener.onMeasurement(
            accelerometerGyroscopeAndMagnetometerCollector,
            syncedMeasurement
        )

        // check
        verify(exactly = 1) { accelerometerProcessorSpy.process(syncedMeasurement) }
        verify(exactly = 1) { accelerometerProcessorSpy.fusedAttitude }

        val fusedAttitude: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude)
        assertEquals(attitude, fusedAttitude)
        assertNotSame(attitude, fusedAttitude)

        verify(exactly = 1) {
            attitudeListener.onAttitudeAvailable(
                estimator,
                fusedAttitude,
                timestamp,
                null,
                null,
                null,
                null
            )
        }
    }

    @Test
    fun accelerometerGyroscopeAndMagnetometerSyncer_whenAccuracyChangedAndNoListener_makesNoAction() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context, useAccelerometer = true)

        assertNull(estimator.accuracyChangedListener)

        val accelerometerGyroscopeAndMagnetometerCollector: AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerGyroscopeAndMagnetometerCollector")
        requireNotNull(accelerometerGyroscopeAndMagnetometerCollector)

        val listener = accelerometerGyroscopeAndMagnetometerCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(
            accelerometerGyroscopeAndMagnetometerCollector,
            SensorType.GRAVITY,
            SensorAccuracy.MEDIUM
        )
    }

    @Test
    fun accelerometerGyroscopeAndMagnetometerSyncer_whenAccuracyChangedAndListener_notifies() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(
            context,
            useAccelerometer = true,
            accuracyChangedListener = accuracyChangedListener
        )

        assertSame(accuracyChangedListener, estimator.accuracyChangedListener)

        val accelerometerGyroscopeAndMagnetometerCollector: AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerGyroscopeAndMagnetometerCollector")
        requireNotNull(accelerometerGyroscopeAndMagnetometerCollector)

        val listener = accelerometerGyroscopeAndMagnetometerCollector.accuracyChangedListener
        requireNotNull(listener)
        listener.onAccuracyChanged(
            accelerometerGyroscopeAndMagnetometerCollector,
            SensorType.GRAVITY,
            SensorAccuracy.MEDIUM
        )

        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                estimator,
                SensorType.GRAVITY,
                SensorAccuracy.MEDIUM
            )
        }
    }

    @Test
    fun accelerometerGyroscopeAndMagnetometerSyncer_whenSyncedMeasurementAndNotProcessed_makesNoAction() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(context, useAccelerometer = true)

        assertNull(estimator.attitudeAvailableListener)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)
        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        every { accelerometerProcessorSpy.process(any()) }.returns(false)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)

        val accelerometerGyroscopeAndMagnetometerCollector: AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerGyroscopeAndMagnetometerCollector")
        requireNotNull(accelerometerGyroscopeAndMagnetometerCollector)

        val listener = accelerometerGyroscopeAndMagnetometerCollector.measurementListener
        requireNotNull(listener)
        val syncedMeasurement = AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement()
        listener.onMeasurement(
            accelerometerGyroscopeAndMagnetometerCollector,
            syncedMeasurement
        )

        // check
        verify(exactly = 1) { accelerometerProcessorSpy.process(syncedMeasurement) }
        verify(exactly = 0) { accelerometerProcessorSpy.fusedAttitude }
    }

    @Test
    fun accelerometerGyroscopeAndMagnetometerSyncer_whenSyncedMeasurementProcessedAndCoordinateTransformationAndEulerAnglesEstimatedAndNoListener_makesEstimation() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(
            context,
            useAccelerometer = true,
            estimateCoordinateTransformation = true,
            estimateEulerAngles = true
        )

        assertNull(estimator.attitudeAvailableListener)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)
        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        every { accelerometerProcessorSpy.process(any()) }.returns(true)
        val attitude = getAttitude()
        every { accelerometerProcessorSpy.fusedAttitude }.returns(attitude)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)

        val accelerometerGyroscopeAndMagnetometerCollector: AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerGyroscopeAndMagnetometerCollector")
        requireNotNull(accelerometerGyroscopeAndMagnetometerCollector)

        val listener = accelerometerGyroscopeAndMagnetometerCollector.measurementListener
        requireNotNull(listener)
        val syncedMeasurement = AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement()
        listener.onMeasurement(
            accelerometerGyroscopeAndMagnetometerCollector,
            syncedMeasurement
        )

        // check
        verify(exactly = 1) { accelerometerProcessorSpy.process(syncedMeasurement) }
        verify(exactly = 1) { accelerometerProcessorSpy.fusedAttitude }

        val fusedAttitude: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude)
        assertEquals(attitude, fusedAttitude)
        assertNotSame(attitude, fusedAttitude)

        val coordinateTransformation: CoordinateTransformation? =
            estimator.getPrivateProperty("coordinateTransformation")
        requireNotNull(coordinateTransformation)
        assertEquals(
            CoordinateTransformation(
                attitude,
                FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME
            ), coordinateTransformation
        )

        val eulerAngles: DoubleArray? =
            estimator.getPrivateProperty("eulerAngles")
        requireNotNull(eulerAngles)
        assertArrayEquals(attitude.toEulerAngles(), eulerAngles, 0.0)
    }

    @Test
    fun accelerometerGyroscopeAndMagnetometerSyncer_whenSyncedMeasurementProcessedAndCoordinateTransformationAndEulerAnglesEstimatedAndListener_makesEstimationAndNotifies() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(
            context,
            useAccelerometer = true,
            estimateCoordinateTransformation = true,
            estimateEulerAngles = true,
            attitudeAvailableListener = attitudeListener
        )

        assertSame(attitudeListener, estimator.attitudeAvailableListener)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)
        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        every { accelerometerProcessorSpy.process(any()) }.returns(true)
        val attitude = getAttitude()
        every { accelerometerProcessorSpy.fusedAttitude }.returns(attitude)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)

        val accelerometerGyroscopeAndMagnetometerCollector: AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerGyroscopeAndMagnetometerCollector")
        requireNotNull(accelerometerGyroscopeAndMagnetometerCollector)

        val listener = accelerometerGyroscopeAndMagnetometerCollector.measurementListener
        requireNotNull(listener)
        val timestamp = System.nanoTime()
        val syncedMeasurement =
            AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement(timestamp = timestamp)
        listener.onMeasurement(
            accelerometerGyroscopeAndMagnetometerCollector,
            syncedMeasurement
        )

        // check
        verify(exactly = 1) { accelerometerProcessorSpy.process(syncedMeasurement) }
        verify(exactly = 1) { accelerometerProcessorSpy.fusedAttitude }

        val fusedAttitude: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude)
        assertEquals(attitude, fusedAttitude)
        assertNotSame(attitude, fusedAttitude)

        val coordinateTransformation: CoordinateTransformation? =
            estimator.getPrivateProperty("coordinateTransformation")
        requireNotNull(coordinateTransformation)
        assertEquals(
            CoordinateTransformation(
                attitude,
                FrameType.BODY_FRAME,
                FrameType.LOCAL_NAVIGATION_FRAME
            ), coordinateTransformation
        )

        val eulerAngles: DoubleArray? =
            estimator.getPrivateProperty("eulerAngles")
        requireNotNull(eulerAngles)
        assertArrayEquals(attitude.toEulerAngles(), eulerAngles, 0.0)

        verify(exactly = 1) {
            attitudeListener.onAttitudeAvailable(
                estimator,
                fusedAttitude,
                timestamp,
                eulerAngles[0],
                eulerAngles[1],
                eulerAngles[2],
                coordinateTransformation
            )
        }
    }

    @Test
    fun accelerometerGyroscopeAndMagnetometerSyncer_whenSyncedMeasurementProcessedAndCoordinateTransformationAndEulerAnglesNotEstimatedAndListener_makesEstimationAndNotifies() {
        val estimator = DoubleFusedGeomagneticAttitudeEstimator(
            context,
            useAccelerometer = true,
            estimateCoordinateTransformation = false,
            estimateEulerAngles = false,
            attitudeAvailableListener = attitudeListener
        )

        assertSame(attitudeListener, estimator.attitudeAvailableListener)

        val accelerometerProcessor: AccelerometerDoubleFusedGeomagneticAttitudeProcessor? =
            estimator.getPrivateProperty("accelerometerProcessor")
        requireNotNull(accelerometerProcessor)
        val accelerometerProcessorSpy = spyk(accelerometerProcessor)
        every { accelerometerProcessorSpy.process(any()) }.returns(true)
        val attitude = getAttitude()
        every { accelerometerProcessorSpy.fusedAttitude }.returns(attitude)
        estimator.setPrivateProperty("accelerometerProcessor", accelerometerProcessorSpy)

        val accelerometerGyroscopeAndMagnetometerCollector: AccelerometerGyroscopeAndMagnetometerSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerGyroscopeAndMagnetometerCollector")
        requireNotNull(accelerometerGyroscopeAndMagnetometerCollector)

        val listener = accelerometerGyroscopeAndMagnetometerCollector.measurementListener
        requireNotNull(listener)
        val timestamp = System.nanoTime()
        val syncedMeasurement =
            AccelerometerGyroscopeAndMagnetometerSyncedSensorMeasurement(timestamp = timestamp)
        listener.onMeasurement(
            accelerometerGyroscopeAndMagnetometerCollector,
            syncedMeasurement
        )

        // check
        verify(exactly = 1) { accelerometerProcessorSpy.process(syncedMeasurement) }
        verify(exactly = 1) { accelerometerProcessorSpy.fusedAttitude }

        val fusedAttitude: Quaternion? = estimator.getPrivateProperty("fusedAttitude")
        requireNotNull(fusedAttitude)
        assertEquals(attitude, fusedAttitude)
        assertNotSame(attitude, fusedAttitude)

        verify(exactly = 1) {
            attitudeListener.onAttitudeAvailable(
                estimator,
                fusedAttitude,
                timestamp,
                null,
                null,
                null,
                null
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