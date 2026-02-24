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

package com.irurueta.android.navigation.inertial.estimators.pose

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorManager
import android.location.Location
import android.os.SystemClock
import com.irurueta.android.navigation.inertial.collectors.AccelerometerAndGyroscopeSyncedSensorCollector
import com.irurueta.android.navigation.inertial.collectors.AccelerometerGravityAndGyroscopeSyncedSensorCollector
import com.irurueta.android.navigation.inertial.collectors.AttitudeAndAccelerometerSyncedSensorCollector
import com.irurueta.android.navigation.inertial.collectors.SensorDelay
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerAndGyroscopeSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerGravityAndGyroscopeSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AccelerometerSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.AttitudeAndAccelerometerSyncedSensorMeasurement
import com.irurueta.android.navigation.inertial.collectors.measurements.AttitudeSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.GyroscopeSensorType
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorAccuracy
import com.irurueta.android.navigation.inertial.collectors.measurements.SensorType
import com.irurueta.android.navigation.inertial.processors.attitude.BaseFusedGeomagneticAttitudeProcessor
import com.irurueta.android.navigation.inertial.processors.filters.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.processors.filters.MedianAveragingFilter
import com.irurueta.android.navigation.inertial.processors.pose.AccelerometerFusedRelativePoseProcessor
import com.irurueta.android.navigation.inertial.processors.pose.AttitudeRelativePoseProcessor
import com.irurueta.android.navigation.inertial.processors.pose.FusedRelativePoseProcessor
import com.irurueta.android.testutils.getPrivateProperty
import com.irurueta.android.testutils.setPrivateProperty
import com.irurueta.geometry.EuclideanTransformation3D
import com.irurueta.navigation.inertial.calibration.AccelerationTriad
import com.irurueta.navigation.inertial.calibration.SpeedTriad
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

class RelativePoseEstimatorTest {

    @get:Rule
    val mockkRule = MockKRule(this)

    @MockK(relaxUnitFun = true)
    private lateinit var poseAvailableListener: RelativePoseEstimator.OnPoseAvailableListener

    @MockK(relaxUnitFun = true)
    private lateinit var accuracyChangedListener: RelativePoseEstimator.OnAccuracyChangedListener

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

    @MockK
    private lateinit var attitudeSensor: Sensor

    @Test
    fun constructor_whenRequiredProperties_setsDefaultValues() {
        val estimator = RelativePoseEstimator(context)

        // check
        assertSame(context, estimator.context)
        assertEquals(SpeedTriad(), estimator.initialSpeed)
        assertEquals(SensorDelay.FASTEST, estimator.sensorDelay)
        assertTrue(estimator.useAttitudeSensor)
        assertFalse(estimator.useAccelerometerForAttitudeEstimation)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED,
            estimator.accelerometerSensorType
        )
        assertNotNull(estimator.accelerometerAveragingFilter)
        assertTrue(estimator.accelerometerAveragingFilter is LowPassAveragingFilter)
        assertEquals(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED, estimator.gyroscopeSensorType)
        assertTrue(estimator.useAccurateRelativeGyroscopeAttitudeProcessor)
        assertNull(estimator.poseAvailableListener)
        assertNull(estimator.accuracyChangedListener)
        assertTrue(estimator.useIndirectAttitudeInterpolation)
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_INTERPOLATION_VALUE,
            estimator.attitudeInterpolationValue,
            0.0
        )
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            estimator.attitudeIndirectInterpolationWeight,
            0.0
        )
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_THRESHOLD,
            estimator.attitudeOutlierThreshold,
            0.0
        )
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            estimator.attitudeOutlierPanicThreshold,
            0.0
        )
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_PANIC_COUNTER_THRESHOLD,
            estimator.attitudePanicCounterThreshold,
        )
        assertNull(estimator.timeIntervalSeconds)
        assertFalse(estimator.running)
        assertNull(estimator.location)
        assertTrue(estimator.adjustGravityNorm)
    }

    @Test
    fun constructor_whenAllProperties_setsExpectedValues() {
        val initialSpeed = SpeedTriad()
        val accelerometerAveragingFilter = MedianAveragingFilter<AccelerationUnit, Acceleration, AccelerationTriad>()
        val estimator = RelativePoseEstimator(
            context,
            initialSpeed,
            SensorDelay.NORMAL,
            useAttitudeSensor = false,
            useAccelerometerForAttitudeEstimation = true,
            AccelerometerSensorType.ACCELEROMETER,
            accelerometerAveragingFilter,
            GyroscopeSensorType.GYROSCOPE,
            useAccurateRelativeGyroscopeAttitudeProcessor = false,
            poseAvailableListener,
            accuracyChangedListener
        )

        // check
        assertSame(context, estimator.context)
        assertSame(initialSpeed, estimator.initialSpeed)
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertFalse(estimator.useAttitudeSensor)
        assertTrue(estimator.useAccelerometerForAttitudeEstimation)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            estimator.accelerometerSensorType
        )
        assertSame(accelerometerAveragingFilter, estimator.accelerometerAveragingFilter)
        assertEquals(GyroscopeSensorType.GYROSCOPE, estimator.gyroscopeSensorType)
        assertFalse(estimator.useAccurateRelativeGyroscopeAttitudeProcessor)
        assertSame(poseAvailableListener, estimator.poseAvailableListener)
        assertSame(accuracyChangedListener, estimator.accuracyChangedListener)
        assertTrue(estimator.useIndirectAttitudeInterpolation)
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_INTERPOLATION_VALUE,
            estimator.attitudeInterpolationValue,
            0.0
        )
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            estimator.attitudeIndirectInterpolationWeight,
            0.0
        )
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_THRESHOLD,
            estimator.attitudeOutlierThreshold,
            0.0
        )
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            estimator.attitudeOutlierPanicThreshold,
            0.0
        )
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_PANIC_COUNTER_THRESHOLD,
            estimator.attitudePanicCounterThreshold,
        )
        val timeIntervalSeconds = estimator.timeIntervalSeconds
        requireNotNull(timeIntervalSeconds)
        assertEquals(0.0, timeIntervalSeconds, 0.0)
        assertFalse(estimator.running)
        assertNull(estimator.location)
        assertTrue(estimator.adjustGravityNorm)
    }

    @Test
    fun initialSpeed_whenUseAttitudeSensor_returnsExpectedValue() {
        val initialSpeed = SpeedTriad()
        val estimator = RelativePoseEstimator(context, initialSpeed, useAttitudeSensor = true)

        val fusedProcessor: FusedRelativePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedRelativePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val attitudeProcessor: AttitudeRelativePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        // check
        assertSame(initialSpeed, estimator.initialSpeed)

        verify(exactly = 1) { attitudeProcessorSpy.initialSpeed }
        verify { fusedProcessorSpy wasNot Called }
        verify { accelerometerFusedProcessorSpy wasNot Called }

        assertSame(initialSpeed, attitudeProcessor.initialSpeed)
        assertSame(initialSpeed, fusedProcessor.initialSpeed)
        assertSame(initialSpeed, accelerometerFusedProcessor.initialSpeed)
    }

    @Test
    fun initialSpeed_whenNotUseAttitudeSensorAndUseAccelerometerForAttitudeEstimation_returnsExpectedValue() {
        val initialSpeed = SpeedTriad()
        val estimator = RelativePoseEstimator(
            context,
            initialSpeed,
            useAttitudeSensor = false,
            useAccelerometerForAttitudeEstimation = true
        )

        val fusedProcessor: FusedRelativePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedRelativePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val attitudeProcessor: AttitudeRelativePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        // check
        assertSame(initialSpeed, estimator.initialSpeed)

        verify(exactly = 1) { accelerometerFusedProcessorSpy.initialSpeed }
        verify { fusedProcessorSpy wasNot Called }
        verify { attitudeProcessorSpy wasNot Called }

        assertSame(initialSpeed, attitudeProcessor.initialSpeed)
        assertSame(initialSpeed, fusedProcessor.initialSpeed)
        assertSame(initialSpeed, accelerometerFusedProcessor.initialSpeed)
    }

    @Test
    fun initialSpeed_whenNotUseAttitudeSensorAndNotUseAccelerometerForAttitudeEstimation_returnsExpectedValue() {
        val initialSpeed = SpeedTriad()
        val estimator = RelativePoseEstimator(
            context,
            initialSpeed,
            useAttitudeSensor = false,
            useAccelerometerForAttitudeEstimation = false
        )

        val fusedProcessor: FusedRelativePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedRelativePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val attitudeProcessor: AttitudeRelativePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        // check
        assertSame(initialSpeed, estimator.initialSpeed)

        verify(exactly = 1) { fusedProcessorSpy.initialSpeed }
        verify { accelerometerFusedProcessorSpy wasNot Called }
        verify { attitudeProcessorSpy wasNot Called }

        assertSame(initialSpeed, attitudeProcessor.initialSpeed)
        assertSame(initialSpeed, fusedProcessor.initialSpeed)
        assertSame(initialSpeed, accelerometerFusedProcessor.initialSpeed)
    }

    @Test
    fun useAccurateRelativeGyroscopeAttitudeProcessor_returnsExpectedValue() {
        val estimator = RelativePoseEstimator(context)

        // check default value
        assertTrue(estimator.useAccurateRelativeGyroscopeAttitudeProcessor)

        val fusedProcessor: FusedRelativePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        assertTrue(fusedProcessor.useAccurateRelativeGyroscopeAttitudeProcessor)

        val accelerometerFusedProcessor: AccelerometerFusedRelativePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        assertTrue(accelerometerFusedProcessor.useAccurateRelativeGyroscopeAttitudeProcessor)
    }

    @Test
    fun useIndirectAttitudeInterpolation_setsExpectedValue() {
        val estimator = RelativePoseEstimator(context)

        // check default value
        assertTrue(estimator.useIndirectAttitudeInterpolation)

        val fusedProcessor: FusedRelativePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedRelativePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        assertTrue(fusedProcessor.useIndirectAttitudeInterpolation)
        assertTrue(accelerometerFusedProcessor.useIndirectAttitudeInterpolation)

        // set new value
        estimator.useIndirectAttitudeInterpolation = false

        // check
        assertFalse(estimator.useIndirectAttitudeInterpolation)
        verify(exactly = 1) { fusedProcessorSpy.useIndirectAttitudeInterpolation = false }
        verify(exactly = 1) {
            accelerometerFusedProcessorSpy.useIndirectAttitudeInterpolation = false
        }
    }

    @Test
    fun attitudeInterpolationValue_setsExpectedValue() {
        val estimator = RelativePoseEstimator(context)

        // check default value
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_INTERPOLATION_VALUE,
            estimator.attitudeInterpolationValue,
            0.0
        )

        val fusedProcessor: FusedRelativePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedRelativePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_INTERPOLATION_VALUE,
            fusedProcessor.attitudeInterpolationValue,
            0.0
        )
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_INTERPOLATION_VALUE,
            accelerometerFusedProcessor.attitudeInterpolationValue,
            0.0
        )

        // set new value
        val randomizer = UniformRandomizer()
        val attitudeInterpolationValue = randomizer.nextDouble()
        estimator.attitudeInterpolationValue = attitudeInterpolationValue

        // check
        assertEquals(attitudeInterpolationValue, estimator.attitudeInterpolationValue, 0.0)
        verify(exactly = 1) {
            fusedProcessorSpy.attitudeInterpolationValue = attitudeInterpolationValue
        }
        verify(exactly = 1) {
            accelerometerFusedProcessorSpy.attitudeInterpolationValue = attitudeInterpolationValue
        }
    }

    @Test
    fun attitudeIndirectInterpolationWeight_setsExpectedValue() {
        val estimator = RelativePoseEstimator(context)

        // check default value
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            estimator.attitudeIndirectInterpolationWeight,
            0.0
        )

        val fusedProcessor: FusedRelativePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedRelativePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            fusedProcessor.attitudeIndirectInterpolationWeight,
            0.0
        )
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_INDIRECT_INTERPOLATION_WEIGHT,
            accelerometerFusedProcessor.attitudeIndirectInterpolationWeight,
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
        verify(exactly = 1) {
            fusedProcessorSpy.attitudeIndirectInterpolationWeight =
                attitudeIndirectInterpolationWeight
        }
        verify(exactly = 1) {
            accelerometerFusedProcessorSpy.attitudeIndirectInterpolationWeight =
                attitudeIndirectInterpolationWeight
        }
    }

    @Test
    fun attitudeOutlierThreshold_setsExpectedValue() {
        val estimator = RelativePoseEstimator(context)

        // check default value
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_THRESHOLD,
            estimator.attitudeOutlierThreshold,
            0.0
        )

        val fusedProcessor: FusedRelativePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedRelativePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_THRESHOLD,
            fusedProcessor.attitudeOutlierThreshold,
            0.0
        )
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_THRESHOLD,
            accelerometerFusedProcessor.attitudeOutlierThreshold,
            0.0
        )

        // set new value
        val randomizer = UniformRandomizer()
        val attitudeOutlierThreshold = randomizer.nextDouble()
        estimator.attitudeOutlierThreshold = attitudeOutlierThreshold

        // check
        assertEquals(
            attitudeOutlierThreshold,
            estimator.attitudeOutlierThreshold,
            0.0
        )
        verify(exactly = 1) {
            fusedProcessorSpy.attitudeOutlierThreshold = attitudeOutlierThreshold
        }
        verify(exactly = 1) {
            accelerometerFusedProcessorSpy.attitudeOutlierThreshold = attitudeOutlierThreshold
        }
    }

    @Test
    fun attitudeOutlierPanicThreshold_setsExpectedValue() {
        val estimator = RelativePoseEstimator(context)

        // check default value
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            estimator.attitudeOutlierPanicThreshold,
            0.0
        )

        val fusedProcessor: FusedRelativePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedRelativePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            fusedProcessor.attitudeOutlierPanicThreshold,
            0.0
        )
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_OUTLIER_PANIC_THRESHOLD,
            accelerometerFusedProcessor.attitudeOutlierPanicThreshold,
            0.0
        )

        // set new value
        val randomizer = UniformRandomizer()
        val attitudeOutlierPanicThreshold = randomizer.nextDouble()
        estimator.attitudeOutlierPanicThreshold = attitudeOutlierPanicThreshold

        // check
        assertEquals(
            attitudeOutlierPanicThreshold,
            estimator.attitudeOutlierPanicThreshold,
            0.0
        )
        verify(exactly = 1) {
            fusedProcessorSpy.attitudeOutlierPanicThreshold = attitudeOutlierPanicThreshold
        }
        verify(exactly = 1) {
            accelerometerFusedProcessorSpy.attitudeOutlierPanicThreshold =
                attitudeOutlierPanicThreshold
        }
    }

    @Test
    fun attitudePanicCounterThreshold_setsExpectedValue() {
        val estimator = RelativePoseEstimator(context)

        // check default value
        assertEquals(
            com.irurueta.android.navigation.inertial.old.processors.attitude.BaseFusedGeomagneticAttitudeProcessor.DEFAULT_PANIC_COUNTER_THRESHOLD,
            estimator.attitudePanicCounterThreshold,
        )

        val fusedProcessor: FusedRelativePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedRelativePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_PANIC_COUNTER_THRESHOLD,
            fusedProcessor.attitudePanicCounterThreshold,
        )
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_PANIC_COUNTER_THRESHOLD,
            accelerometerFusedProcessor.attitudePanicCounterThreshold,
        )

        // set new value
        val randomizer = UniformRandomizer()
        val attitudePanicCounterThreshold = randomizer.nextInt(1, 100)
        estimator.attitudePanicCounterThreshold = attitudePanicCounterThreshold

        // check
        assertEquals(attitudePanicCounterThreshold, estimator.attitudePanicCounterThreshold)
        verify(exactly = 1) {
            fusedProcessorSpy.attitudePanicCounterThreshold = attitudePanicCounterThreshold
        }
        verify(exactly = 1) {
            accelerometerFusedProcessorSpy.attitudePanicCounterThreshold =
                attitudePanicCounterThreshold
        }
    }

    @Test
    fun timeIntervalSeconds_whenUseAttitudeSensor_returnsNull() {
        val estimator = RelativePoseEstimator(context, useAttitudeSensor = true)

        val fusedProcessor: FusedRelativePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedRelativePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val attitudeProcessor: AttitudeRelativePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        assertNull(estimator.timeIntervalSeconds)

        verify { fusedProcessorSpy wasNot Called }
        verify { accelerometerFusedProcessorSpy wasNot Called }
    }

    @Test
    fun timeIntervalSeconds_whenNotUseAttitudeSensorAndUseAccelerometerForAttitudeEstimation_returnsExpectedValue() {
        val estimator = RelativePoseEstimator(
            context,
            useAttitudeSensor = false,
            useAccelerometerForAttitudeEstimation = true
        )

        val fusedProcessor: FusedRelativePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedRelativePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        every { accelerometerFusedProcessorSpy.timeIntervalSeconds }.returns(TIME_INTERVAL)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val attitudeProcessor: AttitudeRelativePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        val timeIntervalSeconds = estimator.timeIntervalSeconds
        requireNotNull(timeIntervalSeconds)
        assertEquals(TIME_INTERVAL, timeIntervalSeconds, 0.0)

        verify { fusedProcessorSpy wasNot Called }
        verify(exactly = 1) { accelerometerFusedProcessorSpy.timeIntervalSeconds }
    }

    @Test
    fun timeIntervalSeconds_whenNotUseAttitudeSensorAndNotUseAccelerometerForAttitudeEstimation_returnsExpectedValue() {
        val estimator = RelativePoseEstimator(
            context,
            useAttitudeSensor = false,
            useAccelerometerForAttitudeEstimation = false
        )

        val fusedProcessor: FusedRelativePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        every { fusedProcessorSpy.timeIntervalSeconds }.returns(TIME_INTERVAL)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedRelativePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val attitudeProcessor: AttitudeRelativePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        val timeIntervalSeconds = estimator.timeIntervalSeconds
        requireNotNull(timeIntervalSeconds)
        assertEquals(TIME_INTERVAL, timeIntervalSeconds, 0.0)

        verify { accelerometerFusedProcessorSpy wasNot Called }
        verify(exactly = 1) { fusedProcessorSpy.timeIntervalSeconds }
    }

    @Test
    fun location_setsExpectedValue() {
        val estimator = RelativePoseEstimator(context)

        val fusedProcessor: FusedRelativePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val accelerometerFusedProcessor: AccelerometerFusedRelativePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val attitudeProcessor: AttitudeRelativePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)

        // check default value
        assertNull(estimator.location)
        assertNull(fusedProcessor.location)
        assertNull(accelerometerFusedProcessor.location)
        assertNull(attitudeProcessor.location)

        // set new value
        val location = getLocation()
        estimator.location = location

        // check
        assertSame(location, estimator.location)
        assertSame(location, fusedProcessor.location)
        assertSame(location, accelerometerFusedProcessor.location)
        assertSame(location, attitudeProcessor.location)
    }

    @Test
    fun adjustGravityNorm_setsExpectedValue() {
        val estimator = RelativePoseEstimator(context)

        val fusedProcessor: FusedRelativePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val accelerometerFusedProcessor: AccelerometerFusedRelativePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val attitudeProcessor: AttitudeRelativePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)

        // check default value
        assertTrue(estimator.adjustGravityNorm)
        assertTrue(fusedProcessor.adjustGravityNorm)
        assertTrue(accelerometerFusedProcessor.adjustGravityNorm)
        assertTrue(attitudeProcessor.adjustGravityNorm)

        // set new value
        estimator.adjustGravityNorm = false

        // check
        assertFalse(estimator.adjustGravityNorm)
        assertFalse(fusedProcessor.adjustGravityNorm)
        assertFalse(accelerometerFusedProcessor.adjustGravityNorm)
        assertFalse(attitudeProcessor.adjustGravityNorm)
    }

    @Test
    fun start_whenRunning_throwsIllegalStateException() {
        mockkStatic(SystemClock::class) {
            val startTimestamp = System.nanoTime()
            every { SystemClock.elapsedRealtimeNanos() }.returns(startTimestamp)

            val estimator = RelativePoseEstimator(context)

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
    fun start_whenUseAttitudeSensor_resetsProcessorAndStartsSyncer() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }
            .returns(accelerometerSensor)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }
            .returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }
            .returns(gyroscopeSensor)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }
            .returns(attitudeSensor)
        every { sensorManager.registerListener(any(), any<Sensor>(), any()) }
            .returns(true)

        val estimator = RelativePoseEstimator(
            context,
            useAttitudeSensor = true
        )

        val startTimestamp = System.nanoTime()

        val fusedProcessor: FusedRelativePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedRelativePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val attitudeProcessor: AttitudeRelativePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        val fusedCollector: AccelerometerGravityAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("fusedCollector")
        requireNotNull(fusedCollector)
        val fusedCollectorSpy = spyk(fusedCollector)
        estimator.setPrivateProperty("fusedCollector", fusedCollectorSpy)

        val accelerometerFusedCollector: AccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerFusedCollector")
        requireNotNull(accelerometerFusedCollector)
        val accelerometerFusedCollectorSpy = spyk(accelerometerFusedCollector)
        estimator.setPrivateProperty("accelerometerFusedCollector", accelerometerFusedCollectorSpy)

        val attitudeCollector: AttitudeAndAccelerometerSyncedSensorCollector? =
            estimator.getPrivateProperty("attitudeCollector")
        requireNotNull(attitudeCollector)
        val attitudeCollectorSpy = spyk(attitudeCollector)
        every { attitudeCollectorSpy.start(startTimestamp) }.returns(true)
        estimator.setPrivateProperty("attitudeCollector", attitudeCollectorSpy)

        assertTrue(estimator.start(startTimestamp))
        assertTrue(estimator.running)

        verify { attitudeProcessorSpy.reset() }
        verify(exactly = 1) { attitudeCollectorSpy.start(startTimestamp) }
        verify { accelerometerFusedProcessorSpy wasNot Called }
        verify { accelerometerFusedCollectorSpy wasNot Called }
        verify { fusedProcessorSpy wasNot Called }
        verify { fusedCollectorSpy wasNot Called }
    }

    @Test
    fun start_whenNotUseAttitudeSensorAndUseAccelerometerForAttitudeEstimation_resetsProcessorAndStartsSyncer() {
        val estimator = RelativePoseEstimator(
            context,
            useAttitudeSensor = false,
            useAccelerometerForAttitudeEstimation = true
        )

        val startTimestamp = System.nanoTime()

        val fusedProcessor: FusedRelativePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedRelativePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val attitudeProcessor: AttitudeRelativePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        val fusedCollector: AccelerometerGravityAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("fusedCollector")
        requireNotNull(fusedCollector)
        val fusedCollectorSpy = spyk(fusedCollector)
        estimator.setPrivateProperty("fusedCollector", fusedCollectorSpy)

        val accelerometerFusedCollector: AccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerFusedCollector")
        requireNotNull(accelerometerFusedCollector)
        val accelerometerFusedCollectorSpy = spyk(accelerometerFusedCollector)
        every { accelerometerFusedCollectorSpy.start(startTimestamp) }.returns(true)
        estimator.setPrivateProperty("accelerometerFusedCollector", accelerometerFusedCollectorSpy)

        val attitudeCollector: AttitudeAndAccelerometerSyncedSensorCollector? =
            estimator.getPrivateProperty("attitudeCollector")
        requireNotNull(attitudeCollector)
        val attitudeCollectorSpy = spyk(attitudeCollector)
        estimator.setPrivateProperty("attitudeCollector", attitudeCollectorSpy)

        assertTrue(estimator.start(startTimestamp))
        assertTrue(estimator.running)

        verify { accelerometerFusedProcessorSpy.reset() }
        verify(exactly = 1) { accelerometerFusedCollectorSpy.start(startTimestamp) }
        verify { attitudeProcessorSpy wasNot Called }
        verify { attitudeCollectorSpy wasNot Called }
        verify { fusedProcessorSpy wasNot Called }
        verify { fusedCollectorSpy wasNot Called }
    }

    @Test
    fun start_whenNotUseAttitudeSensorAndNotUseAccelerometerForAttitudeEstimation_resetsProcessorAndStartsSyncer() {
        val estimator = RelativePoseEstimator(
            context,
            useAttitudeSensor = false,
            useAccelerometerForAttitudeEstimation = false
        )

        val startTimestamp = System.nanoTime()

        val fusedProcessor: FusedRelativePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        val accelerometerFusedProcessor: AccelerometerFusedRelativePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        val attitudeProcessor: AttitudeRelativePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        val fusedCollector: AccelerometerGravityAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("fusedCollector")
        requireNotNull(fusedCollector)
        val fusedCollectorSpy = spyk(fusedCollector)
        every { fusedCollectorSpy.start(startTimestamp) }.returns(true)
        estimator.setPrivateProperty("fusedCollector", fusedCollectorSpy)

        val accelerometerFusedCollector: AccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerFusedCollector")
        requireNotNull(accelerometerFusedCollector)
        val accelerometerFusedCollectorSpy = spyk(accelerometerFusedCollector)
        estimator.setPrivateProperty("accelerometerFusedCollector", accelerometerFusedCollectorSpy)

        val attitudeCollector: AttitudeAndAccelerometerSyncedSensorCollector? =
            estimator.getPrivateProperty("attitudeCollector")
        requireNotNull(attitudeCollector)
        val attitudeCollectorSpy = spyk(attitudeCollector)
        estimator.setPrivateProperty("attitudeCollector", attitudeCollectorSpy)

        assertTrue(estimator.start(startTimestamp))
        assertTrue(estimator.running)

        verify { fusedProcessorSpy.reset() }
        verify(exactly = 1) { fusedCollectorSpy.start(startTimestamp) }
        verify { attitudeProcessorSpy wasNot Called }
        verify { attitudeCollectorSpy wasNot Called }
        verify { accelerometerFusedProcessorSpy wasNot Called }
        verify { accelerometerFusedCollectorSpy wasNot Called }
    }

    @Test
    fun stop_stopsAllSyncers() {
        every { context.getSystemService(Context.SENSOR_SERVICE) }.returns(sensorManager)
        every { sensorManager.getDefaultSensor(AccelerometerSensorType.ACCELEROMETER_UNCALIBRATED.value) }
            .returns(accelerometerSensor)
        every { sensorManager.getDefaultSensor(Sensor.TYPE_GRAVITY) }
            .returns(gravitySensor)
        every { sensorManager.getDefaultSensor(GyroscopeSensorType.GYROSCOPE_UNCALIBRATED.value) }
            .returns(gyroscopeSensor)
        every { sensorManager.getDefaultSensor(AttitudeSensorType.ABSOLUTE_ATTITUDE.value) }
            .returns(attitudeSensor)
        justRun { sensorManager.unregisterListener(any(), any<Sensor>()) }

        val estimator = RelativePoseEstimator(context)

        val startTimestamp = System.nanoTime()

        val fusedCollector: AccelerometerGravityAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("fusedCollector")
        requireNotNull(fusedCollector)
        val fusedCollectorSpy = spyk(fusedCollector)
        every { fusedCollectorSpy.start(startTimestamp) }.returns(true)
        estimator.setPrivateProperty("fusedCollector", fusedCollectorSpy)

        val accelerometerFusedCollector: AccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerFusedCollector")
        requireNotNull(accelerometerFusedCollector)
        val accelerometerFusedCollectorSpy = spyk(accelerometerFusedCollector)
        estimator.setPrivateProperty("accelerometerFusedCollector", accelerometerFusedCollectorSpy)

        val attitudeCollector: AttitudeAndAccelerometerSyncedSensorCollector? =
            estimator.getPrivateProperty("attitudeCollector")
        requireNotNull(attitudeCollector)
        val attitudeCollectorSpy = spyk(attitudeCollector)
        estimator.setPrivateProperty("attitudeCollector", attitudeCollectorSpy)

        // stop
        estimator.stop()

        verify(exactly = 1) { fusedCollectorSpy.stop() }
        verify(exactly = 1) { accelerometerFusedCollectorSpy.stop() }
        verify(exactly = 1) { attitudeCollectorSpy.stop() }
    }

    @Test
    fun fusedCollector_whenAccuracyChangedAndNoListener_makesNoAction() {
        val estimator = RelativePoseEstimator(context)

        val fusedCollector: AccelerometerGravityAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("fusedCollector")
        requireNotNull(fusedCollector)
        val listener = fusedCollector.accuracyChangedListener
        requireNotNull(listener)

        // notify
        listener.onAccuracyChanged(fusedCollector, SensorType.ABSOLUTE_ATTITUDE, SensorAccuracy.HIGH)
    }

    @Test
    fun fusedCollector_whenAccuracyChangedAndListener_notifies() {
        val estimator = RelativePoseEstimator(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val fusedCollector: AccelerometerGravityAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("fusedCollector")
        requireNotNull(fusedCollector)
        val listener = fusedCollector.accuracyChangedListener
        requireNotNull(listener)

        // notify
        listener.onAccuracyChanged(fusedCollector, SensorType.ABSOLUTE_ATTITUDE, SensorAccuracy.HIGH)

        // check
        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                estimator,
                SensorType.ABSOLUTE_ATTITUDE,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun fusedCollector_whenSyncedMeasurementAndNotProcessed_makesNoAction() {
        val estimator = RelativePoseEstimator(
            context,
            poseAvailableListener = poseAvailableListener
        )

        val fusedCollector: AccelerometerGravityAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("fusedCollector")
        requireNotNull(fusedCollector)
        val listener = fusedCollector.measurementListener
        requireNotNull(listener)

        val fusedProcessor: FusedRelativePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        every { fusedProcessorSpy.process(any()) }.returns(false)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        // notify measurement
        val measurement = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement()
        listener.onMeasurement(fusedCollector, measurement)

        // check
        verify(exactly = 1) { fusedProcessorSpy.process(measurement) }
        verify { poseAvailableListener wasNot Called }
    }

    @Test
    fun fusedCollector_whenSyncedMeasurementProcessedAndNoListener_makesExpectedCalls() {
        val estimator = RelativePoseEstimator(context)

        val fusedCollector: AccelerometerGravityAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("fusedCollector")
        requireNotNull(fusedCollector)
        val listener = fusedCollector.measurementListener
        requireNotNull(listener)

        val fusedProcessor: FusedRelativePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        every { fusedProcessorSpy.process(any()) }.returns(true)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        // notify measurement
        val measurement = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement()
        listener.onMeasurement(fusedCollector, measurement)

        // check
        verify(exactly = 1) { fusedProcessorSpy.process(measurement) }
        verify(exactly = 1) { fusedProcessorSpy.poseTransformation }
    }

    @Test
    fun fusedCollector_whenSyncedMeasurementProcessedAndListener_makesExpectedCalls() {
        val estimator = RelativePoseEstimator(
            context,
            poseAvailableListener = poseAvailableListener
        )

        val fusedCollector: AccelerometerGravityAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("fusedCollector")
        requireNotNull(fusedCollector)
        val listener = fusedCollector.measurementListener
        requireNotNull(listener)

        val fusedProcessor: FusedRelativePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        every { fusedProcessorSpy.process(any()) }.returns(true)
        val poseTransformation = EuclideanTransformation3D()
        every { fusedProcessorSpy.poseTransformation }.returns(poseTransformation)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        // notify measurement
        val timestamp = System.nanoTime()
        val measurement =
            AccelerometerGravityAndGyroscopeSyncedSensorMeasurement(timestamp = timestamp)
        listener.onMeasurement(fusedCollector, measurement)

        // check
        verify(exactly = 1) { fusedProcessorSpy.process(measurement) }
        verify(exactly = 1) { fusedProcessorSpy.poseTransformation }
        verify(exactly = 1) {
            poseAvailableListener.onPoseAvailable(
                estimator,
                timestamp,
                poseTransformation
            )
        }
    }

    @Test
    fun accelerometerFusedCollector_whenAccuracyChangedAndNoListener_makesNoAction() {
        val estimator = RelativePoseEstimator(context)

        val accelerometerFusedCollector: AccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerFusedCollector")
        requireNotNull(accelerometerFusedCollector)
        val listener = accelerometerFusedCollector.accuracyChangedListener
        requireNotNull(listener)

        // notify
        listener.onAccuracyChanged(
            accelerometerFusedCollector,
            SensorType.ABSOLUTE_ATTITUDE,
            SensorAccuracy.HIGH
        )
    }

    @Test
    fun accelerometerFusedCollector_whenAccuracyChangedAndListener_notifies() {
        val estimator = RelativePoseEstimator(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val accelerometerFusedCollector: AccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerFusedCollector")
        requireNotNull(accelerometerFusedCollector)
        val listener = accelerometerFusedCollector.accuracyChangedListener
        requireNotNull(listener)

        // notify
        listener.onAccuracyChanged(
            accelerometerFusedCollector,
            SensorType.ABSOLUTE_ATTITUDE,
            SensorAccuracy.HIGH
        )

        // check
        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                estimator,
                SensorType.ABSOLUTE_ATTITUDE,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun accelerometerFusedCollector_whenSyncedMeasurementAndNotProcessed_makesNoAction() {
        val estimator = RelativePoseEstimator(
            context,
            poseAvailableListener = poseAvailableListener
        )

        val accelerometerFusedCollector: AccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerFusedCollector")
        requireNotNull(accelerometerFusedCollector)
        val listener = accelerometerFusedCollector.measurementListener
        requireNotNull(listener)

        val accelerometerFusedProcessor: AccelerometerFusedRelativePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        every { accelerometerFusedProcessorSpy.process(any()) }.returns(false)
        estimator.setPrivateProperty(
            "accelerometerFusedProcessor",
            accelerometerFusedProcessorSpy
        )

        // notify measurement
        val measurement = AccelerometerAndGyroscopeSyncedSensorMeasurement()
        listener.onMeasurement(accelerometerFusedCollector, measurement)

        // check
        verify(exactly = 1) { accelerometerFusedProcessorSpy.process(measurement) }
        verify { poseAvailableListener wasNot Called }
    }

    @Test
    fun accelerometerFusedCollector_whenSyncedMeasurementProcessedAndNoListener_makesExpectedCalls() {
        val estimator = RelativePoseEstimator(context)

        val accelerometerFusedCollector: AccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerFusedCollector")
        requireNotNull(accelerometerFusedCollector)
        val listener = accelerometerFusedCollector.measurementListener
        requireNotNull(listener)

        val accelerometerFusedProcessor: AccelerometerFusedRelativePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        every { accelerometerFusedProcessorSpy.process(any()) }.returns(true)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        // notify measurement
        val measurement = AccelerometerAndGyroscopeSyncedSensorMeasurement()
        listener.onMeasurement(accelerometerFusedCollector, measurement)

        // check
        verify(exactly = 1) { accelerometerFusedProcessorSpy.process(measurement) }
        verify(exactly = 1) { accelerometerFusedProcessorSpy.poseTransformation }
    }

    @Test
    fun accelerometerFusedCollector_whenSyncedMeasurementProcessedAndListener_makesExpectedCalls() {
        val estimator = RelativePoseEstimator(
            context,
            poseAvailableListener = poseAvailableListener
        )

        val accelerometerFusedCollector: AccelerometerAndGyroscopeSyncedSensorCollector? =
            estimator.getPrivateProperty("accelerometerFusedCollector")
        requireNotNull(accelerometerFusedCollector)
        val listener = accelerometerFusedCollector.measurementListener
        requireNotNull(listener)

        val accelerometerFusedProcessor: AccelerometerFusedRelativePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        every { accelerometerFusedProcessorSpy.process(any()) }.returns(true)
        val poseTransformation = EuclideanTransformation3D()
        every { accelerometerFusedProcessorSpy.poseTransformation }.returns(poseTransformation)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        // notify measurement
        val timestamp = System.nanoTime()
        val measurement =
            AccelerometerAndGyroscopeSyncedSensorMeasurement(timestamp = timestamp)
        listener.onMeasurement(accelerometerFusedCollector, measurement)

        // check
        verify(exactly = 1) { accelerometerFusedProcessorSpy.process(measurement) }
        verify(exactly = 1) { accelerometerFusedProcessorSpy.poseTransformation }
        verify(exactly = 1) {
            poseAvailableListener.onPoseAvailable(
                estimator,
                timestamp,
                poseTransformation
            )
        }
    }

    @Test
    fun attitudeCollector_whenAccuracyChangedAndNoListener_makesNoAction() {
        val estimator = RelativePoseEstimator(context)

        val attitudeSyncer: AttitudeAndAccelerometerSyncedSensorCollector? =
            estimator.getPrivateProperty("attitudeCollector")
        requireNotNull(attitudeSyncer)
        val listener = attitudeSyncer.accuracyChangedListener
        requireNotNull(listener)

        // notify
        listener.onAccuracyChanged(
            attitudeSyncer,
            SensorType.ABSOLUTE_ATTITUDE,
            SensorAccuracy.HIGH
        )
    }

    @Test
    fun attitudeCollector_whenAccuracyChangedAndListener_notifies() {
        val estimator = RelativePoseEstimator(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val attitudeCollector: AttitudeAndAccelerometerSyncedSensorCollector? =
            estimator.getPrivateProperty("attitudeCollector")
        requireNotNull(attitudeCollector)
        val listener = attitudeCollector.accuracyChangedListener
        requireNotNull(listener)

        // notify
        listener.onAccuracyChanged(
            attitudeCollector,
            SensorType.ABSOLUTE_ATTITUDE,
            SensorAccuracy.HIGH
        )

        // check
        verify(exactly = 1) {
            accuracyChangedListener.onAccuracyChanged(
                estimator,
                SensorType.ABSOLUTE_ATTITUDE,
                SensorAccuracy.HIGH
            )
        }
    }

    @Test
    fun attitudeCollector_whenSyncedMeasurementAndNotProcessed_makesNoAction() {
        val estimator = RelativePoseEstimator(
            context,
            poseAvailableListener = poseAvailableListener
        )

        val attitudeCollector: AttitudeAndAccelerometerSyncedSensorCollector? =
            estimator.getPrivateProperty("attitudeCollector")
        requireNotNull(attitudeCollector)
        val listener = attitudeCollector.measurementListener
        requireNotNull(listener)

        val attitudeProcessor: AttitudeRelativePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        every { attitudeProcessorSpy.process(any()) }.returns(false)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        // notify measurement
        val measurement = AttitudeAndAccelerometerSyncedSensorMeasurement()
        listener.onMeasurement(attitudeCollector, measurement)

        // check
        verify(exactly = 1) { attitudeProcessorSpy.process(measurement) }
        verify { poseAvailableListener wasNot Called }
    }

    @Test
    fun attitudeCollector_whenSyncedMeasurementProcessedAndNoListener_makesExpectedCalls() {
        val estimator = RelativePoseEstimator(context)

        val attitudeCollector: AttitudeAndAccelerometerSyncedSensorCollector? =
            estimator.getPrivateProperty("attitudeCollector")
        requireNotNull(attitudeCollector)
        val listener = attitudeCollector.measurementListener
        requireNotNull(listener)

        val attitudeProcessor: AttitudeRelativePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        every { attitudeProcessorSpy.process(any()) }.returns(true)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        // notify measurement
        val measurement = AttitudeAndAccelerometerSyncedSensorMeasurement()
        listener.onMeasurement(attitudeCollector, measurement)

        // check
        verify(exactly = 1) { attitudeProcessorSpy.process(measurement) }
        verify(exactly = 1) { attitudeProcessorSpy.poseTransformation }
    }

    @Test
    fun attitudeCollector_whenSyncedMeasurementAttitudeProcessedNotEstimatePoseTransformationAndListener_makesExpectedCalls() {
        val estimator = RelativePoseEstimator(
            context,
            poseAvailableListener = poseAvailableListener
        )

        val attitudeCollector: AttitudeAndAccelerometerSyncedSensorCollector? =
            estimator.getPrivateProperty("attitudeCollector")
        requireNotNull(attitudeCollector)
        val listener = attitudeCollector.measurementListener
        requireNotNull(listener)

        val attitudeProcessor: AttitudeRelativePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        every { attitudeProcessorSpy.process(any()) }.returns(true)
        val poseTransformation = EuclideanTransformation3D()
        every { attitudeProcessorSpy.poseTransformation }.returns(poseTransformation)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        // notify measurement
        val timestamp = System.nanoTime()
        val measurement =
            AttitudeAndAccelerometerSyncedSensorMeasurement(timestamp = timestamp)
        listener.onMeasurement(attitudeCollector, measurement)

        // check
        verify(exactly = 1) { attitudeProcessorSpy.process(measurement) }
        verify(exactly = 1) { attitudeProcessorSpy.poseTransformation }
        verify(exactly = 1) {
            poseAvailableListener.onPoseAvailable(
                estimator,
                timestamp,
                poseTransformation
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

        const val TIME_INTERVAL = 0.02
    }
}