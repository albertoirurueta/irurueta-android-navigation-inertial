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
package com.irurueta.android.navigation.inertial.estimators.pose

import android.content.Context
import android.location.Location
import androidx.test.core.app.ApplicationProvider
import com.irurueta.android.navigation.inertial.collectors.*
import com.irurueta.android.navigation.inertial.estimators.filter.LowPassAveragingFilter
import com.irurueta.android.navigation.inertial.estimators.filter.MedianAveragingFilter
import com.irurueta.android.navigation.inertial.getPrivateProperty
import com.irurueta.android.navigation.inertial.processors.attitude.BaseFusedGeomagneticAttitudeProcessor
import com.irurueta.android.navigation.inertial.processors.pose.*
import com.irurueta.android.navigation.inertial.setPrivateProperty
import com.irurueta.geometry.EuclideanTransformation3D
import com.irurueta.statistics.UniformRandomizer
import io.mockk.*
import org.junit.After
import org.junit.Assert.*
import org.junit.Test
import org.junit.runner.RunWith
import org.robolectric.RobolectricTestRunner

@RunWith(RobolectricTestRunner::class)
class RelativePoseEstimator2Test {

    @After
    fun tearDown() {
        clearAllMocks()
        unmockkAll()
    }

    @Test
    fun constructor_whenRequiredProperties_setsDefaultValues() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator2(context)

        // check
        assertSame(context, estimator.context)
        assertEquals(SpeedTriad(), estimator.initialSpeed)
        assertEquals(SensorDelay.FASTEST, estimator.sensorDelay)
        assertTrue(estimator.useAttitudeSensor)
        assertFalse(estimator.useAccelerometerForAttitudeEstimation)
        assertTrue(estimator.startOffsetEnabled)
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
        assertNull(estimator.bufferFilledListener)
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialSpeed = SpeedTriad()
        val accelerometerAveragingFilter = MedianAveragingFilter()
        val poseAvailableListener = mockk<RelativePoseEstimator2.OnPoseAvailableListener>()
        val accuracyChangedListener = mockk<RelativePoseEstimator2.OnAccuracyChangedListener>()
        val bufferFilledListener = mockk<RelativePoseEstimator2.OnBufferFilledListener>()
        val estimator = RelativePoseEstimator2(
            context,
            initialSpeed,
            SensorDelay.NORMAL,
            useAttitudeSensor = false,
            useAccelerometerForAttitudeEstimation = true,
            startOffsetEnabled = false,
            AccelerometerSensorType.ACCELEROMETER,
            accelerometerAveragingFilter,
            GyroscopeSensorType.GYROSCOPE,
            useAccurateRelativeGyroscopeAttitudeProcessor = false,
            poseAvailableListener,
            accuracyChangedListener,
            bufferFilledListener
        )

        // check
        assertSame(context, estimator.context)
        assertSame(initialSpeed, estimator.initialSpeed)
        assertEquals(SensorDelay.NORMAL, estimator.sensorDelay)
        assertFalse(estimator.useAttitudeSensor)
        assertTrue(estimator.useAccelerometerForAttitudeEstimation)
        assertFalse(estimator.startOffsetEnabled)
        assertEquals(
            AccelerometerSensorType.ACCELEROMETER,
            estimator.accelerometerSensorType
        )
        assertSame(accelerometerAveragingFilter, estimator.accelerometerAveragingFilter)
        assertEquals(GyroscopeSensorType.GYROSCOPE, estimator.gyroscopeSensorType)
        assertFalse(estimator.useAccurateRelativeGyroscopeAttitudeProcessor)
        assertSame(poseAvailableListener, estimator.poseAvailableListener)
        assertSame(accuracyChangedListener, estimator.accuracyChangedListener)
        assertSame(bufferFilledListener, estimator.bufferFilledListener)
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialSpeed = SpeedTriad()
        val estimator = RelativePoseEstimator2(context, initialSpeed, useAttitudeSensor = true)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialSpeed = SpeedTriad()
        val estimator = RelativePoseEstimator2(
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val initialSpeed = SpeedTriad()
        val estimator = RelativePoseEstimator2(
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator2(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator2(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator2(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator2(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator2(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator2(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator2(context)

        // check default value
        assertEquals(
            BaseFusedGeomagneticAttitudeProcessor.DEFAULT_PANIC_COUNTER_THRESHOLD,
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator2(context, useAttitudeSensor = true)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator2(
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator2(
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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator2(context)

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
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator2(context)

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

    @Test(expected = IllegalStateException::class)
    fun start_whenRunning_throwsIllegalStateException() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator2(context)

        assertFalse(estimator.running)

        // set as running
        estimator.setPrivateProperty("running", true)

        assertTrue(estimator.running)

        estimator.start()

        assertFalse(estimator.running)
    }

    @Test
    fun start_whenUseAttitudeSensor_resetsProcessorAndStartsSyncer() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator2(
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

        val fusedSyncer: AccelerometerGravityAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("fusedSyncer")
        requireNotNull(fusedSyncer)
        val fusedSyncerSpy = spyk(fusedSyncer)
        estimator.setPrivateProperty("fusedSyncer", fusedSyncerSpy)

        val accelerometerFusedSyncer: AccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerFusedSyncer")
        requireNotNull(accelerometerFusedSyncer)
        val accelerometerFusedSyncerSpy = spyk(accelerometerFusedSyncer)
        estimator.setPrivateProperty("accelerometerFusedSyncer", accelerometerFusedSyncerSpy)

        val attitudeSyncer: AttitudeAndAccelerometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("attitudeSyncer")
        requireNotNull(attitudeSyncer)
        val attitudeSyncerSpy = spyk(attitudeSyncer)
        every { attitudeSyncerSpy.start(startTimestamp) }.returns(true)
        estimator.setPrivateProperty("attitudeSyncer", attitudeSyncerSpy)

        assertTrue(estimator.start(startTimestamp))
        assertTrue(estimator.running)

        verify { attitudeProcessorSpy.reset() }
        verify(exactly = 1) { attitudeSyncerSpy.start(startTimestamp) }
        verify { accelerometerFusedProcessorSpy wasNot Called }
        verify { accelerometerFusedSyncerSpy wasNot Called }
        verify { fusedProcessorSpy wasNot Called }
        verify { fusedSyncerSpy wasNot Called }
    }

    @Test
    fun start_whenNotUseAttitudeSensorAndUseAccelerometerForAttitudeEstimation_resetsProcessorAndStartsSyncer() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator2(
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

        val fusedSyncer: AccelerometerGravityAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("fusedSyncer")
        requireNotNull(fusedSyncer)
        val fusedSyncerSpy = spyk(fusedSyncer)
        estimator.setPrivateProperty("fusedSyncer", fusedSyncerSpy)

        val accelerometerFusedSyncer: AccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerFusedSyncer")
        requireNotNull(accelerometerFusedSyncer)
        val accelerometerFusedSyncerSpy = spyk(accelerometerFusedSyncer)
        every { accelerometerFusedSyncerSpy.start(startTimestamp) }.returns(true)
        estimator.setPrivateProperty("accelerometerFusedSyncer", accelerometerFusedSyncerSpy)

        val attitudeSyncer: AttitudeAndAccelerometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("attitudeSyncer")
        requireNotNull(attitudeSyncer)
        val attitudeSyncerSpy = spyk(attitudeSyncer)
        estimator.setPrivateProperty("attitudeSyncer", attitudeSyncerSpy)

        assertTrue(estimator.start(startTimestamp))
        assertTrue(estimator.running)

        verify { accelerometerFusedProcessorSpy.reset() }
        verify(exactly = 1) { accelerometerFusedSyncerSpy.start(startTimestamp) }
        verify { attitudeProcessorSpy wasNot Called }
        verify { attitudeSyncerSpy wasNot Called }
        verify { fusedProcessorSpy wasNot Called }
        verify { fusedSyncerSpy wasNot Called }
    }

    @Test
    fun start_whenNotUseAttitudeSensorAndNotUseAccelerometerForAttitudeEstimation_resetsProcessorAndStartsSyncer() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator2(
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

        val fusedSyncer: AccelerometerGravityAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("fusedSyncer")
        requireNotNull(fusedSyncer)
        val fusedSyncerSpy = spyk(fusedSyncer)
        every { fusedSyncerSpy.start(startTimestamp) }.returns(true)
        estimator.setPrivateProperty("fusedSyncer", fusedSyncerSpy)

        val accelerometerFusedSyncer: AccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerFusedSyncer")
        requireNotNull(accelerometerFusedSyncer)
        val accelerometerFusedSyncerSpy = spyk(accelerometerFusedSyncer)
        estimator.setPrivateProperty("accelerometerFusedSyncer", accelerometerFusedSyncerSpy)

        val attitudeSyncer: AttitudeAndAccelerometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("attitudeSyncer")
        requireNotNull(attitudeSyncer)
        val attitudeSyncerSpy = spyk(attitudeSyncer)
        estimator.setPrivateProperty("attitudeSyncer", attitudeSyncerSpy)

        assertTrue(estimator.start(startTimestamp))
        assertTrue(estimator.running)

        verify { fusedProcessorSpy.reset() }
        verify(exactly = 1) { fusedSyncerSpy.start(startTimestamp) }
        verify { attitudeProcessorSpy wasNot Called }
        verify { attitudeSyncerSpy wasNot Called }
        verify { accelerometerFusedProcessorSpy wasNot Called }
        verify { accelerometerFusedSyncerSpy wasNot Called }
    }

    @Test
    fun stop_stopsAllSyncers() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator2(context)

        val startTimestamp = System.nanoTime()

        val fusedSyncer: AccelerometerGravityAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("fusedSyncer")
        requireNotNull(fusedSyncer)
        val fusedSyncerSpy = spyk(fusedSyncer)
        every { fusedSyncerSpy.start(startTimestamp) }.returns(true)
        estimator.setPrivateProperty("fusedSyncer", fusedSyncerSpy)

        val accelerometerFusedSyncer: AccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerFusedSyncer")
        requireNotNull(accelerometerFusedSyncer)
        val accelerometerFusedSyncerSpy = spyk(accelerometerFusedSyncer)
        estimator.setPrivateProperty("accelerometerFusedSyncer", accelerometerFusedSyncerSpy)

        val attitudeSyncer: AttitudeAndAccelerometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("attitudeSyncer")
        requireNotNull(attitudeSyncer)
        val attitudeSyncerSpy = spyk(attitudeSyncer)
        estimator.setPrivateProperty("attitudeSyncer", attitudeSyncerSpy)

        // stop
        estimator.stop()

        verify(exactly = 1) { fusedSyncerSpy.stop() }
        verify(exactly = 1) { accelerometerFusedSyncerSpy.stop() }
        verify(exactly = 1) { attitudeSyncerSpy.stop() }
    }

    @Test
    fun fusedSyncer_whenAccuracyChangedAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator2(context)

        val fusedSyncer: AccelerometerGravityAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("fusedSyncer")
        requireNotNull(fusedSyncer)
        val listener = fusedSyncer.accuracyChangedListener
        requireNotNull(listener)

        // notify
        listener.onAccuracyChanged(fusedSyncer, SensorType.ABSOLUTE_ATTITUDE, SensorAccuracy.HIGH)
    }

    @Test
    fun fusedSyncer_whenAccuracyChangedAndListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val accuracyChangedListener =
            mockk<RelativePoseEstimator2.OnAccuracyChangedListener>(relaxUnitFun = true)
        val estimator = RelativePoseEstimator2(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val fusedSyncer: AccelerometerGravityAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("fusedSyncer")
        requireNotNull(fusedSyncer)
        val listener = fusedSyncer.accuracyChangedListener
        requireNotNull(listener)

        // notify
        listener.onAccuracyChanged(fusedSyncer, SensorType.ABSOLUTE_ATTITUDE, SensorAccuracy.HIGH)

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
    fun fusedSyncer_whenBufferFilledAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator2(context)

        val fusedSyncer: AccelerometerGravityAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("fusedSyncer")
        requireNotNull(fusedSyncer)
        val listener = fusedSyncer.bufferFilledListener
        requireNotNull(listener)

        // notify
        listener.onBufferFilled(fusedSyncer, SensorType.ABSOLUTE_ATTITUDE)
    }

    @Test
    fun fusedSyncer_whenBufferFilledAndListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val bufferFilledListener =
            mockk<RelativePoseEstimator2.OnBufferFilledListener>(relaxUnitFun = true)
        val estimator = RelativePoseEstimator2(
            context,
            bufferFilledListener = bufferFilledListener
        )

        val fusedSyncer: AccelerometerGravityAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("fusedSyncer")
        requireNotNull(fusedSyncer)
        val listener = fusedSyncer.bufferFilledListener
        requireNotNull(listener)

        // notify
        listener.onBufferFilled(fusedSyncer, SensorType.ABSOLUTE_ATTITUDE)

        // check
        verify(exactly = 1) {
            bufferFilledListener.onBufferFilled(
                estimator,
                SensorType.ABSOLUTE_ATTITUDE
            )
        }
    }

    @Test
    fun fusedSyncer_whenSyncedMeasurementAndNotProcessed_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val poseAvailableListener = mockk<RelativePoseEstimator2.OnPoseAvailableListener>()
        val estimator = RelativePoseEstimator2(
            context,
            poseAvailableListener = poseAvailableListener
        )

        val fusedSyncer: AccelerometerGravityAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("fusedSyncer")
        requireNotNull(fusedSyncer)
        val listener = fusedSyncer.syncedMeasurementListener
        requireNotNull(listener)

        val fusedProcessor: FusedRelativePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        every { fusedProcessorSpy.process(any()) }.returns(false)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        // notify measurement
        val measurement = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement()
        listener.onSyncedMeasurements(fusedSyncer, measurement)

        // check
        verify(exactly = 1) { fusedProcessorSpy.process(measurement) }
        verify { poseAvailableListener wasNot Called }
    }

    @Test
    fun fusedSyncer_whenSyncedMeasurementProcessedAndNoListener_makesExpectedCalls() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator2(context)

        val fusedSyncer: AccelerometerGravityAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("fusedSyncer")
        requireNotNull(fusedSyncer)
        val listener = fusedSyncer.syncedMeasurementListener
        requireNotNull(listener)

        val fusedProcessor: FusedRelativePoseProcessor? =
            estimator.getPrivateProperty("fusedProcessor")
        requireNotNull(fusedProcessor)
        val fusedProcessorSpy = spyk(fusedProcessor)
        every { fusedProcessorSpy.process(any()) }.returns(true)
        estimator.setPrivateProperty("fusedProcessor", fusedProcessorSpy)

        // notify measurement
        val measurement = AccelerometerGravityAndGyroscopeSyncedSensorMeasurement()
        listener.onSyncedMeasurements(fusedSyncer, measurement)

        // check
        verify(exactly = 1) { fusedProcessorSpy.process(measurement) }
        verify(exactly = 1) { fusedProcessorSpy.poseTransformation }
    }

    @Test
    fun fusedSyncer_whenSyncedMeasurementProcessedAndListener_makesExpectedCalls() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val poseAvailableListener =
            mockk<RelativePoseEstimator2.OnPoseAvailableListener>(relaxUnitFun = true)
        val estimator = RelativePoseEstimator2(
            context,
            poseAvailableListener = poseAvailableListener
        )

        val fusedSyncer: AccelerometerGravityAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("fusedSyncer")
        requireNotNull(fusedSyncer)
        val listener = fusedSyncer.syncedMeasurementListener
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
        listener.onSyncedMeasurements(fusedSyncer, measurement)

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
    fun accelerometerFusedSyncer_whenAccuracyChangedAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator2(context)

        val accelerometerFusedSyncer: AccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerFusedSyncer")
        requireNotNull(accelerometerFusedSyncer)
        val listener = accelerometerFusedSyncer.accuracyChangedListener
        requireNotNull(listener)

        // notify
        listener.onAccuracyChanged(
            accelerometerFusedSyncer,
            SensorType.ABSOLUTE_ATTITUDE,
            SensorAccuracy.HIGH
        )
    }

    @Test
    fun accelerometerFusedSyncer_whenAccuracyChangedAndListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val accuracyChangedListener =
            mockk<RelativePoseEstimator2.OnAccuracyChangedListener>(relaxUnitFun = true)
        val estimator = RelativePoseEstimator2(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val accelerometerFusedSyncer: AccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerFusedSyncer")
        requireNotNull(accelerometerFusedSyncer)
        val listener = accelerometerFusedSyncer.accuracyChangedListener
        requireNotNull(listener)

        // notify
        listener.onAccuracyChanged(
            accelerometerFusedSyncer,
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
    fun accelerometerFusedSyncer_whenBufferFilledAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator2(context)

        val accelerometerFusedSyncer: AccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerFusedSyncer")
        requireNotNull(accelerometerFusedSyncer)
        val listener = accelerometerFusedSyncer.bufferFilledListener
        requireNotNull(listener)

        // notify
        listener.onBufferFilled(accelerometerFusedSyncer, SensorType.ABSOLUTE_ATTITUDE)
    }

    @Test
    fun accelerometerFusedSyncer_whenBufferFilledAndListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val bufferFilledListener =
            mockk<RelativePoseEstimator2.OnBufferFilledListener>(relaxUnitFun = true)
        val estimator = RelativePoseEstimator2(
            context,
            bufferFilledListener = bufferFilledListener
        )

        val accelerometerFusedSyncer: AccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerFusedSyncer")
        requireNotNull(accelerometerFusedSyncer)
        val listener = accelerometerFusedSyncer.bufferFilledListener
        requireNotNull(listener)

        // notify
        listener.onBufferFilled(accelerometerFusedSyncer, SensorType.ABSOLUTE_ATTITUDE)

        // check
        verify(exactly = 1) {
            bufferFilledListener.onBufferFilled(
                estimator,
                SensorType.ABSOLUTE_ATTITUDE
            )
        }
    }

    @Test
    fun accelerometerFusedSyncer_whenSyncedMeasurementAndNotProcessed_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val poseAvailableListener = mockk<RelativePoseEstimator2.OnPoseAvailableListener>()
        val estimator = RelativePoseEstimator2(
            context,
            poseAvailableListener = poseAvailableListener
        )

        val accelerometerFusedSyncer: AccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerFusedSyncer")
        requireNotNull(accelerometerFusedSyncer)
        val listener = accelerometerFusedSyncer.syncedMeasurementListener
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
        listener.onSyncedMeasurements(accelerometerFusedSyncer, measurement)

        // check
        verify(exactly = 1) { accelerometerFusedProcessorSpy.process(measurement) }
        verify { poseAvailableListener wasNot Called }
    }

    @Test
    fun accelerometerFusedSyncer_whenSyncedMeasurementProcessedAndNoListener_makesExpectedCalls() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator2(context)

        val accelerometerFusedSyncer: AccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerFusedSyncer")
        requireNotNull(accelerometerFusedSyncer)
        val listener = accelerometerFusedSyncer.syncedMeasurementListener
        requireNotNull(listener)

        val accelerometerFusedProcessor: AccelerometerFusedRelativePoseProcessor? =
            estimator.getPrivateProperty("accelerometerFusedProcessor")
        requireNotNull(accelerometerFusedProcessor)
        val accelerometerFusedProcessorSpy = spyk(accelerometerFusedProcessor)
        every { accelerometerFusedProcessorSpy.process(any()) }.returns(true)
        estimator.setPrivateProperty("accelerometerFusedProcessor", accelerometerFusedProcessorSpy)

        // notify measurement
        val measurement = AccelerometerAndGyroscopeSyncedSensorMeasurement()
        listener.onSyncedMeasurements(accelerometerFusedSyncer, measurement)

        // check
        verify(exactly = 1) { accelerometerFusedProcessorSpy.process(measurement) }
        verify(exactly = 1) { accelerometerFusedProcessorSpy.poseTransformation }
    }

    @Test
    fun accelerometerFusedSyncer_whenSyncedMeasurementProcessedAndListener_makesExpectedCalls() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val poseAvailableListener =
            mockk<RelativePoseEstimator2.OnPoseAvailableListener>(relaxUnitFun = true)
        val estimator = RelativePoseEstimator2(
            context,
            poseAvailableListener = poseAvailableListener
        )

        val accelerometerFusedSyncer: AccelerometerAndGyroscopeSensorMeasurementSyncer? =
            estimator.getPrivateProperty("accelerometerFusedSyncer")
        requireNotNull(accelerometerFusedSyncer)
        val listener = accelerometerFusedSyncer.syncedMeasurementListener
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
        listener.onSyncedMeasurements(accelerometerFusedSyncer, measurement)

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
    fun attitudeSyncer_whenAccuracyChangedAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator2(context)

        val attitudeSyncer: AttitudeAndAccelerometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("attitudeSyncer")
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
    fun attitudeSyncer_whenAccuracyChangedAndListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val accuracyChangedListener =
            mockk<RelativePoseEstimator2.OnAccuracyChangedListener>(relaxUnitFun = true)
        val estimator = RelativePoseEstimator2(
            context,
            accuracyChangedListener = accuracyChangedListener
        )

        val attitudeSyncer: AttitudeAndAccelerometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("attitudeSyncer")
        requireNotNull(attitudeSyncer)
        val listener = attitudeSyncer.accuracyChangedListener
        requireNotNull(listener)

        // notify
        listener.onAccuracyChanged(
            attitudeSyncer,
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
    fun attitudeSyncer_whenBufferFilledAndNoListener_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator2(context)

        val attitudeSyncer: AttitudeAndAccelerometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("attitudeSyncer")
        requireNotNull(attitudeSyncer)
        val listener = attitudeSyncer.bufferFilledListener
        requireNotNull(listener)

        // notify
        listener.onBufferFilled(attitudeSyncer, SensorType.ABSOLUTE_ATTITUDE)
    }

    @Test
    fun attitudeSyncer_whenBufferFilledAndListener_notifies() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val bufferFilledListener =
            mockk<RelativePoseEstimator2.OnBufferFilledListener>(relaxUnitFun = true)
        val estimator = RelativePoseEstimator2(
            context,
            bufferFilledListener = bufferFilledListener
        )

        val attitudeSyncer: AttitudeAndAccelerometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("attitudeSyncer")
        requireNotNull(attitudeSyncer)
        val listener = attitudeSyncer.bufferFilledListener
        requireNotNull(listener)

        // notify
        listener.onBufferFilled(attitudeSyncer, SensorType.ABSOLUTE_ATTITUDE)

        // check
        verify(exactly = 1) {
            bufferFilledListener.onBufferFilled(
                estimator,
                SensorType.ABSOLUTE_ATTITUDE
            )
        }
    }

    @Test
    fun attitudeSyncer_whenSyncedMeasurementAndNotProcessed_makesNoAction() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val poseAvailableListener = mockk<RelativePoseEstimator2.OnPoseAvailableListener>()
        val estimator = RelativePoseEstimator2(
            context,
            poseAvailableListener = poseAvailableListener
        )

        val attitudeSyncer: AttitudeAndAccelerometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("attitudeSyncer")
        requireNotNull(attitudeSyncer)
        val listener = attitudeSyncer.syncedMeasurementListener
        requireNotNull(listener)

        val attitudeProcessor: AttitudeRelativePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        every { attitudeProcessorSpy.process(any()) }.returns(false)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        // notify measurement
        val measurement = AttitudeAndAccelerometerSyncedSensorMeasurement()
        listener.onSyncedMeasurements(attitudeSyncer, measurement)

        // check
        verify(exactly = 1) { attitudeProcessorSpy.process(measurement) }
        verify { poseAvailableListener wasNot Called }
    }

    @Test
    fun attitudeSyncer_whenSyncedMeasurementProcessedAndNoListener_makesExpectedCalls() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val estimator = RelativePoseEstimator2(context)

        val attitudeSyncer: AttitudeAndAccelerometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("attitudeSyncer")
        requireNotNull(attitudeSyncer)
        val listener = attitudeSyncer.syncedMeasurementListener
        requireNotNull(listener)

        val attitudeProcessor: AttitudeRelativePoseProcessor? =
            estimator.getPrivateProperty("attitudeProcessor")
        requireNotNull(attitudeProcessor)
        val attitudeProcessorSpy = spyk(attitudeProcessor)
        every { attitudeProcessorSpy.process(any()) }.returns(true)
        estimator.setPrivateProperty("attitudeProcessor", attitudeProcessorSpy)

        // notify measurement
        val measurement = AttitudeAndAccelerometerSyncedSensorMeasurement()
        listener.onSyncedMeasurements(attitudeSyncer, measurement)

        // check
        verify(exactly = 1) { attitudeProcessorSpy.process(measurement) }
        verify(exactly = 1) { attitudeProcessorSpy.poseTransformation }
    }

    @Test
    fun attitudeSyncer_whenSyncedMeasurementAttitudeProcessedNotEstimatePoseTransformationAndListener_makesExpectedCalls() {
        val context = ApplicationProvider.getApplicationContext<Context>()
        val poseAvailableListener =
            mockk<RelativePoseEstimator2.OnPoseAvailableListener>(relaxUnitFun = true)
        val estimator = RelativePoseEstimator2(
            context,
            poseAvailableListener = poseAvailableListener
        )

        val attitudeSyncer: AttitudeAndAccelerometerSensorMeasurementSyncer? =
            estimator.getPrivateProperty("attitudeSyncer")
        requireNotNull(attitudeSyncer)
        val listener = attitudeSyncer.syncedMeasurementListener
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
        listener.onSyncedMeasurements(attitudeSyncer, measurement)

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

    private companion object {
        const val MIN_LATITUDE_DEGREES = -90.0
        const val MAX_LATITUDE_DEGREES = 90.0

        const val MIN_LONGITUDE_DEGREES = -180.0
        const val MAX_LONGITUDE_DEGREES = 180.0

        const val MIN_HEIGHT = -100.0
        const val MAX_HEIGHT = 4000.0

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
    }
}